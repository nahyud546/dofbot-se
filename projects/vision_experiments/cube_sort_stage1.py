#!/usr/bin/env python3
"""Identify cubes and sort one fresh, IK-reachable target per Space press."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import time
from concurrent.futures import ThreadPoolExecutor
import tty

import cv2

from identify_cube import (CUBES, FRAME_SIZE, OneSecondAverager, TagDetector,
                           TrashDetector, detect_frame, draw_tracked_frame,
                           open_camera, repo_root, save_contour_debug)


ROOT = repo_root()
WORKER = ROOT / "projects/t8_pipeline/t8_motion_worker.py"
ROS_SETUP = ROOT / "workspaces/dofbot_ws/install/setup.bash"
AUTO_RETRY_SECONDS = 8.0


class TerminalKeys:
    """Accept single keys from the terminal as well as the OpenCV window."""

    def __enter__(self):
        self.fd = None
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_args):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def poll(self):
        if self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            return os.read(self.fd, 1)[0]
        return -1


def run_motion(command, **params):
    script = f"source /opt/ros/humble/setup.bash && source {ROS_SETUP} && exec {ROOT}/.venv/bin/python {WORKER}"
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/cube_sort_ros_logs")
    try:
        proc = subprocess.run(["bash", "-c", script], input=json.dumps(
            {"command": command, **params}), text=True, capture_output=True,
            cwd=ROOT, env=env, timeout=75, check=False)
    except subprocess.TimeoutExpired:
        return {"ok": False, "reply": "Hết thời gian chờ chuyển động; kiểm tra tay và trạng thái trước khi tiếp tục."}
    for line in reversed(proc.stdout.splitlines()):
        if line.startswith("T8_RESULT:"):
            return json.loads(line[len("T8_RESULT:"):])
    return {"ok": False, "reply": (proc.stderr or proc.stdout).strip()[-300:]}


def rank_candidates(reports, now, max_drift_px=20, excluded_targets=()):
    """Rank all graspable cubes: Tier A sorted by confidence desc.

    Tier A = confirmed ID in CUBES + current geometry valid + fresh ID.
    Everything else (unknown/conflict/no-top) is Tier B and never sent to
    IK because BIN_POSES needs a canonical cube ID.
    """
    candidates = []
    for track, verdict in reports:
        best = track.best_observation
        live = track.observation
        if (not track.confirmed or verdict.cube_id not in CUBES or best is None or
                best.verdict.cube_id != verdict.cube_id or
                not live.geometry_valid or
                getattr(track, "association_ambiguous", False) or
                now - track.last_seen > 0.75):
            continue
        if any(same_target(target_payload((track, verdict)), excluded, 35)
               for excluded in excluded_targets):
            continue
        candidates.append((track, verdict))
    candidates.sort(key=lambda item: confidence(item[1]), reverse=True)
    return candidates


def pick_candidate(reports, now, max_drift_px=20, excluded_targets=()):
    """Only accept a recent ID vote with a fresh, matching upper face."""
    ranked = rank_candidates(reports, now, max_drift_px, excluded_targets)
    return ranked[0] if ranked else None


def describe_unknown(tracked, now, ranked_ids=()):
    """Summarize Tier B cubes (classified-fail or unclassified) for the log."""
    _ranked = set(id(item) for item in ranked_ids)
    notes = []
    for track, verdict in tracked:
        if id(track) in _ranked:
            continue
        reason = track.observation.verdict.reason or verdict.reason or verdict.conflict
        if not reason:
            if not track.confirmed:
                reason = "collecting 1s"
            elif now - getattr(track, "last_id_seen", track.last_seen) > 0.75:
                reason = "id-lost"
            elif getattr(track, "association_ambiguous", False):
                reason = "ambiguous-association"
            elif not track.observation.geometry_valid:
                reason = "invalid-geometry"
            else:
                reason = "rejected"
        geometry_reason = track.observation.group.geometry_reason
        if geometry_reason and geometry_reason not in reason:
            reason += f"; {geometry_reason}"
        top_two = track.observation.verdict.trash_top or verdict.trash_top
        ranking = " [" + ", ".join(
            f"Top{i} {name} {score * 100:.1f}%"
            for i, (name, score) in enumerate(top_two[:2], 1)) + "]" if top_two else ""
        angle = (track.observation.verdict.trash_rotation_deg
                 if track.observation.verdict.trash_top else verdict.trash_rotation_deg)
        if angle is not None and ranking:
            ranking += f" rot={angle}deg"
        if verdict.cube_id is None:
            notes.append(f"unknown({verdict.via}:{verdict.label or '?'}:{reason}){ranking}")
        elif not track.observation.geometry_valid:
            geometry_reason = track.observation.group.geometry_reason or "no-top"
            notes.append(f"ID{verdict.cube_id}:{geometry_reason}{ranking}")
        else:
            notes.append(f"ID{verdict.cube_id}:{reason}{ranking}")
    return notes


def confidence(verdict):
    """Put tag decision margins on the same 0..1 scale as color and DINO."""
    if "tag" in verdict.via.split("+"):
        return 1.0 - math.exp(-max(0.0, verdict.score) / 20.0)
    return min(1.0, max(0.0, verdict.score))


def target_payload(candidate):
    track, verdict = candidate
    observation = track.observation
    return {"cube_id": verdict.cube_id,
            "top_center_px": list(observation.top_center_px),
            "top_quad_px": observation.group.top.corners.tolist(),
            "image_size": list(FRAME_SIZE),
            "geometry_valid": True, "id_confirmed": True,
            "label": verdict.label}


def same_target(left, right, max_drift_px=15):
    if left is None or right is None or left["cube_id"] != right["cube_id"]:
        return False
    ax, ay = left["top_center_px"]
    bx, by = right["top_center_px"]
    return (ax - bx) ** 2 + (ay - by) ** 2 <= max_drift_px ** 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video2")
    parser.add_argument("--dry-run", action="store_true",
                        help="simulate one sort per Space without accessing the arm or buzzer")
    parser.add_argument("--no-trash", action="store_true")
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--sim-thresh", type=float, default=0.40)
    parser.add_argument("--sim-margin", type=float, default=0.0)
    parser.add_argument("--auto", action="store_true",
                        help="tự gắp khi có cube được xác nhận (không cần Space)")
    parser.add_argument("--pick-x-offset-mm", type=float, default=15.0,
                        help="bù X cho tâm gắp: dương kéo kẹp về phía chân robot (mặc định +15 mm, ±20 mm)")
    parser.add_argument("--pick-z-offset-mm", type=float, default=0.0,
                        help="bù độ cao tâm gắp: dương nâng kẹp lên (±8 mm)")
    args = parser.parse_args()
    if (not math.isfinite(args.pick_x_offset_mm) or abs(args.pick_x_offset_mm) > 20 or
            not math.isfinite(args.pick_z_offset_mm) or abs(args.pick_z_offset_mm) > 8):
        parser.error("độ bù gắp phải trong khoảng X ±20 mm, Z ±8 mm")

    if args.dry_run:
        print("[DRY] bỏ qua pose-start (không chạm tay máy)")
    else:
        print("[POSE] đưa tay về pose quan sát trước khi scan...")
        result = run_motion("prepare")
        if not result.get("ok"):
            print(f"[FAIL] pose-start thất bại: {result.get('reply')}")
            print("[HINT] kiểm tra /dev/ttyUSB0 có bị tiến trình khác giữ không")
            return 1
        print(f"[OK] pose-start: {result.get('reply')}")
    tag_detector = TagDetector()
    trash_detector = None if args.no_trash else TrashDetector(
        thresh=args.sim_thresh, margin=args.sim_margin, device=args.device)
    cap = open_camera(args.camera)
    if cap is None:
        print(f"[FAIL] cannot open camera {args.camera}")
        return 1
    averager = OneSecondAverager()
    last_result_at = 0.0
    last_unknown_log_at = 0.0
    last_unknown_notes = None
    tracked = []
    votes = []
    pending = None
    pending_capture_at = 0.0
    discard_pending = False
    motion_pending = None
    motion_error = None
    sorting_candidates = []
    auto_next_at = 0.0

    def current_candidates(now):
        ranked = rank_candidates(votes, now)
        return [target_payload(item) for item in ranked]

    def start_sort(candidates):
        nonlocal sorting_candidates, motion_pending, discard_pending
        sorting_candidates = candidates
        discard_pending = pending is not None
        print("[SORT] Kiểm tra IK theo thứ tự confidence: " +
              ", ".join(f"ID {item['cube_id']}" for item in candidates))
        if args.dry_run:
            motion_pending = motion_executor.submit(
                lambda: {"ok": True, "cube_id": candidates[0]["cube_id"],
                         "candidate_index": 0, "skipped": [],
                         "reply": "dry-run: không kiểm tra IK, không bíp hoặc di chuyển tay"})
        else:
            motion_pending = motion_executor.submit(
                run_motion, "sort_cube_candidates", candidates=candidates,
                pick_x_offset_mm=args.pick_x_offset_mm,
                pick_z_offset_mm=args.pick_z_offset_mm)

    mode = "AUTO" if args.auto else "MANUAL(Space)"
    print(f"[OK] stage 1 ready [{mode}]. Space: chọn cube, thử IK rồi gắp; "
          "chỉ bíp khi bắt đầu gắp; q: quit; s: raw frame; d: contour metrics")
    print(f"[CALIB] bù tâm gắp KDL X={args.pick_x_offset_mm:+.1f} mm, "
          f"Z={args.pick_z_offset_mm:+.1f} mm")
    try:
        with (ThreadPoolExecutor(max_workers=1) as executor,
              ThreadPoolExecutor(max_workers=1) as motion_executor,
              TerminalKeys() as terminal):
            while True:
                ok, frame = cap.read()
                if not ok:
                    cv2.waitKey(1)
                    continue
                if (frame.shape[1], frame.shape[0]) != FRAME_SIZE:
                    frame = cv2.resize(frame, FRAME_SIZE)
                now = time.monotonic()
                if motion_pending is not None and motion_pending.done():
                    result = motion_pending.result()
                    motion_pending = None
                    for skipped in result.get("skipped", []):
                        print(f"[IK SKIP] ID {skipped['cube_id']}: {skipped['reply']}")
                    if result.get("target_xyz") is not None:
                        print(f"[TARGET] ID {result.get('cube_id')} "
                              f"pixel={sorting_candidates[result.get('candidate_index', 0)]['top_center_px']} "
                              f"KDL xyz={result['target_xyz']}")
                    print("[SORT OK]" if result.get("ok") else "[SORT FAIL]", result.get("reply"))
                    if result.get("ok"):
                        averager = OneSecondAverager()
                        tracked = []
                        votes = []
                        last_result_at = 0.0
                        last_unknown_notes = None
                        auto_next_at = 0.0
                        for _ in range(5):
                            cap.grab()
                    elif result.get("code") == "ik_no_solution":
                        auto_next_at = now + AUTO_RETRY_SECONDS
                    else:
                        motion_error = result.get("reply", "Lỗi chuyển động")
                    sorting_candidates = []
                if pending is not None and pending.done():
                    observations = pending.result()
                    pending = None
                    if discard_pending:
                        discard_pending = False
                    elif motion_pending is None and motion_error is None:
                        tracked, _ = averager.update(observations, pending_capture_at)
                        last_result_at = pending_capture_at
                        votes = [(track, getattr(track, "published", None) or verdict)
                                 for track, verdict in tracked]
                        ranked = rank_candidates(votes, now)
                        if not ranked and tracked and now - last_unknown_log_at > 2.0:
                            last_unknown_log_at = now
                            notes = tuple(describe_unknown(tracked, now))
                            if notes and notes != last_unknown_notes:
                                print(f"[UNKNOWN] {len(tracked)} cube chưa gắp được: "
                                      + "; ".join(notes[:4]))
                            last_unknown_notes = notes
                        elif ranked:
                            last_unknown_notes = None
                if pending is None and motion_pending is None and motion_error is None:
                    pending_capture_at = now
                    pending = executor.submit(detect_frame, frame.copy(),
                                              tag_detector, trash_detector, False)
                if now - last_result_at > 2:
                    tracked = []
                    votes = []
                candidates = current_candidates(now) if motion_pending is None else []
                n_unknown = max(0, len(tracked) - len(candidates))
                display = draw_tracked_frame(frame, tracked)
                if motion_pending is not None:
                    status = "IK checking / sorting..."
                elif motion_error:
                    status = "SORT ERROR: inspect arm"
                elif candidates:
                    status = (f"AUTO: {len(candidates)} cube ready" if args.auto else
                              f"SPACE: check IK and sort ({len(candidates)} cube)")
                elif n_unknown > 0:
                    status = f"Resolving {n_unknown} unknown cube(s)..."
                else:
                    status = "Scanning for confirmed ID..."
                cv2.putText(display, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 165, 255), 2, cv2.LINE_AA)
                cv2.imshow("cube_sort_stage1 (q to quit)", display)
                gui_key = cv2.waitKey(1) & 0xFF
                terminal_key = terminal.poll()
                key = gui_key if gui_key != 255 else terminal_key
                if key == ord("q"):
                    break
                if key == ord("s"):
                    path = Path(f"/tmp/cube_sort_raw_{int(time.time())}.png")
                    cv2.imwrite(str(path), frame)
                    print("[OK] raw frame ->", path)
                if key == ord("d"):
                    raw, report = save_contour_debug(
                        frame, Path(f"/tmp/cube_sort_contour_{int(time.time())}"))
                    print(f"[CONTOUR] raw={raw} metrics={report}")
                if key == ord(" "):
                    print("[SPACE] Đã nhận lệnh", flush=True)
                if key == ord(" ") and motion_pending is None and motion_error is None:
                    if candidates:
                        start_sort(candidates)
                    else:
                        print("[WAIT] Chưa có cube với ID và mặt trên xác nhận còn mới")
                elif key == ord(" "):
                    print("[WAIT] Tay đang phân loại hoặc cần kiểm tra lỗi chuyển động")
                if (args.auto and motion_pending is None and motion_error is None and
                        candidates and now >= auto_next_at):
                    start_sort(candidates)
    except KeyboardInterrupt:
        print("[OK] stopped")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
