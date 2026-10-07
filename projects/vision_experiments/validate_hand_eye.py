#!/usr/bin/env python3
"""Kiểm tra hand-eye ở nhiều pose: cùng một cube đứng yên phải cho cùng XY.

Tay đi tới các pose (một nửa trong vùng đã hiệu chuẩn, một nửa ngoài vùng), ở
mỗi pose tính XY của cube bằng đúng công thức search-center (pixel tâm tag +
khớp đo thật + hand-eye, giao tia với mặt phẳng mặt trên). Cube không di
chuyển nên mọi pose phải ra cùng một điểm; độ lệch so với trung vị chính là
sai số của công thức ở pose đó. Không gắp, không chạm bàn (đầu kẹp >= 90 mm).
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import time

import cv2
import numpy as np

import calibrate_hand_eye as C
import cube_search_center_math as M

# Vùng khớp của 24 mẫu hiệu chuẩn (xem hand_eye_samples.json).
ENVELOPE = {"j1": (75, 100), "j2": (80, 129), "j3": (0, 29), "j4": (0, 19)}


def inside_envelope(servo, margin: float = 0.0) -> bool:
    return all(lo - margin <= servo[i] <= hi + margin
               for i, (lo, hi) in enumerate(ENVELOPE.values()))


def tag_xy(corners, servo, tag_top_z):
    """XY trên mặt phẳng mặt trên cube từ góc tag và khớp (cách search-center)."""
    u, v = (float(x) for x in np.asarray(corners).mean(axis=0))
    point, _ = M.pixel_to_base(u, v, servo, z_table=tag_top_z)
    return float(point[0]), float(point[1])


def choose_poses(pool, count: int, rng):
    """Nửa trong, nửa ngoài vùng hiệu chuẩn; chọn xa nhau (farthest point)."""
    inner = [p for p in pool if inside_envelope(p)]
    outer = [p for p in pool if not inside_envelope(p)]

    def spread(items, n):
        if not items or n <= 0:
            return []
        chosen = [items[int(rng.integers(len(items)))]]
        while len(chosen) < min(n, len(items)):
            chosen.append(max(items, key=lambda p: min(C._joint_dist(p, c) for c in chosen)))
        return chosen

    return spread(inner, count // 2) + spread(outer, count - count // 2)


def summarise(rows):
    """rows: [(servo, (x, y))] -> báo cáo độ lệch so với trung vị toàn bộ."""
    xs = np.array([r[1] for r in rows], float)
    centre = np.median(xs, axis=0)
    errors = np.linalg.norm(xs - centre, axis=1) * 1000.0
    inner = [e for (s, _), e in zip(rows, errors) if inside_envelope(s)]
    outer = [e for (s, _), e in zip(rows, errors) if not inside_envelope(s)]
    return {"centre": centre.tolist(), "errors_mm": errors.tolist(),
            "inner_median_mm": float(np.median(inner)) if inner else None,
            "outer_median_mm": float(np.median(outer)) if outer else None,
            "inner_max_mm": float(max(inner)) if inner else None,
            "outer_max_mm": float(max(outer)) if outer else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", default="auto",
                    help="auto = camera tay tự nhận (cube_vision.cameras), hoặc /dev/videoN")
    ap.add_argument("--cube-id", type=int, required=True, help="ID tag của cube đứng yên")
    ap.add_argument("--count", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--start-pose", default="READY",
                    help="pose nhìn đầu tiên (READY, LEFT, FAR_LEFT, RIGHT, FAR_RIGHT, ...; xem "
                         "pose_library.POSES): cube ở hai bên bàn không thấy được từ READY")
    ap.add_argument("--no-align", action="store_true",
                    help="bỏ bước xoay tới --start-pose và xem trực tiếp để căn cube vào giữa khung")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if str(args.camera) == "auto":
        import sys as _sys
        from pathlib import Path as _Path
        _sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
        from cube_vision.cameras import resolve_arg as _resolve_camera
        args.camera = _resolve_camera(args.camera)

    cal = M.load_calibration()
    if cal is None:
        raise SystemExit("Chưa có config/robot/hand_eye.json đạt kiểm chứng.")
    M.apply_calibration(cal)
    fx, fy, cx, cy = cal["K"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    mount = np.asarray(cal["arm4_T_optical"], float)
    z = float(cal["tag_top_z"])

    from Arm_Lib import Arm_Device
    from cube_search_center import open_camera
    cap = open_camera(args.camera)
    if cap is None:
        raise SystemExit(f"Không mở được camera {args.camera}")
    lock = C.LOCK_FILE.open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("Cổng tay máy đang bận (/tmp/t8_motion.lock); tắt T8 trước.")
    arm = Arm_Device("/dev/ttyUSB0")
    det = C._detector()
    rng = np.random.default_rng(args.seed)
    rows, log = [], []

    def observe(pose):
        servo = C.move_and_settle(arm, pose)
        corners, frame = C.grab_corners(cap, det, args.cube_id)
        if args.show and frame is not None:
            if corners is not None:
                cv2.polylines(frame, [corners.astype(int)], True, (0, 255, 0), 2)
            cv2.imshow("validate_hand_eye", frame)
            cv2.waitKey(1)
        return servo, corners

    try:
        import pose_library
        if args.start_pose not in pose_library.POSES:
            raise SystemExit(f"--start-pose phải là một trong {sorted(pose_library.POSES)}")
        start = pose_library.POSES[args.start_pose]
        if args.no_align:
            if not args.yes:
                input("[VALID] Cube có tag ID %d nằm yên, tag ngửa lên, trong tầm nhìn. "
                      "Tay sẽ tự chuyển động (không gắp). Enter để bắt đầu: " % args.cube_id)
            arm.Arm_serial_servo_write6_array(C.READY + [C.OPEN_ANGLE], 1800)
            time.sleep(2.4)
        else:
            # Tay xoay sang pose khởi đầu và bật camera: bạn đặt cube vào GIỮA khung rồi nhấn SPACE.
            C.align_preview(arm, cap, det, args.cube_id, start)
        servo, corners = observe(start)
        if corners is None:
            raise SystemExit(f"Không thấy tag ở pose {args.start_pose}.")
        ref = tag_xy(corners, servo, z)
        tag_base = np.array([ref[0], ref[1], z])
        print(f"[VALID] {args.start_pose}: cube ở ({ref[0]:+.4f}, {ref[1]:+.4f}) m")
        rows.append((servo, ref))
        pool = C.pose_pool(tag_base, mount, K, j1_values=range(20, 161, 5))
        poses = choose_poses(pool, args.count, rng)
        print(f"[VALID] {len(pool)} pose khả dụng; đi {len(poses)} pose "
              f"({sum(inside_envelope(p) for p in poses)} trong / "
              f"{sum(not inside_envelope(p) for p in poses)} ngoài vùng hiệu chuẩn)")
        for pose in poses:
            try:
                servo, corners = observe(pose)
            except RuntimeError as exc:
                print(f"[VALID] bỏ qua {pose}: {exc}")
                continue
            if corners is None:
                print(f"[VALID] {[round(v) for v in servo]}: mất tag")
                continue
            xy = tag_xy(corners, servo, z)
            rows.append((servo, xy))
            log.append({"servo": servo, "xy": xy})
            print(f"[VALID] J=[{', '.join(f'{v:5.1f}' for v in servo[:4])}] "
                  f"{'trong' if inside_envelope(servo) else 'NGOÀI'}  "
                  f"xy=({xy[0]:+.4f}, {xy[1]:+.4f})")
    finally:
        try:
            arm.Arm_serial_servo_write6_array(C.READY + [C.OPEN_ANGLE], 1800)
            time.sleep(2.0)
        except Exception:
            pass
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    if len(rows) < 4:
        raise SystemExit("Quá ít pose hợp lệ để kết luận.")
    report = summarise(rows)
    print("\n[VALID] Trung vị XY = (%+.4f, %+.4f) m" % tuple(report["centre"]))
    for name in ("inner", "outer"):
        med, mx = report[f"{name}_median_mm"], report[f"{name}_max_mm"]
        label = "TRONG vùng hiệu chuẩn" if name == "inner" else "NGOÀI vùng hiệu chuẩn"
        print(f"[VALID] {label}: " + ("không có pose" if med is None
                                      else f"lệch trung vị {med:.1f} mm, tối đa {mx:.1f} mm"))
    out = C.OUT_DIR / "hand_eye_validation.json"
    out.write_text(json.dumps({"created": time.strftime("%Y-%m-%d %H:%M:%S"),
                               "cube_id": args.cube_id, "rows": log, **report}, indent=2))
    print(f"[VALID] Đã ghi {out}")


if __name__ == "__main__":
    main()
