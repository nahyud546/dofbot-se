#!/usr/bin/env python3
"""Hiệu chuẩn camera ngoài ↔ base robot bằng AprilTag trên cube.

Camera tay (đã hiệu chuẩn hand-eye) định vị 4 góc từng tag trong base; camera ngoài thấy cùng các tag đó
ở pixel. Gom nhiều bộ bày cube (có bộ xếp tháp để tag nằm ở NHIỀU độ cao) rồi giải ống kính + pose.

Quy trình (mỗi lệnh chạy xong là thoát; bày lại cube giữa các lần --collect):
    python projects/vision_experiments/calibrate_external.py --reset
    python projects/vision_experiments/calibrate_external.py --collect     # bộ 1: 4 cube rải trên bàn, tag ngửa
    python projects/vision_experiments/calibrate_external.py --collect     # bộ 2: xếp tháp 2 cube (+ cube lẻ)
    python projects/vision_experiments/calibrate_external.py --collect     # bộ 3: tháp 3 cube; bộ 4...: vị trí khác
    python projects/vision_experiments/calibrate_external.py --solve       # ghi config/robot/external_camera.json
    python projects/vision_experiments/calibrate_external.py --validate    # bày vị trí MỚI, so camera ngoài với camera tay
    python projects/vision_experiments/calibrate_external.py --status      # camera có bị dời không
    python projects/vision_experiments/calibrate_external.py --relocalize  # camera bị dời: giải lại pose (giữ ống kính)

Cần: tắt perception/T8 (camera tay và cổng serial phải rảnh), tag của cube ngửa lên, cube nằm trong tầm nhìn
camera tay (giữa bàn, J1 35–140°) và camera ngoài.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
for _extra in (HERE.parent, HERE):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))

import calibrate_hand_eye as C  # noqa: E402
import cube_search_center_math as M  # noqa: E402
from cube_vision import external_camera as X  # noqa: E402

ROOT = HERE.parents[1]
SAMPLES_JSON = ROOT / "config" / "robot" / "external_camera_samples.json"
EXT_SIZE = (1280, 720)          # 640x480 không đọc nổi tag 20 mm từ camera ngoài
EXT_FOURCC = "MJPG"
READY = [90.0, 125.0, 0.0, 0.0, 90.0]
# Pose quan sát của camera tay: quét trái/giữa/phải ở hai độ nghiêng, đều trong vùng hand-eye đã kiểm chứng.
WRIST_POSES = [[j1, j2, 0.0, 0.0, 90.0] for j2 in (125.0, 110.0) for j1 in (60.0, 90.0, 120.0)]
MAX_PNP_ERR_PX = 3.0
LAYER_TOL_M = 0.012
SIDE_TOL_M = 0.003
MIN_TAG_UP = 0.85               # cos giữa pháp tuyến tag (PnP) và phương đứng: tag phải nằm ngửa (≤ ~32°)
MAX_VIEW_SPREAD_M = 0.006       # các lần nhìn của camera tay phải thống nhất vị trí tag trong mức này
OVERLAY_PNG = Path("/tmp/external_calibration_overlay.png")


# ------------------------------------------------------------------ đo bằng camera tay (thuần toán)
def wrist_tag_corners(corners_px, servo, cal):
    """4 góc tag (pixel camera tay) + khớp -> (tầng, (4,3) góc trong base) hoặc (None, lý do).

    Tầng lấy từ khoảng cách PnP (hai tầng cách nhau 30 mm, rõ ràng với camera tay ở gần); XY từng góc là
    giao tia–mặt phẳng tại đúng độ cao tầng đó (cách runtime dùng, sai số ~1–3 mm).
    """
    j1_range = cal.get("j1_valid_range")
    if j1_range and not (j1_range[0] <= servo[0] <= j1_range[1]):
        return None, f"J1={servo[0]:.0f}° ngoài vùng hand-eye đã kiểm chứng"
    fx, fy, cx, cy = cal["K"]
    Kmat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    k1 = float(cal.get("k1", 0.0))
    base_T_opt = M.fk_arm4_cal(servo, cal) @ np.asarray(cal["arm4_T_optical"], float)
    try:
        err, cam_T_tag = C.pnp_tag(corners_px, Kmat, X.TAG_SIZE_M, k1)
    except ValueError as exc:
        return None, str(exc)
    if err > MAX_PNP_ERR_PX:
        return None, f"PnP lệch {err:.1f} px"
    base_T_tag = base_T_opt @ cam_T_tag
    if abs(float(base_T_tag[2, 2])) < MIN_TAG_UP:
        tilt = math.degrees(math.acos(min(1.0, abs(float(base_T_tag[2, 2])))))
        return None, f"tag nghiêng {tilt:.0f}° (cube phải nằm phẳng, tag ngửa lên)"
    z_est = float(base_T_tag[2, 3])
    layer = int(round((z_est - cal["tag_top_z"]) / X.CUBE_EDGE_M))
    z = cal["tag_top_z"] + X.CUBE_EDGE_M * layer
    if not 0 <= layer <= X.MAX_LAYER or abs(z_est - z) > LAYER_TOL_M:
        return None, f"độ cao tag {z_est * 1000:.0f} mm không khớp tầng nào"
    pts = np.array([C.ray_plane_xy(u, v, base_T_opt, Kmat, k1, z)
                    for u, v in np.asarray(corners_px, float).reshape(4, 2)])
    sides = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
    if np.max(np.abs(sides - X.TAG_SIZE_M)) > SIDE_TOL_M:
        return None, f"cạnh tag đo được {sides.min() * 1000:.0f}–{sides.max() * 1000:.0f} mm (tag nghiêng?)"
    return layer, pts


def fuse_views(views):
    """views: [(tầng, (4,3))] của MỘT tag qua nhiều pose camera tay -> (tầng, (4,3) trung vị, spread_m) hoặc None."""
    if not views:
        return None
    layers = [layer for layer, _ in views]
    layer = max(set(layers), key=layers.count)
    stack = np.array([pts for l, pts in views if l == layer])
    if len(stack) < max(1, (len(views) + 1) // 2):
        return None                                       # các lần nhìn không thống nhất tầng
    median = np.median(stack, axis=0)
    spread = float(np.max(np.linalg.norm(stack - median, axis=2)))
    if spread > MAX_VIEW_SPREAD_M:
        return None
    return layer, median, spread


def samples_to_arrays(samples):
    """File mẫu -> (points (N,3), pixels (N,2), nhóm [(bộ, tag)])."""
    points, pixels, groups = [], [], []
    for index, item in enumerate(samples.get("sets", [])):
        for tag in item.get("tags", []):
            points += tag["base"]
            pixels += tag["px"]
            groups += [(index, int(tag["id"]))] * 4
    return np.array(points, float).reshape(-1, 3), np.array(pixels, float).reshape(-1, 2), groups


# ------------------------------------------------------------------ phần cứng
def _detector():
    from cube_vision.tag import TagDetector
    return TagDetector(enhance=True, quiet=True)


def _stable_tags(cap, det, frames=3):
    """{tag_id: (4,2)} các tag đứng yên qua vài khung tươi của camera tay."""
    for _ in range(4):
        cap.grab()
    seen = {}
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            return {}
        if (frame.shape[1], frame.shape[0]) != (640, 480):
            frame = cv2.resize(frame, (640, 480))
        for tag in det.detect(frame):
            seen.setdefault(int(tag["id"]), []).append(np.asarray(tag["corners"], float).reshape(4, 2))
    out = {}
    for tag_id, stack in seen.items():
        stack = np.array(stack)
        inside = stack.min() >= 8 and stack[..., 0].max() <= 632 and stack[..., 1].max() <= 472
        if len(stack) == frames and inside and float(np.max(np.abs(stack - stack.mean(axis=0)))) <= 1.5:
            out[tag_id] = stack.mean(axis=0)
    return out


def measure_scene(log=print):
    """Một bộ mẫu: camera tay quét các pose -> góc tag trong base; về READY -> ảnh camera ngoài -> ghép theo ID.

    Trả {"tags": [...], "landmarks": {...}, "skipped": {...}, "frame": ảnh camera ngoài}.
    """
    import fcntl
    from cube_vision import cameras
    from cube_vision.placement_check import ExternalCamera
    cal = M.load_calibration()
    if cal is None:
        raise SystemExit("Chưa có hand-eye camera tay đạt (config/robot/hand_eye.json).")
    devices = cameras.list_cameras()
    wrist, _ = cameras.resolve_wrist(devices, identify=lambda: cameras.identify_with_arm(devices))
    external = cameras.resolve_external(wrist, devices=devices)
    if not wrist or not external:
        raise SystemExit("Cần cả camera tay và camera ngoài (python -m cube_vision.cameras --identify).")
    lock = C.LOCK_FILE.open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("Cổng tay máy đang bận (tắt T8/perception trước).")
    sys.path.insert(0, str(ROOT / "vendor" / "yahboom"))
    from Arm_Lib import Arm_Device
    arm = Arm_Device("/dev/ttyUSB0")
    det = _detector()
    cap = cv2.VideoCapture(wrist, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    views, why = {}, {}
    try:
        if not cap.isOpened():
            raise SystemExit(f"Không mở được camera tay {wrist}.")
        for pose in WRIST_POSES:
            servo = C.move_and_settle(arm, pose)
            for tag_id, corners in _stable_tags(cap, det).items():
                layer, pts = wrist_tag_corners(corners, servo, cal)
                if layer is None:
                    why[tag_id] = pts
                else:
                    views.setdefault(tag_id, []).append((layer, pts))
        C.move_and_settle(arm, READY, ms=1800)             # READY không che bàn khỏi camera ngoài
    finally:
        cap.release()
        try:
            del arm
        except Exception:  # noqa: BLE001
            pass
        lock.close()
    frame = ExternalCamera(external, width=EXT_SIZE[0], height=EXT_SIZE[1], fourcc=EXT_FOURCC).grab()
    if frame is None or (frame.shape[1], frame.shape[0]) != EXT_SIZE:
        raise SystemExit(f"Camera ngoài {external} không cho ảnh {EXT_SIZE[0]}x{EXT_SIZE[1]}.")
    ext = {int(t["id"]): np.asarray(t["corners"], float).reshape(4, 2) for t in det.detect(frame)}
    tags, skipped = [], {}
    for tag_id in sorted(set(views) | set(ext) | set(why)):
        fused = fuse_views(views.get(tag_id, []))
        if fused is None:
            skipped[tag_id] = why.get(tag_id) or ("camera tay không thấy ổn định" if tag_id not in views
                                                  else "các lần nhìn của camera tay không thống nhất")
        elif tag_id not in ext:
            skipped[tag_id] = "camera ngoài không thấy tag"
        else:
            layer, pts, spread = fused
            tags.append({"id": tag_id, "layer": layer, "base": pts.tolist(), "px": ext[tag_id].tolist(),
                         "wrist_views": len(views[tag_id]), "spread_mm": round(spread * 1000, 1)})
    for tag in tags:
        log(f"  tag {tag['id']}: tầng {tag['layer']}, {tag['wrist_views']} lần nhìn, lệch giữa các lần "
            f"{tag['spread_mm']} mm")
    for tag_id, reason in skipped.items():
        log(f"  tag {tag_id}: BỎ ({reason})")
    return {"tags": tags, "landmarks": X.pad_landmarks(frame), "skipped": skipped, "frame": frame,
            "device": next((d["name"] for d in devices if d["path"] == external), external)}


def load_samples():
    try:
        return json.loads(SAMPLES_JSON.read_text())
    except (OSError, ValueError):
        return {"image_size": list(EXT_SIZE), "sets": []}


def draw_overlay(frame, cal, points, pixels, path=OVERLAY_PNG):
    """Vẽ điểm đo (xanh) và điểm chiếu lại từ hiệu chuẩn (đỏ) lên ảnh camera ngoài để xem bằng mắt."""
    out = frame.copy()
    for (u, v), (pu, pv) in zip(pixels, cal.project(points)):
        cv2.circle(out, (int(round(u)), int(round(v))), 4, (0, 255, 0), 1)
        cv2.circle(out, (int(round(pu)), int(round(pv))), 2, (0, 0, 255), -1)
    for x in np.arange(-0.30, -0.049, 0.05):                # lưới mặt bàn mỗi 5 cm
        a, b = cal.project([x, -0.15, cal.tag_top_z - X.CUBE_EDGE_M]), cal.project([x, 0.15, cal.tag_top_z - X.CUBE_EDGE_M])
        cv2.line(out, tuple(int(v) for v in a), tuple(int(v) for v in b), (255, 200, 0), 1)
    cv2.imwrite(str(path), out)
    return path


# ------------------------------------------------------------------ lệnh
def cmd_collect(args):
    samples = {"image_size": list(EXT_SIZE), "sets": []} if args.reset else load_samples()
    print(f"Thu bộ mẫu {len(samples['sets']) + 1} (đang có {len(samples['sets'])} bộ)...")
    scene = measure_scene()
    if not scene["tags"]:
        raise SystemExit("Không có tag nào được cả hai camera thấy; không lưu bộ này.")
    cv2.imwrite("/tmp/external_last_frame.png", scene["frame"])
    samples["sets"].append({"tags": scene["tags"], "landmarks": scene["landmarks"],
                            "time": time.strftime("%Y-%m-%d %H:%M:%S")})
    samples["device"] = scene["device"]
    SAMPLES_JSON.write_text(json.dumps(samples, ensure_ascii=False, indent=1))
    points, _, groups = samples_to_arrays(samples)
    levels = X.z_levels(points)
    print(f"Đã lưu: {len(samples['sets'])} bộ, {len(set(groups))} tag-mẫu, {len(points)} điểm, "
          f"{len(levels)} độ cao. " + ("Đủ để --solve." if len(levels) >= X.MIN_Z_LEVELS and
                                         len(points) >= X.MIN_POINTS and len(set(groups)) >= 5
                                         else "Chưa đủ: cần ≥ 3 độ cao (xếp tháp), ≥ 24 điểm, ≥ 5 tag-mẫu."))


def cmd_solve(_args):
    samples = load_samples()
    points, pixels, groups = samples_to_arrays(samples)
    if len(points) < 6:
        raise SystemExit("Chưa có mẫu: chạy --collect trước.")
    result = X.solve(points, pixels, samples.get("image_size", EXT_SIZE), groups)
    hand_eye = M.load_calibration()
    landmarks = next((s["landmarks"] for s in reversed(samples["sets"]) if s.get("landmarks")), {})
    cal = X.build(result, hand_eye["tag_top_z"] if hand_eye else 0.0578, landmarks,
                  samples.get("device", ""), len(samples["sets"]))
    path = cal.save()
    print(report(result))
    frame = cv2.imread("/tmp/external_last_frame.png")
    if frame is not None and cal.matches(frame):
        print(f"Ảnh kiểm tra bằng mắt (xanh = đo, đỏ = chiếu lại, lưới 5 cm trên mặt bàn): "
              f"{draw_overlay(frame, cal, points, pixels)}")
    print(f"Đã ghi {path} (accepted={cal.accepted}).")
    raise SystemExit(0 if cal.accepted else 2)


def report(result) -> str:
    T = np.array(result["base_T_ext"])
    lines = [f"Điểm: {result['n_points']} ({result['n_groups']} tag-mẫu), độ cao: {result['z_levels_m']} m",
             f"Ống kính: f={result['K'][0]:.0f} px, tâm=({result['K'][1]:.0f}, {result['K'][2]:.0f}), k1={result['k1']:+.3f}",
             f"Camera trong base: x={T[0, 3]:+.3f} y={T[1, 3]:+.3f} z={T[2, 3]:+.3f} m",
             f"RMS chiếu lại: {result['fit_rms_px']:.2f} px"]
    if result["holdout_m"]:
        mm = np.array(result["holdout_m"]) * 1000
        lines.append(f"Kiểm định ({len(mm)} điểm không tham gia fit): trung vị {np.median(mm):.1f} mm, "
                     f"lớn nhất {mm.max():.1f} mm; {np.median(result['holdout_px']):.1f} px")
    lines.append("ĐẠT" if result["accepted"] else "CHƯA ĐẠT: " + "; ".join(result["reasons"]))
    return "\n".join(lines)


def _require_calibration(require_accepted=True):
    cal = X.ExternalCalibration.load(require_accepted=require_accepted)
    if cal is None:
        raise SystemExit("Chưa có hiệu chuẩn camera ngoài" + (" đạt tiêu chí" if require_accepted else "") +
                         " (chạy --collect nhiều bộ rồi --solve).")
    return cal


def cmd_validate(args):
    cal = _require_calibration(require_accepted=not args.allow_unaccepted)
    scene = measure_scene()
    if cal.moved(scene["frame"]):
        print(f"CẢNH BÁO: camera ngoài đã bị dời ({cal.drift_px(scene['frame']):.0f} px) từ lúc hiệu chuẩn.")
    errors = []
    print("tag  tầng   camera tay (x, y) mm     camera ngoài (x, y) mm    lệch mm")
    for tag in scene["tags"]:
        truth = np.mean(np.array(tag["base"])[:, :2], axis=0)
        found = cal.locate_tag(tag["px"], layer=tag["layer"])
        if found is None:
            continue
        err = float(np.hypot(found["x"] - truth[0], found["y"] - truth[1])) * 1000
        errors.append(err)
        print(f"{tag['id']:>3}  {tag['layer']:>4}   ({truth[0] * 1000:+7.1f}, {truth[1] * 1000:+7.1f})      "
              f"({found['x'] * 1000:+7.1f}, {found['y'] * 1000:+7.1f})      {err:5.1f}")
    if not errors:
        raise SystemExit("Không có tag nào để so.")
    print(f"Lệch trung vị {np.median(errors):.1f} mm, lớn nhất {max(errors):.1f} mm trên {len(errors)} tag.")


def cmd_status(_args):
    from cube_vision import cameras
    from cube_vision.placement_check import ExternalCamera
    cal = _require_calibration(require_accepted=False)
    wrist, _ = cameras.resolve_wrist()
    frame = ExternalCamera(cameras.resolve_external(wrist), width=cal.image_size[0], height=cal.image_size[1],
                           fourcc=EXT_FOURCC).grab()
    if frame is None:
        raise SystemExit("Không đọc được camera ngoài.")
    drift = cal.drift_px(frame)
    print(f"Hiệu chuẩn {cal.meta.get('created', '?')}: accepted={cal.accepted}, RMS {cal.meta.get('fit_rms_px', 0):.2f} px")
    print("Không đủ ô màu để kiểm tra trôi." if drift is None else
          f"Tâm ô màu lệch {drift:.1f} px so với lúc hiệu chuẩn -> " +
          ("camera ĐÃ BỊ DỜI, chạy --relocalize." if drift > X.MAX_DRIFT_PX else "camera còn nguyên vị trí."))
    raise SystemExit(0 if cal.accepted and (drift is None or drift <= X.MAX_DRIFT_PX) else 2)


def cmd_relocalize(_args):
    cal = _require_calibration(require_accepted=False)
    scene = measure_scene()
    points = np.array([p for t in scene["tags"] for p in t["base"]], float)
    pixels = np.array([p for t in scene["tags"] for p in t["px"]], float)
    if len(points) < 12:
        raise SystemExit("Cần ≥ 3 tag được cả hai camera thấy để giải lại pose.")
    T, rms = X.relocalize(points, pixels, cal)
    print(f"Pose mới: RMS chiếu lại {rms:.2f} px trên {len(points)} điểm.")
    if rms > X.MAX_FIT_RMS_PX:
        raise SystemExit("RMS quá lớn: không ghi (ống kính/độ phân giải đã đổi? hiệu chuẩn lại từ đầu).")
    cal.base_T_ext, cal.landmarks = T, scene["landmarks"]
    cal.meta.update(relocalized=time.strftime("%Y-%m-%d %H:%M:%S"), relocalize_rms_px=rms)
    print(f"Đã ghi {cal.save()}.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--collect", action="store_true", help="thu một bộ mẫu với cách bày cube hiện tại")
    ap.add_argument("--reset", action="store_true", help="xóa các bộ mẫu cũ (dùng riêng hoặc kèm --collect)")
    ap.add_argument("--solve", action="store_true")
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--relocalize", action="store_true")
    ap.add_argument("--allow-unaccepted", action="store_true", help="--validate cả khi hiệu chuẩn chưa đạt")
    args = ap.parse_args()
    if args.collect:
        cmd_collect(args)
    elif args.reset:
        SAMPLES_JSON.write_text(json.dumps({"image_size": list(EXT_SIZE), "sets": []}))
        print("Đã xóa các bộ mẫu.")
    elif args.solve:
        cmd_solve(args)
    elif args.validate:
        cmd_validate(args)
    elif args.status:
        cmd_status(args)
    elif args.relocalize:
        cmd_relocalize(args)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
