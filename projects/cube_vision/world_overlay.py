"""Vẽ world đè lên ảnh của một camera: phép thử trực quan "camera này có hiểu chung tọa độ world không".

Nếu pose + intrinsic của camera đúng, khung dây cube, viền tag và lưới mặt bàn (chiếu từ world do camera TAY dựng)
phải nằm trùng lên vật thật trong ảnh của camera KHÁC. Số in ở góc ảnh là độ lệch (px) giữa tag chiếu từ world và
tag camera này đang thấy.

    python -m cube_vision.world_overlay --camera phone                # camera cố định, pose đã lưu trong world
    python -m cube_vision.world_overlay --camera phone --handheld     # cầm tay: định vị lại mỗi khung bằng tag
    python -m cube_vision.world_overlay --camera ext --snapshot /tmp/ext_world.png
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from . import camera_pose as P
from . import cameras
from .world_map import WorldMap

AXIS_COLOURS = [(0, 0, 255), (0, 200, 0), (255, 0, 0)]        # x đỏ, y xanh lá, z xanh dương (BGR)


def _pt(p):
    return None if not np.isfinite(p).all() else (int(round(p[0])), int(round(p[1])))


def _line(img, a, b, colour, width=1):
    a, b = _pt(np.asarray(a, float)), _pt(np.asarray(b, float))
    if a is not None and b is not None and max(abs(a[0]), abs(a[1]), abs(b[0]), abs(b[1])) < 20000:
        cv2.line(img, a, b, colour, width, cv2.LINE_AA)


def draw(frame, world: WorldMap, model, world_T_optical, detections=None, note=""):
    """Ảnh mới có world vẽ đè. detections {id: (4,2)}: tag camera này đang thấy (vẽ xanh, so với world vàng).

    Trả (ảnh, {id: lệch px trung bình}) cho các tag có ở cả world và trong ảnh.
    """
    out = frame.copy()
    drawn = world.project_into(model, world_T_optical)
    for a, b in drawn["grid"]:
        _line(out, a, b, (200, 200, 120), 1)
    for k, (a, b) in enumerate(drawn["axes"]):
        _line(out, a, b, AXIS_COLOURS[k], 3)
    errors = {}
    for tag_id, edges in drawn["cubes"].items():
        sure = world.tags[tag_id]["sure"]
        for a, b in edges:
            _line(out, a, b, (0, 220, 255) if sure else (0, 120, 255), 2)
        quad = drawn["tags"][tag_id]
        centre = _pt(quad.mean(axis=0))
        if centre:
            x, y, z = world.tags[tag_id]["centre"] * 1000
            cv2.putText(out, f"{tag_id}: {x:+.0f},{y:+.0f},{z:+.0f}", (centre[0] + 8, centre[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2, cv2.LINE_AA)
        if detections and tag_id in detections and np.isfinite(quad).all():
            errors[tag_id] = float(np.mean(np.linalg.norm(quad - np.asarray(detections[tag_id], float), axis=1)))
    for tag_id, quad in (detections or {}).items():
        cv2.polylines(out, [np.asarray(quad, np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 1, cv2.LINE_AA)
    for zone, uv in drawn["zones"].items():
        p = _pt(uv)
        if p:
            cv2.drawMarker(out, p, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
            cv2.putText(out, f"zone {zone}", (p[0] + 8, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
    text = note + ("" if not errors else f"  lech tag: tb {np.mean(list(errors.values())):.1f} px")
    cv2.putText(out, text.strip() or "world", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.putText(out, text.strip() or "world", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    return out, errors


def grab(cap, rotate, warm=4):
    frame = None
    for _ in range(warm):
        ok, raw = cap.read()
        if ok:
            frame = raw
    return None if frame is None else cameras.rotate_frame(frame, rotate)


def detect_tags(frame, detector) -> dict:
    return {int(t["id"]): np.asarray(t["corners"], float).reshape(4, 2) for t in detector.detect(frame)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", required=True, help="tên camera có trong world (phone, ext)")
    ap.add_argument("--source", default="auto")
    ap.add_argument("--world", help="file world (mặc định data/world/latest.json)")
    ap.add_argument("--handheld", action="store_true", help="định vị lại camera ở mỗi khung bằng tag trong world")
    ap.add_argument("--snapshot", help="ghi một ảnh rồi thoát (không mở cửa sổ)")
    args = ap.parse_args()
    world = WorldMap.load(args.world)
    model = world.camera_model(args.camera)
    if model is None or not world.tags:
        raise SystemExit(f"World chưa có camera '{args.camera}' hoặc chưa có tag nào. Chạy: "
                         "python projects/vision_experiments/build_world.py --scan  rồi  --locate " + args.camera)
    from .tag import TagDetector
    detector = TagDetector(enhance=True, quiet=True)
    spec = cameras.STREAMS.get(args.camera, {"size": None, "fourcc": None})
    cap = cameras.open_stream(cameras.stream_source(args.camera, args.source), spec["size"], spec["fourcc"])
    if cap is None:
        raise SystemExit(f"Không mở được camera '{args.camera}'.")
    pose = world.cameras[args.camera]["world_T_optical"]
    try:
        grab(cap, model.rotate, warm=25)
        while True:
            frame = grab(cap, model.rotate)
            if frame is None:
                time.sleep(0.05)
                continue
            seen = detect_tags(frame, detector)
            note = "co dinh"
            if args.handheld:
                found = P.pose_from_tags(world.tag_corners(), seen, model)
                if not found["ok"]:
                    view = frame.copy()
                    cv2.putText(view, "CHUA DINH VI: " + "; ".join(found["reasons"])[:80], (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    errors = {}
                else:
                    pose, note = found["world_T_optical"], f"cam tay, {len(found['tags'])} tag"
                    view, errors = draw(frame, world, model, pose, seen, note)
            if not args.handheld:
                view, errors = draw(frame, world, model, pose, seen, note)
            if args.snapshot:
                cv2.imwrite(args.snapshot, view)
                print(f"Đã ghi {args.snapshot}. Lệch tag (px): " +
                      (", ".join(f"{i}: {e:.1f}" for i, e in sorted(errors.items())) or "không có tag chung"))
                return
            cv2.imshow(f"world -> {args.camera}", view)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                return
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
