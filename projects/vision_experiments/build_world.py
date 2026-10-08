#!/usr/bin/env python3
"""Dựng world: camera tay quét bàn, đo từng tag bằng nhiều góc nhìn, rồi đặt các camera khác vào cùng hệ.

    python projects/vision_experiments/build_world.py --scan            # tay nhìn quanh, ghi data/world/latest.json
    python projects/vision_experiments/build_world.py --locate phone    # iPhone đang ở đâu trong world (in ra)
    python projects/vision_experiments/build_world.py --locate phone --save   # ... và lưu làm camera cố định
    python projects/vision_experiments/build_world.py --check phone     # camera cố định có bị xê dịch không
    python projects/vision_experiments/build_world.py --show            # in nội dung world

Thứ tự lần đầu: calibrate_intrinsics.py --camera phone  →  --scan  →  --locate phone --save  →
python -m cube_vision.world_overlay --camera phone. Cảnh phải đứng yên giữa --scan và --locate.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for _extra in (HERE.parent, HERE):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))

import active_view as A  # noqa: E402
import cube_search_center_math as M  # noqa: E402
import dofbot_frames as D  # noqa: E402
from cube_vision import camera_pose as P  # noqa: E402
from cube_vision import cameras, intrinsics as I  # noqa: E402
from cube_vision import world_overlay as O  # noqa: E402
from cube_vision.world_map import WorldMap, snap_to_layer  # noqa: E402

DRIFT_MOVED_PX = 8.0


def camera_model(name):
    """CameraModel của camera theo tên: file ChArUco nếu có, không thì thông số fit chung cũ (chỉ webcam)."""
    model = I.model_from(I.load_camera(name), name)
    if model is None and name == "ext":
        model = D.cameras().get("ext_optical")
    return model


def scan(session, tags=None, look_around=True, use_plane=True, log=print) -> WorldMap:
    """Đo mọi tag bằng camera tay rồi trả world mới (chưa lưu).

    use_plane: tag nằm ngửa đúng tầng thì lấy độ cao đã biết của tầng thay cho độ cao đo (xem snap_to_layer).
    """
    world = WorldMap()
    try:
        world.table_z = float(json.loads(M.CALIB_FILE.read_text())["table_z"])
    except (OSError, ValueError, KeyError, TypeError):
        world.table_z = None
    results = A.measure(session.observe, session.cal, tags, look_around=look_around, log=log)
    for tag_id, result in results.items():
        log(A.describe(tag_id, result))
        fused = result["fused"]
        if not fused:
            continue
        source = "wrist"
        if use_plane:
            fused, layer = snap_to_layer(fused, float(session.cal["tag_top_z"]))
            if layer is not None:
                fused["layer"], source = layer, "wrist+plane"
                log(f"    -> nằm ngửa ở tầng {layer}: dùng độ cao đã biết {fused['centre'][2] * 1000:.1f} mm "
                    f"(đo được lệch {fused['dz_m'] * 1000:+.1f} mm)")
        world.update_tag(tag_id, fused, source, sure=bool(result["quality"]))
    return world


def add_fixed_cameras(world: WorldMap, log=print) -> None:
    """Chép các camera cố định đã hiệu chuẩn vào world để file tự chứa (RViz/điện thoại đọc được)."""
    graph, models = D.build(), D.cameras()
    for name, optical in (("ext", "ext_optical"), ("phone", "phone_optical")):
        if graph.available("world", optical) and optical in models:
            world.set_camera(name, graph.lookup("world", optical), models[optical])
            log(f"camera '{name}' đã có trong world")


def see_tags(name, source="auto"):
    """(CameraModel, {id: (4,2)}) từ một khung của camera theo tên."""
    from cube_vision.tag import TagDetector
    model = camera_model(name)
    if model is None:
        raise SystemExit(f"Camera '{name}' chưa có intrinsic đạt. Chạy: "
                         f"python projects/vision_experiments/calibrate_intrinsics.py --camera {name}")
    spec = cameras.STREAMS.get(name, {"size": None, "fourcc": None})
    cap = cameras.open_stream(cameras.stream_source(name, source), spec["size"], spec["fourcc"])
    if cap is None:
        raise SystemExit(f"Không mở được camera '{name}'.")
    try:
        frame = O.grab(cap, model.rotate, warm=30)
    finally:
        cap.release()
    if frame is None or (frame.shape[1], frame.shape[0]) != tuple(model.image_size):
        got = None if frame is None else (frame.shape[1], frame.shape[0])
        raise SystemExit(f"Camera '{name}' cho ảnh {got}, intrinsic hiệu chuẩn ở {tuple(model.image_size)}.")
    return model, O.detect_tags(frame, TagDetector(enhance=True, quiet=True))


def cmd_scan(args):
    with A.WristSession() as session:
        world = scan(session, set(args.tags or []) or None, look_around=not args.no_look_around,
                     use_plane=not args.no_plane)
    add_fixed_cameras(world)
    sure = sum(1 for t in world.tags.values() if t["sure"])
    print(f"Đã ghi {world.save()}: {len(world.tags)} tag ({sure} chắc chắn).")


def cmd_locate(args):
    world = WorldMap.load()
    if not world.tags:
        raise SystemExit("World chưa có tag nào: chạy --scan trước.")
    model, seen = see_tags(args.locate, args.source)
    found = P.pose_from_tags(world.tag_corners(), seen, model)
    print(f"Camera '{args.locate}' thấy tag {sorted(seen)}; world biết chắc tag {sorted(world.tag_corners())}.")
    if found["world_T_optical"] is not None:
        T = found["world_T_optical"]
        print(f"Vị trí trong world: x={T[0, 3] * 1000:+.0f} y={T[1, 3] * 1000:+.0f} z={T[2, 3] * 1000:+.0f} mm; "
              f"RMS chiếu lại {found['rms_px']:.2f} px trên {len(found['tags'])} tag; "
              f"các tag tách nhau {found['spread_m'] * 1000:.0f} mm")
        if found["leave_one_out_px"]:
            print("Bỏ từng tag rồi đoán lại (px): " +
                  ", ".join(f"{i}: {e:.1f}" for i, e in sorted(found["leave_one_out_px"].items())))
    print("ĐẠT" if found["ok"] else "CHƯA ĐẠT: " + "; ".join(found["reasons"]))
    if args.save:
        if not found["ok"]:
            raise SystemExit("Không lưu pose chưa đạt.")
        world.set_camera(args.locate, found["world_T_optical"], model)
        world.save()
        I.save_camera(args.locate, {"base_T_optical": np.asarray(found["world_T_optical"]).ravel().tolist(),
                                    "pose_accepted": True, "pose_rms_px": found["rms_px"],
                                    "pose_tags": found["tags"], "pose_created": time.strftime("%Y-%m-%d %H:%M:%S")})
        print(f"Đã lưu pose vào world và {I.camera_path(args.locate)}.")


def cmd_check(args):
    world = WorldMap.load()
    if args.check not in world.cameras:
        raise SystemExit(f"World chưa có camera '{args.check}': chạy --locate {args.check} --save.")
    model, seen = see_tags(args.check, args.source)
    drift = P.drift_px(world.tag_corners(), seen, model, world.cameras[args.check]["world_T_optical"])
    if drift is None:
        raise SystemExit("Không thấy tag nào có trong world: không kết luận được.")
    print(f"Tag chiếu từ world lệch {drift:.1f} px so với tag đang thấy -> " +
          ("camera (hoặc cube) ĐÃ BỊ DỜI: chạy lại --scan và --locate." if drift > DRIFT_MOVED_PX
           else "camera còn nguyên vị trí."))
    raise SystemExit(0 if drift <= DRIFT_MOVED_PX else 2)


def cmd_show(_args):
    world = WorldMap.load()
    print(f"World '{world.frame}' (bàn ở z = {world.table_z}): {len(world.tags)} tag, {len(world.zones)} ô, "
          f"camera: {', '.join(world.cameras) or 'chưa có'}")
    now = time.time()
    for tag_id, tag in sorted(world.tags.items()):
        x, y, z = tag["centre"] * 1000
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(tag["normal"][2])))))
        print(f"  tag {tag_id}: ({x:+.1f}, {y:+.1f}, {z:+.1f}) mm, nghiêng {tilt:.0f}°, ±{tag['std_m'] * 1000:.1f} mm, "
              f"{tag['n_views']} góc nhìn, {tag['source']}, {'chắc' if tag['sure'] else 'CHƯA CHẮC'}, "
              f"{now - tag['stamp']:.0f} s trước")
    for name, cam in world.cameras.items():
        x, y, z = cam["world_T_optical"][:3, 3] * 1000
        print(f"  camera {name}: ({x:+.0f}, {y:+.0f}, {z:+.0f}) mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--tags", nargs="*", type=int)
    ap.add_argument("--no-look-around", action="store_true")
    ap.add_argument("--no-plane", action="store_true", help="giữ độ cao đo 3D, không lấy độ cao đã biết của tầng")
    ap.add_argument("--locate", metavar="CAMERA")
    ap.add_argument("--save", action="store_true", help="với --locate: lưu làm pose camera cố định")
    ap.add_argument("--check", metavar="CAMERA")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--source", default="auto")
    args = ap.parse_args()
    if args.scan:
        cmd_scan(args)
    elif args.locate:
        cmd_locate(args)
    elif args.check:
        cmd_check(args)
    else:
        cmd_show(args)


if __name__ == "__main__":
    main()
