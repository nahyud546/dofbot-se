"""Vẽ world đè lên ảnh của một camera: phép thử trực quan "camera này có hiểu chung tọa độ world không".

Nếu pose + intrinsic của camera đúng, khung dây cube, viền tag và lưới mặt bàn (chiếu từ world do camera TAY dựng)
phải nằm trùng lên vật thật trong ảnh của camera KHÁC. Số in ở góc ảnh là độ lệch (px) giữa tag chiếu từ world và
tag camera này đang thấy.

    python -m cube_vision.world_overlay --camera phone                # theo dõi: camera dời và cube dời đều theo kịp
    python -m cube_vision.world_overlay --camera phone --fixed        # dùng pose đã lưu (kiểm camera có bị dời không)
    python -m cube_vision.world_overlay --camera phone --write-live data/world/live.json   # cho RViz theo dõi
    python -m cube_vision.world_overlay --camera ext --snapshot /tmp/ext_world.png

Phím: q/Esc thoát; s lưu khung hiện tại (ảnh thô, ảnh vẽ, góc tag, kết quả theo dõi) vào data/world/debug/ để phát
lại khi thấy vẽ sai.
"""
from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np

from . import camera_pose as P
from . import cameras
from .world_map import WorldMap, default_path

AXIS_COLOURS = [(0, 0, 255), (0, 200, 0), (255, 0, 0)]        # x đỏ, y xanh lá, z xanh dương (BGR)


def _pt(p):
    return None if not np.isfinite(p).all() else (int(round(p[0])), int(round(p[1])))


def _line(img, a, b, colour, width=1):
    a, b = _pt(np.asarray(a, float)), _pt(np.asarray(b, float))
    if a is not None and b is not None and max(abs(a[0]), abs(a[1]), abs(b[0]), abs(b[1])) < 20000:
        cv2.line(img, a, b, colour, width, cv2.LINE_AA)


def draw(frame, world: WorldMap, model, world_T_optical, detections=None, note="", missing=()):
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
        tag = world.tags[tag_id]
        if tag_id in missing:
            colour, suffix = (150, 150, 150), " (khong thay)"
        elif str(tag.get("source", "")).startswith("phone-live"):
            colour, suffix = (0, 140, 255), " (do live)"
        else:
            colour, suffix = ((0, 220, 255) if tag["sure"] else (0, 120, 255)), ""
        for a, b in edges:
            _line(out, a, b, colour, 2)
        quad = drawn["tags"][tag_id]
        centre = _pt(quad.mean(axis=0))
        if centre:
            x, y, z = tag["centre"] * 1000
            tilt = float(np.degrees(np.arccos(np.clip(tag["normal"][2], -1.0, 1.0))))
            if suffix == " (do live)":
                suffix = f" (do live, nghieng {tilt:.0f})"
            cv2.putText(out, f"{tag_id}: {x:+.0f},{y:+.0f},{z:+.0f}{suffix}", (centre[0] + 8, centre[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2, cv2.LINE_AA)
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


def _mtime(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


def save_debug(frame, view, seen, info, folder=None):
    """Lưu khung thô, ảnh vẽ, góc tag và kết quả theo dõi để phát lại offline (phím `s`). Trả đường dẫn thư mục."""
    import json
    from pathlib import Path
    from .world_map import default_path
    folder = Path(folder or default_path().parent / "debug" / time.strftime("%Y%m%d-%H%M%S"))
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / "frame.png"), frame)
    cv2.imwrite(str(folder / "overlay.jpg"), view)
    data = {"tags": {str(i): np.asarray(c).tolist() for i, c in seen.items()}}
    if info is not None:
        data.update(ok=info["ok"], anchors=info["anchors"], moved=info["moved"], new=info["new"],
                    missing=info["missing"], rms_px=info["rms_px"], reasons=info["reasons"],
                    pose=None if info["pose"] is None else np.asarray(info["pose"]).tolist())
        data["live_tags"] = {str(i): {"centre": np.asarray(t["centre"]).tolist(), "normal": np.asarray(t["normal"]).tolist(),
                                      "layer": t.get("layer"), "source": t.get("source")}
                             for i, t in info["world"].tags.items() if t.get("source") == "phone-live"}
    (folder / "info.json").write_text(json.dumps(data, indent=1))
    return str(folder)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", required=True, help="tên camera có trong world (phone, ext)")
    ap.add_argument("--source", default="auto")
    ap.add_argument("--world", help="file world (mặc định data/world/latest.json)")
    ap.add_argument("--fixed", action="store_true",
                    help="dùng pose camera đã lưu trong world, không theo dõi (kiểm camera cố định có bị dời không)")
    ap.add_argument("--handheld", action="store_true", help=argparse.SUPPRESS)       # tên cũ; giờ là mặc định
    ap.add_argument("--write-live", metavar="FILE", help="ghi world sống (camera + cube dời) ra file này ~2 lần/giây")
    ap.add_argument("--snapshot", help="ghi một ảnh rồi thoát (không mở cửa sổ)")
    args = ap.parse_args()
    world = WorldMap.load(args.world)
    model = world.camera_model(args.camera)
    if model is None or not world.tags:
        raise SystemExit(f"World chưa có camera '{args.camera}' hoặc chưa có tag nào. Chạy: "
                         "python projects/vision_experiments/build_world.py --scan  rồi  --locate " + args.camera)
    from .live_world import LiveWorld
    from .tag import TagDetector
    detector = TagDetector(enhance=True, quiet=True)
    spec = cameras.STREAMS.get(args.camera, {"size": None, "fourcc": None})
    source = cameras.stream_source(args.camera, args.source)
    cap = cameras.open_stream(source, spec["size"], spec["fourcc"])
    if cap is None:
        raise SystemExit(f"Không mở được camera '{args.camera}'. (DroidCam chỉ cho một kết nối: tắt cửa sổ khác.)")
    lost = 0
    world_path = args.world or default_path()
    loaded, checked = _mtime(world_path), time.time()
    live = None if args.fixed else LiveWorld(world, model, args.camera)
    pose = world.cameras[args.camera]["world_T_optical"]
    last_write = 0.0
    try:
        grab(cap, model.rotate, warm=25)
        while True:
            frame = grab(cap, model.rotate)
            if frame is None:
                lost += 1
                time.sleep(0.05)
                if lost >= 20:                      # luồng mạng (DroidCam) đứt: mở lại thay vì chờ mãi
                    print("Mất luồng camera, đang kết nối lại...")
                    cap.release()
                    cap, lost = None, 0
                    while cap is None:
                        time.sleep(1.0)
                        cap = cameras.open_stream(source, spec["size"], spec["fourcc"])
                    grab(cap, model.rotate, warm=10)
                continue
            lost = 0
            if time.time() - checked > 0.5:                 # world đổi (camera tay vừa ghi thêm/dời cube): nạp lại
                checked = time.time()
                stamp = _mtime(world_path)
                if stamp != loaded and stamp is not None:
                    fresh = WorldMap.load(world_path)
                    if fresh.tags:
                        fresh.cameras.setdefault(args.camera, world.cameras[args.camera])
                        world, loaded = fresh, stamp
                        if live is not None:
                            live.world = world
            seen = detect_tags(frame, detector)
            errors = {}
            if live is None:
                view, errors = draw(frame, world, model, pose, seen, "co dinh")
            else:
                info = live.update(seen)
                if info["ok"]:
                    note = f"theo doi: {len(info['anchors'])} tag neo, rms {info['rms_px']:.1f}px"
                    if info["moved"] or info["new"]:
                        note += f", cube doi {sorted(info['moved'])} moi {sorted(info['new'])}"
                    view, errors = draw(frame, info["world"], model, info["pose"], seen, note, info["missing"])
                    if args.write_live and time.time() - last_write > 0.5:
                        info["world"].save(args.write_live)
                        last_write = time.time()
                else:
                    view = frame.copy()
                    cv2.putText(view, "CHUA DINH VI: " + "; ".join(info["reasons"])[:90], (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if args.snapshot:
                cv2.imwrite(args.snapshot, view)
                if live is not None:
                    print("neo: %s | cube dời: %s | mới: %s | không thấy: %s | RMS %s px" % (
                        info["anchors"], info["moved"], info["new"], info["missing"],
                        "-" if info["rms_px"] is None else f"{info['rms_px']:.1f}"), "|", "; ".join(info["reasons"]))
                print(f"Đã ghi {args.snapshot}. Lệch tag (px): " +
                      (", ".join(f"{i}: {e:.1f}" for i, e in sorted(errors.items())) or "không có tag chung"))
                return
            cv2.imshow(f"world -> {args.camera}", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                print("Đã lưu khung để chẩn đoán:", save_debug(frame, view, seen, info if live is not None else None))
            if key in (ord("q"), 27):
                return
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
