#!/usr/bin/env python3
"""Dựng world: camera tay quét bàn, đo từng tag bằng nhiều góc nhìn, rồi đặt các camera khác vào cùng hệ.

    python projects/vision_experiments/build_world.py --scan            # tay nhìn quanh, ghi data/world/latest.json
    python projects/vision_experiments/build_world.py --watch           # chỉ nhìn: tay ở pose nào, thấy gì ghi nấy
    python projects/vision_experiments/build_world.py --watch --goto 70 110 10 0 90   # đi tới pose này rồi nhìn
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
from cube_vision.world_map import WorldMap, default_path, snap_to_layer  # noqa: E402

DRIFT_MOVED_PX = 8.0
WATCH_MIN_SHARPNESS = 25.0      # ảnh camera tay nhòe/tối hơn mức này thì "không thấy cube" không có nghĩa là cube mất
WATCH_MIN_BRIGHTNESS = 25
FACE_COLOUR_PERIOD_S = 0.5      # tìm mặt màu + khớp hình học: 2 lần/giây là đủ cho cảnh tĩnh
FACE_DINO_PERIOD_S = 4.0        # bộ nhận hình in (DINO) nặng: thưa hơn
WATCH_FACE_BRIGHTNESS = 90      # đo thật: ở 63 bộ nhận mặt không thấy cube lật mặt, ở 154 thấy đủ.0


def camera_model(name):
    """CameraModel của camera theo tên: file ChArUco nếu có, không thì thông số fit chung cũ (chỉ webcam)."""
    model = I.model_from(I.load_camera(name), name)
    if model is None and name == "ext":
        model = D.cameras().get("ext_optical")
    return model


def scan(session, tags=None, look_around=True, use_plane=True, log=print, faces=True, keep_dir=None,
         objects=True, look_objects=True) -> WorldMap:
    """Đo mọi tag bằng camera tay rồi trả world mới (chưa lưu).

    use_plane: tag nằm ngửa đúng tầng thì lấy độ cao đã biết của tầng thay cho độ cao đo (xem snap_to_layer).
    faces: ở mỗi pose quét chụp thêm 2 khung, nhận cube không ngửa tag bằng mặt màu / hình in.
    keep_dir: lưu khung + góc khớp của từng pose quét vào thư mục này (dữ liệu cho việc dựng hình vật bất kỳ).
    """
    world = WorldMap()
    world.region, area = A.scan_region(session.cal)
    log(f"Vùng world (mặt bàn camera tay quét được): {area * 1e4:.0f} cm². Vật ngoài vùng này không được ghi.")
    shots = []                                                  # [(khớp thật, [khung], {tag thấy})] của các pose quét
    grab_frame = getattr(session, "frame", None)

    def observe(servo):
        real, seen = session.observe(servo)
        if grab_frame is not None and (faces or keep_dir or objects) and len(shots) < len(A.SCAN_POSES):
            frames = [f for f in (grab_frame(), grab_frame()) if f is not None]
            shots.append(([float(v) for v in real[:5]], frames, set(seen)))
        return real, seen
    try:
        world.table_z = float(json.loads(M.CALIB_FILE.read_text())["table_z"])
    except (OSError, ValueError, KeyError, TypeError):
        world.table_z = None
    results = A.measure(observe, session.cal, tags, look_around=look_around, log=log)
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
        if not world.inside(fused["centre"][:2]):
            log(f"    -> tag {tag_id} nằm ngoài vùng world: không ghi")
            continue
        world.update_tag(tag_id, fused, source, sure=bool(result["quality"]))
    if keep_dir and shots:
        import cv2
        keep_dir = Path(keep_dir)
        keep_dir.mkdir(parents=True, exist_ok=True)
        for index, (real, frames, _) in enumerate(shots):
            if frames:
                cv2.imwrite(str(keep_dir / f"pose{index:02d}.png"), frames[0])
        (keep_dir / "poses.json").write_text(json.dumps([real for real, _, _ in shots]))
        log(f"Đã lưu {len(shots)} khung quét vào {keep_dir}")
    if faces and shots:
        scan_faces(session.cal, world, shots, log)
    if objects and shots:
        camera = A.wrist_camera(session.cal)
        frames = [(shot[1][0], camera, A.base_T_optical(shot[0], session.cal)) for shot in shots if shot[1]]
        found = scan_objects(session.cal, world, frames, log)
        if found and look_objects and grab_frame is not None:
            extra = look_at_objects(session, found, frames, log, keep_dir)
            if extra:
                log(f"Dựng lại vật với {extra} khung chụp thêm quanh vật:")
                scan_objects(session.cal, world, frames, log, strict=True)
    return world


MAX_LOOK_OBJECTS = 3            # chỉ chụp thêm quanh chừng này vật (vật nhiều điểm nhất trước)


def look_at_objects(session, found, frames, log=print, keep_dir=None) -> int:
    """Tay tự đi quanh từng vật khác cube và chụp thêm khung có vật nằm GIỮA ảnh (chỉ nhìn, không gắp).

    Khung quét cố định thường cắt mất đỉnh hoặc đáy vật và chồng nhau ít; các khung này dày và trải rộng quanh vật
    nên đám mây điểm của vật dày hơn và thấy cả hai bên. Thêm thẳng vào `frames`; trả số khung đã thêm.
    """
    import cv2
    cal, camera = session.cal, A.wrist_camera(session.cal)
    table_z = float(cal["tag_top_z"]) - 0.030
    added, poses = 0, []
    for item in found[:MAX_LOOK_OBJECTS]:
        target = np.r_[item["centre"], table_z + min(float(item["height_m"]), 0.12) / 2.0]
        views = A.object_views(target, cal)
        x, y = item["centre"] * 1000
        log(f"  nhìn quanh vật '{item['label']}' ở ({x:+.0f}, {y:+.0f}) mm: {len(views)} pose")
        for servo in views:
            real, _ = session.observe(servo)
            frame = session.frame()
            if frame is None:
                continue
            real = [float(v) for v in real[:5]]
            frames.append((frame, camera, A.base_T_optical(real, cal)))
            if keep_dir:
                cv2.imwrite(str(Path(keep_dir) / f"object{added:02d}.png"), frame)
                poses.append(real)
            added += 1
    if keep_dir and poses:
        (Path(keep_dir) / "object_poses.json").write_text(json.dumps(poses))
    return added


_LABELLER = []                  # YOLOE nạp một lần cho cả hai lượt dựng vật


def scan_objects(cal, world, frames, log=print, labeller=None, strict=False) -> list:
    """Vật không phải cube: nối các khung quét thành đám mây điểm 3D (`cube_vision.pointcloud`), tách cụm, khớp hình.

    labeller: tùy chọn, có `.detect(khung) -> [(nhãn, độ tin, mặt nạ)]` (YOLOE) để đặt tên vật; không có thì "vật".
    """
    from cube_vision import pointcloud as PC
    camera, table_z = A.wrist_camera(cal), float(cal["tag_top_z"]) - 0.030
    # Lượt đầu (ít khung, chồng nhau ít) chỉ để biết vật ở đâu mà nhìn quanh: chấp nhận điểm từ 2 khung. Lượt cuối
    # (strict) có khung dày quanh vật: chỉ giữ điểm được ≥ 3 khung xác nhận với thị sai ≥ 6°, đám điểm gọn hơn nhiều
    # (đo thật: mặt trước bịch khăn nhòe 79 mm dọc hướng nhìn còn 24 mm).
    cloud = PC.triangulate(frames, min_frames=3, min_parallax_deg=6.0) if strict else PC.triangulate(frames)
    cubes = [world.entry(i)["centre"][:2] for i in world.cube_ids()]
    found = PC.objects(cloud, table_z, cubes, world.inside)
    log(f"Đám mây điểm từ {len(frames)} khung: {len(cloud['points'])} điểm 3D, {len(found)} vật khác cube.")
    if found and labeller is None:
        if not _LABELLER:
            try:
                import object_masks
                _LABELLER.append(object_masks.ObjectMasks())
            except Exception:  # noqa: BLE001 - không có ultralytics (python hệ thống): vật vẫn có hình, chỉ thiếu tên
                _LABELLER.append(None)
        labeller = _LABELLER[0]
    if found and labeller is not None:
        seen = [(T, labeller.detect(image)) for image, _, T in frames]
        for item in found:                                      # nhãn: lớp được nhiều khung gọi nhất tại thân vật
            names, probe = {}, np.r_[item["centre"], table_z + item["height_m"] / 2.0, 1.0]
            for T, instances in seen:
                uv = camera.project((np.linalg.inv(T) @ probe)[:3].reshape(1, 3))[0]
                if not np.isfinite(uv).all() or not (0 <= uv[0] < 640 and 0 <= uv[1] < 480):
                    continue
                for label, confidence, mask in instances:
                    if mask[int(uv[1]), int(uv[0])]:
                        names[label] = names.get(label, 0.0) + confidence
            if names:
                item["label"] = max(names, key=names.get)
        # Đám mây chỉ có điểm ở chỗ có hoa văn (thường là dải giữa mặt trước) và nhiễu vài mm theo chiều sâu, nên tự nó
        # không đủ để phân biệt mặt cong với mặt phẳng. Vật có nhãn tròn hoặc cung khớp được: dựng trụ từ mặt gần
        # của đám điểm + bề ngang thật đo ở viền vật trong ảnh.
        for index, item in enumerate(found):
            if item["shape"] != "cylinder" and item["label"] not in object_masks_round():
                continue
            probe = np.r_[item["centre"], table_z + item["height_m"] / 2.0]
            widths = [PC.silhouette_width(camera, T, mask, probe) for T, instances in seen for _, _, mask in instances]
            widths = [v for v in widths if v and v / 2.0 <= PC.CIRCLE_RADIUS_M[1]]
            if not widths:
                continue
            found[index] = PC.cylinder_from_near_edge(item, float(np.median(widths)) / 2.0)
            found[index]["fit"] = (f"trụ: mặt gần từ {item['n_points']} điểm 3D, đường kính từ viền vật trong "
                                   f"{len(widths)} khung")
    world.set_objects(found, "wrist-scan")
    for item in world.objects:
        x, y = item["centre"] * 1000
        log(f"  vật '{item['label']}' ({'trụ' if item['shape'] == 'cylinder' else 'hộp'}): tâm ({x:+.0f}, {y:+.0f}) mm, "
            f"ngang ~{item['width_m'] * 1000:.0f} mm, cao ~{item['height_m'] * 1000:.0f} mm, {item['n_points']} điểm 3D từ "
            f"{item['n_views']} khung ({item['fit']})")
    return found


def object_masks_round() -> tuple:
    """Nhãn YOLOE của vật thân tròn: dựng thành hình trụ."""
    return ("cup", "mug", "bottle", "can")


def scan_faces(cal, world, shots, log=print) -> None:
    """Cube không ngửa tag trong các khung quét: nhận bằng mặt, ghi vào world (hai khung phải thống nhất)."""
    import world_watch as W
    from cube_vision import color, registry
    try:
        from cube_vision.identify import FULL, Identifier
        identifier = Identifier(FULL)
    except Exception as exc:  # noqa: BLE001
        log(f"Không nạp được bộ nhận mặt ({exc}): world chỉ có cube đọc được tag.")
        return
    watcher = W.Watcher(cal, world)
    stamp = time.time()
    for real, frames, seen in shots:
        for frame in frames:
            found, tag_ids = W.faces_from_detections(identifier.detect(frame))
            found += color.colour_faces(frame, registry.COLOR_TO_ID)       # mặt màu ngửa lên hoặc quay ngang
            stamp += 1.0
            events = watcher.observe_faces(real, found, tag_ids | set(seen) | set(world.tags), stamp=stamp,
                                           clear_view=False)
            for cube_id in events["new"]:
                face = world.faces[cube_id]
                x, y, z = face["centre"] * 1000
                log(f"  cube {cube_id}: thấy mặt {face['label']} (không đọc tag) ở ({x:+.1f}, {y:+.1f}, {z:+.1f}) mm, "
                    f"tầng {face['layer']}")


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
        keep = default_path().parent / "scan" / time.strftime("%Y%m%d-%H%M%S")
        world = scan(session, set(args.tags or []) or None, look_around=not args.no_look_around,
                     use_plane=not args.no_plane, faces=not args.no_faces, keep_dir=keep,
                     objects=not args.no_objects, look_objects=not args.no_object_looks)
    add_fixed_cameras(world)
    sure = sum(1 for t in world.tags.values() if t["sure"])
    print(f"Đã ghi {world.save()}: {len(world.tags)} tag ({sure} chắc chắn), {len(world.faces)} cube nhận bằng mặt khác, "
          f"{len(world.objects)} vật khác.")


SEEN_FRESH_S = 3.0              # quan sát vật cũ hơn mức này thì không dùng
STEREO_LINE_GAP_M = 0.08        # đường ngắm của camera tay phải đi qua cách tâm iPhone báo dưới mức này mới là cùng vật


_FOLLOWER = []                  # bộ xác nhận "vật đã dời" của tiến trình --watch (giữ trạng thái giữa các lần gộp)


def merge_seen(world, wrist_seen, camera: str = "phone", follower=None, now=None) -> list:
    """Gộp quan sát vật khác cube của iPhone (file phụ do `world_overlay` ghi) và của camera tay vào world: vật bị dời
    thì dời theo, giữ nguyên hình dạng đã quét. Hai camera cùng thấy thì giao hai đường ngắm (stereo đường đáy rộng).
    Trả các dòng mô tả những vật vừa dời (rỗng khi không có gì đổi)."""
    from cube_vision import object_track as OT
    from cube_vision.world_overlay import seen_path
    if not world.objects:
        return []
    now, phone_seen, phone_cubes = (time.time() if now is None else float(now)), [], []
    try:
        data = json.loads(seen_path(camera).read_text())
        if now - float(data.get("stamp", 0.0)) <= SEEN_FRESH_S:
            phone_seen = [OT.from_json(o) for o in data.get("objects", [])]
            phone_cubes = [np.asarray(c, float) for c in data.get("cubes", [])]
    except (OSError, ValueError, TypeError):
        pass
    wrist_seen = [o for o in wrist_seen if now - float(o.get("stamp", 0.0)) <= SEEN_FRESH_S]
    lines = []
    if follower is None:
        if not _FOLLOWER:
            _FOLLOWER.append(OT.Follower())
        follower = _FOLLOWER[0]
    cubes = [world.entry(i)["centre"] for i in world.cube_ids()] + phone_cubes   # cả cube chỉ iPhone đang thấy
    phone_seen, wrist_seen = OT.not_cubes(phone_seen, cubes), OT.not_cubes(wrist_seen, cubes)
    matched = OT.match(world.objects, phone_seen)
    for index, seen in matched.items():
        item, centre, how = world.objects[index], np.asarray(seen["centre"], float), camera
        for other in wrist_seen:
            if (other.get("bearing_ok") or other.get("near_ok")) and OT._compatible(item, dict(other, side_ok=False)) \
                    and OT.line_gap(other, centre) <= STEREO_LINE_GAP_M:
                fused, kind = OT.fuse(seen, other)
                if kind == "stereo":
                    centre, how = fused, f"{camera} + camera tay (giao hai đường ngắm)"
                elif kind == "opposed":
                    centre, how = fused, f"{camera} + camera tay (hai phía đối diện)"
                break
        if np.linalg.norm(centre - item["centre"][:2]) <= OT.MOVED_M:
            item["seen_stamp"] = now                            # camera vừa xác nhận vật còn ở chỗ đang ghi
        centre = follower.confirmed((camera, index), centre, item["centre"], seen.get("stamp", now))
        if centre is not None and world.inside(centre):
            x0, y0 = item["centre"][:2] * 1000
            world.objects[index] = OT.moved_copy(item, centre, how)
            lines.append(f"vật '{item['label']}' dời từ ({x0:+.0f}, {y0:+.0f}) tới ({centre[0] * 1000:+.0f}, "
                         f"{centre[1] * 1000:+.0f}) mm, theo {how}")
    whole = [o for o in wrist_seen if o.get("near_ok") and o.get("side_ok")]
    free = [i for i in range(len(world.objects)) if i not in matched]
    if whole and free:
        stamp = max(float(o.get("stamp", now)) for o in whole)
        updated, moved, _ = OT.relocate([world.objects[i] for i in free], whole, "camera tay", stamp=stamp,
                                        follower=follower, keys=[("wrist", i) for i in free])
        for k in moved:
            if world.inside(updated[k]["centre"]):
                item = world.objects[free[k]]
                world.objects[free[k]] = updated[k]
                x, y = updated[k]["centre"] * 1000
                lines.append(f"vật '{item['label']}' dời tới ({x:+.0f}, {y:+.0f}) mm, theo camera tay")
    return lines


class FaceJob:
    """Tìm cube không ngửa tag ở luồng riêng, trên khung đứng yên mới nhất, để không chặn vòng hiển thị.

    Mỗi vòng cho hai kết quả: (1) MẶT MÀU bằng ngưỡng HSV, vài mili giây, đủ cho cube ngửa hoặc quay ngang mặt màu;
    (2) bộ nhận mặt đầy đủ (hình in, có DINO: ~0,9 s có GPU, ~4 s chỉ CPU), bỏ khi dino=False. Khớp hình học mặt
    30 mm (0,1–0,3 s mỗi mặt) cũng làm ở đây. `servo_at(t)` trả góc khớp lúc chụp, False khi tay đang động, None khi
    chưa có lần đọc khớp sau khung đó.
    """

    def __init__(self, cal, servo_at, dino: bool = True, objects: bool = True):
        import queue
        import threading
        self.cal, self.servo_at = cal, servo_at
        self._lock, self._out, self._stop = threading.Lock(), queue.Queue(), threading.Event()
        self._latest = {"colour": None, "dino": None}           # mỗi luồng một ô "khung mới nhất" riêng
        self._fits = {}                                         # {(nguồn, cube): (quad, khớp, nghiệm)} lần khớp gần nhất
        self._threads = [threading.Thread(target=self._colour_loop, daemon=True)]
        self.objects_seen = []                                  # quan sát vật khác cube mới nhất của camera tay
        if objects:
            self._threads.append(threading.Thread(target=self._object_loop, daemon=True))
        self._latest["object"] = None
        if dino:                                                # luồng riêng: nạp DINO mất vài giây, không chặn mặt màu
            self._threads.append(threading.Thread(target=self._dino_loop, daemon=True))
        for thread in self._threads:
            thread.start()

    def offer(self, stamp, frame, clear, tag_ids):
        job = (stamp, frame.copy(), clear, set(tag_ids))
        with self._lock:
            self._latest = {key: job for key in self._latest}

    def _next(self, key, period, last):
        """Khung mới nhất cho luồng `key` kèm góc khớp lúc chụp, hoặc None (chưa tới nhịp / chưa có khung / tay động)."""
        if time.time() - last < period:                         # nhịp có chủ ý: dùng chung GIL với vòng hiển thị
            time.sleep(0.02)
            return None
        with self._lock:
            job, self._latest[key] = self._latest[key], None
        if job is None:
            time.sleep(0.02)
            return None
        servo = None
        for _ in range(60):                                     # chờ lần đọc khớp sau khung (tối đa ~3 s)
            servo = self.servo_at(job[0])
            if servo is not None or self._stop.is_set():
                break
            time.sleep(0.05)
        return (job, servo) if isinstance(servo, list) else None

    def _emit(self, stamp, servo, faces, tag_ids, clear, source=""):
        import world_watch as W
        for face in faces:
            # Cảnh tĩnh: mặt vẫn ở đúng pixel cũ và tay chưa nhúc nhích thì nghiệm khớp cũ vẫn đúng. Khớp lại mỗi lần
            # tốn 0,1–0,3 s/mặt và giành GIL của vòng hiển thị (đo: 16 khung/giây tụt còn 9).
            key, quad = (source, face["cube_id"]), np.asarray(face["quad"], float).reshape(4, 2)
            old = self._fits.get(key)
            if (old is not None and np.max(np.abs(np.sort(old[0], axis=0) - np.sort(quad, axis=0))) < 2.0
                    and max(abs(a - b) for a, b in zip(old[1], servo)) < 0.5):
                face["fit"] = None if old[2] is None else dict(old[2])
                continue
            face["fit"] = W.fit_face(quad, servo, self.cal)
            self._fits[key] = (quad, list(servo), face["fit"])
        self._out.put((stamp, servo, faces, tag_ids, clear))

    def _colour_loop(self):
        from cube_vision import color, registry
        last = 0.0
        while not self._stop.is_set():
            got = self._next("colour", FACE_COLOUR_PERIOD_S, last)
            if got is None:
                continue
            (stamp, frame, clear, tag_ids), servo = got
            try:
                self._emit(stamp, servo, color.colour_faces(frame, registry.COLOR_TO_ID), tag_ids, clear, "colour")
            except Exception as exc:  # noqa: BLE001
                print(f"  Bộ tìm mặt màu lỗi ở một khung ({exc}).")
            last = time.time()

    def _object_loop(self):
        """Mặt nạ vật khác cube trên khung camera tay -> quan sát trên mặt bàn (thường bị cắt vì camera tay ở gần:
        khi đó chỉ còn hướng ngắm, dùng để giao với đường ngắm của iPhone)."""
        from cube_vision import object_track as OT
        try:
            from cube_vision.object_masks import ObjectMasks
            masks = ObjectMasks()
        except Exception:  # noqa: BLE001 - python hệ thống: camera tay không góp quan sát vật
            return
        camera, table_z, last = A.wrist_camera(self.cal), float(self.cal["tag_top_z"]) - 0.030, 0.0
        while not self._stop.is_set():
            got = self._next("object", FACE_COLOUR_PERIOD_S, last)
            if got is None:
                continue
            (stamp, frame, _, _), servo = got
            try:
                T = A.base_T_optical(servo, self.cal)
                found = [OT.observe_mask(camera, T, mask, table_z, label, confidence)
                         for label, confidence, mask in masks.detect(frame)]
                self.objects_seen = [dict(o, stamp=stamp) for o in found if o is not None]
            except Exception as exc:  # noqa: BLE001
                print(f"  Bộ tách vật lỗi ở một khung ({exc}).")
            last = time.time()

    def _dino_loop(self):
        import world_watch as W
        try:
            from cube_vision.identify import FULL, Identifier
            identifier = Identifier(FULL)
        except Exception as exc:  # noqa: BLE001 - thiếu mô hình: vẫn có tag và mặt màu
            print(f"  Không nạp được bộ nhận hình in ({exc}): chỉ dùng tag và mặt màu.")
            return
        last = 0.0
        while not self._stop.is_set():
            got = self._next("dino", FACE_DINO_PERIOD_S, last)
            if got is None:
                continue
            (stamp, frame, clear, tag_ids), servo = got
            try:
                faces, seen = W.faces_from_detections(identifier.detect(frame))
                self._emit(stamp, servo, faces, tag_ids | seen, clear, "dino")
            except Exception as exc:  # noqa: BLE001
                print(f"  Bộ nhận hình in lỗi ở một khung ({exc}).")
            last = time.time()

    def results(self):
        import queue
        out = []
        while True:
            try:
                out.append(self._out.get_nowait())
            except queue.Empty:
                return out

    def stop(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=10.0)


def cmd_watch(args):
    """Chỉ nhìn: tay đứng yên ở pose nào, camera tay thấy cube nào thì ghi vào world. Không gửi lệnh chuyển động,
    trừ khi có --goto (đi tới một pose rồi nhìn) hoặc --free (nhả lực servo để bạn tự bẻ tay)."""
    import cv2
    import calibrate_hand_eye as C
    import world_watch as W
    world = WorldMap() if args.fresh else WorldMap.load()
    if args.fresh:
        add_fixed_cameras(world, log=lambda *_: None)
    with A.WristSession(park=False) as session:
        try:
            world.table_z = float(json.loads(M.CALIB_FILE.read_text())["table_z"])
        except (OSError, ValueError, KeyError, TypeError):
            pass
        if not world.region:
            world.region, area = A.scan_region(session.cal)
            print(f"Vùng world (mặt bàn camera tay quét được): {area * 1e4:.0f} cm². Vật ngoài vùng này không được ghi.")
        watcher = W.Watcher(session.cal, world)
        if args.goto:
            print(f"Đưa tay tới {args.goto} rồi chỉ nhìn.")
            C.move_and_settle(session.arm, args.goto)
        if args.free:
            print("NHẢ LỰC SERVO: giữ tay máy bằng tay trước khi nhả, nó sẽ rũ xuống. Bẻ tay tới pose muốn nhìn rồi "
                  "giữ yên ~1 giây. Thoát (q) sẽ bật lực lại.")
            time.sleep(2.0)
            session.arm.Arm_serial_set_torque(0)
        print("Đang theo dõi (q hoặc Ctrl+C để thoát). World:", world.save())
        import collections
        model, last_text, last_save = A.wrist_camera(session.cal), "", 0.0
        last_face_text = ""
        recent, pending = collections.deque(maxlen=3), collections.deque(maxlen=90)
        started, frames, looks = time.time(), 0, 0
        session.start_polling()
        face_job = None if args.no_faces else FaceJob(session.cal, session.joints_at, dino=args.dino,
                                                      objects=not args.no_objects)
        last_merge = 0.0
        last_dim = 0.0
        try:
            while True:
                stamp, frame, seen = session.grab()
                if frame is None:
                    time.sleep(0.05)
                    continue
                frames += 1
                recent.append(seen)
                steady = A.stable_tags(recent) if len(recent) == recent.maxlen else {}
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                clear = bool(steady) or (I.sharpness(gray) >= WATCH_MIN_SHARPNESS
                                         and gray.mean() >= WATCH_MIN_BRIGHTNESS)
                pending.append((stamp, steady, clear))
                if face_job is not None:
                    if gray.mean() < WATCH_FACE_BRIGHTNESS and time.time() - last_dim > 30.0:
                        print(f"  Ảnh tối (độ sáng {gray.mean():.0f}/255): mặt không có tag sẽ khó nhận, hãy bật thêm đèn.")
                        last_dim = time.time()
                    face_job.offer(stamp, frame, clear, steady)
                    for at, servo, faces, tag_ids, was_clear in face_job.results():
                        events = watcher.observe_faces(servo, faces, tag_ids, stamp=at, clear_view=was_clear)
                        text = W.describe(events)
                        if text and text != last_face_text:
                            print(f"  [{time.strftime('%H:%M:%S')}] J={[round(v) for v in servo[:4]]}: {text}")
                        last_face_text = text
                        if events["used"] or events["removed"]:
                            world.save()
                # Khung nào đã có lần đọc khớp sau nó thì kết luận được: tay đứng yên (ghi) hay đang động (bỏ).
                confirmed, moving = None, False
                while pending:
                    servo = session.joints_at(pending[0][0])
                    if servo is None:
                        break
                    item = pending.popleft()
                    if servo is False:
                        moving = True
                    else:
                        confirmed = (servo, item)
                if confirmed is not None:
                    servo, (at, tags, was_clear) = confirmed
                    events = watcher.observe(servo, tags, stamp=at, clear_view=was_clear)
                    looks += 1
                    text = W.describe(events)
                    if text and text != last_text:
                        print(f"  [{time.strftime('%H:%M:%S')}] J={[round(v) for v in servo[:4]]}: {text}")
                    last_text = text
                    if (events["used"] or events["removed"]) and time.time() - last_save > 0.4:
                        world.save()
                        last_save = time.time()
                    if args.once:
                        break
                if not args.no_objects and time.time() - last_merge > 0.5:
                    last_merge = time.time()
                    for line in merge_seen(world, face_job.objects_seen if face_job is not None else []):
                        print(f"  [{time.strftime('%H:%M:%S')}] {line}")
                        world.save()
                now_servo = session.latest_joints()
                if not args.no_window:
                    if now_servo is None:
                        view = frame.copy()
                        cv2.putText(view, "dang doc khop...", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    else:
                        j1_range = session.cal.get("j1_valid_range")
                        outside = bool(j1_range) and not (j1_range[0] <= now_servo[0] <= j1_range[1])
                        note = f"J={[round(v) for v in now_servo[:4]]} world: {len(world.cube_ids())} cube" + (
                            " | tay dang chuyen dong" if moving else "") + (" | NGOAI VUNG HAND-EYE" if outside else "")
                        # cube của world không thấy trong khung này: vẽ xám
                        unseen = (set(world.tags) - set(steady)) | {c for c in world.faces if watcher.unseen.get(c)}
                        view, _ = O.draw(frame, world, model, A.base_T_optical(now_servo, session.cal), steady, note,
                                         missing=unseen)
                    cv2.imshow("camera tay -> world", view)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                if args.seconds and time.time() - started > args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            session.stop_polling()
            if face_job is not None:
                face_job.stop()
            elapsed = max(time.time() - started, 1e-6)
            print(f"Đã chạy {elapsed:.0f} s: {frames / elapsed:.1f} khung/giây hiển thị, {looks / elapsed:.1f} lần ghi/giây.")
            if args.free:
                session.arm.Arm_serial_set_torque(1)
                print("Đã bật lại lực servo.")
            cv2.destroyAllWindows()
            world.save()
    cmd_show(args)


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
    for cube_id, face in sorted(world.faces.items()):
        x, y, z = face["centre"] * 1000
        how = (f"mặt màu {face['label'].replace(' o mat ben', '')} quay ngang" if "mat ben" in face["label"]
               else f"mặt {face['label']} ngửa lên")
        print(f"  cube {cube_id} ({how}, không đọc tag): ({x:+.1f}, {y:+.1f}, {z:+.1f}) mm, "
              f"tầng {face['layer']}, khớp {face['rms_px']:.1f} px, {face['n_views']} lần nhìn, "
              f"{now - face['stamp']:.0f} s trước")
    for item in world.objects:
        x, y = item["centre"] * 1000
        tall = "chiều cao chưa đo được" if item.get("height_m") is None else f"cao ~{item['height_m'] * 1000:.0f} mm"
        print(f"  vật '{item['label']}' ({item.get('shape', 'hộp')}): tâm ({x:+.0f}, {y:+.0f}) mm, "
              f"ngang ~{item.get('width_m', 0) * 1000:.0f} mm, {tall}, {item.get('n_points', 0)} điểm 3D, "
              f"{now - item['stamp']:.0f} s trước")
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
    ap.add_argument("--watch", action="store_true",
                    help="chỉ nhìn: tay ở pose nào, thấy cube nào thì ghi vào world (không lái tay)")
    ap.add_argument("--goto", nargs=5, type=float, metavar="J", help="với --watch: đi tới 5 góc servo này trước khi nhìn")
    ap.add_argument("--free", action="store_true", help="với --watch: nhả lực servo để tự bẻ tay bằng tay (tay sẽ rũ!)")
    ap.add_argument("--fresh", action="store_true", help="với --watch: bắt đầu từ world rỗng thay vì world đã lưu")
    ap.add_argument("--once", action="store_true", help="với --watch: ghi một lần nhìn rồi thoát")
    ap.add_argument("--no-window", action="store_true", help="với --watch: không mở cửa sổ")
    ap.add_argument("--seconds", type=float, default=0.0, help="với --watch: tự dừng sau chừng này giây")
    ap.add_argument("--no-objects", action="store_true",
                    help="với --scan: không dựng vật khác cube (cốc, hộp...) từ đám mây điểm")
    ap.add_argument("--no-object-looks", action="store_true",
                    help="với --scan: không chụp thêm khung quanh từng vật (nhanh hơn, hình vật kém hơn)")
    ap.add_argument("--dino", action="store_true",
                    help="với --watch: chạy thêm bộ nhận hình in (DINO). Mặc định tắt: đo thật nó kéo cửa sổ từ 9 xuống "
                         "4 khung/giây và 4 góc mặt hình in nó trả về thường lệch; mặc định chỉ dùng tag + mặt màu")
    ap.add_argument("--no-faces", action="store_true",
                    help="chỉ dùng tag, không nhận cube bằng mặt màu / hình in")
    args = ap.parse_args()
    if args.watch:
        cmd_watch(args)
    elif args.scan:
        cmd_scan(args)
        # torch (CUDA) + OpenCV dọn dẹp lúc thoát làm hỏng heap ("corrupted size vs. prev_size", 3/3 lần quét bằng
        # .venv). Mọi thứ đã lưu và thiết bị đã nhả ở trên, nên thoát thẳng, không chạy phần dọn dẹp của thư viện.
        sys.stdout.flush()
        sys.stderr.flush()
        import os
        os._exit(0)
    elif args.locate:
        cmd_locate(args)
    elif args.check:
        cmd_check(args)
    else:
        cmd_show(args)


if __name__ == "__main__":
    main()
