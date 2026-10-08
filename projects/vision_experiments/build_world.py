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
from cube_vision.world_map import WorldMap, snap_to_layer  # noqa: E402

DRIFT_MOVED_PX = 8.0
WATCH_MIN_SHARPNESS = 25.0      # ảnh camera tay nhòe/tối hơn mức này thì "không thấy cube" không có nghĩa là cube mất
WATCH_MIN_BRIGHTNESS = 25
WATCH_FACE_BRIGHTNESS = 90      # đo thật: ở 63 bộ nhận mặt không thấy cube lật mặt, ở 154 thấy đủ.0


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


class FaceJob:
    """Bộ nhận mặt (màu / hình in, có DINO) chạy ở luồng riêng trên khung mới nhất: chậm hơn dò tag nhiều lần
    (0,7 s có GPU, ~4 s chỉ CPU) nên không được chặn vòng hiển thị."""

    def __init__(self):
        import queue
        import threading
        self._latest, self._lock = None, threading.Lock()
        self._out, self._stop = queue.Queue(), threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def offer(self, stamp, frame, clear):
        with self._lock:
            self._latest = (stamp, frame.copy(), clear)

    def _loop(self):
        try:
            from cube_vision.identify import FULL, Identifier
            identifier = Identifier(FULL)
        except Exception as exc:  # noqa: BLE001 - thiếu mô hình: vẫn theo dõi được bằng tag
            print(f"  Không nạp được bộ nhận mặt ({exc}): chỉ theo dõi bằng tag.")
            return
        while not self._stop.is_set():
            with self._lock:
                job, self._latest = self._latest, None
            if job is None:
                time.sleep(0.02)
                continue
            stamp, frame, clear = job
            try:
                self._out.put((stamp, identifier.detect(frame), clear))
            except Exception as exc:  # noqa: BLE001
                print(f"  Bộ nhận mặt lỗi ở một khung ({exc}).")

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
        self._thread.join(timeout=10.0)


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
        face_job = None if args.no_faces else FaceJob()
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
                    face_job.offer(stamp, frame, clear)
                    for at, detections, was_clear in face_job.results():
                        servo = session.joints_at(at)
                        if not isinstance(servo, list):
                            continue                              # tay đang động quanh khung đó: bỏ
                        faces, tag_ids = W.faces_from_detections(detections)
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
        print(f"  cube {cube_id} (mặt {face['label']} ngửa lên, không đọc tag): ({x:+.1f}, {y:+.1f}, {z:+.1f}) mm, "
              f"tầng {face['layer']}, khớp {face['rms_px']:.1f} px, {face['n_views']} lần nhìn, "
              f"{now - face['stamp']:.0f} s trước")
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
    ap.add_argument("--no-faces", action="store_true",
                    help="với --watch: chỉ dùng tag, không nhận cube bằng mặt màu / hình in")
    args = ap.parse_args()
    if args.watch:
        cmd_watch(args)
    elif args.scan:
        cmd_scan(args)
    elif args.locate:
        cmd_locate(args)
    elif args.check:
        cmd_check(args)
    else:
        cmd_show(args)


if __name__ == "__main__":
    main()
