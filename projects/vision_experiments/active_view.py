#!/usr/bin/env python3
"""Camera tay tự nhìn quanh khi một góc nhìn chưa đủ: ra x, y, z thật của từng tag trong world.

Vòng lặp cho mỗi tag:  quan sát -> chấm (cube_vision.view_quality) -> gộp các góc nhìn (cube_vision.multiview)
-> nếu chưa chắc thì chọn pose nhìn kế tiếp theo gợi ý của luật (lại gần / vào giữa ảnh / đổi chỗ lấy thị sai)
-> lặp, tối đa MAX_EXTRA_VIEWS lần. Tay chỉ NHÌN (kẹp mở, đầu kẹp luôn cao hơn cube), không gắp.

    python projects/vision_experiments/active_view.py                 # đo mọi tag đang thấy
    python projects/vision_experiments/active_view.py --tags 1 4      # chỉ các tag này
    python projects/vision_experiments/active_view.py --no-look-around   # chỉ quét cố định, để so sánh
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _extra in (HERE.parent, HERE):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))

import calibrate_hand_eye as C  # noqa: E402
import cube_search_center_math as M  # noqa: E402
from cube_vision import multiview as MV  # noqa: E402
from cube_vision import view_quality as Q  # noqa: E402
from cube_vision.frames import CameraModel  # noqa: E402

READY = [90.0, 125.0, 0.0, 0.0, 90.0]
SCAN_POSES = [[j1, j2, 0.0, 0.0, 90.0] for j2 in (125.0, 110.0) for j1 in (60.0, 90.0, 120.0)]
MAX_EXTRA_VIEWS = 4
MIN_NEW_BASELINE_M = 0.03       # pose mới phải dời tâm camera ít nhất chừng này so với mọi pose đã dùng
IMAGE_SIZE = (640, 480)


def wrist_camera(cal) -> CameraModel:
    return CameraModel("wrist", tuple(cal["K"]), IMAGE_SIZE, float(cal.get("k1", 0.0)))


def base_T_optical(servo, cal) -> np.ndarray:
    return M.fk_arm4_cal(servo, cal) @ np.asarray(cal["arm4_T_optical"], float)


def make_view(corners_px, servo, cal, label="") -> MV.View:
    view = MV.View(base_T_optical(servo, cal), wrist_camera(cal), np.asarray(corners_px, float).reshape(4, 2), label)
    view.servo = [float(v) for v in servo[:5]]
    return view


# ------------------------------------------------------------------ chọn góc nhìn kế tiếp (thuần toán)
def candidate_poses(target_base, cal, j1_range=None):
    """Mọi pose an toàn mà mục tiêu nằm gọn trong ảnh, trong vùng J1 hand-eye đã kiểm chứng."""
    fx, fy, cx, cy = cal["K"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    lo, hi = j1_range or cal.get("j1_valid_range") or (40.0, 135.0)
    j1_values = range(int(math.ceil(lo / 5.0) * 5), int(hi) + 1, 5)
    return C.pose_pool(np.asarray(target_base, float), np.asarray(cal["arm4_T_optical"], float), K, j1_values)


def next_view(target_base, used_servos, cal, hints=(), normal=(0.0, 0.0, 1.0), pool=None):
    """Pose nhìn kế tiếp cho một mục tiêu, hoặc None khi không còn pose nào khác đáng kể.

    Luôn đòi thị sai (tâm camera cách các pose đã dùng ≥ MIN_NEW_BASELINE_M); gợi ý của luật đổi trọng số:
    closer -> ưu tiên gần mục tiêu, frontal -> ưu tiên nhìn dọc pháp tuyến, centre -> mục tiêu gần tâm ảnh.
    """
    target = np.asarray(target_base, float)
    normal = np.asarray(normal, float) / np.linalg.norm(normal)
    used = [base_T_optical(s, cal)[:3, 3] for s in used_servos]
    fx, fy, cx, cy = cal["K"]
    best = None
    for servo in (candidate_poses(target, cal) if pool is None else pool):
        T = base_T_optical(servo, cal)
        gap = min((float(np.linalg.norm(T[:3, 3] - c)) for c in used), default=1.0)
        if gap < MIN_NEW_BASELINE_M:
            continue
        to_cam = T[:3, 3] - target
        dist = float(np.linalg.norm(to_cam))
        frontal = float(to_cam @ normal) / dist                        # 1 = nhìn thẳng mặt tag
        cam = np.linalg.inv(T) @ np.r_[target, 1.0]
        off = math.hypot(fx * cam[0] / cam[2] + cx - 320.0, fy * cam[1] / cam[2] + cy - 240.0) / 400.0
        travel = min((C._joint_dist(servo, s) for s in used_servos), default=0.0) / 180.0
        score = min(gap, 0.12) / 0.12 + 0.5 * max(0.0, frontal) - 0.5 * off - 0.2 * travel
        if Q.HINT_CLOSER in hints:
            score += 1.5 * (0.30 - dist) / 0.30
        if Q.HINT_FRONTAL in hints:
            score += 1.0 * frontal
        if Q.HINT_CENTRE in hints:
            score -= 1.0 * off
        if best is None or score > best[0]:
            best = (score, servo)
    return None if best is None else list(best[1])


def fuse(views):
    """Gộp các góc nhìn bằng trung vị PnP (bền với sai số pose camera); góc lệch hẳn bị loại ngay trong đó.

    Trả (fused, quality, các view còn dùng). Xem `multiview` về lý do không tam giác hóa với hand-eye hiện tại.
    """
    fused = MV.robust_tag(views) if views else None
    return fused, Q.assess_fused(fused), (fused["views"] if fused else [])


def measure(observe, cal, tags=None, look_around=True, max_extra=MAX_EXTRA_VIEWS, log=print):
    """Đo mọi tag. observe(servo) -> (servo thật, {tag_id: (4,2)}). Trả {tag_id: kết quả}.

    Kết quả mỗi tag: fused (multiview.solve_tag), quality, views đã dùng, rejected [(nhãn, lý do)], extra (số pose
    nhìn thêm). Tách khỏi phần cứng để test được bằng camera giả lập.
    """
    j1_range = cal.get("j1_valid_range")
    views, rejected, used = {}, {}, []

    def look(servo, label):
        real, seen = observe(servo)
        used.append([float(v) for v in real[:5]])
        for tag_id, corners in seen.items():
            if tags and tag_id not in tags:
                continue
            view = make_view(corners, real, cal, label)
            quality = Q.assess_view(view, j1=real[0], j1_range=j1_range)
            if quality:
                views.setdefault(tag_id, []).append(view)
            else:
                rejected.setdefault(tag_id, []).append((label, quality.text(), quality.hints))
        return seen

    for index, servo in enumerate(SCAN_POSES):
        look(servo, f"quét {index + 1}")
    out = {}
    for tag_id in sorted(set(views) | set(rejected)):
        fused, quality, kept = fuse(views.get(tag_id, []))
        extra = 0
        while look_around and not quality and extra < max_extra:
            hints = list(quality.hints) + [h for _, _, hs in rejected.get(tag_id, []) for h in hs]
            target = fused["centre"] if fused else None
            if target is None:
                break                                    # chưa có khung nào dùng được: không biết nhìn về đâu
            servo = next_view(target, [v.servo for v in views.get(tag_id, [])] or used, cal, hints,
                              normal=fused["normal"] if fused["normal"][2] > 0.5 else (0.0, 0.0, 1.0))
            if servo is None:
                log(f"  tag {tag_id}: hết pose nhìn khác trong vùng an toàn")
                break
            extra += 1
            log(f"  tag {tag_id}: {quality.text()} -> nhìn thêm từ {[round(v) for v in servo[:4]]}")
            look(servo, f"thêm {extra}")
            fused, quality, kept = fuse(views.get(tag_id, []))
        out[tag_id] = {"fused": fused, "quality": quality, "views": kept,
                       "rejected": [(label, why) for label, why, _ in rejected.get(tag_id, [])], "extra": extra}
    return out


def stable_tags(recent, max_jitter_px: float = 1.5) -> dict:
    """Tag có mặt trong MỌI khung của `recent` ([{id: (4,2)}]) và đứng yên trong `max_jitter_px`: {id: góc trung bình}."""
    recent = list(recent)
    if not recent:
        return {}
    out = {}
    for tag_id in set.intersection(*(set(frame) for frame in recent)):
        stack = np.array([frame[tag_id] for frame in recent])
        if float(np.max(np.abs(stack - stack.mean(axis=0)))) <= max_jitter_px:
            out[tag_id] = stack.mean(axis=0)
    return out


def describe(tag_id, result) -> str:
    fused, quality = result["fused"], result["quality"]
    if not fused:
        why = "; ".join(sorted({w for _, w in result["rejected"]})) or "không thấy"
        return f"tag {tag_id}: CHƯA ĐO ĐƯỢC ({why})"
    x, y, z = fused["centre"] * 1000
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(fused["normal"][2])))))
    state = "ỔN" if quality else "CHƯA CHẮC: " + quality.text()
    spread = np.asarray(fused.get("spread_m", fused["pos_std_m"]), float) * 1000
    agree = "một góc nhìn" if not np.isfinite(spread).all() else \
        f"các góc nhìn lệch nhau ngang {max(spread[0], spread[1]):.1f} / đứng {spread[2]:.1f} mm"
    dropped = f", bỏ {len(fused['dropped'])} góc lệch" if fused.get("dropped") else ""
    return (f"tag {tag_id}: world ({x:+.1f}, {y:+.1f}, {z:+.1f}) mm, mặt tag nghiêng {tilt:.0f}° so với phương đứng; "
            f"{fused['n_views']} góc nhìn (+{result['extra']} nhìn thêm{dropped}), {agree} — {state}")


# ------------------------------------------------------------------ phần cứng
class WristSession:
    """Giữ cổng serial + camera tay trong một khối `with`; nhả thiết bị khi xong.

    park=True (mặc định): đưa tay về READY khi thoát. park=False: không gửi lệnh chuyển động nào khi thoát
    (dùng cho chế độ chỉ nhìn).
    """

    def __init__(self, park: bool = True):
        self.park = park

    def __enter__(self):
        import fcntl
        import cv2
        from cube_vision import cameras
        from cube_vision.tag import TagDetector
        self.cal = M.load_calibration()
        if self.cal is None:
            raise SystemExit("Chưa có hand-eye camera tay đạt (config/robot/hand_eye.json).")
        devices = cameras.list_cameras()
        self.wrist, _ = cameras.resolve_wrist(devices, identify=lambda: cameras.identify_with_arm(devices))
        if not self.wrist:
            raise SystemExit("Không tìm thấy camera tay (python -m cube_vision.cameras --identify).")
        self.lock = C.LOCK_FILE.open("a+")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Cổng tay máy đang bận (tắt T8/perception trước).")
        sys.path.insert(0, str(ROOT / "vendor" / "yahboom"))
        from Arm_Lib import Arm_Device
        self.arm = Arm_Device("/dev/ttyUSB0")
        self.det = TagDetector(enhance=True, quiet=True)
        self.cap = cv2.VideoCapture(self.wrist, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.__exit__(None, None, None)
            raise SystemExit(f"Không mở được camera tay {self.wrist}.")
        return self

    def observe(self, servo):
        """Đưa tay tới pose, chờ đứng yên, trả (khớp thật, {tag_id: 4 góc}) của các tag đứng yên qua vài khung."""
        import calibrate_external as E
        real = C.move_and_settle(self.arm, servo)
        return real, E._stable_tags(self.cap, self.det)

    def look(self, still_deg: float = 1.0):
        """Chỉ đọc, KHÔNG lái tay: (khớp thật | None nếu tay đang chuyển động, {tag_id: 4 góc}, khung ảnh).

        Đọc khớp trước và sau khi lấy ảnh; hai lần lệch quá `still_deg` thì tay đang động, lần nhìn này không dùng.
        """
        import cv2
        import time
        before = C.read_servo(self.arm)
        # Xả bộ đệm: khung đọc về tức thì là khung cũ (có thể chụp lúc tay còn đang chuyển động); khung phải chờ
        # mới có là khung tươi. Không xả thì vừa tới pose mới sẽ nhận phải ảnh nhòe của lúc đang đi.
        for _ in range(12):
            started = time.time()
            self.cap.grab()
            if time.time() - started > 0.02:
                break
        seen, frame = {}, None
        for _ in range(3):
            ok, raw = self.cap.read()
            if not ok or raw is None:
                return None, {}, None
            frame = raw if (raw.shape[1], raw.shape[0]) == IMAGE_SIZE else cv2.resize(raw, IMAGE_SIZE)
            for tag in self.det.detect(frame):
                seen.setdefault(int(tag["id"]), []).append(np.asarray(tag["corners"], float).reshape(4, 2))
        after = C.read_servo(self.arm)
        stable = {}
        for tag_id, stack in seen.items():
            stack = np.array(stack)
            inside = stack.min() >= 8 and stack[..., 0].max() <= 632 and stack[..., 1].max() <= 472
            if len(stack) == 3 and inside and float(np.max(np.abs(stack - stack.mean(axis=0)))) <= 1.5:
                stable[tag_id] = stack.mean(axis=0)
        if max(abs(a - b) for a, b in zip(before, after)) > still_deg:
            return None, stable, frame
        return [(a + b) / 2.0 for a, b in zip(before, after)], stable, frame

    # -- đọc khớp ở luồng riêng: vòng hiển thị chạy theo tốc độ camera, không phải chờ serial (~0,4 s mỗi lần đọc)
    def start_polling(self):
        import collections
        import threading
        import time
        self._readings = collections.deque(maxlen=60)          # (bắt đầu đọc, đọc xong, 5 góc khớp)
        self._stop = threading.Event()

        def loop():
            while not self._stop.is_set():
                started = time.time()
                try:
                    servo = C.read_servo(self.arm)
                except Exception:  # noqa: BLE001 - lỗi serial tạm thời: thử lại
                    time.sleep(0.1)
                    continue
                self._readings.append((started, time.time(), servo))

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop_polling(self):
        if getattr(self, "_stop", None) is not None:
            self._stop.set()
            self._thread.join(timeout=3.0)
            self._stop = None

    def latest_joints(self):
        readings = getattr(self, "_readings", None)
        return list(readings[-1][2]) if readings else None

    def joints_at(self, t: float, still_deg: float = 1.0):
        """Góc khớp tại thời điểm t của một khung ảnh, chỉ khi tay ĐỨNG YÊN quanh thời điểm đó.

        Cần một lần đọc xong trước t và một lần bắt đầu sau t khớp nhau trong `still_deg`. Trả list 5 góc; False
        khi hai lần đọc lệch nhau (tay đang động) hoặc không có lần đọc trước; None khi lần đọc sau chưa tới.
        """
        readings = list(getattr(self, "_readings", ()))
        before = [r for r in readings if r[1] <= t]
        after = [r for r in readings if r[0] >= t]
        if not after:
            return None
        if not before:
            return False
        a, b = before[-1][2], after[0][2]
        if max(abs(x - y) for x, y in zip(a, b)) > still_deg:
            return False
        return [(x + y) / 2.0 for x, y in zip(a, b)]

    def grab(self):
        """(thời điểm, khung 640x480, {tag_id: 4 góc nằm gọn trong ảnh}) của MỘT khung; (None, None, {}) khi lỗi."""
        import time
        import cv2
        ok, raw = self.cap.read()
        if not ok or raw is None:
            return None, None, {}
        stamp = time.time()
        frame = raw if (raw.shape[1], raw.shape[0]) == IMAGE_SIZE else cv2.resize(raw, IMAGE_SIZE)
        seen = {}
        for tag in self.det.detect(frame):
            corners = np.asarray(tag["corners"], float).reshape(4, 2)
            if corners.min() >= 8 and corners[:, 0].max() <= 632 and corners[:, 1].max() <= 472:
                seen[int(tag["id"])] = corners
        return stamp, frame, seen

    def __exit__(self, *exc):
        self.stop_polling()
        try:
            if self.park and (exc[0] is None or exc[0] is KeyboardInterrupt):
                C.move_and_settle(self.arm, READY, ms=1800)
        except Exception:  # noqa: BLE001
            pass
        for closer in (lambda: self.cap.release(), lambda: self.lock.close()):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.arm = None
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tags", nargs="*", type=int, help="chỉ đo các tag này")
    ap.add_argument("--no-look-around", action="store_true", help="chỉ quét các pose cố định")
    args = ap.parse_args()
    with WristSession() as session:
        results = measure(session.observe, session.cal, set(args.tags or []) or None,
                          look_around=not args.no_look_around)
    if not results:
        print("Không thấy tag nào.")
    for tag_id, result in results.items():
        print(describe(tag_id, result))
        for label, why in result["rejected"]:
            print(f"    bỏ khung '{label}': {why}")


if __name__ == "__main__":
    main()
