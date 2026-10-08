"""Nhận diện camera tay máy trong số các camera USB (số /dev/videoN hay bị đổi).

Camera tay gắn trên cánh tay: khi J1 xoay vài độ, ảnh của nó dịch/đổi TOÀN CỤC, còn camera
cố định (camera ngoài, webcam laptop) hầu như không đổi (chỉ vùng cánh tay đi qua). So hai khung
trước/sau khi xoay: camera có dịch ảnh lớn nhất là camera tay. Kết quả nhớ trong /tmp theo TÊN
thiết bị (nếu các tên khác nhau) nên các lần sau không cần xoay tay, và vẫn đúng khi số cổng đổi.

    python -m cube_vision.cameras --identify     # xoay J1 vài độ rồi in vai trò
    python -m cube_vision.cameras --list
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np

SYS_V4L = Path("/sys/class/video4linux")
CACHE_FILE = Path("/tmp/t8_camera_roles.json")
LOCK_FILE = Path("/tmp/t8_motion.lock")
MIN_SCORE = 1.0          # điểm tối thiểu để tin camera tay
MIN_RATIO = 1.8          # camera tay phải hơn camera kế ít nhất ngần này lần


def list_cameras(sys_root: Path = SYS_V4L):
    """Camera USB thật (mỗi camera một node capture, index 0): [{path, name, usb}]."""
    out = []
    for node in sorted(Path(sys_root).glob("video*"), key=lambda p: int(p.name[5:] or 0)):
        try:
            index = int((node / "index").read_text())
            name = (node / "name").read_text().strip()
        except (OSError, ValueError):
            continue
        if index != 0:
            continue
        try:
            usb = os.path.realpath(node / "device")
        except OSError:
            usb = ""
        out.append({"path": f"/dev/{node.name}", "name": name, "usb": usb})
    return out


def _gray(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    return cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)


def motion_score(before, after):
    """Điểm 'ảnh đổi toàn cục' giữa hai khung: dịch ảnh (px) và tỉ lệ pixel đổi."""
    a, b = _gray(before), _gray(after)
    if a.shape != b.shape:
        return 0.0
    (dx, dy), _ = cv2.phaseCorrelate(a, b)
    shift = float(np.hypot(dx, dy))
    changed = float(np.mean(np.abs(a - b) > 20.0))
    return max(shift / 20.0, changed / 0.15)


def identify_wrist(devices, grab_frames, rotate):
    """Chọn camera tay. grab_frames(devices)->{path: frame}; rotate(+1/-1) xoay J1 một nấc.

    Trả {"wrist": path|None, "scores": {path: điểm}, "reason": str}.
    """
    before = grab_frames(devices)
    rotate(+1)
    after = grab_frames(devices)
    rotate(-1)
    scores = {}
    for dev in devices:
        path = dev["path"]
        if path in before and path in after and before[path] is not None and after[path] is not None:
            scores[path] = motion_score(before[path], after[path])
    if not scores:
        return {"wrist": None, "scores": scores, "reason": "không đọc được khung từ camera nào"}
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    best_path, best = ranked[0]
    second = ranked[1][1] if len(ranked) > 1 else 0.0
    if best < MIN_SCORE:
        return {"wrist": None, "scores": scores,
                "reason": f"không camera nào đổi ảnh khi xoay (cao nhất {best:.2f})"}
    if len(ranked) > 1 and best < MIN_RATIO * max(second, 1e-6):
        return {"wrist": None, "scores": scores,
                "reason": f"mơ hồ ({ranked[0][0]} {best:.2f} so với {ranked[1][0]} {second:.2f})"}
    return {"wrist": best_path, "scores": scores, "reason": ""}


def _names_unique(devices):
    names = [d["name"] for d in devices]
    return len(set(names)) == len(names)


def _cache_key(dev, unique):
    return dev["name"] if unique else dev["usb"] or dev["path"]


def read_cache(devices, cache_file: Path = CACHE_FILE):
    """Đường dẫn camera tay từ cache nếu cache khớp bộ thiết bị hiện tại, ngược lại None."""
    try:
        data = json.loads(Path(cache_file).read_text())
    except (OSError, ValueError):
        return None
    unique = _names_unique(devices)
    keys = sorted(_cache_key(d, unique) for d in devices)
    if data.get("keys") != keys or data.get("unique") != unique:
        return None
    for dev in devices:
        if _cache_key(dev, unique) == data.get("wrist"):
            return dev["path"]
    return None


def write_cache(devices, wrist_path, cache_file: Path = CACHE_FILE):
    unique = _names_unique(devices)
    wrist = next((d for d in devices if d["path"] == wrist_path), None)
    if wrist is None:
        return
    Path(cache_file).write_text(json.dumps({
        "unique": unique, "keys": sorted(_cache_key(d, unique) for d in devices),
        "wrist": _cache_key(wrist, unique), "created": time.time()}))


def resolve_wrist(devices=None, identify=None, cache_file: Path = CACHE_FILE):
    """(path | None, nguồn): 'only' | 'cache' | 'identified' | 'unknown'.

    identify() -> kết quả của identify_wrist; chỉ gọi khi có nhiều camera và cache không khớp.
    """
    devices = list_cameras() if devices is None else devices
    if not devices:
        return None, "unknown"
    if len(devices) == 1:
        return devices[0]["path"], "only"
    cached = read_cache(devices, cache_file)
    if cached:
        return cached, "cache"
    if identify is not None:
        result = identify()
        if result.get("wrist"):
            write_cache(devices, result["wrist"], cache_file)
            return result["wrist"], "identified"
    return None, "unknown"


def open_capture(path):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


PHONE_URL_ENV = "PHONE_CAMERA_URL"
PHONE_URL_DEFAULT = "http://192.168.1.30:4747/video"     # DroidCam trên iPhone trong mạng LAN
STREAMS = {   # tên camera -> cách mở luồng mà intrinsic/extrinsic của nó được hiệu chuẩn theo
    "wrist": {"size": (640, 480), "fourcc": None, "rotate": 0},
    "ext": {"size": (1280, 720), "fourcc": "MJPG", "rotate": 0},
    "phone": {"size": None, "fourcc": None, "rotate": 90},
}


def is_url(source) -> bool:
    return str(source).startswith(("http://", "https://", "rtsp://"))


def stream_source(name: str, explicit=None):
    """Nguồn (đường dẫn /dev/videoN hoặc URL) của camera theo tên; None khi không tìm thấy."""
    if explicit and str(explicit) != "auto":
        return explicit
    if name == "phone":
        return os.environ.get(PHONE_URL_ENV) or PHONE_URL_DEFAULT
    wrist, _ = resolve_wrist()
    return wrist if name == "wrist" else resolve_external(wrist)


def open_stream(source, size=None, fourcc=None):
    """Mở luồng camera (thiết bị V4L2 hoặc URL mạng); None khi không mở được."""
    if source is None:
        return None
    cap = cv2.VideoCapture(str(source)) if is_url(source) else cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    if not is_url(source):
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if size:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    return cap


def rotate_frame(frame, degrees: int):
    """Xoay khung theo chiều kim đồng hồ 0/90/180/270 độ."""
    turns = (int(degrees) // 90) % 4
    return frame if turns == 0 else np.ascontiguousarray(np.rot90(frame, k=-turns))


def identify_with_arm(devices, serial="/dev/ttyUSB0", delta_deg=12.0, log=print):
    """Xoay J1 ±delta bằng cổng serial (khóa /tmp/t8_motion.lock) và so khung của mọi camera.

    Đòi tay rảnh và các camera chưa bị tiến trình khác giữ (tắt perception/T8 trước).
    """
    import fcntl
    lock = LOCK_FILE.open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return {"wrist": None, "scores": {}, "reason": "cổng tay máy đang bận (/tmp/t8_motion.lock)"}
    caps = {d["path"]: open_capture(d["path"]) for d in devices}
    caps = {path: cap for path, cap in caps.items() if cap is not None}
    if len(caps) < 2:
        for cap in caps.values():
            cap.release()
        return {"wrist": None, "scores": {}, "reason": "mở được <2 camera (đang bị giữ?)"}
    from ._paths import ensure
    ensure("vendor")
    from Arm_Lib import Arm_Device
    arm = Arm_Device(serial)
    start = float(arm.Arm_serial_servo_read(1) or 90.0)
    step = delta_deg if start + delta_deg <= 165.0 else -delta_deg

    def grab(devs):
        out = {}
        for dev in devs:
            cap = caps.get(dev["path"])
            frame = None
            if cap is not None:
                for _ in range(6):
                    cap.grab()
                ok, frame = cap.read()
                frame = frame if ok else None
            out[dev["path"]] = frame
        return out

    def rotate(sign):
        target = start + step if sign > 0 else start
        arm.Arm_serial_servo_write(1, target, 900)
        time.sleep(1.5)

    try:
        grab([{"path": p} for p in caps])
        return identify_wrist([d for d in devices if d["path"] in caps], grab, rotate)
    finally:
        for cap in caps.values():
            cap.release()


def resolve_external(wrist_path, explicit=None, devices=None):
    """Camera ngoài (cố định): giá trị chỉ định nếu có, không thì camera USB đầu tiên khác camera tay."""
    if explicit and str(explicit) != "auto":
        return explicit
    for dev in (list_cameras() if devices is None else devices):
        if dev["path"] != wrist_path:
            return dev["path"]
    return None


def resolve_arg(value, fallback="/dev/video2", log=print):
    """Giá trị --camera: 'auto' -> camera tay đã nhận diện; còn lại giữ nguyên."""
    if str(value) != "auto":
        return value
    devices = list_cameras()
    path, source = resolve_wrist(devices, identify=lambda: identify_with_arm(devices))
    if path:
        log(f"[camera] camera tay: {path} ({ {'only': 'duy nhất', 'cache': 'nhớ lần trước', 'identified': 'nhận bằng chuyển động'}[source] })")
        return path
    log(f"[camera] chưa nhận diện được camera tay; dùng {fallback}. Chạy: "
        "python -m cube_vision.cameras --identify (tắt perception/T8 trước) hoặc đặt --camera.")
    return fallback


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--identify", action="store_true")
    args = ap.parse_args()
    devices = list_cameras()
    for dev in devices:
        print(f"{dev['path']}  {dev['name']}  [{dev['usb']}]")
    if args.identify:
        result = identify_with_arm(devices)
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if result.get("wrist"):
            write_cache(devices, result["wrist"])
            print("camera tay:", result["wrist"])
        else:
            raise SystemExit(result["reason"])


if __name__ == "__main__":
    main()
