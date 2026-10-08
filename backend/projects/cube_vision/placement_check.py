"""Kiểm tra cube đã vào đúng ô thả chưa bằng camera ngoài (không cần hiệu chuẩn camera ngoài).

Ý tưởng: ô màu tự tìm ngay trong ảnh camera ngoài bằng mặt nạ màu (không cần thấy hết bàn, chỉ cần
thấy ô), rồi so ảnh TRƯỚC và SAU khi thả: vùng ảnh đổi nằm trong ô là cube mới vào ô; ngoài ô là rơi/lệch.
Không phụ thuộc T8/ROS: bên gọi đưa hai khung ảnh BGR và id zone.

    verdict "in_zone"  cube vào ô (phần đổi nằm chủ yếu trong ô)
            "partial"  cube nằm đè mép ô (một phần ngoài ô)
            "outside"  không có gì mới trong ô nhưng có thay đổi nơi khác (rơi ngoài ô)
            "missing"  thấy ô nhưng KHÔNG có cube mới trong ô và không thấy cube nơi khác (bị văng ra ngoài
                       khung hình hoặc chưa được thả)
            "unseen"   không đủ dữ liệu: không có ảnh hoặc không thấy ô (không kết luận)

Thay đổi trong vùng của chính cánh tay (`exclude`, mặc định quanh đế robot ở nửa trên ảnh) bị bỏ qua vì
tay ở pose khác nhau giữa hai ảnh.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np

# Màu ô thấy từ camera ngoài (zone: danh sách khoảng HSV). Cùng quy ước zone với T8: 1 xanh dương,
# 2 xanh lá, 3 đỏ, 4 xám (cube vàng vào ô xám).
PAD_HSV = {
    1: [((88, 70, 90), (112, 255, 255))],
    2: [((45, 70, 60), (80, 255, 255))],
    3: [((0, 120, 90), (8, 255, 255)), ((168, 120, 90), (179, 255, 255))],
    4: [((0, 0, 50), (180, 45, 110))],
}
MIN_PAD_AREA_FRAC = 0.012          # ô nhỏ hơn thế (so với khung) coi như không thấy
MIN_IN_PIXELS = 500                # vùng đổi trong ô tối thiểu (px) để tính là có cube
MIN_IN_FRAC_OF_PAD = 0.06
IN_ZONE_FRAC = 0.6                 # phần vùng đổi nằm trong ô để tính "vào ô"
DIFF_THRESHOLD = 40
MIN_BLOB_PIXELS = 250
DEFAULT_EXCLUDE = (0.22, 0.0, 0.72, 0.36)     # (x0, y0, x1, y1) theo tỉ lệ khung: quanh đế robot
GOOD = {"in_zone"}
VERDICT_TEXT = {
    "in_zone": "cube nằm trong ô",
    "partial": "cube đè lên mép ô (một phần ngoài ô)",
    "outside": "cube không vào ô, có vẻ rơi ngoài ô",
    "missing": "ô không có cube (có thể bị văng ra ngoài khung hình)",
    "unseen": "không thấy cube mới trong ô",
}


@dataclass
class Pad:
    zone: int
    hull: np.ndarray            # (N,2) int32
    area: float
    inferred: bool = False      # ô bị che/không thấy: vị trí suy từ ba ô còn lại

    def mask(self, shape):
        m = np.zeros(shape[:2], np.uint8)
        cv2.fillConvexPoly(m, self.hull.reshape(-1, 1, 2), 255)
        return cv2.dilate(m, np.ones((9, 9), np.uint8))


def pad_mask(bgr, zone):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in PAD_HSV[int(zone)]:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    k = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)


def _hulls(bgr, zone):
    """Mọi thân lồi của vùng màu `zone` đủ lớn, kèm độ đặc (diện tích mặt nạ / thân lồi)."""
    contours, _ = cv2.findContours(pad_mask(bgr, zone), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        hull = cv2.convexHull(c)
        area = float(cv2.contourArea(hull))
        if area >= MIN_PAD_AREA_FRAC * bgr.shape[0] * bgr.shape[1]:
            out.append((hull.reshape(-1, 2), area, float(cv2.contourArea(c)) / max(area, 1.0)))
    return out


def _centre(hull):
    m = cv2.moments(hull.reshape(-1, 1, 2).astype(np.float32))
    return np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]]) if abs(m["m00"]) > 1e-6 else hull.mean(axis=0)


# Bố trí 4 ô là hình bình hành (xanh dương/xanh lá bên trên, đỏ/xám bên dưới, cặp đối nhau song song):
# tâm ô thiếu = tâm hai ô kề cộng trừ tâm ô đối diện; hình dạng ô lấy từ ô kề rồi tịnh tiến.
_PARALLELOGRAM = {4: (2, 3, 1), 3: (1, 4, 2), 2: (1, 4, 3), 1: (2, 3, 4)}   # thiếu: (ô tịnh tiến, +, -)


def _predicted_centre(pads, zone):
    a, b, c = _PARALLELOGRAM[zone]
    if a in pads and b in pads and c in pads:
        return _centre(pads[a].hull) + _centre(pads[b].hull) - _centre(pads[c].hull), a
    return None, None


def find_pads(bgr, zones=(1, 2, 3, 4)):
    """{zone: Pad} của các ô thấy trong ảnh (kể cả ô bị cube/khung che: suy từ ba ô còn lại).

    Màu xám dễ trùng với phần xám đậm của chính robot (càng kẹp/khớp), nên ô nào có nhiều ứng viên hoặc
    không có ứng viên đều được đối chiếu với vị trí hình bình hành từ ba ô kia, thay vì lấy ô to nhất.
    """
    cands = {z: _hulls(bgr, z) for z in zones}
    pads = {}
    for z in zones:                                       # lượt 1: ô có đúng một ứng viên rõ
        if len(cands[z]) >= 1 and z != 4:
            hull, area, _ = max(cands[z], key=lambda item: item[1])
            pads[z] = Pad(z, hull, area)
    for z in zones:                                       # lượt 2: ô xám và ô thiếu
        if z in pads:
            continue
        guess, donor = _predicted_centre(pads, z)
        options = cands[z]
        if guess is not None:
            mean_area = float(np.mean([pads[k].area for k in pads]))
            near = [(float(np.linalg.norm(_centre(h) - guess)), h, a) for h, a, _ in options
                    if 0.35 * mean_area <= a <= 2.2 * mean_area]
            near = [item for item in near if item[0] <= 0.9 * np.sqrt(mean_area)]
            if near:
                _, hull, area = min(near, key=lambda item: item[0])
                pads[z] = Pad(z, hull, area)
            else:
                shift = guess - _centre(pads[donor].hull)
                hull = (pads[donor].hull + shift).astype(np.int32)
                pads[z] = Pad(z, hull, float(cv2.contourArea(hull.reshape(-1, 1, 2))), inferred=True)
        elif options:                                     # không đủ ô để đối chiếu: chỉ nhận ô đặc
            solid = [item for item in options if item[2] >= 0.7]
            if solid:
                hull, area, _ = max(solid, key=lambda item: item[1])
                pads[z] = Pad(z, hull, area)
    return pads


def find_pad(bgr, zone):
    """Ô `zone` trong ảnh (xem find_pads); None nếu không thấy."""
    return find_pads(bgr).get(int(zone))


def arm_exclude(pads, shape):
    """Vùng loại trừ quanh cánh tay theo bố trí ô: giữa ô xanh dương và xanh lá, từ mép trên tới hàng ô.

    Không có hai ô đó thì dùng vùng mặc định theo tỉ lệ khung.
    """
    h, w = shape[:2]
    if 1 in pads and 2 in pads:
        left, right = pads[1].hull[:, 0].max(), pads[2].hull[:, 0].min()
        bottom = 0.5 * (_centre(pads[1].hull)[1] + _centre(pads[2].hull)[1]) + 0.04 * h
        if right - left > 0.15 * w:
            return (left / w, 0.0, right / w, min(bottom / h, 0.6))
    return DEFAULT_EXCLUDE


def _normalise(before, after):
    """Chỉnh độ sáng ảnh sau về ảnh trước (tự phơi sáng trôi giữa hai lần chụp)."""
    b, a = float(before.mean()), float(after.mean())
    if a < 1e-3:
        return after
    return np.clip(after.astype(np.float32) * (b / a), 0, 255).astype(np.uint8)


def change_blobs(before, after, exclude=DEFAULT_EXCLUDE):
    """Các vùng đổi giữa hai ảnh: [(mask, area)], đã bỏ vùng cánh tay."""
    after = _normalise(before, after)
    a = cv2.GaussianBlur(before, (7, 7), 0)
    b = cv2.GaussianBlur(after, (7, 7), 0)
    diff = np.abs(a.astype(np.int16) - b.astype(np.int16)).max(axis=2)
    mask = (diff > DIFF_THRESHOLD).astype(np.uint8) * 255
    if exclude:
        h, w = mask.shape
        x0, y0, x1, y1 = exclude
        mask[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [((labels == i), int(stats[i, cv2.CC_STAT_AREA])) for i in range(1, count)
            if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_PIXELS]


def _base_fraction(blob, region):
    """Phần nửa dưới (gần bàn nhất trong ảnh) của vùng đổi nằm trong ô."""
    ys, xs = np.nonzero(blob)
    keep = ys >= np.median(ys)
    return float(np.count_nonzero(region[ys[keep], xs[keep]])) / max(int(keep.sum()), 1)


def verify_in_zone(before, after, zone, exclude="auto"):
    """So ảnh trước/sau thả -> {"verdict", "ok", "text", "metrics"}."""
    if before is None or after is None or before.shape != after.shape:
        return _result("unseen", {"reason": "thiếu ảnh trước/sau"})
    pads = find_pads(before)
    pad = pads.get(int(zone))
    if pad is None or pad.inferred:
        pad = find_pads(after).get(int(zone)) or pad
    if pad is None:
        return _result("unseen", {"reason": f"không thấy ô zone {zone} trong ảnh camera ngoài"})
    if exclude == "auto":
        exclude = arm_exclude(pads, before.shape)
    region = pad.mask(before.shape) > 0
    blobs = change_blobs(before, after, exclude)
    best = None
    others = 0
    for blob, area in blobs:
        inside = int(np.count_nonzero(blob & region))
        if inside >= 0.25 * MIN_IN_PIXELS:
            if best is None or inside > best[0]:
                best = (inside, area, _base_fraction(blob, region))
        else:
            others += 1
    metrics = {"blobs": len(blobs), "other_blobs": others, "pad_area_px": round(pad.area),
               "pad_inferred": pad.inferred}
    min_in = max(MIN_IN_PIXELS, MIN_IN_FRAC_OF_PAD * pad.area)
    if best is not None:
        inside, area, base_frac = best
        frac = inside / max(area, 1)
        metrics.update(changed_in_pad_px=inside, in_pad_fraction=round(frac, 2),
                       base_in_pad_fraction=round(base_frac, 2))
        # Cube cao 30 mm nên trong ảnh nghiêng phần trên lòi ra sau ô: chỉ cần phần chạm đáy (nửa dưới) trong ô.
        if inside >= min_in and max(frac, base_frac) >= IN_ZONE_FRAC:
            return _result("in_zone", metrics)
        return _result("partial", metrics)
    # Ô và camera đều ổn mà không có gì mới trong ô: cube không nằm ở đó.
    return _result("outside" if others >= 2 else "missing", metrics)


def _result(verdict, metrics):
    return {"verdict": verdict, "ok": verdict in GOOD, "text": VERDICT_TEXT[verdict], "metrics": metrics}


def save_debug(directory, before, after, zone, exclude, result):
    """Lưu ảnh trước/sau và ảnh chồng (viền ô vàng, vùng loại trừ xanh, vùng đổi đỏ) để audit; trả đường dẫn."""
    import os
    try:
        os.makedirs(directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        pads = find_pads(before)
        overlay = after.copy()
        for k, pad in pads.items():
            colour = (0, 255, 255) if k == int(zone) else (255, 200, 0)
            cv2.polylines(overlay, [pad.hull.reshape(-1, 1, 2)], True, colour, 3 if k == int(zone) else 1)
        box = arm_exclude(pads, before.shape) if exclude == "auto" else exclude
        if box:
            h, w = overlay.shape[:2]
            cv2.rectangle(overlay, (int(box[0] * w), int(box[1] * h)), (int(box[2] * w), int(box[3] * h)),
                          (0, 200, 0), 1)
        for blob, _ in change_blobs(before, after, box):
            overlay[blob] = (0.5 * overlay[blob] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
        cv2.putText(overlay, f"zone {zone}: {result['verdict']}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        base = os.path.join(directory, f"{stamp}_zone{zone}")
        for name, img in (("before", before), ("after", after), ("overlay", overlay)):
            cv2.imwrite(f"{base}_{name}.png", img)
        return base + "_overlay.png"
    except Exception:  # noqa: BLE001 - ghi ảnh debug không được làm hỏng lệnh
        return None


class ExternalCamera:
    """Camera ngoài: mở, chờ tự phơi sáng, lấy ảnh trung vị rồi nhả (không giữ camera giữa các lần)."""

    def __init__(self, path, warm_frames=30, samples=3, width=640, height=480, fourcc=None):
        self.path, self.warm_frames, self.samples = path, warm_frames, samples
        self.size = (width, height)
        self.fourcc = fourcc        # ví dụ "MJPG": cần cho 1280x720 (YUYV ở cỡ đó rất chậm hoặc không có)

    def _configure(self, cap):
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])

    def grab(self):
        cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        try:
            self._configure(cap)
            frames = []
            for i in range(self.warm_frames + self.samples):
                ok, frame = cap.read()
                if ok and i >= self.warm_frames:
                    frames.append(frame)
                if not ok:
                    time.sleep(0.02)
            return np.median(np.stack(frames), axis=0).astype(np.uint8) if frames else None
        finally:
            cap.release()


def _sequence(self, count, interval_s):
    """`count` khung cách nhau ~interval_s, mở camera MỘT lần (chờ phơi sáng một lần)."""
    cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return []
    frames = []
    try:
        self._configure(cap)
        for _ in range(self.warm_frames):
            cap.read()
        for i in range(count):
            if i:
                deadline = time.monotonic() + interval_s
                while time.monotonic() < deadline:       # đọc liên tục để không lấy khung cũ trong bộ đệm
                    cap.read()
            ok, frame = cap.read()
            if ok:
                frames.append(frame)
        return frames
    finally:
        cap.release()


ExternalCamera.sequence = _sequence


class PlacementVerifier:
    """before()/check(zone): bộ kiểm tra dùng trong vòng thực thi (đổi camera bằng ExternalCamera khác)."""

    def __init__(self, camera, exclude="auto", debug_dir="/tmp/t8_verify", locator=None):
        self.camera, self.exclude, self.reference = camera, exclude, None
        # locator(cube_id) -> {"x","y"} | None: vị trí cube trong base từ camera ngoài đã hiệu chuẩn
        # (external_camera.make_locator). Chỉ thêm số đo mét vào kết quả; kết luận vẫn theo so màu.
        self.locator = locator
        self.debug_dir = debug_dir          # ảnh trước/sau + viền ô + vùng đổi để audit; None = không lưu

    def before(self):
        self.reference = self.camera.grab()
        return self.reference is not None

    def check(self, zone, samples=3, interval_s=1.5):
        """Chụp `samples` khung sau khi tay về (camera mở một lần), kết luận theo khung CUỐI; nếu có khung
        đầu đã vào ô mà khung cuối thì không => cube bị văng ra SAU khi thả (`bounced`)."""
        if hasattr(self.camera, "sequence") and samples > 1:
            frames = self.camera.sequence(samples, interval_s)
        else:
            frames = [self.camera.grab()]
        frames = [f for f in frames if f is not None]
        if self.reference is None or not frames:
            return _result("unseen", {"reason": "camera ngoài không cho ảnh"})
        verdicts = [verify_in_zone(self.reference, f, zone, self.exclude) for f in frames]
        after, result = frames[-1], verdicts[-1]
        result["timeline"] = [v["verdict"] for v in verdicts]
        result["bounced"] = bool(any(v["ok"] for v in verdicts[:-1]) and not result["ok"])
        if result["bounced"]:
            result["text"] = "cube đã vào ô rồi bị văng ra ngoài"
        if self.debug_dir:
            result["debug"] = save_debug(self.debug_dir, self.reference, after, zone, self.exclude, result)
        if self.locator is not None:
            try:
                found = self.locator(zone)
            except Exception:  # noqa: BLE001 - số đo phụ không được làm hỏng bước kiểm tra
                found = None
            if found:
                result["cube_xy"] = [round(found["x"], 4), round(found["y"], 4)]
        return result


def main():
    """Kiểm tra camera ngoài: chụp một khung, báo ô nào thấy được, lưu ảnh có vẽ viền ô."""
    import argparse
    from .cameras import list_cameras, resolve_external, resolve_wrist
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("--camera", default="auto", help="camera ngoài (mặc định: camera USB khác camera tay)")
    ap.add_argument("--out", default="/tmp/external_check.png")
    args = ap.parse_args()
    devices = list_cameras()
    for dev in devices:
        print(f"{dev['path']}  {dev['name']}")
    wrist, _ = resolve_wrist(devices)
    path = resolve_external(wrist, args.camera, devices)
    if not path:
        raise SystemExit("Không có camera ngoài (chỉ thấy một camera).")
    frame = ExternalCamera(path).grab()
    if frame is None:
        raise SystemExit(f"Không đọc được khung từ {path} (đang bị tiến trình khác giữ?).")
    print(f"camera ngoài: {path}; độ sáng trung bình {frame.mean():.0f}/255")
    names = {1: "xanh dương", 2: "xanh lá", 3: "đỏ", 4: "xám"}
    found = 0
    pads = find_pads(frame)
    for zone, name in names.items():
        pad = pads.get(zone)
        found += pad is not None
        print(f"  zone {zone} ({name}): " + (
            f"{'SUY RA từ ba ô còn lại (bị che)' if pad.inferred else 'thấy'}, diện tích {pad.area:.0f}px"
            if pad else "KHÔNG thấy"))
        if pad is not None:
            cv2.polylines(frame, [pad.hull.reshape(-1, 1, 2)], True,
                          (0, 165, 255) if pad.inferred else (0, 255, 255), 2)
    cv2.imwrite(args.out, frame)
    print(f"ảnh có viền ô: {args.out}")
    raise SystemExit(0 if found == 4 else 1)


if __name__ == "__main__":
    main()
