#!/usr/bin/env python3
"""Tìm 4 ô thả (zone) bằng camera tay và đối chiếu với góc J1 đang cấu hình.

Ô màu = màu cube (đỏ→3, xanh dương→1, xanh lá→2); cube vàng (4) vào ô xám còn lại.

Điều cần biết là "cube thả ở J1 cấu hình có rơi vào ô không, và sâu bao nhiêu mm bên
trong ô". Với mỗi zone: tay nhìn về góc J1 cấu hình, lấy mặt nạ màu của ô, chiếu
điểm thả (FK của pose thả) xuống mặt bàn rồi vào ảnh bằng hand-eye + khớp đo thật,
đo khoảng cách có dấu tới mép ô (mép chạm khung ảnh không tính là mép vì ô bị cắt).
Nếu biên an toàn đủ lớn thì giữ góc cấu hình; nếu không thì tìm góc J1 lệch ≤15° cho
biên lớn nhất; không chắc chắn thì quay về góc cấu hình và cảnh báo.
Lưu ý: không dùng bearing ảnh của tâm ô — camera nằm trước trục đế nên bearing đó khác
phương vị quanh đế tới ~20° (đã đo trên tay thật). Độ chính xác của kết quả phụ thuộc
độ chính xác hand-eye ở J1 xa vùng hiệu chuẩn (validate_hand_eye.py).
Kết quả chỉ dùng trong lần chạy hiện tại (không ghi file).
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cube_vision import color as _color  # noqa: E402
from cube_vision import registry as _registry  # noqa: E402

# Ô nhận cube theo màu; cube vàng (ID 4) không có ô vàng nên dùng ô xám.
ZONE_PAD = {1: "khoi_xanh_duong", 2: "khoi_xanh", 3: "khoi_do", 4: "xam"}
PAD_HSV = {
    # V>=150 loại gỗ nâu (H~10, S 120-160, V~110) nhưng giữ ô đỏ (S~190, V~228)
    "khoi_do": [((0, 130, 150), (8, 255, 255)), ((168, 130, 150), (179, 255, 255))],
    "khoi_xanh_duong": [((95, 120, 90), (130, 255, 255))],
    "khoi_xanh": [((40, 90, 60), (85, 255, 255))],
    "xam": [((0, 0, 70), (179, 45, 165))],
}
MIN_PAD_AREA = 5000.0          # px; ô thật chiếm hàng chục nghìn px ở pose nhìn
MIN_SOLIDITY = 0.70            # diện tích / bao lồi: ô là khối đặc, thảm vỡ mảnh thì thấp
SAFE_MARGIN_MM = 15.0          # thả cách mép ô >= ngưỡng này: giữ góc cấu hình
MIN_GAIN_MM = 8.0              # chỉ đổi góc khi biên tốt hơn ít nhất ngần này
MAX_SHIFT_DEG = 30.0           # biên độ chỉnh J1 tối đa so với cấu hình (ô có thể bị dời xa)
SHIFT_STEP_DEG = 1.0
MIN_IMAGE_MARGIN_PX = 25       # điểm thả phải nằm trong ảnh cách mép >= ngần này mới tin
FRAME_W, FRAME_H = 640, 480


@dataclass
class PadHit:
    zone_id: int
    center: tuple          # (cx, cy) pixel, tâm vùng nhìn thấy
    area: float
    clipped: bool          # chạm mép ảnh
    solidity: float


def pad_mask(bgr, zone_id: int):
    """Mặt nạ (uint8) của ô lớn nhất đúng màu của zone, hoặc None."""
    if bgr is None or bgr.shape[:2] != (FRAME_H, FRAME_W):
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = _color.hsv_mask(hsv, PAD_HSV[ZONE_PAD[zone_id]])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < MIN_PAD_AREA:
            continue
        hull = float(cv2.contourArea(cv2.convexHull(cnt)))
        solidity = area / hull if hull > 0 else 0.0
        if solidity < MIN_SOLIDITY:
            continue
        if best is None or area > best[0]:
            best = (area, cnt, solidity)
    if best is None:
        return None
    out = np.zeros((FRAME_H, FRAME_W), np.uint8)
    cv2.drawContours(out, [best[1]], -1, 255, thickness=cv2.FILLED)
    return out


def find_pad(bgr, zone_id: int):
    """Ô lớn nhất đúng màu của zone (tâm vùng nhìn thấy), hoặc None."""
    mask = pad_mask(bgr, zone_id)
    if mask is None:
        return None
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(cnt))
    hull = float(cv2.contourArea(cv2.convexHull(cnt)))
    m = cv2.moments(cnt)
    x, y, w, h = cv2.boundingRect(cnt)
    clipped = x <= 2 or y <= 2 or x + w >= FRAME_W - 2 or y + h >= FRAME_H - 2
    return PadHit(zone_id, (m["m10"] / m["m00"], m["m01"] / m["m00"]), area, clipped,
                  area / hull if hull > 0 else 0.0)


def project_to_pixel(point_base, servo5, cal):
    """(u, v, depth_m) của điểm base (x, y, z) trong ảnh, hoặc None nếu ở sau camera."""
    import cube_search_center_math as M
    from cube_layer import _project
    base_T_opt = M.fk_arm4_cal(servo5, cal) @ np.asarray(cal["arm4_T_optical"], float)
    cam = (np.linalg.inv(base_T_opt) @ np.r_[np.asarray(point_base, float), 1.0])[:3]
    if cam[2] <= 0.05:
        return None
    u, v = _project(cam, cal["K"], float(cal.get("k1", 0.0)))
    return float(u), float(v), float(cam[2])


def signed_margin_mm(mask, release_xy, servo5, cal, table_z, fx=None):
    """Khoảng cách có dấu (mm) từ điểm thả (trên mặt bàn) tới mép ô: dương = nằm trong ô.

    None khi điểm thả không nằm trong ảnh (cách mép < MIN_IMAGE_MARGIN_PX) nên chưa kết luận được.
    Mép trùng khung ảnh không tính là mép ô (ô bị cắt bởi khung, không phải hết ô).
    """
    hit = project_to_pixel([release_xy[0], release_xy[1], table_z], servo5, cal)
    if hit is None:
        return None
    u, v, depth = hit
    if not (MIN_IMAGE_MARGIN_PX <= u < FRAME_W - MIN_IMAGE_MARGIN_PX and
            MIN_IMAGE_MARGIN_PX <= v < FRAME_H - MIN_IMAGE_MARGIN_PX):
        return None
    pad = 40
    padded = cv2.copyMakeBorder(mask, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    inside = cv2.distanceTransform((padded > 0).astype(np.uint8), cv2.DIST_L2, 5)
    outside = cv2.distanceTransform((padded == 0).astype(np.uint8), cv2.DIST_L2, 5)
    iu, iv = int(round(u)) + pad, int(round(v)) + pad
    px = float(inside[iv, iu]) if padded[iv, iu] > 0 else -float(outside[iv, iu])
    focal = float((fx if fx is not None else cal["K"][0]))
    return px / (focal / depth) * 1000.0


def rotate_about_base(xy, deg):
    """Quay điểm quanh trục đế theo chiều J1 tăng (ngược chiều kim đồng hồ nhìn từ trên)."""
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return (xy[0] * c - xy[1] * s, xy[0] * s + xy[1] * c)


def best_shift(mask, release_xy, servo5, cal, table_z):
    """(delta_deg, biên tại 0, biên tốt nhất): quét J1 lệch trong ±MAX_SHIFT_DEG."""
    base = signed_margin_mm(mask, release_xy, servo5, cal, table_z)
    best = (0.0, base)
    steps = int(MAX_SHIFT_DEG / SHIFT_STEP_DEG)
    for k in range(-steps, steps + 1):
        d = k * SHIFT_STEP_DEG
        m = signed_margin_mm(mask, rotate_about_base(release_xy, d), servo5, cal, table_z)
        if m is None:
            continue
        if best[1] is None or m > best[1] + 1e-9 or (abs(m - best[1]) <= 1e-9 and abs(d) < abs(best[0])):
            best = (d, m)
    return best[0], base, best[1]


def decide(configured_j1: float, delta_deg: float, margin_cfg, margin_best):
    """(j1 dùng, hành động, ghi chú): keep | shifted | fallback."""
    if margin_cfg is not None and margin_cfg >= SAFE_MARGIN_MM:
        return float(configured_j1), "keep", f"biên {margin_cfg:.0f} mm >= {SAFE_MARGIN_MM:.0f}"
    if margin_best is None:
        return float(configured_j1), "fallback", "không đo được biên (điểm thả ngoài khung ảnh)"
    cfg = -1e9 if margin_cfg is None else margin_cfg
    if margin_best >= SAFE_MARGIN_MM and margin_best - cfg >= MIN_GAIN_MM \
            and abs(delta_deg) <= MAX_SHIFT_DEG:
        return float(configured_j1) + delta_deg, "shifted", \
            f"biên {cfg:.0f} -> {margin_best:.0f} mm khi lệch {delta_deg:+.0f}°"
    return float(configured_j1), "fallback", \
        f"biên cấu hình {cfg:.0f} mm, tốt nhất {margin_best:.0f} mm (không đủ chắc để đổi)"


@dataclass
class ZoneResult:
    zone_id: int
    configured_j1: float
    use_j1: float
    action: str
    margin_cfg_mm: float | None
    margin_best_mm: float | None
    note: str = ""


def survey_zone(zone_id: int, configured_j1: float, release_xy, move_j1, grab, cal,
                table_z: float, log=print) -> ZoneResult:
    """Nhìn một zone. move_j1(j1) di chuyển, chờ đứng yên và trả 5 góc khớp đo thật;
    grab() trả khung BGR 640x480 tươi. release_xy = XY (base) của điểm thả của zone."""
    servo5 = move_j1(float(configured_j1))
    mask = pad_mask(grab(), zone_id)
    if mask is None:
        res = ZoneResult(zone_id, float(configured_j1), float(configured_j1), "fallback",
                         None, None, "không thấy ô")
    else:
        delta, m_cfg, m_best = best_shift(mask, release_xy, servo5, cal, table_z)
        use, action, note = decide(configured_j1, delta, m_cfg, m_best)
        res = ZoneResult(zone_id, float(configured_j1), use, action, m_cfg, m_best, note)
    log(f"[ZONE {zone_id}] J1 cấu hình {configured_j1:.1f}° -> {res.action} "
        f"(dùng {res.use_j1:.1f}°); {res.note}")
    return res
