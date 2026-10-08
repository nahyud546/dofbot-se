"""Mặt nạ HSV và ứng viên ô vuông theo màu."""
from __future__ import annotations

import cv2
import numpy as np


def hsv_mask(hsv, ranges):
    """OR của các dải [(lower, upper), ...] trên ảnh HSV."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    return mask


def square_candidates(bgr, ranges_by_label, label_to_id, want_id=None,
                      area_range=(1000, 30000), source="hsv"):
    """Ô vuông (4 góc lồi, gần vuông) theo từng màu: cách search-center đang dùng.

    Trả [{"cube_id","center","quad","source"}]. Ảnh phải 640x480.
    """
    if bgr is None or bgr.shape[:2] != (480, 640):
        return []
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found = []
    for name, ranges in ranges_by_label.items():
        cid = label_to_id.get(name)
        if want_id is not None and cid != int(want_id):
            continue
        mask = hsv_mask(hsv, ranges)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not area_range[0] <= area <= area_range[1]:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = None
            for eps in (0.04, 0.06, 0.08):
                ap = cv2.approxPolyDP(cnt, eps * peri, True)
                if len(ap) != 4 or not cv2.isContourConvex(ap):
                    continue
                x, y, w, h = cv2.boundingRect(ap)
                if x < 10 or y < 10 or x + w > 635 or \
                        not 0.65 <= w / h <= 1.55 or area / (w * h) < 0.60:
                    continue
                approx = ap
                break
            if approx is None:
                continue
            corners = approx.reshape(4, 2).astype(float)
            center = corners.mean(axis=0)
            found.append({"cube_id": cid, "center": [float(center[0]), float(center[1])],
                          "quad": corners.tolist(), "source": source})
    return found


# Khoảng HSV cho việc tìm MẶT màu (không chồng nhau giữa các màu, khác bảng `registry.hsv_ranges` dùng cho tìm cube
# nhìn thẳng). Đo trên camera tay khi đủ sáng 2026-10-08: vàng H 21–23, S 255; lục đậm H 34–54, S 93–167, V 42–52;
# xanh dương (mặt bên, hơi tối) H 112–121, S 121–171. Bảng cũ cho vàng H 15–45 nên nuốt cả lục đậm, và đòi xanh dương
# S ≥ 150 nên mặt bên bị rỗ.
FACE_RANGES = {
    "khoi_do": [((0, 110, 50), (9, 255, 255)), ((168, 110, 50), (179, 255, 255))],
    "khoi_vang": [((14, 90, 60), (30, 255, 255))],
    "khoi_xanh": [((31, 60, 20), (95, 255, 255))],
    "khoi_xanh_duong": [((96, 90, 25), (135, 255, 255))],
}


def colour_faces(bgr, label_to_id, ranges_by_label=None, min_area: float = 900.0, border: int = 4):
    """Mảng màu đặc của từng cube, xấp xỉ thành tứ giác lồi: mặt màu của cube, dù ngửa lên hay quay ngang.

    Khác `square_candidates` (chỉ nhận ô gần vuông = mặt nhìn thẳng): ở đây KHÔNG lọc theo tỉ lệ cạnh, vì mặt bên
    nhìn chéo bị dẹt. Hình học (mặt 30 mm ở đâu, tầng nào) do bên gọi kiểm bằng `multiview.cube_face`.
    Bỏ mảng chạm mép ảnh (mặt bị cắt). Trả [{"cube_id", "label", "quad" (4,2), "confidence"}].
    """
    if bgr is None:
        return []
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found = []
    for name, ranges in (ranges_by_label or FACE_RANGES).items():
        cid = label_to_id.get(name)
        if cid is None:
            continue
        mask = cv2.morphologyEx(hsv_mask(hsv, ranges), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if x < border or y < border or x + bw > w - border or y + bh > h - border:
                continue
            hull = cv2.convexHull(cnt)
            peri = cv2.arcLength(hull, True)
            quad = None
            for eps in (0.02, 0.03, 0.045, 0.06, 0.08, 0.1):
                approx = cv2.approxPolyDP(hull, eps * peri, True)
                if len(approx) == 4:
                    quad = approx.reshape(4, 2).astype(float)
                    break
            if quad is None:
                continue
            solidity = area / max(cv2.contourArea(quad.astype(np.float32)), 1.0)
            if solidity < 0.8:                                   # mảng màu lồi lõm: không phải một mặt phẳng trọn vẹn
                continue
            found.append({"cube_id": int(cid), "label": name, "quad": quad, "confidence": float(min(1.0, solidity))})
    return found
