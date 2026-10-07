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
