#!/usr/bin/env python3
"""Thư viện pose nhìn của tay (không chỉ READY) và chọn pose thấy được mục tiêu.

Mỗi pose có tên, 5 góc khớp và được kiểm tra an toàn (đầu kẹp cao hơn mặt cube) bằng
cùng hàm với hiệu chuẩn hand-eye. `choose_view` giữ pose hiện tại nếu mục tiêu đã
nằm gọn trong ảnh, ngược lại chọn pose gần nhất trong thư viện thấy được mục tiêu.
"""
from __future__ import annotations

import math

import numpy as np

import calibrate_hand_eye as C

# Tên -> [J1..J5]. J1 quay trái/phải; J2..J4 như READY (nhìn xuống bàn phía trước).
POSES = {
    "READY": [90.0, 125.0, 0.0, 0.0, 90.0],
    "LEFT": [130.0, 125.0, 0.0, 0.0, 90.0],
    "FAR_LEFT": [150.0, 125.0, 0.0, 0.0, 90.0],
    "RIGHT": [50.0, 125.0, 0.0, 0.0, 90.0],
    "FAR_RIGHT": [30.0, 125.0, 0.0, 0.0, 90.0],
    "HIGH": [90.0, 105.0, 15.0, 10.0, 90.0],
    "LEFT_HIGH": [125.0, 105.0, 15.0, 10.0, 90.0],
    "RIGHT_HIGH": [55.0, 105.0, 15.0, 10.0, 90.0],
}
IMAGE_MARGIN_PX = (60, 50)          # tâm mục tiêu cách mép ảnh tối thiểu (x, y)
DEPTH_RANGE_M = (0.08, 0.40)


def is_safe(servo) -> bool:
    return C.tip_z(servo) >= C.MIN_TIP_Z


def visible_in(servo, target_base, cal, margin=IMAGE_MARGIN_PX) -> bool:
    """Điểm base nằm gọn trong ảnh ở pose này?"""
    fx, fy, cx, cy = cal["K"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    hit = C.predict_pixel(servo, np.asarray(target_base, float), np.asarray(cal["arm4_T_optical"], float), K)
    if hit is None:
        return False
    u, v, depth = hit
    return (margin[0] <= u <= 640 - margin[0] and margin[1] <= v <= 480 - margin[1]
            and DEPTH_RANGE_M[0] <= depth <= DEPTH_RANGE_M[1])


def j1_distance(a, b) -> float:
    return abs(float(a[0]) - float(b[0])) + 0.25 * sum(abs(float(x) - float(y)) for x, y in zip(a[1:4], b[1:4]))


def choose_view(current_servo, target_base, cal, poses=None):
    """(tên, servo, đổi_pose): giữ pose hiện tại nếu thấy mục tiêu, không thì pose gần nhất thấy được.

    Trả (None, None, False) khi không pose nào an toàn và thấy được mục tiêu."""
    if current_servo is not None and visible_in(current_servo, target_base, cal):
        return "CURRENT", [float(v) for v in current_servo[:5]], False
    candidates = []
    for name, servo in (poses or POSES).items():
        if is_safe(servo) and visible_in(servo, target_base, cal):
            dist = 0.0 if current_servo is None else j1_distance(current_servo, servo)
            candidates.append((dist, name, servo))
    if not candidates:
        return None, None, False
    _, name, servo = min(candidates, key=lambda c: c[0])
    return name, list(servo), True
