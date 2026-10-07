#!/usr/bin/env python3
"""Xác định tầng và XY của cube từ pose camera + khớp thật + hand-eye đã hiệu chuẩn.

Không dùng z tuyệt đối của PnP (sai số vài mm tới ~1 cm). Thay vào đó thử từng
giả thuyết tầng n: mặt trên cube ở z_n = tag_top_z + 0.030*n, giao tia qua pixel
tâm mặt trên với mặt phẳng đó rồi so quãng đường dọc tia với khoảng cách PnP.
Hai tầng liền nhau cách nhau >= 30 mm dọc tia, lớn hơn nhiều sai số PnP, nên
chọn tầng đủ chắc; mơ hồ thì từ chối thay vì đoán. XY lấy từ giao tia ở tầng
đã chọn (cách search-center đang dùng, sai số ~1-2 mm).
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

import cube_search_center_math as M

CUBE_EDGE_M = 0.030
PICK_TCP_Z_TABLE = 0.047      # độ cao gắp đã kiểm chứng với cube trên bàn
MAX_LAYER = 3                 # 4 cube: tầng 0..3
DEPTH_TOL_M = 0.012           # lệch tối đa giữa khoảng cách PnP và giả thuyết tầng
MIN_TOP_FACING = 0.90         # cos giữa pháp tuyến mặt trên và phương đứng
ARM_JOINTS = [f"arm{i}_Joint" for i in range(1, 6)]


def servo_from_joint_state(names, positions_rad):
    """[J1..J5] độ servo từ JointState (ngược của real_joint_mirror), None nếu thiếu."""
    by_name = dict(zip(names, positions_rad))
    if not all(name in by_name for name in ARM_JOINTS):
        return None
    servo = [math.degrees(float(by_name[name])) + 90.0 for name in ARM_JOINTS]
    return servo if all(math.isfinite(v) for v in servo) else None


class JointWindow:
    """Lịch sử khớp ngắn để biết tay đã đứng yên và đọc góc ổn định."""

    # real_joint_mirror publishes a fresh reading only every ~1.5 s (6 serial
    # reads per cycle), so the window must span several periods or "stationary"
    # is almost never true; stillness is still enforced by tol_deg.
    def __init__(self, window_s: float = 3.5, tol_deg: float = 0.8,
                 max_gap_s: float = 3.5):
        self.window_s, self.tol_deg, self.max_gap_s = window_s, tol_deg, max_gap_s
        self.history: deque = deque()

    def add(self, stamp_s: float, servo5) -> None:
        q = np.asarray(servo5, float)
        if not np.isfinite(q).all():
            self.history.clear()
            return
        if self.history and stamp_s <= self.history[-1][0]:
            self.history.clear()
        self.history.append((float(stamp_s), q))
        while len(self.history) > 2 and self.history[1][0] <= stamp_s - 2.0 * self.window_s:
            self.history.popleft()

    def stationary_servo(self, stamp_s: float):
        """Góc trung bình quanh stamp_s nếu tay đứng yên đủ lâu, ngược lại None."""
        if len(self.history) < 2:
            return None
        recent = [(t, q) for t, q in self.history if t >= self.history[-1][0] - self.window_s]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < min(self.window_s, 0.4):
            return None
        if np.max(np.ptp(np.array([q for _, q in recent]), axis=0)) > self.tol_deg:
            return None
        if abs(stamp_s - self.history[-1][0]) > self.max_gap_s:
            return None
        return np.mean([q for _, q in recent], axis=0).tolist()


def _project(point_cam, K, k1: float):
    x, y = point_cam[0] / point_cam[2], point_cam[1] / point_cam[2]
    d = 1.0 + k1 * (x * x + y * y)
    return K[0] * x * d + K[2], K[1] * y * d + K[3]


def _unproject(u, v, K, k1: float):
    xd, yd = (u - K[2]) / K[0], (v - K[3]) / K[1]
    x, y = xd, yd
    for _ in range(8):
        d = 1.0 + k1 * (x * x + y * y)
        x, y = xd / d, yd / d
    vec = np.array([x, y, 1.0])
    return vec / np.linalg.norm(vec)


READY_SERVO = [90.0, 125.0, 0.0, 0.0, 90.0]


def ready_view_yaw_deg(base_R_cube, top_axis, centre_base, cal, half_edge_m: float = CUBE_EDGE_M / 2):
    """Góc yaw của cube như camera nhìn từ READY_POSE (quy ước yaw cũ của T8).

    Quy ước cũ: góc cạnh mặt trên trong ảnh ở READY (±40°), động cơ dùng
    J5 = J1 - yaw. Ảnh quay theo J1 và nghiêng theo khớp, nên ở pose quan sát
    khác READY phải quy về hướng cạnh trong hệ base rồi chiếu lại như từ READY.
    """
    in_plane = [i for i in range(3) if i != top_axis][0]
    direction = np.asarray(base_R_cube, float)[:, in_plane]
    direction = np.array([direction[0], direction[1], 0.0])
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        return 0.0
    direction /= norm
    base_T_ready = M.fk_arm4_cal(READY_SERVO, cal) @ np.asarray(cal["arm4_T_optical"], float)
    inv = np.linalg.inv(base_T_ready)
    fx, fy, cx, cy = cal["K"]
    pix = []
    for sign in (-1.0, 1.0):
        p = inv @ np.r_[np.asarray(centre_base, float) + sign * half_edge_m * direction, 1.0]
        if p[2] <= 0.02:
            return 0.0
        pix.append((fx * p[0] / p[2], fy * p[1] / p[2]))
    dx, dy = pix[1][0] - pix[0][0], pix[1][1] - pix[0][1]
    angle = math.degrees(math.atan2(dy, dx)) % 90.0
    yaw = -angle if angle <= 45.0 else 90.0 - angle
    return round(max(-40.0, min(40.0, yaw)), 2)


def project_base_point(point_base, servo5, cal):
    """Pixel (u, v) của điểm base trong ảnh ở khớp servo5 (hand-eye đã hiệu chuẩn), hoặc None."""
    base_T_opt = M.fk_arm4_cal(servo5, cal) @ np.asarray(cal["arm4_T_optical"], float)
    cam = (np.linalg.inv(base_T_opt) @ np.r_[np.asarray(point_base, float), 1.0])[:3]
    if cam[2] <= 0.05:
        return None
    fx, fy, cx, cy = cal["K"]
    u, v = _project(cam, (fx, fy, cx, cy), float(cal.get("k1", 0.0)))
    return float(u), float(v)


def locate_cube(camera_T_cube, servo5, cal, perception_K=None,
                half_edge_m: float = CUBE_EDGE_M / 2):
    """Tầng + XY của cube từ camera_T_cube (4x4, của perception) và khớp servo.

    cal: dict từ cube_search_center_math.load_calibration().
    perception_K: (fx, fy, cx, cy, k1) mà perception đã dùng để giải PnP; mặc định
    là chính bộ thông số đã hiệu chuẩn. Dùng nó để chiếu tâm mặt trên về pixel
    (vòng PnP -> pixel khép kín), rồi tia thật lấy theo thông số đã hiệu chuẩn.
    Trả {"ok": bool, "reason": str, ...}; khi ok có x, y, layer, tcp_z, top_z,
    range_error_mm, margin_mm.
    """
    fail = lambda reason: {"ok": False, "reason": reason}  # noqa: E731
    try:
        T = np.asarray(camera_T_cube, float).reshape(4, 4)
        servo = [float(v) for v in servo5[:5]]
    except (TypeError, ValueError):
        return fail("pose/khớp không hợp lệ")
    if not np.isfinite(T).all() or len(servo) < 5:
        return fail("pose/khớp không hợp lệ")
    j1_range = cal.get("j1_valid_range")
    if j1_range and not (j1_range[0] <= servo[0] <= j1_range[1]):
        return fail(f"J1={servo[0]:.0f}° ngoài vùng đã hiệu chuẩn ({j1_range[0]:.0f}-{j1_range[1]:.0f}°); "
                    "xoay về gần giữa bàn để quan sát")
    fx, fy, cx, cy = cal["K"]
    k1 = float(cal.get("k1", 0.0))
    pk = tuple(perception_K) if perception_K is not None else (fx, fy, cx, cy, k1)
    base_T_opt = M.fk_arm4_cal(servo, cal) @ np.asarray(cal["arm4_T_optical"], float)

    # Mặt trên = mặt có pháp tuyến hướng lên trong base_link.
    best = None
    for axis in range(3):
        for sign in (-1.0, 1.0):
            normal = base_T_opt[:3, :3] @ (T[:3, :3] @ (sign * np.eye(3)[axis]))
            if best is None or normal[2] > best[0]:
                best = (float(normal[2]), axis, sign)
    up, axis, sign = best
    if up < MIN_TOP_FACING:
        return fail(f"cube nghiêng ({math.degrees(math.acos(min(1.0, up))):.0f}°): không có mặt trên nằm ngang")
    face_cam = T[:3, 3] + T[:3, :3] @ (sign * half_edge_m * np.eye(3)[axis])
    if face_cam[2] <= 0.05:
        return fail("pose camera của cube không hợp lệ (phía sau camera)")
    pnp_range = float(np.linalg.norm(face_cam))
    u, v = _project(face_cam, pk[:4], pk[4])

    ray = base_T_opt[:3, :3] @ _unproject(u, v, (fx, fy, cx, cy), k1)
    origin = base_T_opt[:3, 3]
    if ray[2] >= -0.05:
        return fail("camera không nhìn xuống bàn ở pose này")
    scored = []
    for layer in range(MAX_LAYER + 1):
        top_z = float(cal["tag_top_z"]) + CUBE_EDGE_M * layer
        t = (top_z - origin[2]) / ray[2]
        if t > 0:
            scored.append((abs(t - pnp_range), layer, top_z, t))
    if not scored:
        return fail("không có tầng nào cắt tia nhìn")
    scored.sort()
    err, layer, top_z, t = scored[0]
    margin = (scored[1][0] - err) if len(scored) > 1 else math.inf
    if err > DEPTH_TOL_M:
        return {"ok": False, "range_error_mm": round(err * 1000, 1),
                "reason": (f"không khớp tầng nào: lệch {err * 1000:.0f} mm "
                           f"(gần nhất tầng {layer}); kiểm tra hiệu chuẩn/khớp")}
    hit = origin + ray * t
    base_R = base_T_opt[:3, :3] @ T[:3, :3]
    yaw_deg = ready_view_yaw_deg(base_R, axis, hit, cal)
    edge_dir = base_R[:, [i for i in range(3) if i != axis][0]]
    base_yaw = (math.atan2(edge_dir[1], edge_dir[0]) + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return {"ok": True, "reason": "", "layer": layer, "yaw_deg": yaw_deg,
            "base_yaw_rad": round(base_yaw, 4),
            "x": round(float(hit[0]), 5), "y": round(float(hit[1]), 5),
            "top_z": round(top_z, 5),
            "tcp_z": round(PICK_TCP_Z_TABLE + CUBE_EDGE_M * layer, 5),
            "range_error_mm": round(err * 1000, 1),
            "margin_mm": None if math.isinf(margin) else round(margin * 1000, 1),
            "pixel": [round(float(u), 1), round(float(v), 1)]}
