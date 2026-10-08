#!/usr/bin/env python3
"""Pose của cube từ 4 góc của MỘT mặt trên + góc khớp thật (cube nằm phẳng trên bàn).

PnP một mặt cần quad chính xác tới ~3 px và vẫn mơ hồ về hướng; ở pose lạ (nhìn thẳng xuống,
mặt không có tag) nó hỏng nên không có pose/skeleton. Ở đây thêm hai điều đã biết:
  - pose camera trong base (khớp thật + hand-eye đã hiệu chuẩn, đúng chuỗi mà locate_cube dùng),
  - cube nằm phẳng: mặt trên ngang ở z = tag_top_z + 0,03·tầng, vuông 30 mm.
Với mỗi tầng giả thuyết, chiếu 4 góc theo tia lên mặt phẳng ngang rồi khớp một hình vuông
30 mm; tầng có sai số khớp nhỏ nhất (và cách tầng kế một biên) là tầng thật. Quad của vật nhỏ
(mảnh hình in) hay quad nghiêng không khớp hình vuông 30 mm ở tầng nào nên bị từ chối.
Không dùng z tuyệt đối của PnP.
"""
from __future__ import annotations

import math

import numpy as np

import cube_layer as L
import cube_search_center_math as M

MAX_RMS_M = 0.0045          # sai số khớp hình vuông 30 mm tối đa (RMS góc, mét)
MIN_MARGIN_M = 0.0012       # tầng tốt nhất phải hơn tầng kế ít nhất chừng này
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


def _fit_square(points, edge):
    """Khớp hình vuông cạnh `edge` vào 4 điểm theo thứ tự vòng; (rms, tâm, yaw mod 90°)."""
    points = np.asarray(points, float)
    centre = points.mean(axis=0)
    rel = points - centre
    area = 0.5 * np.sum(points[:, 0] * np.roll(points[:, 1], -1) -
                        np.roll(points[:, 0], -1) * points[:, 1])
    s = 1.0 if area > 0 else -1.0
    steps = s * np.arange(4) * math.pi / 2
    theta = float(np.angle(np.sum(np.exp(1j * (np.arctan2(rel[:, 1], rel[:, 0]) - steps)))))
    ang = theta + steps
    ideal = centre + edge / math.sqrt(2) * np.stack([np.cos(ang), np.sin(ang)], axis=1)
    rms = float(math.sqrt(np.mean(np.sum((points - ideal) ** 2, axis=1))))
    edge_vec = ideal[1] - ideal[0]
    yaw = math.atan2(edge_vec[1], edge_vec[0])
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return rms, centre, yaw


def solve_top_face(corners_px, base_T_optical, K, k1, tag_top_z, layers=range(L.MAX_LAYER + 1),
                   edge=L.CUBE_EDGE_M, max_rms_m=MAX_RMS_M, min_margin_m=MIN_MARGIN_M):
    """Tầng/XY/yaw từ 4 góc mặt trên (pixel, thứ tự vòng).

    Trả {"ok": bool, "reason": str, ...}; khi ok: layer, top_z, x, y, yaw_rad (mod 90°),
    rms_m, margin_m, side_m, corners_base.
    """
    fail = lambda reason, **kw: {"ok": False, "reason": reason, **kw}  # noqa: E731
    try:
        pts = np.asarray(corners_px, float).reshape(4, 2)
        T = np.asarray(base_T_optical, float).reshape(4, 4)
    except (TypeError, ValueError):
        return fail("quad/pose không hợp lệ")
    if not (np.isfinite(pts).all() and np.isfinite(T).all()):
        return fail("quad/pose không hợp lệ")
    origin = T[:3, 3]
    rays = np.array([T[:3, :3] @ L._unproject(u, v, K, k1) for u, v in pts])
    if np.any(rays[:, 2] >= -0.05):
        return fail("camera không nhìn xuống bàn ở pose này")
    scored = []
    for layer in layers:
        z = float(tag_top_z) + edge * layer
        t = (z - origin[2]) / rays[:, 2]
        if np.any(t <= 0):
            continue
        base_pts = origin + rays * t[:, None]
        rms, centre, yaw = _fit_square(base_pts[:, :2], edge)
        side = float(np.mean(np.linalg.norm(base_pts[:, :2] - np.roll(base_pts[:, :2], -1, axis=0), axis=1)))
        scored.append((rms, layer, z, centre, yaw, side, base_pts[:, :2]))
    if not scored:
        return fail("không có tầng nào cắt tia nhìn")
    scored.sort(key=lambda item: item[0])
    rms, layer, z, centre, yaw, side, base_xy = scored[0]
    margin = scored[1][0] - rms if len(scored) > 1 else math.inf
    if rms > max_rms_m:
        return fail(f"quad không khớp hình vuông {edge * 1000:.0f} mm ở tầng nào "
                    f"(lệch {rms * 1000:.1f} mm, cạnh ~{side * 1000:.0f} mm)",
                    rms_m=rms, side_m=side)
    if margin < min_margin_m:
        return fail(f"mơ hồ tầng {layer}/{scored[1][1]} (hơn kém {margin * 1000:.1f} mm)",
                    rms_m=rms, side_m=side)
    return {"ok": True, "reason": "", "layer": layer, "top_z": z, "x": float(centre[0]),
            "y": float(centre[1]), "yaw_rad": yaw, "rms_m": rms,
            "margin_m": None if math.isinf(margin) else margin, "side_m": side,
            "corners_base": base_xy}


def base_R_cube_from_face(axis: str, yaw_rad: float):
    """Ma trận quay cube->base: mặt nhãn `axis` (pháp tuyến ra ngoài) hướng lên +Z, yaw mod 90°.

    Trả (R, top_axis_index). Hai trục còn lại nằm ngang, thuận tay.
    """
    i = AXIS_INDEX[axis[1]]
    sign = 1.0 if axis[0] == "+" else -1.0
    j, k = (i + 1) % 3, (i + 2) % 3
    up = np.array([0.0, 0.0, 1.0])
    col_i = sign * up
    col_j = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
    col_k = np.cross(col_i, col_j)
    R = np.zeros((3, 3))
    R[:, i], R[:, j], R[:, k] = col_i, col_j, col_k
    return R, i


def locate_face(corners_px, axis, servo5, cal):
    """Như cube_layer.locate_cube nhưng từ 4 góc mặt trên (kết quả cùng khóa).

    axis: nhãn mặt ("+X".."-Z") là mặt đang nhìn thấy ở trên cùng (mặt màu của perception là "-Z").
    """
    fail = lambda reason: {"ok": False, "reason": reason}  # noqa: E731
    try:
        servo = [float(v) for v in servo5[:5]]
    except (TypeError, ValueError):
        return fail("khớp không hợp lệ")
    if len(servo) < 5 or axis not in {f"{s}{a}" for s in "+-" for a in "XYZ"}:
        return fail("khớp/nhãn mặt không hợp lệ")
    j1_range = cal.get("j1_valid_range")
    if j1_range and not (j1_range[0] <= servo[0] <= j1_range[1]):
        return fail(f"J1={servo[0]:.0f}° ngoài vùng đã hiệu chuẩn ({j1_range[0]:.0f}-{j1_range[1]:.0f}°); "
                    "xoay về gần giữa bàn để quan sát")
    fx, fy, cx, cy = cal["K"]
    base_T_opt = M.fk_arm4_cal(servo, cal) @ np.asarray(cal["arm4_T_optical"], float)
    solved = solve_top_face(corners_px, base_T_opt, (fx, fy, cx, cy), float(cal.get("k1", 0.0)),
                            float(cal["tag_top_z"]))
    if not solved["ok"]:
        return {"ok": False, "reason": solved["reason"],
                **({"range_error_mm": round(solved["rms_m"] * 1000, 1)} if "rms_m" in solved else {})}
    R, top_axis = base_R_cube_from_face(axis, solved["yaw_rad"])
    hit = np.array([solved["x"], solved["y"], solved["top_z"]])
    yaw_deg = L.ready_view_yaw_deg(R, top_axis, hit, cal)
    pixel = L.project_base_point(hit, servo, cal)
    return {"ok": True, "reason": "", "source": "face_gravity", "layer": solved["layer"],
            "yaw_deg": yaw_deg, "base_yaw_rad": solved["yaw_rad"],
            "x": round(solved["x"], 5), "y": round(solved["y"], 5),
            "top_z": round(solved["top_z"], 5),
            "tcp_z": round(L.PICK_TCP_Z_TABLE + L.CUBE_EDGE_M * solved["layer"], 5),
            "range_error_mm": round(solved["rms_m"] * 1000, 1),
            "margin_mm": None if solved["margin_m"] is None else round(solved["margin_m"] * 1000, 1),
            "side_mm": round(solved["side_m"] * 1000, 1),
            "pixel": None if pixel is None else [round(pixel[0], 1), round(pixel[1], 1)]}


def wireframe_pixels(x, y, top_z, yaw_rad, servo5, cal, edge=L.CUBE_EDGE_M):
    """8 đỉnh cube (tâm x,y; mặt trên z=top_z; yaw) chiếu về pixel; dùng để vẽ skeleton hand-eye.

    Trả danh sách 8 (u, v) (None nếu đỉnh nằm sau camera), thứ tự: 4 đỉnh mặt trên rồi 4 đỉnh đáy.
    """
    h = edge / 2.0
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    out = []
    for z in (top_z, top_z - edge):
        for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h)):
            p = (x + c * dx - s * dy, y + s * dx + c * dy, z)
            out.append(L.project_base_point(p, servo5, cal))
    return out
