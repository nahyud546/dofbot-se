"""Pose 6D thật của một tag từ NHIỀU góc nhìn (một camera di chuyển, hoặc nhiều camera khác nhau).

Một khung hình chỉ cho hướng; với tag 20 mm, PnP một khung sai khoảng cách cỡ cm và có thể lật pháp tuyến
(hai nghiệm gần như khớp ngang nhau). Ở đây pose camera của từng góc nhìn đã biết trong cùng một hệ (FK + hand-eye
cho camera tay, hiệu chuẩn cho camera ngoài), nên chỉ còn 6 ẩn số: pose của tag trong hệ đó. Tối ưu sai số chiếu lại
trên mọi góc nhìn cùng lúc; thị sai giữa các góc nhìn cho ra z thật, không cần giả thiết "tag nằm trên mặt phẳng".
Thuần toán, không ROS/T8.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .frames import CameraModel, invert

TAG_SIZE_M = 0.020


@dataclass
class View:
    """Một lần quan sát: camera nào, đang ở đâu (a_T_optical), thấy 4 góc tag ở pixel nào."""
    a_T_optical: np.ndarray
    camera: CameraModel
    corners_px: np.ndarray            # (4,2), thứ tự của detector
    label: str = ""

    def centre(self) -> np.ndarray:
        return np.asarray(self.a_T_optical, float)[:3, 3]


def tag_points(size_m: float = TAG_SIZE_M) -> np.ndarray:
    """4 góc tag trong hệ tag (z ra khỏi mặt tag), cùng thứ tự detector."""
    s = float(size_m) / 2.0
    return np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]])


def _T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def _params(T) -> np.ndarray:
    T = np.asarray(T, float)
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def pnp_candidates(view: View, size_m: float = TAG_SIZE_M) -> list:
    """[(sai số px, optical_T_tag)] cả hai nghiệm IPPE của một khung, tốt nhất trước."""
    pts = tag_points(size_m).astype(np.float32)
    img = np.asarray(view.corners_px, np.float32).reshape(4, 2)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(pts, img, view.camera.matrix(), view.camera.dist(),
                                              flags=cv2.SOLVEPNP_IPPE)
    out = []
    for rvec, tvec in zip(rvecs, tvecs) if ok else ():
        if float(np.ravel(tvec)[2]) <= 0:
            continue
        T = _T(rvec, tvec)
        err = view.camera.project(pts @ T[:3, :3].T + T[:3, 3]) - img
        out.append((float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))), T))
    return sorted(out, key=lambda c: c[0])


def _residuals(params, views, pts):
    a_T_tag = _T(params[:3], params[3:])
    world = pts @ a_T_tag[:3, :3].T + a_T_tag[:3, 3]
    res = []
    for view in views:
        opt_T_a = invert(view.a_T_optical)
        proj = view.camera.project(world @ opt_T_a[:3, :3].T + opt_T_a[:3, 3])
        res.append(np.nan_to_num(proj - np.asarray(view.corners_px, float).reshape(4, 2), nan=1e3).ravel())
    return np.concatenate(res)


def baseline_m(views) -> float:
    """Khoảng cách lớn nhất giữa hai tâm camera: thước đo thị sai."""
    centres = np.array([v.centre() for v in views])
    return 0.0 if len(centres) < 2 else float(max(np.linalg.norm(a - b) for a in centres for b in centres))


def solve_tag(views, size_m: float = TAG_SIZE_M):
    """Pose của tag trong hệ a từ các `View`. Trả dict hoặc None khi không khung nào giải được.

    a_T_tag, corners (4,3), centre (3,), normal (3,), rms_px, view_rms_px [..], pos_std_m (3,), n_views,
    baseline_m, flip_margin (tỉ số chi phí nghiệm lật / nghiệm chọn; gần 1 = còn mơ hồ).
    """
    from scipy.optimize import least_squares
    views = list(views)
    pts = tag_points(size_m)
    starts = []
    for view in views:
        for _, opt_T_tag in pnp_candidates(view, size_m):
            starts.append(_params(np.asarray(view.a_T_optical, float) @ opt_T_tag))
    if not starts:
        return None
    solutions = []
    for x0 in starts:
        sol = least_squares(_residuals, x0, loss="huber", f_scale=2.0, args=(views, pts))
        res = _residuals(sol.x, views, pts)
        cost = float(np.sqrt(np.mean(res ** 2)))
        normal = _T(sol.x[:3], sol.x[3:])[:3, 2]
        if not any(np.linalg.norm(sol.x[3:] - s[1][3:]) < 5e-4 and float(normal @ s[2]) > 0.99 for s in solutions):
            solutions.append((cost, sol.x, normal, sol.jac, res))
    solutions.sort(key=lambda s: s[0])
    cost, x, normal, jac, res = solutions[0]
    flip = min((s[0] for s in solutions[1:] if float(normal @ s[2]) < 0.97), default=np.inf)
    a_T_tag = _T(x[:3], x[3:])
    per_view = np.sqrt(np.mean(res.reshape(len(views), 4, 2) ** 2, axis=(1, 2)) * 2.0)
    sigma = max(cost, 0.5)                                 # px; không tin nhiễu dưới nửa pixel
    try:
        cov = np.linalg.inv(jac.T @ jac) * sigma ** 2
        pos_std = np.sqrt(np.clip(np.diag(cov)[3:], 0.0, None))
    except np.linalg.LinAlgError:
        pos_std = np.full(3, np.inf)
    return {"a_T_tag": a_T_tag, "corners": pts @ a_T_tag[:3, :3].T + a_T_tag[:3, 3],
            "centre": a_T_tag[:3, 3].copy(), "normal": a_T_tag[:3, 2].copy(),
            "rms_px": cost, "view_rms_px": [float(v) for v in per_view],
            "pos_std_m": [float(v) for v in pos_std], "n_views": len(views),
            "baseline_m": baseline_m(views), "flip_margin": float(flip / max(cost, 1e-6))}
