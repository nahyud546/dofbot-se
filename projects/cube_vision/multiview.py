"""Pose 6D thật của một tag từ NHIỀU góc nhìn (một camera di chuyển, hoặc nhiều camera khác nhau).

Một khung hình chỉ cho hướng; với tag 20 mm, PnP một khung sai khoảng cách cỡ cm và có thể lật pháp tuyến
(hai nghiệm gần như khớp ngang nhau). Ở đây pose camera của từng góc nhìn đã biết trong cùng một hệ (FK + hand-eye
cho camera tay, hiệu chuẩn cho camera ngoài), nên chỉ còn 6 ẩn số: pose của tag trong hệ đó. Tối ưu sai số chiếu lại
trên mọi góc nhìn cùng lúc; thị sai giữa các góc nhìn cho ra z thật, không cần giả thiết "tag nằm trên mặt phẳng".
Thuần toán, không ROS/T8.

Hai cách gộp, chọn theo NGUỒN SAI SỐ chính:
  solve_tag     tối ưu chung sai số chiếu lại (tam giác hóa). Đúng khi pose các camera chính xác so với thị sai:
                hai camera cố định cách xa nhau, hoặc camera tay có hand-eye đạt 3D.
  robust_tag    mỗi góc nhìn tự giải PnP (độ sâu từ kích thước tag) rồi lấy trung vị trong world, loại góc lệch.
                Đúng khi pose camera sai cỡ mm mà thị sai chỉ vài cm: lúc đó tam giác hóa khuếch đại sai số pose
                thành sai số độ sâu (đo thật 2026-10-08: tam giác hóa cho z lệch tới +8 mm, trung vị PnP ±2–4 mm).
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


def _average_rotation(rotations) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.mean(rotations, axis=0))
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return R


def robust_tag(views, size_m: float = TAG_SIZE_M, outlier_m: float = 0.006):
    """Pose của tag = trung vị các ước lượng PnP từng góc nhìn (đã đưa về hệ a); góc nhìn lệch hẳn bị loại.

    Trả dict như `solve_tag`, thêm: spread_m (3,) độ lệch lớn nhất của các góc nhìn còn lại quanh trung vị theo
    từng trục, dropped [nhãn góc nhìn bị loại], estimates (N,3) tâm tag theo từng góc nhìn còn lại.
    `pos_std_m` là độ tản mạn giữa các góc nhìn (1,4826·MAD), KHÔNG chia cho căn số góc nhìn: sai số ở đây chủ yếu
    là sai số hệ thống của pose camera, không trung bình đi được.
    """
    views = list(views)
    pts = tag_points(size_m)
    cands = []
    for view in views:
        found = [(err, np.asarray(view.a_T_optical, float) @ T) for err, T in pnp_candidates(view, size_m)]
        if found:
            cands.append((view, found))
    if not cands:
        return None
    # Hướng mặt tag: lấy nghiệm tốt nhất của từng khung, rồi cho mỗi khung chọn lại nghiệm gần hướng chung nhất.
    normal = np.median([found[0][1][:3, 2] for _, found in cands], axis=0)
    normal = normal / np.linalg.norm(normal)
    chosen = [max(found, key=lambda c: float(c[1][:3, 2] @ normal) - 0.02 * c[0]) for _, found in cands]
    poses = [T for _, T in chosen]
    centres = np.array([T[:3, 3] for T in poses])
    keep = np.ones(len(poses), bool)
    if len(poses) >= 3:
        dev = np.linalg.norm(centres - np.median(centres, axis=0), axis=1)
        keep = dev <= max(outlier_m, 3.0 * 1.4826 * float(np.median(dev)))
        if keep.sum() < 2:
            keep[:] = True
    kept_views = [cands[i][0] for i in range(len(cands)) if keep[i]]
    inl = centres[keep]
    centre = np.median(inl, axis=0)
    a_T_tag = np.eye(4)
    a_T_tag[:3, :3] = _average_rotation([poses[i][:3, :3] for i in range(len(poses)) if keep[i]])
    a_T_tag[:3, 3] = centre
    res = _residuals(_params(a_T_tag), kept_views, pts)
    per_view = np.sqrt(np.mean(res.reshape(len(kept_views), 4, 2) ** 2, axis=(1, 2)) * 2.0)
    spread = np.max(np.abs(inl - centre), axis=0) if len(inl) > 1 else np.full(3, np.inf)
    std = 1.4826 * np.median(np.abs(inl - centre), axis=0) if len(inl) > 1 else np.full(3, np.inf)
    ratios = [found[1][0] / max(found[0][0], 0.05) if len(found) > 1 else np.inf
              for (_, found), k in zip(cands, keep) if k]
    return {"a_T_tag": a_T_tag, "corners": pts @ a_T_tag[:3, :3].T + centre, "centre": centre,
            "normal": a_T_tag[:3, 2].copy(), "rms_px": float(np.sqrt(np.mean(res ** 2))),
            "view_rms_px": [float(v) for v in per_view], "pos_std_m": [float(v) for v in np.maximum(std, 3e-4)],
            "spread_m": [float(v) for v in spread], "n_views": int(keep.sum()), "baseline_m": baseline_m(kept_views),
            "flip_margin": float(min(ratios)), "estimates": inl,
            "dropped": [cands[i][0].label for i in range(len(cands)) if not keep[i]], "views": kept_views}


def flat_tag(view: View, z: float, size_m: float = TAG_SIZE_M):
    """Tag nằm ngửa (mặt tag hướng lên, ngang) ở độ cao z đã biết: giải 3 ẩn x, y, yaw từ 4 góc của MỘT khung.

    Chắc hơn giải tự do 6 ẩn rất nhiều khi cube nằm trên bàn: không còn nghiệm lật, không còn nghiêng giả, và độ sâu
    đến từ độ cao đã biết thay vì từ kích thước tag. Trả dict {a_T_tag, corners, centre, normal, rms_px} hoặc None
    khi tia không tới mặt phẳng z. rms_px lớn nghĩa là mô hình "nằm phẳng ở z" không hợp (nghiêng, dựng đứng, sai tầng).
    """
    from scipy.optimize import least_squares
    T = np.asarray(view.a_T_optical, float)
    corners = np.asarray(view.corners_px, float).reshape(4, 2)
    hits = []
    for u, v in corners:
        ray = T[:3, :3] @ view.camera.ray(u, v)
        if abs(ray[2]) < 1e-9 or (z - T[2, 3]) / ray[2] <= 0:
            return None
        hits.append(T[:3, 3] + ray * ((z - T[2, 3]) / ray[2]))
    hits = np.array(hits)
    edge = hits[1] - hits[0]
    x0 = np.array([hits[:, 0].mean(), hits[:, 1].mean(), np.arctan2(edge[1], edge[0])])
    pts = tag_points(size_m)
    opt_T_a = invert(T)

    def pose(params):
        c, s = np.cos(params[2]), np.sin(params[2])
        a_T_tag = np.eye(4)
        a_T_tag[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        a_T_tag[:3, 3] = [params[0], params[1], z]
        return a_T_tag

    def residual(params):
        world = pts @ pose(params)[:3, :3].T + pose(params)[:3, 3]
        proj = view.camera.project(world @ opt_T_a[:3, :3].T + opt_T_a[:3, 3])
        return np.nan_to_num(proj - corners, nan=1e3).ravel()

    sol = least_squares(residual, x0, loss="huber", f_scale=3.0)
    a_T_tag = pose(sol.x)
    res = residual(sol.x)
    return {"a_T_tag": a_T_tag, "corners": pts @ a_T_tag[:3, :3].T + a_T_tag[:3, 3], "centre": a_T_tag[:3, 3].copy(),
            "normal": np.array([0.0, 0.0, 1.0]), "rms_px": float(np.sqrt(np.mean(res ** 2))), "z": float(z)}


FACE_SIZE_M = 0.030


def flat_face(view: View, layer_zs, size_m: float = FACE_SIZE_M):
    """Mặt trên của cube (vuông `size_m`, nằm ngang) thấy bằng 4 góc KHÔNG có thứ tự chuẩn: tìm tầng và x, y, yaw.

    Khác tag, 4 góc của một mặt màu/hình in đến từ dò đường viền nên không biết góc nào là góc đầu, chiều nào là
    chiều quay: thử cả 8 thứ tự. Độ cao không biết trước: thử từng độ cao trong `layer_zs` (mặt trên của tầng 0, 1,
    ...). Kích thước mặt đã biết nên chỉ MỘT tầng khớp: sai tầng thì mặt chiếu ra to/nhỏ hơn 4 góc thấy được.
    Trả nghiệm tốt nhất của `flat_tag` kèm "layer" và "runner_up_px" (rms của tầng tốt nhì; inf khi chỉ có một tầng),
    hoặc None khi không tầng nào giải được.
    """
    quad = np.asarray(view.corners_px, float).reshape(4, 2)
    per_layer = []
    for layer, z in enumerate(layer_zs):
        best = None
        for order in (quad, quad[::-1]):
            for shift in range(4):
                fit = flat_tag(View(view.a_T_optical, view.camera, np.roll(order, shift, axis=0), view.label),
                               float(z), size_m)
                if fit is not None and (best is None or fit["rms_px"] < best["rms_px"]):
                    best = fit
        if best is not None:
            best["layer"] = layer
            per_layer.append(best)
    if not per_layer:
        return None
    per_layer.sort(key=lambda fit: fit["rms_px"])
    best = per_layer[0]
    best["runner_up_px"] = per_layer[1]["rms_px"] if len(per_layer) > 1 else float("inf")
    return best
