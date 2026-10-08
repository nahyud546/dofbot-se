"""Một camera đang ở đâu trong world, từ các tag mà world đã biết tọa độ.

Dùng cho cả hai trường hợp:
  - camera cố định (webcam, iPhone trên giá): giải một lần, lưu lại, sau đó chỉ kiểm tra có bị xê dịch không;
  - camera cầm tay: giải lại ở MỖI khung hình; không đủ tag thì trả None chứ không đoán.
Intrinsic phải có trước (`cube_vision.intrinsics`), ở đây chỉ còn 6 ẩn số pose. Thuần toán + OpenCV.
"""
from __future__ import annotations

import cv2
import numpy as np

from .frames import CameraModel, invert

MIN_TAGS = 2                   # một tag 20 mm cho pose rất yếu (xoay bù trừ với dời); cần ≥ 2 tag tách nhau
MIN_SPREAD_M = 0.04            # các tag phải tách nhau ít nhất chừng này trong world
# Ngưỡng theo số đo thật 2026-10-08 (iPhone nhìn xiên sát bàn, world từ camera tay có z lệch ±5 mm): pose đúng cho
# RMS ~4 px (≈ 2 mm ở 0,45 m), nghiệm lật cho ~15 px. Kiểm chéo bỏ-một-tag chỉ có nghĩa khi còn lại ≥ 3 tag.
MAX_RMS_PX = 6.0
MAX_LEAVE_ONE_OUT_PX = 12.0
MIN_TAGS_FOR_LEAVE_ONE_OUT = 4


def _to_T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def _starts(points):
    """Các pose khởi tạo: camera đứng quanh đám điểm (nhiều hướng, nhiều độ cao, hai khoảng cách), nhìn vào tâm."""
    centre = points.mean(axis=0)
    out = []
    for dist in (0.25, 0.5, 0.9):
        for elev in np.radians([8.0, 25.0, 50.0, 80.0]):
            for azim in np.radians(np.arange(0.0, 360.0, 30.0)):
                eye = centre + dist * np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
                z = (centre - eye) / dist
                x = np.cross(z, [0.0, 0.0, 1.0])
                x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
                R = np.stack([x, np.cross(z, x), z], axis=0)             # optical_R_world, ảnh "đứng thẳng"
                for roll in (0.0, np.pi / 2, np.pi, -np.pi / 2):         # camera có thể bị xoay quanh trục nhìn
                    c, s = np.cos(roll), np.sin(roll)
                    Rr = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R
                    out.append((cv2.Rodrigues(Rr)[0], (-Rr @ eye).reshape(3, 1)))
    return out


def _pnp(points, pixels, camera: CameraModel, min_height=None):
    """optical_T_world từ điểm world ↔ pixel (≥ 4 điểm), hoặc None.

    Các tag trên bàn gần như đồng phẳng: PnP khi đó có HAI nghiệm (nghiệm thật và nghiệm lật qua mặt phẳng), nhìn
    càng xiên càng khó phân biệt. Vì vậy tối ưu từ nhiều pose khởi tạo, bỏ nghiệm đặt camera dưới `min_height`
    (dưới mặt bàn), rồi lấy nghiệm chiếu lại khớp nhất.
    """
    pts, px = np.asarray(points, np.float64), np.asarray(pixels, np.float64)
    K, dist = camera.matrix(), camera.dist()
    best = None
    for rvec0, tvec0 in _starts(pts):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(pts, px, K, dist, rvec0.copy(), tvec0.copy())
        except cv2.error:
            continue
        T = _to_T(rvec, tvec)
        if not np.isfinite(T).all() or (pts @ T[:3, :3].T + T[:3, 3])[:, 2].min() <= 0.02:
            continue
        height = float(invert(T)[2, 3])
        if min_height is not None and height < min_height:
            continue
        err = float(np.sqrt(np.mean(_errors(pts, px, camera, T) ** 2)))
        if np.isfinite(err) and (best is None or err < best[0]):
            best = (err, T)
    return None if best is None else best[1]


def _errors(points, pixels, camera, optical_T_world) -> np.ndarray:
    cam = np.asarray(points, float) @ optical_T_world[:3, :3].T + optical_T_world[:3, 3]
    return np.linalg.norm(camera.project(cam) - np.asarray(pixels, float), axis=1)


def pose_from_tags(world_corners: dict, detections: dict, camera: CameraModel,
                   min_tags: int = MIN_TAGS, above_table: bool = True) -> dict:
    """world_corners {id: (4,3)} đã biết trong world; detections {id: (4,2)} pixel (ảnh đã xoay đúng hướng).

    Trả dict: ok, reasons, world_T_optical (khi giải được), rms_px, tags (id đã dùng), leave_one_out_px {id: px},
    spread_m. `ok` False nghĩa là KHÔNG nên dùng pose này. above_table: camera phải nằm cao hơn các tag trừ 3 cm
    (đúng với mọi camera nhìn xuống bàn) để loại nghiệm lật của các tag đồng phẳng.
    """
    ids = sorted(set(world_corners) & set(detections))
    out = {"ok": False, "reasons": [], "tags": ids, "world_T_optical": None, "rms_px": None,
           "leave_one_out_px": {}, "spread_m": 0.0}
    if len(ids) < min_tags:
        out["reasons"].append(f"chỉ thấy {len(ids)} tag đã biết trong world (cần ≥ {min_tags})")
        return out
    pts = {i: np.asarray(world_corners[i], float).reshape(4, 3) for i in ids}
    px = {i: np.asarray(detections[i], float).reshape(4, 2) for i in ids}
    centres = np.array([pts[i].mean(axis=0) for i in ids])
    out["spread_m"] = float(max(np.linalg.norm(a - b) for a in centres for b in centres)) if len(ids) > 1 else 0.0
    if len(ids) > 1 and out["spread_m"] < MIN_SPREAD_M:
        out["reasons"].append(f"các tag quá gần nhau ({out['spread_m'] * 1000:.0f} mm)")
    floor = float(min(p[:, 2].min() for p in pts.values())) - 0.03 if above_table else None
    T = _pnp(np.vstack([pts[i] for i in ids]), np.vstack([px[i] for i in ids]), camera, floor)
    if T is None:
        out["reasons"].append("PnP không giải được")
        return out
    err = _errors(np.vstack([pts[i] for i in ids]), np.vstack([px[i] for i in ids]), camera, T)
    out["rms_px"] = float(np.sqrt(np.mean(err ** 2)))
    out["world_T_optical"] = invert(T)
    if out["rms_px"] > MAX_RMS_PX:
        out["reasons"].append(f"RMS chiếu lại {out['rms_px']:.1f} px (cần ≤ {MAX_RMS_PX:g})")
    if len(ids) >= 3:
        for held in ids:
            rest = [i for i in ids if i != held]
            T_rest = _pnp(np.vstack([pts[i] for i in rest]), np.vstack([px[i] for i in rest]), camera, floor)
            if T_rest is not None:
                out["leave_one_out_px"][held] = float(np.mean(_errors(pts[held], px[held], camera, T_rest)))
        worst = max(out["leave_one_out_px"].values(), default=0.0)
        if len(ids) >= MIN_TAGS_FOR_LEAVE_ONE_OUT and worst > MAX_LEAVE_ONE_OUT_PX:
            out["reasons"].append(f"bỏ một tag thì đoán lại nó lệch {worst:.1f} px (cần ≤ {MAX_LEAVE_ONE_OUT_PX:g}): "
                                  "tọa độ world của tag hoặc intrinsic chưa khớp")
    out["ok"] = not out["reasons"]
    return out


def drift_px(world_corners: dict, detections: dict, camera: CameraModel, world_T_optical) -> float | None:
    """Camera cố định còn ở chỗ cũ không: độ lệch trung vị (px) giữa tag chiếu từ world và tag đang thấy."""
    ids = sorted(set(world_corners) & set(detections))
    if not ids:
        return None
    T = invert(world_T_optical)
    err = np.concatenate([_errors(np.asarray(world_corners[i], float).reshape(4, 3),
                                  np.asarray(detections[i], float).reshape(4, 2), camera, T) for i in ids])
    return float(np.median(err))
