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
MAX_RMS_PX = 3.0
MAX_LEAVE_ONE_OUT_PX = 6.0     # bỏ một tag, giải bằng các tag còn lại, đoán lại tag đó
RANSAC_PX = 8.0


def _to_T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def _pnp(points, pixels, camera: CameraModel):
    """optical_T_world từ điểm world ↔ pixel (≥ 4 điểm), hoặc None."""
    pts, px = np.asarray(points, np.float64), np.asarray(pixels, np.float64)
    K, dist = camera.matrix(), camera.dist()
    ok, rvec, tvec, _ = cv2.solvePnPRansac(pts, px, K, dist, reprojectionError=RANSAC_PX, iterationsCount=200,
                                           flags=cv2.SOLVEPNP_EPNP if len(pts) >= 6 else cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(pts, px, K, dist)
        if not ok:
            return None
    rvec, tvec = cv2.solvePnPRefineLM(pts, px, K, dist, rvec, tvec)
    T = _to_T(rvec, tvec)
    return T if (pts @ T[:3, :3].T + T[:3, 3])[:, 2].min() > 0 else None


def _errors(points, pixels, camera, optical_T_world) -> np.ndarray:
    cam = np.asarray(points, float) @ optical_T_world[:3, :3].T + optical_T_world[:3, 3]
    return np.linalg.norm(camera.project(cam) - np.asarray(pixels, float), axis=1)


def pose_from_tags(world_corners: dict, detections: dict, camera: CameraModel,
                   min_tags: int = MIN_TAGS) -> dict:
    """world_corners {id: (4,3)} đã biết trong world; detections {id: (4,2)} pixel (ảnh đã xoay đúng hướng).

    Trả dict: ok, reasons, world_T_optical (khi giải được), rms_px, tags (id đã dùng), leave_one_out_px {id: px},
    spread_m. `ok` False nghĩa là KHÔNG nên dùng pose này.
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
    T = _pnp(np.vstack([pts[i] for i in ids]), np.vstack([px[i] for i in ids]), camera)
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
            T_rest = _pnp(np.vstack([pts[i] for i in rest]), np.vstack([px[i] for i in rest]), camera)
            if T_rest is not None:
                out["leave_one_out_px"][held] = float(np.mean(_errors(pts[held], px[held], camera, T_rest)))
        worst = max(out["leave_one_out_px"].values(), default=0.0)
        if worst > MAX_LEAVE_ONE_OUT_PX:
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
