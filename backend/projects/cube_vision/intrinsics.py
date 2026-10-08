"""Thông số nội (intrinsic) của từng camera từ bảng ChArUco, và file lưu chung của mỗi camera.

Vì sao tách riêng: trước đây tiêu cự, tâm ảnh và méo được fit CHUNG với pose camera từ vài tag 20 mm; hai nhóm
tham số bù trừ nhau (tâm ảnh camera ngoài nhảy 100 px giữa hai lần giải). Bảng ChArUco cho vài trăm góc trải khắp
ảnh ở nhiều góc nghiêng, đủ để chốt ống kính một lần; sau đó extrinsic chỉ còn 6 tham số pose.

Mỗi camera một file `config/robot/cameras/<tên>.json`:
    K [fx, fy, cx, cy], k1, k2, image_size [rộng, cao] (SAU khi xoay), rotate, source, intrinsics_accepted,
    và khi đã hiệu chuẩn vị trí: base_T_optical (16 số), pose_accepted.
Thuần toán + OpenCV; không ROS, không T8.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .frames import CameraModel
from .registry import repo_root

MAX_RMS_PX = 1.0
MAX_HOLDOUT_PX = 1.5
MIN_VIEWS = 12
MIN_CELLS = 7                  # trong lưới 3x3 của ảnh phải có góc bảng ở ít nhất chừng này ô
MIN_CORNERS = 8                # số góc ChArUco tối thiểu để nhận một khung
MIN_SHARPNESS = 60.0           # phương sai Laplacian; thấp hơn là khung mờ do rung tay
MIN_TILT_SPREAD_DEG = 15.0


def camera_dir() -> Path:
    return repo_root() / "config" / "robot" / "cameras"


def camera_path(name: str) -> Path:
    return camera_dir() / f"{name}.json"


def load_camera(name: str, path=None):
    """dict của file camera, hoặc None khi thiếu/hỏng."""
    try:
        data = json.loads(Path(path or camera_path(name)).read_text())
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def save_camera(name: str, updates: dict, path=None) -> Path:
    """Gộp `updates` vào file camera (giữ các khóa khác, ví dụ pose khi chỉ cập nhật intrinsic)."""
    path = Path(path or camera_path(name))
    data = load_camera(name, path) or {}
    data.update(updates)
    data["name"] = name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1, ensure_ascii=False) + "\n")
    return path


def model_from(data, name="camera"):
    """CameraModel từ file camera đã đạt intrinsic, hoặc None."""
    if not data or data.get("intrinsics_accepted") is not True:
        return None
    try:
        return CameraModel(name, tuple(float(v) for v in data["K"]), tuple(int(v) for v in data["image_size"]),
                           float(data.get("k1", 0.0)), float(data.get("k2", 0.0)), int(data.get("rotate", 0)),
                           f"config/robot/cameras/{name}.json")
    except (KeyError, TypeError, ValueError):
        return None


# ------------------------------------------------------------------ bảng
@dataclass(frozen=True)
class Board:
    """Bảng ChArUco: `cols` x `rows` ô vuông cạnh `square_m`, marker cạnh `marker_m`, từ điển 5x5."""
    cols: int = 8
    rows: int = 5
    square_m: float = 0.035
    marker_ratio: float = 0.72

    @property
    def marker_m(self) -> float:
        return self.square_m * self.marker_ratio

    def corner_points(self) -> np.ndarray:
        """(N,3) góc trong của bàn cờ theo id góc, lấy từ chính OpenCV (gốc tọa độ bảng khác nhau giữa 4.5 và 4.7+)."""
        board = self.cv()
        pts = board.chessboardCorners if hasattr(board, "chessboardCorners") else board.getChessboardCorners()
        return np.asarray(pts, float).reshape(-1, 3)

    def cv(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        if hasattr(cv2.aruco, "CharucoBoard_create"):                      # OpenCV 4.5 (python hệ thống)
            return cv2.aruco.CharucoBoard_create(self.cols, self.rows, self.square_m, self.marker_m, dictionary)
        return cv2.aruco.CharucoBoard((self.cols, self.rows), self.square_m, self.marker_m, dictionary)

    def image(self, px_per_square: int, margin_px: int = 0) -> np.ndarray:
        size = (self.cols * px_per_square, self.rows * px_per_square)
        board = self.cv()
        img = board.draw(size) if hasattr(board, "draw") else board.generateImage(size)
        return cv2.copyMakeBorder(img, margin_px, margin_px, margin_px, margin_px, cv2.BORDER_CONSTANT, value=255)


def detect(gray, board: Board):
    """(ids (N,), góc ảnh (N,2)) của các góc ChArUco thấy được; (rỗng, rỗng) khi không thấy bảng."""
    empty = (np.zeros(0, int), np.zeros((0, 2)))
    cvb = board.cv()
    if hasattr(cv2.aruco, "CharucoDetector"):
        corners, ids, _, _ = cv2.aruco.CharucoDetector(cvb).detectBoard(gray)
    else:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        if marker_ids is None or len(marker_ids) < 2:
            return empty
        _, corners, ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, cvb)
    if ids is None or len(ids) < 4:
        return empty
    return np.asarray(ids, int).ravel(), np.asarray(corners, float).reshape(-1, 2)


def sharpness(gray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ------------------------------------------------------------------ chọn khung và giải
def cells(image_points, size, grid=3) -> set:
    """Các ô (cột, hàng) của lưới grid x grid trên ảnh có ít nhất một điểm."""
    pts = np.asarray(image_points, float).reshape(-1, 2)
    w, h = size
    return {(min(grid - 1, int(u * grid / w)), min(grid - 1, int(v * grid / h))) for u, v in pts}


def is_new_view(image_points, kept, size, min_shift=0.06) -> bool:
    """Khung này có khác đáng kể các khung đã giữ không (tâm bảng dời hoặc bảng đổi cỡ/nghiêng)."""
    pts = np.asarray(image_points, float).reshape(-1, 2)
    centre, spread = pts.mean(axis=0), pts.std(axis=0)
    diag = float(np.hypot(*size))
    for other in kept:
        o = np.asarray(other, float).reshape(-1, 2)
        if (np.linalg.norm(centre - o.mean(axis=0)) < min_shift * diag
                and np.linalg.norm(spread - o.std(axis=0)) < 0.25 * min_shift * diag):
            return False
    return True


def _tilts_deg(rvecs) -> np.ndarray:
    """Góc giữa pháp tuyến bảng và trục nhìn của camera ở từng khung."""
    out = []
    for rvec in rvecs:
        normal = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0][:, 2]
        out.append(np.degrees(np.arccos(min(1.0, abs(float(normal[2]))))))
    return np.array(out)


def solve(views, board: Board, image_size, holdout_every=4):
    """views: [(ids, góc ảnh (N,2))] của từng khung. Trả dict K, k1, k2, sai số, accepted, reasons.

    Mô hình: fx, fy, cx, cy, k1, k2 (không méo tiếp tuyến, không k3): đủ cho webcam/điện thoại và khớp
    `CameraModel`.
    """
    size = (int(image_size[0]), int(image_size[1]))
    world = board.corner_points()
    obj, img = [], []
    for ids, pts in views:
        ids = np.asarray(ids, int).ravel()
        if len(ids) >= MIN_CORNERS:
            obj.append(world[ids].astype(np.float32))
            img.append(np.asarray(pts, np.float32).reshape(-1, 2))
    if len(obj) < 4:
        raise ValueError(f"chỉ có {len(obj)} khung dùng được (cần ≥ {MIN_VIEWS})")
    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3

    def fit(indices):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera([obj[i] for i in indices], [img[i] for i in indices],
                                                         size, None, None, flags=flags)
        return float(rms), K, dist.ravel(), rvecs

    held = list(range(holdout_every - 1, len(obj), holdout_every)) if len(obj) >= 2 * holdout_every else []
    holdout_px = []
    if held:
        _, K, dist, _ = fit([i for i in range(len(obj)) if i not in held])
        for i in held:
            ok, rvec, tvec = cv2.solvePnP(obj[i], img[i], K, dist)
            if ok:
                proj = cv2.projectPoints(obj[i], rvec, tvec, K, dist)[0].reshape(-1, 2)
                holdout_px.append(float(np.sqrt(np.mean(np.sum((proj - img[i]) ** 2, axis=1)))))
    rms, K, dist, rvecs = fit(range(len(obj)))
    covered = set().union(*(cells(p, size) for p in img))
    tilts = _tilts_deg(rvecs)
    reasons = []
    if len(obj) < MIN_VIEWS:
        reasons.append(f"chỉ có {len(obj)} khung (cần ≥ {MIN_VIEWS})")
    if len(covered) < MIN_CELLS:
        reasons.append(f"bảng mới phủ {len(covered)}/9 vùng ảnh (cần ≥ {MIN_CELLS}): đưa bảng ra các góc ảnh")
    if float(tilts.max() - tilts.min()) < MIN_TILT_SPREAD_DEG:
        reasons.append(f"các khung nghiêng gần như nhau ({tilts.min():.0f}–{tilts.max():.0f}°): "
                       "nghiêng camera/bảng nhiều hướng hơn")
    if rms > MAX_RMS_PX:
        reasons.append(f"RMS chiếu lại {rms:.2f} px (cần ≤ {MAX_RMS_PX:g})")
    if not holdout_px:
        reasons.append("không có khung kiểm định")
    elif float(np.median(holdout_px)) > MAX_HOLDOUT_PX:
        reasons.append(f"khung kiểm định lệch trung vị {np.median(holdout_px):.2f} px (cần ≤ {MAX_HOLDOUT_PX:g})")
    w, h = size
    if not (0.3 * w < K[0, 0] < 4.0 * w and abs(K[0, 0] / K[1, 1] - 1.0) < 0.15
            and 0.2 * w < K[0, 2] < 0.8 * w and 0.2 * h < K[1, 2] < 0.8 * h):
        reasons.append("nghiệm phi vật lý (tiêu cự/tâm ảnh)")
    return {"K": [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],
            "k1": float(dist[0]), "k2": float(dist[1]), "image_size": list(size),
            "intrinsics_rms_px": rms, "intrinsics_holdout_px": holdout_px, "intrinsics_views": len(obj),
            "intrinsics_cells": len(covered), "intrinsics_tilt_deg": [float(tilts.min()), float(tilts.max())],
            "intrinsics_accepted": not reasons, "intrinsics_reasons": reasons,
            "intrinsics_created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "board": {"cols": board.cols, "rows": board.rows, "square_m": board.square_m}}
