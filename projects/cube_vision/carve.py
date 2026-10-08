"""Vật bất kỳ trên mặt bàn từ NHIỀU góc nhìn có mặt nạ vật: vết đáy trên mặt bàn (giao các "bóng" của vật).

Mỗi góc nhìn cho một mặt nạ "pixel thuộc vật". Một ô trên mặt bàn thuộc đáy vật khi mọi góc nhìn thấy ô đó đều
thấy vật ở đó (ít nhất `min_views` góc). Đây là lát sát mặt bàn của khối bao thị giác (visual hull).

Giới hạn đã đo (2026-10-08, cốc Ø ~8 cm, 3 góc nhìn từ cùng một phía, cốc bị cắt ở mép ảnh): vết đáy ra to hơn thật
(77 cm² so với ~50 cm²) vì phía sau vật không góc nào nhìn tới, và CHIỀU CAO không xác định được (khối bao không
khép lại phía trên). Vì vậy kết quả ở đây là "vùng vật chiếm, ước lượng", không phải hình dạng chính xác.
Thuần toán: không ROS, không mô hình; mặt nạ do bên gọi cấp.
"""
from __future__ import annotations

import cv2
import numpy as np

from .frames import CameraModel, invert

CELL_M = 0.004
LIFT_M = 0.002                # lát cắt ngay trên mặt bàn
MIN_VIEWS = 2
MIN_AGREE = 0.6               # tỉ lệ góc nhìn (trong số thấy ô đó) phải thấy vật: một góc hụt nhận diện không xóa vật
MIN_AREA_M2 = 4e-4            # 4 cm²: nhỏ hơn thì coi là nhiễu


def footprints(views, table_z: float, bounds, cell: float = CELL_M, min_views: int = MIN_VIEWS,
               keep_out=(), keep_out_m: float = 0.03) -> list:
    """views: [(CameraModel, a_T_optical, mask HxW uint8)]; bounds ((x0, x1), (y0, y1)) trong hệ a.

    keep_out: các điểm (x, y) của vật đã biết (cube): ô cách chúng dưới `keep_out_m` bị loại.
    Trả [{"polygon": (N,2), "centre": (2,), "area_m2", "n_views", "cell"}], vật lớn trước.
    """
    xs = np.arange(bounds[0][0], bounds[0][1], cell)
    ys = np.arange(bounds[1][0], bounds[1][1], cell)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    points = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, float(table_z) + LIFT_M)]
    seen, hit = np.zeros(len(points), int), np.zeros(len(points), int)
    for camera, a_T_optical, mask in views:
        T = invert(np.asarray(a_T_optical, float))
        cam = points @ T[:3, :3].T + T[:3, 3]
        uv = camera.project(cam)
        h, w = mask.shape[:2]
        ok = (np.isfinite(uv).all(axis=1) & (cam[:, 2] > 0.03) & (uv[:, 0] >= 0) & (uv[:, 0] < w)
              & (uv[:, 1] >= 0) & (uv[:, 1] < h))
        px = np.clip(np.nan_to_num(uv).astype(int), 0, [w - 1, h - 1])
        seen += ok
        hit += ok & (mask[px[:, 1], px[:, 0]] > 0)
    keep = (hit >= min_views) & (hit >= MIN_AGREE * seen)
    for x, y in keep_out:
        keep &= np.hypot(points[:, 0] - x, points[:, 1] - y) > keep_out_m
    grid = keep.reshape(gx.shape).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(grid)
    found = []
    for k in range(1, count):
        area = float(stats[k, cv2.CC_STAT_AREA]) * cell * cell
        if area < MIN_AREA_M2:
            continue
        cells = np.argwhere(labels == k)                              # (hàng = chỉ số x, cột = chỉ số y)
        xy = np.c_[xs[cells[:, 0]], ys[cells[:, 1]]]
        hull = cv2.convexHull((xy * 1000.0).astype(np.float32)).reshape(-1, 2) / 1000.0
        found.append({"polygon": hull, "centre": xy.mean(axis=0), "area_m2": area,
                      "n_views": int(hit.reshape(gx.shape)[labels == k].max()), "cell": cell})
    return sorted(found, key=lambda item: -item["area_m2"])
