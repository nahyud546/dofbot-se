#!/usr/bin/env python3
"""Hình học cảnh trong hệ base: ô zone ở đâu, cube nằm trong ô nào.

Nền cho việc kiểm tra "cube đã vào đúng zone chưa" và cứu cube sai zone. Quan sát có
thể đến từ camera tay (qua hand-eye) hoặc, sau này, từ camera ngoài: cả hai chỉ cần
cung cấp polygon ô và vị trí cube trong hệ base, phần còn lại dùng chung ở đây.

Chưa cài đặt: vòng cứu cube hoàn chỉnh (recover_cube) và backend camera ngoài.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import cv2
import numpy as np

IN_ZONE, WRONG_ZONE, OUT_OF_ZONE, UNKNOWN = "in_zone", "wrong_zone", "out_of_zone", "unknown"
DEFAULT_MIN_MARGIN_MM = 8.0       # cube coi là "vào ô" khi tâm cách mép ô >= ngần này


def pad_polygon_base(mask, servo5, cal, table_z, max_points: int = 24):
    """Đa giác (N,2) của phần ô nhìn thấy, trong hệ base (mặt phẳng bàn).

    Cạnh trùng khung ảnh chỉ là cạnh cắt của ô, không phải mép thật: dùng cho kiểm tra
    "điểm có nằm trong ô" (không dùng để tính biên gần khung ảnh).
    """
    import cube_search_center_math as M
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.01 * peri, True).reshape(-1, 2)
    if len(approx) > max_points:
        approx = approx[np.linspace(0, len(approx) - 1, max_points).astype(int)]
    M.apply_calibration(cal)
    try:
        pts = []
        for u, v in approx:
            point, _ = M.pixel_to_base(float(u), float(v), list(servo5), z_table=table_z)
            pts.append([float(point[0]), float(point[1])])
    finally:
        M.apply_calibration(None)
    return np.asarray(pts, float)


def signed_distance_mm(xy, polygon) -> float:
    """Khoảng cách có dấu (mm) tới đa giác lồi: dương = nằm trong."""
    contour = np.asarray(polygon, np.float32).reshape(-1, 1, 2)
    return float(cv2.pointPolygonTest(contour, (float(xy[0]), float(xy[1])), True)) * 1000.0


@dataclass
class ZoneCheck:
    cube_id: int
    expected_zone: int
    status: str
    zone_found: int | None
    margin_mm: float | None
    source: str = ""


def classify(cube_id: int, cube_xy, polygons: dict, expected_zone: int | None = None,
             min_margin_mm: float = DEFAULT_MIN_MARGIN_MM, source: str = "") -> ZoneCheck:
    """Cube (XY base) đang ở ô nào so với ô mong đợi. polygons = {zone_id: đa giác base}."""
    expected = int(cube_id if expected_zone is None else expected_zone)
    if cube_xy is None or not polygons:
        return ZoneCheck(int(cube_id), expected, UNKNOWN, None, None, source)
    margins = {int(z): signed_distance_mm(cube_xy, poly) for z, poly in polygons.items()
               if poly is not None and len(poly) >= 3}
    if not margins:
        return ZoneCheck(int(cube_id), expected, UNKNOWN, None, None, source)
    inside = {z: m for z, m in margins.items() if m >= min_margin_mm}
    if inside:
        zone, margin = max(inside.items(), key=lambda kv: kv[1])
        status = IN_ZONE if zone == expected else WRONG_ZONE
        return ZoneCheck(int(cube_id), expected, status, zone, margin, source)
    if expected in margins:
        return ZoneCheck(int(cube_id), expected, OUT_OF_ZONE, None, margins[expected], source)
    return ZoneCheck(int(cube_id), expected, UNKNOWN, None, None, source)


class ZoneVerifier(Protocol):
    """Nguồn quan sát cho việc kiểm tra zone; camera tay và camera ngoài cùng giao diện."""

    def polygons(self) -> dict:
        """{zone_id: đa giác base} của các ô hiện nhìn thấy."""

    def cube_xy(self, cube_id: int):
        """XY base của cube, hoặc None nếu không thấy."""


def verify(cube_id: int, verifier: ZoneVerifier, expected_zone: int | None = None) -> ZoneCheck:
    return classify(cube_id, verifier.cube_xy(int(cube_id)), verifier.polygons(), expected_zone,
                    source=type(verifier).__name__)


class ExternalCameraVerifier:
    """Chừa chỗ: camera ngoài + homography (config/camera/validate_zones.json) đổi pixel sang base."""

    def polygons(self):
        raise NotImplementedError("camera ngoài chưa có phần cứng/hiệu chuẩn")

    def cube_xy(self, cube_id):
        raise NotImplementedError("camera ngoài chưa có phần cứng/hiệu chuẩn")


def recover_cube(cube_id: int, check: ZoneCheck):
    """Sườn vòng cứu: định vị cube -> gắp ở pose bất kỳ -> sort lại với zone survey.

    Chưa cài đặt; mô tả các bước để nối vào T8 sau khi hand-eye ở J1 xa được xác nhận.
    """
    if check.status in (IN_ZONE, UNKNOWN):
        return {"action": "none", "reason": check.status}
    raise NotImplementedError("recover_cube: cần locate (pose_library.choose_view) + pick + sort")
