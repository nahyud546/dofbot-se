"""Sổ đăng ký hệ tọa độ: mọi phép biến đổi được khai báo MỘT lần, có tên, có nguồn gốc.

Quy ước dùng trong cả repo:
  - `a_T_b` là ma trận 4x4 đưa điểm viết trong hệ b sang hệ a:  p_a = a_T_b @ p_b.
    Ghép chuỗi đọc từ trái sang phải:  a_T_c = a_T_b @ b_T_c.
  - Đơn vị mét, radian. Hệ quang học (`*_optical`): z ra trước ống kính, x sang phải ảnh, y xuống dưới ảnh.

Module này thuần toán, không biết robot nào: robot cụ thể khai báo cạnh của nó (xem
`vision_experiments/dofbot_frames.py`). Cạnh có ba loại:
  hằng       ma trận cố định (quy ước, CAD, hoặc kết quả hiệu chuẩn);
  theo khớp  hàm của góc khớp q (động học thuận);
  còn thiếu  đã khai báo nhưng chưa hiệu chuẩn: tra cứu đi qua nó báo lỗi kèm lệnh cần chạy.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

import numpy as np


class MissingTransform(LookupError):
    """Chuỗi cần một cạnh chưa có số (chưa hiệu chuẩn) hoặc hai hệ không nối với nhau."""


def invert(T) -> np.ndarray:
    """Nghịch đảo ma trận cứng 4x4 (không dùng nghịch đảo tổng quát để khỏi tích sai số)."""
    T = np.asarray(T, float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def is_rigid(T, tol: float = 1e-6) -> bool:
    T = np.asarray(T, float)
    return (T.shape == (4, 4) and np.isfinite(T).all() and np.allclose(T[3], [0, 0, 0, 1], atol=tol)
            and np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=max(tol, 1e-3))
            and np.linalg.det(T[:3, :3]) > 0)


@dataclass
class Edge:
    parent: str
    child: str
    matrix: np.ndarray | None = None                 # parent_T_child khi là hằng
    fn: Callable | None = None                       # q -> parent_T_child khi phụ thuộc khớp
    kind: str = "hằng"                               # quy ước | CAD | FK | hiệu chuẩn | ...
    source: str = ""                                 # file/khóa hoặc hàm chứa số
    how: str = ""                                    # lệnh tạo lại
    note: str = ""

    @property
    def name(self) -> str:
        return f"{self.parent}_T_{self.child}"

    @property
    def state(self) -> str:
        return "theo khớp" if self.fn is not None else ("hằng" if self.matrix is not None else "CÒN THIẾU")

    def value(self, q=None) -> np.ndarray:
        if self.fn is not None:
            if q is None:
                raise MissingTransform(f"{self.name} phụ thuộc góc khớp: cần truyền q")
            return np.asarray(self.fn(q), float)
        if self.matrix is None:
            raise MissingTransform(f"{self.name} chưa có số" + (f" — chạy: {self.how}" if self.how else ""))
        return self.matrix


@dataclass
class FrameGraph:
    """Cây/đồ thị hệ tọa độ. `lookup(a, b, q)` trả a_T_b bằng cách ghép các cạnh trên đường nối a với b."""
    edges: list = field(default_factory=list)
    notes: dict = field(default_factory=dict)        # {tên hệ: mô tả}

    def frame(self, name: str, note: str) -> None:
        self.notes[name] = note

    def add(self, parent, child, matrix=None, fn=None, **meta) -> Edge:
        if any({e.parent, e.child} == {parent, child} for e in self.edges):
            raise ValueError(f"cạnh {parent}–{child} đã được khai báo")
        if matrix is not None:
            matrix = np.asarray(matrix, float).reshape(4, 4)
            if not is_rigid(matrix):
                raise ValueError(f"{parent}_T_{child} không phải phép biến đổi cứng")
        edge = Edge(parent, child, matrix, fn, **meta)
        self.edges.append(edge)
        for name in (parent, child):
            self.notes.setdefault(name, "")
        return edge

    def frames(self) -> list:
        return list(self.notes)

    def chain(self, a: str, b: str) -> list:
        """[(cạnh, đảo?)] đi từ a tới b. Rỗng khi a == b."""
        for name in (a, b):
            if name not in self.notes:
                raise MissingTransform(f"không có hệ tọa độ '{name}' (đang có: {', '.join(self.notes)})")
        prev, queue = {a: None}, deque([a])
        while queue:
            here = queue.popleft()
            if here == b:
                break
            for edge in self.edges:
                for nxt, inverted in ((edge.child, False), (edge.parent, True)):
                    other = edge.parent if not inverted else edge.child
                    if other == here and nxt not in prev:
                        prev[nxt] = (here, edge, inverted)
                        queue.append(nxt)
        if b not in prev:
            raise MissingTransform(f"'{a}' và '{b}' không nối với nhau")
        path, here = [], b
        while prev[here] is not None:
            here, edge, inverted = prev[here]
            path.append((edge, inverted))
        return path[::-1]

    def lookup(self, a: str, b: str, q=None) -> np.ndarray:
        """a_T_b: điểm viết trong hệ b -> hệ a."""
        T = np.eye(4)
        for edge, inverted in self.chain(a, b):
            step = edge.value(q)
            T = T @ (invert(step) if inverted else step)
        return T

    def available(self, a: str, b: str) -> bool:
        try:
            return all(e.fn is not None or e.matrix is not None for e, _ in self.chain(a, b))
        except MissingTransform:
            return False

    def describe(self, a: str, b: str) -> str:
        """Chuỗi dạng chữ, ví dụ 'world_T_base_link · base_link_T_arm1(q) · inv(...)'."""
        parts = []
        for edge, inverted in self.chain(a, b):
            text = edge.name + ("(q)" if edge.fn is not None else "")
            parts.append(f"inv({text})" if inverted else text)
        return " · ".join(parts) or "I"


# ------------------------------------------------------------------ camera
@dataclass
class CameraModel:
    """Thông số nội của một camera: pinhole + méo xuyên tâm k1, k2 (cùng mô hình OpenCV, p1 = p2 = 0).

    `rotate` là số độ (0/90/180/270, theo chiều kim đồng hồ) phải xoay khung thô của luồng để ra đúng ảnh
    mà K mô tả; mọi pixel ở đây là pixel SAU khi xoay.
    """
    name: str
    K: tuple                       # (fx, fy, cx, cy)
    image_size: tuple              # (rộng, cao) sau khi xoay
    k1: float = 0.0
    k2: float = 0.0
    rotate: int = 0
    source: str = ""

    def matrix(self) -> np.ndarray:
        fx, fy, cx, cy = self.K
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    def dist(self) -> np.ndarray:
        return np.array([self.k1, self.k2, 0.0, 0.0, 0.0])

    def _distort(self, x, y):
        r2 = x * x + y * y
        d = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
        return x * d, y * d

    def project(self, points_optical) -> np.ndarray:
        """(N,3) điểm trong hệ optical -> (N,2) pixel; NaN cho điểm nằm sau camera."""
        pts = np.asarray(points_optical, float).reshape(-1, 3)
        z = np.where(pts[:, 2] > 1e-9, pts[:, 2], np.nan)
        x, y = self._distort(pts[:, 0] / z, pts[:, 1] / z)
        fx, fy, cx, cy = self.K
        return np.stack([fx * x + cx, fy * y + cy], axis=1)

    def ray(self, u: float, v: float) -> np.ndarray:
        """Hướng đơn vị (trong hệ optical) của tia qua pixel (u, v)."""
        fx, fy, cx, cy = self.K
        xd, yd = (float(u) - cx) / fx, (float(v) - cy) / fy
        x, y = xd, yd
        for _ in range(12):
            r2 = x * x + y * y
            d = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
            x, y = xd / d, yd / d
        ray = np.array([x, y, 1.0])
        return ray / np.linalg.norm(ray)

    def orient(self, frame):
        """Xoay khung thô của luồng về hướng mà K mô tả."""
        turns = (int(self.rotate) // 90) % 4
        return frame if turns == 0 else np.ascontiguousarray(np.rot90(frame, k=-turns))


def pixel_to_plane(camera: CameraModel, a_T_optical, u: float, v: float, z: float):
    """Điểm (trong hệ a) nơi tia qua pixel cắt mặt phẳng ngang z của hệ a; None nếu tia không tới mặt đó.

    Đây là cách một camera đơn ra được x, y, z: nó chỉ cho HƯỚNG, độ sâu đến từ mặt phẳng đã biết.
    """
    T = np.asarray(a_T_optical, float)
    ray = T[:3, :3] @ camera.ray(u, v)
    if abs(ray[2]) < 1e-9:
        return None
    t = (float(z) - T[2, 3]) / ray[2]
    return None if t <= 0 else T[:3, 3] + ray * t


def rpy_deg(T) -> tuple:
    """(roll, pitch, yaw) độ của phần xoay, quy ước R = Rz(yaw) Ry(pitch) Rx(roll) như URDF/TF."""
    R = np.asarray(T, float)[:3, :3]
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return (math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(pitch),
            math.degrees(math.atan2(R[1, 0], R[0, 0])))
