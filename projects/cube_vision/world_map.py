"""World map: mọi thứ đã đo được, viết trong MỘT hệ `world`, để camera nào cũng dùng lại được.

Nội dung: tag (4 góc 3D + pháp tuyến + độ bất định + nguồn + thời điểm), cube suy ra từ tag, ô màu, mặt bàn, và
pose/intrinsic của các camera cố định. File JSON tự chứa (`data/world/latest.json`): chương trình khác (RViz, điện
thoại) chỉ cần đọc file này, không cần import repo. Thuần dữ liệu; không ROS, không T8.

Giới hạn về thời gian: world là ảnh chụp của một cảnh TĨNH. Mỗi mục có `stamp`; `stale()` cho biết mục nào quá cũ.
Sau khi tay gắp/thả, mục của cube đó phải được đo lại hoặc xóa (`forget`).
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .frames import CameraModel, invert
from .registry import repo_root

CUBE_EDGE_M = 0.030
TAG_SIZE_M = 0.020
_CUBE_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def default_path() -> Path:
    return repo_root() / "data" / "world" / "latest.json"


def cube_from_tag(corners, edge: float = CUBE_EDGE_M) -> dict:
    """Cube (tâm, 3 trục, 8 đỉnh) từ 4 góc của tag in giữa một mặt: tâm cube lùi nửa cạnh theo pháp tuyến tag."""
    corners = np.asarray(corners, float).reshape(4, 3)
    centre = corners.mean(axis=0)
    x = corners[1] - corners[0]
    x = x / np.linalg.norm(x)
    y = corners[3] - corners[0]
    y = y - x * float(y @ x)
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    h = edge / 2.0
    cube_centre = centre - z * h
    vertices = np.array([cube_centre + sx * h * x + sy * h * y + sz * h * z
                         for sz in (-1, 1) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    R = np.stack([x, y, z], axis=1)
    return {"centre": cube_centre, "R": R, "vertices": vertices}


def snap_to_layer(fused: dict, tag_top_z: float, edge: float = CUBE_EDGE_M, max_layer: int = 3,
                  max_tilt_deg: float = 12.0, max_dz: float = 0.010):
    """Tag nằm ngửa trên bàn/trên cube khác: thay độ cao đo được bằng độ cao ĐÃ BIẾT của tầng gần nhất.

    Một camera đơn đo ngang tốt hơn đo sâu nhiều (đo thật: ngang ±1–2 mm, cao ±5 mm). Khi mặt tag gần nằm ngang và
    độ cao đo được cách một tầng (tag_top_z + n·30 mm) không quá `max_dz`, giữ x, y và hướng xoay quanh trục đứng,
    đặt z đúng tầng và làm phẳng mặt tag. Trả (fused mới, tầng) hoặc (fused cũ, None) khi không áp dụng được
    (tag nghiêng/dựng đứng, hoặc lơ lửng giữa hai tầng: khi đó giữ số đo 3D thật).
    """
    normal = np.asarray(fused["normal"], float)
    if normal[2] < np.cos(np.radians(max_tilt_deg)):
        return fused, None
    centre = np.asarray(fused["centre"], float)
    layer = int(round((centre[2] - tag_top_z) / edge))
    z = tag_top_z + edge * layer
    if not 0 <= layer <= max_layer or abs(centre[2] - z) > max_dz:
        return fused, None
    corners = np.asarray(fused["corners"], float).reshape(4, 3)
    x = corners[1] - corners[0]
    yaw = np.arctan2(x[1], x[0])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    half = float(np.linalg.norm(x)) / 2.0
    local = np.array([[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]])
    new_centre = np.array([centre[0], centre[1], z])
    out = dict(fused)
    out.update(corners=local @ R.T + new_centre, centre=new_centre, normal=np.array([0.0, 0.0, 1.0]),
               dz_m=float(centre[2] - z))
    return out, layer


@dataclass
class WorldMap:
    tags: dict = field(default_factory=dict)        # {id: {corners, centre, normal, std_m, n_views, source, stamp, sure}}
    zones: dict = field(default_factory=dict)       # {zone: {xy, source, stamp}}
    cameras: dict = field(default_factory=dict)     # {tên: {world_T_optical, K, k1, k2, image_size, rotate, stamp}}
    table_z: float | None = None
    frame: str = "world"
    created: float = field(default_factory=time.time)

    # -------------------------------------------------------------- ghi
    def update_tag(self, tag_id: int, fused: dict, source: str, sure: bool = True, stamp: float | None = None):
        """Ghi kết quả `multiview.solve_tag`. Không ghi đè một mục chắc chắn bằng một mục kém hơn còn mới."""
        entry = {"corners": np.asarray(fused["corners"], float).reshape(4, 3),
                 "centre": np.asarray(fused["centre"], float), "normal": np.asarray(fused["normal"], float),
                 "std_m": float(np.max(np.nan_to_num(fused["pos_std_m"], posinf=0.02))), "n_views": int(fused["n_views"]),
                 "rms_px": float(fused["rms_px"]), "source": source, "sure": bool(sure),
                 "layer": fused.get("layer"),
                 "stamp": time.time() if stamp is None else float(stamp)}
        old = self.tags.get(int(tag_id))
        if old and old["sure"] and not entry["sure"] and entry["stamp"] - old["stamp"] < 5.0:
            return old
        self.tags[int(tag_id)] = entry
        return entry

    def forget(self, tag_id: int) -> None:
        self.tags.pop(int(tag_id), None)

    def set_zone(self, zone: int, xy, source: str) -> None:
        self.zones[int(zone)] = {"xy": [float(xy[0]), float(xy[1])], "source": source, "stamp": time.time()}

    def set_camera(self, name: str, world_T_optical, model: CameraModel) -> None:
        self.cameras[name] = {"world_T_optical": np.asarray(world_T_optical, float).reshape(4, 4),
                              "K": list(model.K), "k1": model.k1, "k2": model.k2,
                              "image_size": list(model.image_size), "rotate": model.rotate, "stamp": time.time()}

    # -------------------------------------------------------------- đọc
    def tag_corners(self, only_sure: bool = True) -> dict:
        return {i: t["corners"] for i, t in self.tags.items() if t["sure"] or not only_sure}

    def cube(self, tag_id: int):
        tag = self.tags.get(int(tag_id))
        return None if tag is None else cube_from_tag(tag["corners"])

    def camera_model(self, name: str):
        cam = self.cameras.get(name)
        return None if cam is None else CameraModel(name, tuple(cam["K"]), tuple(cam["image_size"]),
                                                    cam["k1"], cam["k2"], cam["rotate"])

    def stale(self, max_age_s: float, now: float | None = None) -> list:
        now = time.time() if now is None else now
        return sorted(i for i, t in self.tags.items() if now - t["stamp"] > max_age_s)

    def project_into(self, model: CameraModel, world_T_optical, grid_m: float = 0.05, extent_m: float = 0.30) -> dict:
        """World -> pixel của một camera: thứ để vẽ đè lên ảnh. Điểm sau camera là NaN.

        Trả {"axes": [(gốc, đầu trục)x3], "grid": [(N,2) đường gấp khúc], "cubes": {id: [12 cạnh]}, "tags": {id: (4,2)},
             "zones": {zone: (u, v)}}.
        """
        opt_T_world = invert(world_T_optical)

        def px(points):
            pts = np.asarray(points, float).reshape(-1, 3)
            return model.project(pts @ opt_T_world[:3, :3].T + opt_T_world[:3, 3])

        z = self.table_z or 0.0
        origin = np.array([0.0, 0.0, z])
        out = {"axes": [tuple(px([origin, origin + 0.05 * np.eye(3)[k]])) for k in range(3)], "grid": [],
               "cubes": {}, "tags": {}, "zones": {}}
        steps = np.arange(-extent_m, extent_m + 1e-9, grid_m)
        along = np.arange(0.0, 1.0 + 1e-9, 0.02 / max(extent_m, 0.02) / 2.0)   # lấy mẫu ~1 cm dọc từng đường

        def polyline(a, b):                                # (N,2) pixel dọc đoạn a-b; điểm không chiếu được là NaN
            a, b = np.asarray(a, float), np.asarray(b, float)
            return px(a + along[:, None] * (b - a))

        for s in steps:                                    # lưới trên mặt bàn, phía làm việc (x âm)
            out["grid"].append(polyline([-extent_m, s, z], [0.0, s, z]))
            if s <= 0:
                out["grid"].append(polyline([s, -extent_m, z], [s, extent_m, z]))
        for tag_id, tag in self.tags.items():
            out["tags"][tag_id] = px(tag["corners"])
            vertices = px(cube_from_tag(tag["corners"])["vertices"])
            out["cubes"][tag_id] = [(vertices[a], vertices[b]) for a, b in _CUBE_EDGES]
        for zone, item in self.zones.items():
            out["zones"][zone] = px([[item["xy"][0], item["xy"][1], z]])[0]
        return out

    # -------------------------------------------------------------- file
    def to_dict(self) -> dict:
        def clean(value):
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, dict):
                return {str(k): clean(v) for k, v in value.items()}
            return value
        cubes = {}
        for tag_id in self.tags:
            cube = self.cube(tag_id)
            cubes[str(tag_id)] = {"centre": cube["centre"].tolist(), "R": cube["R"].tolist(), "edge_m": CUBE_EDGE_M}
        return {"frame": self.frame, "created": self.created, "saved": time.time(), "table_z": self.table_z,
                "tags": clean(self.tags), "cubes": cubes, "zones": clean(self.zones), "cameras": clean(self.cameras)}

    def save(self, path=None) -> Path:
        path = Path(path or default_path())
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.to_dict(), indent=1, ensure_ascii=False) + "\n")
        tmp.replace(path)                                  # thay nguyên tử: bên đọc không bao giờ thấy file dở
        return path

    @classmethod
    def load(cls, path=None):
        """World đã lưu, hoặc world rỗng khi chưa có file / file hỏng."""
        try:
            data = json.loads(Path(path or default_path()).read_text())
        except (OSError, ValueError):
            return cls()
        world = cls(table_z=data.get("table_z"), frame=data.get("frame", "world"),
                    created=float(data.get("created", time.time())))
        for key, tag in (data.get("tags") or {}).items():
            world.tags[int(key)] = {**tag, "corners": np.asarray(tag["corners"], float).reshape(4, 3),
                                    "centre": np.asarray(tag["centre"], float),
                                    "normal": np.asarray(tag["normal"], float)}
        world.zones = {int(k): v for k, v in (data.get("zones") or {}).items()}
        for name, cam in (data.get("cameras") or {}).items():
            world.cameras[name] = {**cam, "world_T_optical": np.asarray(cam["world_T_optical"], float).reshape(4, 4)}
        return world
