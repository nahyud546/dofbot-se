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
REGION_MARGIN_M = 0.015         # vật nằm ngay viền vùng (tâm lấn ra chừng này) vẫn tính là trong vùng
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
    faces: dict = field(default_factory=dict)       # {cube_id: như tags nhưng `corners` là MẶT TRÊN 30 mm; + label}
    objects: list = field(default_factory=list)     # vật không phải cube: [{label, polygon (N,2) đáy trên bàn, centre,
                                                    #   area_m2, n_views, height_m (None = chưa đo được), source, stamp}]
    zones: dict = field(default_factory=dict)       # {zone: {xy, source, stamp}}
    cameras: dict = field(default_factory=dict)     # {tên: {world_T_optical, K, k1, k2, image_size, rotate, stamp}}
    region: list = field(default_factory=list)      # [[x, y], ...] đa giác trên mặt bàn: vùng camera tay quét được
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

    def update_face(self, cube_id: int, fit: dict, label: str, confidence: float, source: str, sure: bool = True,
                    n_views: int = 1, stamp: float | None = None):
        """Cube nhận bằng một mặt KHÔNG có tag (mặt màu, hình in): `fit` từ `multiview.flat_face`.

        Lưu riêng khỏi `tags`: mặt này không làm mốc định vị cho camera khác, nhưng cube vẫn được vẽ và dùng tọa độ.
        Một cube chỉ ở một chỗ: ghi mặt thì bỏ mục tag cũ của cube đó (tag không còn ngửa lên).
        """
        self.tags.pop(int(cube_id), None)
        self.faces[int(cube_id)] = {
            "corners": np.asarray(fit["corners"], float).reshape(4, 3), "centre": np.asarray(fit["centre"], float),
            "normal": np.array([0.0, 0.0, 1.0]), "layer": int(fit["layer"]), "rms_px": float(fit["rms_px"]),
            "label": str(label), "confidence": float(confidence), "n_views": int(n_views), "source": source,
            "sure": bool(sure), "stamp": time.time() if stamp is None else float(stamp)}
        return self.faces[int(cube_id)]

    def set_objects(self, found, source: str, stamp: float | None = None) -> None:
        """Thay toàn bộ danh sách vật bất kỳ bằng kết quả `carve.footprints` (mỗi mục có thêm "label")."""
        stamp = time.time() if stamp is None else float(stamp)
        self.objects = [{"label": str(item.get("label", "vật")), "polygon": np.asarray(item["polygon"], float),
                         "centre": np.asarray(item["centre"], float), "area_m2": float(item["area_m2"]),
                         "n_views": int(item["n_views"]), "height_m": item.get("height_m"), "source": source,
                         "stamp": stamp} for item in found if self.inside(item["centre"])]

    def forget(self, tag_id: int) -> None:
        self.tags.pop(int(tag_id), None)
        self.faces.pop(int(tag_id), None)

    def set_zone(self, zone: int, xy, source: str) -> None:
        self.zones[int(zone)] = {"xy": [float(xy[0]), float(xy[1])], "source": source, "stamp": time.time()}

    def set_camera(self, name: str, world_T_optical, model: CameraModel) -> None:
        self.cameras[name] = {"world_T_optical": np.asarray(world_T_optical, float).reshape(4, 4),
                              "K": list(model.K), "k1": model.k1, "k2": model.k2,
                              "image_size": list(model.image_size), "rotate": model.rotate, "stamp": time.time()}

    def inside(self, xy, margin_m: float = REGION_MARGIN_M) -> bool:
        """Điểm (x, y) trên mặt bàn có nằm trong vùng world không (cho phép lấn ra ngoài viền `margin_m`).
        Chưa khai báo vùng thì coi như mọi chỗ đều trong."""
        if not self.region:
            return True
        import cv2
        poly = (np.asarray(self.region, float) * 1000.0).astype(np.float32).reshape(-1, 1, 2)
        signed_mm = cv2.pointPolygonTest(poly, (float(xy[0]) * 1000.0, float(xy[1]) * 1000.0), True)
        return signed_mm >= -margin_m * 1000.0

    # -------------------------------------------------------------- đọc
    def tag_corners(self, only_sure: bool = True) -> dict:
        return {i: t["corners"] for i, t in self.tags.items() if t["sure"] or not only_sure}

    def cube(self, tag_id: int):
        tag = self.tags.get(int(tag_id)) or self.faces.get(int(tag_id))
        return None if tag is None else cube_from_tag(tag["corners"])

    def cube_ids(self) -> list:
        """Mọi cube của world, dù biết qua tag hay qua mặt khác."""
        return sorted(set(self.tags) | set(self.faces))

    def entry(self, cube_id: int):
        """Mục của một cube: tag nếu có (chính xác hơn), không thì mặt."""
        return self.tags.get(int(cube_id)) or self.faces.get(int(cube_id))

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
               "cubes": {}, "tags": {}, "faces": {}, "zones": {}, "region": None, "objects": []}
        steps = np.arange(-extent_m, extent_m + 1e-9, grid_m)
        along = np.arange(0.0, 1.0 + 1e-9, 0.02 / max(extent_m, 0.02) / 2.0)   # lấy mẫu ~1 cm dọc từng đường

        def polyline(a, b):                                # (N,2) pixel dọc đoạn a-b; điểm không chiếu được là NaN
            a, b = np.asarray(a, float), np.asarray(b, float)
            points = a + along[:, None] * (b - a)
            pixels = px(points)
            if self.region:                                # có vùng world: lưới chỉ vẽ bên trong vùng
                pixels[[not self.inside(p[:2], 0.0) for p in points]] = np.nan
            return pixels

        if self.region:
            ring = np.asarray(self.region + self.region[:1], float)
            dense = np.concatenate([a + np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(b - a) / 0.01)))[:, None]
                                    * (b - a) for a, b in zip(ring[:-1], ring[1:])])
            out["region"] = px(np.c_[dense, np.full(len(dense), z)])

        for s in steps:                                    # lưới trên mặt bàn, phía làm việc (x âm)
            out["grid"].append(polyline([-extent_m, s, z], [0.0, s, z]))
            if s <= 0:
                out["grid"].append(polyline([s, -extent_m, z], [s, extent_m, z]))
        for tag_id, tag in self.tags.items():
            out["tags"][tag_id] = px(tag["corners"])
            vertices = px(cube_from_tag(tag["corners"])["vertices"])
            out["cubes"][tag_id] = [(vertices[a], vertices[b]) for a, b in _CUBE_EDGES]
        for cube_id, face in self.faces.items():
            if cube_id in self.tags:
                continue                                   # đã thấy lại tag của cube này: tag thắng
            out["faces"][cube_id] = px(face["corners"])
            vertices = px(cube_from_tag(face["corners"])["vertices"])
            out["cubes"][cube_id] = [(vertices[a], vertices[b]) for a, b in _CUBE_EDGES]
        for item in self.objects:
            ring = np.r_[item["polygon"], item["polygon"][:1]]
            out["objects"].append((item, px(np.c_[ring, np.full(len(ring), z)])))
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
            if isinstance(value, list):
                return [clean(v) for v in value]
            return value
        cubes = {}
        for tag_id in self.cube_ids():
            cube, entry = self.cube(tag_id), self.entry(tag_id)
            cubes[str(tag_id)] = {"centre": cube["centre"].tolist(), "R": cube["R"].tolist(), "edge_m": CUBE_EDGE_M,
                                  "seen_by": "tag" if tag_id in self.tags else "face",
                                  "top_face": entry.get("label", "tag"), "sure": bool(entry.get("sure", True))}
        return {"frame": self.frame, "created": self.created, "saved": time.time(), "table_z": self.table_z,
                "tags": clean(self.tags), "faces": clean(self.faces), "cubes": cubes, "zones": clean(self.zones), "cameras": clean(self.cameras),
                "region": [[float(x), float(y)] for x, y in self.region], "objects": clean(self.objects)}

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
        world.region = [[float(x), float(y)] for x, y in (data.get("region") or [])]
        world.objects = [{**item, "polygon": np.asarray(item["polygon"], float).reshape(-1, 2),
                          "centre": np.asarray(item["centre"], float)} for item in (data.get("objects") or [])]
        for key, face in (data.get("faces") or {}).items():
            world.faces[int(key)] = {**face, "corners": np.asarray(face["corners"], float).reshape(4, 3),
                                     "centre": np.asarray(face["centre"], float),
                                     "normal": np.asarray(face["normal"], float)}
        world.zones = {int(k): v for k, v in (data.get("zones") or {}).items()}
        for name, cam in (data.get("cameras") or {}).items():
            world.cameras[name] = {**cam, "world_T_optical": np.asarray(cam["world_T_optical"], float).reshape(4, 4)}
        return world
