"""World sống cho một camera đang di chuyển: camera dời và cube dời đều được theo kịp ngay trong từng khung.

World lưu trong file là ảnh chụp lúc camera tay quét (chính xác, nhưng đứng yên). Ở đây, với mỗi khung của một camera
khác (iPhone):
  1. các tag mà world cho là ĐỨNG YÊN làm mốc để định vị camera (`camera_pose.track`);
  2. tag nào lệch khỏi phép dời của camera (cube bị dời) hoặc chưa có trong world (cube mới) được đo lại từ chính khung
     này: PnP một góc nhìn với pose camera vừa giải, rồi lấy độ cao đã biết của tầng nếu tag nằm ngửa;
  3. tag của world lẽ ra phải thấy trong ảnh mà không thấy (cube bị lấy đi, hoặc bị che) được báo `missing`.
Kết quả là một `WorldMap` tạm; file world gốc không bị đổi. Số đo của cube dời kém chính xác hơn số đo quét bằng
camera tay (một góc nhìn, camera ở xa), nên được đánh dấu `sure=False` và nguồn `phone-live`.
Thuần toán; không ROS/T8.
"""
from __future__ import annotations

import copy
import time

import numpy as np

from . import camera_pose as P
from . import multiview as MV
from .frames import CameraModel
from .world_map import CUBE_EDGE_M, WorldMap, snap_to_layer

LIVE_SOURCE = "phone-live"
GLOBAL_RETRY_S = 1.5            # mất dấu thì tìm lại toàn cục (~1 s) tối đa chừng này giây một lần, tránh giật hình
MISSING_MARGIN_PX = 40.0
LIVE_MAX_DZ_M = 0.015           # một góc nhìn đo sâu kém hơn camera tay: cho phép lệch tầng nhiều hơn khi bám tầng


def locate_tag(corners_px, model: CameraModel, world_T_optical, tag_top_z: float):
    """Pose một tag từ MỘT khung (camera đã biết pose): dict kiểu `multiview.solve_tag`, hoặc None."""
    view = MV.View(np.asarray(world_T_optical, float), model, np.asarray(corners_px, float).reshape(4, 2))
    candidates = MV.pnp_candidates(view)
    if not candidates:
        return None
    best = candidates[0][0]
    # Hai nghiệm PnP của một tag phẳng gần như ngang nhau khi nhìn xiên: chọn nghiệm có mặt tag hướng lên nếu
    # sai số chiếu lại không tệ hơn nhiều (cube nằm trên bàn); không thì giữ nghiệm khớp nhất.
    pick = max((c for c in candidates if c[0] <= 2.0 * best + 0.5),
               key=lambda c: float((view.a_T_optical @ c[1])[2, 2]))
    a_T_tag = view.a_T_optical @ pick[1]
    corners = MV.tag_points() @ a_T_tag[:3, :3].T + a_T_tag[:3, 3]
    fused = {"corners": corners, "centre": a_T_tag[:3, 3].copy(), "normal": a_T_tag[:3, 2].copy(),
             "pos_std_m": [0.005] * 3, "n_views": 1, "rms_px": float(pick[0])}
    fused, layer = snap_to_layer(fused, tag_top_z, max_dz=LIVE_MAX_DZ_M)
    fused["layer"] = layer
    return fused


class LiveWorld:
    def __init__(self, world: WorldMap, model: CameraModel, name: str = "phone", tag_top_z: float | None = None):
        self.world, self.model, self.name = world, model, name
        if tag_top_z is None:
            tag_top_z = (world.table_z if world.table_z is not None else 0.0278) + CUBE_EDGE_M
        self.tag_top_z = float(tag_top_z)
        cam = world.cameras.get(name)
        self.pose = None if cam is None else np.asarray(cam["world_T_optical"], float)   # khởi tạo cho khung đầu
        self._next_global = 0.0

    def update(self, seen: dict) -> dict:
        """seen {id: (4,2)} từ một khung (đã xoay đúng hướng). Trả dict: ok, reasons, pose, world (WorldMap sống),
        anchors, moved, new, missing."""
        info = {"ok": False, "reasons": [], "pose": None, "world": self.world, "anchors": [], "moved": [],
                "new": [], "missing": [], "rms_px": None}
        now = time.monotonic()
        allow_global = self.pose is None or now >= self._next_global
        res = P.track(self.world.tag_corners(only_sure=True), seen, self.model, prev=self.pose,
                      allow_global=allow_global)
        info["reasons"] = res["reasons"]
        if not res["ok"]:
            if allow_global and len(res["tags"]) >= P.MIN_TAGS:
                self._next_global = now + GLOBAL_RETRY_S        # giữ pose cuối làm khởi tạo; chưa tìm lại ngay
            return info
        self.pose = res["world_T_optical"]
        live = WorldMap(table_z=self.world.table_z, frame=self.world.frame, created=self.world.created)
        live.tags = copy.deepcopy(self.world.tags)
        live.zones = copy.deepcopy(self.world.zones)
        live.cameras = copy.deepcopy(self.world.cameras)
        for tag_id, corners in seen.items():
            if tag_id in res["anchors"]:
                continue
            known = tag_id in self.world.tags
            if known and tag_id not in res["moved"]:
                continue                                        # lệch ít: coi như đứng yên, giữ số đo của camera tay
            fused = locate_tag(corners, self.model, self.pose, self.tag_top_z)
            if fused is None:
                continue
            live.forget(tag_id)
            live.update_tag(tag_id, fused, LIVE_SOURCE, sure=False)
            info["moved" if known else "new"].append(tag_id)
        w, h = self.model.image_size
        for tag_id, tag in self.world.tags.items():
            if tag_id in seen or tag_id in info["moved"]:
                continue
            uv = self.model.project(((np.linalg.inv(self.pose) @ np.r_[tag["centre"], 1.0])[:3]).reshape(1, 3))[0]
            if (np.isfinite(uv).all() and MISSING_MARGIN_PX <= uv[0] <= w - MISSING_MARGIN_PX
                    and MISSING_MARGIN_PX <= uv[1] <= h - MISSING_MARGIN_PX):
                info["missing"].append(tag_id)
        live.set_camera(self.name, self.pose, self.model)
        info.update(ok=True, pose=self.pose, world=live, anchors=list(res["anchors"]), rms_px=res["rms_px"])
        return info
