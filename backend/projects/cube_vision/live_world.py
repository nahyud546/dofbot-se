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
FACE_KEEP_S = 3.0               # cube thấy bằng mặt màu được giữ chừng này giây sau lần thấy cuối
FACE_MAX_RMS_PX = 8.0
FACE_MAX_RMS_SHARE = 0.07
FACE_LAYER_RATIO = 2.0
GLOBAL_RETRY_S = 1.5            # mất dấu thì tìm lại toàn cục (~1 s) tối đa chừng này giây một lần, tránh giật hình
MISSING_MARGIN_PX = 40.0
LIVE_MAX_DZ_M = 0.015           # một góc nhìn đo sâu kém hơn camera tay: cho phép lệch tầng nhiều hơn khi bám tầng
FLAT_ACCEPT_PX = 3.0            # mô hình "nằm phẳng ở một tầng" khớp 4 góc tới mức này thì tin cube nằm phẳng
FLAT_TIE_PX = 0.6               # hai tầng khớp ngang nhau trong mức này: chọn tầng gần độ cao đo tự do


def _flat_candidates(view, tag_top_z, max_layer=3):
    out = []
    for layer in range(max_layer + 1):
        fit = MV.flat_tag(view, tag_top_z + CUBE_EDGE_M * layer)
        if fit is not None:
            fit["layer"] = layer
            out.append(fit)
    return out


def locate_tag(corners_px, model: CameraModel, world_T_optical, tag_top_z: float):
    """Pose một tag từ MỘT khung (camera đã biết pose): dict kiểu `multiview.solve_tag`, hoặc None.

    Cube lẽ ra nằm phẳng trên bàn hoặc trên cube khác, nên thử trước mô hình "nằm phẳng ở tầng n" (3 ẩn, không có
    nghiệm lật): nếu một tầng khớp 4 góc trong FLAT_ACCEPT_PX thì dùng nó. Chỉ khi không tầng nào khớp (cube đang
    cầm, nghiêng, dựng đứng) mới dùng pose tự do 6 ẩn, và pose đó có thể kém chắc (xem `flat` trong kết quả).
    """
    view = MV.View(np.asarray(world_T_optical, float), model, np.asarray(corners_px, float).reshape(4, 2))
    cam_pos = view.a_T_optical[:3, 3]

    def faces_camera(opt_T_tag):                     # mặt tag chỉ thấy được khi nó quay về phía camera
        a_T = view.a_T_optical @ opt_T_tag
        return float(a_T[:3, 2] @ (cam_pos - a_T[:3, 3])) > 0.0

    candidates = [c for c in MV.pnp_candidates(view) if faces_camera(c[1])]
    free = None
    if candidates:
        best = candidates[0][0]
        # Hai nghiệm PnP của một tag phẳng gần như ngang nhau khi nhìn xiên: chọn nghiệm có mặt tag hướng lên nếu
        # sai số chiếu lại không tệ hơn nhiều; không thì giữ nghiệm khớp nhất.
        pick = max((c for c in candidates if c[0] <= 2.0 * best + 0.5),
                   key=lambda c: float((view.a_T_optical @ c[1])[2, 2]))
        a_T_tag = view.a_T_optical @ pick[1]
        free = {"corners": MV.tag_points() @ a_T_tag[:3, :3].T + a_T_tag[:3, 3], "centre": a_T_tag[:3, 3].copy(),
                "normal": a_T_tag[:3, 2].copy(), "pos_std_m": [0.005] * 3, "n_views": 1, "rms_px": float(pick[0]),
                "flat": False, "layer": None}
    flats = _flat_candidates(view, tag_top_z)
    if flats:
        floor = min(f["rms_px"] for f in flats)
        if floor <= FLAT_ACCEPT_PX:
            close = [f for f in flats if f["rms_px"] <= floor + FLAT_TIE_PX]
            chosen = min(close, key=lambda f: floor if free is None else abs(f["centre"][2] - free["centre"][2]))
            return {"corners": chosen["corners"], "centre": chosen["centre"], "normal": chosen["normal"],
                    "pos_std_m": [0.003, 0.003, 0.001], "n_views": 1, "rms_px": chosen["rms_px"],
                    "flat": True, "layer": chosen["layer"], "a_T_tag": chosen["a_T_tag"]}
    if free is None:
        return None
    fused, layer = snap_to_layer(free, tag_top_z, max_dz=LIVE_MAX_DZ_M)
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
        self.object_moves = {}           # {chỉ số vật trong world gốc: (stamp world gốc lúc dời, vật đã dời)}
        self.face_cubes = {}             # {cube_id: (nghiệm khớp mặt, nhãn, thời điểm)} cube thấy bằng mặt màu
        self.live_cubes = []             # tâm các cube mà camera này tự đo ở khung gần nhất (cube đã dời / mới)

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
        live.faces = copy.deepcopy(self.world.faces)
        live.objects = copy.deepcopy(self.world.objects)
        for index, (base_stamp, item) in list(self.object_moves.items()):
            # world gốc đo lại vật đó (stamp đổi) hoặc không còn vật đó: bỏ phần dời tạm, tin world gốc
            if index >= len(live.objects) or live.objects[index]["stamp"] != base_stamp:
                del self.object_moves[index]
            else:
                live.objects[index] = item
        for cube_id, (fit, label, when) in list(self.face_cubes.items()):
            if cube_id in seen or time.time() - when > FACE_KEEP_S:
                del self.face_cubes[cube_id]                    # đọc được tag rồi, hoặc lâu không thấy lại mặt đó
            elif cube_id not in live.tags:
                live.update_face(cube_id, fit, label, 0.8, LIVE_SOURCE + "-face", sure=False, stamp=when)
        live.region = copy.deepcopy(self.world.region)
        live.zones = copy.deepcopy(self.world.zones)
        live.cameras = copy.deepcopy(self.world.cameras)
        cubes_now = []
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
            cubes_now.append(np.asarray(fused["centre"], float))
            info["moved" if known else "new"].append(tag_id)
        w, h = self.model.image_size
        for tag_id, tag in self.world.tags.items():
            if tag_id in seen or tag_id in info["moved"]:
                continue
            uv = self.model.project(((np.linalg.inv(self.pose) @ np.r_[tag["centre"], 1.0])[:3]).reshape(1, 3))[0]
            if (np.isfinite(uv).all() and MISSING_MARGIN_PX <= uv[0] <= w - MISSING_MARGIN_PX
                    and MISSING_MARGIN_PX <= uv[1] <= h - MISSING_MARGIN_PX):
                info["missing"].append(tag_id)
        self.live_cubes = cubes_now
        live.set_camera(self.name, self.pose, self.model)
        info.update(ok=True, pose=self.pose, world=live, anchors=list(res["anchors"]), rms_px=res["rms_px"])
        return info


    # ------------------------------------------------------------------ vật khác cube + cube không đọc được tag
    def see_objects(self, observations) -> list:
        """Áp quan sát vật (từ `object_track.observe_mask` trên khung của camera này) lên world sống: vật nào bị dời
        thì dời theo, giữ nguyên hình dạng. Trả chỉ số các vật vừa dời."""
        from . import object_track as OT
        current = []
        for index, item in enumerate(self.world.objects):
            moved = self.object_moves.get(index)
            current.append(moved[1] if moved and moved[0] == item["stamp"] else item)
        if not hasattr(self, "follower"):
            self.follower = OT.Follower()
        cubes = [e["centre"] for e in list(self.world.tags.values()) + list(self.world.faces.values())]
        cubes += [fit["centre"] for fit, _, _ in list(self.face_cubes.values())] + list(self.live_cubes)
        updated, moved, matched = OT.relocate(current, observations, LIVE_SOURCE, follower=self.follower, cubes=cubes)
        for index in matched:                                   # dời, hoặc đứng yên nhưng vừa được xác nhận còn ở đó
            self.object_moves[index] = (self.world.objects[index]["stamp"], updated[index])
        return moved

    def see_colour_faces(self, frame, seen_tags=(), pose=None) -> list:
        """Cube không đọc được tag nhưng lộ mặt màu trong khung của camera này: khớp mặt 30 mm (trên / bên) bằng
        pose camera hiện tại. Trả các cube_id vừa nhận. Cần pose (đã `update` thành công ít nhất một lần)."""
        pose = self.pose if pose is None else pose              # pose của ĐÚNG khung đó (luồng tìm vật chạy trễ hơn)
        if pose is None or frame is None:
            return []
        from . import color, registry
        found = []
        layer_zs = [self.tag_top_z + MV.FACE_SIZE_M * k for k in range(4)]
        for face in color.colour_faces(frame, registry.COLOR_TO_ID):
            cube_id = int(face["cube_id"])
            if cube_id in seen_tags:
                continue                                        # tag đọc được: tag thắng
            quad = np.asarray(face["quad"], float)
            fit = MV.cube_face(MV.View(pose, self.model, quad, "live-face"), layer_zs)
            edge = float(np.linalg.norm(quad - np.roll(quad, -1, axis=0), axis=1).mean())
            if fit is None or fit["rms_px"] > max(FACE_MAX_RMS_PX, FACE_MAX_RMS_SHARE * edge):
                continue
            if fit["runner_up_px"] < FACE_LAYER_RATIO * max(fit["rms_px"], 1.0) or not self.world.inside(fit["centre"][:2]):
                continue
            self.face_cubes[cube_id] = (fit, face["label"] + (" o mat ben" if fit.get("kind") == "side" else ""),
                                        time.time())
            found.append(cube_id)
        return found

