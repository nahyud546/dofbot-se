#!/usr/bin/env python3
"""World tự cập nhật từ camera tay ở POSE BẤT KỲ: tay đứng yên ở đâu, thấy cube nào thì ghi cube đó vào world.

Khác `build_world --scan` (tự lái tay qua một bộ pose cố định): ở đây không có lệnh chuyển động nào. `Watcher` nhận
(góc khớp thật, tag thấy trong ảnh) của từng lần nhìn và giữ world luôn đúng với những gì đã thấy:
  - mỗi lần nhìn cho pose camera từ FK + hand-eye ở đúng góc khớp lúc đó (không giả định READY);
  - cube thấy lần đầu được ghi ngay nếu khớp mô hình "nằm phẳng ở một tầng" (một góc nhìn là đủ chắc cho x, y);
    thấy thêm từ pose khác thì gộp lại (trung vị, `multiview.robust_tag`);
  - cube thấy ở chỗ khác hẳn chỗ đã ghi => đã bị dời: bỏ số đo cũ, ghi chỗ mới;
  - cube lẽ ra nằm trong khung nhìn mà nhiều lần liền không thấy => đã bị lấy đi: xóa khỏi world;
  - cube nằm ngoài khung nhìn hiện tại được giữ nguyên (không thấy không có nghĩa là mất).
Chỉ dùng lần nhìn có J1 trong vùng hand-eye đã kiểm chứng; ngoài vùng thì báo lý do và bỏ qua.
Thuần toán (test được bằng tay máy giả lập); phần cứng nằm ở `build_world.py --watch`.
"""
from __future__ import annotations

import numpy as np

import active_view as A
from cube_vision import multiview as MV
from cube_vision import view_quality as Q
from cube_vision.live_world import locate_tag
from cube_vision.world_map import WorldMap, snap_to_layer

SOURCE = "wrist-watch"
MOVED_M = 0.015               # ước lượng mới cách chỗ đã ghi hơn mức này => cube đã bị dời
NEW_VIEW_M = 0.015            # tâm camera phải cách các góc nhìn đã giữ chừng này mới tính là góc nhìn mới
MAX_VIEWS = 8
MISSING_AFTER = 3             # số lần nhìn liên tiếp "lẽ ra thấy mà không thấy" trước khi xóa
MISSING_MARGIN_PX = 70.0


class Watcher:
    def __init__(self, cal, world: WorldMap | None = None):
        self.cal = cal
        self.world = world if world is not None else WorldMap()
        self.tag_top_z = float(cal["tag_top_z"])
        self.views = {}           # {tag_id: [View]} các góc nhìn đang dùng cho từng cube
        self.unseen = {}          # {tag_id: số lần liên tiếp lẽ ra thấy mà không thấy}

    def _fuse(self, tag_id):
        """(fused, chắc?) từ các góc nhìn đang giữ của một tag."""
        views = self.views[tag_id]
        if len(views) == 1:
            est = locate_tag(views[0].corners_px, views[0].camera, views[0].a_T_optical, self.tag_top_z)
            return est, bool(est and est.get("flat"))
        fused = MV.robust_tag(views)
        if fused is None:
            return None, False
        sure = bool(Q.assess_fused(fused))
        if not sure:
            # Hai góc nhìn sát nhau không cho thị sai, nhưng nếu chúng thống nhất và cube khớp mô hình nằm phẳng thì
            # vẫn chắc như một góc nhìn đơn (độ sâu đến từ độ cao tầng, không từ thị sai).
            spread = np.asarray(fused["spread_m"], float)
            latest = locate_tag(views[-1].corners_px, views[-1].camera, views[-1].a_T_optical, self.tag_top_z)
            sure = bool(latest and latest.get("flat") and max(spread[0], spread[1]) <= Q.MAX_SPREAD_XY_M
                        and spread[2] <= Q.MAX_SPREAD_Z_M)
        fused, layer = snap_to_layer(fused, self.tag_top_z)
        fused["layer"] = layer
        return fused, sure

    def observe(self, servo, seen: dict, stamp: float | None = None, clear_view: bool | None = None) -> dict:
        """Một lần nhìn khi tay đứng yên. servo: 5 góc khớp thật (độ); seen {tag_id: (4,2)} pixel camera tay.

        clear_view: ảnh của lần nhìn này có đáng tin để kết luận "không thấy" không (nét, đủ sáng). None: chỉ tin
        khi lần nhìn này có thấy ít nhất một tag khác. Ảnh nhòe/tối mà không thấy gì thì KHÔNG được coi là cube mất.

        Trả {"used": [id], "new": [id], "moved": [id], "removed": [id], "rejected": {id: lý do}, "skipped": lý do|None}.
        """
        events = {"used": [], "new": [], "moved": [], "removed": [], "rejected": {}, "skipped": None}
        servo = [float(v) for v in servo[:5]]
        j1_range = self.cal.get("j1_valid_range")
        if j1_range and not (j1_range[0] <= servo[0] <= j1_range[1]):
            events["skipped"] = (f"J1={servo[0]:.0f}° ngoài vùng hand-eye đã kiểm chứng "
                                 f"({j1_range[0]:.0f}–{j1_range[1]:.0f}°): không ghi lần nhìn này")
            return events
        camera = A.wrist_camera(self.cal)
        base_T_opt = A.base_T_optical(servo, self.cal)
        for tag_id, corners in sorted(seen.items()):
            view = A.make_view(corners, servo, self.cal, f"watch {len(self.views.get(tag_id, [])) + 1}")
            quality = Q.assess_view(view, j1=servo[0], j1_range=j1_range)
            if not quality:
                events["rejected"][tag_id] = quality.text()
                continue
            est = locate_tag(corners, camera, base_T_opt, self.tag_top_z)
            if est is None:
                events["rejected"][tag_id] = "không giải được pose từ lần nhìn này"
                continue
            known = self.world.tags.get(tag_id)
            is_new = known is None
            moved = known is not None and float(np.linalg.norm(est["centre"] - known["centre"])) > MOVED_M
            if moved or tag_id not in self.views:
                self.views[tag_id] = []                           # số đo cũ không còn đúng (hoặc từ phiên trước)
            kept = self.views[tag_id]
            gaps = [float(np.linalg.norm(v.centre() - view.centre())) for v in kept]
            if gaps and min(gaps) < NEW_VIEW_M:
                kept[int(np.argmin(gaps))] = view                 # cùng chỗ nhìn: thay bằng khung mới hơn
            else:
                kept.append(view)
                del kept[:-MAX_VIEWS]
            fused, sure = self._fuse(tag_id)
            if fused is None:
                continue
            self.world.forget(tag_id)
            self.world.update_tag(tag_id, fused, SOURCE, sure=sure, stamp=stamp)
            self.unseen[tag_id] = 0
            events["used"].append(tag_id)
            if is_new:
                events["new"].append(tag_id)
            elif moved:
                events["moved"].append(tag_id)
        # Cube của world lẽ ra nằm gọn trong khung nhìn này mà không thấy.
        if not (clear_view if clear_view is not None else bool(events["used"])):
            return events
        w, h = camera.image_size
        opt_T_base = np.linalg.inv(base_T_opt)
        for tag_id, tag in list(self.world.tags.items()):
            if tag_id in seen:
                continue
            cam = (opt_T_base @ np.r_[tag["centre"], 1.0])[:3]
            uv = camera.project(cam.reshape(1, 3))[0] if cam[2] > 0.05 else np.array([np.nan, np.nan])
            inside = (np.isfinite(uv).all() and MISSING_MARGIN_PX <= uv[0] <= w - MISSING_MARGIN_PX
                      and MISSING_MARGIN_PX <= uv[1] <= h - MISSING_MARGIN_PX)
            if not inside:
                continue
            self.unseen[tag_id] = self.unseen.get(tag_id, 0) + 1
            if self.unseen[tag_id] >= MISSING_AFTER:
                self.world.forget(tag_id)
                self.views.pop(tag_id, None)
                self.unseen.pop(tag_id, None)
                events["removed"].append(tag_id)
        return events


def describe(events) -> str:
    """Một dòng tiếng Việt cho một lần nhìn; rỗng khi không có gì đáng nói."""
    if events["skipped"]:
        return events["skipped"]
    parts = []
    for key, text in (("new", "cube mới"), ("moved", "cube đã dời"), ("removed", "cube đã lấy đi")):
        if events[key]:
            parts.append(f"{text}: {events[key]}")
    for tag_id, why in events["rejected"].items():
        parts.append(f"bỏ tag {tag_id} ({why})")
    return "; ".join(parts)
