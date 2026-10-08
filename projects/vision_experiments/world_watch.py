#!/usr/bin/env python3
"""World tự cập nhật từ camera tay ở POSE BẤT KỲ: tay đứng yên ở đâu, thấy cube nào thì ghi cube đó vào world.

Khác `build_world --scan` (tự lái tay qua một bộ pose cố định): ở đây không có lệnh chuyển động nào. `Watcher` nhận
(góc khớp thật, tag thấy trong ảnh) của từng lần nhìn và giữ world luôn đúng với những gì đã thấy:
  - mỗi lần nhìn cho pose camera từ FK + hand-eye ở đúng góc khớp lúc đó (không giả định READY);
  - cube thấy lần đầu được ghi ngay nếu khớp mô hình "nằm phẳng ở một tầng" (một góc nhìn là đủ chắc cho x, y);
    thấy thêm từ pose khác thì gộp lại (trung vị, `multiview.robust_tag`);
  - cube thấy ở chỗ khác hẳn chỗ đã ghi => đã bị dời: bỏ số đo cũ, ghi chỗ mới;
  - cube lẽ ra nằm trong khung nhìn mà nhiều lần liền không thấy => đã bị lấy đi: xóa khỏi world;
  - cube nằm ngoài khung nhìn hiện tại được giữ nguyên (không thấy không có nghĩa là mất);
  - cube không ngửa tag (ngửa mặt màu / hình in) được ghi qua `observe_faces`: 4 góc mặt trên do bộ nhận mặt đưa
    vào, khớp mô hình "mặt 30 mm nằm phẳng" để ra tầng + x, y. Tag luôn thắng mặt khi cả hai cùng có.
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
MISSING_AFTER = 3             # số lần nhìn liên tiếp "lẽ ra thấy mà không thấy" trước khi xóa...
MISSING_AFTER_S = 4.0         # ...và phải kéo dài chừng này giây: tay người che cube vài giây khi sắp xếp là chuyện thường
MISSING_MARGIN_PX = 14.0      # cả 4 góc tag phải nằm trong ảnh cách mép chừng này thì tag mới "lẽ ra đọc được"
FACE_SOURCE = "wrist-face"
FACE_MIN_CONFIDENCE = 0.5     # độ tin của bộ nhận mặt (màu / hình in) thấp hơn mức này thì không ghi
FACE_MAX_RMS_PX = 7.0         # đo thật khi đủ sáng: đúng tầng 1,8–4,6 px, sai tầng 13–21 px
FACE_LAYER_RATIO = 2.0        # tầng tốt nhất phải khớp tốt hơn tầng nhì chừng này lần, không thì mơ hồ tầng
FACE_AGREE_M = 0.010          # chỗ mới (cube mới / đã dời) phải được hai lần nhìn liền nhau xác nhận trong mức này
FACE_TAG_RECENT_S = 2.0       # tag của cube vừa được đọc trong chừng này giây thì không dùng mặt
FACE_MAX_LAYER = 3
FACE_KEEP = 8                 # số lần khớp gần nhất giữ lại để lấy trung vị
FACE_MISSING_AFTER_S = 6.0    # bộ nhận mặt chập chờn hơn tag: chờ lâu hơn trước khi coi là đã lấy đi


class Watcher:
    def __init__(self, cal, world: WorldMap | None = None):
        self.cal = cal
        self.world = world if world is not None else WorldMap()
        self.tag_top_z = float(cal["tag_top_z"])
        self.views = {}           # {tag_id: [View]} các góc nhìn đang dùng cho từng cube
        self.unseen = {}          # {tag_id: số lần liên tiếp lẽ ra thấy mà không thấy}
        self.unseen_since = {}    # {tag_id: thời điểm bắt đầu chuỗi không thấy đó}
        self.expected = set()     # tag của world mà lần nhìn gần nhất lẽ ra phải thấy nhưng không thấy
        self.face_fits = {}       # {cube_id: [nghiệm flat_face gần đây]} của chỗ đang ghi trong world
        self.face_pending = {}    # {cube_id: tâm của lần nhìn trước} chờ lần nhìn thứ hai xác nhận chỗ mới

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
            if not self.world.inside(est["centre"][:2]):
                events["rejected"][tag_id] = "ngoài vùng world"
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
            self.face_fits.pop(tag_id, None)
            self.face_pending.pop(tag_id, None)
            self.world.update_tag(tag_id, fused, SOURCE, sure=sure, stamp=stamp)
            self.unseen.pop(tag_id, None)
            self.unseen_since.pop(tag_id, None)
            events["used"].append(tag_id)
            if is_new:
                events["new"].append(tag_id)
            elif moved:
                events["moved"].append(tag_id)
        # Cube của world lẽ ra đọc được trong khung nhìn này (cả 4 góc tag nằm trong ảnh) mà không thấy.
        import time as _time
        now = _time.time() if stamp is None else float(stamp)
        w, h = camera.image_size
        opt_T_base = np.linalg.inv(base_T_opt)
        self.expected = set()
        for tag_id, tag in list(self.world.tags.items()):
            if tag_id in seen:
                continue
            cam = np.asarray(tag["corners"], float) @ opt_T_base[:3, :3].T + opt_T_base[:3, 3]
            uv = camera.project(cam) if cam[:, 2].min() > 0.05 else np.full((4, 2), np.nan)
            inside = (np.isfinite(uv).all() and uv[:, 0].min() >= MISSING_MARGIN_PX
                      and uv[:, 0].max() <= w - MISSING_MARGIN_PX and uv[:, 1].min() >= MISSING_MARGIN_PX
                      and uv[:, 1].max() <= h - MISSING_MARGIN_PX)
            if not inside:
                self.unseen.pop(tag_id, None)                     # ra khỏi khung nhìn: không kết luận gì, bắt đầu lại
                self.unseen_since.pop(tag_id, None)
                continue
            self.expected.add(tag_id)
            if not (clear_view if clear_view is not None else bool(events["used"])):
                continue                                          # ảnh không đáng tin: không tính là một lần mất
            self.unseen[tag_id] = self.unseen.get(tag_id, 0) + 1
            since = self.unseen_since.setdefault(tag_id, now)
            if self.unseen[tag_id] >= MISSING_AFTER and now - since >= MISSING_AFTER_S:
                self.world.forget(tag_id)
                self.views.pop(tag_id, None)
                self.unseen.pop(tag_id, None)
                self.unseen_since.pop(tag_id, None)
                self.expected.discard(tag_id)
                events["removed"].append(tag_id)
        return events


    def observe_faces(self, servo, faces, tag_ids=(), stamp: float | None = None, clear_view: bool = True) -> dict:
        """Một lần chạy bộ nhận mặt khi tay đứng yên. faces: [{"cube_id", "quad" (4,2) pixel, "label", "confidence"}]
        là mặt TRÊN của các cube không đọc được tag; tag_ids: cube có tag đọc được trong chính khung đó.

        Trả cùng dạng với `observe` (khóa của "rejected" là chuỗi "mặt cube N").
        """
        import time as _time
        events = {"used": [], "new": [], "moved": [], "removed": [], "rejected": {}, "skipped": None}
        servo = [float(v) for v in servo[:5]]
        j1_range = self.cal.get("j1_valid_range")
        if j1_range and not (j1_range[0] <= servo[0] <= j1_range[1]):
            events["skipped"] = f"J1={servo[0]:.0f}° ngoài vùng hand-eye đã kiểm chứng: không ghi lần nhìn này"
            return events
        now = _time.time() if stamp is None else float(stamp)
        camera = A.wrist_camera(self.cal)
        w, h = camera.image_size
        layer_zs = [self.tag_top_z + MV.FACE_SIZE_M * k for k in range(FACE_MAX_LAYER + 1)]
        detected = set(int(t) for t in tag_ids)
        for face in faces or []:
            cube_id = face.get("cube_id")
            if cube_id is None or face.get("quad") is None:
                continue
            cube_id = int(cube_id)
            detected.add(cube_id)
            tag = self.world.tags.get(cube_id)
            if cube_id in tag_ids or (tag is not None and now - tag["stamp"] < FACE_TAG_RECENT_S):
                continue                                          # tag đọc được: tag thắng
            key = f"mặt cube {cube_id}"
            quad = np.asarray(face["quad"], float).reshape(4, 2)
            if float(face.get("confidence", 0.0)) < FACE_MIN_CONFIDENCE:
                events["rejected"][key] = f"độ tin {face.get('confidence', 0.0):.2f} thấp"
                continue
            if quad.min() < 4 or quad[:, 0].max() > w - 4 or quad[:, 1].max() > h - 4:
                events["rejected"][key] = "mặt bị cắt ở mép ảnh"
                continue
            fit = MV.flat_face(A.make_view(quad, servo, self.cal, "face"), layer_zs)
            if fit is None or fit["rms_px"] > FACE_MAX_RMS_PX:
                events["rejected"][key] = ("không khớp mô hình mặt 30 mm nằm phẳng"
                                           + ("" if fit is None else f" (lệch {fit['rms_px']:.1f} px)"))
                continue
            if fit["runner_up_px"] < FACE_LAYER_RATIO * max(fit["rms_px"], 1.0):
                events["rejected"][key] = (f"mơ hồ tầng (tầng {fit['layer']} lệch {fit['rms_px']:.1f} px, "
                                           f"tầng nhì {fit['runner_up_px']:.1f} px)")
                continue
            if not self.world.inside(fit["centre"][:2]):
                events["rejected"][key] = "ngoài vùng world"
                continue
            known = self.world.entry(cube_id)
            same_place = (known is not None and cube_id in self.world.faces
                          and float(np.linalg.norm(fit["centre"] - known["centre"])) <= MOVED_M)
            if not same_place:
                before = self.face_pending.get(cube_id)
                self.face_pending[cube_id] = fit["centre"].copy()
                if before is None or float(np.linalg.norm(fit["centre"] - before)) > FACE_AGREE_M:
                    continue                                      # chỗ mới: chờ lần nhìn kế tiếp xác nhận
                self.face_fits[cube_id] = []
                self.views.pop(cube_id, None)
                events["new" if known is None else "moved"].append(cube_id)
            self.face_pending.pop(cube_id, None)
            fits = self.face_fits.setdefault(cube_id, [])
            fits.append(fit)
            del fits[:-FACE_KEEP]
            centre = np.median([f["centre"] for f in fits], axis=0)
            merged = dict(fit, centre=centre, corners=np.asarray(fit["corners"], float) + (centre - fit["centre"]))
            self.world.update_face(cube_id, merged, str(face.get("label", "")), float(face.get("confidence", 0.0)),
                                   FACE_SOURCE, sure=True, n_views=len(fits), stamp=now)
            self.unseen.pop(cube_id, None)
            self.unseen_since.pop(cube_id, None)
            events["used"].append(cube_id)
        # Cube ghi bằng mặt, lẽ ra nằm trọn trong khung này mà bộ nhận mặt không thấy gì ở đó.
        opt_T_base = np.linalg.inv(A.base_T_optical(servo, self.cal))
        for cube_id, face in list(self.world.faces.items()):
            if cube_id in detected:
                continue
            cam = np.asarray(face["corners"], float) @ opt_T_base[:3, :3].T + opt_T_base[:3, 3]
            uv = camera.project(cam) if cam[:, 2].min() > 0.05 else np.full((4, 2), np.nan)
            inside = (np.isfinite(uv).all() and uv[:, 0].min() >= MISSING_MARGIN_PX
                      and uv[:, 0].max() <= w - MISSING_MARGIN_PX and uv[:, 1].min() >= MISSING_MARGIN_PX
                      and uv[:, 1].max() <= h - MISSING_MARGIN_PX)
            if not inside:
                self.unseen.pop(cube_id, None)
                self.unseen_since.pop(cube_id, None)
                continue
            if not clear_view:
                continue
            self.unseen[cube_id] = self.unseen.get(cube_id, 0) + 1
            since = self.unseen_since.setdefault(cube_id, now)
            if self.unseen[cube_id] >= MISSING_AFTER and now - since >= FACE_MISSING_AFTER_S:
                self.world.forget(cube_id)
                self.face_fits.pop(cube_id, None)
                self.unseen.pop(cube_id, None)
                self.unseen_since.pop(cube_id, None)
                events["removed"].append(cube_id)
        return events


def faces_from_detections(detections) -> tuple:
    """Kết quả `cube_vision.identify.Identifier(FULL).detect` -> (faces cho `observe_faces`, {cube có tag đọc được})."""
    faces, tag_ids = [], set()
    for det in detections:
        if det.cube_id is None:
            continue
        via = str((det.evidence or {}).get("via", ""))
        if via == "tag" or det.source.endswith("tag"):
            tag_ids.add(int(det.cube_id))
        elif det.quad is not None:
            faces.append({"cube_id": int(det.cube_id), "quad": det.quad, "confidence": float(det.confidence),
                          "label": str((det.evidence or {}).get("label") or via)})
    return faces, tag_ids


def describe(events) -> str:
    """Một dòng tiếng Việt cho một lần nhìn; rỗng khi không có gì đáng nói."""
    if events["skipped"]:
        return events["skipped"]
    parts = []
    for key, text in (("new", "cube mới"), ("moved", "cube đã dời"), ("removed", "cube đã lấy đi")):
        if events[key]:
            parts.append(f"{text}: {events[key]}")
    for tag_id, why in events["rejected"].items():
        parts.append(f"bỏ {tag_id if isinstance(tag_id, str) else f'tag {tag_id}'} ({why})")
    return "; ".join(parts)
