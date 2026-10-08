"""Khảo sát ô thả (zone) bằng camera tay: xoay trái/phải nhìn các ô, đo tâm ô trong tọa độ base.

Module độc lập với T8: không import T8, không import ROS. Mọi thứ phụ thuộc vào robot cụ thể đi qua
ba thứ do bên gọi cung cấp:

  layout  `ZoneLayout`  vị trí/góc thả cấu hình của từng zone và cách tính pose thả khi ô dời
  arm     có `execute("look", servo=[J1..J5]) -> {"ok", "from_servo", ...}` (và tùy chọn `close()`)
  scene   có `zone_survey(zones, expect_j1=...) -> {"ok", "zones": [entry]}` (đo ô từ khung hiện tại)

`entry` của một ô: {"zone_id", "seen", "area_m2", "center_xy", "margin_cfg_mm", ...} (xem
zone_locator.survey_zone). Kết quả trả về là dữ liệu thuần; bên gọi tự quyết định thả bằng IK hay bảng.

Quyết định cho từng zone (`decide_zone`):
  keep     điểm thả cấu hình đã nằm sâu trong ô đo được -> dùng bảng cấu hình
  moved    ô dời -> thả vào tâm ô đo được (`target_xy`)
  fallback không đo được đáng tin -> dùng bảng cấu hình, có cảnh báo
  blocked  KHÔNG được thả: đã thấy ô nhưng tay không tới được, hoặc điểm thả cấu hình đang nằm
           trên ô của zone khác (thả theo bảng sẽ bỏ cube vào sai ô)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Sequence

SAFE_MARGIN_MM = 15.0                  # điểm thả cấu hình cách mép ô >= chừng này thì giữ bảng
SCAN_OFFSETS_DEG = (-25.0, 25.0)       # không thấy ô ở J1 nhìn thì nhìn lệch hai phía
LOOK_J1_RANGE = (8.0, 172.0)
MAX_EXTRA_VIEWS = 2                    # góc nhìn thêm cho nhóm còn zone chưa kết luận
PAD_NOMINAL_AREA_M2 = 0.0053           # cỡ ô thật (đo lặp lại ở hai góc nhìn)
PAD_HALF_SIDE_M = 0.036                # nửa cạnh ô (~73 mm)
MAX_PAD_MOVE_M = 0.15                  # ô đo cách điểm thả cấu hình quá xa => nghi đo sai
DEFAULT_NAMES = {1: "xanh dương", 2: "xanh lá", 3: "đỏ", 4: "xám"}
DEFAULT_OPPOSITE = {1: 2, 2: 1, 3: 4, 4: 3}                  # ô đối xứng qua trục giữa hai bên tay
MIN_PAIR_M, MAX_PAIR_M = 0.20, 0.45                          # khoảng cách hợp lý giữa hai ô đối nhau
MAX_AXIS_TILT_DEG = 25.0                                     # trục đối xứng đo được lệch bảng tối đa
DEFAULT_POSE = (110.0, 0.0, 0.0, 90.0)                       # J2..J5 khi nhìn ô
DEFAULT_VIEWS = ((12.0, (1, 3)), (165.0, (2, 4)))            # (J1 nhìn, các zone trong khung)


@dataclass
class ZoneLayout:
    """Bố trí zone của một robot cụ thể.

    release_xy(zone)   -> [x, y] điểm thả cấu hình trong base
    configured_j1(zone)-> J1 cấu hình (chỉ để báo)
    plan(zone, xy)     -> pose thả nếu tay tới được ô ở xy, ném ValueError (kèm lý do) nếu không
    ik_j1(xy)          -> J1 để nhìn thẳng về xy (None nếu không có IK)
    """
    release_xy: Callable[[int], Sequence[float]]
    configured_j1: Callable[[int], float]
    plan: Callable[[int, Sequence[float]], object]
    ik_j1: Callable[[Sequence[float]], float] | None = None
    names: Dict[int, str] = field(default_factory=lambda: dict(DEFAULT_NAMES))
    views: tuple = DEFAULT_VIEWS
    opposite: Dict[int, int] = field(default_factory=lambda: dict(DEFAULT_OPPOSITE))
    look_pose: Sequence[float] = DEFAULT_POSE


def look_j1_toward(layout, xy, fallback):
    """J1 để camera nhìn thẳng về xy; không tính được thì giữ hướng cũ."""
    if layout.ik_j1 is None:
        return fallback
    try:
        j1 = float(layout.ik_j1(xy))
    except Exception:  # noqa: BLE001 - ngoài tầm IK
        return fallback
    return min(max(j1, LOOK_J1_RANGE[0]), LOOK_J1_RANGE[1])


def is_reliable(entry):
    """Tâm ô đo được đáng tin: thấy và diện tích mặt nạ ≈ cỡ ô thật (0,85–1,25×).

    Mặt nạ nhỏ hơn rõ rệt là ô bị khung ảnh cắt hoặc bắt màu sót (ô xám dễ nhiễu cân bằng trắng):
    tâm bị lệch nên không tin.
    """
    if not entry or not entry.get("seen"):
        return False
    center, area = entry.get("center_xy"), entry.get("area_m2")
    if not (center and area):
        return False
    return PAD_NOMINAL_AREA_M2 * 0.85 <= area <= PAD_NOMINAL_AREA_M2 * 1.25


def best_entry(entries):
    good = [e for e in entries if is_reliable(e)]
    return max(good, key=lambda e: float(e.get("area_m2") or 0.0)) if good else None


def decide_zone(layout, zone, entries, inferred_xy=None, inferred_from=None):
    """entries: các lần đo của zone -> {zone_id, action, target_xy, text, measured_xy, inferred}.

    inferred_xy: tâm ô suy từ đối xứng khi ô này không đo được đáng tin (xem infer_missing).
    """
    name = layout.names.get(zone, str(zone))
    base = layout.configured_j1(zone)
    result = {"zone_id": zone, "action": "fallback", "target_xy": None, "measured_xy": None}
    best = best_entry(entries)
    tag = ""
    if best is None and inferred_xy is not None:
        cfg0 = layout.release_xy(zone)
        margin0 = (PAD_HALF_SIDE_M - max(abs(cfg0[0] - inferred_xy[0]), abs(cfg0[1] - inferred_xy[1]))) * 1000.0
        best = {"center_xy": list(inferred_xy), "margin_cfg_mm": margin0}
        tag = f" [suy ra từ đối xứng với zone {inferred_from}]"
        result["inferred"] = True
    if best is None:
        seen = [e for e in entries if e and e.get("seen")]
        why = ("thấy ô nhưng bị khung ảnh cắt nhiều/diện tích bất thường" if seen else "không thấy ô")
        result["text"] = f"Zone {zone} ({name}): {why}; giữ J1={base:.0f}° cấu hình (cảnh báo)"
        return result
    cx, cy = best["center_xy"]
    result["measured_xy"] = [round(cx, 4), round(cy, 4)]
    margin = best.get("margin_cfg_mm")
    if margin is not None and margin >= SAFE_MARGIN_MM:
        result.update(action="keep", text=f"Zone {zone} ({name}): giữ cấu hình (điểm thả cách mép ô "
                                          f"{margin:.0f} mm){tag}")
        return result
    cfg = layout.release_xy(zone)
    moved = math.hypot(cx - cfg[0], cy - cfg[1])
    if moved > MAX_PAD_MOVE_M:
        result["text"] = (f"Zone {zone} ({name}): ô đo cách điểm thả cấu hình {moved * 1000:.0f} mm "
                          f"(> {MAX_PAD_MOVE_M * 1000:.0f}); nghi đo sai, giữ cấu hình (cảnh báo)")
        return result
    try:
        layout.plan(zone, [cx, cy])                      # tay có tới được ô đo không?
    except ValueError as exc:
        # Đã thấy ô thật mà không tới được: thả theo bảng là thả vào chỗ KHÔNG phải ô này.
        result.update(action="blocked", text=(
            f"Zone {zone} ({name}): đo được ô tại ({cx:+.3f}, {cy:+.3f}) m nhưng {exc}. "
            "Không thả theo bảng cấu hình vì điểm đó không còn là ô này; hãy dời ô vào tầm tay."))
        return result
    result.update(action="moved", target_xy=[round(cx, 4), round(cy, 4)],
                  text=f"Zone {zone} ({name}): ô nằm cách điểm thả cấu hình {moved * 1000:.0f} mm "
                       f"-> thả vào tâm ô ({cx:+.3f}, {cy:+.3f}) m{tag}")
    return result


def pad_conflict(layout, zone, measured_centres):
    """Zone khác (đã đo đáng tin) có ô che điểm thả cấu hình của `zone`, hay None."""
    cfg = layout.release_xy(zone)
    for other, (cx, cy) in measured_centres.items():
        if other == zone:
            continue
        if abs(cfg[0] - cx) <= PAD_HALF_SIDE_M and abs(cfg[1] - cy) <= PAD_HALF_SIDE_M:
            return other
    return None


def _mirror_axis(p, q):
    """Trục đối xứng của hai điểm p,q: (điểm giữa, pháp tuyến đơn vị hướng p->q)."""
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    return ((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0), ((q[0] - p[0]) / d, (q[1] - p[1]) / d)


def infer_missing(layout, measured):
    """measured {zone: (x, y)} -> {zone: ([x, y], zone_nguồn)} cho ô chưa đo mà ô đối xứng đã đo.

    Bố trí đối xứng: mỗi cặp (xanh dương–xanh lá, đỏ–xám) nằm hai bên một trục. Cần ít nhất MỘT cặp đã
    đo đủ để biết trục (đường trung trực của cặp), rồi phản chiếu ô đã đo của cặp còn lại sang bên kia.
    Trục đo được phải hợp lý (khoảng cách cặp, độ lệch so với bảng cấu hình) nếu không không suy.
    """
    axes = []
    for a, b in sorted({tuple(sorted(item)) for item in layout.opposite.items()}):
        if a in measured and b in measured:
            gap = math.hypot(measured[b][0] - measured[a][0], measured[b][1] - measured[a][1])
            if not MIN_PAIR_M <= gap <= MAX_PAIR_M:
                continue
            cfg_a, cfg_b = layout.release_xy(a), layout.release_xy(b)
            _, n_cfg = _mirror_axis(cfg_a, cfg_b)
            m, n = _mirror_axis(measured[a], measured[b])
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, n[0] * n_cfg[0] + n[1] * n_cfg[1]))))
            if tilt > MAX_AXIS_TILT_DEG:
                continue
            axes.append((m, n))
    if not axes:
        return {}
    m = (sum(a[0][0] for a in axes) / len(axes), sum(a[0][1] for a in axes) / len(axes))
    nx, ny = sum(a[1][0] for a in axes), sum(a[1][1] for a in axes)
    norm = math.hypot(nx, ny)
    n = (nx / norm, ny / norm)
    out = {}
    for zone, opp in layout.opposite.items():
        if zone not in measured and opp in measured:
            px, py = measured[opp]
            k = 2.0 * ((px - m[0]) * n[0] + (py - m[1]) * n[1])
            out[zone] = ([round(px - k * n[0], 4), round(py - k * n[1], 4)], opp)
    return out


def decide_zones(layout, per_zone_entries, zones=(1, 2, 3, 4)):
    """{zone: [entries]} -> ({zone: [x, y]}, [kết quả từng zone trong `zones`]).

    per_zone_entries có thể chứa cả zone không cần dùng: chúng chỉ dùng để phát hiện điểm thả cấu hình
    của zone cần dùng đang nằm trên ô của zone khác (-> blocked).
    """
    measured = {}
    for z, entries in per_zone_entries.items():
        best = best_entry(entries)
        if best is not None:
            measured[z] = tuple(best["center_xy"])
    inferred = infer_missing(layout, measured)
    everywhere = {**measured, **{z: tuple(v[0]) for z, v in inferred.items()}}
    targets, report = {}, []
    for zone in zones:
        guess = inferred.get(zone)
        item = decide_zone(layout, zone, per_zone_entries.get(zone, []),
                           inferred_xy=guess[0] if guess else None, inferred_from=guess[1] if guess else None)
        if item["action"] in ("keep", "fallback"):
            other = pad_conflict(layout, zone, everywhere)
            if other is not None and item["action"] != "keep":
                name, oname = layout.names.get(zone, zone), layout.names.get(other, other)
                item.update(action="blocked", text=(
                    f"Zone {zone} ({name}): điểm thả cấu hình nằm trên ô zone {other} ({oname}); "
                    f"thả theo bảng sẽ bỏ cube vào sai ô. " +
                    ("Ô đo được không dùng được; " if item.get("measured_xy") else "Chưa đo được ô này; ") +
                    "hãy dời ô vào tầm tay hoặc cập nhật bảng cấu hình."))
        report.append(item)
        if item["action"] == "moved":
            targets[zone] = item["target_xy"]
    return targets, report


def run_survey(arm, scene, layout, zones=(1, 2, 3, 4), log=print):
    """Camera tay xoay tới từng phía, đo ô rồi quay về pose xuất phát.

    Mỗi nhóm zone được nhìn ở J1 của nhóm; ô chưa kết luận thì nhìn lệch ±25° và/hoặc xoay về tâm vùng
    đã thấy. Các zone cùng khung nhìn nhưng không cần dùng cũng được đo để phát hiện điểm thả cấu hình
    nằm trên ô của zone khác. Không bao giờ ném lỗi.

    Trả {"targets", "report", "restored", "surveyed_views", "measured"}.
    """
    entries, start_servo, views = {}, None, 0
    broken = []          # lỗi hạ tầng (không có khung/khớp): xoay thêm cũng vô ích -> dừng sớm

    def close_arm():
        if hasattr(arm, "close"):
            arm.close()          # nhả cổng serial để real_joint_mirror đọc khớp

    def look_and_measure(j1, zone_ids):
        nonlocal start_servo, views
        try:
            moved = arm.execute("look", servo=[j1, *layout.look_pose])
        finally:
            close_arm()
        if not moved.get("ok"):
            log(f"[zone] Không xoay tới J1={j1:.0f}° được: {moved.get('reply', '')}")
            return
        if start_servo is None:
            start_servo = moved.get("from_servo")
        # expect_j1 = góc ĐÃ LỆNH (readback trả sớm khi khớp trong ~12°); chỉ để loại khớp cũ.
        result = scene.zone_survey([{"zone_id": z, "release_xy": list(layout.release_xy(z))}
                                    for z in zone_ids], expect_j1=j1)
        if not result.get("ok"):
            log(f"[zone] Khảo sát ở J1={j1:.0f}° lỗi: {result.get('reason', '')}")
            broken.append(result.get("reason", "lỗi"))
            return
        views += 1
        found = {int(e["zone_id"]): e for e in result.get("zones", [])}
        for z in zone_ids:
            entries.setdefault(z, []).append(found.get(z, {"seen": False}))

    def last(z):
        return entries[z][-1] if entries.get(z) else None

    def measured_now():
        return {z: tuple(best_entry(e)["center_xy"]) for z, e in entries.items() if best_entry(e)}

    def unresolved():
        """Zone cần dùng mà chưa đo đáng tin và cũng chưa suy được từ đối xứng."""
        inferred = infer_missing(layout, measured_now())
        return [z for z in zones if not is_reliable(last(z)) and z not in inferred]

    try:
        wanted_views = [v for v in layout.views if any(z in zones for z in v[1])]
        other_views = [v for v in layout.views if v not in wanted_views]
        # 1) nhìn phía có zone cần dùng; 2) chưa đủ thì nhìn nốt phía kia (cần cả hai phía để biết trục
        # đối xứng suy ô bị thiếu); 3) vẫn thiếu thì nhìn lệch/xoay về tâm vùng đã thấy.
        for view in wanted_views + other_views:
            if broken or (view in other_views and not unresolved()):
                continue
            look_and_measure(view[0], list(view[1]))
        for base_j1, group in layout.views:
            tried = [base_j1]
            for _ in range(MAX_EXTRA_VIEWS):
                pending = [z for z in unresolved() if z in group]
                if broken or not pending:
                    break
                partial = [last(z) for z in pending if last(z) and last(z).get("center_xy")]
                if partial:
                    j1 = look_j1_toward(layout, partial[0]["center_xy"], base_j1)
                else:
                    scan = [min(max(base_j1 + d, LOOK_J1_RANGE[0]), LOOK_J1_RANGE[1])
                            for d in SCAN_OFFSETS_DEG]
                    scan = [c for c in scan if all(abs(c - t) > 3.0 for t in tried)]
                    if not scan:
                        break
                    j1 = scan[0]
                j1 = min(max(j1, LOOK_J1_RANGE[0]), LOOK_J1_RANGE[1])
                if any(abs(j1 - t) <= 3.0 for t in tried):
                    break
                tried.append(j1)
                look_and_measure(j1, pending)
    except Exception as exc:  # noqa: BLE001 - khảo sát không được làm hỏng lệnh của bên gọi
        log(f"[zone] Khảo sát gián đoạn: {exc}")
    restored = True
    if start_servo is not None:
        try:
            back = arm.execute("look", servo=[float(v) for v in start_servo])
            restored = bool(back.get("ok"))
        except Exception as exc:  # noqa: BLE001
            log(f"[zone] Không quay về pose xuất phát: {exc}")
            restored = False
        finally:
            close_arm()
    targets, report = decide_zones(layout, entries, zones=zones)
    for item in report:
        log("[zone] " + item["text"])
    measured = {z: best_entry(e)["center_xy"] for z, e in entries.items() if best_entry(e)}
    inferred = {z: v[0] for z, v in infer_missing(layout, {k: tuple(v) for k, v in measured.items()}).items()}
    return {"targets": targets, "report": report, "restored": restored, "surveyed_views": views,
            "error": broken[0] if broken else "",
            "measured": {int(z): [round(v, 4) for v in xy] for z, xy in measured.items()},
            "inferred": {int(z): xy for z, xy in inferred.items()}}
