"""Lập kế hoạch cho lệnh "tất cả cube" (sort/stack hàng loạt): chọn cube nào làm trước, bước nào bỏ qua.

Thuần dữ liệu, không ROS/T8: bên gọi đưa danh sách cube thấy được {id, xy, layer, reachable} và nhận
về thứ tự thực hiện kèm lý do với cube bị bỏ. Vòng chạy thật (duyệt viewer, thử lại, báo cáo) nằm ở T8.

  sort   mọi cube đang thấy -> zone của nó; IK tới được làm trước, rồi cube cao tầng trước (gỡ chồng),
         rồi cube gần đế trước (ít vướng nhất).
  stack  một tháp: cube nền (người dùng chỉ định, không thì cube ở tầng bàn, tới được, gần giữa bàn nhất),
         các cube còn lại lần lượt lên trên cùng; tối đa `max_height` cube/tháp, cube dư được báo lại.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

ALL_WORDS = {"all", "tat_ca", "tatca", "moi", "moi_cube", "every", "everything", "het", "all_cubes"}
MAX_TOWER_HEIGHT = 4
# Lỗi của MỘT cube (bỏ cube đó, làm tiếp). Mọi mã khác (tay/viewer/serial lỗi, đang giữ vật...) dừng cả lô.
SKIPPABLE_CODES = {"needs_observation", "pose_not_ready", "zone_unavailable", "approval_timeout",
                   "unknown_selector", "grasp_not_ready", "identity_conflict", "wrong_target_face",
                   "preflight_failed", "stack_geometry_unavailable", "objects_too_close", "same_object",
                   "placement_unverified"}


def is_all(label) -> bool:
    """Nhãn nghĩa 'tất cả cube' ("all", "tất cả", "tat_ca"...)."""
    text = re.sub(r"[\s\-]+", "_", str(label or "").strip().lower())
    return text in ALL_WORDS


@dataclass
class CubeInfo:
    id: int
    xy: tuple | None = None          # base_link (m); None = chưa có tọa độ
    layer: int = 0
    reachable: bool | None = None    # IK tới được; None = chưa biết


def _key_sort(cube: CubeInfo):
    dist = math.hypot(*cube.xy) if cube.xy else math.inf
    reach = 0 if cube.reachable else (1 if cube.reachable is None else 2)
    return (reach, -cube.layer, dist, cube.id)


def plan_sort(cubes):
    """-> (thứ tự id cần sort, [(id, lý do bỏ)]). Cube không tới được bị bỏ sau cùng nhưng vẫn được thử."""
    ordered = sorted(cubes, key=_key_sort)
    skipped = [(c.id, "IK không tới được") for c in ordered if c.reachable is False]
    return [c.id for c in ordered if c.reachable is not False], skipped


def plan_stack(cubes, base=None, max_height=MAX_TOWER_HEIGHT):
    """-> {"base": id|None, "steps": [(source_id, target_id)], "skipped": [(id, lý do)]}.

    target_id của bước k là cube trên cùng của tháp ngay trước bước đó (nguồn của bước k-1).
    """
    skipped = []
    pool = []
    for c in cubes:
        if c.layer and c.layer > 0:
            skipped.append((c.id, f"đang nằm trên cube khác (tầng {c.layer})"))
        elif c.reachable is False:
            skipped.append((c.id, "IK không tới được"))
        else:
            pool.append(c)
    if len(pool) < 2:
        return {"base": pool[0].id if pool else None, "steps": [], "skipped": skipped,
                "reason": "cần ít nhất 2 cube ở tầng bàn để xếp chồng"}
    if base is not None and base not in {c.id for c in pool}:
        return {"base": None, "steps": [], "skipped": skipped,
                "reason": f"cube nền {base} không thấy hoặc không dùng được"}
    if base is None:
        # Nền: tới được chắc chắn trước, gần giữa bàn (|y| nhỏ) rồi gần đế.
        def base_key(c):
            sure = 0 if c.reachable else 1
            return (sure, abs(c.xy[1]) if c.xy else math.inf, math.hypot(*c.xy) if c.xy else math.inf, c.id)
        base = min(pool, key=base_key).id
    rest = sorted((c for c in pool if c.id != base), key=_key_sort)
    steps, top = [], base
    for cube in rest[:max_height - 1]:
        steps.append((cube.id, top))
        top = cube.id
    for cube in rest[max_height - 1:]:
        skipped.append((cube.id, f"tháp đã đủ {max_height} tầng"))
    return {"base": base, "steps": steps, "skipped": skipped, "reason": ""}


def should_continue(result) -> bool:
    """Sau một bước thất bại: làm tiếp cube kế (lỗi riêng của cube) hay dừng cả lô (lỗi hệ thống)."""
    if result.get("ok"):
        return True
    return result.get("code") in SKIPPABLE_CODES


def summarize(kind, rows, skipped=()):
    """rows: [(mô tả bước, result dict)] -> câu tổng kết tiếng Việt."""
    done = [text for text, r in rows if r.get("ok")]
    failed = [(text, r.get("reply") or r.get("code") or "lỗi") for text, r in rows if not r.get("ok")]
    parts = [f"{kind}: xong {len(done)}/{len(rows)} bước."]
    if done:
        parts.append("Đã xong: " + "; ".join(done) + ".")
    if failed:
        parts.append("Chưa xong: " + "; ".join(f"{t} ({str(why)[:120]})" for t, why in failed) + ".")
    if skipped:
        parts.append("Bỏ qua: " + "; ".join(f"cube {i} ({why})" for i, why in skipped) + ".")
    return " ".join(parts)
