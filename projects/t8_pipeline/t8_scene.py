#!/usr/bin/env python3
# coding: utf-8
"""T8 scene memory + label normalization for active perception.

Mục đích: cho LLM-agent nhớ vật đã thấy ở góc nào để:
- tìm cube đỏ -> gắp -> tìm cube id 1 (khác vật đã gắp) -> đặt lên
- tránh nhầm source/target khi cả hai đều là "cube" chung chung.

Không chạm hardware. TTL ngắn để tránh dùng vị trí cũ sau khi tay đã di chuyển.
"""
import re
import time
import unicodedata


def _norm_nodau(s):
    s = (s or "").strip().lower()
    s = re.sub(r"\s+", " ", s).replace("đ", "d")
    return "".join(c for c in unicodedata.normalize("NFD", s)
                   if unicodedata.category(c) != "Mn")

# Canonical labels refer to semantic cube IDs from ROS perception.
# Pixel contours and placement-bin numbers do not establish these IDs.
CUBE_ID_LABELS = {f"cube_{i}" for i in (1, 2, 3, 4)}
CUBE_ID_ALIASES = {
    "cube_id_1": "cube_1", "cube_id_2": "cube_2",
    "cube_id_3": "cube_3", "cube_id_4": "cube_4",
    "cubeid1": "cube_1", "cubeid2": "cube_2",
    "cubeid3": "cube_3", "cubeid4": "cube_4",
}

# "cube id 1" / "khối số 1" / "cube số 1" -> cube_1 (normalize trước khi tra VISION_LABELS)
ID_PATTERNS = [
    (re.compile(r"\b(?:cube|khoi|cuc)[_\s]*(?:(?:id|so)[_\s]*)?([1-4])\b"),
     lambda m: f"cube_{m.group(1)}"),
]


def normalize_cube_label(raw):
    """Chuẩn hoá nhãn vật về canonical. Trả None nếu không phải cube/vật rõ."""
    if not raw:
        return None
    t = _norm_nodau(str(raw))
    for pat, fmt in ID_PATTERNS:
        m = pat.search(t)
        if m:
            return fmt(m)
    # Gọi cube bằng hình rác trên mặt ("cube hình cục pin đã qua sử dụng" -> cube_3).
    try:
        from cube_identity import trash_face_cube_id
    except ImportError:
        try:
            from .cube_identity import trash_face_cube_id  # type: ignore
        except ImportError:
            trash_face_cube_id = None
    if trash_face_cube_id is not None:
        face_id = trash_face_cube_id(t, need_cube_noun=True)
        if face_id is not None:
            return f"cube_{face_id}"
    # alias dính liền
    compact = t.replace(" ", "").replace("_", "")
    for alias, canon in CUBE_ID_ALIASES.items():
        if alias.replace("_", "") == compact:
            return canon
    return None


def detection_base_label(label):
    """Preserve semantic IDs; never substitute a generic contour label."""
    return CUBE_ID_ALIASES.get(label, label)


class SceneMemory:
    """Blackboard ngắn hạn: label -> {box, center, source, conf, j1_deg, t}."""

    def __init__(self, ttl=15.0):
        self.ttl = ttl
        self._items = {}

    def remember(self, label, det, j1_deg=None):
        self._items[label] = {
            "box": list(det.get("box", [])),
            "center": list(det.get("center", [])),
            "source": det.get("source", ""),
            "conf": det.get("conf", 0),
            "corners": det.get("corners"),
            "j1_deg": j1_deg,
            "t": time.monotonic(),
        }

    def get(self, label):
        item = self._items.get(label)
        if item is None:
            return None
        if time.monotonic() - item.get("t", 0) > self.ttl:
            self._items.pop(label, None)
            return None
        return item

    def get_exclude_for(self, label):
        """Pixel exclusion hint for the same camera view; not semantic identity."""
        best = None
        for other_label, item in list(self._items.items()):
            if other_label == label:
                continue
            if time.monotonic() - item.get("t", 0) > self.ttl:
                continue
            if best is None or item["t"] > best["t"]:
                best = item
        if best is None:
            return None
        return {"center": best["center"], "box": best["box"]}

    def clear(self):
        self._items.clear()
