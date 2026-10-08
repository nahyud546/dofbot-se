"""Vật khác cube ĐI THEO khi bị dời: định vị lại trên mặt bàn từ mặt nạ của vật trong MỘT khung, gộp hai camera.

Hình dạng vật (trụ/hộp, bề ngang, chiều cao, điểm 3D) do lượt quét của camera tay đo một lần. Sau đó chỉ cần biết vật
đang ĐỨNG Ở ĐÂU trên mặt bàn, và việc đó một camera bất kỳ đã biết pose làm được từ mặt nạ của vật:
  - chiếu viền mặt nạ xuống mặt bàn: được đáy vật + "bóng" kéo dài ra xa camera;
  - hướng từ chân camera tới vật (`bearing`) và mép GẦN của vết chiếu là phần tin được; bề ngang đo ở mép gần;
  - tâm vật = mép gần + nửa bề ngang dọc theo hướng nhìn.
Hai camera cùng thấy vật (iPhone + camera tay) thì giao hai đường ngắm: như stereo đường đáy rộng, không cần khớp
đặc trưng giữa hai ảnh. Thuần toán: mặt nạ do bên gọi cấp (YOLOE); không ROS, không mô hình.
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .frames import CameraModel, invert

NEAR_BAND_M = 0.03            # dải sát mép gần dùng để đo bề ngang
MAX_RANGE_M = 1.5             # tia chạm bàn xa hơn mức này (gần ngang) thì bỏ
MATCH_GATE_M = 0.40           # vật chỉ được coi là "dời tới đây" trong tầm này
MOVED_M = 0.03                # lệch dưới mức này coi như đứng yên (đo thật: iPhone báo tâm cốc đứng yên dao động 2–3 cm)
CUBE_GATE_M = 0.05            # quan sát có tâm cách một cube dưới mức này là CHÍNH cube đó, không phải vật khác
CONFIRM_SIGHTINGS = 3         # phải thấy vật ở chỗ mới chừng này lần...
CONFIRM_WINDOW_S = 2.5        # ...trong chừng này giây...
CONFIRM_SPREAD_M = 0.03       # ...và các lần đó thống nhất trong mức này, mới dời vật
ANCHOR_M = 0.06               # camera phải từng thấy vật trong tầm này quanh CHỖ ĐANG GHI thì mới được dời vật đó
SIZE_RATIO = (0.5, 2.0)       # bề ngang quan sát / bề ngang đã đo phải trong khoảng này mới là cùng một vật
MIN_STEREO_ANGLE_DEG = 20.0   # hai đường ngắm lệch nhau ít hơn mức này thì giao điểm không ổn: lấy trung bình
ROUND_LABELS = ("cup", "mug", "bottle", "can")


def observe_mask(camera: CameraModel, a_T_optical, mask, table_z: float, label: str = "", confidence: float = 1.0):
    """Một vật nhìn từ một camera -> quan sát trên mặt bàn, hoặc None khi mặt nạ không chiếu được xuống bàn.

    Trả {"label", "confidence", "eye" (2,), "bearing" (2,) đơn vị, "near_m", "width_m", "centre" (2,), "side_ok",
    "near_ok", "stamp"}. side_ok False: mặt nạ chạm mép trái/phải ảnh (bề ngang và hướng không tin được).
    near_ok False: chạm mép dưới (không thấy chân vật: khoảng cách không tin được).
    """
    mask = (np.asarray(mask) > 0).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(float)
    if len(contour) < 12:
        return None
    h, w = mask.shape[:2]
    T = np.asarray(a_T_optical, float)
    step = max(1, len(contour) // 240)
    hits = []
    for u, v in contour[::step]:
        ray = T[:3, :3] @ camera.ray(u, v)
        if ray[2] >= -1e-6:
            continue                                               # tia không đi xuống: không chạm bàn
        s = (float(table_z) - T[2, 3]) / ray[2]
        point = T[:2, 3] + s * ray[:2]
        if s > 0 and np.linalg.norm(point - T[:2, 3]) < MAX_RANGE_M:
            hits.append(point)
    if len(hits) < 8:
        return None
    hits, eye = np.array(hits), T[:2, 3].copy()
    rel = hits - eye
    bearing = rel.mean(axis=0)
    bearing = bearing / np.linalg.norm(bearing)
    perp = np.array([-bearing[1], bearing[0]])
    r, t = rel @ bearing, rel @ perp
    near = float(np.percentile(r, 2))
    band = r <= near + NEAR_BAND_M
    for _ in range(2):                                             # dải đo bề ngang sâu bằng nửa bề ngang (đáy tròn)
        lo, hi = float(t[band].min()), float(t[band].max())
        band = r <= near + max(NEAR_BAND_M, (hi - lo) / 2.0)
    width, mid = hi - lo, (lo + hi) / 2.0
    if width <= 0.005:
        return None
    # Chân vật bị cắt? Lấy các điểm viền chiếu GẦN camera nhất: nếu chúng nằm sát mép ảnh thì chưa thấy chân vật.
    sampled = contour[::step]
    kept = [k for k, (u, v) in enumerate(sampled) if (T[:3, :3] @ camera.ray(u, v))[2] < -1e-6]
    on_edge = lambda u, v: u <= 2 or u >= w - 3 or v <= 2 or v >= h - 3  # noqa: E731
    nearest = [sampled[kept[k]] for k in np.argsort(r)[: max(3, len(r) // 20)] if k < len(kept)]
    near_ok = not any(on_edge(u, v) for u, v in nearest)
    in_band = [sampled[kept[k]] for k in np.nonzero(band)[0] if k < len(kept)]
    side_ok = near_ok and not any(on_edge(u, v) for u, v in in_band)   # bề ngang đo ở dải gần: dải đó phải trọn vẹn
    bearing_to_centre = bearing * (near + width / 2.0) + perp * mid
    centre = eye + bearing_to_centre
    # Hướng ngắm: chân vật thấy trọn thì ngắm vào tâm vừa tính; chân bị cắt thì dùng hướng trung bình của vết chiếu
    # (bóng kéo dài dọc hướng nhìn nên không làm lệch hướng). Hướng chỉ tin được khi vật không bị cắt ở hai bên.
    bearing_ok = not any(u <= 2 or u >= w - 3 for u, _ in sampled)
    sight = bearing_to_centre / np.linalg.norm(bearing_to_centre) if near_ok else bearing
    return {"label": str(label), "confidence": float(confidence), "eye": eye, "bearing": sight, "near_m": near,
            "width_m": float(width), "centre": centre, "side_ok": bool(side_ok), "near_ok": bool(near_ok),
            "bearing_ok": bool(bearing_ok), "near_point": eye + bearing * near + perp * mid, "stamp": time.time()}


def fuse(a: dict, b: dict):
    """Tâm vật từ hai quan sát của hai camera: giao hai đường ngắm (chân camera -> vật) trên mặt bàn.

    Đường ngắm chính xác hơn khoảng cách nhiều (khoảng cách dựa vào mép gần + giả thiết đáy), nên hai camera nhìn từ
    hai hướng khác nhau cho tâm tốt hơn hẳn một camera. Hai hướng gần song song thì lấy trung bình hai tâm.
    Trả (tâm (2,), "stereo" | "average").
    """
    da, db = np.asarray(a["bearing"], float), np.asarray(b["bearing"], float)
    angle = np.degrees(np.arccos(np.clip(abs(float(da @ db)), 0.0, 1.0)))
    if angle >= MIN_STEREO_ANGLE_DEG and a.get("bearing_ok", True) and b.get("bearing_ok", True):
        A = np.column_stack([da, -db])
        s, _ = np.linalg.solve(A, np.asarray(b["eye"], float) - np.asarray(a["eye"], float))
        if s > 0:
            return np.asarray(a["eye"], float) + s * da, "stereo"
    # Hai camera nhìn ĐỐI DIỆN nhau (iPhone đặt trước mặt tay máy): hai đường ngắm gần trùng nên không giao được,
    # nhưng mỗi camera thấy một mép của vật: tâm nằm giữa hai mép gần, không cần giả thiết gì về bề sâu của vật.
    if float(da @ db) < 0 and a.get("near_ok", True) and b.get("near_ok", True) and "near_point" in a and "near_point" in b:
        return (np.asarray(a["near_point"], float) + np.asarray(b["near_point"], float)) / 2.0, "opposed"
    whole = [np.asarray(o["centre"], float) for o in (a, b) if o.get("near_ok", True)]
    if not whole:
        return None, "none"
    return np.mean(whole, axis=0), "average"


def line_gap(obs: dict, point) -> float:
    """Khoảng cách (m) từ một điểm trên mặt bàn tới đường ngắm của một quan sát."""
    rel = np.asarray(point, float)[:2] - np.asarray(obs["eye"], float)
    d = np.asarray(obs["bearing"], float)
    return float(abs(rel[0] * d[1] - rel[1] * d[0]))


def to_json(obs: dict) -> dict:
    return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in obs.items()}


def from_json(data: dict) -> dict:
    return {k: (np.asarray(v, float) if k in ("eye", "bearing", "centre", "near_point") else v) for k, v in data.items()}


def _compatible(item: dict, obs: dict) -> bool:
    label, seen = str(item.get("label", "")), str(obs.get("label", ""))
    if label and seen and label not in ("vật", "object") and seen not in ("vật", "object"):
        if (label in ROUND_LABELS) != (seen in ROUND_LABELS):
            return False                                           # vật tròn không biến thành hộp và ngược lại
    width = float(item.get("width_m") or 0.0)
    if width > 0 and obs.get("side_ok", True):
        ratio = obs["width_m"] / width
        if not SIZE_RATIO[0] <= ratio <= SIZE_RATIO[1]:
            return False
    return True


def not_cubes(observations, cubes) -> list:
    """Bỏ các quan sát thật ra là cube: bộ tách vật hay gọi cube là "box" (đo thật 2026-10-08: cube lục bị nhận là
    box 0,29 và kéo hộp bịch khăn giấy từ cách đó 16 cm về nằm chồng lên cube). cubes: tâm (x, y) các cube đã biết."""
    cubes = [np.asarray(c, float)[:2] for c in cubes]
    return [o for o in observations
            if all(np.linalg.norm(np.asarray(o["centre"], float) - c) > CUBE_GATE_M for c in cubes)]


class Follower:
    """Chỉ cho vật dời khi chỗ mới được thấy LẶP LẠI và thống nhất; tâm mới là trung vị của các lần thấy đó.

    Một mặt nạ sai của một khung (nhận nhầm vật khác, mặt nạ dính hai vật) không còn kéo vật đi, và vật đứng yên
    không còn nhảy theo nhiễu của từng khung.

    Camera còn phải BẮT ĐƯỢC vật tại chỗ đang ghi trước đã (`ANCHOR_M`), rồi mới được dời nó. Đo thật 2026-10-08:
    iPhone không tách được bịch khăn giấy ở chỗ của nó, nhưng gọi cube lục cách đó 16 cm là "box" đều đặn mọi khung,
    nên luật "thấy lặp lại" không chặn được: hộp bịch khăn vẫn bị kéo về cube. Vật mà camera này chưa từng thấy tại
    chỗ thì đứng yên (rồi chuyển xám), không đoán.

    `index` là khóa bất kỳ băm được: nhiều camera dùng chung một Follower thì dùng khóa (camera, chỉ số vật)."""

    def __init__(self):
        self.sightings = {}       # {khóa vật: [(thời điểm, tâm)]} các lần thấy vật ở CHỖ KHÁC chỗ đang ghi
        self.anchors = {}         # {khóa vật: tâm đang ghi lúc camera bắt được vật tại chỗ}

    def confirmed(self, index, centre, current, now: float | None = None):
        """Ghi một lần thấy vật `index` ở `centre`; trả tâm mới (trung vị) khi đã đủ xác nhận để dời, không thì None."""
        now = time.time() if now is None else float(now)
        centre, current = np.asarray(centre, float)[:2], np.asarray(current, float)[:2]
        gap = float(np.linalg.norm(centre - current))
        if gap <= ANCHOR_M:
            self.anchors[index] = current.copy()
        if gap <= MOVED_M:
            self.sightings.pop(index, None)                         # vẫn ở chỗ cũ: xóa mọi nghi ngờ đã tích
            return None
        anchor = self.anchors.get(index)
        if anchor is None or np.linalg.norm(anchor - current) > MOVED_M:
            return None                                             # chưa từng bắt được vật này tại chỗ đang ghi
        seen = [(t, c) for t, c in self.sightings.get(index, []) if now - t <= CONFIRM_WINDOW_S]
        if seen and seen[-1][0] == now:
            return None                                             # cùng một quan sát được đọc lại: không tính hai lần
        seen.append((now, centre))
        self.sightings[index] = seen
        if len(seen) < CONFIRM_SIGHTINGS:
            return None
        recent = np.array([c for _, c in seen[-CONFIRM_SIGHTINGS:]])
        median = np.median(recent, axis=0)
        if np.max(np.linalg.norm(recent - median, axis=1)) > CONFIRM_SPREAD_M:
            return None                                             # các lần thấy chưa thống nhất: chờ thêm
        self.sightings.pop(index, None)
        self.anchors[index] = median.copy()
        return median


def match(objects, observations) -> dict:
    """Ghép quan sát với vật đã biết: {chỉ số vật: quan sát}. Mỗi vật lấy quan sát hợp nhãn + cỡ GẦN chỗ cũ nhất, mỗi
    quan sát chỉ dùng một lần (cặp gần nhau ghép trước)."""
    pairs = []
    for index, item in enumerate(objects):
        for k, obs in enumerate(observations):
            if obs.get("near_ok", True) and _compatible(item, obs):
                gap = float(np.linalg.norm(np.asarray(obs["centre"], float) - np.asarray(item["centre"], float)[:2]))
                if gap <= MATCH_GATE_M:
                    pairs.append((gap, index, k))
    out, used = {}, set()
    for gap, index, k in sorted(pairs):
        if index not in out and k not in used:
            out[index] = observations[k]
            used.add(k)
    return out


def moved_copy(item: dict, centre, source: str, stamp: float | None = None) -> dict:
    """Bản sao của vật dời sang `centre` (x, y): đa giác đáy và các điểm 3D dời theo, hình dạng giữ nguyên."""
    centre = np.asarray(centre, float)[:2]
    delta = centre - np.asarray(item["centre"], float)[:2]
    out = dict(item)
    out["centre"] = centre
    out["polygon"] = np.asarray(item["polygon"], float) + delta
    points = np.asarray(item.get("points", np.zeros((0, 3))), float).reshape(-1, 3)
    out["points"] = points + np.r_[delta, 0.0] if len(points) else points
    out["stamp"] = time.time() if stamp is None else float(stamp)
    out["moved_by"] = source
    return out


def relocate(objects, observations, source: str, stamp: float | None = None, follower: Follower | None = None,
             cubes=(), keys=None):
    """Áp các quan sát của MỘT camera lên danh sách vật. Trả (danh sách vật mới, [chỉ số vật đã dời], {chỉ số: quan sát}).

    follower: có thì vật chỉ dời sau khi chỗ mới được xác nhận nhiều lần (nên dùng với luồng camera sống).
    cubes: tâm các cube đã biết; quan sát trùng chỗ cube bị bỏ. keys: khóa của từng vật trong follower (mặc định
    là chỉ số trong `objects`)."""
    matched = match(objects, not_cubes(observations, cubes))
    out, moved = list(objects), []
    now = time.time() if stamp is None else float(stamp)
    for index, obs in matched.items():
        current = np.asarray(objects[index]["centre"], float)[:2]
        if follower is not None:
            target = follower.confirmed(index if keys is None else keys[index], obs["centre"], current, now)
        else:
            target = obs["centre"] if np.linalg.norm(np.asarray(obs["centre"], float) - current) > MOVED_M else None
        if target is not None:
            out[index] = moved_copy(objects[index], target, source, stamp)
            moved.append(index)
        elif np.linalg.norm(np.asarray(obs["centre"], float) - current) <= MOVED_M:
            out[index] = dict(objects[index], seen_stamp=now)       # thấy vật ở đúng chỗ đang ghi: xác nhận còn đó
    return out, moved, matched
