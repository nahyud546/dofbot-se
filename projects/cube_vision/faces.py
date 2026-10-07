"""Mặt cube: tiền nghiệm kích thước, phân vai quad và mở rộng mảnh hình in thành mặt đủ.

Điểm DINO không phân biệt được "mặt đủ" với "mảnh hình in" (DB tham chiếu là cả
mặt), nên kích thước/hình học quyết định quad có được làm đối tượng hay không:
  FACE   cạnh trong khoảng kỳ vọng của mặt 30 mm  -> có thể là mặt cube
  PART   nhỏ hơn rõ rệt (mảnh hình in)             -> chỉ là mỏ neo danh tính, phải mở rộng
  NOISE  quá nhỏ                                   -> bỏ
Mảnh PART được thử mở rộng bằng `grow_face`; không mở rộng được thì KHÔNG tạo đối
tượng (nhãn của nó chỉ là bằng chứng trọng số thấp).
Thuần numpy/cv2, không phụ thuộc ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

FACE_EDGE_M = 0.030
FACE, PART, NOISE = "FACE", "PART", "NOISE"
REF_WIDTH = 640.0
# Khi chưa có pose camera: cạnh mặt đã đo trên khung 640 px ở các pose đã dùng (84-130 px).
FALLBACK_SIDE_PX = (78.0, 170.0)
PART_MIN_FRACTION = 0.12     # cạnh mảnh / cạnh mặt kỳ vọng thấp nhất còn đáng làm mỏ neo
FACE_MIN_FRACTION = 0.70


@dataclass(frozen=True)
class SidePrior:
    lo: float
    hi: float
    source: str

    @property
    def nominal(self) -> float:
        return math.sqrt(self.lo * self.hi)


def fallback_prior(image_width: float = REF_WIDTH) -> SidePrior:
    scale = float(image_width) / REF_WIDTH
    return SidePrior(FALLBACK_SIDE_PX[0] * scale, FALLBACK_SIDE_PX[1] * scale, "fallback")


def expected_side_prior(base_T_camera, K, table_top_z: float, layers=range(4),
                        image_width: float = REF_WIDTH, slack: float = 0.30) -> SidePrior:
    """Khoảng cạnh pixel của mặt trên 30 mm ở các tầng, từ pose camera (base) và K.

    Độ sâu dọc trục quang tới mặt phẳng z_n qua tâm ảnh; cạnh ≈ fx*0.03/độ sâu.
    Camera không nhìn xuống bàn -> tiền nghiệm dự phòng.
    """
    try:
        T = np.asarray(base_T_camera, float).reshape(4, 4)
        K = np.asarray(K, float).reshape(3, 3)
    except (TypeError, ValueError):
        return fallback_prior(image_width)
    if not np.isfinite(T).all():
        return fallback_prior(image_width)
    axis, origin = T[:3, 2], T[:3, 3]
    sides = []
    for layer in layers:
        z = float(table_top_z) + FACE_EDGE_M * layer
        if axis[2] >= -0.2:
            continue
        depth = (z - origin[2]) / axis[2]
        if depth > 0.05:
            sides.append(K[0, 0] * FACE_EDGE_M / depth)
    if not sides:
        return fallback_prior(image_width)
    return SidePrior(min(sides) * (1.0 - slack), max(sides) * (1.0 + slack), "geometry")


def quad_side(quad) -> float:
    """Cạnh vuông tương đương (căn diện tích) của một quad."""
    return math.sqrt(abs(cv2.contourArea(np.asarray(quad, np.float32).reshape(-1, 2))))


def classify_quad(quad, prior: SidePrior) -> str:
    side = quad_side(quad)
    if side >= FACE_MIN_FRACTION * prior.lo:
        return FACE
    if side >= PART_MIN_FRACTION * prior.hi:
        return PART
    return NOISE


def _rotated_square(centre, side: float, angle_rad: float):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    half = side / 2.0
    pts = np.array([[-half, -half], [half, -half], [half, half], [-half, half]], np.float32)
    rot = np.array([[c, -s], [s, c]], np.float32)
    return pts @ rot.T + np.asarray(centre, np.float32)


def _dominant_angle(quad) -> float:
    """Góc cạnh trội của mảnh, mod 90° (rad)."""
    rect = cv2.minAreaRect(np.asarray(quad, np.float32).reshape(-1, 2))
    return math.radians(rect[2] % 90.0)


def grow_candidates(bgr, anchor_quad, prior: SidePrior, *, k: int = 3, white_s_max: int = 70,
                    white_v_min: int = 140, min_white: float = 0.65, min_edge: float = 0.30,
                    max_overflow: float = 0.25):
    """Tối đa k giả thuyết mặt đủ quanh mảnh `anchor_quad`, [(quad 4x2, điểm, chi tiết)] giảm dần.

    Giả thuyết: hình vuông cạnh trong [lo, hi] của tiền nghiệm, xoay theo cạnh trội của
    mảnh (±vài độ), tâm lệch quanh mỏ neo. Điểm = độ trắng của vành giữa mỏ neo và biên
    giả thuyết + độ ủng hộ cạnh dọc biên (mặt trắng trên thảm trắng ít cạnh nên chỉ cần
    một phần biên có cạnh). Cho phép giả thuyết vượt mép ảnh tối đa `max_overflow` cạnh.
    """
    image = np.asarray(bgr)
    if image.ndim != 3:
        return []
    h, w = image.shape[:2]
    anchor = np.asarray(anchor_quad, np.float32).reshape(4, 2)
    a_side = quad_side(anchor)
    if a_side < 4:
        return []
    centre0 = anchor.mean(axis=0)
    base_angle = _dominant_angle(anchor)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] <= white_s_max) & (hsv[:, :, 2] >= white_v_min)).astype(np.uint8)
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    # Bản đồ ủng hộ cạnh liên tục (1 trên cạnh Canny, giảm tuyến tính tới 0 ở 6 px) để leo đồi có gradient.
    dist = cv2.distanceTransform((cv2.Canny(gray, 30, 100) == 0).astype(np.uint8), cv2.DIST_L2, 3)
    edges = np.clip(1.0 - dist / 6.0, 0.0, 1.0).astype(np.float32)
    anchor_mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(anchor_mask, np.rint(anchor).astype(np.int32), 1)
    anchor_mask = cv2.dilate(anchor_mask, np.ones((5, 5), np.uint8))

    sides = np.linspace(max(prior.lo, a_side * 1.4), prior.hi, 4)
    def valid(quad):
        return not (np.any(quad < -max_overflow * quad_side(quad)) or
                    np.any(quad[:, 0] > w + max_overflow * quad_side(quad)) or
                    np.any(quad[:, 1] > h + max_overflow * quad_side(quad)) or
                    any(cv2.pointPolygonTest(quad.astype(np.float32), tuple(map(float, p)), False) < 0
                        for p in (*anchor, centre0)))

    def evaluate(params):
        cx, cy, side, angle = params
        quad = _rotated_square((cx, cy), side, angle)
        if not valid(quad):
            return None
        result = _score(quad, white, edges, anchor_mask, (h, w))
        return None if result is None else (result[0], quad, result[1], result[2], result[3], params)

    found = []
    for side in sides:
        if side <= a_side * 1.2:
            continue
        shifts = np.linspace(-0.25, 0.25, 5) * side
        for dtheta in (-0.10, 0.0, 0.10):
            for dx in shifts:
                for dy in shifts:
                    result = evaluate((centre0[0] + dx, centre0[1] + dy, float(side), base_angle + dtheta))
                    if result is not None and result[2] >= min_white:
                        found.append(result)
    found.sort(key=lambda item: -item[0])
    # Lưới thô lệch biên vài px nên chỉ có một phần biên chạm cạnh: leo đồi quanh vài ứng viên
    # đầu (tâm, cạnh, góc) để biên bám đúng cạnh ảnh.
    refined = []
    for start in found[:8]:
        best = start
        step_xy, step_side, step_angle = 0.03 * start[5][2], 0.03 * start[5][2], math.radians(1.5)
        for _ in range(14):
            cx, cy, side, angle = best[5]
            moved = False
            for params in ((cx + step_xy, cy, side, angle), (cx - step_xy, cy, side, angle),
                           (cx, cy + step_xy, side, angle), (cx, cy - step_xy, side, angle),
                           (cx, cy, side + step_side, angle), (cx, cy, side - step_side, angle),
                           (cx, cy, side, angle + step_angle), (cx, cy, side, angle - step_angle)):
                if not (prior.lo * 0.9 <= params[2] <= prior.hi * 1.1):
                    continue
                result = evaluate(params)
                if result is not None and result[0] > best[0] + 1e-4:
                    best, moved = result, True
            if not moved:
                step_xy, step_side, step_angle = step_xy / 2, step_side / 2, step_angle / 2
                if step_xy < 0.5:
                    break
        refined.append(best)
    refined.sort(key=lambda item: -item[0])
    chosen = []
    for score, quad, coverage, edge, third, _ in refined:
        if coverage < min_white or edge < min_edge or third < 0.25:
            continue
        if any(_overlap(quad, old[0], (h, w)) > 0.6 for old in chosen):
            continue
        chosen.append((quad, float(score), {"white": round(coverage, 3), "edge": round(edge, 3),
                                            "side": round(quad_side(quad), 1)}))
        if len(chosen) >= k:
            break
    return chosen


def refine_corners(bgr, quad, anchor_quad, *, max_shift: float = 0.25, steps=(6.0, 3.0, 1.5, 0.75)):
    """Chỉnh từng góc của mặt dựng (không còn ép hình vuông) để biên bám cạnh ảnh.

    Mặt trên nhìn xiên là hình thang nên hình vuông chỉ gần đúng; PnP hai mặt cần góc chính
    xác vài px. Leo đồi theo tọa độ từng góc, điểm = _score (độ trắng vành + ủng hộ cạnh từng
    cạnh), giữ lồi, giữ mỏ neo bên trong, góc không đi quá `max_shift` cạnh khỏi vị trí đầu.
    """
    image = np.asarray(bgr)
    h, w = image.shape[:2]
    start = order_quad(quad).astype(np.float64)
    anchor = np.asarray(anchor_quad, np.float32).reshape(4, 2)
    side = quad_side(start)
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    dist = cv2.distanceTransform((cv2.Canny(gray, 30, 100) == 0).astype(np.uint8), cv2.DIST_L2, 3)
    edges = np.clip(1.0 - dist / 6.0, 0.0, 1.0).astype(np.float32)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] <= 70) & (hsv[:, :, 2] >= 140)).astype(np.uint8)
    amask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(amask, np.rint(anchor).astype(np.int32), 1)
    amask = cv2.dilate(amask, np.ones((5, 5), np.uint8))
    area0 = abs(cv2.contourArea(start.astype(np.float32)))

    def evaluate(q):
        poly = q.astype(np.float32)
        if not cv2.isContourConvex(poly.reshape(-1, 1, 2)):
            return None
        area = abs(cv2.contourArea(poly))
        if not 0.6 * area0 <= area <= 1.7 * area0:
            return None
        if any(cv2.pointPolygonTest(poly, tuple(map(float, p)), False) < 0 for p in anchor):
            return None
        r = _score(q, white, edges, amask, (h, w))
        return None if r is None else r[0]

    best_q, best = start.copy(), evaluate(start)
    if best is None:
        return start.astype(np.float32)
    for step in steps:
        improved = True
        while improved:
            improved = False
            for i in range(4):
                for dx, dy in ((step, 0), (-step, 0), (0, step), (0, -step)):
                    q = best_q.copy()
                    q[i] += (dx, dy)
                    if np.linalg.norm(q[i] - start[i]) > max_shift * side:
                        continue
                    score = evaluate(q)
                    if score is not None and score > best + 1e-4:
                        best_q, best, improved = q, score, True
    return best_q.astype(np.float32)


def snap_to_square_projection(quad, K, dist=None, edge_m: float = FACE_EDGE_M, max_move_px: float = 8.0):
    """Chiếu lại 4 góc về dạng ảnh của một hình vuông 30 mm (PnP rồi projectPoints).

    Góc chỉnh tay bằng cạnh ảnh có nhiễu nên PnP một mặt báo lỗi chiếu lại > 3 px và bỏ mặt;
    ảnh của một hình vuông thật luôn khớp chính xác. Trả None nếu quad xa mọi ảnh hợp lệ
    (di chuyển góc > max_move_px).
    """
    K = np.asarray(K, np.float64).reshape(3, 3)
    D = np.zeros(5) if dist is None else np.asarray(dist, np.float64)
    image = order_quad(quad).astype(np.float64)
    h = edge_m / 2.0
    face = np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]], np.float64)
    best = None
    for shift in range(4):
        obj = np.roll(face, shift, axis=0)
        for flip in (False, True):
            pts = obj[::-1] if flip else obj
            try:
                ok, rvec, tvec = cv2.solvePnP(pts, image, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            except cv2.error:
                continue
            if not ok or tvec[2, 0] <= 0:
                continue
            proj = cv2.projectPoints(pts, rvec, tvec, K, D)[0].reshape(-1, 2)
            err = float(np.max(np.linalg.norm(proj - image, axis=1)))
            if best is None or err < best[0]:
                best = (err, proj)
    if best is None or best[0] > max_move_px:
        return None
    return order_quad(best[1])


def grow_face(bgr, anchor_quad, prior: SidePrior, **kw):
    """Giả thuyết hình học tốt nhất (hoặc None); xem grow_candidates."""
    top = grow_candidates(bgr, anchor_quad, prior, k=1, **kw)
    return top[0] if top else None


def _score(quad, white, edges, anchor_mask, shape):
    h, w = shape
    x, y, bw, bh = cv2.boundingRect(np.rint(quad).astype(np.int32))
    x0, y0, x1, y1 = max(0, x), max(0, y), min(w, x + bw), min(h, y + bh)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    local = np.rint(quad - [x0, y0]).astype(np.int32)
    size = (y1 - y0, x1 - x0)
    inner = np.zeros(size, np.uint8)
    cv2.fillConvexPoly(inner, local, 1)
    ring = inner.astype(bool) & ~anchor_mask[y0:y1, x0:x1].astype(bool)
    if np.count_nonzero(ring) < 100:
        return None
    coverage = float(np.mean(white[y0:y1, x0:x1][ring]))
    # Độ ủng hộ cạnh theo từng cạnh của giả thuyết; cạnh ra ngoài ảnh không tính.
    per_side = []
    for i in range(4):
        a, b = local[i], local[(i + 1) % 4]
        side_mask = np.zeros(size, np.uint8)
        cv2.line(side_mask, tuple(map(int, a)), tuple(map(int, b)), 1, 3)
        sel = side_mask.astype(bool)
        if np.count_nonzero(sel) >= 10:
            per_side.append(float(np.mean(edges[y0:y1, x0:x1][sel])))
    if len(per_side) < 2:
        return None
    per_side.sort(reverse=True)
    # Mặt trắng trên thảm trắng: cho phép một cạnh thiếu cạnh (bóng/sát nền) nhưng
    # các cạnh còn lại phải có đường biên thật.
    edge = float(np.mean(per_side[:3]))
    third = per_side[min(2, len(per_side) - 1)]
    return coverage + 0.6 * edge + 0.3 * third, coverage, edge, third


def order_quad(points):
    """4 góc theo thứ tự TL, TR, BR, BL."""
    pts = np.asarray(points, np.float32).reshape(4, 2)
    total, diff = pts.sum(axis=1), np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(total)], pts[np.argmin(diff)],
                     pts[np.argmax(total)], pts[np.argmax(diff)]], np.float32)


def rectify(bgr, quad, size: int = 224):
    dst = np.float32([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]])
    matrix = cv2.getPerspectiveTransform(order_quad(quad), dst)
    return cv2.warpPerspective(np.asarray(bgr), matrix, (size, size),
                               borderMode=cv2.BORDER_REPLICATE)


@dataclass
class FaceProposal:
    quad: np.ndarray
    label: str | None
    score: float
    margin: float
    source: str                 # "face" | "grown"
    anchor_label: str | None = None
    detail: dict | None = None


def _overlap(a, b, shape, containment: bool = False) -> float:
    """IoU của hai quad (containment=True: giao / diện tích nhỏ hơn)."""
    ma = np.zeros(shape, np.uint8)
    mb = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(ma, np.rint(a).astype(np.int32), 1)
    cv2.fillConvexPoly(mb, np.rint(b).astype(np.int32), 1)
    denom = (min(np.count_nonzero(ma), np.count_nonzero(mb)) if containment
             else np.count_nonzero(ma | mb))
    return np.count_nonzero(ma & mb) / denom if denom else 0.0


def propose_faces(bgr, quads, prior: SidePrior, trash, *, anchor_score: float = 0.35,
                  max_anchors: int = 8, min_score: float = 0.45, min_margin: float = 0.05,
                  top_k: int = 3, start: int = 0, max_parts: int = 30, K=None, dist=None):
    """Mặt cube đáng tin từ danh sách quad ứng viên.

    - quad FACE: giữ nguyên (nhận bởi DINO ở nơi gọi).
    - quad PART có nhãn DINO >= anchor_score làm mỏ neo -> grow_face -> DINO chạy lại trên
      mặt đủ đã nắn phẳng; nhãn của mặt đủ thắng nhãn mảnh. Mặt đủ phải có nhãn rác hợp lệ
      và điểm >= min_score.
    Mảnh không dựng được mặt đủ KHÔNG trả về (không tạo đối tượng).
    `trash.match_many(crops) -> [(label, score, detail)]`.
    """
    shape = np.asarray(bgr).shape[:2]
    parts = sorted((q for q in quads if classify_quad(q, prior) == PART), key=lambda q: -quad_side(q))
    parts = parts[:max_parts]
    if not parts:
        return []
    anchors = trash.match_many([rectify(bgr, q) for q in parts], refine=False)
    # Mỏ neo chắc nhất (điểm DINO cao) trước; `start` xoay vòng để các lần gọi sau thử mỏ neo khác.
    ranked = sorted(range(len(parts)), key=lambda i: -float(anchors[i][1] or 0.0))
    ranked = [i for i in ranked if anchors[i][0] and anchors[i][1] >= anchor_score]
    if ranked:
        shift = start % len(ranked)
        ranked = ranked[shift:] + ranked[:shift]
    parts = [parts[i] for i in ranked]
    anchors = [anchors[i] for i in ranked]
    out: list[FaceProposal] = []
    used = 0
    for quad, (label, score, detail) in zip(parts, anchors):
        if used >= max_anchors:
            break
        centre = tuple(map(float, np.asarray(quad, np.float32).reshape(4, 2).mean(axis=0)))
        if any(cv2.pointPolygonTest(p.quad.astype(np.float32), centre, False) >= 0 for p in out):
            continue
        if not label or score < anchor_score:
            continue
        used += 1
        cands = grow_candidates(bgr, quad, prior, k=top_k)
        if not cands:
            continue
        # Mỗi giả thuyết thử cả bản hình vuông lẫn bản đã chỉnh góc; DINO chọn bản đúng hơn
        # (chỉnh góc giúp mặt nhìn xiên nhưng đôi khi kéo góc ra khỏi mặt).
        cands = [c for c in cands for _ in (0, 1)]
        faces_ = []
        for idx, c in enumerate(cands):
            square = order_quad(c[0])
            if idx % 2 == 0:
                faces_.append(square)
                continue
            refined = refine_corners(bgr, square, quad)
            if K is not None:
                refined = snap_to_square_projection(refined, K, dist)
                if refined is None:
                    refined = square
            faces_.append(refined)
        verdicts = trash.match_many([rectify(bgr, f) for f in faces_], refine=False)
        best = None
        for (g_quad, g_geo, g_info), face, (g_label, g_score, g_detail) in zip(cands, faces_, verdicts):
            margin = float((g_detail or {}).get('margin', 0.0))
            # Mặt dựng phải cỡ mặt thật: quad nhỏ hơn (mặt bên bị nhìn xiên, chi tiết hình in)
            # cho PnP ra khoảng cách sai gấp bội.
            if quad_side(face) < 0.9 * prior.lo:
                continue
            if (g_label and g_score >= min_score and margin >= min_margin and
                    (best is None or g_score + margin > best[0] + float((best[3] or {}).get('margin', 0.0)))):
                best = (g_score, face, g_label, g_detail, g_info)
        if best is None:
            continue
        g_score, face, g_label, g_detail, g_info = best
        out.append(FaceProposal(face, g_label, float(g_score),
                                float((g_detail or {}).get("margin", 0.0)), "grown",
                                label, {**g_info, "anchor_score": round(float(score), 3)}))
    # Mặt nằm trọn trong mặt khác (điểm cao hơn) chỉ là chi tiết của mặt đó.
    # Điểm DINO ngang nhau thì mặt lớn hơn (đủ hơn) thắng mảnh nhỏ nằm trong nó.
    out.sort(key=lambda p: -(p.score + p.margin + 0.5 * quad_side(p.quad) / prior.nominal))
    kept: list[FaceProposal] = []
    for proposal in out:
        if any(_overlap(proposal.quad, other.quad, shape, containment=True) > 0.6 for other in kept):
            continue
        kept.append(proposal)
    out = kept
    return out


def merge_duplicate_ids(objects):
    """Mỗi ID tối đa một đối tượng (có đúng 4 cube): giữ cái hợp lệ nhất.

    objects: list dict có "cube_id", "rank" (cao hơn = hợp lệ hơn) và tuỳ chọn "quad".
    Trả (giữ, gộp) — phần gộp chỉ còn là bằng chứng.
    """
    kept, merged = {}, []
    for obj in sorted(objects, key=lambda o: -float(o.get("rank", 0.0))):
        cid = obj.get("cube_id")
        if cid is None or cid not in kept:
            kept[cid if cid is not None else id(obj)] = obj
        else:
            merged.append(obj)
    return list(kept.values()), merged
