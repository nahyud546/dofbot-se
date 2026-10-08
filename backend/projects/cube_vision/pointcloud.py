"""Nối các khung ảnh thành MỘT đám mây điểm 3D trong hệ world, rồi tách vật và hình dạng của nó.

Ý tưởng cốt lõi (như SLAM dựng point cloud): cùng một điểm trên vật xuất hiện ở nhiều khung chồng nhau; biết pose
camera của từng khung thì hai tia qua hai pixel khớp nhau cắt nhau tại vị trí 3D thật của điểm đó. Ở đây pose camera
KHÔNG phải ước lượng: tay máy cho sẵn (FK + hand-eye), nên đám mây có đơn vị mét thật trong hệ base ngay, và một vật
lớn hơn một khung vẫn ghép được vì mọi khung đều nằm trong cùng một hệ.

  1. `triangulate`: đặc trưng SIFT khớp giữa các cặp khung -> giao hai tia -> giữ điểm mà hai tia gần như cắt nhau.
  2. `objects`: bỏ điểm sát mặt bàn và điểm thuộc cube đã biết, gom cụm, mỗi cụm là một vật.
  3. `fit_shape`: cung tròn khớp tốt -> hình trụ (cốc, lon); không thì hộp chữ nhật bao các điểm.

Giới hạn: chỉ ra điểm ở chỗ có hoa văn (mặt trơn một màu không có đặc trưng), và chỉ ở mặt vật mà camera nhìn thấy
(camera tay nhìn từ phía đế ra): phía sau vật là suy ra từ hình (trụ) hoặc giả thiết (hộp sâu bằng ngang).
Thuần toán + OpenCV: không ROS, không mô hình học sâu.
"""
from __future__ import annotations

import itertools

import cv2
import numpy as np

MIN_BASELINE_M = 0.02         # hai khung phải cách nhau chừng này mới có thị sai
MAX_RAY_MISS_M = 0.004        # hai tia khớp phải đi qua nhau trong khoảng này
MIN_PARALLAX_DEG = 3.0
RATIO = 0.75                  # tỉ lệ Lowe: khớp tốt nhất phải hơn hẳn khớp nhì
MIN_HEIGHT_M = 0.012          # thấp hơn mức này coi là hình in trên thảm / nhiễu độ sâu
CLUSTER_CELL_M = 0.01
MIN_CLUSTER_POINTS = 25
MIN_CLUSTER_SHARE = 0.05      # cụm phải có ít nhất chừng này phần số điểm của cụm lớn nhất
MIN_CELL_POINTS = 3           # ô lưới gom cụm phải có chừng này điểm mới tính là thuộc vật
CUBE_KEEP_OUT_M = 0.035
CIRCLE_TOL_M = 0.004
CIRCLE_RADIUS_M = (0.015, 0.09)
MAX_STORED_POINTS = 400


def triangulate(frames) -> dict:
    """frames: [(ảnh BGR, CameraModel, a_T_optical)]. Trả {"points" (N,3) trong hệ a, "colours" (N,3) BGR,
    "pairs" (N,2) chỉ số hai khung sinh ra điểm, "miss_m" (N,)}."""
    sift = cv2.SIFT_create(4000, contrastThreshold=0.02)
    feats = []
    for image, camera, a_T_optical in frames:
        T = np.asarray(a_T_optical, float)
        keypoints, descriptors = sift.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), None)
        pixels = np.array([kp.pt for kp in keypoints]).reshape(-1, 2)
        rays = np.array([T[:3, :3] @ camera.ray(u, v) for u, v in pixels]).reshape(-1, 3)
        feats.append((pixels, descriptors, rays, T[:3, 3], image))
    matcher = cv2.BFMatcher()
    points, colours, pairs, misses = [], [], [], []
    for i, j in itertools.combinations(range(len(feats)), 2):
        (px_i, des_i, rays_i, eye_i, image_i), (_, des_j, rays_j, eye_j, _) = feats[i], feats[j]
        if des_i is None or des_j is None or len(des_i) < 2 or len(des_j) < 2:
            continue
        if np.linalg.norm(eye_i - eye_j) < MIN_BASELINE_M:
            continue
        good = [m[0] for m in matcher.knnMatch(des_i, des_j, k=2)
                if len(m) == 2 and m[0].distance < RATIO * m[1].distance]
        if len(good) < 8:
            continue
        qi = np.array([m.queryIdx for m in good])
        u, v = rays_i[qi], rays_j[[m.trainIdx for m in good]]
        w0 = eye_i - eye_j                                         # điểm gần nhau nhất của hai tia
        b = (u * v).sum(axis=1)
        denom = np.maximum(1.0 - b * b, 1e-9)
        d, e = u @ w0, v @ w0
        s, t = (b * e - d) / denom, (e - b * d) / denom
        on_i, on_j = eye_i + s[:, None] * u, eye_j + t[:, None] * v
        miss = np.linalg.norm(on_i - on_j, axis=1)
        parallax = np.degrees(np.arccos(np.clip(b, -1.0, 1.0)))
        ok = (s > 0.03) & (t > 0.03) & (miss < MAX_RAY_MISS_M) & (parallax > MIN_PARALLAX_DEG)
        if not ok.any():
            continue
        points.append(((on_i + on_j) / 2.0)[ok])
        at = np.clip(np.round(px_i[qi][ok]).astype(int), 0, [image_i.shape[1] - 1, image_i.shape[0] - 1])
        colours.append(image_i[at[:, 1], at[:, 0]])
        pairs.append(np.tile([i, j], (int(ok.sum()), 1)))
        misses.append(miss[ok])
    if not points:
        return {"points": np.zeros((0, 3)), "colours": np.zeros((0, 3), np.uint8), "pairs": np.zeros((0, 2), int),
                "miss_m": np.zeros(0)}
    return {"points": np.vstack(points), "colours": np.vstack(colours), "pairs": np.vstack(pairs),
            "miss_m": np.concatenate(misses)}


def fit_circle(xy, eye_xy=(0.0, 0.0), tol: float = CIRCLE_TOL_M, rounds: int = 300, seed: int = 0):
    """Đường tròn khớp nhiều điểm nhất (RANSAC) với mặt cong LỒI quay về `eye_xy`. Trả (tâm, bán kính, inlier) | None."""
    xy = np.asarray(xy, float).reshape(-1, 2)
    if len(xy) < 8:
        return None
    rng, best = np.random.default_rng(seed), None
    eye = np.asarray(eye_xy, float)
    for _ in range(rounds):
        a, b, c = xy[rng.choice(len(xy), 3, replace=False)]
        A = 2.0 * np.array([b - a, c - a])
        if abs(np.linalg.det(A)) < 1e-10:
            continue
        centre = np.linalg.solve(A, [b @ b - a @ a, c @ c - a @ a])
        radius = float(np.linalg.norm(a - centre))
        if not CIRCLE_RADIUS_M[0] <= radius <= CIRCLE_RADIUS_M[1]:
            continue
        inliers = np.abs(np.linalg.norm(xy - centre, axis=1) - radius) < tol
        # mặt nhìn thấy phải là nửa gần camera của hình trụ
        if np.linalg.norm(centre - eye) <= np.linalg.norm(xy[inliers].mean(axis=0) - eye):
            continue
        if best is None or inliers.sum() > best[2].sum():
            best = (centre, radius, inliers)
    if best is None:
        return None
    pts = xy[best[2]]                                              # tinh chỉnh bình phương tối thiểu trên inlier
    A = np.c_[2.0 * pts, np.ones(len(pts))]
    cx, cy, k = np.linalg.lstsq(A, (pts ** 2).sum(axis=1), rcond=None)[0]
    centre = np.array([cx, cy])
    radius = float(np.sqrt(max(k + cx * cx + cy * cy, 0.0)))
    if not CIRCLE_RADIUS_M[0] <= radius <= CIRCLE_RADIUS_M[1]:
        return best
    return centre, radius, np.abs(np.linalg.norm(xy - centre, axis=1) - radius) < tol


def widen_to_silhouette(xy, centre, radius, eye_xy=(0.0, 0.0)):
    """Chỉnh bán kính hình trụ theo BỀ NGANG của đám điểm khi nó rộng hơn cung tròn khớp được.

    Độ sâu tam giác hóa với đường đáy ngắn (2–5 cm) nhiễu vài mm DỌC hướng nhìn, làm cung bị ép dẹt và vòng tròn khớp
    ra nhỏ hơn thật (đo thật: cốc Ø ~8 cm khớp ra Ø 4,7 cm). Vị trí NGANG hướng nhìn thì chính xác, và mặt trước của
    hình trụ trải gần hết đường kính, nên nửa bề ngang là cận dưới tốt cho bán kính. Giữ mép gần của vòng tròn cũ.
    """
    eye = np.asarray(eye_xy, float)
    radial = centre - eye
    distance = float(np.linalg.norm(radial))
    radial = radial / distance
    tangent = np.array([-radial[1], radial[0]])
    t = (np.asarray(xy, float) - eye) @ tangent
    lo, hi = np.percentile(t, [2, 98])
    half = float(hi - lo) / 2.0
    if half <= radius or half > CIRCLE_RADIUS_M[1]:
        return centre, radius
    near = distance - radius                                       # mép gần camera: phần đo chắc nhất
    return eye + radial * (near + half) + tangent * float((lo + hi) / 2.0), half


def fit_shape(points, table_z: float, eye_xy=(0.0, 0.0)) -> dict:
    """Hình dạng của một cụm điểm: trụ nếu các điểm nằm trên một cung tròn, không thì hộp chữ nhật.

    Trả {"shape": "cylinder"|"box", "centre" (2,), "polygon" (N,2) đáy, "width_m", "height_m", "fit": mô tả}.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    xy = points[:, :2]
    height = float(np.percentile(points[:, 2], 97) - table_z)
    circle = fit_circle(xy, eye_xy)
    if circle is not None:
        centre, radius, inliers = circle
        angles = np.unwrap(np.sort(np.arctan2(*(xy[inliers] - centre).T[::-1])))
        arc = float(np.degrees(angles.max() - angles.min())) if len(angles) else 0.0
        if inliers.mean() >= 0.6 and arc >= 50.0:
            centre, radius = widen_to_silhouette(xy, centre, radius, eye_xy)
            ring = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
            polygon = centre + radius * np.c_[np.cos(ring), np.sin(ring)]
            return {"shape": "cylinder", "centre": centre, "polygon": polygon, "width_m": 2.0 * radius,
                    "height_m": height, "fit": f"cung tròn {arc:.0f}°, {inliers.mean() * 100:.0f}% điểm khớp trong "
                                               f"{CIRCLE_TOL_M * 1000:.0f} mm"}
    # Hộp: thử mọi hướng xoay, lấy hướng cho hộp NHỎ nhất, bề rộng theo phân vị 3–97 % để vài điểm lạc không kéo giãn
    # hộp. (Trục chính PCA không dùng được: đám điểm gần vuông thì trục chính quay tùy ý, hộp ra hình thoi.)
    best = None
    for angle in np.radians(np.arange(0.0, 90.0, 3.0)):
        axes = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
        along = xy @ axes.T
        lo, hi = np.percentile(along, 3, axis=0), np.percentile(along, 97, axis=0)
        area = float(np.prod(hi - lo))
        if best is None or area < best[0]:
            best = (area, axes, lo, hi)
    _, axes, lo, hi = best
    centre = ((lo + hi) / 2.0) @ axes
    half = (hi - lo) / 2.0
    box = np.array([centre + (a * half[0]) * axes[0] + (b * half[1]) * axes[1]
                    for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    w, h = float(2 * half[0]) * 1000.0, float(2 * half[1]) * 1000.0
    return {"shape": "box", "centre": centre, "polygon": box, "width_m": max(w, h) / 1000.0,
            "height_m": height, "fit": f"hộp bao phần nhìn thấy {max(w, h):.0f} x {min(w, h):.0f} mm"}


def objects(cloud: dict, table_z: float, keep_out=(), inside=None, eye_xy=(0.0, 0.0)) -> list:
    """Tách vật từ đám mây: [{label, shape, centre, polygon, width_m, height_m, fit, n_points, n_views, points,
    colours}], vật nhiều điểm trước. keep_out: tâm (x, y) các cube đã biết; inside(xy) -> bool: vùng world."""
    points, colours, pairs = cloud["points"], cloud["colours"], cloud["pairs"]
    keep = points[:, 2] - table_z > MIN_HEIGHT_M
    for x, y in keep_out:
        keep &= np.hypot(points[:, 0] - x, points[:, 1] - y) > CUBE_KEEP_OUT_M
    points, colours, pairs = points[keep], colours[keep], pairs[keep]
    if len(points) < MIN_CLUSTER_POINTS:
        return []
    # Gom cụm trong lưới 3D (không phải trên mặt bàn): hai vật cao đứng cạnh nhau thường bị nối bởi vài điểm thấp
    # nằm giữa (quai cốc, hình in bị đo lệch độ cao); trong 3D chúng chỉ chạm nhau ở sát bàn nên tách được.
    from scipy import ndimage
    cells = np.floor(points / CLUSTER_CELL_M).astype(int)
    low = cells.min(axis=0)
    counts = np.zeros(tuple(cells.max(axis=0) - low + 1), np.int32)
    np.add.at(counts, tuple((cells - low).T), 1)
    # Ô chỉ có vài điểm lẻ là khớp nhầm: không cho chúng bắc cầu giữa hai vật. Đám mây càng dày (nhiều khung) thì
    # ngưỡng càng cao: lấy theo ô điển hình của chính đám mây đó.
    threshold = max(MIN_CELL_POINTS, int(np.percentile(counts[counts > 0], 50)))
    labels, _ = ndimage.label(counts >= threshold, structure=np.ones((3, 3, 3)))
    of_point = labels[tuple((cells - low).T)]
    points, colours, pairs, of_point = (v[of_point > 0] for v in (points, colours, pairs, of_point))
    found = []
    sizes = {int(label): int((of_point == label).sum()) for label in np.unique(of_point)}
    # Cụm quá ít điểm so với vật rõ nhất là chưa đủ dữ liệu để gọi là một vật (dây cáp, mảng nhiễu).
    enough = max(MIN_CLUSTER_POINTS, int(MIN_CLUSTER_SHARE * max(sizes.values(), default=0)))
    for label in sizes:
        sel = of_point == label
        if sizes[label] < enough:
            continue
        shape = fit_shape(points[sel], table_z, eye_xy)
        if inside is not None and not inside(shape["centre"]):
            continue
        order = np.linspace(0, sel.sum() - 1, min(int(sel.sum()), MAX_STORED_POINTS)).astype(int)
        found.append({**shape, "label": "vật", "n_points": int(sel.sum()),
                      "n_views": int(len(np.unique(pairs[sel]))), "points": points[sel][order],
                      "colours": colours[sel][order], "area_m2": float(cv2.contourArea(
                          (shape["polygon"] * 1000.0).astype(np.float32))) / 1e6})
    return sorted(found, key=lambda item: -item["n_points"])


def silhouette_width(camera, a_T_optical, mask, probe_xyz):
    """Bề ngang thật (m) của vật tại độ cao của `probe_xyz`, đo từ MẶT NẠ của vật trong một khung: hai mép trái/phải
    của mặt nạ trên hàng ảnh đi qua điểm dò, chiếu ra mặt phẳng ngang ở độ cao đó. None khi điểm dò không nằm trong
    mặt nạ hoặc mặt nạ chạm mép ảnh (vật bị cắt: bề ngang không đo được)."""
    from .frames import invert, pixel_to_plane
    T = np.asarray(a_T_optical, float)
    inv = invert(T)
    uv = camera.project((inv[:3, :3] @ np.asarray(probe_xyz, float) + inv[:3, 3]).reshape(1, 3))[0]
    h, w = mask.shape[:2]
    if not np.isfinite(uv).all() or not (0 <= uv[0] < w and 0 <= uv[1] < h):
        return None
    row, col = int(uv[1]), int(uv[0])
    if not mask[row, col]:
        return None
    left, right = col, col
    while left > 0 and mask[row, left - 1]:
        left -= 1
    while right < w - 1 and mask[row, right + 1]:
        right += 1
    if left <= 2 or right >= w - 3:
        return None
    a = pixel_to_plane(camera, T, left, row, probe_xyz[2])
    b = pixel_to_plane(camera, T, right, row, probe_xyz[2])
    return None if a is None or b is None else float(np.linalg.norm(a - b))


def cylinder_from_near_edge(item: dict, radius: float, eye_xy=(0.0, 0.0)) -> dict:
    """Hình trụ bán kính `radius` đặt sao cho MẶT GẦN camera của nó đi qua đám điểm của vật.

    Dùng khi biết vật tròn (nhãn cốc/lon/chai, hoặc cung tròn khớp được) và biết bề ngang thật từ viền vật trong ảnh:
    đám điểm cho vị trí mặt trước (đo chắc), bán kính cho phần còn lại. Giữ nguyên chiều cao.
    """
    eye = np.asarray(eye_xy, float)
    xy = np.asarray(item["points"], float)[:, :2] - eye
    radial = xy.mean(axis=0)
    radial = radial / np.linalg.norm(radial)
    tangent = np.array([-radial[1], radial[0]])
    r, t = xy @ radial, xy @ tangent
    # điểm ở dải giữa nằm trên chỗ gần camera nhất của mặt trụ; trung vị chống nhiễu độ sâu
    middle = np.abs(t - np.median(t)) <= max(radius * 0.4, 0.008)
    near = float(np.median(r[middle]))
    centre = eye + radial * (near + radius) + tangent * float(np.median(t))
    ring = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    return {**item, "shape": "cylinder", "centre": centre, "width_m": 2.0 * radius,
            "polygon": centre + radius * np.c_[np.cos(ring), np.sin(ring)], "area_m2": float(np.pi * radius * radius)}
