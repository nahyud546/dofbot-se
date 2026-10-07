"""Camera ngoài (cố định) ↔ base robot: mô hình, bộ giải hiệu chuẩn và các phép đổi tọa độ.

Mô hình giống camera tay (cube_layer._project): pinhole, điểm ảnh vuông (fx = fy), một hệ số méo
hướng kính k1, pose `base_T_ext` (hệ quang của camera ngoài trong base_link):

    điểm base -> hệ camera -> x, y chuẩn hóa -> nhân (1 + k1·r²) -> pixel

Bộ giải (`solve`) nhận các cặp điểm 3D trong base ↔ pixel camera ngoài (góc AprilTag do camera tay đã
hiệu chuẩn hand-eye định vị, ở NHIỀU độ cao) và tìm f, tâm ảnh, k1, pose; để riêng một phần nhóm làm
kiểm định. Thuần numpy/cv2/scipy: không import T8, ROS hay phần cứng. Thu mẫu thật nằm ở
projects/vision_experiments/calibrate_external.py.

    cal = ExternalCalibration.load()            # None nếu chưa có / chưa đạt
    u, v = cal.project([x, y, z])               # base -> pixel
    xyz = cal.pixel_to_base(u, v, z)            # pixel + độ cao đã biết -> base
    cubes = cal.locate_tags(frame)              # tag trên cube -> XY/yaw trong base
    cal.drift_px(frame)                         # camera có bị dời từ lúc hiệu chuẩn không
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .registry import repo_root

TAG_SIZE_M = 0.020
CUBE_EDGE_M = 0.030
MAX_LAYER = 3
# Tiêu chí chấp nhận một lần hiệu chuẩn.
# Thước đo chính là mm trên mặt phẳng ở vị trí CHƯA dùng để fit (kiểm chéo bỏ-một-vị-trí): đó là cách
# runtime dùng. RMS pixel chỉ là chốt chặn thô: ở 1280x720, 3 px ~ 2 mm, chặt hơn cả sai số camera tay.
MAX_FIT_RMS_PX = 6.0
MAX_HOLDOUT_MEDIAN_M = 0.006
MAX_CV_MEDIAN_M = 0.003
MAX_CV_WORST_M = 0.008
CV_CELL_M = 0.015              # hai mẫu cùng ô này và cùng độ cao coi là một vị trí
MIN_Z_LEVELS = 3
MIN_POINTS = 24
MAX_DRIFT_PX = 12.0            # tâm các ô màu lệch quá mức này => camera đã bị dời
PRIOR_CENTRE_PX = 60.0         # tâm ảnh gần giữa khung (độ lệch chuẩn của tiền nghiệm)
PRIOR_K1 = 0.25


def default_path() -> Path:
    return repo_root() / "config" / "robot" / "external_camera.json"


# ------------------------------------------------------------------ mô hình
def _distort(x, y, k1):
    d = 1.0 + k1 * (x * x + y * y)
    return x * d, y * d


def _undistort(xd, yd, k1):
    x, y = xd, yd
    for _ in range(10):
        d = 1.0 + k1 * (x * x + y * y)
        x, y = xd / d, yd / d
    return x, y


def project_points(points_base, K, k1, base_T_ext):
    """(N,3) điểm base -> (N,2) pixel. K = (f, cx, cy)."""
    pts = np.asarray(points_base, float).reshape(-1, 3)
    ext_T_base = np.linalg.inv(np.asarray(base_T_ext, float))
    cam = pts @ ext_T_base[:3, :3].T + ext_T_base[:3, 3]
    z = np.where(np.abs(cam[:, 2]) < 1e-9, 1e-9, cam[:, 2])
    x, y = _distort(cam[:, 0] / z, cam[:, 1] / z, k1)
    f, cx, cy = K
    return np.stack([f * x + cx, f * y + cy], axis=1)


def pixel_ray(u, v, K, k1, base_T_ext):
    """Tia (gốc, hướng đơn vị) trong base qua pixel (u, v)."""
    f, cx, cy = K
    x, y = _undistort((float(u) - cx) / f, (float(v) - cy) / f, k1)
    T = np.asarray(base_T_ext, float)
    ray = T[:3, :3] @ np.array([x, y, 1.0])
    return T[:3, 3].copy(), ray / np.linalg.norm(ray)


def pixel_to_plane(u, v, z, K, k1, base_T_ext):
    """Điểm base nơi tia qua pixel cắt mặt phẳng ngang z; None nếu tia không đi xuống mặt phẳng."""
    origin, ray = pixel_ray(u, v, K, k1, base_T_ext)
    if abs(ray[2]) < 1e-6:
        return None
    t = (float(z) - origin[2]) / ray[2]
    return None if t <= 0 else origin + ray * t


# ------------------------------------------------------------------ bộ giải
def _pack(f, cx, cy, k1, ext_T_base):
    return np.concatenate([[math.log(f), cx, cy, k1], cv2.Rodrigues(ext_T_base[:3, :3])[0].ravel(),
                           ext_T_base[:3, 3]])


def _unpack(params):
    f, cx, cy, k1 = math.exp(params[0]), params[1], params[2], params[3]
    ext_T_base = np.eye(4)
    ext_T_base[:3, :3] = cv2.Rodrigues(np.asarray(params[4:7], float).reshape(3, 1))[0]
    ext_T_base[:3, 3] = params[7:10]
    return (f, cx, cy), k1, np.linalg.inv(ext_T_base)


def _residuals(params, points, pixels, size, fit_intrinsics):
    K, k1, base_T_ext = _unpack(params)
    res = (project_points(points, K, k1, base_T_ext) - pixels).ravel()
    if not fit_intrinsics:
        return res
    w, h = size
    priors = [(K[1] - w / 2.0) / PRIOR_CENTRE_PX, (K[2] - h / 2.0) / PRIOR_CENTRE_PX, k1 / PRIOR_K1]
    return np.concatenate([res, priors])


def _fit(points, pixels, size, K0=None, k1=0.0):
    """Tối ưu f, cx, cy, k1, pose (K0=None) hoặc chỉ pose (K0 cố định). Trả (K, k1, base_T_ext, rms)."""
    from scipy.optimize import least_squares
    points, pixels = np.asarray(points, float), np.asarray(pixels, float)
    w, h = size
    fit_intr = K0 is None
    best = None
    for f0 in ((0.7 * w, 0.95 * w, 1.3 * w) if fit_intr else (K0[0],)):
        cx0, cy0 = (w / 2.0, h / 2.0) if fit_intr else (K0[1], K0[2])
        Kmat = np.array([[f0, 0, cx0], [0, f0, cy0], [0, 0, 1.0]])
        ok, rvec, tvec, _ = cv2.solvePnPRansac(points.astype(np.float64), pixels.astype(np.float64), Kmat,
                                               None, reprojectionError=12.0, iterationsCount=300)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(points.astype(np.float64), pixels.astype(np.float64), Kmat, None)
        if not ok:
            continue
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = np.ravel(tvec)
        x0 = _pack(f0, cx0, cy0, k1, T)
        if fit_intr:
            sol = least_squares(_residuals, x0, loss="huber", f_scale=2.0,
                                args=(points, pixels, size, True))
            params = sol.x
        else:
            # Chỉ pose: giữ nguyên 4 tham số đầu (ống kính), tối ưu 6 tham số pose.
            sol = least_squares(lambda p: _residuals(np.concatenate([x0[:4], p]), points, pixels, size, False),
                                x0[4:], loss="huber", f_scale=2.0)
            params = np.concatenate([x0[:4], sol.x])
        K, kk, base_T_ext = _unpack(params)
        err = project_points(points, K, kk, base_T_ext) - pixels
        rms = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
        if best is None or rms < best[3]:
            best = (K, float(kk), base_T_ext, rms)
    if best is None:
        raise ValueError("không khởi tạo được pose camera ngoài (PnP thất bại)")
    return best


def plane_errors_m(points, pixels, K, k1, base_T_ext):
    """Sai số XY (m) khi đưa từng pixel về mặt phẳng z thật của điểm đó: cách runtime dùng."""
    out = []
    for point, (u, v) in zip(np.asarray(points, float), np.asarray(pixels, float)):
        hit = pixel_to_plane(u, v, point[2], K, k1, base_T_ext)
        out.append(math.inf if hit is None else float(np.hypot(hit[0] - point[0], hit[1] - point[1])))
    return np.array(out)


def z_levels(points, step=0.01):
    return sorted({round(float(z) / step) for z in np.asarray(points, float)[:, 2]})


def cross_validate(points, pixels, size, groups):
    """Bỏ lần lượt từng VỊ TRÍ (mọi mẫu của nó), fit phần còn lại, đo sai số tâm nhóm trên mặt phẳng (m).

    Trả list sai số, mỗi nhóm bị bỏ một số; rỗng khi chỉ có một vị trí hoặc phần còn lại quá ít điểm.
    """
    groups = np.asarray([str(g) for g in groups])
    place = {}
    for g in dict.fromkeys(groups.tolist()):
        c = points[groups == g].mean(axis=0)
        place[g] = (round(c[0] / CV_CELL_M), round(c[1] / CV_CELL_M), round(c[2] / 0.01))
    errors = []
    for cell in dict.fromkeys(place.values()):
        members = [g for g, key in place.items() if key == cell]
        held = np.isin(groups, members)
        if (~held).sum() < 6 or len(set(place.values())) < 2:
            return []
        K, k1, T, _ = _fit(points[~held], pixels[~held], size)
        for g in members:
            centre, (u, v) = points[groups == g].mean(axis=0), pixels[groups == g].mean(axis=0)
            hit = pixel_to_plane(u, v, centre[2], K, k1, T)
            errors.append(math.inf if hit is None else float(np.hypot(hit[0] - centre[0], hit[1] - centre[1])))
    return errors


def solve(points, pixels, image_size, groups=None, holdout_every=5):
    """Hiệu chuẩn từ điểm base ↔ pixel.

    groups: nhãn nhóm của từng điểm (ví dụ (bộ mẫu, tag id)); cứ `holdout_every` nhóm thì để riêng một nhóm
    làm kiểm định (không tham gia fit lần đánh giá). Kết quả cuối fit trên TẤT CẢ điểm.
    Trả dict: K, k1, base_T_ext, fit_rms_px, holdout_*, z_levels, accepted, reasons.
    """
    points, pixels = np.asarray(points, float).reshape(-1, 3), np.asarray(pixels, float).reshape(-1, 2)
    if len(points) != len(pixels) or len(points) < 6:
        raise ValueError("cần ít nhất 6 cặp điểm base–pixel")
    size = (int(image_size[0]), int(image_size[1]))
    groups = list(groups) if groups is not None else list(range(len(points)))
    order = []
    for g in groups:
        if g not in order:
            order.append(g)
    held = set(order[holdout_every - 1::holdout_every]) if len(order) >= holdout_every else set()
    val = np.array([g in held for g in groups])
    holdout_px, holdout_m = [], []
    if val.any() and (~val).sum() >= 6:
        K, k1, T, _ = _fit(points[~val], pixels[~val], size)
        holdout_px = np.linalg.norm(project_points(points[val], K, k1, T) - pixels[val], axis=1).tolist()
        holdout_m = plane_errors_m(points[val], pixels[val], K, k1, T).tolist()
    K, k1, T, rms = _fit(points, pixels, size)
    levels = z_levels(points)
    cv_m = cross_validate(points, pixels, size, groups) if len(order) >= holdout_every else []
    reasons = []
    if len(points) < MIN_POINTS:
        reasons.append(f"chỉ có {len(points)} điểm (cần ≥ {MIN_POINTS})")
    if len(levels) < MIN_Z_LEVELS:
        reasons.append(f"chỉ có {len(levels)} độ cao (cần ≥ {MIN_Z_LEVELS}: xếp cube thành tháp)")
    if rms > MAX_FIT_RMS_PX:
        reasons.append(f"RMS chiếu lại {rms:.1f} px (cần ≤ {MAX_FIT_RMS_PX:g})")
    if not holdout_m:
        reasons.append("không có nhóm kiểm định (cần ≥ 5 nhóm)")
    elif float(np.median(holdout_m)) > MAX_HOLDOUT_MEDIAN_M:
        reasons.append(f"sai số kiểm định trung vị {np.median(holdout_m) * 1000:.1f} mm "
                       f"(cần ≤ {MAX_HOLDOUT_MEDIAN_M * 1000:g})")
    if cv_m and float(np.median(cv_m)) > MAX_CV_MEDIAN_M:
        reasons.append(f"kiểm chéo: tâm cube ở vị trí mới lệch trung vị {np.median(cv_m) * 1000:.1f} mm "
                       f"(cần ≤ {MAX_CV_MEDIAN_M * 1000:g})")
    if cv_m and max(cv_m) > MAX_CV_WORST_M:
        reasons.append(f"kiểm chéo: vị trí tệ nhất lệch {max(cv_m) * 1000:.1f} mm (cần ≤ {MAX_CV_WORST_M * 1000:g})")
    w, h = size
    if not (0.4 * w < K[0] < 3.0 * w) or abs(k1) > 1.0 or T[2, 3] < 0.05:
        reasons.append("nghiệm phi vật lý (tiêu cự/méo/độ cao camera)")
    return {"K": [float(v) for v in K], "k1": float(k1), "base_T_ext": T.tolist(), "image_size": list(size),
            "fit_rms_px": rms, "n_points": int(len(points)), "n_groups": len(order),
            "holdout_px": [float(v) for v in holdout_px], "holdout_m": [float(v) for v in holdout_m],
            "cv_m": [float(v) for v in cv_m],
            "cv_m": [float(v) for v in cv_m],
            "z_levels_m": [round(l * 0.01, 3) for l in levels],
            "accepted": not reasons, "reasons": reasons}


def relocalize(points, pixels, cal):
    """Camera bị dời nhưng ống kính không đổi: chỉ giải lại pose (giữ K, k1). Trả (base_T_ext, rms_px)."""
    _, _, T, rms = _fit(points, pixels, cal.image_size, K0=cal.K, k1=cal.k1)
    return T, rms


# ------------------------------------------------------------------ runtime
def _fit_square(points_xy, edge):
    """Khớp hình vuông cạnh `edge` vào 4 điểm theo thứ tự vòng: (rms, tâm, yaw mod 90°)."""
    pts = np.asarray(points_xy, float)
    centre = pts.mean(axis=0)
    rel = pts - centre
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    steps = (1.0 if area > 0 else -1.0) * np.arange(4) * math.pi / 2
    theta = float(np.angle(np.sum(np.exp(1j * (np.arctan2(rel[:, 1], rel[:, 0]) - steps)))))
    ideal = centre + edge / math.sqrt(2) * np.stack([np.cos(theta + steps), np.sin(theta + steps)], axis=1)
    rms = float(math.sqrt(np.mean(np.sum((pts - ideal) ** 2, axis=1))))
    yaw = math.atan2(*(ideal[1] - ideal[0])[::-1])
    return rms, centre, (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4


@dataclass
class ExternalCalibration:
    K: tuple                    # (f, cx, cy)
    k1: float
    base_T_ext: np.ndarray      # 4x4
    image_size: tuple           # (w, h) lúc hiệu chuẩn; ảnh khác cỡ này không dùng được
    tag_top_z: float = 0.0578   # độ cao mặt tag của cube nằm trên bàn (từ hand-eye camera tay)
    accepted: bool = False
    landmarks: dict = field(default_factory=dict)      # {zone: [u, v]} tâm ô màu lúc hiệu chuẩn
    meta: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data):
        return cls(tuple(float(v) for v in data["K"]), float(data["k1"]),
                   np.asarray(data["base_T_ext"], float).reshape(4, 4),
                   tuple(int(v) for v in data["image_size"]), float(data.get("tag_top_z", 0.0578)),
                   bool(data.get("accepted")),
                   {int(k): v for k, v in (data.get("landmarks") or {}).items()},
                   {k: v for k, v in data.items()
                    if k not in ("K", "k1", "base_T_ext", "image_size", "tag_top_z", "accepted", "landmarks")})

    @classmethod
    def load(cls, path=None, require_accepted=True):
        """Hiệu chuẩn đã lưu; None nếu thiếu, hỏng hoặc (mặc định) chưa đạt tiêu chí."""
        try:
            cal = cls.from_dict(json.loads(Path(path or default_path()).read_text()))
        except (OSError, ValueError, KeyError, TypeError):
            return None
        if not np.isfinite(cal.base_T_ext).all() or (require_accepted and not cal.accepted):
            return None
        return cal

    def to_dict(self):
        return {"K": list(self.K), "k1": self.k1, "base_T_ext": np.asarray(self.base_T_ext).tolist(),
                "image_size": list(self.image_size), "tag_top_z": self.tag_top_z, "accepted": self.accepted,
                "landmarks": {str(k): v for k, v in self.landmarks.items()}, **self.meta}

    def save(self, path=None):
        target = Path(path or default_path())
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=1))
        return target

    def matches(self, frame) -> bool:
        return (frame.shape[1], frame.shape[0]) == tuple(self.image_size)

    def project(self, xyz):
        """Điểm base (3,) hoặc (N,3) -> pixel."""
        pts = np.asarray(xyz, float)
        out = project_points(pts, self.K, self.k1, self.base_T_ext)
        return out[0] if pts.ndim == 1 else out

    def pixel_to_base(self, u, v, z):
        """Pixel + độ cao z đã biết (m, base) -> điểm base; None nếu tia không cắt mặt phẳng."""
        return pixel_to_plane(u, v, z, self.K, self.k1, self.base_T_ext)

    def locate_tag(self, corners_px, layer=None):
        """4 góc tag (pixel) -> {"x","y","yaw_rad","layer","top_z","rms_mm","layer_confident"}.

        layer=None: thử tầng 0..3 và lấy tầng khớp hình vuông 20 mm nhất. Từ xa hai tầng kề nhau chỉ
        khác nhau ~1 mm nên `layer_confident` thường False; biết tầng thì truyền vào.
        """
        corners = np.asarray(corners_px, float).reshape(4, 2)
        scored = []
        for n in (range(MAX_LAYER + 1) if layer is None else (int(layer),)):
            z = self.tag_top_z + CUBE_EDGE_M * n
            hits = [self.pixel_to_base(u, v, z) for u, v in corners]
            if any(h is None for h in hits):
                continue
            rms, centre, yaw = _fit_square(np.array(hits)[:, :2], TAG_SIZE_M)
            scored.append((rms, n, z, centre, yaw))
        if not scored:
            return None
        scored.sort(key=lambda item: item[0])
        rms, n, z, centre, yaw = scored[0]
        margin = scored[1][0] - rms if len(scored) > 1 else math.inf
        return {"x": float(centre[0]), "y": float(centre[1]), "yaw_rad": float(yaw), "layer": n, "top_z": z,
                "rms_mm": rms * 1000.0, "layer_confident": layer is not None or margin > 0.0008}

    def locate_tags(self, frame, detector=None, layers=None):
        """Mọi tag cube thấy trong ảnh -> [{"id","cube_id",...locate_tag}]. layers: {tag_id: tầng} đã biết."""
        if not self.matches(frame):
            raise ValueError(f"ảnh {frame.shape[1]}x{frame.shape[0]} khác cỡ hiệu chuẩn {self.image_size}")
        if detector is None:
            from .tag import TagDetector
            detector = TagDetector(enhance=True, quiet=True)
        out = []
        for tag in detector.detect(frame):
            located = self.locate_tag(tag["corners"], (layers or {}).get(tag["id"]))
            if located is not None:
                out.append({"id": tag["id"], "cube_id": tag.get("cube_id"), **located})
        return out

    def drift_px(self, frame):
        """Độ lệch (px) của tâm các ô màu so với lúc hiệu chuẩn; None nếu không so được.

        Camera bị dời thì MỌI ô cùng lệch; cube nằm trên ô chỉ làm lệch tâm của riêng ô đó. Vì vậy lấy độ lệch
        nhỏ thứ hai (hai ô đứng yên là đủ kết luận camera đứng yên), không lấy trung vị.
        """
        if not self.landmarks or not self.matches(frame):
            return None
        now = pad_landmarks(frame)
        shifts = [math.hypot(now[z][0] - uv[0], now[z][1] - uv[1])
                  for z, uv in self.landmarks.items() if z in now]
        if len(shifts) < 2:
            return None
        return float(sorted(shifts)[1 if len(shifts) >= 3 else 0])

    def moved(self, frame) -> bool:
        drift = self.drift_px(frame)
        return drift is not None and drift > MAX_DRIFT_PX


def pad_landmarks(frame):
    """{zone: [u, v]} tâm các ô màu thấy thật trong ảnh (không lấy ô suy ra): mốc phát hiện camera bị dời."""
    from .placement_check import find_pads
    out = {}
    for zone, pad in find_pads(frame).items():
        if not pad.inferred:
            m = cv2.moments(pad.hull.reshape(-1, 1, 2).astype(np.float32))
            if abs(m["m00"]) > 1e-6:
                out[int(zone)] = [round(m["m10"] / m["m00"], 1), round(m["m01"] / m["m00"], 1)]
    return out


def build(result, tag_top_z, landmarks=None, device="", samples=None):
    """Kết quả `solve` -> ExternalCalibration (kèm thông tin để lưu)."""
    return ExternalCalibration(
        tuple(result["K"]), result["k1"], np.asarray(result["base_T_ext"], float), tuple(result["image_size"]),
        float(tag_top_z), bool(result["accepted"]), dict(landmarks or {}),
        {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "device": device,
         "fit_rms_px": result["fit_rms_px"], "n_points": result["n_points"], "n_groups": result["n_groups"],
         "holdout_px": result["holdout_px"], "holdout_m": result["holdout_m"],
         "z_levels_m": result["z_levels_m"], "reasons": result["reasons"], "n_sets": samples})


def make_locator(cal, camera):
    """locator(cube_id) -> {"x","y","drift_px"} | None cho bước kiểm tra sau khi thả (cube nằm trên bàn/ô, tầng 0).

    camera.grab() phải cho ảnh đúng cỡ hiệu chuẩn. Trả None khi không có ảnh, camera đã bị dời, hoặc không
    thấy tag của cube (tag_id = cube_id): bên gọi coi như không có số đo mét.
    """
    detector = []

    def locate(cube_id):
        frame = camera.grab()
        if frame is None or not cal.matches(frame) or cal.moved(frame):
            return None
        if not detector:
            from .tag import TagDetector
            detector.append(TagDetector(enhance=True, quiet=True))
        for tag in cal.locate_tags(frame, detector[0], layers={int(cube_id): 0}):
            if int(tag["id"]) == int(cube_id):
                return {"x": tag["x"], "y": tag["y"], "drift_px": cal.drift_px(frame)}
        return None
    return locate


def main():
    """In vị trí (base) của các cube mà camera ngoài thấy tag, và kiểm tra camera có bị dời không."""
    import argparse
    from .cameras import resolve_external, resolve_wrist
    from .placement_check import ExternalCamera
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("--camera", default="auto")
    args = ap.parse_args()
    cal = ExternalCalibration.load(require_accepted=False)
    if cal is None:
        raise SystemExit("Chưa có config/robot/external_camera.json (xem calibrate_external.py).")
    wrist, _ = resolve_wrist()
    frame = ExternalCamera(resolve_external(wrist, args.camera), width=cal.image_size[0],
                           height=cal.image_size[1], fourcc="MJPG").grab()
    if frame is None or not cal.matches(frame):
        raise SystemExit("Camera ngoài không cho ảnh đúng cỡ hiệu chuẩn.")
    drift = cal.drift_px(frame)
    print(f"accepted={cal.accepted}; trôi so với lúc hiệu chuẩn: "
          + ("không đo được" if drift is None else f"{drift:.1f} px" + (" (ĐÃ BỊ DỜI)" if drift > MAX_DRIFT_PX else "")))
    for tag in cal.locate_tags(frame):
        print(f"  cube {tag['id']}: x={tag['x'] * 1000:+.0f} y={tag['y'] * 1000:+.0f} mm, tầng {tag['layer']}"
              f"{'' if tag['layer_confident'] else ' (tầng không chắc)'}, khớp {tag['rms_mm']:.1f} mm")


if __name__ == "__main__":
    main()
