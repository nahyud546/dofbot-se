"""ROS-independent geometry and conservative red object detection (metres)."""
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def transform(value):
    t = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError("invalid rigid transform")
    return t


def calibration_error(cfg):
    try:
        c, t = cfg["camera"], cfg["table"]
        if not all([c["calibrated"], c["extrinsic_calibrated"], t["calibrated"]]):
            return "intrinsic / eye-in-hand / table calibration required"
        k = np.asarray(c["K"], float).reshape(3, 3)
        if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
            return "invalid camera matrix"
        if not np.isfinite(c["distortion"]).all():
            return "invalid distortion"
        transform(c["mount_T_optical"])
        transform(t["base_T_table"])
    except (KeyError, ValueError, TypeError):
        return "invalid calibration config"
    return ""


def project_to_plane(pixels, K, distortion, base_T_camera, base_T_table, height=0.0):
    """Undistorted rays intersect a table-parallel plane at local height."""
    cam, table = transform(base_T_camera), transform(base_T_table)
    uv = np.asarray(pixels, float).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(uv, np.asarray(K, float).reshape(3, 3),
                             np.asarray(distortion, float)).reshape(-1, 2)
    rays = np.column_stack((xy, np.ones(len(xy)))) @ cam[:3, :3].T
    normal = table[:3, 2]
    origin = cam[:3, 3]
    plane_point = table[:3, 3] + height * normal
    denominator = rays @ normal
    if np.any(np.abs(denominator) < 1e-5):
        raise ValueError("ray parallel to table")
    distance = ((plane_point - origin) @ normal) / denominator
    if np.any(distance <= 0) or not np.isfinite(distance).all():
        raise ValueError("intersection behind camera")
    return origin + distance[:, None] * rays


@dataclass
class Observation:
    kind: str
    center: np.ndarray
    size: np.ndarray
    boundary: np.ndarray
    yaw: float
    quality: float


def red_contours(bgr, cfg):
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in cfg["red_ranges"]:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = mask.shape
    # Clipped objects cannot provide a reliable full boundary or center.
    return [c for c in contours if cv2.contourArea(c) >= cfg["min_area_px"]
            and cv2.boundingRect(c)[0] > 1 and cv2.boundingRect(c)[1] > 1
            and cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] < w - 1
            and cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] < h - 1], mask


def detect_scene(bgr, cfg, base_T_camera):
    p, c = cfg["perception"], cfg["camera"]
    table = transform(cfg["table"]["base_T_table"])
    contours, mask = red_contours(bgr, p)
    found = []
    for contour in contours:
        candidates = []
        for kind, height in (("cube", p["cube_size"]), ("zone", 0.0)):
            try:
                points = project_to_plane(contour, c["K"], c["distortion"],
                                          base_T_camera, table, height)
            except ValueError:
                continue
            local = (points - table[:3, 3]) @ table[:3, :3]
            rect = cv2.minAreaRect(local[:, :2].astype(np.float32))
            sides = np.array(rect[1])
            if min(sides) <= 0 or max(sides) / min(sides) > p["max_aspect"]:
                continue
            fill = cv2.contourArea(local[:, :2].astype(np.float32)) / np.prod(sides)
            if fill < p["min_fill"]:
                continue
            if kind == "cube":
                if np.max(abs(sides - p["cube_size"])) > p["cube_size_tolerance"]:
                    continue
            elif not (min(sides) >= p["zone_side_min"] and max(sides) <= p["zone_side_max"]):
                continue
            center_local = [*rect[0], height / 2]
            center = table[:3, :3] @ center_local + table[:3, 3]
            corners = cv2.boxPoints(rect)
            boundary = np.column_stack((corners, np.full(4, height))) @ table[:3, :3].T + table[:3, 3]
            candidates.append(Observation(kind, center,
                np.array([*sides, height if kind == "cube" else 0.002]),
                boundary, np.deg2rad(rect[2]), float(min(fill, 1.0))))
        if len(candidates) == 1:
            found.extend(candidates)
    # v1 supports one object of each kind; ambiguity never selects the largest.
    return [o for o in found if sum(x.kind == o.kind for x in found) == 1], mask


class StableObservations:
    def __init__(self, count, radius, max_age):
        self.count, self.radius, self.max_age = count, radius, max_age
        self.samples = {}

    def clear(self):
        self.samples.clear()

    def update(self, observations, stamp):
        present = {o.kind for o in observations}
        self.samples = {k: v for k, v in self.samples.items() if k in present}
        stable = []
        for obj in observations:
            history = self.samples.setdefault(obj.kind, deque(maxlen=self.count))
            if history and stamp <= history[-1][0]:
                continue  # republished frames cannot manufacture stability
            if history and (stamp - history[-1][0] > self.max_age
                            or np.linalg.norm(obj.center - history[-1][1]) > self.radius):
                history.clear()
            history.append((stamp, obj.center.copy()))
            centers = np.array([v for _, v in history])
            if len(history) == self.count and np.max(np.linalg.norm(centers - centers.mean(0), axis=1)) <= self.radius:
                # Keep the measured pose/boundary coherent; filtering gates observations.
                stable.append(obj)
        return stable


class StationaryWindow:
    def __init__(self, window, tolerance):
        self.window, self.tolerance = window, tolerance
        self.history = deque()

    def add(self, stamp, positions):
        q = np.asarray(positions, float)
        if not np.isfinite(q).all():
            self.history.clear()
            return
        if self.history and stamp <= self.history[-1][0]:
            self.history.clear()
        self.history.append((stamp, q))
        while len(self.history) > 2 and self.history[1][0] <= stamp - self.window:
            self.history.popleft()

    def ready(self, stamp, max_age):
        if len(self.history) < 2:
            return False
        first, last = self.history[0][0], self.history[-1][0]
        return (first <= stamp and abs(stamp - last) <= max_age and last - first >= self.window
                and np.max(np.ptp(np.array([q for _, q in self.history]), axis=0)) <= self.tolerance)
