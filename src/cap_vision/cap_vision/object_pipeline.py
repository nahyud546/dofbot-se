"""Object-level perception primitives. No ROS or robot commands live here.

Transforms use target_T_source, metres, and column vectors. The cube origin is
its geometric centre. A tag's local XY axes follow its detector corner order.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import cv2
import numpy as np


CUBE_NAMES = {1: "blue", 2: "green", 3: "red", 4: "yellow"}
COLOR_TO_ID = {"khoi_xanh_duong": 1, "khoi_xanh": 2,
               "khoi_do": 3, "khoi_vang": 4}
TRASH_TO_ID = {
    name: cube_id for cube_id, names in {
        1: ("newspaper", "zip_top_can", "book", "old_school_bag"),
        2: ("fish_bone", "egg_shell", "apple_core", "watermelon_rind"),
        3: ("syringe", "expired_cosmetics", "used_batteries", "expired_tablets"),
        4: ("toilet_paper", "peach_pit", "cigarette_butts", "disposable_chopsticks"),
    }.items() for name in names
}
HSV = {
    1: (((90, 70, 40), (130, 255, 255)),),
    2: (((35, 70, 40), (85, 255, 255)),),
    3: (((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (179, 255, 255))),
    4: (((20, 70, 50), (35, 255, 255)),),
}


def rigid(value):
    matrix = np.asarray(value, dtype=np.float64).reshape(4, 4)
    if (not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-3) or
            np.linalg.det(matrix[:3, :3]) < 0.999):
        raise ValueError("invalid rigid transform")
    return matrix


@dataclass(frozen=True)
class ObjectModel:
    object_type: str
    dimensions: tuple[float, float, float]
    coordinate_frame: str = "object_center"
    grasp_surfaces: tuple[str, ...] = ()


@dataclass(frozen=True)
class CubeModel(ObjectModel):
    tag_id: int = 0
    tag_size_m: float = 0.0
    tag_T_cube: np.ndarray | None = None
    semantic_faces: dict[str, str] = field(default_factory=dict)

    @classmethod
    def measured(cls, cube_id, size_m, tag_size_m, cube_T_tag):
        if cube_id not in CUBE_NAMES or size_m <= 0 or tag_size_m <= 0:
            raise ValueError("cube/tag dimensions must be measured and positive")
        return cls("cube", (size_m,) * 3, "object_center",
                   ("+X", "-X", "+Y", "-Y", "+Z", "-Z"), cube_id,
                   tag_size_m, np.linalg.inv(rigid(cube_T_tag)))

    def vertices(self):
        half = self.dimensions[0] / 2
        return np.array([(x, y, z) for x in (-half, half)
                         for y in (-half, half) for z in (-half, half)], np.float64)

    def surfaces(self, base_T_object):
        tf = rigid(base_T_object)
        half = self.dimensions[0] / 2
        axes = (("+X", (1, 0, 0)), ("-X", (-1, 0, 0)),
                ("+Y", (0, 1, 0)), ("-Y", (0, -1, 0)),
                ("+Z", (0, 0, 1)), ("-Z", (0, 0, -1)))
        return [(name, tf[:3, 3] + tf[:3, :3] @ (half * np.array(normal)),
                 tf[:3, :3] @ np.array(normal, dtype=float)) for name, normal in axes]


@dataclass
class ObjectInstance:
    bbox: tuple[int, int, int, int]
    mask: np.ndarray
    confidence: float
    track_id: str = ""


@dataclass(frozen=True)
class Evidence:
    source: str
    cube_id: int
    reliability: float


@dataclass
class PoseEstimate:
    camera_T_object: np.ndarray
    confidence: float
    reprojection_error_px: float
    method: str
    covariance: np.ndarray | None = None
    # Phân biệt position-only fallback với full 6D pose (yêu cầu lộ trình).
    # rgb_geometry chỉ quan sát được vị trí tâm (orientation unknown);
    # apriltag_ippe / fused mới có orientation đầy đủ để grasp 6D.
    orientation_valid: bool = True


FULL_6D_METHODS = {"apriltag_ippe", "apriltag_rgb_fused"}
POSITION_ONLY_METHODS = {"rgb_geometry"}


def is_full_6d_pose(method, orientation_valid=True):
    """True chỉ khi có orientation đầy đủ, không phải position-only."""
    try:
        return bool(orientation_valid) and str(method) in FULL_6D_METHODS
    except (TypeError, ValueError):
        return False


def is_position_only_pose(method, orientation_valid=True):
    """Fallback chỉ có vị trí: track được nhưng KHÔNG grasp 6D."""
    try:
        return (not bool(orientation_valid)) or str(method) in POSITION_ONLY_METHODS
    except (TypeError, ValueError):
        return True


class PoseEstimator(Protocol):
    def estimate(self, rgb, object_mask, object_model, camera_intrinsics):
        """Return PoseEstimate or None; must never return a 2D target."""


class Legacy2DGeometryEstimator:
    """Diagnostic adapter for the existing contour implementation.

    Its output is deliberately not a PoseEstimate and cannot enter grasp
    planning. Pass projects/vision_experiments/identify_cube.detect_frame as
    the callback when comparing old and new overlays on the same RGB frame.
    """
    def __init__(self, detect_frame):
        self.detect_frame = detect_frame

    def diagnose(self, bgr, tag_detector, trash_detector=None):
        return self.detect_frame(bgr, tag_detector, trash_detector)


class IdentityFusion:
    """Per-track discounted log evidence; conflicting strong cues abstain."""
    def __init__(self, commit_threshold=0.80, margin=0.25, decay=0.85):
        self.logits = np.zeros(4, dtype=float)
        self.commit_threshold, self.margin, self.decay = commit_threshold, margin, decay

    def update(self, evidence):
        self.logits *= self.decay
        for cue in evidence:
            if cue.cube_id not in CUBE_NAMES:
                continue
            r = float(np.clip(cue.reliability, 0.01, 0.99))
            weight = {"tag": 4.0, "color": 1.5, "dino": 1.0}.get(cue.source, 1.0)
            likelihood = np.full(4, (1 - r) / 3)
            likelihood[cue.cube_id - 1] = r
            self.logits += weight * np.log(np.maximum(likelihood, 1e-6))
        shifted = self.logits - np.max(self.logits)
        probabilities = np.exp(shifted)
        probabilities /= probabilities.sum()
        ranked = np.argsort(probabilities)[::-1]
        top, second = probabilities[ranked[0]], probabilities[ranked[1]]
        committed = int(ranked[0] + 1) if (top >= self.commit_threshold and
                                            top - second >= self.margin) else 0
        return committed, probabilities


def mask_iou(first, second):
    if first.shape != second.shape:
        return 0.0
    union = np.count_nonzero(first | second)
    return np.count_nonzero(first & second) / union if union else 0.0


def instance_image_score(inst1, inst2):
    """First-stage 2D association: mask IoU + bbox centroid distance."""
    iou = mask_iou(inst1.mask, inst2.mask)
    if iou >= 0.25:
        return iou + 1.0  # Uu tien IOU
    b1, b2 = inst1.bbox, inst2.bbox
    cx1, cy1 = (b1[0] + b1[2]) / 2, (b1[1] + b1[3]) / 2
    cx2, cy2 = (b2[0] + b2[2]) / 2, (b2[1] + b2[3]) / 2
    dist = ((cx1 - cx2)**2 + (cy1 - cy2)**2)**0.5
    diag = ((b1[2] - b1[0])**2 + (b1[3] - b1[1])**2)**0.5
    if dist < diag * 0.8:
        return 1.0 - (dist / (diag * 0.8))
    return 0.0


def instance_similarity(inst1, inst2):
    """Backward-compatible 2D-only score. Prefer track_association_score."""
    return instance_image_score(inst1, inst2)


def world_distance_score(track_point, det_point, sigma_m=0.05):
    """High weight when both have valid base-frame positions.

    track_point/det_point: (3,) xyz in base frame or None.
    Returns 0..1, or None when world comparison is unavailable.
    """
    if track_point is None or det_point is None:
        return None
    try:
        dist = float(np.linalg.norm(np.asarray(track_point, float).reshape(3) -
                                    np.asarray(det_point, float).reshape(3)))
    except (TypeError, ValueError):
        return None
    if not np.isfinite(dist):
        return None
    return float(np.exp(-(dist / sigma_m) ** 2))


def track_association_score(instance, track, timestamp=None,
                            det_world_point=None, max_gap=0.75,
                            world_weight=0.6, sigma_m=0.05):
    """Combined image + world association score.

    Image IoU/centroid stays as first-stage; world distance dominates
    when both sides have valid base-frame positions (eye-in-hand motion).
    Returns 0.0 when track is too old.
    """
    if timestamp is not None:
        try:
            if timestamp - track.last_seen > max_gap:
                return 0.0
        except TypeError:
            pass
    image = instance_image_score(instance, track.instance)
    track_point = None
    if getattr(track, "base_T_object", None) is not None:
        try:
            track_point = np.asarray(track.base_T_object, float)[:3, 3]
        except (TypeError, ValueError, IndexError):
            track_point = None
    world = world_distance_score(track_point, det_world_point, sigma_m)
    if world is None:
        return image
    # World evidence has high weight: a static cube keeps its track_id
    # even when IoU drops to ~0 under strong camera motion.
    return (1.0 - world_weight) * min(image, 2.0) / 2.0 + world_weight * world * 2.0

@dataclass
class TrackedObject:
    track_id: str
    instance: ObjectInstance
    last_seen: float
    fusion: IdentityFusion = field(default_factory=IdentityFusion)
    pose: PoseEstimate | None = None
    pose_model_id: int = 0
    base_T_object: np.ndarray | None = None
    object_id: int = 0
    probabilities: np.ndarray = field(default_factory=lambda: np.full(4, 0.25))
    state: str = "VISIBLE"
    stable_count: int = 0
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_pose_at: float = 0.0


class ObjectTracker:
    def __init__(self, min_iou=0.25, max_gap=0.75, stable_frames=5,
                 world_weight=0.6, world_sigma_m=0.05):
        self.tracks = {}
        self.next_id = 1
        self.min_iou, self.max_gap, self.stable_frames = min_iou, max_gap, stable_frames
        self.world_weight = world_weight
        self.world_sigma_m = world_sigma_m

    def update(self, instances, timestamp, world_points=None):
        """Associate instances to tracks.

        instances: list[ObjectInstance] (image space, required).
        world_points: optional list aligned with instances, each a (3,)
            xyz in base frame or None (provisional RGB-geometry estimate).
            When provided, world distance has high weight so a static cube
            keeps its track_id despite IoU~0 under eye-in-hand motion.
        """
        if world_points is None:
            world_points = [None] * len(instances)
        elif len(world_points) != len(instances):
            raise ValueError("world_points must align with instances")
        order = sorted(range(len(instances)),
                       key=lambda i: -instances[i].confidence)
        matches = set()
        for idx in order:
            instance = instances[idx]
            det_world = world_points[idx]
            ranked = sorted(
                ((track_association_score(
                    instance, track, timestamp, det_world,
                    self.max_gap, self.world_weight, self.world_sigma_m), key)
                 for key, track in self.tracks.items()
                 if key not in matches and
                 timestamp - track.last_seen <= self.max_gap),
                reverse=True)
            if ranked and ranked[0][0] > 0.0:
                key = ranked[0][1]
                track = self.tracks[key]
                track.instance = instance
                track.last_seen = timestamp
                if track.state != "LOCKED_FOR_GRASP":
                    track.state = "VISIBLE"
            else:
                key = f"object_{self.next_id:03d}"
                self.next_id += 1
                track = TrackedObject(key, instance, timestamp)
                self.tracks[key] = track
            instance.track_id = key
            matches.add(key)
        for key, track in list(self.tracks.items()):
            if key not in matches and track.state != "LOCKED_FOR_GRASP":
                track.state = "OCCLUDED" if timestamp - track.last_seen <= self.max_gap else "LOST"
            if track.state == "LOST":
                del self.tracks[key]
        return [self.tracks[key] for key in matches]

    def observe_pose(self, track, pose, base_T_object, timestamp,
                     max_step_m=0.015, max_step_deg=10):
        if track.state == "LOCKED_FOR_GRASP":
            return
        previous = track.base_T_object
        if previous is not None:
            distance = np.linalg.norm(previous[:3, 3] - base_T_object[:3, 3])
            delta = previous[:3, :3].T @ base_T_object[:3, :3]
            angle = np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1)))
            track.stable_count = track.stable_count + 1 if (distance <= max_step_m and
                                                            angle <= max_step_deg) else 1
            dt = timestamp - track.last_pose_at
            if dt > 0:
                track.velocity = (base_T_object[:3, 3] - previous[:3, 3]) / dt
        else:
            track.stable_count = 1
        track.pose = pose
        track.base_T_object = base_T_object.copy()
        track.last_pose_at = timestamp
        if track.stable_count >= self.stable_frames:
            track.state = "STABLE"


class AprilTagPoseEstimator:
    def __init__(self, detector=None):
        self.detector = detector

    def estimate_tag(self, corners, model, camera_matrix, distortion):
        if model.tag_T_cube is None or model.tag_size_m <= 0:
            return None
        side = model.tag_size_m / 2
        tag_points = np.array([[-side, -side, 0], [side, -side, 0],
                               [side, side, 0], [-side, side, 0]], np.float64)
        image_points = np.asarray(corners, np.float64).reshape(4, 2)
        K = np.asarray(camera_matrix, np.float64).reshape(3, 3)
        D = np.asarray(distortion, np.float64)
        solutions = cv2.solvePnPGeneric(tag_points, image_points, K, D,
                                        flags=cv2.SOLVEPNP_IPPE)
        if not solutions[0]:
            return None
        candidates = []
        for rvec, tvec in zip(solutions[1], solutions[2]):
            if tvec[2, 0] <= 0:
                continue
            projected = cv2.projectPoints(tag_points, rvec, tvec, K, D)[0].reshape(4, 2)
            error = float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=1))))
            camera_T_tag = np.eye(4)
            camera_T_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
            camera_T_tag[:3, 3] = tvec.ravel()
            candidates.append((error, camera_T_tag @ model.tag_T_cube))
        if not candidates:
            return None
        error, camera_T_cube = min(candidates, key=lambda item: item[0])
        if error > 2.0:
            return None
        return PoseEstimate(rigid(camera_T_cube), max(0.0, 1.0 - error / 2.0),
                            error, "apriltag_ippe", orientation_valid=True)

    def estimate(self, rgb, object_mask, object_model, camera_intrinsics):
        if self.detector is None:
            return None
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        mask = np.asarray(object_mask, dtype=bool)
        instance = ObjectInstance((0, 0, rgb.shape[1], rgb.shape[0]), mask, 1.0)
        candidates = [self.estimate_tag(tag.corners, object_model,
                      camera_intrinsics["K"], camera_intrinsics["D"])
                      for tag in self.detector.detect(gray)
                      if int(tag.tag_id) == object_model.tag_id and
                      tag_in_instance(tag.corners, instance)]
        candidates = [candidate for candidate in candidates if candidate is not None]
        return min(candidates, key=lambda p: p.reprojection_error_px) if candidates else None



class RGBPoseEstimator(PoseEstimator):
    """Geometry fallback RGB estimator; AprilTag is optional, not required.

    A trained RGB pose model can replace/upgrade this class without touching
    tracking, TF, ObjectState, grasp planner or motion worker. Until then,
    estimate depth from the known cube edge + pinhole model so perception
    keeps producing T_camera_object when the tag face is hidden (Case B).
    Confidence is deliberately lower than AprilTag IPPE.
    """
    def __init__(self, model_path=None):
        self.model_path = model_path
        # Trained weights are optional; geometry fallback always available.

    def estimate(self, rgb, object_mask, object_model, camera_intrinsics):
        try:
            mask = np.asarray(object_mask, dtype=bool)
        except (TypeError, ValueError):
            return None
        if mask.size == 0 or not np.count_nonzero(mask):
            return None
        try:
            K = np.asarray(camera_intrinsics["K"], dtype=np.float64).reshape(3, 3)
        except (KeyError, TypeError, ValueError):
            return None
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if not (fx > 0 and fy > 0):
            return None
        ys, xs = np.nonzero(mask)
        if len(xs) < 100:
            return None
        x1, x2 = int(xs.min()), int(xs.max())
        y1, y2 = int(ys.min()), int(ys.max())
        px = max(1, x2 - x1 + 1)
        py = max(1, y2 - y1 + 1)
        try:
            size_m = float(object_model.dimensions[0])
        except (AttributeError, TypeError, ValueError, IndexError):
            return None
        if not np.isfinite(size_m) or size_m <= 0:
            return None
        # Apparent size -> depth. Use geometric mean to soften perspective.
        pixel_size = float(np.sqrt(px * py))
        depth = float(fx * size_m / max(1.0, pixel_size))
        if not np.isfinite(depth) or not 0.03 <= depth <= 2.0:
            return None
        u = float(xs.mean())
        v = float(ys.mean())
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        camera_T_object = np.eye(4)
        camera_T_object[:3, 3] = [x, y, depth]
        # Identity rotation = unknown yaw/pitch/roll; low confidence by design.
        # Đây là position-only fallback: KHÔNG có orientation để grasp 6D.
        coverage = float(np.count_nonzero(mask) / mask.size)
        confidence = float(np.clip(0.30 + 0.25 * coverage, 0.30, 0.55))
        reproj = float(pixel_size * 0.05)
        return PoseEstimate(rigid(camera_T_object), confidence, reproj,
                            "rgb_geometry", orientation_valid=False)


def pose_distance(pos_a, pos_b):
    """Return (position_m, angle_deg) between two T_camera_object."""
    try:
        a, b = rigid(pos_a), rigid(pos_b)
    except ValueError:
        return float("inf"), 180.0
    dist = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    delta = a[:3, :3].T @ b[:3, :3]
    angle = float(np.degrees(np.arccos(
        np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0))))
    return dist, angle


def fuse_poses(tag_pose, rgb_pose, pos_tol_m=0.03, ang_tol_deg=20.0):
    """Fuse optional AprilTag + RGB measurements.

    Returns (fused_or_None, status) with status in:
      fused | tag_only | rgb_only | conflict | none
    Conflict (both valid but disagree) must NOT be blind-trusted (Case G).
    """
    if tag_pose is None and rgb_pose is None:
        return None, "none"
    if tag_pose is not None and rgb_pose is None:
        return tag_pose, "tag_only"
    if rgb_pose is not None and tag_pose is None:
        return rgb_pose, "rgb_only"
    dist, angle = pose_distance(tag_pose.camera_T_object,
                                rgb_pose.camera_T_object)
    if dist <= pos_tol_m and angle <= ang_tol_deg:
        # High-confidence tag dominates position; keep tag method label.
        fused = PoseEstimate(
            tag_pose.camera_T_object.copy(),
            float(min(0.99, 0.6 * tag_pose.confidence + 0.4 * rgb_pose.confidence + 0.1)),
            float(min(tag_pose.reprojection_error_px, rgb_pose.reprojection_error_px)),
            "apriltag_rgb_fused", orientation_valid=True)
        return fused, "fused"
    return None, "conflict"


def instance_regions(bgr, instance, min_side_px=28):
    """Split an instance bbox into visible-surface region crops for DINO.

    A cube can show 2-3 faces at once; a single full-cube crop must NOT be
    assumed to be one face. Returns [(region_id, crop_bgr, coverage)] with
    region_id in {full, top, bottom, left, right}. Caller batches them
    through DINO and fuses per-region evidence.
    """
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in instance.bbox]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 - x1 < min_side_px or y2 - y1 < min_side_px:
        return []
    try:
        mask = np.asarray(instance.mask, dtype=bool)
    except (TypeError, ValueError):
        return []
    boxes = {"full": (x1, y1, x2, y2),
             "top": (x1, y1, x2, (y1 + y2) // 2),
             "bottom": (x1, (y1 + y2) // 2, x2, y2),
             "left": (x1, y1, (x1 + x2) // 2, y2),
             "right": ((x1 + x2) // 2, y1, x2, y2)}
    out = []
    for rid, (ax1, ay1, ax2, ay2) in boxes.items():
        if ax2 - ax1 < min_side_px // 2 or ay2 - ay1 < min_side_px // 2:
            continue
        m = mask[ay1:ay2, ax1:ax2] if mask.shape == (h, w) else None
        coverage = float(np.count_nonzero(m) / max(1, m.size)) if m is not None else 1.0
        if coverage < 0.25:
            continue
        out.append((rid, bgr[ay1:ay2, ax1:ax2].copy(), coverage))
    return out

def color_evidence(bgr, instance):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    valid = instance.mask.astype(bool)
    area = np.count_nonzero(valid)
    if area == 0:
        return []
    result = []
    for cube_id, ranges in HSV.items():
        selected = np.zeros(valid.shape, bool)
        for low, high in ranges:
            selected |= cv2.inRange(hsv, np.array(low), np.array(high)).astype(bool)
        selected &= valid
        coverage = np.count_nonzero(selected) / area
        if coverage >= 0.12 and np.median(hsv[:, :, 1][selected]) >= 95:
            result.append(Evidence("color", cube_id, min(0.90, 0.55 + coverage)))
    return result


def tag_in_instance(corners, instance, min_fraction=0.75):
    points = np.rint(corners).astype(int)
    height, width = instance.mask.shape
    return (np.count_nonzero([(0 <= x < width and 0 <= y < height and
                               instance.mask[y, x] != 0) for x, y in points]) / 4
            >= min_fraction)


def tag_seed_instances(tags, models, estimator, K, distortion, shape):
    """Project known geometry to form a commissioning-only instance mask."""
    height, width = shape
    instances = []
    for tag in tags:
        model = models.get(int(tag.tag_id))
        if model is None:
            continue
        pose = estimator.estimate_tag(tag.corners, model, K, distortion)
        if pose is None:
            continue
        points = (pose.camera_T_object[:3, :3] @ model.vertices().T).T
        points += pose.camera_T_object[:3, 3]
        pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K,
                                   distortion)[0].reshape(-1, 2)
        hull = cv2.convexHull(np.rint(pixels).astype(np.int32))
        mask = np.zeros((height, width), np.uint8)
        cv2.fillConvexPoly(mask, hull, 1)
        ys, xs = np.nonzero(mask)
        if not len(xs):
            continue
        bbox = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
        confidence = float(np.clip(tag.decision_margin / 80.0, 0.5, 0.99))
        instances.append(ObjectInstance(bbox, mask.astype(bool), confidence))
    return instances
