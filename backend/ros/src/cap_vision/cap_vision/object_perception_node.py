"""Instance-first RGB perception for the four tagged cubes.

Missing segmentation weights, physical dimensions, or accepted calibration
leave base_pose_valid/pose_valid false. A tag-seeded projected mask is allowed
for commissioning and dataset generation, but never replaces the trained
segmenter for tag-hidden operation. This node never commands hardware.
"""
from __future__ import annotations

import importlib.util
import json
import time
from copy import deepcopy
from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from cap_scene_interfaces.msg import ObjectState as ObjectStateMsg, ObjectStates
from cap_vision.cube_face_pose import (CubeFacePoseEstimator, FaceObservation,
                                       extract_face_quads, extract_scene_face_quads,
                                       enclosing_white_face_quad, face_seed_instances, load_face_models,
                                       rectify_quad)
from cap_vision.object_pipeline import (AprilTagPoseEstimator, CubeModel, Evidence,
                                        ObjectInstance, ObjectTracker, RGBPoseEstimator,
                                        color_evidence, fuse_poses, instance_regions,
                                        is_full_6d_pose, rigid, tag_in_instance, tag_seed_instances,
                                        HSV, TRASH_TO_ID)
from cap_vision.red_scene_node import rotation_quaternion, tf_matrix


DISPLAY_CONFIRM_FRAMES = 3


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def latest_image_qos():
    """Latest-frame QoS; native rclpy profiles cannot be deep-copied.

    RELIABLE, not BEST_EFFORT: 900 KB camera frames sent best-effort were dropped in transit (measured
    ~5 fps with 1-4 s holes), starving tracking and the approval viewer. KEEP_LAST depth 1 still means a
    slow consumer only ever sees the newest frame.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)


def extend_ai_packages(repo_root):
    """Expose AI-only venv packages after ROS has loaded its NumPy ABI."""
    if not repo_root:
        return None
    candidates = sorted((Path(repo_root) / ".venv" / "lib").glob("python*/site-packages"))
    for candidate in candidates:
        value = str(candidate)
        if candidate.is_dir() and value not in sys.path:
            # NumPy/OpenCV/cv_bridge are imported above, so their ROS-compatible
            # modules are already pinned in sys.modules.  Put the AI venv first
            # for subsequent imports so torch, torchvision, ultralytics and
            # pupil_apriltags all come from one coherent environment.  Appending
            # here mixed ~/.local torch with .venv torchvision and disabled both
            # YOLOE and AprilTag detection at runtime.
            sys.path.insert(0, value)
            return value
    return None


def load_models(path):
    with open(path, encoding="utf-8") as stream:
        specs = yaml.safe_load(stream)["cubes"]
    models = {}
    for raw_id, spec in specs.items():
        if all(spec.get(key) is not None for key in ("size_m", "tag_size_m", "cube_T_tag")):
            cube_id = int(raw_id)
            models[cube_id] = CubeModel.measured(cube_id, float(spec["size_m"]),
                                                float(spec["tag_size_m"]), spec["cube_T_tag"])
    return models


def segment(frame, model, confidence=0.5):
    # YOLOE open-vocab path: model owns text + visual prompts.
    segment_fn = getattr(model, "segment", None)
    if callable(segment_fn):
        try:
            return segment_fn(frame, conf=confidence)
        except TypeError:
            return segment_fn(frame)
    result = model.predict(frame, conf=confidence, verbose=False)[0]
    if result.masks is None:
        return []
    height, width = frame.shape[:2]
    instances = []
    for box, mask in zip(result.boxes, result.masks.data.cpu().numpy()):
        x1, y1, x2, y2 = (int(round(v)) for v in box.xyxy[0].cpu().numpy())
        if x2 <= x1 or y2 <= y1:
            continue
        if mask.shape != (height, width):
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        binary = mask > 0.5
        if np.count_nonzero(binary) < 100:
            continue
        instances.append(ObjectInstance((x1, y1, x2, y2), binary,
                                        float(box.conf[0])))
    return instances


def pose_message(matrix, header, frame):
    msg = PoseStamped()
    msg.header = deepcopy(header)
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, matrix[:3, 3])
    q = rotation_quaternion(matrix[:3, :3])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
    return msg


class ObjectPerceptionNode(Node):
    def __init__(self):
        super().__init__("object_perception")
        for name, default in (("image_topic", "/cap_vision/image_raw"),
                              ("model_path", ""), ("config", ""),
                              ("color_hsv_config", ""),
                              ("models", ""), ("face_geometry", ""),
                              # cube_4x6_face_geometry.yaml is the owner-provided
                              # physical face convention.  Treat it as authoritative
                              # unless an experiment explicitly asks for provisional
                              # diagnostics.
                              ("face_mapping_verified", True),
                              ("base_frame", "base_link"),
                              ("mount_frame", "measured/Camera_Link"),
                              ("dino_enabled", False), ("dino_threshold", 0.35),
                              ("repo_root", ""),
                               ("allow_tag_seed_fallback", False),
                              ("mask_confidence", 0.5), ("max_age_sec", 0.75),
                              ("text_prompts", "cube,toy block,colored cube"),
                              ("visual_prompt_path", ""),
                              ("trash_core_enabled", True),
                              ("face_table_top_z", 0.03),
                              ("face_grow_enabled", True),
                               ("trash_white_s_max", 70),
                               ("trash_white_v_min", 140),
                               ("trash_face_expand_max", 2.5),
                               ("trash_fallback_stable_frames", 3),
                               ("provisional_uncalibrated_base", False),
                               ("smoothing_preset", "normal"),
                               ("display_smoothing", True),
                               ("overlay_debug", False)):
            self.declare_parameter(name, default)
        self.bridge = CvBridge()
        # DINO, segmentation and timestamped TF can put processed frames more
        # than 0.75 s apart. Keep identity across that gap; grasp freshness is
        # checked separately by the bridge against the latest observation.
        self.tracker = ObjectTracker(max_gap=3.0)
        # Vibration-tolerant display path: raw pose stays conservative for
        # grasp decisions; overlay uses a heavier EMA + corner median.
        # Relaxed preset only affects the display path + trash voting.
        self.smoothing_preset = str(
            self.get_parameter("smoothing_preset").value).lower()
        if self.smoothing_preset not in ("normal", "relaxed"):
            self.smoothing_preset = "normal"
        self.display_smoothing = bool(
            self.get_parameter("display_smoothing").value)
        self.overlay_debug = bool(self.get_parameter("overlay_debug").value)
        from collections import deque
        self.pose_display = {}
        self.corner_history = {}
        self.trash_vote = {}
        self._deque = deque
        self.estimator = AprilTagPoseEstimator()
        self.rgb_estimator = RGBPoseEstimator()
        self.face_estimator = CubeFacePoseEstimator()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(ObjectStates, "/vision/object_states", 10)
        self.markers = self.create_publisher(MarkerArray, "/vision/object_markers", 10)
        self.annotated = self.create_publisher(Image, "/vision/object_annotated", 2)
        self.status = self.create_publisher(String, "/vision/object_status", 10)
        # 4 góc từng mặt đã nhận (pixel) theo track: T8 dùng chúng để dựng pose/tầng từ góc khớp
        # khi PnP mặt đơn thất bại (cube_vision/gravity_pose), không cần đổi msg ROS.
        self.face_quads = self.create_publisher(String, "/vision/face_quads", 10)
        self.base = self.get_parameter("base_frame").value
        self.mount = self.get_parameter("mount_frame").value
        self.max_age = float(self.get_parameter("max_age_sec").value)
        self.models = load_models(self.get_parameter("models").value)
        geometry_path = str(self.get_parameter("face_geometry").value)
        if geometry_path:
            self.models = load_face_models(geometry_path, self.models)
        self.face_mapping_verified = bool(self.get_parameter("face_mapping_verified").value)
        with open(self.get_parameter("config").value, encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        hsv_path = str(self.get_parameter("color_hsv_config").value)
        if not hsv_path:
            hsv_path = str(Path(self.get_parameter("config").value).with_name(
                "cube_color_hsv.yaml"))
        self.load_cube_hsv(hsv_path)
        camera = self.config["camera"]
        if camera.get("source") == "assumed_urdf_camera_link":
            self.get_logger().warning(
                "Using URDF Camera_Link and the existing 270-degree optical fit; "
                "camera centre and FOV are assumed, not independently calibrated")
        self.K = np.asarray(camera["K"], float).reshape(3, 3)
        self.D = np.asarray(camera["distortion"], float)
        validation = self.config.get("calibration_validation", {})
        held_out = validation.get("held_out_errors_m", [])
        accepted = (camera.get("source") in {"checkerboard_eye_in_hand",
                                                "fixed_apriltag_eye_in_hand"}
                    and len(held_out) >= 3
                    and all(isinstance(error, (int, float)) and
                            np.isfinite(error) and 0 <= error <= 0.005
                            for error in held_out))
        self.calibrated = (bool(camera.get("calibrated")) and
                           bool(camera.get("extrinsic_calibrated")) and
                           camera.get("mount_T_optical") is not None and accepted)
        if not self.calibrated:
            self.get_logger().warning(
                "Camera/base pose disabled: use independently validated intrinsics "
                "and eye-in-hand calibration (>=3 held-out errors <=5mm)")
        self.mount_T_optical = (rigid(camera["mount_T_optical"])
                                if self.calibrated else None)
        # Debug-only provisional base: reuse the assumed URDF mount when the
        # operator explicitly opts in. Truthful `calibrated` stays False; only
        # base_T_* publication is enabled for DRY-RUN visualization/preflight.
        self.provisional_base = bool(
            self.get_parameter("provisional_uncalibrated_base").value)
        self.mount_T_provisional = None
        if not self.calibrated and self.provisional_base:
            try:
                self.mount_T_provisional = rigid(camera["mount_T_optical"])
            except (KeyError, TypeError, ValueError):
                self.mount_T_provisional = None
            if self.mount_T_provisional is not None:
                self.get_logger().warning(
                    "PROVISIONAL UNCALIBRATED BASE: publishing assumed "
                    "base poses for DRY-RUN debug only; DO NOT grasp. "
                    "Run eye-in-hand calibration for real motion.")
        external_packages = extend_ai_packages(self.get_parameter("repo_root").value)
        if external_packages:
            self.get_logger().info(f"AI package fallback: {external_packages}")
        model_path = Path(self.get_parameter("model_path").value)
        self.segmenter = None
        self.get_logger().info(f"DEBUG: model_path={model_path}, exists={model_path.is_file()}")
        self.segment_mode = "none"
        if model_path.is_file():
            try:
                if "yoloe" in model_path.name.lower():
                    from cap_vision.yoloe_segmenter import YOLOESegmenter

                    prompts = [p.strip() for p in
                               str(self.get_parameter("text_prompts").value).split(",")
                               if p.strip()]
                    self.segmenter = YOLOESegmenter(
                        str(model_path), text_prompts=prompts,
                        conf=float(self.get_parameter("mask_confidence").value))
                    vp = str(self.get_parameter("visual_prompt_path").value)
                    if vp and Path(vp).is_file():
                        self.segmenter.load_enrollment(vp)
                        self.get_logger().info(f"YOLOE visual enrollment: {vp}")
                    self.segment_mode = "yoloe-visual" if self.segmenter.visual_prompts else "yoloe-text"
                    self.get_logger().info(
                        f"YOLOE open-vocab segmenter: prompts={self.segmenter.active_prompts}")
                else:
                    from ultralytics import YOLO
                    self.segmenter = YOLO(str(model_path))
                    self.segment_mode = "yolo-closed"
            except (ImportError, RuntimeError, ValueError) as exc:
                self.get_logger().warning(
                    f"segmentation initialization failed; tag-only mode: {exc}")
        else:
            self.get_logger().warning(
                "Instance segmentation weights missing; tag-seeded commissioning masks only")
        self.tag_detector = None
        try:
            from dt_apriltags import Detector
        except ImportError:
            try:
                from pupil_apriltags import Detector
            except ImportError:
                Detector = None
        if Detector is not None:
            self.tag_detector = Detector(families="tag36h11", nthreads=4,
                                         quad_decimate=1.0, refine_edges=1)
        else:
            self.get_logger().warning("AprilTag library missing in ROS Python environment; tag pose disabled")
        self.dino = None
        self.face_tools = None
        self.last_base_T_camera = None
        self.grown_faces = []   # [(monotonic stamp, FaceProposal)]
        self.grow_index = 0
        if self.get_parameter("dino_enabled").value:
            source = (Path(self.get_parameter("repo_root").value) /
                      "projects/vision_experiments/identify_cube.py")
            if source.is_file():
                spec = importlib.util.spec_from_file_location("legacy_cube_identity", source)
                module = importlib.util.module_from_spec(spec)
                import sys
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                self.dino = module.TrashDetector(
                    thresh=float(self.get_parameter("dino_threshold").value),
                    margin=0.0)
                try:
                    sys.path.insert(0, str(Path(self.get_parameter("repo_root").value) / "projects"))
                    from cube_vision import faces as face_tools
                    self.face_tools = face_tools
                except ImportError as exc:
                    self.get_logger().warning(f"cube_vision.faces unavailable: {exc}")
            else:
                self.get_logger().warning("DINO adapter source missing; cue disabled")
        self.last_stamp = 0.0
        self.pose_smoothing = {}
        self.scene_probe_index = 0
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self.on_image, latest_image_qos())

    def load_cube_hsv(self, path):
        """Load exactly the calibrated Yahboom-compatible cube color bounds."""
        with open(path, encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)["colors"]
        loaded = {}
        for spec in raw.values():
            cube_id = int(spec["id"])
            ranges = []
            for low, high in spec["ranges"]:
                low = tuple(int(v) for v in low)
                high = tuple(int(v) for v in high)
                if (len(low) != 3 or len(high) != 3 or
                        any(a < 0 or b > limit or a > b
                            for a, b, limit in zip(low, high, (179, 255, 255)))):
                    raise ValueError(f"invalid cube HSV range for id={cube_id}")
                ranges.append((low, high))
            if cube_id not in (1, 2, 3, 4) or not ranges:
                raise ValueError(f"invalid cube HSV entry for id={cube_id}")
            loaded[cube_id] = tuple(ranges)
        if set(loaded) != {1, 2, 3, 4}:
            raise ValueError("cube HSV config must define IDs 1, 2, 3, 4")
        HSV.clear()
        HSV.update(loaded)
        self.get_logger().info(f"Cube HSV bounds loaded: {path}")

    def smooth_corners(self, key, corners):
        """Median of last 3 quads to calm 1-2px vibration jitter pre-PnP."""
        history = self.corner_history.get(key)
        if history is None:
            history = self._deque(maxlen=3)
            self.corner_history[key] = history
        history.append(np.asarray(corners, float).copy())
        return np.median(np.stack(list(history)), axis=0)

    def vote_trash_label(self, track_id, cube_id, source):
        """Require 3/5 same trash id+source before switching display label."""
        history = self.trash_vote.get(track_id)
        if history is None:
            history = self._deque(maxlen=5)
            self.trash_vote[track_id] = history
        history.append((int(cube_id), str(source)))
        if len(history) < 3:
            return int(cube_id), str(source), False
        votes = list(history)[-5:]
        best = max(set(votes), key=votes.count)
        stable = votes.count(best) >= 3
        return best[0], best[1], stable

    def display_camera_T(self, track_id, raw_T, timestamp):
        """Filter overlay poses; require three agreeing frames before drawing."""
        raw_T = np.asarray(raw_T, float)
        slot = self.pose_display.get(track_id)
        if slot is None or timestamp - slot["stamp"] > self.tracker.max_gap:
            slot = {"T": raw_T.copy(), "hist": self._deque(maxlen=5),
                    "stamp": timestamp, "confirmed": False,
                    "pending": raw_T.copy(), "pending_count": 1}
            slot["hist"].append(raw_T[:3, 3].copy())
            self.pose_display[track_id] = slot
            return None
        reference = slot["T"] if slot["confirmed"] else slot["pending"]
        delta_r, _ = cv2.Rodrigues(reference[:3, :3].T @ raw_T[:3, :3])
        close = (np.linalg.norm(reference[:3, 3] - raw_T[:3, 3]) <= 0.012 and
                 np.linalg.norm(delta_r) <= np.deg2rad(15.0))
        if not slot["confirmed"]:
            slot["stamp"] = timestamp
            if close:
                slot["pending_count"] += 1
                if slot["pending_count"] >= DISPLAY_CONFIRM_FRAMES:
                    slot["confirmed"] = True
                    slot["T"] = raw_T.copy()
                    slot["hist"].clear()
                    slot["hist"].append(raw_T[:3, 3].copy())
                    return raw_T.copy()
            else:
                slot["pending"] = raw_T.copy()
                slot["pending_count"] = 1
            return None
        if not close:
            pending = slot.get("pending")
            if pending is not None:
                pending_r, _ = cv2.Rodrigues(pending[:3, :3].T @ raw_T[:3, :3])
                consistent = (np.linalg.norm(pending[:3, 3] - raw_T[:3, 3]) <= 0.006 and
                              np.linalg.norm(pending_r) <= np.deg2rad(8.0))
            else:
                consistent = False
            slot["pending_count"] = slot.get("pending_count", 0) + 1 if consistent else 1
            slot["pending"] = raw_T.copy()
            slot["stamp"] = timestamp
            if slot["pending_count"] < DISPLAY_CONFIRM_FRAMES:
                return slot["T"].copy()
            # A persistent new pose is probably real movement; reacquire it
            # without drawing an interpolation between incompatible poses.
            slot["T"] = raw_T.copy()
            slot["hist"].clear()
            slot["hist"].append(raw_T[:3, 3].copy())
            slot["pending"] = None
            slot["pending_count"] = 0
            return raw_T.copy()
        slot["pending"] = None
        slot["pending_count"] = 0
        alpha = 0.15 if self.smoothing_preset == "normal" else 0.12
        slot["hist"].append(raw_T[:3, 3].copy())
        med_t = np.median(np.stack(list(slot["hist"])), axis=0)
        # Apply a relative rotation step to avoid Rodrigues +/-pi wraparound.
        try:
            delta_r, _ = cv2.Rodrigues(slot["T"][:3, :3].T @ raw_T[:3, :3])
            step_r, _ = cv2.Rodrigues(alpha * delta_r)
            R_mix = slot["T"][:3, :3] @ step_r
        except cv2.error:
            R_mix = raw_T[:3, :3]
        T = np.eye(4)
        T[:3, :3] = R_mix
        T[:3, 3] = (1.0 - alpha) * slot["T"][:3, 3] + alpha * med_t
        slot["T"] = T.copy()
        slot["stamp"] = timestamp
        return T

    def smooth_camera_pose(self, track_id, pose, model_id, timestamp):
        """Align single-face square symmetry, then filter pose per track."""
        current = np.asarray(pose.camera_T_object, dtype=float).copy()
        previous = self.pose_smoothing.get(track_id)
        alpha = 0.35
        if previous is not None and previous["model"] == model_id and \
                0.0 < timestamp - previous["stamp"] <= self.tracker.max_gap:
            old = previous["pose"]
            if pose.symmetry_axis_local is not None:
                axis = np.asarray(pose.symmetry_axis_local, dtype=float)
                axis /= max(1e-9, np.linalg.norm(axis))
                candidates = []
                for turns in range(4):
                    theta = np.deg2rad(90.0 * turns)
                    c, s = np.cos(theta), np.sin(theta)
                    x, y, z = axis
                    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    local_turn = c * np.eye(3) + (1-c) * np.outer(axis, axis) + s * skew
                    rotation = current[:3, :3] @ local_turn
                    delta = old[:3, :3].T @ rotation
                    angle = float(np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1)))
                    candidates.append((angle, rotation))
                current[:3, :3] = min(candidates, key=lambda item: item[0])[1]
            delta_r, _ = cv2.Rodrigues(old[:3, :3].T @ current[:3, :3])
            angle = float(np.linalg.norm(delta_r))
            distance = float(np.linalg.norm(old[:3, 3] - current[:3, 3]))
            if angle <= np.deg2rad(10.0) and distance <= 0.015:
                step_r, _ = cv2.Rodrigues(delta_r * alpha)
                current[:3, :3] = old[:3, :3] @ step_r
                current[:3, 3] = (1.0 - alpha) * old[:3, 3] + alpha * current[:3, 3]
                previous["pending"] = None
                previous["pending_count"] = 0
            else:
                pending = previous.get("pending")
                pending_count = previous.get("pending_count", 0)
                if pending is not None:
                    p_delta, _ = cv2.Rodrigues(pending[:3, :3].T @ current[:3, :3])
                    p_angle = float(np.linalg.norm(p_delta))
                    p_distance = float(np.linalg.norm(pending[:3, 3] - current[:3, 3]))
                    if p_angle <= np.deg2rad(5.0) and p_distance <= 0.005:
                        pending_count += 1
                    else:
                        pending_count = 1
                else:
                    pending_count = 1
                if pending_count >= 3:
                    previous["pending"] = None
                    previous["pending_count"] = 0
                else:
                    previous["pending"] = current.copy()
                    previous["pending_count"] = pending_count
                    current = old.copy()
                    pose.method = "rgb_pose_reacquiring"
                    pose.confidence = 0.0
        else:
            previous = {"pose": current.copy(), "model": model_id,
                        "stamp": timestamp, "pending": None, "pending_count": 0}
        pose.camera_T_object = current
        if previous is None:
            previous = {"pending": None, "pending_count": 0}
        previous.update({"pose": current.copy(), "model": model_id, "stamp": timestamp})
        self.pose_smoothing[track_id] = previous
        return pose

    def on_image(self, msg):
        started = time.perf_counter()
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if stamp <= self.last_stamp or not 0 <= now - stamp <= self.max_age:
            self.publish_empty(msg, "stale or unordered frame")
            return
        self.last_stamp = stamp
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if bgr.shape[:2] != (self.config["camera"]["height"],
                                  self.config["camera"]["width"]):
                raise ValueError("frame resolution differs from calibration")
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            tags = self.tag_detector.detect(gray) if self.tag_detector is not None else []
            instances = segment(bgr, self.segmenter,
                                float(self.get_parameter("mask_confidence").value)) \
                if self.segmenter is not None else []
            segmentation_ms = 1000.0 * (time.perf_counter() - started)
            if self.get_parameter("allow_tag_seed_fallback").value:
                # Add a projected tag instance for every visible tag that is
                # not already owned by YOLOE. Do this per tag; testing only
                # `not instances` dropped later tags whenever one proposal
                # happened to exist elsewhere in the frame.
                uncovered_tags = [tag for tag in tags
                                  if not any(tag_in_instance(tag.corners, instance)
                                             for instance in instances)]
                instances.extend(tag_seed_instances(
                    uncovered_tags, self.models, self.estimator,
                    self.K, self.D, bgr.shape[:2]))
            # YOLOE is only one proposal source. A recognised physical face
            # must also be able to seed an instance, otherwise HSV/DINO would
            # never run when the tag is hidden and YOLOE misses the cube.
            semantic_seeds = self.scene_face_seed_instances(bgr)
            proposals_ms = 1000.0 * (time.perf_counter() - started)
            instances = self.merge_core_anchors(instances, semantic_seeds)
            # World-space association hint: provisional base-frame points from
            # RGB geometry so a static cube keeps its track_id when IoU~0.
            # TF must use the image stamp, never "current" TF (eye-in-hand).
            base_T_camera_hint = None
            hint_optical = (self.mount_T_optical if self.calibrated
                            else self.mount_T_provisional)
            if hint_optical is not None:
                try:
                    tf_hint = self.tf.lookup_transform(
                        self.base, self.mount, Time.from_msg(msg.header.stamp))
                    base_T_camera_hint = tf_matrix(tf_hint) @ hint_optical
                except TransformException:
                    base_T_camera_hint = None
            intrinsics = {"K": self.K, "D": self.D}
            world_points = []
            for inst in instances:
                det_world = None
                try:
                    # Geometry model id unknown pre-identity: use cube 1 size
                    # (all cubes share 30mm edge) for the association hint only.
                    any_model = next(iter(self.models.values()), None)
                    if any_model is not None and base_T_camera_hint is not None:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        est = self.rgb_estimator.estimate(
                            rgb, inst.mask, any_model, intrinsics)
                        if est is not None:
                            base_T_obj = base_T_camera_hint @ est.camera_T_object
                            det_world = base_T_obj[:3, 3].copy()
                except (ValueError, cv2.error):
                    det_world = None
                world_points.append(det_world)
            tracks = self.tracker.update(instances, stamp, world_points)
            self.pose_smoothing = {
                key: value for key, value in self.pose_smoothing.items()
                if key in self.tracker.tracks and
                stamp - value.get("stamp", stamp) <= self.tracker.max_gap
            }
            # Prune vibration-filter state for dead tracks (bounded memory).
            live = set(self.tracker.tracks)
            self.pose_display = {k: v for k, v in self.pose_display.items()
                                 if k in live}
            self.trash_vote = {k: v for k, v in self.trash_vote.items()
                               if k in live}
            self.corner_history = {k: v for k, v in self.corner_history.items()
                                   if k[0] in live} if self.corner_history else {}
            associations = {track.track_id: [] for track in tracks}
            for tag in tags:
                owners = [track for track in tracks if tag_in_instance(tag.corners, track.instance)]
                if len(owners) == 1:
                    associations[owners[0].track_id].append(tag)
            base_T_camera = None
            is_provisional_frame = False
            provisional_tf_latest = False
            active_optical = (self.mount_T_optical if self.calibrated
                              else self.mount_T_provisional)
            if active_optical is not None:
                try:
                    # Real joints arrive at ~0.67 Hz while images arrive at
                    # 15 Hz. Wait for a joint sample newer than this image so
                    # tf2 can interpolate at the actual capture time. The TF
                    # listener runs in another executor thread (see main()).
                    tf = self.tf.lookup_transform(self.base, self.mount,
                                                   Time.from_msg(msg.header.stamp),
                                                   timeout=Duration(seconds=0.6))
                    base_T_camera = tf_matrix(tf) @ active_optical
                    is_provisional_frame = (not self.calibrated and
                                            self.mount_T_provisional is not None and
                                            base_T_camera is not None)
                except TransformException:
                    # Joints publish at ~0.67 Hz with measurement-time stamps,
                    # so the exact-stamp lookup often extrapolates into the
                    # future. Provisional DRY-RUN only: fall back to the
                    # latest TF (arm must be stationary; labelled below).
                    # Calibrated path stays exact-stamp only (grasp safety).
                    if not self.calibrated and self.mount_T_provisional is not None:
                        try:
                            tf_latest = self.tf.lookup_transform(
                                self.base, self.mount, Time())
                            base_T_camera = tf_matrix(tf_latest) @ active_optical
                            is_provisional_frame = base_T_camera is not None
                            provisional_tf_latest = is_provisional_frame
                        except TransformException:
                            pass
            self.last_base_T_camera = base_T_camera
            out = ObjectStates()
            out.header = deepcopy(msg.header)
            out.header.frame_id = self.base
            out.calibrated = self.calibrated and base_T_camera is not None
            if out.calibrated:
                prefix = "ok"
            elif not self.calibrated:
                prefix = "camera/hand-eye calibration not validated"
            else:
                prefix = "timestamped base-to-camera TF unavailable"
            if is_provisional_frame:
                prefix = "PROVISIONAL-UNCALIBRATED-BASE DRY-RUN ONLY; " + prefix
            mode = "segmenter" if self.segmenter is not None else "tag-seed"
            out.status = (f"{prefix}; mode={mode}; tags={len(tags)}; "
                          f"instances={len(instances)}; face_seeds={len(semantic_seeds)}")
            if is_provisional_frame:
                out.status += "; provisional_base=true"
                if provisional_tf_latest:
                    out.status += "; provisional_tf=latest-not-stamped"
            overlay = bgr.copy()
            # Show the detector result even when metric pose or base TF is not
            # available. Otherwise a real 2D detection looks like no detection.
            if self.overlay_debug:
                for tag in tags:
                    corners = np.rint(tag.corners).astype(np.int32)
                    cv2.polylines(overlay, [corners], True, (255, 80, 255), 2,
                                  cv2.LINE_AA)
                    self.draw_status_label(overlay, tuple(corners[0]),
                                           f"tag {int(tag.tag_id)}", (255, 160, 255))
            markers = []
            processed = []
            for track in tracks:
                item = self.process_track(track, associations[track.track_id], bgr,
                                          msg.header, base_T_camera)
                if is_provisional_frame and item.base_pose_valid:
                    extra = ("PROVISIONAL uncalibrated base DRY-RUN ONLY; "
                             + ("TF latest-not-stamped, arm must be stationary; "
                                if provisional_tf_latest else ""))
                    item.reason = extra + item.reason
                processed.append((track, item))
            for track, item in self.dedupe_pose_results(processed):
                out.objects.append(item)
                if self.overlay_debug:
                    for cube_id, face in getattr(track, "face_detections", []):
                        corners = np.rint(face.corners).astype(np.int32)
                        cv2.polylines(overlay, [corners], True, (255, 200, 0), 2,
                                      cv2.LINE_AA)
                        label = self.models[cube_id].semantic_faces.get(face.axis, "")
                        text = f"{face.axis} {label}"
                        if face.axis != "-Z":
                            # Trash/DINO faces flip under vibration; vote 3/5
                            # before trusting the display label (grasp keeps raw).
                            _, _, stable = self.vote_trash_label(
                                track.track_id, cube_id, track.proposal_source)
                            if not stable:
                                text += "?"
                        self.draw_status_label(overlay, tuple(corners[0]),
                                               text, (255, 220, 90))
                if item.pose_valid and not is_provisional_frame:
                    markers.extend(self.surface_markers(track, item, msg.header))
                    self.publish_tf(track, item)
                display_pose_valid = self.should_draw_cube_pose(item, self.models)
                if display_pose_valid:
                    model = self.models.get(track.pose_model_id)
                    if model is not None:
                        # Overlay uses the heavy display filter; grasp keeps raw.
                        disp_T = track.pose.camera_T_object
                        if self.display_smoothing:
                            try:
                                disp_T = self.display_camera_T(
                                    track.track_id, track.pose.camera_T_object,
                                    stamp_seconds(msg.header.stamp))
                            except (ValueError, cv2.error):
                                disp_T = None
                        if disp_T is None:
                            continue
                        xyz = (disp_T[:3, :3] @ model.vertices().T).T \
                              + disp_T[:3, 3]
                        px = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                               self.K, self.D)[0].reshape(-1, 2).astype(int)
                        wire_color = ((255, 255, 0) if track.pose.orientation_valid
                                      else (0, 165, 255))
                        for a in range(8):
                            for bit in (1, 2, 4):
                                b = a ^ bit
                                if a < b:
                                    cv2.line(overlay, tuple(px[a]), tuple(px[b]), wire_color, 1)
                        if self.overlay_debug:
                            axis_m = model.dimensions[0] * 0.65
                            object_axes = np.array([[0, 0, 0], [axis_m, 0, 0],
                                                    [0, axis_m, 0], [0, 0, axis_m]], float)
                            camera_axes = (disp_T[:3, :3] @ object_axes.T).T
                            camera_axes += disp_T[:3, 3]
                            axis_px = cv2.projectPoints(camera_axes, np.zeros(3),
                                                        np.zeros(3), self.K, self.D)[0]
                            axis_px = np.rint(axis_px.reshape(-1, 2)).astype(int)
                            for idx, color in enumerate(((0, 0, 255), (0, 255, 0),
                                                          (255, 0, 0)), start=1):
                                cv2.line(overlay, tuple(axis_px[0]), tuple(axis_px[idx]),
                                         color, 2, cv2.LINE_AA)
                        # TCP is the centre of the physical face pointing at
                        # the eye-in-hand camera, never the YOLO bbox centre.
                        face_centres = []
                        for axis_index in range(3):
                            for sign in (-1.0, 1.0):
                                local = np.zeros(3)
                                local[axis_index] = sign * model.dimensions[0] / 2
                                normal = disp_T[:3, :3] @ (np.eye(3)[axis_index] * sign)
                                if -float(normal[2]) >= 0.65:
                                    centre = (disp_T[:3, :3] @ local +
                                              disp_T[:3, 3])
                                    face_centres.append((-float(normal[2]), centre))
                        if face_centres:
                            _facing, centre = max(face_centres, key=lambda item: item[0])
                            tcp = cv2.projectPoints(centre.reshape(1, 3), np.zeros(3),
                                                    np.zeros(3), self.K, self.D)[0].reshape(2)
                            tcp = np.rint(tcp).astype(int)
                        else:
                            tcp = np.rint(px.mean(axis=0)).astype(int)
                        cv2.drawMarker(overlay, tuple(tcp), (255, 0, 255),
                                       cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
                        cv2.putText(overlay, "TCP", (tcp[0] + 7, tcp[1] - 7),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                                    (255, 0, 255), 1, cv2.LINE_AA)
                        anchor = (int(px[:, 0].min()), int(px[:, 1].min()))
                        self.draw_pose_label(overlay, anchor, track, item,
                                             detailed=self.overlay_debug)
                else:
                    state_text = (f"ID {item.object_id or '?'} | {track.track_id} | 2D only"
                                  if self.overlay_debug else
                                  f"ID {item.object_id or '?'} | 2D only")
                    self.draw_status_label(
                        overlay,
                        (int(track.instance.bbox[0]), int(track.instance.bbox[1])),
                        state_text)
            if not tracks:
                self.draw_status_label(overlay, (8, bgr.shape[0] - 8),
                                       f"No cube proposal | tags={len(tags)} "
                                       f"faces={len(semantic_seeds)}")
            total_ms = 1000.0 * (time.perf_counter() - started)
            out.status += (f"; seg_ms={segmentation_ms:.0f}; "
                           f"proposal_ms={proposals_ms-segmentation_ms:.0f}; "
                           f"total_ms={total_ms:.0f}")
            self.publisher.publish(out)
            self.face_quads.publish(String(data=self.face_quads_json(msg.header, processed)))
            clear = Marker(action=Marker.DELETEALL)
            self.markers.publish(MarkerArray(markers=[clear, *markers]))
            self.status.publish(String(data=out.status))
            annotated = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            annotated.header = deepcopy(msg.header)
            self.annotated.publish(annotated)
        except (ValueError, TransformException, cv2.error) as exc:
            self.publish_empty(msg, str(exc))

    @staticmethod
    def draw_status_label(image, anchor, text, color=(230, 230, 230)):
        """Draw one readable label with an opaque background."""
        font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x = int(np.clip(anchor[0], 2, max(2, image.shape[1] - tw - 8)))
        y = int(np.clip(anchor[1] - 7, th + 8, image.shape[0] - baseline - 3))
        cv2.rectangle(image, (x - 3, y - th - 4),
                      (x + tw + 4, y + baseline + 3), (20, 20, 20), -1)
        cv2.putText(image, text, (x, y), font, scale, color, thickness,
                    cv2.LINE_AA)

    def draw_pose_label(self, image, anchor, track, item, detailed=False):
        identity = f"ID {item.object_id}" if item.object_id else "ID ?"
        validity = ("6D" if track.pose.orientation_valid and item.top_grasp_ready else
                    "TOP-GRASP" if item.top_grasp_ready else "WAIT")
        text = f"{identity} | {validity}"
        if detailed:
            text += (f" | {track.pose.method} | "
                     f"{track.proposal_source}:{track.proposal_streak}/"
                     f"fallback:{track.fallback_streak}")
        self.draw_status_label(image, anchor, text,
                               (80, 255, 120) if validity == "6D"
                               else (0, 190, 255))

    @staticmethod
    def should_draw_cube_pose(item, models):
        """An uncommitted identity or position-only pose is not a cube box."""
        return (item.camera_pose_valid and item.object_id in models and
                item.geometry_model_id == item.object_id and
                item.pose_method in {"apriltag_ippe", "apriltag_rgb_fused",
                                     "rgb_faces_pnp", "rgb_single_face_cube"})

    @staticmethod
    def face_quads_json(header, processed):
        """JSON {stamp, tracks: {track_id: {object_id, faces: [{axis, quad, conf}]}}}."""
        tracks = {}
        for track, item in processed:
            faces = []
            for _cube_id, face in getattr(track, "face_detections", []):
                quad = np.asarray(face.corners, float).reshape(-1, 2)
                if quad.shape == (4, 2) and np.isfinite(quad).all():
                    faces.append({"axis": str(face.axis), "quad": quad.round(1).tolist(),
                                  "conf": round(float(face.confidence), 3)})
            if faces:
                tracks[str(item.track_id)] = {"object_id": int(item.object_id), "faces": faces}
        return json.dumps({"stamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
                           "tracks": tracks})

    @staticmethod
    def _inside_same_id(track, item, kept):
        """Có đúng 4 cube, mỗi ID một khối: đối tượng thứ hai cùng ID là ghost/bằng chứng thừa.

        Bỏ nếu đã giữ (xếp hạng cao hơn) một đối tượng cùng ID có pose đo được (bất kể vị trí:
        các ghost như vệt màu trên thảm nằm xa cube), hoặc nếu tâm nó nằm trong khung đối tượng cùng ID.
        """
        object_id = getattr(item, "object_id", None)
        if not object_id:
            return False
        box = getattr(getattr(track, "instance", None), "bbox", None)
        for other_track, other in kept:
            if getattr(other, "object_id", None) != object_id:
                continue
            if getattr(other, "camera_pose_valid", False):
                return True
            other_box = getattr(getattr(other_track, "instance", None), "bbox", None)
            if box is None or other_box is None:
                continue
            cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
            if other_box[0] <= cx <= other_box[2] and other_box[1] <= cy <= other_box[3]:
                return True
        return False

    @staticmethod
    def dedupe_pose_results(processed, max_center_distance_m=0.012):
        """Suppress duplicate 3D poses without changing masks or estimation.

        Separate 30 mm cubes cannot have centres within 12 mm. Keep 2D-only
        observations until they have a metric pose; screen proximity alone is
        insufficient to decide that neighbouring cubes are the same object.
        """
        ranked = sorted(processed, key=lambda pair: (
            bool(pair[1].top_grasp_ready), bool(pair[1].camera_pose_valid),
            float(pair[1].pose_confidence), float(pair[1].identity_confidence)),
            reverse=True)
        kept = []
        for track, item in ranked:
            if item.camera_pose_valid:
                center = np.asarray(track.pose.camera_T_object[:3, 3], float)
                if any(other.camera_pose_valid and
                       np.linalg.norm(center - other_track.pose.camera_T_object[:3, 3])
                       < max_center_distance_m for other_track, other in kept):
                    continue
            # Chỉ có 4 cube, mỗi ID một khối: đối tượng nằm trong khung của đối tượng tốt hơn
            # cùng ID (mảnh/mặt phụ/ước lượng thứ hai) là bằng chứng của cube đó, không phải
            # một cube nữa. Hai cube thật cùng ID không tồn tại.
            if ObjectPerceptionNode._inside_same_id(track, item, kept):
                continue
            kept.append((track, item))
        return kept

    def scene_face_seed_instances(self, bgr):
        """Seed from a semantic face; trash seeds require a non-white core."""
        quads = extract_scene_face_quads(bgr)
        labelled = []
        dino_quads = []
        tools = getattr(self, "face_tools", None)
        if tools is not None and not self.get_parameter("face_grow_enabled").value:
            tools = None
        prior = None
        if tools is not None:
            prior = (tools.expected_side_prior(
                getattr(self, "last_base_T_camera", None), self.K,
                float(self.get_parameter("face_table_top_z").value), image_width=bgr.shape[1])
                if getattr(self, "last_base_T_camera", None) is not None
                else tools.fallback_prior(bgr.shape[1]))
        for quad in quads:
            # Quad quá nhỏ là nhiễu; mảnh hình in (PART) không bao giờ thành đối tượng bằng ô
            # màu hay mặt "trash_partial": nó chỉ được dùng làm mỏ neo, và chỉ thành đối tượng
            # khi mặt trắng bao quanh được đo (trash_outer_face) hoặc dựng được (grown).
            role = tools.classify_quad(quad, prior) if prior is not None else None
            if tools is not None and role == tools.NOISE:
                continue
            is_part = tools is not None and role == tools.PART
            center = np.rint(np.mean(quad, axis=0)).astype(int)
            cx, cy = map(int, center)
            crop = rectify_quad(bgr, quad)
            crop_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            best = None
            for cube_id, ranges in HSV.items():
                mask = np.zeros(crop_hsv.shape[:2], np.uint8)
                for low, high in ranges:
                    mask |= cv2.inRange(crop_hsv, np.asarray(low), np.asarray(high))
                # Ignore a narrow border where cube bevel/background leaks in.
                mask[:12] = mask[-12:] = 0
                mask[:, :12] = mask[:, -12:] = 0
                coverage = float(np.count_nonzero(mask) / mask.size)
                if best is None or coverage > best[0]:
                    best = (coverage, cube_id)
            if best and best[0] >= 0.55:
                if is_part:
                    continue        # mảng màu nhỏ (vệt thảm, chi tiết hình in) không phải mặt cube
                labelled.append((best[1], quad, min(0.95, 0.55 + best[0] * 0.4),
                                 (float(cx), float(cy)), "hsv_face"))
            elif self.dino is not None and getattr(self.dino, "model", None) is not None:
                # Identify the printed graphic as an anchor before accepting
                # any surrounding white material as part of the object.
                if self.get_parameter("trash_core_enabled").value:
                    s_max = int(self.get_parameter("trash_white_s_max").value)
                    v_min = int(self.get_parameter("trash_white_v_min").value)
                    white = (crop_hsv[:, :, 1] <= s_max) & (crop_hsv[:, :, 2] >= v_min)
                    inner = np.zeros(white.shape, bool)
                    margin = max(4, int(round(white.shape[0] * 0.08)))
                    inner[margin:-margin, margin:-margin] = True
                    core = (~white) & inner
                    core_u8 = cv2.morphologyEx(core.astype(np.uint8), cv2.MORPH_CLOSE,
                                               np.ones((3, 3), np.uint8))
                    count, labels, stats, centroids = cv2.connectedComponentsWithStats(core_u8)
                    candidates = [i for i in range(1, count)
                                  if stats[i, cv2.CC_STAT_AREA] >= max(20, int(core.size * 0.012))]
                    if not candidates:
                        continue
                    core_id = max(candidates, key=lambda i: stats[i, cv2.CC_STAT_AREA])
                    core_fraction = float(stats[core_id, cv2.CC_STAT_AREA] / core.size)
                    if core_fraction > 0.65:
                        dino_quads.append((quad, crop, (float(cx), float(cy))))
                        continue
                    core_xy = centroids[core_id]
                    dst = np.float32([[0, 0], [crop.shape[1]-1, 0],
                                      [crop.shape[1]-1, crop.shape[0]-1],
                                      [0, crop.shape[0]-1]])
                    inverse = cv2.getPerspectiveTransform(dst, np.asarray(quad, np.float32))
                    anchor = cv2.perspectiveTransform(
                        np.asarray(core_xy, np.float32).reshape(1, 1, 2), inverse)[0, 0]
                    dino_quads.append((quad, crop, tuple(map(float, anchor))))
                else:
                    dino_quads.append((quad, crop, (float(cx), float(cy))))
        # On the robot CPU four DINO crops take >1 s, making every published
        # ObjectState stale at the bridge. Probe one contour per frame. Hold
        # it for three frames so the bridge can confirm two successive poses,
        # then rotate to the next candidate.
        colour_quads = [np.asarray(item[1], np.float32) for item in labelled]
        dino_quads = [item for item in dino_quads
                      if not any(cv2.pointPolygonTest(face, item[2], False) >= 0
                                 for face in colour_quads)]
        if dino_quads:
            probe_count = getattr(self, "scene_probe_index", 0)
            probe_index = (probe_count // 3) % len(dino_quads)
            self.scene_probe_index = probe_count + 1
            dino_quads = [dino_quads[probe_index]]
        if dino_quads:
            try:
                results = self.dino.match_many([crop for _, crop, _ in dino_quads])
            except (ValueError, RuntimeError, cv2.error):
                results = []
            for (quad, _crop, anchor), (label, score, detail) in zip(dino_quads, results):
                margin = float(detail.get("margin", 0.0)) if isinstance(detail, dict) else 0.0
                cube_id = TRASH_TO_ID.get((label or "").lower())
                if (cube_id and
                        score >= float(self.get_parameter("dino_threshold").value) and
                        margin >= 0.08):
                    outer = None
                    if self.get_parameter("trash_core_enabled").value:
                        # DINO may label either the artwork or its white face.
                        # Try every smaller printed contour contained by this
                        # one, then the classified contour as the core.
                        possible_cores = [np.asarray(q, np.float32) for q in quads
                            if abs(cv2.contourArea(np.asarray(q, np.float32))) <
                               0.85 * abs(cv2.contourArea(np.asarray(quad, np.float32))) and
                            cv2.pointPolygonTest(np.asarray(quad, np.float32),
                                tuple(map(float, np.mean(q, axis=0))), False) >= 0]
                        for core_quad in [*possible_cores, quad]:
                            outer = enclosing_white_face_quad(
                                bgr, core_quad, quads, anchor,
                                int(self.get_parameter("trash_white_s_max").value),
                                int(self.get_parameter("trash_white_v_min").value),
                                float(self.get_parameter("trash_face_expand_max").value))
                            if outer is not None:
                                break
                    source = ("trash_outer_face" if outer is not None else
                              "trash_partial" if self.get_parameter(
                                  "trash_core_enabled").value else "trash_face")
                    if source == "trash_partial" and is_part:
                        continue    # mảnh hình in không dựng được mặt trắng: chỉ là mỏ neo, xem grown_face_labels
                    labelled.append((cube_id, outer if outer is not None else quad,
                                     min(0.95, score + margin), anchor, source,
                                     (label or "").lower()))
        verified = [item for item in labelled if item[4] == "trash_outer_face"]
        labelled = [item for item in labelled
                    if item[4] != "trash_partial" or not any(
                        outer[0] == item[0] and
                        cv2.pointPolygonTest(np.asarray(outer[1], np.float32),
                                             tuple(map(float, item[3])), False) >= 0 and
                        abs(cv2.contourArea(np.asarray(item[1], np.float32))) <=
                        1.05 * abs(cv2.contourArea(np.asarray(outer[1], np.float32)))
                        for outer in verified)]
        if (prior is not None and self.dino is not None and
                getattr(self.dino, "model", None) is not None):
            labelled.extend(self.grown_face_labels(bgr, quads, prior, labelled))
        return ObjectPerceptionNode.merge_same_cube_seeds(
            face_seed_instances(bgr.shape[:2], labelled, expansion=0.0))

    @staticmethod
    def merge_same_cube_seeds(instances, gap_px: int = 30):
        """Gộp các hạt giống cùng ID chạm nhau (mặt trên + mặt bên của cùng một cube).

        Mỗi hạt chỉ là một mặt; nếu để riêng, mặt màu bên hông cho PnP một mặt nghiêng
        ("cube nghiêng", không có mặt trên nằm ngang) còn mặt hình in thành đối tượng thứ hai.
        Gộp thành một đối tượng để hai mặt cùng vào một lần PnP; mặt có nhãn rác làm hạt chính.
        """
        kernel = np.ones((2 * gap_px + 1, 2 * gap_px + 1), np.uint8)
        groups = []
        for inst in instances:
            near = [g for g in groups
                    if g[0].seed_cube_id == inst.seed_cube_id and
                    np.any(cv2.dilate(g[0].mask.astype(np.uint8), kernel) & inst.mask.astype(np.uint8))]
            merged = [inst]
            for g in near:
                groups.remove(g)
                merged.extend(g)
            groups.append(merged)
        output = []
        for members in groups:
            if len(members) == 1:
                output.append(members[0])
                continue
            primary = max(members, key=lambda m: (bool(m.seed_label), m.confidence))
            mask = np.zeros_like(primary.mask, bool)
            for m in members:
                mask |= m.mask.astype(bool)
            ys, xs = np.nonzero(mask)
            primary.mask = mask
            primary.bbox = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
            primary.confidence = max(m.confidence for m in members)
            # Giữ quad từng mặt đã gộp để PnP dùng cả hai mặt (process_track).
            extra = list(getattr(primary, "extra_faces", []))
            for m in members:
                extra.extend(getattr(m, "extra_faces", []))
                if m is not primary and m.seed_quad is not None:
                    extra.append((m.seed_cube_id, m.seed_quad, m.proposal_source,
                                  m.seed_label, m.confidence))
            primary.extra_faces = extra
            output.append(primary)
        return output

    GROWN_FACE_TTL_S = 8.0
    GROWN_FACE_MIN_HITS = 2
    GROW_MIN_INTERVAL_S = 1.5

    def grown_face_labels(self, bgr, quads, prior, existing):
        """Mặt đủ dựng từ mảnh hình in (cube_vision.faces.propose_faces).

        DINO chậm trên CPU nên mỗi khung chỉ xử lý 2 mỏ neo luân phiên. Một mặt dựng sai
        thường không lặp lại đúng chỗ ở lần dò sau, nên chỉ phát đi mặt đã được dựng lại
        >= GROWN_FACE_MIN_HITS lần cùng nhãn ở cùng chỗ trong GROWN_FACE_TTL_S giây.
        """
        tools = self.face_tools
        now = time.monotonic()
        self.grown_faces = [e for e in self.grown_faces if now - e[0] <= self.GROWN_FACE_TTL_S]
        # Camera đã dịch/xoay thì ảnh đổi: mặt đã nhớ không còn đúng chỗ.
        pose = getattr(self, "last_base_T_camera", None)
        ref = getattr(self, "grown_pose", None)
        if pose is not None:
            if ref is not None:
                moved = np.linalg.norm(pose[:3, 3] - ref[:3, 3])
                turned = np.degrees(np.arccos(np.clip(
                    (np.trace(pose[:3, :3].T @ ref[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
                if moved > 0.012 or turned > 2.5:
                    self.grown_faces = []
                    self.grown_pose = pose
            else:
                self.grown_pose = pose
        fresh = []
        # DINO trên nhiều crop mất ~1 s/khung (CPU): chỉ dò lại mỗi GROW_MIN_INTERVAL_S,
        # các khung giữa dùng kết quả đã nhớ để không kéo tụt tần số ObjectStates.
        if now - getattr(self, "last_grow_s", 0.0) >= self.GROW_MIN_INTERVAL_S:
            try:
                fresh = tools.propose_faces(bgr, quads, prior, self.dino, max_anchors=3,
                                            start=self.grow_index, max_parts=30,
                                            K=getattr(self, "K", None), dist=getattr(self, "D", None))
            except (ValueError, RuntimeError, TypeError, cv2.error):
                fresh = []
            self.grow_index += 3
            self.last_grow_s = time.monotonic()
        shape = bgr.shape[:2]
        for proposal in fresh:
            hits = 1
            keep = []
            for entry in self.grown_faces:
                same = (entry[1].label == proposal.label and
                        tools._overlap(entry[1].quad, proposal.quad, shape) > 0.4)
                if same:
                    hits = max(hits, entry[2] + 1)
                elif tools._overlap(entry[1].quad, proposal.quad, shape) <= 0.5:
                    keep.append(entry)
            self.grown_faces = keep + [(now, proposal, hits)]
        output = []
        for _, proposal, hits in self.grown_faces:
            cube_id = TRASH_TO_ID.get((proposal.label or "").lower())
            if not cube_id or hits < self.GROWN_FACE_MIN_HITS:
                continue
            centre = tuple(map(float, np.mean(proposal.quad, axis=0)))
            # Bỏ nếu đã có mặt đo trực tiếp (màu/viền trắng) cùng cube ở chỗ này.
            if any(item[0] == cube_id and
                   cv2.pointPolygonTest(np.asarray(item[1], np.float32), centre, False) >= 0
                   for item in existing):
                continue
            output.append((cube_id, proposal.quad,
                           min(0.9, proposal.score + proposal.margin), centre,
                           "trash_grown_face", (proposal.label or "").lower()))
        return output

    @staticmethod
    def merge_core_anchors(instances, anchors):
        """Attach a validated face/core proposal to a matching YOLO mask."""
        output = list(instances)
        for anchor in anchors:
            point = anchor.anchor_center
            owner = None
            best_overlap = 0.0
            if point is not None:
                x, y = (int(round(v)) for v in point)
                for idx, instance in enumerate(output):
                    if not (0 <= y < instance.mask.shape[0] and
                            0 <= x < instance.mask.shape[1]):
                        continue
                    patch = instance.mask[max(0, y-3):y+4, max(0, x-3):x+4]
                    overlap = np.count_nonzero(instance.mask & anchor.mask) / max(
                        1, np.count_nonzero(anchor.mask))
                    if (np.count_nonzero(patch) >= 4 or overlap >= 0.25) and \
                            overlap >= best_overlap:
                        owner = idx
                        best_overlap = overlap
            if owner is None:
                output.append(anchor)
                continue
            current = output[owner]
            # The measured face is canonical.  A broad white YOLO proposal
            # must not pull the object onto the table or a neighbouring cube.
            merged = anchor.mask.copy() if anchor.proposal_source in {
                "trash_outer_face", "hsv_face"} \
                else current.mask | anchor.mask
            ys, xs = np.nonzero(merged)
            if not len(xs):
                continue
            current.mask = merged
            current.bbox = (int(xs.min()), int(ys.min()),
                            int(xs.max()) + 1, int(ys.max()) + 1)
            current.proposal_source = anchor.proposal_source
            current.anchor_center = point
            current.confidence = max(current.confidence, anchor.confidence)
            current.seed_cube_id = anchor.seed_cube_id
            current.seed_label = anchor.seed_label
            current.seed_quad = anchor.seed_quad
        return output

    def face_observations(self, bgr, instance, classify_trash=True):
        """Classify rectified physical face candidates, never whole-cube crops."""
        if not any(model.semantic_faces for model in self.models.values()):
            return []
        quads = extract_face_quads(bgr, instance)
        if not quads:
            return []
        crops = [rectify_quad(bgr, quad) for quad in quads]
        detected = []
        if (classify_trash and self.dino is not None and
                getattr(self.dino, "model", None) is not None):
            try:
                results = self.dino.match_many(crops)
            except (ValueError, RuntimeError, cv2.error):
                results = []
            for quad, (label, score, detail) in zip(quads, results):
                margin = float(detail.get("margin", 0.0)) if isinstance(detail, dict) else 0.0
                cube_id = TRASH_TO_ID.get((label or "").lower())
                if cube_id and score >= 0.35 and margin >= 0.08:
                    axes = [axis for axis, value in self.models[cube_id].semantic_faces.items()
                            if value == (label or "").lower()]
                    if axes:
                        # DINO cosine score and class margin carry independent
                        # evidence. Their sum is a better confidence for the
                        # downstream geometric gate than cosine alone.
                        confidence = float(min(0.95, score + margin))
                        detected.append((cube_id, FaceObservation(
                            axes[0], quad, confidence)))
        # A mostly uniform coloured square can supply the -Z face. This high
        # coverage requirement keeps small coloured artwork from becoming a face.
        for quad, crop in zip(quads, crops):
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            for cube_id, ranges in HSV.items():
                selected = np.zeros(hsv.shape[:2], bool)
                for low, high in ranges:
                    selected |= cv2.inRange(hsv, np.array(low), np.array(high)).astype(bool)
                coverage = np.count_nonzero(selected) / selected.size
                if coverage >= 0.65:
                    detected.append((cube_id, FaceObservation("-Z", quad,
                                                               float(min(0.95, coverage)))))
        return detected

    def process_track(self, track, tags, bgr, header, base_T_camera):
        cues = color_evidence(bgr, track.instance)
        if track.instance.seed_cube_id in self.models:
            source = "color" if track.proposal_source == "hsv_face" else "dino"
            cues.append(Evidence(source, track.instance.seed_cube_id,
                                 track.instance.confidence))
        # A valid tag already fixes identity and 6D pose; running DINO over its
        # printed faces again only reduces the live frame rate.
        face_detections = self.face_observations(
            bgr, track.instance,
            classify_trash=not bool(tags) and not track.instance.seed_label and
            track.proposal_source != "hsv_face")
        if track.instance.seed_quad is not None and track.instance.seed_cube_id in self.models:
            cid = track.instance.seed_cube_id
            if track.proposal_source == "hsv_face":
                face_detections.append((cid, FaceObservation(
                    "-Z", track.instance.seed_quad, track.instance.confidence)))
            elif track.instance.seed_label:
                for axis, label in self.models[cid].semantic_faces.items():
                    if label == track.instance.seed_label:
                        face_detections.append((cid, FaceObservation(
                            axis, track.instance.seed_quad,
                            track.instance.confidence)))
                        break
        for cid, quad, source, label, conf in getattr(track.instance, "extra_faces", []):
            if cid not in self.models:
                continue
            if source == "hsv_face":
                face_detections.append((cid, FaceObservation("-Z", quad, conf)))
            elif label:
                for axis, name in self.models[cid].semantic_faces.items():
                    if name == label:
                        face_detections.append((cid, FaceObservation(axis, quad, conf)))
                        break
        track.face_detections = face_detections
        for cube_id, face in face_detections:
            cues.append(Evidence("dino" if face.axis != "-Z" else "color",
                                 cube_id, face.confidence))
        if (not tags and not face_detections and self.dino is not None and
                getattr(self.dino, "model", None) is not None):
            # Per-region evidence: a cube can show 2-3 faces at once, so a
            # single full-cube crop must NOT be forced to one trash class.
            try:
                regions = instance_regions(bgr, track.instance)
            except (ValueError, cv2.error):
                regions = []
            if regions:
                names = [rid for rid, _, _ in regions]
                crops = []
                for _, crop, _ in regions:
                    x1, y1, x2, y2 = track.instance.bbox
                    # Mask out background inside each region crop.
                    h, w = bgr.shape[:2]
                    crops.append(crop)
                try:
                    results = self.dino.match_many(crops)
                except (ValueError, RuntimeError):
                    results = []
                for rid, (label, score, detail) in zip(names, results):
                    try:
                        margin = float(detail.get("margin", 0.0))
                    except (AttributeError, TypeError, ValueError):
                        margin = 0.0
                    cube_id = TRASH_TO_ID.get((label or "").lower())
                    # Region evidence is weaker than a clean full-face match.
                    if cube_id and score >= 0.55 and margin >= 0.08:
                        weight = 0.85 if rid == "full" else 0.65
                        cues.append(Evidence(
                            "dino", cube_id,
                            min(0.85, weight * (0.50 + margin + (score - 0.55)))))
                # Fallback: legacy single masked full-crop if regions failed.
                if not results:
                    x1, y1, x2, y2 = track.instance.bbox
                    crop = bgr[y1:y2, x1:x2].copy()
                    if crop.size:
                        mask = track.instance.mask[y1:y2, x1:x2]
                        crop[~mask] = (127, 127, 127)
                        label, score, detail = self.dino.match_many([crop])[0]
                        margin = float(detail.get("margin", 0.0))
                        cube_id = TRASH_TO_ID.get((label or "").lower())
                        if cube_id and score >= 0.55 and margin >= 0.08:
                            cues.append(Evidence("dino", cube_id,
                                                 min(0.85, 0.50 + margin + (score - 0.55))))
        for tag in tags:
            if int(tag.tag_id) in self.models or int(tag.tag_id) in (1, 2, 3, 4):
                cues.append(Evidence("tag", int(tag.tag_id),
                                     float(np.clip(tag.decision_margin / 40, 0.7, 0.99))))
        track.object_id, track.probabilities = track.fusion.update(cues)
        # ---- Pose: AprilTag is one optional evidence source, not a gate. ----
        intrinsics = {"K": self.K, "D": self.D}
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        tag_pose, rgb_pose = None, None
        tag_model_id, rgb_model_id = 0, 0
        known_tags = {int(tag.tag_id) for tag in tags if int(tag.tag_id) in self.models}
        if len(known_tags) == 1:
            tag_model_id = next(iter(known_tags))
            for tag in tags:
                if int(tag.tag_id) == tag_model_id:
                    try:
                        candidate = self.estimator.estimate_tag(
                            tag.corners, self.models[tag_model_id], self.K, self.D)
                    except (ValueError, cv2.error):
                        candidate = None
                    if candidate is not None and (tag_pose is None or
                                                  candidate.reprojection_error_px < tag_pose.reprojection_error_px):
                        tag_pose = candidate
        # Face labels + measured cube vertices can recover full rotation even
        # when the AprilTag is hidden. Never upgrade an unverified physical
        # face ring to a graspable full-6D method.
        face_try_ids = ([track.object_id] if track.object_id in self.models
                        else sorted(self.models))
        for cid in face_try_ids:
            labelled = [face for owner, face in face_detections if owner == cid]
            if len(labelled) < 2:
                continue
            candidate = self.face_estimator.estimate(labelled, self.models[cid],
                                                     self.K, self.D)
            if candidate is not None:
                if not self.face_mapping_verified:
                    candidate.method = "rgb_faces_provisional"
                    candidate.orientation_valid = False
                rgb_pose, rgb_model_id = candidate, cid
                break
        # One labelled physical face is sufficient for the metric cube
        # wireframe and centre, but leaves a four-way semantic yaw ambiguity.
        # Keep orientation_valid=False so downstream grasp logic cannot confuse
        # this display/TCP fallback with a complete 6D object frame.
        if rgb_pose is None:
            for cid in face_try_ids:
                labelled = [face for owner, face in face_detections if owner == cid]
                if not labelled:
                    continue
                # Thử lần lượt từng mặt (lớn/chắc trước): mặt đầu có thể bị cắt khỏi khung
                # hay quá xiên nên PnP một mặt thất bại trong khi mặt kia dùng được.
                for face in sorted(labelled, key=lambda f: -(f.confidence * abs(
                        cv2.contourArea(np.asarray(f.corners, np.float32))))):
                    candidate = self.face_estimator.estimate_single_face(
                        face, self.models[cid], self.K, self.D, bgr=bgr)
                    if candidate is not None:
                        rgb_pose, rgb_model_id = candidate, cid
                        break
                if rgb_pose is not None:
                    break
        # Position-only RGB fallback when there are too few reliable faces.
        rgb_try_ids = ([track.object_id] if track.object_id in self.models
                       else sorted(self.models))
        for cid in ([] if rgb_pose is not None else rgb_try_ids):
            try:
                cand = self.rgb_estimator.estimate(
                    rgb, np.asarray(track.instance.mask, dtype=bool),
                    self.models[cid], intrinsics)
            except (ValueError, cv2.error, KeyError):
                cand = None
            if cand is not None and (rgb_pose is None or
                                     cand.confidence > rgb_pose.confidence):
                rgb_pose, rgb_model_id = cand, cid
        pose, fusion_status = fuse_poses(tag_pose, rgb_pose)
        pose_model_id = 0
        if fusion_status == "fused":
            pose_model_id = tag_model_id
        elif fusion_status == "tag_only":
            pose_model_id = tag_model_id
        elif fusion_status == "rgb_only":
            # RGB geometry cannot resolve identity by itself: model id follows
            # the fused identity when it matches, else stays unknown.
            pose_model_id = rgb_model_id if track.object_id in (0, rgb_model_id) else 0
            if track.object_id not in (0, rgb_model_id):
                pose, fusion_status = None, "conflict"
        else:  # conflict / none
            pose, pose_model_id = None, 0
        if pose is not None:
            if pose.method in ("rgb_faces_pnp", "rgb_faces_provisional",
                               "rgb_single_face_cube", "rgb_geometry"):
                pose = self.smooth_camera_pose(
                    track.track_id, pose, pose_model_id, stamp_seconds(header.stamp))
            if base_T_camera is not None:
                self.tracker.observe_pose(track, pose,
                                          base_T_camera @ pose.camera_T_object,
                                          stamp_seconds(header.stamp))
            else:
                track.pose = pose
                track.base_T_object = None
                track.stable_count = 0
            track.pose_model_id = pose_model_id
        elif track.state != "LOCKED_FOR_GRASP":
            track.pose = None
            track.base_T_object = None
            track.pose_model_id = 0
            track.stable_count = 0
        item = ObjectStateMsg()
        item.header = deepcopy(header)
        item.track_id = track.track_id
        item.object_type = "cube"
        item.object_id = track.object_id
        item.geometry_model_id = track.pose_model_id
        item.identity_confidence = float(max(track.probabilities))
        item.identity_probabilities = [float(v) for v in track.probabilities]
        item.bbox_xyxy = list(track.instance.bbox)
        item.mask = self.bridge.cv2_to_imgmsg(track.instance.mask.astype(np.uint8) * 255,
                                               "mono8")
        item.mask.header = deepcopy(header)
        item.state = track.state
        item.camera_pose_valid = (track.pose is not None and
                                  track.pose_model_id in self.models and
                                  track.pose.method != "rgb_pose_reacquiring" and
                                  track.pose.confidence > 0.0)
        item.base_pose_valid = (item.camera_pose_valid and track.base_T_object is not None
                                and base_T_camera is not None)
        item.pose_valid = (item.base_pose_valid and track.pose is not None and
                           is_full_6d_pose(track.pose.method, track.pose.orientation_valid))
        proposal_ready = (track.proposal_source in {"trash_outer_face", "tag_seed"} or
                          (track.pose is not None and
                           track.pose.method in {"apriltag_ippe", "apriltag_rgb_fused"}) or
                          track.fallback_streak >= int(self.get_parameter(
                              "trash_fallback_stable_frames").value))
        item.top_grasp_ready = (item.camera_pose_valid and proposal_ready and
                                track.proposal_source != "trash_partial" and
                                track.pose.method in {"apriltag_ippe", "apriltag_rgb_fused",
                                                      "rgb_faces_pnp", "rgb_single_face_cube"} and
                                track.object_id == track.pose_model_id)
        if item.camera_pose_valid:
            item.camera_pose = pose_message(track.pose.camera_T_object, header,
                                            header.frame_id)
            item.pose_confidence = float(track.pose.confidence)
            item.reprojection_error_px = float(track.pose.reprojection_error_px)
            item.pose_method = track.pose.method
            model = self.models[track.pose_model_id]
            item.visible_surfaces = [name for name, _, normal in
                                     model.surfaces(track.pose.camera_T_object)
                                     if normal[2] < -0.2]
        if item.base_pose_valid:
            item.base_pose = pose_message(track.base_T_object, header, self.base)
            if not item.top_grasp_ready:
                if track.pose.method == "rgb_faces_provisional":
                    item.reason = "face mapping not verified"
                elif track.pose.method == "rgb_geometry":
                    item.reason = "position-only pose; orientation unknown"
                elif track.object_id != track.pose_model_id:
                    item.reason = "identity and pose model disagree"
                elif not proposal_ready or track.proposal_source == "trash_partial":
                    item.reason = "top-face proposal not yet stable"
                else:
                    item.reason = "top-grasp geometry unavailable"
        else:
            if fusion_status == "conflict":
                item.reason = "tag/rgb pose conflict; not graspable"
            elif item.camera_pose_valid and not proposal_ready:
                item.reason = "proposal not yet stable or outer trash face unverified"
            elif item.camera_pose_valid and not item.top_grasp_ready:
                item.reason = "camera pose measured; top-grasp geometry or identity unverified"
            elif item.camera_pose_valid:
                item.reason = ("camera pose valid; calibrated base transform unavailable")
            elif tag_pose is None and rgb_pose is None:
                item.reason = "no tag or RGB pose evidence"
            else:
                item.reason = "pose fusion rejected measurement"
        return item

    def surface_markers(self, track, item, header):
        model = self.models[track.pose_model_id]
        marker = Marker()
        marker.header = item.base_pose.header
        marker.ns = "object_frame"
        marker.id = int(track.track_id.split("_")[-1])
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = item.base_pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = model.dimensions[0]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.8, 1.0, 0.35
        output = [marker]
        for index, (_name, center, normal) in enumerate(model.surfaces(track.base_T_object)):
            arrow = Marker()
            arrow.header = deepcopy(item.base_pose.header)
            arrow.ns = f"{track.track_id}_normal"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [Point(x=float(center[0]), y=float(center[1]), z=float(center[2])),
                            Point(x=float(center[0] + normal[0] * 0.025),
                                  y=float(center[1] + normal[1] * 0.025),
                                  z=float(center[2] + normal[2] * 0.025))]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.003, 0.006, 0.006
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.7, 0.0, 0.9
            output.append(arrow)
        return output

    def publish_tf(self, track, item):
        tf = TransformStamped()
        tf.header = item.base_pose.header
        tf.child_frame_id = track.track_id
        p, q = item.base_pose.pose.position, item.base_pose.pose.orientation
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = q
        self.broadcaster.sendTransform(tf)

    def publish_empty(self, image, reason):
        msg = ObjectStates()
        msg.header = deepcopy(image.header)
        msg.header.frame_id = self.base
        msg.calibrated = self.calibrated
        msg.status = reason
        self.publisher.publish(msg)
        self.status.publish(String(data=reason))
        # Keep the viewer diagnostic alive on rejected frames and recoverable
        # processing errors. A missing annotated topic hides the real cause.
        try:
            overlay = self.bridge.imgmsg_to_cv2(image, "bgr8").copy()
            self.draw_status_label(overlay, (8, overlay.shape[0] - 8),
                                   f"Perception: {reason}", (0, 165, 255))
            annotated = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            annotated.header = deepcopy(image.header)
            self.annotated.publish(annotated)
        except (ValueError, cv2.error):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPerceptionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.shutdown()
        except Exception:  # launch SIGINT may already have shut the context down
            pass
