"""Instance-first RGB perception for the four tagged cubes.

Missing segmentation weights, physical dimensions, or accepted calibration
leave base_pose_valid/pose_valid false. A tag-seeded projected mask is allowed
for commissioning and dataset generation, but never replaces the trained
segmenter for tag-hidden operation. This node never commands hardware.
"""
from __future__ import annotations

import importlib.util
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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from cap_scene_interfaces.msg import ObjectState as ObjectStateMsg, ObjectStates
from cap_vision.object_pipeline import (AprilTagPoseEstimator, CubeModel, Evidence,
                                        ObjectInstance, ObjectTracker, RGBPoseEstimator,
                                        color_evidence, fuse_poses, instance_regions,
                                        rigid, tag_in_instance, tag_seed_instances,
                                        TRASH_TO_ID)
from cap_vision.red_scene_node import rotation_quaternion, tf_matrix


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def extend_ai_packages(repo_root):
    """Expose AI-only venv packages after ROS has loaded its NumPy ABI."""
    if not repo_root:
        return None
    candidates = sorted((Path(repo_root) / ".venv" / "lib").glob("python*/site-packages"))
    for candidate in candidates:
        value = str(candidate)
        if candidate.is_dir() and value not in sys.path:
            # Append, never prepend: cv_bridge must retain ROS's NumPy/OpenCV.
            sys.path.append(value)
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
                              ("models", ""), ("base_frame", "base_link"),
                              ("mount_frame", "measured/Camera_Link"),
                               ("dino_enabled", False), ("repo_root", ""),
                               ("allow_tag_seed_fallback", False),
                              ("mask_confidence", 0.5), ("max_age_sec", 0.75),
                              ("text_prompts", "cube,toy block,colored cube"),
                              ("visual_prompt_path", "")):
            self.declare_parameter(name, default)
        self.bridge = CvBridge()
        self.tracker = ObjectTracker()
        self.estimator = AprilTagPoseEstimator()
        self.rgb_estimator = RGBPoseEstimator()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(ObjectStates, "/vision/object_states", 10)
        self.markers = self.create_publisher(MarkerArray, "/vision/object_markers", 10)
        self.annotated = self.create_publisher(Image, "/vision/object_annotated", 2)
        self.status = self.create_publisher(String, "/vision/object_status", 10)
        self.base = self.get_parameter("base_frame").value
        self.mount = self.get_parameter("mount_frame").value
        self.max_age = float(self.get_parameter("max_age_sec").value)
        self.models = load_models(self.get_parameter("models").value)
        with open(self.get_parameter("config").value, encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        camera = self.config["camera"]
        self.K = np.asarray(camera["K"], float).reshape(3, 3)
        self.D = np.asarray(camera["distortion"], float)
        self.calibrated = (bool(camera.get("calibrated")) and
                           bool(camera.get("extrinsic_calibrated")) and
                           camera.get("mount_T_optical") is not None)
        self.mount_T_optical = (rigid(camera["mount_T_optical"])
                                if self.calibrated else None)
        external_packages = extend_ai_packages(self.get_parameter("repo_root").value)
        if external_packages:
            self.get_logger().info(f"AI package fallback: {external_packages}")
        model_path = Path(self.get_parameter("model_path").value)
        self.segmenter = None
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
            except ImportError:
                self.get_logger().warning("ultralytics missing in ROS Python environment; segmentation disabled")
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
        if self.get_parameter("dino_enabled").value:
            source = (Path(self.get_parameter("repo_root").value) /
                      "projects/vision_experiments/identify_cube.py")
            if source.is_file():
                spec = importlib.util.spec_from_file_location("legacy_cube_identity", source)
                module = importlib.util.module_from_spec(spec)
                import sys
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                self.dino = module.TrashDetector()
            else:
                self.get_logger().warning("DINO adapter source missing; cue disabled")
        self.last_stamp = 0.0
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
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
            if (not instances and self.get_parameter("allow_tag_seed_fallback").value):
                instances = tag_seed_instances(tags, self.models, self.estimator,
                                               self.K, self.D, bgr.shape[:2])
            # World-space association hint: provisional base-frame points from
            # RGB geometry so a static cube keeps its track_id when IoU~0.
            # TF must use the image stamp, never "current" TF (eye-in-hand).
            base_T_camera_hint = None
            if self.calibrated:
                try:
                    tf_hint = self.tf.lookup_transform(
                        self.base, self.mount, Time.from_msg(msg.header.stamp))
                    base_T_camera_hint = tf_matrix(tf_hint) @ self.mount_T_optical
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
            associations = {track.track_id: [] for track in tracks}
            for tag in tags:
                owners = [track for track in tracks if tag_in_instance(tag.corners, track.instance)]
                if len(owners) == 1:
                    associations[owners[0].track_id].append(tag)
            base_T_camera = None
            if self.calibrated:
                try:
                    tf = self.tf.lookup_transform(self.base, self.mount,
                                                   Time.from_msg(msg.header.stamp))
                    base_T_camera = tf_matrix(tf) @ self.mount_T_optical
                except TransformException:
                    pass
            out = ObjectStates()
            out.header = deepcopy(msg.header)
            out.header.frame_id = self.base
            out.calibrated = self.calibrated and base_T_camera is not None
            prefix = "ok" if out.calibrated else "calibration or timestamped TF required"
            mode = "segmenter" if self.segmenter is not None else "tag-seed"
            out.status = (f"{prefix}; mode={mode}; tags={len(tags)}; "
                          f"instances={len(instances)}")
            overlay = bgr.copy()
            markers = []
            for track in tracks:
                item = self.process_track(track, associations[track.track_id], bgr,
                                          msg.header, base_T_camera)
                out.objects.append(item)
                x1, y1, x2, y2 = track.instance.bbox
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 180, 255), 2)
                cv2.putText(overlay, f"{track.track_id} ID{item.object_id} {item.state}",
                            (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 255, 0), 1)
                if item.base_pose_valid:
                    markers.extend(self.surface_markers(track, item, msg.header))
                    self.publish_tf(track, item)
                if item.camera_pose_valid:
                    model = self.models.get(track.pose_model_id)
                    if model is not None:
                        xyz = (track.pose.camera_T_object[:3, :3] @ model.vertices().T).T \
                              + track.pose.camera_T_object[:3, 3]
                        px = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3),
                                               self.K, self.D)[0].reshape(-1, 2).astype(int)
                        for a in range(8):
                            for bit in (1, 2, 4):
                                b = a ^ bit
                                if a < b:
                                    cv2.line(overlay, tuple(px[a]), tuple(px[b]),
                                             (255, 255, 0), 1)
            self.publisher.publish(out)
            clear = Marker(action=Marker.DELETEALL)
            self.markers.publish(MarkerArray(markers=[clear, *markers]))
            self.status.publish(String(data=out.status))
            annotated = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            annotated.header = deepcopy(msg.header)
            self.annotated.publish(annotated)
        except (ValueError, TransformException, cv2.error) as exc:
            self.publish_empty(msg, str(exc))

    def process_track(self, track, tags, bgr, header, base_T_camera):
        cues = color_evidence(bgr, track.instance)
        if self.dino is not None and getattr(self.dino, "model", None) is not None:
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
        # RGB fallback uses committed identity when available, else tries each
        # configured cube model and keeps the best geometric estimate.
        rgb_try_ids = ([track.object_id] if track.object_id in self.models
                       else sorted(self.models))
        for cid in rgb_try_ids:
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
                                  track.pose_model_id in self.models)
        item.base_pose_valid = (item.camera_pose_valid and track.base_T_object is not None
                                and base_T_camera is not None)
        item.pose_valid = item.base_pose_valid
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
        else:
            if fusion_status == "conflict":
                item.reason = "tag/rgb pose conflict; not graspable"
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


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # launch SIGINT may already have shut the context down
            pass
