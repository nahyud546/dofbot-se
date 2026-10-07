"""Read-only, timestamped perception. Does not open serial or command a robot."""
import math
from copy import deepcopy
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseArray, TransformStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster
from cap_scene_interfaces.msg import SceneObject, SceneObjects

from cap_vision.red_scene_core import (
    load_config, calibration_error, transform, detect_scene,
    StableObservations, StationaryWindow,
)

ARM_JOINTS = [f"arm{i}_Joint" for i in range(1, 6)]


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tf_matrix(t):
    q, v = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = [v.x, v.y, v.z]
    return transform(out)


def rotation_quaternion(rotation):
    # Rodrigues handles rotations near 180 degrees without trace singularities.
    vector = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = np.linalg.norm(vector)
    xyz = vector * (math.sin(angle / 2) / angle) if angle > 1e-10 else vector / 2
    return [*xyz, math.cos(angle / 2)]


class RedSceneNode(Node):
    def __init__(self):
        super().__init__("red_scene")
        for name, value in [("config", ""), ("image_topic", "/cap_vision/image_raw"),
                            ("joint_topic", "/real_joint_states"), ("base_frame", "base_link"),
                            ("mount_frame", "measured/Camera_Link"), ("legacy_topics", False)]:
            self.declare_parameter(name, value)
        self.cfg = load_config(self.get_parameter("config").value)
        self.base = self.get_parameter("base_frame").value
        self.mount = self.get_parameter("mount_frame").value
        self.error = calibration_error(self.cfg)
        p = self.cfg["perception"]
        self.filter = StableObservations(p["stable_frames"], p["stable_radius_m"], p["max_age_sec"])
        self.stationary = StationaryWindow(p["stationary_window_sec"], p["stationary_delta_rad"])
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.static_tf = StaticTransformBroadcaster(self)
        if not self.error:
            transforms = []
            for parent, child, value in [
                (self.mount, "measured/camera_optical_frame", self.cfg["camera"]["mount_T_optical"]),
                (self.base, "red_scene/table", self.cfg["table"]["base_T_table"])]:
                matrix = transform(value)
                item = TransformStamped()
                item.header.stamp = self.get_clock().now().to_msg()
                item.header.frame_id, item.child_frame_id = parent, child
                item.transform.translation.x, item.transform.translation.y, item.transform.translation.z = map(float, matrix[:3, 3])
                q = rotation_quaternion(matrix[:3, :3])
                item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w = q
                transforms.append(item)
            self.static_tf.sendTransform(transforms)
        self.objects = self.create_publisher(SceneObjects, "/red_scene/objects", 10)
        self.markers = self.create_publisher(MarkerArray, "/red_scene/markers", 10)
        self.status = self.create_publisher(String, "/red_scene/status", 10)
        self.annotated = self.create_publisher(Image, "/red_scene/annotated", 2)
        self.legacy = self.create_publisher(PoseArray, "/vision/cubes", 10) if self.get_parameter("legacy_topics").value else None
        self.pending_images = deque(maxlen=30)
        self.last_received = None
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(JointState, self.get_parameter("joint_topic").value, self.on_joints, qos_profile_sensor_data)
        self.last_image = None
        self.last_processed = None
        self.create_timer(0.1, self.watchdog)
        self.create_timer(0.02, self.drain_images)

    def receive_image(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if self.last_received is not None and stamp <= self.last_received:
            self.invalidate("duplicate / out-of-order image", msg.header)
            return
        self.last_received = stamp
        self.pending_images.append(msg)
        if self.error:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
                cv2.putText(frame, "CALIBRATION REQUIRED", (8, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                preview = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                preview.header = deepcopy(msg.header)
                self.annotated.publish(preview)
            except (ValueError, cv2.error) as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=5.0)

    def drain_images(self):
        # Joint feedback is slower than images. Wait for a bracketing TF sample
        # without blocking the callbacks that must receive that sample.
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending_images:
            msg = self.pending_images[0]
            age = now - stamp_seconds(msg.header.stamp)
            if age < 0 or age > self.cfg["perception"]["max_age_sec"]:
                self.pending_images.popleft()
                continue
            if not self.error and not self.tf.can_transform(
                    self.base, self.mount, Time.from_msg(msg.header.stamp)):
                return
            self.pending_images.popleft()
            self.on_image(msg)
            return

    def on_joints(self, msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q for j in ARM_JOINTS):
            self.stationary.add(stamp_seconds(msg.header.stamp), [q[j] for j in ARM_JOINTS])
        else:
            self.stationary.history.clear()

    def watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_image is None or now - self.last_image > self.cfg["perception"]["max_age_sec"]:
            self.invalidate("image timeout" if self.last_image else self.error or "waiting for camera")

    def invalidate(self, reason, header=None):
        self.filter.clear()
        msg = SceneObjects()
        if header is not None:
            msg.header = deepcopy(header)
        msg.header.frame_id = self.base
        msg.calibrated = not bool(self.error)
        msg.status = reason
        self.objects.publish(msg)
        self.status.publish(String(data=reason))
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.markers.publish(MarkerArray(markers=[clear]))
        if self.legacy:
            self.legacy.publish(PoseArray(header=msg.header))

    def on_image(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        p = self.cfg["perception"]
        if stamp <= 0 or not 0 <= now - stamp <= p["max_age_sec"]:
            self.invalidate("stale / future image", msg.header)
            return
        if self.last_processed is not None and stamp <= self.last_processed:
            self.invalidate("duplicate / out-of-order image", msg.header)
            return
        self.last_processed = self.last_image = stamp
        if self.error:
            self.invalidate(self.error, msg.header)
            return
        if not self.stationary.ready(stamp, p["joint_max_age_sec"]):
            self.invalidate("robot moving or measured joints unavailable", msg.header)
            return
        try:
            c = self.cfg["camera"]
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if bgr.shape[:2] != (c["height"], c["width"]):
                raise ValueError("image size differs from calibration")
            # Zero timeout: never block the executor that must receive TF.
            mount = self.tf.lookup_transform(self.base, self.mount, Time.from_msg(msg.header.stamp))
            camera = tf_matrix(mount) @ transform(c["mount_T_optical"])
            observations, mask = detect_scene(bgr, self.cfg, camera)
            stable = self.filter.update(observations, stamp)
            self.publish(stable, msg.header)
            overlay = bgr.copy()
            overlay[mask != 0] = cv2.addWeighted(bgr, 0.5, np.full_like(bgr, (0, 0, 255)), 0.5, 0)[mask != 0]
            cv2.putText(overlay, "stable: " + ", ".join(o.kind for o in stable),
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            annotated = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            annotated.header = msg.header
            self.annotated.publish(annotated)
        except (TransformException, ValueError, cv2.error) as exc:
            self.invalidate(str(exc), msg.header)

    def publish(self, observations, header):
        msg = SceneObjects(header=deepcopy(header), calibrated=True)
        msg.header.frame_id = self.base
        msg.status = "stable observations" if observations else "no unambiguous stable objects"
        clear = Marker(action=Marker.DELETEALL)
        markers = MarkerArray(markers=[clear])
        table = transform(self.cfg["table"]["base_T_table"])
        cubes = PoseArray(header=msg.header)
        for i, obj in enumerate(observations):
            out = SceneObject(header=msg.header, id="red_" + obj.kind,
                              kind=obj.kind, color="red", valid=True, quality=obj.quality)
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, obj.center)
            yaw = obj.yaw
            rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            q = rotation_quaternion(table[:3, :3] @ rz)
            out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
            out.size.x, out.size.y, out.size.z = map(float, obj.size)
            out.boundary = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in obj.boundary]
            msg.objects.append(out)
            marker = Marker(header=msg.header, ns="red_scene", id=i, type=Marker.CUBE,
                            action=Marker.ADD, pose=out.pose, scale=out.size)
            marker.color.r, marker.color.a = 1.0, 0.8 if obj.kind == "cube" else 0.3
            marker.lifetime = Duration(seconds=self.cfg["perception"]["max_age_sec"]).to_msg()
            markers.markers.append(marker)
            center = Marker(header=msg.header, ns="centers", id=i, type=Marker.SPHERE,
                            action=Marker.ADD, pose=out.pose)
            center.scale.x = center.scale.y = center.scale.z = 0.006
            center.color.g = center.color.a = 1.0
            center.lifetime = marker.lifetime
            markers.markers.append(center)
            if obj.kind == "cube":
                cubes.poses.append(out.pose)
        self.objects.publish(msg)
        self.status.publish(String(data=msg.status))
        self.markers.publish(markers)
        if self.legacy:
            self.legacy.publish(cubes)


def main(args=None):
    rclpy.init(args=args)
    node = RedSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
