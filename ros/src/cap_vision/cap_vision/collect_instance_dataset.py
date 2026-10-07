"""Record YOLO-seg labels from tag-anchored projected ObjectState masks."""
from __future__ import annotations

import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cap_scene_interfaces.msg import ObjectStates


def stamp_key(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def mask_polygon(mask):
    contours = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[0]
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    contour = cv2.approxPolyDP(contour, 1.5, True).reshape(-1, 2)
    return contour if len(contour) >= 3 else None


def quaternion_angle(first, second):
    a, b = np.asarray(first, float), np.asarray(second, float)
    a /= max(np.linalg.norm(a), 1e-12)
    b /= max(np.linalg.norm(b), 1e-12)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(a, b)), 0.0, 1.0)))


class DatasetCollector(Node):
    def __init__(self):
        super().__init__("collect_instance_dataset")
        for name, default in (("image_topic", "/cap_vision/image_raw"),
                              ("states_topic", "/vision/object_states"),
                              ("output_root", "/tmp/cube_instance_dataset"),
                              ("split", "train"), ("min_interval_sec", 0.25),
                              ("min_translation_m", 0.003),
                              ("min_rotation_deg", 3.0),
                              ("min_mask_area_px", 500)):
            self.declare_parameter(name, default)
        split = str(self.get_parameter("split").value)
        if split not in ("train", "val"):
            raise ValueError("split must be train or val")
        self.root = Path(str(self.get_parameter("output_root").value)).expanduser()
        self.image_dir = self.root / "images" / split
        self.label_dir = self.root / "labels" / split
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.label_dir.mkdir(parents=True, exist_ok=True)
        descriptor = {"path": str(self.root.resolve()), "train": "images/train",
                      "val": "images/val", "names": {0: "object"}}
        with open(self.root / "data.yaml", "w", encoding="utf-8") as stream:
            yaml.safe_dump(descriptor, stream, sort_keys=False)
        self.bridge = CvBridge()
        self.frames = {}
        self.last_saved_at = 0.0
        self.last_pose = {}
        self.saved = 0
        self.create_subscription(Image, str(self.get_parameter("image_topic").value),
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(ObjectStates,
                                 str(self.get_parameter("states_topic").value),
                                 self.on_states, 10)
        self.get_logger().info(f"collecting {split} masks -> {self.root}")

    def on_image(self, message):
        self.frames[stamp_key(message.header.stamp)] = self.bridge.imgmsg_to_cv2(
            message, "bgr8").copy()
        while len(self.frames) > 30:
            self.frames.pop(next(iter(self.frames)))

    def diverse(self, item):
        pose = item.camera_pose.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = np.array([pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w])
        previous = self.last_pose.get(int(item.geometry_model_id))
        if previous is None:
            return True, position, quaternion
        distance = np.linalg.norm(position - previous[0])
        angle = math.degrees(quaternion_angle(quaternion, previous[1]))
        return (distance >= float(self.get_parameter("min_translation_m").value) or
                angle >= float(self.get_parameter("min_rotation_deg").value)), position, quaternion

    def on_states(self, message):
        frame = self.frames.pop(stamp_key(message.header.stamp), None)
        if frame is None:
            return
        now = time.monotonic()
        if now - self.last_saved_at < float(self.get_parameter("min_interval_sec").value):
            return
        height, width = frame.shape[:2]
        labels, accepted = [], []
        for item in message.objects:
            if (not item.camera_pose_valid or item.pose_method != "apriltag_ippe" or
                    int(item.geometry_model_id) not in (1, 2, 3, 4)):
                continue
            mask = self.bridge.imgmsg_to_cv2(item.mask, "mono8") > 0
            if mask.shape != (height, width) or np.count_nonzero(mask) < int(
                    self.get_parameter("min_mask_area_px").value):
                continue
            is_diverse, position, quaternion = self.diverse(item)
            if not is_diverse:
                continue
            polygon = mask_polygon(mask)
            if polygon is None:
                continue
            coords = [f"{x / width:.6f} {y / height:.6f}" for x, y in polygon]
            labels.append("0 " + " ".join(coords))
            accepted.append((int(item.geometry_model_id), position, quaternion))
        if not labels:
            return
        name = f"frame_{stamp_key(message.header.stamp)}"
        if not cv2.imwrite(str(self.image_dir / f"{name}.jpg"), frame,
                           [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError("failed to write dataset image")
        (self.label_dir / f"{name}.txt").write_text("\n".join(labels) + "\n",
                                                     encoding="utf-8")
        for cube_id, position, quaternion in accepted:
            self.last_pose[cube_id] = (position, quaternion)
        self.saved += 1
        self.last_saved_at = now
        self.get_logger().info(f"saved sample {self.saved}: {name}, objects={len(labels)}")


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
