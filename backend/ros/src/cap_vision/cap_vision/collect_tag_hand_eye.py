"""Interactively collect a fixed-AprilTag eye-in-hand dataset.

This node never commands a servo.  Start the real joint/TF publisher and the
camera first, fix one AprilTag rigidly to the table, then move the arm with the
normal safe controller.  Press SPACE only when the tag is clearly visible;
the node accepts the sample only when the joint state is stationary and a
timestamp-matched ``base_link -> Camera_Link`` transform is available.

The output directory contains images and ``dataset.yaml`` ready for
``calibrate_tag_hand_eye``.  The last ``--validation-count`` captures are
held out, so deliberately vary their wrist orientation too.
"""
from __future__ import annotations

import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener, TransformException

from cap_vision.calibrate_tag_hand_eye import detect_corners, detector
from cap_vision.red_scene_core import StationaryWindow
from cap_vision.red_scene_node import ARM_JOINTS, stamp_seconds, tf_matrix


def matching_capture_image(images, now, stationary, transform_available):
    """Use the newest stationary camera frame with a matching measured TF."""
    for frame in reversed(images):
        stamp = stamp_seconds(frame.header.stamp)
        if (0 <= now - stamp <= 2.0 and stationary.ready(stamp, 1.5)
                and transform_available(frame.header.stamp)):
            return frame
    return None


class TagDatasetCollector(Node):
    def __init__(self, args):
        super().__init__("collect_tag_hand_eye")
        self.args = args
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.stationary = StationaryWindow(0.5, 0.005)
        # Joint readback is much slower than the camera. Keep enough frames
        # for a later joint sample to bracket an earlier image timestamp.
        self.images = deque(maxlen=64)
        self.samples = []
        self.tag_detector = detector()
        self.message = "Move arm safely; SPACE captures, q cancels."
        self.capture_deadline = None
        self.create_subscription(JointState, args.joint_topic, self.on_joint,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, args.image_topic, self.on_image,
                                 qos_profile_sensor_data)

    def on_joint(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in ARM_JOINTS):
            self.stationary.add(stamp_seconds(msg.header.stamp),
                                [positions[name] for name in ARM_JOINTS])

    def on_image(self, msg):
        self.images.append(msg)

    def _latest_fresh_image(self):
        if not self.images:
            return None
        msg = self.images[-1]
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - stamp_seconds(msg.header.stamp)
        return msg if 0.0 <= age <= 1.0 else None

    def capture(self):
        if self.capture_deadline is None:
            return
        if len(self.samples) >= self.args.count:
            self.message = "Dataset complete; press q."
            self.capture_deadline = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        msg = matching_capture_image(
            self.images, now, self.stationary,
            lambda stamp: self.tf.can_transform(
                self.args.base_frame, self.args.mount_frame, Time.from_msg(stamp)))
        if msg is None:
            if time.monotonic() >= self.capture_deadline:
                self.message = "No stationary image with timestamp-matched TF; retry SPACE."
                self.capture_deadline = None
            return
        stamp = stamp_seconds(msg.header.stamp)
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Reject blurry/wrong IDs before saving a sample that will later
            # invalidate the whole hand-eye solve.
            detect_corners(image, self.tag_detector, self.args.tag_id)
            pose = self.tf.lookup_transform(self.args.base_frame, self.args.mount_frame,
                                            Time.from_msg(msg.header.stamp))
        except (TransformException, ValueError) as exc:
            self.message = f"Capture rejected: {str(exc)[:70]}"
            self.capture_deadline = None
            return

        index = len(self.samples)
        folder = self.args.output / "samples" / f"{index:02d}"
        folder.mkdir(parents=True, exist_ok=False)
        image_path = folder / "image.png"
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f"cannot write {image_path}")
        validation = index >= self.args.count - self.args.validation_count
        self.samples.append({
            "image": str(image_path.relative_to(self.args.output)),
            "stamp": float(stamp),
            "base_T_mount": tf_matrix(pose).ravel().tolist(),
            "validation": validation,
        })
        self.write_manifest()
        phase = "validation" if validation else "fit"
        self.message = f"Saved {index + 1}/{self.args.count} ({phase}). Move to a new angle."
        self.capture_deadline = None
        self.get_logger().info(self.message)

    def write_manifest(self):
        payload = {
            "target": {
                "type": "apriltag", "id": int(self.args.tag_id),
                "size_m": float(self.args.tag_size_m),
                "tag_to_table_m": float(self.args.tag_to_table_m),
            },
            "samples": self.samples,
        }
        with (self.args.output / "dataset.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False)

    def preview(self):
        msg = self._latest_fresh_image()
        if msg is None:
            return None
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
        cv2.putText(image, f"{len(self.samples)}/{self.args.count}: {self.message}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.putText(image, "SPACE capture | q cancel", (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
        return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="new dataset directory")
    parser.add_argument("--tag-id", required=True, type=int)
    parser.add_argument("--tag-size-m", default=0.020, type=float)
    parser.add_argument("--tag-to-table-m", default=0.0, type=float,
                        help="tag plane height over table; 0 for a flat printed tag")
    parser.add_argument("--count", default=14, type=int)
    parser.add_argument("--validation-count", default=4, type=int)
    parser.add_argument("--image-topic", default="/cap_vision/image_raw")
    parser.add_argument("--joint-topic", default="/real_joint_states")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--mount-frame", default="measured/Camera_Link")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output exists: {args.output}")
    if args.tag_size_m <= 0 or args.count < 11 or not 3 <= args.validation_count < args.count:
        raise SystemExit("need positive tag size, >=11 captures, and 3..count-1 validation captures")
    args.output.mkdir(parents=True)

    rclpy.init()
    node = TagDatasetCollector(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            frame = node.preview()
            if frame is None:
                continue
            cv2.imshow("Tag hand-eye dataset", frame)
            key = cv2.waitKey(20) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                node.capture_deadline = time.monotonic() + 4.0
                node.message = "Waiting for stationary image and matching TF..."
            if node.capture_deadline is not None:
                node.capture()
            if len(node.samples) >= args.count:
                node.get_logger().info(f"Complete: {args.output / 'dataset.yaml'}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
