"""Capture one stationary image/TF pair. No movement, no serial access."""
import argparse
from collections import deque
from pathlib import Path

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener, TransformException

from cap_vision.red_scene_core import StationaryWindow
from cap_vision.red_scene_node import ARM_JOINTS, stamp_seconds, tf_matrix


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", required=True, help="new directory containing image.png and sample.yaml")
    ap.add_argument("--validation", action="store_true")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()
    folder = Path(args.output)
    if folder.exists():
        raise SystemExit("output directory exists")
    rclpy.init()
    node = Node("capture_calibration")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    stationary = StationaryWindow(0.5, 0.005)
    result = []
    pending = deque(maxlen=30)

    def joints(msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q for j in ARM_JOINTS):
            stationary.add(stamp_seconds(msg.header.stamp), [q[j] for j in ARM_JOINTS])

    def image(msg):
        pending.append(msg)

    def capture_ready():
        if result:
            return
        while pending:
            msg = pending[0]
            stamp = stamp_seconds(msg.header.stamp)
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= 1.0:
                pending.popleft()
                continue
            if not buffer.can_transform("base_link", "measured/Camera_Link", Time.from_msg(msg.header.stamp)):
                return
            pending.popleft()
            if not stationary.ready(stamp, 0.5):
                continue
            try:
                pose = buffer.lookup_transform("base_link", "measured/Camera_Link", Time.from_msg(msg.header.stamp))
                result.append((CvBridge().imgmsg_to_cv2(msg, "bgr8"),
                               {"image": "image.png", "stamp": stamp,
                                "base_T_mount": tf_matrix(pose).ravel().tolist(),
                                "validation": args.validation}))
                return
            except TransformException:
                pass

    node.create_subscription(JointState, "/real_joint_states", joints, qos_profile_sensor_data)
    node.create_subscription(Image, "/cap_vision/image_raw", image, qos_profile_sensor_data)
    node.create_timer(0.02, capture_ready)
    import time
    deadline = time.monotonic() + args.timeout
    try:
        while not result and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result:
            raise SystemExit("No stationary, timestamp-matched image/real TF pair received")
        folder.mkdir(parents=True)
        if not cv2.imwrite(str(folder / "image.png"), result[0][0]):
            raise RuntimeError("image write failed")
        with (folder / "sample.yaml").open("x") as stream:
            yaml.safe_dump(result[0][1], stream)
        print(folder / "sample.yaml")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
