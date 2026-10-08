"""Save one ROS image message for headless commissioning/debug."""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SnapshotNode(Node):
    def __init__(self):
        super().__init__("snapshot_topic")
        self.declare_parameter("topic", "/cap_vision/image_raw")
        self.declare_parameter("output", "/tmp/ros_image.png")
        self.declare_parameter("timeout_sec", 5.0)
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, str(self.get_parameter("topic").value),
                                 self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        if self.done:
            return
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        output = str(self.get_parameter("output").value)
        if not cv2.imwrite(output, frame):
            raise RuntimeError(f"cannot write {output}")
        self.get_logger().info(f"saved {message.width}x{message.height} -> {output}")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotNode()
    deadline = time.monotonic() + float(node.get_parameter("timeout_sec").value)
    try:
        while not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.done:
            node.get_logger().error("timed out waiting for image")
            return 2
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
