#!/usr/bin/env python3
"""Minimal /image_raw publisher from robot camera (usb_cam not installed).

Builds sensor_msgs/Image manually with numpy (cv_bridge on this laptop is
built for NumPy 1.x and crashes with the installed NumPy 2.x).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CamPub(Node):
    def __init__(self):
        super().__init__("cam_pub")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10.0)
        dev = self.get_parameter("device").value
        w = self.get_parameter("width").value
        h = self.get_parameter("height").value
        fps = self.get_parameter("fps").value
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {dev}")
        self.pub = self.create_publisher(Image, "/image_raw", 10)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"Publishing /image_raw from {dev} {w}x{h} @{fps}Hz")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("frame read failed")
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CamPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
