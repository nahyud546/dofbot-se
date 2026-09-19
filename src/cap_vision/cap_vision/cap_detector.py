"""Detect nap TRANG tren mat ban, doi pixel -> table_frame (met), publish ROS.

Subscribe:  image_topic   (sensor_msgs/Image, tu camera_test)
Publish:    poses_topic   (geometry_msgs/PoseArray, frame table_frame, z = table_z)
            markers_topic (visualization_msgs/MarkerArray, CYLINDER O30x12mm)
            annotated_topic (sensor_msgs/Image, ve vong tron + label)

Phuong phap: HSV trang (S thap, V cao) + loc dien tich + do tron.
Nap trang nhay cam voi anh sang: KHOA exposure/white-balance camera
truoc khi chay, chinh nguong white_lo/hi tai cho theo anh that.
Can homography da calibrate; neu chua thi van annotate anh nhung bao
warn va khong publish pose (tranh chay sai toa do).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np

from cap_vision.test_pixel_to_xy import (
    load_homography, pixel_to_xy, inside_workspace,
)

LABELS = ("white",)

# Kich thuoc nap that (met) theo plan: cylinder O30mm x 12mm.
CAP_DIAMETER = 0.030
CAP_HEIGHT = 0.012


def detect_caps(bgr, H, cfg):
    """H: 3x3 hoac None. Tra ve [dict(label, u, v, x, y, area)]."""
    blur = cv2.GaussianBlur(bgr, (cfg["blur"], cfg["blur"]), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    masks = {
        "white": cv2.inRange(hsv, np.array(cfg["white_lo"]), np.array(cfg["white_hi"])),
    }
    kernel = np.ones((5, 5), np.uint8)
    dets = []
    for label, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = float(cv2.contourArea(c))
            if not cfg["min_area"] <= area <= cfg["max_area"]:
                continue
            peri = float(cv2.arcLength(c, True))
            if peri < 1e-6:
                continue
            circularity = 4.0 * np.pi * area / (peri * peri)
            if circularity < cfg["min_circularity"]:
                continue
            m = cv2.moments(c)
            if m["m00"] < 1e-6:
                continue
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            x = y = None
            if H is not None:
                try:
                    x, y = pixel_to_xy(H, u, v)
                except ValueError:
                    continue
                if not inside_workspace(x, y):
                    continue
            dets.append({"label": label, "u": float(u), "v": float(v),
                         "x": x, "y": y, "area": area})
    return dets


class CapDetectorNode(Node):
    def __init__(self):
        super().__init__("cap_detector")
        self.declare_parameter("image_topic", "/cap_vision/image_raw")
        self.declare_parameter("poses_topic", "/cap_vision/cap_poses")
        self.declare_parameter("markers_topic", "/cap_vision/cap_markers")
        self.declare_parameter("annotated_topic", "/cap_vision/image_annotated")
        self.declare_parameter("homography_file", "")
        self.declare_parameter("frame_id", "table_frame")
        self.declare_parameter("table_z", CAP_HEIGHT / 2.0)
        self.declare_parameter("min_area", 300.0)
        self.declare_parameter("max_area", 20000.0)
        self.declare_parameter("min_circularity", 0.5)
        # Trang = S thap + V cao. Chinh tai cho theo anh sang that.
        self.declare_parameter("white_lo", [0, 0, 150])
        self.declare_parameter("white_hi", [180, 60, 255])

        p = self.get_parameters_by_prefix("")
        self.cfg = {
            "blur": 5,
            "min_area": float(p["min_area"].value),
            "max_area": float(p["max_area"].value),
            "min_circularity": float(p["min_circularity"].value),
            "white_lo": list(p["white_lo"].value),
            "white_hi": list(p["white_hi"].value),
        }
        self.frame_id = str(p["frame_id"].value)
        self.table_z = float(p["table_z"].value)

        self.H, _ = load_homography(str(p["homography_file"].value)) \
            if str(p["homography_file"].value) else (None, {})
        if self.H is None:
            self.get_logger().warn(
                "Chua co homography calibrate: chi annotate anh, khong publish pose."
                " Chay calibrate_table truoc khi map toa do.")

        self.bridge = CvBridge()
        self.poses_pub = self.create_publisher(
            PoseArray, str(p["poses_topic"].value), 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, str(p["markers_topic"].value), 10)
        self.annot_pub = self.create_publisher(
            Image, str(p["annotated_topic"].value), 10)
        self.create_subscription(
            Image, str(p["image_topic"].value), self.on_image, 10)
        self.get_logger().info(
            f"cap_detector san sang (frame={self.frame_id}).")

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - frame loi thi bo qua
            self.get_logger().warn(f"Bo frame loi: {exc}")
            return
        dets = detect_caps(bgr, self.H, self.cfg)
        stamp = self.get_clock().now().to_msg()

        annot = bgr.copy()
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.frame_id
        markers = MarkerArray()
        for i, d in enumerate(dets):
            cv2.circle(annot, (int(d["u"]), int(d["v"])), 12, (0, 255, 0), 2)
            txt = d["label"] if d["x"] is None else f"{d['label']} ({d['x'] * 1000:.0f},{d['y'] * 1000:.0f})mm"
            cv2.putText(annot, txt, (int(d["u"]) + 14, int(d["v"])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if d["x"] is None:
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = d["x"], d["y"], self.table_z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, self.frame_id
            mk.ns, mk.id, mk.action = "caps", i, Marker.ADD
            mk.type, mk.lifetime.sec = Marker.CYLINDER, 1
            mk.pose = pose
            mk.scale.x = mk.scale.y = CAP_DIAMETER
            mk.scale.z = CAP_HEIGHT
            mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
            markers.markers.append(mk)
        if self.H is not None:
            self.poses_pub.publish(poses)
            self.markers_pub.publish(markers)
        self.annot_pub.publish(self.bridge.cv2_to_imgmsg(annot, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = CapDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001 - SIGINT doi khi shutdown context 2 lan
            pass


if __name__ == "__main__":
    main()
