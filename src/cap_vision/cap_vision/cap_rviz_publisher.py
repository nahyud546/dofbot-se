"""Gom MarkerArray cua cap_detector + khung ban -> topic duy nhat cho RViz.

Subscribe: markers_topic (/cap_vision/cap_markers)
Publish:   rviz_topic    (/caps_markers): marker nap giu nguyen
             + LINE_STRIP vien ban 300x250mm (ns "table") de can goc nhin.

Khung ban lay tu param (met, frame table_frame) - phai khop vung calibrate.
RViz: Fixed Frame = world (qua static TF world -> table_frame).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class CapRvizPublisherNode(Node):
    def __init__(self):
        super().__init__("cap_rviz_publisher")
        self.declare_parameter("markers_topic", "/cap_vision/cap_markers")
        self.declare_parameter("rviz_topic", "/caps_markers")
        self.declare_parameter("frame_id", "table_frame")
        self.declare_parameter("x_min", 0.00)
        self.declare_parameter("x_max", 0.30)
        self.declare_parameter("y_min", 0.00)
        self.declare_parameter("y_max", 0.25)
        self.declare_parameter("table_z", 0.006)

        p = self.get_parameters_by_prefix("")
        self.frame_id = str(p["frame_id"].value)
        self.bounds = {k: float(p[k].value) for k in
                       ("x_min", "x_max", "y_min", "y_max", "table_z")}
        self.pub = self.create_publisher(
            MarkerArray, str(p["rviz_topic"].value), 10)
        self.create_subscription(
            MarkerArray, str(p["markers_topic"].value), self.on_markers, 10)
        self.get_logger().info(
            f"cap_rviz_publisher: {p['markers_topic'].value}"
            f" -> {p['rviz_topic'].value}")

    def table_marker(self, stamp):
        b = self.bounds
        pts = [(b["x_min"], b["y_min"]), (b["x_max"], b["y_min"]),
               (b["x_max"], b["y_max"]), (b["x_min"], b["y_max"]),
               (b["x_min"], b["y_min"])]
        mk = Marker()
        mk.header.stamp, mk.header.frame_id = stamp, self.frame_id
        mk.ns, mk.id, mk.action = "table", 1000, Marker.ADD
        mk.type, mk.lifetime.sec = Marker.LINE_STRIP, 1
        mk.scale.x = 0.003
        mk.color.g = mk.color.a = 1.0
        for x, y in pts:
            mk.points.append(Point(x=x, y=y, z=b["table_z"]))
        return mk

    def on_markers(self, msg):
        out = MarkerArray()
        out.markers = list(msg.markers)
        stamp = self.get_clock().now().to_msg()
        out.markers.append(self.table_marker(stamp))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CapRvizPublisherNode()
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
