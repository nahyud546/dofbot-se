"""Ve 4 o zone tinh len RViz sim (khong can camera).

Doc table_zones.yaml, publish 1Hz MarkerArray /vision/zones_markers (frame base_link):
  - 4 tam phang 90x90x2mm mau zone, alpha 0.35, tai (x, y, table_z+1mm)
  - 4 nhan text ten mau phia tren.
RViz: Add MarkerArray -> /vision/zones_markers, Fixed Frame = base_link.
"""

import rclpy
import yaml
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

COLOR = {"yellow": (1.0, 0.85, 0.0), "green": (0.0, 0.8, 0.0),
         "blue": (0.1, 0.3, 1.0), "red": (1.0, 0.1, 0.1)}


class ZoneMarkersNode(Node):
    def __init__(self):
        super().__init__("zone_markers")
        self.declare_parameter("table_file", "")
        self.declare_parameter("markers_topic", "/vision/zones_markers")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("zone_size", 0.090)
        p = self.get_parameters_by_prefix("")
        tfile = str(p["table_file"].value)
        with open(tfile) as f:
            self.table = yaml.safe_load(f) if tfile else {}
        self.frame_id = str(p["frame_id"].value)
        self.zone_size = float(p["zone_size"].value)
        self.pub = self.create_publisher(MarkerArray, str(p["markers_topic"].value), 10)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f"zone_markers zones={list((self.table or {}).get('zones', {}))}")

    def on_timer(self):
        stamp = self.get_clock().now().to_msg()
        t = self.table or {}
        tz = float(t.get("table_z", 0.045))
        out = MarkerArray()
        for i, (label, xy) in enumerate(t.get("zones", {}).items()):
            x, y = float(xy[0]), float(xy[1])
            r, g, b = COLOR.get(label, (0.7, 0.7, 0.7))
            plate = Marker()
            plate.header.stamp, plate.header.frame_id = stamp, self.frame_id
            plate.ns, plate.id, plate.action = "zones", i, Marker.ADD
            plate.type = Marker.CUBE
            plate.pose.position.x, plate.pose.position.y = x, y
            plate.pose.position.z = tz + 0.001
            plate.pose.orientation.w = 1.0
            plate.scale.x = plate.scale.y = self.zone_size
            plate.scale.z = 0.002
            plate.color.r, plate.color.g, plate.color.b, plate.color.a = r, g, b, 0.35
            out.markers.append(plate)
            txt = Marker()
            txt.header.stamp, txt.header.frame_id = stamp, self.frame_id
            txt.ns, txt.id, txt.action = "zone_labels", 100 + i, Marker.ADD
            txt.type, txt.lifetime.sec = Marker.TEXT_VIEW_FACING, 0
            txt.pose.position.x, txt.pose.position.y = x, y
            txt.pose.position.z = tz + 0.030
            txt.scale.z = 0.018
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = label
            out.markers.append(txt)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneMarkersNode()
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
