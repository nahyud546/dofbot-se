"""B5: pick 4 cube 30mm vao 4 o theo mau (dry-run mac dinh, an toan).

Subscribe: /vision/cubes (PoseArray base_link) - lay kem label tu /vision/cubes_markers? Don gian:
  Node doc /vision/detections_2d (co label) + tu tinh base nhu localizer, hoac doc service.
  De tranh phu thuoc, node nay subscribe /vision/cubes_markers, trich pose + color->label.

Modes:
  dry_run:=true  (mac dinh): chi log + publish /vision/plan_markers (mũi ten pick->place), KHONG dieu khien tay.
  dry_run:=false: dung pymoveit2/MoveIt2 di chuyen that (can move_group dang chay).

Zones doc tu table_zones.yaml (base_link, m). Grasp width 37mm ~ Rlink1_Joint tuong ung.
"""

import json

import rclpy
import yaml
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

LABEL_FROM_RGB = {
    (1.0, 0.85, 0.0): "yellow",
    (0.0, 0.8, 0.0): "green",
    (0.1, 0.3, 1.0): "blue",
    (1.0, 0.1, 0.1): "red",
}


def closest_label(r, g, b):
    best, bd = "?", 1e9
    for (cr, cg, cb), lb in LABEL_FROM_RGB.items():
        d = abs(r - cr) + abs(g - cg) + abs(b - cb)
        if d < bd:
            bd, best = d, lb
    return best


class CubesPickPlaceNode(Node):
    def __init__(self):
        super().__init__("cubes_pick_place")
        self.declare_parameter("markers_topic", "/vision/cubes_markers")
        self.declare_parameter("plan_topic", "/vision/plan_markers")
        self.declare_parameter("table_file", "")
        self.declare_parameter("dry_run", True)
        p = self.get_parameters_by_prefix("")
        self.dry_run = bool(p["dry_run"].value)
        tfile = str(p["table_file"].value)
        with open(tfile) as f:
            self.table = yaml.safe_load(f) if tfile else {}
        self.zones = (self.table or {}).get("zones", {})
        self.pub = self.create_publisher(MarkerArray, str(p["plan_topic"].value), 10)
        self.create_subscription(MarkerArray, str(p["markers_topic"].value), self.on_markers, 10)
        self._planned = False
        self.get_logger().info(f"cubes_pick_place dry_run={self.dry_run} zones={self.zones}")

    def on_markers(self, msg):
        cubes = []
        for mk in msg.markers:
            if mk.ns != "cubes":
                continue
            label = closest_label(round(mk.color.r, 2), round(mk.color.g, 2), round(mk.color.b, 2))
            cubes.append((label, mk.pose.position.x, mk.pose.position.y, mk.pose.position.z))
        if not cubes or self._planned:
            return
        self._planned = True  # lap 1 lan de tranh spam; restart node de plan lai
        out = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (label, x, y, z) in enumerate(sorted(cubes)):
            zone = self.zones.get(label)
            if zone is None:
                self.get_logger().warn(f"Cube {label} khong co zone - bo qua.")
                continue
            gx, gy = float(zone[0]), float(zone[1])
            # Cao do lay tu table (khong tin marker z): pick mat tren cube, place tam cube.
            # (Sua bug marker z lac 50mm tu text-marker.)
            table_z = float((self.table or {}).get("table_z", 0.0))
            cube_size = float((self.table or {}).get("cube_size", 0.030))
            cube_half = float((self.table or {}).get("cube_half", 0.015))
            z_off = float((self.table or {}).get("pick_z_offset", 0.005))
            pz = table_z + cube_half
            topz = table_z + cube_size + z_off
            self.get_logger().info(
                f"PLAN {i}: {label} pick=({x*1000:.1f},{y*1000:.1f},{topz*1000:.1f})mm "
                f"-> place=({gx*1000:.1f},{gy*1000:.1f},{pz*1000:.1f})mm "
                f"grasp=37mm approach={self.table.get('approach', 0.06)*1000:.0f}mm "
                f"{'[DRY-RUN]' if self.dry_run else '[EXECUTE]'}")
            arrow = Marker()
            arrow.header.stamp, arrow.header.frame_id = stamp, "base_link"
            arrow.ns, arrow.id, arrow.action = "plan", i, Marker.ADD
            arrow.type, arrow.lifetime.sec = Marker.ARROW, 0
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.012, 0.012
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 1.0, 0.2, 0.9
            arrow.points = [Point(x=x, y=y, z=topz + 0.06), Point(x=gx, y=gy, z=pz + 0.06)]
            out.markers.append(arrow)
        self.pub.publish(out)
        if not self.dry_run:
            self.get_logger().warn("EXECUTE chua bat trong ban nay: can MoveIt action client + safety_gate. Dang giu DRY-RUN Until B5.2.")


def main(args=None):
    rclpy.init(args=args)
    node = CubesPickPlaceNode()
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
