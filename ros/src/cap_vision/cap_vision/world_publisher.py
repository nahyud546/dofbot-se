"""Đưa world map (data/world/latest.json) lên RViz: TF của các camera + marker cube/tag/ô/mặt bàn.

Chỉ đọc file JSON tự chứa do `projects/vision_experiments/build_world.py` ghi; không import mã ngoài ROS, nên chạy
được bằng python hệ thống. File đổi thì tự nạp lại.

  TF       world -> base_link (đơn vị), world -> <tên>_optical cho từng camera cố định
  Marker   /world/markers: cube (hộp 30 mm, mờ khi chưa chắc), nhãn tọa độ, hình chóp nhìn của camera, mặt bàn, ô màu
  Khớp     /joint_states ở pose READY khi không có khớp thật (static_joints:=true) để RViz vẽ được tay máy
"""
import json
import math
import os
from pathlib import Path

import numpy as np

JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint",
          "Rlink1_Joint", "Rlink2_Joint", "Llink1_Joint", "Llink2_Joint", "Rlink3_Joint", "Llink3_Joint"]
READY_DEG = [90.0, 125.0, 0.0, 0.0, 90.0]
ZONE_COLOURS = {1: (0.2, 0.5, 1.0), 2: (0.2, 0.8, 0.3), 3: (0.9, 0.2, 0.2), 4: (0.6, 0.6, 0.6)}


def default_world_file() -> str:
    root = os.environ.get("ROBOT_ARM_ROOT")
    if not root:
        for parent in Path(__file__).resolve().parents:
            if (parent / "projects").is_dir() and (parent / "ros").is_dir():
                root = str(parent)
                break
    return str(Path(root or ".") / "data" / "world" / "latest.json")


def quaternion(R) -> tuple:
    """(x, y, z, w) từ ma trận xoay 3x3."""
    R = np.asarray(R, float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k]))
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
        q = tuple(q)
    norm = math.sqrt(sum(v * v for v in q))
    return tuple(v / norm for v in q)


def frustum_lines(K, image_size, depth=0.12) -> list:
    """Các đoạn thẳng [(p, q)] của hình chóp nhìn trong hệ optical (bỏ qua méo: chỉ để minh họa)."""
    fx, fy, cx, cy = K
    w, h = image_size
    corners = [np.array([(u - cx) / fx, (v - cy) / fy, 1.0]) * depth for u, v in ((0, 0), (w, 0), (w, h), (0, h))]
    origin = np.zeros(3)
    return [(origin, c) for c in corners] + [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def load_world(path):
    """dict của file world, hoặc None khi thiếu/hỏng."""
    try:
        data = json.loads(Path(path).read_text())
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    class WorldPublisher(Node):
        def __init__(self):
            super().__init__("world_publisher")
            self.declare_parameter("world_file", default_world_file())
            self.declare_parameter("static_joints", True)
            self.path = str(self.get_parameter("world_file").value) or default_world_file()
            self.static_joints = bool(self.get_parameter("static_joints").value)
            self.tf = TransformBroadcaster(self)
            self.markers = self.create_publisher(MarkerArray, "/world/markers", 2)
            self.joints = self.create_publisher(JointState, "/joint_states", 2) if self.static_joints else None
            self.world, self.mtime = None, None
            self.create_timer(0.5, self.tick)
            self.get_logger().info(f"world_publisher: đọc {self.path}")

        def reload(self):
            try:
                mtime = os.path.getmtime(self.path)
            except OSError:
                mtime = None
            if mtime != self.mtime:
                self.mtime = mtime
                self.world = load_world(self.path) if mtime is not None else None
                n = 0 if not self.world else len(self.world.get("tags") or {})
                self.get_logger().info(f"nạp world: {n} tag" if self.world else "chưa có file world")

        def transform(self, child, T, stamp):
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, "world", child
            T = np.asarray(T, float).reshape(4, 4)
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, T[:3, 3])
            q = quaternion(T[:3, :3])
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = map(float, q)
            return msg

        def marker(self, ns, mid, kind, stamp, frame="world", rgba=(1.0, 1.0, 1.0, 1.0), scale=(0.01, 0.01, 0.01)):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, frame
            m.ns, m.id, m.type, m.action = ns, int(mid), kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = map(float, scale)
            m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
            return m

        def tick(self):
            self.reload()
            stamp = self.get_clock().now().to_msg()
            tfs = [self.transform("base_link", np.eye(4), stamp)]
            if self.joints is not None:
                js = JointState()
                js.header.stamp = stamp
                js.name = JOINTS
                js.position = [math.radians(v - 90.0) for v in READY_DEG] + [0.0] * (len(JOINTS) - 5)
                self.joints.publish(js)
            array = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            array.markers.append(clear)
            world = self.world or {}
            table_z = world.get("table_z")
            if table_z is not None:
                m = self.marker("table", 0, Marker.CUBE, stamp, rgba=(0.75, 0.85, 0.95, 0.35), scale=(0.40, 0.60, 0.001))
                m.pose.position.x, m.pose.position.z = -0.20, float(table_z)
                array.markers.append(m)
            region = world.get("region") or []
            if len(region) >= 3:
                ring = self.marker("region", 0, Marker.LINE_STRIP, stamp, rgba=(0.0, 0.45, 1.0, 1.0), scale=(0.003, 0, 0))
                for x, y in list(region) + [region[0]]:
                    ring.points.append(Point(x=float(x), y=float(y), z=float(table_z or 0.0) + 0.001))
                array.markers.append(ring)
            for index, item in enumerate(world.get("objects") or []):
                polygon = item.get("polygon") or []
                if len(polygon) < 3:
                    continue
                ring = self.marker("object", index, Marker.LINE_STRIP, stamp, rgba=(1.0, 0.0, 1.0, 1.0), scale=(0.003, 0, 0))
                for x, y in list(polygon) + [polygon[0]]:
                    ring.points.append(Point(x=float(x), y=float(y), z=float(table_z or 0.0) + 0.002))
                array.markers.append(ring)
                text = self.marker("object_label", index, Marker.TEXT_VIEW_FACING, stamp, scale=(0.0, 0.0, 0.012))
                text.pose.position.x, text.pose.position.y = float(item["centre"][0]), float(item["centre"][1])
                text.pose.position.z = float(table_z or 0.0) + 0.03
                text.text = f"{item.get('label', 'vật')} (đáy ước lượng, chưa đo chiều cao)"
                array.markers.append(text)
            for key, cube in (world.get("cubes") or {}).items():
                tag = (world.get("tags") or {}).get(key, {})
                by_face = cube.get("seen_by") == "face"
                sure = bool(cube.get("sure", tag.get("sure", True)))
                edge = float(cube.get("edge_m", 0.03))
                m = self.marker("cube", key, Marker.CUBE, stamp,
                                rgba=((0.1, 0.75, 1.0, 0.9) if by_face else (1.0, 0.8, 0.1, 0.9)) if sure
                                else (1.0, 0.5, 0.1, 0.35), scale=(edge, edge, edge))
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, cube["centre"])
                q = quaternion(cube["R"])
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
                array.markers.append(m)
                if "corners" in tag:
                    quad = self.marker("tag", key, Marker.LINE_STRIP, stamp, rgba=(0.0, 0.0, 0.0, 1.0), scale=(0.002, 0, 0))
                    for p in list(tag["corners"]) + [tag["corners"][0]]:
                        quad.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                    array.markers.append(quad)
                    c = tag.get("centre", cube["centre"])
                    text = self.marker("label", key, Marker.TEXT_VIEW_FACING, stamp, scale=(0.0, 0.0, 0.012))
                    text.pose.position.x, text.pose.position.y, text.pose.position.z = float(c[0]), float(c[1]), float(c[2]) + 0.025
                    text.text = (f"tag {key}: {c[0] * 1000:+.0f}, {c[1] * 1000:+.0f}, {c[2] * 1000:+.0f} mm"
                                 + ("" if sure else " (?)"))
                    array.markers.append(text)
                elif by_face:
                    c = cube["centre"]
                    text = self.marker("label", key, Marker.TEXT_VIEW_FACING, stamp, scale=(0.0, 0.0, 0.012))
                    text.pose.position.x, text.pose.position.y, text.pose.position.z = float(c[0]), float(c[1]), float(c[2]) + 0.04
                    text.text = (f"cube {key} (mặt {cube.get('top_face', '?')}): "
                                 f"{c[0] * 1000:+.0f}, {c[1] * 1000:+.0f}, {c[2] * 1000:+.0f} mm" + ("" if sure else " (?)"))
                    array.markers.append(text)
            for key, zone in (world.get("zones") or {}).items():
                colour = ZONE_COLOURS.get(int(key), (1.0, 0.0, 1.0))
                m = self.marker("zone", key, Marker.CYLINDER, stamp, rgba=(*colour, 0.6), scale=(0.05, 0.05, 0.002))
                m.pose.position.x, m.pose.position.y = float(zone["xy"][0]), float(zone["xy"][1])
                m.pose.position.z = float(table_z or 0.0)
                array.markers.append(m)
            for index, (name, cam) in enumerate((world.get("cameras") or {}).items()):
                frame = f"{name}_optical"
                tfs.append(self.transform(frame, cam["world_T_optical"], stamp))
                lines = self.marker("frustum", index, Marker.LINE_LIST, stamp, frame=frame,
                                    rgba=(0.1, 0.9, 0.9, 1.0), scale=(0.002, 0, 0))
                for a, b in frustum_lines(cam["K"], cam["image_size"]):
                    lines.points.append(Point(x=float(a[0]), y=float(a[1]), z=float(a[2])))
                    lines.points.append(Point(x=float(b[0]), y=float(b[1]), z=float(b[2])))
                array.markers.append(lines)
                text = self.marker("camera_name", index, Marker.TEXT_VIEW_FACING, stamp, frame=frame, scale=(0, 0, 0.015))
                text.pose.position.z = -0.02
                text.text = name
                array.markers.append(text)
            self.tf.sendTransform(tfs)
            self.markers.publish(array)

    rclpy.init(args=args)
    node = WorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:                      # Ctrl+C lần hai trong lúc đang dọn: thoát im lặng
            pass


if __name__ == "__main__":
    main()
