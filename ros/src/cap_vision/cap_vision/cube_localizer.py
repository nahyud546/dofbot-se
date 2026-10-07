"""B4: (u,v) -> base_link bang fast-path observation + fallback TF ray-plane.

Subscribe: /vision/detections_2d (String JSON [{label,u,v,area,w,h}])
Publish:   /vision/cubes (PoseArray frame base_link, z=tam cube)
           /vision/cubes_markers (MarkerArray BOX 30mm mau tuong ung)

Fast-path (mac dinh, dung pose Z120 da validate):
    Xb = cam_x + sign_x*(u-cx)*sx
    Yb = cam_y + sign_y*(v-cy)*sy
    Zb = table_z + cube_half
Fallback TF (use_tf:=true): can camera_info + TF base->camera_frame, intersect plane.
"""

import json

import copy

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

COLOR = {"yellow": (1.0, 0.85, 0.0), "green": (0.0, 0.8, 0.0),
         "blue": (0.1, 0.3, 1.0), "red": (1.0, 0.1, 0.1)}


def load_camera_info(path):
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    K = d.get("camera_matrix", [853.33, 0, 320, 0, 748.05, 240, 0, 0, 1])
    return float(K[0]), float(K[4]), float(K[2]), float(K[5])


def load_table(path):
    with open(path) as f:
        return yaml.safe_load(f)


class CubeLocalizerNode(Node):
    def __init__(self):
        super().__init__("cube_localizer")
        self.declare_parameter("detections_topic", "/vision/detections_2d")
        self.declare_parameter("poses_topic", "/vision/cubes")
        self.declare_parameter("markers_topic", "/vision/cubes_markers")
        self.declare_parameter("camera_info_file", "")
        self.declare_parameter("table_file", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("use_tf", False)
        p = self.get_parameters_by_prefix("")
        self.frame_id = str(p["frame_id"].value)
        self.use_tf = bool(p["use_tf"].value)
        tfile = str(p["table_file"].value)
        cfile = str(p["camera_info_file"].value)
        self.table = load_table(tfile) if tfile else {}
        self.fx, self.fy, self.cx, self.cy = load_camera_info(cfile) if cfile else (853.33, 748.05, 320.0, 240.0)
        self.pub_poses = self.create_publisher(PoseArray, str(p["poses_topic"].value), 10)
        self.pub_markers = self.create_publisher(MarkerArray, str(p["markers_topic"].value), 10)
        self.create_subscription(String, str(p["detections_topic"].value), self.on_dets, 10)
        self.get_logger().info(
            f"cube_localizer ready fx={self.fx:.1f} fy={self.fy:.1f} use_tf={self.use_tf} table={self.table.get('table_z', '?')}")

    def pixel_to_base_fast(self, u, v):
        t = self.table
        sx = float(t.get("mm_per_px_x", 0.0001406))
        sy = float(t.get("mm_per_px_y", 0.0001604))
        cx = float(t.get("obs_cam_x", 0.166)) if "obs_cam_x" in t else 0.166
        cy = float(t.get("obs_cam_y", 0.001)) if "obs_cam_y" in t else 0.001
        # dung cx ảnh tu camera_info, khong nham voi cam pose
        sign_x = float(t.get("obs_sign_x", -1.0))
        sign_y = float(t.get("obs_sign_y", -1.0))
        Xb = cx + sign_x * (u - self.cx) * sx
        Yb = cy + sign_y * (v - self.cy) * sy
        Zb = float(t.get("table_z", 0.0)) + float(t.get("cube_half", 0.015))
        return Xb, Yb, Zb

    def on_dets(self, msg):
        try:
            dets = json.loads(msg.data or "[]")
        except Exception:
            return
        stamp = self.get_clock().now().to_msg()
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.frame_id
        markers = MarkerArray()
        for i, d in enumerate(dets):
            label = str(d.get("label", "?"))
            u, v = float(d.get("u", 0)), float(d.get("v", 0))
            if self.use_tf:
                self.get_logger().warn_once("use_tf=true chua calibrate optical TF - tam dung fast-path. Dat use_tf=false.")
            Xb, Yb, Zb = self.pixel_to_base_fast(u, v)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = Xb, Yb, Zb
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, self.frame_id
            mk.ns, mk.id, mk.action = "cubes", i, Marker.ADD
            mk.type = Marker.CUBE
            mk.lifetime.sec = 1
            mk.pose = copy.deepcopy(pose)
            s = float(self.table.get("cube_size", 0.030))
            mk.scale.x = mk.scale.y = mk.scale.z = s
            r, g, b = COLOR.get(label, (0.7, 0.7, 0.7))
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.95
            # label id -> giu mau rieng khi nhieu cube: them text marker
            markers.markers.append(mk)
            txt = Marker()
            txt.header.stamp, txt.header.frame_id = stamp, self.frame_id
            txt.ns, txt.id, txt.action = "labels", 100 + i, Marker.ADD
            txt.type, txt.lifetime.sec = Marker.TEXT_VIEW_FACING, 1
            txt.pose = copy.deepcopy(pose)
            txt.pose.position.z += 0.035
            txt.scale.z = 0.018
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = label
            markers.markers.append(txt)
        self.pub_poses.publish(poses)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CubeLocalizerNode()
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
