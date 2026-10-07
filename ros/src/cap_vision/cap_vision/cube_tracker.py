"""Theo cube that real-time cho RViz (mirror): detections_2d -> base_link.

Moi detection (u,v): back-project ray trong optical frame (fx,fy,cx,cy) ->
TF base_link->camera (live, doi hoi sim==real) -> cat mat phang ngang
z = plane_z + cube_half -> tam cube. Publish PoseArray + MarkerArray BOX.

Khac cube_localizer (fast-path pose co dinh): node nay dung TF that nen dung
o MOI tu the tay. Optical = TF base->Camera_Link (mount, live) compose voi
optical_rpy (param, FIT 2-diem thuoc). Co tinh optical_rpy lam single source
of truth, khong doc TF camera_frame truc tiep.

An toan: read-only (khong dieu khien, khong cham serial). Canh bao neu
/joint_states (sim) lech /real_joint_states qua 0.05 rad (TF sai -> map sai).
"""

import json
import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException

SIM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    # R = Rz(yaw) Ry(pitch) Rx(roll) (cung quy uoc static_transform_publisher)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def mat_vec(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def mat_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def quat_to_mat(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


class CubeTrackerNode(Node):
    def __init__(self):
        super().__init__("cube_tracker")
        cb = ReentrantCallbackGroup()
        self.declare_parameter("detections_topic", "/vision/detections_2d")
        self.declare_parameter("poses_topic", "/vision/cubes")
        self.declare_parameter("markers_topic", "/vision/cubes_markers")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("optical_frame", "camera_frame")
        self.declare_parameter("mount_frame", "Camera_Link")
        # FIT 2-diem thuoc 2026-09-22: [0,0,270] (lech 12do; cac goc khac >78do).
        self.declare_parameter("optical_rpy_deg", [0.0, 0.0, 270.0])
        self.declare_parameter("fx", 902.0)
        self.declare_parameter("fy", 875.4)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("plane_z", 0.015)
        self.declare_parameter("cube_size", 0.030)
        self.declare_parameter("check_sim_real", True)
        p = self.get_parameters_by_prefix("")
        self.frame_id = str(p["frame_id"].value)
        self.optical_frame = str(p["optical_frame"].value)
        self.mount_frame = str(p["mount_frame"].value)
        rpy = [float(v) for v in p["optical_rpy_deg"].value]
        self._optical_rpy = rpy
        self.R_mount_opt = rpy_to_mat(*[math.radians(v) for v in rpy])
        self.fx = float(p["fx"].value)
        self.fy = float(p["fy"].value)
        self.cx = float(p["cx"].value)
        self.cy = float(p["cy"].value)
        self.plane_z = float(p["plane_z"].value)
        self.cube_size = float(p["cube_size"].value)
        self.check_sim_real = bool(p["check_sim_real"].value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_poses = self.create_publisher(PoseArray, str(p["poses_topic"].value), 10)
        self.pub_markers = self.create_publisher(MarkerArray, str(p["markers_topic"].value), 10)
        self.create_subscription(String, str(p["detections_topic"].value),
                                 self.on_dets, 10, callback_group=cb)
        self._last = []
        self.create_timer(1.0, self.on_repub, callback_group=cb)
        self._sim_real_ok = None
        self._sim_js: dict = {}
        self._real_js: dict = {}
        self._sim_t = 0.0
        self._real_t = 0.0
        self.create_subscription(JointState, "/joint_states",
                                 self._on_sim_js, 10, callback_group=cb)
        self.create_subscription(JointState, "/real_joint_states",
                                 self._on_real_js, 10, callback_group=cb)
        self.create_timer(2.0, self.check_once, callback_group=cb)
        self.get_logger().info(
            f"cube_tracker ready frame={self.frame_id} plane_z={self.plane_z} "
            f"f=({self.fx},{self.fy}) optical_rpy={rpy}")

    def _on_sim_js(self, msg):
        import time
        self._sim_js = {n: float(v) for n, v in zip(msg.name, msg.position)}
        self._sim_t = time.monotonic()

    def _on_real_js(self, msg):
        import time
        self._real_js = {n: float(v) for n, v in zip(msg.name, msg.position)}
        self._real_t = time.monotonic()

    def check_once(self):
        import time
        now = time.monotonic()
        if now - self._sim_t > 2.0 or now - self._real_t > 2.0:
            return
        if not all(j in self._sim_js and j in self._real_js for j in SIM_JOINTS):
            return
        worst = max(abs(self._sim_js[j] - self._real_js[j]) for j in SIM_JOINTS)
        self._sim_real_ok = worst <= 0.05
        if self._sim_real_ok:
            self.get_logger().info(f"sim==real (lech max {worst:.3f} rad) - TF dung cho map that.")
        else:
            self.get_logger().warn(
                f"sim LECH real {worst:.3f} rad>0.05 - chay sync_sim_to_real --from-topic truoc khi tin marker!")

    def base_T_optical(self):
        """Tra ve (origin_xyz, R_base_opt). LUON compose mount TF + optical_rpy
        (single source of truth); khong dung TF optical truc tiep vi static pub
        co the khong khop RPY da fit (bug cu: param bi bypass)."""
        t = self.tf_buffer.lookup_transform(
            self.frame_id, self.mount_frame, rclpy.time.Time())
        o = t.transform.translation
        q = t.transform.rotation
        R = mat_mul(quat_to_mat((q.x, q.y, q.z, q.w)), self.R_mount_opt)
        return ((o.x, o.y, o.z), R)

    def pixel_to_base(self, u, v):
        o, R = self.base_T_optical()
        d = mat_vec(R, ((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0))
        if abs(d[2]) < 1e-9:
            raise ValueError("ray song song mat phang")
        zt = self.plane_z + self.cube_size / 2.0
        t = (zt - o[2]) / d[2]
        if t <= 0:
            raise ValueError("ray huong len tren")
        return (o[0] + t * d[0], o[1] + t * d[1], zt)

    def on_dets(self, msg):
        try:
            dets = json.loads(msg.data or "[]")
        except Exception:
            return
        if not dets:
            self._last = []
            self.publish([])
            return
        pts = []
        for d in dets:
            pts.append((str(d.get("label", "?")),
                        float(d.get("u", 0)), float(d.get("v", 0))))
        if not pts:
            return
        self._last = pts
        self.publish_uv(pts)

    def on_repub(self):
        # doc lai optical_rpy live (doi chieu sensor khong can restart)
        try:
            rpy = [float(v) for v in self.get_parameter("optical_rpy_deg").value]
            if len(rpy) == 3 and rpy != self._optical_rpy:
                self._optical_rpy = rpy
                self.R_mount_opt = rpy_to_mat(*[math.radians(v) for v in rpy])
                self.get_logger().info(f"optical_rpy live -> {rpy}")
        except Exception:
            pass
        # Never reproject a stored pixel with a newer camera transform.
        # Legacy messages have no source stamp; use red_scene for robot tasks.

    def publish_uv(self, pts_uv):
        pts = []
        for label, u, v in pts_uv:
            try:
                pts.append((label, self.pixel_to_base(u, v)))
            except (TransformException, ValueError) as exc:
                self.get_logger().warn(f"Bo frame (TF/ray): {exc}",
                                       throttle_duration_sec=5.0)
                return
        self.publish(pts)

    def publish(self, pts):
        stamp = self.get_clock().now().to_msg()
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.frame_id
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, (label, (x, y, z)) in enumerate(pts):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, self.frame_id
            mk.ns, mk.id, mk.action = "cubes", i, Marker.ADD
            mk.type = Marker.CUBE
            mk.lifetime.sec = 1
            mk.pose = pose
            mk.scale.x = mk.scale.y = mk.scale.z = self.cube_size
            if label == "red":
                mk.color.r, mk.color.a = 1.0, 0.95
                mk.color.g = mk.color.b = 0.1
            else:
                mk.color.r = mk.color.g = mk.color.b = 0.7
                mk.color.a = 0.9
            markers.markers.append(mk)
            self.get_logger().info(f"TRACK {label}: base=({x*1000:.1f},{y*1000:.1f},{z*1000:.1f})mm",
                                   throttle_duration_sec=5.0)
        self.pub_poses.publish(poses)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CubeTrackerNode()
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
