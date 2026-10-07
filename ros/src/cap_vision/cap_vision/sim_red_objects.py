"""Sim-only: ve cube do 30mm + zone do 80mm len RViz + collision MoveIt.

Su dung cho check an toan tren sim (demo.launch.py, FakeSystem).
KHONG dung cho hardware that, KHONG publish /joint_states.

Pose chot (frame base_link, met) - ca hai DAT TREN MAT SAN z=0:
  - sim_red_cube: tam (0.160, 0.000, 0.015), box 0.030^3 (day cham san)
  - sim_red_zone: tam (0.080, -0.160, 0.001), plate 0.080x0.080x0.002 (nam tren san)

Publish:  /vision/sim_red_markers (MarkerArray, 1Hz, frame base_link)
           RViz: Fixed Frame=base_link, Add MarkerArray -> topic nay.
Collision: id sim_red_cube / sim_red_zone qua MoveIt2 (arm_group),
           visual mong 2mm nhung collision day 10mm de planner khong lot.
           Tat node se tu remove collision (hoac ros2 run ... --ros-args -p add_collision:=false).

Chay:
  ros2 launch dofbot_moveit demo.launch.py          # terminal 1 (giu RViz dang bat)
  ros2 run cap_vision sim_red_objects               # terminal 2 (sim-only)
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

FRAME = "base_link"
TOPIC = "/vision/sim_red_markers"
CUBE_ID = "sim_red_cube"
ZONE_ID = "sim_red_zone"
EE_LINK = "Gripping_point_Link"


def _qmult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qrot(q, v):
    t = _qmult(q, (v[0], v[1], v[2], 0.0))
    r = _qmult(t, _qconj(q))
    return (r[0], r[1], r[2])


class SimRedObjectsNode(Node):
    def __init__(self):
        super().__init__("sim_red_objects")
        cb = ReentrantCallbackGroup()
        self._cb = cb
        self.declare_parameter("markers_topic", TOPIC)
        self.declare_parameter("frame_id", FRAME)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("cube_xy", [0.160, 0.000])
        self.declare_parameter("cube_z", -1.0)  # <0 = auto nam san; sim_red_run set khi dat xuong
        self.declare_parameter("cube_hidden", False)  # True khi cube dang duoc gap di
        self.declare_parameter("cube_follow", False)  # True: marker bam gripper qua TF
        self.declare_parameter("cube_local", [0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 1.0])  # cube trong frame EE
        self.declare_parameter("cube_size", 0.030)
        self.declare_parameter("zone_xy", [0.080, -0.160])
        self.declare_parameter("zone_size", 0.080)
        self.declare_parameter("add_collision", True)
        self.declare_parameter("cube_collision", True)  # False: chi giu zone, bo cube sim

        p = self.get_parameters_by_prefix("")
        self.markers_topic = str(p["markers_topic"].value)
        self.frame_id = str(p["frame_id"].value)
        self.ground_z = float(p["ground_z"].value)
        self.cube_xy = [float(v) for v in p["cube_xy"].value]
        self.cube_z = float(p["cube_z"].value)
        self.cube_hidden = bool(p["cube_hidden"].value)
        self.cube_follow = bool(p["cube_follow"].value)
        try:
            self.cube_local = [float(v) for v in p["cube_local"].value]
        except Exception:
            self.cube_local = [0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 1.0]
        self._carry_xyz = None
        self._carry_quat = None
        self.cube_size = float(p["cube_size"].value)
        self.zone_xy = [float(v) for v in p["zone_xy"].value]
        self.zone_size = float(p["zone_size"].value)
        self.add_collision = bool(p["add_collision"].value)
        self.cube_collision = bool(p["cube_collision"].value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.create_timer(1.0, self.on_timer, callback_group=cb)

        self._apply_cli = None
        if self.add_collision:
            self._apply_cli = self.create_client(
                ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
        self._collision_added = False
        self._collision_retry = 0
        self.get_logger().info(
            f"sim_red_objects visual={self.markers_topic} frame={self.frame_id} "
            f"cube_xy={self.cube_xy} zone_xy={self.zone_xy} ground_z={self.ground_z} "
            f"add_collision={self.add_collision} (SIM-ONLY, FakeSystem)")

    def cube_pose(self):
        z = self.cube_z if self.cube_z >= 0.0 else self.ground_z + self.cube_size / 2.0
        return (self.cube_xy[0], self.cube_xy[1], z)

    def refresh_params(self):
        """Doc lai params dong (sim_red_run set khi gap/dat) - fail-open giu gia tri cu."""
        try:
            self.cube_xy = [float(v) for v in self.get_parameter("cube_xy").value]
        except Exception:
            pass
        try:
            self.cube_z = float(self.get_parameter("cube_z").value)
        except Exception:
            pass
        try:
            self.cube_hidden = bool(self.get_parameter("cube_hidden").value)
        except Exception:
            pass
        try:
            self.cube_follow = bool(self.get_parameter("cube_follow").value)
        except Exception:
            pass
        try:
            loc = [float(v) for v in self.get_parameter("cube_local").value]
            if len(loc) == 7:
                self.cube_local = loc
        except Exception:
            pass

    def zone_pose(self):
        return (self.zone_xy[0], self.zone_xy[1], self.ground_z + 0.001)

    def zone_collision_pose(self):
        # collision day 10mm, day cham san (khong thung xuong duoi san)
        return (self.zone_xy[0], self.zone_xy[1], self.ground_z + 0.005)

    def carry_world_pose(self):
        """Pose cube trong base_link khi dang gap: base_T_ee * local. None neu chua co TF."""
        if not self.cube_follow or len(self.cube_local) != 7:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, EE_LINK, rclpy.time.Time())
        except TransformException:
            return None
        lx, ly, lz = self.cube_local[0:3]
        lq = tuple(self.cube_local[3:7])
        eq = (t.transform.rotation.x, t.transform.rotation.y,
              t.transform.rotation.z, t.transform.rotation.w)
        rx, ry, rz = _qrot(eq, (lx, ly, lz))
        wq = _qmult(eq, lq)
        return ((t.transform.translation.x + rx, t.transform.translation.y + ry,
                 t.transform.translation.z + rz), wq)

    def build_markers(self, stamp):
        out = MarkerArray()
        carry = self.carry_world_pose()
        if carry is not None:
            # dang gap: khoi do bam theo gripper
            (cx, cy, cz), (qx, qy, qz, qw) = carry
            self._carry_xyz = (cx, cy, cz)
            cube = Marker()
            cube.header.stamp, cube.header.frame_id = stamp, self.frame_id
            cube.ns, cube.id, cube.action = "sim_red", 0, Marker.ADD
            cube.type = Marker.CUBE
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = cx, cy, cz
            cube.pose.orientation.x, cube.pose.orientation.y = qx, qy
            cube.pose.orientation.z, cube.pose.orientation.w = qz, qw
            cube.scale.x = cube.scale.y = cube.scale.z = self.cube_size
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.1, 0.1, 0.95
            out.markers.append(cube)
        elif not self.cube_hidden:
            cx, cy, cz = self.cube_pose()
            cube = Marker()
            cube.header.stamp, cube.header.frame_id = stamp, self.frame_id
            cube.ns, cube.id, cube.action = "sim_red", 0, Marker.ADD
            cube.type = Marker.CUBE
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = cx, cy, cz
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = self.cube_size
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.1, 0.1, 0.95
            out.markers.append(cube)
        else:
            # an marker cu di (ghi de DELETE de RViz khong giu bong ma)
            hide = Marker()
            hide.header.stamp, hide.header.frame_id = stamp, self.frame_id
            hide.ns, hide.id, hide.action = "sim_red", 0, Marker.DELETE
            out.markers.append(hide)

        zx, zy, zz = self.zone_pose()
        zone = Marker()
        zone.header.stamp, zone.header.frame_id = stamp, self.frame_id
        zone.ns, zone.id, zone.action = "sim_red", 1, Marker.ADD
        zone.type = Marker.CUBE
        zone.pose.position.x, zone.pose.position.y, zone.pose.position.z = zx, zy, zz
        zone.pose.orientation.w = 1.0
        zone.scale.x = zone.scale.y = self.zone_size
        zone.scale.z = 0.002
        zone.color.r, zone.color.g, zone.color.b, zone.color.a = 1.0, 0.1, 0.1, 0.35
        out.markers.append(zone)
        return out

    def make_box(self, obj_id, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.header = Header(frame_id=self.frame_id)
        obj.id = obj_id
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(sx), float(sy), float(sz)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        return obj

    def ensure_collision(self):
        if not self.add_collision or self._collision_added or self._apply_cli is None:
            return
        if not self._apply_cli.service_is_ready():
            self._collision_retry += 1
            if self._collision_retry % 5 == 1:
                self.get_logger().warn("/apply_planning_scene chua san sang, retry 1s...")
            return
        try:
            cx, cy, cz = self.cube_pose()
            qx, qy, qz = self.zone_collision_pose()
            req = ApplyPlanningScene.Request()
            req.scene.is_diff = True
            req.scene.robot_state.is_diff = True
            req.scene.world.collision_objects = [
                self.make_box(ZONE_ID, qx, qy, qz,
                              self.zone_size, self.zone_size, 0.010),
            ]
            if self.cube_collision:
                req.scene.world.collision_objects.insert(
                    0, self.make_box(CUBE_ID, cx, cy, cz,
                                     self.cube_size, self.cube_size, self.cube_size))
            fut = self._apply_cli.call_async(req)
            # doi dong bo toi da 5s trong timer (an toan vi ReentrantCB)
            import time
            deadline = time.monotonic() + 5.0
            while not fut.done():
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.02)
            if fut.done() and fut.result() is not None and fut.result().success:
                self._collision_added = True
                self.get_logger().info(
                    f"Da them collision {CUBE_ID} ({cx:.3f},{cy:.3f},{cz:.3f}) + "
                    f"{ZONE_ID} ({qx:.3f},{qy:.3f},{qz:.3f}) - SIM-ONLY")
            else:
                self.get_logger().warn("apply_planning_scene chua success, retry 1s.")
        except Exception as exc:
            self.get_logger().warn(f"Them collision that bai, retry 1s: {exc}")

    def on_timer(self):
        self.refresh_params()
        self.pub.publish(self.build_markers(self.get_clock().now().to_msg()))
        self.ensure_collision()

    def cleanup(self):
        if self._apply_cli is not None and self._collision_added:
            try:
                req = ApplyPlanningScene.Request()
                req.scene.is_diff = True
                for oid in (CUBE_ID, ZONE_ID):
                    obj = CollisionObject()
                    obj.id = oid
                    obj.operation = CollisionObject.REMOVE
                    req.scene.world.collision_objects.append(obj)
                self._apply_cli.call_async(req)
            except Exception:
                pass
            self.get_logger().info("Da go collision sim (cleanup).")


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor

    node = SimRedObjectsNode()
    ex = MultiThreadedExecutor(2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cleanup()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
