"""Lệnh 2: check xong TỰ GẮP-ĐẶT cube vào zone trên sim luôn.

Phase 1 - check (giống sim_red_check): markers + collision scene đúng mới đi tiếp.
Phase 2 - pick & place (chỉ chạy khi Phase 1 PASS):
    mo gripper -> hover cube -> ha xuong -> kep -> attach (cube dinh theo tay)
    -> nang -> sang zone -> ha xuong -> mo -> detach (cube nam trong zone)
    -> rut len -> ve HOME. Toc do cham 25%.

SIM-ONLY: tự chặn nếu phát hiện node `dofbot_driver` (cầu nối tay thật) -> exit 2.
Demo 1-shot: xong cube đã nằm trong zone; chạy lại cần restart lệnh 1.

Chạy sau lệnh 1 (sim_red_demo.launch.py đang mở):
    ros2 run cap_vision sim_red_run
"""

import math
import sys
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as PMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException

from pymoveit2 import MoveIt2

from cap_vision.sim_red_check import run_checks

ARM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
EE_LINK = "Gripping_point_Link"
BASE_LINK = "base_link"
CUBE_ID = "sim_red_cube"
ZONE_ID = "sim_red_zone"
FINGER_LINKS = [EE_LINK,
                "Rlink1_Link", "Rlink2_Link", "Rlink3_Link",
                "Llink1_Link", "Llink2_Link", "Llink3_Link"]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0]
CUBE_START = (0.160, 0.000, 0.015)
CUBE_SIZE = 0.030
ZONE_XY = (0.080, -0.160)
ZONE_SIZE = 0.080
CUBE_PLACE = (ZONE_XY[0], ZONE_XY[1], 0.002 + CUBE_SIZE / 2.0)  # day cube = mat zone
HOVER_Z = 0.120
GRASP_TCP_Z = 0.060   # fingertip ~0.025, kep giua cube
PLACE_TCP_Z = 0.067   # cube day cham mat zone khi mo
GRIP_OPEN = 0.0
GRIP_CLOSE = 1.0
SLOW = 0.25


def step(name, cond, detail=""):
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}", flush=True)
    return bool(cond)


def current_tcp_quat(node, tf_buffer, timeout_sec=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            t = tf_buffer.lookup_transform(BASE_LINK, EE_LINK, rclpy.time.Time())
            q = t.transform.rotation
            return [float(q.x), float(q.y), float(q.z), float(q.w)]
        except TransformException:
            continue
    return None


def current_arm_joints(node, timeout_sec=10.0):
    got = {}

    def on_js(msg):
        for n, p in zip(msg.name, msg.position):
            if n in ARM_JOINTS:
                got[n] = float(p)

    sub = node.create_subscription(JointState, "/joint_states", on_js, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec and not all(j in got for j in ARM_JOINTS):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if not all(j in got for j in ARM_JOINTS):
        return None
    return {j: got[j] for j in ARM_JOINTS}


def query_ik(node, ik_cli, xyz, quat, seed, timeout_sec=5.0):
    """IK position-only (KDL) qua /compute_ik, avoid_collisions=True.
    Tra ve list 5 joint pos hoac None. Seed arm1 = atan2(y,x) (kieu chess)."""
    req = GetPositionIK.Request()
    ik = req.ik_request
    ik.group_name = "arm_group"
    ik.ik_link_name = EE_LINK
    ik.avoid_collisions = True
    names = list(ARM_JOINTS)
    pos = [float(seed[j]) for j in names]
    pos[0] = math.atan2(float(xyz[1]), float(xyz[0]))
    ik.robot_state = RobotState()
    ik.robot_state.is_diff = True
    ik.robot_state.joint_state.name = names
    ik.robot_state.joint_state.position = pos
    ik.pose_stamped = PoseStamped()
    ik.pose_stamped.header.frame_id = BASE_LINK
    ik.pose_stamped.pose.position.x = float(xyz[0])
    ik.pose_stamped.pose.position.y = float(xyz[1])
    ik.pose_stamped.pose.position.z = float(xyz[2])
    ik.pose_stamped.pose.orientation.x = float(quat[0])
    ik.pose_stamped.pose.orientation.y = float(quat[1])
    ik.pose_stamped.pose.orientation.z = float(quat[2])
    ik.pose_stamped.pose.orientation.w = float(quat[3])
    ik.timeout.sec = 1
    fut = ik_cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not fut.done():
        return None
    res = fut.result()
    if res is None or res.error_code.val != 1:
        return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    if not all(j in sol for j in ARM_JOINTS):
        return None
    return [float(sol[j]) for j in ARM_JOINTS]


def call_apply(node, apply_cli, world_objects=None, attached=None, timeout_sec=5.0):
    from moveit_msgs.srv import ApplyPlanningScene
    req = ApplyPlanningScene.Request()
    req.scene.is_diff = True
    req.scene.robot_state.is_diff = True
    if world_objects:
        req.scene.world.collision_objects = list(world_objects)
    if attached is not None:
        req.scene.robot_state.attached_collision_objects = [attached]
    fut = apply_cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout_sec:
        time.sleep(0.02)
    res = fut.result() if fut.done() else None
    return res is not None and bool(res.success)


def world_box(obj_id, xyz, size_xyz, operation=CollisionObject.ADD):
    from moveit_msgs.msg import CollisionObject as CO
    obj = CO()
    obj.header = Header(frame_id=BASE_LINK)
    obj.id = obj_id
    obj.operation = operation
    if operation == CO.ADD:
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in size_xyz]
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
    return obj


def _pmsg_bool(name, value):
    return PMsg(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                               bool_value=bool(value)))


def _pmsg_double(name, value):
    return PMsg(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                               double_value=float(value)))


def _pmsg_darray(name, values):
    return PMsg(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                               double_array_value=[float(v) for v in values]))


def set_marker_params(node, params, timeout_sec=5.0):
    """params: dict name->PMsg. Goi truc tiep sim_red_objects/set_parameters."""
    try:
        cli = node.create_client(SetParameters, "sim_red_objects/set_parameters")
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            return False
        makers = {"cube_hidden": _pmsg_bool, "cube_follow": _pmsg_bool,
                  "cube_z": _pmsg_double, "cube_xy": _pmsg_darray,
                  "cube_local": _pmsg_darray}
        req = SetParameters.Request()
        req.parameters = [makers[k](k, v) for k, v in params.items()]
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout_sec:
            time.sleep(0.02)
        if not fut.done():
            return False
        return all(bool(r.successful) for r in fut.result().results)
    except Exception:
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Node("sim_red_run")
    rc = 1
    ex = None
    th = None
    try:
        if "dofbot_driver" in node.get_node_names():
            print("[FAIL] Phát hiện node dofbot_driver (tay thật) -> từ chối thực thi.", flush=True)
            return 2

        print("== Phase 1: check visual + collision ==", flush=True)
        if not run_checks(node):
            print("KET LUAN: check CHUA DAT -> không thực thi.", flush=True)
            return 1

        print("== Phase 2: pick & place tren SIM ==", flush=True)
        cb = ReentrantCallbackGroup()
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
        quat = current_tcp_quat(node, tf_buffer)
        if quat is None:
            print("[FAIL] Không đọc được TF TCP -> không thực thi.", flush=True)
            return 1
        print(f"[PASS] Giữ orientation TCP quat={[round(v, 3) for v in quat]}", flush=True)
        seed = current_arm_joints(node)
        if seed is None:
            print("[FAIL] Không đọc được /joint_states -> không thực thi.", flush=True)
            return 1

        ik_cli = node.create_client(GetPositionIK, "/compute_ik")
        apply_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not ik_cli.wait_for_service(timeout_sec=10.0):
            print("[FAIL] /compute_ik chua san sang.", flush=True)
            return 1
        if not apply_cli.service_is_ready():
            # doi them chut (service ton tai tu luc check scene)
            t0 = time.monotonic()
            while not apply_cli.service_is_ready() and time.monotonic() - t0 < 10.0:
                rclpy.spin_once(node, timeout_sec=0.2)
            if not apply_cli.service_is_ready():
                print("[FAIL] /apply_planning_scene chua san sang.", flush=True)
                return 1

        hover_cube = (CUBE_START[0], CUBE_START[1], HOVER_Z)
        grasp_xyz = (CUBE_START[0], CUBE_START[1], GRASP_TCP_Z)
        hover_zone = (ZONE_XY[0], ZONE_XY[1], HOVER_Z)
        place_xyz = (ZONE_XY[0], ZONE_XY[1], PLACE_TCP_Z)

        sols = {}
        ok = True
        for label, xyz in [("hover cube", hover_cube), ("grasp", grasp_xyz),
                           ("hover zone", hover_zone), ("place", place_xyz)]:
            sol = query_ik(node, ik_cli, xyz, quat, seed)
            ok = step(f"IK {label}", sol is not None,
                      f"target=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"
                      + (f" joints={[round(v, 3) for v in sol]}" if sol else "")) and ok
            if sol is not None:
                sols[label] = sol
                seed = dict(zip(ARM_JOINTS, sol))
        if not ok:
            print("KET LUAN: IK that bai -> không thực thi.", flush=True)
            return 1

        arm = MoveIt2(node=node, joint_names=ARM_JOINTS, base_link_name=BASE_LINK,
                      end_effector_name=EE_LINK, group_name="arm_group", callback_group=cb)
        arm.max_velocity = SLOW
        arm.max_acceleration = SLOW
        arm.allowed_planning_time = 10.0
        grip = MoveIt2(node=node, joint_names=["Rlink1_Joint"], base_link_name=BASE_LINK,
                       end_effector_name=EE_LINK, group_name="grip_group", callback_group=cb)
        ex = MultiThreadedExecutor(2)
        ex.add_node(node)
        th = Thread(target=ex.spin, daemon=True)
        th.start()
        node.create_rate(1.0).sleep()

        def go_joints(label, joints, detail=""):
            traj = arm.plan(joint_positions=[float(v) for v in joints])
            if traj is None:
                return step(f"plan {label}", False, detail or "(plan that bai)")
            arm.execute(traj)
            return step(f"execute {label}", bool(arm.wait_until_executed()), detail)

        def do_grip(label, angle):
            traj = grip.plan(joint_positions=[float(angle)])
            if traj is None:
                return step(f"gripper {label}", False, f"angle={angle}")
            grip.execute(traj)
            return step(f"gripper {label}", bool(grip.wait_until_executed()), f"angle={angle}")

        # 1. mo gripper + hover cube
        ok = do_grip("mo", GRIP_OPEN) and ok
        ok = go_joints("hover cube", sols["hover cube"],
                       f"target=({hover_cube[0]:.3f},{hover_cube[1]:.3f},{hover_cube[2]:.3f})") and ok
        # 2. go bo cube khoi scene (ngon khong va cham luc ha/kep) + ha xuong
        ok = step("go cube khoi scene",
                  call_apply(node, apply_cli,
                             world_objects=[world_box(CUBE_ID, (0, 0, 0), (0, 0, 0),
                                                      operation=CollisionObject.REMOVE)])) and ok
        ok = go_joints("ha xuong gap", sols["grasp"],
                       f"target=({grasp_xyz[0]:.3f},{grasp_xyz[1]:.3f},{grasp_xyz[2]:.3f})") and ok
        # 3. kep + attach (cube dinh theo tay) + an marker cu
        ok = do_grip("kep", GRIP_CLOSE) and ok
        local = None
        try:
            tf_ee_base = tf_buffer.lookup_transform(EE_LINK, BASE_LINK, rclpy.time.Time())
            ps = PoseStamped()
            ps.header.frame_id = BASE_LINK
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = CUBE_START
            ps.pose.orientation.w = 1.0
            local = do_transform_pose(ps.pose, tf_ee_base)  # Humble: Pose -> Pose
            aco = AttachedCollisionObject()
            aco.link_name = EE_LINK
            aco.object.header.frame_id = EE_LINK
            aco.object.id = CUBE_ID
            aco.object.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [CUBE_SIZE] * 3
            aco.object.primitives = [prim]
            aco.object.primitive_poses = [local]
            aco.touch_links = list(FINGER_LINKS)
            ok = step("attach cube", call_apply(node, apply_cli, attached=aco)) and ok
        except Exception as exc:
            ok = step("attach cube", False, f"({exc})") and ok
        if not ok:
            print("KET LUAN: loi khi gap -> dung, khong co tiep.", flush=True)
            return 1
        follow = [0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 1.0] if local is None else [
            float(local.position.x), float(local.position.y), float(local.position.z),
            float(local.orientation.x), float(local.orientation.y),
            float(local.orientation.z), float(local.orientation.w)]
        if not step("marker bam gripper (warn-only)",
                     set_marker_params(node, {"cube_hidden": True, "cube_follow": True,
                                              "cube_local": follow})):
            print("WARN: marker chua theo kịp (can restart lenh 1 lay code moi) - van tiep tuc.", flush=True)
        # 4. nang -> sang zone (go zone khoi scene truoc khi ha de tranh cham)
        ok = go_joints("nang len", sols["hover cube"]) and ok
        ok = go_joints("sang zone", sols["hover zone"],
                       f"target=({hover_zone[0]:.3f},{hover_zone[1]:.3f},{hover_zone[2]:.3f})") and ok
        ok = step("go zone khoi scene",
                  call_apply(node, apply_cli,
                             world_objects=[world_box(ZONE_ID, (0, 0, 0), (0, 0, 0),
                                                      operation=CollisionObject.REMOVE)])) and ok
        ok = go_joints("ha xuong dat", sols["place"],
                       f"target=({place_xyz[0]:.3f},{place_xyz[1]:.3f},{place_xyz[2]:.3f})") and ok
        # 5. mo -> detach + them cube vao scene tai zone + hien marker tai zone + tra zone
        ok = do_grip("mo (tha)", GRIP_OPEN) and ok
        det = AttachedCollisionObject()
        det.link_name = EE_LINK
        det.object.id = CUBE_ID
        det.object.operation = CollisionObject.REMOVE
        ok = step("detach cube",
                  call_apply(node, apply_cli, attached=det,
                             world_objects=[world_box(CUBE_ID, CUBE_PLACE, [CUBE_SIZE] * 3)])) and ok
        ok = step("tra zone vao scene",
                  call_apply(node, apply_cli,
                             world_objects=[world_box(ZONE_ID, (ZONE_XY[0], ZONE_XY[1], 0.005),
                                                      (ZONE_SIZE, ZONE_SIZE, 0.010))])) and ok
        if not step("hien marker cube tai zone (warn-only)",
                     set_marker_params(node, {"cube_xy": [float(ZONE_XY[0]), float(ZONE_XY[1])],
                                              "cube_z": float(CUBE_PLACE[2]),
                                              "cube_hidden": False, "cube_follow": False})):
            print("WARN: marker chua theo kịp (can restart lenh 1 lay code moi) - van tiep tuc.", flush=True)
        # 6. rut len + ve HOME
        ok = go_joints("rut len", sols["hover zone"]) and ok
        ok = go_joints("ve HOME", HOME, "(0,0,0,0,0)") and ok
        print("KET LUAN: " + ("XONG - cube da nam trong zone, tay ve HOME "
                              "(demo 1-shot; chay lai can restart lenh 1)" if ok
                              else "LOI GIỮA CHỪNG - kiểm tra RViz truoc khi chay tiep"),
              flush=True)
        rc = 0 if ok else 1
        return rc
    except KeyboardInterrupt:
        print("Huy boi nguoi dung (Ctrl+C).", flush=True)
        return 130
    finally:
        try:
            if ex is not None:
                ex.shutdown()
        except Exception:
            pass
        try:
            if th is not None:
                th.join(timeout=5.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(rc)


if __name__ == "__main__":
    main()
