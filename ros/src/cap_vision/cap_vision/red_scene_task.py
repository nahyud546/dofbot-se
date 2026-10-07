"""Observe -> collision-aware staged plan -> export/simulate -> verify.

Default is plan-only. Hardware consumes the full export through arm_hw_bridge;
use --verify after retreat with measured joints and fresh camera observations.
"""
import argparse
from copy import deepcopy
import json
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from moveit_msgs.msg import (RobotState, Constraints, JointConstraint, CollisionObject,
    AttachedCollisionObject, PlanningSceneComponents, AllowedCollisionEntry, DisplayTrajectory,
    RobotTrajectory)
from moveit_msgs.srv import (GetPositionIK, GetPositionFK, GetMotionPlan,
    GetPlanningScene, ApplyPlanningScene, GetCartesianPath)
from moveit_msgs.action import ExecuteTrajectory
from cap_scene_interfaces.msg import SceneObjects

from cap_vision.red_scene_core import load_config, transform
from cap_vision.red_scene_node import stamp_seconds
from cap_vision.red_task_core import JOINTS, validate_task, targets, append_trajectory, placement_error
from rosidl_runtime_py.convert import message_to_ordereddict

EE = "Gripping_point_Link"
FINGERS = [EE, "Rlink1_Link", "Rlink2_Link", "Rlink3_Link", "Llink1_Link", "Llink2_Link", "Llink3_Link"]


def xyz(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def robot_state(q):
    state = RobotState(is_diff=False)
    state.joint_state.name = list(JOINTS)
    state.joint_state.position = [float(q[j]) for j in JOINTS]
    return state


def box(oid, pose, size):
    out = CollisionObject(header=Header(frame_id="base_link"), id=oid, operation=CollisionObject.ADD)
    out.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
    out.primitive_poses = [deepcopy(pose)]
    return out


class Task(Node):
    def __init__(self, cfg, state_topic):
        super().__init__("red_scene_task")
        self.cfg, self.observed, self.q, self.q_stamp = cfg, {}, {}, 0.0
        self.started = self.get_clock().now().nanoseconds * 1e-9
        self.state_pub = self.create_publisher(String, "/red_scene/task_state", 10)
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.create_subscription(SceneObjects, "/red_scene/objects", self.on_objects, 10)
        self.create_subscription(JointState, state_topic, self.on_joints, 10)
        self.service_clients = {name: self.create_client(kind, name) for name, kind in [
            ("/compute_ik", GetPositionIK), ("/compute_fk", GetPositionFK),
            ("/plan_kinematic_path", GetMotionPlan), ("/compute_cartesian_path", GetCartesianPath),
            ("/get_planning_scene", GetPlanningScene), ("/apply_planning_scene", ApplyPlanningScene)]}
        self.attachment = None

    def state(self, name):
        self.state_pub.publish(String(data=name))
        self.get_logger().info(name)

    def on_objects(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not msg.calibrated or msg.header.frame_id != "base_link":
            self.observed.clear()
            return
        for obj in msg.objects:
            stamp = stamp_seconds(obj.header.stamp)
            if (obj.valid and obj.header.frame_id == "base_link" and stamp >= self.started
                    and 0 <= now - stamp <= self.cfg["perception"]["max_age_sec"]):
                self.observed[(obj.kind, obj.color)] = deepcopy(obj)

    def on_joints(self, msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q and math.isfinite(q[j]) for j in JOINTS):
            self.q, self.q_stamp = q, stamp_seconds(msg.header.stamp)

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError("timeout waiting for fresh observations / state / service")

    def call(self, name, request):
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"missing {name}")
        future = client.call_async(request)
        self.wait(future.done, 15)
        return future.result()

    def snapshot(self, timeout):
        self.state("OBSERVE: waiting for stable cube and zone (may use separate stationary views)")
        t = self.cfg["task"]
        keys = [(t["object_kind"], t["object_color"]), (t["target_kind"], t["target_color"])]

        def ready():
            now = self.get_clock().now().nanoseconds * 1e-9
            return (all(k in self.observed and 0 <= now-stamp_seconds(self.observed[k].header.stamp) <= t["snapshot_age_sec"] for k in keys)
                    and all(j in self.q for j in JOINTS) and 0 <= now-self.q_stamp <= 0.5)
        self.wait(ready, timeout)
        return [deepcopy(self.observed[k]) for k in keys]

    def apply(self, objects=None, attachment=None, acm=None):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        if objects is not None:
            req.scene.world.collision_objects = objects
        if attachment is not None:
            req.scene.robot_state.attached_collision_objects = [attachment]
        if acm is not None:
            req.scene.allowed_collision_matrix = acm
        if not self.call("/apply_planning_scene", req).success:
            raise RuntimeError("Planning Scene update failed")

    def fk(self, q):
        req = GetPositionFK.Request(header=Header(frame_id="base_link"), fk_link_names=[EE], robot_state=robot_state(q))
        res = self.call("/compute_fk", req)
        if res.error_code.val != 1 or not res.pose_stamped:
            raise RuntimeError("FK failed")
        return res.pose_stamped[0].pose

    def ik(self, target, q):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm_group"
        req.ik_request.ik_link_name = EE
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state = robot_state(q)
        if self.attachment:
            req.ik_request.robot_state.attached_collision_objects = [self.attachment]
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        pose = req.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, target)
        quat = self.cfg["task"]["tcp_orientation_xyzw"]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.ik_request.timeout.sec = 2
        res = self.call("/compute_ik", req)
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {res.error_code.val}")
        solution = dict(q)
        solution.update(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        actual = self.fk(solution)
        aq = actual.orientation
        dot = abs(float(np.dot([aq.x, aq.y, aq.z, aq.w], quat)))
        if np.linalg.norm(xyz(actual) - target) > 0.003 or dot < math.cos(math.radians(5)/2):
            raise RuntimeError("IK violates grasp pose (position-only IK is insufficient)")
        return solution

    def plan(self, q, target, group, cartesian_pose=None):
        if cartesian_pose is not None:
            req = GetCartesianPath.Request()
            req.header.frame_id = "base_link"
            req.start_state = robot_state(q)
            if self.attachment:
                req.start_state.attached_collision_objects = [self.attachment]
            req.group_name, req.link_name = group, EE
            req.waypoints = [cartesian_pose]
            req.max_step, req.jump_threshold = 0.003, 2.0
            req.avoid_collisions = True
            # Support Humble installations with and without scaling fields.
            if hasattr(req, "max_velocity_scaling_factor"):
                req.max_velocity_scaling_factor = 1.0
                req.max_acceleration_scaling_factor = 1.0
            res = self.call("/compute_cartesian_path", req)
            if res.error_code.val != 1 or res.fraction < 0.999:
                raise RuntimeError(f"incomplete Cartesian path: {res.fraction}")
            traj = res.solution
            # Humble Cartesian service has no scaling fields: stretch time uniformly.
            scale = min(self.cfg["task"]["velocity_scale"], math.sqrt(self.cfg["task"]["acceleration_scale"]))
            for pt in traj.joint_trajectory.points:
                seconds = stamp_seconds(pt.time_from_start) / scale
                pt.time_from_start = Duration(seconds=seconds).to_msg()
                pt.velocities = [v * scale for v in pt.velocities]
                pt.accelerations = [v * scale * scale for v in pt.accelerations]
            return traj
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name = group
        motion.start_state = robot_state(q)
        if self.attachment:
            motion.start_state.attached_collision_objects = [self.attachment]
        motion.allowed_planning_time = 5.0
        motion.num_planning_attempts = 3
        motion.max_velocity_scaling_factor = self.cfg["task"]["velocity_scale"]
        motion.max_acceleration_scaling_factor = self.cfg["task"]["acceleration_scale"]
        joints = JOINTS[:5] if group == "arm_group" else JOINTS[5:]
        goal = Constraints()
        goal.joint_constraints = [JointConstraint(joint_name=j, position=float(target[j]),
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for j in joints]
        motion.goal_constraints = [goal]
        res = self.call("/plan_kinematic_path", req).motion_plan_response
        if res.error_code.val != 1:
            raise RuntimeError(f"planning failed: {res.error_code.val}")
        return res.trajectory

    def execute_sim(self, trajectory):
        client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        try:
            if not client.wait_for_server(timeout_sec=3):
                raise RuntimeError("execute action unavailable")
            future = client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
            self.wait(future.done)
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("trajectory rejected")
            result = handle.get_result_async()
            try:
                duration = stamp_seconds(trajectory.joint_trajectory.points[-1].time_from_start)
                self.wait(result.done, duration + 10)
            except BaseException:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=2)
                raise
            if result.result().result.error_code.val != 1:
                raise RuntimeError("trajectory execution failed")
        finally:
            client.destroy()


def main(argv=None, task_node=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--output", default="/tmp/red_scene_trajectory.json")
    ap.add_argument("--observation-timeout", type=float, default=60)
    ap.add_argument("--execute-sim", action="store_true")
    ap.add_argument("--verify", action="store_true", help="fresh post-retreat camera verification only")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    validate_task(cfg)
    if not args.verify and Path(args.output).exists():
        raise SystemExit("output exists; choose a new --output")
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = task_node or Task(cfg, "/joint_states" if args.execute_sim else "/real_joint_states")
    original_acm = None
    cube_obj = None
    completed = False
    try:
        if args.execute_sim:
            # A dedicated ROS domain is required, so no sim follower can see commands.
            import os
            if int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0:
                raise RuntimeError("--execute-sim requires a dedicated nonzero ROS_DOMAIN_ID")
            forbidden = {"dofbot_driver", "arm_hw_mirror", "follow_sim", "duplex_sync"}
            if forbidden.intersection(node.get_node_names()):
                raise RuntimeError("hardware bridge/follower present in simulation graph")
        cube, zone = node.snapshot(args.observation_timeout)
        if args.verify:
            error = placement_error(xyz(cube.pose), xyz(zone.pose), cfg)
            node.state(f"VERIFY: center error={error*1000:.1f}mm")
            if error > cfg["task"]["placement_tolerance_m"]:
                raise RuntimeError("placement outside tolerance")
            node.state("SUCCESS: fresh visual verification passed")
            return
        # Ensure the zone can contain the cube with clearance to its edges.
        if min(zone.size.x, zone.size.y) < cfg["perception"]["cube_size"] + 2*cfg["task"]["placement_tolerance_m"]:
            raise RuntimeError("zone too small for requested placement tolerance")
        start = {j: node.q[j] for j in JOINTS}
        q = dict(start)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        original_acm = deepcopy(node.call("/get_planning_scene", req).scene.allowed_collision_matrix)
        acm = deepcopy(original_acm)
        for name in ["red_cube", *FINGERS]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        # Only fingertips may contact the target cube. No blanket collision disable.
        ci = acm.entry_names.index("red_cube")
        for link in FINGERS:
            li = acm.entry_names.index(link)
            acm.entry_values[ci].enabled[li] = acm.entry_values[li].enabled[ci] = True
        environment = []
        for item in cfg["task"]["environment_boxes"]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, item["center"])
            pose.orientation.w = 1.0
            environment.append(box(item["id"], pose, item["size"]))
        cube_obj = box("red_cube", cube.pose, [cube.size.x, cube.size.y, cube.size.z])
        node.apply(objects=environment + [cube_obj], acm=acm)
        execution_scene = {"objects": [message_to_ordereddict(o) for o in environment + [cube_obj]],
                           "acm": message_to_ordereddict(acm),
                           "restore_acm": message_to_ordereddict(original_acm), "events": []}
        goals = targets(xyz(cube.pose), xyz(zone.pose), cfg)
        sequence = [("OPEN", "grip", cfg["task"]["gripper_open_rad"]),
                    ("APPROACH", "arm", "hover_cube"), ("DESCEND_PICK", "linear", "grasp"),
                    ("CLOSE", "grip", cfg["task"]["gripper_closed_rad"]),
                    ("LIFT", "linear", "hover_cube"), ("TRANSFER", "arm", "hover_zone"),
                    ("DESCEND_PLACE", "linear", "place"),
                    ("RELEASE", "grip", cfg["task"]["gripper_open_rad"]),
                    ("RETREAT", "linear", "hover_zone"), ("RETURN_OBSERVE", "home", None)]
        points = [{"positions": [q[j] for j in JOINTS], "time_from_start": 0.01}]
        offset, segments = 0.01, []
        for label, kind, target in sequence:
            node.state("PLAN " + label)
            before = dict(q)
            target_q = dict(q)
            cartesian_pose = None
            if kind == "grip":
                target_q[JOINTS[-1]] = target
            elif kind == "home":
                target_q.update({j: start[j] for j in JOINTS[:5]})
            else:
                target_q = node.ik(goals[target], q)
                if kind == "linear":
                    cartesian_pose = node.fk(target_q)
            trajectory = node.plan(q, target_q, "grip_group" if kind == "grip" else "arm_group", cartesian_pose)
            begin = len(points)
            offset = append_trajectory(points, trajectory.joint_trajectory, q, offset)
            if max(abs(q[j] - target_q[j]) for j in JOINTS) > 0.02:
                raise RuntimeError("planned endpoint differs from requested goal")
            node.display.publish(DisplayTrajectory(trajectory_start=robot_state(before), trajectory=[trajectory]))
            segments.append({"state": label, "first_point": max(0, begin-1), "last_point": len(points)-1})
            if args.execute_sim:
                node.execute_sim(trajectory)
            if label == "CLOSE":
                # Express the measured cube relative to the planned gripper pose.
                tcp = node.fk(q)
                orientation = tcp.orientation
                from cap_vision.red_scene_node import rotation_quaternion
                from scipy.spatial.transform import Rotation
                base_T_tcp = np.eye(4)
                base_T_tcp[:3, :3] = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
                base_T_tcp[:3, 3] = xyz(tcp)
                cp = cube.pose.orientation
                base_T_cube = np.eye(4)
                base_T_cube[:3, :3] = Rotation.from_quat([cp.x, cp.y, cp.z, cp.w]).as_matrix()
                base_T_cube[:3, 3] = xyz(cube.pose)
                relative = np.linalg.inv(base_T_tcp) @ base_T_cube
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, relative[:3, 3])
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation_quaternion(relative[:3, :3])
                attached_obj = box("red_cube", pose, [cube.size.x, cube.size.y, cube.size.z])
                attached_obj.header.frame_id = EE
                node.attachment = AttachedCollisionObject(link_name=EE, object=attached_obj, touch_links=FINGERS)
                node.apply(objects=[CollisionObject(id="red_cube", operation=CollisionObject.REMOVE)], attachment=node.attachment)
                execution_scene["events"].append({"after_point": len(points)-1,
                    "objects": [message_to_ordereddict(CollisionObject(id="red_cube", operation=CollisionObject.REMOVE))],
                    "attachment": message_to_ordereddict(node.attachment)})
            elif label == "RELEASE":
                dropped = deepcopy(cube_obj)
                normal = transform(cfg["table"]["base_T_table"])[:3, 2]
                center = xyz(zone.pose) + normal*(cfg["perception"]["cube_size"]/2)
                dropped.primitive_poses[0].position.x, dropped.primitive_poses[0].position.y, dropped.primitive_poses[0].position.z = map(float, center)
                node.apply(objects=[dropped], attachment=AttachedCollisionObject(link_name=EE,
                    object=CollisionObject(id="red_cube", operation=CollisionObject.REMOVE)))
                node.attachment = None
                execution_scene["events"].append({"after_point": len(points)-1,
                    "objects": [message_to_ordereddict(dropped)],
                    "attachment": message_to_ordereddict(AttachedCollisionObject(link_name=EE,
                        object=CollisionObject(id="red_cube", operation=CollisionObject.REMOVE)))})
        payload = {"joint_names": JOINTS, "points": points, "segments": segments,
                   "execution_scene": execution_scene,
                   "meta": {"source": "red_scene", "cube": xyz(cube.pose).tolist(),
                            "zone": xyz(zone.pose).tolist(), "config": str(Path(args.config).resolve()),
                            "config_sha256": hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
                            "observation_stamps": [stamp_seconds(cube.header.stamp), stamp_seconds(zone.header.stamp)],
                            "scene_objects": [b["id"] for b in cfg["task"]["environment_boxes"]],
                            "velocity_scale": cfg["task"]["velocity_scale"],
                            "acceleration_scale": cfg["task"]["acceleration_scale"],
                            "created_at": node.get_clock().now().nanoseconds*1e-9,
                            "visual_verification_required": True}}
        with open(args.output, "x", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2)
        preview = RobotTrajectory()
        preview.joint_trajectory.joint_names = list(JOINTS)
        preview.joint_trajectory.points = [JointTrajectoryPoint(positions=row["positions"],
            time_from_start=Duration(seconds=row["time_from_start"]).to_msg()) for row in points]
        node.display.publish(DisplayTrajectory(trajectory_start=robot_state(start), trajectory=[preview]))
        completed = True
        node.state(f"{'SIMULATED' if args.execute_sim else 'PLANNED'}: {args.output}; physical success requires --verify")
    except BaseException:
        node.state("FAILED: no further segments will execute")
        raise
    finally:
        # Restore hypothetical scene after plan-only. During execution failure keep
        # attachment/scene evidence for recovery; never pretend the cube was released.
        if not args.execute_sim and cube_obj is not None:
            try:
                node.apply(objects=[cube_obj], attachment=AttachedCollisionObject(link_name=EE,
                    object=CollisionObject(id="red_cube", operation=CollisionObject.REMOVE)), acm=original_acm)
            except Exception as exc:
                node.get_logger().error(f"Scene cleanup failed: {exc}")
        elif completed and original_acm is not None:
            node.apply(acm=original_acm)
        if task_node is None:
            node.destroy_node()
        if owns_context:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
