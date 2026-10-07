"""Calibrated coordinates free T8 from READY_POSE; the fixed map does not."""
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dofbot_ik  # noqa: E402
import t8_motion_worker as worker  # noqa: E402
from t8_ros_scene_worker import metric_top_grasp, table_level_centre_z  # noqa: E402

AWAY_FROM_READY = [60.0, 110.0, 20.0, 30.0, 90.0]


class FakeArm:
    def __init__(self, joints, gripper=worker.OPEN_ANGLE):
        self.joints = [float(v) for v in joints] + [float(gripper)]
        self.moves = []

    def Arm_serial_servo_read(self, joint):
        return int(round(self.joints[joint - 1]))

    def Arm_serial_servo_write6(self, *values):
        self.joints = [float(v) for v in values[:6]]
        self.moves.append(list(values[:5]))

    def Arm_serial_servo_write6_array(self, values, _ms):
        self.joints = [float(v) for v in values[:6]]
        self.moves.append(list(values[:5]))

    def Arm_serial_servo_write(self, joint, angle, _ms):
        self.joints[joint - 1] = float(angle)


def lock(tcp, source):
    return {"object_kind": "cube", "cube_id": 3, "geometry_model_id": 3,
            "identity_source": "apriltag", "track_id": "t3",
            "grasp": {"tcp_position_base": list(tcp), "preferred_yaw_rad": 0.0,
                      "coordinate_source": source}}


def pick_request(tcp, source):
    kin = worker.Kinematics()
    plan = []
    for stage, lift in (("approach_pick", .020), ("pick", 0.0), ("lift_pick", .030)):
        point = [tcp[0], tcp[1], tcp[2] + lift]
        plan.append({"stage": stage, "tcp_position_base": point,
                     "ik_joints_deg": worker.solve_approved_tcp(kin, point, 0.0)})
    return {"command": "pick_cube_3d", "approval_token": "a" * 32,
            "approved_at": time.time(), "preflight_ok": True,
            "preflight_plan": plan, "source": lock(tcp, source)}, kin


class AnyPoseWorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        patcher = patch.object(worker, "STATE_FILE", Path(self.tmp.name) / "state.json")
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self.tmp.cleanup)
        sleeper = patch.object(worker.time, "sleep", lambda _s: None)
        sleeper.start()
        self.addCleanup(sleeper.stop)

    def test_calibrated_pick_runs_from_a_non_ready_pose(self):
        tcp = [-0.21, -0.05, 0.047]
        request, kin = pick_request(tcp, "calibrated_base_pose")
        arm = FakeArm(AWAY_FROM_READY)
        result = worker.execute(request, arm, kin)
        self.assertTrue(result["ok"] and result["holding"])
        self.assertEqual(len(arm.moves), 3)
        self.assertLess(math.dist(dofbot_ik.fk(arm.moves[1]), tcp), 0.004)

    def test_fixed_map_pick_still_refuses_a_non_ready_pose(self):
        request, kin = pick_request([-0.18, 0.0, 0.047], "fixed_ready_pose")
        arm = FakeArm(AWAY_FROM_READY)
        with self.assertRaisesRegex(RuntimeError, "Khớp chưa tới đích"):
            worker.execute(request, arm, kin)
        self.assertEqual(arm.moves, [])

    def test_keep_pose_prepare_does_not_move_and_opens_gripper(self):
        arm = FakeArm(AWAY_FROM_READY, gripper=worker.CLOSE_ANGLE)
        result = worker.execute({"command": "prepare", "keep_pose": True}, arm)
        self.assertTrue(result["ok"] and result["kept_pose"])
        self.assertEqual(arm.moves, [])
        self.assertEqual(arm.joints[:5], AWAY_FROM_READY)
        self.assertEqual(arm.joints[5], worker.OPEN_ANGLE)

    def test_plain_prepare_still_returns_to_ready(self):
        arm = FakeArm(AWAY_FROM_READY)
        self.assertTrue(worker.execute({"command": "prepare"}, arm)["ok"])
        self.assertEqual(arm.joints[:5], worker.READY_POSE[:5])

    def test_first_move_is_slower_for_a_long_approach(self):
        arm = FakeArm(AWAY_FROM_READY)
        near = worker.first_move_ms(arm, [62, 112, 22, 30, 90])
        far = worker.first_move_ms(arm, [150, 40, 80, 10, 90])
        self.assertEqual(near, 1000)
        self.assertGreater(far, 1700)


class MetricGraspTests(unittest.TestCase):
    @staticmethod
    def pose(x, y, z, yaw_deg=0.0, tilt_deg=0.0):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        ct, st = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
        # Rz(yaw) @ Rx(tilt)
        return [[c, -s * ct, s * st, x], [s, c * ct, -c * st, y],
                [0.0, st, ct, z], [0, 0, 0, 1]]

    def test_table_cube_uses_proven_height_and_base_yaw(self):
        grasp, why = metric_top_grasp(self.pose(-0.2, 0.05, 0.051, yaw_deg=20), 0.048)
        self.assertEqual(why, "")
        self.assertEqual(grasp["tcp_position_base"], [-0.2, 0.05, 0.047])
        self.assertAlmostEqual(math.degrees(grasp["preferred_yaw_rad"]), 20.0, places=5)
        self.assertEqual(grasp["surface_id"], "+Z")
        self.assertEqual(grasp["coordinate_source"], "calibrated_base_pose")

    def test_yaw_is_reduced_by_cube_symmetry(self):
        grasp, _ = metric_top_grasp(self.pose(-0.2, 0.0, 0.048, yaw_deg=110), 0.048)
        self.assertAlmostEqual(math.degrees(grasp["preferred_yaw_rad"]), 20.0, places=5)

    def test_second_layer_adds_one_cube_height(self):
        grasp, _ = metric_top_grasp(self.pose(-0.2, 0.0, 0.080), 0.048)
        self.assertAlmostEqual(grasp["tcp_position_base"][2], 0.077)

    def test_tilted_or_floating_cube_is_rejected(self):
        self.assertIsNone(metric_top_grasp(self.pose(-0.2, 0.0, 0.048, tilt_deg=40), 0.048)[0])
        self.assertIsNone(metric_top_grasp(self.pose(-0.2, 0.0, 0.063), 0.048)[0])
        self.assertIsNone(metric_top_grasp(self.pose(-0.2, 0.0, 0.020), 0.048)[0])

    def test_without_calibration_file_height_is_left_to_the_pose(self):
        self.assertIsNone(table_level_centre_z("/nonexistent/hand_eye.json"))
        grasp, _ = metric_top_grasp(self.pose(-0.2, 0.0, 0.050), None)
        self.assertAlmostEqual(grasp["tcp_position_base"][2], 0.052)


if __name__ == "__main__":
    unittest.main()
