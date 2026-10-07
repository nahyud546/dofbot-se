"""Opt-in: hand-eye layer detection through real ROS topics (localhost domain 78)."""
import copy
import json
import math
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))

from t8_ros_scene import RosSceneBridge  # noqa: E402

SERVO = [86.0, 118.0, 6.0, 9.0, 90.0]


@unittest.skipUnless(os.environ.get("T8_RUN_ROS_INTEGRATION") == "1", "opt-in ROS transport test")
class HandEyeIntegrationTests(unittest.TestCase):
    def setUp(self):
        import test_cube_layer as world
        from scipy.spatial.transform import Rotation
        self.world, self.rotation = world, Rotation
        self.tmp = tempfile.TemporaryDirectory()
        cal = Path(self.tmp.name) / "hand_eye.json"
        cal.write_text(json.dumps({
            "accepted": True, "created": "test", "tag_top_z": world.TAG_TOP_Z,
            "arm4_T_optical": world.MOUNT.ravel().tolist(),
            "K": list(world.TRUE_K), "k1": world.TRUE_K1,
            "held_out_xy_m": [0.001] * 3}))
        config = Path(self.tmp.name) / "perception.yaml"
        config.write_text("camera:\n  K: [%s]\n  distortion: [%s, 0, 0, 0, 0]\n" % (
            ", ".join(str(v) for v in (world.TRUE_K[0], 0, world.TRUE_K[2], 0,
                                       world.TRUE_K[1], world.TRUE_K[3], 0, 0, 1)),
            world.TRUE_K1))
        os.environ.update(ROS_DOMAIN_ID="78", ROS_LOCALHOST_ONLY="1",
                          ROS_LOG_DIR="/tmp/t8_scene_test_logs",
                          T8_HAND_EYE_FILE=str(cal), T8_PERCEPTION_CONFIG=str(config))
        import rclpy
        from rclpy.node import Node
        from cap_scene_interfaces.msg import ObjectStates
        from sensor_msgs.msg import JointState
        self.ros = rclpy
        rclpy.init()
        self.node = Node("t8_handeye_integration")
        self.scenes = self.node.create_publisher(ObjectStates, "/vision/object_states", 10)
        self.joints = self.node.create_publisher(JointState, "/real_joint_states", 10)
        self.moving = False
        self.rng = np.random.default_rng(4)
        # Cube 3 on the table, cube 1 on top of another cube, as perception would see them.
        self.truth = {}
        self.poses = {}
        for identity, pixel, layer in ((3, (250, 250), 0), (1, (400, 200), 1)):
            xy = self.cube_under_pixel(pixel, layer)
            T = world.perceive(xy, layer, 0.3, SERVO, world.TRUE_PERCEPTION_K, self.rng)
            self.assertIsNotNone(T, "test cube must be visible")
            self.truth[identity] = (xy, layer)
            self.poses[identity] = T
        self.node.create_timer(0.1, self.publish)
        self.thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        self.thread.start()

    def cube_under_pixel(self, pixel, layer):
        """Table XY whose top face (at its layer height) projects to this pixel."""
        world = self.world
        import cube_search_center_math as M
        B = M.fk_arm4(SERVO) @ world.MOUNT
        xd, yd = (pixel[0] - world.TRUE_K[2]) / world.TRUE_K[0], (pixel[1] - world.TRUE_K[3]) / world.TRUE_K[1]
        x, y = xd, yd
        for _ in range(8):
            d = 1 + world.TRUE_K1 * (x * x + y * y)
            x, y = xd / d, yd / d
        ray = B[:3, :3] @ np.array([x, y, 1.0])
        z = world.TAG_TOP_Z + 0.03 * layer
        hit = B[:3, 3] + ray * (z - B[2, 3]) / ray[2]
        return (float(hit[0]), float(hit[1]))

    def publish(self):
        from cap_scene_interfaces.msg import ObjectState, ObjectStates
        from sensor_msgs.msg import JointState
        stamp = self.node.get_clock().now().to_msg()
        servo = [v + (5.0 * math.sin(stamp.nanosec * 1e-9 * 7) if self.moving else 0.0)
                 for v in SERVO]
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f"arm{i}_Joint" for i in range(1, 6)]
        joints.position = [math.radians(v - 90.0) for v in servo]
        self.joints.publish(joints)
        scene = ObjectStates()
        scene.header.stamp = stamp
        scene.calibrated = False
        for identity, T in self.poses.items():
            obj = ObjectState()
            obj.object_id = obj.geometry_model_id = identity
            obj.track_id = f"cube-{identity}"
            obj.bbox_xyxy = [250 + identity, 170, 330 + identity, 250]
            obj.pose_method = "rgb_single_face_cube"
            obj.camera_pose_valid = obj.top_grasp_ready = True
            obj.camera_pose.header.stamp = copy.deepcopy(stamp)
            obj.camera_pose.pose.position.x, obj.camera_pose.pose.position.y, \
                obj.camera_pose.pose.position.z = T[:3, 3]
            q = self.rotation.from_matrix(T[:3, :3]).as_quat()
            (obj.camera_pose.pose.orientation.x, obj.camera_pose.pose.orientation.y,
             obj.camera_pose.pose.orientation.z, obj.camera_pose.pose.orientation.w) = q
            scene.objects.append(obj)
        self.scenes.publish(scene)

    def tearDown(self):
        if self.ros.ok():
            self.ros.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.tmp.cleanup()

    def test_snapshot_reports_layers_and_xy_from_real_topics(self):
        result = RosSceneBridge().snapshot()
        self.assertTrue(result.get("ok"), result)
        self.assertTrue(result["handeye_available"], result)
        by_id = {o["object_id"]: o for o in result["objects"]}
        for identity, (xy, layer) in self.truth.items():
            located = by_id[identity]["handeye"]
            self.assertTrue(located["ok"], located)
            self.assertEqual(located["layer"], layer)
            self.assertLess(math.hypot(located["x"] - xy[0], located["y"] - xy[1]), 0.004)

    def test_moving_arm_is_not_trusted(self):
        self.moving = True
        result = RosSceneBridge().snapshot()
        self.assertTrue(result.get("ok"), result)
        for obj in result["objects"]:
            self.assertFalse(obj["handeye"]["ok"], obj["handeye"])
            self.assertIn("khớp", obj["handeye"]["reason"])
