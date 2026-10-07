"""Opt-in ROS transport test using synthetic publishers on localhost domain 77."""

import os
import copy
import threading
import unittest

from t8_ros_scene import RosSceneBridge


@unittest.skipUnless(os.environ.get("T8_RUN_ROS_INTEGRATION") == "1", "opt-in ROS transport test")
class RosSceneIntegrationTests(unittest.TestCase):
    def setUp(self):
        os.environ.update(ROS_DOMAIN_ID="77", ROS_LOCALHOST_ONLY="1",
                          ROS_LOG_DIR="/tmp/t8_scene_test_logs")
        import rclpy
        from rclpy.node import Node
        from cap_scene_interfaces.msg import ObjectStates
        from sensor_msgs.msg import JointState
        self.ros = rclpy
        rclpy.init()
        self.node = Node("t8_synthetic_scene_test")
        self.scenes = self.node.create_publisher(ObjectStates, "/vision/object_states", 10)
        self.joints = self.node.create_publisher(JointState, "/real_joint_states", 10)
        self.calibrated = True
        self.stale_pose = False
        self.node.create_timer(0.1, self.publish)

        def spin():
            from rclpy.executors import ExternalShutdownException
            try:
                rclpy.spin(self.node)
            except ExternalShutdownException:
                pass
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()

    def publish(self):
        from cap_scene_interfaces.msg import ObjectState, ObjectStates
        from sensor_msgs.msg import JointState
        stamp = self.node.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f"arm{i}_Joint" for i in range(1, 6)]
        joints.position = [0.0] * 5
        self.joints.publish(joints)
        scene = ObjectStates()
        scene.header.stamp = stamp
        scene.calibrated = self.calibrated
        for identity, y in [(3, -0.03), (1, 0.04)]:
            obj = ObjectState()
            obj.object_id = obj.geometry_model_id = identity
            obj.track_id = f"cube-{identity}"
            obj.bbox_xyxy = [250 + identity, 170, 330 + identity, 250]
            obj.pose_method = "rgb_single_face_cube"
            obj.camera_pose_valid = obj.base_pose_valid = obj.top_grasp_ready = True
            obj.camera_pose.header.stamp = copy.deepcopy(stamp)
            obj.camera_pose.pose.position.z = .2
            obj.camera_pose.pose.orientation.x = 1.0
            obj.camera_pose.pose.orientation.w = 0.0
            obj.base_pose.header.stamp = copy.deepcopy(stamp)
            obj.base_pose.header.frame_id = "base_link"
            obj.base_pose.pose.position.x = -.2
            obj.base_pose.pose.position.y = y
            obj.base_pose.pose.position.z = .045
            obj.base_pose.pose.orientation.w = 1.0
            if self.stale_pose:
                obj.camera_pose.header.stamp.sec -= 3
                obj.base_pose.header.stamp.sec -= 3
            scene.objects.append(obj)
        self.scenes.publish(scene)

    def tearDown(self):
        self.ros.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()

    def test_measured_scene_crosses_system_python_boundary(self):
        result = RosSceneBridge().snapshot()
        self.assertTrue(result.get("ok"), result)
        self.assertEqual({obj["object_id"] for obj in result["objects"]}, {1, 3})
        self.assertTrue(all(obj["confirmed"] and obj["stationary"] for obj in result["objects"]), result)

    def test_uncalibrated_scene_keeps_ids_but_rejects_metric_targets(self):
        self.calibrated = False
        result = RosSceneBridge().snapshot()
        self.assertTrue(result.get("ok"), result)
        self.assertFalse(result["calibrated"])
        self.assertTrue(all(not obj["base_pose_valid"] and obj["grasp"] is not None
                            for obj in result["objects"]))

    def test_fresh_envelope_cannot_refresh_an_old_object_pose(self):
        self.stale_pose = True
        result = RosSceneBridge().snapshot()
        self.assertTrue(result.get("ok"), result)
        self.assertTrue(all(not obj["base_pose_valid"] and not obj["confirmed"]
                            and obj["grasp"] is None for obj in result["objects"]))
