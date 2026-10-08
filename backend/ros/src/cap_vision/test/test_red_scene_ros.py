"""ROS message/node integration; run in an isolated ROS_DOMAIN_ID."""
from copy import deepcopy
import time

import numpy as np
import pytest
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter

from cap_vision.red_scene_node import RedSceneNode
from test_red_scene import cfg, camera, scene_image


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))


@pytest.fixture
def node(tmp_path, cfg):
    path = tmp_path / "scene.yaml"
    path.write_text(yaml.safe_dump(cfg))
    rclpy.init(args=["--ros-args", "-p", f"config:={path}"])
    instance = RedSceneNode()
    for name in ("objects", "markers", "status", "annotated"):
        setattr(instance, name, Collector())
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_stamped_detection_and_watchdog(node):
    node.cfg["perception"]["stable_frames"] = 1
    node.filter.count = 1
    now = node.get_clock().now().nanoseconds * 1e-9
    node.stationary.add(now - 0.45, [0]*5)
    node.stationary.add(now - 0.01, [0]*5)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "measured/Camera_Link"
    tf.transform.translation.z = 0.4
    tf.transform.rotation.x = 1.0
    tf.transform.rotation.w = 0.0
    node.tf.set_transform_static(tf, "test")
    msg = CvBridge().cv2_to_imgmsg(scene_image(), "bgr8")
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "camera_optical_frame"
    from cap_vision.red_scene_node import tf_matrix
    from rclpy.time import Time
    expected_camera = np.diag([1., -1., -1., 1.])
    expected_camera[2, 3] = 0.4
    assert np.allclose(tf_matrix(node.tf.lookup_transform("base_link", "measured/Camera_Link", Time.from_msg(msg.header.stamp))), expected_camera)
    assert np.array_equal(CvBridge().imgmsg_to_cv2(msg, "bgr8"), scene_image())
    node.on_image(msg)
    actual = node.objects.messages[-1]
    assert len(actual.objects) == 2, (actual.status, node.cfg, node.filter.samples)
    assert actual.header.stamp == msg.header.stamp
    assert msg.header.frame_id == "camera_optical_frame"
    assert node.annotated.messages[-1].header.frame_id == "camera_optical_frame"
    node.on_image(msg)
    assert not node.objects.messages[-1].objects
    assert "duplicate" in node.objects.messages[-1].status
    node.last_image -= 2
    node.watchdog()
    assert node.objects.messages[-1].status == "image timeout"
    assert not node.objects.messages[-1].objects


def test_uncalibrated_and_missing_tf_never_publish_objects(node):
    msg = CvBridge().cv2_to_imgmsg(scene_image(), "bgr8")
    msg.header.stamp = node.get_clock().now().to_msg()
    node.error = "calibration missing"
    node.on_image(msg)
    assert not node.objects.messages[-1].objects
    assert not node.objects.messages[-1].calibrated
    node.error = ""
    now = node.get_clock().now().nanoseconds * 1e-9
    node.stationary.add(now - 0.45, [0]*5)
    node.stationary.add(now - 0.01, [0]*5)
    msg.header.stamp = node.get_clock().now().to_msg()
    node.on_image(msg)
    assert not node.objects.messages[-1].objects


def test_task_invalid_scene_clears_observation_cache(node):
    from cap_vision.red_scene_task import Task
    from cap_scene_interfaces.msg import SceneObjects, SceneObject
    task = Task(node.cfg, "/test_measured_joints")
    try:
        msg = SceneObjects(calibrated=True)
        msg.header.frame_id = "base_link"
        obj = SceneObject(kind="cube", color="red", valid=True)
        obj.header.frame_id = "base_link"
        obj.header.stamp = task.get_clock().now().to_msg()
        msg.objects = [obj]
        task.on_objects(msg)
        assert ("cube", "red") in task.observed
        msg.calibrated = False
        task.on_objects(msg)
        assert not task.observed
    finally:
        task.destroy_node()
