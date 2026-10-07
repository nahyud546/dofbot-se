"""Cấu hình pytest chung: test cần ROS thì bỏ qua rõ ràng khi chưa source ROS + workspace ros/."""
import importlib.util

import pytest

HAS_ROS = importlib.util.find_spec("rclpy") is not None
HAS_PERCEPTION_MSGS = importlib.util.find_spec("cap_scene_interfaces") is not None
NEEDS_ROS = ("test_ros_approval.py",)


def pytest_ignore_collect(collection_path, config):
    # Test của cap_vision import message ROS ngay khi nạp module.
    if "ros/src" in str(collection_path) and not (HAS_ROS and HAS_PERCEPTION_MSGS):
        return True
    return None


def pytest_collection_modifyitems(config, items):
    if HAS_ROS:
        return
    skip = pytest.mark.skip(reason="cần ROS: source /opt/ros/humble/setup.bash && source ros/install/setup.bash")
    for item in items:
        if item.fspath.basename in NEEDS_ROS:
            item.add_marker(skip)
