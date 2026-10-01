import math
import time
from types import SimpleNamespace

import numpy as np

from cap_scene_interfaces.msg import ObjectState

from cube_sort_3d import is_graspable, update_confirmation, visible_camera_frame


def observed_cube(ready=True, yaw_deg=0.0, x=0.0):
    obj = ObjectState()
    obj.object_id = 1
    obj.geometry_model_id = 1
    obj.pose_method = "rgb_single_face_cube"
    obj.camera_pose_valid = True
    obj.top_grasp_ready = ready
    obj.reason = "outer face not verified" if not ready else ""
    obj.camera_pose.pose.position.x = x
    obj.camera_pose.pose.position.z = 0.2
    theta = math.radians(yaw_deg) / 2.0
    obj.camera_pose.pose.orientation.z = math.sin(theta)
    obj.camera_pose.pose.orientation.w = math.cos(theta)
    return obj


def test_top_grasp_gate_distinguishes_measured_pose_from_ready():
    sample = update_confirmation(None, observed_cube(ready=False), time.monotonic())
    assert sample is None
    ok, reason = is_graspable(observed_cube(ready=False))
    assert not ok and reason == "outer face not verified"


def test_confirmation_uses_tcp_and_square_yaw_symmetry():
    now = time.monotonic()
    first = update_confirmation(None, observed_cube(yaw_deg=0), now)
    second = update_confirmation(first, observed_cube(yaw_deg=90), now + 0.05)
    assert second["count"] == 2
    assert is_graspable(observed_cube(yaw_deg=90), second, now + 0.05)[0]
    shifted = update_confirmation(second, observed_cube(yaw_deg=90, x=0.005), now + 0.1)
    assert shifted["count"] == 1
    assert not is_graspable(observed_cube(yaw_deg=90), second, now + 1.0)[0]


def test_camera_display_falls_back_to_raw_then_wait_screen():
    annotated = np.full((480, 640, 3), 200, np.uint8)
    raw = np.full((480, 640, 3), 100, np.uint8)
    node = SimpleNamespace(latest_image=annotated, latest_image_at=1.0,
                           latest_raw_image=raw, latest_raw_at=2.0)
    frame, kind = visible_camera_frame(node, 2.1)
    assert kind == "raw" and frame is raw
    node.latest_image_at = 2.0
    frame, kind = visible_camera_frame(node, 2.1)
    assert kind == "annotated" and frame is annotated
    frame, kind = visible_camera_frame(node, 3.0)
    assert kind == "waiting" and not np.any(frame)
