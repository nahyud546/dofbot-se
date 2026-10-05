import math
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from cap_scene_interfaces.msg import ObjectState, ObjectStates

from cube_sort_3d import (CubeSort3D, grasp_candidates_for_obj, is_graspable,
                          update_confirmation, visible_camera_frame)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
import t8_motion_worker as worker


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
    node = SimpleNamespace(latest_image=annotated, latest_image_at=0.0,
                           latest_raw_image=raw, latest_raw_at=2.0)
    frame, kind = visible_camera_frame(node, 2.1)
    assert kind == "raw" and frame is raw
    node.latest_image_at = 2.0
    frame, kind = visible_camera_frame(node, 2.1)
    assert kind == "annotated" and frame is annotated
    frame, kind = visible_camera_frame(node, 4.1)
    assert kind == "waiting" and not np.any(frame)


def test_ready_pose_pixel_map_does_not_need_base_pose():
    obj = observed_cube()
    obj.base_pose_valid = False
    target = grasp_candidates_for_obj(obj, pick_x_offset_mm=15)[0]
    assert target["tcp_position_base"] == [-0.199, 0.0, 0.047]
    obj.base_pose.pose.position.x = 0.25
    assert grasp_candidates_for_obj(obj, pick_x_offset_mm=15)[0] == target


def test_fixed_pose_bridge_accepts_completed_one_second_old_observation():
    node = SimpleNamespace(
        confirmations={}, latest_states=None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    msg = ObjectStates()
    msg.header.stamp.sec = 9
    obj = observed_cube()
    obj.track_id = "cube_1"
    msg.objects.append(obj)
    CubeSort3D.on_states(node, msg)
    assert node.latest_states is msg


def test_worker_rejects_sort_when_arm_is_not_at_ready_pose():
    class Arm:
        def Arm_serial_servo_read(self, joint):
            return [110, 90, 70, 80, 120, 25][joint - 1]

    payload = {"command": "sort_cube_3d", "cube_id": 1,
               "tcp_position_base": [-0.2, 0.0, 0.047],
               "surface_id": "-Z", "pose_method": "apriltag_ippe"}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"):
        try:
            worker.execute(payload, Arm(), kin=None)
        except RuntimeError as exc:
            assert "pose" in str(exc).lower() or "khớp" in str(exc).lower()
        else:
            raise AssertionError("fixed-pose map requires READY_POSE")
        assert worker.load_state()["phase"] == "empty"
