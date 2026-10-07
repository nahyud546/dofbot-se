"""Eye-in-hand stability: cube static, camera moving -> T_base_cube fixed.

Case D validation without hardware: same base_T_cube observed from two
different base_T_camera (synchronized TF(t)) must keep one track_id and
STABLE state, even when image IoU ~ 0.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cap_vision.object_pipeline import (ObjectInstance, ObjectTracker,
                                        PoseEstimate, is_full_6d_pose,
                                        is_position_only_pose)


def make_mask(shape, box):
    mask = np.zeros(shape, bool)
    x1, y1, x2, y2 = box
    mask[y1:y2, x1:x2] = True
    return mask


def test_static_cube_keeps_track_and_base_pose_despite_camera_motion():
    tracker = ObjectTracker(world_weight=0.6, world_sigma_m=0.05, stable_frames=3)
    # Cube đứng yên trong base frame.
    base_T_cube = np.eye(4)
    base_T_cube[:3, 3] = [0.20, 0.0, 0.06]
    # Camera di chuyển mạnh -> bbox dịch xa, IoU ~ 0.
    inst1 = ObjectInstance((50, 200, 150, 300), make_mask((480, 640), (50, 200, 150, 300)), 0.9)
    inst2 = ObjectInstance((400, 180, 500, 280), make_mask((480, 640), (400, 180, 500, 280)), 0.9)
    t1 = tracker.update([inst1], 1.0)[0]
    pose = PoseEstimate(np.eye(4), 0.9, 0.5, "apriltag_ippe")
    tracker.observe_pose(t1, pose, base_T_cube, 1.0)
    # Provisional world từ RGB geometry + TF(t) đúng stamp -> cùng điểm base.
    world_hint = base_T_cube[:3, 3].copy()
    t2 = tracker.update([inst2], 1.1, [world_hint])[0]
    assert t2.track_id == t1.track_id, "camera moves but static cube must keep track_id"
    tracker.observe_pose(t2, pose, base_T_cube, 1.1)
    tracker.observe_pose(t2, pose, base_T_cube, 1.2)
    assert t2.state == "STABLE"
    assert np.allclose(t2.base_T_object[:3, 3], [0.20, 0.0, 0.06], atol=1e-9)


def test_wrong_timestamp_tf_breaks_stability():
    tracker = ObjectTracker(stable_frames=3)
    inst = ObjectInstance((50, 200, 150, 300), make_mask((480, 640), (50, 200, 150, 300)), 0.9)
    track = tracker.update([inst], 1.0)[0]
    good = np.eye(4)
    good[:3, 3] = [0.20, 0.0, 0.06]
    wrong = np.eye(4)
    wrong[:3, 3] = [0.26, 0.03, 0.06]  # dùng TF(t2) cho ảnh t1
    pose = PoseEstimate(np.eye(4), 0.9, 0.5, "apriltag_ippe")
    tracker.observe_pose(track, pose, good, 1.0)
    tracker.observe_pose(track, pose, good, 1.1)
    assert track.stable_count == 2
    tracker.observe_pose(track, pose, wrong, 1.2)
    assert track.stable_count == 1, "sai TF timestamp phải reset stability"
    assert track.state != "STABLE"


def test_position_only_is_not_full_6d():
    assert is_full_6d_pose("apriltag_ippe", True)
    assert is_full_6d_pose("apriltag_rgb_fused", True)
    assert not is_full_6d_pose("rgb_geometry", False)
    assert not is_full_6d_pose("rgb_geometry", True)
    assert is_position_only_pose("rgb_geometry", False)
    assert not is_position_only_pose("apriltag_ippe", True)
