import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "cap_grasp"))

from cap_vision.object_pipeline import (AprilTagPoseEstimator, CubeModel, Evidence,
                                        IdentityFusion, ObjectInstance, ObjectTracker,
                                        tag_in_instance, tag_seed_instances)
# cap_grasp (lập kế hoạch gắp bằng MoveIt) nằm ở kho dofbot_robot_arm_6dof, không thuộc repo này.
_grasps = pytest.importorskip("cap_grasp.object_grasps", reason="cần package cap_grasp (kho dofbot_robot_arm_6dof)")
cube_candidates, geometric_top = _grasps.cube_candidates, _grasps.geometric_top


def test_tag_pose_recovers_cube_translation_and_reprojects():
    cube_T_tag = np.eye(4)
    cube_T_tag[2, 3] = 0.015
    model = CubeModel.measured(2, 0.030, 0.018, cube_T_tag)
    K = np.array([[900, 0, 320], [0, 880, 240], [0, 0, 1.0]], float)
    half = model.tag_size_m / 2
    points = np.array([[-half, -half, 0], [half, -half, 0],
                       [half, half, 0], [-half, half, 0]], float)
    rvec = np.array([0.12, -0.2, 0.3])
    tvec = np.array([0.02, -0.01, 0.20])
    corners = cv2.projectPoints(points, rvec, tvec, K, np.zeros(5))[0].reshape(4, 2)
    pose = AprilTagPoseEstimator().estimate_tag(corners, model, K, np.zeros(5))
    assert pose is not None
    assert pose.reprojection_error_px < 0.01
    expected = tvec - cv2.Rodrigues(rvec)[0] @ cube_T_tag[:3, 3]
    assert np.linalg.norm(pose.camera_T_object[:3, 3] - expected) < 1e-3
    assert np.linalg.norm(pose.camera_T_object[:3, :3] -
                          cv2.Rodrigues(rvec)[0]) < 1e-2


def test_tag_seed_fallback_projects_one_instance_mask():
    cube_T_tag = np.eye(4)
    cube_T_tag[2, 3] = 0.015
    model = CubeModel.measured(1, 0.030, 0.018, cube_T_tag)
    K = np.array([[900, 0, 320], [0, 880, 240], [0, 0, 1.0]], float)
    half = model.tag_size_m / 2
    points = np.array([[-half, -half, 0], [half, -half, 0],
                       [half, half, 0], [-half, half, 0]], float)
    corners = cv2.projectPoints(points, np.array([0.1, -0.1, 0.2]),
                                np.array([0.01, 0.0, 0.20]), K,
                                np.zeros(5))[0].reshape(4, 2)
    tag = SimpleNamespace(tag_id=1, corners=corners, decision_margin=70.0)
    instances = tag_seed_instances([tag], {1: model}, AprilTagPoseEstimator(),
                                   K, np.zeros(5), (480, 640))
    assert len(instances) == 1
    assert instances[0].mask.dtype == bool
    assert np.count_nonzero(instances[0].mask) > 1000
    assert tag_in_instance(corners, instances[0])


def test_identity_conflict_and_instance_tracking_are_separate():
    fusion = IdentityFusion()
    cube_id, probabilities = fusion.update([
        Evidence("tag", 2, 0.98), Evidence("color", 1, 0.9)])
    assert cube_id == 2 and probabilities[1] > probabilities[0]
    mask_a = np.zeros((40, 40), bool)
    mask_a[5:15, 5:15] = True
    mask_b = np.zeros((40, 40), bool)
    mask_b[20:30, 20:30] = True
    tracker = ObjectTracker()
    first = tracker.update([ObjectInstance((5, 5, 15, 15), mask_a, 0.9),
                            ObjectInstance((20, 20, 30, 30), mask_b, 0.8)], 1.0)
    ids = {track.track_id for track in first}
    second = tracker.update([ObjectInstance((20, 20, 30, 30), mask_b, 0.9),
                             ObjectInstance((5, 5, 15, 15), mask_a, 0.8)], 1.1)
    assert {track.track_id for track in second} == ids
    assert not tag_in_instance(np.array([[21, 21], [22, 21], [22, 22], [21, 22]]),
                               next(track.instance for track in second
                                    if track.instance.bbox[0] == 5))


def test_fallback_streak_survives_normal_cpu_inference_gap():
    mask = np.zeros((40, 40), bool)
    mask[5:25, 5:25] = True
    tracker = ObjectTracker()
    first = tracker.update([ObjectInstance((5, 5, 25, 25), mask, 0.8,
                                           proposal_source="hsv_face")], 1.0)[0]
    second = tracker.update([ObjectInstance((5, 5, 25, 25), mask, 0.8,
                                            proposal_source="yoloe")], 1.55)[0]
    third = tracker.update([ObjectInstance((5, 5, 25, 25), mask, 0.8,
                                           proposal_source="hsv_face")], 2.1)[0]
    assert first.track_id == second.track_id == third.track_id
    assert third.fallback_streak == 3


def test_track_identity_survives_slow_inference_without_matching_new_cube():
    mask = np.zeros((40, 80), bool)
    mask[5:25, 5:25] = True
    first_box = ObjectInstance((5, 5, 25, 25), mask, 0.8,
                               proposal_source="hsv_face")
    tracker = ObjectTracker(max_gap=3.0)
    first = tracker.update([first_box], 1.0)[0]
    same = tracker.update([ObjectInstance((5, 5, 25, 25), mask, 0.8,
                                          proposal_source="hsv_face")], 3.5)[0]
    assert same.track_id == first.track_id
    assert same.fallback_streak == 2
    moved_mask = np.zeros((40, 80), bool)
    moved_mask[5:25, 50:70] = True
    different = tracker.update([ObjectInstance((50, 5, 70, 25), moved_mask, 0.8,
                                               proposal_source="hsv_face")], 3.6)[0]
    assert different.track_id != first.track_id


def test_cube_surfaces_generate_3d_candidates():
    model = CubeModel.measured(1, 0.030, 0.018, np.eye(4))
    base_T_cube = np.eye(4)
    base_T_cube[:3, 3] = [0.2, 0.0, 0.06]
    grasps = cube_candidates("object_001", 1, model, base_T_cube, 0.05, 0.04)
    assert geometric_top(model, base_T_cube) == "+Z"
    assert any(g.surface_id == "+Z" and np.isclose(g.position_base[2], 0.075)
               for g in grasps)
    assert not cube_candidates("object_001", 1, model, base_T_cube, 0.02, 0.04)


def test_pose_stability_and_lock_keep_base_target_fixed():
    mask = np.zeros((20, 20), bool)
    mask[3:12, 4:13] = True
    tracker = ObjectTracker(stable_frames=2)
    track = tracker.update([ObjectInstance((4, 3, 13, 12), mask, 0.9)], 1.0)[0]
    camera_T_cube = np.eye(4)
    base_T_cube = np.eye(4)
    base_T_cube[:3, 3] = [0.2, 0.0, 0.06]
    from cap_vision.object_pipeline import PoseEstimate
    pose = PoseEstimate(camera_T_cube, 0.9, 0.2, "apriltag_ippe")
    tracker.observe_pose(track, pose, base_T_cube, 1.0)
    tracker.update([ObjectInstance((4, 3, 13, 12), mask, 0.9)], 1.1)
    tracker.observe_pose(track, pose, base_T_cube, 1.1)
    assert track.state == "STABLE"
    track.state = "LOCKED_FOR_GRASP"
    moved = base_T_cube.copy()
    moved[0, 3] += 0.05
    tracker.observe_pose(track, pose, moved, 1.2)
    assert np.allclose(track.base_T_object, base_T_cube)
