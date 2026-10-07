from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cap_vision.cube_face_pose import (CubeFacePoseEstimator, FaceObservation,
                                       extract_face_quads, extract_scene_face_quads,
                                       enclosing_white_face_quad, face_corners, face_seed_instances,
                                       load_face_models, semantic_patch_corners)
from cap_vision.object_pipeline import (ObjectInstance, PoseEstimate, fuse_poses,
                                        is_full_6d_pose)


ROOT = Path(__file__).resolve().parents[4]          # <repo>/ros/src/cap_vision/test
GEOMETRY = ROOT / "config/robot/cube_4x6_face_geometry.yaml"
K = np.array([[900., 0., 320.], [0., 875., 240.], [0., 0., 1.]])
D = np.zeros(5)


def test_face_geometry_loads_four_cubes_with_distinct_labels():
    models = load_face_models(GEOMETRY)
    assert set(models) == {1, 2, 3, 4}
    assert models[1].semantic_faces["+X"] == "book"
    assert models[2].semantic_faces["-Y"] == "apple_core"
    assert models[3].semantic_faces["+Y"] == "expired_cosmetics"
    assert models[4].semantic_faces["-Z"] == "khoi_vang"
    assert all(np.isclose(m.tag_size_m, 0.020) for m in models.values())


def test_two_adjacent_faces_recover_metric_pose():
    model = load_face_models(GEOMETRY)[1]
    rvec = np.array([-0.7, 0.7, 0.1])
    tvec = np.array([0.01, 0.005, 0.18])
    observations = [FaceObservation(
        axis, cv2.projectPoints(semantic_patch_corners(model, axis), rvec, tvec, K, D)[0].reshape(4, 2),
        0.92) for axis in ("+X", "+Y")]
    pose = CubeFacePoseEstimator().estimate(observations, model, K, D)
    assert pose is not None and pose.method == "rgb_faces_pnp"
    assert pose.reprojection_error_px < 0.01
    assert np.linalg.norm(pose.camera_T_object[:3, 3] - tvec) < 1e-4
    assert np.linalg.norm(pose.camera_T_object[:3, :3] - cv2.Rodrigues(rvec)[0]) < 1e-3


def test_single_face_and_opposite_faces_do_not_claim_full_pose():
    model = load_face_models(GEOMETRY)[2]
    quad = np.array([[100, 100], [200, 100], [200, 200], [100, 200]])
    estimator = CubeFacePoseEstimator()
    assert estimator.estimate([FaceObservation("+X", quad, 0.9)], model, K, D) is None
    assert estimator.estimate([FaceObservation("+X", quad, 0.9),
                               FaceObservation("-X", quad, 0.9)], model, K, D) is None


def test_single_face_recovers_cube_wireframe_but_not_semantic_6d():
    model = load_face_models(GEOMETRY)[1]
    rvec = np.array([-0.7, 0.7, 0.1])
    tvec = np.array([0.01, 0.005, 0.18])
    quad = cv2.projectPoints(semantic_patch_corners(model, "+X"), rvec, tvec, K, D)[0]
    pose = CubeFacePoseEstimator().estimate_single_face(
        FaceObservation("+X", quad.reshape(4, 2), 0.8), model, K, D)
    assert pose is not None
    assert pose.method == "rgb_single_face_cube"
    assert not pose.orientation_valid
    assert not is_full_6d_pose(pose.method, pose.orientation_valid)
    assert pose.reprojection_error_px < 0.01


def test_physical_face_quads_can_be_found_inside_cube_mask():
    model = load_face_models(GEOMETRY)[1]
    rvec = np.array([-0.7, 0.7, 0.1])
    tvec = np.array([0.01, 0.005, 0.18])
    image = np.zeros((480, 640, 3), np.uint8)
    mask = np.zeros((480, 640), np.uint8)
    for axis, color in (("-Z", (80, 80, 240)), ("+X", (180, 120, 20)),
                        ("+Y", (20, 180, 80))):
        quad = cv2.projectPoints(face_corners(model, axis), rvec, tvec, K, D)[0]
        quad = quad.reshape(4, 2).astype(np.int32)
        cv2.fillConvexPoly(mask, quad, 1)
        cv2.fillConvexPoly(image, quad, color)
        cv2.polylines(image, [quad], True, (240, 240, 240), 2)
    ys, xs = np.nonzero(mask)
    instance = ObjectInstance((int(xs.min()), int(ys.min()), int(xs.max()) + 1,
                               int(ys.max()) + 1), mask.astype(bool), 0.9)
    assert len(extract_face_quads(image, instance)) >= 2


def test_semantic_face_can_seed_instance_without_yolo_mask():
    image = np.full((480, 640, 3), 235, np.uint8)
    quad = np.array([[260, 170], [390, 185], [375, 305], [245, 285]], np.int32)
    cv2.fillConvexPoly(image, quad, (220, 80, 20))
    cv2.polylines(image, [quad], True, (20, 20, 20), 4)
    candidates = extract_scene_face_quads(image)
    assert candidates
    instances = face_seed_instances(image.shape, [(1, candidates[0], 0.9)])
    assert len(instances) == 1
    cx, cy = quad.mean(axis=0).astype(int)
    assert instances[0].mask[cy, cx]
    assert instances[0].bbox[0] < quad[:, 0].min()
    assert instances[0].bbox[2] > quad[:, 0].max()


def test_trash_core_requires_closed_white_outer_face():
    image = np.full((240, 320, 3), (50, 80, 110), np.uint8)
    outer = np.array([[100, 60], [190, 60], [190, 150], [100, 150]], np.float32)
    core = np.array([[115, 75], [175, 75], [175, 135], [115, 135]], np.float32)
    cv2.fillConvexPoly(image, outer.astype(np.int32), (240, 240, 240))
    cv2.fillConvexPoly(image, core.astype(np.int32), (15, 30, 80))
    result = enclosing_white_face_quad(image, core, [core, outer], (145, 105))
    assert result is not None and np.allclose(result, outer)
    small_core = np.array([[125, 85], [165, 85], [165, 125], [125, 125]], np.float32)
    assert enclosing_white_face_quad(image, small_core, [small_core, outer],
                                     (145, 105)) is not None
    assert enclosing_white_face_quad(image, core, [core], (145, 105)) is None
    cv2.fillConvexPoly(image, outer.astype(np.int32), (20, 30, 80))
    assert enclosing_white_face_quad(image, core, [core, outer], (145, 105)) is None


def test_position_only_rgb_never_rejects_tag_rotation():
    T = np.eye(4)
    T[:3, 3] = [0.01, 0.0, 0.2]
    tag = PoseEstimate(T, 0.9, 0.2, "apriltag_ippe")
    position_only = PoseEstimate(np.eye(4), 0.4, 4.0, "rgb_geometry",
                                 orientation_valid=False)
    fused, status = fuse_poses(tag, position_only)
    assert fused is tag and status == "tag_only"
    assert is_full_6d_pose("rgb_faces_pnp", True)
    assert not is_full_6d_pose("rgb_faces_provisional", False)
