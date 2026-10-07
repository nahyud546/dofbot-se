import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cap_vision.calibrate_tag_hand_eye import solve, tag_points


def test_tag_bundle_recovers_intrinsic_and_hand_eye():
    K = np.array([[610.0, 0.0, 318.0], [0.0, 605.0, 242.0], [0.0, 0.0, 1.0]])
    mount_T_camera = np.eye(4)
    mount_T_camera[:3, :3] = Rotation.from_euler("xyz", [0.08, -0.04, 0.12]).as_matrix()
    mount_T_camera[:3, 3] = [0.018, -0.006, 0.038]
    base_T_tag = np.eye(4)
    points = tag_points(0.018)
    observations = []
    rng = np.random.default_rng(4)
    for index in range(18):
        base_T_camera = np.eye(4)
        base_T_camera[:3, :3] = Rotation.from_euler(
            "xyz", [np.pi + rng.uniform(-0.38, 0.38),
                    rng.uniform(-0.35, 0.35), rng.uniform(-0.35, 0.35)]).as_matrix()
        base_T_camera[:3, 3] = [rng.uniform(-0.035, 0.035),
                                rng.uniform(-0.025, 0.025),
                                rng.uniform(0.17, 0.26)]
        camera_T_tag = np.linalg.inv(base_T_camera) @ base_T_tag
        corners = cv2.projectPoints(points, cv2.Rodrigues(camera_T_tag[:3, :3])[0],
                                    camera_T_tag[:3, 3], K, np.zeros(5))[0].reshape(4, 2)
        observations.append({
            "corners": corners,
            "base_T_mount": (base_T_camera @ np.linalg.inv(mount_T_camera)).ravel(),
            "validation": index >= 14,
        })
    result = solve(observations, (640, 480), 0.018,
                   np.array([[590.0, 0, 320.0], [0, 590.0, 240.0], [0, 0, 1.0]]))
    assert np.allclose(result["K"], K, atol=1.0)
    assert np.allclose(result["mount_T_camera"], mount_T_camera, atol=2e-3)
    assert max(result["held_out_errors_m"]) < 0.001


def test_tag_bundle_requires_holdout():
    with pytest.raises(ValueError, match="held-out"):
        solve([], (640, 480), 0.018)
