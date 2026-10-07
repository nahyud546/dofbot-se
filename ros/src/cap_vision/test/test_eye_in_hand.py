import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cap_vision import calibrate_eye_in_hand as module


def test_hand_eye_recovers_known_transform_and_validates_holdout(monkeypatch):
    pattern, square = (9, 6), 0.015
    K = np.array([[600., 0, 320.], [0, 600., 240.], [0, 0, 1.]])
    camera = dict(K=K.ravel().tolist(), distortion=[0.]*5, width=640, height=480)
    mount_T_camera = np.eye(4)
    mount_T_camera[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.05, 0.2]).as_matrix()
    mount_T_camera[:3, 3] = [0.02, 0.01, 0.04]
    board = np.eye(4)
    obj = np.zeros((pattern[0]*pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)*square
    poses, corners = [], []
    rng = np.random.default_rng(10)
    for i in range(12):
        base_T_camera = np.eye(4)
        base_T_camera[:3, :3] = Rotation.from_euler("xyz", [np.pi + rng.uniform(-0.3, 0.3),
                                                   rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)]).as_matrix()
        base_T_camera[:3, 3] = [0.06+rng.uniform(-0.02, 0.02), 0.04, 0.4+rng.uniform(-0.03, 0.03)]
        base_T_mount = base_T_camera @ np.linalg.inv(mount_T_camera)
        cam_T_board = np.linalg.inv(base_T_camera) @ board
        pixels = cv2.projectPoints(obj, cv2.Rodrigues(cam_T_board[:3, :3])[0],
                                   cam_T_board[:3, 3], K, np.zeros(5))[0]
        corners.append(pixels.astype(np.float32))
        poses.append(dict(image=str(i), base_T_mount=base_T_mount.ravel().tolist(), validation=i >= 9))
    iterator = iter(corners)
    monkeypatch.setattr(cv2, "imread", lambda _: np.zeros((480, 640, 3), np.uint8))
    monkeypatch.setattr(module, "find_corners", lambda *_: next(iterator))
    recovered, table, errors = module.calibrate(poses, camera, pattern, square)
    assert np.allclose(recovered, mount_T_camera, atol=1e-4)
    assert max(errors) < 0.0001
    assert table[2, 2] > 0.99


def test_requires_held_out_poses(monkeypatch):
    with pytest.raises(ValueError, match="training"):
        module.calibrate([], dict(K=np.eye(3).ravel(), distortion=[0]*5), (9, 6), 0.015)
