from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from cap_vision.red_scene_core import (load_config, project_to_plane, detect_scene,
    StableObservations, Observation, StationaryWindow, calibration_error)
from cap_vision.red_task_core import targets, append_trajectory, JOINTS, validate_task, placement_error
from cap_vision.calibrate_intrinsic import save_yaml


@pytest.fixture
def cfg():
    c = load_config(Path(__file__).parents[1] / "config/red_scene.yaml")
    c["camera"].update(K=[600., 0, 320., 0, 600., 240., 0, 0, 1.],
        calibrated=True, extrinsic_calibrated=True, mount_T_optical=np.eye(4).ravel().tolist())
    c["table"].update(calibrated=True, base_T_table=np.eye(4).ravel().tolist())
    return c


@pytest.fixture
def camera():
    t = np.diag([1., -1., -1., 1.])
    t[2, 3] = 0.4
    return t


def scene_image(cube=True, zone=True):
    image = np.full((480, 640, 3), 240, np.uint8)
    if cube:
        cv2.rectangle(image, (135, 175), (183, 223), (0, 0, 210), -1)
    if zone:
        cv2.rectangle(image, (355, 165), (490, 300), (0, 0, 210), -1)
        cv2.rectangle(image, (405, 205), (435, 250), (255, 255, 255), -1)
    return image


def test_cube_zone_both_kept_and_white_logo_does_not_shift_center(cfg, camera):
    found, _ = detect_scene(scene_image(), cfg, camera)
    assert {o.kind for o in found} == {"cube", "zone"}
    zone = next(o for o in found if o.kind == "zone")
    expected = project_to_plane([[422.5, 232.5]], cfg["camera"]["K"], [0]*5, camera, np.eye(4))[0]
    assert np.linalg.norm(zone.center - expected) < 0.001
    assert next(o for o in found if o.kind == "cube").center[2] == pytest.approx(0.015)


def test_absent_and_ambiguous_objects(cfg, camera):
    assert not detect_scene(np.zeros((480, 640, 3), np.uint8), cfg, camera)[0]
    assert [o.kind for o in detect_scene(scene_image(cube=False), cfg, camera)[0]] == ["zone"]
    image = scene_image(zone=False)
    cv2.rectangle(image, (250, 170), (298, 218), (0, 0, 210), -1)
    assert not detect_scene(image, cfg, camera)[0]


def test_clipped_and_merged_shape_rejected(cfg, camera):
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (0, 170), (48, 218), (0, 0, 200), -1)
    assert not detect_scene(image, cfg, camera)[0]
    image = scene_image()
    cv2.rectangle(image, (180, 195), (360, 203), (0, 0, 210), -1)
    assert not detect_scene(image, cfg, camera)[0]


def test_distortion_projection_and_moving_camera(cfg, camera):
    point = np.array([[0.06, 0.03, 0.03]])
    dist = np.array([0.1, -0.04, 0.001, 0.001, 0.0])
    for dx in (0, 0.04):
        cam = camera.copy()
        cam[0, 3] = dx
        inv = np.linalg.inv(cam)
        uv = cv2.projectPoints(point, cv2.Rodrigues(inv[:3, :3])[0], inv[:3, 3],
                               np.array(cfg["camera"]["K"]).reshape(3, 3), dist)[0]
        actual = project_to_plane(uv, cfg["camera"]["K"], dist, cam, np.eye(4), 0.03)
        assert np.allclose(actual, point, atol=1e-6)


def test_bad_ray_and_missing_calibration(cfg):
    assert calibration_error(cfg) == ""
    cfg["camera"]["calibrated"] = False
    assert calibration_error(cfg)
    with pytest.raises(ValueError):
        project_to_plane([[320, 240]], cfg["camera"]["K"], [0]*5, np.eye(4), np.eye(4), -0.1)


def test_stability_requires_distinct_consecutive_images():
    gate = StableObservations(3, 0.005, 0.5)
    obj = Observation("cube", np.zeros(3), np.ones(3), np.zeros((4, 3)), 0., 1.)
    assert not gate.update([obj], 1.)
    assert not gate.update([obj], 1.)
    assert not gate.update([obj], 1.1)
    assert gate.update([obj], 1.2)
    assert not gate.update([], 1.3)
    assert not gate.update([obj], 1.4)
    assert not gate.update([obj], 2.)


def test_stationary_window_rejects_motion_and_stale_joints():
    window = StationaryWindow(0.3, 0.005)
    window.add(1, [0]*5)
    window.add(1.4, [0]*5)
    assert window.ready(1.45, 0.25)
    assert not window.ready(2., 0.25)
    window.add(1.5, [0.1]*5)
    assert not window.ready(1.55, 0.25)


def test_failed_intrinsic_never_marked_calibrated(tmp_path):
    path = tmp_path / "bad.yaml"
    assert save_yaml(path, np.diag([600., 600., 1.]), np.zeros(5), 0.8, (640, 480)) == 2
    assert not load_config(path)["calibrated"]
    assert save_yaml(path, np.diag([600., 600., 1.]), np.zeros(5), 0.2, (640, 480)) == 0


def test_task_blocks_bootstrap_and_targets_follow_zone(cfg):
    with pytest.raises(ValueError, match="TCP"):
        validate_task(cfg)
    cfg["task"]["tcp_contact_offset_m"] = 0.04
    a = targets([0.1, 0, 0.015], [0.1, -0.15, 0], cfg)
    b = targets([0.1, 0, 0.015], [0.12, -0.15, 0], cfg)
    assert b["place"][0] - a["place"][0] == pytest.approx(0.02)
    assert placement_error([0.1, -0.15, 0.015], [0.1, -0.15, 0], cfg) == 0


def test_export_preserves_intermediate_points_and_gripper():
    def point(t, q):
        return SimpleNamespace(time_from_start=SimpleNamespace(sec=t, nanosec=0), positions=q)
    traj = SimpleNamespace(joint_names=JOINTS[:1], points=[point(0, [0.]), point(1, [0.1]), point(2, [0.2])])
    state = dict.fromkeys(JOINTS, 0.)
    state[JOINTS[-1]] = 0.7
    rows = []
    assert append_trajectory(rows, traj, state, 0.01) == 2.01
    assert len(rows) == 3 and rows[1]["positions"][0] == 0.1
    assert all(p["positions"][-1] == 0.7 for p in rows)
    traj.points[0].positions = [0.4]
    with pytest.raises(ValueError, match="discontinuous"):
        append_trajectory(rows, traj, state, 2.01)
