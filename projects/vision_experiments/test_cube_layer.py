import math
from pathlib import Path

import cv2
import numpy as np
import pytest

import calibrate_hand_eye as C
import cube_layer as L
import cube_search_center_math as M

TRUE_K = (921.7, 983.3, 320.0, 240.0)
TRUE_K1 = -0.467
TAG_TOP_Z = 0.0584
TAG_SIZE = 0.020
# Mount close to what the real calibration found (camera ~42 mm ahead, optical axis ~ link axis).
MOUNT = C.to_T([0.012, -0.020, 0.030], [0.0419, -0.0005, 0.0433]) @ np.eye(4)
MOUNT[:3, :3] = C.nominal_mount(0.0)[:3, :3] @ C.to_T([0.012, -0.020, 0.030], [0, 0, 0])[:3, :3]
CAL = {"arm4_T_optical": MOUNT, "K": TRUE_K, "k1": TRUE_K1, "tag_top_z": TAG_TOP_Z}
NOMINAL_PERCEPTION_K = (902.0, 875.4, 320.0, 240.0, 0.0)
TRUE_PERCEPTION_K = (*TRUE_K, TRUE_K1)


def perceive(base_xy, layer, yaw, servo, perception_K, rng, px_noise=0.4):
    """What perception would hand us: PnP of the tag corners with ITS intrinsics."""
    z = TAG_TOP_Z + L.CUBE_EDGE_M * layer
    base_T_tag = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    base_T_tag[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    base_T_tag[:3, 3] = [base_xy[0], base_xy[1], z]
    B = M.fk_arm4(servo) @ MOUNT
    pts = C.tag_points(TAG_SIZE)
    cam = (np.linalg.inv(B) @ base_T_tag @ np.c_[pts, np.ones(4)].T)[:3]
    if cam[2].min() < 0.06:
        return None
    x, y = cam[0] / cam[2], cam[1] / cam[2]
    d = 1 + TRUE_K1 * (x * x + y * y)
    uv = np.c_[TRUE_K[0] * x * d + TRUE_K[2], TRUE_K[1] * y * d + TRUE_K[3]]
    if uv.min() < 30 or uv[:, 0].max() > 610 or uv[:, 1].max() > 450:
        return None
    uv = uv + rng.normal(0, px_noise, uv.shape)
    K = np.array([[perception_K[0], 0, perception_K[2]], [0, perception_K[1], perception_K[3]], [0, 0, 1.0]])
    D = np.array([perception_K[4], 0, 0, 0, 0])
    err, camera_T_tag = C.pnp_tag(uv, K, TAG_SIZE, perception_K[4])
    cube_T_tag = np.eye(4)
    cube_T_tag[2, 3] = L.CUBE_EDGE_M / 2
    return camera_T_tag @ np.linalg.inv(cube_T_tag)


def make_case(rng, perception_K, layer):
    """Random arm pose; cube placed where a random image pixel meets its layer plane."""
    z = TAG_TOP_Z + L.CUBE_EDGE_M * layer
    for _ in range(500):
        servo = [rng.uniform(60, 120), rng.uniform(100, 132), rng.uniform(0, 25),
                 rng.uniform(0, 22), 90.0]
        B = M.fk_arm4(servo) @ MOUNT
        u, v = rng.uniform(110, 530), rng.uniform(90, 390)
        xd, yd = (u - TRUE_K[2]) / TRUE_K[0], (v - TRUE_K[3]) / TRUE_K[1]
        x, y = xd, yd
        for _ in range(8):
            d = 1 + TRUE_K1 * (x * x + y * y)
            x, y = xd / d, yd / d
        ray = B[:3, :3] @ np.array([x, y, 1.0])
        if ray[2] > -0.3:
            continue
        hit = B[:3, 3] + ray * (z - B[2, 3]) / ray[2]
        if not (0.10 <= math.hypot(hit[0], hit[1]) <= 0.27 and hit[0] < -0.05):
            continue
        try:
            T = perceive((hit[0], hit[1]), layer, rng.uniform(-0.7, 0.7), servo, perception_K, rng)
        except ValueError:
            continue
        if T is None:
            continue
        return servo, (hit[0], hit[1]), T
    raise AssertionError("no visible configuration")


def trial(rng, perception_K, layer, joint_noise=0.3):
    servo, xy, T = make_case(rng, perception_K, layer)
    noisy = [q + rng.uniform(-joint_noise, joint_noise) for q in servo]
    return xy, L.locate_cube(T, noisy, CAL, perception_K)


@pytest.mark.parametrize("perception_K,name", [
    (TRUE_PERCEPTION_K, "perception_uses_calibrated_K"),
    (NOMINAL_PERCEPTION_K, "perception_uses_nominal_K")])
def test_layers_identified_and_xy_accurate(perception_K, name):
    rng = np.random.default_rng(7)
    wrong = rejected = total = 0
    errors = []
    for layer in (0, 1, 2, 3):
        for _ in range(60):
            xy, result = trial(rng, perception_K, layer)
            total += 1
            if not result["ok"]:
                rejected += 1
                continue
            if result["layer"] != layer:
                wrong += 1
                continue
            errors.append(math.hypot(result["x"] - xy[0], result["y"] - xy[1]))
    assert wrong == 0, f"{name}: {wrong} wrong layers"
    assert rejected / total < 0.10, f"{name}: rejected {rejected}/{total}"
    assert np.percentile(errors, 95) < 0.004, np.percentile(errors, 95)


def test_tcp_height_follows_layer():
    rng = np.random.default_rng(1)
    for layer, expected in ((0, 0.047), (1, 0.077), (2, 0.107), (3, 0.137)):
        _, result = trial(rng, TRUE_PERCEPTION_K, layer)
        assert result["ok"] and result["tcp_z"] == pytest.approx(expected)


def test_wrong_depth_is_rejected_not_guessed():
    rng = np.random.default_rng(3)
    servo, xy, T = make_case(rng, TRUE_PERCEPTION_K, 0)
    T = T.copy()
    T[:3, 3] *= 1.18          # PnP says the cube is ~18% farther than it is
    result = L.locate_cube(T, servo, CAL, TRUE_PERCEPTION_K)
    assert not result["ok"] and "không khớp tầng" in result["reason"]


def test_tilted_cube_has_no_top_face():
    rng = np.random.default_rng(5)
    servo, xy, T = make_case(rng, TRUE_PERCEPTION_K, 0)
    tilt = np.eye(4)
    tilt[:3, :3] = cv2.Rodrigues(np.array([0.7, 0, 0.0]))[0]
    result = L.locate_cube(T @ tilt, servo, CAL, TRUE_PERCEPTION_K)
    assert not result["ok"] and "nghiêng" in result["reason"]


def test_joint_state_conversion_matches_mirror_convention():
    names = list(L.ARM_JOINTS) + ["grip_Joint"]
    rad = [math.radians(v - 90.0) for v in (87.5, 124.0, 3.0, 11.0, 95.0)] + [0.4]
    assert L.servo_from_joint_state(names, rad) == pytest.approx([87.5, 124.0, 3.0, 11.0, 95.0])
    assert L.servo_from_joint_state(["arm1_Joint"], [0.0]) is None


def test_joint_window_works_with_the_slow_real_mirror():
    """The mirror gives one reading per ~1.5 s; the default window must still see stillness."""
    window = L.JointWindow()
    for i in range(6):
        window.add(100.0 + 1.5 * i, [89.84, 124.5 + 0.08 * (i % 2), -0.5, 0.0, 89.8])
        if i >= 1:
            assert window.stationary_servo(100.0 + 1.5 * i + 0.3) is not None, i
    window.add(109.5, [93.0, 124.5, -0.5, 0.0, 89.8])          # moved 3 degrees between readings
    assert window.stationary_servo(109.8) is None


def test_joint_window_requires_stillness():
    window = L.JointWindow(window_s=0.5, tol_deg=0.6)
    assert window.stationary_servo(0.0) is None
    for i in range(8):
        window.add(10.0 + 0.1 * i, [90, 125, 0, 0, 90.0 + 0.1 * (i % 2)])
    assert window.stationary_servo(10.7) == pytest.approx([90, 125, 0, 0, 90.05], abs=0.1)
    window.add(10.8, [95, 125, 0, 0, 90])          # arm starts moving
    assert window.stationary_servo(10.8) is None
    assert window.stationary_servo(40.0) is None   # stale versus the image


def test_four_high_tower_is_reachable_through_preflight():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
    from cube_3d_preflight import check_target
    from t8_motion_worker import Kinematics
    kin = Kinematics()
    for z in (0.107, 0.137, 0.167, 0.182):       # lift/pick/place/hover up to the 4th cube
        for x, y in ((-0.18, 0.0), (-0.22, 0.06), (-0.20, -0.08)):
            check_target({"stage": "place", "tcp_position_base": [x, y, z],
                          "preferred_yaw_rad": 0.1}, kin)


def test_yaw_is_the_same_from_any_observation_pose():
    """T8 turns J5 = J1 - yaw: yaw must describe the cube, not the camera's roll."""
    rng = np.random.default_rng(3)
    poses = ([90, 125, 0, 0, 90], [78, 118, 6, 5, 90], [102, 118, 6, 5, 90], [90, 105, 15, 10, 90])
    checked = 0
    for x in np.arange(-0.26, -0.20, 0.01):
        shots = [L for L in poses if perceive((x, 0.0), 0, 0.3, L, TRUE_PERCEPTION_K, rng, 0) is not None]
        if len(shots) < len(poses):
            continue
        for heading in (-0.5, 0.0, 0.4):
            yaws = [L.locate_cube(perceive((x, 0.0), 0, heading, s, TRUE_PERCEPTION_K, rng, 0),
                                  s, CAL, TRUE_PERCEPTION_K)["yaw_deg"] for s in poses]
            assert max(yaws) - min(yaws) < 0.5, yaws
            checked += 1
    assert checked >= 3


def test_handeye_place_correction_is_separate_and_defaults_to_zero():
    import math
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
    from cube_3d_preflight import check_target
    from t8_motion_worker import Kinematics
    kin = Kinematics()
    place = {"stage": "place", "tcp_position_base": [-0.2, 0.04, 0.077], "preferred_yaw_rad": 0.0}
    fixed = check_target({**place, "coordinate_source": "fixed_ready_pose"}, kin)
    assert math.hypot(*fixed["placement_offset_base_xy"]) == pytest.approx(0.012, abs=1e-6)
    default = check_target({**place, "coordinate_source": "handeye_tag_plane"}, kin)
    assert default["placement_offset_base_xy"] == [0.0, 0.0]
    tuned = check_target({**place, "coordinate_source": "handeye_tag_plane"}, kin,
                         handeye_correction_gripper_xy_m=(0.006, 0.0))
    assert math.hypot(*tuned["placement_offset_base_xy"]) == pytest.approx(0.006, abs=1e-6)
    assert tuned["tcp_position_base"][0] > -0.2           # +x pulls toward the base


def test_validate_hand_eye_recovers_a_fixed_cube_from_many_poses():
    import validate_hand_eye as V
    M.apply_calibration(CAL)
    try:
        _check_validate_hand_eye(V)
    finally:
        M.apply_calibration(None)        # do not leak the calibrated model into other tests


def _check_validate_hand_eye(V):
    cube = np.array([-0.2, 0.03, TAG_TOP_Z])
    K = np.array([[TRUE_K[0], 0, 320.0], [0, TRUE_K[1], 240.0], [0, 0, 1.0]])
    pool = C.pose_pool(cube, MOUNT, K)
    assert len(pool) > 50
    poses = V.choose_poses(pool, 12, np.random.default_rng(1))
    assert any(V.inside_envelope(p) for p in poses) and any(not V.inside_envelope(p) for p in poses)
    rows = []
    for servo in poses:
        cam = (np.linalg.inv(M.fk_arm4(servo) @ MOUNT) @ np.append(cube, 1.0))[:3]
        x, y = cam[0] / cam[2], cam[1] / cam[2]
        d = 1 + TRUE_K1 * (x * x + y * y)
        centre = np.array([[TRUE_K[0] * x * d + 320.0, TRUE_K[1] * y * d + 240.0]])
        rows.append((servo, V.tag_xy(centre, servo, TAG_TOP_Z)))
    report = V.summarise(rows)
    assert report["outer_max_mm"] < 1.0 and report["inner_max_mm"] < 1.0   # noise-free: exact
    assert report["centre"] == pytest.approx([-0.2, 0.03], abs=1e-3)
    assert V.summarise(rows + [(poses[0], (-0.2, 0.05))])["errors_mm"][-1] > 15   # a 20 mm miss shows


def test_observation_outside_the_calibrated_j1_range_is_refused():
    servo = [90.0, 125.0, 0.0, 0.0, 90.0]
    rng = np.random.default_rng(5)
    s, xy, T = make_case(rng, TRUE_PERCEPTION_K, 0)
    cal = dict(CAL, j1_valid_range=(40.0, 135.0))
    ok = L.locate_cube(T, s, cal, TRUE_PERCEPTION_K)
    assert ok["ok"] == (40 <= s[0] <= 135)
    far = list(s)
    far[0] = 150.0
    refused = L.locate_cube(T, far, cal, TRUE_PERCEPTION_K)
    assert not refused["ok"] and "ngoài vùng đã hiệu chuẩn" in refused["reason"]
    assert L.locate_cube(T, far, CAL, TRUE_PERCEPTION_K)["reason"] != refused["reason"]   # no range -> no gate


def test_load_calibration_trims_the_validated_j1_range(tmp_path):
    import json
    base = json.loads(Path(M.CALIB_FILE).read_text())
    base.update(accepted=True, n_groups=3, envelope={"j1": [35.0, 140.0]})
    f = tmp_path / "he.json"
    f.write_text(json.dumps(base))
    assert M.load_calibration(f)["j1_valid_range"] == (40.0, 135.0)
    base["n_groups"] = 1                                   # single placement: no side data, no range gate
    f.write_text(json.dumps(base))
    assert M.load_calibration(f)["j1_valid_range"] is None
