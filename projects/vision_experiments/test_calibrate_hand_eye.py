import json
import math

import numpy as np
import pytest

import calibrate_hand_eye as C
import cube_search_center_math as M

TAG_Z = 0.0625


def make_world(seed, pitch, f_true, readback_step=0.082, backlash=0.3, ratio=1.0, k1=0.0, j1_scale=1.0):
    """Simulated arm + camera whose true mount differs from the nominal model."""
    rng = np.random.default_rng(seed)
    X = C.nominal_mount(pitch) @ C.to_T([0.05, 0.03, 0.04], [0.004, -0.003, 0.006])
    K = C.nominal_K()
    K[0, 0] *= f_true
    K[1, 1] *= f_true * ratio
    # Put the cube where the READY view really looks.
    cam = M.fk_arm4(C.READY) @ X
    centre = cam[:3, 3] + cam[:3, 2] * (TAG_Z - cam[2, 3]) / cam[2, 2]
    T = C.to_T([0, 0, math.radians(25)], centre + [0.01, -0.008, 0.0])

    def see(points4, pose):
        true = [v + rng.uniform(-backlash, backlash) for v in pose]
        read = [round(v / readback_step) * readback_step for v in true]
        true[0] = 90.0 + j1_scale * (true[0] - 90.0)         # the J1 servo really turns j1_scale x the reading
        p = np.linalg.inv(M.fk_arm4_raw(true) @ X) @ points4
        d = 1.0 + k1 * ((p[0] / p[2]) ** 2 + (p[1] / p[2]) ** 2)
        uv = np.c_[K[0, 0] * d * p[0] / p[2] + K[0, 2], K[1, 1] * d * p[1] / p[2] + K[1, 2]]
        visible = p[2].min() > 0.05 and uv.min() > 10 and uv[:, 0].max() < 630 and uv[:, 1].max() < 470
        return read, (uv + rng.normal(0, 0.3, uv.shape) if visible else None)

    tag4 = T @ np.c_[C.tag_points(), np.ones(4)].T
    return rng, X, K, T, see, (lambda pose: see(tag4, pose))


def runtime_xy_errors(rng, X, K, see, result, n=200):
    """Cube anywhere in view from arbitrary poses -> XY error (m)."""
    errors = []
    for _ in range(4 * n):
        pose = [rng.uniform(30, 150), rng.uniform(110, 135), rng.uniform(0, 20),
                rng.uniform(0, 30), 90.0]
        cam = M.fk_arm4(pose) @ X
        ray = cam[:3, :3] @ [(rng.uniform(120, 520) - K[0, 2]) / K[0, 0],
                             (rng.uniform(100, 380) - K[1, 2]) / K[1, 1], 1.0]
        if ray[2] > -0.2:
            continue
        P = np.append(cam[:3, 3] + ray * (TAG_Z - cam[2, 3]) / ray[2], 1.0)
        if not 0.08 <= math.hypot(P[0], P[1]) <= 0.26:   # reachable table only
            continue
        read, uv = see(P.reshape(4, 1), pose)
        if uv is None:
            continue
        hit = C.ray_plane_xy(uv[0, 0], uv[0, 1], M.fk_arm4(read) @ result["arm4_T_optical"],
                             result["K"], result["k1"], result["tag_top_z"])
        errors.append(float(np.linalg.norm(hit[:2] - P[:2])))
    return np.array(errors)


def test_pose_pool_is_safe():
    pool = C.pose_pool(np.array([-0.176, 0.004, 0.055]), C.nominal_mount(), C.nominal_K())
    assert len(pool) > 25
    assert all(C.tip_z(p) >= C.MIN_TIP_Z for p in pool)
    assert all(0 <= v <= 180 for p in pool for v in p)


@pytest.mark.parametrize("seed,pitch,f_true,ratio,k1", [
    (0, 3.0, 0.96, 1.0, 0.0), (1, -3.0, 1.0, 1.0, 0.0), (2, 6.0, 1.05, 1.06, -0.4)])
def test_adaptive_collection_and_solve(seed, pitch, f_true, ratio, k1):
    rng, X, K, T, see, observe = make_world(seed, pitch, f_true, ratio=ratio, k1=k1)
    samples = C.collect_loop(observe, 16, log=lambda *_: None)
    assert len(samples) >= 12
    result = C.solve(samples)
    assert result["accepted"], C.report(result)
    assert max(result["held_out_xy_m"]) <= C.ACCEPT_XY_M
    assert abs(result["tag_top_z"] - TAG_Z) < 0.008
    errors = runtime_xy_errors(rng, X, K, see, result)
    assert len(errors) > 30
    assert np.median(errors) < 0.005 and np.percentile(errors, 95) < 0.007


def test_integer_readback_is_detectably_worse():
    rng, X, K, T, see, observe = make_world(3, 4.0, 0.97, readback_step=1.0, backlash=0.4)
    result = C.solve(C.collect_loop(observe, 16, log=lambda *_: None))
    assert result["fit_rms_px"] > 4.0


def test_solver_rejects_too_few_or_degenerate_samples():
    *_, observe = make_world(0, 2.0, 1.0)
    samples = C.collect_loop(observe, 16, log=lambda *_: None)
    with pytest.raises(ValueError):
        C.solve(samples[:8])
    with pytest.raises(ValueError):
        C.solve([samples[0]] * 16)


def test_missing_tag_at_ready_is_reported():
    with pytest.raises(RuntimeError):
        C.collect_loop(lambda pose: (list(pose), None), 16, log=lambda *_: None)


def test_fine_angle_uses_raw_counts_only_when_consistent():
    class Arm:
        id, servo_H, servo_L = 0x32, (2011 >> 8), (2011 & 0xFF)
    angle = M.fine_angle(Arm, 2, 89)
    assert abs(angle - (180 - 180 * (2011 - 900) / 2200)) < 1e-9 and abs(angle - 89) <= 1.01
    assert M.fine_angle(Arm, 3, 40) == 40.0          # stale reply for another joint
    assert M.fine_angle(object(), 1, 77) == 77.0     # fake arm without raw fields


def test_outputs_round_trip(tmp_path):
    import yaml
    *_, observe = make_world(0, 3.0, 0.96)
    result = C.solve(C.collect_loop(observe, 16, log=lambda *_: None))
    result["accepted_3d"] = True   # exercise the YAML writer regardless of the 3D gate
    written = C.write_outputs(result, C.TAG_SIZE_M, tmp_path / "he.json", tmp_path / "c.yaml")
    assert len(written) == 2
    data = json.loads((tmp_path / "he.json").read_text())
    assert data["accepted"] and len(data["arm4_T_optical"]) == 16
    cal = M.load_calibration(tmp_path / "he.json")
    try:
        M.apply_calibration(cal)
        tag = result["base_T_tag"][:3, 3]
        cam = np.linalg.inv(M.fk_arm4(C.READY) @ result["arm4_T_optical"]) @ np.append(tag, 1.0)
        x, y = cam[0] / cam[2], cam[1] / cam[2]
        d = 1 + result["k1"] * (x * x + y * y)
        u = result["K"][0, 0] * x * d + result["K"][0, 2]
        v = result["K"][1, 1] * y * d + result["K"][1, 2]
        P, _ = M.pixel_to_base(u, v, C.READY)
        assert np.linalg.norm(P - tag) < 1e-4
    finally:
        M.apply_calibration(None)
    cfg = yaml.safe_load((tmp_path / "c.yaml").read_text())
    camera = cfg["camera"]
    assert camera["calibrated"] and camera["extrinsic_calibrated"]
    assert camera["source"] == "fixed_apriltag_eye_in_hand"
    held = cfg["calibration_validation"]["held_out_errors_m"]
    assert len(held) >= 3
    mount = np.array(camera["mount_T_optical"]).reshape(4, 4)
    assert np.allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-6)


def test_intrinsics_only_yaml_when_3d_gate_fails(tmp_path):
    import yaml
    *_, observe = make_world(0, 3.0, 0.96)
    result = C.solve(C.collect_loop(observe, 16, log=lambda *_: None))
    result["accepted"], result["accepted_3d"] = True, False
    written = C.write_outputs(result, C.TAG_SIZE_M, tmp_path / "he.json", tmp_path / "c.yaml")
    assert len(written) == 2
    camera = yaml.safe_load((tmp_path / "c.yaml").read_text())["camera"]
    assert camera["source"] == "fixed_apriltag_eye_in_hand_intrinsics_only"
    assert not camera["calibrated"] and not camera["extrinsic_calibrated"]
    assert camera["K"][0] == pytest.approx(result["K"][0, 0]) and camera["distortion"][0] == pytest.approx(result["k1"])
    result["accepted"] = False
    C.write_outputs(result, C.TAG_SIZE_M, tmp_path / "he.json", tmp_path / "c.yaml")
    assert not (tmp_path / "c.yaml").exists()


def test_two_cube_placements_extend_the_calibrated_envelope():
    """Cube in the middle only -> extrapolation error at the sides; add a side placement -> fixed."""
    import pose_library as PL
    rng, X, K, T, see, observe_mid = make_world(11, 4.0, 0.97, ratio=1.04, k1=-0.45, backlash=0.4)
    mid = C.collect_loop(observe_mid, 16, log=lambda *_: None)
    # A second cube spot like the real one: x -0.126, y -0.108, seen from the LEFT pose.
    side_T = T.copy()
    side_T[:3, 3] += [0.05, -0.108, 0.0]
    side_tag = side_T @ np.c_[C.tag_points(), np.ones(4)].T
    observe_side = lambda pose: see(side_tag, pose)
    side = C.collect_loop(observe_side, 14, log=lambda *_: None, start_pose=PL.POSES["LEFT"],
                          j1_values=range(20, 161, 5), group=1)
    assert len(side) >= 8, "no side samples collected"
    assert min(s["servo"][0] for s in side) > 100 or max(s["servo"][0] for s in side) < 80
    both = mid + side
    result = C.solve(both)
    assert result["n_groups"] == 2 and result["accepted"], C.report(result)
    assert max(result["held_out_xy_m"]) <= C.ACCEPT_XY_M
    j1_min, j1_max = result["envelope"]["j1"]
    assert j1_max - j1_min > 40                     # the envelope now spans the side poses too
    # runtime check at the side: cube at the side spot seen from side poses
    errors = []
    for s in side:
        hit = C.ray_plane_xy(*np.mean(s["corners"], axis=0), M.fk_arm4(s["servo"]) @ result["arm4_T_optical"],
                             result["K"], result["k1"], result["tag_top_z"])
        errors.append(float(np.linalg.norm(hit[:2] - side_T[:2, 3])))
    assert np.median(errors) < 0.004


def test_align_preview_moves_to_the_start_pose_first_and_waits_for_space(monkeypatch):
    import cv2
    import pose_library as PL

    class Arm:
        def __init__(self):
            self.moves = []

        def Arm_serial_servo_write6_array(self, values, ms):
            self.moves.append([round(v) for v in values[:5]])

    class Cap:
        def read(self):
            return True, np.zeros((480, 640, 3), np.uint8)

    keys = iter([-1, -1, 65, 32, -1])               # two idle frames, a stray key, then SPACE
    shown = []
    monkeypatch.setattr(C.time, "sleep", lambda *_: None)
    monkeypatch.setattr(cv2, "namedWindow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "imshow", lambda title, frame: shown.append(frame.shape))
    monkeypatch.setattr(cv2, "waitKey", lambda *_: next(keys))
    monkeypatch.setattr(cv2, "destroyWindow", lambda *_: None)
    monkeypatch.setattr(C, "detect_tag", lambda frame, det, tag_id: None)
    arm = Arm()
    C.align_preview(arm, Cap(), None, 4, PL.POSES["LEFT"], log=lambda *_: None)
    assert arm.moves == [[90, 125, 0, 0, 90], [130, 125, 0, 0, 90]]    # via READY, then to LEFT
    assert len(shown) == 4                                              # live view until SPACE

    keys = iter([27, -1])
    with pytest.raises(KeyboardInterrupt):
        C.align_preview(Arm(), Cap(), None, 4, PL.POSES["RIGHT"], log=lambda *_: None)


def test_j1_scale_is_recovered_from_both_sides_and_fixes_the_side_poses():
    import pose_library as PL
    rng, X, K, T, see, observe_mid = make_world(21, 4.0, 0.97, ratio=1.04, k1=-0.45, backlash=0.3, j1_scale=1.12)
    mid = C.collect_loop(observe_mid, 16, log=lambda *_: None)
    groups = list(mid)
    for group, pose_name in ((1, "LEFT"), (2, "RIGHT")):
        # the user centres the cube in the frame at the start pose: put the tag where that pose looks
        start = PL.POSES[pose_name]
        cam = M.fk_arm4_raw([90.0 + 1.12 * (start[0] - 90.0), *start[1:]]) @ X
        centre = cam[:3, 3] + cam[:3, 2] * (T[2, 3] - cam[2, 3]) / cam[2, 2]
        sT = T.copy()
        sT[:3, 3] = [centre[0], centre[1], T[2, 3]]
        tag = sT @ np.c_[C.tag_points(), np.ones(4)].T
        observe = lambda pose, tag=tag: see(tag, pose)
        assert observe(start)[1] is not None, f"the {pose_name} spot is not visible from its start pose"
        got = C.collect_loop(observe, 14, log=lambda *_: None, start_pose=start,
                             j1_values=range(20, 161, 5), group=group)
        assert len(got) >= 8, pose_name
        groups += got
    spread = max(s["servo"][0] for s in groups) - min(s["servo"][0] for s in groups)
    assert spread > 60
    fixed = C.solve(groups, fit_j1=False)
    fitted = C.solve(groups, fit_j1=True)
    assert fitted["j1_fitted"] and abs(fitted["j1_scale"] - 1.12) < 0.03, C.report(fitted)
    assert abs(fitted["j1_offset_deg"]) < 1.0
    assert max(fitted["held_out_xy_m"]) < max(fixed["held_out_xy_m"])
    assert fitted["accepted"], C.report(fitted)
    # centre-only data must not invent a J1 correction
    centre_only = C.solve(mid, fit_j1=True)          # asked for, but the J1 span is too small to fit
    assert not centre_only["j1_fitted"] and centre_only["j1_scale"] == 1.0
    assert not C.solve(groups)["j1_fitted"]            # and it is never fitted by default


def test_anchored_multi_group_solve_keeps_the_centre_model(tmp_path):
    """The centre-only model was proven by real picks; side data must not shift it."""
    import pose_library as PL
    rng, X, K, T, see, observe_mid = make_world(31, 4.0, 0.97, ratio=1.04, k1=-0.45, backlash=0.3)
    mid = C.collect_loop(observe_mid, 16, log=lambda *_: None)
    centre = C.solve(mid)
    proven = tmp_path / "hand_eye.json"
    C.write_outputs(centre, C.TAG_SIZE_M, json_path=proven, yaml_path=None)
    anchor = C.anchor_from_json(proven)
    assert anchor is not None and anchor["tag_xy"][1] < 0.001 and anchor["cam_pos"][1] <= 0.001
    assert C.anchor_from_json(tmp_path / "missing.json") is None
    samples = list(mid)
    for group, pose_name in ((1, "LEFT"), (2, "RIGHT")):
        start = PL.POSES[pose_name]
        cam = M.fk_arm4_raw(start) @ X
        spot = cam[:3, 3] + cam[:3, 2] * (T[2, 3] - cam[2, 3]) / cam[2, 2]
        sT = T.copy()
        sT[:3, 3] = [spot[0], spot[1], T[2, 3]]
        tag = sT @ np.c_[C.tag_points(), np.ones(4)].T
        samples += C.collect_loop(lambda pose, tag=tag: see(tag, pose), 14, log=lambda *_: None,
                                  start_pose=start, j1_values=range(20, 161, 5), group=group)
    result = C.solve(samples, anchor=anchor)
    assert result["accepted"] and result["n_groups"] == 3, C.report(result)
    assert abs(result["tag_top_z"] - centre["tag_top_z"]) < 0.002          # plane = group 0 (not averaged)
    shift = np.linalg.norm(result["base_T_tag"][:2, 3] - centre["base_T_tag"][:2, 3])
    assert shift < 0.0015                                                  # centre cube did not move
    assert result["envelope"]["j1"][1] - result["envelope"]["j1"][0] > 50
    assert not result["j1_fitted"]                                         # J1 correction is opt-in
