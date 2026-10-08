import math

from safety_gate import ALL_JOINTS, SafetyGate, normalize_readback_limits


def test_quantized_readback_snaps_only_near_limit():
    cfg = {
        "readback_limit_snap_rad": 0.01,
        "urdf_limits_rad": {"arm3_Joint": [-1.57, 1.57]},
    }
    assert normalize_readback_limits({"arm3_Joint": math.pi / 2}, cfg)["arm3_Joint"] == 1.57
    # A materially invalid sensor value must remain invalid for the gate.
    assert normalize_readback_limits({"arm3_Joint": 1.59}, cfg)["arm3_Joint"] == 1.59


def test_readback_normalization_does_not_mutate_input():
    cfg = {
        "readback_limit_snap_rad": 0.01,
        "urdf_limits_rad": {"arm3_Joint": [-1.57, 1.57]},
    }
    measured = {"arm3_Joint": math.pi / 2}
    normalize_readback_limits(measured, cfg)
    assert measured["arm3_Joint"] == math.pi / 2


def test_gate_allows_only_first_waypoint_to_escape_margin():
    cfg = {
        "joint_limit_margin_rad": 0.02,
        "start_state_tol_rad": 0.05,
        "max_joint_step_rad": 0.6,
        "velocity_scaling_max_hw": 0.25,
        "acceleration_scaling_max_hw": 0.25,
        "max_velocity_rad_s": 2.0,
        "max_accel_rad_s2": 4.0,
        "joint_state_max_age_sec": 1.0,
        "tcp_offset_calibrated": True,
        "urdf_limits_rad": {name: ([0.0, 1.57] if name == "Rlink1_Joint"
                                    else [-1.57, 1.57]) for name in ALL_JOINTS},
    }
    gate = SafetyGate(cfg)
    start = [0.0, -0.55, 1.57, 1.57, 0.1, 1.0]
    inward = [0.03, -0.46, 1.52, 1.48, 0.03, 1.0]
    points = [{"positions": start, "time_from_start": 0.01},
              {"positions": inward, "time_from_start": 5.0}]
    q0 = dict(zip(ALL_JOINTS, start))
    assert gate.check_trajectory(ALL_JOINTS, points, q0_real=q0,
                                 state_age_sec=0.1, allow_hw=True)[0]

    # Staying in the margin is not an escape and remains rejected.
    points[1]["positions"][2] = 1.56
    ok, reasons = gate.check_trajectory(ALL_JOINTS, points, q0_real=q0,
                                        state_age_sec=0.1, allow_hw=True)
    assert not ok
    assert any("sát limit" in reason for reason in reasons)


def test_gate_allows_monotonic_interpolated_escape_prefix():
    cfg = {
        "joint_limit_margin_rad": 0.02,
        "start_state_tol_rad": 0.05,
        "max_joint_step_rad": 0.6,
        "velocity_scaling_max_hw": 0.25,
        "acceleration_scaling_max_hw": 0.25,
        "max_velocity_rad_s": 2.0,
        "max_accel_rad_s2": 4.0,
        "joint_state_max_age_sec": 1.0,
        "tcp_offset_calibrated": True,
        "urdf_limits_rad": {name: ([0.0, 1.57] if name == "Rlink1_Joint"
                                    else [-1.57, 1.57]) for name in ALL_JOINTS},
    }
    gate = SafetyGate(cfg)
    rows = []
    for k, arm3 in enumerate((1.57, 1.565, 1.555, 1.54, 1.52)):
        rows.append({"positions": [0.0, -0.5, arm3, 1.4, 0.0, 1.0],
                     "time_from_start": 1.0 + k})
    q0 = dict(zip(ALL_JOINTS, rows[0]["positions"]))
    assert gate.check_trajectory(ALL_JOINTS, rows, q0_real=q0,
                                 state_age_sec=0.1, allow_hw=True)[0]

    rows[2]["positions"][2] = 1.568  # reverses toward the endpoint
    assert not gate.check_trajectory(ALL_JOINTS, rows, q0_real=q0,
                                     state_age_sec=0.1, allow_hw=True)[0]
