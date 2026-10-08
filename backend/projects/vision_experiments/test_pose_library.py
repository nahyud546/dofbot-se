import math

import numpy as np

import pose_library as PL
from test_cube_layer import CAL, TAG_TOP_Z


def target_at(servo, u=320, v=240):
    import cube_search_center_math as M
    M.apply_calibration(CAL)
    try:
        P, _ = M.pixel_to_base(u, v, servo, z_table=TAG_TOP_Z - 0.03)
    finally:
        M.apply_calibration(None)
    return [float(P[0]), float(P[1]), TAG_TOP_Z - 0.03]


def test_every_library_pose_keeps_the_tip_above_the_table():
    assert all(PL.is_safe(servo) for servo in PL.POSES.values())


def test_keeps_the_current_pose_when_the_target_is_already_in_view():
    cur = PL.POSES["READY"]
    name, servo, moved = PL.choose_view(cur, target_at(cur), CAL)
    assert (name, moved) == ("CURRENT", False) and servo == cur


def test_picks_the_nearest_pose_that_sees_a_side_target():
    right_target = target_at(PL.POSES["RIGHT"])
    name, servo, moved = PL.choose_view(PL.POSES["READY"], right_target, CAL)
    assert moved and name in ("RIGHT", "RIGHT_HIGH", "FAR_RIGHT")
    assert PL.visible_in(servo, right_target, CAL)
    left_target = target_at(PL.POSES["LEFT"])
    name, servo, moved = PL.choose_view(PL.POSES["READY"], left_target, CAL)
    assert moved and name.startswith(("LEFT", "FAR_LEFT")) and servo[0] > 90


def test_unseeable_target_returns_none():
    assert PL.choose_view(PL.POSES["READY"], [0.5, 0.0, 0.03], CAL) == (None, None, False)
