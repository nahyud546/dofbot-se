import math

import cv2
import numpy as np
import pytest

import cube_search_center_math as M
import zone_locator as Z
from test_cube_layer import CAL, TAG_TOP_Z

TABLE_Z = TAG_TOP_Z - 0.030
SERVO = [50.0, 125.0, 0.0, 0.0, 90.0]
RED = (0, 0, 200)


def render(pad_corners_base, servo=SERVO, colour=RED):
    """Khung 640x480 với ô (đa giác trên mặt bàn) chiếu bằng đúng mô hình hand-eye."""
    img = np.full((480, 640, 3), 215, np.uint8)
    pts = []
    for x, y in pad_corners_base:
        u, v, _ = Z.project_to_pixel([x, y, TABLE_Z], servo, CAL)
        pts.append([u, v])
    cv2.fillPoly(img, [np.round(np.array(pts)).astype(np.int32)], colour)
    return img


def rect(cx, cy, w, h, deg=0.0):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]


def look_at(servo=SERVO):
    """Điểm bàn mà camera nhìn tại tâm ảnh (để đặt ô vào khung)."""
    M.apply_calibration(CAL)
    try:
        P, _ = M.pixel_to_base(320, 240, servo, z_table=TABLE_Z)
    finally:
        M.apply_calibration(None)
    return float(P[0]), float(P[1])


def test_margin_matches_the_metric_distance_to_the_pad_edge():
    cx, cy = look_at()
    img = render(rect(cx, cy, 0.09, 0.07))
    mask = Z.pad_mask(img, 3)
    assert mask is not None
    centre = Z.signed_margin_mm(mask, (cx, cy), SERVO, CAL, TABLE_Z)
    assert 30.0 <= centre <= 40.0                      # half of the short side is 35 mm
    off = Z.signed_margin_mm(mask, (cx + 0.025, cy), SERVO, CAL, TABLE_Z)
    assert abs(off - centre) > 3.0
    outside = Z.signed_margin_mm(mask, (cx + 0.06, cy), SERVO, CAL, TABLE_Z)
    assert outside is not None and outside < 0         # past the edge -> negative


def test_pad_clipped_by_the_frame_does_not_count_its_image_border_as_an_edge():
    cx, cy = look_at()
    img = render(rect(cx, cy, 0.20, 0.20))              # pad larger than the view
    mask = Z.pad_mask(img, 3)
    M.apply_calibration(CAL)
    try:
        near_border, _ = M.pixel_to_base(40, 240, SERVO, z_table=TABLE_Z)   # 40 px from the left frame edge
    finally:
        M.apply_calibration(None)
    margin = Z.signed_margin_mm(mask, (float(near_border[0]), float(near_border[1])), SERVO, CAL, TABLE_Z)
    # the frame edge is ~10 mm away (40 px), but that is not a pad edge: the real margin (nearest true pad edge) is larger
    assert margin is not None and margin > 20.0         # border-as-edge would give ~9 mm


def test_best_shift_recovers_a_pad_rotated_about_the_base():
    cx, cy = look_at()
    release = (cx, cy)
    for true_shift in (-10.0, 6.0):
        centre = Z.rotate_about_base(release, true_shift)
        corners = rect(centre[0], centre[1], 0.06, 0.06)
        mask = Z.pad_mask(render(corners), 3)
        delta, m_cfg, m_best = Z.best_shift(mask, release, SERVO, CAL, TABLE_Z)
        assert abs(delta - true_shift) <= 3.0, (true_shift, delta)
        assert m_best > (m_cfg if m_cfg is not None else -99) + 5


def test_decide_keeps_shifts_or_falls_back():
    assert Z.decide(50.0, 0.0, 30.0, 30.0)[:2] == (50.0, "keep")
    j1, action, _ = Z.decide(50.0, 8.0, -20.0, 25.0)
    assert (j1, action) == (58.0, "shifted")
    assert Z.decide(50.0, 12.0, 2.0, 9.0)[1] == "fallback"           # best still unsafe
    assert Z.decide(50.0, 8.0, 5.0, 12.0)[1] == "fallback"           # gain too small / margin below safe
    assert Z.decide(50.0, 8.0, None, None)[1] == "fallback"
    assert Z.decide(50.0, 40.0, -50.0, 40.0)[1] == "fallback"        # beyond the allowed shift (30°)


def test_survey_zone_end_to_end_with_a_misplaced_pad():
    cx, cy = look_at()
    release = (cx, cy)
    centre = Z.rotate_about_base(release, 9.0)         # the mat sits 9 degrees off
    img = render(rect(centre[0], centre[1], 0.06, 0.06))
    res = Z.survey_zone(3, 50.0, release, lambda j1: SERVO, lambda: img, CAL, TABLE_Z, log=lambda *_: None)
    assert res.action == "shifted" and abs(res.use_j1 - 59.0) <= 3.0
    absent = Z.survey_zone(3, 50.0, release, lambda j1: SERVO,
                           lambda: np.full((480, 640, 3), 215, np.uint8), CAL, TABLE_Z, log=lambda *_: None)
    assert absent.action == "fallback" and absent.use_j1 == 50.0


def test_zone_j1_override_lives_only_in_the_process_and_feeds_the_zone_poses():
    import cube_search_center as SC
    M.ZONE_J1_RUNTIME.clear()
    try:
        assert M.zone_j1(3) == 50.0 and SC.zone_pose(SC.ZONE_RELEASE, 3)[0] == 50.0
        M.ZONE_J1_RUNTIME[3] = 58.0
        assert M.zone_j1(3) == 58.0
        pose = SC.zone_pose(SC.ZONE_RELEASE, 3)
        assert pose[0] == 58.0 and pose[1:] == SC.ZONE_RELEASE[3][1:]    # only J1 changes
        assert SC.ZONE_RELEASE[3][0] == 50.0                              # tables stay untouched
        assert SC.compute_place_delta(90.0, 3)["j1_zone"] == 58.0
    finally:
        M.ZONE_J1_RUNTIME.clear()
    assert M.zone_j1(3) == 50.0


def test_preflight_accepts_side_targets_and_still_rejects_out_of_reach():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
    from cube_3d_preflight import check_target
    from t8_motion_worker import Kinematics
    kin = Kinematics()
    # Far to the side (J1 ~ 50 and ~130): outside the old rectangle |y| <= 0.12 m, inside the polar area.
    for x, y in ((-0.17, 0.17), (-0.17, -0.17)):
        check_target({"stage": "pick", "tcp_position_base": [x, y, 0.047], "preferred_yaw_rad": 0.0}, kin)
    for x, y in ((-0.05, 0.0), (0.05, 0.10), (-0.34, 0.0)):             # too close / wrong side / too far
        with pytest.raises(ValueError):
            check_target({"stage": "pick", "tcp_position_base": [x, y, 0.047],
                          "preferred_yaw_rad": 0.0}, kin)
