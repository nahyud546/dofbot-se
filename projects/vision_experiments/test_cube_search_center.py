"""Unit test luồng search-center độc lập. Không chạm file/state luồng cũ."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cube_search_center as sc
import cube_search_center_math as m


def test_ray_plane_ready_center():
    # Baseline CAD (pitch_corr=0): giữ nguyên số cũ làm neo hồi quy.
    T = m.fk_optical([90, 125, 0, 0, 90], pitch_corr_deg=0.0)
    P, d, _v = m.ray_plane(320, 240, T)
    assert P[2] == pytest.approx(0.045)
    assert P[0] == pytest.approx(-0.0988, abs=0.005)
    assert d == pytest.approx(0.135, abs=0.01)


def test_mount_pitch_corr_default_14():
    assert m.MOUNT_PITCH_CORR_DEG == pytest.approx(14.0)
    # Tắt hiệu chuẩn phải về đúng baseline CAD.
    T = m.fk_optical([125, 125, 0, 0, 90], pitch_corr_deg=0.0)
    P, d, _v = m.ray_plane(319.29, 202.47, T)
    assert P[0] == pytest.approx(-0.0872, abs=0.002)
    assert P[1] == pytest.approx(-0.0604, abs=0.002)


def test_mount_pitch_corr_lock_shift():
    # Frame LOCK thật: correction phải đẩy P dọc tia ~+50mm, ngang ~0.
    T0 = m.fk_optical([125, 125, 0, 0, 90], pitch_corr_deg=0.0)
    P0, _d0, _v = m.ray_plane(319.29, 202.47, T0)
    P1, _d1 = m.pixel_to_base(319.29, 202.47, [125, 125, 0, 0, 90])
    pc = T0[:3, 3]
    g = (P0[:2] - pc[:2])
    g = g / float(np.linalg.norm(g))
    delta = (P1[:2] - P0[:2]) * 1000.0
    along = float(delta @ g)
    lat = float(delta[0] * (-g[1]) + delta[1] * g[0])
    assert along == pytest.approx(50.0, abs=12.0)
    assert abs(lat) < 3.0


def test_ray_plane_rejects_upward():
    T = m.fk_optical([90, 125, 0, 0, 90])
    # Giả lập camera ngửa lên: lật R để vbase.z dương.
    T_bad = T.copy()
    T_bad[:3, :3] = np.eye(3)
    with pytest.raises(ValueError, match="không nhìn bàn"):
        m.ray_plane(320, 240, T_bad)


def test_workspace_widened_to_005():
    m.check_workspace(-0.06, 0.0)
    m.check_workspace(-0.05, 0.0)
    with pytest.raises(ValueError, match="ngoài vùng"):
        m.check_workspace(-0.04, 0.0)
    with pytest.raises(ValueError, match="ngoài vùng"):
        m.check_workspace(-0.35, 0.0)


def test_j1_delta_abort_over_45():
    with pytest.raises(ValueError, match="vượt 45"):
        sc.compute_place_delta(90.0, 1)
    with pytest.raises(ValueError, match="vượt 45"):
        sc.compute_place_delta(30.0, 2)
    # Trong giới hạn thì pass.
    assert sc.compute_place_delta(40.0, 1)["delta"] == pytest.approx(-10.0)


def test_staged_rotate_splits_over_45():
    place = sc.compute_place_delta(102.3, 3, allow_staged=True)
    assert place["staged"] is True
    assert place["delta"] == pytest.approx(-52.3, abs=0.5)
    steps = place["steps"]
    assert 1 <= len(steps) <= 4
    # Mỗi nhịp ≤40° tính từ mốc trước đó (case này không qua wrap 0/180).
    prev = 102.3
    for s in steps:
        assert abs(float(s) - float(prev)) <= 40.0 + 1e-9
        prev = s
    assert steps[-1] == pytest.approx(50.0, abs=1.0)
    # split helper độc lập.
    assert sc.split_j1_steps(102.0, 50.0) == pytest.approx([76.0, 50.0])
    assert sc.split_j1_steps(40.0, 30.0) == [30.0]


def test_proven_color_detects_solid_red():
    import cv2
    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (280, 200), (360, 280), (0, 0, 255), -1)
    cands = sc.proven_color_candidates(img, img.copy(), want_id=3)
    assert len(cands) == 1
    assert cands[0]["cube_id"] == 3
    cx, cy = cands[0]["center"]
    assert abs(cx - 320) < 15 and abs(cy - 240) < 15


def test_white_card_fullframe_rejected():
    """Ảnh abort: nền trắng tràn frame -> không candidate nào (P0)."""
    import cv2
    img = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(img, (0, 0), (500, 480), (245, 245, 245), -1)
    cv2.line(img, (100, 100), (300, 300), (30, 30, 30), 3)
    # Gate size loại quad nền lớn.
    huge = [[0, 0], [500, 0], [500, 480], [0, 480]]
    assert sc.is_plausible_cube(huge, [250, 240]) is False
    normal = [[280, 200], [360, 200], [360, 280], [280, 280]]
    assert sc.is_plausible_cube(normal, [320, 240]) is True
    # confirm 2-frame trên cùng ảnh cũng phải rỗng (không hsv_single loạn).
    got = sc.detect_confirmed(lambda: img, 3)
    assert got == []


def test_f1_lock_gate_x_strict_y_advisory():
    # X chặt ±10 (trục có servo).
    assert sc.is_x_centered(315) is True
    assert sc.is_x_centered(331) is False
    # Y advisory ±60 (audit: J1 không chỉnh được Y).
    assert sc.is_y_ok(185) is True
    assert sc.is_y_ok(300) is True
    assert sc.is_y_ok(179) is False
    assert sc.is_y_ok(325) is False
    # Case audit run abort: y=190/217 pass advisory, y=325 fail.
    assert sc.is_y_ok(190) is True
    assert sc.is_y_ok(217) is True


def test_f2_assoc_gate():
    assert sc.assoc_ok(None, 500, 300) is True
    assert sc.assoc_ok([320, 240], 350, 260) is True
    assert sc.assoc_ok([320, 240], 500, 300) is False  # nhảy cube<->thảm
    assert sc.assoc_ok([220, 276], 359, 190) is False  # đúng case log abort


def test_f4_cube_size_cap():
    mid = [[255, 175], [385, 175], [385, 305], [255, 305]]  # 130x130
    assert sc.is_plausible_cube(mid, [320, 240]) is True
    big = [[235, 155], [405, 155], [405, 325], [235, 325]]  # 170x170 thảm
    assert sc.is_plausible_cube(big, [320, 240]) is False


def test_f3_flicker_single_frame_rejected():
    """2 frame tươi khác nhau: flicker 1 frame -> rỗng (F3)."""
    import cv2
    cube = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(cube, (280, 200), (360, 280), (0, 0, 255), -1)
    blank = np.full((480, 640, 3), 190, np.uint8)
    frames = [cube, blank]
    assert sc.detect_confirmed(lambda: frames.pop(0), 3) == []
    stable = [cube.copy(), cube.copy()]
    got = sc.detect_confirmed(lambda: stable.pop(0), 3)
    assert len(got) == 1 and got[0]["cube_id"] == 3


def test_grasp_confirm_input_yes_no(monkeypatch):
    import builtins

    class _Cap:
        def read(self):
            return False, None

    lock = {"locked": {"cube_id": 3, "center": [320, 240],
                       "quad": [[280, 200], [360, 200],
                                [360, 280], [280, 280]]},
            "j1": 90.0}
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    monkeypatch.setattr(builtins, "input", lambda *a, **k: "")
    assert sc._wait_grasp_confirm(_Cap(), lock, 3, False) is True
    monkeypatch.setattr(builtins, "input", lambda *a, **k: "q")
    assert sc._wait_grasp_confirm(_Cap(), lock, 3, False) is False


def test_patterned_cube_white_top_caught_by_proven():
    """Cube thật mặt trên trắng + dải màu hông -> proven bắt (P0)."""
    import cv2
    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (280, 200), (360, 280), (255, 255, 255), -1)
    cv2.rectangle(img, (280, 280), (360, 320), (0, 0, 255), -1)
    got = sc.detect_confirmed(lambda: img.copy(), 3)
    assert len(got) == 1
    assert got[0]["cube_id"] == 3
    assert got[0]["source"].startswith("proven")


def test_sweep_pingpong():
    nxt, d = sc.sweep_next_j1(155, 1)
    assert (nxt, d) == (160.0, -1)
    nxt, d = sc.sweep_next_j1(160.0, -1)
    assert nxt < 160.0 and d == -1
    nxt, d = sc.sweep_next_j1(20.0, -1)
    assert (nxt, d) == (20.0, 1)


def test_slow_strict_centering():
    assert sc.is_centered(325, 250) is True
    assert sc.is_centered(335, 250) is False
    assert sc.is_centered(320, 265) is False
    assert sc.confirm_center([True] * 5) is True
    assert sc.confirm_center([True] * 4) is False
    assert sc.confirm_center([True] * 4 + [False]) is False
    # Bước chậm bị clamp ±2°.
    assert abs(sc.proportional_j1(90.0, 320 + 500) - 90.0) <= 2.0 + 1e-9


def test_compute_pick_uses_live_pose():
    quad = [[280, 200], [360, 200], [360, 280], [280, 280]]
    pick = sc.compute_pick(320, 240, quad, [90, 125, 0, 0, 90])
    # Khớp ray-plane ở hiệu chuẩn hiện hành (mount pitch +14°).
    assert pick["x"] == pytest.approx(-0.1485, abs=0.01)
    assert pick["z"] == 0.047


def test_state_file_isolated():
    assert str(sc.STATE_FILE) == "/tmp/t8_search_center_state.json"
    assert "t8_hold_state" not in str(sc.STATE_FILE)


def test_no_legacy_import():
    import re
    src = Path(__file__).with_name("cube_search_center.py").read_text()
    assert not re.search(r"^\s*(import|from)\s+t8_motion_worker", src, re.M)
    assert not re.search(r"^\s*(import|from)\s+cube_sort_stage1", src, re.M)
    assert "t8_hold_state" not in src
    srcm = Path(__file__).with_name("cube_search_center_math.py").read_text()
    assert not re.search(r"^\s*(import|from)\s+t8_motion_worker", srcm, re.M)


class _HoleKin:
    """Giả lập lỗ IK: low-z fail, anchor z>=0.08 OK; FK tuyến tính theo J2."""

    def ik(self, x, y, z):
        if z < 0.08:
            raise RuntimeError("no solution")
        return [84.0, 112.5, 10.0, 20.0, 60.0]

    def fk(self, joints):
        j2 = joints[1]
        return [-0.099 + 0.0002 * (j2 - 112.5), 0.012,
                0.21 - 0.00128 * j2]


class _DirectKin:
    def ik(self, x, y, z):
        return [84.0, 70.0, 30.0, 40.0, 60.0]

    def fk(self, joints):
        raise AssertionError("direct không cần FK")


def test_descend_plans_verified_move():
    approach, grasp, info = sc.plan_descend(_HoleKin(), -0.099, 0.012)
    assert info["mode"] == "descend"
    assert info["axis"] == 2  # J2 nhạy z nhất trong stub
    assert abs(info["deg"]) <= sc.DESCEND_MAX_DEG
    assert info["drift_mm"] <= sc.DESCEND_MAX_LATERAL * 1000
    assert abs(grasp[1] - approach[1] - info["deg"]) < 0.01
    assert abs(info["z_reached"] - sc.DESCEND_TARGET_Z) <= sc.DESCEND_Z_TOL


def test_solve_direct_unchanged_when_no_hole():
    sol = sc.solve_pick_ik(_DirectKin(), -0.13, 0.0)
    assert sol["mode"] == "direct"
    assert sol["approach"] is None
    assert sol["z"] == 0.047


def test_solve_descend_fallback_through_hole():
    sol = sc.solve_pick_ik(_HoleKin(), -0.099, 0.012)
    assert sol["mode"] == "descend"
    assert sol["approach"][1] == 112.5
    assert sol["z"] == sc.DESCEND_TARGET_Z


def test_descend_aborts_on_excess_drift():
    class _Drifty(_HoleKin):
        def fk(self, joints):
            x, y, z = super().fk(joints)
            return [x + 0.05 * (joints[1] - 112.5), y, z]

    with pytest.raises(RuntimeError, match="trôi XY"):
        sc.plan_descend(_Drifty(), -0.099, 0.012)


def test_descend_aborts_when_too_deep():
    class _Deep(_HoleKin):
        def fk(self, joints):
            x, y, z = super().fk(joints)
            return [x, y, z + 0.05]  # anchor đã cao hơn, cần hạ sâu

    with pytest.raises(RuntimeError, match="cần hạ"):
        sc.plan_descend(_Deep(), -0.099, 0.012)


class _HighKin:
    """Chỉ solve ở z>=0.068 (đúng case (-0.087,-0.060) audit): direct cao
    phải được hạ thay vì gắp luôn."""

    def __init__(self):
        self._z = 0.068
        self._j2 = 90.0

    def ik(self, x, y, z):
        if z < 0.068:
            raise RuntimeError("no solution")
        self._z = float(z)
        return [123.7, 90.0, 2.9, 8.0, 100.8]

    def fk(self, joints):
        j2 = joints[1]
        return [-0.087 + 0.0002 * (j2 - 90.0), -0.060,
                self._z - 0.00128 * (j2 - 90.0)]


def test_direct_high_descends_instead_of_top_grip():
    sol = sc.solve_pick_ik(_HighKin(), -0.087, -0.060)
    assert sol["mode"] == "descend"
    assert sol["z"] == sc.DESCEND_TARGET_Z
    assert sol["approach"] is not None


class _CancelKin:
    """J2/J3 trôi ngược nhau -> LSQ phối hợp triệt drift gần 0."""

    def ik(self, x, y, z):
        if z < 0.08:
            raise RuntimeError("no solution")
        return [84.0, 100.0, 100.0, 20.0, 60.0]

    def fk(self, joints):
        a, b = joints[1] - 100.0, joints[2] - 100.0
        return [-0.099 + 0.0008 * a - 0.0008 * b, 0.012,
                0.066 - 0.001 * a - 0.001 * b]


def test_lsq_two_axis_cancels_drift():
    approach, grasp, info = sc.plan_descend(_CancelKin(), -0.099, 0.012)
    assert info["mode"] == "descend"
    assert info.get("how") == "lsq-J2J3"
    assert info["drift_mm"] < 2.0
    # Cả 2 trục cùng động (chia đôi ~4° mỗi trục).
    assert abs(grasp[1] - approach[1]) > 1.0
    assert abs(grasp[2] - approach[2]) > 1.0


def test_wait_settled_ok_and_timeout():
    class _Arm:
        def __init__(self, joints):
            self._j = list(joints)

        def read_joints(self):
            return list(self._j)

    arm = sc.ArmAdapter.__new__(sc.ArmAdapter)
    arm._arm = None
    arm.read_joints = _Arm([90.0, 125.0, 0.0, 0.0, 90.0]).read_joints
    st = sc.ArmAdapter.wait_settled(arm, [90.0, 125.0, 0.0, 0.0, 90.0],
                                    timeout_s=1.0)
    assert st["ok"] is True
    st = sc.ArmAdapter.wait_settled(arm, [90.0, 80.0, 0.0, 0.0, 90.0],
                                    timeout_s=0.5)
    assert st["ok"] is False
    assert st["err"][1] == 45.0


def _verify_fixtures():
    import cv2
    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (280, 200), (360, 280), (0, 0, 255), -1)

    class _Cap:
        def read(self):
            return True, img.copy()

    class _Arm:
        def read_joints(self):
            return [90.0, 125.0, 0.0, 0.0, 90.0]

    return _Cap(), _Arm()


def test_verify_no_correction_when_aligned():
    cap, arm = _verify_fixtures()
    P0, _d = m.pixel_to_base(320.5, 240.5, [90, 125, 0, 0, 90])
    pick = {"x": float(P0[0]), "y": float(P0[1]), "z": 0.047, "yaw": 0.0}
    joints = [90, 50, 60, 20, 90]
    j2, sol2, rep = sc.verify_and_correct(cap, arm, None, pick, joints, 3)
    assert rep["corrected"] is False
    assert sol2 is None
    assert j2 == joints


def test_verify_corrects_systematic_bias():
    cap, arm = _verify_fixtures()
    P0, _d = m.pixel_to_base(320.5, 240.5, [90, 125, 0, 0, 90])
    # pick lệch hệ thống +30mm X so với mắt thấy.
    pick = {"x": float(P0[0]) + 0.030, "y": float(P0[1]),
            "z": 0.047, "yaw": 0.0}
    joints = [90, 50, 60, 20, 90]

    class _Kin:
        def ik(self, x, y, z):
            self.seen = (x, y, z)
            return [84.0, 70.0, 30.0, 40.0, 60.0]

    kin = _Kin()
    j2, sol2, rep = sc.verify_and_correct(cap, arm, kin, pick, joints, 3)
    assert rep["corrected"] is True
    assert rep["res_mm"] == pytest.approx(30.0, abs=3.0)
    # Mục tiêu mới = trừ thẳng residual.
    assert kin.seen[0] == pytest.approx(pick["x"] - 0.030, abs=0.004)
    assert j2[4] == pytest.approx(84.0)  # J5 = J1 - yaw


def test_verify_aborts_on_huge_residual():
    cap, arm = _verify_fixtures()
    pick = {"x": -0.200, "y": 0.100, "z": 0.047, "yaw": 0.0}
    with pytest.raises(RuntimeError, match="vượt"):
        sc.verify_and_correct(cap, arm, None, pick, [90, 50, 60, 20, 90],
                              3)


def test_refresh_lock_recomputes_after_space(monkeypatch):
    import cv2

    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (280, 200), (360, 280), (0, 0, 255), -1)

    class _Cap:
        def read(self):
            return True, img.copy()

    class _Arm:
        def read_joints(self):
            return [125.0, 125.0, 0.0, 0.0, 90.0]

    old = {"locked": {"cube_id": 3, "center": [320.0, 240.0],
                      "quad": [[280, 200], [360, 200],
                               [360, 280], [280, 280]],
                      "source": "proven_hsv"},
           "j1": 125.0}
    new = sc.refresh_lock(_Cap(), _Arm(), 3, old, 3)
    assert new["servo5"] == [125.0, 125.0, 0.0, 0.0, 90.0]
    assert new["locked"]["cube_id"] == 3
    # Cube bị dời sau SPACE -> abort chứ không gắp mù.
    moved = {"locked": dict(old["locked"], center=[100.0, 100.0]),
             "j1": 125.0}
    with pytest.raises(RuntimeError, match="đổi vị trí"):
        sc.refresh_lock(_Cap(), _Arm(), 3, moved, 3)


def test_quad_yaw_neutral_when_near_square():
    # Quad gần vuông (120x102, đúng case run abort): nhiễu -> trung tính,
    # thay vì xoay 26° húc góc cube.
    assert m.quad_yaw([[200, 150], [320, 150],
                       [320, 252], [200, 252]]) == 0.0
    # Mặt rõ chữ nhật vẫn căn theo cạnh dài như cũ.
    yaw = m.quad_yaw([[200, 150], [360, 150],
                      [360, 230], [200, 230]])
    assert yaw == pytest.approx(0.0, abs=1.0)
    yaw2 = m.quad_yaw([[200, 150], [330, 190],
                       [310, 260], [180, 220]])
    assert abs(yaw2) > 1.0


def _tag_seq(dets):
    """TagDetector giả: trả detection preset mỗi lần detect (theo frame)."""

    class _Tagger:
        def __init__(self):
            self._rest = list(dets)

        def detect(self, _bgr):
            if self._rest:
                return [self._rest.pop(0)]
            return []

    return _Tagger()


def _tag_det(cx, cy, side, cid=3):
    h = side / 2.0
    return {"cube_id": cid, "source": "apriltag",
            "center": [float(cx), float(cy)],
            "quad": [[cx - h, cy - h], [cx + h, cy - h],
                     [cx + h, cy + h], [cx - h, cy + h]]}


def _dummy_frame():
    return np.zeros((480, 640, 3), np.uint8)


def test_closeup_relaxed_pair_for_vibration():
    # Rung tay ở cự ly gần: drift 18px trên tag 200px -> strict loại,
    # closeup (gate 10% size) chấp nhận.
    a = _tag_det(300, 200, 200)
    b = _tag_det(318, 218, 208)
    assert sc.detect_confirmed(_dummy_frame, 3,
                               tag=_tag_seq([a, b]),
                               max_side=320.0) == []
    got = sc.detect_confirmed(_dummy_frame, 3,
                              tag=_tag_seq([dict(a), dict(b)]),
                              max_side=320.0, closeup=True)
    assert len(got) == 1 and got[0]["confirm"] == "relaxed"


def test_closeup_single_frame_when_flicker():
    # Tag chớp 1 frame (kẹp che): strict bỏ, closeup lấy single-frame.
    a = _tag_det(300, 200, 200)
    assert sc.detect_confirmed(_dummy_frame, 3,
                               tag=_tag_seq([a]),
                               max_side=320.0) == []
    got = sc.detect_confirmed(_dummy_frame, 3,
                              tag=_tag_seq([dict(a)]),
                              max_side=320.0, closeup=True)
    assert len(got) == 1 and got[0]["confirm"] == "single"


def test_closeup_still_rejects_ambiguity():
    # 2 tag cùng ID trong 1 frame -> mơ hồ, vẫn loại.
    a = _tag_det(300, 200, 200)
    b = _tag_det(400, 300, 200)

    class _Two:
        def detect(self, _bgr):
            return [a, b]

    assert sc.detect_confirmed(_dummy_frame, 3, tag=_Two(),
                               max_side=320.0, closeup=True) == []


def test_verify_max_side_accepts_closeup():
    # Tag 30mm dí gần ở điểm gắp nở ~250px: gate verify phải qua,
    # nhưng thẻ trắng full-frame vẫn bị loại.
    big = [[190, 110], [450, 110], [450, 370], [190, 370]]  # 260x260
    assert sc.is_plausible_cube(big, [320, 240],
                                sc.VERIFY_MAX_SIDE) is True
    card = [[0, 0], [640, 0], [640, 480], [0, 480]]
    assert sc.is_plausible_cube(card, [320, 240],
                                sc.VERIFY_MAX_SIDE) is False


def test_verify_abort_reports_debug_counts():
    class _Cap:
        def read(self):
            return True, np.zeros((480, 640, 3), np.uint8)

    class _Arm:
        def read_joints(self):
            return [90.0, 125.0, 0.0, 0.0, 90.0]

    with pytest.raises(RuntimeError, match="debug1f"):
        sc.verify_and_correct(
            _Cap(), _Arm(), None,
            {"x": -0.1, "y": 0.0, "z": 0.047, "yaw": 0.0},
            [90, 50, 60, 20, 90], 3)


def test_execute_verifies_at_approach_before_grasp(monkeypatch):
    """Verify phải đọc frame khi tay còn ở approach (trước write grasp)."""
    import cv2
    monkeypatch.setattr(sc.time, "sleep", lambda _s: None)
    img = np.full((480, 640, 3), 190, np.uint8)
    cv2.rectangle(img, (280, 200), (360, 280), (0, 0, 255), -1)
    events = []

    class _Cap:
        def read(self):
            events.append("R")
            return True, img.copy()

    class _Arm:
        def __init__(self):
            self.j1 = 124.3
            self._arm = self

        def Arm_serial_servo_write(self, *a):
            events.append("S2")

        def read_joints(self):
            return [124.3, 125.0, 0.0, 0.0, 90.0]

        def read_j1(self):
            return self.j1

        def write_j1(self, angle, ms):
            self.j1 = float(angle)
            events.append("J1")

        def write6(self, joints, _angle, _ms):
            events.append(("W", [round(float(v), 1)
                                 for v in joints[:5]]))

        def wait_settled(self, joints):
            return {"ok": True, "err": [0.0] * 5}

        def write_gripper(self, angle, ms):
            events.append("G")

        def write6_array(self, joints, ms):
            events.append("Z")

    locked = {"center": (320.0, 240.0),
              "quad": [[280, 200], [360, 200], [360, 280], [280, 280]]}
    out = sc.execute_pick_place(_Arm(), _HighKin(), locked,
                                [124.3, 125.0, 0.0, 0.0, 90.0], 3,
                                dry_move=False, allow_staged=True,
                                cap=_Cap())
    assert out["ok"] is True
    writes = [e for e in events if isinstance(e, tuple)]
    assert len(writes) == 2  # approach rồi grasp (residual ~0)
    assert writes[0][1][1] == pytest.approx(90.0)  # anchor J2 chưa hạ
    assert writes[1][1][1] != pytest.approx(90.0)  # grasp đã hạ J2
    i_w1 = events.index(writes[0])
    i_w2 = events.index(writes[1])
    reads = [i for i, e in enumerate(events) if e == "R"]
    assert reads and min(reads) > i_w1 and max(reads) < i_w2
