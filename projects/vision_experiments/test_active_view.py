"""Vòng nhìn quanh, chạy với tay máy + camera giả lập (hình học thật của hand-eye)."""
import math
import unittest

import cv2
import numpy as np

import active_view as A
import calibrate_hand_eye as C
import cube_search_center_math as M
from cube_vision import multiview as MV
from cube_vision import view_quality as Q
from cube_vision.frames import invert

# Bản sao hand-eye thật (2026-10-06) để hình học trong test giống tay thật; không đọc file cấu hình.
MOUNT = np.array([[-0.0311152, -0.9993733, 0.0168748, 0.0412829], [0.9991986, -0.0315262, -0.0246642, -0.0009115],
                  [0.0251807, 0.0160939, 0.9995534, 0.0432369], [0.0, 0.0, 0.0, 1.0]])
MOUNT[:3, :3] = cv2.Rodrigues(cv2.Rodrigues(MOUNT[:3, :3])[0])[0]          # trực chuẩn lại sau khi làm tròn
CAL = {"arm4_T_optical": MOUNT, "K": (935.26, 990.32, 322.57, 229.68), "k1": -0.444,
       "j1_scale": 1.0, "j1_offset_deg": 0.0, "j1_valid_range": (40.0, 135.0), "tag_top_z": 0.0578}


def tag_pose(centre, rvec=(0.0, 0.0, 0.3)):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.array(rvec, float))[0]
    T[:3, 3] = centre
    return T


class FakeArm:
    """observe(servo): chiếu các tag thật qua đúng chuỗi FK + hand-eye, thêm nhiễu pixel; ghi lại các pose đã đi."""

    def __init__(self, tags, noise=0.3, seed=0, pose_error=None):
        self.tags, self.noise, self.rng = tags, noise, np.random.default_rng(seed)
        self.pose_error = pose_error or {}
        self.visited = []

    def observe(self, servo):
        self.visited.append(list(servo))
        cam = A.wrist_camera(CAL)
        T = A.base_T_optical(servo, CAL)
        if len(self.visited) in self.pose_error:                 # camera thật lệch khỏi chỗ FK nói
            T = T.copy()
            T[:3, 3] += self.pose_error[len(self.visited)]
        seen = {}
        for tag_id, base_T_tag in self.tags.items():
            world = MV.tag_points() @ base_T_tag[:3, :3].T + base_T_tag[:3, 3]
            opt = invert(T)
            pts = world @ opt[:3, :3].T + opt[:3, 3]
            if pts[:, 2].min() <= 0.05:
                continue
            px = cam.project(pts) + self.rng.normal(0, self.noise, (4, 2))
            if px.min() >= 8 and px[:, 0].max() <= 632 and px[:, 1].max() <= 472:
                seen[tag_id] = px
        return servo, seen


def spot(lo, hi=99):
    """Vị trí trên bàn mà lượt quét cố định cho số khung dùng được nằm trong [lo, hi]."""
    for x in np.arange(-0.27, -0.10, 0.01):
        for y in np.arange(-0.16, 0.161, 0.02):
            truth = tag_pose([x, y, 0.058])
            got = A.measure(FakeArm({9: truth}).observe, CAL, look_around=False, log=lambda *_: None)
            n = got[9]["fused"]["n_views"] if 9 in got and got[9]["fused"] else 0
            if lo <= n <= hi:
                return truth
    raise unittest.SkipTest(f"không có vị trí cho {lo}–{hi} khung với bộ pose quét hiện tại")


class Loop(unittest.TestCase):
    def test_tags_seen_from_several_scan_poses_need_no_extra_views(self):
        truth = spot(3)
        arm = FakeArm({1: truth})
        result = A.measure(arm.observe, CAL, log=lambda *_: None)[1]
        self.assertTrue(result["quality"], result["quality"].text())
        self.assertEqual(result["extra"], 0)
        self.assertEqual(len(arm.visited), len(A.SCAN_POSES))
        self.assertLess(np.linalg.norm(result["fused"]["centre"] - truth[:3, 3]), 0.002)

    def test_tag_seen_once_triggers_looking_around_until_it_is_sure(self):
        truth = spot(1, 1)
        plain = A.measure(FakeArm({7: truth}).observe, CAL, look_around=False, log=lambda *_: None)
        self.assertFalse(plain[7]["quality"])
        arm = FakeArm({7: truth})
        log = []
        result = A.measure(arm.observe, CAL, log=log.append)[7]
        self.assertGreaterEqual(result["extra"], 1)
        self.assertGreater(len(arm.visited), len(A.SCAN_POSES))
        self.assertTrue(result["quality"], result["quality"].text())
        self.assertLess(np.linalg.norm(result["fused"]["centre"] - truth[:3, 3]), 0.003)
        self.assertTrue(any("nhìn thêm" in line for line in log))
        for servo in arm.visited:                                 # mọi pose nhìn thêm đều an toàn và trong vùng hand-eye
            self.assertGreaterEqual(C.tip_z(servo), C.MIN_TIP_Z)
            self.assertTrue(40.0 <= servo[0] <= 135.0)

    def test_without_look_around_it_reports_unsure_instead_of_guessing(self):
        result = A.measure(FakeArm({3: spot(1, 1)}).observe, CAL, look_around=False, log=lambda *_: None)[3]
        self.assertFalse(result["quality"])
        self.assertIn("CHƯA CHẮC", A.describe(3, result))

    def test_one_bad_pose_is_dropped_not_averaged_in(self):
        truth = spot(3)
        clean = A.measure(FakeArm({2: truth}, noise=0.2).observe, CAL, look_around=False, log=lambda *_: None)[2]
        bad_label = clean["views"][1].label                       # làm lệch đúng pose quét này 15 mm
        index = int(bad_label.split()[1])
        arm = FakeArm({2: truth}, noise=0.2, pose_error={index: np.array([0.015, 0.0, 0.0])})
        result = A.measure(arm.observe, CAL, look_around=False, log=lambda *_: None)[2]
        self.assertTrue(result["quality"], result["quality"].text())
        self.assertNotIn(bad_label, [v.label for v in result["views"]])
        self.assertLess(np.linalg.norm(result["fused"]["centre"] - truth[:3, 3]), 0.002)


class Chooser(unittest.TestCase):
    target = np.array([-0.19, 0.02, 0.058])

    def test_next_view_moves_the_camera_and_keeps_the_target_in_frame(self):
        used = [A.READY]
        servo = A.next_view(self.target, used, CAL)
        self.assertIsNotNone(servo)
        gap = np.linalg.norm(A.base_T_optical(servo, CAL)[:3, 3] - A.base_T_optical(A.READY, CAL)[:3, 3])
        self.assertGreaterEqual(gap, A.MIN_NEW_BASELINE_M)
        fx, fy, cx, cy = CAL["K"]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        u, v, _ = C.predict_pixel(servo, self.target, CAL["arm4_T_optical"], K)
        self.assertTrue(95 <= u <= 545 and 80 <= v <= 400)

    def test_closer_hint_shortens_the_range(self):
        plain = A.next_view(self.target, [A.READY], CAL)
        closer = A.next_view(self.target, [A.READY], CAL, hints=[Q.HINT_CLOSER])
        dist = lambda s: np.linalg.norm(A.base_T_optical(s, CAL)[:3, 3] - self.target)  # noqa: E731
        self.assertLessEqual(dist(closer), dist(plain) + 1e-9)

    def test_no_candidates_left_returns_none(self):
        self.assertIsNone(A.next_view(self.target, [A.READY], CAL, pool=[A.READY]))
        self.assertIsNone(A.next_view([0.5, 0.5, 0.0], [A.READY], CAL))


if __name__ == "__main__":
    unittest.main()
