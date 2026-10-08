"""World tự cập nhật từ camera tay ở pose bất kỳ, chạy với tay giả lập (hình học hand-eye thật)."""
import unittest

import numpy as np

import active_view as A
import calibrate_hand_eye as C
import world_watch as W
from cube_vision import camera_pose as P
from cube_vision.test_multiview import look_at, tag_pose
from cube_vision.test_world import PHONE
from cube_vision.test_live_world import corners_of, see
from test_active_view import CAL, FakeArm

# Pose tùy ý trong vùng hand-eye, KHÔNG phải READY và không thuộc bộ pose quét.
POSES = ([72.0, 118.0, 6.0, 3.0, 90.0], [97.0, 104.0, 14.0, 8.0, 40.0], [113.0, 121.0, 2.0, 0.0, 150.0],
         [84.0, 96.0, 20.0, 5.0, 90.0], [64.0, 128.0, 0.0, 0.0, 90.0], [122.0, 112.0, 8.0, 2.0, 90.0])


def visible_spots(pose):
    """Các vị trí trên bàn mà camera tay thấy rõ ở pose này."""
    found = []
    for x in np.arange(-0.27, -0.10, 0.01):
        for y in np.arange(-0.14, 0.141, 0.02):
            truth = tag_pose([x, y, 0.0578], (0, 0, 0.3))
            _, seen = FakeArm({1: truth}, noise=0.0).observe(pose)
            if 1 in seen and seen[1].min() > 90 and seen[1][:, 0].max() < 550 and seen[1][:, 1].max() < 390:
                found.append(truth)
    if not found:
        raise unittest.SkipTest("pose này không nhìn thấy chỗ nào phù hợp")
    return found


def visible_spot(pose, k=0, far_from=()):
    """Một vị trí thấy rõ ở pose này; có `far_from` thì lấy vị trí xa các điểm đó nhất."""
    found = visible_spots(pose)
    if far_from:
        return max(found, key=lambda T: min(np.linalg.norm(T[:3, 3] - o[:3, 3]) for o in far_from))
    return found[(len(found) // 2 + 7 * k) % len(found)]


def in_view(truth, pose, margin=-40.0):
    cam = (np.linalg.inv(A.base_T_optical(pose, CAL)) @ np.r_[truth[:3, 3], 1.0])[:3]
    if cam[2] <= 0.05:
        return False
    u, v = A.wrist_camera(CAL).project(cam.reshape(1, 3))[0]
    return margin <= u <= 640 - margin and margin <= v <= 480 - margin


class Watch(unittest.TestCase):
    def test_a_cube_seen_once_from_an_arbitrary_pose_lands_in_the_world(self):
        for pose in POSES[:4]:
            truth = visible_spot(pose)
            watcher = W.Watcher(CAL)
            events = watcher.observe(*FakeArm({5: truth}).observe(pose))
            self.assertEqual(events["new"], [5], pose)
            tag = watcher.world.tags[5]
            self.assertTrue(tag["sure"])
            self.assertLess(np.linalg.norm(tag["centre"] - truth[:3, 3]), 0.003, pose)
            self.assertEqual(tag["source"], W.SOURCE)

    def test_views_from_different_poses_are_fused_not_duplicated(self):
        truth = visible_spot(POSES[0])
        watcher, arm = W.Watcher(CAL), FakeArm({2: truth})
        used = 0
        for pose in POSES:
            real, seen = arm.observe(pose)
            used += 2 in watcher.observe(real, seen)["used"]
        self.assertGreaterEqual(used, 2)
        self.assertGreaterEqual(watcher.world.tags[2]["n_views"], 2)
        self.assertLess(np.linalg.norm(watcher.world.tags[2]["centre"] - truth[:3, 3]), 0.003)
        again = len(watcher.views[2])
        watcher.observe(*arm.observe(POSES[0]))                    # nhìn lại đúng pose cũ: không thêm góc nhìn
        self.assertEqual(len(watcher.views[2]), again)

    def test_a_moved_cube_replaces_its_old_position(self):
        pose = POSES[0]
        first, second = visible_spot(pose, 0), visible_spot(pose, 3)
        self.assertGreater(np.linalg.norm(first[:3, 3] - second[:3, 3]), 0.02)
        watcher = W.Watcher(CAL)
        watcher.observe(*FakeArm({3: first}).observe(pose))
        events = watcher.observe(*FakeArm({3: second}, seed=1).observe(pose))
        self.assertEqual(events["moved"], [3])
        self.assertLess(np.linalg.norm(watcher.world.tags[3]["centre"] - second[:3, 3]), 0.003)
        self.assertEqual(len(watcher.views[3]), 1)

    def test_a_cube_taken_away_is_removed_only_after_repeated_misses_in_view(self):
        pose = POSES[0]
        truth = visible_spot(pose)
        watcher = W.Watcher(CAL)
        watcher.observe(*FakeArm({4: truth}).observe(pose))
        empty = FakeArm({})
        blurry = [watcher.observe(*empty.observe(pose), stamp=100.0 + 5 * k)["removed"] for k in range(5)]
        self.assertEqual(blurry, [[]] * 5)                          # không thấy gì + ảnh không đáng tin: không xóa
        self.assertIn(4, watcher.world.tags)
        # Ảnh rõ nhưng mới mất ít giây (tay người che lúc sắp xếp): chưa xóa, dù đã nhiều lần nhìn.
        quick = [watcher.observe(*empty.observe(pose), stamp=200.0 + 0.2 * k, clear_view=True)["removed"]
                 for k in range(10)]
        self.assertEqual(quick, [[]] * 10)
        self.assertEqual(watcher.expected, {4})
        late = watcher.observe(*empty.observe(pose), stamp=200.0 + W.MISSING_AFTER_S + 0.1, clear_view=True)
        self.assertEqual(late["removed"], [4])
        self.assertNotIn(4, watcher.world.tags)

    def test_a_hand_covering_a_cube_briefly_does_not_remove_it(self):
        pose = POSES[0]
        truth = visible_spot(pose)
        watcher, arm = W.Watcher(CAL), FakeArm({4: truth})
        watcher.observe(*arm.observe(pose), stamp=0.0)
        for k in range(8):                                          # bị che ~2,4 giây
            watcher.observe(*FakeArm({}).observe(pose), stamp=0.3 * (k + 1), clear_view=True)
        self.assertIn(4, watcher.world.tags)
        events = watcher.observe(*arm.observe(pose), stamp=3.0)     # thấy lại: chuỗi "mất" bị xóa sạch
        self.assertEqual(events["used"], [4])
        self.assertEqual(watcher.unseen, {})
        for k in range(8):
            watcher.observe(*FakeArm({}).observe(pose), stamp=3.3 + 0.3 * k, clear_view=True)
        self.assertIn(4, watcher.world.tags)

    def test_seeing_another_cube_proves_the_view_is_clear(self):
        pose = POSES[0]
        spots = visible_spots(pose)
        first = spots[0]
        other = max(spots, key=lambda T: np.linalg.norm(T[:3, 3] - first[:3, 3]))
        watcher = W.Watcher(CAL)
        watcher.observe(*FakeArm({1: first, 2: other}).observe(pose))
        only_other = FakeArm({2: other})
        removed = [watcher.observe(*only_other.observe(pose), stamp=10.0 + 2.5 * k)["removed"] for k in range(3)]
        self.assertEqual(removed[-1], [1])                          # thấy cube 2 rõ mà cube 1 không còn: đã lấy đi
        self.assertIn(2, watcher.world.tags)

    def test_a_cube_outside_the_current_view_is_kept(self):
        truth = visible_spot(POSES[4])
        watcher = W.Watcher(CAL)
        watcher.observe(*FakeArm({6: truth}).observe(POSES[4]))
        elsewhere = next((p for p in ([130.0, 125.0, 0.0, 0.0, 90.0], [45.0, 125.0, 0.0, 0.0, 90.0],
                                      [130.0, 100.0, 10.0, 0.0, 90.0]) if not in_view(truth, p)), None)
        if elsewhere is None:                                      # quay sang phía khác: cube ra khỏi khung nhìn
            self.skipTest("không có pose nào đưa cube ra khỏi khung nhìn")
        for k in range(W.MISSING_AFTER + 2):
            self.assertEqual(watcher.observe(*FakeArm({}).observe(elsewhere), stamp=50.0 + 5 * k,
                                             clear_view=True)["removed"], [])
        self.assertIn(6, watcher.world.tags)

    def test_looks_outside_the_hand_eye_envelope_are_not_recorded(self):
        watcher = W.Watcher(CAL)
        events = watcher.observe([160.0, 120.0, 0.0, 0.0, 90.0], {1: np.array([[300, 200], [340, 200], [340, 240], [300, 240.0]])})
        self.assertIn("ngoài vùng hand-eye", events["skipped"])
        self.assertEqual(watcher.world.tags, {})
        self.assertIn("J1=160", W.describe(events))

    def test_world_built_while_watching_lets_a_phone_locate_itself(self):
        watcher = W.Watcher(CAL)
        truths = {}
        for tag_id, pose in zip((1, 2, 3), (POSES[0], POSES[2], POSES[3])):
            truths[tag_id] = visible_spot(pose, far_from=list(truths.values()))
        if min(np.linalg.norm(truths[a][:3, 3] - truths[b][:3, 3]) for a in truths for b in truths if a < b) < 0.04:
            self.skipTest("các vị trí thử quá gần nhau")
        arm = FakeArm(truths)
        for pose in POSES:
            watcher.observe(*arm.observe(pose))
        self.assertEqual(sorted(watcher.world.tags), [1, 2, 3])
        middle = np.mean([T[:3, 3] for T in truths.values()], axis=0)
        phone_pose = look_at(middle + [-0.30, 0.18, 0.38], middle)
        seen = see(truths, phone_pose)
        self.assertEqual(sorted(seen), [1, 2, 3])
        found = P.pose_from_tags(watcher.world.tag_corners(), seen, PHONE)
        self.assertIsNotNone(found["world_T_optical"], found["reasons"])
        self.assertLess(np.linalg.norm(found["world_T_optical"][:3, 3] - phone_pose[:3, 3]), 0.03)


if __name__ == "__main__":
    unittest.main()


class Timing(unittest.TestCase):
    """Ghép khung ảnh với góc khớp đọc ở luồng riêng: chỉ nhận khung khi tay đứng yên quanh thời điểm đó."""

    def session(self, readings):
        import collections
        s = A.WristSession.__new__(A.WristSession)
        s._readings = collections.deque(readings)
        return s

    def test_frame_between_two_equal_readings_is_accepted(self):
        s = self.session([(0.0, 0.4, [90, 125, 0, 0, 90]), (0.5, 0.9, [90.4, 125, 0, 0, 90])])
        self.assertEqual(s.joints_at(0.45), [90.2, 125.0, 0.0, 0.0, 90.0])
        self.assertEqual(s.latest_joints(), [90.4, 125, 0, 0, 90])

    def test_frame_taken_while_the_arm_moved_is_rejected(self):
        s = self.session([(0.0, 0.4, [90, 125, 0, 0, 90]), (0.5, 0.9, [96, 125, 0, 0, 90])])
        self.assertIs(s.joints_at(0.45), False)

    def test_frame_newer_than_every_reading_waits(self):
        s = self.session([(0.0, 0.4, [90, 125, 0, 0, 90])])
        self.assertIsNone(s.joints_at(0.45))
        self.assertIs(self.session([(0.5, 0.9, [90, 125, 0, 0, 90])]).joints_at(0.45), False)
        self.assertIsNone(self.session([]).latest_joints())

    def test_only_tags_steady_across_every_recent_frame_count(self):
        quad = np.array([[100, 100], [140, 100], [140, 140], [100, 140.0]])
        recent = [{1: quad, 2: quad + 300}, {1: quad + 0.5, 2: quad + 310}, {1: quad - 0.4}]
        steady = A.stable_tags(recent)
        self.assertEqual(sorted(steady), [1])
        np.testing.assert_allclose(steady[1], quad + (0.5 - 0.4) / 3, atol=1e-9)
        self.assertEqual(A.stable_tags([]), {})
