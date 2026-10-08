import time
import unittest

import numpy as np

from cube_vision import camera_pose as P
from cube_vision import multiview as MV
from cube_vision.frames import invert
from cube_vision.live_world import LiveWorld, locate_tag
from cube_vision.test_multiview import look_at, tag_pose
from cube_vision.test_world import PHONE, TAGS, TRUE_POSE
from cube_vision.world_map import WorldMap

TABLE_Z = 0.0278


def corners_of(T):
    return MV.tag_points() @ T[:3, :3].T + T[:3, 3]


def see(poses, camera_pose, ids=None, noise=0.3, seed=0):
    rng = np.random.default_rng(seed)
    opt = invert(camera_pose)
    out = {}
    for i, T in poses.items():
        if ids is not None and i not in ids:
            continue
        c = corners_of(T) @ opt[:3, :3].T + opt[:3, 3]
        px = PHONE.project(c) + rng.normal(0, noise, (4, 2))
        if np.isfinite(px).all() and 0 <= px[:, 0].min() and px[:, 0].max() < 720 and 0 <= px[:, 1].min() \
                and px[:, 1].max() < 1280:
            out[i] = px
    return out


def build_world():
    world = WorldMap(table_z=TABLE_Z)
    for i, T in TAGS.items():
        world.update_tag(i, {"corners": corners_of(T), "centre": T[:3, 3], "normal": T[:3, 2],
                             "pos_std_m": [0.0005] * 3, "n_views": 3, "rms_px": 0.5}, "wrist", stamp=1.0)
    world.set_camera("phone", TRUE_POSE, PHONE)
    return world


class Live(unittest.TestCase):
    def test_nothing_changed_means_no_moved_and_no_missing(self):
        live = LiveWorld(build_world(), PHONE)
        info = live.update(see(TAGS, TRUE_POSE))
        self.assertTrue(info["ok"], info["reasons"])
        self.assertEqual((info["moved"], info["new"], info["missing"]), ([], [], []))
        self.assertEqual(sorted(info["anchors"]), [1, 2, 3, 4])

    def test_phone_moving_between_frames_is_followed_quickly(self):
        live = LiveWorld(build_world(), PHONE)
        live.update(see(TAGS, TRUE_POSE))
        for k, eye in enumerate(([-0.46, 0.18, 0.24], [-0.43, 0.12, 0.26], [-0.40, 0.02, 0.28], [-0.42, -0.10, 0.25])):
            pose = look_at(eye, [-0.19, 0.0, 0.05])
            started = time.time()
            info = live.update(see(TAGS, pose, seed=k))
            took = time.time() - started
            self.assertTrue(info["ok"], info["reasons"])
            self.assertLess(np.linalg.norm(info["pose"][:3, 3] - pose[:3, 3]), 0.012, eye)
            self.assertEqual((info["moved"], info["new"]), ([], []))
            self.assertLess(took, 0.4)

    def test_a_moved_cube_is_found_at_its_new_place_and_not_used_as_a_landmark(self):
        world = build_world()
        live = LiveWorld(world, PHONE)
        live.update(see(TAGS, TRUE_POSE))
        moved_to = dict(TAGS)
        moved_to[1] = tag_pose([-0.12, 0.10, 0.058], (0, 0, 0.9))             # cube 1 bị nhấc sang chỗ khác
        info = live.update(see(moved_to, TRUE_POSE, seed=3))
        self.assertTrue(info["ok"], info["reasons"])
        self.assertEqual(info["moved"], [1])
        self.assertNotIn(1, info["anchors"])
        centre = info["world"].tags[1]["centre"]
        self.assertLess(np.linalg.norm(centre - moved_to[1][:3, 3]), 0.012)
        self.assertEqual(info["world"].tags[1]["source"], "phone-live")
        np.testing.assert_allclose(world.tags[1]["centre"], TAGS[1][:3, 3])      # world gốc không bị đổi
        self.assertLess(np.linalg.norm(info["pose"][:3, 3] - TRUE_POSE[:3, 3]), 0.012)

    def test_a_cube_moved_far_does_not_drag_the_camera_estimate_off(self):
        # Cube dời xa (như thật: nhấc sang chỗ khác cách ~10 cm): nghiệm trên TẤT CẢ tag sai nặng, vẫn phải tách được.
        for new_xy in ((-0.10, 0.10), (-0.26, -0.10), (-0.12, -0.08)):
            live = LiveWorld(build_world(), PHONE)
            live.update(see(TAGS, TRUE_POSE))
            moved_to = dict(TAGS)
            moved_to[4] = tag_pose([new_xy[0], new_xy[1], 0.058], (0, 0, -0.5))
            started = time.time()
            info = live.update(see(moved_to, TRUE_POSE, seed=8))
            self.assertTrue(info["ok"], (new_xy, info["reasons"]))
            self.assertEqual(info["moved"], [4], new_xy)
            self.assertEqual(sorted(info["anchors"]), [1, 2, 3])
            self.assertLess(np.linalg.norm(info["pose"][:3, 3] - TRUE_POSE[:3, 3]), 0.012)
            self.assertLess(time.time() - started, 0.4)               # không rơi vào tìm toàn cục

    def test_phone_jumping_far_between_frames_is_found_again(self):
        live = LiveWorld(build_world(), PHONE)
        live.update(see(TAGS, TRUE_POSE))
        far = look_at([-0.30, -0.36, 0.30], [-0.19, 0.0, 0.05])
        info = live.update(see(TAGS, far, seed=2))
        self.assertTrue(info["ok"], info["reasons"])
        self.assertLess(np.linalg.norm(info["pose"][:3, 3] - far[:3, 3]), 0.012)

    def test_phone_and_a_cube_moving_together_still_work_with_enough_still_cubes(self):
        live = LiveWorld(build_world(), PHONE)
        live.update(see(TAGS, TRUE_POSE))
        pose = look_at([-0.42, -0.12, 0.27], [-0.19, 0.0, 0.05])
        moved_to = dict(TAGS)
        moved_to[2] = tag_pose([-0.205, -0.02, 0.058], (0, 0, 0.7))
        info = live.update(see(moved_to, pose, seed=5))
        self.assertTrue(info["ok"], info["reasons"])
        self.assertEqual(info["moved"], [2])
        self.assertLess(np.linalg.norm(info["pose"][:3, 3] - pose[:3, 3]), 0.012)

    def test_new_cube_is_added_and_a_cube_taken_away_is_reported_missing(self):
        live = LiveWorld(build_world(), PHONE)
        seen = see(TAGS, TRUE_POSE, ids=[1, 2, 4])                               # cube 3 bị lấy khỏi bàn
        seen.update(see({7: tag_pose([-0.11, -0.02, 0.058])}, TRUE_POSE, seed=2))
        info = live.update(seen)
        self.assertTrue(info["ok"], info["reasons"])
        self.assertEqual(info["new"], [7])
        self.assertEqual(info["missing"], [3])
        self.assertIn(7, info["world"].tags)

    def test_too_few_still_tags_gives_no_pose_instead_of_following_a_moved_cube(self):
        live = LiveWorld(build_world(), PHONE)
        live.update(see(TAGS, TRUE_POSE))
        info = live.update(see(TAGS, TRUE_POSE, ids=[2]))
        self.assertFalse(info["ok"])
        self.assertIsNone(info["pose"])
        two = {1: TAGS[1], 2: tag_pose([-0.10, 0.12, 0.058])}                     # hai tag, một cái đã dời: không biết cái nào
        info = live.update(see(two, TRUE_POSE, seed=1))
        self.assertFalse(info["ok"])
        self.assertIsNone(info["pose"])

    def test_locating_a_flat_tag_from_one_frame_snaps_to_the_layer(self):
        truth = tag_pose([-0.15, 0.05, 0.058], (0, 0, 0.4))
        fused = locate_tag(see({1: truth}, TRUE_POSE)[1], PHONE, TRUE_POSE, 0.0578)
        self.assertEqual(fused["layer"], 0)
        self.assertAlmostEqual(fused["centre"][2], 0.0578, places=6)
        self.assertLess(np.linalg.norm(fused["centre"][:2] - truth[:2, 3]), 0.008)


class Track(unittest.TestCase):
    def test_warm_start_is_much_faster_than_a_global_search(self):
        world = {i: corners_of(T) for i, T in TAGS.items()}
        seen = see(TAGS, TRUE_POSE)
        t0 = time.time()
        cold = P.track(world, seen, PHONE)
        cold_s = time.time() - t0
        t0 = time.time()
        warm = P.track(world, seen, PHONE, prev=cold["world_T_optical"])
        warm_s = time.time() - t0
        self.assertTrue(cold["ok"] and warm["ok"])
        self.assertLess(warm_s, 0.5 * cold_s)
        self.assertLess(warm_s, 0.1)


if __name__ == "__main__":
    unittest.main()
