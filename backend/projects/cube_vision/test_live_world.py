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


class SingleFrame(unittest.TestCase):
    """Cube dời đo từ MỘT khung của iPhone: nằm phẳng thì không được lật/nghiêng dù góc tag nhiễu."""

    def locate(self, truth, noise, seed):
        return locate_tag(see({1: truth}, TRUE_POSE, noise=noise, seed=seed)[1], PHONE, TRUE_POSE, 0.0578)

    def test_flat_cube_stays_flat_and_accurate_under_blur_like_noise(self):
        for seed in range(25):
            rng = np.random.default_rng(seed)
            truth = tag_pose([rng.uniform(-0.25, -0.13), rng.uniform(-0.09, 0.09), 0.058], (0, 0, rng.uniform(0, 6.2)))
            fused = self.locate(truth, noise=2.0, seed=seed)
            self.assertTrue(fused["flat"], seed)
            np.testing.assert_allclose(fused["normal"], [0, 0, 1])
            self.assertLess(np.linalg.norm(fused["centre"] - truth[:3, 3]), 0.006, seed)

    def test_the_layer_of_a_stacked_cube_is_recovered(self):
        for layer in (1, 2):
            truth = tag_pose([-0.17, 0.01, 0.0578 + 0.03 * layer], (0, 0, 0.8))
            fused = self.locate(truth, noise=0.6, seed=layer)
            self.assertEqual(fused["layer"], layer)
            self.assertLess(np.linalg.norm(fused["centre"] - truth[:3, 3]), 0.004)

    def test_a_cube_really_tilted_in_the_hand_keeps_its_tilt(self):
        truth = tag_pose([-0.17, 0.0, 0.08], (0.0, 0.5, 0.3))             # nghiêng ~29° và đang nâng lên
        fused = self.locate(truth, noise=0.4, seed=1)
        self.assertFalse(fused["flat"])
        tilt = np.degrees(np.arccos(fused["normal"][2]))
        self.assertAlmostEqual(tilt, np.degrees(np.arccos(truth[2, 2])), delta=6.0)

    def test_an_upright_tag_facing_the_camera_is_not_forced_flat(self):
        z_dir = np.array([-0.96, -0.2, 0.0])                                # mặt tag hướng về phía iPhone
        z_dir = z_dir / np.linalg.norm(z_dir)
        x_dir = np.cross([0.0, 0.0, 1.0], z_dir)
        x_dir = x_dir / np.linalg.norm(x_dir)
        upright = np.eye(4)
        upright[:3, :3] = np.stack([x_dir, np.cross(z_dir, x_dir), z_dir], axis=1)
        upright[:3, 3] = [-0.20, 0.0, 0.05]
        fused = self.locate(upright, noise=0.4, seed=2)
        self.assertFalse(fused["flat"])
        # Nhìn gần chính diện thì một khung không phân biệt được độ nghiêng chính xác (đây là giới hạn của PnP một
        # góc nhìn): chỉ đòi hỏi nó KHÔNG bị ép phẳng và vẫn xa phương ngang vài chục độ.
        self.assertLess(abs(fused["normal"][2]), 0.7)
        self.assertLess(np.linalg.norm(fused["centre"] - upright[:3, 3]), 0.012)

    def test_a_tag_facing_away_from_the_camera_is_not_measured(self):
        away = tag_pose([-0.20, 0.0, 0.05], (0.0, 0.0, 0.0))
        away[:3, :3] = away[:3, :3] @ np.diag([1.0, -1.0, -1.0])            # lật mặt xuống: camera không thể thấy
        self.assertIsNone(self.locate(away, noise=0.3, seed=1)) if see({1: away}, TRUE_POSE) else None

    def test_flat_model_rejects_a_tag_that_is_not_on_any_layer(self):
        from cube_vision import multiview as MV
        view = MV.View(TRUE_POSE, PHONE, see({1: tag_pose([-0.17, 0.0, 0.10])}, TRUE_POSE, noise=0.2)[1])
        fits = [MV.flat_tag(view, 0.0578 + 0.03 * n) for n in range(4)]
        self.assertGreater(min(f["rms_px"] for f in fits), 0.5)           # 100 mm: lơ lửng giữa hai tầng


class Track(unittest.TestCase):
    FLAT = {1: tag_pose([-0.223, -0.060, 0.0578], (0, 0, 0.2)), 2: tag_pose([-0.146, -0.057, 0.0578], (0, 0, -0.4)),
            3: tag_pose([-0.191, 0.060, 0.0578], (0, 0, 1.1)), 4: tag_pose([-0.188, -0.009, 0.0578], (0, 0, 0.7))}

    def test_tracking_escapes_the_flipped_pose_of_coplanar_tags(self):
        # Như đo thật khi cầm iPhone đi vòng: khung trước đã rơi vào nghiệm lật; khung này phải quay về nghiệm thật.
        world = {i: corners_of(T) for i, T in self.FLAT.items()}
        for eye, ids in (([0.166, -0.314, 0.201], [2, 4]), ([0.173, 0.198, 0.232], [2, 3, 4]),
                         ([-0.43, -0.03, 0.154], [1, 2, 4])):
            true_pose = look_at(eye, [-0.18, 0.0, 0.05])
            seen = see(self.FLAT, true_pose, ids=ids, noise=0.5, seed=4)
            flipped = invert(P.flip_start(invert(true_pose), np.vstack([world[i] for i in seen])))
            self.assertGreater(np.linalg.norm(flipped[:3, 3] - true_pose[:3, 3]), 0.2)
            result = P.track(world, seen, PHONE, prev=flipped)
            self.assertTrue(result["ok"], result["reasons"])
            self.assertLess(np.linalg.norm(result["world_T_optical"][:3, 3] - true_pose[:3, 3]), 0.03, eye)

    def test_a_correct_previous_pose_is_not_abandoned_for_its_flip(self):
        world = {i: corners_of(T) for i, T in self.FLAT.items()}
        for seed in range(10):
            true_pose = look_at([-0.43, -0.03, 0.154], [-0.18, 0.0, 0.05])
            result = P.track(world, see(self.FLAT, true_pose, noise=1.0, seed=seed), PHONE, prev=true_pose)
            self.assertLess(np.linalg.norm(result["world_T_optical"][:3, 3] - true_pose[:3, 3]), 0.03, seed)

    def test_non_coplanar_points_have_no_flip(self):
        stacked = np.vstack([corners_of(self.FLAT[1]), corners_of(tag_pose([-0.15, 0.0, 0.12]))])
        self.assertIsNone(P.flip_start(np.eye(4), stacked))
        self.assertIsNotNone(P.flip_start(np.eye(4), np.vstack([corners_of(T) for T in self.FLAT.values()])))

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


class ColourFaces(unittest.TestCase):
    def test_solid_colour_patches_become_quads_whatever_their_aspect_and_edge_patches_are_dropped(self):
        import cv2
        from cube_vision import color, registry
        image = np.full((480, 640, 3), 235, np.uint8)
        blue_side = np.array([[430, 372], [565, 368], [578, 420], [432, 424]], np.int32)       # mặt bên nhìn chéo: dẹt
        green_top = np.array([[100, 290], [215, 260], [270, 350], [150, 395]], np.int32)
        cut_yellow = np.array([[0, 100], [60, 100], [60, 160], [0, 160]], np.int32)            # chạm mép ảnh
        hsv = np.full((480, 640, 3), (0, 0, 235), np.uint8)
        for quad, colour in ((blue_side, (116, 146, 121)), (green_top, (43, 122, 48)), (cut_yellow, (22, 255, 226))):
            cv2.fillConvexPoly(hsv, quad, colour)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        found = {f["cube_id"]: f for f in color.colour_faces(image, registry.COLOR_TO_ID)}
        self.assertEqual(sorted(found), sorted([registry.COLOR_TO_ID["khoi_xanh"], registry.COLOR_TO_ID["khoi_xanh_duong"]]))
        got = found[registry.COLOR_TO_ID["khoi_xanh_duong"]]["quad"]
        for corner in blue_side:
            self.assertLess(np.min(np.linalg.norm(got - corner, axis=1)), 4.0)


class LiveCarriesEverything(unittest.TestCase):
    def test_live_world_keeps_objects_and_region_for_the_phone_overlay(self):
        world, seen = build_world(), see(TAGS, TRUE_POSE)
        world.region = [[-0.4, -0.3], [0.0, -0.3], [0.0, 0.3], [-0.4, 0.3]]
        world.set_objects([{"label": "cup", "polygon": np.array([[-0.2, 0.0], [-0.15, 0.0], [-0.15, 0.05], [-0.2, 0.05]]),
                            "centre": np.array([-0.175, 0.025]), "area_m2": 0.0025, "n_views": 5, "height_m": 0.1,
                            "width_m": 0.05, "shape": "cylinder"}], "test")
        info = LiveWorld(world, PHONE).update(seen)
        self.assertTrue(info["ok"])
        self.assertEqual(len(info["world"].objects), 1)
        self.assertEqual(info["world"].region, world.region)
