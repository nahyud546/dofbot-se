import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from cube_vision import camera_pose as P
from cube_vision import multiview as MV
from cube_vision.frames import CameraModel, invert
from cube_vision.test_multiview import look_at, tag_pose
from cube_vision.world_map import CUBE_EDGE_M, WorldMap, cube_from_tag

PHONE = CameraModel("phone", (980.0, 985.0, 362.0, 641.0), (720, 1280), k1=0.08, k2=-0.1, rotate=90)
TAGS = {1: tag_pose([-0.16, 0.05, 0.058], (0, 0, 0.3)), 2: tag_pose([-0.21, -0.04, 0.058], (0, 0, -0.6)),
        3: tag_pose([-0.14, -0.08, 0.088], (0, 0, 1.0)), 4: tag_pose([-0.24, 0.07, 0.045], (1.5, 0, 0.2))}
WORLD = {i: MV.tag_points() @ T[:3, :3].T + T[:3, 3] for i, T in TAGS.items()}
TRUE_POSE = look_at([-0.48, 0.22, 0.24], [-0.19, 0.0, 0.05])


def detect(pose=TRUE_POSE, noise=0.3, seed=0, ids=None):
    rng = np.random.default_rng(seed)
    opt = invert(pose)
    return {i: PHONE.project(WORLD[i] @ opt[:3, :3].T + opt[:3, 3]) + rng.normal(0, noise, (4, 2))
            for i in (ids or WORLD)}


class CameraPose(unittest.TestCase):
    def test_camera_is_located_from_tags_the_world_already_knows(self):
        result = P.pose_from_tags(WORLD, detect(), PHONE)
        self.assertTrue(result["ok"], result["reasons"])
        self.assertLess(np.linalg.norm(result["world_T_optical"][:3, 3] - TRUE_POSE[:3, 3]), 0.01)
        self.assertEqual(set(result["leave_one_out_px"]), {1, 2, 3, 4})

    def test_a_moved_handheld_camera_is_relocated_each_frame(self):
        elsewhere = look_at([-0.30, -0.35, 0.30], [-0.19, 0.0, 0.05])
        result = P.pose_from_tags(WORLD, detect(elsewhere, seed=4), PHONE)
        self.assertTrue(result["ok"], result["reasons"])
        self.assertLess(np.linalg.norm(result["world_T_optical"][:3, 3] - elsewhere[:3, 3]), 0.01)
        self.assertGreater(P.drift_px(WORLD, detect(elsewhere), PHONE, TRUE_POSE), 50.0)
        self.assertLess(P.drift_px(WORLD, detect(), PHONE, TRUE_POSE), 1.5)

    def test_too_few_or_unknown_tags_give_no_pose_instead_of_a_guess(self):
        one = P.pose_from_tags(WORLD, detect(ids=[2]), PHONE)
        self.assertFalse(one["ok"])
        self.assertIsNone(one["world_T_optical"])
        self.assertFalse(P.pose_from_tags({}, detect(), PHONE)["ok"])
        self.assertIsNone(P.drift_px({}, detect(), PHONE, TRUE_POSE))

    def test_a_tag_whose_world_position_is_wrong_is_caught(self):
        wrong = dict(WORLD)
        wrong[3] = WORLD[3] + [0.03, 0.0, 0.0]                           # cube bị dời 3 cm sau khi dựng world
        result = P.pose_from_tags(wrong, detect(noise=0.2), PHONE)
        self.assertFalse(result["ok"])


class World(unittest.TestCase):
    def fused(self, T):
        corners = MV.tag_points() @ T[:3, :3].T + T[:3, 3]
        return {"corners": corners, "centre": T[:3, 3], "normal": T[:3, 2], "pos_std_m": [0.0004] * 3,
                "n_views": 3, "rms_px": 0.6}

    def test_cube_sits_half_an_edge_behind_its_tag(self):
        cube = cube_from_tag(WORLD[1])
        np.testing.assert_allclose(cube["centre"], TAGS[1][:3, 3] - [0, 0, CUBE_EDGE_M / 2], atol=1e-9)
        self.assertAlmostEqual(cube["vertices"][:, 2].max(), 0.058, places=6)
        self.assertAlmostEqual(cube["vertices"][:, 2].min(), 0.028, places=6)
        side = cube_from_tag(WORLD[4])                                   # tag dựng đứng: cube lùi theo phương ngang
        self.assertAlmostEqual(side["centre"][2], TAGS[4][2, 3], delta=0.002)

    def test_round_trip_through_the_file_keeps_everything(self):
        world = WorldMap(table_z=0.028)
        for i, T in TAGS.items():
            world.update_tag(i, self.fused(T), "wrist")
        world.set_zone(3, (-0.105, 0.164), "zone_survey")
        world.set_camera("phone", TRUE_POSE, PHONE)
        with tempfile.TemporaryDirectory() as tmp:
            path = world.save(Path(tmp) / "w.json")
            again = WorldMap.load(path)
            self.assertEqual(WorldMap.load(Path(tmp) / "missing.json").tags, {})
        np.testing.assert_allclose(again.tags[4]["corners"], WORLD[4], atol=1e-12)
        np.testing.assert_allclose(again.cameras["phone"]["world_T_optical"], TRUE_POSE, atol=1e-12)
        self.assertEqual(again.camera_model("phone").rotate, 90)
        self.assertEqual(again.zones[3]["xy"], [-0.105, 0.164])
        self.assertEqual(again.table_z, 0.028)

    def test_another_camera_sees_the_world_where_the_tags_really_are(self):
        world = WorldMap(table_z=0.028)
        for i, T in TAGS.items():
            world.update_tag(i, self.fused(T), "wrist")
        drawn = world.project_into(PHONE, TRUE_POSE)
        seen = detect(noise=0.0)
        for i in TAGS:
            np.testing.assert_allclose(drawn["tags"][i], seen[i], atol=1e-6)
            self.assertEqual(len(drawn["cubes"][i]), 12)
        self.assertEqual(len(drawn["axes"]), 3)

    def test_unsure_result_does_not_overwrite_a_fresh_sure_one_and_old_entries_go_stale(self):
        world = WorldMap()
        world.update_tag(1, self.fused(TAGS[1]), "wrist", sure=True, stamp=100.0)
        world.update_tag(1, self.fused(TAGS[2]), "phone", sure=False, stamp=102.0)
        np.testing.assert_allclose(world.tags[1]["centre"], TAGS[1][:3, 3])
        world.update_tag(1, self.fused(TAGS[2]), "phone", sure=False, stamp=200.0)
        np.testing.assert_allclose(world.tags[1]["centre"], TAGS[2][:3, 3])
        self.assertEqual(world.tag_corners(), {})
        self.assertEqual(world.stale(50.0, now=300.0), [1])
        world.forget(1)
        self.assertEqual(world.tags, {})


if __name__ == "__main__":
    unittest.main()
