"""Dựng world bằng tay giả lập rồi cho một camera khác tự định vị và vẽ đè: cả chuỗi trừ phần cứng."""
import unittest

import numpy as np

import active_view as A
import build_world as B
from cube_vision import camera_pose as P
from cube_vision import world_overlay as O
from cube_vision.frames import CameraModel, invert
from cube_vision.test_multiview import look_at
from test_active_view import CAL, FakeArm, spot

PHONE = CameraModel("phone", (980.0, 985.0, 362.0, 641.0), (720, 1280), k1=0.05, rotate=90)


class Session:
    def __init__(self, arm):
        self.cal, self.observe = CAL, arm.observe


class Pipeline(unittest.TestCase):
    def test_world_built_by_the_wrist_camera_is_usable_by_a_phone(self):
        first = spot(3)
        truths = {1: first, 2: first.copy(), 3: first.copy()}
        truths[2][:3, 3] += [0.03, -0.06, 0.0]
        truths[3][:3, 3] += [-0.04, 0.05, 0.03]                    # cube nằm trên cube khác (cao hơn 30 mm)
        world = B.scan(Session(FakeArm(truths)), log=lambda *_: None)
        self.assertEqual(sorted(world.tags), [1, 2, 3])
        for tag_id, truth in truths.items():
            self.assertLess(np.linalg.norm(world.tags[tag_id]["centre"] - truth[:3, 3]), 0.004, tag_id)
        self.assertGreater(world.tags[3]["centre"][2] - world.tags[1]["centre"][2], 0.024)   # z thật, không ép tầng

        phone_pose = look_at([-0.50, 0.20, 0.22], first[:3, 3])
        opt = invert(phone_pose)
        from cube_vision import multiview as MV
        seen = {i: PHONE.project((MV.tag_points() @ T[:3, :3].T + T[:3, 3]) @ opt[:3, :3].T + opt[:3, 3])
                for i, T in truths.items()}
        found = P.pose_from_tags(world.tag_corners(only_sure=False), seen, PHONE)
        self.assertIsNotNone(found["world_T_optical"], found["reasons"])
        self.assertLess(np.linalg.norm(found["world_T_optical"][:3, 3] - phone_pose[:3, 3]), 0.03)

        world.set_camera("phone", found["world_T_optical"], PHONE)
        frame = np.zeros((1280, 720, 3), np.uint8)
        view, errors = O.draw(frame, world, PHONE, found["world_T_optical"], seen)
        self.assertEqual(view.shape, frame.shape)
        self.assertGreater(int(view.sum()), 0)
        self.assertLess(max(errors.values()), 6.0)


if __name__ == "__main__":
    unittest.main()


class MergeSeen(unittest.TestCase):
    """`--watch` gộp quan sát vật của iPhone (file phụ) và của camera tay vào world."""

    def setUp(self):
        import json
        import tempfile
        import time
        from pathlib import Path
        from cube_vision import object_track as OT
        from cube_vision import test_object_track as T
        from cube_vision import world_overlay as O
        from cube_vision.world_map import WorldMap
        self.OT, self.T, self.json, self.time = OT, T, json, time
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "seen_phone.json"
        self._seen_path = O.seen_path
        O.seen_path = lambda camera, world_path=None: self.path
        self.world = WorldMap()
        self.world.set_objects([T.cup_item([-0.20, -0.10])], "wrist-scan")

    def tearDown(self):
        from cube_vision import world_overlay as O
        O.seen_path = self._seen_path
        self.tmp.cleanup()

    def phone_reports(self, centre, age=0.0, eye=None):
        obs = self.T.see(eye or self.T.PHONE, centre, target=[-0.22, 0.0, 0.03])
        self.path.write_text(self.json.dumps({"stamp": self.time.time() - age, "camera": "phone",
                                              "objects": [self.OT.to_json(obs)]}))
        return obs

    def test_the_cup_follows_what_the_phone_sees_and_keeps_its_shape(self):
        self.phone_reports([-0.27, 0.05])
        lines = B.merge_seen(self.world, [])
        self.assertEqual(len(lines), 1)
        cup = self.world.objects[0]
        self.assertLess(np.linalg.norm(cup["centre"] - [-0.27, 0.05]), 0.012)
        self.assertAlmostEqual(cup["width_m"], 0.074)
        self.assertEqual(B.merge_seen(self.world, []), [])                # đã tới nơi: không dời nữa

    def test_stale_phone_reports_and_a_missing_file_change_nothing(self):
        self.phone_reports([-0.27, 0.05], age=10.0)
        self.assertEqual(B.merge_seen(self.world, []), [])
        self.path.unlink()
        self.assertEqual(B.merge_seen(self.world, []), [])
        np.testing.assert_allclose(self.world.objects[0]["centre"], [-0.20, -0.10])

    def test_phone_and_wrist_together_intersect_their_sight_lines(self):
        truth = [-0.25, -0.02]
        phone = self.phone_reports(truth, eye=[-0.30, 0.40, 0.35])            # iPhone đứng chếch một bên
        off = dict(phone, centre=phone["centre"] + phone["bearing"] * 0.04)   # iPhone đoán khoảng cách sai 4 cm
        self.path.write_text(self.json.dumps({"stamp": self.time.time(), "camera": "phone",
                                              "objects": [self.OT.to_json(off)]}))
        wrist = dict(self.T.see(self.T.WRIST, truth), near_ok=False, side_ok=False, stamp=self.time.time())  # chân bị cắt
        lines = B.merge_seen(self.world, [wrist])
        self.assertIn("giao hai đường ngắm", lines[0])
        self.assertLess(np.linalg.norm(self.world.objects[0]["centre"] - truth), 0.01)

    def test_the_wrist_alone_moves_an_object_only_when_it_sees_it_whole(self):
        whole = dict(self.T.see(self.T.WRIST, [-0.24, -0.02]), stamp=self.time.time())
        cut = dict(whole, near_ok=False, side_ok=False)
        self.assertEqual(B.merge_seen(self.world, [cut]), [])
        self.assertEqual(len(B.merge_seen(self.world, [whole])), 1)

    def test_phone_facing_the_arm_and_wrist_take_the_middle_of_the_two_near_edges(self):
        truth = [-0.25, -0.02]
        phone = self.phone_reports(truth)                                     # iPhone đối diện tay máy
        off = dict(phone, centre=phone["centre"] + phone["bearing"] * 0.04)
        self.path.write_text(self.json.dumps({"stamp": self.time.time(), "camera": "phone",
                                              "objects": [self.OT.to_json(off)]}))
        wrist = dict(self.T.see(self.T.WRIST, truth), stamp=self.time.time())
        lines = B.merge_seen(self.world, [wrist])
        self.assertIn("hai phía đối diện", lines[0])
        self.assertLess(np.linalg.norm(self.world.objects[0]["centre"] - truth), 0.01)

