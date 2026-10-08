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
