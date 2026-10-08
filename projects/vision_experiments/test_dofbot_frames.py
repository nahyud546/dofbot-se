"""Đồ thị hệ tọa độ phải cho đúng những con số mà đường gắp/nhìn đang dùng."""
import unittest

import numpy as np

import cube_search_center_math as M
import dofbot_frames as D
import dofbot_ik
from cube_vision import frames as F

POSES = ([90, 125, 0, 0, 90], [60, 110, 10, 5, 90], [120, 80, 25, 15, 40], [35, 134, 0, 19, 150])
CAL = {"arm4_T_optical": M._trans((0.041, -0.001, 0.043)) @ M.MOUNT_RZ90 @ M._rot_axis((1, 0, 0), 0.05),
       "K": (935.0, 990.0, 322.0, 230.0), "k1": -0.44, "j1_scale": 1.0, "j1_offset_deg": 0.0}


class Chains(unittest.TestCase):
    def setUp(self):
        self.g = D.build(cal=CAL, external=None, phone=None)

    def test_wrist_camera_chain_matches_the_hand_eye_path_in_use(self):
        for q in POSES:
            expect = M.fk_arm4_cal(q, CAL) @ CAL["arm4_T_optical"]
            np.testing.assert_allclose(self.g.lookup("base_link", "wrist_optical", q), expect, atol=1e-12)
            np.testing.assert_allclose(self.g.lookup("world", "wrist_optical", q), expect, atol=1e-12)

    def test_tcp_matches_the_ik_forward_kinematics(self):
        for q in POSES:
            np.testing.assert_allclose(self.g.lookup("base_link", "tcp", q)[:3, 3], dofbot_ik.fk(q), atol=1e-9)

    def test_j1_correction_from_the_calibration_is_applied(self):
        cal = dict(CAL, j1_scale=1.1, j1_offset_deg=1.5)
        g = D.build(cal=cal, external=None, phone=None)
        q = [60, 110, 10, 5, 90]
        np.testing.assert_allclose(g.lookup("base_link", "wrist_optical", q),
                                   M.fk_arm4_cal(q, cal) @ cal["arm4_T_optical"], atol=1e-12)

    def test_j5_turns_the_gripper_but_not_the_camera(self):
        a, b = [90, 110, 10, 5, 30], [90, 110, 10, 5, 150]
        np.testing.assert_allclose(self.g.lookup("base_link", "wrist_optical", a),
                                   self.g.lookup("base_link", "wrist_optical", b), atol=1e-12)
        self.assertGreater(np.abs(self.g.lookup("base_link", "tcp", a) - self.g.lookup("base_link", "tcp", b)).max(),
                           1e-3)

    def test_the_two_copies_of_the_urdf_constants_agree(self):
        for name in ("J1_O", "J2_O", "J3_O", "J4_O", "J5_O"):
            np.testing.assert_allclose(getattr(M, name), getattr(dofbot_ik, name), atol=1e-12, err_msg=name)

    def test_uncalibrated_cameras_say_how_to_calibrate(self):
        self.assertFalse(self.g.available("world", "phone_optical"))
        with self.assertRaises(F.MissingTransform) as err:
            self.g.lookup("world", "ext_optical")
        self.assertIn("calibrate_external.py", str(err.exception))
        with self.assertRaises(F.MissingTransform):
            D.build(cal=None, external=None, phone=None).lookup("world", "wrist_optical", POSES[0])

    def test_camera_to_camera_goes_through_the_base(self):
        ext = type("Cal", (), {"base_T_ext": M._trans((-0.35, 0.0, 0.3)) @ M._rot_axis((0, 1, 0), 2.0),
                               "K": (930.0, 640.0, 360.0), "k1": 0.0, "image_size": (1280, 720)})()
        g = D.build(cal=CAL, external=ext, phone=None)
        q = POSES[1]
        direct = F.invert(ext.base_T_ext) @ M.fk_arm4_cal(q, CAL) @ CAL["arm4_T_optical"]
        np.testing.assert_allclose(g.lookup("ext_optical", "wrist_optical", q), direct, atol=1e-12)
        self.assertIn("inv(base_link_T_ext_optical)", g.describe("ext_optical", "wrist_optical"))

    def test_dump_lists_every_frame_and_edge(self):
        text = D.dump(self.g)
        for name in self.g.frames():
            self.assertIn(f"`{name}`", text)
        self.assertIn("CÒN THIẾU", text)


if __name__ == "__main__":
    unittest.main()
