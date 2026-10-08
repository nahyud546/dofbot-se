import math
import unittest

import cv2
import numpy as np

from cube_vision import frames as F


def T(xyz=(0, 0, 0), rvec=(0, 0, 0)):
    out = np.eye(4)
    out[:3, :3] = cv2.Rodrigues(np.array(rvec, float))[0]
    out[:3, 3] = xyz
    return out


class Graph(unittest.TestCase):
    def setUp(self):
        self.g = F.FrameGraph()
        self.a_T_b = T((1, 0, 0), (0, 0, 0.3))
        self.c_T_d = T((0, 0.5, 0), (0.2, 0, 0))
        self.g.add("a", "b", self.a_T_b)
        self.g.add("b", "c", fn=lambda q: T((0, 0, q[0]), (0, q[1], 0)))
        self.g.add("c", "d", self.c_T_d)
        self.g.add("a", "e", None, how="chạy hiệu chuẩn e")

    def test_composes_left_to_right_and_inverts_when_walking_up(self):
        q = (0.2, 0.4)
        b_T_c = T((0, 0, 0.2), (0, 0.4, 0))
        np.testing.assert_allclose(self.g.lookup("a", "d", q), self.a_T_b @ b_T_c @ self.c_T_d, atol=1e-12)
        np.testing.assert_allclose(self.g.lookup("d", "a", q), np.linalg.inv(self.a_T_b @ b_T_c @ self.c_T_d),
                                   atol=1e-12)
        np.testing.assert_allclose(self.g.lookup("a", "a"), np.eye(4))
        self.assertEqual(self.g.describe("d", "b"), "inv(c_T_d) · inv(b_T_c(q))")

    def test_point_convention(self):
        p_b = np.array([0.1, 0.2, 0.3, 1.0])
        np.testing.assert_allclose(self.g.lookup("a", "b") @ p_b, self.a_T_b @ p_b)

    def test_missing_pieces_are_reported_not_guessed(self):
        with self.assertRaisesRegex(F.MissingTransform, "cần truyền q"):
            self.g.lookup("a", "d")
        with self.assertRaisesRegex(F.MissingTransform, "chạy hiệu chuẩn e"):
            self.g.lookup("a", "e")
        with self.assertRaisesRegex(F.MissingTransform, "không có hệ"):
            self.g.lookup("a", "zzz")
        self.g.frame("island", "")
        with self.assertRaisesRegex(F.MissingTransform, "không nối"):
            self.g.lookup("a", "island")
        self.assertFalse(self.g.available("d", "e"))
        self.assertTrue(self.g.available("d", "a"))

    def test_rejects_duplicates_and_non_rigid_matrices(self):
        with self.assertRaises(ValueError):
            self.g.add("b", "a", np.eye(4))
        bad = np.eye(4)
        bad[0, 0] = 2.0
        with self.assertRaises(ValueError):
            self.g.add("d", "f", bad)


class Camera(unittest.TestCase):
    cam = F.CameraModel("t", (930.0, 940.0, 620.0, 380.0), (1280, 720), k1=-0.2, k2=0.05)

    def test_projection_matches_opencv(self):
        pts = np.array([[0.05, -0.02, 0.4], [-0.1, 0.08, 0.6], [0.0, 0.0, 0.3]])
        ours = self.cam.project(pts)
        theirs = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), self.cam.matrix(), self.cam.dist())[0].reshape(-1, 2)
        np.testing.assert_allclose(ours, theirs, atol=1e-6)
        self.assertTrue(np.isnan(self.cam.project([[0, 0, -1.0]])).all())

    def test_ray_inverts_projection(self):
        point = np.array([0.07, -0.05, 0.5])
        u, v = self.cam.project(point)[0]
        np.testing.assert_allclose(self.cam.ray(u, v), point / np.linalg.norm(point), atol=1e-7)

    def test_pixel_to_plane_recovers_a_point_on_a_known_height(self):
        a_T_opt = T((-0.35, 0.0, 0.3), (0, math.radians(140), 0))
        point = np.array([-0.15, 0.04, 0.058])
        u, v = self.cam.project((F.invert(a_T_opt) @ np.r_[point, 1])[:3])[0]
        np.testing.assert_allclose(F.pixel_to_plane(self.cam, a_T_opt, u, v, 0.058), point, atol=1e-6)
        self.assertIsNone(F.pixel_to_plane(self.cam, np.eye(4), 620, 380, -1.0))

    def test_orient_turns_the_raw_stream_clockwise(self):
        raw = np.arange(6, dtype=np.uint8).reshape(2, 3)             # [[0 1 2], [3 4 5]]
        turned = F.CameraModel("p", (1, 1, 0, 0), (2, 3), rotate=90).orient(raw)
        np.testing.assert_array_equal(turned, [[3, 0], [4, 1], [5, 2]])
        self.assertIs(self.cam.orient(raw), raw)

    def test_rpy_round_trip(self):
        r, p, y = F.rpy_deg(T(rvec=(0, 0, math.radians(30))))
        self.assertAlmostEqual(y, 30.0, places=6)
        self.assertAlmostEqual(r, 0.0, places=6)
        self.assertAlmostEqual(p, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
