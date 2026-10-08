import math
import unittest

import cv2
import numpy as np

from cube_vision import multiview as MV
from cube_vision import view_quality as Q
from cube_vision.frames import CameraModel, invert

CAM = CameraModel("wrist", (935.0, 990.0, 322.0, 230.0), (640, 480), k1=-0.44)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """a_T_optical của camera đặt ở `eye` nhìn về `target` (z quang học hướng tới target)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, up)
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], axis=1)
    T[:3, 3] = eye
    return T


def tag_pose(centre, rvec=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.array(rvec, float))[0]
    T[:3, 3] = centre
    return T


def observe(a_T_tag, eyes, noise=0.3, seed=0, camera=CAM):
    rng = np.random.default_rng(seed)
    world = MV.tag_points() @ a_T_tag[:3, :3].T + a_T_tag[:3, 3]
    views = []
    for i, eye in enumerate(eyes):
        a_T_opt = look_at(eye, a_T_tag[:3, 3])
        opt = invert(a_T_opt)
        px = camera.project(world @ opt[:3, :3].T + opt[:3, 3]) + rng.normal(0, noise, (4, 2))
        views.append(MV.View(a_T_opt, camera, px, f"v{i}"))
    return views


FLAT = tag_pose([-0.18, 0.03, 0.058], (0, 0, 0.4))                       # tag ngửa trên bàn
UPRIGHT = tag_pose([-0.17, -0.04, 0.045], (math.radians(90), 0, 0.3))    # tag dựng đứng ở mặt bên cube
EYES = [[-0.08, 0.0, 0.23], [-0.11, 0.09, 0.21], [-0.10, -0.08, 0.22], [-0.20, 0.02, 0.24]]


class Fusion(unittest.TestCase):
    def test_several_views_recover_true_height_without_a_plane(self):
        fused = MV.solve_tag(observe(FLAT, EYES))
        self.assertLess(np.linalg.norm(fused["centre"] - FLAT[:3, 3]), 0.0015)
        self.assertGreater(float(fused["normal"] @ FLAT[:3, 2]), 0.98)
        self.assertEqual(fused["n_views"], 4)
        self.assertTrue(Q.assess_fused(fused), Q.assess_fused(fused).text())

    def test_more_views_beat_one_view_on_depth(self):
        errors = {n: [] for n in (1, 4)}
        for seed in range(12):
            for n in errors:
                fused = MV.solve_tag(observe(FLAT, EYES[:n], noise=0.6, seed=seed))
                errors[n].append(np.linalg.norm(fused["centre"] - FLAT[:3, 3]))
        self.assertLess(np.median(errors[4]), 0.5 * np.median(errors[1]))
        self.assertLess(np.median(errors[4]), 0.002)

    def test_upright_tag_is_measured_too(self):
        eyes = [[-0.10, -0.14, 0.12], [-0.20, -0.16, 0.10], [-0.14, -0.15, 0.17]]
        fused = MV.solve_tag(observe(UPRIGHT, eyes))
        self.assertLess(np.linalg.norm(fused["centre"] - UPRIGHT[:3, 3]), 0.002)
        self.assertLess(abs(float(fused["normal"][2])), 0.15)             # pháp tuyến nằm ngang
        self.assertGreater(float(fused["normal"] @ UPRIGHT[:3, 2]), 0.97)

    def test_two_different_cameras_triangulate_together(self):
        phone = CameraModel("phone", (900.0, 900.0, 360.0, 640.0), (720, 1280))
        views = observe(FLAT, EYES[:1], seed=1) + observe(FLAT, [[-0.45, 0.30, 0.20]], seed=2, camera=phone)
        fused = MV.solve_tag(views)
        self.assertLess(np.linalg.norm(fused["centre"] - FLAT[:3, 3]), 0.002)
        self.assertGreater(fused["baseline_m"], 0.3)

    def test_wrong_camera_pose_shows_up_as_disagreement(self):
        views = observe(FLAT, EYES, noise=0.2)
        views[2].a_T_optical = views[2].a_T_optical.copy()
        views[2].a_T_optical[:3, 3] += [0.012, 0.0, 0.0]                  # hand-eye/khớp sai 12 mm ở một pose
        fused = MV.solve_tag(views)
        quality = Q.assess_fused(fused)
        self.assertFalse(quality)
        self.assertIn(Q.HINT_RESHOOT, quality.hints)
        self.assertEqual(Q.worst_view(fused), 2)


class Rules(unittest.TestCase):
    def test_good_single_view_passes_but_alone_is_not_enough(self):
        view = observe(FLAT, EYES[:1], noise=0.1)[0]
        self.assertTrue(Q.assess_view(view), Q.assess_view(view).text())
        fused = Q.assess_fused(MV.solve_tag([view]))
        self.assertFalse(fused)
        self.assertEqual(fused.hints, [Q.HINT_PARALLAX])

    def test_far_tag_asks_to_come_closer(self):
        quality = Q.assess_view(observe(FLAT, [[-0.10, 0.0, 0.95]], noise=0.1)[0])
        self.assertFalse(quality)
        self.assertIn(Q.HINT_CLOSER, quality.hints)

    def test_tag_at_image_edge_asks_to_recentre(self):
        view = observe(FLAT, EYES[:1], noise=0.0)[0]
        view.corners_px = view.corners_px - (view.corners_px.min(axis=0) - 5.0)
        self.assertIn(Q.HINT_CENTRE, Q.assess_view(view).hints)

    def test_grazing_view_asks_for_a_frontal_one(self):
        quality = Q.assess_view(observe(FLAT, [[0.05, 0.03, 0.085]], noise=0.1)[0])
        self.assertIn(Q.HINT_FRONTAL, quality.hints)

    def test_outside_the_hand_eye_envelope_is_flagged(self):
        view = observe(FLAT, EYES[:1], noise=0.1)[0]
        self.assertIn(Q.HINT_IN_ENVELOPE, Q.assess_view(view, j1=165.0, j1_range=(40.0, 135.0)).hints)
        self.assertTrue(Q.assess_view(view, j1=90.0, j1_range=(40.0, 135.0)))

    def test_views_from_almost_one_spot_lack_parallax(self):
        near = [[-0.08, 0.0, 0.23], [-0.085, 0.004, 0.232]]
        quality = Q.assess_fused(MV.solve_tag(observe(FLAT, near, noise=0.2)))
        self.assertFalse(quality)
        self.assertIn(Q.HINT_PARALLAX, quality.hints)


if __name__ == "__main__":
    unittest.main()
