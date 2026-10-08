"""Vết đáy của vật bất kỳ từ nhiều góc nhìn, bằng camera giả lập nhìn một khối trụ."""
import unittest

import cv2
import numpy as np

from cube_vision import carve
from cube_vision.frames import CameraModel, invert
from cube_vision.test_multiview import look_at

CAM = CameraModel("sim", (600.0, 600.0, 320.0, 240.0), (640, 480))
BOUNDS = ((-0.35, -0.05), (-0.15, 0.15))


def silhouette(a_T_optical, centre, radius, height):
    """Mặt nạ của một khối trụ đứng trên mặt bàn z = 0 nhìn từ camera này."""
    angles = np.linspace(0, 2 * np.pi, 60, endpoint=False)
    ring = np.c_[centre[0] + radius * np.cos(angles), centre[1] + radius * np.sin(angles)]
    points = np.r_[np.c_[ring, np.zeros(60)], np.c_[ring, np.full(60, height)]]
    T = invert(a_T_optical)
    uv = CAM.project(points @ T[:3, :3].T + T[:3, 3])
    mask = np.zeros((480, 640), np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
    return mask


def views_of(centre, radius, height, eyes):
    out = []
    for eye in eyes:
        T = look_at(eye, [centre[0], centre[1], 0.0])
        out.append((CAM, T, silhouette(T, centre, radius, height)))
    return out


AROUND = [[-0.20 + 0.25 * np.cos(a), 0.25 * np.sin(a), 0.30] for a in np.radians([0, 90, 180, 270])]


class Footprints(unittest.TestCase):
    def test_views_from_all_around_recover_the_base_of_a_cylinder(self):
        found = carve.footprints(views_of([-0.20, 0.02], 0.035, 0.09, AROUND), 0.0, BOUNDS)
        self.assertEqual(len(found), 1)
        self.assertLess(np.linalg.norm(found[0]["centre"] - [-0.20, 0.02]), 0.008)
        true_area = np.pi * 0.035 ** 2
        self.assertGreater(found[0]["area_m2"], 0.8 * true_area)
        self.assertLess(found[0]["area_m2"], 1.8 * true_area)

    def test_views_from_one_side_only_overestimate_but_still_contain_the_base(self):
        one_side = [[-0.02, y, 0.25] for y in (-0.10, 0.0, 0.10)]
        found = carve.footprints(views_of([-0.20, 0.0], 0.035, 0.09, one_side), 0.0, BOUNDS)
        self.assertEqual(len(found), 1)
        poly = (found[0]["polygon"] * 1000).astype(np.float32).reshape(-1, 1, 2)
        self.assertGreaterEqual(cv2.pointPolygonTest(poly, (-200.0, 0.0), False), 0)
        self.assertGreater(found[0]["area_m2"], np.pi * 0.035 ** 2)

    def test_one_view_is_not_enough_and_empty_masks_give_nothing(self):
        views = views_of([-0.20, 0.0], 0.035, 0.09, AROUND)
        self.assertEqual(carve.footprints(views[:1], 0.0, BOUNDS), [])
        blank = [(c, T, np.zeros_like(m)) for c, T, m in views]
        self.assertEqual(carve.footprints(blank, 0.0, BOUNDS), [])

    def test_one_view_that_missed_the_object_does_not_erase_it(self):
        views = views_of([-0.20, 0.0], 0.035, 0.09, AROUND)
        views[0] = (views[0][0], views[0][1], np.zeros_like(views[0][2]))
        self.assertEqual(len(carve.footprints(views, 0.0, BOUNDS)), 1)

    def test_known_cubes_are_kept_out(self):
        views = views_of([-0.20, 0.0], 0.02, 0.03, AROUND)
        self.assertEqual(carve.footprints(views, 0.0, BOUNDS, keep_out=[(-0.20, 0.0)], keep_out_m=0.04), [])
