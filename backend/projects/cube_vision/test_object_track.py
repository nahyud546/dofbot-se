"""Vật khác cube đi theo khi bị dời: định vị từ mặt nạ một khung, gộp hai camera như stereo."""
import unittest

import cv2
import numpy as np

from cube_vision import object_track as OT
from cube_vision.frames import CameraModel, invert
from cube_vision.test_multiview import look_at

CAM = CameraModel("sim", (600.0, 600.0, 320.0, 240.0), (640, 480))
PHONE = [-0.45, 0.05, 0.55]          # iPhone trên cao
WRIST = [-0.05, -0.02, 0.25]         # camera tay, nhìn từ đế ra


def cylinder_mask(T, centre, radius, height):
    ring = np.linspace(0, 2 * np.pi, 60, endpoint=False)
    rim = np.c_[centre[0] + radius * np.cos(ring), centre[1] + radius * np.sin(ring)]
    solid = np.r_[np.c_[rim, np.zeros(60)], np.c_[rim, np.full(60, height)]]
    inv = invert(T)
    uv = CAM.project(solid @ inv[:3, :3].T + inv[:3, 3])
    mask = np.zeros((480, 640), np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
    return mask


def cup_item(centre, radius=0.037, height=0.10):
    ring = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    return {"label": "cup", "shape": "cylinder", "centre": np.array(centre, float), "width_m": 2 * radius,
            "height_m": height, "polygon": np.array(centre) + radius * np.c_[np.cos(ring), np.sin(ring)],
            "points": np.c_[np.tile(centre, (5, 1)), np.linspace(0.03, 0.1, 5)], "stamp": 0.0,
            "area_m2": float(np.pi * radius ** 2), "n_views": 5}


def see(eye, centre, radius=0.037, height=0.10, target=None):
    T = look_at(eye, target or [centre[0], centre[1], 0.03])
    return OT.observe_mask(CAM, T, cylinder_mask(T, centre, radius, height), 0.0, "cup", 0.8)


class OneCamera(unittest.TestCase):
    def test_a_whole_object_is_located_on_the_table_from_its_mask(self):
        for eye in (PHONE, WRIST, [-0.30, 0.40, 0.35]):
            for centre in ([-0.20, -0.10], [-0.28, 0.08]):
                obs = see(eye, centre)
                self.assertTrue(obs["near_ok"] and obs["side_ok"])
                self.assertLess(np.linalg.norm(obs["centre"] - centre), 0.012, (eye, centre))
                self.assertAlmostEqual(obs["width_m"], 0.074, delta=0.012)

    def test_an_object_cut_by_the_frame_is_flagged_not_trusted(self):
        centre = [-0.20, -0.10]
        T = look_at(WRIST, [centre[0], centre[1], 0.03])
        mask = cylinder_mask(T, centre, 0.037, 0.10)
        rows = np.nonzero(mask.any(axis=1))[0]
        cut = np.zeros_like(mask)
        shift = 480 - int(rows.max()) + 30                              # đẩy vật xuống: chân vật ra ngoài ảnh
        cut[shift:] = mask[:480 - shift]
        obs = OT.observe_mask(CAM, T, cut, 0.0, "cup")
        self.assertFalse(obs["near_ok"])
        self.assertEqual(OT.match([cup_item(centre)], [obs]), {})


class Following(unittest.TestCase):
    def test_a_moved_cup_keeps_its_shape_and_follows(self):
        cup = cup_item([-0.20, -0.10])
        obs = see(PHONE, [-0.26, 0.06], target=[-0.22, 0.0, 0.03])
        objects, moved, _ = OT.relocate([cup], [obs], "phone")
        self.assertEqual(moved, [0])
        self.assertLess(np.linalg.norm(objects[0]["centre"] - [-0.26, 0.06]), 0.012)
        self.assertAlmostEqual(objects[0]["width_m"], cup["width_m"])
        np.testing.assert_allclose(objects[0]["polygon"].mean(axis=0), objects[0]["centre"], atol=1e-6)
        np.testing.assert_allclose(objects[0]["points"][:, :2], np.tile(objects[0]["centre"], (5, 1)), atol=1e-6)
        np.testing.assert_allclose(cup["centre"], [-0.20, -0.10])       # bản gốc không bị sửa

    def test_small_jitter_does_not_move_it_and_wrong_size_or_kind_is_not_matched(self):
        cup = cup_item([-0.20, -0.10])
        still = see(PHONE, [-0.205, -0.097])
        _, moved, matched = OT.relocate([cup], [still], "phone")
        self.assertEqual((moved, list(matched)), ([], [0]))
        huge = see(PHONE, [-0.25, 0.0], radius=0.12)
        self.assertEqual(OT.match([cup], [huge]), {})
        box = dict(see(PHONE, [-0.25, 0.0]), label="box")
        self.assertEqual(OT.match([cup], [box]), {})

    def test_two_objects_each_take_their_nearest_observation(self):
        a, b = cup_item([-0.20, -0.10]), cup_item([-0.30, 0.10])
        seen = [see(PHONE, [-0.31, 0.12], target=[-0.25, 0.0, 0.03]), see(PHONE, [-0.19, -0.12], target=[-0.25, 0.0, 0.03])]
        matched = OT.match([a, b], seen)
        self.assertIs(matched[0], seen[1])
        self.assertIs(matched[1], seen[0])


class Stereo(unittest.TestCase):
    def test_two_cameras_intersect_their_sight_lines(self):
        centre = [-0.24, -0.06]
        phone, wrist = see(PHONE, centre), see(WRIST, centre)
        fused, how = OT.fuse(phone, wrist)
        self.assertEqual(how, "stereo")
        self.assertLess(np.linalg.norm(fused - centre), 0.008)

    def test_a_wrong_range_from_one_camera_is_corrected_by_the_other(self):
        centre = [-0.24, -0.06]
        phone, wrist = see(PHONE, centre), see(WRIST, centre)
        wrist = dict(wrist, centre=wrist["centre"] + wrist["bearing"] * 0.05)   # camera tay đoán xa hơn 5 cm
        fused, how = OT.fuse(phone, wrist)
        self.assertEqual(how, "stereo")
        self.assertLess(np.linalg.norm(fused - centre), 0.008)

    def test_nearly_parallel_sight_lines_fall_back_to_the_average(self):
        centre = [-0.24, -0.06]
        a, b = see(WRIST, centre), see([-0.04, -0.01, 0.30], centre)
        fused, how = OT.fuse(a, b)
        self.assertEqual(how, "average")
        self.assertLess(np.linalg.norm(fused - centre), 0.015)
