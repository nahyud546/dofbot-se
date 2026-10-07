import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cube_vision import faces  # noqa: E402

PRIOR = faces.SidePrior(90, 165, "test")


def synthetic(face=(100, 60, 120), part=(30, 55), table=90):
    image = np.full((260, 360, 3), table, np.uint8)
    x, y, side = face
    cv2.rectangle(image, (x, y), (x + side, y + side), (240, 240, 240), -1)
    px, py = x + part[0], y + part[1]
    cv2.rectangle(image, (px, py), (px + 30, py + 30), (20, 20, 20), -1)
    anchor = np.float32([[px, py], [px + 30, py], [px + 30, py + 30], [px, py + 30]])
    truth = np.float32([[x, y], [x + side, y], [x + side, y + side], [x, y + side]])
    return image, anchor, truth


class FakeTrash:
    """Chấm nhãn theo kích thước crop gốc là không thể; giả lập: mặt đủ => 'book' điểm cao."""

    def __init__(self):
        self.calls = []

    def match_many(self, crops, refine=True):
        self.calls.append(len(crops))
        out = []
        for crop in crops:
            white = float(np.mean(crop.min(axis=2) > 200))
            out.append(("book", 0.5 + 0.3 * white, {"margin": 0.1 + 0.1 * white}))
        return out


class TestPrior(unittest.TestCase):
    def test_geometry_prior_matches_pinhole(self):
        # Camera cao 0.30 m nhìn thẳng xuống, mặt trên cách 0.27 m: cạnh ~ fx*0.03/0.27.
        T = np.eye(4)
        T[:3, :3] = np.diag([1, -1, -1])
        T[:3, 3] = [0, 0, 0.30]
        K = np.array([[900.0, 0, 320], [0, 900.0, 240], [0, 0, 1]])
        prior = faces.expected_side_prior(T, K, 0.03, layers=[0])
        self.assertEqual(prior.source, "geometry")
        self.assertAlmostEqual(prior.nominal, 900 * 0.03 / 0.27, delta=6)
        self.assertLess(prior.lo, 100 * 1.0)
        self.assertGreater(prior.hi, 100)

    def test_looking_sideways_falls_back(self):
        prior = faces.expected_side_prior(np.eye(4), np.eye(3) * 900, 0.03)
        self.assertEqual(prior.source, "fallback")

    def test_roles(self):
        face = np.float32([[0, 0], [120, 0], [120, 120], [0, 120]])
        part = np.float32([[0, 0], [30, 0], [30, 30], [0, 30]])
        dust = np.float32([[0, 0], [8, 0], [8, 8], [0, 8]])
        self.assertEqual(faces.classify_quad(face, PRIOR), faces.FACE)
        self.assertEqual(faces.classify_quad(part, PRIOR), faces.PART)
        self.assertEqual(faces.classify_quad(dust, PRIOR), faces.NOISE)


class TestGrow(unittest.TestCase):
    def test_print_patch_grows_to_whole_face(self):
        image, anchor, truth = synthetic()
        grown = faces.grow_face(image, anchor, PRIOR)
        self.assertIsNotNone(grown)
        self.assertGreater(faces._overlap(grown[0], truth, image.shape[:2]), 0.6)

    def test_face_touching_image_edge_still_grows(self):
        image, anchor, truth = synthetic(face=(240, 60, 120), part=(80, 55))
        grown = faces.grow_face(image, anchor, PRIOR)
        self.assertIsNotNone(grown)

    def test_no_face_around_isolated_speck(self):
        image = np.full((260, 360, 3), 90, np.uint8)
        cv2.rectangle(image, (100, 100), (130, 130), (20, 20, 20), -1)
        anchor = np.float32([[100, 100], [130, 100], [130, 130], [100, 130]])
        self.assertIsNone(faces.grow_face(image, anchor, PRIOR))


class TestPropose(unittest.TestCase):
    def test_part_becomes_one_full_face_proposal(self):
        image, anchor, truth = synthetic()
        trash = FakeTrash()
        out = faces.propose_faces(image, [anchor], PRIOR, trash)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].source, "grown")
        self.assertGreater(faces._overlap(out[0].quad, truth, image.shape[:2]), 0.6)

    def test_speck_without_face_makes_no_object(self):
        image = np.full((260, 360, 3), 90, np.uint8)
        cv2.rectangle(image, (100, 100), (130, 130), (20, 20, 20), -1)
        anchor = np.float32([[100, 100], [130, 100], [130, 130], [100, 130]])
        self.assertEqual(faces.propose_faces(image, [anchor], PRIOR, FakeTrash()), [])

    def test_noise_quads_never_reach_dino(self):
        image, _, _ = synthetic()
        dust = np.float32([[10, 10], [18, 10], [18, 18], [10, 18]])
        trash = FakeTrash()
        self.assertEqual(faces.propose_faces(image, [dust], PRIOR, trash), [])
        self.assertEqual(trash.calls, [])

    def test_merge_duplicate_ids_keeps_best(self):
        keep, merged = faces.merge_duplicate_ids([
            {"cube_id": 3, "rank": 1.0}, {"cube_id": 3, "rank": 2.0}, {"cube_id": 1, "rank": 0.1}])
        self.assertEqual(sorted(o["cube_id"] for o in keep), [1, 3])
        self.assertEqual(next(o for o in keep if o["cube_id"] == 3)["rank"], 2.0)
        self.assertEqual(len(merged), 1)


if __name__ == "__main__":
    unittest.main()


class TestSnap(unittest.TestCase):
    def test_noisy_square_projection_is_made_consistent(self):
        K = np.array([[900.0, 0, 320], [0, 900.0, 240], [0, 0, 1]])
        h = 0.015
        obj = np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]], np.float64)
        rvec = np.array([0.5, 0.2, 0.1])
        tvec = np.array([0.0, 0.0, 0.25])
        pts = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))[0].reshape(-1, 2)
        noisy = pts + np.array([[1.5, -1], [-1, 1.2], [0.8, 1], [-1.2, -0.8]])
        snapped = faces.snap_to_square_projection(noisy, K)
        self.assertIsNotNone(snapped)
        self.assertLess(np.max(np.linalg.norm(snapped - faces.order_quad(pts), axis=1)), 3.0)

    def test_implausible_quad_is_rejected(self):
        K = np.array([[900.0, 0, 320], [0, 900.0, 240], [0, 0, 1]])
        bad = np.float32([[100, 100], [300, 110], [310, 130], [90, 300]])
        self.assertIsNone(faces.snap_to_square_projection(bad, K))
