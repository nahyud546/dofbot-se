import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from cube_vision import intrinsics as I
from cube_vision.frames import CameraModel

BOARD = I.Board(cols=8, rows=5, square_m=0.035)
SIZE = (1280, 720)
TRUE = CameraModel("true", (905.0, 912.0, 652.0, 371.0), SIZE, k1=-0.12, k2=0.04)


def views(count=20, noise=0.25, seed=0, spread=True):
    """Bảng đặt ở nhiều vị trí/độ nghiêng trước một camera biết trước; góc khuất khỏi ảnh bị bỏ."""
    rng = np.random.default_rng(seed)
    world = BOARD.corner_points()
    centre = world.mean(axis=0)
    out = []
    while len(out) < count:
        tilt = rng.uniform(-0.6, 0.6, 2) if spread else rng.uniform(-0.03, 0.03, 2)
        R = cv2.Rodrigues(np.array([tilt[0], tilt[1], rng.uniform(-0.3, 0.3)]))[0]
        offset = np.array([rng.uniform(-0.16, 0.16), rng.uniform(-0.08, 0.08), rng.uniform(0.3, 0.55)]) if spread \
            else np.array([0.0, 0.0, 0.45])
        cam = (world - centre) @ R.T + offset
        px = TRUE.project(cam) + rng.normal(0, noise, (len(world), 2))
        keep = (px[:, 0] > 5) & (px[:, 0] < SIZE[0] - 5) & (px[:, 1] > 5) & (px[:, 1] < SIZE[1] - 5)
        if keep.sum() >= I.MIN_CORNERS:
            out.append((np.flatnonzero(keep), px[keep]))
    return out


class Solve(unittest.TestCase):
    def test_recovers_a_known_lens(self):
        result = I.solve(views(), BOARD, SIZE)
        self.assertTrue(result["intrinsics_accepted"], result["intrinsics_reasons"])
        fx, fy, cx, cy = result["K"]
        self.assertAlmostEqual(fx, TRUE.K[0], delta=6)
        self.assertAlmostEqual(fy, TRUE.K[1], delta=6)
        self.assertAlmostEqual(cx, TRUE.K[2], delta=6)
        self.assertAlmostEqual(cy, TRUE.K[3], delta=6)
        self.assertAlmostEqual(result["k1"], TRUE.k1, delta=0.02)
        self.assertLess(result["intrinsics_rms_px"], 0.6)

    def test_frontal_views_from_one_spot_are_rejected(self):
        result = I.solve(views(spread=False), BOARD, SIZE)
        self.assertFalse(result["intrinsics_accepted"])
        text = " ".join(result["intrinsics_reasons"])
        self.assertTrue("vùng ảnh" in text or "nghiêng" in text, text)

    def test_too_few_views_are_rejected(self):
        self.assertFalse(I.solve(views(count=6), BOARD, SIZE)["intrinsics_accepted"])
        with self.assertRaises(ValueError):
            I.solve(views(count=2), BOARD, SIZE)

    def test_noisy_detections_fail_the_rms_gate(self):
        result = I.solve(views(noise=3.0), BOARD, SIZE)
        self.assertFalse(result["intrinsics_accepted"])


class BoardAndStore(unittest.TestCase):
    def test_rendered_board_is_detected_with_matching_corner_ids(self):
        img = BOARD.image(px_per_square=120, margin_px=40)
        ids, pts = I.detect(img, BOARD)
        self.assertGreaterEqual(len(ids), 20)
        # Góc ảnh phải là ảnh phẳng (homography) của tọa độ bảng theo đúng id, sai số dưới 1,5 px.
        plane = BOARD.corner_points()[ids, :2].astype(np.float32)
        H, _ = cv2.findHomography(plane, pts.astype(np.float32))
        back = cv2.perspectiveTransform(plane.reshape(-1, 1, 2), H).reshape(-1, 2)
        self.assertLess(np.abs(back - pts).max(), 1.5)
        self.assertAlmostEqual(np.linalg.norm(pts[1] - pts[0]), 120.0, delta=2.0)
        self.assertEqual(len(I.detect(np.full((300, 400), 128, np.uint8), BOARD)[0]), 0)

    def test_coverage_and_new_view_rules(self):
        pts = np.array([[10, 10], [1270, 710], [640, 360]])
        self.assertEqual(I.cells(pts, SIZE), {(0, 0), (2, 2), (1, 1)})
        a = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], float)
        self.assertFalse(I.is_new_view(a + 3, [a], SIZE))
        self.assertTrue(I.is_new_view(a + 300, [a], SIZE))
        self.assertTrue(I.is_new_view((a - 150) * 2.5 + 150, [a], SIZE))

    def test_camera_file_merges_pose_and_intrinsics(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "phone.json"
            I.save_camera("phone", {"base_T_optical": np.eye(4).ravel().tolist(), "pose_accepted": True}, path)
            I.save_camera("phone", {"K": [900, 905, 640, 360], "k1": -0.1, "k2": 0.0, "image_size": [720, 1280],
                                    "rotate": 90, "intrinsics_accepted": True}, path)
            data = I.load_camera("phone", path)
            self.assertTrue(data["pose_accepted"])
            model = I.model_from(data, "phone")
            self.assertEqual((model.rotate, model.image_size), (90, (720, 1280)))
            self.assertIsNone(I.model_from(dict(data, intrinsics_accepted=False)))
            self.assertIsNone(I.load_camera("phone", Path(tmp) / "none.json"))


if __name__ == "__main__":
    unittest.main()
