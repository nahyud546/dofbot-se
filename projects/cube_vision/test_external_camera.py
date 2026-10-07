"""external_camera: camera tổng hợp biết trước -> bộ giải phải tìm lại; thiếu dữ liệu thì từ chối."""
import math
import unittest

import numpy as np

from cube_vision import external_camera as X

SIZE = (1280, 720)
TOP_Z = 0.0578


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """base_T_ext của camera đặt tại eye nhìn về target (trục quang +z, ảnh y xuống)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], axis=1)
    T[:3, 3] = eye
    return T


TRUE_K, TRUE_K1 = (980.0, 655.0, 345.0), -0.08
TRUE_T = look_at([-0.62, 0.02, 0.42], [-0.18, 0.0, 0.03])


def tag_corners(x, y, layer, yaw=0.0):
    s = X.TAG_SIZE_M / 2
    c, n = math.cos(yaw), math.sin(yaw)
    z = TOP_Z + X.CUBE_EDGE_M * layer
    return np.array([[x + c * dx - n * dy, y + n * dx + c * dy, z]
                     for dx, dy in ((-s, -s), (s, -s), (s, s), (-s, s))])


def dataset(layers=(0, 0, 0, 0, 1, 1, 2, 2, 0, 0, 1, 3), noise_px=0.4, seed=0, T=TRUE_T):
    rng = np.random.default_rng(seed)
    points, pixels, groups = [], [], []
    for i, layer in enumerate(layers):
        x, y = -0.12 - 0.16 * rng.random(), -0.12 + 0.24 * rng.random()
        corners = tag_corners(x, y, layer, yaw=rng.uniform(-0.7, 0.7))
        px = X.project_points(corners, TRUE_K, TRUE_K1, T) + rng.normal(0, noise_px, (4, 2))
        points += list(corners)
        pixels += list(px)
        groups += [i] * 4
    return np.array(points), np.array(pixels), groups


class Model(unittest.TestCase):
    def test_projection_matches_the_wrist_camera_model(self):
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))
        import cube_layer as L
        point_cam = np.array([0.03, -0.02, 0.4])
        base_T_ext = np.eye(4)
        ours = X.project_points(point_cam, TRUE_K, TRUE_K1, base_T_ext)[0]
        theirs = L._project(point_cam, (TRUE_K[0], TRUE_K[0], TRUE_K[1], TRUE_K[2]), TRUE_K1)
        np.testing.assert_allclose(ours, theirs, atol=1e-9)

    def test_pixel_to_plane_inverts_projection(self):
        point = np.array([-0.2, 0.05, 0.09])
        u, v = X.project_points(point, TRUE_K, TRUE_K1, TRUE_T)[0]
        np.testing.assert_allclose(X.pixel_to_plane(u, v, point[2], TRUE_K, TRUE_K1, TRUE_T), point, atol=1e-6)

    def test_ray_that_never_reaches_the_plane_is_none(self):
        self.assertIsNone(X.pixel_to_plane(640, 10, 5.0, TRUE_K, TRUE_K1, TRUE_T))


class Solver(unittest.TestCase):
    def test_recovers_a_known_camera_from_noisy_multi_height_tags(self):
        points, pixels, groups = dataset()
        result = X.solve(points, pixels, SIZE, groups)
        self.assertTrue(result["accepted"], result["reasons"])
        self.assertLess(result["fit_rms_px"], 1.5)
        self.assertLess(np.median(result["holdout_m"]), 0.003)
        self.assertAlmostEqual(result["K"][0], TRUE_K[0], delta=0.06 * TRUE_K[0])
        T = np.array(result["base_T_ext"])
        self.assertLess(np.linalg.norm(T[:3, 3] - TRUE_T[:3, 3]), 0.03)

    def test_metric_use_is_accurate_even_when_focal_and_distance_trade_off(self):
        points, pixels, groups = dataset(noise_px=0.6, seed=3)
        result = X.solve(points, pixels, SIZE, groups)
        fresh = tag_corners(-0.21, 0.07, 1)
        px = X.project_points(fresh, TRUE_K, TRUE_K1, TRUE_T)
        errors = X.plane_errors_m(fresh, px, tuple(result["K"]), result["k1"], np.array(result["base_T_ext"]))
        self.assertLess(errors.max(), 0.004)

    def test_single_height_is_not_accepted(self):
        points, pixels, groups = dataset(layers=(0,) * 12)
        result = X.solve(points, pixels, SIZE, groups)
        self.assertFalse(result["accepted"])
        self.assertTrue(any("độ cao" in r for r in result["reasons"]))

    def test_too_few_points_is_not_accepted(self):
        points, pixels, groups = dataset(layers=(0, 1, 2))
        self.assertFalse(X.solve(points, pixels, SIZE, groups)["accepted"])

    def test_gross_outliers_do_not_break_the_fit(self):
        points, pixels, groups = dataset()
        pixels[5] += 60.0
        pixels[22] -= 45.0
        result = X.solve(points, pixels, SIZE, groups)
        T = np.array(result["base_T_ext"])
        self.assertLess(np.linalg.norm(T[:3, 3] - TRUE_T[:3, 3]), 0.05)

    def test_bad_input_raises(self):
        with self.assertRaises(ValueError):
            X.solve(np.zeros((3, 3)), np.zeros((3, 2)), SIZE)


class Runtime(unittest.TestCase):
    def calibration(self):
        points, pixels, groups = dataset()
        return X.build(X.solve(points, pixels, SIZE, groups), TOP_Z,
                       landmarks={1: [200.0, 300.0], 2: [1000.0, 310.0], 3: [150.0, 420.0]})

    def test_locate_tag_gives_xy_and_yaw_at_a_known_layer(self):
        cal = self.calibration()
        corners = tag_corners(-0.19, 0.04, 2, yaw=0.3)
        px = X.project_points(corners, TRUE_K, TRUE_K1, TRUE_T)
        found = cal.locate_tag(px, layer=2)
        self.assertAlmostEqual(found["x"], -0.19, delta=0.004)
        self.assertAlmostEqual(found["y"], 0.04, delta=0.004)
        self.assertAlmostEqual(found["yaw_rad"], 0.3, delta=0.08)
        self.assertTrue(found["layer_confident"])

    def test_wrong_layer_assumption_shifts_xy_so_the_layer_must_be_known(self):
        cal = self.calibration()
        px = X.project_points(tag_corners(-0.19, 0.04, 2), TRUE_K, TRUE_K1, TRUE_T)
        right, wrong = cal.locate_tag(px, layer=2), cal.locate_tag(px, layer=0)
        self.assertGreater(math.hypot(right["x"] - wrong["x"], right["y"] - wrong["y"]), 0.02)

    def test_save_load_roundtrip_and_unaccepted_is_not_loaded(self):
        import tempfile
        from pathlib import Path
        cal = self.calibration()
        with tempfile.TemporaryDirectory() as tmp:
            path = cal.save(Path(tmp) / "ext.json")
            again = X.ExternalCalibration.load(path)
            np.testing.assert_allclose(again.base_T_ext, cal.base_T_ext)
            self.assertEqual(again.landmarks[2], [1000.0, 310.0])
            cal.accepted = False
            cal.save(path)
            self.assertIsNone(X.ExternalCalibration.load(path))
            self.assertIsNotNone(X.ExternalCalibration.load(path, require_accepted=False))
            self.assertIsNone(X.ExternalCalibration.load(Path(tmp) / "missing.json"))

    def test_relocalize_recovers_a_moved_camera_keeping_the_lens(self):
        cal = self.calibration()
        moved = look_at([-0.66, -0.05, 0.47], [-0.18, 0.0, 0.03])
        points, pixels, _ = dataset(layers=(0, 1, 2, 0), seed=7, T=moved)
        T, rms = X.relocalize(points, pixels, cal)
        self.assertLess(rms, 1.5)
        self.assertLess(np.linalg.norm(T[:3, 3] - moved[:3, 3]), 0.03)

    def test_wrong_image_size_is_refused(self):
        cal = self.calibration()
        with self.assertRaises(ValueError):
            cal.locate_tags(np.zeros((480, 640, 3), np.uint8))

    def test_drift_is_measured_against_the_stored_pad_centres(self):
        cal = self.calibration()
        frame = np.zeros((720, 1280, 3), np.uint8)
        original = X.pad_landmarks
        try:
            X.pad_landmarks = lambda f: {1: [203.0, 304.0], 2: [1003.0, 314.0], 3: [153.0, 424.0]}
            self.assertAlmostEqual(cal.drift_px(frame), 5.0, delta=0.01)
            self.assertFalse(cal.moved(frame))
            X.pad_landmarks = lambda f: {1: [240.0, 330.0], 2: [1040.0, 340.0], 3: [190.0, 450.0]}
            self.assertTrue(cal.moved(frame))
            X.pad_landmarks = lambda f: {1: [240.0, 330.0]}
            self.assertIsNone(cal.drift_px(frame))                  # chỉ một ô: không đủ để kết luận
        finally:
            X.pad_landmarks = original

    def test_locator_reports_cube_xy_and_refuses_a_moved_camera_or_missing_tag(self):
        cal = self.calibration()
        truth = tag_corners(-0.2, 0.05, 0)
        px = X.project_points(truth, TRUE_K, TRUE_K1, TRUE_T)

        class Camera:
            frame = np.zeros((720, 1280, 3), np.uint8)

            def grab(self):
                return self.frame

        class Detector:
            def detect(self, frame):
                return [{"id": 3, "cube_id": 3, "corners": px}]

        original_landmarks, original_tags = X.pad_landmarks, X.ExternalCalibration.locate_tags
        try:
            X.pad_landmarks = lambda f: dict(cal.landmarks)
            X.ExternalCalibration.locate_tags = lambda self, frame, detector=None, layers=None: \
                original_tags(self, frame, Detector(), layers)
            locate = X.make_locator(cal, Camera())
            found = locate(3)
            self.assertAlmostEqual(found["x"], -0.2, delta=0.004)
            self.assertAlmostEqual(found["y"], 0.05, delta=0.004)
            self.assertIsNone(locate(1))                                   # không thấy tag của cube 1
            X.pad_landmarks = lambda f: {k: [v[0] + 40, v[1] + 30] for k, v in cal.landmarks.items()}
            self.assertIsNone(locate(3))                                   # camera đã bị dời
            Camera.frame = None
            self.assertIsNone(locate(3))
        finally:
            X.pad_landmarks, X.ExternalCalibration.locate_tags = original_landmarks, original_tags


if __name__ == "__main__":
    unittest.main()
