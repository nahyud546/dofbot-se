"""calibrate_external: phần đo bằng camera tay và ghép mẫu (không cần phần cứng)."""
import math
import unittest

import numpy as np

import calibrate_external as E
import calibrate_hand_eye as C
import cube_layer as L
from cube_vision import external_camera as X

CAL = {"arm4_T_optical": C.nominal_mount(), "K": (902.0, 875.4, 320.0, 240.0), "k1": -0.12,
       "tag_top_z": 0.0578, "j1_valid_range": (35.0, 140.0)}
SERVO = [90.0, 125.0, 0.0, 0.0, 90.0]


def tag_in_base(x, y, layer, yaw=0.2):
    s = X.TAG_SIZE_M / 2
    c, n = math.cos(yaw), math.sin(yaw)
    z = CAL["tag_top_z"] + X.CUBE_EDGE_M * layer
    return np.array([[x + c * dx - n * dy, y + n * dx + c * dy, z]
                     for dx, dy in ((-s, -s), (s, -s), (s, s), (-s, s))])


def centre_of_view(layer=0):
    """XY trên mặt phẳng tầng `layer` nằm giữa ảnh camera tay ở SERVO."""
    import cube_search_center_math as M
    base_T_opt = M.fk_arm4_cal(SERVO, CAL) @ CAL["arm4_T_optical"]
    Kmat = np.array([[902.0, 0, 320.0], [0, 875.4, 240.0], [0, 0, 1.0]])
    return C.ray_plane_xy(320, 240, base_T_opt, Kmat, CAL["k1"], CAL["tag_top_z"] + X.CUBE_EDGE_M * layer)[:2]


def seen_by_wrist(corners_base):
    return np.array([L.project_base_point(p, SERVO, CAL) for p in corners_base])


class WristMeasurement(unittest.TestCase):
    def test_recovers_tag_corners_and_layer_on_the_table_and_on_a_stack(self):
        for layer in (0, 1, 2):
            x, y = centre_of_view(layer)
            truth = tag_in_base(x + 0.01, y - 0.008, layer)
            found_layer, pts = E.wrist_tag_corners(seen_by_wrist(truth), SERVO, CAL)
            self.assertEqual(found_layer, layer)
            np.testing.assert_allclose(pts, truth, atol=0.0015)

    def test_pose_outside_the_validated_hand_eye_envelope_is_refused(self):
        x, y = centre_of_view()
        layer, why = E.wrist_tag_corners(seen_by_wrist(tag_in_base(x, y, 0)), [20.0, 125.0, 0.0, 0.0, 90.0], CAL)
        self.assertIsNone(layer)
        self.assertIn("ngoài vùng", why)

    def test_tag_floating_between_layers_is_refused(self):
        x, y = centre_of_view()
        floating = tag_in_base(x, y, 0)
        floating[:, 2] += 0.015                           # giữa tầng 0 và 1
        layer, why = E.wrist_tag_corners(seen_by_wrist(floating), SERVO, CAL)
        self.assertIsNone(layer)
        self.assertIn("không khớp tầng", why)

    def test_tilted_tag_is_refused(self):
        # Tag nghiêng 50° (cube kê lệch): chiếu các góc xuống mặt phẳng ngang không còn ra hình vuông 20 mm.
        x, y = centre_of_view()
        flat = tag_in_base(x, y, 0, yaw=0.0)
        centre = flat.mean(axis=0)
        tilt = math.radians(50)
        tilted = flat.copy()
        tilted[:, 1] = centre[1] + (flat[:, 1] - centre[1]) * math.cos(tilt)
        tilted[:, 2] = centre[2] + (flat[:, 1] - centre[1]) * math.sin(tilt)
        layer, why = E.wrist_tag_corners(seen_by_wrist(tilted), SERVO, CAL)
        self.assertIsNone(layer)
        self.assertIn("nghiêng", why)


class Fusion(unittest.TestCase):
    pts = tag_in_base(-0.2, 0.0, 1)

    def test_median_of_consistent_views(self):
        views = [(1, self.pts + d) for d in (0.0, 0.001, -0.001)]
        layer, median, spread = E.fuse_views(views)
        self.assertEqual(layer, 1)
        np.testing.assert_allclose(median, self.pts, atol=1e-9)
        self.assertLess(spread, 0.002)

    def test_views_that_disagree_on_position_are_dropped(self):
        self.assertIsNone(E.fuse_views([(1, self.pts), (1, self.pts + 0.02)]))

    def test_layer_is_decided_by_majority_and_a_split_vote_is_dropped(self):
        self.assertEqual(E.fuse_views([(1, self.pts), (1, self.pts), (0, self.pts)])[0], 1)
        self.assertIsNone(E.fuse_views([(1, self.pts), (0, self.pts), (2, self.pts)]))
        self.assertIsNone(E.fuse_views([]))


class Samples(unittest.TestCase):
    def test_arrays_and_groups_follow_set_and_tag(self):
        base, px = tag_in_base(-0.2, 0.0, 0).tolist(), [[1, 2], [3, 4], [5, 6], [7, 8]]
        samples = {"sets": [{"tags": [{"id": 1, "base": base, "px": px}, {"id": 2, "base": base, "px": px}]},
                            {"tags": [{"id": 1, "base": base, "px": px}]}]}
        points, pixels, groups = E.samples_to_arrays(samples)
        self.assertEqual((points.shape, pixels.shape), ((12, 3), (12, 2)))
        self.assertEqual(sorted(set(groups)), [(0, 1), (0, 2), (1, 1)])

    def test_report_states_the_verdict(self):
        rng = np.random.default_rng(1)
        points = np.concatenate([tag_in_base(-0.15 - 0.03 * i, -0.1 + 0.04 * i, i % 3) for i in range(7)])
        T = np.eye(4)
        T[:3, :3] = [[0, -0.6, 0.8], [-1, 0, 0], [0, -0.8, -0.6]]
        T[:3, 3] = [-0.6, 0.0, 0.4]
        pixels = X.project_points(points, (950.0, 640.0, 360.0), 0.0, T) + rng.normal(0, 0.3, (28, 2))
        result = X.solve(points, pixels, (1280, 720), [i // 4 for i in range(28)])
        text = E.report(result)
        self.assertIn("RMS chiếu lại", text)
        self.assertTrue(text.strip().splitlines()[-1].startswith(("ĐẠT", "CHƯA ĐẠT")))


if __name__ == "__main__":
    unittest.main()
