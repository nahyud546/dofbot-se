"""Scene worker: cube chỉ có 4 góc một mặt (không pose PnP) vẫn có tầng/XY/skeleton từ góc khớp."""
import json
import math
import sys
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))

import test_cube_layer as T  # noqa: E402
import test_gravity_pose as TG  # noqa: E402
import t8_ros_scene_worker as W  # noqa: E402


class Joints:
    def __init__(self, servo):
        self.servo = servo

    def stationary_servo(self, stamp):
        return self.servo


def scene_case(layer=1, seed=4):
    rng = np.random.default_rng(seed)
    for _ in range(50):
        servo, xy = TG.case(rng, layer)
        yaw = rng.uniform(-0.6, 0.6)
        quad = TG.face_corners_px(xy, layer, yaw, servo, noise=0.4, rng=rng)
        if quad is not None:
            return servo, xy, yaw, quad
    raise AssertionError("no case")


class TestFaceFallback(unittest.TestCase):
    def test_no_pose_but_face_quad_gives_layer_and_xy(self):
        servo, xy, yaw, quad = scene_case(layer=2)
        faces = [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}]
        loc = W.locate_pose(None, "no pose", 1.0, Joints(servo), T.CAL, None, faces)
        self.assertTrue(loc["ok"])
        self.assertEqual(loc["layer"], 2)
        self.assertEqual(loc["source"], "face_gravity")
        self.assertLess(math.hypot(loc["x"] - xy[0], loc["y"] - xy[1]), 0.004)

    def test_good_pose_is_still_preferred(self):
        rng = np.random.default_rng(2)
        servo, xy, _ = T.make_case(rng, T.TRUE_PERCEPTION_K, 0)
        pose = T.perceive(xy, 0, 0.2, servo, T.TRUE_PERCEPTION_K, rng)
        quad = TG.face_corners_px(xy, 0, 0.2, servo)
        faces = [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}]
        loc = W.locate_pose(pose, "", 1.0, Joints(servo), T.CAL, T.TRUE_PERCEPTION_K, faces)
        self.assertTrue(loc["ok"])
        self.assertNotEqual(loc.get("source"), "face_gravity")

    def test_marginal_pose_is_replaced_by_a_better_fitting_face(self):
        servo, xy, yaw, quad = scene_case(layer=0, seed=11)
        faces = [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}]
        marginal = {"ok": True, "layer": 0, "x": 0.0, "y": 0.0, "range_error_mm": 9.0, "reason": ""}
        with unittest.mock.patch("cube_layer.locate_cube", return_value=marginal):
            loc = W.locate_pose(np.eye(4), "", 1.0, Joints(servo), T.CAL, None, faces)
        self.assertEqual(loc["source"], "face_gravity")
        self.assertLess(loc["range_error_mm"], 9.0)

    def test_accurate_pose_is_not_overridden_by_the_face(self):
        servo, xy, yaw, quad = scene_case(layer=0, seed=12)
        faces = [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}]
        good = {"ok": True, "layer": 0, "x": 0.0, "y": 0.0, "range_error_mm": 1.5, "reason": ""}
        with unittest.mock.patch("cube_layer.locate_cube", return_value=good):
            loc = W.locate_pose(np.eye(4), "", 1.0, Joints(servo), T.CAL, None, faces)
        self.assertIs(loc, good)

    def test_wrong_pose_falls_back_to_the_face(self):
        servo, xy, yaw, quad = scene_case(layer=0, seed=7)
        tilted = np.eye(4)
        tilted[:3, :3] = np.array([[1, 0, 0], [0, math.cos(0.8), -math.sin(0.8)],
                                   [0, math.sin(0.8), math.cos(0.8)]])
        tilted[2, 3] = 0.2
        faces = [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}]
        loc = W.locate_pose(tilted, "", 1.0, Joints(servo), T.CAL, T.TRUE_PERCEPTION_K, faces)
        self.assertTrue(loc["ok"] and loc["source"] == "face_gravity")

    def test_small_patch_gives_no_location_and_explains(self):
        rng = np.random.default_rng(9)
        servo, xy = TG.case(rng, 0)
        quad = TG.face_corners_px(xy, 0, 0.1, servo, edge=0.010)
        loc = W.locate_pose(None, "no pose", 1.0, Joints(servo), T.CAL, None,
                            [{"axis": "+X", "quad": quad.tolist(), "conf": 0.9}])
        self.assertFalse(loc["ok"])
        self.assertIn("từ mặt", loc["reason"])

    def test_moving_arm_is_pending_not_guessed(self):
        servo, xy, yaw, quad = scene_case()
        loc = W.locate_pose(None, "", 1.0, Joints(None), T.CAL, None,
                            [{"axis": "+X", "quad": quad.tolist(), "conf": 0.9}])
        self.assertTrue(loc.get("pending"))


class TestFeedAndDrawing(unittest.TestCase):
    def test_feed_keeps_recent_faces_only(self):
        feed = W.FaceQuadFeed(ttl_s=1.0)
        quad = [[0, 0], [10, 0], [10, 10], [0, 10]]
        feed.update(json.dumps({"stamp": 1.0, "tracks": {"t1": {"object_id": 3, "faces": [
            {"axis": "+X", "quad": quad, "conf": 0.7}]}}}), now=10.0)
        self.assertEqual(len(feed.faces("t1", 10.5)), 1)
        self.assertEqual(feed.faces("t1", 12.0), [])
        self.assertEqual(feed.faces("missing", 10.5), [])
        feed.update("not json", now=11.0)          # không ném lỗi

    def test_skeleton_is_drawn_on_the_projected_location(self):
        import cv2
        servo, xy, yaw, quad = scene_case(layer=1, seed=5)
        loc = W.locate_pose(None, "", 1.0, Joints(servo), T.CAL, None,
                            [{"axis": "+X", "quad": quad.tolist(), "conf": 0.8}])
        frame = np.zeros((480, 640, 3), np.uint8)
        self.assertTrue(W.draw_handeye_skeleton(frame, loc, servo, T.CAL, True))
        ys, xs = np.nonzero(frame[:, :, 1])
        self.assertGreater(len(xs), 100)
        cx, cy = quad.mean(axis=0)
        self.assertLess(abs(xs.mean() - cx), 60)
        self.assertFalse(W.draw_handeye_skeleton(frame, {"ok": False}, servo, T.CAL, True))


if __name__ == "__main__":
    unittest.main()
