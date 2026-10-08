"""T8 hand-eye mode: XY + layer from measured joints, any start pose."""
import copy
import time
import unittest
from unittest.mock import patch

from t8_ros_tasks import RosTaskRunner

TABLE_TCP_Z = 0.047


def located(x, y, layer):
    return {"ok": True, "x": x, "y": y, "layer": layer, "reason": "",
            "top_z": 0.0584 + 0.03 * layer, "tcp_z": TABLE_TCP_Z + 0.03 * layer,
            "range_error_mm": 2.0}


def obj(identity, x=-0.20, y=0.0, layer=0):
    z = TABLE_TCP_Z + 0.03 * layer
    return {"object_id": identity, "geometry_model_id": identity,
            "track_id": f"cube-{identity}", "age_s": 0.1, "confirmed": True,
            "stationary": True, "reason": "", "base_pose_valid": False,
            "base_position": None, "handeye": located(x, y, layer),
            "grasp": {"tcp_position_base": [x, y, z], "preferred_yaw_rad": 0.0,
                      "surface_id": "+Z"}}


def lock(item, source="handeye_tag_plane"):
    handeye = item["handeye"]
    return {"object_kind": "cube", "cube_id": item["object_id"],
            "geometry_model_id": item["geometry_model_id"],
            "track_id": item["track_id"], "identity_source": "apriltag",
            "identity_confidence": 1.0, "pose_confidence": 1.0,
            "calibrated": False, "base_pose_valid": False,
            "pose_method": "apriltag_ippe",
            "grasp": {**item["grasp"], "coordinate_source": source,
                      "layer": handeye["layer"], "top_z": handeye["top_z"]}}


def scene(objects, handeye_available=True, joints=True, stamp=None):
    data = {"ok": True, "objects": objects, "calibrated": False,
            "status": "camera/hand-eye calibration not validated",
            "handeye_available": handeye_available,
            "robot": {"phase": "empty",
                      "joint_positions_rad": [0.0] * 5 if joints else None}}
    if stamp is not None:
        data["stamp"] = stamp
    return data


class Backend:
    backend = "ros3d"

    def __init__(self, scenes, approved):
        self.scenes, self.approved = iter(scenes), approved
        self.targets, self.approval_calls = [], []

    def snapshot(self):
        return copy.deepcopy(next(self.scenes))

    def preflight(self, targets):
        self.targets.append(targets)
        return {"ok": True, "stages": targets, "collision_checked": False}

    def approve(self, ids, operation="pick_hold", notice="", **options):
        self.approval_calls.append((ids, operation))
        self.approval_options = getattr(self, "approval_options", []) + [options]
        now = time.time()
        return {"ok": True, "approval_token": "a" * 32, "scene_stamp": 100.0,
                "approved_at": now, "expires_at": now + 15,
                "objects": copy.deepcopy(self.approved)}


class Motion:
    def __init__(self):
        self.calls = []

    def execute(self, command, **payload):
        self.calls.append((command, payload))
        return {"ok": True, "holding": command == "pick_cube_3d",
                "status": "executed_unverified"}

    def close(self):
        pass


class HandEyeModeTests(unittest.TestCase):
    def test_pick_keeps_pose_and_marks_handeye_source(self):
        red = obj(3)
        backend = Backend([scene([red])], [lock(red)])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual(motion.calls[0], ("prepare", {"keep_pose": True}))
        self.assertEqual(motion.calls[1][1]["coordinate_source"], "handeye_tag_plane")
        self.assertEqual(backend.approval_calls, [([3], "sort")])
        self.assertEqual(backend.approval_options, [{"handeye": True}])

    def test_stack_on_a_second_level_target_uses_layer_heights(self):
        source, target = obj(4, y=.07), obj(1, layer=1)
        after = scene([obj(4, y=0.0, layer=2)], stamp=time.time() + 5)
        backend = Backend([scene([source, target]), after], [lock(source), lock(target)])
        motion = Motion()
        runner = RosTaskRunner(backend, motion=motion)
        result = runner.execute("stack_cubes", {"source": "cube_4", "target": "cube_1"})
        self.assertEqual(backend.approval_calls, [([4, 1], "stack")])
        stages = {t["stage"]: t["tcp_position_base"][2] for t in backend.targets[0]}
        self.assertAlmostEqual(stages["pick"], 0.047)
        self.assertAlmostEqual(stages["place"], 0.107)
        self.assertAlmostEqual(stages["hover_place"], 0.122)
        payload = motion.calls[1][1]
        self.assertEqual(payload["coordinate_mode"], "handeye")
        self.assertEqual(payload["stack_layers"], 2)
        self.assertEqual(result["status"], "scene_verified")
        self.assertEqual(runner.last_stack_top_id, 4)

    def test_pick_from_a_cube_on_top_of_another_uses_its_layer_height(self):
        top = obj(2, layer=1)
        backend = Backend([scene([top])], [lock(top)])
        motion = Motion()
        RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "cube_2"})
        self.assertAlmostEqual(motion.calls[1][1]["tcp_position_base"][2], 0.077)

    def test_verification_rejects_wrong_layer_or_offset(self):
        runner = RosTaskRunner(Backend([], []))
        good = obj(4, x=-0.20, y=0.0, layer=2)
        for item, expected in ((good, ((-0.20, 0.0), 2)),
                               (obj(4, x=-0.20, y=0.0, layer=1), ((-0.20, 0.0), 2)),
                               (obj(4, x=-0.20, y=0.03, layer=2), ((-0.20, 0.0), 2))):
            runner.backend = Backend([scene([item], stamp=200.0)] * 3, [])
            with patch("t8_ros_tasks.time.sleep"):
                clock = iter([0, 0, 1, 9, 9, 9]).__next__
                runner.clock = clock
                outcome = runner.verify_stack(4, 1, 100.0, timeout_s=5, expected=expected)
            self.assertEqual(outcome["ok"], item is good, outcome)

    def test_verification_falls_back_to_pixel_position_when_the_top_face_is_not_a_tag(self):
        """Colour/trash top face -> RGB pose is tilted, hand-eye rejects it; the bbox position still verifies."""
        def source(px_error):
            item = obj(4, layer=2)
            item["handeye"] = {"ok": False, "reason": "cube nghieng (40): khong co mat tren nam ngang"}
            item["expected_px_error"] = px_error
            return item

        class PixelBackend(Backend):
            def snapshot(self, expect=None):
                self.expect = expect
                return super().snapshot()

        runner = RosTaskRunner(Backend([], []))
        for error, ok in ((12.0, True), (120.0, False)):
            runner.backend = PixelBackend([scene([source(error)], stamp=200.0)] * 3, [])
            runner.clock = iter([0, 0, 1, 9, 9, 9]).__next__
            with patch("t8_ros_tasks.time.sleep"):
                outcome = runner.verify_stack(4, 1, 100.0, timeout_s=5,
                                              expected=((-0.20, 0.0), 2, 0.0884))
            self.assertEqual(outcome["ok"], ok, outcome)
            self.assertEqual(runner.backend.expect[0]["object_id"], 4)
            self.assertAlmostEqual(runner.backend.expect[0]["xyz"][2], 0.0884 + 0.015)
            if ok:
                self.assertEqual(outcome["method"], "pixel")
            else:
                self.assertIn("pixel", outcome["reason"])

    def test_observe_scene_separates_seen_cubes_from_identity_only_tracks(self):
        runner = RosTaskRunner(Backend([], []))
        seen = obj(4, x=-0.1749, y=0.0415, layer=1)
        ghost = dict(obj(3), camera_pose_valid=False, handeye={"ok": False, "reason": "pose fusion rejected"})
        duplicate = dict(obj(4), handeye={"ok": False, "reason": "x"})
        text = runner.describe_scene({"objects": [seen, ghost, duplicate]})
        self.assertIn("ID 4", text)
        self.assertIn("x=-175, y=42 mm, tầng 2", text)
        self.assertEqual(text.count("ID 4"), 1)                      # no duplicate listing
        self.assertIn("Nghi vấn", text)
        self.assertIn("ID 3", text.split("Nghi vấn")[1])             # ghost track is flagged, not "seen"
        self.assertNotIn("ID 3", text.split("Nghi vấn")[0])

    def test_without_joint_feed_falls_back_to_ready_pose_map(self):
        red = obj(3)
        fixed = lock(red, source="fixed_ready_pose")
        backend = Backend([scene([red], joints=False)], [fixed])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual(motion.calls[0], ("prepare", {}))
        self.assertEqual(backend.approval_options, [{}])

    def test_lock_without_handeye_geometry_returns_to_ready_and_reapproves(self):
        red = obj(3)
        backend = Backend([scene([red])], [lock(red, source="fixed_ready_pose")])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual([c for c in motion.calls[:2]],
                         [("prepare", {"keep_pose": True}), ("prepare", {})])
        self.assertEqual(len(backend.approval_calls), 2)
        self.assertEqual(backend.approval_options, [{"handeye": True}, {}])


if __name__ == "__main__":
    unittest.main()


class ResolveWithIdentityOnlyTracks(unittest.TestCase):
    def test_pad_track_without_pose_does_not_make_the_cube_ambiguous(self):
        runner = RosTaskRunner(Backend([], []))
        cube = dict(obj(3), camera_pose_valid=True, track_id="real")
        pad = dict(obj(3), camera_pose_valid=False, track_id="pad", handeye={"ok": False, "reason": "x"})
        runner.latest = {"objects": [pad, cube]}
        runner.memory[3] = cube
        found, why = runner.resolve("cube_3")
        self.assertEqual(why, "")
        self.assertEqual(found["track_id"], "real")
        # two tracks that BOTH have a pose are still ambiguous
        second = dict(cube, track_id="other")
        runner.latest = {"objects": [cube, second]}
        self.assertIsNone(runner.resolve("cube_3")[0])
