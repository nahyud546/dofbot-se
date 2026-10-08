"""Approval transport and motion sequencing regressions; no hardware access."""
import math
import os
import subprocess
import sys
import time
import unittest
import numpy as np
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

os.environ["T8_HAND_EYE_FILE"] = "/nonexistent/hand_eye.json"  # hermetic: no hand-eye mode
from t8_ros_scene import RosSceneBridge
from t8_ros_scene_worker import (approve, live_approval_error, approval_image,
                                 ApprovalDisplayStatus, select_ready_targets)
from t8_ros_tasks import RosTaskRunner
from t8_assistant import execute_plan
from test_ros_tasks import Backend, PhysicalBackend, Motion, lock_for, obj, scene


class ApprovalTests(unittest.TestCase):
    def test_invalid_duplicate_does_not_block_one_valid_track(self):
        valid = SimpleNamespace(object_id=3, track_id="good")
        rejected = SimpleNamespace(object_id=3, track_id="noise")
        matches, reasons = select_ready_targets(
            [(valid, True, ""), (rejected, False, "pose fusion rejected")], [3])
        self.assertIs(matches[3], valid)
        self.assertEqual(reasons, [])

    def test_two_valid_tracks_remain_ambiguous(self):
        first = SimpleNamespace(object_id=3, track_id="a")
        second = SimpleNamespace(object_id=3, track_id="b")
        matches, reasons = select_ready_targets(
            [(first, True, ""), (second, True, "")], [3])
        self.assertEqual(matches, {})
        self.assertIn("multiple valid tracks", reasons[0])

    def test_smoothed_annotated_image_wins_over_faster_raw(self):
        annotated, raw = object(), object()
        state = {"annotated": annotated, "annotated_stamp": 10,
                 "raw": raw, "raw_stamp": 10.9}
        selected, age = approval_image(state, 11)
        self.assertIs(selected, annotated)
        self.assertEqual(age, 1)
        state["raw_stamp"] = 12.5
        selected, age = approval_image(state, 13)
        self.assertIs(selected, raw)
        self.assertEqual(age, .5)
        self.assertIsNone(approval_image(state, 16)[0])

    def test_status_debounce_never_keeps_stale_ready(self):
        display = ApprovalDisplayStatus()
        self.assertFalse(display.update("READY", True, 0, (1,))[1])
        self.assertFalse(display.update("READY", True, .9, (1,))[1])
        self.assertTrue(display.update("READY", True, 1, (1,))[1])
        text, ready = display.update("ID conflict", False, 1.1)
        self.assertFalse(ready)
        self.assertNotIn("READY", text)
        self.assertFalse(display.update("READY", True, 1.2, (1,))[1])
        self.assertFalse(display.update("READY", True, 2.2, (2,))[1])

    def test_waiting_text_does_not_flicker_between_targets(self):
        display = ApprovalDisplayStatus()
        self.assertEqual(display.update("calibration missing", False, 0)[0], "calibration missing")
        self.assertEqual(display.update("ID 1 missing", False, .1)[0], "calibration missing")
        self.assertEqual(display.update("ID 3 missing", False, .2)[0], "calibration missing")
        self.assertEqual(display.update("ID 3 missing", False, 1.3)[0], "ID 3 missing")
    def gui_runtime(self, keys, visible=1):
        cv = MagicMock()
        cv.error = RuntimeError
        cv.waitKey.side_effect = keys
        cv.getWindowProperty.return_value = visible
        node = MagicMock()
        ros = MagicMock()
        modules = {"cv2": cv, "rclpy": ros,
                   "rclpy.node": SimpleNamespace(Node=lambda name: node),
                   "rclpy.qos": SimpleNamespace(
                       qos_profile_sensor_data=object(), QoSProfile=lambda **kw: SimpleNamespace(**kw),
                       HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                       ReliabilityPolicy=SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)),
                   "cv_bridge": SimpleNamespace(CvBridge=MagicMock()),
                   "sensor_msgs": SimpleNamespace(), "sensor_msgs.msg": SimpleNamespace(Image=object, JointState=object),
                   "cap_scene_interfaces": SimpleNamespace(),
                   "cap_scene_interfaces.msg": SimpleNamespace(ObjectStates=object),
                   "cube_sort_3d": SimpleNamespace(grasp_candidates_for_obj=MagicMock(),
                       is_graspable=MagicMock(), pose_matrix=MagicMock(), update_confirmation=MagicMock())}
        return modules, cv, node, ros

    def test_indefinite_viewer_survives_sixty_seconds_and_cancels(self):
        modules, cv, node, ros = self.gui_runtime([0, 27])
        with patch.dict(sys.modules, modules), \
             patch("t8_ros_scene_worker.time.monotonic", side_effect=[0, 61, 61, 62, 62]):
            result = approve([1], "pick_hold", 0)
        self.assertEqual(result["code"], "approval_cancelled")
        self.assertEqual(cv.imshow.call_count, 2)
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()

    def test_window_close_cancels_and_releases_resources(self):
        modules, cv, node, ros = self.gui_runtime([0], visible=-1)
        with patch.dict(sys.modules, modules):
            result = approve([1], "pick_hold", 0)
        self.assertEqual(result["code"], "approval_cancelled")
        node.destroy_node.assert_called_once()

    def test_ctrl_c_closes_viewer(self):
        modules, cv, node, ros = self.gui_runtime([KeyboardInterrupt()])
        with patch.dict(sys.modules, modules), self.assertRaises(KeyboardInterrupt):
            approve([1], "pick_hold", 0)
        cv.destroyWindow.assert_called_once()
        node.destroy_node.assert_called_once()

    def test_space_without_scene_stays_blocked_then_timeout_explains_why(self):
        modules, cv, node, ros = self.gui_runtime([32])
        with patch.dict(sys.modules, modules), \
             patch("t8_ros_scene_worker.time.monotonic", side_effect=[0, .1, .1, 3]):
            result = approve([1], "pick_hold", 1)
        self.assertEqual(result["code"], "approval_timeout")
        self.assertIn("CAMERA", result["reason"])

    def test_rejected_pose_reason_reaches_progress_events(self):
        modules, cv, node, ros = self.gui_runtime([0, 27])
        node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        bridge = modules["cv_bridge"].CvBridge.return_value
        bridge.imgmsg_to_cv2.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
        modules["cube_sort_3d"].update_confirmation.return_value = None
        rejected = SimpleNamespace(track_id="bad", object_id=1,
                                   bbox_xyxy=[10, 10, 50, 50], reason="pose fusion rejected measurement")
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
                              calibrated=False, status="waiting", objects=[rejected])
        def spin(*args, **kwargs):
            callbacks = node.create_subscription.call_args_list
            callbacks[0].args[2](msg)
            callbacks[1].args[2](object())
        ros.spin_once.side_effect = spin
        with patch.dict(sys.modules, modules), patch("t8_ros_scene_worker.approval_event") as events:
            result = approve([1], "pick_hold", 0)
        self.assertEqual(result["code"], "approval_cancelled")
        self.assertTrue(any("pose fusion rejected measurement" in call.args[1] for call in events.call_args_list))
        # Only the fixed status panel is drawn; no extra object bbox rectangles.
        self.assertTrue(all(call.args[1] == (0, 0) for call in cv.rectangle.call_args_list))

    def handeye_scene(self, locations, keys=(0, 32), empty_candidates=False):
        """Run approve() in hand-eye mode against fake ROS; returns (result, events)."""
        modules, cv, node, ros = self.gui_runtime(list(keys))
        node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        modules["cv_bridge"].CvBridge.return_value.imgmsg_to_cv2.return_value = np.zeros(
            (480, 640, 3), dtype=np.uint8)
        stamp = SimpleNamespace(sec=100, nanosec=0)
        objects = []
        for identity in locations:
            objects.append(SimpleNamespace(
                track_id=f"t{identity}", object_id=identity, geometry_model_id=identity,
                bbox_xyxy=[200, 150, 280, 230], reason="", pose_method="apriltag_ippe",
                identity_confidence=1.0, pose_confidence=1.0, camera_pose_valid=True,
                base_pose_valid=False,
                camera_pose=SimpleNamespace(header=SimpleNamespace(stamp=stamp))))
        msg = SimpleNamespace(header=SimpleNamespace(stamp=stamp), calibrated=False,
                              status="waiting", objects=objects)
        cube = modules["cube_sort_3d"]
        cube.update_confirmation.side_effect = lambda *_: {"time_s": time.monotonic()}
        cube.is_graspable.return_value = (True, "")
        cube.grasp_candidates_for_obj.return_value = [{
            "surface_id": "+Z", "tcp_position_base": [-0.1, 0.0, 0.047],
            "preferred_yaw_rad": 0.2, "center_px": [240.0, 190.0]}]
        if empty_candidates:
            cube.grasp_candidates_for_obj.return_value = []

        def spin(*args, **kwargs):
            callbacks = node.create_subscription.call_args_list
            callbacks[0].args[2](msg)
            callbacks[1].args[2](object())      # a fresh annotated camera frame
        ros.spin_once.side_effect = spin

        class Display:
            def update(self, text, ready, now, key=()):
                return ("READY - PRESS SPACE" if ready else text), ready

        def locate(obj, stamp_s, joints, cal, intrinsics, pose_matrix, faces=None):
            value = locations[obj.object_id]
            return value() if callable(value) else value
        with patch.dict(sys.modules, modules), \
             patch("t8_ros_scene_worker.handeye_calibration", return_value={"tag_top_z": .058}), \
             patch("t8_ros_scene_worker.perception_intrinsics",
                   return_value=((902., 875., 320., 240., 0.), "test")), \
             patch("t8_ros_scene_worker.locate_object", side_effect=locate), \
             patch("t8_ros_scene_worker.ApprovalDisplayStatus", Display), \
             patch("t8_ros_scene_worker.approval_event") as events:
            result = approve(list(locations), "stack" if len(locations) == 2 else "sort", 0,
                             handeye=True)
        return result, events

    def test_handeye_approval_builds_locks_from_layer_and_xy_not_the_fixed_map(self):
        good = lambda x, y, layer: {"ok": True, "x": x, "y": y, "layer": layer, "top_z": .0584 + .03 * layer,
                                    "tcp_z": .047 + .03 * layer, "range_error_mm": 3.0, "reason": "",
                                    "yaw_deg": 5.0, "pixel": [240.0, 190.0]}
        result, _ = self.handeye_scene({4: good(-.21, .05, 0), 1: good(-.17, -.02, 1)})
        self.assertTrue(result["ok"], result)
        by_id = {lock["cube_id"]: lock["grasp"] for lock in result["objects"]}
        self.assertEqual(by_id[1]["tcp_position_base"], [-.17, -.02, .077])
        self.assertEqual(by_id[1]["layer"], 1)
        self.assertEqual(by_id[4]["tcp_position_base"], [-.21, .05, .047])
        self.assertTrue(all(g["coordinate_source"] == "handeye_tag_plane" for g in by_id.values()))
        # yaw comes from the hand-eye result (READY-view convention), not the raw image quad
        self.assertAlmostEqual(by_id[4]["preferred_yaw_rad"], math.radians(5.0))

    def test_handeye_approval_rides_through_flickering_perception_frames(self):
        """Cube is still, but perception alternates good frames with 'tilted' ones."""
        good = {"ok": True, "x": -.2, "y": .02, "layer": 0, "top_z": .0584, "tcp_z": .047,
                "range_error_mm": 2.0, "reason": "", "yaw_deg": 4.0, "pixel": [240.0, 190.0]}
        bad = {"ok": False, "reason": "cube nghieng (34): khong co mat tren nam ngang"}
        frames = iter([good, good, bad, bad, bad, bad, bad, bad])
        result, _ = self.handeye_scene({3: lambda: next(frames)}, keys=(0, 0, 0, 0, 0, 32))
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["objects"][0]["grasp"]["tcp_position_base"], [-.2, .02, .047])

    def test_hold_expires_when_the_cube_moves_or_goes_stale(self):
        from t8_ros_scene_worker import HandEyeHold
        loc = lambda x: {"ok": True, "x": x, "y": 0.0, "layer": 0, "yaw_deg": 0.0, "range_error_mm": 1.0}
        box = lambda cx: SimpleNamespace(object_id=3, bbox_xyxy=[cx - 40, 100, cx + 40, 180])
        hold = HandEyeHold(ttl_s=2.0)
        for t, x in enumerate((-.200, -.204, -.196)):
            smoothed = hold.update("a", box(300), loc(x), float(t) * .2)
        self.assertAlmostEqual(smoothed["x"], -.200)          # median, not the last sample
        self.assertIsNotNone(hold.recall("a", box(310), .5))
        self.assertIsNone(hold.recall("a", box(400), .5))      # cube moved in the image
        self.assertIsNone(hold.recall("a", box(300), 5.0))     # too old
        hold.prune(set(), 5.0)
        self.assertEqual(hold.tracks, {})

    def test_hold_confirms_across_a_bad_frame_and_a_track_id_change(self):
        from t8_ros_scene_worker import HandEyeHold
        loc = lambda x, yaw=0.0, layer=0: {"ok": True, "x": x, "y": 0.02, "layer": layer,
                                           "yaw_deg": yaw, "range_error_mm": 2.0}
        obj = lambda tid, cx=300: SimpleNamespace(object_id=4, track_id=tid,
                                                  bbox_xyxy=[cx - 40, 100, cx + 40, 180])
        hold = HandEyeHold()
        hold.update("a", obj("a"), loc(-.2), 0.0)
        self.assertEqual(hold.agreeing("a", 0.0), 1)
        self.assertIsNotNone(hold.recall("a", obj("a"), 0.3))      # a bad frame in between: still held
        hold.update("a", obj("a"), loc(-.201), 0.6)
        self.assertEqual(hold.agreeing("a", 0.6), 2)               # 2 of 3 frames suffice, no streak needed
        # perception re-labels the same cube: the new track inherits the samples
        hold.update("b", obj("b", 305), loc(-.2005), 0.9)
        self.assertEqual(hold.agreeing("b", 0.9), 3)
        # a cube that really moved (>10 mm) does not count as the same fix
        hold.update("b", obj("b", 305), loc(-.19), 1.2)
        self.assertEqual(hold.agreeing("b", 1.2), 1)
        # other layer or other cube ID never inherits
        other = SimpleNamespace(object_id=2, track_id="c", bbox_xyxy=[260, 100, 340, 180])
        self.assertIsNone(hold.recall("c", other, 1.2))

    def test_hold_yaw_median_is_circular(self):
        from t8_ros_scene_worker import HandEyeHold, yaw_delta_deg
        loc = lambda yaw: {"ok": True, "x": -.2, "y": 0.0, "layer": 0, "yaw_deg": yaw,
                           "range_error_mm": 1.0}
        box = SimpleNamespace(object_id=1, bbox_xyxy=[260, 100, 340, 180])
        hold = HandEyeHold()
        for t, yaw in enumerate((44.0, -44.0, 43.0, -43.0, 44.5)):
            smoothed = hold.update("a", box, loc(yaw), t * .1)
        self.assertLess(abs(yaw_delta_deg(smoothed["yaw_deg"], 44.0)), 2.0)   # not ~0 from a linear median
        self.assertEqual(hold.agreeing("a", .4), 5)               # all within 15 deg modulo 90

    def test_handeye_approval_works_when_cube_touches_the_image_border(self):
        """grasp_candidates_for_obj returns [] when the top face is clipped by the image."""
        loc = {"ok": True, "x": -.155, "y": .045, "layer": 0, "top_z": .0584, "tcp_z": .047,
               "range_error_mm": 1.0, "reason": "", "yaw_deg": -8.0, "pixel": [568.0, 405.0]}
        original = self.handeye_scene

        result, _ = original({1: loc}, empty_candidates=True)
        self.assertTrue(result["ok"], result)
        grasp = result["objects"][0]["grasp"]
        self.assertEqual(grasp["tcp_position_base"], [-.155, .045, .047])
        self.assertEqual(grasp["coordinate_source"], "handeye_tag_plane")

    def test_handeye_approval_blocks_space_until_layer_is_resolved_and_supported(self):
        good = lambda layer: {"ok": True, "x": -.2, "y": 0, "layer": layer, "top_z": .0584 + .03 * layer,
                              "tcp_z": .047 + .03 * layer, "range_error_mm": 2, "reason": "",
                              "yaw_deg": 0.0, "pixel": [240.0, 190.0]}
        for locations, text in (
                ({3: {"ok": False, "pending": True, "reason": "khoi chua dung yen"}}, "HAND-EYE"),
                ({4: good(0), 1: good(3)}, "LAYER 3 above verified limit 2")):   # target too high
            result, events = self.handeye_scene(locations, keys=(0, 32, 27))
            self.assertEqual(result["code"], "approval_cancelled", result)
            blocked = [call.args[1] for call in events.call_args_list if "Space bị khóa" in call.args[1]]
            self.assertTrue(blocked and text in blocked[0], (blocked, text))
        ok_result, _ = self.handeye_scene({3: good(3)})           # a source on layer 3 is fine
        self.assertTrue(ok_result["ok"], ok_result)

    def test_freshness_and_resolution_gate(self):
        self.assertEqual(live_approval_error(.1, .1, (480, 640)), "")
        self.assertIn("CAMERA", live_approval_error(3, .1, (480, 640)))
        self.assertIn("OBJECT", live_approval_error(.1, 3, (480, 640)))
        self.assertIn("640x480", live_approval_error(.1, .1, (720, 1280)))

    def test_gtk_autosize_zero_means_open_window(self):
        modules, cv, node, ros = self.gui_runtime([0, 27], visible=0)
        with patch.dict(sys.modules, modules):
            result = approve([1], "pick_hold", 0)
        self.assertEqual(cv.waitKey.call_count, 2)
        self.assertEqual(result["code"], "approval_cancelled")

    def test_preflight_workspace_allows_fourth_cube_hover_only(self):
        from cube_3d_preflight import check_target
        with self.assertRaisesRegex(ValueError, "work area"):
            check_target({"tcp_position_base": [-.2, 0, .191], "preferred_yaw_rad": 0}, None)

    def test_no_desktop_returns_explicit_error(self):
        with patch.dict(os.environ, {"DISPLAY": "", "WAYLAND_DISPLAY": ""}):
            result = RosSceneBridge().approve([1])
        self.assertEqual(result["code"], "viewer_unavailable")

    def test_stream_handles_batched_events_and_final_result(self):
        real_popen = subprocess.Popen
        code = ('import sys,json; json.load(sys.stdin); '
                'print(\'T8_EVENT:{"event":"viewer_opened","reason":"opened"}\'); '
                'print(\'T8_EVENT:{"event":"ready","reason":"READY"}\'); '
                'print(\'T8_SCENE:{"ok":true,"status":"approved"}\')')
        def launch(args, **kwargs):
            self.assertNotIn("cv2", kwargs["env"].get("QT_QPA_PLATFORM_PLUGIN_PATH", ""))
            return real_popen([sys.executable, "-c", code], **kwargs)
        with patch.dict(os.environ, {"DISPLAY": ":test", "QT_QPA_PLATFORM_PLUGIN_PATH": "/venv/cv2/qt"}), \
             patch("t8_ros_scene.subprocess.Popen", side_effect=launch):
            result = RosSceneBridge().approve([1])
        self.assertTrue(result["ok"])
        self.assertIn("log_path", result)

    def test_worker_crash_preserves_log_location(self):
        real_popen = subprocess.Popen
        with patch.dict(os.environ, {"DISPLAY": ":test"}), \
             patch("t8_ros_scene.subprocess.Popen", side_effect=lambda args, **kw:
                   real_popen([sys.executable, "-c", 'import sys; sys.stdin.read(); sys.stderr.write("GUI crash")'], **kw)):
            result = RosSceneBridge().approve([1])
        self.assertEqual(result["code"], "approval_worker_failed")
        self.assertIn("/tmp/t8_approval_", result["reason"])

    def test_prepare_failure_never_opens_viewer(self):
        backend = PhysicalBackend([], [lock_for(obj(1))])
        motion = Motion()
        with patch.object(motion, "execute", return_value={"ok": False, "reply": "readback failed"}):
            result = RosTaskRunner(backend, motion=motion).execute("vision_pick_hold", {"label": "cube_1"})
        self.assertEqual(result["code"], "prepare_failed")
        self.assertEqual(backend.approval_calls, [])
        self.assertEqual(motion.closed, 1)

    def test_uncalibrated_stack_falls_back_only_after_prepare(self):
        locks = [lock_for(obj(1)), lock_for(obj(3, y=.07))]
        locks[1]["calibrated"] = False
        uncalibrated = scene([obj(1), obj(3, y=.07)], calibrated=False)
        uncalibrated["status"] = "camera/hand-eye calibration not validated"
        backend = PhysicalBackend([uncalibrated], locks)
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute("stack_cubes", {"source": "cube_1", "target": "cube_3"})
        self.assertEqual(result["code"], "stack_geometry_unavailable")
        self.assertEqual(motion.calls, [("prepare", {})])
        self.assertEqual(backend.approval_calls, [([1, 3], "stack_fixed")])

    def test_stack_precheck_requires_live_scene_before_prepare(self):
        backend = PhysicalBackend([{"ok": False, "reason": "ROS scene timeout"}],
                                  [lock_for(obj(1)), lock_for(obj(3))])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "stack_cubes", {"source": "cube_1", "target": "cube_3"})
        self.assertEqual(result["code"], "observation_unavailable")
        self.assertEqual(motion.calls, [])

    def test_temporary_tf_gap_does_not_abort_calibrated_stack(self):
        source, target = obj(1), obj(3, y=.07)
        missing_tf = scene([source, target], calibrated=False)
        missing_tf["status"] = "timestamped base-to-camera TF unavailable"
        backend = PhysicalBackend([missing_tf, scene([source, target])],
                                  [lock_for(source), lock_for(target)])
        motion = Motion()
        with patch.object(RosTaskRunner, "verify_stack", return_value={"ok": False}):
            result = RosTaskRunner(backend, motion=motion, max_retries=0).execute(
                "stack_cubes", {"source": "cube_1", "target": "cube_3"})
        self.assertTrue(result["ok"])
        self.assertEqual([command for command, _ in motion.calls], ["prepare", "stack_cube_3d"])

    def test_verified_stack_updates_top_and_uses_elevated_target(self):
        source, target = obj(4, y=.07), obj(1, z=.075)
        observed = scene([obj(4, z=.105), obj(1, z=.075)])
        observed["stamp"] = time.time() + 1
        backend = PhysicalBackend([scene([source, target]), observed],
                                  [lock_for(source), lock_for(target)])
        motion = Motion()
        runner = RosTaskRunner(backend, motion=motion)
        result = runner.execute("stack_cubes", {"source": "cube_4", "target": "cube_1"})
        self.assertEqual(result["status"], "scene_verified")
        self.assertEqual(runner.last_stack_top_id, 4)
        place = next(t for t in backend.targets[0] if t["stage"] == "place")
        self.assertAlmostEqual(place["tcp_position_base"][2], .107)

    def test_old_or_wrong_post_scene_does_not_verify_stack(self):
        observed = scene([obj(1, z=.075), obj(3)])
        observed["stamp"] = 99
        with patch("t8_ros_tasks.time.sleep"):
            runner = RosTaskRunner(Backend([observed]), clock=iter([0, 0, 9]).__next__)
            self.assertFalse(runner.verify_stack(1, 3, 100)["ok"])
        observed["stamp"] = 101
        observed["objects"][0]["base_position"][0] += .05
        with patch("t8_ros_tasks.time.sleep"):
            runner = RosTaskRunner(Backend([observed]), clock=iter([0, 0, 9]).__next__)
            self.assertFalse(runner.verify_stack(1, 3, 100)["ok"])

    def test_unverified_stack_stops_following_step(self):
        class Executor:
            _ros_runner = object()
            calls = []
            def execute(self, intent, entities):
                self.calls.append(intent)
                return {"ok": True, "status": "executed_unverified"}
        executor = Executor()
        execute_plan({"sequence": [("stack_cubes", {}, "none"), ("stack_cubes", {}, "none")]}, executor, None)
        self.assertEqual(executor.calls, ["stack_cubes"])


if __name__ == "__main__":
    unittest.main()
