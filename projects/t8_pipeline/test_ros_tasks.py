import copy
import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from t8_assistant import execute_plan, run_with_vision_retry, request_stop
from t8_executor import Executor
from t8_pipeline import InvalidPlan, Pipeline, validate_plan
from t8_ros_tasks import RosTaskRunner
from t8_motion_worker import approved_motion_plan


def obj(identity, x=-0.20, y=0.0, z=0.045, track=None):
    return {"object_id": identity, "geometry_model_id": identity,
            "track_id": track or f"cube-{identity}", "age_s": 0.1,
            "base_position": [x, y, z], "base_pose_valid": True,
            "confirmed": True, "stationary": True, "reason": "",
            "grasp": {"tcp_position_base": [x + .015, y, z + .002],
                      "preferred_yaw_rad": 0.0, "surface_id": "+Z"}}


def scene(objects, calibrated=True):
    return {"ok": True, "objects": objects, "calibrated": calibrated,
            "robot": {"phase": "empty"}}


class Backend:
    backend = "ros3d"

    def __init__(self, scenes):
        self.scenes = iter(scenes)
        self.targets = []

    def snapshot(self):
        return copy.deepcopy(next(self.scenes))

    def preflight(self, targets):
        self.targets.append(targets)
        return {"ok": True, "stages": targets, "collision_checked": False}


def lock_for(item):
    return {"object_kind": "cube", "cube_id": item["object_id"],
            "geometry_model_id": item["geometry_model_id"],
            "track_id": item["track_id"], "identity_source": "geometry_3d",
            "identity_confidence": 1.0, "pose_confidence": 1.0,
            "calibrated": True, "base_pose_valid": True,
            "pose_method": "rgb_single_face_cube",
            "grasp": {**item["grasp"], "coordinate_source": "calibrated_base_pose"}}


def fixed_lock_for(item):
    lock = lock_for(item)
    lock.update(calibrated=False, base_pose_valid=False, base_position=None)
    lock["grasp"]["coordinate_source"] = "fixed_ready_pose"
    return lock


class PhysicalBackend(Backend):
    def __init__(self, scenes, approved):
        super().__init__(scenes)
        self.approved = approved
        self.approval_calls = []
        self.approval_notices = []

    def approve(self, ids, operation="pick_hold", notice=""):
        self.approval_calls.append((ids, operation))
        self.approval_notices.append(notice)
        now = time.time()
        return {"ok": True, "approval_token": "a" * 32,
                "scene_stamp": 100.0,
                "approved_at": now, "expires_at": now + 15,
                "objects": copy.deepcopy(self.approved)}


class RetryPreflightBackend(PhysicalBackend):
    def __init__(self, scenes, approved):
        super().__init__(scenes, approved)
        self.preflight_results = iter([
            {"ok": False, "reason": "IK no solution"},
            {"ok": True, "stages": [], "collision_checked": False},
        ])

    def preflight(self, targets):
        self.targets.append(targets)
        return next(self.preflight_results)


class Motion:
    def __init__(self):
        self.calls = []
        self.closed = 0

    def execute(self, command, **payload):
        self.calls.append((command, payload))
        return {"ok": True, "holding": command == "pick_cube_3d",
                "status": "executed_unverified"}

    def close(self):
        self.closed += 1


class RosTasksTests(unittest.TestCase):
    def test_failed_ik_reopens_approval_until_a_valid_target_is_confirmed(self):
        one, three = obj(1), obj(3, y=.07)
        backend = RetryPreflightBackend(
            [scene([one, three], calibrated=False)],
            [fixed_lock_for(one), fixed_lock_for(three)])
        motion = Motion()

        result = RosTaskRunner(backend, motion=motion).execute(
            "stack_cubes", {"source": "cube_1", "target": "cube_3"})

        self.assertTrue(result["ok"])
        self.assertEqual(backend.approval_calls,
                         [([1, 3], "stack_fixed"), ([1, 3], "stack_fixed")])
        self.assertEqual(len(backend.targets), 2)
        self.assertIn("chỉ tính độ cao cube đặt trực tiếp trên bàn",
                      backend.approval_notices[0])
        self.assertIn("IK/FK chưa đạt: IK no solution",
                      backend.approval_notices[1])
        self.assertEqual([call[0] for call in motion.calls],
                         ["prepare", "stack_cube_3d"])

    def test_physical_ros3d_requires_approval_then_repreflights(self):
        red, blue = obj(3), obj(1, y=.07)
        backend = PhysicalBackend([scene([red, blue])],
                                  [lock_for(red), lock_for(blue)])
        motion = Motion()
        with patch.object(RosTaskRunner, "verify_stack", return_value={"ok": False}):
            result = RosTaskRunner(backend, motion=motion, max_retries=0).execute(
                "stack_cubes", {"source": "khoi_do", "target": "cube_1"})
        self.assertTrue(result["ok"])
        self.assertFalse(result["dry_run"])
        self.assertTrue(result["execution_enabled"])
        self.assertEqual(backend.approval_calls, [([3, 1], "stack")])
        self.assertEqual(len(backend.targets), 1)
        self.assertEqual(motion.calls[0][0], "prepare")
        self.assertEqual(motion.calls[1][0], "stack_cube_3d")
        self.assertEqual(motion.calls[1][1]["approval_token"], "a" * 32)
        self.assertEqual(motion.closed, 2)

    def test_motion_worker_uses_only_complete_matching_preflight_waypoints(self):
        source = [-.20, 0.0, .047]
        target = [-.18, .02, .047]
        offset = [.012, -.003]
        joints = [90.0, 70.0, 40.0, 30.0, 90.0]
        points = {
            "approach_pick": [source[0], source[1], source[2] + .020],
            "pick": source,
            "lift_pick": [source[0], source[1], source[2] + .030],
            "hover_place": [target[0] + offset[0], target[1] + offset[1], .092],
            "place": [target[0] + offset[0], target[1] + offset[1], .077],
        }
        plan = [{"stage": stage, "tcp_position_base": points[stage],
                 "ik_joints_deg": joints}
                for stage in ("approach_pick", "pick", "lift_pick",
                              "hover_place", "place")]

        result = approved_motion_plan(plan, source, target,
                                      placement_offset=offset)

        self.assertEqual(list(result), [item["stage"] for item in plan])
        self.assertEqual(result["place"], joints)
        with self.assertRaisesRegex(ValueError, "stage thiếu|thiếu stage|sai thứ tự"):
            approved_motion_plan(plan[:-1], source, target,
                                 placement_offset=offset)
        plan[-1]["tcp_position_base"][0] += .001
        with self.assertRaisesRegex(ValueError, "không khớp target"):
            approved_motion_plan(plan, source, target,
                                 placement_offset=offset)

    def test_fixed_pose_does_not_reuse_unverified_stack_height(self):
        one, three, four = obj(1), obj(3, y=.07), obj(4, y=-.06)
        first_scene = scene([one, three], calibrated=False)
        second_scene = scene([four, one], calibrated=False)
        backend = PhysicalBackend([first_scene, second_scene],
                                  [fixed_lock_for(one), fixed_lock_for(three)])
        motion = Motion()
        runner = RosTaskRunner(backend, motion=motion)

        first = runner.execute("stack_cubes", {"source": "cube_1", "target": "cube_3"})
        self.assertEqual(first["status"], "executed_unverified")
        self.assertFalse(first["stack_verified"])
        self.assertEqual(backend.approval_calls[-1], ([1, 3], "stack_fixed"))

        backend.approved = [fixed_lock_for(four), fixed_lock_for(one)]
        second = runner.execute("stack_cubes", {"source": "cube_4", "target": "cube_1"})
        self.assertEqual(second["status"], "executed_unverified")
        self.assertEqual(backend.approval_calls[-1], ([4, 1], "stack_fixed"))
        second_payload = motion.calls[-1][1]
        self.assertEqual(second_payload["stack_layers"], 1)
        self.assertAlmostEqual(
            second_payload["target"]["grasp"]["tcp_position_base"][2], .047)
        second_place = next(item for item in backend.targets[-1]
                            if item["stage"] == "place")
        self.assertAlmostEqual(second_place["tcp_position_base"][2], .077)
        second_hover = next(item for item in backend.targets[-1]
                            if item["stage"] == "hover_place")
        self.assertAlmostEqual(second_hover["tcp_position_base"][2], .092)
        self.assertAlmostEqual(second_payload["hover_clearance_m"], .015)

    def test_physical_ros3d_rejects_identity_conflict_after_space(self):
        red = obj(3)
        bad = {**lock_for(red), "geometry_model_id": 2}
        backend = PhysicalBackend([scene([red])], [bad])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "vision_pick_hold", {"label": "khoi_do", "hold": True})
        self.assertEqual(result["code"], "identity_conflict")
        self.assertEqual(motion.calls, [("prepare", {"keep_pose": True})])

    def test_calibrated_scene_keeps_current_pose_and_marks_sort_source(self):
        red = obj(3)
        backend = PhysicalBackend([scene([red])], [lock_for(red)])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual(motion.calls[0], ("prepare", {"keep_pose": True}))
        self.assertEqual(motion.calls[1][1]["coordinate_source"], "calibrated_base_pose")

    def test_uncalibrated_single_pick_still_prepares_ready_pose(self):
        red = obj(3)
        backend = PhysicalBackend([scene([red], calibrated=False)], [fixed_lock_for(red)])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual(motion.calls[0], ("prepare", {}))
        self.assertEqual(motion.calls[1][1]["coordinate_source"], "fixed_ready_pose")

    def test_fixed_lock_in_calibrated_scene_returns_to_ready_before_reapproval(self):
        red = obj(3)
        backend = PhysicalBackend([scene([red])], [fixed_lock_for(red)])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual([call for call in motion.calls[:2]],
                         [("prepare", {"keep_pose": True}), ("prepare", {})])
        self.assertEqual(len(backend.approval_calls), 2)
        self.assertEqual(motion.calls[2][0], "sort_cube_3d")

    def test_physical_ros3d_can_place_or_release_a_held_cube(self):
        motion = Motion()
        runner = RosTaskRunner(Backend([]), motion=motion)
        placed = runner.execute("place_held", {"bin": "ban"})
        self.assertTrue(placed["ok"])
        self.assertEqual(motion.calls[0][0], "place")
        released = runner.execute("release_hold", {})
        self.assertTrue(released["ok"])
        self.assertEqual(motion.calls[1][0], "release")
        self.assertEqual(motion.closed, 2)

    def test_physical_ros3d_pose_start_reaches_motion_worker(self):
        motion = Motion()
        result = RosTaskRunner(Backend([]), motion=motion).execute("pose_start", {})
        self.assertTrue(result["ok"])
        self.assertEqual(motion.calls, [("prepare", {})])
        self.assertEqual(motion.closed, 1)

    def test_physical_ros3d_sort_uses_approved_tcp_and_id_zone(self):
        red = obj(3)
        backend = PhysicalBackend([scene([red])], [lock_for(red)])
        motion = Motion()
        result = RosTaskRunner(backend, motion=motion).execute(
            "sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertEqual(backend.approval_calls, [([3], "sort")])
        command, payload = motion.calls[1]
        self.assertEqual(command, "sort_cube_3d")
        self.assertEqual(payload["cube_id"], 3)
        self.assertEqual(payload["tcp_position_base"], red["grasp"]["tcp_position_base"])

    def test_read_only_mode_does_not_convert_off_or_bin_request_into_motion(self):
        backend = Backend([])
        runner = RosTaskRunner(backend)
        result = runner.execute("detection_mode", {"state": "off"})
        self.assertEqual(result["status"], "disabled")
        self.assertNotIn("scan", result)
        result = runner.execute("vision_pick_hold", {"label": "khoi_do", "hold": False})
        self.assertEqual(result["code"], "unsupported_placement")
        self.assertEqual(backend.targets, [])
        with patch("t8_assistant.dispatch", side_effect=AssertionError("manager motion")):
            self.assertIsNone(request_stop(Executor(vision=backend))[1])

    def test_red_to_id_one_requires_real_ids_and_uses_target_height(self):
        source = obj(3)
        target = obj(1, y=.07, z=.035)
        backend = Backend([scene([source, target])])
        runner = RosTaskRunner(backend)
        result = runner.execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})
        self.assertTrue(result["ok"])
        self.assertEqual(result["status"], "planned")
        self.assertFalse(result["actual_grasp_verified"])
        self.assertEqual(result["motor_commands_sent"], 0)
        stages = backend.targets[0]
        self.assertEqual([item["stage"] for item in stages],
                         ["approach_pick", "pick", "lift_pick", "hover_place", "place"])
        self.assertAlmostEqual(stages[-1]["tcp_position_base"][2], .067)

    def test_source_survives_camera_change_in_base_frame_without_pixel_exclusion(self):
        backend = Backend([scene([obj(3)]), scene([obj(1, y=.07)])])
        runner = RosTaskRunner(backend)
        first = runner.execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})
        self.assertEqual(first["code"], "needs_observation")
        self.assertEqual(backend.targets, [])  # no virtual pick before target exists
        second = runner.execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})
        self.assertTrue(second["ok"])
        self.assertEqual(second["detail"]["source"]["base_position"], [-.2, 0, .045])
        self.assertTrue(second["detail"]["requires_revalidation_before_motion"])

    def test_expired_source_is_not_reused(self):
        now = [10.0]
        backend = Backend([scene([obj(3)]), scene([obj(1, y=.07)])])
        runner = RosTaskRunner(backend, clock=lambda: now[0])
        runner.execute("search_object", {"label": "khoi_do"})
        now[0] = 26.0
        result = runner.execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})
        self.assertFalse(result["ok"])
        self.assertEqual(backend.targets, [])

    def test_uncalibrated_and_moving_scenes_never_reach_ik(self):
        for data in (scene([obj(3), obj(1, y=.07)], calibrated=False),
                     scene([{**obj(3), "stationary": False}, obj(1, y=.07)])):
            backend = Backend([data])
            result = RosTaskRunner(backend).execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})
            self.assertFalse(result["ok"])
            self.assertEqual(backend.targets, [])

    def test_same_object_ambiguous_id_and_unconfirmed_pose_fail(self):
        cases = [([obj(3)], {"source": "khoi_do", "target": "cube_3"}),
                 ([obj(3), obj(1), obj(1, track="other")], {"source": "khoi_do", "target": "cube_1"}),
                 ([{**obj(3), "confirmed": False}, obj(1)], {"source": "khoi_do", "target": "cube_1"})]
        for objects, entities in cases:
            backend = Backend([scene(objects)])
            self.assertFalse(RosTaskRunner(backend).execute("stack_cubes", entities)["ok"])
            self.assertEqual(backend.targets, [])

    def test_geometry_model_mismatch_cannot_establish_identity(self):
        backend = Backend([scene([{**obj(3), "geometry_model_id": 1}])])
        result = RosTaskRunner(backend).execute("search_object", {"label": "khoi_do"})
        self.assertFalse(result["found"])
        self.assertEqual(result["scan"]["motor_commands_sent"], 0)

    def test_new_invalid_pose_replaces_a_previous_valid_metric_pose(self):
        backend = Backend([scene([obj(3)]), scene([
            {**obj(3), "base_pose_valid": False, "grasp": None}, obj(1, y=.07)])])
        runner = RosTaskRunner(backend)
        runner.execute("search_object", {"label": "khoi_do"})
        self.assertFalse(runner.execute("stack_cubes", {"source": "khoi_do", "target": "cube_1"})["ok"])
        self.assertEqual(backend.targets, [])

    def test_ros_dispatch_never_enters_pixel_or_motion_paths(self):
        backend = Backend([scene([obj(3), obj(1, y=.07)])])
        executor = Executor(vision=backend)
        executor._ros_runner.log_path = None
        outcome = {"observation_used": True, "need_vision": True,
                   "sequence": [("stack_cubes", {"source": "khoi_do", "target": "cube_1"}, "none")]}
        with patch("t8_assistant.run_stack_sequence", side_effect=AssertionError("legacy pixel path")), \
             patch("t8_assistant.dispatch", side_effect=AssertionError("manager motion")):
            self.assertTrue(execute_plan(outcome, executor, lambda: None)[0]["ok"])
            self.assertFalse(executor.execute("open_task", {"task": "stack"})["ok"])

    def test_stack_is_one_atomic_task_and_worker_only_accepts_joint_one(self):
        def data(actions):
            return {"reply": "preview", "actions": actions, "need_vision": True, "search_query": ""}
        with self.assertRaises(InvalidPlan):
            validate_plan(data([
                {"intent": "vision_pick_hold", "entities": {"label": "khoi_do", "hold": True}},
                {"intent": "stack_cubes", "entities": {"source": "khoi_do", "target": "cube_1"}}]))
        with self.assertRaises(InvalidPlan):
            validate_plan(data([{"intent": "rotate_relative", "entities": {"joint": 2, "delta_deg": 10}}]))

    def test_scene_context_replaces_image_capture_for_gemini(self):
        data = {"reply": "preview", "actions": [{"intent": "search_object", "entities": {"label": "cube_1"}}],
                "need_vision": True, "search_query": ""}
        calls = []
        def generate(**kw):
            calls.append(kw)
            return SimpleNamespace(text=json.dumps(data))
        pipe = Pipeline(SimpleNamespace(models=SimpleNamespace(generate_content=generate)), enable_cache=False)
        with patch("t8_assistant.camera_image", side_effect=AssertionError("camera opened")):
            out, frame = run_with_vision_retry(pipe, "cube id 1 ở đâu", None,
                                              scene_provider=lambda: scene([obj(1)]))
        self.assertIsNone(frame)
        self.assertTrue(out["observation_used"])
        self.assertEqual(out["counts"].gemini, 2)
        self.assertTrue(any("ROS 3D" in str(part) for part in calls[-1]["contents"]))


if __name__ == "__main__":
    unittest.main()
