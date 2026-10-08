"""Lệnh 'tất cả cube': thứ tự, khảo sát zone một lần, bỏ cube lỗi riêng, dừng khi lỗi hệ thống, không treo."""
import copy
import time
import unittest

from t8_ros_tasks import RosTaskRunner
from test_handeye_mode import lock, obj, scene

POS = {1: (-0.20, 0.00), 2: (-0.18, 0.04), 3: (-0.22, -0.04), 4: (-0.17, -0.02)}


class Backend:
    """Scene cố định; approve trả khóa đúng cube được xin hoặc lỗi theo kịch bản."""
    backend = "ros3d"

    def __init__(self, ids=(1, 2, 3, 4), failures=None):
        self.objects = {i: obj(i, *POS[i]) for i in ids}
        self.failures = dict(failures or {})          # {cube_id: mã lỗi approve} (một lần)
        self.approved, self.timeouts, self.approval_timeout = [], [], 0.0
        self.zone_check = False

    def snapshot(self, expect=None):
        return copy.deepcopy(scene(list(self.objects.values())))

    def preflight(self, targets):
        return {"ok": True, "stages": targets, "collision_checked": False}

    def approve(self, ids, operation="pick_hold", notice="", **options):
        self.timeouts.append(self.approval_timeout)
        self.approved.append(tuple(ids))
        code = self.failures.pop(ids[0], None)
        if code:
            return {"ok": False, "code": code, "reason": code}
        now = time.time()
        return {"ok": True, "approval_token": "a" * 32, "scene_stamp": 100.0, "approved_at": now,
                "expires_at": now + 15, "objects": [lock(self.objects[i]) for i in ids]}


class Motion:
    def __init__(self, fail_on=None):
        self.calls, self.fail_on = [], fail_on

    def execute(self, command, **payload):
        self.calls.append((command, payload))
        if command == self.fail_on:
            return {"ok": False, "reply": "servo lỗi"}
        return {"ok": True, "status": "executed_unverified", "reply": f"Đã {command}."}

    def close(self):
        pass


def runner(backend=None, motion=None):
    task = RosTaskRunner(backend or Backend(), motion=motion or Motion())
    task.visible_cubes = lambda settle_s=8.0, _o=task.visible_cubes: _o(settle_s=0.0)
    return task


def sorted_ids(motion):
    return [p["cube_id"] for c, p in motion.calls if c == "sort_cube_3d"]


class Sorting(unittest.TestCase):
    def test_every_visible_cube_is_sorted_once_in_planned_order(self):
        backend, motion = Backend(), Motion()
        result = runner(backend, motion).execute("sort_cube", {"label": "all"})
        self.assertTrue(result["ok"], result["reply"])
        self.assertEqual(sorted(sorted_ids(motion)), [1, 2, 3, 4])
        self.assertEqual(len(sorted_ids(motion)), 4)
        self.assertEqual(result["code"], "batch_complete")
        self.assertEqual(len(result["batch"]), 4)
        self.assertIn("xong 4/4", result["reply"])

    def test_per_cube_viewer_timeout_skips_that_cube_and_continues(self):
        backend = Backend(failures={2: "approval_timeout"})
        motion = Motion()
        result = runner(backend, motion).execute("sort_cube", {"label": "all"})
        self.assertEqual(sorted(sorted_ids(motion)), [1, 3, 4])
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "batch_partial")
        self.assertIn("xong 3/4", result["reply"])

    def test_user_cancel_stops_the_whole_batch(self):
        backend = Backend(failures={2: "approval_cancelled"})        # thứ tự dự kiến: 4, 2, 1, 3
        motion = Motion()
        result = runner(backend, motion).execute("sort_cube", {"label": "all"})
        self.assertEqual(sorted_ids(motion), [4])
        self.assertFalse(result["ok"])
        self.assertIn("chưa làm do lô dừng", result["reply"])

    def test_hardware_failure_stops_instead_of_pushing_on(self):
        motion = Motion(fail_on="sort_cube_3d")
        result = runner(Backend(), motion).execute("sort_cube", {"label": "all"})
        self.assertEqual(len(sorted_ids(motion)), 1)               # đúng một lần thử rồi dừng
        self.assertFalse(result["ok"])

    def test_batch_gives_the_viewer_a_timeout_and_restores_it(self):
        backend = Backend(ids=(1, 2))
        task = runner(backend)
        task.batch_approval_timeout = 42.0
        task.execute("sort_cube", {"label": "all"})
        self.assertEqual(set(backend.timeouts), {42.0})
        self.assertEqual(backend.approval_timeout, 0.0)

    def test_user_timeout_is_kept_when_already_set(self):
        backend = Backend(ids=(1,))
        backend.approval_timeout = 15.0
        runner(backend).execute("sort_cube", {"label": "all"})
        self.assertEqual(set(backend.timeouts), {15.0})

    def test_survey_runs_once_for_the_whole_batch(self):
        backend, motion = Backend(ids=(1, 3)), Motion()
        backend.zone_check = True
        calls = []
        backend.zone_survey = lambda zones, timeout_s=12.0, expect_j1=None: (
            calls.append([z["zone_id"] for z in zones]) or {"ok": True, "zones": [
                {"zone_id": z["zone_id"], "seen": True, "area_m2": 0.0053, "center_xy": z["release_xy"],
                 "margin_cfg_mm": 25.0} for z in zones]})
        orig = motion.execute

        def execute(command, **payload):
            if command == "look":
                return {"ok": True, "servo": payload["servo"], "from_servo": [90, 100, 20, 10, 90]}
            return orig(command, **payload)
        motion.execute = execute
        result = runner(backend, motion).execute("sort_cube", {"label": "all"})
        self.assertTrue(result["ok"], result["reply"])
        self.assertEqual(sum(1 for c, _ in motion.calls if c == "sort_cube_3d"), 2)
        looks = [c for c in calls]
        self.assertLessEqual(len(looks), 4)                       # hai phía (không nhân theo số cube)

    def test_no_cube_visible_does_nothing(self):
        motion = Motion()
        result = runner(Backend(ids=()), motion).execute("sort_cube", {"label": "all"})
        self.assertFalse(result["ok"])
        self.assertEqual(sorted_ids(motion), [])

    def test_single_label_still_takes_the_single_path(self):
        motion = Motion()
        result = runner(Backend(ids=(3,)), motion).execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertNotIn("batch", result)

    def test_dry_run_only_describes_the_plan(self):
        task = RosTaskRunner(Backend(), motion=None)
        task.visible_cubes = lambda settle_s=3.0, _o=task.visible_cubes: _o(settle_s=0.0)
        result = task.execute("sort_cube", {"label": "all"})
        self.assertEqual(result["status"], "planned")
        self.assertIn("cube 1", result["reply"])


class Stacking(unittest.TestCase):
    def test_tower_is_built_bottom_up_each_step_targets_the_previous_top(self):
        backend, motion = Backend(ids=(1, 2, 3)), Motion()
        task = runner(backend, motion)
        verified = []

        def verify(source, target, after_stamp, expected=None, timeout_s=8.0):
            verified.append((source, target))
            return {"ok": True}
        task.verify_stack = verify
        result = task.execute("stack_cubes", {"source": "all", "target": "all"})
        self.assertEqual(len(verified), 2)
        self.assertEqual(verified[1][1], verified[0][0])          # bước 2 lên cube vừa đặt ở bước 1
        self.assertTrue(result["ok"], result["reply"])

    def test_explicit_base_cube(self):
        backend, motion = Backend(ids=(1, 2, 3)), Motion()
        task = runner(backend, motion)
        seen = []
        task.verify_stack = lambda s, t, a, expected=None, timeout_s=8.0: seen.append((s, t)) or {"ok": True}
        task.execute("stack_cubes", {"source": "all", "target": "cube_1"})
        self.assertEqual(seen[0][1], 1)

    def test_single_cube_cannot_form_a_tower(self):
        result = runner(Backend(ids=(1,))).execute("stack_cubes", {"source": "all", "target": "all"})
        self.assertFalse(result["ok"])

    def test_unverified_stack_stops_the_tower(self):
        backend, motion = Backend(ids=(1, 2, 3, 4)), Motion()
        task = runner(backend, motion)
        task.max_retries = 0
        task.verify_stack = lambda *a, **k: {"ok": False, "reason": "không thấy"}
        result = task.execute("stack_cubes", {"source": "all", "target": "all"})
        self.assertEqual(sum(1 for c, _ in motion.calls if c == "stack_cube_3d"), 1)
        self.assertFalse(result["ok"])


class Plumbing(unittest.TestCase):
    def test_planner_accepts_all_for_sort_and_stack(self):
        import t8_pipeline as P
        base = {"reply": "ok", "need_vision": True, "search_query": ""}
        sort = P.validate_plan({**base, "actions": [{"intent": "sort_cube", "entities": {"label": "all"}}]})
        self.assertEqual(sort, [("sort_cube", {"label": "all"}, "none")])
        stack = P.validate_plan({**base, "actions": [
            {"intent": "stack_cubes", "entities": {"source": "all", "target": "all"}}]})
        self.assertEqual(stack[0][1], {"source": "all", "target": "all"})

    def test_batch_that_runs_out_of_time_stops_and_reports(self):
        backend, motion = Backend(), Motion()
        task = runner(backend, motion)
        task.batch_deadline_s = -1.0
        result = task.execute("sort_cube", {"label": "all"})
        self.assertEqual(sorted_ids(motion), [])
        self.assertIn("quá thời gian", result["reply"])


if __name__ == "__main__":
    unittest.main()
