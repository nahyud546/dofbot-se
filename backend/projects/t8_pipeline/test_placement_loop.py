"""Vòng thực thi: kiểm tra camera ngoài sau khi thả, thử lại khi lệch/rơi, dừng sau max_retries."""
import unittest

from t8_ros_tasks import RosTaskRunner
from test_handeye_mode import Backend, lock, obj, scene


def check(verdict):
    return {"verdict": verdict, "ok": verdict == "in_zone", "text": verdict, "metrics": {}}


class Verifier:
    def __init__(self, verdicts, before_ok=True):
        self.verdicts, self.before_calls, self.checks, self.before_ok = list(verdicts), 0, [], before_ok

    def before(self):
        self.before_calls += 1
        return self.before_ok

    def check(self, zone):
        self.checks.append(zone)
        return check(self.verdicts.pop(0))


class Motion:
    def __init__(self):
        self.calls, self.payloads = [], []

    def execute(self, command, **payload):
        self.calls.append(command)
        self.payloads.append(payload)
        return {"ok": True, "status": "executed_unverified", "reply": "Đã thả cube."}

    def close(self):
        pass


def runner(verdicts, retries=2, before_ok=True):
    red = obj(3)
    backend = Backend([scene([red]) for _ in range(6)], [lock(red)])
    backend.approve_calls = 0
    original = backend.approve

    def approve(*a, **k):
        backend.approve_calls += 1
        return original(*a, **k)
    backend.approve = approve
    verifier = Verifier(verdicts, before_ok)
    return RosTaskRunner(backend, motion=Motion(), placement_verifier=verifier, max_retries=retries), backend, verifier


class Loop(unittest.TestCase):
    def test_correct_placement_runs_once_and_is_reported(self):
        task, backend, verifier = runner(["in_zone"])
        result = task.execute("sort_cube", {"label": "khoi_do"})
        self.assertEqual(result["placement_check"]["verdict"], "in_zone")
        self.assertIn("gắp chính xác", result["reply"])
        self.assertEqual((backend.approve_calls, verifier.before_calls, result.get("retries")), (1, 1, None))

    def test_dropped_cube_is_retried_and_succeeds_second_time(self):
        task, backend, verifier = runner(["outside", "in_zone"])
        result = task.execute("sort_cube", {"label": "khoi_do"})
        self.assertEqual(result["retries"], 1)
        self.assertEqual(backend.approve_calls, 2)                  # mỗi lần thử lại vẫn phải duyệt viewer
        self.assertTrue(result["placement_check"]["ok"])

    def test_gives_up_after_max_retries_and_says_so(self):
        task, backend, _ = runner(["outside", "partial", "outside"], retries=2)
        result = task.execute("sort_cube", {"label": "khoi_do"})
        self.assertEqual(result["retries"], 2)
        self.assertEqual(backend.approve_calls, 3)
        self.assertIn("Đã thử lại 2 lần vẫn chưa đúng", result["reply"])

    def test_retry_releases_gentler_each_time(self):
        task, _, _ = runner(["outside", "missing", "in_zone"], retries=2)
        task.execute("sort_cube", {"label": "khoi_do"})
        sorts = [p for c, p in zip(task.motion.calls, task.motion.payloads) if c == "sort_cube_3d"]
        self.assertEqual([p.get("release_gentleness") for p in sorts], [None, 1, 2])

    def test_missing_cube_is_retried_but_unseen_is_not(self):
        task, backend, _ = runner(["missing", "in_zone"])
        task.execute("sort_cube", {"label": "khoi_do"})
        self.assertEqual(backend.approve_calls, 2)

    def test_bounced_cube_reason_is_named(self):
        self.assertEqual(RosTaskRunner.needs_retry(
            {"ok": True, "placement_check": {"verdict": "outside", "bounced": True}}),
            "cube vào ô rồi bị văng ra")

    def test_metric_offset_is_reported_only_when_both_cube_xy_and_zone_target_exist(self):
        task, _, verifier = runner([])
        verifier.check = lambda zone: {**check("in_zone"), "cube_xy": [-0.08, 0.17]}
        self.assertNotIn("cách tâm ô", task.execute("sort_cube", {"label": "khoi_do"})["reply"])   # không có target đo

    def test_unseen_is_not_retried(self):
        task, backend, _ = runner(["unseen"])
        result = task.execute("sort_cube", {"label": "khoi_do"})
        self.assertEqual(backend.approve_calls, 1)
        self.assertIn("chưa xác nhận", result["reply"])

    def test_no_external_frame_skips_verification_without_failing(self):
        task, backend, verifier = runner([], before_ok=False)
        result = task.execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertNotIn("placement_check", result)
        self.assertEqual(verifier.checks, [])

    def test_without_a_verifier_nothing_changes(self):
        red = obj(3)
        backend = Backend([scene([red])], [lock(red)])
        result = RosTaskRunner(backend, motion=Motion()).execute("sort_cube", {"label": "khoi_do"})
        self.assertTrue(result["ok"])
        self.assertNotIn("placement_check", result)


class StackRetry(unittest.TestCase):
    def test_unverified_stack_triggers_retry_but_unverified_fixed_pose_does_not(self):
        checked = {"ok": True, "stack_verified": False, "status": "executed_unverified",
                   "detail": {"verification": {"ok": False}}}
        self.assertTrue(RosTaskRunner.needs_retry(checked))                    # scene đã kiểm và thấy không nằm trên đích
        self.assertFalse(RosTaskRunner.needs_retry({"ok": True, "stack_verified": False,
                                                    "status": "executed_unverified", "detail": {}}))
        self.assertFalse(RosTaskRunner.needs_retry({"ok": True, "stack_verified": True}))
        self.assertFalse(RosTaskRunner.needs_retry({"ok": False, "stack_verified": False}))


if __name__ == "__main__":
    unittest.main()
