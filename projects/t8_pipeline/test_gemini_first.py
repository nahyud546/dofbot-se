import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from t8_pipeline import InvalidPlan, Pipeline, validate_plan
from t8_assistant import emergency_command, execute_plan, run_with_vision_retry


def plan(actions=None, need_vision=False, reply="Đã hiểu.", search_query=""):
    return json.dumps({"reply": reply, "actions": actions or [],
                       "need_vision": need_vision, "search_query": search_query},
                      ensure_ascii=False)


class Models:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.calls = []

    def generate_content(self, **kwargs):
        self.calls.append(kwargs)
        return SimpleNamespace(text=next(self.replies))


class GeminiFirstTests(unittest.TestCase):
    def test_each_query_calls_gemini_even_with_local_and_cache_enabled(self):
        models = Models([plan(reply="Một"), plan(reply="Hai")])
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=True,
                        enable_cache=True)
        self.assertEqual(pipe.run("xin chào")["reply"], "Một")
        self.assertEqual(pipe.run("xin chào")["reply"], "Hai")
        self.assertEqual(len(models.calls), 2)

    def test_invalid_middle_step_rejects_whole_plan(self):
        actions = [{"intent": "rotate_relative", "entities": {"joint": 1, "delta_deg": 45}},
                   {"intent": "rotate_relative", "entities": {"joint": 1, "delta_deg": 180}}]
        with self.assertRaises(InvalidPlan):
            validate_plan(json.loads(plan(actions)))

    def test_one_repair_then_fail_closed(self):
        bad = plan([{"intent": "rotate_relative", "entities": {"joint": 1,
                                                               "delta_deg": 180}}])
        models = Models([bad, plan(reply="Bạn muốn xoay bao nhiêu độ?")])
        out = Pipeline(SimpleNamespace(models=models), enable_cache=False).run("xoay đi")
        self.assertEqual(out["sequence"], [])
        self.assertEqual(out["counts"].gemini, 2)

    def test_malformed_json_is_repaired_before_execution(self):
        models = Models(["not json", plan(reply="Bạn nói rõ góc xoay giúp tôi.")])
        out = Pipeline(SimpleNamespace(models=models), enable_cache=False).run("xoay")
        self.assertEqual(out["sequence"], [])
        self.assertEqual(out["counts"].gemini, 2)

    def test_vision_two_pass_and_no_move_without_frame(self):
        act = [{"intent": "vision_pick_hold", "entities": {"label": "khoi_do", "hold": True}}]
        models = Models([plan(act, True), plan(act, True)])
        pipe = Pipeline(SimpleNamespace(models=models), enable_cache=False)
        with patch("t8_assistant.camera_image", return_value=object()):
            out, frame = run_with_vision_retry(pipe, "gắp khối đỏ", None)
        self.assertIsNotNone(frame)
        self.assertTrue(out["image_used"])
        self.assertEqual(len(models.calls), 2)
        pending = {"need_vision": True, "sequence": [("vision_pick_hold", act[0]["entities"], "none")]}
        self.assertFalse(execute_plan(pending, None, lambda: None)[0]["ok"])

    def test_stack_uses_special_preflight_path(self):
        outcome = {"image_used": True, "need_vision": True,
                   "sequence": [("stack_cubes", {"source": "khoi_do", "target": "khoi_xanh"}, "none")]}
        with patch("t8_assistant.run_stack_sequence", return_value=[{"ok": True, "reply": "xong"}]) as runner:
            self.assertTrue(execute_plan(outcome, object(), lambda: object())[0]["ok"])
        self.assertEqual(runner.call_args.args[0], {"source": "khoi_do", "target": "khoi_xanh"})

    def test_emergency_stop_is_standalone_only(self):
        self.assertEqual(emergency_command("dừng ngay"), "stop")
        self.assertIsNone(emergency_command("đừng xoay"))
        self.assertIsNone(emergency_command("nếu cần thì dừng"))

    def test_dispatch_stops_after_failed_motion_step(self):
        steps = [("rotate_relative", {"joint": 1, "delta_deg": 45}, "none"),
                 ("arm_pose", {"pose": "up"}, "none")]
        with patch("t8_assistant.run_motion_sequence", return_value=[{"ok": False,
                                                                    "reply": "servo lỗi"}]) as runner:
            results = execute_plan({"sequence": steps}, object(), lambda: None)
        self.assertEqual(len(results), 1)
        self.assertEqual(runner.call_count, 1)


if __name__ == "__main__":
    unittest.main()
