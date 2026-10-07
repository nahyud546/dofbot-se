import json
import subprocess
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from t8_pipeline import InvalidPlan, Pipeline, validate_plan
from t8_assistant import emergency_command, execute_plan, run_with_vision_retry
import t8_motion_worker as motion_worker
from t8_motion import MotionBridge


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
    def test_pose_start_is_zero_call_even_in_llm_first_mode(self):
        models = Models([])
        out = Pipeline(SimpleNamespace(models=models), enable_local=False,
                       enable_cache=False).run("về start pose")
        self.assertEqual(out["intent"], "pose_start")
        self.assertEqual(out["source"], "local-safe-control")
        self.assertEqual(out["counts"].gemini, 0)
        self.assertEqual(models.calls, [])

    def test_motion_bridge_reuses_a_live_worker(self):
        worker_code = (
            "import json, os, sys\n"
            "print('T8_READY:' + json.dumps({'ok': True}), flush=True)\n"
            "for line in sys.stdin:\n"
            " data = json.loads(line)\n"
            " print('T8_RESULT:' + json.dumps({'ok': True, 'pid': os.getpid(), "
            "'command': data['command']}), flush=True)\n"
        )
        real_popen = subprocess.Popen
        def fake_popen(_command, **kwargs):
            return real_popen([sys.executable, "-u", "-c", worker_code], **kwargs)

        with patch("t8_motion.subprocess.Popen", side_effect=fake_popen):
            bridge = MotionBridge()
            try:
                self.assertTrue(bridge.start()["ok"])
                first = bridge.execute("rotate", joint=1, delta_deg=30)
                second = bridge.execute("prepare")
                self.assertEqual(first["pid"], second["pid"])
                self.assertEqual([first["command"], second["command"]],
                                 ["rotate", "prepare"])
            finally:
                bridge.close()
            self.assertIsNone(bridge.proc)

    def test_single_rotation_returns_to_start_in_same_worker_request(self):
        class Motion:
            def __init__(self):
                self.calls = []

            def execute_sequence(self, commands):
                self.calls.append(commands)
                return [{"ok": True, "reply": item["command"]} for item in commands]

        motion = Motion()
        result = execute_plan({"sequence": [("rotate_relative", {"joint": 1,
                         "delta_deg": 30}, "none")]}, SimpleNamespace(_motion=motion), lambda: None)
        self.assertEqual([item["command"] for item in motion.calls[0]],
                         ["rotate", "prepare"])
        self.assertEqual(result[-1]["reply"], "prepare")

    def test_holding_object_skips_auto_home(self):
        class Motion:
            def __init__(self):
                self.calls = []

            def execute_sequence(self, commands):
                self.calls.append(commands)
                return [{"ok": True, "holding": True, "reply": "Đã xoay"}]

        motion = Motion()
        with TemporaryDirectory() as directory:
            state = Path(directory) / "state.json"
            state.write_text('{"phase":"holding"}')
            with patch("t8_assistant.HOLD_STATE_FILE", state):
                result = execute_plan({"sequence": [("rotate_relative", {"joint": 1,
                                 "delta_deg": 30}, "none")]},
                                      SimpleNamespace(_motion=motion), lambda: None)
        self.assertEqual([item["command"] for item in motion.calls[0]], ["rotate"])
        self.assertTrue(result[-1]["holding"])

    def test_compound_motion_uses_one_worker_and_returns_to_start(self):
        class Motion:
            def __init__(self):
                self.calls = []

            def execute_sequence(self, commands):
                self.calls.append(commands)
                return [{"ok": True, "reply": command["command"]} for command in commands]

        motion = Motion()
        executor = SimpleNamespace(_motion=motion)
        steps = [("pose_start", {}, "none"),
                 ("rotate_relative", {"joint": 1, "delta_deg": -30}, "none"),
                 ("rotate_relative", {"joint": 1, "delta_deg": 45}, "none"),
                 ("arm_pose", {"pose": "up"}, "none")]
        result = execute_plan({"sequence": steps}, executor, lambda: None)
        self.assertEqual(len(motion.calls), 1)
        self.assertEqual([item["command"] for item in motion.calls[0]],
                         ["prepare", "rotate", "rotate", "arm_pose", "prepare"])
        self.assertEqual(len(result), 5)

    def test_arm_pose_fails_when_readback_does_not_move(self):
        class Arm:
            def Arm_serial_servo_read(self, joint):
                return [90, 125, 0, 0, 90, 25][joint - 1]

            def Arm_serial_servo_write6_array(self, joints, ms):
                pass

        with TemporaryDirectory() as directory, \
                patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
                patch("t8_motion_worker.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "Khớp chưa tới đích"):
                motion_worker.execute({"command": "arm_pose", "pose": "up"}, Arm())
            self.assertEqual(motion_worker.load_state()["phase"], "moving")

    def test_arm_pose_confirms_motion_before_success(self):
        class Arm:
            def __init__(self):
                self.joints = [90, 125, 0, 0, 90, 25]

            def Arm_serial_servo_read(self, joint):
                return self.joints[joint - 1]

            def Arm_serial_servo_write6_array(self, joints, ms):
                self.joints = list(joints)

        with TemporaryDirectory() as directory, \
                patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
                patch("t8_motion_worker.time.sleep"):
            arm = Arm()
            result = motion_worker.execute({"command": "arm_pose", "pose": "up"}, arm)
            self.assertTrue(result["ok"])
            self.assertEqual(arm.joints[1:4], [90, 90, 90])
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_each_query_calls_gemini_even_with_local_and_cache_enabled(self):
        # Tối ưu mới: chitchat/local + dedup 0-call (ép clarify, giảm request).
        models = Models([plan(reply="Một"), plan(reply="Hai")])
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=True,
                        enable_cache=True)
        self.assertEqual(pipe.run("xin chào")["source"], "local")
        self.assertEqual(pipe.run("xin chào")["source"], "filtered")
        self.assertEqual(len(models.calls), 0)

    def test_invalid_middle_step_rejects_whole_plan(self):
        actions = [{"intent": "rotate_relative", "entities": {"joint": 1, "delta_deg": 45}},
                   {"intent": "rotate_relative", "entities": {"joint": 1, "delta_deg": 180}}]
        with self.assertRaises(InvalidPlan):
            validate_plan(json.loads(plan(actions)))

    def test_one_repair_then_fail_closed(self):
        bad = plan([{"intent": "rotate_relative", "entities": {"joint": 1,
                                                               "delta_deg": 180}}])
        models = Models([bad, plan(reply="Bạn muốn xoay bao nhiêu độ?")])
        out = Pipeline(SimpleNamespace(models=models), enable_local=False,
                       enable_cache=False).run("xoay đi")
        self.assertEqual(out["sequence"], [])
        self.assertEqual(out["counts"].gemini, 2)

    def test_malformed_json_is_repaired_before_execution(self):
        models = Models(["not json", plan(reply="Bạn nói rõ góc xoay giúp tôi.")])
        out = Pipeline(SimpleNamespace(models=models), enable_local=False,
                       enable_cache=False).run("xoay")
        self.assertEqual(out["sequence"], [])
        self.assertEqual(out["counts"].gemini, 2)

    def test_vision_two_pass_and_no_move_without_frame(self):
        # Tối ưu mới: pre-capture 1-call (assess_needs cần ảnh -> chụp trước).
        act = [{"intent": "vision_pick_hold", "entities": {"label": "khoi_do", "hold": True}}]
        models = Models([plan(act, True)])
        pipe = Pipeline(SimpleNamespace(models=models), enable_cache=False)
        with patch("t8_assistant.camera_image", return_value=object()):
            out, frame = run_with_vision_retry(pipe, "gắp khối đỏ", None)
        self.assertIsNotNone(frame)
        self.assertTrue(out["image_used"])
        self.assertEqual(len(models.calls), 1)
        pending = {"need_vision": True, "sequence": [("vision_pick_hold", act[0]["entities"], "none")]}
        self.assertFalse(execute_plan(pending, None, lambda: None)[0]["ok"])

    def test_stack_uses_special_preflight_path(self):
        outcome = {"image_used": True, "need_vision": True,
                   "sequence": [("stack_cubes", {"source": "khoi_do", "target": "khoi_xanh"}, "none")]}
        with patch("t8_assistant.run_stack_sequence", return_value=[{"ok": True, "reply": "xong"}]) as runner:
            self.assertTrue(execute_plan(outcome, object(), lambda: object())[0]["ok"])
        self.assertEqual(runner.call_args.args[0], {"source": "khoi_do", "target": "khoi_xanh"})

    def test_compound_pick_rotate_place_skips_rag_for_gemini(self):
        # Hồi quy: lệnh ghép gắp-xoay-đặt từng bị RAG cướp rồi trả lời chay.
        import pathlib
        three = [ {"intent": "vision_pick_hold",
                   "entities": {"label": "khoi_do", "hold": True}},
                  {"intent": "rotate_relative",
                   "entities": {"joint": 1, "delta_deg": 45}},
                  {"intent": "place_held", "entities": {"bin": "ban"}} ]
        models = Models([plan(three, True, reply="rõ")])
        from rag.retriever import LocalRetriever
        kb = pathlib.Path(__file__).parent / "rag" / "kb_vi.jsonl"
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=True,
                        enable_cache=False,
                        retriever=LocalRetriever(str(kb)), rag_threshold=0.2)
        with patch("t8_assistant.camera_image", return_value=object()):
            out, frame = run_with_vision_retry(
                pipe, "gắp cube màu đỏ xoay sang phải 45 độ rồi đặt xuống", None)
        self.assertEqual(out["source"], "llm")
        self.assertEqual([s[0] for s in out["sequence"]],
                         ["vision_pick_hold", "rotate_relative", "place_held"])
        self.assertEqual(len(models.calls), 1)

    def test_sequential_stacks_allowed_but_not_mixed_with_pick(self):
        import json
        from t8_pipeline import validate_plan, InvalidPlan
        two = json.loads(plan(
            [{"intent": "stack_cubes",
              "entities": {"source": "khoi_xanh_duong", "target": "cube_4"}},
             {"intent": "stack_cubes",
              "entities": {"source": "cube", "target": "khoi_xanh_duong"}}],
            need_vision=True))
        steps = validate_plan(two)
        self.assertEqual([s[0] for s in steps], ["stack_cubes", "stack_cubes"])
        mixed = json.loads(plan(
            [{"intent": "search_object", "entities": {"label": "khoi_do"}},
             {"intent": "stack_cubes",
              "entities": {"source": "khoi_do", "target": "cube_1"}}],
            need_vision=True))
        with self.assertRaises(InvalidPlan):
            validate_plan(mixed)

    def test_patterned_cube_without_class_falls_back_to_generic_cube(self):
        from cube_identity import canonical_label
        self.assertEqual(canonical_label("cube syringe"), "cube")
        self.assertEqual(canonical_label("blue"), "khoi_xanh_duong")

    def test_label_maps_to_terminal1_object_ids(self):
        from cube_identity import object_ids_for_label
        self.assertEqual(object_ids_for_label("cube_4"), {4})
        self.assertEqual(object_ids_for_label("cube id 1"), {1})
        self.assertEqual(object_ids_for_label("khoi_vang"), {4})
        self.assertEqual(object_ids_for_label("yellow"), {4})
        self.assertEqual(object_ids_for_label("blue"), {1})
        self.assertEqual(object_ids_for_label("red cube"), {3})
        self.assertEqual(object_ids_for_label("cube"), set())
        self.assertEqual(object_ids_for_label("syringe"), set())
        self.assertEqual(object_ids_for_label("cube_9"), set())

    def test_ros3d_match_needs_single_confirmed_object(self):
        from cube_identity import select_ros3d_match
        def obj(oid, confirmed=True, box=(10, 10, 100, 100)):
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            return {"object_id": oid, "confirmed": confirmed,
                    "bbox_xyxy": list(box), "center_px": [cx, cy]}
        good = obj(4)
        self.assertIs(select_ros3d_match([good], {4}), good)
        self.assertIsNone(select_ros3d_match([obj(4, confirmed=False)], {4}))
        self.assertIsNone(select_ros3d_match([obj(4, box=(0, 0, 0, 0))], {4}))
        self.assertIsNone(select_ros3d_match([obj(2)], {4}))
        self.assertIsNone(select_ros3d_match([obj(4), obj(4)], {4}))
        near = {"center": good["center_px"], "box": good["bbox_xyxy"]}
        self.assertIsNone(select_ros3d_match([good], {4}, exclude=near))

    def test_ros3d_detect_builds_pixel_detection(self):
        from t8_assistant import ros3d_detect
        snap = {"ok": True, "objects": [
            {"object_id": 4, "confirmed": True, "bbox_xyxy": [148, 127, 291, 299],
             "center_px": [219.5, 213.0], "identity_confidence": 0.93,
             "track_id": "t7", "pose_method": "apriltag_ippe",
             "base_position": [0.1, 0.2, 0.3]}]}
        with patch("t8_assistant._ros3d_snapshot", return_value=snap) as grab:
            det = ros3d_detect({4}, "cube_4")
        self.assertEqual(det["box"], [148, 127, 291, 299])
        self.assertEqual(det["source"], "ros3d")
        self.assertEqual(det["object_id"], 4)
        self.assertTrue(det["geometry_verified"])
        self.assertEqual(grab.call_count, 1)
        with patch("t8_assistant._ros3d_snapshot", return_value=snap):
            self.assertIsNone(ros3d_detect({1}, "cube_1"))
        with patch("t8_assistant._ros3d_snapshot",
                   side_effect=AssertionError("khong goi snapshot")):
            self.assertIsNone(ros3d_detect(set(), "cube"))

    def test_english_color_labels_normalize_to_khoi(self):
        import json
        from t8_pipeline import validate_plan
        data = json.loads(plan(
            [{"intent": "stack_cubes",
              "entities": {"source": "blue", "target": "red cube"}}],
            need_vision=True))
        steps = validate_plan(data)
        self.assertEqual((steps[0][1]["source"], steps[0][1]["target"]),
                         ("khoi_xanh_duong", "khoi_do"))

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
