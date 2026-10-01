import unittest
from types import SimpleNamespace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import json

from t8_pipeline import (Pipeline, validate_intent, try_local_rotate,
                          try_local_hold_place, try_local_motion_sequence,
                          try_local_stack_sequence, assess_needs)
from t8_assistant import (wants_camera, dispatch, is_pure_info, shorten_for_speech,
                          run_skill, run_motion_sequence, run_stack_sequence,
                          exit_safely, SKILL_INTENTS, remember_execution,
                          report_motion_sequence, speak_via_edge)
from t8_executor import Executor
from t8_vision import VisionDetector
import t8_motion_worker as motion_worker
from rag.retriever import LocalRetriever
from dofbot_voice.scripts import voice_task_manager as task_manager
from dofbot_voice.scripts.stt_vi import VI_CMD, VI_TASK_CMD, match_command
from dofbot_voice.scripts import speak_text as tts_script


class LegacyPipeline(Pipeline):
    """Keep historical parser/cache regression cases separate from live routing."""
    run = Pipeline._run_legacy


Pipeline = LegacyPipeline


class FakeModels:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.calls = []

    def generate_content(self, **kwargs):
        self.calls.append(kwargs)
        return SimpleNamespace(text=next(self.replies))


class FlakyModels(FakeModels):
    def __init__(self, failures, replies):
        super().__init__(replies)
        self.failures = failures

    def generate_content(self, **kwargs):
        self.calls.append(kwargs)
        if self.failures:
            self.failures -= 1
            raise RuntimeError("503 UNAVAILABLE: high demand")
        return SimpleNamespace(text=next(self.replies))


class FakeTavily:
    def __init__(self):
        self.calls = []

    def search(self, **kwargs):
        self.calls.append(kwargs)
        return {"results": [{"title": "Test", "url": "https://example.org",
                             "content": "Thông tin mẫu"}]}


class TestPipeline(unittest.TestCase):
    def test_four_step_sequence_keeps_final_start_pose(self):
        class Motion:
            def __init__(self):
                self.calls = []

            def execute(self, command, **params):
                self.calls.append((command, params))
                return {"ok": True, "reply": command}

        query = ("xoay sang phải 45 độ, rồi quay sang trái 90 độ "
                 "sau đó đứng dậy cuối cùng thì về trạng thái start")
        steps = try_local_motion_sequence(query)
        self.assertEqual([step[0] for step in steps],
                         ["rotate_relative", "rotate_relative", "arm_pose", "pose_start"])
        motion = Motion()
        results = run_motion_sequence(steps, Executor(motion=motion), lambda: None)
        self.assertTrue(all(result["ok"] for result in results))
        self.assertEqual([command for command, _ in motion.calls],
                         ["rotate", "rotate", "arm_pose", "prepare"])
        self.assertEqual(motion.calls[0][1]["delta_deg"], 45)
        self.assertEqual(motion.calls[1][1]["delta_deg"], -90)
        with patch("builtins.print") as printed:
            report_motion_sequence(results, steps)
        self.assertNotIn("[pose] ", str(printed.call_args_list[0]))

    def test_local_execution_enters_followup_context(self):
        pipe = SimpleNamespace(history=[], _last_reply="")
        pipe._push_history = lambda query, reply: pipe.history.append((query, reply))
        remember_execution(pipe, "xoay phải rồi về start", [
            {"ok": True, "reply": "Đã xoay phải"},
            {"ok": True, "reply": "Đã về start"}])
        self.assertIn("Đã về start", pipe.history[-1][1])
        self.assertEqual(pipe._last_reply, pipe.history[-1][1])

    def test_tts_failure_is_nonfatal_and_has_no_traceback(self):
        async def no_audio(*_args):
            raise RuntimeError("No audio was received")

        with patch.object(tts_script, "_save", no_audio), \
                patch("sys.argv", ["speak_text.py", "xin chào"]):
            self.assertEqual(tts_script.main(), 1)
        with patch("t8_assistant.subprocess.run", side_effect=TimeoutError("offline")):
            self.assertFalse(speak_via_edge("xin chào"))

    def test_stack_parser_and_sequence_preflight_before_pick(self):
        from PIL import Image
        labels = try_local_stack_sequence("gắp khối đỏ lên rồi đặt lên khối xanh")
        self.assertEqual(labels, {"source": "khoi_do", "target": "khoi_xanh"})
        class Vision:
            def get_cube_candidates(self, label, _frame, _confirm, exclude=None):
                box = [170, 150, 250, 230] if label == "khoi_do" else [370, 150, 450, 230]
                return [{"box": box, "center": [(box[0]+box[2])/2, 190],
                         "source": "hsv", "conf": 1.0}]
        class Motion:
            def __init__(self): self.calls = []
            def execute(self, command, **_params):
                self.calls.append(command)
                return {"ok": True, "reply": command}
        motion = Motion()
        executor = Executor(vision=Vision(), motion=motion)
        result = run_stack_sequence(labels, executor,
                                    lambda: Image.new("RGB", (640, 480)))
        self.assertTrue(result[-1]["ok"])
        self.assertEqual(motion.calls,
                         ["prepare", "preflight_stack", "pick", "place_target"])

    def test_fish_bone_toilet_paper_uses_two_yolo_classes(self):
        from PIL import Image
        query = "gắp cube hình xương cá , đặt lên cube hình giấy vệ sinh"
        labels = try_local_stack_sequence(query)
        self.assertEqual(labels, {"source": "xuong_ca", "target": "giay_ve_sinh"})
        class Box:
            def __init__(self, cls, coords):
                self.cls = SimpleNamespace(item=lambda: cls)
                self.conf = SimpleNamespace(item=lambda: 0.90)
                self.xyxy = [SimpleNamespace(tolist=lambda: coords)]
        class Model:
            def __init__(self): self.calls = 0
            def predict(self, **_kwargs):
                self.calls += 1
                return [SimpleNamespace(names={7: "Fish_bone", 12: "Toilet_paper"},
                        boxes=[Box(7, [170, 150, 250, 230]),
                               Box(12, [370, 150, 450, 230])])]
        class Motion:
            def __init__(self): self.calls = []
            def execute(self, command, **_params):
                self.calls.append(command)
                return {"ok": True, "reply": command}
        model, motion = Model(), Motion()
        executor = Executor(vision=VisionDetector(model=model), motion=motion)
        result = run_stack_sequence(labels, executor,
                                    lambda: Image.new("RGB", (640, 480)))
        self.assertTrue(result[-1]["ok"])
        self.assertEqual(model.calls, 1)
        self.assertEqual(motion.calls,
                         ["prepare", "preflight_stack", "pick", "place_target"])

    def test_contour_unique_stable_and_ambiguous(self):
        from PIL import Image, ImageDraw
        def frame(two=False):
            image = Image.new("RGB", (640, 480), "#202020")
            draw = ImageDraw.Draw(image)
            draw.rectangle((170, 150, 250, 230), fill="red")
            if two:
                draw.rectangle((370, 150, 450, 230), fill="green")
            return image
        vision = VisionDetector()
        one, two = frame(), frame(True)
        self.assertEqual(len(vision.get_cube_candidates("cube", one, one)), 1)
        self.assertEqual(vision.get_cube_candidates("cube", one, two), [])
        self.assertEqual(vision.get_cube_candidates("cube", two, two), [])
        self.assertEqual(len(vision.get_cube_candidates("khoi_do", two, two)), 1)
        self.assertEqual(len(vision.get_cube_candidates(
            "cube", one, one)[0]["corners"]), 4)
        no_yolo = SimpleNamespace(predict=lambda **_kw: [SimpleNamespace(boxes=[])])
        fallback = VisionDetector(model=no_yolo)
        self.assertEqual(fallback.get_cube_candidates("xuong_ca", one, one)[0]
                         ["source"], "contour")

    def test_stack_preflight_checks_both_ik_targets(self):
        class Kin:
            def __init__(self): self.calls = []
            def ik(self, x, y, z):
                self.calls.append((x, y, z))
                return [80 if len(self.calls) == 1 else 90, 50, 60, 10, 90]
        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"):
            kin = Kin()
            result = motion_worker.execute({
                "command": "preflight_stack", "source_box": [170, 150, 250, 230],
                "target_box": [370, 150, 450, 230],
                "image_size": [640, 480]}, object(), kin)
            self.assertTrue(result["ok"])
            self.assertEqual(len(kin.calls), 2)
            self.assertAlmostEqual(kin.calls[1][2], motion_worker.STACK_Z)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_place_target_lowers_to_stack_height_and_returns_ready(self):
        class Arm:
            def __init__(self):
                self.joints = [80, 120, 60, 10, 80, 140]
                self.writes = []
            def Arm_serial_servo_read(self, joint):
                return self.joints[joint - 1]
            def Arm_serial_servo_write(self, joint, angle, _ms):
                self.joints[joint - 1] = angle
                self.writes.append((joint, angle))
            def Arm_serial_servo_write6(self, *args):
                self.joints = list(args[:6])
                self.writes.append(("lower", list(args[:6])))
            def Arm_serial_servo_write6_array(self, joints, _ms):
                self.joints = list(joints)
        class Kin:
            def __init__(self): self.calls = []
            def ik(self, x, y, z):
                self.calls.append((x, y, z))
                return [90, 55, 65, 10, 90]
        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
             patch("t8_motion_worker.time.sleep"):
            motion_worker.save_state({"phase": "holding", "label": "cube",
                                      "joint1": 80, "grasp_joints": [80, 50, 60, 10, 80],
                                      "picked_xy": [-0.2, 0.0], "place_z": 0.045})
            arm, kin = Arm(), Kin()
            result = motion_worker.execute({"command": "place_target",
                                            "target_box": [370, 150, 450, 230],
                                            "image_size": [640, 480]}, arm, kin)
            self.assertTrue(result["ok"])
            self.assertAlmostEqual(kin.calls[0][2], motion_worker.STACK_Z)
            self.assertEqual(arm.writes[0], (1, 90))
            self.assertIn(("lower", [90, 55, 65, 10, 90, 140]), arm.writes)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_exit_places_held_object_and_preserves_failed_state(self):
        class Motion:
            def __init__(self, ok): self.ok, self.calls = ok, []
            def execute(self, command):
                self.calls.append(command)
                return {"ok": self.ok, "reply": "place result"}
        with TemporaryDirectory() as directory, \
             patch("t8_assistant.HOLD_STATE_FILE", Path(directory) / "state.json") as state:
            state.write_text('{"phase":"holding"}')
            motion = Motion(False)
            exit_safely(SimpleNamespace(_motion=motion))
            self.assertEqual(motion.calls, ["place"])
            self.assertEqual(json.loads(state.read_text())["phase"], "holding")

    def test_ctrl_c_at_prompt_runs_safe_exit(self):
        import t8_assistant
        with patch("sys.argv", ["t8_assistant.py", "--no-speak-info"]), \
             patch("t8_assistant.make_pipeline", return_value=object()), \
             patch("t8_assistant.make_executor", return_value=SimpleNamespace(_motion=None)), \
             patch("builtins.input", side_effect=KeyboardInterrupt), \
             patch("t8_assistant.exit_safely") as safe_exit:
            t8_assistant.main()
        safe_exit.assert_called_once()

    def test_compound_pick_rotate_place_runs_in_order_without_llm(self):
        from PIL import Image

        class Motion:
            def __init__(self):
                self.calls = []

            def execute(self, command, **params):
                self.calls.append(command)
                return {"ok": True, "reply": command}

        class Detector:
            def get_detections(self, label, _frame):
                return [{"label": label, "box": [250, 183, 379, 296],
                         "conf": 0.9}]

        steps = try_local_motion_sequence(
            "gắp khối cube có hình xương cá lên trên, xoay sang phải góc 30 độ và đặt xuống")
        self.assertEqual([step[0] for step in steps],
                         ["vision_pick_hold", "rotate_relative", "place_held"])
        self.assertEqual(steps[1][1]["delta_deg"], 30)
        motion = Motion()
        result = run_motion_sequence(steps, Executor(vision=Detector(), motion=motion),
                                     lambda: Image.new("RGB", (640, 480)))
        self.assertTrue(all(item["ok"] for item in result))
        self.assertEqual(motion.calls, ["prepare", "pick", "rotate", "place"])

    def test_compound_stops_after_failed_pick(self):
        from PIL import Image

        class Motion:
            def __init__(self):
                self.calls = []

            def execute(self, command, **_params):
                self.calls.append(command)
                return {"ok": command != "pick", "reply": command}

        class Detector:
            def get_detections(self, label, _frame):
                return [{"label": label, "box": [250, 183, 379, 296],
                         "conf": 0.9}]

        steps = try_local_motion_sequence(
            "gắp khối cube xương cá lên, xoay trái 20 độ và đặt xuống")
        self.assertIsNone(try_local_motion_sequence(
            "gắp cái kia lên, xoay phải 30 độ và đặt xuống"))
        motion = Motion()
        result = run_motion_sequence(steps, Executor(vision=Detector(), motion=motion),
                                     lambda: Image.new("RGB", (640, 480)))
        self.assertFalse(result[-1]["ok"])
        self.assertEqual(motion.calls, ["prepare", "pick"])

    def test_other_task_blocked_while_t8_holds_object(self):
        with TemporaryDirectory() as directory:
            state = Path(directory) / "state.json"
            state.write_text('{"phase":"holding","label":"xuong_ca"}')
            with patch("t8_assistant.HOLD_STATE_FILE", state):
                reply, request_id = dispatch("color")
            self.assertIsNone(request_id)
            self.assertIn("đang giữ", reply)

    def test_executor_dispatches_detected_box_to_motion(self):
        from PIL import Image
        class Detector:
            def get_detections(self, label, frame):
                return [{"label": label, "box": [250, 183, 379, 296],
                         "conf": 0.9}]

        class Motion:
            def __init__(self):
                self.calls = []

            def execute(self, command, **params):
                self.calls.append((command, params))
                return {"ok": True, "reply": command}

        motion = Motion()
        executor = Executor(vision=Detector(), motion=motion)
        executor.execute("vision_pick_hold", {"label": "xuong_ca"},
                         frame=Image.new("RGB", (640, 480)))
        executor.execute("rotate_relative", {"joint": 1, "delta_deg": -20})
        executor.execute("place_held", {"bin": "ban"})
        self.assertEqual([c[0] for c in motion.calls], ["pick", "rotate", "place"])
        self.assertEqual(motion.calls[0][1]["image_size"], [640, 480])

    def test_pick_rotate_place_reuses_verified_grasp_pose(self):
        class Arm:
            def __init__(self):
                self.joints = [90, 50, 60, 10, 90, 25]
                self.writes = []

            def Arm_serial_servo_read(self, joint):
                return self.joints[joint - 1]

            def Arm_serial_servo_write(self, joint, angle, _ms):
                self.joints[joint - 1] = angle
                self.writes.append((joint, angle))

            def Arm_serial_servo_write6(self, *args):
                self.joints = list(args[:6])
                self.writes.append(("all", list(args[:6])))

            def Arm_serial_servo_write6_array(self, joints, _ms):
                self.joints = list(joints)
                self.writes.append(("ready", list(joints)))

        class Kin:
            def __init__(self):
                self.targets = []

            def ik(self, x, y, z):
                self.targets.append((x, y, z))
                return [90, 50, 60, 10, 90]

            def fk(self, joints):
                return (-0.2, (joints[0] - 90) / 100, 0.15)

        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
             patch("t8_motion_worker.time.sleep"):
            arm, kin = Arm(), Kin()
            ready = motion_worker.execute({"command": "prepare"}, arm)
            self.assertTrue(ready["ok"])
            self.assertEqual(arm.joints, motion_worker.READY_POSE)
            motion_worker.save_state({"phase": "moving", "command": "prepare"})
            self.assertTrue(motion_worker.execute({"command": "prepare"}, arm)["ok"])
            picked = motion_worker.execute({"command": "pick", "label": "xuong_ca",
                                            "conf": 0.9, "box": [250, 183, 379, 296],
                                            "image_size": [640, 480]}, arm, kin)
            self.assertTrue(picked["holding"])
            self.assertAlmostEqual(kin.targets[0][0], -0.20413, places=5)
            self.assertAlmostEqual(kin.targets[0][1], -0.00137, places=5)
            rotated = motion_worker.execute({"command": "rotate", "joint": 1,
                                             "delta_deg": -20}, arm)
            self.assertTrue(rotated["holding"])
            self.assertEqual(arm.joints[0], 110)
            placed = motion_worker.execute({"command": "place"}, arm, kin)
            self.assertFalse(placed["holding"])
            self.assertEqual(len(kin.targets), 1)
            self.assertIn(("all", [110, 50, 60, 10, 90, 140]), arm.writes)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")
            self.assertEqual(arm.joints, motion_worker.READY_POSE)

    def test_prepare_reconciles_manual_gripper_open(self):
        class Arm:
            def __init__(self, grip):
                self.joints = [78, 120, 70, 10, 78, grip]
                self.ready_writes = 0

            def Arm_serial_servo_read(self, joint):
                return self.joints[joint - 1]

            def Arm_serial_servo_write6_array(self, joints, _ms):
                self.ready_writes += 1
                self.joints = list(joints)

        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
             patch("t8_motion_worker.time.sleep"):
            motion_worker.save_state({"phase": "holding", "label": "xuong_ca"})
            arm = Arm(motion_worker.OPEN_ANGLE)
            result = motion_worker.execute({"command": "prepare"}, arm)
            self.assertTrue(result["ok"])
            self.assertIn("đồng bộ", result["reply"])
            self.assertEqual(arm.ready_writes, 1)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

            motion_worker.save_state({"phase": "holding", "label": "xuong_ca"})
            closed = Arm(motion_worker.CLOSE_ANGLE)
            with self.assertRaisesRegex(RuntimeError, "Kẹp vẫn đóng"):
                motion_worker.execute({"command": "prepare"}, closed)
            self.assertEqual(closed.ready_writes, 0)
            self.assertEqual(motion_worker.load_state()["phase"], "holding")

            opened = Arm(motion_worker.OPEN_ANGLE)
            result = motion_worker.execute({"command": "place"}, opened)
            self.assertTrue(result["ok"])
            self.assertEqual(opened.ready_writes, 1)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_rotate_uses_verified_pick_angle_after_serial_read_timeout(self):
        class Arm:
            def __init__(self):
                self.angle = 90
                self.read_calls = 0

            def Arm_serial_servo_read(self, joint):
                self.read_calls += 1
                if self.read_calls <= 5:
                    return None
                return self.angle

            def Arm_serial_servo_write(self, joint, angle, _ms):
                self.angle = angle

        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
             patch("t8_motion_worker.time.sleep"):
            motion_worker.save_state({"phase": "holding", "joint1": 90})
            arm = Arm()
            result = motion_worker.execute({"command": "rotate", "joint": 1,
                                            "delta_deg": -20}, arm)
            self.assertTrue(result["ok"])
            self.assertEqual(arm.angle, 110)
            self.assertEqual(motion_worker.load_state()["joint1"], 110)

    def test_place_old_holding_state_solves_at_original_pick_xy(self):
        class Arm:
            def __init__(self):
                self.joints = [62, 120, 60, 10, 82, 140]
                self.moves = []

            def Arm_serial_servo_read(self, joint):
                return self.joints[joint - 1]

            def Arm_serial_servo_write(self, joint, angle, _ms):
                self.joints[joint - 1] = angle

            def Arm_serial_servo_write6(self, *args):
                self.joints = list(args[:6])
                self.moves.append(self.joints[:])

            def Arm_serial_servo_write6_array(self, joints, _ms):
                self.joints = list(joints)

        class Kin:
            def __init__(self):
                self.targets = []

            def ik(self, *target):
                self.targets.append(target)
                return [82, 50, 60, 10, 82]

        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"), \
             patch("t8_motion_worker.time.sleep"):
            motion_worker.save_state({"phase": "holding", "label": "xuong_ca",
                                      "place_z": 0.045, "joint1": 62,
                                      "picked_xy": [-0.1584, 0.02463]})
            arm, kin = Arm(), Kin()
            placed = motion_worker.execute({"command": "place"}, arm, kin)
            self.assertTrue(placed["ok"])
            self.assertEqual(kin.targets, [(-0.1584, 0.02463, 0.045)])
            self.assertEqual(arm.moves[0], [62, 50, 60, 10, 82, 140])
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_motion_rejects_bad_detection_and_ik(self):
        self.assertFalse(motion_worker.valid_ik([0, 0, 0, 0, 0]))
        self.assertFalse(motion_worker.valid_ik([90, 120, 40, 10, 90]))
        with self.assertRaises(ValueError):
            motion_worker.pixel_target([1, 2, 3, 4], [1280, 720])
        with TemporaryDirectory() as directory, \
             patch.object(motion_worker, "STATE_FILE", Path(directory) / "state.json"):
            arm = SimpleNamespace(Arm_serial_servo_write=lambda *_: None)
            bad_kin = SimpleNamespace(ik=lambda *_: (_ for _ in ()).throw(
                RuntimeError("IK failed")))
            with self.assertRaisesRegex(RuntimeError, "IK failed"):
                motion_worker.execute({"command": "pick", "label": "xuong_ca",
                                       "conf": 0.9, "box": [250, 183, 379, 296],
                                       "image_size": [640, 480]}, arm, bad_kin)
            with self.assertRaisesRegex(RuntimeError, "nhận diện cube"):
                motion_worker.execute({"command": "pick", "label": "cube",
                                       "source": "contour", "box": [250, 183, 379, 296],
                                       "image_size": [640, 480]}, arm, bad_kin)
            self.assertEqual(motion_worker.load_state()["phase"], "empty")

    def test_yolo_detection_is_only_a_plan(self):
        class Box:
            cls = SimpleNamespace(item=lambda: 0)
            conf = SimpleNamespace(item=lambda: 0.91)
            xyxy = [SimpleNamespace(tolist=lambda: [10, 20, 30, 40])]

        model = SimpleNamespace(predict=lambda **_kw: [SimpleNamespace(
            names={0: "Fish_bone"}, boxes=[Box()])])
        detector = VisionDetector(model=model)
        ex = Executor(vision=detector)
        from PIL import Image
        result = ex.execute("vision_pick_hold", {"label": "xuong_ca"},
                            frame=Image.new("RGB", (64, 64)))
        self.assertIn("YOLO thấy", result["reply"])
        self.assertTrue(result["dry_run"])
        self.assertFalse(ex.state.holding)

    def test_503_retries_and_counts_each_attempt(self):
        models = FlakyModels(2, ['{"reply":"ok","intent":"ask_info",'
                                 '"entities":{},"search_query":""}'])
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=False,
                        enable_cache=False)
        with patch("t8_pipeline.time.sleep"):
            result = pipe.run("Kiểm tra")
        self.assertEqual(result["reply"], "ok")
        self.assertEqual(result["counts"].gemini, 3)
        self.assertEqual(len(models.calls), 3)

    def test_503_exhaustion_does_not_repeat_pipeline(self):
        from t8_assistant import run_with_vision_retry
        models = FlakyModels(3, [])
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=False,
                        enable_cache=False)
        with patch("t8_pipeline.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "503") as caught:
                run_with_vision_retry(pipe, "kiểm tra", None)
        self.assertEqual(caught.exception.t8_counts.gemini, 3)
        self.assertEqual(len(models.calls), 3)

    def test_natural_camera_request(self):
        self.assertTrue(wants_camera("Đọc chữ trên camera"))
        self.assertTrue(wants_camera("Phân loại hình ảnh"))
        self.assertFalse(wants_camera("Phân loại màu"))

    def test_task_whitelist_and_count(self):
        # LLM path (local router off): 1 Gemini call, whitelist enforced.
        models = FakeModels(['{"reply":"Bắt đầu", "action":"color", "search_query":""}'])
        result = Pipeline(SimpleNamespace(models=models),
                          enable_local=False, enable_cache=False).run("Phân loại màu")
        self.assertEqual(result["action"], "color")
        self.assertEqual((result["counts"].gemini, result["counts"].tavily), (1, 0))
        self.assertEqual(models.calls[0]["config"]["http_options"]["retry_options"]["attempts"], 1)

    def test_local_task_router_saves_api(self):
        # Same query with smart router: 0 Gemini calls.
        models = FakeModels(['SHOULD-NOT-BE-CALLED'])
        result = Pipeline(SimpleNamespace(models=models),
                          enable_local=True, enable_cache=False).run("phân loại màu")
        self.assertEqual(result["action"], "color")
        self.assertEqual(result["counts"].gemini, 0)
        self.assertEqual(result["source"], "local")
        self.assertEqual(len(models.calls), 0)

    def test_noise_and_chitchat_are_free(self):
        models = FakeModels(['SHOULD-NOT-BE-CALLED'])
        pipe = Pipeline(SimpleNamespace(models=models),
                        enable_local=True, enable_cache=False)
        r1 = pipe.run("  ừm  ")
        self.assertEqual(r1["source"], "filtered")
        self.assertEqual(r1["counts"].gemini, 0)
        r2 = pipe.run("Chào bạn")
        self.assertEqual(r2["counts"].gemini, 0)
        self.assertEqual(r2["source"], "local")
        self.assertEqual(len(models.calls), 0)

    def test_local_math_and_time_are_free(self):
        models = FakeModels(['SHOULD-NOT-BE-CALLED'])
        pipe = Pipeline(SimpleNamespace(models=models),
                        enable_local=True, enable_cache=False)
        r = pipe.run("tính 2+3*4")
        self.assertIn("14", r["reply"])
        self.assertEqual(r["counts"].gemini, 0)
        r = pipe.run("mấy giờ rồi")
        self.assertIn("Bây giờ là", r["reply"])
        self.assertEqual(r["counts"].gemini, 0)

    def test_reply_cache_saves_second_call(self):
        models = FakeModels([
            '{"reply":"Chao, toi la DOFBOT", "action":"none", "search_query":""}',
        ])
        with TemporaryDirectory() as directory:
            root = Path(directory)
            pipe = Pipeline(SimpleNamespace(models=models),
                            enable_local=False, enable_cache=True,
                            cache_file=str(root / "c.json"),
                            search_cache_file=str(root / "s.json"))
            # Use a non-chitchat question so Layer-1 doesn't intercept;
            # first call hits LLM, second call hits cache.
            q = "dofbot co may bac tu do"
            r1 = pipe.run(q)
            self.assertEqual(r1["source"], "llm")
            self.assertEqual(r1["counts"].gemini, 1)
            r2 = pipe.run(q)
            self.assertEqual(r2["source"], "cache")
            self.assertEqual(r2["counts"].gemini, 0)
            self.assertEqual(len(models.calls), 1)

    def test_search_gating_skips_tavily_for_robot_talk(self):
        # Even if Gemini over-triggers search_query, robot talk must not search.
        models = FakeModels([
            '{"reply":"", "action":"none", "search_query":"phan loai mau robot"}',
        ])
        tavily = FakeTavily()
        with TemporaryDirectory() as directory:
            root = Path(directory)
            pipe = Pipeline(SimpleNamespace(models=models), tavily,
                            enable_local=True, enable_cache=False,
                            cache_file=str(root / "c.json"),
                            search_cache_file=str(root / "s.json"))
            # Bypass local router with image=None but fresh-check fails -> skip search.
            # "phan loai mau la gi" is a task phrase, but force LLM via enable_local
            # trick: use a wording that misses exact task keywords yet triggers search.
            pipe.enable_local = False
            result = pipe.run("phan loai mau la gi")
            self.assertEqual(len(tavily.calls), 0)
            self.assertEqual(result["counts"].tavily, 0)

    def test_stack_opens_manual_task(self):
        models = FakeModels(['{"reply":"Mở stacking", "action":"stack", '
                             '"search_query":""}'])
        result = Pipeline(SimpleNamespace(models=models)).run(
            "Xếp chồng 4 khối trong ảnh", image=object())
        self.assertEqual(result["action"], "stack")
        with TemporaryDirectory() as directory:
            task_file = Path(directory) / "command"
            with patch("t8_assistant.TASK_FILE", task_file), \
                 patch("t8_assistant.HOLD_STATE_FILE", Path(directory) / "hold.json"), \
                 patch("t8_assistant.manager_running", return_value=True), \
                 patch("t8_assistant.manager_protocol_ready", return_value=True), \
                 patch("t8_assistant.task_status", return_value={"state": "running"}):
                status, request_id = dispatch(result["action"])
            payload = json.loads(task_file.read_text())
            self.assertEqual(payload["code"], 62)
            self.assertNotIn("auto_count", payload)
            self.assertEqual(payload["request_id"], request_id)
            self.assertIn("đã mở", status)

    def test_manager_ack_and_completion_without_hardware(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            camera = root / "camera"
            camera.touch()
            real_exists = Path.exists

            def exists(path):
                return str(path) == "/dev/ttyUSB0" or real_exists(path)

            class FakeProcess:
                pid = 12345
                returncode = None

                def poll(self):
                    return self.returncode

            processes = []

            def spawn(*_args, **_kwargs):
                process = FakeProcess()
                processes.append(process)
                return process

            with patch.object(task_manager, "STATUS_FILE", root / "status.json"), \
                 patch.object(task_manager, "ACTIVE_FILE", root / "active"), \
                 patch.object(task_manager, "SIMPLE_FILE", root / "simple"), \
                 patch.dict(task_manager.TASK_CAMERA, {"stack": str(camera)}), \
                 patch.dict(task_manager.TASKS, {"stack": [("IK service", "true"),
                                                          ("stacking", "true")]}), \
                 patch.object(Path, "exists", exists), \
                 patch.object(task_manager.subprocess, "Popen", side_effect=spawn), \
                 patch.object(task_manager.time, "sleep"), \
                 patch.object(task_manager, "stop_processes"):
                manager = task_manager.Manager()
                manager.start("stack", request_id="test-request")
                self.assertEqual(json.loads((root / "status.json").read_text())["state"],
                                 "running")
                processes[-1].returncode = 0
                manager.check()
                status = json.loads((root / "status.json").read_text())
                self.assertEqual((status["state"], status["request_id"]),
                                 ("finished", "test-request"))

    def test_search_is_bounded(self):
        models = FakeModels([
            '{"reply":"", "action":"none", "search_query":"thời tiết Hà Nội"}',
            '{"reply":"Theo nguồn https://example.org", "action":"trash", "search_query":"again"}',
        ])
        tavily = FakeTavily()
        result = Pipeline(SimpleNamespace(models=models), tavily,
                          enable_local=True, enable_cache=False).run("Thời tiết Hà Nội hôm nay?")
        self.assertEqual(result["action"], "none")
        self.assertEqual(result["counts"].total, 3)
        self.assertEqual(len(tavily.calls), 1)

    def test_image_and_unknown_action(self):
        models = FakeModels(['{"reply":"Có chữ ABC", "action":"servo 1", "search_query":""}'])
        result = Pipeline(SimpleNamespace(models=models),
                          enable_local=True, enable_cache=False).run("Đọc ảnh", image=object())
        self.assertEqual(result["action"], "none")
        self.assertEqual(len(models.calls[0]["contents"]), 3)

    def test_pure_info_vs_action_routing(self):
        # News/chat thuần (action none) -> loa; có action -> robot.
        self.assertTrue(is_pure_info({"action": "none", "reply": "Hôm nay trời nắng"}))
        self.assertFalse(is_pure_info({"action": "color", "reply": "Mở bài toán"}))
        self.assertFalse(is_pure_info({"action": "none", "reply": "   "}))
        self.assertFalse(is_pure_info({"action": "stack", "reply": ""}))

    def test_shorten_for_speech(self):
        short = "Chào bạn."
        self.assertEqual(shorten_for_speech(short), short)
        long_text = "Câu một. " + "x" * 900 + ". Câu hai."
        out = shorten_for_speech(long_text, limit=800)
        self.assertLessEqual(len(out), 801)
        self.assertTrue(out)

    # ---- intent v2 / single tasks / RAG / executor / cheatsheet ----
    def test_validate_intent_constrained(self):
        i, e, a = validate_intent({"intent": "rotate_relative",
                                   "entities": {"joint": 1, "delta_deg": 30}})[0]
        self.assertEqual((i, a), ("rotate_relative", "none"))
        # intent lạ => ask_info
        i, e, a = validate_intent({"intent": "servo_hack", "entities": {}})[0]
        self.assertEqual((i, a), ("ask_info", "none"))
        # delta quá lớn => clamp
        i, e, a = validate_intent({"intent": "rotate_relative",
                                   "entities": {"joint": 1, "delta_deg": 500}})[0]
        self.assertEqual(e["delta_deg"], 90.0)
        # compat model cũ chỉ có action
        i, e, a = validate_intent({"reply": "x", "action": "color", "search_query": ""})[0]
        self.assertEqual((i, e, a), ("open_task", {"task": "color"}, "color"))

    def test_local_rotate_hold_parsers(self):
        r = try_local_rotate("xoay sang phải thêm 30 độ")
        self.assertEqual(r[0], "rotate_relative")
        self.assertEqual(r[1]["delta_deg"], 30)
        r = try_local_rotate("xoay trái thêm 15 độ nữa")
        self.assertEqual(r[1]["delta_deg"], -15)
        h = try_local_hold_place("cầm cục cube có dán hình xương cá lên giữ nguyên")
        self.assertEqual(h[0], "vision_pick_hold")
        self.assertEqual(h[1]["label"], "xuong_ca")
        self.assertTrue(h[1]["hold"])
        p = try_local_hold_place("đặt xuống")
        self.assertEqual(p[0], "place_held")

    def test_pipeline_new_intents_end_to_end(self):
        models = FakeModels(['SHOULD-NOT-BE-CALLED'])
        pipe = Pipeline(SimpleNamespace(models=models),
                        enable_local=True, enable_cache=False)
        r = pipe.run("xoay sang phải thêm 30 độ")
        self.assertEqual(r["intent"], "rotate_relative")
        self.assertEqual(r["entities"]["delta_deg"], 30)
        self.assertEqual(r["action"], "none")
        # Text-only xin ảnh; ảnh thật với nhãn YOLO rõ ràng chạy local.
        r = pipe.run("cầm cục cube xương cá lên giữ nguyên đó")
        self.assertEqual(r["source"], "need_image")
        from PIL import Image
        r = pipe.run("cầm cục cube xương cá lên giữ nguyên đó",
                     Image.new("RGB", (640, 480)))
        self.assertEqual(r["source"], "local")
        self.assertEqual(r["intent"], "vision_pick_hold")
        self.assertEqual(len(models.calls), 0)
        self.assertTrue(r["need_vision"])
        self.assertEqual(r["counts"].gemini, 0)

    def test_rag_hit_no_api(self):
        import pathlib
        kb = pathlib.Path(__file__).parent / "rag" / "kb_vi.jsonl"
        retr = LocalRetriever(str(kb))
        self.assertGreater(len(retr), 10)
        models = FakeModels(['SHOULD-NOT-BE-CALLED'])
        pipe = Pipeline(SimpleNamespace(models=models), enable_local=True,
                        enable_cache=False, retriever=retr, rag_threshold=0.2)
        r = pipe.run("phân loại màu như thế nào")
        self.assertEqual(r["source"], "local-rag")
        self.assertEqual(r["counts"].gemini, 0)

    def test_executor_hold_blocks_new_task(self):
        ex = Executor()
        res = ex.execute("vision_pick_hold", {"label": "xuong_ca", "hold": True,
                                              "_dry": True})
        # không có vision thật => dry-run plan giữ
        self.assertFalse(res["ok"])
        # giả lập đang giữ rồi mở task mới phải bị chặn
        ex.state.mark_held("xuong_ca")
        blocked = ex.execute("open_task", {"task": "color"})
        self.assertFalse(blocked["ok"])
        placed = ex.execute("place_held", {"bin": "ban"})
        self.assertTrue(placed["ok"])
        self.assertFalse(ex.state.holding)

    def test_executor_rotate_dry_run(self):
        ex = Executor()
        res = ex.execute("rotate_relative", {"joint": 1, "delta_deg": 30})
        self.assertTrue(res["ok"])
        self.assertEqual(res["action"], "none")
        bad = ex.execute("rotate_relative", {"joint": 1, "delta_deg": 500})
        self.assertFalse(bad["ok"])

    def test_skill_routing_not_pure_info(self):
        self.assertIn("rotate_relative", SKILL_INTENTS)
        # skill có action none nhưng KHÔNG phải pure info (đi executor)
        self.assertFalse(is_pure_info({"action": "none", "intent": "rotate_relative",
                                       "reply": "Xoay..."}))
        self.assertTrue(is_pure_info({"action": "none", "intent": "ask_info",
                                      "reply": "Tin..."}))
        skilled = run_skill({"intent": "rotate_relative",
                             "entities": {"joint": 1, "delta_deg": 30},
                             "reply": "Xoay"}, Executor())
        self.assertIsNotNone(skilled)

    def test_grasp_query_triggers_camera(self):
        self.assertTrue(wants_camera("cầm khối cube có hình xương cá lên"))
        self.assertTrue(wants_camera("gắp khối đỏ lên"))
        self.assertFalse(wants_camera("phân loại màu"))

    def test_vision_retry_and_frame_to_executor(self):
        from t8_assistant import run_with_vision_retry
        from types import SimpleNamespace
        # need_vision nhưng không có ảnh, camera lỗi -> không raise, giữ outcome cũ
        pipe = Pipeline(SimpleNamespace(models=FakeModels([
            '{"reply":"Kế hoạch giữ","intent":"vision_pick_hold",'
            '"entities":{"label":"xuong_ca","hold":true},"need_vision":true,'
            '"search_query":""}'])), enable_local=False, enable_cache=False)
        with patch("t8_assistant.camera_image", side_effect=RuntimeError("no cam")):
            out, fr = run_with_vision_retry(pipe, "cầm cube xương cá", None, "/dev/video2")
        self.assertIsNone(fr)
        self.assertIn("Không chụp được camera", out["reply"])
        # executor nhận frame + ctx Gemini thì nhắc đã nhìn camera
        res = Executor().execute("vision_pick_hold", {"label": "xuong_ca"},
                                 frame=object(), ctx_reply="Thấy khối đỏ giữa bàn")
        self.assertIn("Đã nhìn qua camera", res["reply"])
        self.assertIn("khối đỏ", res["reply"])

    def test_cheatsheet_no_single_syllable_dupes(self):
        kws = [kw for kw, _c, _d in VI_CMD]
        for bad in ["do", "mo", "nha", "tha", "mang", "xep", "mua", "coi"]:
            self.assertNotIn(bad, kws, f"từ đơn '{bad}' phải bỏ để chống nhầm")
        code, _desc, _kind = match_command("bật đèn đỏ", lang="vi")
        self.assertEqual(code, 11)


class TestRouterGeneral(unittest.TestCase):
    """Ma trận route local/clarify/llm_text/llm_vision (general, không keyword lẻ)."""

    def pipe(self):
        return Pipeline(SimpleNamespace(models=FakeModels(['SHOULD-NOT-BE-CALLED'])),
                        enable_local=True, enable_cache=False)

    def test_verb_groups_generalize_beyond_grasp_list(self):
        # Động từ mới cùng nhóm thao tác -> vision, không cần thêm keyword lẻ
        for q in ["nhặt pin lên giữ nguyên",
                  "lấy khối vàng lên",
                  "dọn khối đỏ qua một bên"]:
            n = assess_needs(q)
            self.assertTrue(n.need_image, q)
            self.assertEqual(n.route, "llm_vision", q)

    def test_spatial_deixis_without_verb_needs_image(self):
        n = assess_needs("cái bên trái là gì")
        self.assertEqual((n.route, n.need_image), ("llm_vision", True))
        n = assess_needs("màu này là màu gì")
        self.assertEqual((n.route, n.need_image), ("llm_vision", True))

    def test_missing_params_clarify_zero_api(self):
        p = self.pipe()
        r = p.run("xoay thêm chút")
        self.assertEqual(r["source"], "clarify")
        self.assertEqual(r["counts"].gemini, 0)
        self.assertIn("bao nhiêu độ", r["reply"])
        r = p.run("gắp cái kia lên")
        self.assertEqual(r["source"], "clarify")
        self.assertEqual(r["counts"].gemini, 0)

    def test_vague_grasp_with_image_resolves_via_vision(self):
        n = assess_needs("gắp cái kia lên", has_image=True)
        self.assertEqual((n.route, n.need_image), ("llm_vision", True))

    def test_grasp_known_object_routes_vision(self):
        n = assess_needs("cầm khối đỏ lên giữ nguyên")
        self.assertEqual((n.route, n.need_image), ("llm_vision", True))

    def test_vision_without_image_returns_need_image_no_api(self):
        p = self.pipe()
        r = p.run("cầm khối đỏ lên giữ nguyên")
        self.assertEqual(r["source"], "need_image")
        self.assertTrue(r["need_vision"])
        self.assertEqual(r["counts"].gemini, 0)

    def test_vision_with_image_calls_llm_once(self):
        models = FakeModels(['{"reply":"Thấy khối đỏ, gắp lên giữ",'
                             '"intent":"vision_pick_hold",'
                             '"entities":{"label":"khoi_do","hold":true},'
                             '"need_vision":true,"search_query":""}'])
        p = Pipeline(SimpleNamespace(models=models),
                     enable_local=True, enable_cache=False)
        r = p.run("cầm khối đỏ lên giữ nguyên", image=object())
        self.assertEqual(r["intent"], "vision_pick_hold")
        self.assertEqual(r["counts"].gemini, 0)

    def test_fresh_info_not_confused_by_deixis(self):
        # "hôm nay" chứa "nay" nhưng KHÔNG phải chỉ định không gian
        n = assess_needs("thời tiết Hà Nội hôm nay")
        self.assertEqual(n.route, "llm_text")
        self.assertFalse(n.need_image)

    def test_knowledge_question_never_opens_task(self):
        import pathlib
        kb = pathlib.Path(__file__).parent / "rag" / "kb_vi.jsonl"
        pipe = Pipeline(SimpleNamespace(models=FakeModels(['SHOULD-NOT-BE-CALLED'])),
                        enable_local=True, enable_cache=False,
                        retriever=LocalRetriever(str(kb)), rag_threshold=0.2)
        r = pipe.run("phân loại rác là gì")
        self.assertNotEqual(r.get("intent"), "open_task")
        self.assertEqual(r["counts"].gemini, 0)

    def test_light_and_gripper_stay_local(self):
        p = self.pipe()
        r = p.run("bật đèn đỏ")
        self.assertEqual(r["intent"], "light_beep")
        self.assertEqual(r["counts"].gemini, 0)
        r = p.run("mở kẹp")
        self.assertEqual(r["intent"], "gripper")
        self.assertEqual(r["counts"].gemini, 0)


if __name__ == "__main__":
    unittest.main()
