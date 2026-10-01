#!/usr/bin/env python3
"""T8 text/voice assistant: Gemini JSON planning and safe task dispatch."""

import argparse
from collections import deque
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import site
import subprocess
import sys
import termios
import time
import uuid

from t8_pipeline import ACTIONS, make_pipeline, norm_nodau


ROOT = Path(__file__).resolve().parents[1]
DOF_VOICE_DIR = ROOT.parent / "vendor/yahboom/dofbot_voice"
TASK_FILE = Path("/tmp/dofbot_task_command")
LOCK_FILE = Path("/tmp/dofbot_task_manager.lock")
STATUS_FILE = Path("/tmp/dofbot_task_status.json")
HOLD_STATE_FILE = Path("/tmp/t8_hold_state.json")
# Pure-info (news/chat, action == "none") is spoken via loa.
# Long web answers are truncated at a sentence boundary so Edge TTS
# doesn't block the loop for minutes.
MAX_SPEAK_CHARS = 800


def emergency_command(query):
    """Only explicit standalone stop/exit commands bypass Gemini."""
    text = norm_nodau(query).strip(" .!?")
    if text in {"exit", "quit", "thoat"}:
        return "exit"
    if text in {"dung", "dung ngay", "dung lai", "dung robot",
                "dung bai toan", "stop", "stop now", "stop robot"}:
        return "stop"
    return None


def wants_camera(query):
    """Legacy helper for callers; live routing is now Gemini-only."""
    if query.startswith("/see "):
        return True
    from t8_pipeline import assess_needs
    return bool(assess_needs(query).need_image)


def enable_user_sherpa():
    """Expose the existing user-site Sherpa install to an isolated venv."""
    if importlib.util.find_spec("sherpa_onnx") is not None:
        return
    user_site = Path(site.getusersitepackages())
    if (user_site / "sherpa_onnx").is_dir():
        site.addsitedir(str(user_site))
    if importlib.util.find_spec("sherpa_onnx") is None:
        raise RuntimeError(
            "Thiếu sherpa_onnx trong .venv và Python người dùng. "
            "Cài bằng: python -m pip install sherpa-onnx")


def manager_running():
    with LOCK_FILE.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(lock, fcntl.LOCK_UN)
            return False
        except BlockingIOError:
            return True


def manager_protocol_ready():
    try:
        pid = int(LOCK_FILE.read_text().strip())
        status = json.loads(STATUS_FILE.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return False
    return (status.get("protocol") == 1 and status.get("manager_pid") == pid
            and Path(f"/proc/{pid}").exists())


def dispatch(action):
    code = ACTIONS[action]
    if code is None:
        return "", None
    if action != "stop":
        try:
            hold = json.loads(HOLD_STATE_FILE.read_text())
        except FileNotFoundError:
            hold = {}
        except (OSError, ValueError):
            hold = {"phase": "moving"}
        if hold.get("phase") in ("holding", "moving"):
            return "Tay T8 đang giữ vật hoặc chưa rõ trạng thái; đặt/thả vật trước khi mở bài toán khác.", None
    if not manager_running():
        return "Task manager chưa chạy; không gửi lệnh robot.", None
    if not manager_protocol_ready():
        return "Manager đang chạy bản cũ; dừng manager đó và khởi động lại T8.", None
    request_id = uuid.uuid4().hex
    temporary = TASK_FILE.with_name(f"{TASK_FILE.name}.{os.getpid()}.tmp")
    payload = {"code": code, "request_id": request_id}
    temporary.write_text(json.dumps(payload))
    os.replace(temporary, TASK_FILE)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        status = task_status(request_id)
        if status and status.get("state") in ("running", "finished"):
            return f"Task {action} đã mở (code {code}).", request_id
        if status and status.get("state") in ("rejected", "failed"):
            return f"Task {action} không chạy: {status.get('detail', status['state'])}", None
        if not manager_running():
            return "Task manager đã thoát trước khi xác nhận lệnh.", None
        time.sleep(0.2)
    return "Task manager không xác nhận trong 20 giây; kiểm tra log manager.", None


def task_status(request_id):
    try:
        status = json.loads(STATUS_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    return status if status.get("request_id") == request_id else None


def is_pure_info(outcome):
    """True khi response là thông tin thuần (news/chat), không kèm action/skill.

    Quy ước: intent == ask_info + action == "none" + reply non-empty => phát ra loa.
    Skill (rotate/hold/...) có action none nhưng intent khác => đi executor, không loa trực tiếp.
    Action task (color/stack/...) => dispatch robot như thường.
    """
    if outcome.get("intent", "ask_info") != "ask_info":
        return False
    return outcome.get("action", "none") == "none" and bool(
        (outcome.get("reply") or "").strip())


SKILL_INTENTS = {"rotate_relative", "vision_pick_hold", "place_held",
                 "release_hold", "gripper", "light_beep", "pose_start", "arm_pose"}


def make_executor(enable_motion=True):
    """Executor an toàn cho skill mới. Trả None nếu import lỗi (chạy legacy)."""
    try:
        from t8_executor import Executor
        from t8_motion import MotionBridge
        from t8_vision import VisionDetector
        return Executor(vision=VisionDetector(),
                        motion=MotionBridge() if enable_motion else None)
    except Exception as exc:
        print(f"[executor] không khởi tạo được, chạy legacy: {exc}",
              file=sys.stderr, flush=True)
        return None


def run_skill(outcome, executor, frame=None, confirm_frame=None):
    """Chạy skill mới qua executor. Trả (text_show, text_speak, result) hoặc None."""
    intent = outcome.get("intent", "ask_info")
    if intent not in SKILL_INTENTS or executor is None:
        return None
    try:
        res = executor.execute(intent, outcome.get("entities") or {},
                               frame=frame, ctx_reply=outcome.get("reply", ""),
                               confirm_frame=confirm_frame)
    except Exception as exc:
        return (f"Lỗi skill {intent}: {exc}", None, {"ok": False})
    # Ưu tiên reply của executor (có dry-run/state), fallback reply pipeline
    show = res.get("reply") or outcome.get("reply", "")
    # skill không phát loa dài tự động như tin tức; chỉ nói kết quả ngắn
    speak = show if res.get("ok") else show
    return (show, speak, res)


def run_motion_sequence(steps, executor, get_frame, prepare=True):
    """Execute a recognized compound command; stop at the first failed step."""
    results = []
    if executor is None:
        return [{"ok": False, "reply": "Bộ điều khiển T8 chưa sẵn sàng."}]

    has_vision = any(s[0] == "vision_pick_hold" for s in steps)

    if prepare and has_vision:
        if executor._motion is None:
            return [{"ok": False, "reply": "Bộ điều khiển chuyển động chưa sẵn sàng."}]
        ready = executor._motion.execute("prepare")
        results.append(ready)
        if not ready.get("ok"):
            return results

    frame = None
    if has_vision:
        try:
            frame = get_frame()
        except Exception as exc:
            results.append({"ok": False, "reply": f"Không chụp được ảnh camera: {exc}"})
            return results
        if frame is None:
            results.append({"ok": False, "reply": "Chưa có ảnh camera mới để gắp."})
            return results

    for index, (intent, entities, _reply) in enumerate(steps):
        try:
            result = executor.execute(intent, entities,
                                      frame=frame if intent == "vision_pick_hold" else None,
                                      confirm_frame=get_frame if intent == "vision_pick_hold" else None)
        except Exception as exc:
            result = {"ok": False, "reply": f"Bước {index + 1} lỗi: {exc}"}
        results.append(result)
        if not result.get("ok"):
            break
    return results


def execute_plan(outcome, executor, get_frame, prepare=True):
    """Single dispatcher for CLI and ROS; stop at the first failed step."""
    steps = outcome.get("sequence") or []
    if not steps:
        return [{"ok": True, "reply": outcome.get("reply", "")}]
    if outcome.get("need_vision") and not outcome.get("image_used"):
        return [{"ok": False, "reply": "Chưa có ảnh mới; không chạy kế hoạch vision."}]
    results = []
    for intent, entities, _ in steps:
        if intent == "ask_info":
            results.append({"ok": True, "reply": outcome.get("reply", "")})
        elif intent == "stack_cubes":
            results.extend(run_stack_sequence(entities, executor, get_frame, prepare=prepare))
        elif intent in {"open_task", "stop_task"}:
            action = entities["task"] if intent == "open_task" else "stop"
            status, request_id = dispatch(action)
            results.append({"ok": request_id is not None, "reply": status,
                            "task_request_id": request_id, "task_action": action})
        else:
            results.extend(run_motion_sequence([(intent, entities, "")], executor,
                                               get_frame, prepare=prepare))
        if not results[-1].get("ok"):
            break
    return results


def report_motion_sequence(results, steps):
    """Print only an actual prepare result as [pose], not a four-step command."""
    for index, result in enumerate(results):
        prefix = "[pose] " if index == 0 and len(results) == len(steps) + 1 else ""
        print(prefix + result.get("reply", "Lệnh không có phản hồi"), flush=True)


def remember_execution(pipe, query, results):
    """Put real local execution results into the next turn's LLM context."""
    if not results:
        return
    summary = "; ".join(str(item.get("reply", "")) for item in results)
    if len(summary) > 800:
        summary = summary[:797] + "..."
    pipe._push_history(query, summary)
    pipe._last_reply = summary


def run_stack_sequence(labels, executor, get_frame, prepare=True):
    """Identify both cubes, preflight IK, then pick and place on target face."""
    results = []
    if executor is None or executor._vision is None:
        return [{"ok": False, "reply": "Bộ nhận diện T8 chưa sẵn sàng."}]
    if labels["source"] == labels["target"] == "cube":
        return [{"ok": False, "reply":
                 "Hai cube chưa phân biệt được; hãy nêu màu hoặc họa tiết của một khối."}]
    if prepare:
        ready = executor._motion.execute("prepare")
        results.append(ready)
        if not ready.get("ok"):
            return results
    try:
        frame = get_frame()
        if frame is None:
            raise RuntimeError("Camera chưa có ảnh mới")
        # Geometric fallback needs a second independent camera frame.
        second = get_frame()
        if second is None:
            raise RuntimeError("Không chụp được ảnh xác nhận")
        vision = executor._vision
        if labels["target"] == "cube":
            source_list = vision.get_cube_candidates(labels["source"], frame, second)
            if len(source_list) != 1:
                raise RuntimeError(f"Nguồn {labels['source']}: {len(source_list)} "
                                   "ứng viên hợp lệ; chưa gắp")
            source = source_list[0]
            target_list = vision.get_cube_candidates(labels["target"], frame, second,
                                                      exclude=source)
        else:
            target_list = vision.get_cube_candidates(labels["target"], frame, second)
            if len(target_list) != 1:
                raise RuntimeError(f"Đích {labels['target']}: {len(target_list)} "
                                   "ứng viên hợp lệ; chưa gắp")
            target = target_list[0]
            source_list = vision.get_cube_candidates(labels["source"], frame, second,
                                                      exclude=target)
        if len(source_list) != 1 or len(target_list) != 1:
            raise RuntimeError(f"Nguồn {labels['source']}: {len(source_list)}; "
                               f"đích {labels['target']}: {len(target_list)} "
                               "ứng viên hợp lệ; chưa gắp")
        source, target = source_list[0], target_list[0]
        import math
        if math.dist(source["center"], target["center"]) < 55:
            raise RuntimeError("Cube nguồn và mặt đích trùng hoặc quá gần nhau")
        from t8_motion_worker import pixel_target, STACK_Z
        source_xyz = pixel_target(source["box"], list(frame.size))
        target_xyz = pixel_target(target["box"], list(frame.size))
        detail = {"source": source, "source_xyz_m": source_xyz,
                  "target": target, "target_xyz_m":
                  [target_xyz[0], target_xyz[1], STACK_Z]}
        if executor._motion is None:
            results.append({"ok": True, "dry_run": True, "detail": detail,
                            "reply": f"[dry-run] Nguồn {source['source']} {source['box']} "
                                     f"góc {source.get('corners')}; đích {target['source']} "
                                     f"{target['box']} góc {target.get('corners')}; "
                                     f"XYZ nguồn {source_xyz}, XYZ đích "
                                     f"{detail['target_xyz_m']}. Chưa gửi lệnh robot."})
            return results
        motion = executor._motion
        preflight = motion.execute("preflight_stack", source_box=source["box"],
                                   target_box=target["box"], image_size=list(frame.size))
        results.append(preflight)
        if not preflight.get("ok"):
            return results
        picked = motion.execute("pick", label=labels["source"], box=source["box"],
                                conf=source.get("conf", 0), source=source["source"],
                                geometry_verified=source.get("geometry_verified", False),
                                image_size=list(frame.size))
        results.append(picked)
        if not picked.get("ok"):
            return results
        placed = motion.execute("place_target", target_box=target["box"],
                                image_size=list(frame.size))
        results.append(placed)
        return results
    except Exception as exc:
        results.append({"ok": False, "reply": str(exc)})
        return results


def run_with_vision_retry(pipe, query, frame, camera_device="/dev/video2"):
    """Chạy pipeline; nếu outcome cần vision mà chưa có ảnh thì chụp 1 frame
    rồi chạy lại đúng 1 lần để Gemini vision phân tích (tốn thêm 1 Gemini call).

    Trả (outcome, frame). Không bao giờ raise vì lỗi camera — trả outcome cũ
    kèm cảnh báo trong reply nếu không chụp được.
    """
    outcome = pipe.run(query, frame)
    if frame is None and outcome.get("need_vision") and not outcome.get("image_used"):
        try:
            frame2 = camera_image(camera_device)
        except Exception as exc:
            outcome["reply"] = (f"Không chụp được camera {camera_device}: {exc}. "
                                "Chưa thể xác định vật để gắp; kiểm tra camera rồi thử lại.")
            return outcome, None
        outcome2 = pipe.run(query, frame2)
        outcome2["image_used"] = True
        for name in ("gemini", "tavily", "local", "cached"):
            setattr(outcome2["counts"], name,
                    getattr(outcome2["counts"], name) + getattr(outcome["counts"], name))
        return outcome2, frame2
    return outcome, frame


def short_error(exc):
    """Rút gọn lỗi API thành 1 dòng tiếng Việt (không dump JSON raw)."""
    msg = str(exc)
    if any(m in msg for m in ("503", "UNAVAILABLE", "overloaded")):
        return ("Gemini đang quá tải (503) — đã tự thử lại nhưng chưa được. "
                "Đợi 1–2 phút rồi nói lại câu đó giúp tôi.")
    if "429" in msg or "RESOURCE_EXHAUSTED" in msg:
        return "Gemini báo hết quota tạm thời (429). Nghỉ một lúc rồi thử lại."
    if "API_KEY" in msg or "API key" in msg:
        return "Lỗi GEMINI_API_KEY (thiếu/sai). Kiểm tra file .env."
    short = msg.strip().split("\n")[0][:200]
    return f"Lỗi gọi Gemini: {short}"


def shorten_for_speech(text, limit=MAX_SPEAK_CHARS):
    """Cắt tin dài cho TTS: ưu tiên ngắt ở hết câu trong giới hạn."""
    text = (text or "").strip()
    if len(text) <= limit:
        return text
    cut = text[:limit]
    for sep in (". ", "! ", "? ", ".\n", "\n"):
        pos = cut.rfind(sep)
        if pos > limit * 0.5:
            return cut[:pos + 1].strip()
    return cut.rstrip() + "…"


def speak_via_edge(text):
    """Phát text thuần ra loa laptop qua Edge TTS. Trả về True/False."""
    text = shorten_for_speech(text)
    if not text:
        return False
    try:
        result = subprocess.run([sys.executable,
                                 str(DOF_VOICE_DIR / "scripts/speak_text.py"),
                                 text], check=False, stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=25)
        if result.returncode != 0:
            detail = (result.stderr or "").strip().splitlines()
            reason = detail[-1] if detail else f"mã {result.returncode}"
            print(f"[TTS bỏ qua] {reason}",
                  file=sys.stderr, flush=True)
            return False
        print("[TTS] 1 synthesis job (info -> loa); "
              "số HTTP nội bộ phụ thuộc Edge TTS", flush=True)
        return True
    except Exception as exc:
        print(f"[TTS lỗi] {exc}", file=sys.stderr, flush=True)
        return False


def wait_for_task(request_id):
    print("[task] Đang chạy; đợi bài toán kết thúc trước khi nhận lệnh mới. "
          "Đóng cửa sổ task hoặc Ctrl+C để dừng.", flush=True)
    while True:
        status = task_status(request_id)
        if status and status.get("state") in ("finished", "failed"):
            detail = status.get("detail", "")
            print(f"[task] {status['state']}: {detail}", flush=True)
            return
        if not manager_running():
            print("[task] manager đã thoát; bài toán không còn được theo dõi.", flush=True)
            return
        time.sleep(0.3)


def image_from_file(path):
    from PIL import Image
    image = Image.open(path)
    image.load()
    return image


def camera_image(device):
    import cv2
    from PIL import Image
    # Reuse the ROS camera publisher when a task already owns /dev/video2.
    if Path("/tmp/dofbot_task_active").exists():
        try:
            import rclpy
            from rclpy.wait_for_message import wait_for_message
            from sensor_msgs.msg import Image as RosImage
            rclpy.init()
            node = rclpy.create_node("t8_camera_snapshot")
            try:
                ok, message = wait_for_message(RosImage, node, "/image_raw", time_to_wait=1.5)
                if ok and message.encoding.lower() in ("bgr8", "rgb8"):
                    import numpy as np
                    channels = 3
                    array = np.frombuffer(message.data, dtype=np.uint8).reshape(
                        message.height, message.step)[:, :message.width * channels]
                    array = array.reshape(message.height, message.width, channels)
                    if message.encoding.lower() == "bgr8":
                        array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
                    return Image.fromarray(array.copy())
            finally:
                node.destroy_node()
                rclpy.shutdown()
        except (ImportError, RuntimeError, ValueError, AttributeError):
            pass
    cap = cv2.VideoCapture(device)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Không mở được camera {device}")
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Không đọc được frame từ {device}")
        return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    finally:
        cap.release()


def restore_terminal(saved):
    """Undo terminal changes made by camera/audio child processes before input."""
    if saved is not None and sys.stdin.isatty():
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, saved)


def exit_safely(executor):
    """Place a held object before leaving; keep persisted state on failure."""
    try:
        phase = json.loads(HOLD_STATE_FILE.read_text()).get("phase", "empty")
    except FileNotFoundError:
        phase = "empty"
    except (OSError, ValueError):
        phase = "moving"
    if phase == "empty":
        print("T8 dừng", flush=True)
        return
    if phase != "holding" or executor is None or executor._motion is None:
        print("T8 dừng; trạng thái tay chưa rõ, cần kiểm tra trước khi chạy tiếp.", flush=True)
        return
    try:
        result = executor._motion.execute("place")
    except KeyboardInterrupt:
        # MotionBridge lets the isolated worker finish before relaying Ctrl+C.
        try:
            settled = json.loads(HOLD_STATE_FILE.read_text()).get("phase")
        except (OSError, ValueError):
            settled = "moving"
        result = {"ok": settled == "empty",
                  "reply": "Đã đặt và xác nhận tay trống." if settled == "empty"
                  else "Chưa xác nhận được bước đặt; kiểm tra tay máy."}
    if result.get("ok"):
        print("[thoát] " + result["reply"], flush=True)
    else:
        try:
            after = json.loads(HOLD_STATE_FILE.read_text()).get("phase")
        except (OSError, ValueError):
            after = "moving"
        status = ("Kẹp và trạng thái giữ vật được giữ nguyên." if after == "holding"
                  else "Trạng thái chuyển động chưa rõ; kiểm tra tay máy trước lần chạy sau.")
        print("[thoát] Không đặt được vật: " + result.get("reply", "lỗi không rõ") +
              ". " + status, flush=True)
    print("T8 dừng", flush=True)


def ros_loop(pipe, speak=False, speak_info=True, executor=None):
    """Bridge existing text_chat/asr and camera topics without legacy action_service.

    Routing:
    - skill mới (rotate/hold/...): executor an toàn, text_response = kết quả.
    - pure info (ask_info, action none): text_response = reply, tts_topic = reply (loa).
    - action task (color/stack/...): dispatch robot như thường, text_response = status.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image as RosImage
    import numpy as np
    from PIL import Image

    class Bridge(Node):
        def __init__(self):
            super().__init__("t8_assistant")
            self.queries = deque()
            self.frame = None
            self.frame_time = 0.0
            self.create_subscription(String, "asr", self.on_asr, 10)
            self.create_subscription(RosImage, "/image_raw", self.on_image, 1)
            self.text_pub = self.create_publisher(String, "text_response", 10)
            self.tts_pub = self.create_publisher(String, "tts_topic", 10)

        def on_asr(self, msg):
            if msg.data.strip():
                self.queries.append(msg.data.strip())

        def on_image(self, msg):
            if msg.encoding.lower() not in ("rgb8", "bgr8"):
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding.lower() == "bgr8":
                frame = frame[:, :, ::-1]
            self.frame = Image.fromarray(frame.copy())
            self.frame_time = time.monotonic()

    rclpy.init()
    node = Bridge()
    try:
        print("[T8 ROS] nghe /asr, /image_raw; trả /text_response, /tts_topic", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.queries:
                continue
            query = node.queries.popleft()
            if emergency_command(query) == "stop":
                status, _ = dispatch("stop")
                node.text_pub.publish(String(data=status))
                continue
            if emergency_command(query) == "exit":
                exit_safely(executor)
                break
            see = query.startswith("/see ")
            if query.startswith("/see "):
                query = query[5:]
            counts = None
            source = "?"
            try:
                def fresh_frame():
                    deadline = time.monotonic() + 3.0
                    while node.frame is None or time.monotonic() - node.frame_time > 2.0:
                        if time.monotonic() >= deadline:
                            raise RuntimeError("Chưa có ảnh camera mới từ /image_raw")
                        rclpy.spin_once(node, timeout_sec=0.1)
                    return node.frame

                outcome = pipe.run(query, fresh_frame() if see else None)
                if outcome.get("need_vision") and not outcome.get("image_used"):
                    outcome = pipe.run(query, fresh_frame())
                counts = outcome["counts"]
                source = outcome.get("source", "?")
                results = execute_plan(outcome, executor, fresh_frame)
                for result in results:
                    reply = result.get("reply", "")
                    if reply:
                        node.text_pub.publish(String(data=reply))
                        print(reply, flush=True)
                if outcome.get("sequence"):
                    remember_execution(pipe, query, results)
                spoken = results[-1].get("reply", "")
                if spoken and (speak or speak_info):
                    node.tts_pub.publish(String(data=shorten_for_speech(spoken)))
                request_id = results[-1].get("task_request_id")
                if request_id is not None:
                    wait_for_task(request_id)
            except Exception as exc:
                counts = getattr(exc, "t8_counts", counts)
                node.text_pub.publish(String(data=f"T8 lỗi: {short_error(exc)}"))
                print(f"[lỗi] {short_error(exc)}", file=sys.stderr, flush=True)
            finally:
                from t8_pipeline import Counts
                counts = counts or Counts()
                print(f"[REQ] src={source} gemini={counts.gemini} tavily={counts.tavily} "
                      f"local={counts.local} cached={counts.cached} total={counts.total}",
                      flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--text", help="câu hỏi đầu tiên; sau đó tiếp tục nhận từ terminal")
    ap.add_argument("--voice", action="store_true", help="ASR Sherpa tiếng Việt offline")
    ap.add_argument("--seconds", type=int, default=5, help="thời gian ghi âm mỗi lượt")
    ap.add_argument("--image", help="ảnh camera đã lưu; gửi kèm lượt đầu")
    ap.add_argument("--camera", default="/dev/video2", help="camera dùng với --see")
    ap.add_argument("--see", action="store_true", help="chụp một frame ở lượt đầu")
    ap.add_argument("--speak", action="store_true", help="phát TTS Edge online")
    ap.add_argument("--no-speak-info", action="store_true",
                    help="tắt auto-phát-loa cho tin thuần (mặc định: info luôn phát loa)")
    ap.add_argument("--with-manager", action="store_true", help="tự chạy task manager")
    ap.add_argument("--ros", action="store_true", help="nhận /asr, /image_raw; trả /text_response, /tts_topic")
    ap.add_argument("--once", action="store_true", help="xử lý một lượt rồi thoát")
    ap.add_argument("--no-local", action="store_true", help="tương thích cũ; Gemini hiện luôn lập kế hoạch")
    ap.add_argument("--dry-run-motion", action="store_true",
                    help="chỉ nhận diện/lập kế hoạch; không gửi lệnh tay máy")
    ap.add_argument("--clear-cache", action="store_true", help="tương thích cũ; luồng mới không dùng cache")
    args = ap.parse_args()
    if args.image and args.see:
        ap.error("chọn --image hoặc --see")
    if args.ros and (args.voice or args.text or args.image or args.see or args.once):
        ap.error("--ros dùng riêng; gửi text vào topic /asr")
    if args.once and not (args.text or args.voice):
        ap.error("--once cần --text hoặc --voice")

    manager = None
    if args.with_manager and not manager_running():
        manager = subprocess.Popen([sys.executable, "-u",
            str(DOF_VOICE_DIR / "scripts/voice_task_manager.py")], cwd=DOF_VOICE_DIR)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not manager_running():
            if manager.poll() is not None:
                raise SystemExit(f"Task manager thoát sớm (code {manager.returncode})")
            time.sleep(0.1)
        if not manager_running():
            raise SystemExit("Task manager không lấy được lock trong 3 giây")
        ready_deadline = time.monotonic() + 3
        while time.monotonic() < ready_deadline and not manager_protocol_ready():
            time.sleep(0.1)
        if not manager_protocol_ready():
            raise SystemExit("Task manager không công bố giao thức trạng thái")
        print(f"[task] manager PID {manager.pid} đã sẵn sàng", flush=True)
    elif args.with_manager:
        if not manager_protocol_ready():
            raise SystemExit("Manager bản cũ đang chạy ở terminal khác. "
                             "Ctrl+C terminal đó rồi chạy lại T8.")
        print("[task] dùng manager đã chạy từ terminal khác", flush=True)
    backend = None
    if args.voice:
        enable_user_sherpa()
        sys.path.insert(0, str(DOF_VOICE_DIR / "scripts"))
        from stt_vi import SherpaViBackend
        backend = SherpaViBackend()
    pipe = make_pipeline()
    if args.clear_cache:
        pipe.cache_clear()
        print("[cache] luồng Gemini-first không dùng cache câu trả lời", flush=True)
        if args.once and not (args.text or args.voice):
            return
    first = True
    speak_info = not args.no_speak_info
    executor = make_executor(enable_motion=not args.dry_run_motion)
    preview_executor = make_executor(enable_motion=False) if args.image else None
    saved_terminal = termios.tcgetattr(sys.stdin.fileno()) if sys.stdin.isatty() else None
    try:
        if args.ros:
            ros_loop(pipe, args.speak, speak_info, executor)
            return
        while True:
            if first and args.text:
                query = args.text
            elif backend:
                from stt_vi import record_wav
                wav = record_wav(args.seconds)
                try:
                    query = backend.transcribe(wav)
                finally:
                    os.unlink(wav)
                print(f"[nghe] {query}", flush=True)
                if not query.strip():
                    if args.once:
                        return
                    continue
            else:
                restore_terminal(saved_terminal)
                query = input("T8> ").strip()
                if not query:
                    continue
            if emergency_command(query) == "exit":
                exit_safely(executor)
                return
            counts = None
            source = "?"
            try:
                if emergency_command(query) == "stop":
                    status, request_id = dispatch("stop")
                    print(f"[task] {status}", flush=True)
                    if request_id is not None:
                        wait_for_task(request_id)
                    if args.once and request_id is None:
                        raise SystemExit(1)
                    if args.once:
                        return
                    first = False
                    continue
                see_now = args.see if first else False
                if query.startswith("/see "):
                    query = query[5:]
                    see_now = True
                frame = image_from_file(args.image) if first and args.image else None
                if frame is None and see_now:
                    try:
                        frame = camera_image(args.camera)
                    except Exception as exc:
                        print(f"[cam] không chụp được {args.camera}: {exc}", flush=True)
                        frame = None
                outcome, frame = run_with_vision_retry(
                    pipe, query, frame, args.camera)
                counts = outcome["counts"]
                source = outcome.get("source", "?")
                selected_executor = preview_executor if first and args.image else executor
                results = execute_plan(outcome, selected_executor,
                    lambda: image_from_file(args.image) if first and args.image
                    else camera_image(args.camera),
                    prepare=not args.dry_run_motion and not (first and args.image))
                for result in results:
                    if result.get("reply"):
                        print(result["reply"], flush=True)
                if outcome.get("sequence"):
                    remember_execution(pipe, query, results)
                spoken = results[-1].get("reply", "")
                if spoken and (args.speak or (speak_info and not outcome.get("sequence"))):
                    speak_via_edge(spoken)
                request_id = results[-1].get("task_request_id")
                if request_id is not None:
                    wait_for_task(request_id)
                if args.once and not results[-1].get("ok"):
                    raise SystemExit(1)
            except Exception as exc:
                counts = getattr(exc, "t8_counts", counts)
                print(f"[lỗi] {short_error(exc)}", file=sys.stderr, flush=True)
                if args.once:
                    raise SystemExit(1)
            finally:
                from t8_pipeline import Counts
                counts = counts or Counts()
                print(f"[REQ] src={source} gemini={counts.gemini} tavily={counts.tavily} "
                      f"local={counts.local} cached={counts.cached} total={counts.total}", flush=True)
            first = False
            if args.once:
                return
    except (KeyboardInterrupt, EOFError):
        print("", flush=True)
        exit_safely(executor)
    finally:
        restore_terminal(saved_terminal)
        if manager is not None:
            manager.terminate()
            try:
                manager.wait(timeout=100)
            except subprocess.TimeoutExpired:
                manager.kill()
                manager.wait()


if __name__ == "__main__":
    main()
