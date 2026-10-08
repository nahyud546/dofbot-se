#!/usr/bin/env python3
"""T8 text/voice assistant: Gemini JSON planning and safe task dispatch."""

import argparse
from collections import deque
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import site
import signal
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
PERCEPTION_PROCESS = None
PERCEPTION_LOG = None


def _ros_camera_publisher_present():
    """Return whether an existing ROS process owns a supported image topic."""
    try:
        import rclpy
        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init()
        node = rclpy.create_node("t8_camera_probe")
        try:
            return any(node.get_publishers_info_by_topic(topic)
                       for topic in ("/cap_vision/image_raw", "/image_raw"))
        finally:
            node.destroy_node()
            if owns_context and rclpy.ok():
                rclpy.shutdown()
    except (ImportError, RuntimeError, AttributeError):
        return False


def _ros_camera_frame_present(timeout=1.0):
    """Require an actual recent Image message, not merely a live publisher."""
    try:
        import rclpy
        from sensor_msgs.msg import Image
        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init()
        node = rclpy.create_node("t8_camera_frame_probe")
        seen = {"value": False}

        def receive(_msg):
            seen["value"] = True

        subscriptions = [node.create_subscription(Image, topic, receive, 1)
                         for topic in ("/cap_vision/image_raw", "/image_raw")]
        deadline = time.monotonic() + max(.1, float(timeout))
        try:
            while not seen["value"] and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            return seen["value"]
        finally:
            subscriptions.clear()
            node.destroy_node()
            if owns_context and rclpy.ok():
                rclpy.shutdown()
    except (ImportError, RuntimeError, AttributeError):
        return False


def running_perception_config():
    """Giá trị config:= của perception đang chạy (đọc từ /proc), hoặc None."""
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath("cmdline").read_bytes().decode(errors="replace").split("\0")
        except OSError:
            continue
        if any(a.endswith("cube_6d_camera.launch.py") for a in args):
            for arg in args:
                if arg.startswith("config:="):
                    return arg[len("config:="):] or None
            return None
    return None


def ensure_cube_perception(device="/dev/video2", timeout=20.0, ros_config=None):
    """Start cube ROS perception automatically when Terminal 1 is absent."""
    global PERCEPTION_PROCESS, PERCEPTION_LOG
    if importlib.util.find_spec("rclpy") is None:
        # Không có rclpy thì mọi phép dò topic đều trả "không có camera" và ta sẽ bật trùng perception.
        return {"ok": False, "started": False,
                "reply": "Chưa source ROS trong terminal này (không import được rclpy): chạy "
                         "`source /opt/ros/humble/setup.bash && source ros/install/setup.bash` rồi thử lại."}
    if _ros_camera_publisher_present():
        if _ros_camera_frame_present(timeout=2.0):
            # Tránh truy vấn tham số chậm (perception chiếm CPU): lấy K từ chính
            # file config mà process đang chạy.
            running = running_perception_config()
            if running:
                os.environ["T8_PERCEPTION_CONFIG"] = running
            else:
                os.environ.pop("T8_PERCEPTION_CONFIG", None)
            return {"ok": True, "started": False,
                    "reply": ("Đang dùng perception ROS đã chạy và có frame camera."
                              + (" --ros-config không áp dụng cho process đã chạy; "
                                 "hãy khởi động lại perception với config:=<file.yaml>."
                                 if ros_config else ""))}
        return {"ok": False, "started": False,
                "reply": ("Có ROS camera publisher nhưng không nhận được frame thật. "
                          f"Kiểm tra {device}, tiến trình đang giữ camera và "
                          "/tmp/t8_cube_perception.log.")}
    if PERCEPTION_PROCESS is not None and PERCEPTION_PROCESS.poll() is not None:
        PERCEPTION_PROCESS = None
    if PERCEPTION_PROCESS is None:
        try:
            index = int(str(device).rsplit("video", 1)[1])
        except (IndexError, ValueError):
            index = 2
        workspace = Path(os.environ.get("ROBOT_ARM_ROS_WS") or ROOT.parent / "ros")
        # DINO/torch mặc định dùng mọi nhân (~1000% CPU) và làm đói các tiến trình
        # ROS khác (truy vấn tham số, mirror khớp). Chừa một nửa số nhân.
        threads = max(2, (os.cpu_count() or 4) // 2)
        script = (f"export OMP_NUM_THREADS={threads} MKL_NUM_THREADS={threads} && "
                  f"export ROS_LOG_DIR=/tmp/t8_cube_ros_logs && "
                  f"mkdir -p /tmp/t8_cube_ros_logs && "
                  f"source /opt/ros/humble/setup.bash && "
                  f"source {workspace}/install/setup.bash && "
                  f"exec ros2 launch cap_vision cube_6d_camera.launch.py "
                  f"device_index:={index} dino_enabled:=true viewer:=false"
                  + (f" config:={shlex.quote(str(ros_config))}" if ros_config else ""))
        if ros_config:
            os.environ["T8_PERCEPTION_CONFIG"] = str(ros_config)
        else:
            os.environ.pop("T8_PERCEPTION_CONFIG", None)
        PERCEPTION_LOG = open("/tmp/t8_cube_perception.log", "a", encoding="utf-8")
        PERCEPTION_PROCESS = subprocess.Popen(
            ["bash", "-lc", script], cwd=workspace,
            stdout=PERCEPTION_LOG, stderr=subprocess.STDOUT,
            start_new_session=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if PERCEPTION_PROCESS.poll() is not None:
            return {"ok": False,
                    "reply": "Perception ROS thoát sớm; xem /tmp/t8_cube_perception.log"}
        if (_ros_camera_publisher_present() and
                _ros_camera_frame_present(timeout=.5)):
            return {"ok": True, "started": True,
                    "reply": "Đã tự khởi động perception ROS và nhận frame camera thật."}
        time.sleep(0.25)
    return {"ok": False,
            "reply": "Hết thời gian chờ perception ROS; xem /tmp/t8_cube_perception.log"}


def stop_owned_perception():
    """Stop only the perception process started by this assistant."""
    global PERCEPTION_PROCESS, PERCEPTION_LOG
    proc, PERCEPTION_PROCESS = PERCEPTION_PROCESS, None
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
    if PERCEPTION_LOG is not None:
        PERCEPTION_LOG.close()
        PERCEPTION_LOG = None


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
                 "sort_cube", "release_hold", "gripper", "light_beep", "pose_start", "arm_pose",
                 "detection_mode", "search_object", "observe_scene", "robot_status", "capabilities"}


def make_executor(enable_motion=False, vision_backend="legacy2d"):
    """Executor an toàn cho skill mới. Trả None nếu import lỗi (chạy legacy)."""
    try:
        from t8_executor import Executor
        from t8_motion import MotionBridge
        from t8_vision import VisionDetector
        if vision_backend == "ros3d":
            from t8_ros_scene import RosSceneBridge
            return Executor(vision=RosSceneBridge(),
                            motion=MotionBridge() if enable_motion else None)
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

    # Simple pose/rotate chains share one serial session and one worker startup.
    motion = executor._motion
    simple = {"pose_start", "rotate_relative", "arm_pose", "detection_mode"}
    if motion is not None and hasattr(motion, "execute_sequence") and all(
            intent in simple for intent, _, _ in steps):
        commands = []
        for intent, entities, _ in steps:
            if intent == "pose_start" and not entities:
                commands.append({"command": "prepare"})
            elif intent == "detection_mode" and set(entities) == {"state"} and entities["state"] in ("on", "off"):
                commands.append({"command": "detection_mode", **entities})
            elif intent == "arm_pose" and set(entities) == {"pose"} and entities["pose"] in ("up", "down"):
                commands.append({"command": "arm_pose", "pose": entities["pose"]})
            elif intent == "rotate_relative" and set(entities) == {"joint", "delta_deg"} and (
                    type(entities["joint"]) is int and entities["joint"] == 1 and
                    type(entities["delta_deg"]) in (int, float) and
                    0 < abs(entities["delta_deg"]) <= 90):
                commands.append({"command": "rotate", **entities})
            else:
                return [{"ok": False, "reply": "Tham số chuyển động không hợp lệ."}]
        result = motion.execute_sequence(commands)
        return result if isinstance(result, list) else [result]

    has_vision = any(s[0] in ("vision_pick_hold", "search_object", "observe_scene") for s in steps)
    need_prepare = any(s[0] in ("vision_pick_hold",) for s in steps)

    if prepare and need_prepare:
        if executor._motion is None:
            return [{"ok": False, "reply": "Bộ điều khiển chuyển động chưa sẵn sàng."}]
        ready = executor._motion.execute("prepare")
        results.append(ready)
        if not ready.get("ok"):
            return results

    frame = None
    if has_vision and not any(s[0] == "search_object" for s in steps):
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
            if intent == "search_object":
                found, _f, _s = active_search_object(
                    entities.get("label", "vat_the"), executor, get_frame)
                result = found
            else:
                need_frame = intent in ("vision_pick_hold", "observe_scene")
                result = executor.execute(intent, entities,
                                          frame=frame if need_frame else None,
                                          confirm_frame=get_frame if need_frame else None)
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
    if outcome.get("need_vision") and not (outcome.get("image_used") or outcome.get("observation_used")):
        return [{"ok": False, "reply": "Chưa có ảnh mới; không chạy kế hoạch vision."}]
    if getattr(executor, "_ros_runner", None) is not None:
        results = []
        print("[plan] " + " → ".join(f"{intent} {entities}" for intent, entities, _ in steps), flush=True)
        for intent, entities, _ in steps:
            result = ({"ok": True, "reply": outcome.get("reply", "")}
                      if intent == "ask_info" else executor.execute(intent, entities))
            results.append(result)
            if not result.get("ok") or (intent == "stack_cubes" and result.get("status") == "executed_unverified"):
                break
        return results
    results = []
    motion = getattr(executor, "_motion", None)
    had_motion = False
    home_done = False
    try:
        holding = json.loads(HOLD_STATE_FILE.read_text()).get("phase") == "holding"
    except (OSError, ValueError):
        holding = False
    index = 0
    while index < len(steps):
        intent, entities, _ = steps[index]
        if intent == "ask_info":
            results.append({"ok": True, "reply": outcome.get("reply", "")})
        elif intent == "stack_cubes":
            results.extend(run_stack_sequence(entities, executor, get_frame, prepare=prepare))
        elif intent in {"open_task", "stop_task"}:
            action = entities["task"] if intent == "open_task" else "stop"
            if getattr(executor, "dry_run", False):
                results.append({"ok": True, "dry_run": True,
                                "reply": f"[dry-run] Đề xuất task {action}; chưa mở task robot."})
                index += 1
                continue
            if intent == "open_task" and motion is not None and hasattr(motion, "close"):
                motion.close()  # task manager needs exclusive access to the serial port
            status, request_id = dispatch(action)
            results.append({"ok": request_id is not None, "reply": status,
                            "task_request_id": request_id, "task_action": action})
            if request_id is None and intent == "open_task" and motion is not None:
                motion.start()
        else:
            simple = {"pose_start", "rotate_relative", "arm_pose", "detection_mode"}
            if intent in simple:
                end = index + 1
                while end < len(steps) and steps[end][0] in simple:
                    end += 1
                group = steps[index:end]
                had_motion = True
                if group[-1][0] not in ("pose_start", "detection_mode") and prepare and not holding:
                    group = [*group, ("pose_start", {}, "none")]
                home_done = group[-1][0] == "pose_start"
                results.extend(run_motion_sequence(group, executor, get_frame,
                                                   prepare=prepare))
                index = end - 1
            elif intent == "search_object":
                had_motion = True
                results.extend(run_search_step(entities.get("label", "vat_the"),
                                               executor, get_frame, prepare=False))
            else:
                had_motion = intent in SKILL_INTENTS or had_motion
                home_done = intent in {"pose_start", "release_hold"}
                results.extend(run_motion_sequence([(intent, entities, "")], executor,
                                                   get_frame, prepare=prepare))
        if results and "holding" in results[-1]:
            holding = bool(results[-1]["holding"])
        if not results[-1].get("ok"):
            break
        index += 1
    if (had_motion and not home_done and not holding and prepare and motion is not None
            and results and results[-1].get("ok")):
        results.append(motion.execute("prepare"))
    elif had_motion and holding and results and results[-1].get("ok"):
        results.append({"ok": True, "holding": True,
                        "reply": "Tay đang giữ vật; sẽ về start sau khi đặt hoặc thả vật."})
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


def _holding_now(executor):
    try:
        return bool(json.loads(HOLD_STATE_FILE.read_text()).get("phase") == "holding")
    except (OSError, ValueError):
        return bool(getattr(getattr(executor, "state", None), "holding", False))


def active_search_object(label, executor, get_frame, exclude=None,
                         max_steps=None, step_deg=None):
    """Inspect two frames in the current legacy camera view, without rotating.

    Old sweep arguments remain for caller compatibility. Changing views needs
    timestamped ROS TF; the fixed pixel map cannot support that operation.
    """
    if executor is None or getattr(executor, "_vision", None) is None:
        return ({"ok": False, "reply": "Bộ nhận diện T8 chưa sẵn sàng."}, None, None)
    try:
        frame, second = get_frame(), get_frame()
        if frame is None or second is None:
            return ({"ok": False, "reply": "Camera chưa có hai ảnh mới để xác nhận."}, None, None)
        candidates = executor._vision.get_cube_candidates(label, frame, second, exclude=exclude)
    except Exception as exc:
        return ({"ok": False, "reply": f"Detector lỗi khi tìm {label}: {exc}."}, None, None)
    if len(candidates) > 1:
        return ({"ok": False, "reply": f"Thấy {len(candidates)} ứng viên {label}; cần xác nhận vật đích."}, None, None)
    if not candidates:
        return ({"ok": False, "code": "dynamic_tf_required",
                 "reply": f"Chưa thấy {label} trong khung hình. Quét đổi góc nhìn cần "
                          "ROS 3D và TF động; luồng pixel legacy chưa tự xoay để gắp."}, None, None)
    found = candidates[0]
    executor._scene.remember(label, found)
    size = list(frame.size) if hasattr(frame, "size") else None
    return ({"ok": True, "found": True,
             "reply": f"Đã thấy {label} tại box {found.get('box')} (nguồn {found.get('source', '?')}).",
             "detail": {"label": label, **found, "sweep_deg": 0.0, "attempt": 0},
             "image_size": size}, frame, size)


def run_search_step(label, executor, get_frame, prepare=False):
    """Legacy search in the current camera view; optional observation pose."""
    results = []
    if executor is None:
        return [{"ok": False, "reply": "Bộ điều khiển T8 chưa sẵn sàng."}]
    motion = getattr(executor, "_motion", None)
    if prepare and motion is not None:
        try:
            dm = motion.execute("detection_mode", state="on")
        except Exception:
            dm = None
        if isinstance(dm, dict):
            results.append(dm)
            if not dm.get("ok"):
                return results
    found, _frame, _size = active_search_object(label, executor, get_frame)
    results.append(found)
    return results


STACK_MEMORY_FILE = Path("/tmp/t8_stack_memory.json")
STACK_TTL_S = 600.0
STACK_MATCH_PX = 70.0
# Dai tu chi dinh chung chung ("them") = dinh chong vua xep (bo nho stack).
STACK_PRONOUNS = {"them", "they", "it", "that", "those", "nó", "chúng",
                  "cái đó", "cai do", "chồng đó", "chong do"}


def _load_stacks():
    try:
        data = json.loads(STACK_MEMORY_FILE.read_text(encoding="utf-8"))
        return data if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def _save_stacks(entries):
    try:
        STACK_MEMORY_FILE.write_text(json.dumps(entries[-8:]), encoding="utf-8")
    except OSError:
        pass


def _fresh_stacks():
    now = time.time()
    return [e for e in _load_stacks()
            if isinstance(e, dict) and now - float(e.get("updated", 0)) <= STACK_TTL_S]


def _stack_center(entry):
    try:
        return [float(entry["center"][0]), float(entry["center"][1])]
    except (TypeError, KeyError, IndexError, ValueError):
        return None


def _find_stack(box, max_dist=STACK_MATCH_PX):
    """Tim chong gan box nhat (tam cach <= max_dist, con tuoi)."""
    try:
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    except (TypeError, IndexError):
        return None
    best, best_d = None, max_dist
    for entry in _fresh_stacks():
        center = _stack_center(entry)
        if center is None:
            continue
        dist = math.hypot(center[0] - cx, center[1] - cy)
        if dist < best_d:
            best, best_d = entry, dist
    return best


def _latest_stack():
    fresh = _fresh_stacks()
    return max(fresh, key=lambda e: float(e.get("updated", 0))) if fresh else None


def _record_stack(target_box, top_label):
    """Sau khi xep thanh cong: tang so tang cua chong tai target_box."""
    try:
        cx, cy = ((target_box[0] + target_box[2]) / 2,
                  (target_box[1] + target_box[3]) / 2)
    except (TypeError, IndexError):
        return None
    entries = _load_stacks()
    now = time.time()
    match = _find_stack(target_box)
    if match is not None:
        for entry in entries:
            if entry is match or (isinstance(entry, dict) and
                                  entry.get("center") == match.get("center") and
                                  entry.get("updated") == match.get("updated")):
                entry["layers"] = max(2, int(entry.get("layers", 1)) + 1)
                entry["top_label"] = top_label
                entry["updated"] = now
                match = entry
                break
    else:
        match = {"center": [cx, cy], "box": list(target_box), "layers": 2,
                 "top_label": top_label, "updated": now}
        entries.append(match)
    _save_stacks(entries)
    return match


def _stack_layers_for(box):
    """So tang hien co tai box (mac dinh 1 = cube le tren ban)."""
    entry = _find_stack(box)
    if entry is None:
        return 1
    try:
        return max(1, int(entry.get("layers", 1)))
    except (TypeError, ValueError):
        return 1


def _pop_stack_top(source_box):
    """Sau khi gap tu dinh chong: giam so tang (toi thieu giu 1)."""
    match = _find_stack(source_box)
    if match is None or int(match.get("layers", 1)) <= 1:
        return None
    entries = _load_stacks()
    for entry in entries:
        if entry is match or (isinstance(entry, dict) and
                              entry.get("center") == match.get("center") and
                              entry.get("updated") == match.get("updated")):
            entry["layers"] = max(1, int(entry.get("layers", 1)) - 1)
            entry["updated"] = time.time()
            match = entry
            break
    _save_stacks(entries)
    return match


def _ros3d_snapshot(timeout_s=5.0):
    """1 snapshot /vision/object_states qua worker system-python (nhu bridge)."""
    try:
        from t8_ros_scene import RosSceneBridge
    except ImportError:
        return {"ok": False, "reason": "thieu RosSceneBridge"}
    try:
        raw = RosSceneBridge()._request(
            {"command": "snapshot", "timeout_s": timeout_s},
            timeout=timeout_s + 8.0)
    except Exception as exc:
        return {"ok": False, "reason": str(exc)}
    return raw if isinstance(raw, dict) else {"ok": False, "reason": "snapshot hong"}


def ros3d_detect(wanted_ids, label, exclude=None, timeout_s=5.0):
    """Tim 1 detection tu perception Terminal 1 khop tap object_id.

    ID la roi rac: chi nhan khi snapshot da confirmed (2 frame nhat quan,
    kieu bridge); khong trung binh ID. Tra None khi thieu/chua commit/mo ho.
    """
    if not wanted_ids:
        return None
    try:
        from cube_identity import select_ros3d_match
    except ImportError:
        return None
    snap = _ros3d_snapshot(timeout_s)
    if not snap.get("ok"):
        return None
    hit = select_ros3d_match(snap.get("objects"), set(wanted_ids), exclude=exclude)
    if hit is None:
        return None
    box = list(hit["bbox_xyxy"])
    det = {"label": label, "box": box, "center": list(hit["center_px"]),
           "conf": float(hit.get("identity_confidence", 0.0) or 0.0),
           "source": "ros3d", "geometry_verified": True,
           "object_id": int(hit.get("object_id", 0)),
           "track_id": str(hit.get("track_id", "")),
           "pose_method": str(hit.get("pose_method", "")),
           "base_position": hit.get("base_position")}
    print(f"[vision] ROS3D: {label} = object {det['object_id']} "
          f"({det['pose_method']}) box {box}.", flush=True)
    return det


def run_stack_sequence(labels, executor, get_frame, prepare=True):
    """Sequential stack: wait source, pick, then wait target and place."""
    results = []
    # Chuẩn hóa nhãn EN (blue/red...) về khoi_* (defense in depth sau validate).
    try:
        from cube_identity import canonical_label
        labels = {"source": canonical_label(labels.get("source", "")),
                  "target": canonical_label(labels.get("target", ""))}
    except Exception:
        pass
    if executor is None or executor._vision is None:
        return [{"ok": False, "reply": "Bộ nhận diện T8 chưa sẵn sàng."}]
    if labels["source"] == labels["target"] == "cube":
        return [{"ok": False, "reply":
                 "Hai cube chưa phân biệt được; hãy nêu màu hoặc họa tiết của một khối."}]
    # Dai tu ("them") = dinh chong vua xep trong bo nho stack.
    mem_target = None
    if labels["target"] in STACK_PRONOUNS:
        mem_target = _latest_stack()
        if mem_target is None or not mem_target.get("box"):
            return [{"ok": False, "reply":
                     "Mình chưa nhớ chồng nào mới xếp — bạn nói rõ đặt lên cube nào giúp mình."}]
    # Chỉ từ chối khi nhãn trùng nhau theo chữ; cùng ID qua mapping (vd xanh
    # dương + cube_1) vẫn cho vision phân biệt bằng vị trí box (loại trừ nguồn
    # khi tìm đích, check khoảng cách ở motion_worker).
    try:
        from cube_identity import color_for_id
    except ImportError:
        try:
            from projects.t8_pipeline.cube_identity import color_for_id  # type: ignore
        except ImportError:
            color_for_id = None
    if prepare:
        if executor._motion is None:
            return [{"ok": False, "reply": "Bộ điều khiển chuyển động chưa sẵn sàng."}]
        ready = executor._motion.execute("prepare")
        results.append(ready)
        if not ready.get("ok"):
            return results

    def detect_label(label, frame, second, exclude=None):
        """Detect 1 label trên cặp frame có sẵn (kèm fallback ID->màu)."""
        candidates = executor._vision.get_cube_candidates(
            label, frame, second, exclude=exclude)
        fallback = None
        if not candidates and color_for_id is not None \
                and str(label).startswith("cube_"):
            try:
                want = int(str(label).rsplit("_", 1)[1])
                fallback = color_for_id(want)
            except (ValueError, IndexError):
                fallback = None
            if fallback is not None:
                candidates = executor._vision.get_cube_candidates(
                    fallback, frame, second, exclude=exclude)
        if len(candidates) == 1:
            det = dict(candidates[0])
            if fallback is not None and det.get("label") == fallback:
                det["color_inferred"] = True
                det["requested"] = label
                print(f"[vision] {label}: dùng màu {fallback} "
                      f"(chưa thấy tag).", flush=True)
            return det
        return None

    def pair_frames():
        """1 cặp frame trong 1 lần spawn ROS; tương thích get_frame cũ."""
        try:
            return camera_image_pair()
        except Exception:
            pass
        frame, second = get_frame(), get_frame()
        return frame, second

    try:
        from cube_identity import object_ids_for_label
        _want_src = object_ids_for_label(labels["source"])
        _want_tgt = object_ids_for_label(labels["target"])
    except Exception:
        _want_src, _want_tgt = set(), set()

    def wait_one(label, phase, timeout_s=30.0, exclude=None):
        print(f"[vision] {phase}: đang chờ {label} (tối đa {timeout_s:g}s); "
              f"Ctrl+C để hủy.", flush=True)
        try:
            from cube_identity import object_ids_for_label as _ids
            wanted = _ids(label)
        except Exception:
            wanted = set()
        try:
            from PIL import Image as _PILFrame
        except ImportError:
            _PILFrame, wanted = None, set()
        camera_errors = 0
        last_ros3d = 0.0
        deadline = time.monotonic() + timeout_s
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Hết {timeout_s:g}s chờ {label}; có thể vật bị che/mất. "
                    f"Tay vẫn giữ an toàn, kiểm tra camera rồi thử lại.")
            if wanted and _PILFrame is not None and time.monotonic() - last_ros3d >= 8.0:
                last_ros3d = time.monotonic()
                try:
                    hit = ros3d_detect(wanted, label, exclude=exclude, timeout_s=4.0)
                except Exception:
                    hit = None
                if hit is not None:
                    return hit, _PILFrame.new("RGB", (640, 480))
            try:
                frame, second = pair_frames()
                if frame is None or second is None:
                    raise RuntimeError("camera chưa có hai frame mới")
                det = detect_label(label, frame, second, exclude=exclude)
                camera_errors = 0
                if det is not None:
                    return det, frame
            except KeyboardInterrupt:
                raise
            except TimeoutError:
                raise
            except Exception as exc:
                camera_errors += 1
                if camera_errors >= 5:
                    raise RuntimeError(f"Camera/detector lỗi khi chờ {label}: {exc}")
            time.sleep(0.1)

    try:
        # Precompute: chốt cả nguồn + đích ngay từ đầu (Isố=0), preflight IK
        # trước khi gắp để tránh treo pha 2 khi cube bị che sau gắp.
        # ROS3D (perception Terminal 1) truoc: ID da commit + bbox pixel san,
        # dung 1 snapshot thay vi trung binh ID. Rot moi chay vong HSV 8s.
        precomputed = None
        if _want_src and _want_tgt:
            try:
                _r_src = ros3d_detect(_want_src, labels["source"], timeout_s=5.0)
                _r_tgt = None
                if _r_src is not None:
                    _r_tgt = ros3d_detect(
                        _want_tgt, labels["target"],
                        exclude={"center": _r_src.get("center"),
                                 "box": _r_src.get("box")}, timeout_s=5.0)
                if (_r_src is not None and _r_tgt is not None
                        and executor._motion is not None):
                    from PIL import Image as _PILRos
                    _pf = executor._motion.execute(
                        "preflight_stack", source_box=_r_src["box"],
                        target_box=_r_tgt["box"], image_size=[640, 480],
                        stack_layers=_stack_layers_for(_r_tgt["box"]))
                    if _pf.get("ok"):
                        precomputed = (_r_src, _r_tgt,
                                       _PILRos.new("RGB", (640, 480)))
                        print(f"[vision] ROS3D đã chốt nguồn {labels['source']} "
                              f"{list(_r_src['box'])} và đích {labels['target']} "
                              f"{list(_r_tgt['box'])}; IK gắp/đặt đạt.", flush=True)
            except Exception as exc:
                print(f"[vision] ROS3D precompute hong ({exc}); dung HSV.", flush=True)
        try:
            # Chong flicker: chi chot khi 2 cap frame LIEN TIEP cho cung box
            # (tam lech <=15px); quad ao 1-frame khong duoc dat coc.
            _prev = None
            _deadline = time.monotonic() + 8.0
            while (time.monotonic() < _deadline and precomputed is None
                   and mem_target is None):
                _f0, _f1 = pair_frames()
                if _f0 is None or _f1 is None:
                    continue
                _src = detect_label(labels["source"], _f0, _f1)
                _tgt = None
                if _src is not None:
                    _ex = {"center": _src.get("center"), "box": _src.get("box")}
                    # Đích khác nguồn: loại trừ box nguồn để khỏi nhầm cùng vật.
                    try:
                        _tgt = detect_label(labels["target"], _f0, _f1, exclude=_ex)
                    except TypeError:
                        _tgt = detect_label(labels["target"], _f0, _f1)
                    if _tgt is None:
                        _tgt = detect_label(labels["target"], _f0, _f1)
                if _src is not None and _tgt is not None:
                    _cur = (_src.get("center") or [0, 0],
                            _tgt.get("center") or [0, 0])
                    _agree = (
                        _prev is not None and all(
                            abs(_cur[i][j] - _prev[i][j]) <= 15
                            for i in (0, 1) for j in (0, 1)))
                    _prev = _cur
                    if not _agree:
                        continue
                if _src is not None and _tgt is not None and executor._motion is not None:
                    _pf = executor._motion.execute(
                        "preflight_stack", source_box=_src["box"], target_box=_tgt["box"],
                        image_size=list(_f0.size),
                        stack_layers=_stack_layers_for(_tgt["box"]))
                    if _pf.get("ok"):
                        precomputed = (_src, _tgt, _f0)
                        print(f"[vision] đã chốt nguồn {labels['source']} {list(_src['box'])} "
                              f"và đích {labels['target']} {list(_tgt['box'])}; "
                              f"IK gắp/đặt đạt.", flush=True)
                    else:
                        print(f"[vision] preflight upfront chưa đạt ({_pf.get('reply','')}); "
                              f"chuyển sang 2 pha sau gắp.", flush=True)
                        break
        except Exception as exc:
            print(f"[vision] precompute bỏ qua ({exc}); dùng 2 pha.", flush=True)
        if precomputed is not None:
            source, target, source_frame = precomputed[0], precomputed[1], precomputed[2]
            target_frame = source_frame
        else:
            source, source_frame = wait_one(labels["source"], "pha 1")
        results.append({"ok": True,
                        "reply": f"Đã thấy {labels['source']}; bắt đầu gắp."})
        if executor._motion is None:
            if mem_target is not None:
                target = {"box": list(mem_target["box"])}
                target_frame = source_frame
            else:
                target, target_frame = wait_one(labels["target"], "pha 2")
            from t8_motion_worker import pixel_target, STACK_Z
            target_xyz = pixel_target(target["box"], list(target_frame.size))
            results.append({"ok": True, "dry_run": True,
                            "reply": (f"[dry-run] Đã thấy nguồn {source['box']} rồi đích "
                                      f"{target['box']}; Z đặt={STACK_Z:.3f} m.")})
            return results

        motion = executor._motion
        picked = motion.execute(
            "pick", label=labels["source"], box=source["box"],
            conf=source.get("conf", 0), source=source.get("source", "contour"),
            geometry_verified=source.get("geometry_verified", False),
            image_size=list(source_frame.size))
        results.append(picked)
        if not picked.get("ok"):
            return results
        # Nguon co the la dinh chong cu: tru tang de lan dat sau dung cao do.
        try:
            _pop_stack_top(source.get("box"))
        except Exception:
            pass
        observing = motion.execute("holding_observe")
        results.append(observing)
        if not observing.get("ok"):
            return results

        if mem_target is not None:
            # Dai tu "them": dat thang len dinh chong da nho, khoi cho detect.
            _mc = [float(v) for v in mem_target["center"]]
            target = {"label": mem_target.get("top_label", labels["target"]),
                      "box": list(mem_target["box"]), "center": _mc,
                      "conf": 1.0, "source": "stack_memory",
                      "geometry_verified": True}
            target_frame = source_frame
            results.append({"ok": True,
                            "reply": f"Đặt lên đỉnh chồng đã nhớ "
                                     f"({int(mem_target.get('layers', 1))} tầng); "
                                     f"bắt đầu đặt chồng."})
        elif precomputed is not None:
            # Đã giữ vật: verify nhanh đích (3s, loại trừ nguồn cũ); nếu bị che
            # thì dùng box đã chốt upfront thay vì treo.
            revalidated = False
            try:
                _ex = {"center": source.get("center"), "box": source.get("box")}
                target, target_frame = wait_one(
                    labels["target"], "pha 2 verify", timeout_s=12.0, exclude=_ex)
                revalidated = True
            except (TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
                if isinstance(exc, KeyboardInterrupt):
                    raise
                target, target_frame = precomputed[1], precomputed[2]
                print(f"[vision] không re-verify được đích sau gắp ({exc}); "
                      f"dùng box đã chốt upfront.", flush=True)
            results.append({"ok": True,
                            "reply": f"Đã thấy {labels['target']}"
                                     f"{'' if revalidated else ' (dùng vị trí đã chốt trước gắp)'}; "
                                     f"bắt đầu đặt chồng."})
        else:
            target, target_frame = wait_one(labels["target"], "pha 2")
            results.append({"ok": True,
                            "reply": f"Đã thấy {labels['target']}; bắt đầu đặt chồng."})
        layers = _stack_layers_for(target["box"])
        placed = motion.execute("place_target", target_box=target["box"],
                                image_size=list(target_frame.size),
                                stack_layers=layers)
        if placed.get("ok"):
            try:
                _record_stack(target["box"], labels["source"])
            except Exception:
                pass
        results.append(placed)
        return results
    except KeyboardInterrupt:
        results.append({"ok": False, "holding": _holding_now(executor),
                        "reply": "Đã hủy chờ nhận diện theo yêu cầu người dùng."})
        return results
    except TimeoutError as exc:
        results.append({"ok": False, "holding": _holding_now(executor),
                        "reply": str(exc) + " Tay vẫn đang giữ vật an toàn."})
        return results
    except Exception as exc:
        results.append({"ok": False, "holding": _holding_now(executor),
                        "reply": str(exc)})
        return results


def run_with_vision_retry(pipe, query, frame, camera_device="/dev/video2", scene_provider=None):
    """Tối ưu 1-call: nếu router báo cần ảnh mà chưa có, chụp trước rồi run 1 lần.

    Trả (outcome, frame). Không bao giờ raise vì lỗi camera — trả outcome cũ
    kèm cảnh báo trong reply nếu không chụp được.
    """
    if frame is None and scene_provider is None:
        try:
            from t8_pipeline import assess_needs
            needs = assess_needs(query, has_image=False)
        except Exception:
            needs = None
        if needs is not None and needs.need_image:
            try:
                frame = camera_image(camera_device)
            except Exception as exc:
                dummy = pipe.run(query, None)
                # pipe.run đã clarify/local thì giữ nguyên; chỉ gắn cảnh báo camera
                # khi outcome thật sự cần vision.
                if dummy.get("need_vision"):
                    dummy["reply"] = (f"Không chụp được camera {camera_device}: {exc}. "
                                      "Chưa thể xác định vật để gắp; kiểm tra camera rồi thử lại.")
                return dummy, None
            outcome = pipe.run(query, frame)
            outcome["image_used"] = True
            return outcome, frame
    outcome = pipe.run(query, frame)
    if frame is None and outcome.get("need_vision") and not outcome.get("image_used"):
        if scene_provider is not None:
            scene = scene_provider()
            if not scene.get("ok"):
                outcome["reply"] = scene.get("reason", "Chưa có quan sát ROS 3D")
                return outcome, None
            outcome2 = pipe.run(query, observation=scene)
            for name in ("gemini", "tavily", "local", "cached"):
                setattr(outcome2["counts"], name,
                        getattr(outcome2["counts"], name) + getattr(outcome["counts"], name))
            return outcome2, None
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
    import re as _re
    text = shorten_for_speech(text)
    # Lọc ký tự điều khiển/ANSI lọt từ terminal (tránh TTS treo vì rác).
    text = _re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", text)
    text = "".join(c for c in text if c == "\n" or ord(c) >= 32)
    if not text.strip():
        return False
    try:
        result = subprocess.run([sys.executable,
                                 str(DOF_VOICE_DIR / "scripts/speak_text.py"),
                                 text], check=False, stdin=subprocess.DEVNULL,
                                 capture_output=True, text=True, timeout=12)
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


def _camera_candidates(device="/dev/video2"):
    """Chỉ thử đúng camera tay đã chọn (cổng USB hay bị đổi: dùng --camera auto/--camera <dev>).

    KHÔNG fallback sang số cứng hay webcam laptop: detector thấy mặt người thay vì cube rồi treo
    pha chờ. Camera bị perception ROS giữ thì đọc qua topic ROS, không mở V4L2 trực tiếp.
    """
    cands = [device]
    try:
        index = int(str(device).rsplit("video", 1)[1])
    except (IndexError, ValueError):
        index = None
    if index is not None and index not in cands:
        cands.append(index)
    return cands


def _grab_ros_pair(timeout_s=25.0):
    """Lấy 1 CẶP frame bàn qua /cap_vision/image_raw trong 1 lần spawn helper.

    wait_one/precompute luôn cần 2 frame (frame + confirm). Spawn 1 helper cho
    cả cặp thay vì 2 lần -> nhanh gấp đôi, đỡ flicker lệch thời gian.
    Trả (PIL, PIL) hoặc (None, None).
    """
    import tempfile
    helper = (
        "import rclpy, sys\n"
        "from rclpy.wait_for_message import wait_for_message\n"
        "from sensor_msgs.msg import Image as RosImage\n"
        "import cv2, numpy as np, time\n"
        "rclpy.init()\n"
        "node = rclpy.create_node('t8_camera_snapshot_helper')\n"
        "def one(path):\n"
        "    ok, msg = wait_for_message(RosImage, node, '/cap_vision/image_raw', time_to_wait=3.0)\n"
        "    assert ok, 'no /cap_vision/image_raw'\n"
        "    assert msg.encoding.lower() in ('bgr8','rgb8'), msg.encoding\n"
        "    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3]\n"
        "    a = a.reshape(msg.height, msg.width, 3)\n"
        "    if msg.encoding.lower() == 'bgr8':\n"
        "        a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)\n"
        "    cv2.imwrite(path, cv2.cvtColor(a, cv2.COLOR_RGB2BGR))\n"
        "try:\n"
        "    one(sys.argv[1])\n"
        "    time.sleep(0.15)\n"
        "    one(sys.argv[2])\n"
        "finally:\n"
        "    node.destroy_node()\n"
        "    rclpy.shutdown()\n"
    )
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as t1, \
         tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as t2:
        out1, out2 = t1.name, t2.name
    helper_path = None
    try:
        project_root = str(ROOT.parent)
        with tempfile.NamedTemporaryFile(suffix=".py", delete=False,
                                         dir="/tmp") as hf:
            hf.write(helper.encode())
            helper_path = hf.name
        cmd = (f"source /opt/ros/humble/setup.bash && "
               f"source {os.environ.get('ROBOT_ARM_ROS_WS') or str(project_root) + '/ros'}/install/setup.bash && "
               f"exec /usr/bin/python3 {helper_path} {out1} {out2}")
        proc = subprocess.run(["bash", "-lc", cmd], cwd=project_root,
                              stdin=subprocess.DEVNULL, capture_output=True,
                              text=True, timeout=timeout_s)
        if proc.returncode != 0:
            return None, None
        from PIL import Image as PILImage
        img1 = PILImage.open(out1)
        img1.load()
        img2 = PILImage.open(out2)
        img2.load()
        if img1.size != (640, 480):
            img1 = img1.resize((640, 480))
        if img2.size != (640, 480):
            img2 = img2.resize((640, 480))
        return img1, img2
    except Exception:
        return None, None
    finally:
        for path in (out1, out2, helper_path):
            try:
                if path:
                    os.unlink(path)
            except OSError:
                pass


def _grab_ros_frame(timeout_s=25.0):
    """Lấy 1 frame bàn (giữ tương thích): dùng cặp rồi trả frame đầu."""
    first, _second = _grab_ros_pair(timeout_s=timeout_s)
    return first


def _grab_v4l2_frame(target):
    """Mở 1 target, ép 640x480, bỏ 2 frame đầu, retry ioctl QBUF."""
    import cv2
    cap = None
    try:
        try:
            cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        except Exception:
            cap = cv2.VideoCapture(target)
        if not cap.isOpened():
            return None, f"Không mở được camera {target}"
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception:
            pass
        frame = None
        last_err = ""
        for _ in range(5):
            try:
                ok, frm = cap.read()
            except Exception as exc:
                last_err = str(exc)
                continue
            if not ok or frm is None:
                last_err = f"Không đọc được frame từ {target}"
                continue
            frame = frm  # giữ frame cuối (bỏ frame cũ trong buffer)
        if frame is None:
            return None, last_err or f"Không đọc được frame từ {target}"
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            try:
                frame = cv2.resize(frame, (640, 480))
            except Exception as exc:
                return None, f"Resize 640x480 lỗi: {exc}"
        return frame, ""
    finally:
        try:
            if cap is not None:
                cap.release()
        except Exception:
            pass


def camera_image(device="/dev/video2"):
    import cv2
    from PIL import Image
    # Terminal 1 normally owns /dev/video2. Consume its ROS stream first;
    # opening the same V4L2 device repeatedly can invalidate queued buffers.
    ros_camera_owned = False
    try:
        import rclpy
        from rclpy.wait_for_message import wait_for_message
        from sensor_msgs.msg import Image as RosImage
        import numpy as np
        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init()
        node = rclpy.create_node("t8_camera_snapshot")
        try:
            for topic in ("/cap_vision/image_raw", "/image_raw"):
                ros_camera_owned = (ros_camera_owned or
                                    bool(node.get_publishers_info_by_topic(topic)))
                ok, message = wait_for_message(
                    RosImage, node, topic, time_to_wait=0.8)
                encoding = message.encoding.lower() if ok else ""
                if encoding not in ("bgr8", "rgb8"):
                    continue
                array = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    message.height, message.step)[:, :message.width * 3]
                array = array.reshape(message.height, message.width, 3)
                if encoding == "bgr8":
                    array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(array.copy())
                if img.size != (640, 480):
                    img = img.resize((640, 480))
                return img
        finally:
            node.destroy_node()
            if owns_context and rclpy.ok():
                rclpy.shutdown()
    except (ImportError, RuntimeError, ValueError, AttributeError):
        pass
    if ros_camera_owned:
        raise RuntimeError("Camera đang do ROS quản lý nhưng chưa nhận được frame mới")
    # .venv không import được rclpy: thử helper có ROS env trước khi chạm V4L2.
    # (video2 đang bị perception giữ; mở V4L2 trực tiếp sẽ EBUSY.)
    try:
        ros_frame = _grab_ros_frame()
    except Exception:
        ros_frame = None
    if ros_frame is not None:
        return ros_frame
    errors = []
    for target in _camera_candidates(device):
        frame, err = _grab_v4l2_frame(target)
        if frame is not None:
            return Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        errors.append(f"{target}: {err}")
    raise RuntimeError("Không lấy được ảnh bàn (ROS /cap_vision/image_raw không có, "
                       "V4L2 bận/mất). Kiểm tra perception ROS, không dùng webcam laptop. "
                       + "; ".join(errors))


def camera_image_pair(device="/dev/video2"):
    """Lấy cặp (frame, confirm) cho detector 2-frame trong 1 lần spawn ROS.

    get_frame đơn lẻ tốn 1 subprocess (~4-6s); cặp tốn ~1 lần. wait_one và
    precompute dùng hàm này; fallback V4L2 trực tiếp nếu ROS không có.
    """
    pair = _grab_ros_pair()
    if pair[0] is not None and pair[1] is not None:
        return pair
    import cv2
    from PIL import Image
    for target in _camera_candidates(device):
        first, err1 = _grab_v4l2_frame(target)
        if first is None:
            continue
        second, _err2 = _grab_v4l2_frame(target)
        if second is None:
            second = first
        return (Image.fromarray(cv2.cvtColor(first, cv2.COLOR_BGR2RGB)),
                Image.fromarray(cv2.cvtColor(second, cv2.COLOR_BGR2RGB)))
    raise RuntimeError("Không lấy được cặp ảnh bàn; kiểm tra perception ROS / camera tay (--camera).")


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


def request_stop(executor):
    """Keep read-only sessions from stopping a separate hardware task manager."""
    if executor is None or getattr(executor, "dry_run", False):
        return "Dry run: không có chuyển động T8 đang chạy; chưa gửi lệnh đến manager robot.", None
    return dispatch("stop")


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
            node.text_pub.publish(String(data="Đã nhận lệnh; đang xử lý."))
            print("[T8] Đã nhận lệnh; đang xử lý.", flush=True)
            if emergency_command(query) == "stop":
                status, _ = request_stop(executor)
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

                if getattr(executor, "_ros_runner", None) is not None:
                    outcome, _ = run_with_vision_retry(pipe, query, None,
                                                      scene_provider=executor._vision.snapshot)
                else:
                    outcome = pipe.run(query, fresh_frame() if see else None)
                    if outcome.get("need_vision") and not outcome.get("image_used"):
                        outcome = pipe.run(query, fresh_frame())
                counts = outcome["counts"]
                source = outcome.get("source", "?")
                if outcome.get("sequence"):
                    node.text_pub.publish(String(data="Đang thực hiện lệnh."))
                    print("[T8] Đang thực hiện lệnh.", flush=True)
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
                    if executor is not None and executor._motion is not None:
                        executor._motion.start()
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
        if executor is not None and executor._motion is not None:
            executor._motion.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--text", help="câu hỏi đầu tiên; sau đó tiếp tục nhận từ terminal")
    ap.add_argument("--voice", action="store_true", help="ASR Sherpa tiếng Việt offline")
    ap.add_argument("--seconds", type=int, default=5, help="thời gian ghi âm mỗi lượt")
    ap.add_argument("--image", help="ảnh camera đã lưu; gửi kèm lượt đầu")
    ap.add_argument("--camera", default="auto",
                    help="camera tay máy: auto = tự nhận bằng chuyển động (nhớ theo tên thiết bị), hoặc /dev/videoN")
    ap.add_argument("--external-camera", default=None,
                    help="camera ngoài (cố định) kiểm tra cube vào đúng ô sau sort; mặc định tự lấy camera USB khác camera tay")
    ap.add_argument("--verify-placement", choices=("auto", "off"), default="auto",
                    help="auto = sau mỗi sort so ảnh camera ngoài trước/sau để xác nhận cube vào đúng ô "
                         "(lệch/rơi thì tự thử lại); off = tắt")
    ap.add_argument("--batch-approval-timeout", type=float, default=180.0,
                    help="lệnh 'tất cả cube': giây chờ Space cho MỖI cube; hết hạn thì bỏ cube đó và làm tiếp "
                         "(không treo). 0 = chờ vô hạn")
    ap.add_argument("--max-retries", type=int, default=2,
                    help="số lần thử lại tối đa khi cube lệch/rơi hoặc xếp tầng không xác minh được")
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
    ap.add_argument("--vision-backend", choices=("ros3d", "legacy2d"), default="ros3d",
                    help="ros3d dùng /vision/object_states; legacy2d dùng ảnh camera cũ")
    ap.add_argument("--enable-motion", action="store_true",
                    help="cho phép chuyển động; ROS3D luôn yêu cầu viewer + Space")
    ap.add_argument("--approval", choices=("viewer",), default="viewer",
                    help="cổng xác nhận target cho ROS3D motion")
    ap.add_argument("--ros-config", type=Path,
                    help="cấu hình ROS 3D đã hiệu chuẩn; chỉ dùng khi T8 tự khởi động perception")
    ap.add_argument("--approval-timeout", type=float, default=0.0,
                    help="giây chờ duyệt; 0 chờ đến khi Space/Esc")
    ap.add_argument("--pick-x-offset-mm", type=float, default=15.0,
                    help="offset X của mapping fixed-pose, trong [-20, 20] mm")
    ap.add_argument("--place-correction-x-mm", type=float, default=12.0,
                    help="bù đặt theo trục kẹp tại READY_POSE, trong [-20, 20] mm")
    ap.add_argument("--place-correction-y-mm", type=float, default=0.0,
                    help="bù đặt ngang theo trục kẹp tại READY_POSE, trong [-20, 20] mm")
    ap.add_argument("--handeye-place-correction-x-mm", type=float, default=0.0,
                    help="bù đặt (theo trục kẹp) riêng cho chế độ hand-eye, [-20, 20] mm; "
                         "+ kéo cube về phía chân robot. Mặc định 0: chưa đo.")
    ap.add_argument("--handeye-place-correction-y-mm", type=float, default=0.0,
                    help="như trên, theo ngang kẹp, [-20, 20] mm")
    ap.add_argument("--zone-check", choices=("always", "off"), default="always",
                    help="trước mỗi lệnh sort (ros3d + hand-eye), camera tay xoay hai phía đo vị trí các "
                         "ô thả rồi dùng góc đo thay vì góc cứng; off = chỉ dùng góc cấu hình")
    ap.add_argument("--clear-cache", action="store_true", help="tương thích cũ; luồng mới không dùng cache")
    args = ap.parse_args()
    if not math.isfinite(args.approval_timeout) or args.approval_timeout < 0:
        ap.error("--approval-timeout phải >= 0 và hữu hạn")
    if not math.isfinite(args.pick_x_offset_mm) or abs(args.pick_x_offset_mm) > 20:
        ap.error("--pick-x-offset-mm phải nằm trong [-20, 20]")
    if (not math.isfinite(args.place_correction_x_mm) or
            abs(args.place_correction_x_mm) > 20 or
            not math.isfinite(args.place_correction_y_mm) or
            abs(args.place_correction_y_mm) > 20):
        ap.error("--place-correction-x/y-mm phải nằm trong [-20, 20]")
    if (not math.isfinite(args.handeye_place_correction_x_mm) or
            abs(args.handeye_place_correction_x_mm) > 20 or
            not math.isfinite(args.handeye_place_correction_y_mm) or
            abs(args.handeye_place_correction_y_mm) > 20):
        ap.error("--handeye-place-correction-x/y-mm phải nằm trong [-20, 20]")
    if str(args.camera) == "auto":
        sys.path.insert(0, str(ROOT))
        from cube_vision.cameras import resolve_arg
        args.camera = resolve_arg(args.camera, fallback="/dev/video0")
    if args.max_retries < 0 or args.max_retries > 5:
        ap.error("--max-retries phải trong [0, 5]")
    if args.enable_motion and args.dry_run_motion:
        ap.error("--enable-motion không dùng cùng --dry-run-motion")
    if args.with_manager and (not args.enable_motion or args.vision_backend != "legacy2d"):
        ap.error("--with-manager cần --enable-motion và --vision-backend legacy2d")
    if args.image and args.vision_backend == "ros3d":
        ap.error("ảnh tĩnh dùng --vision-backend legacy2d --image; ROS 3D cần scene có timestamp/TF")
    if args.ros_config and (args.vision_backend != "ros3d" or not args.ros_config.is_file()):
        ap.error("--ros-config cần backend ros3d và file YAML tồn tại")
    default_ros_config = ROOT.parent / "config" / "robot" / "cube_6d_calibrated.yaml"
    if (not args.ros_config and args.vision_backend == "ros3d" and
            default_ros_config.is_file()):
        args.ros_config = default_ros_config
        print(f"[T8] Dùng cấu hình camera đã hiệu chuẩn: {default_ros_config}", flush=True)
    if args.ros_config:
        args.ros_config = args.ros_config.resolve()
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
    executor = make_executor(enable_motion=args.enable_motion, vision_backend=args.vision_backend)
    if args.vision_backend == "ros3d" and getattr(executor, "_vision", None) is not None:
        executor._vision.approval_timeout = args.approval_timeout
        executor._vision.pick_x_offset_mm = args.pick_x_offset_mm
        executor._vision.zone_check = args.zone_check == "always"
        executor._vision.place_correction_gripper_xy_m = (
            args.place_correction_x_mm / 1000.0,
            args.place_correction_y_mm / 1000.0)
        executor._vision.handeye_place_correction_gripper_xy_m = (
            args.handeye_place_correction_x_mm / 1000.0,
            args.handeye_place_correction_y_mm / 1000.0)
    runner = getattr(executor, "_ros_runner", None)
    if runner is not None:
        runner.max_retries = args.max_retries
        runner.batch_approval_timeout = args.batch_approval_timeout
        if args.enable_motion and args.verify_placement == "auto":
            sys.path.insert(0, str(ROOT))
            from cube_vision.cameras import resolve_external
            from cube_vision.placement_check import ExternalCamera, PlacementVerifier
            external = resolve_external(args.camera, args.external_camera)
            if external:
                locator = None
                from cube_vision.external_camera import ExternalCalibration, make_locator
                ext_cal = ExternalCalibration.load()
                if ext_cal is not None:
                    locator = make_locator(ext_cal, ExternalCamera(
                        external, width=ext_cal.image_size[0], height=ext_cal.image_size[1], fourcc="MJPG"))
                    print("[camera] camera ngoài đã hiệu chuẩn với base: báo thêm khoảng cách cube–tâm ô (mm).",
                          flush=True)
                runner.placement_verifier = PlacementVerifier(ExternalCamera(external), locator=locator)
                print(f"[camera] camera ngoài {external}: xác nhận cube vào ô sau mỗi sort "
                      f"(lệch/rơi thì thử lại tối đa {args.max_retries} lần).", flush=True)
            else:
                print("[camera] không thấy camera ngoài: bỏ qua bước xác nhận ô thả.", flush=True)
    if args.vision_backend == "ros3d":
        from cube_identity import set_ros3d_face_labels
        set_ros3d_face_labels(True)      # "cube hình pin đã qua sử dụng" -> cube_3 ngay ở bước hiểu lệnh
        from t8_ros_tasks import capability_context
        mode = "ros3d_confirmed_motion" if args.enable_motion else "ros3d_dry_run"
        pipe.runtime_capabilities = capability_context(mode)
        if args.enable_motion:
            print("[T8] ROS 3D motion: mọi target phải được duyệt (viewer báo READY) bằng phím Space.", flush=True)
        else:
            print("[T8] ROS 3D dry run: đọc scene/IK/FK; không mở serial hoặc xoay/gắp robot.", flush=True)
    else:
        from t8_ros_tasks import capability_context
        pipe.runtime_capabilities = capability_context("legacy2d_motion")
    if args.vision_backend in ("legacy2d", "ros3d"):
        perception = ensure_cube_perception(args.camera, ros_config=args.ros_config)
        print("[vision] " + perception["reply"], flush=True)
        if not perception.get("ok") and args.enable_motion:
            raise SystemExit("Perception chưa có frame thật; không bật phiên chuyển động.")
    if (executor is not None and executor._motion is not None and
            args.vision_backend != "ros3d"):
        ready = executor._motion.start()
        print("[robot] " + ready.get("reply", "Worker T8 không sẵn sàng."), flush=True)
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
                # Bỏ mã phím mũi tên/history (ESC [ A/B/C/D) lọt từ terminal.
                query = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", query).strip()
                if not query:
                    continue
            print("[T8] Đã nhận lệnh; đang xử lý.", flush=True)
            if emergency_command(query) == "exit":
                exit_safely(executor)
                return
            counts = None
            source = "?"
            try:
                if emergency_command(query) == "stop":
                    status, request_id = request_stop(executor)
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
                if frame is None and see_now and args.vision_backend != "ros3d":
                    try:
                        frame = camera_image(args.camera)
                    except Exception as exc:
                        print(f"[cam] không chụp được {args.camera}: {exc}", flush=True)
                        frame = None
                outcome, frame = run_with_vision_retry(
                    pipe, query, frame, args.camera,
                    scene_provider=executor._vision.snapshot
                    if getattr(executor, "_ros_runner", None) is not None else None)
                counts = outcome["counts"]
                source = outcome.get("source", "?")
                if outcome.get("sequence"):
                    print("[T8] Đang thực hiện lệnh.", flush=True)
                selected_executor = preview_executor if first and args.image else executor
                results = execute_plan(outcome, selected_executor,
                    lambda: image_from_file(args.image) if first and args.image
                    else camera_image(args.camera),
                    prepare=args.enable_motion and not (first and args.image))
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
                    if executor is not None and executor._motion is not None:
                        ready = executor._motion.start()
                        print("[robot] " + ready.get("reply", "Worker T8 không sẵn sàng."), flush=True)
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
        if executor is not None and executor._motion is not None:
            executor._motion.close()
        stop_owned_perception()
        if manager is not None:
            manager.terminate()
            try:
                manager.wait(timeout=100)
            except subprocess.TimeoutExpired:
                manager.kill()
                manager.wait()


if __name__ == "__main__":
    main()
