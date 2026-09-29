#!/usr/bin/env python3
"""Run one DOFBOT task at a time from STT codes 61-65 or the command line."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
WS = ROOT / "dofbot_ws"
TASK_FILE = Path("/tmp/dofbot_task_command")
ACTIVE_FILE = Path("/tmp/dofbot_task_active")
STATUS_FILE = Path("/tmp/dofbot_task_status.json")
SIMPLE_FILE = Path("/tmp/speech_mock_code")
LOCK_FILE = Path("/tmp/dofbot_task_manager.lock")

ROS_BASE = "source /opt/ros/humble/setup.bash; " \
    "source install/dofbot_interface/share/dofbot_interface/local_setup.bash; " \
    "source install/dofbot_info/share/dofbot_info/local_setup.bash; " \
    "export LD_LIBRARY_PATH=$PWD/install/dofbot_info/lib:$PWD/install/dofbot_interface/lib:${LD_LIBRARY_PATH:-}; "
COLOR_BASE = "source /opt/ros/humble/setup.bash; " \
    "source install/dofbot_interface/share/dofbot_interface/local_setup.bash; " \
    "source install/dofbot_info/share/dofbot_info/local_setup.bash; " \
    "export LD_LIBRARY_PATH=$PWD/install/dofbot_info/lib:$PWD/install/dofbot_interface/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}; " \
    "export PYTHONPATH=$PWD/src/dofbot_color_stacking/scripts:$PWD/install/dofbot_sorting_3d/lib/python3.10/site-packages:${PYTHONPATH:-}; "
T6_BASE = "source ../.venv/bin/activate; source ./setup_t6.bash; "

# Every command runs in dofbot_ws. The source scripts set ROS and Python paths.
TASKS = {
    "color": [
        ("IK service", COLOR_BASE + "exec ./install/dofbot_info/lib/dofbot_info/kinemarics_dofbot"),
        ("camera", COLOR_BASE + "exec python3 install/dofbot_sorting_3d/lib/dofbot_sorting_3d/cam_pub --ros-args -p device:=/dev/video2"),
        ("color bin grasp", COLOR_BASE + "exec python3 install/dofbot_sorting_3d/lib/dofbot_sorting_3d/color_bin_grasp"),
        ("color perception", COLOR_BASE + "exec python3 install/dofbot_sorting_3d/lib/dofbot_sorting_3d/color_sorting"),
    ],
    "stack": [
        ("IK service", ROS_BASE + "exec ros2 run dofbot_info kinemarics_dofbot"),
        ("stacking", ROS_BASE + "exec /usr/bin/python3 src/dofbot_color_stacking/scripts/color_stacking_play.py 2"),
    ],
    "face": [
        ("face follow", "exec ../.venv/bin/python ../face_follow_gpu.py"),
    ],
    "trash": [
        ("camera", T6_BASE + "exec ros2 run dofbot_sorting_3d cam_pub --ros-args -p device:=/dev/video2"),
        ("IK service", T6_BASE + "exec ros2 run dofbot_info kinemarics_dofbot"),
        ("grasp", T6_BASE + "exec ros2 run dofbot_sorting_3d grasp"),
        ("YOLO", T6_BASE + "exec ros2 run dofbot_yolov11 yolov11_sortation --show"),
    ],
}
CODE_TASK = {61: "color", 62: "stack", 63: "face", 64: "trash"}
TASK_CAMERA = {"color": "/dev/video2", "stack": "/dev/video2",
               "face": "/dev/video0", "trash": "/dev/video2"}


def write_status(request_id, state, task, detail=""):
    if request_id is None and state != "idle":
        return
    temporary = STATUS_FILE.with_name(f"{STATUS_FILE.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps({"request_id": request_id, "state": state,
                                     "task": task, "detail": detail,
                                     "protocol": 1, "manager_pid": os.getpid()},
                                    ensure_ascii=False))
    os.replace(temporary, STATUS_FILE)


def stop_processes(children, grace=3.0):
    """Signal whole ROS process groups, then reap each child."""
    for _name, proc in reversed(children):
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace
    for _name, proc in reversed(children):
        try:
            proc.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
    for _name, proc in reversed(children):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 2.0
    for _name, proc in reversed(children):
        try:
            proc.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()


class Manager:
    def __init__(self):
        self.name = None
        self.children = []
        self.request_id = None

    def start(self, name, request_id=None):
        if self.name == name:
            print(f"[task] {name} đang chạy", flush=True)
            write_status(request_id, "rejected", name, "Bài toán này đang chạy")
            return
        if self.name is not None:
            detail = f"Đang chạy {self.name}; dừng bài trước khi đổi"
            print(f"[task] {detail}", flush=True)
            write_status(request_id, "rejected", name, detail)
            return
        camera = TASK_CAMERA[name]
        if not Path(camera).exists() or not Path("/dev/ttyUSB0").exists():
            detail = f"Thiếu {camera} hoặc /dev/ttyUSB0"
            print(f"[task] {detail}", flush=True)
            write_status(request_id, "rejected", name, detail)
            return
        if name == "trash" and not (WS / "install_t6_regular/dofbot_yolov11/lib/python3.10/site-packages/dofbot_yolov11/best.onnx").is_file():
            detail = "Thiếu best.onnx trong install_t6_regular"
            print(f"[task] {detail}", flush=True)
            write_status(request_id, "rejected", name, detail)
            return
        ACTIVE_FILE.write_text(name)
        SIMPLE_FILE.unlink(missing_ok=True)
        self.name = name
        self.request_id = request_id
        write_status(request_id, "starting", name)
        time.sleep(1.2)  # let simple_voice_ctrl finish any in-flight servo command
        print(f"[task] bắt đầu {name}", flush=True)
        try:
            for label, command in TASKS[name]:
                env = os.environ.copy()
                if name == "face":
                    env["QT_QPA_PLATFORM"] = "xcb"
                proc = subprocess.Popen(["bash", "-c", "set -e; " + command],
                                        cwd=WS, env=env, start_new_session=True)
                self.children.append((label, proc))
                print(f"[task] {label}: PID {proc.pid}", flush=True)
                time.sleep(0.8)
                if proc.poll() is not None:
                    raise RuntimeError(f"{label} thoát sớm (code {proc.returncode})")
            write_status(request_id, "running", name)
        except Exception as exc:
            print(f"[task] không khởi động được {name}: {exc}", file=sys.stderr, flush=True)
            self.stop(state="failed", detail=str(exc))

    def stop(self, state="finished", detail=""):
        name, request_id = self.name, self.request_id
        if self.name is not None:
            print(f"[task] dừng {self.name}", flush=True)
        if self.name in ("stack", "color"):
            print("[task] đợi chuyển động hiện tại hoàn tất và tay về pose ready", flush=True)
            # Keep IK available while a sorting/stacking worker finishes motion.
            stop_processes(self.children[1:], grace=90.0)
            stop_processes(self.children[:1])
        else:
            stop_processes(self.children)
        self.children.clear()
        self.name = None
        self.request_id = None
        SIMPLE_FILE.unlink(missing_ok=True)
        ACTIVE_FILE.unlink(missing_ok=True)
        if name is not None:
            write_status(request_id, state, name, detail)

    def check(self):
        for label, proc in self.children:
            if proc.poll() is not None:
                print(f"[task] {label} đã thoát (code {proc.returncode}); dừng cả bài", flush=True)
                self.stop(state="finished" if proc.returncode == 0 else "failed",
                          detail=f"{label} thoát (code {proc.returncode})")
                return


def read_code():
    claimed = TASK_FILE.with_name(TASK_FILE.name + f".{os.getpid()}.read")
    try:
        os.replace(TASK_FILE, claimed)
    except FileNotFoundError:
        return None
    try:
        raw = claimed.read_text().strip()
        payload = json.loads(raw) if raw.startswith("{") else {"code": int(raw)}
        code = payload.get("code")
        request_id = payload.get("request_id")
        if type(code) is not int:
            return None
        if request_id is not None and not isinstance(request_id, str):
            return None
        return code, request_id
    except (ValueError, AttributeError):
        return None
    finally:
        claimed.unlink(missing_ok=True)


def handle_termination(_signum, _frame):
    raise KeyboardInterrupt


def run():
    signal.signal(signal.SIGTERM, handle_termination)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-voice", action="store_true",
                        help="start simple_voice_ctrl.py and stt_vi.py too")
    parser.add_argument("--lang", choices=("vi", "zh", "auto"), default="vi",
                        help="STT language with --with-voice (default: vi)")
    parser.add_argument("--task", choices=TASKS,
                        help="run a task directly, without speech recognition")
    parser.add_argument("--dry-run", action="store_true", help="print commands without starting hardware")
    args = parser.parse_args()

    if args.dry_run:
        for name, steps in TASKS.items():
            print(f"{name}:")
            for label, command in steps:
                print(f"  {label}: {command}")
        return

    with LOCK_FILE.open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("voice_task_manager đang chạy ở terminal khác")
        lock.write(str(os.getpid()))
        lock.flush()
        TASK_FILE.unlink(missing_ok=True)
        ACTIVE_FILE.unlink(missing_ok=True)
        STATUS_FILE.unlink(missing_ok=True)
        write_status(None, "idle", "manager")
        manager = Manager()
        voice_children = []
        try:
            if args.with_voice:
                env = os.environ.copy()
                env["PYTHONPATH"] = str(ROOT / "dofbot_voice/scripts") + ":" + env.get("PYTHONPATH", "")
                for label, script in (("voice control", "simple_voice_ctrl.py"),
                                      ("STT", "stt_vi.py")):
                    command = [sys.executable, "-u", str(ROOT / "dofbot_voice/scripts" / script)]
                    if script == "stt_vi.py":
                        command += ["--lang", args.lang]
                    proc = subprocess.Popen(command,
                                            cwd=ROOT, env=env, start_new_session=True)
                    voice_children.append((label, proc))
                    print(f"[voice] {label}: PID {proc.pid}", flush=True)
            if args.task:
                manager.start(args.task)
                if manager.name is None:
                    return 1
            else:
                print("[task] sẵn sàng: 61 màu, 62 xếp chồng, 63 theo mặt, 64 rác, 65 dừng", flush=True)
            while True:
                command = read_code()
                code, request_id = command if command is not None else (None, None)
                if code == 65:
                    manager.stop()
                    write_status(request_id, "finished", "stop")
                elif code in CODE_TASK:
                    manager.start(CODE_TASK[code], request_id=request_id)
                manager.check()
                for label, proc in voice_children:
                    if proc.poll() is not None:
                        raise RuntimeError(f"{label} đã thoát (code {proc.returncode})")
                if args.task and manager.name is None:
                    break
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n[task] Ctrl+C", flush=True)
        finally:
            manager.stop()
            stop_processes(voice_children)
            TASK_FILE.unlink(missing_ok=True)


if __name__ == "__main__":
    sys.exit(run())
