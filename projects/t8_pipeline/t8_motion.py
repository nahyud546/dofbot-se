"""Subprocess bridge for T8 robot motion. The worker owns ROS and serial setup."""

import json
import os
from pathlib import Path
import signal
import subprocess


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "dofbot_ws"
RESULT_PREFIX = "T8_RESULT:"


class MotionBridge:
    def execute(self, command, **params):
        script = ("source ../.venv/bin/activate && "
                  "source ./setup_t6.bash && "
                  "exec python3 ../LargeModel_ws/t8_motion_worker.py")
        env = os.environ.copy()
        env.setdefault("ROS_LOG_DIR", "/tmp/t8_ros_logs")
        proc = subprocess.Popen(["bash", "-c", script], cwd=WS,
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, env=env,
                                start_new_session=True)
        try:
            payload = json.dumps({"command": command, **params})
            interrupted = False
            previous = signal.getsignal(signal.SIGINT)
            def defer_interrupt(_signum, _frame):
                nonlocal interrupted
                interrupted = True
            signal.signal(signal.SIGINT, defer_interrupt)
            try:
                stdout, stderr = proc.communicate(payload, timeout=50)
            finally:
                signal.signal(signal.SIGINT, previous)
            if interrupted:
                raise KeyboardInterrupt
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.communicate()
            if interrupted:
                raise KeyboardInterrupt
            return {"ok": False, "reply": "Hết thời gian chờ chuyển động; kiểm tra tay máy trước khi gửi lệnh tiếp."}
        for line in reversed(stdout.splitlines()):
            if line.startswith(RESULT_PREFIX):
                try:
                    return json.loads(line[len(RESULT_PREFIX):])
                except json.JSONDecodeError:
                    break
        detail = (stderr or stdout).strip().splitlines()
        return {"ok": False, "reply": "Bộ điều khiển chuyển động lỗi: " +
                (detail[-1][:240] if detail else f"mã {proc.returncode}")}
