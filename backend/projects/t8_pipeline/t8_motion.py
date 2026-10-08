"""Persistent subprocess bridge for T8 robot motion."""

import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parent / "workspaces/dofbot_ws"
RESULT_PREFIX = "T8_RESULT:"
READY_PREFIX = "T8_READY:"


class MotionBridge:
    def __init__(self):
        self.proc = None
        self._stdout_pending = b""
        self._started_at = None

    def _read_message(self, prefix, timeout):
        deadline = time.monotonic() + timeout
        while True:
            while b"\n" in self._stdout_pending:
                raw, self._stdout_pending = self._stdout_pending.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace")
                if line.startswith(prefix):
                    return json.loads(line[len(prefix):])
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([self.proc.stdout], [], [], max(0, remaining))
            if not readable:
                break
            chunk = os.read(self.proc.stdout.fileno(), 4096)
            if not chunk:
                break
            self._stdout_pending += chunk
        return {"ok": False, "reply": "Worker T8 không phản hồi đúng hạn hoặc đã thoát."}

    def start(self):
        if self.proc is not None and self.proc.poll() is None:
            return {"ok": True, "reply": "Worker T8 đang sẵn sàng."}
        self.close()
        self._stdout_pending = b""
        project_root = ROOT.parent
        script = (f"source {project_root}/.venv/bin/activate && "
                  f"source {project_root}/scripts/run/setup_t6.bash && "
                  f"exec python3 -u {project_root}/projects/t8_pipeline/t8_motion_worker.py --persistent")
        env = os.environ.copy()
        env.setdefault("ROS_LOG_DIR", "/tmp/t8_ros_logs")
        with open("/tmp/t8_motion_worker.log", "a", encoding="utf-8") as log:
            self._started_at = time.monotonic()
            self.proc = subprocess.Popen(["bash", "-c", script], cwd=WS,
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=log, text=True, bufsize=1,
                                         env=env, start_new_session=True)
        try:
            ready = self._read_message(READY_PREFIX, 15)
        except (OSError, ValueError) as exc:
            ready = {"ok": False, "reply": f"Worker T8 khởi động lỗi: {exc}"}
        if not ready.get("ok"):
            self.close()
        return ready

    def execute(self, command, **params):
        return self._request({"command": command, **params})

    def execute_sequence(self, commands):
        return self._request({"command": "sequence", "commands": commands})

    def _request(self, payload):
        ready = self.start()
        if not ready.get("ok"):
            return ready
        interrupted = False
        previous = signal.getsignal(signal.SIGINT)

        def defer_interrupt(_signum, _frame):
            nonlocal interrupted
            interrupted = True

        signal.signal(signal.SIGINT, defer_interrupt)
        try:
            self.proc.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self.proc.stdin.flush()
            result = self._read_message(RESULT_PREFIX, 90)
        except (BrokenPipeError, OSError, ValueError) as exc:
            result = {"ok": False, "reply": f"Mất kết nối worker T8: {exc}"}
        finally:
            signal.signal(signal.SIGINT, previous)
        failed = result if isinstance(result, dict) else None
        if failed is not None and not failed.get("ok") and "không phản hồi đúng hạn" in failed.get("reply", ""):
            self.close()
        if interrupted:
            raise KeyboardInterrupt
        return result

    def close(self):
        proc, self.proc = self.proc, None
        if proc is None:
            return
        started = getattr(self, "_started_at", None)
        if started is not None:
            try:
                with open("/tmp/t8_motion_worker.log", "a", encoding="utf-8") as log:
                    log.write(f"[serial] session_duration_s={time.monotonic() - started:.3f}\n")
            except OSError:
                pass
            self._started_at = None
        if proc.poll() is None:
            try:
                proc.stdin.close()
                proc.wait(timeout=3)
            except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        proc.stdout.close()
