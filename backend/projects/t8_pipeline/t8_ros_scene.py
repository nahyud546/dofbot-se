"""JSON boundary between the T8 venv and ROS's system Python."""

import json
import os
import subprocess
import selectors
import time
import uuid
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
# Workspace ROS của repo (cap_vision, cap_scene_interfaces); $ROBOT_ARM_ROS_WS để dùng workspace khác.
ROS_WS = Path(os.environ.get("ROBOT_ARM_ROS_WS") or ROOT / "ros")
SETUP = ROS_WS / "install/setup.bash"
WORKER = Path(__file__).with_name("t8_ros_scene_worker.py")


class RosSceneBridge:
    backend = "ros3d"
    approval_timeout = 0.0
    pick_x_offset_mm = 15.0
    place_correction_gripper_xy_m = (0.012, 0.0)
    handeye_place_correction_gripper_xy_m = (0.0, 0.0)

    def _approve_stream(self, request):
        session = uuid.uuid4().hex[:12]
        log_path = f"/tmp/t8_approval_{session}.log"
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        # cv2 imported in the venv can leave its Qt plugin/font paths behind.
        for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
            if "cv2" in env.get(key, ""):
                env.pop(key, None)
        env.setdefault("ROS_LOG_DIR", "/tmp/t8_scene_ros_logs")
        if not (env.get("DISPLAY") or env.get("WAYLAND_DISPLAY")):
            return {"ok": False, "code": "viewer_unavailable",
                    "reason": "Không có DISPLAY/WAYLAND_DISPLAY để mở viewer."}
        script = 'source /opt/ros/humble/setup.bash && source "$1" && exec /usr/bin/python3 -u "$2"'
        process = None
        opened = False
        try:
            with open(log_path, "a", encoding="utf-8") as log, selectors.DefaultSelector() as selector:
                process = subprocess.Popen(["bash", "-c", script, "t8-approval", str(SETUP), str(WORKER)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                    cwd=ROOT, env=env)
                process.stdin.write(json.dumps(request).encode("utf-8"))
                process.stdin.close()
                selector.register(process.stdout, selectors.EVENT_READ)
                started = time.monotonic()
                pending = b""
                print(f"[approval] {request['operation']} IDs {request['object_ids']}; log {log_path}", flush=True)
                while True:
                    if not opened and time.monotonic() - started > 15:
                        return {"ok": False, "code": "viewer_unavailable",
                                "reason": f"Viewer chưa khởi tạo sau 15 giây; xem {log_path}."}
                    for key, _ in selector.select(.1):
                        chunk = os.read(key.fileobj.fileno(), 4096)
                        if not chunk:
                            return {"ok": False, "code": "approval_worker_failed",
                                    "reason": f"Worker đóng luồng kết quả; xem {log_path}."}
                        pending += chunk
                        while b"\n" in pending:
                            raw, pending = pending.split(b"\n", 1)
                            line = raw.decode("utf-8", errors="replace")
                            log.write(line + "\n")
                            log.flush()
                            if line.startswith("T8_EVENT:"):
                                event = json.loads(line[len("T8_EVENT:"):])
                                opened |= event.get("event") == "viewer_opened"
                                print("[approval] " + event.get("reason", event["event"]), flush=True)
                            elif line.startswith("T8_SCENE:"):
                                result = json.loads(line[len("T8_SCENE:"):])
                                result["session_id"] = session
                                result["log_path"] = log_path
                                return result
        except (OSError, ValueError) as exc:
            return {"ok": False, "code": "approval_worker_failed", "reason": f"{exc}; log {log_path}"}
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                if process.stdout:
                    process.stdout.close()

    def _request(self, request, timeout):
        if not SETUP.is_file():
            return {"ok": False, "reason": "Chưa build workspace ROS 3D."}
        script = ('source /opt/ros/humble/setup.bash && '
                  'if [ -f "$3" ]; then source "$3"; fi && '
                  'source "$1" && exec /usr/bin/python3 "$2"')
        env = os.environ.copy()
        # Avoid importing venv NumPy into ROS/cv_bridge's system Python.
        env.pop("PYTHONPATH", None)
        env.setdefault("ROS_LOG_DIR", "/tmp/t8_scene_ros_logs")
        try:
            process = subprocess.run(["bash", "-c", script, "t8-scene", str(SETUP), str(WORKER),
                                      str(ROOT / "workspaces/dofbot_ws/install/setup.bash")],
                                     input=json.dumps(request), text=True, capture_output=True,
                                     timeout=timeout, cwd=ROOT, env=env, check=False)
            for line in reversed(process.stdout.splitlines()):
                if line.startswith("T8_SCENE:"):
                    return json.loads(line[len("T8_SCENE:"):])
            return {"ok": False, "reason": (process.stderr or process.stdout)[-400:] or "ROS worker không trả kết quả."}
        except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
            return {"ok": False, "reason": f"ROS scene worker: {exc}"}

    def snapshot(self, expect=None):
        """expect: [{"object_id": id, "xyz": [x, y, z]}] -> mỗi object cùng ID có expected_px_error."""
        request = {"command": "snapshot", "timeout_s": 4.0}
        if expect:
            request["expect"] = expect
        return self._request(request, timeout=25)

    def zone_survey(self, zones, timeout_s=12.0, expect_j1=None):
        """Đo biên thả các zone từ khung hiện tại; zones: [{"zone_id", "release_xy"}].

        expect_j1: góc J1 vừa xoay tới; chỉ nhận khung khi real_joint_states đã báo gần góc đó."""
        request = {"command": "zone_survey", "zones": zones, "timeout_s": timeout_s}
        if expect_j1 is not None:
            request["expect_j1"] = float(expect_j1)
        return self._request(request, timeout=timeout_s + 20)

    def preflight(self, targets):
        return self._request({"command": "preflight", "targets": targets,
                              "placement_correction_gripper_xy_m":
                                  list(self.place_correction_gripper_xy_m),
                              "handeye_place_correction_gripper_xy_m":
                                  list(self.handeye_place_correction_gripper_xy_m)}, timeout=30)

    def approve(self, object_ids, operation="pick_hold", timeout_s=None, notice="",
                handeye=False):
        """Show the live ROS view and return an immutable, human-approved lock.

        The viewer runs under system Python because ROS Humble's cv_bridge is
        not compatible with the NumPy version used by the assistant venv.
        """
        ids = [int(value) for value in object_ids]
        return self._approve_stream({"command": "approve", "object_ids": ids,
                              "operation": str(operation),
                              "notice": str(notice or "")[:240],
                              "handeye": bool(handeye),
                              "pick_x_offset_mm": self.pick_x_offset_mm,
                              "timeout_s": self.approval_timeout if timeout_s is None else float(timeout_s)})
