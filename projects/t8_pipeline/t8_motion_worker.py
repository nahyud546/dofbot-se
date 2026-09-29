#!/usr/bin/env python3
"""One-shot ROS/serial worker for T8 pick, rotate, place and release."""

import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


STATE_FILE = Path("/tmp/t8_hold_state.json")
LOCK_FILE = Path("/tmp/t8_motion.lock")
ACTIVE_TASK = Path("/tmp/dofbot_task_active")
SERIAL = "/dev/ttyUSB0"
IK_BINARY = (Path(__file__).resolve().parents[1] / "dofbot_ws" / "install" /
             "dofbot_info" / "lib" / "dofbot_info" / "kinemarics_dofbot")
OPEN_ANGLE = 25
CLOSE_ANGLE = 140
PICK_Z = 0.03 + 0.03 / 2  # T6 table height + cube half height
STACK_Z = PICK_Z + 0.03  # second cube center above a 3 cm cube
X_OFFSET = 0.010
READY_POSE = [90.0, 125.0, 0.0, 0.0, 90.0, OPEN_ANGLE]


def pixel_target(box, image_size):
    """T6 calibration for the 640x480 Sonix camera at its fixed mount."""
    if list(image_size) != [640, 480]:
        raise ValueError("Hiệu chuẩn T6 yêu cầu ảnh camera 640x480")
    if len(box) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in box):
        raise ValueError("YOLO box không hợp lệ")
    x1, y1, x2, y2 = box
    if not (0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480):
        raise ValueError("YOLO box nằm ngoài ảnh")
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    fwd = (480 - cy) * 0.8 / 3000 + 0.13
    lat = (cx - 320) / 4000
    return round(-(fwd + X_OFFSET), 5), round(lat, 5), PICK_Z


def valid_ik(joints):
    if len(joints) < 5 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in joints[:5]):
        return False
    if all(abs(v) < 1e-6 for v in joints[:5]):
        return False
    return (all(0 <= v <= 180 for v in joints[:4]) and 0 <= joints[4] <= 270
            and joints[1] <= 100 and joints[3] <= 120)


def load_state():
    try:
        return json.loads(STATE_FILE.read_text())
    except FileNotFoundError:
        return {"phase": "empty"}
    except (OSError, ValueError):
        return {"phase": "moving", "error": "state unreadable"}


def save_state(state):
    tmp = STATE_FILE.with_name(f"{STATE_FILE.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False))
    os.replace(tmp, STATE_FILE)


def read_joint(arm, joint):
    # Arm_Lib may return None for a delayed serial reply. Retry briefly.
    for _ in range(5):
        value = arm.Arm_serial_servo_read(joint)
        if value is not None:
            try:
                angle = float(value)
                if math.isfinite(angle) and 0 <= angle <= (270 if joint == 5 else 180):
                    return angle
            except (TypeError, ValueError):
                pass
        time.sleep(0.08)
    raise RuntimeError(f"Không đọc được góc khớp {joint} sau 5 lần thử")


def read_joints(arm):
    return [read_joint(arm, j) for j in range(1, 6)]


def gripper_is_open(arm):
    """Confirm manual release from two consistent servo 6 readbacks."""
    first = read_joint(arm, 6)
    time.sleep(0.1)
    second = read_joint(arm, 6)
    if abs(first - second) > 15:
        raise RuntimeError("Góc kẹp đọc không ổn định; chưa cập nhật trạng thái giữ vật")
    return max(first, second) <= 55


def check_readback(arm, expected, joints=(1, 2, 3, 4, 5), tolerance=12):
    for joint in joints:
        actual = read_joint(arm, joint)
        if abs(actual - expected[joint - 1]) > tolerance:
            raise RuntimeError(f"Khớp {joint} chưa tới đích ({actual:.0f}° / {expected[joint-1]:.0f}°)")


def go_ready(arm):
    """Return to the same observation pose used by T6; verify J1-J5 and grip."""
    last_error = None
    for attempt in range(3):
        # T6's verified return path uses write6_array (serial command 0x1e).
        arm.Arm_serial_servo_write6_array(READY_POSE, 1800)
        time.sleep(2.4 if attempt == 0 else 2.0)
        try:
            check_readback(arm, READY_POSE)
            if read_joint(arm, 6) > 55:
                raise RuntimeError("Không xác nhận được kẹp mở ở pose chờ")
            return
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"Tay chưa về pose chờ sau 3 lần gửi lệnh: {last_error}")


class Kinematics:
    def __init__(self):
        import rclpy
        from dofbot_interface.srv import Kinemarics
        self.rclpy, self.service_type = rclpy, Kinemarics
        rclpy.init()
        self.node = rclpy.create_node("t8_motion_kinematics")
        self.client = self.node.create_client(Kinemarics, "dofbot_kinemarics")
        self.child = None
        try:
            if not self.client.wait_for_service(timeout_sec=0.5):
                self.child = subprocess.Popen([str(IK_BINARY)], stdout=subprocess.DEVNULL,
                                              stderr=subprocess.DEVNULL)
                if not self.client.wait_for_service(timeout_sec=8.0):
                    raise RuntimeError("IK service không khởi động trong 8 giây")
        except Exception:
            self.close()
            raise

    def call(self, request):
        future = self.client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("IK/FK service không phản hồi trong 5 giây")
        return future.result()

    def ik(self, x, y, z):
        req = self.service_type.Request()
        req.kin_name = "ik"
        req.tar_x, req.tar_y, req.tar_z = float(x), float(y), float(z)
        req.roll, req.pitch, req.yaw = 0.0, 1.04, -math.pi
        out = self.call(req)
        joints = [out.joint1, out.joint2, out.joint3, out.joint4, out.joint5]
        if not valid_ik(joints):
            raise RuntimeError("IK không có nghiệm trong giới hạn an toàn")
        return joints

    def fk(self, joints):
        req = self.service_type.Request()
        req.kin_name = "fk"
        for i, value in enumerate(joints[:5], 1):
            setattr(req, f"cur_joint{i}", float(value))
        out = self.call(req)
        if not all(math.isfinite(v) for v in (out.x, out.y, out.z)) or out.z <= 0:
            raise RuntimeError("FK không trả vị trí hợp lệ")
        return out.x, out.y, out.z

    def close(self):
        if self.child is not None:
            self.child.terminate()
            try:
                self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait()
        self.node.destroy_node()
        self.rclpy.shutdown()


def execute(data, arm, kin=None):
    command = data.get("command")
    state = load_state()
    phase = state.get("phase", "empty")
    if phase == "moving" and not (command == "prepare" and state.get("command") == "prepare"):
        raise RuntimeError("Lệnh trước dừng giữa chừng; kiểm tra tay máy và trạng thái trước khi chạy tiếp")
    if command == "preflight_stack":
        if phase != "empty":
            raise RuntimeError("Tay phải trống trước khi xếp chồng")
        source = pixel_target(data["source_box"], data["image_size"])
        target = pixel_target(data["target_box"], data["image_size"])
        sc = [(data["source_box"][0] + data["source_box"][2]) / 2,
              (data["source_box"][1] + data["source_box"][3]) / 2]
        tc = [(data["target_box"][0] + data["target_box"][2]) / 2,
              (data["target_box"][1] + data["target_box"][3]) / 2]
        if math.dist(sc, tc) < 55:
            raise RuntimeError("Khối nguồn và mặt đích quá gần hoặc trùng nhau")
        source_joints = kin.ik(*source)
        target_joints = kin.ik(target[0], target[1], STACK_Z)
        if abs(source_joints[0] - target_joints[0]) > 45:
            raise RuntimeError("Góc xoay giữa hai khối vượt giới hạn 45° khi giữ vật")
        return {"ok": True, "reply": "Đã kiểm tra IK gắp và xếp chồng cube 3 cm.",
                "source_xy": source[:2], "target_xy": target[:2], "target_z": STACK_Z}
    if phase == "holding" and command in ("prepare", "place", "release"):
        if gripper_is_open(arm):
            # The user may have opened the gripper and placed the object by
            # hand. Reconcile persisted state only after hardware confirms it.
            save_state({**state, "phase": "moving", "command": "prepare"})
            go_ready(arm)
            save_state({"phase": "empty", "ready": True, "updated": time.time()})
            return {"ok": True, "holding": False,
                    "reply": "Kẹp đã mở từ trước; đã đồng bộ trạng thái và xác nhận pose chờ."}
    if command == "prepare":
        if phase == "holding":
            raise RuntimeError("Kẹp vẫn đóng và T8 đang ghi nhận giữ vật; đặt vật hoặc mở kẹp trước")
        save_state({"phase": "moving", "command": "prepare"})
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "reply": "Tay đã về pose chờ; đã xác nhận các khớp và kẹp mở."}
    if command == "pick":
        if phase == "holding":
            raise RuntimeError("Tay đang giữ vật; hãy đặt hoặc thả trước")
        label = data.get("label")
        source = data.get("source", "yolo")
        if label not in ("xuong_ca", "giay_ve_sinh", "cube", "khoi_do", "khoi_xanh",
                         "khoi_xanh_duong", "khoi_vang") or source not in (
                             "yolo", "hsv", "contour") or (
                             source == "yolo" and float(data.get("conf", 0)) < 0.65) or (
                             source != "yolo" and data.get("geometry_verified") is not True):
            raise RuntimeError("Chưa có nhận diện cube hợp lệ để gắp")
        x, y, z = pixel_target(data.get("box", []), data.get("image_size", []))
        joints = kin.ik(x, y, z)
        # T6 grasp uses J5=J1 for a neutral gripper yaw at the cube.
        joints[4] = joints[0]
        save_state({"phase": "moving", "command": "pick", "label": label})
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, 500)
        time.sleep(0.6)
        arm.Arm_serial_servo_write6(*joints, OPEN_ANGLE, 2000)
        time.sleep(2.2)
        check_readback(arm, joints)
        arm.Arm_serial_servo_write(6, CLOSE_ANGLE, 1500)
        time.sleep(1.7)
        if read_joint(arm, 6) < 60:
            raise RuntimeError("Không xác nhận được kẹp đã đóng")
        arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.2)
        if abs(read_joint(arm, 2) - 120) > 12:
            raise RuntimeError("Không xác nhận được bước nâng tay")
        save_state({"phase": "holding", "label": label, "place_z": z,
                    "joint1": joints[0],
                    "grasp_joints": joints,
                    "picked_xy": [x, y], "updated": time.time()})
        return {"ok": True, "holding": True, "reply":
                f"Đã gắp và nâng {label} ({source}); kẹp đang đóng. Kiểm tra vật trên tay."}
    if command == "rotate":
        joint = int(data.get("joint", 1))
        delta = float(data.get("delta_deg", 0))
        limit = 45 if phase == "holding" else 90
        if joint != 1 or not 0 < abs(delta) <= limit:
            raise RuntimeError(f"Chỉ xoay khớp đế tối đa {limit}° mỗi lệnh")
        try:
            current = read_joint(arm, joint)
        except RuntimeError:
            # Previous pick/rotate verified this angle. Never guess from a
            # default pose when no verified holding state exists.
            if phase != "holding" or not isinstance(state.get("joint1"), (int, float)):
                raise
            current = float(state["joint1"])
        # User-facing left/right is opposite the J1 servo angle on this mount.
        target = current - delta
        if not 0 <= target <= 180:
            raise RuntimeError("Góc đế đích nằm ngoài 0–180°")
        save_state({**state, "phase": "moving", "command": "rotate"})
        arm.Arm_serial_servo_write(1, target, 1200)
        time.sleep(1.4)
        if abs(read_joint(arm, 1) - target) > 12:
            raise RuntimeError("Không xác nhận được góc xoay đế")
        save_state({**state, "joint1": target, "updated": time.time()})
        return {"ok": True, "holding": phase == "holding",
                "reply": f"Đã xoay đế sang {'trái' if delta < 0 else 'phải'} "
                         f"{abs(delta):g}° ({current:.0f}° → {target:.0f}°)" +
                         ("; vẫn giữ vật." if phase == "holding" else ".")}
    if phase != "holding":
        raise RuntimeError("Chưa có vật nào được ghi nhận đang giữ")
    if command == "release":
        save_state({**state, "phase": "moving", "command": "release"})
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, 700)
        time.sleep(0.9)
        if read_joint(arm, 6) > 55:
            raise RuntimeError("Không xác nhận được kẹp đã mở")
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "holding": False,
                "reply": "Đã mở kẹp và về pose chờ; đã xác nhận pose chờ."}
    if command in ("place", "place_target"):
        current = read_joints(arm)
        grasp = state.get("grasp_joints")
        if not isinstance(grasp, list) or not valid_ik(grasp):
            # State written by an older T8 version: solve at the original
            # grasp XY (known reachable), then rotate only J1 to current angle.
            x, y = state["picked_xy"]
            grasp = kin.ik(x, y, float(state["place_z"]))
            grasp[4] = grasp[0]
        if abs(current[1] - 120) > 12:
            raise RuntimeError("Tay không ở pose nâng đã xác nhận; chưa thể hạ vật")
        if abs(current[0] - float(state["joint1"])) > 12 or any(
                abs(current[i] - grasp[i]) > 12 for i in (2, 3, 4)):
            raise RuntimeError("Góc tay đã đổi ngoài chuỗi gắp/xoay; chưa thể hạ vật")
        if command == "place_target":
            x, y, _ = pixel_target(data["target_box"], data["image_size"])
            joints = kin.ik(x, y, STACK_Z)
            if abs(current[0] - joints[0]) > 45:
                raise RuntimeError("Góc xoay tới mặt đích vượt giới hạn 45°")
            joints[4] = joints[0]
        else:
            joints = list(grasp)
            joints[0] = current[0]
            joints[4] = current[4]
        if not valid_ik(joints):
            raise RuntimeError("Pose hạ vật vượt giới hạn an toàn")
        save_state({**state, "phase": "moving", "command": command})
        if command == "place_target":
            arm.Arm_serial_servo_write(1, joints[0], 1200)
            time.sleep(1.4)
            if abs(read_joint(arm, 1) - joints[0]) > 12:
                raise RuntimeError("Không xác nhận được góc đế tại mặt đích")
        arm.Arm_serial_servo_write6(*joints, CLOSE_ANGLE, 2000)
        time.sleep(2.2)
        check_readback(arm, joints)
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, 700)
        time.sleep(0.9)
        if read_joint(arm, 6) > 55:
            raise RuntimeError("Không xác nhận được kẹp đã mở")
        arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.2)
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "holding": False,
                "reply": ("Đã xếp lên mặt cube 3 cm" if command == "place_target"
                          else "Đã đặt vật tại XY hiện tại") +
                         " và về pose chờ; đã xác nhận pose chờ."}
    raise ValueError(f"Lệnh chuyển động không hỗ trợ: {command}")


def main():
    try:
        data = json.loads(sys.stdin.read())
        with LOCK_FILE.open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if ACTIVE_TASK.exists():
                raise RuntimeError("Bài toán robot khác đang chạy; dừng bài đó trước")
            if not Path(SERIAL).exists():
                raise RuntimeError(f"Không thấy cổng tay máy {SERIAL}")
            if subprocess.run(["fuser", "-s", SERIAL], check=False,
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0:
                raise RuntimeError(f"Cổng {SERIAL} đang được tiến trình khác sử dụng")
            from Arm_Lib import Arm_Device
            arm = Arm_Device(SERIAL)
            kin = None
            try:
                if data.get("command") in ("pick", "preflight_stack", "place_target") or (data.get("command") == "place" and
                    not isinstance(load_state().get("grasp_joints"), list)):
                    kin = Kinematics()
                result = execute(data, arm, kin)
            finally:
                if kin is not None:
                    kin.close()
                del arm
    except Exception as exc:
        result = {"ok": False, "reply": str(exc)}
    print("T8_RESULT:" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
