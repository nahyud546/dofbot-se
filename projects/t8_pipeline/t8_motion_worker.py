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
IK_BINARY = (Path(__file__).resolve().parents[2] / "workspaces" / "dofbot_ws" / "install" /
             "dofbot_info" / "lib" / "dofbot_info" / "kinemarics_dofbot")
OPEN_ANGLE = 25
CLOSE_ANGLE = 140
PICK_Z = 0.03 + 0.03 / 2  # T6 table height + cube half height
STACK_Z = PICK_Z + 0.03  # second cube center above a 3 cm cube
X_OFFSET = 0.010
READY_POSE = [90.0, 125.0, 0.0, 0.0, 90.0, OPEN_ANGLE]

# Poses calibrated by color_bin_grasp.py, indexed by canonical cube ID.
BIN_POSES = {1: [30, 70, 0, 54, 265], 2: [150, 70, 7, 56, 265],
             3: [50, 70, 7, 58, 265], 4: [135, 70, 7, 54, 265]}
BIN_RELEASE_POSES = {
    1: [30, 65.7098, 0.379519, 56.1977, 265],
    2: [150, 66.6198, 7.04465, 57.3374, 265],
    3: [50, 66.7004, 7.04872, 59.2308, 265],
    4: [135, 66.5425, 7.0367, 55.4436, 265],
}
BIN_LIFT_POSES = {
    1: [30, 89.348, 5.94074, 40.3418, 265],
    2: [150, 85.7708, 12.7032, 47.1349, 265],
    3: [50, 85.3714, 12.4169, 49.9596, 265],
    4: [135, 86.139, 13.0342, 44.3005, 265],
}


class IKNoSolution(RuntimeError):
    """The requested pose has no safe IK solution; no arm motion has started."""


def cube_pick_target(center, quad, image_size, x_offset_mm=0.0, z_offset_mm=0.0):
    """Map the observed top-face center and axis to a calibrated KDL pick."""
    if list(image_size) != [640, 480] or len(center) != 2 or len(quad) != 4:
        raise ValueError("Cần tâm và tứ giác mặt trên trong ảnh 640x480")
    points = [center] + list(quad)
    if any(len(point) != 2 or not all(isinstance(v, (int, float)) and
            math.isfinite(v) for v in point) for point in points):
        raise ValueError("Tọa độ mặt trên không hợp lệ")
    cx, cy = center
    if not (0 <= cx < 640 and 0 <= cy < 480) or any(
            not (0 <= p[0] < 640 and 0 <= p[1] < 480) for p in quad):
        raise ValueError("Mặt trên nằm ngoài ảnh")
    edges = [(quad[(i + 1) % 4][0] - quad[i][0],
              quad[(i + 1) % 4][1] - quad[i][1]) for i in range(4)]
    dx, dy = max(edges, key=lambda edge: edge[0] ** 2 + edge[1] ** 2)
    if dx * dx + dy * dy < 100:
        raise ValueError("Mặt trên quá nhỏ để gắp")
    angle = math.degrees(math.atan2(dy, dx)) % 90
    yaw = -angle if angle <= 45 else 90 - angle
    yaw = max(-40.0, min(40.0, yaw))
    rn2 = ((cx - 320) / 320) ** 2 + ((cy - 240) / 240) ** 2
    scale = 1 + 0.06 * rn2
    ux, uy = 320 + (cx - 320) * scale, 240 + (cy - 240) * scale
    x = -((480 - uy) * (0.8 / 3000) + 0.15)
    y = (ux - 320) / 4000
    if (not isinstance(x_offset_mm, (int, float)) or
            not math.isfinite(x_offset_mm) or abs(x_offset_mm) > 20 or
            not isinstance(z_offset_mm, (int, float)) or
            not math.isfinite(z_offset_mm) or abs(z_offset_mm) > 8):
        raise ValueError("Độ bù gắp phải trong khoảng X ±20 mm, Z ±8 mm")
    x += x_offset_mm / 1000  # KDL X dương: kéo kẹp về phía chân robot.
    if not (-0.30 <= x <= -0.10 and -0.12 <= y <= 0.12):
        raise ValueError("Tâm gắp nằm ngoài vùng làm việc")
    return round(x, 5), round(y, 5), round(0.047 + z_offset_mm / 1000, 5), yaw


def solve_cube_pick(data, arm, kin, x_offset_mm=0.0, z_offset_mm=0.0):
    """Validate a detected cube and solve its pick pose without moving the arm."""
    cube_id = data.get("cube_id")
    if type(cube_id) is not int or cube_id not in BIN_POSES:
        raise ValueError("ID cube phải là 1, 2, 3 hoặc 4")
    if data.get("geometry_valid") is not True or data.get("id_confirmed") is not True:
        raise ValueError("ID và hình học mặt trên chưa được xác nhận")
    x, y, z, yaw = cube_pick_target(data["top_center_px"],
                                    data["top_quad_px"], data["image_size"],
                                    x_offset_mm, z_offset_mm)
    check_readback(arm, READY_POSE)
    if not gripper_is_open(arm):
        raise RuntimeError("Kẹp chưa mở tại pose quan sát")
    joints = None
    for z_try in (z, 0.058 + z_offset_mm / 1000,
                  0.068 + z_offset_mm / 1000):
        try:
            joints = kin.ik(x, y, z_try)
            z = z_try
            break
        except IKNoSolution:
            continue
    if joints is None:
        raise IKNoSolution(
            f"IK không tìm được pose gắp tại x={x:.5f}, y={y:.5f}, "
            f"z=({z:.3f}, {0.058 + z_offset_mm / 1000:.3f}, "
            f"{0.068 + z_offset_mm / 1000:.3f}) m; "
            "kiểm tra URDF/service và hiệu chuẩn camera")
    joints[4] = min(270, max(0, joints[0] - yaw))
    if not valid_ik(joints):
        raise RuntimeError("Góc kẹp của mục tiêu vượt giới hạn")
    return cube_id, (x, y, z), joints


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
    errors = []
    for joint in joints:
        actual = read_joint(arm, joint)
        if abs(actual - expected[joint - 1]) > tolerance:
            errors.append(f"J{joint} {actual:.0f}°/{expected[joint-1]:.0f}°")
    if errors:
        raise RuntimeError("Khớp chưa tới đích: " + ", ".join(errors))


def write_pose_and_wait(arm, joints, gripper_angle, move_ms):
    """Command a 5-axis arm target and verify it has actually settled.

    USB serial arms do not always complete a large base/wrist move within the
    nominal servo duration.  A bounded poll avoids reporting that execution
    failure as an IK failure, and leaves the persistent moving state intact if
    the target is genuinely not reached.
    """
    before = read_joints(arm)
    arm.Arm_serial_servo_write6(*joints, gripper_angle, move_ms)
    max_delta = max(abs(float(target) - actual)
                    for target, actual in zip(joints, before))
    deadline = time.monotonic() + max(move_ms / 1000.0 + 0.35,
                                     min(4.0, 0.9 + max_delta / 28.0))
    last_error = None
    while time.monotonic() < deadline:
        time.sleep(0.18)
        try:
            check_readback(arm, joints)
            actual = read_joints(arm)
            return actual
        except RuntimeError as exc:
            last_error = exc
    raise RuntimeError(f"Motion timeout after {max_delta:.0f}° move: {last_error}")


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
            try:
                self.fk([90, 35, 65, 15, 90])
            except RuntimeError as exc:
                raise RuntimeError(
                    "IK service không đọc được URDF; kiểm tra ROS server cũ và build dofbot_info") from exc
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
            raise IKNoSolution(f"IK trả về góc không hợp lệ tại "
                               f"({x:.5f}, {y:.5f}, {z:.3f}): {joints}")
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
    # A previous motion can stop after writing the persistent `moving` state.
    # Permit only explicit prepare/recovery through this gate; all pick commands
    # remain blocked until the arm is verified back at READY_POSE.
    if phase == "moving" and command != "prepare":
        raise RuntimeError("Lệnh trước dừng giữa chừng; kiểm tra tay máy và trạng thái trước khi chạy tiếp")

    if command == "light":
        dev = data.get("device")
        st = data.get("state")
        if dev == "red": arm.Arm_RGB_set(50, 0, 0)
        elif dev == "green": arm.Arm_RGB_set(0, 50, 0)
        elif dev == "blue": arm.Arm_RGB_set(0, 0, 50)
        elif dev == "yellow": arm.Arm_RGB_set(50, 50, 0)
        elif dev == "beep":
            if st == "on":
                for _ in range(3):
                    arm.Arm_Buzzer_On(); time.sleep(0.2)
                    arm.Arm_Buzzer_Off(); time.sleep(0.2)
        return {"ok": True, "reply": f"Đã chỉnh {dev} {st}."}

    if command == "gripper":
        st = data.get("state")
        angle = OPEN_ANGLE if st == "open" else CLOSE_ANGLE
        arm.Arm_serial_servo_write(6, angle, 500)
        time.sleep(1)
        return {"ok": True, "reply": f"Đã {'mở' if st == 'open' else 'đóng'} kẹp."}

    if command == "arm_pose":
        pose = data.get("pose")
        if pose == "up":
            for j in [2, 3, 4]: arm.Arm_serial_servo_write(j, 90, 1000); time.sleep(0.001)
        elif pose == "down":
            for j in [2, 3]: arm.Arm_serial_servo_write(j, 50, 1000); time.sleep(0.001)
        time.sleep(1.2)
        return {"ok": True, "reply": f"Đã nâng tay lên." if pose == "up" else "Đã hạ tay xuống."}

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
        if phase == "moving":
            # Do not blindly send the robot home if it may still be holding a
            # cube. Once the gripper is confirmed open, READY_POSE is the
            # controlled recovery path and clears the stale motion marker.
            if not gripper_is_open(arm):
                raise RuntimeError("Motion dang do; kep van dong. Hay lay cube/mở kẹp an toàn rồi chạy lại để recovery pose-start")
            save_state({"phase": "moving", "command": "prepare", "recovery": True,
                        "updated": time.time()})
            go_ready(arm)
            save_state({"phase": "empty", "ready": True, "recovered": True,
                        "updated": time.time()})
            return {"ok": True, "reply": "Đã recovery motion fail: kẹp mở, tay về pose quan sát và xác nhận khớp."}
        if phase == "empty":
            try:
                check_readback(arm, READY_POSE)
                if gripper_is_open(arm):
                    return {"ok": True, "reply": "Tay đã ở pose quan sát; không cần di chuyển lại."}
            except RuntimeError:
                pass
        save_state({"phase": "moving", "command": "prepare"})
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "reply": "Tay đã về pose chờ; đã xác nhận các khớp và kẹp mở."}
    if command in ("preflight_cube_pick", "sort_cube_zone", "sort_cube_candidates", "sort_cube_3d"):
        if phase != "empty":
            raise RuntimeError("Tay phải trống trước khi phân loại cube")
        candidates = data.get("candidates") if command == "sort_cube_candidates" else [data]
        if not isinstance(candidates, list) or not candidates:
            raise ValueError("Cần ít nhất một cube đã xác nhận để phân loại")
        skipped = []
        yaw_from_vision = 0.0

        if command == "sort_cube_3d":
            candidate_index = 0
            cube_id = data.get("cube_id")
            if type(cube_id) is not int or cube_id not in BIN_POSES:
                raise ValueError("ID cube phải là 1, 2, 3 hoặc 4")
            # Đóng vòng GraspCandidate -> TCP -> worker: worker KHÔNG tự suy
            # grasp từ object XYZ/quaternion. Bridge phải gửi selected TCP.
            tcp = data.get("tcp_position_base")
            surface_id = data.get("surface_id")
            if (not isinstance(tcp, (list, tuple)) or len(tcp) != 3 or
                    not isinstance(surface_id, str) or not surface_id):
                raise ValueError("Thiếu selected TCP từ grasp planner (tcp_position_base/surface_id); worker không tự suy từ XYZ")
            try:
                x, y, z = float(tcp[0]), float(tcp[1]), float(tcp[2])
            except (TypeError, ValueError):
                raise ValueError("Selected TCP không hợp lệ")
            if not all(math.isfinite(v) for v in (x, y, z)):
                raise ValueError("Selected TCP không hợp lệ")
            try:
                preferred_yaw = float(data.get("preferred_yaw_rad", 0.0))
                gripper_width = float(data.get("gripper_width_m", 0.033))
            except (TypeError, ValueError):
                raise ValueError("preferred_yaw_rad/gripper_width_m không hợp lệ")
            if not math.isfinite(preferred_yaw) or abs(preferred_yaw) > math.pi:
                raise ValueError("preferred_yaw_rad ngoài [-pi, pi]")
            if not (0.02 <= gripper_width <= 0.08):
                raise ValueError("gripper_width_m ngoài [20mm, 80mm]")
            # Yaw kẹp từ grasp planner (5-DOF), chuẩn hoá về [-45,45] cho T8.
            yaw_from_vision = math.degrees(preferred_yaw)
            yaw_from_vision = (yaw_from_vision + 180) % 360 - 180
            while yaw_from_vision > 45:
                yaw_from_vision -= 90
            while yaw_from_vision < -45:
                yaw_from_vision += 90
            quat = None  # worker không dùng quaternion object trực tiếp nữa
            pose_method = str(data.get("pose_method", "unknown"))
            if pose_method in ("conflict", "rgb_geometry"):
                raise ValueError(f"Pose {pose_method} không đủ full-6D để grasp")
            check_readback(arm, READY_POSE)
            if not gripper_is_open(arm):
                raise RuntimeError("Kẹp chưa mở tại pose quan sát")
            joints = None
            for z_try in (z, 0.058, 0.068):
                try:
                    joints = kin.ik(x, y, z_try)
                    z = z_try
                    break
                except IKNoSolution:
                    pass
            if joints is None:
                raise IKNoSolution(f"Không có giải pháp IK cho 3D pose ({x:.3f}, {y:.3f}, {z:.3f})")
            # 5 arm DOF + 1 gripper DOF: J5 từ yaw vision (không coi gripper là DOF6).
            if len(joints) >= 5:
                joints[4] = min(270, max(0, joints[0] - yaw_from_vision))
            if not valid_ik(joints):
                raise RuntimeError("Góc J5 sau yaw vision vượt giới hạn an toàn")
        else:
            for candidate_index, candidate in enumerate(candidates):
                if not isinstance(candidate, dict):
                    raise ValueError("Dữ liệu cube không hợp lệ")
                try:
                    cube_id, (x, y, z), joints = solve_cube_pick(
                        candidate, arm, kin, data.get("pick_x_offset_mm", 0.0),
                        data.get("pick_z_offset_mm", 0.0))
                    break
                except IKNoSolution as exc:
                    skipped.append({"cube_id": candidate.get("cube_id"), "reply": str(exc)})
            else:
                raise IKNoSolution("Không cube nào trong lượt có pose gắp IK hợp lệ: " +
                                   "; ".join(item["reply"] for item in skipped))
        if command == "preflight_cube_pick":
            return {"ok": True, "cube_id": cube_id, "target_xyz": [x, y, z],
                    "joints": joints, "reply": f"IK hợp lệ cho cube ID {cube_id}; chưa di chuyển tay."}
        save_state({"phase": "moving", "command": command, "cube_id": cube_id,
                    "target_xy": [x, y], "updated": time.time()})
        arm.Arm_Buzzer_On(1)
        pick_actual = write_pose_and_wait(arm, joints, OPEN_ANGLE, 1200)
        arm.Arm_serial_servo_write(6, CLOSE_ANGLE, 600)
        time.sleep(0.8)
        if read_joint(arm, 6) < 60:
            raise RuntimeError("Không xác nhận được kẹp đã đóng")
        arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.3)
        if abs(read_joint(arm, 2) - 120) > 12:
            raise RuntimeError("Không xác nhận được bước nâng tay")
        approach = BIN_POSES[cube_id]
        release = BIN_RELEASE_POSES[cube_id]
        lift = BIN_LIFT_POSES[cube_id]
        write_pose_and_wait(arm, approach, CLOSE_ANGLE, 1200)
        write_pose_and_wait(arm, release, CLOSE_ANGLE, 900)
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, 600)
        time.sleep(0.8)
        if not gripper_is_open(arm):
            raise RuntimeError("Không xác nhận được kẹp đã mở tại zone")
        write_pose_and_wait(arm, lift, OPEN_ANGLE, 900)
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "holding": False, "cube_id": cube_id,
                "target_xyz": [x, y, z],
                "surface_id": data.get("surface_id") if command == "sort_cube_3d" else None,
                "yaw_from_vision_deg": yaw_from_vision if command == "sort_cube_3d" else 0.0,
                "pose_method": data.get("pose_method", "unknown") if command == "sort_cube_3d" else "2d",
                "candidate_index": candidate_index, "skipped": skipped,
                "reply": (f"Đã thả cube ID {cube_id} vào zone {cube_id} và về pose quan sát. "
                          f"yaw={yaw_from_vision:.1f}°, "
                          f"J1/J5={joints[0]:.0f}/{joints[4]:.0f}° "
                          f"(readback {pick_actual[0]:.0f}/{pick_actual[4]:.0f}°).")}
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
            # Mirror joint (Terminal 1) giu lock trong luc doc serial.
            # Cho toi 12s thay vi fail ngay voi [Errno 11].
            for _ in range(60):
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    time.sleep(0.2)
            else:
                raise RuntimeError("Kenh dieu khien ban (qua 12s); kiem tra tien trinh giu /tmp/t8_motion.lock")
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
                if data.get("command") in ("pick", "preflight_stack", "preflight_cube_pick", "place_target", "sort_cube_zone", "sort_cube_candidates", "sort_cube_3d") or (data.get("command") == "place" and
                    not isinstance(load_state().get("grasp_joints"), list)):
                    kin = Kinematics()
                result = execute(data, arm, kin)
            finally:
                if kin is not None:
                    kin.close()
                del arm
    except Exception as exc:
        result = {"ok": False, "reply": str(exc)}
        if isinstance(exc, IKNoSolution):
            result["code"] = "ik_no_solution"
    print("T8_RESULT:" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
