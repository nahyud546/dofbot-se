#!/usr/bin/env python3
"""ROS/serial worker for T8 pick, rotate, place and release."""

import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import dofbot_ik


STATE_FILE = Path("/tmp/t8_hold_state.json")
LOCK_FILE = Path("/tmp/t8_motion.lock")
ACTIVE_TASK = Path("/tmp/dofbot_task_active")
SERIAL = "/dev/ttyUSB0"
OPEN_ANGLE = 25
CLOSE_ANGLE = 140
PICK_Z = 0.03 + 0.03 / 2  # T6 table height + cube half height
# Điểm HẠ khi đặt: nhấc thêm PLACE_LIFT để kẹp/vật không chạm sàn khi hạ
# (place) và không cấn mặt cube đích khi xếp chồng (place_target).
# Gắp (PICK_Z) giữ nguyên để kẹp vẫn ngậm chắc vật.
PLACE_LIFT = 0.003
STACK_Z = PICK_Z + 0.03 + PLACE_LIFT  # second cube center above a 3 cm cube
CUBE_H = 0.03  # cube 3 cm chuan; moi tang chong cao them segnay


def stack_z(layers=1):
    """Cao do TCP khi dat len dinh chong dang co `layers` cube (mac dinh 1)."""
    try:
        extra = max(0, int(layers) - 1)
    except (TypeError, ValueError):
        extra = 0
    return STACK_Z + extra * CUBE_H
X_OFFSET = 0.010
# Bù kẹp giữ lệch khi ĐẶT (không đụng gắp: gắp đã chuẩn, X_OFFSET đúng).
# Đo thực tế: đặt lệch về phía trước ~12mm, test B xác nhận vector lệch xoay
# theo J1 (cố định trong khung kẹp). +X = xa robot (trước bàn), +Y = sang trái
# nhìn từ robot. Đo lại số exact rồi sửa GRIP_EX/EY; ở đúng J1 hiệu chuẩn thì
# dấu xoay không ảnh hưởng (delta=0). Nếu đặt ở J1 khác mà lệch ngược hướng
# thì đảo PLACE_GRIP_SIGN thành -1.
PLACE_GRIP_EX = -0.012
PLACE_GRIP_EY = 0.0
PLACE_GRIP_CAL_J1 = 90.0
PLACE_GRIP_SIGN = 1.0
READY_POSE = [90.0, 125.0, 0.0, 0.0, 90.0, OPEN_ANGLE]
HOLDING_OBSERVE_POSE = READY_POSE[:5] + [CLOSE_ANGLE]

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


MAX_ZONE_MOVE_M = 0.15
ZONE_J1_LIMITS = (10.0, 170.0)      # ngoài khoảng này hand-eye/đế chưa kiểm chứng: dùng bảng + cảnh báo


def zone_poses_for_target(cube_id, xy, max_move_m=MAX_ZONE_MOVE_M):
    """Pose tiếp cận/thả/nâng của zone khi ô thật nằm ở xy (base) thay vì vị trí cấu hình.

    Giữ nguyên hình dạng đã hiệu chuẩn của bảng: mỗi pose = điểm thả mới cộng độ lệch (FK) của nó
    so với pose thả cấu hình, giải IK với cùng độ nghiêng kẹp; J5 giữ bảng. Ném ValueError khi xy xa
    vị trí cấu hình quá max_move_m hoặc không có nghiệm IK (người gọi giữ bảng cấu hình).
    """
    release0 = BIN_RELEASE_POSES[cube_id]
    fk0 = dofbot_ik.fk(release0[:5])
    try:
        x, y = float(xy[0]), float(xy[1])
    except (TypeError, ValueError, IndexError):
        raise ValueError("vị trí zone không hợp lệ")
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("vị trí zone không hợp lệ")
    if math.hypot(x - fk0[0], y - fk0[1]) > max_move_m:
        raise ValueError(f"ô đo được cách vị trí cấu hình {math.hypot(x - fk0[0], y - fk0[1]) * 1000:.0f} mm "
                         f"(> {max_move_m * 1000:.0f} mm): nghi khảo sát sai")
    tables = (("approach", BIN_POSES), ("release", BIN_RELEASE_POSES), ("lift", BIN_LIFT_POSES))

    def solve(use_table_tilt):
        out = {}
        for name, table in tables:
            pose = table[cube_id]
            fk = dofbot_ik.fk(pose[:5])
            sol = dofbot_ik.ik(x + fk[0] - fk0[0], y + fk[1] - fk0[1], fk0[2] + fk[2] - fk0[2],
                               j5_deg=pose[4],
                               tilt_deg=dofbot_ik.tilt_of(pose) if use_table_tilt else None)
            out[name] = [float(v) for v in sol[:4]] + [pose[4]]
        return out

    # Ưu tiên giữ độ nghiêng kẹp của bảng; ô dời gần đế thì không còn nghiệm với độ nghiêng đó,
    # khi ấy cả ba pose dùng độ nghiêng IK chọn (gần thẳng đứng nhất) để đường đi nhất quán.
    try:
        poses = solve(True)
    except dofbot_ik.NoSolution:
        try:
            poses = solve(False)
        except dofbot_ik.NoSolution as exc:
            raise ValueError(f"không có nghiệm IK tới ô tại ({x:.3f}, {y:.3f}): {exc}")
    for name, pose in poses.items():
        if not ZONE_J1_LIMITS[0] <= pose[0] <= ZONE_J1_LIMITS[1]:
            raise ValueError(f"{name} cần J1={pose[0]:.0f}° sát giới hạn đế "
                             f"({ZONE_J1_LIMITS[0]:g}-{ZONE_J1_LIMITS[1]:g}°): không tự đưa tay tới đó")
    return poses


RELEASE_FLOOR_Z = 0.050     # TCP không hạ thấp hơn mức này khi thả nhẹ (gắp ở 0,047 là chạm bàn)
# Mức "nhẹ tay" khi thử lại vì cube nảy/văng ra khỏi ô: hạ điểm thả gần mặt ô hơn, di chuyển/mở kẹp/nâng chậm hơn.
GENTLE_LEVELS = {
    0: dict(drop_m=0.000, move_ms=900, open_ms=600, open_wait=0.8, settle=0.0, lift_ms=900),
    1: dict(drop_m=0.012, move_ms=1400, open_ms=900, open_wait=1.2, settle=0.4, lift_ms=1600),
    2: dict(drop_m=0.022, move_ms=1800, open_ms=1300, open_wait=1.6, settle=0.6, lift_ms=2200),
}


def gentle_release_pose(pose, level):
    """Pose thả hạ thấp thêm GENTLE_LEVELS[level].drop_m (không thấp hơn RELEASE_FLOOR_Z, không bao giờ nâng cao).

    Trả (pose mới, ghi chú). Không có nghiệm IK thì giữ pose cũ."""
    drop = GENTLE_LEVELS[level]["drop_m"]
    fk = dofbot_ik.fk(pose[:5])
    z_new = max(float(fk[2]) - drop, min(float(fk[2]), RELEASE_FLOOR_Z))
    if drop <= 0 or z_new >= float(fk[2]) - 1e-4:
        return list(pose), ""
    for tilt in (dofbot_ik.tilt_of(pose), None):
        try:
            sol = dofbot_ik.ik(float(fk[0]), float(fk[1]), z_new, j5_deg=pose[4], tilt_deg=tilt)
            new = [float(v) for v in sol[:4]] + [pose[4]]
            if abs(new[0] - pose[0]) <= 1.5:
                return new, f"thả thấp hơn {1000 * (float(fk[2]) - z_new):.0f} mm (TCP z={z_new:.3f})"
        except dofbot_ik.NoSolution:
            continue
    return list(pose), "không hạ thấp được điểm thả (IK)"


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
    started = time.monotonic()
    for attempt in range(5):
        value = arm.Arm_serial_servo_read(joint)
        if value is not None:
            try:
                angle = float(value)
                if math.isfinite(angle) and 0 <= angle <= (270 if joint == 5 else 180):
                    print(f"[readback] joint={joint} angle={angle:.2f} attempts={attempt + 1} elapsed_s={time.monotonic() - started:.3f}",
                          file=sys.stderr, flush=True)
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


CALIBRATED_SOURCES = ("calibrated_base_pose", "handeye_tag_plane")


def calibrated_source(*grasps):
    """True when every grasp carries a calibrated base_link coordinate."""
    return bool(grasps) and all(
        isinstance(g, dict) and g.get("coordinate_source") in CALIBRATED_SOURCES
        for g in grasps)


def first_move_ms(arm, joints, floor_ms=1000):
    """Slow the first move down when the arm starts far from the approach pose."""
    delta = max(abs(float(t) - a) for t, a in zip(joints, read_joints(arm)))
    return int(min(2800, max(floor_ms, 20 * delta)))


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


def approved_grasp(data, name):
    """Validate a strict ROS3D target lock at the hardware boundary."""
    item = data.get(name)
    if not isinstance(item, dict) or item.get("object_kind") != "cube":
        raise ValueError(f"Thiếu target lock {name}")
    cube_id = item.get("cube_id")
    if (type(cube_id) is not int or cube_id not in BIN_POSES or
            item.get("geometry_model_id") != cube_id or
            item.get("identity_source") not in ("apriltag", "geometry_3d")):
        raise ValueError(f"Identity 3D của {name} không hợp lệ")
    grasp = item.get("grasp")
    if not isinstance(grasp, dict):
        raise ValueError(f"Thiếu grasp trong target lock {name}")
    tcp = grasp.get("tcp_position_base")
    if (not isinstance(tcp, (list, tuple)) or len(tcp) != 3 or
            not all(type(value) in (int, float) and math.isfinite(value)
                    for value in tcp)):
        raise ValueError(f"TCP của {name} không hợp lệ")
    if not (-.30 <= tcp[0] <= -.10 and -.12 <= tcp[1] <= .12 and
            .035 <= tcp[2] <= .145):
        raise ValueError(f"TCP của {name} ngoài workspace đã kiểm chứng")
    yaw = grasp.get("preferred_yaw_rad", 0.0)
    if type(yaw) not in (int, float) or not math.isfinite(yaw) or abs(yaw) > math.pi:
        raise ValueError(f"Yaw của {name} không hợp lệ")
    return item, [float(value) for value in tcp], float(yaw)


def solve_approved_tcp(kin, tcp, yaw_rad, z_add=0.0):
    joints = kin.ik(tcp[0], tcp[1], tcp[2] + z_add)
    yaw_deg = math.degrees(yaw_rad)
    yaw_deg = (yaw_deg + 180) % 360 - 180
    while yaw_deg > 45:
        yaw_deg -= 90
    while yaw_deg < -45:
        yaw_deg += 90
    joints[4] = min(270, max(0, joints[0] - yaw_deg))
    if not valid_ik(joints):
        raise IKNoSolution("Yaw/TCP đã duyệt không có nghiệm khớp an toàn")
    return joints


def approved_motion_plan(plan, source_tcp, target_tcp=None,
                         hover_clearance=0.015, placement_offset=(0.0, 0.0), kin=None):
    """Validate and return the exact joint waypoints checked by ROS preflight."""
    stages = (["approach_pick", "pick", "lift_pick"] if target_tcp is None else
              ["approach_pick", "pick", "lift_pick", "hover_place", "place"])
    if (not isinstance(plan, list) or len(plan) != len(stages) or
            any(not isinstance(item, dict) for item in plan) or
            [item.get("stage") for item in plan] != stages):
        raise ValueError("Preflight plan thiếu stage hoặc sai thứ tự; chưa di chuyển")
    by_stage = {item["stage"]: item for item in plan}
    expected = {
        "approach_pick": [source_tcp[0], source_tcp[1], source_tcp[2] + .020],
        "pick": list(source_tcp),
        "lift_pick": [source_tcp[0], source_tcp[1], source_tcp[2] + .030],
    }
    if target_tcp is not None:
        if (not isinstance(placement_offset, (list, tuple)) or
                len(placement_offset) != 2 or
                not all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= .020
                        for v in placement_offset)):
            raise ValueError("Bù lệch đặt không hợp lệ")
        place_x = target_tcp[0] + float(placement_offset[0])
        place_y = target_tcp[1] + float(placement_offset[1])
        expected.update({
            "hover_place": [place_x, place_y,
                            target_tcp[2] + .030 + hover_clearance],
            "place": [place_x, place_y, target_tcp[2] + .030],
        })
    result = {}
    for name in stages:
        item = by_stage[name]
        tcp = item.get("tcp_position_base")
        joints = item.get("ik_joints_deg")
        if (not isinstance(tcp, list) or len(tcp) != 3 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in tcp) or
                any(abs(float(a) - float(b)) > 1e-6
                    for a, b in zip(tcp, expected[name]))):
            raise ValueError(f"Preflight TCP stage {name} không khớp target đã duyệt")
        if (not isinstance(joints, list) or not valid_ik(joints) or
                not all(type(v) in (int, float) and math.isfinite(v) for v in joints[:5])):
            raise ValueError(f"Preflight thiếu nghiệm khớp hợp lệ tại {name}")
        if kin is not None:
            fk = kin.fk(joints)
            if math.dist(fk, tcp) > .005:
                raise ValueError(f"IK/FK preflight stage {name} lệch quá 5 mm")
        result[name] = [float(v) for v in joints[:5]]
    if target_tcp is not None and abs(result["pick"][0] - result["place"][0]) > 45:
        raise ValueError("Góc J1 nguồn-đích vượt 45° khi giữ vật")
    return result


class Kinematics:
    """Closed-form DOFBOT kinematics; no ROS service or child process."""

    def ik(self, x, y, z):
        try:
            joints = dofbot_ik.ik(x, y, z)
        except dofbot_ik.NoSolution as exc:
            raise IKNoSolution(f"IK không có nghiệm tại "
                               f"({x:.5f}, {y:.5f}, {z:.3f}): {exc}") from exc
        if not valid_ik(joints):
            raise IKNoSolution(f"IK trả về góc không hợp lệ tại "
                               f"({x:.5f}, {y:.5f}, {z:.3f}): {joints}")
        return joints

    def fk(self, joints):
        position = dofbot_ik.fk(joints)
        if not all(math.isfinite(v) for v in position) or position[2] <= 0:
            raise RuntimeError("FK không trả vị trí hợp lệ")
        return position

    def close(self):
        pass


def execute(data, arm, kin=None):
    command = data.get("command")
    state = load_state()
    phase = state.get("phase", "empty")
    # A previous motion can stop after writing the persistent `moving` state.
    # Permit only explicit prepare/recovery through this gate; all pick commands
    # remain blocked until the arm is verified back at READY_POSE.
    if phase == "moving" and command not in ("prepare", "detection_mode"):
        raise RuntimeError("Lệnh trước dừng giữa chừng; kiểm tra tay máy và trạng thái trước khi chạy tiếp")

    if command in ("pick_cube_3d", "stack_cube_3d"):
        token = data.get("approval_token")
        approved_at = data.get("approved_at")
        if (not isinstance(token, str) or len(token) < 16 or
                type(approved_at) not in (int, float) or
                not 0 <= time.time() - float(approved_at) <= 15.0 or
                data.get("preflight_ok") is not True):
            raise RuntimeError("Thiếu target lock/Space approval mới; chưa di chuyển")
        if phase != "empty":
            raise RuntimeError("Tay phải trống trước lệnh cube 3D")
        source, source_tcp, source_yaw = approved_grasp(data, "source")
        target = target_tcp = target_yaw = None
        if command == "stack_cube_3d":
            target, target_tcp, target_yaw = approved_grasp(data, "target")
            coordinate_mode = data.get("coordinate_mode", "metric")
            expected_source = {"fixed_ready_pose": "fixed_ready_pose",
                               "handeye": "handeye_tag_plane"}.get(coordinate_mode,
                                                                   "calibrated_base_pose")
            if any(item["grasp"].get("coordinate_source") != expected_source
                   for item in (source, target)):
                raise RuntimeError("stack_geometry_unavailable: nguồn tọa độ không khớp")
            if coordinate_mode == "metric" and any(
                    not item.get("calibrated") or not item.get("base_pose_valid")
                    for item in (source, target)):
                raise RuntimeError("stack_geometry_unavailable: cần tọa độ base đã hiệu chuẩn")
            if coordinate_mode not in ("metric", "fixed_ready_pose", "handeye"):
                raise RuntimeError("Chế độ tọa độ stack không hợp lệ")
            if (target["cube_id"] == source["cube_id"] or
                    target.get("track_id") == source.get("track_id")):
                raise RuntimeError("Nguồn và đích là cùng một cube")
        # The fixed pixel map is only valid at READY_POSE; calibrated base_link
        # coordinates are valid wherever the arm was when the scene was observed.
        if not calibrated_source(source["grasp"], *([target["grasp"]] if target else [])):
            check_readback(arm, READY_POSE)
        if not gripper_is_open(arm):
            raise RuntimeError("Kẹp chưa mở trước khi gắp")
        hover_clearance = .015
        if target is not None:
            hover_clearance = float(data.get("hover_clearance_m", .015))
            if not 0.0 <= hover_clearance <= .030:
                raise RuntimeError("Khoảng nâng hover phải nằm trong 0-30mm")
        plan_joints = approved_motion_plan(
            data.get("preflight_plan"), source_tcp, target_tcp,
            hover_clearance=hover_clearance,
            placement_offset=data.get("placement_offset_base_xy", (0.0, 0.0)),
            kin=kin)

        save_state({"phase": "moving", "command": command,
                    "approval_token": token, "cube_id": source["cube_id"],
                    "updated": time.time()})
        approach_actual = write_pose_and_wait(
            arm, plan_joints["approach_pick"], OPEN_ANGLE,
            first_move_ms(arm, plan_joints["approach_pick"]))
        pick_actual = write_pose_and_wait(arm, plan_joints["pick"], OPEN_ANGLE, 1000)
        arm.Arm_serial_servo_write(6, CLOSE_ANGLE, 600)
        time.sleep(.8)
        if read_joint(arm, 6) < 60:
            raise RuntimeError("Không xác nhận được kẹp đã đóng")
        lift_actual = write_pose_and_wait(
            arm, plan_joints["lift_pick"], CLOSE_ANGLE, 1000)
        if command == "pick_cube_3d":
            save_state({"phase": "holding", "command": command,
                        "label": f"cube_{source['cube_id']}",
                        "approval_token": token, "joint1": read_joint(arm, 1),
                        "picked_xy": source_tcp[:2], "updated": time.time()})
            return {"ok": True, "holding": True, "status": "executed_unverified",
                    "actual_grasp_verified": False,
                    "reply": (f"Đã tới TCP và xác nhận kẹp đóng với cube ID {source['cube_id']}; "
                              "đang giữ vật. Chưa có cảm biến xác nhận vật trong kẹp."),
                    "detail": {"approach_readback": approach_actual,
                               "pick_readback": pick_actual,
                               "lift_readback": lift_actual}}

        write_pose_and_wait(arm, plan_joints["hover_place"], CLOSE_ANGLE, 1300)
        place_actual = write_pose_and_wait(
            arm, plan_joints["place"], CLOSE_ANGLE, 900)
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, 600)
        time.sleep(.8)
        if not gripper_is_open(arm):
            raise RuntimeError("Không xác nhận được kẹp đã mở khi xếp")
        write_pose_and_wait(arm, plan_joints["hover_place"], OPEN_ANGLE, 900)
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time()})
        return {"ok": True, "holding": False, "status": "executed_unverified",
                "actual_grasp_verified": False,
                "reply": (f"Đã thực hiện gắp ID {source['cube_id']} và đặt lên ID "
                          f"{target['cube_id']}; đã xác nhận khớp/kẹp và về pose quan sát, "
                          "nhưng scene chưa xác minh kết quả xếp."),
                "detail": {"approach_readback": approach_actual,
                           "pick_readback": pick_actual,
                           "lift_readback": lift_actual,
                           "place_readback": place_actual}}

    if command == "detection_mode":
        st = data.get("state", "on")
        if st not in ("on", "off"):
            raise ValueError("detection_mode chỉ nhận on/off")
        if st == "off":
            return {"ok": True, "holding": phase == "holding",
                    "reply": "Đã thoát detection mode."}
        # on: tư thế quan sát an toàn để quét camera.
        # - đang giữ vật: KHÔNG về home (tránh va chạm); giữ nguyên, chỉ xác nhận kẹp đóng.
        # - tay trống: về READY_POSE như prepare (kẹp mở, bao phủ FOV).
        if phase == "holding":
            try:
                if gripper_is_open(arm):
                    raise RuntimeError("Kẹp mở bất thường khi đang giữ vật")
            except RuntimeError:
                raise
            return {"ok": True, "holding": True,
                    "reply": "Đã vào detection mode khi đang giữ vật: giữ nguyên tay, quét chậm ±15°/bước."}
        if phase == "moving":
            if not gripper_is_open(arm):
                raise RuntimeError("Motion dang do; kep van dong. Hay lay cube/mở kẹp an toàn rồi chạy lại để recovery pose-start")
            save_state({"phase": "moving", "command": "detection_mode", "recovery": True,
                        "updated": time.time()})
            go_ready(arm)
            save_state({"phase": "empty", "ready": True, "recovered": True,
                        "updated": time.time(), "detection_mode": True})
            return {"ok": True, "reply": "Đã recovery và vào detection mode: tay về pose quan sát."}
        if phase == "empty":
            try:
                check_readback(arm, READY_POSE)
                if gripper_is_open(arm):
                    save_state({**state, "detection_mode": True, "updated": time.time()})
                    return {"ok": True, "reply": "Đã ở pose quan sát; bật detection mode (quét chậm, chụp liên tục)."}
            except RuntimeError:
                pass
        save_state({"phase": "moving", "command": "detection_mode"})
        go_ready(arm)
        save_state({"phase": "empty", "ready": True, "updated": time.time(),
                    "detection_mode": True})
        return {"ok": True, "reply": "Đã vào detection mode: tay về pose quan sát, kẹp mở, sẵn sàng quét tìm vật."}

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
        if pose not in ("up", "down"):
            raise ValueError("Pose tay không hợp lệ")
        if phase == "holding":
            raise RuntimeError("Tay đang giữ vật; đặt vật trước khi đổi pose")
        before = [read_joint(arm, j) for j in range(1, 7)]
        joints = (2, 3, 4) if pose == "up" else (2, 3)
        target = list(before)
        for joint in joints:
            target[joint - 1] = 90 if pose == "up" else 50
        label = "nâng tay lên" if pose == "up" else "hạ tay xuống"
        if all(abs(before[j - 1] - target[j - 1]) <= 8 for j in joints):
            return {"ok": True, "reply": f"Tay đã ở pose {pose}; không cần di chuyển."}
        save_state({**state, "phase": "moving", "command": "arm_pose", "pose": pose})
        arm.Arm_serial_servo_write6_array(target, 1200)
        time.sleep(1.5)
        check_readback(arm, target, joints=joints, tolerance=8)
        save_state({**state, "updated": time.time()})
        return {"ok": True, "reply": f"Đã {label}; xác nhận các khớp đã tới đích."}

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
        try:
            layers = max(1, int(data.get("stack_layers", 1)))
        except (TypeError, ValueError):
            layers = 1
        place_z = stack_z(layers)
        source_joints = kin.ik(*source)
        target_joints = kin.ik(target[0], target[1], place_z)
        if abs(source_joints[0] - target_joints[0]) > 45:
            raise RuntimeError("Góc xoay giữa hai khối vượt giới hạn 45° khi giữ vật")
        return {"ok": True, "reply": f"Đã kiểm tra IK gắp và xếp chồng tầng {layers + 1}.",
                "source_xy": source[:2], "target_xy": target[:2], "target_z": place_z,
                "stack_layers": layers}
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
        if phase == "empty" and data.get("keep_pose") is True:
            # Calibrated scenes do not need READY_POSE: stay where the arm is
            # and only make sure the gripper is open.
            if not gripper_is_open(arm):
                arm.Arm_serial_servo_write(6, OPEN_ANGLE, 500)
                time.sleep(0.8)
                if not gripper_is_open(arm):
                    raise RuntimeError("Không mở được kẹp tại pose hiện tại")
            return {"ok": True, "kept_pose": True, "joints": read_joints(arm),
                    "reply": "Giữ nguyên pose hiện tại; kẹp mở, sẵn sàng quan sát."}
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
        if command == "sort_cube_3d" and data.get("approval_token") is not None:
            token, approved_at = data.get("approval_token"), data.get("approved_at")
            if (not isinstance(token, str) or len(token) < 16 or
                    type(approved_at) not in (int, float) or
                    not 0 <= time.time() - float(approved_at) <= 15.0 or
                    data.get("preflight_ok") is not True):
                raise RuntimeError("Approval ROS3D đã thiếu/hết hạn; chưa phân loại")
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
            # The bridge's pixel-to-TCP map was tuned with the camera at
            # READY_POSE. Refuse to use that map from another arm pose;
            # calibrated base_link coordinates carry no such restriction.
            if not calibrated_source(data):
                check_readback(arm, READY_POSE)
            if not gripper_is_open(arm):
                raise RuntimeError("Kẹp chưa mở trước khi gắp")
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
        # Pose zone chọn TRƯỚC khi di chuyển (không để lỗi IK xảy ra khi đã cầm cube): ô đo được bằng
        # camera tay -> tính lại từ vị trí ô; không tính được -> bảng cấu hình + cảnh báo trong reply.
        zone_plan = {"approach": BIN_POSES[cube_id], "release": BIN_RELEASE_POSES[cube_id],
                     "lift": BIN_LIFT_POSES[cube_id]}
        zone_note = ""
        zone_target = (data.get("zone_target_xy") or {}).get(str(cube_id))
        if zone_target is not None and command == "sort_cube_3d":
            try:
                zone_plan = zone_poses_for_target(cube_id, zone_target)
                zone_note = (f" Thả theo ô đo được tại ({zone_target[0]:+.3f}, {zone_target[1]:+.3f}) m "
                             f"(J1={zone_plan['release'][0]:.0f}°).")
            except ValueError as exc:
                zone_note = f" Không dùng được vị trí ô đo ({exc}); thả theo góc cấu hình."
        save_state({"phase": "moving", "command": command, "cube_id": cube_id,
                    "target_xy": [x, y], "updated": time.time()})
        arm.Arm_Buzzer_On(1)
        pick_actual = write_pose_and_wait(arm, joints, OPEN_ANGLE,
                                          first_move_ms(arm, joints, 1200))
        arm.Arm_serial_servo_write(6, CLOSE_ANGLE, 600)
        time.sleep(0.8)
        if read_joint(arm, 6) < 60:
            raise RuntimeError("Không xác nhận được kẹp đã đóng")
        arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.3)
        if abs(read_joint(arm, 2) - 120) > 12:
            raise RuntimeError("Không xác nhận được bước nâng tay")
        approach, release, lift = (zone_plan["approach"], zone_plan["release"], zone_plan["lift"])
        level = max(0, min(2, int(data.get("release_gentleness") or 0))) if command == "sort_cube_3d" else 0
        style = GENTLE_LEVELS[level]
        if level:
            release, gentle_note = gentle_release_pose(release, level)
            zone_note += f" Thử lại nhẹ tay mức {level}: {gentle_note or 'chỉ chậm hơn'}."
        write_pose_and_wait(arm, approach, CLOSE_ANGLE, 1200)
        write_pose_and_wait(arm, release, CLOSE_ANGLE, style["move_ms"])
        arm.Arm_serial_servo_write(6, OPEN_ANGLE, style["open_ms"])
        time.sleep(style["open_wait"])
        if not gripper_is_open(arm):
            raise RuntimeError("Không xác nhận được kẹp đã mở tại zone")
        if style["settle"]:
            time.sleep(style["settle"])          # để cube yên rồi mới nâng tay, tránh kẹp kéo cube văng ra
        write_pose_and_wait(arm, lift, OPEN_ANGLE, style["lift_ms"])
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
                          f"(readback {pick_actual[0]:.0f}/{pick_actual[4]:.0f}°)." + zone_note)}
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
    if command == "look":
        # Di chuyển camera tay tới một pose quan sát (tay trống) để khảo sát zone/cảnh; trả góc đo
        # thật và góc trước khi đi để người gọi quay lại đúng pose xuất phát.
        if phase != "empty":
            raise RuntimeError("Chỉ xoay tới pose nhìn khi tay trống")
        servo = data.get("servo")
        if servo is None:
            j1 = float(data.get("j1"))
            servo = [j1] + READY_POSE[1:5]
        if (not isinstance(servo, (list, tuple)) or len(servo) != 5 or
                not all(type(v) in (int, float) and math.isfinite(v) and 0.0 <= v <= 270.0
                        for v in servo)):
            raise RuntimeError("Pose nhìn không hợp lệ")
        if not 5.0 <= float(servo[0]) <= 175.0:
            raise RuntimeError(f"J1={servo[0]:g}° ngoài giới hạn nhìn 5-175°")
        previous = read_joints(arm)[:5]
        move_ms = int(min(2400, max(1000, 22 * abs(float(servo[0]) - previous[0]))))
        actual = write_pose_and_wait(arm, [float(v) for v in servo], OPEN_ANGLE, move_ms)
        return {"ok": True, "servo": [float(v) for v in actual[:5]],
                "from_servo": [float(v) for v in previous],
                "reply": f"Đã xoay camera tới J1={actual[0]:.0f}°."}
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
    if command == "holding_observe":
        if phase != "holding":
            raise RuntimeError("Chưa có vật để chuyển về pose quan sát giữ vật")
        save_state({**state, "phase": "moving", "command": command})
        arm.Arm_serial_servo_write6_array(HOLDING_OBSERVE_POSE, 1800)
        time.sleep(2.1)
        check_readback(arm, HOLDING_OBSERVE_POSE, tolerance=12)
        if gripper_is_open(arm):
            raise RuntimeError("Kẹp mở bất thường khi về pose quan sát giữ vật")
        save_state({**state, "phase": "holding", "joint1": READY_POSE[0],
                    "holding_observe": True, "updated": time.time()})
        return {"ok": True, "holding": True,
                "reply": "Đã giữ cube và về pose quan sát; đang chờ cube đích."}
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
        observing = bool(state.get("holding_observe"))
        if observing:
            if any(abs(current[i] - HOLDING_OBSERVE_POSE[i]) > 12
                   for i in range(5)) or gripper_is_open(arm):
                raise RuntimeError("Tay không còn ở pose quan sát giữ vật")
        else:
            if abs(current[1] - 120) > 12:
                raise RuntimeError("Tay không ở pose nâng đã xác nhận; chưa thể hạ vật")
            if abs(current[0] - float(state["joint1"])) > 12 or any(
                    abs(current[i] - grasp[i]) > 12 for i in (2, 3, 4)):
                raise RuntimeError("Góc tay đã đổi ngoài chuỗi gắp/xoay; chưa thể hạ vật")
        if command == "place_target":
            try:
                layers = max(1, int(data.get("stack_layers", 1)))
            except (TypeError, ValueError):
                layers = 1
            place_z = stack_z(layers)
            x, y, _ = pixel_target(data["target_box"], data["image_size"])
            joints = kin.ik(x, y, place_z)
            # Bù cube nằm lệch trong kẹp (cố định theo khung kẹp, xoay theo J1).
            dj = math.radians((joints[0] - PLACE_GRIP_CAL_J1) * PLACE_GRIP_SIGN)
            ex = PLACE_GRIP_EX * math.cos(dj) - PLACE_GRIP_EY * math.sin(dj)
            ey = PLACE_GRIP_EX * math.sin(dj) + PLACE_GRIP_EY * math.cos(dj)
            if abs(ex) > 1e-9 or abs(ey) > 1e-9:
                joints = kin.ik(x - ex, y - ey, place_z)
            if abs(current[0] - joints[0]) > 45:
                raise RuntimeError("Góc xoay tới mặt đích vượt giới hạn 45°")
            joints[4] = joints[0]
        else:
            # Hạ đặt cao hơn điểm gắp PLACE_LIFT để không chạm sàn; solve lại IK
            # tại XY đã gắp nếu có kin, fallback khớp gắp cũ khi không có kin.
            try:
                if kin is not None and isinstance(state.get("picked_xy"), list):
                    x0, y0 = state["picked_xy"]
                    joints = kin.ik(float(x0), float(y0),
                                    float(state.get("place_z", PICK_Z)) + PLACE_LIFT)
                    joints[0] = current[0]
                    joints[4] = current[4]
                else:
                    raise IKNoSolution("không có kin")
            except (IKNoSolution, RuntimeError, ValueError, TypeError):
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
        if command == "place_target":
            try:
                layers = max(1, int(data.get("stack_layers", 1)))
            except (TypeError, ValueError):
                layers = 1
            placed_reply = (f"Đã xếp lên tầng {layers + 1} "
                            f"(cao {round((layers + 1) * CUBE_H * 100):g} cm, đã bù kẹp lệch)")
        else:
            placed_reply = "Đã đặt vật tại XY hiện tại"
        return {"ok": True, "holding": False,
                "reply": placed_reply + " và về pose chờ; đã xác nhận pose chờ.",
                "stack_layers": layers if command == "place_target" else 1}
    raise ValueError(f"Lệnh chuyển động không hỗ trợ: {command}")


def run_request(data, arm):
    kin = None
    try:
        if data.get("command") in ("pick", "preflight_stack", "preflight_cube_pick", "place", "place_target", "sort_cube_zone", "sort_cube_candidates", "sort_cube_3d", "pick_cube_3d", "stack_cube_3d"):
            kin = Kinematics()
        if data.get("command") == "sequence":
            commands = data.get("commands")
            allowed = {"prepare", "rotate", "arm_pose", "detection_mode"}
            if (not isinstance(commands, list) or not 1 <= len(commands) <= 12 or
                    any(not isinstance(item, dict) or
                        item.get("command") not in allowed for item in commands)):
                raise ValueError("Chuỗi chuyển động không hợp lệ")
            result = []
            for item in commands:
                try:
                    step = execute(item, arm, kin)
                except Exception as exc:
                    step = {"ok": False, "reply": str(exc)}
                result.append(step)
                if not step.get("ok"):
                    break
            return result
        return execute(data, arm, kin)
    finally:
        if kin is not None:
            kin.close()


def main():
    persistent = "--persistent" in sys.argv[1:]
    try:
        with LOCK_FILE.open("a+") as lock:
            lock_started = time.monotonic()
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
            print(f"[serial] lock_wait_s={time.monotonic() - lock_started:.3f}", file=sys.stderr, flush=True)
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
            try:
                if persistent:
                    print("T8_READY:" + json.dumps({"ok": True, "reply": "Đã kết nối tay máy."}, ensure_ascii=False), flush=True)
                    for line in sys.stdin:
                        try:
                            result = run_request(json.loads(line), arm)
                        except Exception as exc:
                            print(f"[serial/motion] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
                            result = {"ok": False, "reply": str(exc)}
                            if isinstance(exc, IKNoSolution):
                                result["code"] = "ik_no_solution"
                        print("T8_RESULT:" + json.dumps(result, ensure_ascii=False), flush=True)
                    return
                result = run_request(json.loads(sys.stdin.read()), arm)
            finally:
                if hasattr(arm, "ser"):
                    arm.ser.close()
                del arm
    except Exception as exc:
        print(f"[serial/motion] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        result = {"ok": False, "reply": str(exc)}
        if isinstance(exc, IKNoSolution):
            result["code"] = "ik_no_solution"
        if persistent:
            print("T8_READY:" + json.dumps(result, ensure_ascii=False), flush=True)
            return
    print("T8_RESULT:" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
