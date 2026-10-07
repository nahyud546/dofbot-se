"""Step 1 of auto calibration: generate + validate calibration poses (PLAN-ONLY).

No serial, no motion. Needs MoveIt (/plan_kinematic_path + /compute_fk) running.

Pipeline tuong lai: generate_calib_poses -> hardware/auto_calibrate.py (execute +
capture) -> calibrate_intrinsic -> calibrate_eye_in_hand -> versioned YAML.

Vi mount_T_optical chua biet (dang can calibrate), FOV check dung GIA DINH
lam viec optical==mount + K nominal tu config, CHI de loc candidate. Pose xau
that se bi loai o capture-time (khong thay checkerboard -> skip). Khong dung
gia dinh nay cho bat ky gia tri calibration nao.

Nguoi dung chi can: dat board, dua tay toi 1 tu the nhin ro board, chay lenh.
Seed mac dinh doc tu the tay hien tai; board-xy tuy chon:
    ros2 run cap_vision generate_calib_poses -- \
        --config src/cap_vision/config/red_scene.yaml \
        --output calib_poses_v1.yaml
"""

import argparse
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from std_msgs.msg import Header

ALL_JOINTS = [f"arm{i}_Joint" for i in range(1, 6)] + ["Rlink1_Joint"]
ARM = [f"arm{i}_Joint" for i in range(1, 6)]
GRIP = "Rlink1_Joint"
LIMITS = {f"arm{i}_Joint": (-1.57, 1.57) for i in range(1, 6)}
LIMITS[GRIP] = (0.0, 1.57)
MARGIN_SEED = 0.02  # seed: theo safety_config (tu the 90deg la binh thuong)
MARGIN_CAND = 0.05  # candidate: du cho nhieu joint ma khong sat bien
MARGIN = MARGIN_CAND
PERTURB = [0.35, 0.25, 0.30, 0.30, 0.40]  # bien do nhieu joint 1..5
PERTURB_CONSERVATIVE = [0.15, 0.12, 0.15, 0.15, 0.20]  # khi khong co board-xy
MOUNT_LINK = "Camera_Link"


def read_live_seed(node, timeout_sec=10.0):
    """Doc 6 joints hien tai tu topic that. Tra ve dict hoac None."""
    from sensor_msgs.msg import JointState
    got = {}

    def on_js(msg):
        q = dict(zip(msg.name, msg.position))
        if all(j in q for j in ALL_JOINTS):
            for j in ALL_JOINTS:
                got[j] = float(q[j])

    subs = [node.create_subscription(JointState, t, on_js, 10)
            for t in ("/real_joint_states", "/joint_states")]
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec and not all(j in got for j in ALL_JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    if not all(j in got for j in ALL_JOINTS):
        return None
    return {j: got[j] for j in ALL_JOINTS}


def read_armlib_seed():
    """Fallback: doc truc tiep servo (mo-doc-dong, khong giu serial)."""
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "hardware"))
    from real_joint_mirror import find_port
    from arm_hw_bridge import read_servos, servo_to_rad_dict
    import Arm_Lib
    port = find_port(["/dev/ttyUSB0", "/dev/ttyAMA0", "/dev/myserial"])
    if port is None:
        return None
    arm = Arm_Lib.Arm_Device(port)
    try:
        return servo_to_rad_dict(read_servos(arm)[0])
    finally:
        try:
            arm.ser.close()
        except Exception:
            pass


def plan_joint(node, cli, q_from, q_to):
    req = GetMotionPlan.Request()
    motion = req.motion_plan_request
    motion.group_name = "arm_group"
    state = RobotState(is_diff=False)
    state.joint_state.name = ARM + [GRIP]
    state.joint_state.position = [float(q_from[j]) for j in ARM + [GRIP]]
    motion.start_state = state
    motion.allowed_planning_time = 5.0
    motion.num_planning_attempts = 3
    motion.max_velocity_scaling_factor = 0.1
    motion.max_acceleration_scaling_factor = 0.1
    goal = Constraints()
    goal.joint_constraints = [JointConstraint(
        joint_name=j, position=float(q_to[j]),
        tolerance_above=0.01, tolerance_below=0.01, weight=1.0) for j in ARM]
    motion.goal_constraints = [goal]
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not fut.done():
        return False
    return fut.result().motion_plan_response.error_code.val == 1


def fk_mount(node, cli, q):
    req = GetPositionFK.Request(header=Header(frame_id="base_link"),
                                fk_link_names=[MOUNT_LINK])
    req.robot_state = RobotState(is_diff=False)
    req.robot_state.joint_state.name = ARM + [GRIP]
    req.robot_state.joint_state.position = [float(q[j]) for j in ARM + [GRIP]]
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not fut.done():
        return None
    res = fut.result()
    if res.error_code.val != 1 or not res.pose_stamped:
        return None
    return res.pose_stamped[0].pose


def quat_to_mat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def board_visible(mount_pose, board_xy, board_half, table_z, K, W, H):
    """Chieu 4 goc board len anh voi gia dinh optical==mount (loc tho)."""
    p = mount_pose.position
    o = mount_pose.orientation
    R = quat_to_mat([o.x, o.y, o.z, o.w])
    t = np.array([p.x, p.y, p.z])
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            pw = np.array([board_xy[0] + sx * board_half,
                           board_xy[1] + sy * board_half, table_z])
            pc = R.T @ (pw - t)
            if pc[2] <= 0.05:
                return False
            u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
            v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
            if not (30 <= u <= W - 30 and 30 <= v <= H - 30):
                return False
            corners.append((u, v))
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--seed-joints", default=None,
                    help="6 joint goc (rad). Bo qua = doc tu the tay hien tai "
                         "(topic /real_joint_states, /joint_states hoac ArmLib). "
                         "Tay phai dang o tu the nhin ro board.")
    ap.add_argument("--board-xy", default=None,
                    help="tam board do tay, vd 0.15,0.0. Bo qua = nhieu bao thu "
                         "quanh seed, chap nhan/loai bang detection that luc capture.")
    ap.add_argument("--table-z", type=float, default=0.045)
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--square", type=float, default=0.025)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--tries", type=int, default=400)
    ap.add_argument("--rng-seed", type=int, default=7)
    ap.add_argument("--max-seed-delta", type=float, default=None,
                    help="optional per-joint bound around live seed (rad); useful for commissioning")
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    if a.max_seed_delta is not None and not 0.05 <= a.max_seed_delta <= 0.6:
        raise SystemExit("max-seed-delta must be in [0.05, 0.6] rad")
    if Path(a.output).exists():
        raise SystemExit("output exists; choose a new poses file")
    cols, rows = (int(v) for v in a.pattern.lower().split("x"))
    if a.seed_joints:
        seed = [float(v) for v in a.seed_joints.split(",")]
        if len(seed) != 6 or not all(math.isfinite(v) for v in seed):
            raise SystemExit("seed-joints can 6 gia tri rad huu han")
        seed_q = dict(zip(ARM + [GRIP], seed))
    else:
        seed_q = None
    board_xy = None
    if a.board_xy:
        board_xy = [float(v) for v in a.board_xy.split(",")]
        if len(board_xy) != 2 or not all(math.isfinite(v) for v in board_xy):
            raise SystemExit("board-xy can 2 gia tri m huu han")
    with open(a.config, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cam = cfg["camera"]
    K = np.array(cam["K"], float).reshape(3, 3)
    W, H = int(cam["width"]), int(cam["height"])
    board_half = max(cols, rows) * a.square / 2.0 + 0.02

    rclpy.init()
    node = Node("generate_calib_poses")
    try:
        if seed_q is None:
            seed_q = read_live_seed(node)
            if seed_q is None:
                print("topic khong co state, thu doc truc tiep ArmLib...", flush=True)
                seed_q = read_armlib_seed()
            if seed_q is None:
                raise SystemExit(
                    "khong doc duoc tu the hien tai (topic + ArmLib deu that bai). "
                    "Dua tay toi tu the nhin ro board roi chay lai, hoac truyen --seed-joints.")
            print("seed = tu the tay hien tai: "
                  + ",".join(f"{seed_q[j]:.3f}" for j in ARM + [GRIP]), flush=True)
        for j in ARM:
            lo, hi = LIMITS[j]
            if seed_q[j] < lo - 0.01 or seed_q[j] > hi + 0.01:
                raise SystemExit(
                    f"seed {j}={seed_q[j]:.3f} vuot limit - dua tay ve tu the "
                    "nhin ro board (giua hanh trinh) roi chay lai")
            clamped = float(min(max(seed_q[j], lo + 0.005), hi - 0.005))
            if clamped != seed_q[j]:
                # < do phan giai servo (1do ~= 0.017rad): tu the that khong doi.
                print(f"WARN: kep seed {j} {seed_q[j]:.4f} -> {clamped:.4f} "
                      "(duoi do phan giai servo)", flush=True)
                seed_q[j] = clamped
        lo, hi = LIMITS[GRIP]  # gripper dung yen suot calibration: chi check limit cung
        if not (lo <= seed_q[GRIP] <= hi):
            raise SystemExit(f"seed gripper={seed_q[GRIP]:.3f} vuot limit")
        plan_cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        fk_cli = node.create_client(GetPositionFK, "/compute_fk")
        if not plan_cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit("thieu /plan_kinematic_path (mo MoveIt)")
        if not fk_cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit("thieu /compute_fk (mo MoveIt)")

        rng = np.random.default_rng(a.rng_seed)
        amps = PERTURB if board_xy is not None else PERTURB_CONSERVATIVE
        if board_xy is None:
            print("khong co board-xy: nhieu bao thu quanh seed, chap nhan/loai "
                  "bang detection that luc capture", flush=True)
        accepted = [dict(seed_q)]
        prev = dict(seed_q)
        for _ in range(a.tries):
            if len(accepted) >= a.count:
                break
            cand = dict(prev)
            for j, amp in zip(ARM, amps):
                lo, hi = LIMITS[j]
                v = prev[j] + float(rng.uniform(-amp, amp))
                if a.max_seed_delta is not None:
                    v = min(max(v, seed_q[j] - a.max_seed_delta),
                            seed_q[j] + a.max_seed_delta)
                cand[j] = float(min(max(v, lo + MARGIN), hi - MARGIN))
            cand[GRIP] = seed_q[GRIP]
            if max(abs(cand[j] - prev[j]) for j in ARM) < 0.05:
                continue
            mount = fk_mount(node, fk_cli, cand)
            if mount is None:
                continue
            if board_xy is not None and not board_visible(
                    mount, board_xy, board_half, a.table_z, K, W, H):
                continue
            if not plan_joint(node, plan_cli, prev, cand):
                continue
            accepted.append(cand)
            prev = cand
        if len(accepted) < a.count:
            raise SystemExit(f"chi duoc {len(accepted)}/{a.count} poses (tang --tries / doi seed)")

        # Kiem tra da dang xoay (cung gate voi solver).
        import cv2
        rots = []
        mounts = []
        for cand in accepted:
            m = fk_mount(node, fk_cli, cand)
            o = m.orientation
            mounts.append(quat_to_mat([o.x, o.y, o.z, o.w]))
        for R in mounts[1:]:
            r, _ = cv2.Rodrigues(mounts[0].T @ R)
            rots.append(r.ravel())
        s = np.linalg.svd(np.array(rots), compute_uv=False)
        print(f"rotation diversity singular={s[0]:.3f},{s[1]:.3f} (can s[1]>=0.15)")
        if s[1] < 0.15:
            raise SystemExit("thieu da dang xoay (doi seed / tang bien do)")

        samples = [{"joints": [cand[j] for j in ARM + [GRIP]],
                    "validation": (i % 5 == 4)}
                   for i, cand in enumerate(accepted)]
        n_val = sum(1 for s_ in samples if s_["validation"])
        if n_val < 3:
            raise SystemExit("can >=3 held-out (tang --count)")
        out = {"pattern": [cols, rows], "square_m": a.square,
               "board_xy_guess": board_xy, "table_z_guess": a.table_z,
               "optical_assumption": "identity mount==optical, candidate filtering only",
               "conservative": board_xy is None,
               "max_seed_delta": a.max_seed_delta,
               "seed_joints": [seed_q[j] for j in ARM + [GRIP]], "rng_seed": a.rng_seed,
               "rotation_singular": [float(s[0]), float(s[1])],
               "samples": samples}
        with open(a.output, "x", encoding="utf-8") as f:
            yaml.safe_dump(out, f, sort_keys=False)
        print(f"Saved {a.output}: {len(samples)} poses ({n_val} held-out)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
