#!/usr/bin/env python3
"""safety_gate.py — cổng an toàn trước MỖI lần gửi tín hiệu ra tay thật.

Hiện thực các mục trong docs/real_world_plan.md thành check fail-loud:

  #1  joint limit + software margin (position)
  #2  velocity/acceleration scaling thấp (trần HW 0.25)
  #3  acceleration spike / jerk ước lượng từ waypoint
  #6  TCP calibrated flag (chưa đo -> chặn HW, sim vẫn qua)
  #10 start-state == state thật (chống giật về point 0)
  #11 joint_states / servo readback phải tươi (MAX_STATE_AGE)
  #12 kiểm tra TOÀN trajectory, không chỉ goal
  #13 branch jump giữa waypoint kề (MAX_JOINT_STEP)
  #14 gripper limits (0..1.57, cấm nội suy mm->rad tuyến tính)
  #4,5,7,8,9,15,16 được gate ở tầng caller (scene/TF/collision/attach/HOME/E-stop)
      qua check_scene() + checklist trong RUNBOOK.md.

KHÔNG phụ thuộc ROS: đọc trajectory JSON + config YAML là chạy được.
Caller ROS (arm_hw_bridge) import SafetyGate và gọi cùng hàm.

Định dạng trajectory JSON:
  {
    "joint_names": ["arm1_Joint", ..., "arm5_Joint", "Rlink1_Joint"],
    "points": [{"positions": [6 số rad], "time_from_start": 1.2}, ...],
    "velocity_scaling": 0.1, "acceleration_scaling": 0.1
  }
Current state JSON:
  {"positions_rad": {"arm1_Joint": 0.0, ...}, "age_sec": 0.2}
Hoặc dùng --q0 "0,0,0,0,0,0" (rad, thứ tự joint_names) + --q0-age 0.2.

Ví dụ:
  python3 safety_gate.py --self-test
  python3 safety_gate.py --traj traj.json --q0 0,0,0,0,0,0 --q0-age 0.2
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "safety_config.yaml"

ARM_JOINTS = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joint", "arm5_Joint"]
GRIPPER_JOINT = "Rlink1_Joint"
ALL_JOINTS = [*ARM_JOINTS, GRIPPER_JOINT]


# ---------------------------------------------------------------- mapping servo
# VERIFY với dofbot_ws/.../dofbot_driver.py + Arm_Lib 0.0.5 (đọc code thật):
#   driver gốc gửi write6(x1, 180-x2, 180-x3, 180-x4, x5, grip) với x=deg(rad)+90,
#   Arm_Lib.write6 ĐẢO s2..s4 lần nữa (s=180-s) -> góc vật lý = 90+deg(rad).
# => Muốn vật lý trùng driver gốc (trùng URDF), ta phải gửi TRƯỚC khi Arm_Lib đảo:
#   s2..s4 = 180 - (90+deg) = 90 - deg. Gửi 90+deg như trước đây là ĐẢO NGƯỢC
#   khớp 2,3,4 so với sim (real đi ngược sim) — đã fix 2026-09-21.
_MIRRORED = ("arm2_Joint", "arm3_Joint", "arm4_Joint")


def rad_to_servo_deg(joint: str, rad: float) -> float:
    """MoveIt rad -> servo độ (chưa clip)."""
    deg = math.degrees(rad)
    if joint in ("arm1_Joint", "arm5_Joint"):
        return deg + 90.0
    if joint in _MIRRORED:
        return 90.0 - deg  # bù đảo 180-x của Arm_Lib.write6 để vật lý = 90+deg
    if joint == GRIPPER_JOINT:
        # driver gốc: grip_angle = deg+90 in [90,180] -> interp [30,180]
        grip_angle = deg + 90.0
        lo_in, hi_in, lo_out, hi_out = 90.0, 180.0, 30.0, 180.0
        t = (grip_angle - lo_in) / (hi_in - lo_in)
        return lo_out + t * (hi_out - lo_out)
    raise KeyError(f"joint lạ: {joint}")


def servo_deg_to_rad(joint: str, servo: float) -> float:
    """Servo độ -> MoveIt rad (nghịch đảo của rad_to_servo_deg)."""
    if joint in ("arm1_Joint", "arm5_Joint"):
        return math.radians(servo - 90.0)
    if joint in _MIRRORED:
        # read() trả s = 180 - góc_vật_lý = 180 - (90+deg) = 90-deg
        return math.radians(90.0 - servo)
    if joint == GRIPPER_JOINT:
        lo_in, hi_in, lo_out, hi_out = 90.0, 180.0, 30.0, 180.0
        t = (servo - lo_out) / (hi_out - lo_out)
        return math.radians(lo_in + t * (hi_in - lo_in) - 90.0)
    raise KeyError(f"joint lạ: {joint}")


def clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_readback_limits(positions: dict[str, float], cfg: dict) -> dict[str, float]:
    """Snap quantized *measured* state to a nearby URDF endpoint.

    Yahboom reports integer servo degrees, while its URDF uses the rounded
    endpoint 1.57 rad.  Thus a legitimate 90 degree readback is pi/2 and lies
    0.0008 rad outside that file.  Keep genuinely out-of-range measurements
    untouched so SafetyGate still rejects them; never call this for commands.
    """
    tolerance = float(cfg.get("readback_limit_snap_rad", 0.01))
    limits = cfg.get("urdf_limits_rad", {})
    normalized = dict(positions)
    for name, value in positions.items():
        if name not in limits:
            continue
        lo, hi = map(float, limits[name])
        if lo - tolerance <= value < lo:
            normalized[name] = lo
        elif hi < value <= hi + tolerance:
            normalized[name] = hi
    return normalized


# ---------------------------------------------------------------- config

def load_config(path: Path | str = DEFAULT_CONFIG) -> dict:
    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None  # fallback parser tối giản bên dưới
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    if yaml is not None:
        return yaml.safe_load(text)
    # Fallback: chỉ đọc các key số mà gate cần (tránh phụ thuộc pyyaml trên image).
    import re
    cfg: dict = {}
    def num(key: str, default: float) -> float:
        m = re.search(rf"^\s*{re.escape(key)}:\s*([0-9.]+)", text, re.M)
        return float(m.group(1)) if m else default
    cfg["joint_limit_margin_rad"] = num("joint_limit_margin_rad", 0.02)
    cfg["readback_limit_snap_rad"] = num("readback_limit_snap_rad", 0.01)
    cfg["start_state_tol_rad"] = num("start_state_tol_rad", 0.05)
    cfg["max_joint_step_rad"] = num("max_joint_step_rad", 0.6)
    cfg["velocity_scaling_max_hw"] = num("velocity_scaling_max_hw", 0.25)
    cfg["acceleration_scaling_max_hw"] = num("acceleration_scaling_max_hw", 0.25)
    cfg["max_velocity_rad_s"] = num("max_velocity_rad_s", 2.0)
    cfg["max_accel_rad_s2"] = num("max_accel_rad_s2", 4.0)
    cfg["joint_state_max_age_sec"] = num("joint_state_max_age_sec", 1.0)
    cfg["execute_endpoint_tol_rad"] = num("execute_endpoint_tol_rad", 0.08)
    cfg["tcp_offset_calibrated"] = "tcp_offset_calibrated: true" in text
    cfg["urdf_limits_rad"] = {
        "arm1_Joint": [-1.57, 1.57], "arm2_Joint": [-1.57, 1.57],
        "arm3_Joint": [-1.57, 1.57], "arm4_Joint": [-1.57, 1.57],
        "arm5_Joint": [-1.57, 1.57], "Rlink1_Joint": [0.0, 1.57],
    }
    cfg["required_scene_objects"] = ["chessboard", "table"]
    return cfg


# ---------------------------------------------------------------- gate

class SafetyGate:
    """Gom mọi check trajectory. fail-loud: raise hoặc trả reasons, không clamp câm."""

    def __init__(self, cfg: dict | None = None):
        self.cfg = cfg or load_config()

    # -- #11 freshness ------------------------------------------------------
    def check_fresh(self, age_sec: float | None) -> list[str]:
        max_age = float(self.cfg.get("joint_state_max_age_sec", 1.0))
        if age_sec is None:
            return ["state age unknown (không có timestamp readback)"]
        if age_sec > max_age:
            return [f"state stale {age_sec:.1f}s > {max_age:.1f}s (real_world_plan #11)"]
        return []

    # -- #1 limits + margin --------------------------------------------------
    def check_limits(self, positions: dict[str, float], *,
                     allowed_margin_joints: set[str] | None = None) -> list[str]:
        margin = float(self.cfg.get("joint_limit_margin_rad", 0.02))
        limits = self.cfg.get("urdf_limits_rad", {})
        allowed_margin_joints = allowed_margin_joints or set()
        out: list[str] = []
        for name, v in positions.items():
            lim = limits.get(name)
            if lim is None:
                out.append(f"{name}: không có URDF limit trong config")
                continue
            lo, hi = float(lim[0]), float(lim[1])
            if not (lo <= v <= hi):
                out.append(f"{name}={v:.3f}rad vượt URDF [{lo},{hi}] (#1)")
            elif (name in ARM_JOINTS and min(v - lo, hi - v) < margin and
                  name not in allowed_margin_joints):
                # Margin chỉ áp cho arm (đừng dùng sát biên cơ khí). Gripper
                # open=0.0 / close=1.57 là điểm vận hành bình thường (#14),
                # không warn margin.
                out.append(f"{name}={v:.3f}rad sát limit (margin<{margin}rad) (#1)")
        # servo clip tương ứng (phát hiện sớm trước khi gửi Arm_Lib)
        servo_limits = {"s1": (0, 180), "s2": (0, 180), "s3": (0, 180),
                        "s4": (0, 180), "s5": (0, 270), "s6": (30, 180)}
        order = ["arm1_Joint", "arm2_Joint", "arm3_Joint",
                 "arm4_Joint", "arm5_Joint", GRIPPER_JOINT]
        for i, name in enumerate(order):
            if name in positions:
                s = rad_to_servo_deg(name, positions[name])
                lo, hi = servo_limits[f"s{i+1}"]
                if not (lo - 1e-9 <= s <= hi + 1e-9):
                    out.append(f"{name}: servo {s:.1f}deg vượt [{lo},{hi}] (#1/#14)")
        return out

    # -- #2 scaling -----------------------------------------------------------
    def check_scaling(self, vel_scale: float, acc_scale: float) -> list[str]:
        out: list[str] = []
        vmax = float(self.cfg.get("velocity_scaling_max_hw", 0.25))
        amax = float(self.cfg.get("acceleration_scaling_max_hw", 0.25))
        for label, v, m in (("velocity", vel_scale, vmax), ("acceleration", acc_scale, amax)):
            if not (0.0 < v <= m):
                out.append(f"{label}_scaling={v} vượt trần HW {m} (#2)")
        return out

    # -- #10 start-state -------------------------------------------------------
    def check_start(self, q0_traj: dict[str, float], q0_real: dict[str, float]) -> list[str]:
        tol = float(self.cfg.get("start_state_tol_rad", 0.05))
        out: list[str] = []
        for name in ALL_JOINTS:
            if name in q0_traj and name in q0_real:
                d = abs(q0_traj[name] - q0_real[name])
                if d > tol:
                    out.append(f"start mismatch {name}: |traj-real|={d:.3f}rad > {tol} (#10)")
        return out

    # -- #12/#13/#3 full trajectory -------------------------------------------
    def check_trajectory(self, joint_names: list[str], points: list[dict],
                         q0_real: dict[str, float] | None = None,
                         state_age_sec: float | None = 0.0,
                         vel_scale: float = 0.1, acc_scale: float = 0.1,
                         scene_objects: list[str] | None = None,
                         allow_hw: bool = False) -> tuple[bool, list[str]]:
        """Check toàn bộ waypoint. allow_hw=False -> chặn ở gate TCP calibration."""
        reasons: list[str] = []
        reasons += self.check_scaling(vel_scale, acc_scale)
        if allow_hw and not bool(self.cfg.get("tcp_offset_calibrated", False)):
            reasons.append("TCP_OFFSET chưa calibration -> chặn gửi HW (#6)")
        if q0_real is None:
            reasons.append("thiếu state thật để so start-state (#10)")
        else:
            reasons += self.check_fresh(state_age_sec)
        if scene_objects is not None:
            reasons += self.check_scene(scene_objects)
        if not points:
            return False, reasons + ["trajectory rỗng (#12)"]

        max_step = float(self.cfg.get("max_joint_step_rad", 0.6))
        max_vel = float(self.cfg.get("max_velocity_rad_s", 2.0)) * max(vel_scale, 1e-6)
        max_acc = float(self.cfg.get("max_accel_rad_s2", 4.0))
        idx = {n: i for i, n in enumerate(joint_names)}

        def pos_at(p: dict, name: str) -> float:
            return float(p["positions"][idx[name]])

        def margin_escape_prefix() -> dict[int, set[str]]:
            """Return per-point joints allowed while monotonically escaping.

            Some Yahboom arms power up exactly at a joint endpoint. Rejecting
            that measured start forever makes it impossible to move toward the
            interior. The exception applies only to joints already near a limit
            at the measured point 0, and only for a non-decreasing-distance
            prefix that eventually exits the software margin.
            """
            allowed: dict[int, set[str]] = {k: set() for k in range(len(points))}
            if q0_real is None or len(points) < 2:
                return allowed
            margin = float(self.cfg.get("joint_limit_margin_rad", 0.02))
            limits = self.cfg.get("urdf_limits_rad", {})
            for name in ARM_JOINTS:
                if name not in idx or name not in limits:
                    continue
                lo, hi = map(float, limits[name])
                start = pos_at(points[0], name)
                start_distance = min(start - lo, hi - start)
                if start_distance >= margin:
                    continue
                prefix = []
                previous = start_distance
                escaped = False
                for k, point in enumerate(points):
                    value = pos_at(point, name)
                    distance = min(value - lo, hi - value)
                    if distance + 1e-6 < previous:
                        break
                    if distance >= margin:
                        escaped = True
                        break
                    prefix.append(k)
                    previous = distance
                if escaped:
                    for k in prefix:
                        allowed[k].add(name)
            return allowed

        # point 0 vs limits + start
        q0 = {n: pos_at(points[0], n) for n in joint_names if n in idx}
        margin_escape = margin_escape_prefix()
        reasons += self.check_limits(q0, allowed_margin_joints=margin_escape[0])
        if q0_real is not None:
            reasons += self.check_start(q0, q0_real)

        prev_vel: dict[str, float] = {}
        prev_t = 0.0
        for k, p in enumerate(points):
            q = {n: pos_at(p, n) for n in joint_names if n in idx}
            reasons += [f"pt{k}: {r}" for r in self.check_limits(
                q, allowed_margin_joints=margin_escape[k])]
            t = float(p.get("time_from_start", k + 1))
            dt = t - prev_t if k > 0 else t
            if dt <= 0:
                reasons.append(f"pt{k}: time_from_start không tăng (dt={dt}) (#3)")
                continue
            if k > 0:
                qp = {n: pos_at(points[k - 1], n) for n in joint_names if n in idx}
                for n in joint_names:
                    step = abs(q[n] - qp[n])
                    if step > max_step:
                        reasons.append(f"pt{k} {n}: joint jump {step:.3f}rad > {max_step} (#13)")
                    vel = (q[n] - qp[n]) / dt
                    if abs(vel) > max_vel * 1.5:  # *1.5 dung sai ước lượng số
                        reasons.append(f"pt{k} {n}: vel {vel:.2f}rad/s > {max_vel:.2f} (#2)")
                    if n in prev_vel:
                        acc = (vel - prev_vel[n]) / dt
                        if abs(acc) > max_acc * 2.0:
                            reasons.append(f"pt{k} {n}: accel spike {acc:.1f}rad/s2 (#3)")
                    prev_vel[n] = vel
            prev_t = t
        return (len(reasons) == 0), reasons

    # -- #4 scene ---------------------------------------------------------------
    def check_scene(self, objects: list[str]) -> list[str]:
        required = list(self.cfg.get("required_scene_objects", []))
        missing = [o for o in required if o not in objects]
        if missing:
            return [f"PlanningScene thiếu {missing} (MoveIt chỉ tránh thứ nó biết) (#4)"]
        return []


# ---------------------------------------------------------------- CLI

def _parse_q0(s: str, joint_names: list[str]) -> dict[str, float]:
    vals = [float(x) for x in s.split(",")]
    if len(vals) != len(joint_names):
        raise ValueError(f"--q0 cần {len(joint_names)} số theo {joint_names}")
    return dict(zip(joint_names, vals))


def self_test() -> int:
    cfg = load_config()
    gate = SafetyGate(cfg)
    joints = list(ALL_JOINTS)
    ok_all = True

    # 1. trajectory tốt phải PASS
    good = [{"positions": [0.0, -0.2, 0.3, -0.1, 0.0, 0.0], "time_from_start": 2.0},
            {"positions": [0.1, -0.25, 0.35, -0.1, 0.05, 0.0], "time_from_start": 4.0}]
    q0 = dict(zip(joints, good[0]["positions"]))
    ok, reasons = gate.check_trajectory(joints, good, q0_real=q0, state_age_sec=0.1,
                                        vel_scale=0.1, acc_scale=0.1,
                                        scene_objects=["chessboard", "table"],
                                        allow_hw=False)
    print(f"[SELF-TEST] good traj: {'PASS' if ok else 'FAIL ' + str(reasons)}")
    ok_all &= ok

    # 2. vượt limit phải FAIL
    bad_lim = [{"positions": [3.0, 0, 0, 0, 0, 0], "time_from_start": 2.0}]
    ok, reasons = gate.check_trajectory(joints, bad_lim, q0_real=dict(zip(joints, bad_lim[0]["positions"])),
                                        state_age_sec=0.1, scene_objects=["chessboard", "table"])
    print(f"[SELF-TEST] over-limit rejected: {'OK' if not ok else 'NOT-REJECTED'} {reasons[:1]}")
    ok_all &= (not ok)

    # 3. scaling cao phải FAIL
    ok, reasons = gate.check_trajectory(joints, good, q0_real=q0, state_age_sec=0.1,
                                        vel_scale=1.0, acc_scale=1.0,
                                        scene_objects=["chessboard", "table"])
    print(f"[SELF-TEST] scaling=1.0 rejected: {'OK' if not ok else 'NOT-REJECTED'} {reasons[:1]}")
    ok_all &= (not ok)

    # 4. start mismatch phải FAIL
    q0_wrong = dict(q0); q0_wrong["arm1_Joint"] = 0.5
    ok, reasons = gate.check_trajectory(joints, good, q0_real=q0_wrong, state_age_sec=0.1,
                                        scene_objects=["chessboard", "table"])
    print(f"[SELF-TEST] start-mismatch rejected: {'OK' if not ok else 'NOT-REJECTED'} {reasons[:1]}")
    ok_all &= (not ok)

    # 5. stale state phải FAIL
    ok, reasons = gate.check_trajectory(joints, good, q0_real=q0, state_age_sec=5.0,
                                        scene_objects=["chessboard", "table"])
    print(f"[SELF-TEST] stale rejected: {'OK' if not ok else 'NOT-REJECTED'} {reasons[:1]}")
    ok_all &= (not ok)

    # 6. jump phải FAIL
    jump = [{"positions": [0, 0, 0, 0, 0, 0], "time_from_start": 1.0},
            {"positions": [1.2, 0, 0, 0, 0, 0], "time_from_start": 2.0}]
    ok, reasons = gate.check_trajectory(joints, jump, q0_real=dict(zip(joints, jump[0]["positions"])),
                                        state_age_sec=0.1, scene_objects=["chessboard", "table"])
    print(f"[SELF-TEST] jump rejected: {'OK' if not ok else 'NOT-REJECTED'} {reasons[:1]}")
    ok_all &= (not ok)

    # 7. mapping servo sanity: rad 0 -> 90, 1.57 -> 180
    assert abs(rad_to_servo_deg("arm1_Joint", 0.0) - 90.0) < 1e-9
    assert abs(rad_to_servo_deg("arm1_Joint", 1.57) - 180.0) < 1.0
    assert abs(rad_to_servo_deg("Rlink1_Joint", 0.0) - 30.0) < 1e-9
    assert abs(rad_to_servo_deg("Rlink1_Joint", 1.57) - 180.0) < 1.0
    print("[SELF-TEST] servo mapping OK (0rad->90deg, grip 0->30/1.57->180)")
    print("[SELF-TEST] TỔNG:", "ALL PASS" if ok_all else "CÓ CASE FAIL")
    return 0 if ok_all else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Safety gate trước khi gửi tay thật")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--traj", help="trajectory JSON")
    ap.add_argument("--current", help="current state JSON {positions_rad, age_sec}")
    ap.add_argument("--q0", help="current rad, csv theo joint_names (khi không có --current)")
    ap.add_argument("--q0-age", type=float, default=0.0)
    ap.add_argument("--vel-scale", type=float, default=0.1)
    ap.add_argument("--acc-scale", type=float, default=0.1)
    ap.add_argument("--scene", default="chessboard,table", help="objects trong PlanningScene, csv")
    ap.add_argument("--allow-hw", action="store_true", help="check như khi gửi HW (bật gate TCP calibration)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.traj:
        ap.error("--traj bắt buộc (hoặc --self-test)")

    cfg = load_config(args.config)
    gate = SafetyGate(cfg)
    traj = json.loads(Path(args.traj).read_text(encoding="utf-8"))
    joints = traj["joint_names"]
    points = traj["points"]
    if args.current:
        cur = json.loads(Path(args.current).read_text(encoding="utf-8"))
        q0_real = {k: float(v) for k, v in cur["positions_rad"].items()}
        age = float(cur.get("age_sec", 0.0))
    else:
        if not args.q0:
            ap.error("cần --current hoặc --q0")
        q0_real = _parse_q0(args.q0, joints)
        age = args.q0_age
    ok, reasons = gate.check_trajectory(
        joints, points, q0_real=q0_real, state_age_sec=age,
        vel_scale=args.vel_scale, acc_scale=args.acc_scale,
        scene_objects=[s.strip() for s in args.scene.split(",") if s.strip()],
        allow_hw=args.allow_hw)
    if ok:
        print("SAFETY-GATE: PASS — được phép chuyển sang bước gửi HW (vẫn cần operator xác nhận + tay trên E-stop).")
        return 0
    print("SAFETY-GATE: FAIL — chặn gửi HW:")
    for r in reasons:
        print(f"  - {r}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
