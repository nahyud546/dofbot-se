#!/usr/bin/env python3
"""Closed-form FK/IK for the 5-DOF DOFBOT (base_link -> Gripping_point_Link).

Pure Python, no ROS. Servo convention: q_rad = (servo_deg - 90) * pi / 180.
The work side is -X (elbow as in READY [90,125,0,0,90]).

A 5-DOF arm cannot hold one gripper pitch over the whole table, so the solver
picks the pitch per target: the most vertical gripper tilt that respects the
servo limits. Tilt is measured from straight-down, positive leaning outward.
"""
import json
import math
import os
from pathlib import Path

# dofbot.urdf joint origins (metres).
J1_O = (0.0, 0.0, 0.0925)
J2_O = (0.0, 5e-05, 0.033)
J3_O = (0.0, 0.00055, 0.08285)
J4_O = (0.0, 5e-05, 0.08285)
J5_O = (-0.00215, -4.5e-05, 0.078149)
GRIP_O = (-0.00265, 9.7552e-05, 0.068091)

L1 = J3_O[2]
L2 = J4_O[2]
SHOULDER_Z = J1_O[2] + J2_O[2]

TILT_MIN_DEG = -20.0
TILT_MAX_DEG = 90.0
TILT_STEP_DEG = 0.5
LIMIT_MARGIN_DEG = 3.0
# A nearly straight elbow is singular: small tilt changes swing J3 by degrees.
MIN_ELBOW_BEND_DEG = 20.0
MAX_J2_DEG = 100.0
MAX_J4_DEG = 120.0


# Hiệu chỉnh J1: góc thật = 90 + scale*(đọc-90) + offset. Chỉ bật khi hand_eye.json có
# j1_correction.enabled (cùng cờ với cube_search_center_math.fk_arm4) để lệnh gắp (IK) và
# quan sát (hand-eye) dùng cùng một hình học. Mặc định không đổi gì.
J1_SCALE = 1.0
J1_OFFSET_DEG = 0.0
HAND_EYE_ENV = "T8_HAND_EYE_FILE"
_HAND_EYE_DEFAULT = Path(__file__).resolve().parents[2] / "config" / "robot" / "hand_eye.json"


def set_j1_correction(scale: float = 1.0, offset_deg: float = 0.0) -> None:
    global J1_SCALE, J1_OFFSET_DEG
    if not (0.85 <= float(scale) <= 1.25 and abs(float(offset_deg)) <= 5.0):
        raise ValueError("hiệu chỉnh J1 ngoài khoảng cho phép (scale 0.85-1.25, offset <= 5°)")
    J1_SCALE, J1_OFFSET_DEG = float(scale), float(offset_deg)


def load_j1_correction(path=None) -> tuple[float, float]:
    """Đọc j1_correction từ hand_eye.json (nếu enabled và accepted); mặc định (1, 0)."""
    try:
        data = json.loads(Path(path or os.environ.get(HAND_EYE_ENV) or _HAND_EYE_DEFAULT).read_text())
        j1 = data.get("j1_correction") or {}
        if data.get("accepted") is True and j1.get("enabled") is True:
            set_j1_correction(float(j1["scale"]), float(j1.get("offset_deg", 0.0)))
            return J1_SCALE, J1_OFFSET_DEG
    except (OSError, ValueError, KeyError, TypeError):
        pass
    set_j1_correction(1.0, 0.0)
    return 1.0, 0.0


def _true_j1(reading):
    return 90.0 + J1_SCALE * (float(reading) - 90.0) + J1_OFFSET_DEG


def _reading_j1(true_deg):
    return 90.0 + (float(true_deg) - 90.0 - J1_OFFSET_DEG) / J1_SCALE


class NoSolution(ValueError):
    """Target is outside the reachable workspace for every allowed tilt."""


def _ry(p, q):
    c, s = math.cos(q), math.sin(q)
    return (p[0] * c + p[2] * s, p[1], -p[0] * s + p[2] * c)


def _rz(p, q):
    c, s = math.cos(q), math.sin(q)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def fk(servo_deg):
    """Gripping point (x, y, z) in base_link for 5 servo angles in degrees."""
    q = [math.radians(float(v) - 90.0) for v in servo_deg[:5]]
    q[0] = math.radians(_true_j1(servo_deg[0]) - 90.0)
    p = _add(_rz(GRIP_O, q[4]), J5_O)
    p = _add(_ry(p, q[3]), J4_O)
    p = _add(_ry(p, q[2]), J3_O)
    p = _add(_ry(p, q[1]), J2_O)
    p = _add(_rz(p, q[0]), J1_O)
    return p


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _planar(px, pz, tilt_deg, ox, l3):
    """Servo J2..J4 candidates for a planar target at one gripper tilt."""
    a4 = math.radians(tilt_deg - 180.0)
    wx = px - l3 * math.sin(a4) - ox * math.cos(a4)
    wz = pz - SHOULDER_Z - l3 * math.cos(a4) + ox * math.sin(a4)
    cos_q3 = (wx * wx + wz * wz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(cos_q3) > 1.0:
        return []
    out = []
    for q3 in (-math.acos(cos_q3), math.acos(cos_q3)):
        a2 = math.atan2(wx, wz) - math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
        q4 = _wrap(a4 - a2 - q3)
        out.append([math.degrees(_wrap(a2)) + 90.0, math.degrees(q3) + 90.0,
                    math.degrees(q4) + 90.0])
    return out


def _within(j234, margin, bend, max_j2, max_j4):
    j2, j3, j4 = j234
    return (abs(j3 - 90.0) >= bend and margin <= j2 <= min(180.0 - margin, max_j2) and
            margin <= j3 <= 180.0 - margin and
            margin <= j4 <= min(180.0 - margin, max_j4))


def ik(x, y, z, j5_deg=90.0, max_j2=MAX_J2_DEG, max_j4=MAX_J4_DEG, tilt_deg=None):
    """Servo angles [J1..J5] reaching (x, y, z); raises NoSolution if none.

    tilt_deg=None chooses the most vertical feasible tilt; a number forces it.
    """
    x, y, z = float(x), float(y), float(z)
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise NoSolution("target is not finite")
    tip = _add(_rz(GRIP_O, math.radians(float(j5_deg) - 90.0)), J5_O)
    ox, l3 = tip[0], tip[2]
    lateral = J2_O[1] + J3_O[1] + J4_O[1] + tip[1]
    r2 = x * x + y * y - lateral * lateral
    if r2 <= 1e-8:
        raise NoSolution("target is on the base axis")
    px = -math.sqrt(r2)
    j1 = math.degrees(_wrap(math.atan2(y, x) - math.atan2(lateral, px))) + 90.0
    j1 = _reading_j1(j1)
    if not 0.0 <= j1 <= 180.0:
        raise NoSolution(f"J1 {j1:.1f} deg outside 0-180 (target on the +X side)")
    if tilt_deg is None:
        steps = int(round((TILT_MAX_DEG - TILT_MIN_DEG) / TILT_STEP_DEG)) + 1
        tilts = sorted((TILT_MIN_DEG + i * TILT_STEP_DEG for i in range(steps)), key=abs)
    else:
        tilts = [float(tilt_deg)]
    for margin, bend in ((LIMIT_MARGIN_DEG, MIN_ELBOW_BEND_DEG),
                         (LIMIT_MARGIN_DEG, 0.0), (0.0, 0.0)):
        for tilt in tilts:
            for j234 in _planar(px, z, tilt, ox, l3):
                if _within(j234, margin, bend, max_j2, max_j4):
                    return [j1, *j234, float(j5_deg)]
    raise NoSolution(f"no joint solution at ({x:.4f}, {y:.4f}, {z:.4f})")


def tilt_of(servo_deg):
    """Gripper tilt from straight-down (deg) for a joint vector."""
    return float(servo_deg[1]) + float(servo_deg[2]) + float(servo_deg[3]) - 270.0 + 180.0


load_j1_correction()
