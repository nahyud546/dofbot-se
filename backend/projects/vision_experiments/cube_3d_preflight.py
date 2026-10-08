#!/usr/bin/env python3
"""Read-only IK/FK check for a selected 3D grasp; never opens the serial port."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
from t8_motion_worker import (Kinematics, PLACE_GRIP_CAL_J1, PLACE_GRIP_EY,
                              PLACE_GRIP_EX, PLACE_GRIP_SIGN, valid_ik)  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cube_search_center_math import check_workspace  # noqa: E402


MIN_REACH_M = 0.10      # closer than this the gripper would hit the base column
GRIP_COMPENSATED_SOURCES = ("fixed_ready_pose", "handeye_tag_plane")


def check_target(target, kin, placement_correction_gripper_xy_m=None,
                 handeye_correction_gripper_xy_m=None):
    tcp = target["tcp_position_base"]
    if not isinstance(tcp, list) or len(tcp) != 3:
        raise ValueError("TCP must contain three base-frame coordinates")
    x, y, z = (float(value) for value in tcp)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("TCP coordinates must be finite")
    # 190 mm includes the hover above the fourth cube of a 4-high tower
    # (place TCP 167 mm + 15 mm hover). IK/FK still has to close within 5 mm.
    # Polar work area (same gate as search-center): reach 0.10-0.30 m from the base and
    # within 70 degrees of the work direction, so side targets (J1 20-160) are allowed.
    # The old rectangle (x -0.30..-0.10, |y|<=0.12) only covered the READY_POSE view.
    try:
        check_workspace(x, y)
        in_area = math.hypot(x, y) >= MIN_REACH_M
    except ValueError:
        in_area = False
    if not (in_area and 0.025 <= z <= 0.19):
        raise ValueError("TCP outside the configured work area")
    yaw = float(target["preferred_yaw_rad"])
    if not math.isfinite(yaw) or abs(yaw) > math.pi:
        raise ValueError("gripper yaw invalid")
    placement_offset = [0.0, 0.0]
    if (target.get("stage") in {"place", "hover_place"} and
            target.get("coordinate_source") in GRIP_COMPENSATED_SOURCES):
        # The held cube sits off the TCP in the gripper frame. The fixed-map
        # value (12 mm) was measured with the legacy pick bias; hand-eye picks
        # have their own, separately tuned value (default 0: not yet measured).
        # Estimate the gripper frame at the same XY on the table, where this
        # fixed READY_POSE workspace has a solution even for elevated stacks.
        reference_z = min(max(z, 0.047), 0.077)
        reference = kin.ik(x, y, reference_z)
        dj = math.radians((reference[0] - PLACE_GRIP_CAL_J1) * PLACE_GRIP_SIGN)
        if target.get("coordinate_source") == "handeye_tag_plane":
            correction = handeye_correction_gripper_xy_m or (0.0, 0.0)
        else:
            correction = (placement_correction_gripper_xy_m or
                          (-PLACE_GRIP_EX, -PLACE_GRIP_EY))
        cx, cy = (float(value) for value in correction)
        if not all(math.isfinite(value) and abs(value) <= .020 for value in (cx, cy)):
            raise ValueError("placement correction must be finite and within ±20 mm")
        dx = cx * math.cos(dj) - cy * math.sin(dj)
        dy = cx * math.sin(dj) + cy * math.cos(dj)
        x, y = x + dx, y + dy
        placement_offset = [dx, dy]
    joints = kin.ik(x, y, z)
    yaw_deg = (math.degrees(yaw) + 180) % 360 - 180
    while yaw_deg > 45:
        yaw_deg -= 90
    while yaw_deg < -45:
        yaw_deg += 90
    joints[4] = min(270, max(0, joints[0] - yaw_deg))
    if not valid_ik(joints):
        raise ValueError("IK wrist angle outside servo limits")
    fk_xyz = [float(value) for value in kin.fk(joints)]
    fk_error_mm = math.dist((x, y, z), fk_xyz) * 1000
    if fk_error_mm > 5.0:
        raise ValueError(f"IK/FK closure {fk_error_mm:.1f}mm exceeds 5mm")
    return {"ok": True, "tcp_position_base": [x, y, z], "ik_joints_deg": joints,
            "fk_position_base": fk_xyz, "ik_fk_error_mm": round(fk_error_mm, 2),
            "placement_offset_base_xy": placement_offset}


def main():
    try:
        target = json.load(sys.stdin)
        kin = Kinematics()
        try:
            result = check_target(target, kin)
        finally:
            kin.close()
    except Exception as exc:
        result = {"ok": False, "reason": str(exc)}
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
