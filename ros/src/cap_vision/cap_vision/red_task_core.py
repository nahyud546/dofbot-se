"""Task validation/export independent of ROS and hardware."""
import math
import numpy as np

from cap_vision.red_scene_core import calibration_error, transform

JOINTS = [f"arm{i}_Joint" for i in range(1, 6)] + ["Rlink1_Joint"]


def validate_task(cfg):
    error = calibration_error(cfg)
    if error:
        raise ValueError(error)
    t = cfg["task"]
    if not t["tcp_calibrated"] or not t["gripper_calibrated"]:
        raise ValueError("TCP and gripper calibration required")
    for key in ("tcp_contact_offset_m", "gripper_open_rad", "gripper_closed_rad"):
        if t[key] is None or not math.isfinite(t[key]):
            raise ValueError(f"missing {key}")
    q = np.asarray(t["tcp_orientation_xyzw"], float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError("configure a measured unit TCP grasp quaternion")
    if not 0 <= t["gripper_open_rad"] < t["gripper_closed_rad"] <= 1.57:
        raise ValueError("invalid gripper open/closed angles")
    if not 0 <= t["tcp_contact_offset_m"] <= 0.15 or not 0.02 <= t["approach_m"] <= 0.20:
        raise ValueError("invalid TCP offset / approach clearance")
    if not all(0 < t[k] <= 0.25 for k in ("velocity_scale", "acceleration_scale")):
        raise ValueError("velocity/acceleration scale must be in (0, 0.25]")
    if not t["environment_boxes"]:
        raise ValueError("configure measured table/environment collision boxes")
    ids = set()
    for box in t["environment_boxes"]:
        if box["id"] in ids or not box["id"] or box["id"].startswith("red_"):
            raise ValueError("environment IDs must be unique and not start with red_")
        ids.add(box["id"])
        if (len(box["center"]) != 3 or len(box["size"]) != 3
                or not np.isfinite(box["center"] + box["size"]).all() or min(box["size"]) <= 0):
            raise ValueError("invalid environment box")


def targets(cube_xyz, zone_xyz, cfg):
    normal = transform(cfg["table"]["base_T_table"])[:3, 2]
    task = cfg["task"]
    cube, zone = np.asarray(cube_xyz), np.asarray(zone_xyz)
    half = cfg["perception"]["cube_size"] / 2
    grasp = cube + normal * task["tcp_contact_offset_m"]
    place = zone + normal * (half + task["tcp_contact_offset_m"] + task["placement_clearance_m"])
    return {"hover_cube": grasp + normal * task["approach_m"], "grasp": grasp,
            "hover_zone": place + normal * task["approach_m"], "place": place}


def append_trajectory(points, trajectory, state, time_offset):
    """Keep all planned points and fill nonmoving joints from the start state."""
    names = trajectory.joint_names
    if not trajectory.points or not set(names).issubset(JOINTS):
        raise ValueError("empty trajectory or unexpected joints")
    previous = -1.0
    for pt in trajectory.points:
        sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        if sec < 0 or sec <= previous or len(pt.positions) != len(names) or not np.isfinite(pt.positions).all():
            raise ValueError("malformed trajectory")
        previous = sec
        state.update(zip(names, map(float, pt.positions)))
        at = time_offset + sec
        row = {"positions": [state[j] for j in JOINTS], "time_from_start": at}
        if points and abs(at - points[-1]["time_from_start"]) < 1e-8:
            if max(abs(a-b) for a,b in zip(row["positions"], points[-1]["positions"])) > 1e-4:
                raise ValueError("discontinuous segment boundary")
            continue
        points.append(row)
    return time_offset + previous


def placement_error(cube_xyz, zone_xyz, cfg):
    table = transform(cfg["table"]["base_T_table"])
    delta = table[:3, :3].T @ (np.asarray(cube_xyz) - zone_xyz)
    return float(np.linalg.norm(delta[:2]))
