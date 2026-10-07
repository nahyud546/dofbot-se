"""Trajectory gap cube THAT vao zone SIM (0.08,-0.16) - CHE DO DRY-RUN.

Chien luoc chot (snapshot + open-loop, zone sim, calibrate truoc):
  Phase 0 - snapshot: doc /vision/cubes (PoseArray, frame base_link), trung binh
      N mau, loai outlier; abort neu mau < 3 hoac std > 10mm.
  Phase 1 - scene THAT: dung collision ban (mat z=TABLE_Z) + plate zone
      (0.08,-0.16) + cube tai pose snapshot (thay ground_z=0 cua sim).
  Phase 2 - IK x4 (hover cube / grasp / hover zone / place) seed atan2 nhu sim,
      heights tinh tu TABLE_Z + half + tcp_offset.
  Phase 3 - DRY-RUN: plan moi waypoint (khong execute), verify limits
      +-1.57 / max step 0.6, luu JSON tuong thich safety_gate --traj.

  Mac dinh DRY-RUN (plan-only). Co --execute nhung TU CHOI (exit 2) cho den khi
  Phase 0 calibrate xong (tcp_offset_calibrated=true) + gate PASS.
  Tu chan dofbot_driver nhu sim_red_run (node nay chi plan tren sim stack).

  Can sim_red_demo.launch.py dang mo (move_group + /compute_ik + tracker).

  Chay: ros2 run cap_vision real_red_run [--tcp-offset 0.039] [--grip-close 1.0]
"""

import argparse
import json
import math
import sys
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pymoveit2 import MoveIt2

from cap_vision.sim_red_run import (
    ARM_JOINTS, EE_LINK, BASE_LINK, FINGER_LINKS, HOME,
    current_tcp_quat, current_arm_joints, query_ik, call_apply, world_box, step,
)

TABLE_Z = 0.045       # table_zones.yaml (teach-TCP mean 0.0452)
CUBE_SIZE = 0.030
CUBE_HALF = CUBE_SIZE / 2.0
ZONE_XY = (0.080, -0.160)   # zone SIM (user chot, khong dung red that 0.109,-0.168)
ZONE_SIZE = 0.080
CUBE_ID = "real_red_cube"
ZONE_ID = "real_red_zone"
# Ban that: 4 manh chua lo quanh chan robot (x +-0.05, y +-0.06).
# Box don 0.5x0.5 nuot ca base -> start state in-collision -> moi plan rot.
TABLE_BOXES = [
    ("real_table_f", (0.20, 0.0, TABLE_Z / 2.0), (0.30, 0.50, TABLE_Z)),
    ("real_table_b", (-0.10, 0.0, TABLE_Z / 2.0), (0.10, 0.50, TABLE_Z)),
    ("real_table_l", (0.0, 0.155, TABLE_Z / 2.0), (0.10, 0.19, TABLE_Z)),
    ("real_table_r", (0.0, -0.155, TABLE_Z / 2.0), (0.10, 0.19, TABLE_Z)),
]
HOVER_Z = 0.120
GRIP_OPEN = 0.0
SLOW = 0.25
OUT_PATH = "/tmp/real_red_traj.json"
SNAP_MIN = 3
SNAP_STD_MAX = 0.010
VERIFY_DRIFT_MAX = 0.010


def snapshot_cube(node, timeout_sec=6.0):
    """Trung binh pose cube tu /vision/cubes. Tra ve (x, y) hoac None."""
    samples = []

    def on_msg(msg):
        for p in msg.poses:
            samples.append((float(p.position.x), float(p.position.y)))

    sub = node.create_subscription(PoseArray, "/vision/cubes", on_msg, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if len(samples) < SNAP_MIN:
        return None, f"mau={len(samples)}<{SNAP_MIN} (mo lenh 1 de tracker chay)"
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    # loai outlier > 30mm roi trung binh lai
    kept = [(x, y) for x, y in samples if math.hypot(x - mx, y - my) < 0.030]
    if len(kept) < SNAP_MIN:
        return None, "outlier qua nhieu"
    mx = sum(x for x, _ in kept) / len(kept)
    my = sum(y for _, y in kept) / len(kept)
    std = max(
        (sum((x - mx) ** 2 for x, _ in kept) / len(kept)) ** 0.5,
        (sum((y - my) ** 2 for _, y in kept) / len(kept)) ** 0.5,
    )
    if std > SNAP_STD_MAX:
        return None, f"std={std * 1000:.1f}mm>10mm (cube dang bi dich chuyen?)"
    return (mx, my), f"n={len(kept)} std={std * 1000:.1f}mm"


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tcp-offset", type=float, default=0.039,
                    help="tcp_to_contact_offset_z (SIM tam; Phase 0 do lai)")
    ap.add_argument("--grip-close", type=float, default=1.0,
                    help="goc kep Rlink1 (SIM; Phase 0 cal gripper roi thay)")
    ap.add_argument("--execute", action="store_true",
                    help="MO KHOA cho den khi calibrate + gate PASS")
    ns, _ = ap.parse_known_args()
    rclpy.init(args=args)
    node = Node("real_red_run")
    rc = 1
    ex = None
    th = None
    try:
        if "dofbot_driver" in node.get_node_names():
            print("[FAIL] Phat hien dofbot_driver (tay that) -> tu choi (chi plan).",
                  flush=True)
            return 2
        if ns.execute:
            print("[FAIL] --execute KHOA: can Phase 0 (tcp_offset_calibrated=true)"
                  " + safety_gate --allow-hw PASS truoc. Dry-run only.", flush=True)
            return 2

        grasp_z = TABLE_Z + CUBE_HALF + ns.tcp_offset
        place_z = grasp_z + 0.002
        print(f"== Heights that: table={TABLE_Z:.3f} grasp_tcp={grasp_z:.3f} "
              f"place_tcp={place_z:.3f} (tcp_offset={ns.tcp_offset:.3f} TAM - "
              "Phase 0 do lai) ==", flush=True)

        print("== Phase 0: snapshot cube that ==", flush=True)
        snap, detail = snapshot_cube(node)
        ok = step("snapshot /vision/cubes", snap is not None, detail
                  + (f" xy=({snap[0] * 1000:.1f},{snap[1] * 1000:.1f})mm"
                     if snap else ""))
        if snap is None:
            print("KET LUAN: khong thay cube -> dung.", flush=True)
            return 1
        cube_xy = snap

        print("== Phase 1: scene that (ban + zone + cube snapshot) ==", flush=True)
        cb = ReentrantCallbackGroup()
        apply_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        t0 = time.monotonic()
        while not apply_cli.service_is_ready() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not apply_cli.service_is_ready():
            print("[FAIL] /apply_planning_scene chua san sang.", flush=True)
            return 1
        ex = MultiThreadedExecutor(2)
        ex.add_node(node)
        th = Thread(target=ex.spin, daemon=True)
        th.start()
        tables = [world_box(oid, xyz, size) for oid, xyz, size in TABLE_BOXES]
        zone = world_box(ZONE_ID, (ZONE_XY[0], ZONE_XY[1], TABLE_Z + 0.001),
                         (ZONE_SIZE, ZONE_SIZE, 0.002))
        cube = world_box(CUBE_ID, (cube_xy[0], cube_xy[1], TABLE_Z + CUBE_HALF),
                         [CUBE_SIZE] * 3)
        ok = step("scene ban(4 manh)+zone+cube", call_apply(
            node, apply_cli, world_objects=tables + [zone, cube])) and ok
        if not ok:
            return 1

        print("== Phase 2: IK that (seed atan2) ==", flush=True)
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
        quat = current_tcp_quat(node, tf_buffer)
        if quat is None:
            print("[FAIL] Khong doc duoc TF TCP.", flush=True)
            return 1
        seed = current_arm_joints(node)
        if seed is None:
            print("[FAIL] Khong doc duoc /joint_states.", flush=True)
            return 1
        ik_cli = node.create_client(GetPositionIK, "/compute_ik")
        if not ik_cli.wait_for_service(timeout_sec=10.0):
            print("[FAIL] /compute_ik chua san sang.", flush=True)
            return 1
        hover_cube = (cube_xy[0], cube_xy[1], HOVER_Z)
        grasp_xyz = (cube_xy[0], cube_xy[1], grasp_z)
        hover_zone = (ZONE_XY[0], ZONE_XY[1], HOVER_Z)
        place_xyz = (ZONE_XY[0], ZONE_XY[1], place_z)
        sols = {}

        def ask_ik(label, xyz):
            sol = query_ik(node, ik_cli, xyz, quat, seed)
            detail = (f"target=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"
                      + (f" joints={[round(v, 3) for v in sol]}" if sol else ""))
            return sol, step(f"IK {label}", sol is not None, detail)

        sol, good = ask_ik("hover cube", hover_cube)
        ok = good and ok
        if sol is not None:
            sols["hover cube"] = sol
            seed = dict(zip(ARM_JOINTS, sol))
        def remove_warn(label, obj_id):
            # REMOVE doi luc tra success=false do planning scene ban
            # (thu tay van True). Retry 3 lan; that bai -> warn-only vi IK
            # da PASS ngay ca khi object con trong scene.
            for _ in range(3):
                if call_apply(node, apply_cli, world_objects=[world_box(
                        obj_id, (0, 0, 0), (0, 0, 0),
                        operation=CollisionObject.REMOVE)]):
                    return step(label, True)
                time.sleep(1.0)
            print(f"WARN: {label} that bai sau retry - van tiep tuc "
                  "(IK da kha thi).", flush=True)
            return True

        # go cube khoi scene truoc khi IK ha (pattern sim_red_run)
        remove_warn("go cube khoi scene (IK ha)", CUBE_ID)
        sol, good = ask_ik("grasp", grasp_xyz)
        ok = good and ok
        if sol is not None:
            sols["grasp"] = sol
            seed = dict(zip(ARM_JOINTS, sol))
        sol, good = ask_ik("hover zone", hover_zone)
        ok = good and ok
        if sol is not None:
            sols["hover zone"] = sol
            seed = dict(zip(ARM_JOINTS, sol))
        # go zone khoi scene truoc khi IK dat
        remove_warn("go zone khoi scene (IK dat)", ZONE_ID)
        sol, good = ask_ik("place", place_xyz)
        ok = good and ok
        if sol is not None:
            sols["place"] = sol
            seed = dict(zip(ARM_JOINTS, sol))
        # tra scene ve de RViz thay (dry-run, khong anh huong plan sau)
        call_apply(node, apply_cli, world_objects=tables + [zone, cube])
        if not ok or len(sols) != 4:
            print("KET LUAN: IK that bai -> dung.", flush=True)
            return 1

        print("== Verify lai truoc ha (snapshot 2) ==", flush=True)
        snap2, detail2 = snapshot_cube(node, timeout_sec=3.0)
        drift = math.hypot(snap2[0] - cube_xy[0], snap2[1] - cube_xy[1]) \
            if snap2 else 999.0
        ok = step("cube dung yen", drift <= VERIFY_DRIFT_MAX,
                  detail2 + f" drift={drift * 1000:.1f}mm") and ok
        if not ok:
            print("KET LUAN: cube bi dich (>10mm) -> snapshot lai, khong plan.",
                  flush=True)
            return 1

        print("== Phase 3: DRY-RUN plan (khong execute) ==", flush=True)
        arm = MoveIt2(node=node, joint_names=list(ARM_JOINTS),
                      base_link_name=BASE_LINK, end_effector_name=EE_LINK,
                      group_name="arm_group", callback_group=cb)
        arm.max_velocity = SLOW
        arm.max_acceleration = SLOW
        arm.allowed_planning_time = 5.0
        grip = MoveIt2(node=node, joint_names=["Rlink1_Joint"],
                       base_link_name=BASE_LINK, end_effector_name=EE_LINK,
                       group_name="grip_group", callback_group=cb)
        node.create_rate(1.0).sleep()

        seq = [("mo gripper", ("grip", GRIP_OPEN)),
               ("hover cube", ("arm", sols["hover cube"])),
               ("ha gap", ("arm", sols["grasp"])),
               ("kep", ("grip", ns.grip_close)),
               ("nang len", ("arm", sols["hover cube"])),
               ("sang zone", ("arm", sols["hover zone"])),
               ("ha dat", ("arm", sols["place"])),
               ("mo (tha)", ("grip", GRIP_OPEN)),
               ("rut len", ("arm", sols["hover zone"])),
               ("ve HOME", ("arm", HOME))]
        waypoints = []  # joint positions arm theo thu tu
        for label, (kind, target) in seq:
            mover = grip if kind == "grip" else arm
            traj = mover.plan(joint_positions=[float(v) for v in
                                               ([target] if kind == "grip"
                                                else target)])
            if traj is None:
                ok = step(f"plan {label}", False, "(plan that bai)") and ok
                continue
            detail = (f"angle={target}" if kind == "grip"
                      else f"joints={[round(v, 3) for v in target]}")
            ok = step(f"plan {label} (dry-run, chua execute)", True, detail) and ok
            if kind == "arm":
                waypoints.append([float(v) for v in target])

        # verify gioi han + max step kieu safety_gate
        lim_ok = all(-1.57 - 1e-6 <= v <= 1.57 + 1e-6
                     for wp in waypoints for v in wp)
        ok = step("limits +-1.57", lim_ok, f"{len(waypoints)} waypoints") and ok
        max_step = max(math.dist(a, b) for a, b in
                       zip(waypoints, waypoints[1:])) if len(waypoints) > 1 else 0.0
        ok = step("max step <= 0.6", max_step <= 0.6,
                  f"max={max_step:.3f}rad") and ok

        # luu JSON tuong thich safety_gate --traj
        t = 0.0
        pts = []
        prev = None
        for wp in waypoints:
            d = math.dist(prev, wp) if prev else 0.5
            t += max(d / (2.0 * SLOW), 0.5)
            pts.append({"positions": wp, "time_from_start": round(t, 3)})
            prev = wp
        with open(OUT_PATH, "w", encoding="utf-8") as f:
            json.dump({"joint_names": list(ARM_JOINTS), "points": pts,
                       "meta": {"cube_xy_mm": [round(cube_xy[0] * 1000, 1),
                                               round(cube_xy[1] * 1000, 1)],
                                "zone_xy": list(ZONE_XY),
                                "grasp_tcp_z": round(grasp_z, 4),
                                "tcp_offset": ns.tcp_offset,
                                "grip_close": ns.grip_close,
                                "dry_run": True}}, f, indent=1)
        ok = step(f"luu {OUT_PATH}", True,
                  f"{len(pts)}pts (gate: --traj {OUT_PATH} --q0 ... )") and ok
        print("KET LUAN: " + ("DRY-RUN XONG - trajectory hop le, cho Phase 0 "
                               "calibrate + gate PASS roi moi --execute" if ok
                               else "DRY-RUN LOI - xem FAIL o tren"),
              flush=True)
        rc = 0 if ok else 1
        return rc
    except KeyboardInterrupt:
        print("Huy boi nguoi dung (Ctrl+C).", flush=True)
        return 130
    finally:
        try:
            if ex is not None:
                ex.shutdown()
        except Exception:
            pass
        try:
            if th is not None:
                th.join(timeout=5.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(rc)


if __name__ == "__main__":
    main()
