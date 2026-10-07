"""1 lenh chay + check collision sim (SIM-ONLY, khong dieu khien tay).

Kiem tra trong 1 lan chay:
  1. /vision/sim_red_markers co cube (0.160,0,0.015,s=0.030) + zone (0.080,-0.160,0.001,s=0.080) frame base_link (nam tren san z=0)
  2. /get_planning_scene co 2 collision sim_red_cube / sim_red_zone dung pose/size/frame
In PASS/FAIL tung muc + lech mm. Exit 0 neu dat, 1 neu loi.

Chay sau lenh 1 (sim_red_demo.launch.py dang mo).
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from visualization_msgs.msg import MarkerArray

EXP_CUBE = (0.160, 0.000, 0.015, 0.030)
EXP_ZONE = (0.080, -0.160, 0.001, 0.080)
EXP_ZONE_COL = (0.080, -0.160, 0.005)
TOL = 0.002


def close(a, b, tol=TOL):
    return abs(float(a) - float(b)) <= tol


def run_checks(node):
    """Chay toan bo check marker + planning scene. Tra ve True neu dat."""
    ok_all = True

    def check(name, cond, detail=""):
        nonlocal ok_all
        print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}", flush=True)
        if not cond:
            ok_all = False

    # ---- 1. markers ----
    got = {}

    def on_markers(msg):
        for mk in msg.markers:
            if getattr(mk, "ns", "") == "sim_red":
                got[int(mk.id)] = mk

    sub = node.create_subscription(MarkerArray, "/vision/sim_red_markers", on_markers, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0 and len(got) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    check("markers topic /vision/sim_red_markers co du lieu", len(got) >= 2, f"(nhan {len(got)} markers ns=sim_red)")
    cube = got.get(0)
    zone = got.get(1)
    if cube is not None:
        p, s = cube.pose.position, cube.scale
        check("marker frame base_link", cube.header.frame_id == "base_link", f"(frame={cube.header.frame_id})")
        check("cube pose (0.160,0,0.015) nam san", close(p.x, EXP_CUBE[0]) and close(p.y, EXP_CUBE[1]) and close(p.z, EXP_CUBE[2]),
              f"(thuc=({p.x:.3f},{p.y:.3f},{p.z:.3f}) lech=({abs(p.x-EXP_CUBE[0])*1000:.1f},{abs(p.y-EXP_CUBE[1])*1000:.1f},{abs(p.z-EXP_CUBE[2])*1000:.1f})mm)")
        check("cube size 30mm", close(s.x, 0.030) and close(s.y, 0.030) and close(s.z, 0.030),
              f"(thuc=({s.x*1000:.1f},{s.y*1000:.1f},{s.z*1000:.1f})mm)")
    else:
        check("cube marker id=0 ton tai", False)
    if zone is not None:
        p, s = zone.pose.position, zone.scale
        check("zone pose (0.080,-0.160,0.001) nam san", close(p.x, EXP_ZONE[0]) and close(p.y, EXP_ZONE[1]) and close(p.z, EXP_ZONE[2]),
              f"(thuc=({p.x:.3f},{p.y:.3f},{p.z:.3f}))")
        check("zone size 80x80x2mm", close(s.x, 0.080) and close(s.y, 0.080) and close(s.z, 0.002),
              f"(thuc=({s.x*1000:.1f},{s.y*1000:.1f},{s.z*1000:.1f})mm)")
    else:
        check("zone marker id=1 ton tai", False)
    node.destroy_subscription(sub)

    # ---- 2. planning scene ----
    cli = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        check("service /get_planning_scene san sang", False)
    else:
        req = GetPlanningScene.Request()
        req.components.components = 1023
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not fut.done():
            check("get_planning_scene tra loi", False)
        else:
            objs = {o.id: o for o in fut.result().scene.world.collision_objects}
            for oid, exp, esz in (("sim_red_cube", EXP_CUBE[:3], (0.030, 0.030, 0.030)),
                                  ("sim_red_zone", EXP_ZONE_COL, (0.080, 0.080, 0.010))):
                o = objs.get(oid)
                check(f"collision {oid} ton tai", o is not None, f"(scene co {len(objs)} objects)")
                if o is None:
                    continue
                # MoveIt tra frame o header (collision_object.header)
                check(f"{oid} frame base_link", (o.header.frame_id or "base_link") == "base_link",
                      f"(frame={o.header.frame_id})")
                if not o.primitives or not o.primitive_poses:
                    check(f"{oid} co geometry", False, "(thieu primitives)")
                    continue
                # MoveIt chuan hoa: obj.pose = world, primitive_poses = tuong doi (0,0,0).
                # Tam world = compose 2 pose (giong chess _collision_object_center).
                pp0 = o.primitive_poses[0].position
                wx, wy, wz = (o.pose.position.x + pp0.x, o.pose.position.y + pp0.y, o.pose.position.z + pp0.z)
                d = tuple(float(v) for v in o.primitives[0].dimensions)
                check(f"{oid} pose", close(wx, exp[0]) and close(wy, exp[1]) and close(wz, exp[2]),
                      f"(thuc=({wx:.3f},{wy:.3f},{wz:.3f}))")
                ok_sz = len(d) == 3 and all(close(a, b) for a, b in zip(d, esz))
                check(f"{oid} size", ok_sz,
                      f"(thuc=({','.join(f'{v*1000:.1f}mm' for v in d)}))")

    print("KET LUAN: " + ("AT - collision + visual dung, tiep tuc check duong plan trong RViz" if ok_all else "CHUA DAT - xem dong FAIL o tren"), flush=True)
    return ok_all


def main(args=None):
    rclpy.init(args=args)
    node = Node("sim_red_check")
    ok = run_checks(node)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
