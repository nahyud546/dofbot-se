"""Live independent validation of mapping (checklist point 6). OBSERVE-ONLY, no motion.

Yeu cau config da calibrated (camera + extrinsic + table) neu khong se tu choi.
Doi voi cube that, khong dung du lieu fit.

3 che do (doc ky huong dan tren man hinh khi chay):
  references:   dat cube lan luot tai N diem reference (do thuoc) -> XY error
                tung diem, mean, max (target <= 5-10mm).
  consistency:  giu cube dung yen, doi tu the camera (teach tay) K lan ->
                do phan tan XYZ cung 1 cube qua nhieu goc nhin.
  drift:        giu cube + tay dung yen T giay -> do troi.

    ros2 run cap_vision validate_mapping -- --config <red_scene_cal_ts.yaml> \\
        --mode references --references refs.yaml --output report_v1.yaml
"""

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class Watcher(Node):
    def __init__(self, cfg):
        super().__init__("validate_mapping")
        from cap_scene_interfaces.msg import SceneObjects
        t = cfg["task"]
        self.want = (t["object_kind"], t["object_color"])
        p = cfg["perception"]
        self.frames = int(p.get("stable_frames", 5))
        self.radius = float(p.get("stable_radius_m", 0.005))
        self.latest = []
        self.create_subscription(SceneObjects, "/red_scene/objects",
                                 self.on_msg, qos_profile_sensor_data)

    def on_msg(self, msg):
        if not msg.calibrated or msg.header.frame_id != "base_link":
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for o in msg.objects:
            if ((o.kind, o.color) != self.want or not o.valid
                    or o.header.frame_id != "base_link"):
                continue
            stamp = o.header.stamp.sec + o.header.stamp.nanosec * 1e-9
            if 0 <= now - stamp <= 1.0:
                self.latest.append((stamp, o.pose.position.x,
                                    o.pose.position.y, o.pose.position.z))

    def stable_pose(self, timeout=30.0):
        """Doi cube on dinh: N mau lien tiep trong ban kinh. Tra ve (x,y,z mean)."""
        self.latest.clear()
        t0 = time.monotonic()
        window = []
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            for stamp, x, y, z in self.latest:
                window.append((x, y, z))
            self.latest.clear()
            window = window[-self.frames:]
            if len(window) == self.frames:
                mx = sum(p[0] for p in window) / self.frames
                my = sum(p[1] for p in window) / self.frames
                mz = sum(p[2] for p in window) / self.frames
                if max(math.dist((mx, my, mz), p) for p in window) <= self.radius:
                    return mx, my, mz
        raise RuntimeError("timeout: khong thay cube on dinh (kiem tra perception/RViz)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--mode", required=True, choices=["references", "consistency", "drift"])
    ap.add_argument("--references", default=None, help="YAML [{id,x,y}] cho mode references")
    ap.add_argument("--rounds", type=int, default=5, help="so lan capture (consistency/drift)")
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    if Path(a.output).exists():
        raise SystemExit("output exists; choose a new report file")
    cfg = load(a.config)
    for flag in ("calibrated", "extrinsic_calibrated"):
        if not cfg["camera"].get(flag):
            raise SystemExit(f"camera.{flag}=false -> tu choi validation")
    if not cfg["table"].get("calibrated"):
        raise SystemExit("table.calibrated=false -> tu choi validation")

    rclpy.init()
    node = Watcher(cfg)
    try:
        report = {"mode": a.mode, "config": str(Path(a.config).resolve()), "points": []}
        if a.mode == "references":
            refs = load(a.references)
            for r in refs:
                input(f"Dat cube tai '{r['id']}' ({r['x']*1000:.0f},{r['y']*1000:.0f})mm roi ENTER... ")
                x, y, z = node.stable_pose()
                err = math.dist((x, y), (r["x"], r["y"]))
                report["points"].append({"id": r["id"], "ref": [r["x"], r["y"]],
                                         "meas": [round(x, 4), round(y, 4), round(z, 4)],
                                         "err_m": round(err, 4)})
                print(f"  {r['id']}: err={err*1000:.1f}mm", flush=True)
            errs = [p["err_m"] for p in report["points"]]
            report["mean_m"] = round(sum(errs) / len(errs), 4)
            report["max_m"] = round(max(errs), 4)
            print(f"MEAN={report['mean_m']*1000:.1f}mm MAX={report['max_m']*1000:.1f}mm "
                  f"(target <= 5-10mm)", flush=True)
        elif a.mode == "consistency":
            for k in range(a.rounds):
                input(f"Di chuyen tay sang goc nhin {k+1}/{a.rounds} (giu cube yen) roi ENTER... ")
                x, y, z = node.stable_pose()
                report["points"].append({"view": k + 1, "meas": [round(x, 4), round(y, 4), round(z, 4)]})
                print(f"  view{k+1}: ({x*1000:.1f},{y*1000:.1f},{z*1000:.1f})mm", flush=True)
            pts = [p["meas"] for p in report["points"]]
            spread = max(math.dist(p, q) for p in pts for q in pts)
            report["spread_m"] = round(spread, 4)
            print(f"SPREAD={spread*1000:.1f}mm (cung cube, nhieu goc nhin - cang nho cang tot)",
                  flush=True)
        else:
            input("Giu cube + tay DUNG YEN roi ENTER de bat dau do troi... ")
            samples = [node.stable_pose(timeout=60.0) for _ in range(a.rounds)]
            report["points"] = [{"meas": [round(x, 4), round(y, 4), round(z, 4)]}
                                 for x, y, z in samples]
            mx = sum(s[0] for s in samples) / len(samples)
            my = sum(s[1] for s in samples) / len(samples)
            drift = max(math.dist((mx, my), (x, y)) for x, y, _ in samples)
            report["drift_m"] = round(drift, 4)
            print(f"DRIFT={drift*1000:.1f}mm (cube yen, tay yen)", flush=True)
        with open(a.output, "x", encoding="utf-8") as f:
            yaml.safe_dump(report, f, sort_keys=False)
        print(f"Saved {a.output}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
