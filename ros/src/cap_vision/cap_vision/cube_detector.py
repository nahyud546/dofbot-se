"""B3: detect 4 cube 30mm (vang/xanh la/xanh bien/do) bang HSV.

Subscribe: image_topic (/cap_vision/image_raw 640x480 bgr8)
Publish:   annotated_topic (/vision/annotated) + detections_topic (/vision/detections_2d, std_msgs/String JSON)
JSON: [{label,u,v,area,w,h}]  (u,v pixel, area px)

Gate area o pose Z120: cube 30mm ~213x187px ~39k px -> loc 15k-60k, aspect 0.75-1.3.
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# HSV ranges (tune 2026-09-22 tu /tmp/robot_cam.png that, 640x480):
#   yellow mean (25,69,247) -> S ha xuong 50 (anh sang manh)
#   green mean (97,168,102) -> xanh dam H~97, V thap (khac blue H~108 V~254)
#   blue mean (108,249,254) -> hep H tu 104 de tach green
#   red mean (172,179,214) -> mo rong H tu 155 (live frame trôi ve 160).
# Live frame /tmp/live_now.png cho thay WB troi manh (yellow H 25->46, S->15)
# nen ngoai absolute con fallback adaptive 2x2 o detect_cubes_adaptive.
RANGES = {
    # vang
    "yellow": [((15, 40, 80), (50, 255, 255))],
    # xanh la dam (teal)
    "green": [((83, 80, 50), (104, 255, 180))],
    # xanh bien
    "blue": [((104, 80, 60), (125, 255, 255))],
    # do 2 dai H (mo rong de chiu WB drift; V ha 60->30 vi anh thuc te toi 2026-09-22,
    # cube do V~38: nen S>=90 + H hep giu do dac hieu, ha V khong bat nen toi)
    "red": [((0, 90, 30), (8, 255, 255)), ((155, 90, 30), (180, 255, 255))],
}
# H tham chieu de fallback adaptive (cap nhat chay theo WB drift).
REF_H = {"yellow": 25.0, "green": 97.0, "blue": 108.0, "red": 172.0}
DRAW = {"yellow": (0, 255, 255), "green": (0, 200, 0),
        "blue": (255, 0, 0), "red": (0, 0, 255)}


def detect_cubes(bgr, min_area=15000.0, max_area=60000.0):
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    dets = []
    for label, ranges in RANGES.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = float(cv2.contourArea(c))
            if not min_area <= area <= max_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < 1 or h < 1:
                continue
            aspect = max(w, h) / max(min(w, h), 1e-6)
            if not 0.70 <= aspect <= 1.45:
                continue
            rect_area = float(w * h)
            solidity = area / max(rect_area, 1e-6)
            if solidity < 0.70:  # loai vung le, bong
                continue
            m = cv2.moments(c)
            if m["m00"] < 1e-6:
                continue
            dets.append({"label": label, "u": float(m["m10"] / m["m00"]),
                         "v": float(m["m01"] / m["m00"]), "area": area,
                         "w": int(w), "h": int(h)})
    # moi mau giu 1 detection lon nhat (tranh double).
    best = {}
    for d in dets:
        if d["label"] not in best or d["area"] > best[d["label"]]["area"]:
            best[d["label"]] = d
    out = list(best.values())
    return out


def detect_cubes_auto(bgr, table, expected_count=4,
                      min_area=0.0, max_area=0.0):
    """Wrapper: cong area theo scale that + fallback co dieu kien.

    - exp = (cube/sx)*(cube/sy) px; gate mac dinh [0.35*exp, 3.0*exp].
    - Fallback adaptive CHI chay khi scene nhieu cube (expected>=3 VA
      absolute thay >=2 mau) - tranh bia them mau nen o scene 1 cube.
    """
    sx = float(table.get("mm_per_px_x", 0.0001884)) if table else 0.0001884
    sy = float(table.get("mm_per_px_y", 0.0001771)) if table else 0.0001771
    cube = float(table.get("cube_size", 0.030)) if table else 0.030
    exp = (cube / sx) * (cube / sy)
    lo = min_area or 0.35 * exp
    hi = max_area or 3.0 * exp
    out = detect_cubes(bgr, lo, hi)
    if expected_count >= 3 and 2 <= len(out) < expected_count:
        extra = detect_cubes_adaptive(bgr, lo, hi) or []
        have = {d["label"] for d in out}
        out = out + [d for d in extra if d["label"] not in have]
    return out, (lo, hi, exp)


def _circ_dist(a, b):
    d = abs(float(a) - float(b)) % 180.0
    return min(d, 180.0 - d)


def detect_cubes_adaptive(bgr, min_area=15000.0, max_area=100000.0):
    """Fallback: tim khe doc/ngang toi nhat tam anh -> 4 quadrant -> classify theo H."""
    h, w = bgr.shape[:2]
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # khe toi: cot/hang co V trung binh thap nhat trong 1/3 giua anh.
    vmean_col = hsv[:, w // 3:2 * w // 3, 2].mean(axis=0)
    vmean_row = hsv[h // 3:2 * h // 3, :, 2].mean(axis=1)
    gx = int(w // 3 + int(np.argmin(vmean_col)))
    gy = int(h // 3 + int(np.argmin(vmean_row)))
    quads = {"q1": (0, 0, gx, gy), "q2": (gx, 0, w, gy),
             "q3": (0, gy, gx, h), "q4": (gx, gy, w, h)}
    # mau nen trang: S thap -> loai.
    sat = hsv[:, :, 1]
    dets = []
    used = set()
    order = []
    for key, (x0, y0, x1, y1) in quads.items():
        roi_h = hsv[y0:y1, x0:x1]
        roi_s = sat[y0:y1, x0:x1]
        m = roi_s > 25
        if int(m.sum()) < 3000:
            continue
        hmean = float(roi_h[:, :, 0][m].mean())
        vmean = float(roi_h[:, :, 2][m].mean())
        # chon label co H gan nhat, chua dung, uu tien green toi (V thap).
        cands = sorted(REF_H.items(), key=lambda kv: _circ_dist(hmean, kv[1]))
        label = next((lb for lb, _ in cands if lb not in used), cands[0][0])
        if label == "green" and vmean > 200 and "blue" not in used:
            label = "blue"  # tranh nham blue sang thanh green toi
        used.add(label)
        ys, xs = np.nonzero(m)
        u, v = float(x0 + xs.mean()), float(y0 + ys.mean())
        area = float(m.sum())
        if not min_area <= area <= max_area:
            continue
        dets.append({"label": label, "u": u, "v": v, "area": area,
                     "w": int(x1 - x0), "h": int(y1 - y0)})
        order.append((label, hmean, vmean))
    # cap nhat REF_H chay theo quan sat (chong drift lau dai).
    for d, (label, hmean, vmean) in zip(dets, order):
        if d["label"] == label:
            REF_H[label] = 0.9 * REF_H[label] + 0.1 * hmean
    return dets if len(dets) >= 3 else []


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.declare_parameter("image_topic", "/cap_vision/image_raw")
        self.declare_parameter("annotated_topic", "/vision/annotated")
        self.declare_parameter("detections_topic", "/vision/detections_2d")
        self.declare_parameter("min_area", 0.0)
        self.declare_parameter("max_area", 0.0)
        self.declare_parameter("table_file", "")
        self.declare_parameter("expected_count", 4)
        p = self.get_parameters_by_prefix("")
        self.min_area = float(p["min_area"].value)
        self.max_area = float(p["max_area"].value)
        self.expected_count = int(p["expected_count"].value)
        tfile = str(p["table_file"].value)
        self.table = {}
        if tfile:
            import yaml  # noqa: PLC0415 - lazy giu import nhe khi test offline
            with open(tfile) as f:
                self.table = yaml.safe_load(f) or {}
        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, str(p["annotated_topic"].value), 10)
        self.pub_det = self.create_publisher(String, str(p["detections_topic"].value), 10)
        self.create_subscription(Image, str(p["image_topic"].value), self.on_image, 10)
        self.get_logger().info(
            f"cube_detector ready expected={self.expected_count} table={tfile or 'none'}")

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Bo frame: {exc}")
            return
        dets, (lo, hi, exp) = detect_cubes_auto(
            bgr, self.table, self.expected_count, self.min_area, self.max_area)
        annot = bgr.copy()
        for d in dets:
            c = DRAW.get(d["label"], (0, 255, 0))
            cv2.circle(annot, (int(d["u"]), int(d["v"])), 8, c, -1)
            cv2.rectangle(annot, (int(d["u"] - d["w"] / 2), int(d["v"] - d["h"] / 2)),
                          (int(d["u"] + d["w"] / 2), int(d["v"] + d["h"] / 2)), c, 2)
            cv2.putText(annot, f"{d['label']} {d['area']:.0f}px",
                        (int(d["u"]) + 10, int(d["v"])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        out = String()
        out.data = json.dumps(dets)
        self.pub_det.publish(out)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(annot, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
