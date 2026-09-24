#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""T6 trash-sorting inference node (dofbot_yolov11.yolov11_sortation).

Flow:  /image_raw (cam_pub) -> YOLOv11 (best.onnx, 16 garbage classes)
       -> class->bin id -> PosInfo (AprilTagInfo) -> grasp node (IK + pick/place)
       -> grasp_done=True -> ve scan-only (cho SPACE moi, khong tu dong gap).
   Che do mac dinh: SCAN-ONLY (chi quet + preview, khong publish).
   Nhan SPACE (cua so preview --show dang focus) -> publish 1 item conf
   cao nhat -> grasp thuc hien -> xong ve lai scan-only.

Upgrade 2026-09-24 (plan B1/B2):
- conf 0.65 -> CONF_THRESHOLD (0.40), them imgsz=IMG_SIZE (640, khop input onnx).
- Inference chay tren worker thread rieng: callback chi luu frame moi nhat
  (drop frame cu), khong block rclpy spin. Dem FPS callback + inference.
- Bo imshow/waitKey trong callback; preview tuy chon qua --show.
- processed_items: key luong tu pixel (nhay 1-2px khong tao item moi),
  het han TTL de vat dung yen duoc thu lai; van 1 publish/grasp-cycle.
- Device: mac dinh thu cuda:0 truoc (RTX 2050 + onnxruntime-gpu san),
  tu fallback cpu neu warmup fail. Ep bang env T6_YOLO_DEVICE=cpu|cuda:0.
- Yaw (port stacking_target.get_Sqaure): uoc luong goc xoay vat tu crop box
  (canh dai minAreaRect), publish set_joint5 (Int16 deg) TRUOC PosInfo de
  grasp xoay J5 = J1 - delta, het hut khi vat nghieng. Fallback yaw=0.
- TTA-rot90: scan-only ma lau khong thay box thi thu frame xoay 90 do.
"""
import argparse
import math
import os
import threading
import time
import warnings
from collections import deque
from pathlib import Path
from statistics import median

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int16, String

from dofbot_interface.msg import AprilTagInfo
from dofbot_interface.srv import Kinemarics
from ultralytics import YOLO

warnings.filterwarnings("ignore")

# ---------------- T6 tuning constants (plan B1) ----------------
# CONF chinh live khong can sua code: T6_CONF=0.25 ros2 run ... (mac dinh 0.30).
CONF_THRESHOLD = float(os.environ.get("T6_CONF", "0.30"))
IMG_SIZE = 960          # tang tu 640: vat nho/anh in tren cube net hon
CONF_PASS2 = 0.15       # san thap cho pass-2 crop upscale (chi chay khi SPACE)
# Temporal vote: track box qua cac frame bang IoU, SPACE publish track on
# dinh nhat (majority class + median conf/tam) thay vi frame don.
TRACK_IOU_MIN = 0.3
TRACK_MAX_AGE_S = 1.0
TRACK_HIST_LEN = 10
FORCED_DEVICE = os.environ.get("T6_YOLO_DEVICE", "").strip()  # "" = auto
# Pixel -> world (goc Yahboom, fit tai pose [90,120]).
# TODO(B3): hieu chuan lai tai pose dung that [90,125] (AprilTag tai world
# known -> ox, oy) roi dien X_OFFSET_M/Y_OFFSET_M, xoa bu cung ben grasp.
PIXEL_TO_M_Y = 1.0 / 4000.0
PIXEL_TO_M_X = 0.8 / 3000.0
X_BASE_M = 0.13
X_OFFSET_M = 0.0
Y_OFFSET_M = 0.0
# Dau truc ngang - KIEM CHUNG VAT LY 2026-09-24: cong thuc goc (320-cx) GAP
# NGUOC trai/phai (Fish_bone px=481 di -Y, Newspaper px=82 di +Y, deu nguoc
# vi tri that). Camera nay: anh +px = world -Y -> dung (cx-320).
# Neu doi camera/mount ma thay nguoc lai thi doi LAT_SIGN = -1.0.
LAT_SIGN = 1.0
PICK_Z_M = 0.03
# Item da publish giu trong processed_map TTL giay de vat dung yen duoc thu
# lai (key luong tu 10px chong nhay pixel).
PROCESSED_TTL_S = 60.0
PROCESSED_GRID_PX = 10
FRAME_WAIT_S = 0.005
FPS_LOG_S = 5.0
# ---- Yaw bam theo goc xoay vat (port tu stacking_target.get_Sqaure) ----
# Toan hoc giong het stacking: anh +px = world +Y, +py(down) = world +X ->
# canh dai minAreaRect cho alpha (anh, [0,180)) -> delta wrap ve [-45,45],
# J5_pick = J1 - delta_signed (yaw_mo_cang = J1 + 90 - J5, do FK).
# DAU YAW 2026-09-24: truc ngang bi mirror (xem LAT_SIGN) nen dx doi dau ->
# alpha' = 180-alpha -> delta' = -delta_true. Vi vay publish delta_signed =
# YAW_SIGN * delta_raw voi YAW_SIGN = -1.0 de grasp giu nguyen J5 = J1-delta.
# Neu test thay cang xoay NGUOC huong vat (sau khi da fix mirror) thi doi
# YAW_SIGN = +1.0 (1 dong, test lai 1 lan).
YAW_SIGN = -1.0
YAW_MAX_DEG = 40.0
YAW_CROP_PAD_PX = 6
# Gan vuong (dai/rong < nguong) thi yaw mo -> fallback 0 (J5=J1).
YAW_MIN_ASPECT = 1.15
# set_joint5 het han sau bao lau thi grasp coi nhu khong co yaw.
YAW_FRESH_S = 5.0
# TTA fallback: scan-only ma >N giay khong thay box thi thu them 1 lan
# inference tren frame xoay 90 (vat nghieng/xeo hay rot khoi detector).
TTA_STALE_S = 8.0
# Yaw median theo track (port stacking angle_history median-5): can it nhat
# N sample reliable, spread (max-min) qua lon -> FALLBACK nhu cube khong on dinh.
YAW_MIN_SAMPLES = 3
YAW_MAX_SPREAD_DEG = 15.0
# Quad-fit mat vuong tren cube: dien tich quad phai chiem bao nhieu crop,
# do lech goc toi da (deg) de chap nhan la mat vuong.
# MIN 0.15 (tang tu 0.08): loai khung/chu nhat NHO cua anh IN tren cube, chi
# nhan quad co mat cube (sticker phu gan kin mat -> quad to, giua crop).
QUAD_MIN_AREA_FRAC = 0.15
QUAD_MAX_AREA_FRAC = 0.85
QUAD_MAX_ANGLE_DEV = 20.0


def box_iou(a, b):
    """IoU 2 box (x1,y1,x2,y2) de track qua cac frame."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0

# ---------------- Bin mapping (plan B2) ----------------
# Tái chế - Xanh dương (Blue), id=1 -> p_1
RECYCLABLE_BLUE = ['Newspaper', 'Zip_top_can', 'Book', 'Old_school_bag']
# Ướt - Xanh lá (Green), id=2 -> p_2
WET_GREEN = ['Fish_bone', 'Egg_shell', 'Apple_core', 'Watermelon_rind']
# Độc hại - Đỏ (Red), id=3 -> p_3
HAZARDOUS_RED = ['Syringe', 'Expired_cosmetics', 'Used_batteries',
                 'Expired_tablets']
# Khô - Xám (Grey), id=4 -> p_4 (TAM dung pose Vang cu, TODO: thay sau khi jog)
DRY_GREY = ['Toilet_paper', 'Peach_pit', 'Cigarette_butts',
            'Disposable_chopsticks']
ALL_WASTE = RECYCLABLE_BLUE + HAZARDOUS_RED + WET_GREEN + DRY_GREY

# Load YOLO model with verbose=False to suppress output
model_path = Path(__file__).resolve().parent / "best.onnx"
if not model_path.is_file():
    raise FileNotFoundError(f"YOLO model not found: {model_path}")
model = YOLO(str(model_path), task='detect')


def quantize_key(name, cx, cy, grid=PROCESSED_GRID_PX):
    """Khoa item luong tu theo luoi pixel de tam nhay vai px khong tao item moi."""
    return f"{name}_{int(cx) // grid}_{int(cy) // grid}"


def prune_processed(processed, now, ttl=PROCESSED_TTL_S):
    """Xoa item het han de vat dung yen duoc thu lai."""
    expired = [k for k, t in processed.items() if now - t > ttl]
    for k in expired:
        del processed[k]
    return len(expired)


def pixel_to_world(center_x, center_y):
    """Doi tam box (px) sang (fwd_x, lat_y) theo fit tuyen tinh Yahboom."""
    lat_y = round(LAT_SIGN * (center_x - 320) * PIXEL_TO_M_Y + Y_OFFSET_M, 5)
    fwd_x = round((480 - center_y) * PIXEL_TO_M_X + X_BASE_M + X_OFFSET_M, 5)
    return fwd_x, lat_y


def _largest_usable_contour(contours, area_wh):
    # Nguong rong (3%-97%): vat mong/cheo trong box YOLO rong chi chiem vai %
    # dien tich crop (vd Fish_bone) - nguong 10% cu loai het -> FALLBACK oan.
    for area, cnt in sorted(((cv.contourArea(c), c) for c in contours),
                            reverse=True):
        if 0.03 * area_wh < area < 0.97 * area_wh:
            return cnt
    return None


def _contour_from_thresh(gray, area_wh):
    """Otsu + close, thu ca nen sang vat toi va nguoc lai."""
    for flag in (cv.THRESH_BINARY + cv.THRESH_OTSU,
                 cv.THRESH_BINARY_INV + cv.THRESH_OTSU):
        _, th = cv.threshold(gray, 0, 255, flag)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        th = cv.morphologyEx(th, cv.MORPH_CLOSE, kernel)
        found = cv.findContours(th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = found[0] if len(found) == 2 else found[1]
        cnt = _largest_usable_contour(contours, area_wh)
        if cnt is not None:
            return cnt
    return None


def _contour_from_edges(gray, area_wh):
    """Du phong cho nen van/hoa tiet (Otsu gam ca nen vao vat): Canny."""
    edges = cv.Canny(gray, 50, 150)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    edges = cv.dilate(edges, kernel, iterations=2)
    found = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]
    return _largest_usable_contour(contours, area_wh)


YAW_DEBUG = os.environ.get("T6_YAW_DEBUG", "").strip() not in ("", "0")
_YAW_DEBUG_COUNT = 0


def _yaw_debug_dump(crop_rgb, cnt, delta, tier):
    """Luu crop yaw ra /tmp/yaw_debug (toi da 20 anh) khi T6_YAW_DEBUG=1."""
    global _YAW_DEBUG_COUNT
    if not YAW_DEBUG or _YAW_DEBUG_COUNT >= 20:
        return
    try:
        d = "/tmp/yaw_debug"
        os.makedirs(d, exist_ok=True)
        vis = cv.cvtColor(crop_rgb, cv.COLOR_RGB2BGR)
        if cnt is not None:
            box = np.asarray(cnt).reshape(-1, 2).astype(int)
            cv.drawContours(vis, [box], 0, (0, 255, 0), 2)
        cv.putText(vis, f"{delta:+.1f} {tier}",
                   (5, 18), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        _YAW_DEBUG_COUNT += 1
        cv.imwrite(f"{d}/yaw_{_YAW_DEBUG_COUNT:02d}.jpg", vis)
    except Exception:
        pass


def _edge_alpha_delta(pts):
    """Huong canh DAI nhat cua polygon -> delta wrap [-45,45] (giong stacking)."""
    best_len, alpha = -1.0, 0.0
    n = len(pts)
    for i in range(n):
        dx = float(pts[(i + 1) % n][0] - pts[i][0])
        dy = float(pts[(i + 1) % n][1] - pts[i][1])
        ln = dx * dx + dy * dy
        if ln > best_len:
            best_len, alpha = ln, math.degrees(math.atan2(dy, dx)) % 180.0
    r = alpha % 90.0
    delta = -r if r <= 45.0 else 90.0 - r
    return max(-YAW_MAX_DEG, min(YAW_MAX_DEG, delta))


def estimate_yaw(crop_rgb):
    """Uoc luong goc xoay vat (delta, ok, box4) - box4: 4 dinh minAreaRect.

    Port tu stacking_target.get_Sqaure: contour lon nhat -> canh DAI
    minAreaRect (boxPoints, khong dung w/h) -> alpha anh -> delta.
    Chu y: frame tu cam_pub la RGB -> dung RGB2GRAY (ban cu dung BGR2GRAY
    tren du lieu RGB, lech trong so xam). Gan vuong / khong tach duoc
    contour sach -> (0.0, False, None).
    """
    try:
        gray = cv.cvtColor(crop_rgb, cv.COLOR_RGB2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 1)
        h, w = gray.shape
        area_wh = float(w * h)
        if area_wh <= 0:
            return 0.0, False, None
        cnt = _contour_from_thresh(gray, area_wh)
        if cnt is None:
            cnt = _contour_from_edges(gray, area_wh)
        if cnt is None:
            _yaw_debug_dump(crop_rgb, None, 0.0, "NONE")
            return 0.0, False, None
        rect = cv.minAreaRect(cnt)
        long_side = max(rect[1])
        short_side = min(rect[1])
        if short_side <= 0 or long_side / short_side < YAW_MIN_ASPECT:
            _yaw_debug_dump(crop_rgb, cnt, 0.0, "MASK_SQUARE")
            return 0.0, False, None
        box = cv.boxPoints(rect)
        delta = _edge_alpha_delta(box)
        _yaw_debug_dump(crop_rgb, cnt, delta, "MASK")
        return delta, True, np.asarray(box, dtype=float)
    except Exception:
        return 0.0, False, None


def _quad_squareness(quad):
    """Do lech goc lon nhat (deg) cua tu giac so voi 90 do; cang nho cang vuong."""
    pts = np.asarray(quad).reshape(4, 2).astype(float)
    worst = 0.0
    for i in range(4):
        v1 = pts[i] - pts[(i - 1) % 4]
        v2 = pts[(i + 1) % 4] - pts[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 <= 0 or n2 <= 0:
            return 180.0
        cosang = max(-1.0, min(1.0, float(v1 @ v2) / (n1 * n2)))
        worst = max(worst, abs(math.degrees(math.acos(cosang)) - 90.0))
    return worst


def estimate_quad_yaw(crop_rgb):
    """Fit MAT VUONG TREN cube: Canny -> tu giac loi 4 dinh gan vuong nhat.

    Vat la cube dan anh: mat tren hien nhu tu giac (goc nhin xien), chinh xac
    hon longest-edge ca mask (mask gom ca mat ben). Tra (delta, True) hoac
    (0.0, False) de roi xuong tang MASK.
    """
    try:
        gray = cv.cvtColor(crop_rgb, cv.COLOR_RGB2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 1)
        h, w = gray.shape
        area_wh = float(w * h)
        if area_wh <= 0:
            return 0.0, False, None
        edges = cv.Canny(gray, 50, 150)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        edges = cv.dilate(edges, kernel, iterations=1)
        found = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = found[0] if len(found) == 2 else found[1]
        best, best_score = None, None
        for cnt in contours:
            area = cv.contourArea(cnt)
            if not (QUAD_MIN_AREA_FRAC * area_wh < area < QUAD_MAX_AREA_FRAC * area_wh):
                continue
            peri = cv.arcLength(cnt, True)
            if peri <= 0:
                continue
            # Thu 2 epsilon: 0.02 truoc (chuan), 0.035 du phong khi vien in
            # tren cube lam contour gay khuc (approx thua -> >4 dinh).
            quad = None
            for eps in (0.02, 0.035):
                approx = cv.approxPolyDP(cnt, eps * peri, True)
                if len(approx) == 4 and cv.isContourConvex(approx):
                    quad = approx
                    break
            if quad is None:
                continue
            dev = _quad_squareness(quad)
            if dev > QUAD_MAX_ANGLE_DEV:
                continue
            # Uu tien vua to vua vuong vua GIUA crop: sticker phu mat cube nen
            # quad mat that nam giua; quad anh in lech thuong nho/lech tam.
            moments = cv.moments(quad)
            if moments["m00"] > 0:
                qcx = moments["m10"] / moments["m00"]
                qcy = moments["m01"] / moments["m00"]
            else:
                qcx, qcy = w / 2.0, h
            off = abs(qcx / w - 0.5) + abs(qcy / h - 0.5)
            score = (dev, -area + area * (0.2 * (qcy / h) + 0.5 * off))
            if best_score is None or score < best_score:
                best_score, best = score, quad.reshape(4, 2)
        if best is None:
            return 0.0, False, None
        return _edge_alpha_delta(best), True, best
    except Exception:
        return 0.0, False, None


def estimate_yaw_top(crop_rgb):
    """Yaw 3 tang: QUAD (mat vuong tren) -> MASK (longest-edge) -> FALLBACK 0.

    Tra (yaw, ok, tier, poly): poly la 4 dinh (crop coords) de ve overlay,
    None khi FALLBACK.
    """
    yaw, ok, quad = estimate_quad_yaw(crop_rgb)
    if ok:
        _yaw_debug_dump(crop_rgb, quad, yaw, "QUAD")
        return yaw, True, "QUAD", np.asarray(quad, dtype=float)
    yaw, ok, box = estimate_yaw(crop_rgb)
    if ok:
        return yaw, True, "MASK", box
    return 0.0, False, "FALLBACK", None


def yaw_sample_frame(frame, box, pad=YAW_CROP_PAD_PX):
    """1 yaw sample tu box tren frame day du -> (yaw, ok, tier, poly_frame).

    poly_frame la 4 dinh theo toa do FRAME (da offset), dung ve overlay.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    cx1, cy1 = max(0, x1 - pad), max(0, y1 - pad)
    cx2, cy2 = min(w, x2 + pad), min(h, y2 + pad)
    crop = frame[cy1:cy2, cx1:cx2]
    if crop.size == 0:
        return 0.0, False, "EMPTY", None
    yaw, ok, tier, poly = estimate_yaw_top(crop)
    if poly is not None:
        poly = poly + np.array([cx1, cy1], dtype=float)
    if not ok:
        yaw = 0.0
    return yaw, ok, tier, poly


# Mau overlay theo tier (BGR): QUAD xanh, MASK vang, FALLBACK do.
TIER_COLORS = {"QUAD": (0, 255, 0), "MASK": (0, 255, 255),
               "FALLBACK": (0, 0, 255), "EMPTY": (0, 0, 255)}


def draw_yaw_overlay(bgr, items):
    """Ve goc yaw kieu stacking len anh BGR: polygon + text '{name} {yaw}d'.

    items: list dict(box, name, yaw, ok, tier, poly, tid, votes).
    Giong stacking_target.get_Sqaure: drawContours + putText goc len hinh.
    """
    for it in items:
        x1, y1, x2, y2 = it["box"]
        color = TIER_COLORS.get(it["tier"], (0, 0, 255))
        if it["poly"] is not None:
            cv.drawContours(bgr, [np.asarray(it["poly"], dtype=int)],
                            -1, color, 2)
        label = (f"{it['name']} {it['yaw']:+.0f}d({it['tier']})"
                 f"#{it['tid']}v{it['votes']}")
        ty = max(0, y1 - 8)
        cv.putText(bgr, label, (x1, ty),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return bgr


class YoloAllGarbageDetectNode(Node):
    def __init__(self, show=False):
        super().__init__('yolo_all_garbage_detect')
        self.show = show
        self.joint5 = Int16()
        self.detect_flag = False
        self.compute_height = True
        self.index = None
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.subscription = self.create_subscription(Bool, 'grasp_done', self.GraspStatusCallback, qos_profile=1)
        # SCAN-ONLY mac dinh: khong tu publish dau turn; doi SPACE moi gap.
        self.pubPos_flag = False
        self.pr_time = time.time()
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []
        self.dist = 0.13
        self.subscription = self.create_subscription(Image, '/image_raw', self.ImageCallback, 1)
        print("All waste types initialized:", len(ALL_WASTE), "classes.")
        self.largemodel_arm_done_pub = self.create_publisher(String, '/largemodel_arm_done', 1)
        self.processed_items = {}
        self.last_publish_time = 0
        self._last_det_time = time.time()
        # Temporal tracks: moi track giu lich su vote class + box de SPACE
        # publish track on dinh nhat thay vi frame don (tri flicker class).
        self._tracks = []
        self._track_id = 0
        self._no_track_log_t0 = 0.0
        # Frame moi nhat tu callback (worker thread doc, drop frame cu).
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = 0
        self._cb_count = 0
        self._cb_t0 = time.time()
        self._infer_count = 0
        self._infer_t0 = time.time()
        self._fps_t0 = time.time()
        # Chon device bang warmup that (cuda:0 truoc, fallback cpu).
        self.device = self._select_device()
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"yolov11_sortation.py initialized (conf={CONF_THRESHOLD}, "
            f"imgsz={IMG_SIZE}, device={self.device}, show={self.show})")

    def _select_device(self):
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        candidates = [FORCED_DEVICE] if FORCED_DEVICE else ["cuda:0", "cpu"]
        for dev in candidates:
            try:
                t0 = time.time()
                model(dummy, verbose=False, conf=CONF_THRESHOLD,
                      imgsz=IMG_SIZE, device=dev)
                dt = (time.time() - t0) * 1000.0
                self.get_logger().info(f"YOLO warmup OK on {dev} ({dt:.0f} ms)")
                return dev
            except Exception as e:
                self.get_logger().warn(f"YOLO warmup fail on {dev}: {e}")
        self.get_logger().error("All YOLO devices failed, fallback cpu")
        return "cpu"

    def ImageCallback(self, color_frame):
        # Chi decode + luu frame moi nhat, KHONG inference o day.
        rgb_image = np.frombuffer(color_frame.data, dtype=np.uint8).reshape(
            color_frame.height, color_frame.step)[:, :color_frame.width * 3].reshape(
            color_frame.height, color_frame.width, 3)
        with self._frame_lock:
            self._latest_frame = rgb_image
            self._latest_stamp += 1
        self._cb_count += 1

    def _infer_loop(self):
        last_stamp = -1
        while rclpy.ok():
            with self._frame_lock:
                frame = self._latest_frame
                stamp = self._latest_stamp
            if frame is None or stamp == last_stamp:
                time.sleep(FRAME_WAIT_S)
                continue
            last_stamp = stamp
            self._infer_count += 1
            self._infer_once(np.copy(frame))
            now = time.time()
            if now - self._fps_t0 >= FPS_LOG_S:
                cb_fps = self._cb_count / max(now - self._cb_t0, 1e-6)
                in_fps = self._infer_count / max(now - self._infer_t0, 1e-6)
                self.get_logger().info(
                    f"FPS cb={cb_fps:.1f} infer={in_fps:.1f} "
                    f"queued_items={len(self.processed_items)}")
                self._cb_count = 0
                self._infer_count = 0
                self._cb_t0 = self._infer_t0 = self._fps_t0 = now

    def _detect_on(self, frame):
        """Chay YOLO 1 frame -> (results, dets). Moi det: name/conf/cx/cy/box."""
        results = model(frame, verbose=False, conf=CONF_THRESHOLD,
                        imgsz=IMG_SIZE, device=self.device)
        dets = []
        boxes = results[0].boxes
        if boxes is not None:
            for box in boxes:
                class_id = int(box.cls)
                res_name = model.names[class_id]
                if res_name not in ALL_WASTE:
                    continue
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                dets.append({
                    "name": res_name,
                    "conf": float(box.conf),
                    "cx": (x_min + x_max) // 2,
                    "cy": (y_min + y_max) // 2,
                    "box": (x_min, y_min, x_max, y_max),
                })
        return results, dets

    def _infer_once(self, frame):
        # Suppress verbose output during inference
        h, w = frame.shape[:2]
        results, dets = self._detect_on(frame)
        # TTA fallback cho vat nghieng/xeo: lau khong thay box thi thu frame
        # xoay 90 do, map box ve toa do goc (xem inverse ROTATE_90_CLOCKWISE).
        if not dets and time.time() - self._last_det_time > TTA_STALE_S:
            rot = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
            rot_results, rot_dets = self._detect_on(rot)
            if rot_dets:
                results = rot_results
                mapped = []
                for d in rot_dets:
                    rcx = (d["box"][0] + d["box"][2]) // 2
                    rcy = (d["box"][1] + d["box"][3]) // 2
                    cx, cy = rcy, (h - 1) - rcx
                    x1 = max(0, d["box"][1])
                    y1 = max(0, (h - 1) - d["box"][2])
                    x2 = min(w - 1, d["box"][3])
                    y2 = min(h - 1, (h - 1) - d["box"][0])
                    if x2 > x1 and y2 > y1:
                        mapped.append({"name": d["name"], "conf": d["conf"],
                                       "cx": cx, "cy": cy,
                                       "box": (x1, y1, x2, y2)})
                if mapped:
                    dets = mapped
                    self.get_logger().info(
                        f"TTA-rot90 recovered {len(dets)} box(es)")
        if dets:
            self._last_det_time = time.time()
        self._update_tracks(frame, dets, time.time())
        if self.pubPos_flag:
            # SPACE: publish track on dinh nhat (majority class + median),
            # khong lay box frame don.
            pick = self._best_track(time.time())
            if pick is None:
                if time.time() - self._no_track_log_t0 > 3.0:
                    self._no_track_log_t0 = time.time()
                    self.get_logger().info("SPACE but no stable track yet")
            else:
                self._publish_pick(frame, pick)

        # Preview tuy chon (mac dinh TAT khi chay that de max FPS).
        # Overlay yaw kieu stacking: polygon + text '{class} {yaw}d(tier)'.
        if self.show:
            annotated_frame = results[0].plot()
            annotated_frame = cv.cvtColor(annotated_frame, cv.COLOR_RGB2BGR)
            draw_yaw_overlay(annotated_frame, self._viz_items())
            cv.imshow("YOLO Inference - All Garbage Detection",
                      cv.resize(annotated_frame, (640, 480)))
            if cv.waitKey(1) == 32:
                # SPACE: gap 1 item conf cao nhat hien tai, xong ve scan-only.
                self.pubPos_flag = True
                self.get_logger().info("Space pressed, grasp highest-confidence item")

    def _viz_items(self):
        """Item de ve overlay: track thay frame nay, kem yaw sample moi nhat."""
        now = time.time()
        items = []
        for tr in self._tracks:
            if now - tr["last_seen"] > 0.3 or tr["last_viz"] is None:
                continue
            s = self._track_summary(tr)
            yaw, ok, tier, poly = tr["last_viz"]
            items.append({"box": tr["box"], "name": s["name"], "yaw": yaw,
                          "ok": ok, "tier": tier, "poly": poly,
                          "tid": tr["id"],
                          "votes": sum(tr["votes"].values())})
        return items

    def _update_tracks(self, frame, dets, now):
        """Ghep det vao track theo IoU (greedy), track gia chet sau TTL.

        Moi det duoc lay 1 yaw sample (nhu stacking do goc moi frame); track
        giu yaw_hist de luc SPACE lay median thay vi 1 frame don.
        """
        for d in dets:
            yaw, ok, tier, poly = yaw_sample_frame(frame, d["box"])
            d["yaw"], d["yok"], d["ytier"], d["ypoly"] = yaw, ok, tier, poly
        unmatched = list(dets)
        for tr in self._tracks:
            best_i, best_iou = -1, TRACK_IOU_MIN
            for i, d in enumerate(unmatched):
                iou = box_iou(tr["box"], d["box"])
                if iou > best_iou:
                    best_i, best_iou = i, iou
            if best_i >= 0:
                d = unmatched.pop(best_i)
                tr["box"] = d["box"]
                tr["hist_box"].append(d["box"])
                tr["votes"][d["name"]] = tr["votes"].get(d["name"], 0) + 1
                tr["confs"].append(d["conf"])
                tr["yaw_hist"].append((d["yaw"], d["yok"], d["ytier"]))
                tr["last_viz"] = (d["yaw"], d["yok"], d["ytier"], d["ypoly"])
                tr["last_seen"] = now
        for d in unmatched:
            self._track_id += 1
            self._tracks.append({
                "id": self._track_id,
                "box": d["box"],
                "hist_box": deque([d["box"]], maxlen=TRACK_HIST_LEN),
                "votes": {d["name"]: 1},
                "confs": deque([d["conf"]], maxlen=TRACK_HIST_LEN),
                "yaw_hist": deque([(d["yaw"], d["yok"], d["ytier"])],
                                  maxlen=TRACK_HIST_LEN),
                "last_viz": (d["yaw"], d["yok"], d["ytier"], d["ypoly"]),
                "last_seen": now,
            })
        self._tracks = [tr for tr in self._tracks
                        if now - tr["last_seen"] <= TRACK_MAX_AGE_S]

    @staticmethod
    def _track_summary(tr):
        """majority class + median conf/box cua 1 track."""
        name = max(tr["votes"], key=lambda k: tr["votes"][k])
        conf = median(tr["confs"])
        xs = [b[0] for b in tr["hist_box"]]
        ys = [b[1] for b in tr["hist_box"]]
        xe = [b[2] for b in tr["hist_box"]]
        ye = [b[3] for b in tr["hist_box"]]
        box = (int(median(xs)), int(median(ys)),
               int(median(xe)), int(median(ye)))
        return {"name": name, "conf": conf, "box": box,
                "cx": (box[0] + box[2]) // 2, "cy": (box[1] + box[3]) // 2,
                "votes": sum(tr["votes"].values()), "id": tr["id"]}

    def _best_track(self, now):
        """Track on dinh nhat: diem = so vote * median conf."""
        best, best_score = None, 0.0
        for tr in self._tracks:
            if now - tr["last_seen"] > TRACK_MAX_AGE_S:
                continue
            s = self._track_summary(tr)
            score = s["votes"] * s["conf"]
            if score > best_score:
                best, best_score = s, score
        return best

    def _track_yaw(self, tr):
        """Yaw cua track = median yaw_hist (port stacking angle_history).

        Gate y nhu stacking get_Sqaure: kich clamp + spread lon / o ria anh
        -> FALLBACK yaw=0 (J5=J1). Tra (yaw_raw, reliable, tier, n, spread).
        """
        rels = [(y, t) for (y, ok, t) in tr["yaw_hist"] if ok]
        n = len(rels)
        if n < YAW_MIN_SAMPLES:
            return 0.0, False, "FALLBACK", n, 0.0
        ymed_all = median([y for y, _ in rels])
        # Chi xet inlier quanh median (+-10deg): 1-2 frame nhieu (vd khoa nham
        # khung anh in) khong duoc phu dinh ca track nhu spread tho.
        inl = [(y, t) for (y, t) in rels if abs(y - ymed_all) <= 10.0]
        if len(inl) < YAW_MIN_SAMPLES:
            return 0.0, False, "FALLBACK", n, 0.0
        ys = [y for y, _ in inl]
        ymed = median(ys)
        spread = max(ys) - min(ys)
        tiers = [t for _, t in inl]
        tier = max(set(tiers), key=tiers.count)
        box = tr["box"]
        cx = (box[0] + box[2]) / 2.0
        at_edge = cx < 120.0 or cx > 520.0
        kick = (abs(ymed) >= YAW_MAX_DEG - 0.5
                and (spread > 5.0 or at_edge))
        if kick or spread > YAW_MAX_SPREAD_DEG:
            return 0.0, False, "FALLBACK", n, spread
        return ymed, True, tier, n, spread

    def _refine_class(self, frame, box):
        """Pass-2 khi SPACE: crop box + margin, upscale, infer lai san thap.

        Tra (name, conf) tot nhat trong {crop, crop-flip} hoac (None, None).
        Chi tinh class/conf (box giu median track on dinh).
        """
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = box
        mx, my = int(0.25 * (x2 - x1)), int(0.25 * (y2 - y1))
        cx1, cy1 = max(0, x1 - mx), max(0, y1 - my)
        cx2, cy2 = min(w, x2 + mx), min(h, y2 + my)
        crop = frame[cy1:cy2, cx1:cx2]
        if crop.size == 0:
            return None, None
        scale = min(IMG_SIZE / max(crop.shape[:2]), 4.0)
        up = cv.resize(crop, (0, 0), fx=scale, fy=scale,
                       interpolation=cv.INTER_LINEAR)
        best_name, best_conf = None, 0.0
        for variant in (up, cv.flip(up, 1)):
            try:
                res = model(variant, verbose=False, conf=CONF_PASS2,
                            imgsz=IMG_SIZE, device=self.device)[0]
            except Exception:
                continue
            bx = res.boxes
            if bx is None:
                continue
            for b in bx:
                nm = model.names[int(b.cls)]
                cf = float(b.conf)
                if nm in ALL_WASTE and cf > best_conf:
                    best_name, best_conf = nm, cf
        return best_name, best_conf

    def _publish_pick(self, frame, pick):
        """Publish 1 pick tu track: pass-2 refine class -> yaw -> PosInfo."""
        res_name, confidence = pick["name"], pick["conf"]
        center_x, center_y = pick["cx"], pick["cy"]
        # Pass-2 co the sua class (conf thap frame don nhan nham).
        rname, rconf = self._refine_class(frame, pick["box"])
        if rname is not None and rname != res_name:
            self.get_logger().info(
                f"PASS2 override {res_name}({confidence:.2f}) -> "
                f"{rname}({rconf:.2f})")
            res_name, confidence = rname, rconf
        elif rname is not None:
            confidence = max(confidence, rconf)
        fwd_x, lat_y = pixel_to_world(center_x, center_y)
        # Vat dung yen: het TTL duoc thu lai; nhay vai px khong tao moi.
        item_key = quantize_key(res_name, center_x, center_y)
        prune_processed(self.processed_items, time.time())
        if item_key in self.processed_items:
            self.pubPos_flag = False
            return
        self.get_logger().info(
            f"Publishing position for {res_name} "
            f"(track#{pick['id']} votes={pick['votes']} "
            f"confidence={confidence:.2f}, pixel=({center_x},{center_y}))")
        self.pubPos_flag = False
        # Yaw truoc, PosInfo sau: grasp doc yaw trong thread rieng
        # nen yaw phai toi truoc (sleep 0.2 de chac chan thu tu).
        # Yaw = median yaw_hist cua track (port stacking), khong do 1 frame.
        tr = next((t for t in self._tracks if t["id"] == pick["id"]), None)
        if tr is not None:
            yaw_raw, reliable, tier, n, spread = self._track_yaw(tr)
        else:
            yaw_raw, reliable, tier, n, spread = 0.0, False, "FALLBACK", 0, 0.0
        self.get_logger().info(
            f"[{res_name}] yaw median {n} samples spread={spread:.1f}deg")
        yaw_deg = self._publish_yaw(yaw_raw, reliable, tier, res_name)
        center = AprilTagInfo()
        center.x = fwd_x
        center.y = lat_y
        center.z = PICK_Z_M
        if res_name in RECYCLABLE_BLUE:
            center.id = 1
        elif res_name in WET_GREEN:
            center.id = 2
        elif res_name in HAZARDOUS_RED:
            center.id = 3
        elif res_name in DRY_GREY:
            center.id = 4  # TODO(GREY): p_4 tam la pose Vang cu
        self.get_logger().info(
            f"Grasp target: {center} yaw={yaw_deg:+.1f}deg({tier})")
        self.pos_info_pub.publish(center)
        self.processed_items[item_key] = time.time()
        self.last_publish_time = time.time()

    def _publish_yaw(self, yaw_raw, reliable, tier, name):
        """Publish yaw (dang median track) len set_joint5, tra deg da dau.

        Publish gia tri DA DAU (YAW_SIGN): grasp giu nguyen J5 = J1 - delta.
        """
        if not reliable:
            yaw_raw = 0.0
        yaw_deg = YAW_SIGN * yaw_raw
        yaw_msg = Int16()
        yaw_msg.data = int(round(yaw_deg))
        self.TargetJoint5_pub.publish(yaw_msg)
        time.sleep(0.2)
        self.get_logger().info(
            f"[{name}] yaw_raw={yaw_raw:+.1f} signed={yaw_deg:+.1f}deg"
            f"({tier})")
        return yaw_deg

    def GraspStatusCallback(self, msg):
        self.get_logger().info(f"GraspStatusCallback received: {msg.data}")
        if msg.data:
            # Ve scan-only: KHONG tu mo khoa nhu ban cu (tranh tu gap lien tiep).
            self.pubPos_flag = False
            self.detect_flag = False
            self.compute_height = True
            self.get_logger().info("Grasp done, back to scan-only (SPACE to grasp next)")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--show", action="store_true",
                        help="Bat cua so preview (mac dinh tat de max FPS)")
    parsed, _ = parser.parse_known_args(args)
    rclpy.init(args=args)
    tag_detect = YoloAllGarbageDetectNode(show=parsed.show)
    try:
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        if parsed.show:
            cv.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
