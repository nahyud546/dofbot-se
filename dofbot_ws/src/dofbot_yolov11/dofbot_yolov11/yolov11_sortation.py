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
"""
import argparse
import os
import threading
import time
import warnings
from pathlib import Path

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
CONF_THRESHOLD = 0.40   # ha tu 0.65: vat nho/mo duoc bat, doi lai loc bang TTL+grasp
IMG_SIZE = 640          # khop input onnx [1,3,640,640]
FORCED_DEVICE = os.environ.get("T6_YOLO_DEVICE", "").strip()  # "" = auto
# Pixel -> world (goc Yahboom, fit tai pose [90,120]).
# TODO(B3): hieu chuan lai tai pose dung that [90,125] (AprilTag tai world
# known -> ox, oy) roi dien X_OFFSET_M/Y_OFFSET_M, xoa bu cung ben grasp.
PIXEL_TO_M_Y = 1.0 / 4000.0
PIXEL_TO_M_X = 0.8 / 3000.0
X_BASE_M = 0.13
X_OFFSET_M = 0.0
Y_OFFSET_M = 0.0
PICK_Z_M = 0.03
# Item da publish giu trong processed_map TTL giay de vat dung yen duoc thu
# lai (key luong tu 10px chong nhay pixel).
PROCESSED_TTL_S = 60.0
PROCESSED_GRID_PX = 10
FRAME_WAIT_S = 0.005
FPS_LOG_S = 5.0

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
    lat_y = round((320 - center_x) * PIXEL_TO_M_Y + Y_OFFSET_M, 5)
    fwd_x = round((480 - center_y) * PIXEL_TO_M_X + X_BASE_M + X_OFFSET_M, 5)
    return fwd_x, lat_y


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

    def _infer_once(self, frame):
        # Suppress verbose output during inference
        results = model(frame, verbose=False, conf=CONF_THRESHOLD,
                        imgsz=IMG_SIZE, device=self.device)
        boxes = results[0].boxes
        if boxes is not None:
            for box in sorted(boxes, key=lambda b: float(b.conf), reverse=True):
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_id = int(box.cls)
                confidence = float(box.conf)
                res_name = model.names[class_id]
                if res_name not in ALL_WASTE:
                    continue
                # 计算重心位置
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                fwd_x, lat_y = pixel_to_world(center_x, center_y)
                # Vat dung yen: het TTL duoc thu lai; nhay vai px khong tao moi.
                item_key = quantize_key(res_name, center_x, center_y)
                prune_processed(self.processed_items, time.time())
                if item_key in self.processed_items:
                    continue
                # 1 publish/grasp-cycle: khoa toi khi grasp_done mo lai.
                if self.pubPos_flag:
                    self.get_logger().info(
                        f"Publishing position for {res_name} "
                        f"(confidence={confidence:.2f}, pixel=({center_x},{center_y}))")
                    self.pubPos_flag = False
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
                    self.get_logger().info(f"Grasp target: {center}")
                    self.pos_info_pub.publish(center)
                    self.processed_items[item_key] = time.time()
                    self.last_publish_time = time.time()
                    # publish xong + khoa flag: cac box con lai doi chu ky sau.
                    break

        # Preview tuy chon (mac dinh TAT khi chay that de max FPS).
        if self.show:
            annotated_frame = results[0].plot()
            annotated_frame = cv.cvtColor(annotated_frame, cv.COLOR_RGB2BGR)
            cv.imshow("YOLO Inference - All Garbage Detection",
                      cv.resize(annotated_frame, (640, 480)))
            if cv.waitKey(1) == 32:
                # SPACE: gap 1 item conf cao nhat hien tai, xong ve scan-only.
                self.pubPos_flag = True
                self.get_logger().info("Space pressed, grasp highest-confidence item")

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
