#!/usr/bin/env python3
# coding: utf-8
"""Color stacking play - ban .py chuyen tu notebook (khong can jupyter).

Chay: xem command o cuoi file / tin nhan kem theo.
Phim:
  c: calibration mode | o: calibration OK + luu XYT | x: huy calibration
  1/2/3/4: them red/green/blue/yellow vao thu tu xep (lan 1 = tang day)
  0: xoa danh sach mau
  d: detection
  SPACE/g: grap (xep) | q/ESC: thoat
Trackbar: joint1, joint2, thresh (giong slider notebook).
Luu y: joint2 thay doi pose camera; phep doi pixel -> X hien tai chua
hieu chuan theo joint2.
Hien thi: VIEW:warp-preview (xem calib) / VIEW:detect-raw (anh that dung
de detect). Goc nhin "khac" sau OK chi la warp preview, khong phai TF.
"""
import os
import sys
import threading
import argparse
from time import sleep

import cv2 as cv

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import Arm_Lib
from dofbot_config import Arm_Calibration, read_HSV, read_XYT, write_XYT
from stacking_target import stacking_GetTarget

HSV_PATH = os.path.join(SCRIPT_DIR, "HSV_config.txt")
XYT_PATH = os.path.join(SCRIPT_DIR, "XYT_config.txt")
parser = argparse.ArgumentParser(description="Color stacking camera and arm control")
parser.add_argument("camera", nargs="?", default="2", help="camera index hoặc /dev/videoN")
args = parser.parse_args()
try:
    CAMERA = f"/dev/video{int(args.camera)}"
except ValueError:
    CAMERA = args.camera
PORT = "/dev/ttyUSB0"
WIN = "stacking"

COLOR_KEYS = {"1": "red", "2": "green", "3": "blue", "4": "yellow"}


def draw_help(img):
    lines = [
        "c:calib o:ok x:cancel 1-4:add color 0:clear",
        "d:detect SPACE/g:grap q:quit",
    ]
    y = 20
    for t in lines:
        cv.putText(img, t, (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y += 20
    return img


def read_observation_pose(arm):
    """Read the physical camera pose; slider values are only commands."""
    j1 = arm.Arm_serial_servo_read(1)
    j2 = arm.Arm_serial_servo_read(2)
    if j1 is None or j2 is None:
        return None
    return [j1, j2]


def main():
    dp = []
    msg = {}
    detection_pose = None
    model = "General"
    color_list = {}
    # Names placed successfully in this run. They remain skipped on later
    # SPACE presses, so a failed later layer can be retried without restarting
    # already placed cubes from layer 1.
    completed_colors = []
    motion_busy = threading.Event()
    motion_thread = None
    last_calibration_xy = None
    threshold = 140
    xy = [90, 135]
    color_hsv = {"red": ((0, 43, 46), (10, 255, 255)),
                 "green": ((35, 43, 46), (77, 255, 255)),
                 "blue": ((100, 43, 46), (124, 255, 255)),
                 "yellow": ((26, 43, 46), (34, 255, 255))}
    try:
        read_HSV(HSV_PATH, color_hsv)
    except Exception:
        print("No HSV_config file, dung default!!!")
    try:
        xy, threshold = read_XYT(XYT_PATH)
    except Exception:
        print("No XYT_config file, dung default!!!")
    print("HSV:", color_hsv)
    print("XYT:", xy, threshold)

    arm = Arm_Lib.Arm_Device(PORT)
    joints_0 = [xy[0], xy[1], 0, 0, 90, 30]
    # Go to the camera/ready pose before constructing stacking_GetTarget,
    # whose constructor may wait indefinitely for the IK service.
    print(f"Moving to stacking ready pose: {joints_0}", flush=True)
    arm.Arm_serial_servo_write6_array(joints_0, 1500)
    sleep(1.5)

    cap = cv.VideoCapture(CAMERA, cv.CAP_V4L2)
    if not cap.isOpened():
        print(f"Khong mo duoc {CAMERA}, thu index 2")
        cap = cv.VideoCapture(2)
    if not cap.isOpened():
        print("Khong mo duoc camera, thoat.")
        return 1
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    cv.namedWindow(WIN, cv.WINDOW_AUTOSIZE)
    ok, preview = cap.read()
    if not ok:
        cap.release()
        cv.destroyAllWindows()
        raise RuntimeError(f"Camera {CAMERA} opened but did not return a frame")
    cv.imshow(WIN, cv.resize(preview, (640, 480)))
    cv.waitKey(1)
    print("Camera preview opened; waiting for IK service if needed.", flush=True)
    target = stacking_GetTarget()
    calibration = Arm_Calibration()
    cv.createTrackbar("joint1", WIN, xy[0], 110, lambda v: None)
    cv.createTrackbar("joint2", WIN, xy[1], 155, lambda v: None)
    cv.createTrackbar("thresh", WIN, threshold, 255, lambda v: None)
    # gioi han duoi trackbar khong set duoc bang OpenCV, doc + clamp khi dung
    print("Nhan d: detect, SPACE/g: grap, q: quit. Xem help tren anh.")

    try:
        while True:
            ok, img = cap.read()
            if not ok:
                print("frame read failed")
                sleep(0.1)
                continue
            img = cv.resize(img, (640, 480))
            j1 = max(70, cv.getTrackbarPos("joint1", WIN))
            j2 = max(115, cv.getTrackbarPos("joint2", WIN))
            th = cv.getTrackbarPos("thresh", WIN)
            cur_xy = [j1, j2]

            key = cv.waitKey(10) & 0xFF

            if key == ord("c"):
                model = "Calibration"
                print(model)
            elif key == ord("o"):
                try:
                    write_XYT(XYT_PATH, cur_xy, th)
                    print("Saved XYT:", cur_xy, th)
                except Exception as e:
                    print("File XYT_config Error:", e)
                dp, img = calibration.calibration_map(img, cur_xy, th, move_arm=False)
                model = "General"
            elif key == ord("x"):
                dp = []
                msg = {}
                model = "General"
                print("calibration_Cancel")
            elif chr(key) in COLOR_KEYS if key < 128 else False:
                name = COLOR_KEYS[chr(key)]
                slot = str(len(color_list) + 1)
                if len(color_list) < 4 and name not in color_list.values():
                    color_list[slot] = name
                    model = "General"
                    print("color_list:", color_list)
            elif key == ord("0"):
                if motion_busy.is_set():
                    print("Cannot clear the layer list while the arm is moving.")
                else:
                    color_list = {}
                    completed_colors.clear()
                    msg = {}
                    target.center_history.clear()
                    model = "General"
                    print("reset color_list and layer progress")
            elif key == ord("d"):
                detection_pose = read_observation_pose(arm)
                msg = {}
                target.center_history.clear()
                if detection_pose is None:
                    model = "General"
                    print("Khong doc duoc J1/J2 tu servo; chua detect/gap.")
                else:
                    model = "Detection"
                    print("Detection at actual J1/J2:", detection_pose)
            elif key in (ord(" "), ord("g")):
                model = "Grap"
                print(model)

            if model == "Calibration":
                # Move only when entering calibration or when a pose trackbar
                # changes. Reissuing servo commands on every video frame can
                # interfere with the arm's motion.
                if last_calibration_xy != cur_xy:
                    calibration.arm.Arm_serial_servo_write6_array(
                        [cur_xy[0], cur_xy[1], 0, 0, 90, 30], 1500)
                    last_calibration_xy = list(cur_xy)
                _, img = calibration.calibration_map(img, cur_xy, th, move_arm=False)
            # Cong thuc pixel->robot trong stacking_target.get_Sqaure la fit
            # tuyen tinh cho anh THO tai pose calib (goc Yahboom
            # dofbot_sorting/.../color_sorting.py:120). Neu detect tren anh da
            # Perspective_transform thi tam bi keo di -> sai he thong theo X.
            # Giu frame raw cho detect, warp chi de hien thi calib.
            frame_raw = img.copy()
            display_warped = None
            view_txt = "VIEW:raw"
            if len(dp) != 0 and model != "Detection":
                display_warped = calibration.Perspective_transform(dp, img)
                img = display_warped
                view_txt = "VIEW:warp-preview (chi de xem, detect dung anh raw)"
            pending_color_list = {
                slot: name for slot, name in color_list.items()
                if name not in completed_colors
            }
            # Dong bang perception trong khi tay chuyen dong; camera nam tren
            # tay nen frame khi quay khong duoc phep cap nhat target da chot.
            frozen = motion_busy.is_set()
            if len(pending_color_list) != 0 and model == "Detection" and not frozen:
                view_txt = "VIEW:detect-raw"
                img, msg = target.select_color(frame_raw, color_hsv,
                                                pending_color_list)
            if frozen:
                view_txt = "VIEW:detect-raw (frozen: tay dang chuyen dong)"
            if model == "Grap":
                remaining_order = [name for name in color_list.values()
                                   if name not in completed_colors]
                actual_pose = read_observation_pose(arm)
                if (detection_pose is None or actual_pose is None or
                        any(abs(a - b) > 2 for a, b in zip(actual_pose, detection_pose))):
                    print("Pose camera da doi hoac khong doc duoc J1/J2; nhan d de detect lai.")
                    msg = {}
                    model = "General"
                elif not remaining_order:
                    print("All selected layers are already placed.")
                    model = "Detection"
                elif motion_busy.is_set():
                    print("Stack ignored: previous arm motion is still running.")
                    model = "Detection"
                elif any(name not in msg for name in remaining_order):
                    missing = [n for n in remaining_order if n not in msg]
                    print(f"Chua du detection ({missing}): nhan d, doi tam on "
                          f"dinh roi SPACE lai. Khong chay kho.")
                    model = "Detection"
                else:
                    motion_busy.set()

                    def run_stack(snapshot, pose, order):
                        try:
                            target.target_run(
                                snapshot, pose, color_order=order,
                                completed=completed_colors
                            )
                        finally:
                            motion_busy.clear()

                    motion_thread = threading.Thread(
                        target=run_stack,
                        args=(dict(msg), list(detection_pose), list(remaining_order)),
                        daemon=True)
                    motion_thread.start()
                    msg = {}
                    model = "Detection"

            info = f"model:{model} colors:{color_list} msg:{len(msg)} dp:{len(dp)}"
            cv.putText(img, info, (10, 470), cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            cv.putText(img, view_txt, (10, 452), cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
            img = draw_help(img)
            cv.imshow(WIN, img)

            if key == ord("q") or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if motion_thread is not None and motion_thread.is_alive():
            print("Waiting for the current stacking sequence to finish before shutdown.")
            motion_thread.join()
        print(f"Returning to stacking ready pose: {joints_0}", flush=True)
        arm.Arm_serial_servo_write6_array(joints_0, 1500)
        sleep(1.5)
        cap.release()
        cv.destroyAllWindows()
        try:
            target.destroy_node()
        except Exception:
            pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
