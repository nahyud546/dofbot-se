#!/usr/bin/env python3
# T3 face follow GPU (YOLOv8n-face, RTX 2050) - center-lock visual servo.
# Dieu khien sach dang P (do/phut) thay vi dung sai lop PID cu -> het windup/tsx bay.
# Chay:  source scripts/setup/setup_env.sh && .venv/bin/python projects/vision_experiments/face_follow_gpu.py
import os
import sys
from pathlib import Path
import cv2 as cv
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "tools"))
try:
    from repo_paths import ROBOT_ARM_ROOT, first_existing
    _ROOT = ROBOT_ARM_ROOT
except Exception:
    _ROOT = Path(os.environ.get("ROBOT_ARM_ROOT", Path(__file__).resolve().parents[2]))
    def first_existing(cands):
        for c in cands:
            p = Path(c)
            if p.exists():
                return p
        return None
import Arm_Lib
from ultralytics import YOLO

# ---- tham so chinh o day ----
DIR_X = -1         # dao chieu trai/phai: van nguoc thi doi 1
DEAD = 6           # px: dung yen khi mat cach tam < DEAD
KP_X = 0.03        # do servo / px lech X (0.03*100px ~ 3 do/buoc)
KP_Y = 0.028       # do servo / px lech Y
MAX_STEP = 8       # kep buoc xoay moi lenh (do)
THROTTLE = 0.15    # s giua 2 lenh servo
RUNTIME = 180      # ms runtime servo (cang nho cang nhanh/giat)
CONF = 0.4
HOLD = 0.4         # s giu vi tri khi OUT chop
MODEL = str(first_existing([
    _ROOT / "ai/models/detection/yolov8n-face.pt",
    _ROOT / "models/yolov8n-face.pt",  # pre-restructure fallback
    Path("/home/jloy/Desktop/robot-arm/ai/models/detection/yolov8n-face.pt"),
    Path("/home/jloy/Desktop/robot-arm/models/yolov8n-face.pt"),
]) or (_ROOT / "ai/models/detection/yolov8n-face.pt"))
PORT = '/dev/ttyUSB0'
CAM = '/dev/video0'

_O = Arm_Lib.Arm_Device.__init__
Arm_Lib.Arm_Device.__init__ = lambda s, com=PORT, *a, **k: _O(s, PORT, *a, **k)
arm = Arm_Lib.Arm_Device(PORT)
arm.Arm_serial_servo_write6_array([90, 135, 20, 25, 90, 30], 800)

m = YOLO(MODEL)
m.to('cuda:0')

tsx, tsy, last = 90.0, 45.0, 0.0
sb, last_seen = None, 0.0

cap = cv.VideoCapture(CAM, cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    r = m.predict(frame, imgsz=320, conf=CONF, verbose=False, device='cuda:0')[0]
    if len(r.boxes):
        b = max(r.boxes, key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1]))
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        sb = (x1, y1, x2, y2) if sb is None else tuple(0.75 * v + 0.25 * o for v, o in zip((x1, y1, x2, y2), sb))
        last_seen = time.time()
    if time.time() - last_seen < HOLD and sb is not None:
        x1, y1, x2, y2 = sb
        scx, scy = (x1 + x2) / 2, (y1 + y2) / 2
        cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv.drawMarker(frame, (320, 240), (255, 0, 0), cv.MARKER_CROSS, 20, 2)
        cv.line(frame, (320, 240), (int(scx), int(scy)), (0, 255, 255), 2)
        ex, ey = 320 - scx, 240 - scy
        if (abs(ex) > DEAD or abs(ey) > DEAD) and time.time() - last > THROTTLE:
            dx = max(-MAX_STEP, min(MAX_STEP, -DIR_X * KP_X * ex))
            dy = max(-MAX_STEP, min(MAX_STEP, KP_Y * ey))
            tsx = min(180.0, max(0.0, tsx + dx))
            tsy = min(180.0, max(0.0, tsy + dy))
            arm.Arm_serial_servo_write6_array([tsx, 135, tsy / 2, tsy / 2, 90, 0], RUNTIME)
            last = time.time()
            print(f"ex={ex:.0f} ey={ey:.0f} tsx={tsx:.1f} tsy={tsy:.1f}", end='\r')
    else:
        cv.putText(frame, 'OUT - giu vi tri', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv.imshow('center-lock GPU (q=thoat)', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
