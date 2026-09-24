#!/usr/bin/env python3
# T3 KCF follow (CPU OpenCV) - khoanh vat bat ky roi tay bam theo.
# Chon ROI: keo chuot tren cua so -> nhan SPACE bat dau follow, R chon lai, Q thoat.
# Chay:  QT_QPA_PLATFORM=xcb .venv/bin/python kcf_follow.py
import cv2 as cv
import time
import Arm_Lib

DIR_X = -1         # giong ban face (doi thanh 1 neu nguoc trai/phai)
DEAD = 10
KP_X = 0.03
KP_Y = 0.028
MAX_STEP = 8
THROTTLE = 0.15
RUNTIME = 180
PORT = '/dev/ttyUSB0'
CAM = '/dev/video0'

_O = Arm_Lib.Arm_Device.__init__
Arm_Lib.Arm_Device.__init__ = lambda s, com=PORT, *a, **k: _O(s, PORT, *a, **k)
arm = Arm_Lib.Arm_Device(PORT)
arm.Arm_serial_servo_write6_array([90, 135, 20, 25, 90, 30], 800)

try:
    tracker = cv.TrackerKCF_create()
except AttributeError:
    tracker = cv.legacy.TrackerKCF_create()

tsx, tsy, last = 90.0, 45.0, 0.0
roi, tracking, selecting = None, False, False
p0 = (0, 0)

cap = cv.VideoCapture(CAM, cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)


def on_mouse(event, x, y, flags, param):
    global roi, tracking, selecting, p0, tracker
    if event == cv.EVENT_LBUTTONDOWN:
        selecting, p0 = True, (x, y)
        tracking = False
    elif event == cv.EVENT_MOUSEMOVE and selecting:
        roi = (min(p0[0], x), min(p0[1], y), abs(x - p0[0]), abs(y - p0[1]))
    elif event == cv.EVENT_LBUTTONUP:
        selecting = False
        x0, y0 = min(p0[0], x), min(p0[1], y)
        w, h = abs(x - p0[0]), abs(y - p0[1])
        roi = (x0, y0, w, h) if w > 10 and h > 10 else None


cv.namedWindow('kcf follow')
cv.setMouseCallback('kcf follow', on_mouse)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('r'):
        tracking, roi = False, None
    if key == 32 and roi is not None:  # SPACE
        try:
            tracker = cv.TrackerKCF_create()
        except AttributeError:
            tracker = cv.legacy.TrackerKCF_create()
        tracking = tracker.init(frame, tuple(map(int, roi)))
    if tracking:
        ok, box = tracker.update(frame)
        if ok:
            x, y, w, h = map(int, box)
            scx, scy = x + w / 2, y + h / 2
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
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
                print(f"cx={scx:.0f} cy={scy:.0f} tsx={tsx:.1f}", end='\r')
        else:
            cv.putText(frame, 'LOST - nhan R chon lai', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    elif roi is not None and not selecting:
        x, y, w, h = map(int, roi)
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv.putText(frame, 'SPACE= follow  R= chon lai', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    else:
        cv.putText(frame, 'keo chuot khoanh vat', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    if selecting and roi is not None:
        x, y, w, h = map(int, roi)
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
    cv.imshow('kcf follow', frame)
cap.release()
cv.destroyAllWindows()
