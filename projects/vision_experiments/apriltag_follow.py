#!/usr/bin/env python3
# T3 AprilTag follow - in file apriltag_id0.png ra giay, dua truoc camera, tay bam theo tam tag.
# Tag: tag36h11 id 0. In cang to cang de bam (khuyen 8-10cm), de phang, du sang.
# Chay:  QT_QPA_PLATFORM=xcb .venv/bin/python apriltag_follow.py
import cv2 as cv
import time
import Arm_Lib
from pupil_apriltags import Detector

DIR_X = -1         # giong ban face (doi thanh 1 neu nguoc trai/phai)
DEAD = 8
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

det = Detector(families='tag36h11', nthreads=4, quad_decimate=2.0)

tsx, tsy, last = 90.0, 45.0, 0.0
scx, scy, last_seen = 320.0, 240.0, 0.0

cap = cv.VideoCapture(CAM, cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    tags = det.detect(cv.cvtColor(frame, cv.COLOR_BGR2GRAY))
    if tags:
        t = max(tags, key=lambda t: (t.corners[2][0] - t.corners[0][0]) ** 2)
        fx, fy = float(t.center[0]), float(t.center[1])
        scx, scy = 0.7 * fx + 0.3 * scx, 0.7 * fy + 0.3 * scy
        last_seen = time.time()
        for i in range(4):
            p1 = tuple(t.corners[i].astype(int))
            p2 = tuple(t.corners[(i + 1) % 4].astype(int))
            cv.line(frame, p1, p2, (0, 255, 0), 2)
        cv.putText(frame, f"id={t.tag_id}", (int(scx) + 10, int(scy)), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    if time.time() - last_seen < 0.4:
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
            print(f"id:center ex={ex:.0f} ey={ey:.0f} tsx={tsx:.1f}", end='\r')
    else:
        cv.putText(frame, 'NO TAG - dua tag id0 truoc camera', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv.imshow('apriltag follow (q=thoat)', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
