"""Camera-to-KDL mapping and cube yaw shared by stacking and color sorting.

The calibration applies to a raw 640x480 frame at the XYT observation pose.
It is an empirical map, not a tf2 transform or a camera intrinsic calibration.
"""

import math
from collections import deque
from statistics import median

import cv2 as cv


PIXEL_TO_M_Y = 1.0 / 4000.0
PIXEL_TO_M_X = 0.8 / 3000.0
X_BASE_M = 0.15
K_DISTORTION = 0.06
Y_OFFSET_M = 0.0
CUBE_YAW_MAX_DEG = 40.0
CUBE_YAW_SIGN = 1.0
PICK_Z_M = 0.039
PICK_PITCH = 1.04


class CubeGeometry:
    def __init__(self, logger=None):
        self.logger = logger
        self.center_history = {}
        self.angle_history = {}
        self._yaw_warn = {}

    def clear(self):
        self.center_history.clear()
        self.angle_history.clear()
        self._yaw_warn.clear()

    def detect(self, raw_image, hsv_range, color_name, display_image=None):
        """Return ((KDL x, KDL y), cube yaw) or None; annotate display image."""
        if display_image is None:
            display_image = raw_image
        lower, upper = hsv_range
        hsv = cv.cvtColor(raw_image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(hsv, lower, upper)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)
        binary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)
        result = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = result[1] if len(result) == 3 else result[0]
        candidates = [(cv.contourArea(cnt), cnt) for cnt in contours
                      if cv.contourArea(cnt) > 1000]
        if not candidates:
            self.center_history.pop(color_name, None)
            self.angle_history.pop(color_name, None)
            return None, binary
        _, cnt = max(candidates, key=lambda item: item[0])
        moments = cv.moments(cnt)
        if moments["m00"] <= 0:
            return None, binary
        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        history = self.center_history.setdefault(color_name, deque(maxlen=5))
        history.append((center_x, center_y))
        point_x = median(p[0] for p in history)
        point_y = median(p[1] for p in history)

        # A square has a 90-degree symmetry. Match the nearest edge axis,
        # then convert image rotation to the KDL/world yaw convention.
        box = cv.boxPoints(cv.minAreaRect(cnt))
        best_len, alpha = -1.0, 0.0
        for i in range(4):
            dx = float(box[(i + 1) % 4][0] - box[i][0])
            dy = float(box[(i + 1) % 4][1] - box[i][1])
            length = dx * dx + dy * dy
            if length > best_len:
                best_len = length
                alpha = math.degrees(math.atan2(dy, dx)) % 180.0
        r = alpha % 90.0
        delta = -r if r <= 45.0 else 90.0 - r
        delta = max(-CUBE_YAW_MAX_DEG, min(CUBE_YAW_MAX_DEG, delta))
        angles = self.angle_history.setdefault(color_name, deque(maxlen=5))
        angles.append(delta)
        cube_yaw = median(angles)
        spread = max(angles) - min(angles)
        at_edge = point_x < 120.0 or point_x > 520.0
        yaw_reliable = not (abs(cube_yaw) >= CUBE_YAW_MAX_DEG - 0.5
                            and (spread > 5.0 or at_edge))
        if not yaw_reliable:
            if self.logger and not self._yaw_warn.get(color_name, False):
                self.logger.warning(
                    f"[{color_name}] cube yaw unreliable at image edge; "
                    "using 0 degrees. Move cube toward image center to retry.")
            self._yaw_warn[color_name] = True
            cube_yaw = 0.0
        else:
            self._yaw_warn[color_name] = False

        # Same raw-image calibration as stacking_target.get_Sqaure.
        rn2 = ((point_x - 320.0) / 320.0) ** 2 + ((point_y - 240.0) / 240.0) ** 2
        undistort = 1.0 + K_DISTORTION * rn2
        ux = 320.0 + (point_x - 320.0) * undistort
        uy = 240.0 + (point_y - 240.0) * undistort
        forward = round((480.0 - uy) * PIXEL_TO_M_X + X_BASE_M, 5)
        lateral = round((ux - 320.0) * PIXEL_TO_M_Y + Y_OFFSET_M, 8)
        target = (-forward, lateral)

        x, y, w, h = cv.boundingRect(cnt)
        cv.drawContours(display_image, [cnt], -1, (0, 255, 0), 2)
        cv.rectangle(display_image, (x, y), (x + w, y + h), (255, 255, 0), 1)
        cv.circle(display_image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
        cv.putText(display_image, f"{color_name} {cube_yaw:+.0f}d",
                   (int(x - 15), int(y - 15)), cv.FONT_HERSHEY_SIMPLEX,
                   1, (255, 0, 255), 2)
        if self.logger:
            self.logger.info(
                f"[{color_name}] px=({point_x:.1f},{point_y:.1f}) "
                f"undist={(undistort - 1.0) * 100:.1f}% "
                f"-> KDL=({target[0]:.5f},{target[1]:.5f}) "
                f"yaw={cube_yaw:+.1f}deg"
                f"{'' if yaw_reliable else ' FALLBACK(J5=J1)'}")
        return (target, cube_yaw), binary
