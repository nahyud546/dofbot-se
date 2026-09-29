#!/usr/bin/env python3
"""HSV cube selection using the same camera geometry as color stacking.

Keys: b/g/r/y select a color; SPACE picks the selected cube; c calibrates
HSV from a dragged ROI; i returns to detection. Motion is owned by
color_bin_grasp, which places the cube in the selected color's fixed bin.
"""

import os
from pathlib import Path
import shutil
import sys
import time

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from Arm_Lib import Arm_Device
from dofbot_sorting_3d.color_common import color_detect, read_HSV, write_HSV

def _repo_root() -> Path:
    env = os.environ.get("ROBOT_ARM_ROOT")
    if env and Path(env).exists():
        return Path(env)
    # new layout -> legacy fallbacks (never hardcode a single machine path)
    for c in [
        Path(__file__).resolve().parents[5] if len(Path(__file__).resolve().parents) >= 6 else None,
        Path("/home/jloy/Desktop/robot-arm"),
        Path("/home/yahboom"),
    ]:
        if c and (c / "workspaces").exists():
            return c
    return Path("/home/jloy/Desktop/robot-arm")

_REPO = _repo_root()
STACK_SCRIPTS = _REPO / "workspaces/dofbot_ws/src/dofbot_color_stacking/scripts"
if str(STACK_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(STACK_SCRIPTS))
from cube_geometry import CubeGeometry


PACKAGE_DIR = Path(__file__).resolve().parent
XYT_PATH = _REPO / "workspaces/dofbot_ws/src/dofbot_color_stacking/scripts/XYT_config.txt"
COLORS = {ord('b'): (1, 'blue'), ord('g'): (2, 'green'),
          ord('r'): (3, 'red'), ord('y'): (4, 'yellow')}


def ready_pose():
    config = dict(line.strip().split('=', 1) for line in XYT_PATH.read_text().splitlines()
                  if '=' in line)
    return [int(config['x']), int(config['y']), 0, 0, 90, 30]


class ColorRecognizeNode(Node):
    def __init__(self):
        super().__init__('color_sorting')
        self.arm = Arm_Device('/dev/ttyUSB0')
        self.home = ready_pose()
        self.geometry = CubeGeometry(self.get_logger())
        self.color_calibration = color_detect()
        self.target_color = None
        self.target_id = None
        self.calibrating = False
        self.drag_start = None
        self.drag_end = None
        self.motion_busy = False
        self.windows_name = 'color sorting'
        # Atomic request: [color_id, KDL x, KDL y, cube yaw degrees].
        # Keeping yaw in the same message prevents picking with a stale angle.
        self.pick_pub = self.create_publisher(Float32MultiArray, 'color_pick', 10)
        self.create_subscription(Bool, 'grasp_done', self.on_grasp_done, 10)
        self.create_subscription(Image, '/image_raw', self.on_image, 1)
        self.arm.Arm_serial_servo_write6_array(self.home, 1500)
        time.sleep(1.5)
        cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback(self.windows_name, self.on_mouse)
        if shutil.which('v4l2-ctl'):
            os.system('v4l2-ctl -d /dev/video2 -c brightness=10')
        self.get_logger().info(f'Ready pose {self.home}; b/g/r/y select color, SPACE sort')

    def on_grasp_done(self, msg):
        self.motion_busy = False
        self.geometry.clear()
        if msg.data:
            self.get_logger().info('Sort complete; ready for a fresh detection')
        else:
            self.get_logger().error('Sort failed; check grasp/IK log before retrying')

    def on_mouse(self, event, x, y, _flags, _param):
        if not self.calibrating or self.target_color is None:
            return
        if event == cv.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.drag_end = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_end = (x, y)
        elif event == cv.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_end = (x, y)

    @staticmethod
    def decode(msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')
        channels = 3
        row_pixels = msg.step // channels
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, row_pixels, channels)[:, :msg.width, :]
        return cv.cvtColor(image, cv.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else image.copy()

    def camera_at_ready(self):
        actual = [self.arm.Arm_serial_servo_read(i) for i in (1, 2)]
        return all(v is not None and abs(v - ref) <= 2
                   for v, ref in zip(actual, self.home[:2]))

    def on_image(self, msg):
        try:
            raw = cv.resize(self.decode(msg), (640, 480))
        except (ValueError, TypeError) as exc:
            self.get_logger().error(str(exc))
            return
        frame = raw.copy()
        key = cv.waitKey(10) & 0xff
        if key in COLORS:
            self.target_id, self.target_color = COLORS[key]
            self.calibrating = False
            self.geometry.clear()
            self.get_logger().info(f'Selected {self.target_color}')
        elif key == ord('c') and self.target_color and not self.motion_busy:
            self.calibrating = True
            self.drag_start = None
            self.drag_end = None
        elif key == ord('i'):
            self.calibrating = False
            self.geometry.clear()

        detection = None
        if self.target_color and not self.motion_busy:
            hsv_path = PACKAGE_DIR / f'{self.target_color}_colorHSV.text'
            hsv_range = read_HSV(str(hsv_path))
            if self.calibrating:
                if self.drag_start and self.drag_end:
                    x0, y0 = self.drag_start
                    x1, y1 = self.drag_end
                    roi = (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
                    if roi[2] > roi[0] and roi[3] > roi[1]:
                        cv.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]),
                                     (0, 255, 0), 2)
                        if key == ord('s'):
                            _, new_hsv = self.color_calibration.Roi_hsv(raw.copy(), roi)
                            write_HSV(str(hsv_path), new_hsv)
                            self.calibrating = False
                            self.geometry.clear()
                            self.get_logger().info(f'Saved HSV for {self.target_color}')
            elif hsv_range:
                detection, _mask = self.geometry.detect(raw, hsv_range,
                                                        self.target_color, frame)

        if key == 32:
            if self.motion_busy:
                self.get_logger().warning('Arm is moving; request ignored')
            elif detection is None or self.target_id is None:
                self.get_logger().warning('No selected cube detected; request ignored')
            elif not self.camera_at_ready():
                self.get_logger().warning('Camera is not at XYT ready pose; request ignored')
                self.geometry.clear()
            else:
                (target_x, target_y), yaw = detection
                self.motion_busy = True
                request = Float32MultiArray()
                request.data = [float(self.target_id), float(target_x),
                                float(target_y), float(yaw)]
                self.pick_pub.publish(request)
                self.get_logger().info(
                    f'Sort {self.target_color}: KDL x={target_x:.4f} '
                    f'y={target_y:.4f} yaw={yaw:+.1f}')
        label = self.target_color or 'none'
        cv.putText(frame, f'{label}  b/g/r/y select  SPACE sort  c HSV  i detect',
                   (8, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if self.motion_busy:
            cv.putText(frame, 'ARM MOVING - detection frozen', (8, 48),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        elif self.calibrating:
            cv.putText(frame, 'Drag ROI, press s to save HSV', (8, 48),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv.imshow(self.windows_name, frame)


def main(args=None):
    rclpy.init(args=args)
    node = ColorRecognizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
