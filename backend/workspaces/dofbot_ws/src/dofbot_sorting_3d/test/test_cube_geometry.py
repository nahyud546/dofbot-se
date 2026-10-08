"""Check the shared raw-camera mapping and wrist angle without robot hardware."""

from pathlib import Path
import sys

import cv2 as cv
import numpy as np

STACK_SCRIPTS = Path(__file__).resolve().parents[2] / 'dofbot_color_stacking' / 'scripts'
sys.path.insert(0, str(STACK_SCRIPTS))
from cube_geometry import CubeGeometry


RED_HSV = ((0, 140, 70), (8, 255, 255))


def cube_frame(angle):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    vertices = cv.boxPoints(((320, 240), (85, 85), angle)).astype(np.int32)
    cv.fillConvexPoly(frame, vertices, (0, 0, 255))
    return frame


def test_cube_rotation_changes_wrist_yaw_in_world_frame():
    for image_angle, expected_yaw in ((-25, 25), (0, 0), (25, -25)):
        frame = cube_frame(image_angle)
        detected, _ = CubeGeometry().detect(frame, RED_HSV, 'red', frame.copy())
        assert detected is not None
        (x, y), yaw = detected
        assert -0.23 < x < -0.20
        assert abs(y) < 0.005
        assert abs(yaw - expected_yaw) < 3.0


def test_detection_history_clears_when_cube_disappears():
    geometry = CubeGeometry()
    frame = cube_frame(25)
    assert geometry.detect(frame, RED_HSV, 'red')[0] is not None
    blank = np.zeros_like(frame)
    assert geometry.detect(blank, RED_HSV, 'red')[0] is None
    assert 'red' not in geometry.center_history
    assert 'red' not in geometry.angle_history
