"""Khung 640x480 tổng hợp (tag + ô màu) dùng cho test đặc trưng của cube_vision."""
import cv2
import numpy as np


def tag_image(tag_id: int, size: int = 120) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(d, tag_id, size)
    return cv2.copyMakeBorder(marker, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)


def frame(tags=((3, 300, 180),), squares=()):
    """tags: (id, x, y) góc trên-trái; squares: (bgr, x, y, side)."""
    img = np.full((480, 640, 3), 200, np.uint8)
    for tag_id, x, y in tags:
        patch = tag_image(tag_id)
        img[y:y + patch.shape[0], x:x + patch.shape[1]] = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
    for bgr, x, y, side in squares:
        img[y:y + side, x:x + side] = bgr
    return img


SQUARES = {"red": (0, 0, 200), "blue": (200, 30, 0), "green": (0, 180, 0), "yellow": (0, 220, 220)}

FRAMES = {
    "tag3": lambda: frame(((3, 300, 180),)),
    "tag1_tag4": lambda: frame(((1, 60, 60), (4, 420, 300))),
    "red_square": lambda: frame((), ((SQUARES["red"], 250, 200, 80),)),
    "blue_yellow": lambda: frame((), ((SQUARES["blue"], 100, 100, 70), (SQUARES["yellow"], 400, 250, 70))),
    "green_and_tag2": lambda: frame(((2, 400, 60),), ((SQUARES["green"], 100, 300, 80),)),
    "empty": lambda: frame(()),
}
