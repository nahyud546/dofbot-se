"""Module nhận diện cube dùng chung (không phụ thuộc ROS).

Mỗi cube có 3 loại mặt: AprilTag, màu, ảnh rác. Mọi chương trình gọi lại các
phần rời rạc ở đây thay vì tự viết lại:

    registry  - bảng 4 cube (ID, màu, tag, lớp rác) + các profile HSV có tên
    tag       - TagDetector (AprilTag tag36h11)
    color     - mặt nạ HSV + ứng viên ô vuông
    trash     - TrashDetector (DINOv2)
    identify  - Identifier(profile) gom các phần trên: TAG / TAG_COLOR / FULL
"""
from .registry import (CUBES, COLOR_TO_ID, ID_TO_COLOR, HSV_PROFILES, TAG_TO_CUBE,
                       TRASH_TO_CUBE, hsv_ranges)

__all__ = ["CUBES", "COLOR_TO_ID", "ID_TO_COLOR", "HSV_PROFILES", "TAG_TO_CUBE",
           "TRASH_TO_CUBE", "hsv_ranges"]
