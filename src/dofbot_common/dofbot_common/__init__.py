"""Single source of truth cho toàn workspace.

Trước đây BOARD_ORIGIN/SQUARE_SIZE copy ở 3 nơi (chess_utils.py,
board_visualizer.py, offline_ik_calibrate.py) và đã lệch nhau. Từ nay mọi
demo + plugin LeRobot import từ đây. File cũ giữ lại để tương thích nhưng
nên re-export từ module này khi sửa tiếp.
"""

from dofbot_common.board_constants import (
    BOARD_CENTER,
    BOARD_ORIGIN,
    BOARD_THICKNESS,
    BOARD_TOP_Z,
    SQUARE_SIZE,
    square_xy,
)
from dofbot_common.robot_constants import (
    ARM_JOINTS,
    GRIPPER_JOINT,
    TCP_LINK,
    ARM_ACTION_TOPIC,
    GRIP_ACTION_TOPIC,
    JOINT_STATES_TOPIC,
    HOME_JOINTS_RAD,
)

__all__ = [
    "ARM_JOINTS",
    "GRIPPER_JOINT",
    "TCP_LINK",
    "ARM_ACTION_TOPIC",
    "GRIP_ACTION_TOPIC",
    "JOINT_STATES_TOPIC",
    "HOME_JOINTS_RAD",
    "BOARD_CENTER",
    "BOARD_ORIGIN",
    "BOARD_THICKNESS",
    "BOARD_TOP_Z",
    "SQUARE_SIZE",
    "square_xy",
]
