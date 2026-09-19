"""Hằng số bàn cờ. CHỐT theo chess_utils.py (bàn thật 24cm).

Mapping: X=rank, Y=file (hàng 1 gần robot). Mọi visualizer/planner mới phải
import từ đây, không hard-code lại (0.1105, -0.0915).
"""

BOARD_SIZE_X = 0.240
BOARD_SIZE_Y = 0.240
BOARD_THICKNESS = 0.020
BOARD_TOP_Z = 0.005
BOARD_CENTER = (0.2015, -0.0005)
BOARD_ORIGIN = (0.1105, -0.0915, 0.005)  # tâm ô a1
SQUARE_SIZE = 0.026
FILES = "abcdefgh"


def square_xy(square: str) -> tuple[float, float]:
    """Tâm ô (x, y) theo mapping X=rank, Y=file."""
    s = square.lower()
    file_index = FILES.index(s[0])
    rank = int(s[1]) - 1
    return (BOARD_ORIGIN[0] + rank * SQUARE_SIZE, BOARD_ORIGIN[1] + file_index * SQUARE_SIZE)
