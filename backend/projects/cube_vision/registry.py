"""Nguồn duy nhất cho định danh cube và các profile HSV.

Trước đây bảng ID↔màu và HSV bị chép ở 4 nơi và đã lệch nhau (search-center
đang map vàng=1/xanh dương=4 trong khi T8 và bảng cube là xanh dương=1/vàng=4).
Ngưỡng HSV KHÔNG giống nhau giữa các nơi một cách có chủ đích (mỗi nơi tinh chỉnh
cho ngữ cảnh riêng), nên chúng là các profile có tên, không bị gộp thành một:

    identify  - identify_cube.py (ngưỡng chặt hơn, dùng cho nhận diện mặt trên)
    proven    - t8_vision.py + cube_search_center.py (đã chạy tốt ngoài thực tế;
                xanh dương S>=150 để loại bóng xanh nhạt của bàn)
    runtime   - cap_vision/config/cube_color_hsv.yaml (node perception ROS)
"""
from __future__ import annotations

import os
from pathlib import Path

CUBES = {
    1: {"name": "blue", "color": "khoi_xanh_duong", "tag_id": 1,
        "trash": {"newspaper", "zip_top_can", "book", "old_school_bag"}},
    2: {"name": "green", "color": "khoi_xanh", "tag_id": 2,
        "trash": {"fish_bone", "egg_shell", "apple_core", "watermelon_rind"}},
    3: {"name": "red", "color": "khoi_do", "tag_id": 3,
        "trash": {"syringe", "expired_cosmetics", "used_batteries", "expired_tablets"}},
    4: {"name": "yellow", "color": "khoi_vang", "tag_id": 4,
        "trash": {"toilet_paper", "peach_pit", "cigarette_butts", "disposable_chopsticks"}},
}

TAG_TO_CUBE = {spec["tag_id"]: cid for cid, spec in CUBES.items()}
TRASH_TO_CUBE = {cls: cid for cid, spec in CUBES.items() for cls in spec["trash"]}
COLOR_TO_ID = {spec["color"]: cid for cid, spec in CUBES.items()}
ID_TO_COLOR = {cid: color for color, cid in COLOR_TO_ID.items()}

_RED_LOW, _RED_HIGH = ((0, 60, 25), (20, 255, 255)), ((165, 60, 25), (179, 255, 255))
HSV_PROFILES = {
    "identify": {
        "khoi_do": [((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (179, 255, 255))],
        "khoi_xanh": [((35, 70, 40), (85, 255, 255))],
        "khoi_xanh_duong": [((90, 70, 40), (130, 255, 255))],
        "khoi_vang": [((20, 70, 50), (35, 255, 255))],
    },
    "proven": {
        "khoi_do": [_RED_LOW, _RED_HIGH],
        "khoi_xanh": [((35, 35, 20), (95, 255, 255))],
        "khoi_xanh_duong": [((90, 150, 25), (135, 255, 255))],
        "khoi_vang": [((15, 45, 35), (45, 255, 255))],
    },
}

# Màu trong cube_color_hsv.yaml dùng tên tiếng Anh.
_YAML_NAME_TO_LABEL = {spec["name"]: spec["color"] for spec in CUBES.values()}


def repo_root() -> Path:
    env = os.environ.get("ROBOT_ARM_ROOT")
    if env and Path(env).exists():
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / "workspaces").exists():
            return parent
    return Path(__file__).resolve().parents[2]


def ros_workspace() -> Path:
    """Workspace ROS của repo (<repo>/ros: cap_vision, cap_scene_interfaces); $ROBOT_ARM_ROS_WS ghi đè."""
    env = os.environ.get("ROBOT_ARM_ROS_WS")
    return Path(env) if env else repo_root() / "ros"


def runtime_hsv_path() -> Path:
    return ros_workspace() / "src" / "cap_vision" / "config" / "cube_color_hsv.yaml"


def _runtime_profile():
    import yaml
    data = yaml.safe_load(runtime_hsv_path().read_text())["colors"]
    return {_YAML_NAME_TO_LABEL[name]: [(tuple(lo), tuple(hi)) for lo, hi in spec["ranges"]]
            for name, spec in data.items()}


def hsv_ranges(profile: str = "proven"):
    """{nhãn màu: [(lower, upper), ...]} của một profile có tên."""
    if profile == "runtime":
        return _runtime_profile()
    try:
        return HSV_PROFILES[profile]
    except KeyError:
        raise ValueError(f"profile HSV không tồn tại: {profile!r} "
                         f"(có: {sorted([*HSV_PROFILES, 'runtime'])})") from None
