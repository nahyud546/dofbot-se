"""Một chỗ duy nhất thêm thư mục của repo vào sys.path cho import muộn (Arm_Lib, module trần của
vision_experiments/t8_pipeline). Module thư viện gọi `ensure("vendor")` thay vì tự chèn sys.path."""
from __future__ import annotations

import sys

from .registry import repo_root

_DIRS = {
    "projects": "projects",
    "vision": "projects/vision_experiments",
    "t8": "projects/t8_pipeline",
    "vendor": "vendor/yahboom",
    "tools": "scripts/tools",
}


def ensure(*names: str) -> None:
    """Đưa các thư mục (theo khóa trong _DIRS) lên sys.path nếu chưa có."""
    root = repo_root()
    for name in names:
        path = str(root / _DIRS[name])
        if path not in sys.path:
            sys.path.insert(0, path)
