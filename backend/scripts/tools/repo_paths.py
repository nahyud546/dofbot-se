"""Central repo-root resolver (ROBOT_ARM_ROOT + fallback).

Priority:
  1. $ROBOT_ARM_ROOT env (set by scripts/setup/setup_env.sh)
  2. walk up from this file looking for marker dirs (workspaces/, vendor/, config/)
  3. legacy fallbacks: /home/jloy/Desktop/robot-arm, /home/yahboom (read-only compat)

Usage:
    from repo_paths import ROBOT_ARM_ROOT, repo_path, first_existing
    MODEL = first_existing([
        repo_path("ai/models/detection/yolov8n-face.pt"),
        "/home/jloy/Desktop/robot-arm/models/yolov8n-face.pt",
        "/home/jloy/Desktop/robot-arm/ai/models/detection/yolov8n-face.pt",
    ])
"""
from __future__ import annotations

import os
from pathlib import Path

_HERE = Path(__file__).resolve()
# scripts/tools/repo_paths.py -> repo root is parents[2]
_CANDIDATE_ROOT = _HERE.parents[2] if len(_HERE.parents) >= 3 else _HERE.parent

_LEGACY_ROOTS = [
    Path("/home/jloy/Desktop/robot-arm"),
    Path("/home/yahboom"),
]

_MARKERS = ("workspaces", "vendor", "config", "projects")


def _walk_up(start: Path) -> Path | None:
    for p in [start, *_HERE.parents]:
        if all((p / m).exists() for m in ("workspaces", "config")):
            return p
        if any((p / m).exists() for m in _MARKERS) and (p / "scripts").exists():
            return p
    return None


def resolve_root() -> Path:
    env = os.environ.get("ROBOT_ARM_ROOT")
    if env and (Path(env) / "projects").is_dir():          # biến cũ trỏ sai chỗ (repo đã dời) thì không tin
        return Path(env).resolve()
    w = _walk_up(_CANDIDATE_ROOT)
    if w:
        return w.resolve()
    if _CANDIDATE_ROOT.exists():
        return _CANDIDATE_ROOT.resolve()
    for legacy in _LEGACY_ROOTS:
        if legacy.exists():
            return legacy.resolve()
    return _CANDIDATE_ROOT.resolve()


ROBOT_ARM_ROOT: Path = resolve_root()


# Workspace ROS của repo (cap_vision, cap_scene_interfaces, dofbot_moveit); $ROBOT_ARM_ROS_WS ghi đè.
ROS_WS: Path = Path(os.environ.get("ROBOT_ARM_ROS_WS") or ROBOT_ARM_ROOT / "ros")
ROS_SETUP: Path = ROS_WS / "install" / "setup.bash"


def repo_path(*parts: str) -> Path:
    """Path under the new layout: repo_path('ai','models',...)."""
    return ROBOT_ARM_ROOT.joinpath(*parts)


def first_existing(candidates: list[str | Path]) -> Path | None:
    """Return first candidate that exists, else None (caller decides fallback)."""
    for c in candidates:
        p = Path(c)
        if not p.is_absolute():
            p = ROBOT_ARM_ROOT / p
        if p.exists():
            return p
    return None


def require(candidates: list[str | Path], what: str = "file") -> Path:
    p = first_existing(candidates)
    if p is None:
        raise FileNotFoundError(
            f"Missing {what}. Tried: {candidates} (ROBOT_ARM_ROOT={ROBOT_ARM_ROOT})"
        )
    return p
