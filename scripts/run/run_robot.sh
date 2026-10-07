#!/usr/bin/env bash
# run_robot.sh — bring-up tay máy chính (workspace ros/).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROBOT_ARM_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/setup/setup_env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/setup/setup_ros.sh"
echo "[INFO] launch robot bringup — thay lệnh dưới bằng launch file của bạn"
# ros2 launch dofbot_moveit demo.launch.py
