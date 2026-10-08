#!/usr/bin/env bash
# run_moveit.sh — MoveIt demo cho dofbot (rviz + planning).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROBOT_ARM_ROOT="$ROOT"
# shellcheck disable=SC1091
source "$ROOT/scripts/setup/setup_env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/setup/setup_ros.sh"
ros2 launch dofbot_moveit demo.launch.py
