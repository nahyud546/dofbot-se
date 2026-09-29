#!/usr/bin/env bash
# clean_workspace.sh — remove colcon artifacts for a fresh rebuild.
# Usage: scripts/tools/clean_workspace.sh [all|dofbot_robot_arm_6dof|dofbot_ws|LargeModel_ws]
set -eu
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${1:-all}"
clean_one() {
  rm -rf "$1/build" "$1/install" "$1/log"
  echo "[OK] cleaned $1"
}
if [ "$TARGET" = "all" ]; then
  for _ws in dofbot_robot_arm_6dof dofbot_ws LargeModel_ws; do
    clean_one "$_ROOT/workspaces/$_ws"
  done
else
  clean_one "$_ROOT/workspaces/$TARGET"
fi
