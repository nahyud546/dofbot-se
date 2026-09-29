#!/usr/bin/env bash
# setup_env.sh — single entry to export ROBOT_ARM_ROOT + PYTHONPATH for the new layout.
# Usage: source scripts/setup/setup_env.sh
set -u
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROBOT_ARM_ROOT="${ROBOT_ARM_ROOT:-$_ROOT}"
export PROJECT="$ROBOT_ARM_ROOT"

# Arm_Lib (Yahboom serial) lives in vendor + .venv; vendor first so ROS never shadows it.
for _p in \
  "$ROBOT_ARM_ROOT/vendor/yahboom/Dofbot/0.py_install" \
  "$ROBOT_ARM_ROOT/vendor/yahboom/rosmaster" \
  "$ROBOT_ARM_ROOT/scripts/tools" \
  "$ROBOT_ARM_ROOT/projects/vision_experiments" \
  "$ROBOT_ARM_ROOT/projects/t8_pipeline" \
  ; do
  case ":${PYTHONPATH:-}:" in
    *":$_p:"*) ;;
    *) export PYTHONPATH="$_p:${PYTHONPATH:-}" ;;
  esac
done
echo "[OK] ROBOT_ARM_ROOT=$ROBOT_ARM_ROOT"
echo "[OK] PYTHONPATH=$PYTHONPATH"
