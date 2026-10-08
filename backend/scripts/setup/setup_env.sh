#!/usr/bin/env bash
# setup_env.sh — single entry to export ROBOT_ARM_ROOT + PYTHONPATH for the new layout.
# Usage: source scripts/setup/setup_env.sh
set -u
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Terminal cũ có thể còn ROBOT_ARM_ROOT trỏ về chỗ cũ (trước khi repo chuyển vào backend/): chỗ đó không có
# projects/ nên mọi thứ tìm sai đường (không import được cube_vision, ghi data ra ngoài). Khi đó dùng lại _ROOT.
if [ -n "${ROBOT_ARM_ROOT:-}" ] && [ ! -d "$ROBOT_ARM_ROOT/projects" ]; then
  echo "[WARN] ROBOT_ARM_ROOT=$ROBOT_ARM_ROOT không có projects/ (biến cũ): đổi sang $_ROOT"
  # bỏ các đường dẫn cũ khỏi PYTHONPATH
  PYTHONPATH="$(printf '%s' "${PYTHONPATH:-}" | tr ':' '\n' | grep -v -x -e "$ROBOT_ARM_ROOT/projects.*" \
    -e "$ROBOT_ARM_ROOT/scripts/tools" -e "$ROBOT_ARM_ROOT/vendor/.*" | paste -sd: -)"
  export PYTHONPATH
  unset ROBOT_ARM_ROOT
fi
export ROBOT_ARM_ROOT="${ROBOT_ARM_ROOT:-$_ROOT}"
export PROJECT="$ROBOT_ARM_ROOT"

# Arm_Lib (Yahboom serial) lives in vendor + .venv; vendor first so ROS never shadows it.
# vendor/yahboom exposes namespace package `dofbot_voice.scripts` (no __init__.py, PEP 420).
for _p in \
  "$ROBOT_ARM_ROOT/vendor/yahboom/Dofbot/0.py_install" \
  "$ROBOT_ARM_ROOT/vendor/yahboom/rosmaster" \
  "$ROBOT_ARM_ROOT/vendor/yahboom" \
  "$ROBOT_ARM_ROOT/scripts/tools" \
  "$ROBOT_ARM_ROOT/projects/vision_experiments" \
  "$ROBOT_ARM_ROOT/projects/t8_pipeline" \
  "$ROBOT_ARM_ROOT/projects" \
  ; do
  case ":${PYTHONPATH:-}:" in
    *":$_p:"*) ;;
    *) export PYTHONPATH="$_p:${PYTHONPATH:-}" ;;
  esac
done
echo "[OK] ROBOT_ARM_ROOT=$ROBOT_ARM_ROOT"
echo "[OK] PYTHONPATH=$PYTHONPATH"
