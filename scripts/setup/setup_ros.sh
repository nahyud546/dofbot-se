#!/usr/bin/env bash
# setup_ros.sh — source ROS Humble + all built workspaces in dependency order.
# Usage: source scripts/setup/setup_env.sh && source scripts/setup/setup_ros.sh
set -u
_WS_ORDER=(
  "$ROBOT_ARM_ROOT/workspaces/dofbot_ws"
  "$ROBOT_ARM_ROOT/workspaces/LargeModel_ws"
  "${ROBOT_ARM_ROS_WS:-$ROBOT_ARM_ROOT/ros}"
)
if [ -f /opt/ros/humble/setup.bash ]; then
  # ROS setup scripts reference unbound vars -> relax set -u temporarily.
  set +u; # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash; set -u
  echo "[OK] sourced /opt/ros/humble"
else
  echo "[WARN] /opt/ros/humble/setup.bash not found"
fi
for _ws in "${_WS_ORDER[@]}"; do
  if [ -f "$_ws/install/setup.bash" ]; then
    set +u; # shellcheck disable=SC1091
    source "$_ws/install/setup.bash"; set -u
    echo "[OK] sourced $_ws/install/setup.bash"
  else
    echo "[SKIP] $_ws/install/setup.bash (run colcon build first)"
  fi
done
# NOTE: intentionally NEVER auto-source workspaces/legacy/colcon_ws (Arm_Lib shadow risk).
