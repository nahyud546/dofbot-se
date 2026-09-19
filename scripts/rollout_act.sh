#!/usr/bin/env bash
# Rollout policy đã train lên robot (sim hoặc thật). NGUY HIỂM trên tay thật:
# luôn test mock/sim trước, để arm_lib_time_ms cao + người cầm nút dừng.
set -e
POLICY_PATH="${1:?dùng: rollout_act.sh <policy_path> [backend] [episodes]}"
BACKEND="${2:-sim_ros2}"
EPISODES="${3:-5}"
echo "[rollout] policy=$POLICY_PATH backend=$BACKEND episodes=$EPISODES"
lerobot-rollout \
  --robot.type=dofbot_follower \
  --robot.backend="$BACKEND" \
  --policy.path="$POLICY_PATH" \
  --dataset.num_episodes="$EPISODES"
