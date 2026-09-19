#!/usr/bin/env bash
# Record dataset cho Dofbot. Chạy trong venv lerobot (py3.12).
# Backend ROS sim phải chạy sẵn ở venv ROS khác (tea_backend / simplify_backend).
set -e
TASK="${1:-cube_pick_place}"
BACKEND="${2:-sim_ros2}"   # sim_ros2 | arm_lib | mock
EPISODES="${3:-5}"
REPO_ID="${4:-local/dofbot_${TASK}}"
MOCK_FLAG=""
if [ "$BACKEND" = "mock" ]; then MOCK_FLAG="--robot.mock=true"; fi
echo "[record] task=$TASK backend=$BACKEND episodes=$EPISODES repo=$REPO_ID"
lerobot-record \
  --robot.type=dofbot_follower \
  --robot.backend="$BACKEND" \
  $MOCK_FLAG \
  --teleop.type=keyboard \
  --dataset.repo_id="$REPO_ID" \
  --dataset.num_episodes="$EPISODES" \
  --dataset.single_task="Dofbot $TASK"
