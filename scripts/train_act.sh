#!/usr/bin/env bash
# Train ACT baseline. Chạy trong venv lerobot (py3.12, có GPU càng tốt).
set -e
REPO_ID="${1:-local/dofbot_cube}"
OUT="${2:-ai/checkpoints/act_$(date +%F_%H%M)}"
echo "[train] dataset=$REPO_ID out=$OUT"
lerobot-train \
  --dataset.repo_id="$REPO_ID" \
  --policy.type=act \
  --output_dir="$OUT" \
  --job_name=dofbot_act
echo "[train] xong -> $OUT . Ghi chép vào ai/experiments/ ."
