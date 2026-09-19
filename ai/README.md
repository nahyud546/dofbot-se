# ai/ — mọi thứ liên quan model, KHÔNG trộn vào src/ (ROS).
# - configs/: tham số train/eval cho từng task (ACT trước vì nhẹ, hợp data ít).
# - experiments/: ghi chép từng lần chạy (ngày, dataset, ckpt, success_rate).
# - policies/: KHÔNG copy code policy từ lerobot vào đây; chỉ lưu wrapper/adapter nếu cần.
# - checkpoints/: weights sau train (gitignore, nặng).
# Chạy trong venv lerobot py3.12 (xem lerobot_plugins/lerobot_robot_dofbot/README.md).

# Ví dụ train ACT cube (50 ep mock):
#   lerobot-train --dataset.repo_id=local/dofbot_cube --policy.type=act \
#     --policy.config=configs/act_dofbot_cube.yaml --output_dir=checkpoints/act_cube_$(date +%F)
