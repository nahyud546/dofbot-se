# lerobot-robot-dofbot

LeRobot plugin cho Yahboom Dofbot 6DOF (5 arm joints + 1 gripper).

## Cài đặt

```bash
# venv lerobot (python >=3.12, theo lerobot v0.6.2)
uv venv --python 3.12 /tmp/lerobot-dofbot
source /tmp/lerobot-dofbot/bin/activate
pip install -e /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof/lerobot_plugins/lerobot_robot_dofbot
# kiểm tra discovery
python -c "from lerobot_robot_dofbot import DofbotFollower; print(DofbotFollower.name)"
```

Không cần fork repo lerobot gốc: discovery qua prefix `lerobot_robot_*` +
`@RobotConfig.register_subclass("dofbot_follower")`.

## Dùng

```bash
# Sim (cần ROS Humble chạy backend ở terminal khác)
lerobot-record --robot.type=dofbot_follower --robot.backend=sim_ros2 \
  --robot.mock=false --dataset.repo_id=local/dofbot_test --dataset.num_episodes=2

# Tay thật (chạy trong venv có Arm_Lib, thường là image Yahboom py3.10)
lerobot-record --robot.type=dofbot_follower --robot.backend=arm_lib \
  --dataset.repo_id=local/dofbot_real --dataset.num_episodes=5

# Test không hardware
lerobot-record --robot.type=dofbot_follower --robot.mock=true \
  --dataset.repo_id=local/dofbot_mock --dataset.num_episodes=1
```

## Vì sao 2 venv?

- `lerobot>=0.6.2` yêu cầu `python>=3.12`.
- ROS Humble lock `python3.10` (+ `rclpy`, `Arm_Lib-0.0.5` cũng chỉ có wheel py3.10).

Giữ 2 venv: `venv-ros` (py3.10, chạy `tea_backend`/`simplify_backend`) +
`venv-lerobot` (py3.12, record/train/rollout). Plugin `sim_ros2` nói chuyện qua
topics (`/joint_states`, `/arm_group_controller/follow_joint_trajectory`),
không import rclpy trong venv lerobot khi chưa cần.
