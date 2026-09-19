# Kiến trúc sau tái cấu trúc

## Vấn đề cũ (đã xác nhận qua khảo sát)
1. 3 pipeline không tái dùng: `chess_sim` (Python pymoveit2 OMPL+Cartesian 7.8k dòng)
   vs `tea` (C++ MoveGroup 2.5k dòng) vs `simplify` (replay FollowJointTrajectory).
   Chung robot/link/controller nhưng không có lib chung.
2. Launch fork: `tea_backend` hand-roll lại `demo.launch.py`; `simplify_*` copy
   `_acquire_lock/_refuse/_write_urdf` x3. Đổi controller phải nhớ 4 chỗ.
3. Hằng số bàn cờ hard-fork: `BOARD_ORIGIN=(0.1105,-0.0915)` ở `chess_utils.py`,
   `board_visualizer.py`, `offline_ik_calibrate.py`; spec gripper lệch nhau.
4. Luồng chính bị chôn: phải đọc 4 README + launch + grep `/chess/|/tea/`.
5. `src/simplify_chess_game` là symlink -> `../simplify-chess-game` (không phải
   duplicate, giữ nguyên để colcon build được). `setup_standalone.sh` cũ quên
   build package này.

## Layout mới (giữ src/ nguyên, thêm lớp AI/hardware)
```
src/                        # ROS2/MoveIt2 hiện tại (không di chuyển để khỏi vỡ build)
  dofbot_common/            # NEW: single source of truth (joints, TCP, board)
  chess_moveit_demo/        # cờ full MoveIt (Stockfish + pick_place_node)
  dofbot_tea_moveit/        # rót trà C++
  dofbot_moveit/            # config MoveIt vendored (demo.launch.py là gốc)
  dofbot_urdf/              # meshes Yahboom (gitignore, ~104M)
  pymoveit2/                # wrapper Python (vendor + patch no-spin)
  simplify_chess_game -> ../simplify-chess-game
simplify-chess-game/        # replay joint-space, 64 routes VALIDATED
lerobot_plugins/lerobot_robot_dofbot/  # NEW: plugin LeRobot (sim_ros2|arm_lib|mock)
tasks/{cube_pick_place,cap_sorting,chess}/  # NEW: định nghĩa task độc lập backend
ai/{configs,experiments,policies,checkpoints}/  # NEW: train/eval, không vendor policy
scripts/{record_dataset,train_act,rollout_act}.sh  # NEW: wrapper lerobot-* CLI
hardware/README.md          # NEW: ghi chú Arm_Lib + an toàn tay thật
datasets/ outputs/          # NEW: gitignore (dataset, log rollout)
docs/ARCHITECTURE.md        # file này
```

## Luồng chính (đọc 1 hình là chạy được)
- Chess base (test luật): `chess_base.launch.py` -> `/chess/start` -> `/chess/move` -> ACK.
- Chess sim (MoveIt): `chess_sim.launch.py` (include `demo.launch.py`) -> `pick_place_node`
  dựng scene 33 objects -> READY `/chess/system_ready` -> `chess_brain_node` cho start.
- Tea: `tea_backend.launch.py` (T1) -> `tea_task.launch.py` (T2) -> `/tea/next` từng bước.
- Simplify: `simplify_chess.launch.py` -> `auto_play`/`chess_cli` (ngoài launch).
- LeRobot mới: `scripts/record_dataset.sh` (sim_ros2/arm_lib/mock) -> `train_act.sh`
  -> `rollout_act.sh`. Task đọc `tasks/<name>/task.yaml`.

## Quy ước
- Hằng số mới viết vào `src/dofbot_common/`, không hard-code lại. Code cũ dần
  chuyển sang import từ đó (hiện đã có `robot_constants.py`, `board_constants.py`).
- Thêm task = copy `tasks/cube_pick_place/`, không copy package ROS.
- Policy: dùng sẵn của lerobot (`act`, `diffusion`...), không copy vào `ai/policies/`.
- 2 venv bắt buộc: `venv-ros` py3.10 (ROS Humble + Arm_Lib) và `venv-lerobot` py3.12
  (lerobot>=0.6.2). Giao nhau qua ROS topics, không hạ lerobot về 3.10.
