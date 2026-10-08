# DOFBOT-SE — 6-DOF Yahboom DOFBOT on ROS 2 Humble

English overview of this monorepo: ROS 2 workspaces, vision/sorting skills,
an LLM task pipeline (T8), hand teleop, and curated Yahboom vendor sources —
all reorganized into one clean layout.

## Features

- **ROS 2 Humble workspaces** — `dofbot_ws` builds 13/13 ament packages
  (interfaces, driver, sorting, YOLOv11, MediaPipe, voice, MoveIt).
- **Vision skills** — color / shape / ArUco / AprilTag sorting, garbage
  sorting (YOLOv11), face/KCF follow experiments.
- **T8 LLM pipeline** (`projects/t8_pipeline`) — cost-smart planner:
  noise gate → local router → cache → Gemini → gated Tavily search,
  with motion/vision executors and a Vietnamese voice command set.
- **One entrypoint for env** — `scripts/setup/setup_env.sh` (PYTHONPATH)
  + `scripts/setup/setup_ros.sh` (ROS + workspace overlays).
- **Cube tasks on the real arm** — sort/stack any or all cubes from any start
  pose: wrist camera surveys the drop zones, the external camera verifies each
  placement (retry lower and slower if the cube bounced out), optional metric
  calibration of the external camera to the robot base.
- **Tests** — `/usr/bin/python3 -m pytest` from `backend/` (config in
  `pyproject.toml`); see `CLAUDE.md` for the working commands and gotchas.

## Repository layout

| Path | Contents |
| --- | --- |
| `workspaces/dofbot_ws/` | Main ROS 2 workspace (17 src entries; 13 ament packages built, 4 legacy ROS 1 catkin packages kept but skipped) |
| `workspaces/LargeModel_ws/` | LLM/arm-integration workspace (separate build, not part of default build) |
| `workspaces/legacy/` | Archived `colcon_ws` (ROS 1 / Arm_Lib era, never auto-sourced) |
| `ros/` | Perception workspace used by T8: `cap_vision`, `cap_scene_interfaces`, `dofbot_moveit` (+ `dofbot_urdf` symlink). Build with system Python (see `CLAUDE.md`) |
| `projects/cube_vision/` | Standalone vision library: tags, faces, zone survey, placement check, external-camera calibration, batch planning (no ROS/T8 imports) |
| `projects/t8_pipeline/` | T8 assistant: Gemini planner, executor, ROS scene worker, motion worker |
| `projects/vision_experiments/` | Geometry, hand-eye / external calibration scripts, standalone experiments |
| `config/` | Camera (`camera.yaml`, `homography.yaml`), robot (`joint_limits.yaml`, `poses.yaml`, `tcp.yaml`), udev rules |
| `scripts/setup/` | `setup_env.sh`, `setup_ros.sh`, `setup_serial.sh` |
| `scripts/run/` | `run_robot.sh`, `run_camera.sh`, `run_moveit.sh`, `run_yahboom.sh` |
| `scripts/tools/` | `check_serial.sh`, `clean_workspace.sh`, `repo_paths.py` |
| `vendor/yahboom/` | Curated upstream Yahboom sources (incl. `dofbot_voice`, `rosmaster`, `Dofbot`) |
| `ai/` | Experiments, models (git-ignored weights), datasets (ignored) |
| `docs/` | Architecture, calibration, MoveIt, teleop, troubleshooting |
| `archive/` | Old installers, old code/configs, old workspaces |
| `assets/` | Images and static assets |

> Ignored on purpose: `ai/lerobot/`, `workspaces/dofbot_robot_arm_6dof/`
> (external git checkouts; the perception packages T8 needs live in `ros/src`),
> all `build/`/`install/`/`log/`, `temp_imgs/`, `.local/`, model weights
> (`*.onnx`, `*.pt`, `*.engine`), datasets/captures, and credential files.

## Requirements

- Ubuntu 22.04, Python 3.10
- ROS 2 Humble (`/opt/ros/humble/setup.bash`)
- `colcon` (colcon-common-extensions)
- Serial device `/dev/ttyUSB0` for the real arm (optional for build/tests)

## Quickstart

```bash
# 1. Environment (run from backend/; root on PYTHONPATH, incl. dofbot_voice namespace)
source scripts/setup/setup_env.sh
source scripts/setup/setup_ros.sh

# 2. Build — interfaces first (dofbot_info needs dofbot_interface),
#    then the remaining ament packages
colcon --log-base workspaces/dofbot_ws/log build \
  --base-paths workspaces/dofbot_ws/src \
  --build-base workspaces/dofbot_ws/build \
  --install-base workspaces/dofbot_ws/install \
  --symlink-install \
  --packages-select dofbot_interface dofbot_msgs

source /opt/ros/humble/setup.bash
source workspaces/dofbot_ws/install/setup.bash
colcon --log-base workspaces/dofbot_ws/log build \
  --base-paths workspaces/dofbot_ws/src \
  --build-base workspaces/dofbot_ws/build \
  --install-base workspaces/dofbot_ws/install \
  --symlink-install \
  --packages-select dofbot_info dofbot_urdf dofbot_driver dofbot_follow \
    dofbot_mediapipe dofbot_sorting dofbot_sorting_3d dofbot_voice_ctrl \
    dofbot_yolov11 yahboom_speech dofbot_moveit
```

## Run & verify

```bash
# ROS overlay smoke test — expect 13 dofbot/yahboom packages
source /opt/ros/humble/setup.bash
source workspaces/dofbot_ws/install/setup.bash
ros2 pkg list | grep -E "dofbot|yahboom"
ros2 pkg executables dofbot_sorting_3d
ros2 interface list | grep -i dofbot

# MoveIt launch file parses (no GUI needed for this check)
ros2 launch dofbot_moveit demo.launch.py --show-args

# Python task pipeline — expect 52 passed
source scripts/setup/setup_env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest projects/t8_pipeline/test_t8_pipeline.py -q

# Convenience launchers
bash scripts/run/run_moveit.sh    # MoveIt demo (rviz + planning)
bash scripts/run/run_camera.sh    # cap_vision vision.launch.py
bash scripts/run/run_robot.sh     # robot bring-up placeholder
bash scripts/tools/check_serial.sh /dev/ttyUSB0  # STM32 reachability, no motion
```

## Notes & scope limits

- `workspaces/dofbot_ws/src/CMakeLists.txt` is a leftover ROS 1 catkin
  toplevel file — colcon warns and ignores it; harmless.
- `dofbot_color_identify`, `dofbot_color_stacking`, `dofbot_face_follow`,
  `dofbot_snake_follow` are `ros.catkin` (ROS 1) packages and are **not**
  part of the Humble build. Script-only dirs (`dofbot_apriltag`,
  `dofbot_color_follow`, …) have no `package.xml` and are used directly.
- `workspaces/LargeModel_ws` and `workspaces/legacy` are intentionally
  excluded from the default build and from `setup_ros.sh` auto-sourcing.
- `pytest` must run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` on machines
  with ROS-installed `launch_testing` pytest plugins (version conflict
  between old ROS hooks and pytest 9).
