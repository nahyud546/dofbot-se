"""Camera + cube 6D perception only; no MoveIt, RViz, or robot control."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # This launch file is installed as a symlink to the source tree. Resolve
    # its own package instead of the first cap_vision entry in AMENT_PREFIX_PATH:
    # another (older) workspace can otherwise start a node with missing config.
    resolved = Path(__file__).resolve()
    package_source = resolved.parents[1]
    if (package_source / "setup.py").is_file():
        # Symlink-install: __file__ resolves into src/cap_vision.
        share = package_source
        workspace = package_source.parents[1]
    else:
        # Regular install: __file__ is install/cap_vision/share/cap_vision/launch.
        share = resolved.parent.parent
        # share=.../install/cap_vision/share/cap_vision -> workspace root.
        workspace = share.parents[3]
    local_bin = workspace / "install/cap_vision/lib/cap_vision"
    # Fall back to the install prefix layout when workspace lookup fails
    # (e.g. overlay/symlink edge cases).
    if not (local_bin / "object_perception").is_file():
        fallback = share.parents[1] / "lib/cap_vision"
        if (fallback / "object_perception").is_file():
            local_bin = fallback
    local_python_env = {
        "PYTHONPATH": f"{workspace / 'build/cap_vision'}:{os.environ.get('PYTHONPATH', '')}"
    }
    if not (local_bin / "object_perception").is_file():
        raise RuntimeError(f"Build cap_vision in {workspace} before launching cube_6d_camera")
    for required in ("cube_6d_urdf.yaml", "cube_color_hsv.yaml", "object_models.yaml"):
        if not (share / "config" / required).is_file():
            raise RuntimeError(f"Missing cap_vision config: {share / 'config' / required}")
    # Repo root: $ROBOT_ARM_ROOT, else the directory holding this workspace (<repo>/ros).
    repo_root = Path(os.environ.get("ROBOT_ARM_ROOT") or workspace.parent)
    if not (repo_root / "projects").is_dir():
        raise RuntimeError(f"Không tìm thấy repo root (thiếu {repo_root}/projects); "
                           "đặt ROBOT_ARM_ROOT hoặc source scripts/setup/setup_env.sh")
    # Model weights are not tracked in git (*.pt): package config first, then ai/models/segmentation.
    weights = next((path for path in (share / "config/yoloe-11s-seg.pt",
                                      repo_root / "ai/models/segmentation/yoloe-11s-seg.pt")
                    if path.is_file()), None)
    if weights is None:
        raise RuntimeError("Thiếu yoloe-11s-seg.pt: đặt vào "
                           f"{repo_root / 'ai/models/segmentation'} (xem CLAUDE.md, mục mô hình)")
    robot_description = (MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
                         .robot_description(file_path="config/dofbot.urdf.xacro")
                         .to_moveit_configs().robot_description)
    return LaunchDescription([
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("repo_root", default_value=str(repo_root)),
        DeclareLaunchArgument("config", default_value=str(share / "config/cube_6d_urdf.yaml")),
        DeclareLaunchArgument("models", default_value=str(share / "config/object_models.yaml")),
        DeclareLaunchArgument("color_hsv_config", default_value=str(share / "config/cube_color_hsv.yaml")),
        DeclareLaunchArgument("face_geometry", default_value=PathJoinSubstitution([
            LaunchConfiguration("repo_root"), "config/robot/cube_4x6_face_geometry.yaml"])),
        DeclareLaunchArgument("segmentation_weights",
                              default_value=str(weights)),
        DeclareLaunchArgument("dino_enabled", default_value="true"),
        DeclareLaunchArgument("dino_threshold", default_value="0.35"),
        DeclareLaunchArgument("segmentation_confidence", default_value="0.2"),
        DeclareLaunchArgument("provisional_uncalibrated_base", default_value="false"),
        DeclareLaunchArgument("smoothing_preset", default_value="normal"),
        DeclareLaunchArgument("display_smoothing", default_value="true"),
        DeclareLaunchArgument("overlay_debug", default_value="false"),
        DeclareLaunchArgument("viewer", default_value="false"),
        # The camera is eye-in-hand. Measured servo joints + RSP supply the
        # timestamped base_link -> measured/Camera_Link transform.
        Node(executable=str(local_bin / "real_joint_mirror"),
             name="cube_6d_real_joints", output="screen",
             additional_env=local_python_env),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="cube_6d_robot_state", output="screen",
             parameters=[robot_description, {"frame_prefix": "measured/",
                                            "publish_frequency": 30.0}],
             remappings=[("joint_states", "/real_joint_states"),
                         ("robot_description", "/cube_6d_robot_description")]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="cube_6d_base", output="screen",
             arguments=["--frame-id", "base_link", "--child-frame-id", "measured/base_link"]),
        Node(
            executable=str(local_bin / "camera_test"), name="cube_6d_camera",
            output="screen",
            additional_env=local_python_env,
            parameters=[{
                "device_index": ParameterValue(LaunchConfiguration("device_index"),
                                               value_type=int),
                "width": 640, "height": 480, "fps": 15.0,
                "frame_id": "measured/camera_optical_frame",
                "show": False,
            }],
        ),
        Node(
            executable=str(local_bin / "object_perception"),
            name="object_perception", output="screen",
            additional_env=local_python_env,
            parameters=[{
                "config": LaunchConfiguration("config"),
                "models": LaunchConfiguration("models"),
                "color_hsv_config": LaunchConfiguration("color_hsv_config"),
                "face_geometry": LaunchConfiguration("face_geometry"),
                "face_mapping_verified": True,
                "model_path": LaunchConfiguration("segmentation_weights"),
                "mask_confidence": ParameterValue(
                    LaunchConfiguration("segmentation_confidence"), value_type=float),
                "text_prompts": "cube,toy block,colored cube",
                "dino_enabled": ParameterValue(
                    LaunchConfiguration("dino_enabled"), value_type=bool),
                "dino_threshold": ParameterValue(
                    LaunchConfiguration("dino_threshold"), value_type=float),
                "allow_tag_seed_fallback": True,
                "provisional_uncalibrated_base": ParameterValue(
                    LaunchConfiguration("provisional_uncalibrated_base"),
                    value_type=bool),
                "smoothing_preset": LaunchConfiguration("smoothing_preset"),
                "display_smoothing": ParameterValue(
                    LaunchConfiguration("display_smoothing"), value_type=bool),
                "overlay_debug": ParameterValue(
                    LaunchConfiguration("overlay_debug"), value_type=bool),
                "repo_root": LaunchConfiguration("repo_root"),
            }],
        ),
        Node(
            package="rqt_image_view", executable="rqt_image_view",
            name="cube_6d_viewer", output="screen",
            condition=IfCondition(LaunchConfiguration("viewer")),
            arguments=["/vision/object_annotated"],
        ),
    ])
