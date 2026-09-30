"""Run calibrated instance perception with the existing planning-only stack.

The default empty segmentation weights path intentionally yields no poses.
Supply a trained model and measured calibration/model files to enable 6D pose.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("cap_vision"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/red_scene.yaml")),
        DeclareLaunchArgument("models", default_value=str(share / "config/object_models.yaml")),
        DeclareLaunchArgument("segmentation_weights", default_value=str(share / "config/yoloe-11s-seg.pt")),
        DeclareLaunchArgument("text_prompts", default_value="cube,toy block,colored cube"),
        DeclareLaunchArgument("visual_prompt_path", default_value=""),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("dino_enabled", default_value="true"),
        DeclareLaunchArgument("allow_tag_seed_fallback", default_value="true"),
        DeclareLaunchArgument("repo_root", default_value="/home/jloy/Desktop/robot-arm"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/red_scene_system.launch.py")),
            launch_arguments={"config": LaunchConfiguration("config"),
                              "start_camera": LaunchConfiguration("start_camera"),
                              "device_index": LaunchConfiguration("device_index"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
        Node(package="cap_vision", executable="object_perception",
             name="object_perception", output="screen",
              parameters=[{"config": LaunchConfiguration("config"),
                           "models": LaunchConfiguration("models"),
                           "model_path": LaunchConfiguration("segmentation_weights"),
                           "text_prompts": LaunchConfiguration("text_prompts"),
                           "visual_prompt_path": LaunchConfiguration("visual_prompt_path"),
                          "dino_enabled": ParameterValue(LaunchConfiguration("dino_enabled"), value_type=bool),
                          "allow_tag_seed_fallback": ParameterValue(
                              LaunchConfiguration("allow_tag_seed_fallback"), value_type=bool),
                          "repo_root": LaunchConfiguration("repo_root")}]),
    ])
