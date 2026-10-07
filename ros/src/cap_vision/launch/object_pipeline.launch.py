"""Run calibrated instance perception with the existing planning-only stack.

The default empty segmentation weights path intentionally yields no poses.
Supply a trained model and measured calibration/model files to enable 6D pose.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("cap_vision"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/red_scene.yaml")),
        DeclareLaunchArgument("models", default_value=str(share / "config/object_models.yaml")),
        DeclareLaunchArgument("color_hsv_config", default_value=str(share / "config/cube_color_hsv.yaml")),
        DeclareLaunchArgument("repo_root", default_value=os.environ.get(
            "ROBOT_ARM_ROOT", str(Path(__file__).resolve().parents[4]))),
        DeclareLaunchArgument("face_geometry", default_value=PathJoinSubstitution([
            LaunchConfiguration("repo_root"), "config/robot/cube_4x6_face_geometry.yaml"])),
        # The repository geometry file is the declared physical face mapping.
        # Set false only when testing a different, unverified cube build.
        DeclareLaunchArgument("face_mapping_verified", default_value="true"),
        DeclareLaunchArgument("segmentation_weights", default_value=str(share / "config/yolov8n-seg.pt")),
        DeclareLaunchArgument("text_prompts", default_value="cube,toy block,colored cube"),
        DeclareLaunchArgument("visual_prompt_path", default_value=""),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fake_joints", default_value="true"),
        DeclareLaunchArgument("real_joints", default_value="false"),
        DeclareLaunchArgument("dino_enabled", default_value="true"),
        DeclareLaunchArgument("allow_tag_seed_fallback", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/red_scene_system.launch.py")),
            launch_arguments={"config": LaunchConfiguration("config"),
                              "start_camera": LaunchConfiguration("start_camera"),
                              "device_index": LaunchConfiguration("device_index"),
                              "fake_joints": LaunchConfiguration("fake_joints"),
                              "real_joints": LaunchConfiguration("real_joints"),
                              "rviz": LaunchConfiguration("rviz")}.items()),
        Node(package="cap_vision", executable="object_perception",
             name="object_perception", output="screen",
              parameters=[{"config": LaunchConfiguration("config"),
                           "models": LaunchConfiguration("models"),
                           "color_hsv_config": LaunchConfiguration("color_hsv_config"),
                           "face_geometry": LaunchConfiguration("face_geometry"),
                           "face_mapping_verified": ParameterValue(
                               LaunchConfiguration("face_mapping_verified"), value_type=bool),
                           "model_path": LaunchConfiguration("segmentation_weights"),
                           "text_prompts": LaunchConfiguration("text_prompts"),
                           "visual_prompt_path": LaunchConfiguration("visual_prompt_path"),
                          "dino_enabled": ParameterValue(LaunchConfiguration("dino_enabled"), value_type=bool),
                          "allow_tag_seed_fallback": ParameterValue(
                              LaunchConfiguration("allow_tag_seed_fallback"), value_type=bool),
                          "repo_root": LaunchConfiguration("repo_root")}]),
    ])
