"""RViz xem world: mô hình tay + các hệ tọa độ + camera + cube từ data/world/latest.json.

    ros2 launch cap_vision world_view.launch.py                       # tay vẽ ở pose READY (không cần cắm tay máy)
    ros2 launch cap_vision world_view.launch.py live_joints:=true     # khớp thật từ /real_joint_states
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    resolved = Path(__file__).resolve()
    share = resolved.parents[1]
    robot_description = (MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
                         .robot_description(file_path="config/dofbot.urdf.xacro")
                         .to_moveit_configs().robot_description)
    live = LaunchConfiguration("live_joints")
    return LaunchDescription([
        DeclareLaunchArgument("live_joints", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="cap_vision", executable="world_publisher", name="world_publisher", output="screen",
             parameters=[{"static_joints": True}], condition=UnlessCondition(live)),
        Node(package="cap_vision", executable="world_publisher", name="world_publisher", output="screen",
             parameters=[{"static_joints": False}], condition=IfCondition(live)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="world_robot_state",
             output="screen", parameters=[robot_description], condition=UnlessCondition(live)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="world_robot_state",
             output="screen", parameters=[robot_description],
             remappings=[("joint_states", "/real_joint_states")], condition=IfCondition(live)),
        Node(package="rviz2", executable="rviz2", name="world_rviz", output="screen",
             arguments=["-d", str(share / "config" / "world.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
