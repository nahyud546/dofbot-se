"""Planning-only MoveIt + camera/perception. Serial belongs to run_red_scene.py."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = Path(get_package_share_directory("cap_vision"))
    # Use the real five-axis model.  dofbot_fixed.urdf intentionally turns
    # arm5_Joint into a fixed joint for an older four-axis experiment; pairing
    # that file with dofbot.srdf makes MoveIt abort as soon as a request contains
    # arm5_Joint.  Perception/calibration needs the measured wrist rotation too.
    mc = (MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
          .robot_description(file_path="config/dofbot.urdf.xacro")
          .to_moveit_configs())
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/red_scene.yaml")),
        # Camera robot la USB camera (/dev/video2); video0/1 la webcam laptop.
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # fake_joints=true: hien thi RViz khi chua co tay that.
        # Co tay that: fake_joints:=false real_joints:=true de RViz theo servo that.
        DeclareLaunchArgument("fake_joints", default_value="true"),
        DeclareLaunchArgument("real_joints", default_value="false"),
        Node(package="cap_vision", executable="fake_real_joints",
             name="fake_real_joints", output="log",
             condition=IfCondition(LaunchConfiguration("fake_joints"))),
        Node(package="cap_vision", executable="real_joint_mirror",
             name="real_joint_mirror", output="screen",
             condition=IfCondition(LaunchConfiguration("real_joints"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="red_scene_planning_state", parameters=[mc.robot_description],
             remappings=[("joint_states", "/real_joint_states")]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="red_scene_world",
             arguments=["--frame-id", "world", "--child-frame-id", "base_link"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[mc.to_dict(), {"allow_trajectory_execution": False,
                 "publish_robot_description_semantic": True,
                 "publish_planning_scene": True, "publish_geometry_updates": True,
                 "publish_state_updates": True, "publish_transforms_updates": True}],
             remappings=[("joint_states", "/real_joint_states")]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/red_scene.launch.py")),
             launch_arguments={"config": LaunchConfiguration("config"),
                               "device_index": LaunchConfiguration("device_index"),
                               "start_camera": LaunchConfiguration("start_camera")}.items()),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(share / "config/red_scene.rviz")],
             parameters=[mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics]),
    ])
