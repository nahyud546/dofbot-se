"""Read-only perception + measured TF. No controller or hardware commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import xacro


def generate_launch_description():
    # The camera is mounted upstream of arm5, but the measured TF tree must
    # still accept and represent all five physical arm joints.  The historical
    # dofbot_fixed.urdf deliberately removes arm5 and is not a hardware model.
    robot_urdf = (Path(get_package_share_directory("dofbot_moveit")) /
                  "config/dofbot.urdf.xacro")
    urdf = xacro.process_file(str(robot_urdf)).toxml()
    default_config = PathJoinSubstitution([FindPackageShare("cap_vision"), "config", "red_scene.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("start_camera", default_value="false"),
        # Camera robot la USB camera (/dev/video2); video0/1 la webcam laptop.
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("legacy_topics", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="red_scene_measured_state", parameters=[{"robot_description": urdf,
                 "frame_prefix": "measured/", "publish_frequency": 30.0}],
             remappings=[("joint_states", "/real_joint_states"),
                         ("robot_description", "/measured_robot_description")]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="red_scene_base",
             arguments=["--frame-id", "base_link", "--child-frame-id", "measured/base_link"]),
        Node(package="cap_vision", executable="camera_test", name="red_scene_camera",
             condition=IfCondition(LaunchConfiguration("start_camera")),
             parameters=[{"device_index": ParameterValue(LaunchConfiguration("device_index"), value_type=int),
                          "frame_id": "measured/camera_optical_frame", "width": 640, "height": 480}]),
        Node(package="cap_vision", executable="red_scene", output="screen",
             parameters=[{"config": LaunchConfiguration("config"),
                          "legacy_topics": ParameterValue(LaunchConfiguration("legacy_topics"), value_type=bool)}]),
    ])
