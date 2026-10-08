"""Chay full pipeline camera that: camera -> detector -> RViz markers.

TF: world -> table_frame (static, identity mac dinh; khi robot that toi
se thay bang TF base_link -> table_frame do that).
RViz: Fixed Frame = world, add MarkerArray /caps_markers.

Vi du:
    ros2 launch cap_vision cap_mapping.launch.py
    ros2 launch cap_vision cap_mapping.launch.py show_image:=true device_index:=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("cap_vision")
    camera_config = LaunchConfiguration("camera_config")
    homography = LaunchConfiguration("homography")

    camera_node = Node(
        package="cap_vision",
        executable="camera_test",
        name="camera_test",
        output="screen",
        parameters=[
            camera_config,
            {"show": LaunchConfiguration("show_image"),
             "device_index": LaunchConfiguration("device_index"),
             "width": LaunchConfiguration("width"),
             "height": LaunchConfiguration("height")},
        ],
    )
    detector_node = Node(
        package="cap_vision",
        executable="cap_detector",
        name="cap_detector",
        output="screen",
        parameters=[{"homography_file": homography}],
    )
    rviz_pub_node = Node(
        package="cap_vision",
        executable="cap_rviz_publisher",
        name="cap_rviz_publisher",
        output="screen",
        parameters=[{"rviz_topic": "/caps_markers"}],
    )
    # world -> table_frame: tam thoi identity (chua co robot that).
    # Khi do vi tri O + huong truc that, sua x/y/z/yaw o day.
    world_to_table = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_table_frame",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("table_x"),
            "--y", LaunchConfiguration("table_y"),
            "--z", LaunchConfiguration("table_z"),
            "--yaw", LaunchConfiguration("table_yaw"),
            "--frame-id", "world",
            "--child-frame-id", "table_frame",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_config",
            default_value=PathJoinSubstitution([pkg, "config", "camera.yaml"]),
            description="File cau hinh camera (device, size, topic)."),
        DeclareLaunchArgument(
            "homography",
            default_value=PathJoinSubstitution([pkg, "config", "homography.yaml"]),
            description="File homography da calibrate."),
        DeclareLaunchArgument("show_image", default_value="false",
                              description="Mo cua so preview OpenCV."),
        DeclareLaunchArgument("device_index", default_value="10",
                              description="Index camera (/dev/videoN)."),
        DeclareLaunchArgument("width", default_value="1280",
                              description="Rong anh camera."),
        DeclareLaunchArgument("height", default_value="720",
                              description="Cao anh camera."),
        DeclareLaunchArgument("table_x", default_value="0.0",
                              description="Vi tri O cua table_frame trong world (m)."),
        DeclareLaunchArgument("table_y", default_value="0.0"),
        DeclareLaunchArgument("table_z", default_value="0.0"),
        DeclareLaunchArgument("table_yaw", default_value="0.0",
                              description="Yaw table_frame quanh Z (rad)."),
        camera_node, detector_node, rviz_pub_node, world_to_table,
    ])
