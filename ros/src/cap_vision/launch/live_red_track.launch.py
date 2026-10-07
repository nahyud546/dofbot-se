"""Live-track cube that cho RViz (read-only, khong cham serial/HW).

camera (dang chay) -> cube_detector -> cube_tracker (TF ray-plane) -> RViz.
RViz: Fixed Frame=base_link, Add MarkerArray -> /vision/cubes_markers.

    ros2 launch cap_vision live_red_track.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("cap_vision")
    table = PathJoinSubstitution([pkg, "config", "table_zones.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("expected_count", default_value="1"),
        DeclareLaunchArgument("plane_z", default_value="0.015"),
        Node(package="cap_vision", executable="cube_detector", name="cube_detector",
             output="screen",
             parameters=[{"image_topic": "/cap_vision/image_raw",
                          "annotated_topic": "/vision/annotated",
                          "detections_topic": "/vision/detections_2d",
                          "table_file": table,
                          "expected_count": LaunchConfiguration("expected_count"),
                          "min_area": 0.0, "max_area": 0.0}]),
        Node(package="cap_vision", executable="cube_tracker", name="cube_tracker",
             output="screen",
             parameters=[{"detections_topic": "/vision/detections_2d",
                          "poses_topic": "/vision/cubes",
                          "markers_topic": "/vision/cubes_markers",
                          "frame_id": "base_link",
                          "optical_frame": "camera_frame",
                          "mount_frame": "Camera_Link",
                          "optical_rpy_deg": [0.0, 0.0, 270.0],
                          "fx": 902.0, "fy": 875.4, "cx": 320.0, "cy": 240.0,
                          "plane_z": LaunchConfiguration("plane_z"),
                          "cube_size": 0.030}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="cam_optical_tf_live",
             output="log",
             arguments=["--x", "0", "--y", "0", "--z", "0",
                        "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "Camera_Link", "--child-frame-id", "camera_frame"]),
    ])
