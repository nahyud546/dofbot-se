"""Launch tong B1-B5: camera that 640x480 -> detector -> localizer -> pick-place (dry-run) + RViz TF.

    ros2 launch cap_vision cubes.launch.py
    ros2 launch cap_vision cubes.launch.py show_image:=true dry_run:=false

RViz: Fixed Frame=base_link, add /vision/cubes_markers, /vision/plan_markers, /vision/annotated.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("cap_vision")
    cam_info = PathJoinSubstitution([pkg, "config", "camera_info_640x480.yaml"])
    table = PathJoinSubstitution([pkg, "config", "table_zones.yaml"])

    camera_node = Node(package="cap_vision", executable="camera_test", name="camera_test",
                       output="screen",
                       parameters=[{"device_index": LaunchConfiguration("device_index"),
                                    "width": 640, "height": 480,
                                    "fps": 30.0, "frame_id": "camera_frame",
                                    "image_topic": "/cap_vision/image_raw",
                                    "show": LaunchConfiguration("show_image")}])
    detector_node = Node(package="cap_vision", executable="cube_detector", name="cube_detector",
                         output="screen",
                         parameters=[{"image_topic": "/cap_vision/image_raw",
                                      "annotated_topic": "/vision/annotated",
                                      "detections_topic": "/vision/detections_2d",
                                      "table_file": table,
                                      "expected_count": LaunchConfiguration("expected_count"),
                                      "min_area": 0.0, "max_area": 0.0}])
    localizer_node = Node(package="cap_vision", executable="cube_localizer", name="cube_localizer",
                          output="screen",
                          parameters=[{"detections_topic": "/vision/detections_2d",
                                       "poses_topic": "/vision/cubes",
                                       "markers_topic": "/vision/cubes_markers",
                                       "camera_info_file": cam_info,
                                       "table_file": table,
                                       "frame_id": "base_link",
                                       "use_tf": False}])
    planner_node = Node(package="cap_vision", executable="cubes_pick_place", name="cubes_pick_place",
                        output="screen",
                        parameters=[{"markers_topic": "/vision/cubes_markers",
                                     "plan_topic": "/vision/plan_markers",
                                     "table_file": table,
                                     "dry_run": LaunchConfiguration("dry_run")}])
    # Camera_Link -> camera_frame (quang hoc, mac dinh nhin xuong; tinh chinh sau ArUco).
    # rpy 180/0/180 bien truc URDF ve chuan optical (z forward).
    cam_static = Node(package="tf2_ros", executable="static_transform_publisher", name="cam_optical_tf",
                      output="screen",
                      arguments=["--x", "0", "--y", "0", "--z", "0",
                                 "--roll", "3.1416", "--pitch", "0", "--yaw", "3.1416",
                                 "--frame-id", "Camera_Link", "--child-frame-id", "camera_frame"])
    return LaunchDescription([
        DeclareLaunchArgument("device_index", default_value="2"),
        DeclareLaunchArgument("show_image", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("expected_count", default_value="4"),
        camera_node, detector_node, localizer_node, planner_node, cam_static,
    ])
