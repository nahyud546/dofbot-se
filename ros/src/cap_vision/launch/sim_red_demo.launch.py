"""1 lenh mo full sim + track that: arm (FakeSystem) + RViz + zone do + cube vision.

Mo ta: include dofbot_moveit demo.launch.py (use_rviz:=false) + rviz2 rieng
voi config sim_red.rviz + sim_red_objects (cube sim an, chi giu zone) +
cube_detector + cube_tracker (mirror cube THAT theo camera live) + TF optical.

SIM-ONLY: khong chay dofbot_driver, khong Arm_Lib, khong /joint_states that.
Camera that (camera_test) chay rieng, khong nam trong launch nay.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dofbot_share = FindPackageShare("dofbot_moveit")
    cap_share = FindPackageShare("cap_vision")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([dofbot_share, "launch", "demo.launch.py"])),
            launch_arguments={"model": "dofbot", "use_rviz": "false"}.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([cap_share, "config", "sim_red.rviz"])],
        ),
        Node(
            package="cap_vision",
            executable="sim_red_objects",
            name="sim_red_objects",
            output="screen",
            parameters=[{"cube_hidden": True,  # tat cube sim co dinh, chi giu zone
                           "cube_collision": False}],
        ),
        Node(package="cap_vision", executable="cube_detector", name="cube_detector",
             output="log",
             parameters=[{"image_topic": "/cap_vision/image_raw",
                          "annotated_topic": "/vision/annotated",
                          "detections_topic": "/vision/detections_2d",
                          "table_file": PathJoinSubstitution(
                              [cap_share, "config", "table_zones.yaml"]),
                          "expected_count": 1,
                          "min_area": 0.0, "max_area": 0.0}]),
        Node(package="cap_vision", executable="cube_tracker", name="cube_tracker",
             output="screen",
             parameters=[{"detections_topic": "/vision/detections_2d",
                          "poses_topic": "/vision/cubes",
                          "markers_topic": "/vision/cubes_markers",
                          "frame_id": "base_link",
                          "optical_frame": "camera_frame",
                          "mount_frame": "Camera_Link",
                          # FIT 2-diem thuoc 2026-09-22: phi=270 (lech 12do; cac goc khac >78do)
                          "optical_rpy_deg": [0.0, 0.0, 270.0],
                          "fx": 902.0, "fy": 875.4, "cx": 320.0, "cy": 240.0,
                          "plane_z": 0.015,
                          "cube_size": 0.030}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="cam_optical_tf_live",
             output="log",
             arguments=["--x", "0", "--y", "0", "--z", "0",
                        "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", "Camera_Link", "--child-frame-id", "camera_frame"]),
    ])
