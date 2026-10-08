"""real_mirror.launch.py — vẽ bóng tay thật trong RViz, cạnh tay sim.

Khởi động robot_state_publisher thứ 2 (namespace `real_mirror`) đọc URDF
dofbot, subscribe /real_joint_states (do hardware/real_joint_mirror.py hoặc
arm_hw_bridge.py --mirror-topic publish) và phát TF với frame_prefix `real_`.

Kết quả trong RViz: tay sim ở TF base_link... (FakeSystem /joint_states),
bóng tay thật ở real_base_link... — nhìn 2 tư thế cùng lúc, lệch là thấy ngay.

AN TOÀN: launch này CHỈ đọc + vẽ. Không controller, không move_group,
không execute. Lệnh duy nhất ra servo vẫn là arm_hw_bridge.py --yes.

Dùng (sau khi đã colcon build):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch dofbot_moveit real_mirror.launch.py
  # RViz: Displays -> Add -> RobotModel -> Description Source: Topic,
  #   Description Topic: /real_mirror/robot_description, TF Prefix: real_
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dofbot", package_name="dofbot_moveit")
        .robot_description(file_path="config/dofbot.urdf.xacro")
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="real_mirror",
            name="robot_state_publisher",
            parameters=[
                moveit_config.robot_description,
                {"frame_prefix": "real_"},
            ],
            remappings=[
                ("/joint_states", "/real_joint_states"),
            ],
            output="screen",
            ros_arguments=["--log-level", "WARN"],
        ),
        # Treo bóng tay thật vào cùng world với tay sim để so trực tiếp.
        # Pose identity: bóng đè khít tay sim khi 2 tư thế trùng nhau.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_real_base",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "world",
                       "--child-frame-id", "real_base_link"],
            output="log",
            ros_arguments=["--log-level", "WARN"],
        ),
    ])
