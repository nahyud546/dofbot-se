"""Config cho Dofbot follower. Single source of truth cho joints/features.

Quy ước LeRobot: mọi joint key phải có suffix ``.pos`` và dùng degrees
(``use_degrees=True``) để khớp policy/dataset cũ (so101/koch).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig


# Khớp 100% src/dofbot_moveit/config/ros2_controllers.yaml
ARM_JOINTS: tuple[str, ...] = (
    "arm1_Joint",
    "arm2_Joint",
    "arm3_Joint",
    "arm4_Joint",
    "arm5_Joint",
)
GRIPPER_JOINT: str = "Rlink1_Joint"
TCP_LINK: str = "Gripping_point_Link"

ARM_ACTION_TOPIC = "/arm_group_controller/follow_joint_trajectory"
GRIP_ACTION_TOPIC = "/grip_group_controller/gripper_cmd"
JOINT_STATES_TOPIC = "/joint_states"


@RobotConfig.register_subclass("dofbot_follower")
@dataclass(kw_only=True)
class DofbotFollowerConfig(RobotConfig):
    """Config cho Dofbot 6DOF follower.

    backend:
      - "sim_ros2": dùng ros2_control (khuyên dùng khi record/eval trên sim,
        chạy trong venv ROS py3.10 hoặc bridge node).
      - "arm_lib": dùng Yahboom Arm_Lib trên tay thật (servo bus, không cần ROS).
    """

    backend: str = "sim_ros2"
    # -- sim_ros2 --
    arm_action_topic: str = ARM_ACTION_TOPIC
    grip_action_topic: str = GRIP_ACTION_TOPIC
    joint_states_topic: str = JOINT_STATES_TOPIC
    # -- arm_lib (tay thật) --
    # Arm_Lib mở serial nội bộ, không cần port cấu hình. time_ms là thời gian
    # thực thi mỗi send_action (giữ thấp khi teleop/record, cao khi rollout mượt).
    arm_lib_time_ms: int = 500
    # Giới hạn an toàn theo độ (s1-s4,s6: 0-180, s5: 0-270 theo Arm_Lib).
    # Clip trước khi gửi để tránh kẹt servo.
    joint_limits_deg: dict = field(
        default_factory=lambda: {
            "arm1_Joint": (0.0, 180.0),
            "arm2_Joint": (0.0, 180.0),
            "arm3_Joint": (0.0, 180.0),
            "arm4_Joint": (0.0, 180.0),
            "arm5_Joint": (0.0, 270.0),
            "Rlink1_Joint": (0.0, 180.0),
        }
    )
    cameras: dict = field(default_factory=dict)
    mock: bool = False
