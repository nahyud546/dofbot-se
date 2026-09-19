"""Tên joints/topics dùng chung. Khớp src/dofbot_moveit/config/ros2_controllers.yaml."""

ARM_JOINTS: tuple[str, ...] = (
    "arm1_Joint",
    "arm2_Joint",
    "arm3_Joint",
    "arm4_Joint",
    "arm5_Joint",
)
GRIPPER_JOINT = "Rlink1_Joint"
TCP_LINK = "Gripping_point_Link"

ARM_ACTION_TOPIC = "/arm_group_controller/follow_joint_trajectory"
GRIP_ACTION_TOPIC = "/grip_group_controller/gripper_cmd"
JOINT_STATES_TOPIC = "/joint_states"

# Home an toàn (rad) cho ros2_control. Gripper 0.0=mở.
HOME_JOINTS_RAD = (0.0, 0.0, 0.0, 0.0, 0.0)
GRIPPER_OPEN_RAD = 0.0
GRIPPER_CLOSE_RAD = 1.57
