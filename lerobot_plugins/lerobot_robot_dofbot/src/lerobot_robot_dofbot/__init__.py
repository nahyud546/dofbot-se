"""LeRobot plugin: Yahboom Dofbot 6DOF.

Discovery: pip package prefix ``lerobot_robot_*`` được LeRobot tự quét
(``lerobot.utils.import_utils.register_third_party_plugins``), cộng với
``@RobotConfig.register_subclass("dofbot_follower")`` nên dùng ngay:

    lerobot-record --robot.type=dofbot_follower --robot.backend=sim_ros2 ...

Hai backend trong cùng một class để sim và hardware thật chia sẻ
observation/action features (single source of truth cho AI):
  - sim_ros2 : nói chuyện với ros2_control qua /joint_states +
               /arm_group_controller/follow_joint_trajectory (khớp
               src/dofbot_moveit/config/ros2_controllers.yaml).
  - arm_lib  : nói chuyện với tay thật qua Yahboom Arm_Lib
               (Arm_serial_servo_write6, đơn vị độ 0-180, s5 0-270).
"""

from lerobot_robot_dofbot.config_dofbot import DofbotFollowerConfig
from lerobot_robot_dofbot.dofbot import DofbotFollower

__all__ = ["DofbotFollower", "DofbotFollowerConfig"]
