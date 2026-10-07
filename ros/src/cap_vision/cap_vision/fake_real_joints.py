"""Fake /real_joint_states publisher for RViz visualization only.

The planning + measured robot_state_publishers both subscribe
``/real_joint_states``. When no hardware daemon owns /dev/ttyUSB0,
no TF is published and RViz RobotModel stays red
("No transform from [X_Link] to [base_link]").

This node publishes the T8 READY observation pose at 10 Hz so the
TF tree resolves and RViz turns green. NEVER use for grasp math:
eye-in-hand perception requires timestamped measured joints.

Disable as soon as a real driver publishes:
  ros2 launch cap_vision object_pipeline.launch.py fake_joints:=false
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# T8 READY_POSE (deg) shared with t8_motion_worker.py.
READY_DEG = [90.0, 125.0, 0.0, 0.0, 90.0]
ARM_JOINTS = [f"arm{i}_Joint" for i in range(1, 6)]
# Gripper: Rlink1 independent; the other 5 mimic it but Humble's
# robot_state_publisher ignores <mimic>, so resolve explicitly here.
# Multipliers from dofbot.urdf: Llink1=-1, Rlink2=-1, Llink2=+1, Rlink3=+1, Llink3=-1.
GRIPPER_JOINTS = ["Rlink1_Joint", "Llink1_Joint", "Rlink2_Joint",
                  "Llink2_Joint", "Rlink3_Joint", "Llink3_Joint"]
GRIPPER_SIGN = [1.0, -1.0, -1.0, 1.0, 1.0, -1.0]
# Gripper open (safety_config: open=0.0 rad, close=1.57 rad).
GRIPPER_OPEN_RAD = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = Node("fake_real_joints")
    pub = node.create_publisher(JointState, "/real_joint_states", 10)
    positions = [math.radians(v) for v in READY_DEG]
    gripper = [GRIPPER_OPEN_RAD * s for s in GRIPPER_SIGN]
    names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
    full_positions = list(positions) + list(gripper)
    timer_period = 0.1

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = list(full_positions)
        pub.publish(msg)

    node.create_timer(timer_period, tick)
    node.get_logger().info(
        "fake /real_joint_states at READY pose (viz only); "
        "set fake_joints:=false with real hardware")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
