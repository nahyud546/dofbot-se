#!/usr/bin/env python3
"""Pick with the stacking IK/geometry and place into four color bins."""

import math
from pathlib import Path
import sys
import threading
import traceback

import os
import rclpy
from std_msgs.msg import Bool, Float32MultiArray

def _repo_root() -> Path:
    env = os.environ.get("ROBOT_ARM_ROOT")
    if env and Path(env).exists():
        return Path(env)
    for c in [Path("/home/jloy/Desktop/robot-arm"), Path("/home/yahboom")]:
        if (c / "workspaces").exists() or c.exists():
            return c
    return Path(__file__).resolve().parents[5]

_REPO = _repo_root()
STACK_SCRIPTS = _REPO / "workspaces/dofbot_ws/src/dofbot_color_stacking/scripts"
if str(STACK_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(STACK_SCRIPTS))
from stacking_target import stacking_GetTarget  # same server_joint as color stack
from cube_geometry import CUBE_YAW_SIGN
from dofbot_sorting_3d.color_sorting import ready_pose


# The original sorting destinations, color IDs: blue, green, red, yellow.
# The sixth joint is closed while approaching the bin; move() opens it there.
BIN_POSES = {
    1: [30, 70, 0, 54, 265, 140],
    2: [150, 70, 7, 56, 265, 140],
    3: [50, 70, 7, 58, 265, 140],
    4: [135, 70, 7, 54, 265, 140],
}
# Color sorting alone picks 8 mm above the shared stacking pick height.
PICK_RAISE_M = 0.008
# After reaching the original bin approach pose, descend ~10 mm at the same
# XY before opening the gripper. FK errors from the original bin XY are
# <=0.5 mm on the base_link -> Gripping_point_Link chain.
BIN_RELEASE_POSES = {
    1: [30, 65.7098, 0.379519, 56.1977, 265, 140],
    2: [150, 66.6198, 7.04465, 57.3374, 265, 140],
    3: [50, 66.7004, 7.04872, 59.2308, 265, 140],
    4: [135, 66.5425, 7.0367, 55.4436, 265, 140],
}
# FK checked against base_link -> Gripping_point_Link (same KDL chain as
# stacking). Each pose raises TCP 60 mm at the same bin XY, with XY drift
# below 0.21 mm. Keep J1/J5 at the bin values while clearing the cube.
BIN_LIFT_POSES = {
    1: [30, 89.348, 5.94074, 40.3418, 265, 30],
    2: [150, 85.7708, 12.7032, 47.1349, 265, 30],
    3: [50, 85.3714, 12.4169, 49.9596, 265, 30],
    4: [135, 86.139, 13.0342, 44.3005, 265, 30],
}


class ColorBinGrasp(stacking_GetTarget):
    def __init__(self):
        super().__init__()
        self._executor_spinning = True
        self.home = ready_pose()
        self.busy = False
        self.motion_thread = None
        self.create_subscription(Float32MultiArray, 'color_pick', self.on_pick, 10)
        self.done_pub = self.create_publisher(Bool, 'grasp_done', 10)
        self.get_logger().info('Color bin grasp ready; stacking IK, four fixed bins')

    def on_pick(self, msg):
        if self.busy:
            self.get_logger().warning('Arm busy; ignoring new pick request')
            return
        if len(msg.data) != 4 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().error('Invalid color pick request')
            self.done_pub.publish(Bool(data=False))
            return
        color_id, x, y, yaw = msg.data
        if color_id not in BIN_POSES:
            self.get_logger().error(f'Invalid color ID: {color_id}')
            self.done_pub.publish(Bool(data=False))
            return
        if not (-0.30 <= x <= -0.10 and -0.12 <= y <= 0.12):
            self.get_logger().error(f'Target outside pick workspace: ({x:.3f},{y:.3f})')
            self.done_pub.publish(Bool(data=False))
            return
        self.busy = True
        self.motion_thread = threading.Thread(
            target=self.sort_cube, args=(int(color_id), (x, y), yaw))
        self.motion_thread.start()

    def sort_cube(self, color_id, pos, yaw):
        success = False
        try:
            # server_joint uses the stack's KDL frame, pitch, z retries and
            # all-zero IK guard. pos already has negative KDL X; no extra
            # X/Z correction is applied here.
            joints = self.server_joint(pos, pick_height_offset_m=PICK_RAISE_M)
            if joints is None:
                self.get_logger().error('IK failed; arm stayed at ready pose')
                return
            yaw = max(-40.0, min(40.0, yaw))
            j5 = min(270.0, max(0.0, joints[0] - CUBE_YAW_SIGN * yaw))
            pick = [joints[0], joints[1], joints[2], joints[3], j5, 30]
            place = list(BIN_POSES[color_id])
            release = list(BIN_RELEASE_POSES[color_id])
            lift = list(BIN_LIFT_POSES[color_id])
            self.get_logger().info(
                f'Pick color={color_id} at {pos}, z offset=+{PICK_RAISE_M:.3f}m; '
                f'yaw={yaw:+.1f}, J5={j5:.1f}; '
                f'bin approach={place[:5]}, release lower={release[:5]}')
            self.grap.move(pick, place, lift, self.home,
                           joints_release=release)
            success = True
        except Exception:
            self.get_logger().error(f'Color sort failed:\n{traceback.format_exc()}')
        finally:
            self.busy = False
            self.done_pub.publish(Bool(data=success))


def main(args=None):
    # stacking_target initializes rclpy on import for the standalone stack.
    if not rclpy.ok():
        rclpy.init(args=args)
    node = ColorBinGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.motion_thread is not None and node.motion_thread.is_alive():
            node.get_logger().info('Waiting for current color sort motion to finish')
            node.motion_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
