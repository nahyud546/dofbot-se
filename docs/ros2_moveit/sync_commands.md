# Note command: sim <-> real (chi chay, khong giai thich dai)

> Moi lenh chay tai `~/Desktop/robot-arm/dofbot_robot_arm_6dof`, da `source /opt/ros/humble/setup.bash && source install/setup.bash`.
> Can T1 (`ros2 launch dofbot_moveit demo.launch.py use_rviz:=true`) dang chay cho cac lenh sim.

ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video* 


## 1. Reset ve HOME

```bash
# Real ve HOME (tay that cu dong, cham):
python3 hardware/calibrate_safety.py --park-check --yes --port /dev/ttyUSB0

# Sim ve HOME (tay that dung yen):
ros2 action send_goal /arm_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joint, arm5_Joint], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"
ros2 action send_goal /grip_group_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 1.0}}"
```

## 2. Bat trang thai hien tai (1 lan)

```bash
# Sim (can T1):
ros2 topic echo /joint_states --once

# Real truc tiep tu servo (khong can ROS, read-only):
python3 hardware/check_stm32.py --port /dev/ttyUSB0
```

## 3. Real theo sim 

  Terminal 1 — mở MoveIt và RViz:

  cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch dofbot_moveit demo.launch.py

  Terminal 2 — chạy follower tới /dev/ttyUSB0:

  cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 hardware/follow_sim.py --yes --allow-uncalibrated --port /dev/ttyUSB0
