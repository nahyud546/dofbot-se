cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash && source install/setup.bash

ls -la /dev/video*

ros2 pkg list | grep cap_vision

ros2 run cap_vision camera_test --ros-args -p device_index:=0 -p width:=640 -p height:=480 -p show:=true -p snapshot_path:=/tmp/robot_cam.png


#state robot
timeout 5 ros2 run tf2_ros tf2_echo base_link Gripping_point_Link
timeout 5 ros2 topic echo /joint_states --once