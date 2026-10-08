# bottle_cap_sorting
App phân loại nắp chai. Implementation ROS nằm ở:
- `workspaces/dofbot_ws/src/dofbot_sorting_3d` (color_sorting + color_bin_grasp)
- `workspaces/dofbot_ws/src/dofbot_color_stacking` (stacking_target, XYT_config)
- `workspaces/dofbot_robot_arm_6dof/src/cap_vision` + `cap_grasp`
Thư mục này chứa config/scripts/data riêng của app (không move package ROS vào đây để tránh gãy colcon).
Config dùng chung: `config/robot/`, `config/camera/`.
