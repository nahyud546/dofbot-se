# hardware/ — cầu nối tay thật, tách khỏi demo sim.

- Tay thật dùng `Arm_Lib.Arm_Device` (0.0.5, đã có trên image Yahboom):
  `Arm_serial_servo_write6(s1..s6, time_ms)` — s1-s4,s6: 0-180 độ, s5: 0-270.
  Đọc: `Arm_serial_servo_read_any(id 1..6)`.
- Code ROS cũ (`dofbot_driver`) KHÔNG cần để build/sim; khi cần thì copy
  `dofbot_driver/` vào `src/` theo README gốc, nó chỉ là bridge /joint_states.
- Đường an toàn: plugin LeRobot `backend=arm_lib` đã clip theo
  `joint_limits_deg` + `arm_lib_time_ms` (để cao khi test thật). Cờ chặn
  execute hardware (`TCP_OFFSET_CALIBRATED=False`) vẫn giữ ở task chess cho
  tới khi đo OFFSET thật (xem tasks/chess/task.yaml + README gốc).
- Sim dùng chung topics: `/joint_states`, `/arm_group_controller/...`,
  `/grip_group_controller/...` (xem src/dofbot_common + ros2_controllers.yaml).
