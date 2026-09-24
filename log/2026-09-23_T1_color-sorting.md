# T1 - Color sorting fixed-center

```text
Task: T1 color sorting fixed-center
Ngày test: 2026-09-23
Entry/command:
  PYTHONPATH="$PWD/colcon_ws/src/dofbot_utils/src:$PWD/colcon_ws/src/dofbot_color_sorting/scripts" \
  python3 colcon_ws/src/dofbot_color_sorting/scripts/color_sorting.py --camera /dev/video0 --port /dev/ttyUSB0
Camera/serial: /dev/video0 (robot Sonix/Microdia, 640x480) / /dev/ttyUSB0
Input vật lý: cube trong ảnh robot (img/2026-09-23_T1_video0.jpg - khối vuông trên nền trắng)
Kết quả: PARTIAL (perception PASS, motion FAIL)
Robot có chuyển động không: KHÔNG (lần chạy 23-09: detect + buzzer có, servo không nhúc nhích; hôm qua đã từng move OK)
File chính đã đọc: colcon_ws/src/dofbot_color_sorting/scripts/color_sorting.py (226 dòng); robot_controller.py (P_BLACK_CENTER, 4 zone, arm_move_6); Arm_Lib.py (write/write6_array/Buzzer_On/read)
Ý tưởng học được (user xác nhận 23-09):
- Pose quét camera P_LOOK_MAP=[90,125,0,0,90,0] -> ROI [200:480,160:400] HSV+close+contour>1000, debounce 10 frame, state waiting/Runing + thread.
- Camera chỉ quyết định tên màu, KHÔNG quyết định XY. Pose gắp luôn cố định P_BLACK_CENTER=[90,35,65,15,90,*].
- sorting_move: up -> open(30) -> center -> close(135) -> up -> xoay joint1 tới target[0] -> tới joints_target -> open -> nhấc joint2 -> joint1 về 90 -> về P_LOOK_MAP. 4 zone phân biệt chủ yếu ở joint1 + pose: red[117..], blue[44..], green[136..], yellow[65..]. Open-loop sleep, không IK, không feedback kẹp.
Lỗi gặp phải:
- Chạy T1 với --camera /dev/video2 nên mở nhầm webcam laptop (ACER, mặt người) thay vì cam robot.
- /dev/video0 đang bị giữ bởi PID 34379 `dofbot_robot_arm_6dof/.../camera_test --ros-args -p device_index:=0` (folder loại trừ) -> đã kill để giải phóng.
- Map thật từ /dev/v4l/by-id: video0/1 = Sonix USB 2.0 Camera (robot), video2/3 = ACER HD User Facing (laptop).
- Lần chạy 23-09 --camera /dev/video0: RUNNING OK, cửa sổ mở, detect màu + buzzer, nhưng servo không move. Log chỉ có Qt font warning + KeyboardInterrupt lúc Ctrl-C ở capture.read(), không thấy traceback từ thread sorting_run. Hôm qua cùng command đã move được -> nghi nguồn servo / kẹt status Runing / thread exception bị nuốt.
- Debug /tmp/t1_motion_debug.py: read 1-6 đều None (1-3 in `serial error`, 4-6 None im lặng); buzzer/write6_array chỉ `ser.write` fire-and-forget nên print `sent` không chứng minh move thật. Kết luận: TX logic tới board còn sống, RX bus servo chết hoàn toàn -> nghi mất nguồn động cơ / tuột cáp bus servo, không phải sai code.
Thay đổi đã làm:
- Kill process giữ video0, verify mở lại OK 640x480
- Sửa docs/yahboom_task_test_plan.md: camera robot /dev/video2 -> /dev/video0 (+ note webcam laptop)
- Chưa sửa code, giữ nguyên VideoCapture(args.camera, CAP_V4L2)
- Tạo /tmp/t1_motion_debug.py để tách RX (read servo) / TX (buzzer) / motion (P_LOOK_MAP + joint1 +-10deg)
Ảnh/video/log: img/2026-09-23_T1_video0.jpg (frame robot cam, có cube); log terminal 23-09 (RUNNING + Qt warning + KeyboardInterrupt)
Bước tiếp theo:
1. Kiểm tra công tắc nguồn tay máy + adapter (buzzer kêu không chứng minh motor có điện)
2. Chạy /tmp/t1_motion_debug.py để chốt read/motion tối thiểu
3. Nếu motion tối thiểu OK mà T1 vẫn đứng -> chạy T1 debug có traceback thread rồi đánh PASS/FAIL
```

## Phụ lục kiểm tra device ngày 2026-09-23 10:44
- /dev/ttyUSB0 có (dialout, 188,0)
- /dev/video0,1,2,3 đều có (video group)
- v4l2-ctl chưa cài
