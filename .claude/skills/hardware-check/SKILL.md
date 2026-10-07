---
name: hardware-check
description: Kiểm tra camera tay, camera ngoài và cổng serial của tay máy có rảnh và hoạt động không; tìm tiến trình đang giữ thiết bị và dọn tiến trình sót một cách an toàn. Dùng trước và sau mọi lần chạy phần cứng, hoặc khi gặp "cổng đang bận", "không đọc được khung", viewer đen.
---

# Kiểm tra phần cứng

## Ai đang giữ thiết bị
```bash
fuser /dev/video0 /dev/video2 /dev/ttyUSB0            # không in gì = rảnh
ps -eo pid,ppid,etime,cmd | grep -E "t8_assistant|ros2 launch|object_perception|camera_test|real_joint_mirror|t8_motion_worker" | grep -v grep
```
- `t8_assistant` đang chạy là phiên của người dùng: **không tắt**, báo lại và chờ.
- Perception gồm 5 tiến trình con của một `ros2 launch` (camera_test giữ camera, real_joint_mirror giữ serial).

## Dọn tiến trình sót (chỉ cái do mình bật)
Dừng theo pid của tiến trình cha và các con của nó, rồi kiểm lại:
```bash
P=<pid của "ros2 launch">; ps -eo pid,ppid | awk -v p=$P '$1==p||$2==p{print $1}' | xargs -r kill
sleep 3; fuser /dev/video0 /dev/ttyUSB0
```
Không dùng `pkill -f <mẫu>`: mẫu dễ khớp chính dòng lệnh đang chạy và giết luôn shell.

## Camera nào là camera nào
Số `/dev/videoN` đổi giữa các lần cắm. Mỗi camera USB có hai node; node chẵn (index 0) mới là hình.
```bash
python -m cube_vision.cameras --list
python -m cube_vision.cameras --identify      # xoay J1 ~12° rồi so ảnh: camera đổi ảnh toàn cục là camera tay
python -m cube_vision.placement_check         # camera ngoài: thấy đủ 4 ô màu? lưu /tmp/external_check.png
```
Kết quả nhận diện nhớ ở `/tmp/t8_camera_roles.json` theo tên thiết bị.

## Triệu chứng → nguyên nhân
| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| "Cổng tay máy đang bận" | T8 khác hoặc `real_joint_mirror` còn sống; khóa `/tmp/t8_motion.lock` |
| "Không đọc được khung" camera ngoài | `camera_test` của người dùng đang mở camera đó |
| Ảnh camera ngoài tối | Khung đầu chưa phơi sáng xong (chờ ~1 s / 30 khung) |
| Viewer đen, `NO FRESH CAMERA IMAGE` | Perception chết hoặc nhận ảnh bằng QoS best-effort (phải RELIABLE) |
| `Arm_serial_servo_read serial error` lặp lại | Hai tiến trình cùng đọc serial; bình thường thoáng qua khi motion worker đang giữ cổng |
| T8 báo "Chưa source ROS" | Terminal thiếu `source /opt/ros/humble/setup.bash && source ros/install/setup.bash` |

Đo ảnh ROS có đứt không (cần ROS đã source): `ros2 topic hz /cap_vision/image_raw` (kỳ vọng ~7–15 Hz, không đứt > 1 s).
