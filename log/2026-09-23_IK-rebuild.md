# IK rebuild - thay thế libkin_srv.so + libdofbot_kinematics.so bằng KDL

```text
Task: Infra - rebuild IK dofbot_info (mở khóa grasp tọa độ tự do)
Ngày test: 2026-09-23
Entry/command:
  cd ~/Desktop/robot-arm/dofbot_ws
  rm -rf build/dofbot_interface install/dofbot_interface build/dofbot_info install/dofbot_info
  source /opt/ros/humble/setup.bash
  colcon build --packages-select dofbot_interface --executor sequential
  colcon build --packages-select dofbot_info --cmake-args -DCMAKE_BUILD_TYPE=Release -Ddofbot_interface_DIR=$PWD/install/dofbot_interface/share/dofbot_interface/cmake
Camera/serial: không dùng (build + test offline, không chạm robot)
Input vật lý: URDF dofbot_ws/src/dofbot_urdf/urdf/dofbot.urdf + header dofbot_info/include/dofbot_kinematics.h
Kết quả: PASS (build)
Robot có chuyển động không: không
File chính đã đọc:
  dofbot_ws/src/dofbot_info/CMakeLists.txt (cũ link cứng /usr/lib/libkin_srv.so)
  dofbot_ws/src/dofbot_info/src/kinemarics_dofbot.cpp (getFK/getIK, urdf path /home/yahboom/...)
  dofbot_ws/src/dofbot_info/include/dofbot_kinematics.h (API getFK/getIK + class DOFBOT_Pro)
  dofbot_ws/src/dofbot_urdf/urdf/dofbot.urdf (chain base_link -> ... -> arm5 -> Gripping_point_Link)
Ý tưởng học được:
- libkin_srv + libdofbot_kinematics chỉ là wrapper FK/IK đóng nguồn; API lộ qua header nên viết lại được bằng Orocos-KDL có sẵn.
- FK = ChainFkSolverPos_recursive; IK = ChainIkSolverPos_NR_JL (KDL 1.5 không còn ctor LMA 6 tham số) limits +-1.57, tip Gripping_point_Link.
- Quy ước khớp: kdl_q = (servo_deg - 90) * DE2RA, 0 rad == 90 do servo.
Lỗi gặp phải:
- build/ + install/ cũ cache từ /home/yahboom -> xóa build+install của 2 package rồi build lại.
- dofbot_info configure trước dofbot_interface khi build song song -> build tuần tự từng package.
- KDL 1.5 đổi ctor ChainIkSolverPos_LMA -> chuyển sang NR_JL + Vel_pinv.
Thay đổi đã làm:
- Mới: dofbot_ws/src/dofbot_info/src/dofbot_kinematics.cpp (namespace + class, ~150 dòng).
- Sửa: CMakeLists.txt build 2 lib libdofbot_kinematics.so + libkin_srv.so từ cùng source, node link lib local (bỏ /usr/lib tuyệt đối).
- Sửa: kinemarics_dofbot.cpp urdf path /home/yahboom/... -> /home/jloy/Desktop/robot-arm/...
- Sửa: include header NR_JL/pinv ở cả .h và .cpp.
Ảnh/video/log:
- install/dofbot_info/lib/libdofbot_kinematics.so, libkin_srv.so, lib/dofbot_info/kinemarics_dofbot
- /tmp/kin_test.cpp + /tmp/kin_test2.cpp (giữ ở /tmp, chưa đưa vào repo)
Bước tiếp theo:
1. Test service thật: chạy kinemarics_dofbot với LD_LIBRARY_PATH rồi ros2 service call dofbot_kinemarics (fk rồi ik).
2. Sau đó mở khóa T5/T6 grasp động vision-only trước, motion sau.
3. Cân nhắc đưa install/setup.bash tổng về trạng thái source sạch (hiện vẫn hỏng ref dofbot_moveit).
```

## Phụ lục verify 23-09
- `colcon build --packages-select dofbot_info`: Finished (3.6s).
- `/tmp/kin_test`: FK home X=-0.0048 Y=0.0007 Z=0.4374; IK back joint err 0.00000 OK (KDL warning root inertia vô hại).
- `/tmp/kin_test2`: IK pose reach err 0.00000 m / 0.00000 rad POSE-OK (sai khác joint-space ở test 1 chỉ là nghiệm nhánh khác, pose vẫn khớp).
- Binary service: `kinemarics_dofbot` in `Dofbot is waiting to receive.....`, cần `LD_LIBRARY_PATH=install/dofbot_info/lib:install/dofbot_interface/lib:/opt/ros/humble/lib`.
