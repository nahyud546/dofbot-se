# T-dynamic sorting - sự cố motion loạn + fix IK/grasp (23-09)

```text
Task: color sorting tọa độ tự do (dofbot_sorting_3d color_sorting + grasp + cam_pub + kinemarics_dofbot)
Ngày test: 2026-09-23
Entry/command: ros2 run dofbot_sorting_3d {cam_pub,color_sorting,grasp} + install/dofbot_info/lib/dofbot_info/kinemarics_dofbot
Camera/serial: /dev/video1 (Sonix, sau khi rút/cắm lại; video0 biến mất) / /dev/ttyUSB0
Input vật lý: cube đỏ trong view, bấm r -> i -> SPACE
Kết quả: FAIL (motion loạn đã dừng; đã tìm ra 3 nguyên nhân + fix, verify offline OK)
Robot có chuyển động không: CÓ (loạn trước fix; sau fix chưa re-test motion)
File chính đã đọc:
  dofbot_sorting_3d/color_sorting.py (278 dòng: HSV text, phím b/g/r/y/i/SPACE, map pixel->world, publish PosInfo)
  dofbot_sorting_3d/grasp.py (166 dòng: sub PosInfo -> IK pitch 1.04 -> write6 -> zone p_1..p_4 -> grasp_done)
  dofbot_sorting_3d/color_common.py (object_follow, minAreaRect)
  Arm_Lib.py write6 (142-187) vs write6_array (255-307)
Ý tưởng học được:
- Flow động: /image_raw -> HSV circle (cx,cy,r>30) -> a=(320-cx)/4000, b=((480-cy)/3000)*0.8+0.12 -> PosInfo(id=màu,x=b,y=a+0.01,z=0.039) -> grasp IK(z+0.03) -> zone theo id.
- Khác bản fixed-center: pose gắp tính từ pixel, không còn pose cứng.
Lỗi gặp phải:
1. camera_test của dofbot_robot_arm_6dof (folder loại trừ) giữ /dev/video1 -> kill mới mở được cam.
2. install thiếu local_setup.bash cho package python -> chạy bằng PYTHONPATH + path script trực tiếp.
3. cv_bridge (NumPy 1.x) crash trên NumPy 2.2.6 (KeyError 16) -> viết cam_pub pack Image thủ công (rgb8) + color_sorting decode numpy, bỏ cv_bridge.
4. cv.imshow trong thread mới -> lỗi Qt GuiReceiver::showImage -> vẽ trực tiếp ở executor thread.
5. np.int0 bị xóa ở NumPy 2.x -> np.intp (color_common.py:398).
6. MOTION LOẠN - 3 nguyên nhân cộng hưởng:
   a) grasp.py đảo 180-x cho J2-J4 TRƯỚC khi gọi write6 (mà write6 đã đảo nội bộ như write6_array của bản fixed-center) -> double inversion, khớp ra vị trí gương.
   b) Map pixel->world cho +X nhưng bàn thật ở -X (FK P_BLACK_CENTER -> X=-0.207); target +X chỉ giải được ở thế vặn vẹo J4=180.
   c) Solver NR đơn-seed kẹt clamp; full-pose 6D quá ràng buộc cho tay 5DOF (nhạy Y 1cm, gimbal lock pitch -90).
Thay đổi đã làm:
- grasp.py: bỏ 180-x (pass servo degrees thẳng), request.yaw=-pi (khớp tip Gripping_point_Link), request.tar_x=-pos_x.
- dofbot_kinematics.cpp: NR multi-seed (13 seeds, J1 spread) + fallback random-restart 200x40 position-first, chấp nhận pos<3mm & pitch<0.35, seed RNG cố định.
- color_sorting.py: bỏ sudo v4l2-ctl (guard shutil.which), bỏ thread imshow, decode numpy, fix video0 path.
- Mới cam_pub.py (entry point) publish /image_raw rgb8 640x480@10Hz.
Ảnh/video/log: chưa có (dừng ở vòng motion); log service /tmp/kin_service.log
Bước tiếp theo:
1. ĐÃ XÁC MINH PHẦN CỨNG SỐNG (13:0x): follow_sim teach bám slider mượt 6/6; "J1/J2 chết" hóa ra là first-read-after-open race (lần đọc đầu sau mở port luôn lỗi 1 phát do byte tồn) — đọc lần 2 ra đủ [117,81,81,92,90,28]. Không hư servo nào. Bài học: mọi chẩn đoán read phải đọc 2 lượt, lượt 1 bỏ.
2. Pose nhìn bàn chưa chốt (pose init hiện tại chĩa tường) -> snapshot 3 pose ứng viên ([90,125,0,0], [90,135,20,25], [90,110,10,10]) rồi ghim init_joints cả 2 node.
3. Re-test AN TOÀN: perception (nguồn OFF) -> nguồn ON + Button_Mode(0) -> start grasp -> SPACE tay sẵn công tắc -> đối chiếu joints [~90,~50-65,~35-50,~0-15]; abort nếu J2>100/J4>120.
4. Nếu tay với lệch trái/phải (gương Y) -> báo để đổi dấu pos.y.
```

## Phụ lục số liệu verify offline 23-09
- FK home [90]*5 -> X=-0.0048 Y=0.0007 Z=0.4374 (service + offline khớp).
- FK grasp-down [90,35,65,15,90] -> X=-0.2069 Y=0.0007 Z=0.0528 R=0 P=1.1345 Yaw=-pi (~ pitch grasp 1.04).
- IK(-0.2069,0.0007,0.0828, 0/1.04/-pi) -> [90,44.7,64.7,11,90] OK.
- Grid +X chỉ ra thế J4=180 (vặn vẹo) -> quyết định đảo dấu X.
- Grid -X sau random-restart: [90.4,58.1,42.5,3.8,44.9] poserr 0.0027 pitcherr 0.28 OK.
