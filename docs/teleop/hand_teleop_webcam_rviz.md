# HUMAN-TO-ROBOT motion retargeting cho DOFBOT-SE trong RViz

## Chạy webcam hoặc video

### Webcam + robot thật qua STM32

Trước khi bật chuyển động, có thể xác nhận cổng và đọc 6 servo mà không gửi lệnh:

```bash
cd /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 hardware/check_stm32.py --port /dev/ttyUSB0 --read-only
```

Đóng tiến trình probe trước khi launch để cổng serial chỉ có một chủ. Sau khi đặt robot trong vùng trống, chạy webcam, RViz và robot thật cùng lúc:

```bash
cd /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hand_teleop preview.launch.py \
  source:=0 \
  real_robot:=true \
  serial_port:=/dev/ttyUSB0 \
  python_executable:="$PWD/src/hand_teleop/.venv/bin/python"
```

> **Lưu ý từ người dùng (28/09/2026):**
> Hiện tại kết quả điều khiển tay máy theo thao tác thực tế chưa hoàn toàn đạt được như mong muốn. Người dùng cũng chưa thử nghiệm tính năng áp dụng lên nguồn video có sẵn nên chưa rõ hiệu quả ra sao. Sẽ tiếp tục theo dõi, đánh giá và cập nhật thêm nhận xét sau khi có kết quả test.


Việc thấy `/dev/ttyUSB0` qua `ls` chỉ chứng minh thiết bị USB đã tạo cổng; nó chưa xác nhận STM32 trả lời hoặc 6 servo phản hồi. Nếu RViz báo thiếu TF `real/arm*_Link`, xem terminal: phải có dòng `STM32 /dev/ttyUSB0 connected; current=...`. Dòng `[HAND_TELEOP_HW_ERROR]` cho biết lỗi mở cổng, quyền truy cập hoặc servo readback cụ thể; launch sẽ dừng nếu node phần cứng thoát. Chạy lệnh probe ở trên và gửi toàn bộ kết quả nếu không thấy 6/6 servo phản hồi. Khi tay chưa nằm trong ảnh nhưng camera vẫn chạy, node chờ tracking và tiếp tục xuất góc servo lên RViz, không tự ra lệnh di chuyển lúc khởi động.

Servo đọc ở đúng biên cơ khí có thể lệch vài phần nghìn rad so với giới hạn URDF làm tròn 1,57 rad (ví dụ servo 4 = 0° tương ứng π/2 rad). Follower cho phép sai số đo tối đa 2° rồi chuẩn hóa về giới hạn URDF; lệnh điều khiển vẫn phải nằm trong giới hạn cấu hình. Một lần lỗi đọc serial được thử lại tối đa 3 lần. Khi khởi động, node cần hai lần đọc liên tiếp lệch nhau không quá 5° mỗi khớp rồi mới cho phép bám tay. Nếu `check_stm32.py` báo 6/6 nhưng hai lần đọc cho các góc rất khác nhau khi robot đứng yên, kiểm tra nguồn servo và bus trước khi bật chế độ bám tay thật.

Với `real_robot:=true`, RViz chỉ hiển thị **một mô hình DOFBOT actual servo readback**: tư thế đo từ STM32 qua `/real_joint_states`. Mô hình lệnh không còn hiển thị; topic `/hand_teleop/commanded_joint_states` vẫn dùng để chẩn đoán. Chỉ node `hardware_follower` gửi lệnh serial. Camera có ảnh và skeleton như lệnh webcam thường. Node bắt đầu bám tay sau 3 quan sát Hand/Pose liên tiếp. Sau khi đã bám tay, nếu cả Hand và Pose mất, nguồn camera ngừng cập nhật, hoặc IK liên tục không tạo được mục tiêu hợp lệ trong 2 giây, follower đưa robot về `safe_home_rad` (6 khớp bằng 0) với giới hạn 0,1 rad/s và 0,6 rad/s². Lỗi ngắn hơn 2 giây giữ tư thế cuối. Việc về HOME cần STM32 và đường serial còn hoạt động. Cảnh báo orientation có thể xuất hiện khi tool không đạt pitch yêu cầu nhưng IK vẫn tạo được pose an toàn. Log terminal ghi `HW mode=FOLLOW`, `HOLD_INVALID_TARGET`, `RETURN_HOME`, `RETURN_HOME_INVALID_TARGET` hoặc `RETURN_HOME_STALE_SOURCE`.

Follower kiểm tra trạng thái mỗi **0,2 giây** và gửi mục tiêu mới sau khi lệnh servo **500 ms** trước hoàn tất nội suy. Tốc độ tối đa mặc định vẫn là **0,1 rad/s ≈ 5,7°/s**. Follower đọc từng servo luân phiên và cập nhật mô hình đo trong RViz sau một vòng đọc. Nếu góc đo lệch khỏi lệnh quá 0,5 rad hoặc mất readback, follower giữ lệnh và báo `HOLD_SERVO_DIVERGENCE` hoặc `HOLD_READBACK_FAULT` thay vì làm tắt launch; khi readback phục hồi, nó tiếp tục điều khiển. Lệnh nhỏ hơn 0,003 rad được bỏ qua khi tay đứng yên. Có thể điều chỉnh `hardware_send_period_s` trong khoảng 0,1–0,5 s và `hardware_max_speed_rad_s` tối đa 0,25. J1 ưu tiên độ quét vai→cổ tay trong ảnh, chia cho chiều dài vai→khuỷu→cổ tay rồi trừ pose trung tính. Một nguồn hình ảnh ổn định giúp tránh cú nhảy do `Pose world` đổi trục khi mốc torso bị che; nếu nguồn hình ảnh không đủ, dùng `Pose world`. Nếu Pose mất hoàn toàn, đường mapping webcam cũ vẫn là fallback. Log `TRAJ` ghi `az=image:...` hoặc `az=world:...`; log phần cứng ghi `HW J1 target=... command=... readback=...` để đối chiếu target, lệnh đã giới hạn tốc độ và góc servo thực đo.

`real_robot:=false` là mặc định. Node phần cứng dùng công thức servo và giới hạn trong `hardware/safety_gate.py` và `hardware/safety_config.yaml`; `/dev/ttyUSB0` phải là cổng STM32 thật và không được có driver khác đang giữ cổng. Đây là thử nghiệm chuyển động trong vùng trống; calibration TCP và kiểm tra va chạm với vật/bàn chưa được xác nhận cho thao tác gắp thật. Nếu robot không phản hồi đúng chiều khớp hoặc đi gần vật cản, dừng launch bằng Ctrl+C và ngắt nguồn servo.

Lệnh test chính bằng webcam trực tiếp:

```bash
cd /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hand_teleop preview.launch.py \
  source:=0 \
  real_robot:=false \
  python_executable:="$PWD/src/hand_teleop/.venv/bin/python"
```

Sau khi đóng preview, có thể xem thống kê log vừa thu:

```bash
ros2 run hand_teleop benchmark /tmp/hand_teleop_webcam_run.jsonl
```

Để dùng video, đặt `VIDEO_FILE` thành **đường dẫn tuyệt đối tới file có thật** rồi chạy:

```bash
cd /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
source install/setup.bash
VIDEO_FILE="$HOME/Videos/ten_video_thuc_te.mp4"  # sửa thành file video của bạn
if test -f "$VIDEO_FILE"; then
  ros2 launch hand_teleop preview.launch.py \
    source:="$VIDEO_FILE" video_realtime:=false \
    record_jsonl:=/tmp/hand_teleop_run.jsonl \
    python_executable:="$PWD/src/hand_teleop/.venv/bin/python" && \
  test -s /tmp/hand_teleop_run.jsonl && \
    ros2 run hand_teleop benchmark /tmp/hand_teleop_run.jsonl
else
  echo "Không tìm thấy video: $VIDEO_FILE"
fi
```

`/duong/dan/video.mp4` trong ví dụ cũ chỉ là ký hiệu thay chỗ, không có file này trên máy. Có thể dùng `source:=0` cho webcam hoặc URI `rtsp://...` cho luồng live. Mặc định video phát theo media timestamp (`video_realtime:=true`); `video_realtime:=false` xử lý từng frame nhanh nhất theo thứ tự file. Các nguồn đi qua **cùng MediaPipe, estimator, retargeter, state machine và IK**. Video kết thúc sẽ xuất `SOURCE_EOF` rồi đóng launch. Benchmark chỉ chạy sau khi JSONL đã được tạo; dùng **cùng video** cho những lần chạy muốn so sánh. Benchmark tính tracking availability, độ rung beta, FPS, độ trễ, tỷ lệ IK và command được nhận. `j5_error_deg` để trống vì chưa có góc cổ tay đo chuẩn.

Nếu camera báo `Cannot open camera/video source 0`, node chưa xử lý được frame nào. Kiểm tra `ls -l /dev/video*`, đóng ứng dụng đang dùng camera và thử `source:=/dev/video2` theo thiết bị thực tế. Lệnh preview chỉ phát `/hand_teleop/preview_joint_states` cho RViz; **không gửi servo tới robot thật**.

## Luồng xử lý hiện tại

```text
Camera / video file
  → FrameSource: capture riêng, giữ frame mới nhất (video offline đọc tuần tự)
  → PerceptionWorker: MediaPipe Hand + Pose riêng; WiLoR tùy chọn chạy nền
  → HumanState: vai, khuỷu, cổ tay; hướng quét, độ duỗi, chiều cao;
                pitch/twist/pinch và confidence theo từng chức năng
  → IntentEstimator: HumanIntent → RobotTaskTarget (theta, r, z, beta, psi, grip)
  → One Euro filter và semantic grasp state machine
  → IK J1 + planar J2/J3/J4; J5 twist; gripper pinch
  → kiểm tra joint, bước nhảy, khoảng hở mặt bàn
  → /hand_teleop/preview_joint_states + target_pose + TF + diagnostics/JSONL
```

Capture, inference và timer control là ba luồng riêng khi dùng webcam hoặc video realtime. Buffer frame và observation đều chỉ giữ mẫu mới nhất; frame cũ bị bỏ nếu inference chậm. Timer chạy chu kỳ 20 ms và giữ pose đã chấp nhận giữa hai observation. Observation quá `max_observation_age_s` (mặc định 0,25 s) bị loại với `COMMAND_STALE`. Video offline không bỏ frame; `dt` của mapper và bộ lọc lấy từ timestamp video, không lấy từ tốc độ xử lý wall clock. WiLoR chỉ tinh chỉnh hướng khi có kết quả mới; nếu không sẵn sàng hoặc quá cũ, MediaPipe vẫn tiếp tục.

### Tọa độ và hiệu chỉnh neutral

MediaPipe image `u` tăng sang phải, `v` tăng xuống dưới. MediaPipe Pose world là tọa độ tương đối với camera, **không phải mét trong `base_link`**. Trong body frame, trục `+right` đi từ vai trái đến vai phải, `+up` từ giữa hông tới giữa vai, `+forward = right × up`. Hướng tay lấy từ vai tới cổ tay trong body frame; dấu cuối cùng từ người sang robot chỉnh bằng `theta_gain`. Robot dùng `base_link`: `Z` hướng lên; `theta=atan2(y,x)` là góc quay quanh Z, `r=hypot(x,y)` là tầm với. `camera_to_base` chỉ căn chỉnh **orientation**; neutral giữ sai khác zero pose giữa người và robot.

Launch mặc định lấy trung vị 10 frame đầu có đủ năm chuỗi ngón hợp lệ làm neutral của bàn tay. Pose arm neutral được lấy từ frame Pose hợp lệ đầu sau đó; khi hiệu chỉnh lại, các filter và target về HOME. Để hiệu chuẩn thủ công, dùng `/hand_teleop/calibrate_neutral`, `/hand_teleop/calibrate_open`, `/hand_teleop/calibrate_closed`. Profile được lưu tại `~/.ros/hand_teleop/calibration.yaml`; nếu có Pose, frame cẳng tay neutral cũng được lưu để tách J5 khỏi arm sweep. Với `auto_neutral_on_start:=true`, mỗi lần chạy lấy neutral mới.

### Chuyển động người → task target

| Tín hiệu | Công thức/nguồn chính | Tác dụng robot |
| --- | --- | --- |
| Quét cả cánh tay | Độ quét ngang vai→cổ tay trong ảnh, chuẩn hóa theo chiều dài cánh tay rồi trừ neutral; fallback `atan2(arm·body_right, arm·body_forward)` khi thiếu mốc ảnh | `theta = theta_home + theta_gain × delta_arm_sweep` → J1 |
| Duỗi/thu | `distance(shoulder,wrist) / (upper_arm + forearm)` từ Pose world, fallback Pose ảnh | `r = r_home + r_gain × delta_extension` |
| Nâng/hạ | `(shoulder_y − wrist_y) / arm_length` trong Pose world/ảnh | `z = z_home + z_gain × delta_height` |
| Hướng đầu kẹp | Palm frame từ 0/5/9/17 và vector ngón đáng tin | `beta` là pitch đầu kẹp; J2–J4 phối hợp |
| Xoắn riêng cổ tay | Palm rotation **tương đối với forearm frame** và neutral; fallback palm-only khi forearm gần thẳng/không nhìn rõ | `psi` → J5, giới hạn `wrist_sign` và tốc độ bước |
| Pinch | `distance(thumb_tip,index_tip) / palm_width`, chuẩn hóa open/closed | Gripper mở/đóng liên tục |

Các delta có deadzone khởi động/tiếp tục khác nhau để giảm rung và tránh bật/tắt đột ngột. `theta/r/z/beta/psi/grip` dùng One Euro filter theo timestamp của nguồn; J5 có thêm giới hạn vận tốc/gia tốc góc. `theta` và `psi` wrap qua ±180°. Palm width không còn là nguồn độ sâu chính khi Pose có arm geometry; mapper cũ vẫn là fallback khi Pose không đủ tín hiệu. Đây là ánh xạ **chuyển động tương đối**, không suy ra tọa độ vật thật từ một webcam RGB.

Palm frame hiện dùng `x_hand = normalize(P17−P5)`, `y_hand` là `(P9−P0)` sau khi bỏ thành phần song song `x_hand`, và `z_hand = x_hand × y_hand`. Cẳng tay dùng `+Y` theo khuỷu→cổ tay; trục còn lại theo mặt phẳng vai–khuỷu–cổ tay. J1 lấy từ **arm azimuth**, còn J5 từ **twist của lòng bàn tay so với cẳng tay**. Khi chỉ xoay bàn tay mà không quét tay, `theta` nên ổn định. Khi quét cả cánh tay với cổ tay giữ nguyên so với cẳng tay, J5 nên ổn định. Nếu cẳng tay gần thẳng, forearm frame không xác định chắc chắn; node dùng palm twist cũ và log cần được đối chiếu.

`beta = q2+q3+q4` theo URDF: gần 0° thì gripper hướng lên, gần 90° thì nằm ngang, gần 180° thì hướng xuống. **Ngón hướng xuống chỉ đổi hướng tool**; TCP hạ khi vị trí cổ tay trong body frame đi xuống. Trục J5 quay quanh hướng gripper, không thay chiều cao TCP. Pinch chỉ thay độ mở, không tự dịch TCP.

### State machine gắp thả

`FREE → APPROACH → DESCEND → GRASP → LIFT → TRANSPORT → RELEASE → FREE`.

`FREE` theo tay người trong giới hạn khả thi. Muốn vào `APPROACH`, người phải đồng thời **chỉ ngón xuống**, **hạ cổ tay** và TCP nằm trong vùng gần mặt bàn; độ cao TCP thấp một mình không ép gripper hướng xuống. `APPROACH` blend beta về top-down theo khoảng cách đến mặt bàn. `DESCEND` giới hạn tốc độ ngang/Z, giữ beta gần hướng xuống. `GRASP` giữ gần vị trí gắp khi pinch đóng. `LIFT` ưu tiên Z và giữ hướng gắp. `TRANSPORT` cho phép đưa vật đi sau khi đã nâng đủ, `RELEASE` mở kẹp rồi trở về điều khiển tự do. Chỉ ngón lên trong lúc approach/descend đưa về `FREE`.

Hiện không có object detector hoặc độ cao vật thật; grasp zone là mặt bàn cấu hình trong `teleop.yaml`. Vật ở cao/thấp vẫn có thể điều khiển bằng `z/r/beta` trong FREE; một grasp state tự động chính xác quanh vật riêng cần object pose hoặc cấu hình vùng vật.

### IK, giới hạn và output

URDF xác nhận J1 quay quanh Z, J2/J3/J4 quanh Y; chiều dài hai đoạn tay đều 0,08285 m. Tool offset dùng `preview_tool_length=0.14624 m`, `preview_tool_x=-0.0048 m`, vai cao `0.1255 m`. Các giá trị này được đọc theo model DOFBOT hiện có và khai báo trong `PreviewIkConfig`, không tự suy đoán chiều dài mới.

Với mục tiêu `(theta,r,z,beta)`, IK tính wrist center:

```text
rw = r - Ltool*sin(beta) - tool_x*cos(beta)
zw = z - shoulder_z - Ltool*cos(beta) + tool_x*sin(beta)
D  = (rw² + zw² - L2² - L3²)/(2*L2*L3)
q3 = ±acos(D)
q2 = atan2(rw,zw) - atan2(L3*sin(q3), L2+L3*cos(q3))
q4 = beta - q2 - q3
```

`|D|>1` là `TARGET_UNREACHABLE`. Hai nhánh khuỷu được chấm theo khoảng cách joint tới pose trước và kiểm tra joint limits/jump. Với beta có dung sai, solver có thể thử beta lân cận để tiến được khi nghiệm chính xác vướng giới hạn. Preview kiểm tra FK/TCP, khoảng hở link so với bàn và giữ pose cuối nếu lệnh không được nhận. Chỉ có kiểm tra mặt bàn và giới hạn hình học hiện tại; **chưa có kiểm tra va chạm mesh đầy đủ hay watchdog phần cứng**. Vì vậy đầu ra vẫn chỉ nối RViz.

### Khi bàn tay bị che

Không yêu cầu đủ năm ngón sau hiệu chỉnh đầu tiên. Vị trí tiếp tục từ cổ tay/arm Pose; J1 cần vai–khuỷu–cổ tay; beta cần palm MCP/hướng ngón; J5 cần palm frame; gripper cần thumb/index. Thiếu một chức năng thì chức năng đó giữ giá trị gần nhất. Khi Hand mất nhưng Pose vẫn thấy cánh tay và đã biết độ lệch cổ tay Hand–Pose, preview tiếp tục cập nhật vị trí; beta/J5/grip giữ. `/hand_teleop/tracking_valid` vẫn `false` trong Pose fallback để không coi đó là bàn tay đo được cho Servo. Mất cả Hand lẫn Pose hoặc observation quá cũ thì giữ pose cuối và báo lý do rõ ràng.

## Diagnostics và đọc log

| Topic/trường | Nội dung |
| --- | --- |
| `/hand_teleop/image_raw`, `/hand_teleop/image_annotated` | Frame nguồn và overlay. |
| `/hand_teleop/observations` | JSON human state, intent, target trụ, joints, WiLoR, timing, trạng thái preview. |
| `/hand_teleop/control_status` | `perception_status`, `mapping_status`, `state_machine_state`, `ik_status`, `safety_status`, `output_status`, `hold_reason`. |
| `/hand_teleop/preview_joint_states` | J1–J5 và gripper của robot ảo. |
| `/hand_teleop/target_pose`, TF `base_link → hand_target` | TCP/hướng tool đã được preview chấp nhận. |

Terminal in `TRAJ` mỗi 1 giây và `TIMING` mỗi 5 giây. `TRAJ human[...]` là nguồn Hand/Pose, `requested_xyz` là yêu cầu, `tcp` là TCP đã nhận, `beta_target` là hướng cần, `beta234` là hướng thực, `J5_twist` là twist, `hold`/`ik_err` nêu giới hạn. `TIMING` báo mean/p50/p95/max của inference, mapping, IK, frame age và pipeline. `finger_img` chỉ là hướng chiếu ngón trên ảnh; khi lòng bàn tay xoay vào trong, nó có thể khác hướng ngón 3D. Overlay hiển thị nguồn, FPS, confidence arm, target trụ, joints, state, hold và latency.

## Cách thử các chuyển động

| Thử | Cử động người | Quan sát đúng |
| --- | --- | --- |
| Base | Quét cả cánh tay trái/phải, giữ cổ tay tương đối với cẳng tay | `theta/J1` đổi; J5 ít đổi. |
| Wrist | Giữ vai–khuỷu–cổ tay gần cố định, xoắn bàn tay | J5 đổi; J1 ít đổi. |
| Reach | Duỗi rồi thu cánh tay | `r` đổi; xoay palm đơn thuần ít làm `r` trôi. |
| Height | Nâng/hạ cổ tay trong frame | `z` đổi; J4 bù để giữ beta cần. |
| Pitch | Chỉ ngón lên/xuống, giữ cổ tay gần cùng vị trí | `beta` đổi, TCP không tự hạ chỉ vì chỉ xuống. |
| Top-down grasp | Chỉ xuống, hạ tay gần bàn, pinch, nâng, mở | `APPROACH→DESCEND→GRASP→LIFT→TRANSPORT→RELEASE`. |
| Tracking loss | Che ngón, sau đó che cả hand | Pose giữ vị trí trong ngắn hạn, hướng/kẹp giữ; mất lâu thì hold. |

Để thu mẫu ba góc, giữ preview chạy và ở terminal khác chạy lần lượt:

```bash
ros2 run hand_teleop capture_sample --label front_0
ros2 run hand_teleop capture_sample --label side_90
ros2 run hand_teleop capture_sample --label side_-90
```

Các nhãn lần lượt chỉ **chính diện, quay người phải, quay người trái** theo cách đặt tên mẫu; xem `body_yaw_deg` trong JSON để xác nhận dấu góc thực tế. Mỗi mẫu nằm dưới `~/hand_teleop_samples/`. Nếu gửi 9 log, giữ từng tư thế 2–3 giây theo thứ tự dưới trái, dưới giữa, dưới phải; trên trái, trên giữa, trên phải; giữa trái, giữa giữa, giữa phải. Ghi thêm hướng ngón lên/xuống, palm vào trong hay ngoài và nguồn `human[hand|pose]`.

## Giới hạn và bước nối robot thật

Webcam RGB đơn không nhìn xuyên được ngón bị che và không đo chính xác chiều sâu vật. Pose world và WiLoR đều có thể sai khi khuất hoặc quay cạnh. Các gain, dấu trục và grasp zone cần kiểm tra với 9 log/video thật của người vận hành. `max_observation_age_s`, joint jump, workspace và table clearance hiện bảo vệ preview; collision mesh, vận tốc/gia tốc joint theo thời gian, watchdog lệnh và e-stop chưa được triển khai cho robot thật. Hãy dùng RViz trước; shadow mode và driver servo riêng chỉ nên nối khi các điều kiện an toàn đó đã được kiểm chứng.
