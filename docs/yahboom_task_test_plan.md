# Kế hoạch test và đọc hiểu các tác vụ Yahboom DOFBOT-SE

## 1. Mục tiêu và phạm vi

Tài liệu này dùng để:

1. Test lần lượt các demo Yahboom từ đơn giản đến khó, theo **bài toán end-to-end**, không test lẻ từng sub-task perception.
2. Sau mỗi bài, hiểu ý tưởng chính, input, output và luồng xử lý.
3. Phân biệt code gốc Yahboom, bản ROS 2 và phần đã sửa riêng cho laptop này.
4. Gộp 3 lớp **CV / Voice / LLM** của cùng một bài toán vào chung một mục để so sánh trực tiếp.

Các nguồn được xét:

- `Dofbot/`: notebook và ví dụ Python trực tiếp của Yahboom.
- `colcon_ws/`: các package/demo Python đời cũ; nhiều package mang metadata ROS 1 nhưng phần thuật toán vẫn dùng lại được.
- `dofbot_ws/`: bản ROS 2, chia thành node/topic/service.
- `rosmaster/`: web app, camera và joystick điều khiển thủ công (phụ trợ).
- `dofbot_voice/`: notebook/script giọng nói Yahboom đời cũ, kết hợp nhận màu và YOLO.
- `LargeModel_ws/`: tầng ROS 2 ASR + LLM + action server điều phối tay máy, camera và mobile base.

Không dùng `dofbot_robot_arm_6dof` trong kế hoạch này.

> Nguyên tắc gộp: mỗi bài toán tiêu biểu có tối đa 3 biến thể:
> - **CV core:** camera → thuật toán → pose/servo, chạy trực tiếp.
> - **Voice variant:** thêm `Speech_Lib`/speech code làm trigger/chọn mode, perception + motion giữ nguyên.
> - **LLM variant:** thêm `asr → model_service → action_service → skill`, LLM chỉ sinh action whitelist, không phát servo trực tiếp.

## 2. Cấu hình máy hiện tại (23-09-2026, đã chốt lại)

- Cổng tay máy: `/dev/ttyUSB0`.
- Camera robot: `/dev/video0` (USB Sonix/Microdia `0c45:6340`), 640x480. `/dev/video1` là node phụ của cùng camera.
- Webcam laptop: `/dev/video2,/dev/video3` (ACER HD User Facing) — không dùng cho task robot.
- `Arm_Lib` đã import được.
- Bài fixed-center color sorting đã chạy thật thành công.
- `usb_cam` chưa được cài, vì vậy ưu tiên OpenCV `VideoCapture('/dev/video0')`.
- Overlay `dofbot_ws/install/setup.bash` đang có tham chiếu hỏng tới `dofbot_moveit`.
- Pipeline IK của `dofbot_info` đã được build lại ngày 23-09-2026 bằng implementation KDL mã nguồn mở (`src/dofbot_kinematics.cpp`, chain `base_link → Gripping_point_Link`, FK recursive + IK NR_JL limits ±1.57). Lib gốc Yahboom vẫn thiếu, nhưng `install/dofbot_info/lib/{libdofbot_kinematics.so,libkin_srv.so}` + binary `kinemarics_dofbot` đã có. Cần export `LD_LIBRARY_PATH` chứa `install/dofbot_info/lib` + `install/dofbot_interface/lib` vì `install/setup.bash` tổng còn hỏng ref `dofbot_moveit`. Đã verify FK/IK roundtrip pose err ~0.
- `dofbot_voice` còn thiếu `Speech_Lib`, `smbus`; các bài YOLO còn thiếu `ultralytics`.
- `LargeModel_ws/install/setup.bash` còn tham chiếu tuyệt đối tới `/home/yahboom/colcon_ws` và `/home/yahboom/dofbot_ws`, nên chưa source sạch trên laptop.
- `LargeModel_ws` chứa cấu hình API/credential ghi trực tiếp trong YAML. Không đưa chúng vào log/tài liệu; phải xác minh nguồn, chuyển sang biến môi trường và rotate nếu là credential thật trước khi kết nối mạng.

Quy tắc test:

- Mỗi lúc chỉ cho **một process** sở hữu `/dev/ttyUSB0`.
- Mỗi lúc chỉ cho **một process** sở hữu `/dev/video0`.
- Test vision-only trước, sau đó mới cho phép gửi lệnh servo.
- Dọn vùng làm việc, chạy không tải trước và luôn sẵn tay ở công tắc nguồn.
- Không chạy demo có đường dẫn `/home/yahboom/...` trước khi đổi sang đường dẫn máy hiện tại.
- Mỗi bài phải ghi lại: `PASS/FAIL`, command, ảnh/video, lỗi và thay đổi đã thực hiện.

## 3. Cách đọc một bài demo (format chung giữ nguyên)

Với mỗi tác vụ, đọc theo thứ tự:

1. **Entry point**: file hoặc command bắt đầu chương trình.
2. **Perception**: camera/ảnh được xử lý bằng HSV, AprilTag, MediaPipe, KCF hay YOLO.
3. **Decision**: kết quả nhận dạng được đổi thành màu, ID, gesture hay loại rác như thế nào.
4. **Motion**: dùng pose servo định sẵn hay gọi IK để tính joint.
5. **Actuation**: lời gọi `Arm_serial_servo_write*()` nào thực sự làm robot chuyển động.
6. **Feedback**: chương trình có đọc servo/topic trạng thái hay chỉ sleep theo thời gian.

Thêm 3 câu hỏi gộp CV/Voice/LLM:

- **Voice thêm gì?** Chỉ trigger/chọn queue màu/action, hay đổi luôn perception?
- **LLM thêm gì?** Prompt → action list nào, whitelist ở `action_service.py` / `promot.py`?
- **Motion có đổi không?** Hay vẫn gọi lại pose/grasp helper của bản CV?

## 4. Thứ tự test tổng quát (đã gộp)

| Giai đoạn | Bài toán end-to-end | Mức rủi ro | Điều kiện |
|---|---|---|---|
| 0 | T0. Camera + đọc servo + pose cố định (prereq) | Thấp–TB | `/dev/video0`, `/dev/ttyUSB0`, vùng tay trống |
| 1 | T1. Color sorting fixed-center (đã PASS) | Trung bình | cube 4 màu, zone define sẵn |
| 2 | T2. Color grab + stacking | Cao | pose gắp và zone/tầng đúng |
| 3 | T3. Follow: color / face / KCF / AprilTag | Trung bình | PID và serial đúng |
| 4 | T4. Gesture grasp / stacking | Cao | MediaPipe + ánh xạ gesture |
| 5 | T5. AprilTag / ArUco sorting + follow | Cao | tag, calibration, IK tùy bài |
| 6 | T6. Garbage / YOLO sorting | Cao | model + pose/bin |
| 7 | T7. Voice action + voice sorting/stack | Trung bình–Cao | speech module/mic, đổi serial+camera |
| 8 | T8. LargeModel orchestration (text → vision → grasp) | Cao nhất | các T1–T6 đã PASS, mock executor trước |
| 9 | Phụ trợ: joystick / web / IK / MoveIt | TB–Cao nhất | gamepad/network; IK native khôi phục sau cùng |

> Không test riêng các sub-task: RGB LED, buzzer, HSV calibration, `Color recognition` hiển thị nhãn, MediaPipe hiển thị landmark chay, `Object_Recognition` hiển thị box chay. Các sub-task đó chỉ là bước debug vision-only bên trong T1–T8.

---

## 5. Danh sách bài toán tiêu biểu để test (chỉ test những dòng này)

| ID | Bài toán | Bản CV core (phải chạy trước) | Bản Voice | Bản LLM | Kết quả mong đợi |
|---|---|---|---|---|---|
| T1 | Color sorting fixed-center | `colcon_ws/src/dofbot_color_sorting/scripts/color_sorting.py` | `dofbot_voice/scripts/voice_color_sorting.py` + `identify_target.py`/`identify_grap.py` | `largemodel_arm/largemodel_arm/color_sorting.py`, `color_remove_higher.py` | thấy màu ở ROI trung tâm → gắp pose cố định → thả đúng zone màu |
| T2 | Color grab + stacking | `color_grab.py`+`put_grab.py`+`action_group.py`; `color_stacking.py`+`stacking_target.py`+`stacking_grap.py`+`stacking_move.py` | `color_grab_broadcast.py`, `put_grab_broadcast.py`, `voice_control_stack.py` | `Stack_Grap_Move.py`, `grasp_desktop*.py`, `Change_Pose.py` | gắp 1 block / xếp chồng nhiều tầng đúng màu và độ cao |
| T3 | Follow mục tiêu | `color_follow.py`+`color_position.py`; `face_follow.py`+`face_position.py`; `KCF_follow.py`; `apriltag_follow.py` | `voice_target_follow.py` | `color_follow_2D.py`, `apriltag_follow_2D.py`, `Obj_follow.py`, `ALM_KCF_Tracker.py`, `Dofbot_Track.py` | giữ tâm target ở giữa ảnh bằng pan/tilt servo |
| T4 | Gesture grasp / stacking | `Gesture_Grasp.ipynb`, `Gesture_Stacking.ipynb`, `gesture_grasp.py`, `gesture_stacking.py`, `11_GestureAction.py`, `17_GestureGrasp.py`, `18_HandCtrlArm.py` | trigger bằng speech code rồi nhận gesture (bản `dofbot_voice` V4) | `mediapipe_gesture.py` trong `dofbot_sorting`, skill grasp trong `largemodel_arm` | gesture ID → pose/chuỗi gắp-xếp định sẵn |
| T5 | AprilTag / ArUco sorting | `apriltag_identify.py`, `apriltag_grasp.py`, `apriltag_sorting.py`+`grasp.py`+`compute_joint5.py`, `aruco_sorting.py` | `speech_*` adapter trong `dofbot_voice_ctrl` / `dofbot_voice` | `apriltag_sorting.py`, `grasp_desktop_apritag.py`, `apriltag_remove_higher.py`, `point_to.py` | tag ID + center/hướng → gắp → zone theo ID |
| T6 | Garbage / YOLO sorting | `7.Garbage_Identify.ipynb`, `yolov11.py`+`yolov11_sortation.py`+`msgToimg.py`, `garbage_identify.py`+`garbage_grap.py` | `garbage_broadcast.py`, `garbage_sorting_broadcast.py`, `speech_garbage*.py` | `yolov11_garbage.py`, `yolov11_sortation.py`, `yolov11_ALM.py` trong `largemodel_arm` | YOLO class+box → nhóm rác → bin/zone tương ứng |
| T7 | Voice action control | `simple_voice_ctrl.py`, `intelligent_voice_ctrl.py`, `action_voice_ctrl.py` (rule `code → servo`) | chính là bản voice của T1–T3,T6 | `asr.py` + `text_chat.py` thay speech board | nói mã/lệnh → gripper/joint/pose/action định sẵn |
| T8 | LargeModel end-to-end | `Detect_Obj.py`, `Get_Target_Pose_KCF.py`, `KCF_Track_Move.py`, `grasp_obj.py`, `Record_Video.py`, `Record_pose.py` | mic/VAD thay keyboard | `text_chat.py` → `model_service.py` → `Rot.action` → `action_service.py` → skill | text/voice → action list whitelist → follow/grasp/sort/point-to có status feedback |

Chi tiết mỗi bài toán ở các mục 6–13 dưới đây, cùng một format: Entry 3 lớp → Input/Output → Luồng → Ý chính → Test an toàn → Blocker.

---

## 6. T1 — Color sorting fixed-center (đã PASS, làm mẫu gộp)

**Entry CV:**
- `colcon_ws/src/dofbot_color_sorting/scripts/color_sorting.py`
- Phụ: `sorting_move.py`, `identify_target.py`, `identify_grap.py`, `robot_controller.py`, `dofbot_config.py`

**Entry Voice:**
- `dofbot_voice/scripts/voice_color_sorting.py` + `identify_target.py`, `identify_grap.py`, `speech_identify_target.py`
- ROS 2 adapter: `dofbot_ws/src/dofbot_color_sorting/speech_color_sorting.py`, `colcon_ws/.../speech_identify_target.py`

**Entry LLM:**
- `LargeModel_ws/src/largemodel_arm/largemodel_arm/color_sorting.py`, `color_remove_higher.py`, `grasp_desktop_remove_color.py`

**Input:** một cube đỏ, xanh lá, xanh dương hoặc vàng đặt ở vùng gắp giữa ảnh `/dev/video0`.

**Output:** cube được gắp từ pose trung tâm và thả vào zone màu tương ứng.

**Luồng chung:**
`camera → crop ROI → BGR→HSV → inRange → morphology close → contour area > 1000 → màu ổn định N frame → pose gắp cố định → pose zone cố định → thả → về pose nhìn`

**Ý chính cần hiểu:**
- Camera chỉ quyết định **màu**, không quyết định pose gắp XY.
- Vị trí gắp và 4 zone đều là mảng 6 góc servo define sẵn.
- Voice chỉ thêm `speech code → update_sort_queue` (cho phép chỉ định màu/thứ tự), không đổi HSV.
- LLM chỉ thêm `text → action list [detect_color, grasp, place]` rồi gọi lại skill sorting.
- Đây là open-loop: dùng `sleep`, không kiểm tra vật có thực sự nằm trong gripper.

**Command chuẩn (CV):**
```bash
cd ~/Desktop/robot-arm
PYTHONPATH="$PWD/colcon_ws/src/dofbot_utils/src:$PWD/colcon_ws/src/dofbot_color_sorting/scripts" \
python3 colcon_ws/src/dofbot_color_sorting/scripts/color_sorting.py \
  --camera /dev/video0 --port /dev/ttyUSB0
```

**Test an toàn:** vision-only (comment motion) → 1 màu → đủ 4 màu → bật voice queue → LLM text mode với 1 action.

## 7. T2 — Color grab + stacking

**Entry CV:**
- Grab: `colcon_ws/src/dofbot_color_grab/scripts/color_grab.py`, `put_grab.py`, `action_group.py`, `color_grab_notebook.py`; notebook `Dofbot/6.AI_Visual/3.color_grab.ipynb`
- Stack: `colcon_ws/src/dofbot_color_stacking/scripts/color_stacking.py`, `stacking_target.py`, `stacking_grap.py`, `stacking_move.py`

**Entry Voice:**
- `dofbot_voice/scripts/color_grab_broadcast.py`, `put_grab_broadcast.py`, `voice_control_stack.py`, `voice_color_identify.py`

**Entry LLM:**
- `LargeModel_ws/.../Stack_Grap_Move.py`, `grasp.py`, `grasp_desktop.py`, `Change_Pose.py`, `Record_pose.py`

**Input:** 1 block (grab) hoặc nhiều block màu (stack).

**Output:** block được gắp tới pose đích / xếp chồng theo tầng với độ cao tăng dần.

**Luồng:** `HSV mask → contour lớn nhất → màu + (tầng counter) → pose gắp → pose tầng → thả`.

**Ý chính:** perception giống T1; khác ở state/count tầng và pose Z/joint cho từng layer. Voice/LLM chỉ đổi cách trigger và thứ tự tầng, motion vẫn open-loop pose định sẵn.

**Test an toàn:** grab 1 block pose cố định → stack 2 tầng → 3-4 tầng → voice chọn màu tầng → LLM `stack [red, green, blue]`.

## 8. T3 — Follow mục tiêu (color / face / KCF / AprilTag)

**Entry CV:**
- Color: `colcon_ws/.../color_follow.py` + `color_position.py`; ROS 2 `dofbot_follow/color_follow_2D.py` + `PID.py`
- Face: `face_follow.py` + `face_position.py`; notebook `6.Face recognition.ipynb`
- KCF: `dofbot_follow/KCF_follow.py` + `Track_Lib.py`
- Tag: `apriltag_follow.py`, `apriltag_follow_2D.py`

**Entry Voice:**
- `dofbot_voice/scripts/voice_target_follow.py` (speech bật/tắt follow, chọn target)

**Entry LLM:**
- `largemodel_arm/color_follow_2D.py`, `apriltag_follow_2D.py`, `Obj_follow.py`, `ALM_KCF_Tracker.py`, `Dofbot_Track.py`, `Get_Target_Pose_KCF.py`, `KCF_Track_Move.py`

**Input → Output:** tâm target `(cx, cy)` → pixel error so với tâm ảnh → PID → góc pan/tilt giữ target ở giữa.

**Luồng:** `detect (HSV/face/KCF/tag) → center → error → PID → servo command`.

**Ý chính:** chỉ khác detector đầu vào; PID + servo follow dùng lại. KCF theo texture ROI nên không cần màu/class nhưng dễ drift khi che khuất. Voice/LLM chỉ bật/tắt follow và chọn target, không thay PID.

**Test an toàn:** camera-only xem box/center → bật 1 servo → cả pan/tilt → đổi target → voice trigger.

## 9. T4 — Gesture grasp / stacking

**Entry CV:**
- Notebook: `Dofbot/6.AI_Visual/1.gesture_action.ipynb`, `4.Gesture recognition.ipynb`, `2.gesture_stack.ipynb`, `Gesture_Grasp.ipynb`, `Gesture_Stacking.ipynb`
- ROS 2: `11_GestureAction.py`, `09_HandCtrl.py`, `14_HandFollow.py`, `18_HandCtrlArm.py`, `12_PoseArm.py`, `16_FingerAction.py`, `17_GestureGrasp.py`
- Package cũ: `colcon_ws/src/dofbot_gesture/scripts/gesture_grasp.py`, `gesture_stacking.py`

**Entry Voice:** speech code mở cổng nhận gesture rồi mới cho grasp/stack (bản V4 `dofbot_voice`).

**Entry LLM:** `mediapipe_gesture.py`, skill grasp trong `largemodel_arm` được gọi từ `action_service.py`.

**Input → Output:** 21 hand landmarks MediaPipe → góc/khoảng cách ngón → gesture ID → pose/chuỗi gắp-xếp định sẵn.

**Ý chính:** MediaPipe chỉ tạo landmark; rule góc ngón tạo gesture class; class chọn action. Không học end-to-end. Voice/LLM chỉ thay nút bấm trigger.

**Test an toàn:** `01_HandDetector`, `10_GestureRecognition` camera-only → `11_GestureAction` pose không tải → `17_GestureGrasp` có tải.

## 10. T5 — AprilTag / ArUco sorting + follow

**Entry CV:**
- Nhận diện: `Dofbot/6.AI_Visual/9.Apriltag_Recognition.ipynb`, `apriltag_identify.py`, `dofbot_driver/apriltag_detect.py`
- Follow: `apriltag_follow.py`, `apriltag_follow_2D.py`
- Grasp động: `apriltag_grasp.py`, `apriltag_sorting.py` + `grasp.py` + `compute_joint5.py` + `vutils.py`, `aruco_sorting.py`, `aruco_gesture_height.py`

**Entry Voice:** adapter `dofbot_voice_ctrl` AprilTag/KCF/YOLO (voice chọn ID/mode).

**Entry LLM:** `largemodel_arm/apriltag_sorting.py`, `grasp_desktop_apritag.py`, `apriltag_remove_higher*.py`, `point_to.py`, `ALM_Point_To.py`.

**Input → Output:** ảnh grayscale → tag corners/center/ID (+ pose nếu có intrinsics) → pose gắp → zone theo ID. Bốn góc tag còn cho hướng joint 5.

**Luồng follow:** `tag center → pixel error → PID → pan/tilt`. **Luồng sort:** `ID + center/hướng → IK hoặc pose → grasp → zone`.

**Trạng thái:** nhận diện + follow test được ngay; gắp tọa độ tự do đang bị chặn bởi IK native thiếu (`libkin_srv.so`, `libdofbot_kinematics.so`).

**Test an toàn:** identify camera-only → follow → grasp pose cố định theo ID → dynamic grasp sau khi có IK.

## 11. T6 — Garbage / YOLO sorting

**Entry CV:**
- Notebook: `Dofbot/6.AI_Visual/7.Garbage_Identify.ipynb`, `8.Object_Recognition.ipynb`
- ROS 2: `dofbot_yolov11/yolov11.py` + `yolov11_sortation.py` + `msgToimg.py`, `dofbot_garbage_yolov11/garbage_identify.py` + `garbage_grap.py` + `identify_target.py`
- Model: `best.onnx` / `best.pt` (trong `Dofbot` và `dofbot_voice/scripts/`)

**Entry Voice:**
- `dofbot_voice/scripts/garbage_broadcast.py`, `garbage_sorting_broadcast.py`, `garbage_identify.py`, `garbage_grap.py`, `speech_garbage*.py`
- ROS: `dofbot_garbage_yolov11/speech_garbage*.py`, `single_garbage_identify.py`

**Entry LLM:**
- `largemodel_arm/yolov11_garbage.py`, `yolov11_sortation.py`, `yolov11_ALM.py`, `Detect_Obj.py`

**Input → Output:** frame → YOLO class + confidence + box → nhóm rác → bin/zone tương ứng.

**Luồng ROS:** `/image_raw → msgToimg → /image_data → YOLO → detection/center → IK hoặc pose → grasp → zone`.

**Ý chính:** YOLO thay HSV để nhận class phức tạp; voice dùng để trigger và đọc kết quả; LLM dùng `seewhat()`/`Detect_Obj` rồi dispatch grasp. Motion vẫn pose/bin định sẵn nếu chưa có IK.

**Trạng thái:** cần sửa path `/home/yahboom/...`, xác nhận model tồn tại, cài `ultralytics`, đổi camera `0→2`. Test inference-only trước, gắp thật sau.

## 12. T7 — Voice action + voice sorting/stack (lớp Voice gộp)

Bản chất `dofbot_voice`: demo trực tiếp, không phải ROS package hoàn chỉnh. Ghép 3 lớp: module speech phần cứng qua `Speech_Lib`/I2C + rule `mã lệnh → hàm/action` + camera HSV/YOLO và pose định sẵn. Chưa phải LLM.

**Entry theo độ khó (đã gộp vào T1–T4,T6, liệt kê lại để chạy):**
- V1 read-only: `02.Loop_detection.ipynb`, `03.Password_trigger_mode.ipynb`, `11.Learning_model.ipynb` + `learnning_model.py` — `speech_read()` polling mã, chưa gọi `Arm_Lib`.
- V2 broadcast: `05.Voice_Chinese_broadcast.ipynb`, `06.Phonetic_Pinyin_Broadcast.ipynb`, `07.Voice_Number_broadcast.ipynb` — test loa độc lập.
- V3 action đơn: `08.Simple_voice_control.ipynb` + `simple_voice_ctrl.py`, `09.Voice_intelligent_control.ipynb` + `intelligent_voice_ctrl.py`, `10.Voice_action_control.ipynb` + `action_voice_ctrl.py` — `code → Arm_serial_servo_write*`, dance/clamp/move/heap-up.
- V4 color/follow/stack/sort: `voice_color_identify.py`, `color_grab_broadcast.py`, `put_grab_broadcast.py`, `voice_target_follow.py`, `voice_control_stack.py`, `voice_color_sorting.py` + `identify_target.py`/`identify_grap.py`.
- V5 garbage: `garbage_broadcast.py`, `garbage_sorting_broadcast.py` + `garbage_identify.py`/`garbage_grap.py`.

**Blocker:** `Speech_Lib`, `smbus` chưa import được; nhiều file dùng `/dev/myserial`, camera `0`, audio `/home/yahboom/speech_music`.

**Test an toàn:** V1 read-only PASS mới sang V2 loa → V3 từng action đơn không tải → V4/V5 camera-only rồi mới gắp thật.

## 13. T8 — LargeModel orchestration (lớp LLM gộp)

Kiến trúc: `voice hoặc text → ASR topic → model_service/LLM → action list → action_service → node camera/grasp/follow → trạng thái → LLM/TTS`. LLM không phát servo trực tiếp.

**Entry:**
- Hợp đồng: `interfaces/action/Rot.action`, `arm_msgs`, `arm_interface`
- Não: `largemodel/asr.py`, `model_service.py`, `action_service.py` (file trung tâm, có side effect servo ngay khi import), `utils/large_model_interface.py`, `utils/promot.py`
- Input thay thế mic: `text_chat/text_chat.py` (publish topic `asr`)
- Skill: `largemodel_arm/*.py` (`Detect_Obj`, `ALM_KCF_Tracker`, `KCF_Track_Move`, `grasp_obj`, `point_to`, `Record_Video`, `Record_pose`, `Change_Pose`, `yolov11_*`, `apriltag_*`, `color_*`), `dofbot_pro_kcf/*`, `testtime/cam.py`
- Launch: `largemodel/launch/largemodel_control.launch.py`, `largemodel_arm/launch/*.launch.py`
- Config: `largemodel/config/yahboom.yaml`, `large_model_interface.yaml` (có credential hard-code — phải chuyển env + rotate), `map_mapping.yaml` (mobile base, không dùng cho DOFBOT để bàn)

**Luồng message:**
1. `asr.py` VAD+ASR → topic `asr`/`wakeup`, hoặc `text_chat.py` publish `asr` từ bàn phím.
2. `model_service.py` gọi model/Dify, parse JSON/action list → goal `Rot` tới `action_service`.
3. `action_service.py` dispatch whitelist: pose/gripper/track/grasp/point-to/sort/record/stack + `/cmd_vel` (cấm chạy nav trên bàn).
4. Skill publish `/largemodel_arm_done`; action server publish `actionstatus`; `model_service` tiếp tục vòng suy luận; `text_response`/`tts_topic` trả lời.

**Thứ tự test an toàn:**
- L1: build `interfaces` + `text_chat`, check `ros2 topic echo /asr`, không chạy `action_service`.
- L2: `model_service` + mock executor, kiểm tra JSON chỉ chứa whitelist, không sinh shell/joint tùy ý.
- L3: `action_service` với I/O vô hại (`light_on/off`, `beep`), chưa arm motion.
- L4: gripper/1 joint/pose/dance sau khi T0 PASS.
- L5: vision-only skills (`Detect_Obj`, KCF tracker, AprilTag/YOLO inference, `Record_Video`).
- L6: follow → point-to → grasp fixed → stacking.
- L7: LLM end-to-end bằng text (`text_chat_mode:=true`): đèn/beep → pose → vision/grasp.
- L8: thêm mic/ASR/TTS. L9 navigation chỉ khi có mobile base + bản đồ thật.

**Blocker:** overlay copy từ `/home/yahboom` nên source lỗi; launch gọi `dofbot_info` camera+IK thiếu lib; camera hard-code `/dev/video0`; `Arm_Device()` mặc định `/dev/myserial`; model local `/home/yahboom/MODELS/...` chưa có; online cần mạng/quota/credential.

---

## 14. Phụ trợ và việc để cuối (không phải bài toán chính)

### T0. Prereq: camera, servo, pose cố định
- Camera preview: `Dofbot/6.AI_Visual/0.摄像头驱动教程.ipynb`, `rosmaster/camera.py`, `rosmaster/open_camera.py` — `VideoCapture('/dev/video0')` → `imshow` 640x480.
- Đọc servo: `Dofbot/3.ctrl_Arm/4.read_servo.ipynb` + `Arm_Lib.py` — read-only serial.
- Pose: `1.rgb.ipynb`, `2.beep.ipynb`, `3.ctrl_servo.ipynb`, `5.ctrl_all.ipynb`, `6.left_right.ipynb`, `7.dance.ipynb`, `8.study_mode.ipynb`, `9.clamp_block.ipynb`, `10.move_block.ipynb`, `11.heap_up.ipynb` — primitive `Arm_serial_servo_write6_array()`.

### Joystick / Web
- Joystick: `rosmaster/joystick.py`, `handle_test.py` — button/axis → joint/gripper/RGB/buzzer.
- Web: `rosmaster/start_app.sh`, `camera.py`, `templates/index.html`, `config.ini`, `yb-discover.py` — request/socket → servo/camera stream.

### ROS 2 driver / IK / MoveIt (để cuối)
- Driver: `dofbot_driver/dofbot_driver.py`, `arm_driver.py` — bridge ROS ↔ serial, cấm forward state giả sang robot thật.
- Kinematics: `dofbot_info/src/kinemarics_dofbot.cpp` + `Kinemarics.srv` + URDF — **BLOCKED**, thiếu 2 lib native.
- MoveIt/RViz: `dofbot_moveit/launch/` — chỉ test sau cùng, không nối trajectory giả sang servo thật.

---

## 15. Ma trận file trùng chức năng (CV / Voice / LLM)

| Bài toán | Bản direct/notebook | Bản package cũ | Bản ROS 2 | Bản Voice trigger | Bản LLM skill |
|---|---|---|---|---|---|
| Color sorting | — | `dofbot_color_sorting/color_sorting.py` | `dofbot_sorting*/color_sorting.py` + `grasp.py` | `voice_color_sorting.py` | `largemodel_arm/color_sorting.py` |
| Color grab/stack | `3.color_grab.ipynb` | `dofbot_color_grab`, `dofbot_color_stacking` | grasp/stack variants trong sorting | `color_grab_broadcast.py`, `voice_control_stack.py` | `Stack_Grap_Move.py`, `grasp_desktop*.py` |
| Follow | — | `dofbot_color_follow`, `dofbot_face_follow` | `dofbot_follow/*_follow*.py` | `voice_target_follow.py` | `largemodel_arm/*follow*.py`, `*Tracker.py` |
| Gesture | `1.gesture_action.ipynb`, `Gesture_Grasp/Stacking.ipynb` | `dofbot_gesture` | `dofbot_mediapipe/*`, `mediapipe_gesture.py` | speech gate trong `dofbot_voice` V4 | skill grasp qua `action_service` |
| AprilTag | `9.Apriltag_Recognition.ipynb` | `dofbot_apriltag` | `dofbot_driver`, `dofbot_follow`, `dofbot_sorting*` | adapter trong `dofbot_voice_ctrl` | `apriltag_sorting.py`, `point_to.py` |
| Garbage YOLO | `7.Garbage_Identify.ipynb` | — | `dofbot_yolov11`, `yolov11_garbage.py` | `garbage_*broadcast.py` | `largemodel_arm/yolov11_*.py` |
| Action control | notebooks servo | — | `dofbot_driver` | `simple/intelligent/action_voice_ctrl.py` | `action_service.py` whitelist |
| Manual | notebooks servo | — | `dofbot_driver` | — | `text_chat.py`, `asr.py` input |

Nguyên tắc: test bản CV đơn giản để hiểu thuật toán trước; chỉ test bản Voice khi cần trigger giọng nói; chỉ test bản LLM khi cần multi-skill orchestration hoặc gắp tọa độ động.

## 16. Mẫu ghi kết quả sau mỗi bài

```text
Task:
Ngày test:
Entry/command:
Camera/serial:
Input vật lý:
Kết quả: PASS | PARTIAL | FAIL | BLOCKED
Robot có chuyển động không:
File chính đã đọc:
Ý tưởng học được:
Lỗi gặp phải:
Thay đổi đã làm:
Ảnh/video/log:
Bước tiếp theo:
```

Log lưu ở `/home/jloy/Desktop/robot-arm/log/YYYY-MM-DD_<T-id>_<ten-bai>.md`, kèm ảnh nếu có.

## 17. Thứ tự thực tế khuyến nghị (theo T1–T8)

Vì T1 đã PASS:

1. T0 prereq: camera preview, read-servo, 1 pose không tải.
2. T1: voice queue + LLM text 1 action cho sorting.
3. T2: grab 1 block → stack 2–4 tầng.
4. T3: color follow camera-only → servo PID; rồi face → KCF → tag follow.
5. T4: gesture camera-only → action → grasp/stack.
6. T5: tag identify → follow → grasp theo ID (dynamic grasp chờ IK).
7. T6: YOLO inference-only → broadcast → sorting thật.
8. T7: voice V1 read-only → V3 action đơn → V4/V5 ghép T1–T3,T6.
9. T8: L1 text topic → L2 mock planner → L3 I/O vô hại → L4 pose → L5 vision skill → L6 grasp → L7 text e2e → L8 ASR/TTS.
10. Phụ trợ + IK/MoveIt cuối cùng.
