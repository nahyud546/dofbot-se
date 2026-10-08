# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Repo điều khiển tay máy Yahboom DOFBOT 5 khớp + kẹp (ROS 2 Humble, Ubuntu 22.04, Python 3.10): nhận diện cube 30 mm
(AprilTag / màu / hình rác), trợ lý T8 ra lệnh bằng tiếng Việt để sort và xếp tầng, hiệu chuẩn camera tay và camera
ngoài. Người dùng làm việc bằng tiếng Việt; comment và thông báo trong code cũng bằng tiếng Việt.

## Lệnh thường dùng

```bash
# Môi trường (PYTHONPATH cho projects/, vendor/; rồi ROS + workspace ros/)
source scripts/setup/setup_env.sh && source scripts/setup/setup_ros.sh

# Build workspace perception. PHẢI dùng Python hệ thống: tắt .venv trước, nếu không CMake lấy nhầm python của venv
cd ros && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# Test (cấu hình trong pyproject.toml; .venv không có pytest nên dùng python hệ thống)
/usr/bin/python3 -m pytest                                   # tất cả; test cần ROS tự skip khi chưa source
/usr/bin/python3 -m pytest projects/cube_vision              # một thư mục
/usr/bin/python3 -m pytest projects/t8_pipeline/test_batch_loop.py::Sorting::test_user_cancel_stops_the_whole_batch
# Đủ bộ (có test perception): source ROS + ros/install/setup.bash trước. Bộ vision_experiments mất ~6 phút (nạp DINO).

# T8 thật (cần ROS đã source trong terminal; tự bật perception, tự nhận camera tay)
python projects/t8_pipeline/t8_assistant.py --vision-backend ros3d --enable-motion --approval viewer \
  --text "sort cube vàng vào đúng zone"

# Công cụ kiểm tra không cần T8
python -m cube_vision.cameras --identify                      # camera nào là camera tay (xoay J1 rồi so ảnh)
python -m cube_vision.placement_check                         # camera ngoài thấy đủ 4 ô màu không
python projects/t8_pipeline/zone_survey.py --zones 3 4        # đo vị trí ô thả, không gắp
python projects/vision_experiments/calibrate_external.py --status
python projects/vision_experiments/dofbot_frames.py --dump     # bảng hệ tọa độ + ma trận hiện tại
python projects/vision_experiments/build_world.py --scan       # tay nhìn quanh, dựng world (data/world/latest.json)
ros2 launch cap_vision world_view.launch.py                    # xem world trong RViz
```

Lỗi test có sẵn, không liên quan thay đổi mới: `test_cube_sort_stage1.py::test_pick_waits_for_confirmed_id_and_current_top_face`.

## Kiến trúc

Bốn tiến trình, hai trình thông dịch Python. Đây là điều khó thấy nhất khi chỉ đọc từng file:

```
ros/src/cap_vision (python hệ thống, ROS)        projects/t8_pipeline (python .venv)
  camera_test ──/cap_vision/image_raw──▶ object_perception ──/vision/object_states, /vision/face_quads─┐
  real_joint_mirror ──/real_joint_states (đọc servo, ~1,5 s/mẫu)──────────────────────────────────────┤
                                                                                                        ▼
  t8_assistant ─▶ t8_pipeline (Gemini lập kế hoạch) ─▶ t8_executor ─▶ t8_ros_tasks.RosTaskRunner
                       ├─ t8_ros_scene ⇄ t8_ros_scene_worker  (subprocess python HỆ THỐNG: đọc scene, viewer duyệt Space)
                       └─ t8_motion    ⇄ t8_motion_worker      (subprocess giữ cổng serial, chạy IK + servo)
```

- **Vì sao tách tiến trình**: `cv_bridge`/rclpy của ROS Humble không tương thích NumPy trong `.venv` (torch, DINO).
  Mọi thứ nói chuyện ROS chạy bằng `/usr/bin/python3`; T8 và mô hình AI chạy trong `.venv`. Worker tự source ROS.
- **Tọa độ cube không lấy từ TF của perception** (thường `None`). `t8_ros_scene_worker` tự tính XY + tầng từ pixel +
  khớp thật + hand-eye (`vision_experiments/cube_layer.locate_cube`, `gravity_pose` khi chỉ thấy một mặt), nên chạy được
  từ pose bất kỳ chứ không buộc READY_POSE.
- **Hệ tọa độ** khai báo một chỗ: `vision_experiments/dofbot_frames.py` (đồ thị `world`/`base_link`/…/camera), toán ở
  `cube_vision/frames.py`, ghi chú ở `docs/architecture/frames.md`. Quy ước `a_T_b`: điểm hệ b → hệ a.
- **World chung cho nhiều camera**: camera tay đo tag bằng nhiều góc nhìn (`active_view.py` tự nhìn thêm khi luật
  trong `cube_vision/view_quality.py` báo chưa chắc) → `cube_vision/world_map.py`; camera khác tự định vị bằng tag
  đã có trong world (`cube_vision/camera_pose.py`). Chi tiết và lệnh: `docs/architecture/frames.md`.
- **`projects/cube_vision/`** là thư viện độc lập: không được import T8 hay ROS. Robot cụ thể đi vào qua tham số
  (`ZoneLayout`, `arm.execute("look")`, `scene.zone_survey`). T8 nối vào bằng adapter mỏng (`t8_pipeline/zone_survey.py`).
- **`projects/vision_experiments/` và `projects/t8_pipeline/`** là module phẳng, import nhau bằng tên trần
  (`import cube_layer`, `import t8_motion_worker`); đường dẫn do `pyproject.toml` (pytest) và `setup_env.sh` cấp.
  Script entry-point giữ một dòng bootstrap `sys.path`; module thư viện dùng `cube_vision._paths.ensure(...)`.
- **Một lệnh sort** đi qua: khảo sát zone (camera tay xoay nhìn ô, đo tâm ô trong base, ô khuất thì suy từ đối xứng) →
  viewer duyệt Space → preflight IK → gắp/thả → camera ngoài so ảnh trước/sau → lệch hoặc văng thì thử lại, mỗi lần thả
  thấp và chậm hơn. Lệnh "tất cả cube" (`label: "all"`) lập thứ tự một lần rồi lặp các bước đó.
- **Đường dẫn workspace ROS** có một nguồn: `<repo>/ros`, ghi đè bằng `$ROBOT_ARM_ROS_WS`
  (`scripts/tools/repo_paths.py`, `cube_vision.registry.ros_workspace`).

## Dữ liệu hiệu chuẩn (`config/robot/`)

- `hand_eye.json` — camera tay ↔ arm4 (K, k1, `arm4_T_optical`, `tag_top_z`). Chỉ tin trong J1 35–140°.
  `hand_eye.center_proven.json` là mốc đã kiểm chứng bằng gắp thật: **không xóa, không ghi đè**.
- `external_camera.json` — camera ngoài ↔ base (1280×720). Camera ngoài là webcam dễ bị xê dịch: mọi chỗ dùng số đo mét
  phải kiểm `ExternalCalibration.moved(frame)`. Tạo bằng `calibrate_external.py` (xem skill `calibrate-cameras`).
- `cube_6d_calibrated.yaml` — config perception sinh từ hand-eye; `cube_4x6_face_geometry.yaml` — mặt nào của cube in gì.

## Bất biến an toàn

- Mọi chuyển động gắp/thả phải qua viewer và phím Space của người dùng. Không thêm đường tắt tự duyệt.
- Giới hạn có chủ ý, đừng nới khi chưa đo trên tay thật: J1 thả ô 10–170° (`ZONE_J1_LIMITS`), hand-eye 35–140°,
  sàn thả `RELEASE_FLOOR_Z` 0,050 m, tháp tối đa 4 tầng.
- Zone `blocked` (ô thấy được nhưng tay không tới, hoặc điểm thả cấu hình nằm trên ô khác) thì từ chối trước khi gắp;
  không rơi về bảng `BIN_*` cứng.
- Sau khi chạy phần cứng: kiểm `fuser /dev/video0 /dev/video2 /dev/ttyUSB0`, dừng tiến trình sót **theo pid**
  (không `pkill -f` theo mẫu: dễ giết nhầm chính shell đang chạy). Không tắt tiến trình của người dùng.

## Bẫy đã gặp

- **Ảnh ROS phải nhận bằng QoS RELIABLE.** Best-effort làm rớt khung 900 KB (còn ~3 khung/s, đứt 1–4 s) → viewer đen,
  tracking hụt. Xem `image_qos()` trong `t8_ros_scene_worker.py`.
- **Chưa source ROS thì T8 tưởng không có camera** và bật trùng perception; `ensure_cube_perception` giờ báo rõ.
  Khi gọi lệnh đừng ghi đè `PYTHONPATH` (mất đường dẫn rclpy): dùng `PYTHONPATH=vendor/yahboom:$PYTHONPATH`.
- **Số `/dev/videoN` hay đổi** giữa camera tay và camera ngoài: dùng `--camera auto` (mặc định), đừng hard-code.
- **Camera ngoài** cần ~1 s tự phơi sáng (khung đầu tối); tag 20 mm chỉ đọc được ở 1280×720 (MJPG), không phải 640×480.
- **Trọng số mô hình không nằm trong git** (`*.pt`, `mobileclip_blt.ts`): đặt ở `ai/models/segmentation/`,
  `ai/models/clip/`; `ros/mobileclip_blt.ts` là symlink vì ultralytics tìm file đó ở thư mục chạy.
- `workspaces/dofbot_robot_arm_6dof/` là kho git riêng (chess, teleop, MoveIt), bị ignore. Perception đang dùng đã được
  chép sang `ros/src`; sửa perception thì sửa ở `ros/src`, không sửa bản trong kho lồng.
- `workspaces/dofbot_ws`, `LargeModel_ws`, `legacy`, `vendor/yahboom` là mã Yahboom gốc: không thuộc luồng T8.
