# Bringup camera → nắp trắng trong RViz (làm đúng thứ tự từng bước)

Mức test: **camera thật + tay ảo (hiển thị) + nắp thật**.
Nguyên tắc: chưa PASS bước trước thì không sang bước sau.
Mọi lệnh chạy tại workspace root `dofbot_robot_arm_6dof`.

```bash
cd /home/jloy/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Bước 0 — Camera live (PASS 1)

1. Kiểm tra OS thấy camera:
   ```bash
   ls -la /dev/video*
   ```
   Thấy `/dev/video0` là được (`/dev/video1` thường là node metadata, bỏ qua).
2. Test qua node ROS (chụp 1 frame rồi Ctrl-C):
   ```bash
   timeout -s INT 10 ros2 run cap_vision camera_test \
     --ros-args -p snapshot_path:=/tmp/cap0.png
   ```
   Đổi camera khác: thêm `-p device_index:=1`.
3. PASS 1 khi: log `Da luu snapshot: /tmp/cap0.png`, mở ảnh thấy hình rõ nét.

> Nắp trắng nhạy sáng: nên khóa exposure/white-balance camera
> (`sudo apt install -y v4l-utils` rồi chỉnh `v4l2-ctl`) trước khi calib.

## Bước 1 — Cố định camera + chọn gốc bàn

1. Gắn cứng camera nhìn xuống bàn. **Từ đây cấm đụng vào camera**
   (dời/cao/xoay đều làm sai mapping, phải calib lại).
2. Kẻ workspace, chọn gốc O của `table_frame` (mặc định: bàn 300×250mm,
   O = góc top-left).
3. Đặt 4+ vật đánh dấu tại vị trí đã đo đúng bằng thước, ghi lại tọa độ
   mét theo đúng thứ tự, ví dụ mặc định:
   `(0,0) → (0.30,0) → (0.30,0.25) → (0,0.25)`.

## Bước 2 — Calibrate homography

1. Chụp ảnh bàn đã đặt vật đánh dấu:
   ```bash
   timeout -s INT 8 ros2 run cap_vision camera_test \
     --ros-args -p snapshot_path:=/tmp/calib.png
   ```
2. Calib (click theo đúng thứ tự world_points đã đo ở Bước 1):
   ```bash
   ros2 run cap_vision calibrate_table -- \
     --image /tmp/calib.png \
     --output src/cap_vision/config/homography.yaml
   ```
   Trong cửa sổ ảnh: click từng điểm → `s` lưu → `r` làm lại → `q` thoát.
   Không màn hình: thêm `--pixels "u1,v1 u2,v2 ..."`; số đo khác mặc định:
   thêm `--world "x1,y1 x2,y2 ..."`.
3. PASS khi `reprojection_error_px` nhỏ (vài px) rồi sang **gate thước**.

## Bước 3 — Gate thước (bắt buộc, sai thì DỪNG)

```bash
ros2 run cap_vision test_pixel_to_xy -- \
  --homography src/cap_vision/config/homography.yaml \
  --click --image /tmp/calib.png
```

Click vào điểm đã đo thước → console in `(X, Y)` mm → so với thước thật.
**PASS 2 khi sai số < 10mm.** Sai hơn: kiểm tra lại thứ tự click,
điểm đo, camera có bị đụng không — tuyệt đối không thêm detector.

## Bước 4 — 1 nắp trắng trong RViz

1. Chạy pipeline:
   ```bash
   ros2 launch cap_vision cap_mapping.launch.py
   ```
2. Mở RViz: Fixed Frame = `world`, add MarkerArray topic `/caps_markers`.
   Sẽ thấy khung bàn xanh + cylinder trắng Ø30×12mm tại vị trí nắp.
3. PASS khi: **dịch nắp thật 30mm → nắp ảo dịch ≈30mm** cùng hướng.

Tinh chỉnh HSV-trắng tại chỗ nếu thiếu/thừa detection
(`-p white_lo:="[...]" -p white_hi:="[...]"` cho `cap_detector`).

## Bước 5 — N nắp trắng (checkpoint)

Đặt nhiều nắp → RViz hiện đủ N cylinder, layout khớp bàn thật.
Xong bước này là hết phase camera; tay ảo (MoveIt fake) chỉ mở cùng
scene để đối chiếu vị trí, chưa IK/pick.

## Đổi camera / dời camera sau này

Chỉ cần làm lại **Bước 2 → Bước 3** (vài phút), không sửa code.
`device_index` khác thì truyền vào launch: `device_index:=1`.

## Khi robot thật tới (ngoài phạm vi đợt này)

1. Đo TF `base_link → table_frame`, thay static TF identity trong
   `cap_mapping.launch.py` (`table_x/y/z/yaw`).
2. Marker → MoveIt CollisionObject → IK/pick.
