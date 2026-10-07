# T8 — Gemini và quan sát ROS 3D

## Chạy phiên hiện tại

Giữ perception ROS đang chạy bằng lệnh `cube_6d_camera.launch.py` đã dùng.
T8 đọc `/vision/object_states` và `/real_joint_states`; không mở camera riêng.

```bash
cd /home/jloy/Desktop/robot-arm
source .venv/bin/activate
python projects/t8_pipeline/t8_assistant.py
```

Mặc định là `--vision-backend ros3d`: quan sát và dry run, không mở serial,
không đưa tay về ready pose, không tự xoay/gắp. Worker ROS dùng Python hệ thống
và tự source hai workspace để tránh xung đột NumPy với `.venv`.

Ví dụ câu lệnh:

- `trên bàn đang thấy những cube nào`
- `tay đang ở trạng thái nào, đã hiệu chuẩn chưa`
- `tìm cube đỏ đang ở đâu`
- `tự quan sát tìm cube đỏ rồi gắp đặt lên cube id 1`
- `bạn có thể làm gì trong chế độ hiện tại`

Lệnh cuối về tác vụ xếp được Gemini chuyển thành **một `stack_cubes`**.
Executor xác thực nguồn/đích và preview 5 stage: approach, pick, lift, hover,
place. Kết quả `planned` không có nghĩa là đã gắp được vật.

### ROS 3D có chuyển động và xác nhận người dùng

```bash
python projects/t8_pipeline/t8_assistant.py \
  --vision-backend ros3d --enable-motion --approval viewer
```

Với gắp/xếp, T8 chuẩn bị và xác nhận pose quan sát, trả serial cho perception,
rồi mở viewer. Terminal in tiến độ và đường dẫn log `/tmp/t8_approval_<session>.log`.
Viewer ưu tiên ảnh annotated với skeleton/TCP đã được perception làm mượt bằng
median + EMA; ảnh raw chỉ dùng khi annotated hết hạn. T8 không vẽ thêm bbox/nhãn
lên từng cube. Trạng thái hiển thị được ổn định trong 1 giây, READY cần target
hợp lệ liên tục 1 giây trên cùng track; mất dữ liệu hợp lệ khóa Space ngay.
Chỉ khi ảnh/scene/pose còn mới và cube có ID/model 3D nhất quán qua các frame,
viewer mới hiện `READY - PRESS SPACE`. Nhấn **Space** tạo target lock có hạn
15 giây; **Esc**, đóng cửa sổ hoặc Ctrl+C hủy. T8 chạy preflight từ lock trước
khi gửi motion. Viewer mặc định chờ không giới hạn; `--approval-timeout 120`
đặt thời hạn 120 giây. `--pick-x-offset-mm` mặc định 15, áp dụng mapping fixed-pose.
Độ bù đặt mặc định là `--place-correction-x-mm 12 --place-correction-y-mm 0`
theo trục kẹp; đo độ lệch sau lần đặt thử rồi chỉnh hai giá trị này nếu cần.

### Chế độ hand-eye (mặc định khi đã hiệu chuẩn, không cần READY_POSE)

Chạy một lần `projects/vision_experiments/calibrate_hand_eye.py --cube-id 3`
(tay tự di chuyển, đặt 1 cube cách chân robot ~15 cm). Công cụ ghi
`config/robot/hand_eye.json` (mount camera, K, k1, độ cao mặt tag) và
`config/robot/cube_6d_calibrated.yaml` (K/k1 cho perception; `calibrated` vẫn
`false` nếu kiểm chứng PnP 3D chưa đạt 5 mm). T8 tự dùng file YAML này khi tự
khởi động perception.

Khi `hand_eye.json` đạt và `/real_joint_states` đang chạy, T8 tính XY **giống
search-center**: pixel tâm mặt trên + góc khớp đọc thật (tay phải đứng yên) +
mount đo được, giao tia với mặt phẳng của **tầng** cube (`cube_layer.py`). Tầng
chọn bằng cách so khoảng cách PnP với từng giả thuyết 0/1/2 (mỗi tầng cách
nhau ≥ 30 mm dọc tia, lớn hơn nhiều sai số PnP); không khớp tầng nào thì từ chối
thay vì đoán. Độ cao gắp = 0,047 m + 0,030 m × tầng; đặt lên đích = TCP đích +
0,030 m. Tháp tối đa 4 cube: gắp được cube ở tầng 0–3, đặt lên đích ở tầng 0–2 (IK đã kiểm
chứng; sát chân robot x ≥ −0,14 m ở độ cao tháp có thể không có nghiệm và bị
preflight báo rõ). Tay giữ
nguyên pose hiện tại (`prepare` với `keep_pose`) thay vì về READY_POSE. Sau khi
xếp, T8 xác minh bằng cách tìm lại cube nguồn tại XY của đích và tầng + 1 (tag
của cube đáy đã bị che). Nếu không có khớp hoặc hand-eye chưa đạt, T8 tự về
chế độ fixed bên dưới.

Xếp chồng ở chế độ cũ có hai chế độ tọa độ. Khi scene đã hiệu chuẩn, T8 dùng pose
`base_link`. Khi cấu hình mặc định còn `calibrated: false`, T8 dùng fallback
`fixed_ready_pose`: lệnh `prepare` phải đọc lại đúng các joint của READY_POSE,
sau đó viewer khóa ID và mapping pixel→XY đã canh trong `cube_sort_3d.py` mới
được phép chạy. Mapping chỉ phù hợp với cube nằm trên bàn ở pose này. Preflight
trả đúng các waypoint tiếp cận/gắp/nâng/hover/đặt mà worker sẽ chạy; worker kiểm
tra lại FK trước khi gửi servo. Khi đặt fixed-pose, bù lệch kẹp 12 mm được áp
dụng nhất quán trong preflight và chuyển động. Viewer không thể xác minh chiều cao
chồng ở chế độ fixed, vì vậy T8 không nạp hay ghi nhớ số tầng chưa được xác minh;
xếp tầng tiếp theo cần scene base đã hiệu chuẩn. Không dùng mapping này sau khi
camera/tay đổi pose.

Chế độ fixed không thay thế hand-eye calibration tổng quát. Quy trình chụp ảnh,
đo hand-eye và kiểm chứng nằm trong
[hướng dẫn cube 3D](../vision_experiments/README.md).
Sau khi có file YAML đo đạt, có thể để T8 khởi động perception với
`--ros-config /path/cube_6d_measured.yaml`, hoặc tự chạy Terminal 1 với
`config:=/path/cube_6d_measured.yaml`. Nếu Terminal 1 đã chạy, T8 dùng chính
process đó; cờ `--ros-config` không thay đổi cấu hình của process đang chạy.
Không có file hiệu chuẩn đủ dữ liệu trong workspace hiện tại.
Nếu dùng AprilTag cố định trên bàn, công cụ
`ros2 run cap_vision collect_tag_hand_eye` chụp tương tác: tay phải được đưa
thủ công đến ít nhất 11 góc nhìn khác nhau, nhấn Space khi tag rõ và tay đã
dừng; nó không gửi lệnh motor. Công cụ này đã được sửa để đợi frame có TF
cùng timestamp khi joint chậm hơn camera. Đo cạnh hình vuông đen của tag và
chiều cao mặt tag so với bàn bằng thước thực trước khi truyền
`--tag-size-m` và `--tag-to-table-m`. Ví dụ với các kích thước đã đo:

```bash
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
ros2 run cap_vision collect_tag_hand_eye -- \
  --output /path/new_tag_run --tag-id 3 \
  --tag-size-m <black-square-metres> --tag-to-table-m <height-metres>
ros2 run cap_vision calibrate_tag_hand_eye -- \
  --dataset /path/new_tag_run/dataset.yaml \
  --config ros/src/cap_vision/config/cube_6d_urdf.yaml \
  --output /path/new_tag_run/cube_6d_measured.yaml
```

Solver đòi ít nhất 8 ảnh fit, 3 ảnh kiểm chứng với sai số tối đa 5 mm.
Không dùng kích thước danh nghĩa để tự đánh dấu cấu hình đạt chuẩn.
Sau mỗi bước, T8 quan sát lại tối đa 8 giây (mỗi request ROS có thời hạn riêng),
kiểm tra nguồn nằm cao hơn đích 30 mm (sai số Z 10 mm, XY 12 mm).
Chỉ kết quả `scene_verified` mới cập nhật “trên cùng” và cho chạy bước tiếp.
Thiếu bằng chứng, trả `executed_unverified` và dừng chuỗi. Đây là kiểm chứng bằng
pose perception, chưa phải đo độ bền/ổn định cơ học của chồng.
Giới hạn preflight hiện tại Z ≤ 130 mm, đủ cho hover 127 mm của cube thứ ba.
IK/FK vẫn phải khép sai số ≤5 mm; preflight chưa kiểm tra va chạm vật lý, nên
người vận hành cần nhìn chồng hiện tại trước khi Space bước kế tiếp.

Màu trong câu nói chỉ dùng để chọn ID mong muốn. HSV không được phép thay thế
AprilTag/classifier 3D để gắp theo ID. Kết quả `executed_unverified` nghĩa là
worker đã xác nhận khớp và trạng thái kẹp, nhưng chưa có cảm biến chứng minh vật
thật sự nằm trong kẹp hoặc chồng đã đứng đúng.

Khi thiếu vật: trả `needs_observation` và đề xuất quét J1 hai phía. **Chưa có
controller thực thi quét**. Đề xuất các góc servo chưa được kiểm chứng đường đi.
Bộ nhớ nguồn giữ pose trong `base_link` tối đa 15 giây và chỉ phục vụ preview;
trước chuyển động thật phải đo lại. Đích phải có trong scene mới.

Hiệu chuẩn mặc định `cube_6d_urdf.yaml` vẫn là giả định URDF, chưa đạt hiệu chuẩn
camera/hand-eye thực nghiệm. Có thể đọc ID nhưng preview gắp sẽ bị chặn. Không
đổi cờ calibrated thành true để bỏ qua bước đo kiểm.

Nhật ký kết quả tool: `/tmp/t8_ros_task_events.jsonl`.
Log readback/lỗi serial worker: `/tmp/t8_motion_worker.log`.
[Báo cáo đánh giá và việc còn thiếu](ROS3D_AUDIT.md).

## Tương thích ảnh và task cũ

Ảnh tĩnh dùng backend legacy, vẫn mặc định dry run:

```bash
python projects/t8_pipeline/t8_assistant.py \
  --vision-backend legacy2d --image '/path/to/image.png' \
  --text 'trong ảnh có gì' --once --no-speak-info
```

Chuyển động legacy cần bật rõ `--vision-backend legacy2d --enable-motion`.
`--with-manager` chỉ dùng với backend legacy vì manager có thể chạy motor.
Legacy không tự xoay tìm vật rồi dùng lại công thức pixel cố định để gắp. Cube
ID không được suy từ contour hoặc màu để thực hiện chuyển động.

## Kiểm tra

```bash
PYTHONPATH=vendor/yahboom PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  projects/t8_pipeline/test_ros_tasks.py \
  projects/t8_pipeline/test_gemini_first.py \
  projects/t8_pipeline/test_t8_pipeline.py -q
```

Test transport ROS tùy chọn dùng publisher giả trên localhost domain 77, không
mở serial:

```bash
source /opt/ros/humble/setup.bash
source ros/install/setup.bash
T8_RUN_ROS_INTEGRATION=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  projects/t8_pipeline/test_ros_scene_integration.py -q
```
