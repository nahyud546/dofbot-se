# Quyết định kỹ thuật

Mới nhất ở trên. Mỗi mục: quyết định · vì sao · cái đã loại. Chi tiết thô: `archive/`.

## 2026-10-07 · Ảnh ROS nhận bằng QoS RELIABLE
- **Vì sao**: best-effort làm rớt khung 900 KB trong lúc truyền (3–5 khung/s, đứt 1–4 s) → viewer đen. Đo bằng node
  probe riêng: dấu thời gian đứt đúng lúc nhận, độ trễ 0, nên mất khi truyền chứ không phải do camera (đọc V4L2 trực
  tiếp ổn định 7,2 khung/s).
- **Ở đâu**: `image_qos()` trong `t8_ros_scene_worker.py`, `latest_image_qos()` trong `object_perception_node`.

## 2026-10-07 · Tầng/XY/yaw của cube dựng từ 4 góc mặt trên + góc khớp, không dùng TF của perception
- **Vì sao**: TF của node perception hay `None`; pose RGB một mặt nhấp nháy ("cube nghiêng 34°") dù cube đứng yên.
- **Cách làm**: perception chỉ phát `/vision/face_quads`; scene worker tính bằng `gravity_pose.py` (giả định cube nằm
  phẳng, vuông 30 mm). Khớp với pose tag/PnP trong 0,1–2 mm.
- **Đã loại**: dựng pose trong node perception.

## 2026-10-07 · Hand-eye chỉ tin trong J1 40–135° (cắt 5° mỗi đầu so với vùng đã lấy mẫu 35–140°)
- **Vì sao**: kiểm chứng tay thật: sai số LEFT lên 6,3 mm ở J1≈140°, nhưng ≤ 2,5 mm tới J1 130°.
- **Ở đâu**: `j1_valid_range` trong `hand_eye.json`; `cube_layer` từ chối ngoài khoảng.

## 2026-10-06 · Hand-eye nhiều nhóm phải neo vào mô hình tâm đã kiểm chứng bằng gắp thật
- **Vì sao**: giải tự do 52 mẫu (tâm + LEFT + RIGHT) làm cube ở tâm lệch 6–8 mm, do tiêu cự / độ sâu camera / chiều
  cao tag bù trừ lẫn nhau. Neo chặt (`anchor_from_json(hand_eye.center_proven.json)`) thì tâm lệch < 1 mm và sai số
  hai bên giảm.
- **Hệ quả**: `hand_eye.center_proven.json` là mốc neo: không xóa, không ghi đè.

## 2026-10-06 · Hiệu chỉnh tỉ lệ J1 để TẮT (giả thuyết đã bị bác)
- **Giả thuyết**: J1 thật = 90 + k·(số đọc − 90), k≈1,12, khớp hai lần đo một phía tới ~1 mm.
- **Bác bỏ**: có mẫu thật cả trái lẫn phải thì k=1,03 và không cải thiện XY.
- **Còn lại trong code**: `j1_correction` trong `hand_eye.json` (`enabled: false`), cờ `--fit-j1`,
  `--enable-j1-correction`. Đừng bật lại khi chưa có dữ liệu mới.

## 2026-10-06 · Ô thả định vị bằng phép đo mét, không bằng góc trong ảnh
- **Vì sao**: camera nằm trước trục base nên góc trong ảnh lệch ~20° so với góc J1 thật.
- **Cách làm**: điểm thả (FK) → mặt bàn → biên mặt nạ ô (`zone_locator.py`); T8 khảo sát bằng `zone_survey.py`.

## 2026-10-06 · Tầng của cube suy từ tia–mặt phẳng ở độ cao tag, không từ PnP 3D
- **Vì sao**: độ sâu từ một tag 20 mm kém: PnP-3D sai 2–7 mm (trượt ngưỡng 5 mm), trong khi XY trên mặt bàn giữ
  1–2 mm. `cube_layer.py` so khoảng cách PnP với các giả thuyết tầng.
- **Lỗi gốc đã sửa**: bản cũ cắt tia với mặt bàn dù điểm theo dõi là tag trên đỉnh cube (+30 mm) → lệch XY 40–55 mm.

## 2026-10-06 · IK giải tích 5 bậc (`t8_pipeline/dofbot_ik.py`) thay dịch vụ IK ROS
- **Vì sao**: `dofbot_kinematics.cpp` là bản KDL viết lại; client luôn xin RPY cố định mà tay 5 bậc không đạt được,
  nên mọi nghiệm đến từ nhánh dự phòng ngẫu nhiên chỉ khớp vị trí. Chỉ 312/728 điểm lưới giải được.
- **Cách làm**: chọn góc nghiêng kẹp theo tầm với (≈0° ở r=0,10 m, 30° ở 0,20, 50° ở 0,25); cổng workspace theo tọa
  độ cực thay hình chữ nhật.
- **Ghi nhớ**: IK không phụ thuộc pose xuất phát. Lỗi "chỉ gắp được từ READY_POSE" là do vision dùng bảng pixel tuyến
  tính của Yahboom.

## Trước 2026-10-06 · Tách hai trình thông dịch Python
- Xem mục Kiến trúc trong `CLAUDE.md` (cv_bridge/rclpy của Humble không hợp NumPy trong `.venv`).
