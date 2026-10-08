# Bẫy đã dính

Bổ sung cho mục "Bẫy đã gặp" trong `CLAUDE.md` (không lặp lại những mục ở đó). Ngày là ngày dính bẫy.

## Phần cứng và tiến trình

- **Lệnh serial đầu tiên sau khi mở `Arm_Device` bị nuốt** (2026-10-06). Chờ ~0,5 s sau khi mở rồi mới ghi.
- **`ros2 launch` sống sót khi chỉ giết node camera** (2026-10-06). Perception sót chạy ~800 % CPU, làm
  `real_joint_mirror` đói → triệu chứng giả "khớp chưa đứng yên". Kiểm `ps` + `fuser`, giết theo pid.
- **Script chẩn đoán giữ `/tmp/t8_motion.lock`** làm joint mirror bỏ chu kỳ → trạng thái "pending" giả (2026-10-06).
- **Bước "verify" trước khi gắp dùng lại khung V4L2 cũ** → sai số giả 38–43 mm, gắp hụt (2026-10-06). `--verify` nay
  phải bật tay.
- **Cube quá gần đế không bao giờ khóa** trong search-center: cần tâm cube cy trong 240 ± 60 px (2026-10-06).

## Hiệu chuẩn

- **`calibrate_hand_eye.py --append` GHI `hand_eye.json` kể cả khi kết quả bị từ chối, và XÓA
  `cube_6d_calibrated.yaml`** (2026-10-06) → T8 lặng lẽ mất hiệu chuẩn. Sau mỗi lần chạy: kiểm trường `accepted` và
  xem file yaml còn không.
- **Cube đặt nghiêng làm hỏng nhóm mẫu**: nhóm LEFT nghiêng ~6° bị cổng nghiêng loại (nay cho phép ≤ 8° với nhóm phụ)
  (2026-10-06).
- **Hand-eye ngoài khoảng J1 đã lấy mẫu là ngoại suy**: điểm thả chiếu lệch 3–5 cm so với ô nhìn thấy (2026-10-06).

## Vision

- **`pupil_apriltags` segfault khi tạo nhiều `Detector`** (2026-10-06). Dùng chung một instance.
- **Ô màu trên thảm bị gán ID cube** (track chỉ có danh tính, pose bị loại) → đếm sai, báo "nhiều track cùng ID"
  (2026-10-07). `describe_scene` tách "thấy" và "nghi vấn".
- **Mặt trên là màu/hình rác cho pose RGB nghiêng**, xyz trong base vô nghĩa → `verify_stack` trượt (2026-10-07).
  Đang dùng so pixel thay thế (`expected_px_error`).
- **Mặt trên bị mép ảnh cắt** thì `grasp_candidates_for_obj` cũ trả `[]` (2026-10-06). Đường handeye có
  `handeye_confirmation` / `handeye_grasp`.
- **Yaw trong ảnh chỉ đúng ở READY** (2026-10-06). Dùng `cube_layer.ready_view_yaw_deg`.
- **Ô xám nhạy cân bằng trắng** (mặt nạ sót); **đỏ phải V ≥ 150** để không lẫn màu gỗ (2026-10-07).

## Ngôn ngữ và T8

- **Khớp chuỗi con trong lệnh tiếng Việt**: "…đã qua **sử dụng lên** trên…" khớp "dung len" → thành lệnh nâng tay,
  vì luật điều khiển cục bộ chạy TRƯỚC Gemini (2026-10-07). Luật mới phải khớp theo ranh giới từ và bỏ qua khi câu có
  động từ gắp/xếp.
- **Tên hình rác dùng chung giữa luồng ros3d và luồng 2D cũ** (YOLO class). Chỉ đổi sang ID cube khi
  `set_ros3d_face_labels(True)` và câu có cube/khối/block. **Test làm rò cờ toàn cục này**: reset trong `setUp`.

## Khi sửa code

- **`__pycache__` cũ được dùng lại** khi sửa file bằng `sed` cùng kích thước trong vòng 1 s (2026-10-06). Nghi thì
  xóa `__pycache__`.
