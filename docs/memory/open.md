# Việc treo

Xong thì xóa mục (git giữ lịch sử). "Ghi ngày" là ngày ghi nhận; mục chưa đánh dấu `kiểm 2026-10-08` là chép từ ghi chú
cũ, **chưa đối chiếu lại với code hiện tại**: grep trước khi tin.

## Chưa chạy trên tay thật / ngưỡng chưa chỉnh

- **Ngưỡng so pixel của `verify_stack` (45 px) chưa chỉnh** bằng một tháp thật. Ghi 2026-10-07.
  (`expected_px_error` còn trong `t8_ros_scene_worker.py`, kiểm 2026-10-08.)
- **Nhận tầng ở pose bên trái chưa kiểm chứng**: PnP-3D nhóm LEFT sai ~22 mm. Ghi 2026-10-06.
- **Bù kẹp khi thả (−12 mm) chưa áp dụng ở chế độ handeye.** Ghi 2026-10-06, có thể đã cũ.
- **TCP offset thật chưa đo.** Ghi 2026-10-06.

## Chưa làm

- **Vật khác cube chưa có chiều cao**, mới có vết đáy ước lượng (`cube_vision/carve.py`). Ghi 2026-10-08.
- **Ô 1 và 2 không thả được** (cần J1≈2°/176°, ngoài `ZONE_J1_LIMITS` 10–170°). Hoặc dời thảm/ô, hoặc đo trên tay
  thật rồi mới nới giới hạn. Ghi 2026-10-07.
- **Bảng `BIN_*` lệch 6–9 cm** so với ô thật; đang sống nhờ tâm ô đo được. Ghi 2026-10-07.
- **Chất lượng nhận mặt hình rác**: cần người dùng thu dữ liệu (`collect_trash_faces.py`, `eval_trash_faces.py`);
  `TrashDetector` tự nạp `*_real.pt` khi có. Ghi 2026-10-07.
- **`cube_sort_3d.py` vẫn nhận ảnh best-effort** (`qos_profile_sensor_data`, dòng ~322; kiểm 2026-10-08). Chưa đổi
  sang RELIABLE vì không thuộc luồng T8.
- **`recover_cube` là stub.** Ghi 2026-10-06, có thể đã cũ.

## Lỗi có sẵn

- `test_cube_sort_stage1.py::test_pick_waits_for_confirmed_id_and_current_top_face` đỏ từ trước (ghi trong
  `CLAUDE.md`). Chưa ai tìm nguyên nhân.

## Câu hỏi cho người dùng

- Các file sửa dở không rõ mục đích (xem `state.md`): giữ, commit, hay bỏ?
- Cài graphify không? (bị chặn quyền 2026-10-08; xem `log.md`.)
