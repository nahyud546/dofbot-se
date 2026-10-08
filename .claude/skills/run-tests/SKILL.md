---
name: run-tests
description: Chạy test của repo đúng môi trường (Python hệ thống, có hoặc không có ROS) và diễn giải kết quả. Dùng trước khi báo một thay đổi đã xong, trước khi commit, hoặc khi pytest lỗi ngay lúc khởi động/thu thập test.
---

# Chạy test

> Mọi lệnh và đường dẫn dưới đây tính từ `backend/`: `cd backend` trước khi chạy.

Cấu hình nằm trong `pyproject.toml` (`pythonpath`, `testpaths`, tắt plugin ROS) và `conftest.py` (tự bỏ qua test cần ROS).

```bash
/usr/bin/python3 -m pytest                                   # nhanh nhất: không ROS, test ROS tự skip
/usr/bin/python3 -m pytest projects/cube_vision projects/t8_pipeline     # ~15 s, đủ cho đa số thay đổi
/usr/bin/python3 -m pytest path/to/test_x.py::Class::test_name           # một test
```
Đủ bộ, gồm perception (`ros/src/cap_vision/test`) và test viewer:
```bash
source /opt/ros/humble/setup.bash && source ros/install/setup.bash
/usr/bin/python3 -m pytest projects ros/src/cap_vision/test ros/hardware
```
`projects/vision_experiments` mất ~6 phút vì nạp DINO; chỉ chạy khi đụng tới nhận diện hình rác hoặc trước khi commit.

## Kỳ vọng
- Một lỗi có sẵn, không do thay đổi mới: `test_cube_sort_stage1.py::test_pick_waits_for_confirmed_id_and_current_top_face`.
- Skip hợp lệ: test cần ROS khi chưa source; `test_object_pipeline.py` khi thiếu package `cap_grasp` (nằm ở kho khác).
- Mọi lỗi khác là lỗi thật: báo nguyên văn, đừng sửa test cho qua khi chưa hiểu test đang bảo vệ điều gì.

## Lỗi môi trường hay gặp
- `No module named pytest`: đang dùng python của `.venv` (không có pytest) → gọi `/usr/bin/python3 -m pytest`.
- `PluginValidationError: launch_testing`: chạy `pytest` ngoài `backend/` nên không đọc `pyproject.toml` → `cd backend`.
- `No module named 'std_msgs'` / `cap_scene_interfaces`: chưa source ROS hoặc chưa build `ros/`.
- Build `ros/` lỗi `No module named 'catkin_pkg'`: CMake lấy python của `.venv`; tắt venv và thêm
  `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

Test nằm cạnh module (`test_*.py` cùng thư mục). Thay đổi hành vi phần cứng vẫn cần chạy thật: test dùng arm/camera giả.
