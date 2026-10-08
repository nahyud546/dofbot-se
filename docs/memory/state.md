# Trạng thái hiện tại

Cập nhật: 2026-10-08 · nhánh `world-frames` (chưa gộp vào `main`)

## Bố cục repo (đổi 2026-10-08)

Toàn bộ mã điều khiển tay máy nằm trong `backend/`; `frontend/` trống, chờ người dùng ra lệnh. Ở gốc chỉ còn
`CLAUDE.md` (ngắn), `README.md`, `.claude/skills/`, `docs/memory/`, `routine.md`. Hướng dẫn backend: `backend/CLAUDE.md`.
**Việc dời chưa commit**: đổi tên đã `git mv` (staged), file mới ở gốc còn untracked. Người dùng tự commit.

## Đang làm

World chung cho nhiều camera (`cube_vision/world_map.py`, `vision_experiments/build_world.py`). Commit gần nhất
`9cb95da`: vật khác cube dựng từ đám mây điểm (`cube_vision/pointcloud.py`) thay cho `carve.py`.

## Chưa commit (2026-10-08, nguồn: `git status`)

- Việc dời sang `backend/` + sửa đường dẫn: `scripts/tools/repo_paths.py` (bỏ bước lấy gốc từ git),
  `vision_experiments/cube_sort_3d.py` (bỏ đường dẫn tuyệt đối), 4 skill, `README.md`, `.gitignore` gốc.
- Đang sửa dở ở phiên khác trong lúc dời (không phải của phiên dời thư mục): `vision_experiments/active_view.py`,
  `build_world.py`, `test_active_view.py`, `ros/src/cap_vision/cap_vision/world_publisher.py`.
- Không rõ mục đích, đừng tự commit: `vendor/yahboom/dofbot_voice/scripts/speak_text.py` (dòng 1 bị dán nhầm một lệnh
  shell → `SyntaxError`, làm `test_t8_pipeline.py` không nạp được), `docs/calibration/check_camera.md`,
  `docs/ros2_moveit/sync_commands.md`, `docs/hand_teleop_webcam_rviz.md` (trùng tên với bản trong `docs/teleop/`),
  `routine.md`.

## Bước kế (suy từ code và ghi chú, người dùng chưa xác nhận)

- `frontend/`: chờ lệnh.
- `workspaces/dofbot_robot_arm_6dof` (kho lồng) chưa build lại sau khi dời: `install/` của nó còn trỏ đường dẫn cũ.
- **Chưa chạy tay thật sau khi dời** (chỉ build + pytest). Lần chạy T8 đầu tiên nên là dry run.
- `routine.md` là lộ trình tự học 8 bài của người dùng; nhánh này đang ở khoảng bài 5–7.
- Danh sách việc treo: [open.md](open.md).

## Môi trường phiên tới

- Terminal cũ còn `VIRTUAL_ENV`/`ROBOT_ARM_ROOT` trỏ đường dẫn trước khi dời: mở terminal mới rồi source lại.
- Plugin user-scope cài 2026-10-08: ponytail, agent-skills, ui-ux-pro-max (xem `CLAUDE.md` gốc).
- graphify **chưa cài** (bị chặn quyền); lệnh cài bằng venv riêng nằm trong `log.md` mục 2026-10-08.
