# Trạng thái hiện tại

Cập nhật: 2026-10-08 · nhánh `world-frames` (chưa gộp vào `main`)

## Bố cục repo (đổi 2026-10-08)

Toàn bộ mã điều khiển tay máy nằm trong `backend/`; `frontend/` trống, chờ người dùng ra lệnh. Ở gốc chỉ còn
`CLAUDE.md` (ngắn), `README.md`, `.claude/skills/`, `docs/memory/`, `routine.md`. Hướng dẫn backend: `backend/CLAUDE.md`.
Đã commit: `bd07b3a` (directory partitioning).

## Đang làm

World chung cho nhiều camera (`cube_vision/world_map.py`, `vision_experiments/build_world.py`). Commit gần nhất
`7919f4b`: tay nhìn quanh từng vật, gom cụm đám mây điểm dày hơn trong 3D (`cube_vision/pointcloud.py`).

## Lỗi đang có (2026-10-08, nguồn: pytest đủ bộ sau khi dời: 617 passed, 1 failed có sẵn, 1 lỗi nạp)

- `vendor/yahboom/dofbot_voice/scripts/speak_text.py` dòng 1 bị dán nhầm một lệnh shell trước `#!/usr/bin/env python3`
  → `SyntaxError`, `projects/t8_pipeline/test_t8_pipeline.py` không nạp được, và T8 import file này cũng sẽ lỗi.
  Đã vào git. Chưa sửa vì không rõ ý người dùng.

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
