# Nhật ký phiên

Mới nhất ở trên. Mỗi phiên 3–6 dòng: làm gì, kết quả, để lại gì. Chi tiết đi vào các file khác.

## 2026-10-08 · Tách `backend/` và `frontend/`
- Dời mọi thứ (kể cả `.venv`, `.env`, `data/`, `workspaces/`) vào `backend/`; `frontend/` trống. `docs/memory/` và
  `.claude/skills/` ở lại gốc. `CLAUDE.md` tách đôi: gốc (bố cục, plugin, bộ nhớ) và `backend/CLAUDE.md` (kỹ thuật).
- Thứ gãy vì đường dẫn tuyệt đối và đã sửa: 48 file trong `.venv/bin` (sed), `ros/` và `workspaces/dofbot_ws`
  (xóa `build install log`, build lại: 4 + 13 package xong), `repo_paths.py` (lấy gốc từ git → sai), `cube_sort_3d.py`.
- Chưa làm: build lại `workspaces/dofbot_robot_arm_6dof`; chưa chạy tay thật. Chưa commit.

## 2026-10-08 · Plugin và bộ nhớ dự án
- Cài ở phạm vi user: ponytail 5.0.0, agent-skills 0.6.12, ui-ux-pro-max 2.13.0. agent-skills clone qua SSH nên phải
  ép HTTPS một lần bằng biến `GIT_CONFIG_*`; lần cập nhật sau có thể lỗi lại.
- graphify **chưa cài**: lệnh cài bị bộ phân loại quyền chặn. Gói PyPI là `graphifyy`, kéo theo numpy nên phải cài
  vào venv riêng, không `pip` vào Python hệ thống (ROS). Lệnh người dùng tự chạy:
  `/usr/bin/python3 -m venv ~/.local/share/graphify/venv && ~/.local/share/graphify/venv/bin/pip install graphifyy && ln -sfn ~/.local/share/graphify/venv/bin/graphify ~/.local/bin/graphify`
- Dựng `docs/memory/` từ hai ghi chú auto-memory cũ (bản nguyên văn ở `archive/`) và nối vào `CLAUDE.md`.
- Không đụng code robot, không chạy test.

## 2026-10-07 · Hand-eye 3 nhóm được kiểm chứng, T8 xếp 4 tầng
- Kiểm chứng tay thật → cắt khoảng tin J1 còn 40–135°. T8 xếp 4 tầng từ pose lệch.
- Viewer đen: tìm ra QoS best-effort rớt khung, đổi sang RELIABLE.
- `gravity_pose.py`, khảo sát zone T8, nhận camera tay bằng chuyển động (`cube_vision/cameras.py`).
- Sửa lỗi hiểu lệnh tiếng Việt do khớp chuỗi con; tách track "nghi vấn" khỏi cube thật.

## 2026-10-06 · Kiểm toán IK + hiệu chuẩn, kế hoạch 4 pha
- Tìm nguyên nhân gắp hỏng ngoài READY_POSE: IK ROS cũ và bảng pixel tuyến tính. Viết IK giải tích.
- `calibrate_hand_eye.py`: 16 → 24 → 52 mẫu; giả thuyết tỉ lệ J1 bị bác; neo vào `hand_eye.center_proven.json`.
- Pha 0–2 xong (đường handeye không rơi về luồng cũ, thư viện `cube_vision/`, `search_planner.py`); pha 3–4 dở
  (zone locator, pose library).
- Nguyên văn: `archive/2026-10-06_ik_calibration_audit.md`.
