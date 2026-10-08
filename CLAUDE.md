# CLAUDE.md

Repo tay máy Yahboom DOFBOT, chia hai phần. Người dùng làm việc bằng tiếng Việt; comment và thông báo trong code
cũng bằng tiếng Việt.

| Thư mục | Chứa gì | Hướng dẫn cho Claude |
|---|---|---|
| `backend/` | Toàn bộ mã điều khiển tay máy: perception ROS, trợ lý T8, hiệu chuẩn, test, `.venv`, `.env` | `backend/CLAUDE.md` |
| `frontend/` | Giao diện. **Đang trống**, chờ người dùng ra lệnh; đừng tự dựng khung | (chưa có) |
| `docs/memory/` | Bộ nhớ dự án, dùng chung cho cả hai phần | mục dưới |
| `.claude/skills/` | Skill của repo: `run-tests`, `run-t8`, `hardware-check`, `calibrate-cameras` (đều chạy trong `backend/`) | |

## Quy tắc chung

- **Trước khi sửa hay chạy bất cứ gì trong `backend/`: đọc `backend/CLAUDE.md`** (lệnh, kiến trúc, bất biến an toàn,
  bẫy). File đó tự nạp khi mở file trong `backend/`, nhưng nếu chưa thấy nó trong ngữ cảnh thì tự đọc.
- Lệnh backend chạy từ `backend/` (`cd backend`): `pyproject.toml`, `.venv`, `.env` nằm ở đó, không ở gốc.
- `routine.md` là ghi chú học của người dùng: không sửa, không commit khi chưa được bảo.

## Plugin Claude Code (cài ở `~/.claude`, phạm vi user)

- **ponytail** — tự bật mỗi phiên: viết ít code nhất mà đủ. Không được cắt phần "Bất biến an toàn" của backend.
- **agent-skills** — `/spec` → `/plan` → `/build` → `/test` → `/review` → `/ship`. Bước test của backend luôn là
  `/usr/bin/python3 -m pytest` trong `backend/` (skill `run-tests`); thay đổi chạm phần cứng thì "verify" nghĩa là
  chạy tay thật có người duyệt, không tự chạy.
- **ui-ux-pro-max** — chỉ dùng cho `frontend/`.

## Bộ nhớ dự án (`docs/memory/`)

Quyết định + lý do, số đo trên tay thật, bẫy, việc treo: những thứ không đọc ra được từ code hay git. Quy tắc đọc/ghi
và trạng thái hiện tại được nạp tự động:

@docs/memory/README.md
@docs/memory/state.md

- Trước khi phân tích lại hiệu chuẩn, IK, hay một lỗi phần cứng: đọc `decisions.md`, `measurements.md`, `pitfalls.md`.
- Cuối phiên, hoặc khi người dùng nói "lưu memory": ghi đè `state.md`, thêm một mục vào `log.md`, chuyển điều mới học
  vào đúng file. Kiến thức về repo ghi ở đây (vào git), không ghi vào auto-memory ở `~/.claude`.
