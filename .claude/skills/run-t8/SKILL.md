---
name: run-t8
description: Chạy trợ lý T8 trên tay máy thật (sort, xếp tầng, lệnh "tất cả cube") và đọc log của nó. Dùng khi cần khởi động T8, chọn cờ dòng lệnh, hoặc chẩn đoán vì sao một lệnh sort/stack không chạy, bị bỏ qua, hay báo lệch.
---

# Chạy T8

> Mọi lệnh và đường dẫn dưới đây tính từ `backend/`: `cd backend` trước khi chạy.

## Trước khi chạy
1. Phần cứng rảnh: xem skill `hardware-check`. `camera_test` hay T8 cũ còn mở sẽ giữ camera/serial.
2. Terminal đã source ROS, nếu không T8 báo "Chưa source ROS":
   ```bash
   source /opt/ros/humble/setup.bash && source ros/install/setup.bash
   ```
3. Cube cần gắp nằm giữa bàn (J1 35–140°), không đặt cube lên ô thả trước khi sort.

## Lệnh
```bash
python projects/t8_pipeline/t8_assistant.py --vision-backend ros3d --enable-motion --approval viewer \
  --text "sort cube vàng vào đúng zone"
```
Câu lệnh phải nằm trong dấu nháy sau `--text`. Bỏ `--text` để vào chế độ nhập `T8>`. Bỏ `--enable-motion` là dry run.

| Cờ | Mặc định | Ý nghĩa |
|---|---|---|
| `--zone-check always\|off` | always | Camera tay xoay đo vị trí ô trước khi sort |
| `--verify-placement auto\|off` | auto | Camera ngoài so ảnh trước/sau khi thả |
| `--max-retries N` | 2 | Số lần gắp lại khi lệch/văng (lần sau thả thấp và chậm hơn) |
| `--batch-approval-timeout S` | 180 | Lệnh "tất cả": hết hạn chờ Space thì bỏ cube đó |
| `--camera`, `--external-camera` | auto | Chỉ đặt tay khi tự nhận sai |

Câu lệnh mẫu: `"sort cube đỏ vào đúng zone"`, `"xếp cube xanh lá lên cube xanh dương"`,
`"sorting color tất cả các cube"` (→ `sort_cube {"label":"all"}`), `"xếp chồng tất cả các cube"`.

Mỗi cube phải được duyệt bằng **Space** trong cửa sổ viewer (Esc = hủy; trong lô, Esc dừng cả lô).

## Đọc log
- `[plan] …` — Gemini hiểu lệnh thành gì. Sai ở đây thì sửa câu lệnh hoặc prompt trong `t8_pipeline.py`.
- `[zone] Zone N: …` — `giữ cấu hình` / `thả vào tâm ô (x, y)` / `[suy ra từ đối xứng…]` / `Không thả…` (bị chặn:
  ô ngoài tầm J1 10–170° hoặc điểm thả cấu hình nằm trên ô khác → dời ô, đừng nới giới hạn).
- `[approval] …` — `READY - PRESS SPACE` mới bấm được. `NO FRESH CAMERA IMAGE` kéo dài = perception không phát ảnh.
- `[verify] Zone N: …` — `gắp chính xác` / `lệch…` / `ô không có cube` / `chưa xác nhận`. Ảnh để audit ở
  `/tmp/t8_verify/*_overlay.png` (viền ô vàng, vùng đổi đỏ).
- `[batch i/N] …` và câu tổng kết cuối: xong bao nhiêu, bước nào chưa xong và vì sao.
- Log perception: `/tmp/t8_cube_perception.log`; log viewer: `/tmp/t8_approval_<id>.log`.

## Khi không chạy
- "Chưa source ROS" → bước 2. "Cổng tay máy đang bận" → skill `hardware-check`.
- Cube không được gắp trong lô: đọc dòng `[batch]` tương ứng; `zone_unavailable` là ô bị chặn, không phải lỗi nhận diện.
- Viewer đen: kiểm QoS ảnh (phải RELIABLE) và perception còn sống không.
