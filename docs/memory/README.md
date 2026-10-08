# Bộ nhớ dự án

Những điều **không đọc ra được từ code hay git**: vì sao chọn cách này, số đã đo trên tay thật, việc đang dở,
bẫy đã dính. `CLAUDE.md` (gốc + `backend/CLAUDE.md`) nói repo chạy thế nào; thư mục này nói repo đã đi qua những gì.

Đường dẫn code trong các file này (`projects/…`, `config/…`, `ros/…`, `docs/architecture/…`) tính từ `backend/`.

| File | Chứa gì | Cách ghi |
|---|---|---|
| [state.md](state.md) | Đang làm gì, dở ở đâu, bước kế | **Ghi đè** cuối mỗi phiên. Tối đa ~40 dòng |
| [decisions.md](decisions.md) | Quyết định kỹ thuật + lý do + cái đã loại | Thêm mục mới lên đầu. Đổi ý thì ghi mục mới, đánh dấu mục cũ "thay bởi" |
| [measurements.md](measurements.md) | Số đo trên phần cứng thật (sai số, tầm với, thời gian) | Thêm dòng, luôn kèm ngày và cách đo |
| [pitfalls.md](pitfalls.md) | Bẫy đã dính và cách nhận ra | Thêm mục. Bẫy quan trọng nhất thì đưa lên `CLAUDE.md` |
| [open.md](open.md) | Việc chưa xong, giả thuyết chưa kiểm, thứ chưa chạy trên tay thật | Thêm khi phát hiện, **xóa** khi xong (git giữ lịch sử) |
| [log.md](log.md) | Nhật ký phiên: mỗi phiên 3–6 dòng | Thêm lên đầu |
| [archive/](archive/) | Ghi chú thô cũ, giữ nguyên văn | Không sửa |

## Quy tắc

1. **Đầu phiên**: đọc `state.md` (đã tự nạp qua `CLAUDE.md`). Chạm vùng nào thì đọc file liên quan trước khi phân
   tích lại: hiệu chuẩn/IK → `decisions.md` + `measurements.md`; chạy phần cứng → `pitfalls.md`.
2. **Cuối phiên** (hoặc khi người dùng nói "lưu memory"): ghi đè `state.md`, thêm một mục `log.md`, và chuyển những gì
   mới học được vào đúng file. Không có gì mới thì không ghi.
3. **Chỉ ghi điều đã kiểm.** Mỗi mục có ngày `YYYY-MM-DD` và nguồn: `đo tay thật`, `mô phỏng`, `đọc code`, hay
   `suy đoán`. Thứ chỉ chạy mô phỏng thì ghi rõ là chưa chạy tay thật.
4. **Không chép lại code hay git log.** Ghi tên file/hàm để tìm, không dán nội dung.
5. **Ghi chú có thể cũ.** Trước khi dựa vào một mục có tên hàm, cờ, hay con số ngưỡng: grep xem còn đúng không. Sai thì
   sửa ngay tại chỗ.
6. **Không ghi bí mật** (API key, token) và không ghi sở thích cá nhân của người dùng: thứ đó thuộc auto-memory của
   Claude Code ở `~/.claude/projects/…/memory/`, không vào git.
