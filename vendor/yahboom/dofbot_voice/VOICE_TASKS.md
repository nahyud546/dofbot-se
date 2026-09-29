# Chạy bài toán DOFBOT bằng giọng nói

Từ thư mục `/home/jloy/Desktop/robot-arm`, dừng hai script voice cũ rồi chạy **một terminal**:

```bash
cd /home/jloy/Desktop/robot-arm
python3 -u dofbot_voice/scripts/voice_task_manager.py --with-voice
```

Lệnh này mở `simple_voice_ctrl.py` và `stt_vi.py --lang vi` giống hai terminal voice đã test, đồng thời quản lý các tiến trình của bài toán được chọn. Có thể chọn ngôn ngữ khác bằng `--lang zh` hoặc `--lang auto`. Mỗi lần chỉ chạy một bài toán. Khi đổi bài, đợi tay hoàn tất chuyển động, nói **"dừng bài toán"**, đợi log `[task] dừng ...`, rồi nói tên bài mới. Ctrl+C dừng các tiến trình do manager mở.

Chạy riêng `stt_vi.py` chỉ nhận dạng giọng nói và ghi mã lệnh vào `/tmp/dofbot_task_command`; nó không tự mở camera hay khởi động bài toán. Nếu đã chạy riêng STT và điều khiển giọng nói ở hai terminal, mở thêm `voice_task_manager.py` ở terminal thứ ba bằng lệnh phía dưới. Nên mở manager trước khi nói lệnh bài toán.

| Câu nói | Mã | Tiến trình được mở |
|---|---:|---|
| "phân loại màu" | 61 | IK, camera `/dev/video2`, chọn cube HSV, gắp theo tọa độ/góc như bài stack, thả vào 4 góc màu |
| "xếp chồng màu" | 62 | IK, color stacking với camera `/dev/video2` |
| "theo dõi khuôn mặt" | 63 | face follow GPU với camera `/dev/video0` |
| "phân loại rác" | 64 | camera `/dev/video2`, IK, grasp, YOLO `best.onnx` |
| "dừng bài toán" | 65 | dừng bài hiện tại |

STT cũng chấp nhận "xếp trồng màu" vì mô hình tiếng Việt đôi khi phiên âm "chồng" thành "trồng". Log phải hiện `code 62` và `đã gửi tới /tmp/dofbot_task_command`; `code 51` là lệnh action cũ và không khởi động bài toán xếp chồng màu.

Trong bài xếp chồng, chọn màu và nhấn phím trong cửa sổ `stacking` theo hướng dẫn hiện trên ảnh. Trong bài phân loại rác, YOLO chỉ quét và hiển thị mặc định; nhấn **SPACE** trong cửa sổ preview để yêu cầu gắp một vật. Cửa sổ cần có focus để nhận phím.

Trong bài phân loại màu, nhấn **b/g/r/y** ở cửa sổ `color sorting` để chọn màu xanh dương/xanh lá/đỏ/vàng, rồi **SPACE** để gắp cube đang thấy và đặt vào góc màu tương ứng. Camera dùng `/dev/video2` và pose nhìn lấy từ `XYT_config.txt`, giống bài stack. Cả hai bài dùng chung phép đổi pixel ảnh thô sang tọa độ KDL và góc xoay cube; node gắp màu gọi cùng hàm IK của bài stack. Yêu cầu gắp truyền màu, XY và yaw trong một message `color_pick`, tránh lấy góc xoay cũ. Tay tới pose góc màu cũ, hạ đến pose nhả, rút lên theo các pose đã kiểm tra FK rồi về ready. Nhấn `c`, kéo ROI, rồi `s` để lưu HSV nếu cần hiệu chỉnh.

Điều chỉnh riêng cho bài phân loại màu: điểm gắp IK có offset **+8 mm** so với Z nền 39 mm (nominal 47 mm; các mức thử lại 58/68 mm). Sau khi nâng stack lên 44 mm, điểm gắp color sorting hiện cao hơn stack **3 mm**. Khi tới pose góc màu cũ, tay hạ thẳng khoảng **10 mm** rồi mới mở kẹp. FK offline xác nhận độ hạ 9,89–9,98 mm, lệch XY tối đa 0,47 mm; sau đó tay rút lên khoảng 70 mm so với điểm nhả mới. Bài stack không dùng offset thả này.

Bài xếp chồng gửi lệnh về pose ready ngay khi bắt đầu, trước lúc đợi service IK, và gửi lại pose ready khi thoát. Pose lấy từ `dofbot_ws/src/dofbot_color_stacking/scripts/XYT_config.txt` (`x=87`, `y=128` hiện tại), thành `[87, 128, 0, 0, 90, 30]`. Z gắp stack nâng thêm **5 mm** so với trước: mức danh nghĩa 39 → 44 mm, các mức IK thử lại 50/60 → 55/65 mm; color sorting giữ offset riêng +8 mm. Khi dừng bằng giọng nói, manager đợi chuỗi chuyển động xếp chồng hiện tại hoàn tất rồi mới dừng service IK.

Nếu muốn giữ hai terminal voice đang chạy, mở thêm một terminal manager thay cho `--with-voice`:

```bash
cd /home/jloy/Desktop/robot-arm
python3 -u dofbot_voice/scripts/voice_task_manager.py
```

Thử launcher mà không dùng micro hoặc robot:

```bash
python3 dofbot_voice/scripts/voice_task_manager.py --dry-run
python3 dofbot_voice/scripts/stt_vi.py --cheatsheet
```

Chạy trực tiếp một bài không cần nói: `python3 -u dofbot_voice/scripts/voice_task_manager.py --task trash` (thay `trash` bằng `color`, `stack`, `face`).
