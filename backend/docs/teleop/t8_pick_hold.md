# T8: gắp, xoay, đặt tại vị trí hiện tại

T8 dùng camera `/dev/video2`, YOLO `best.onnx`, phép đổi pixel sang tọa độ
và IK của T6. YOLO được ưu tiên cho cube xương cá (conf từ 0,65); HSV tìm cube
đỏ/xanh/vàng; contour tứ giác là phương án cuối khi không nhận ra nhãn/màu.
Contour chỉ được tự dùng nếu đúng một ứng viên ổn định qua hai ảnh 640×480.
Ảnh truyền bằng `--image` chỉ xem trước.

```bash
cd ~/Desktop/robot-arm
source .venv/bin/activate
cd LargeModel_ws
python3 t8_assistant.py --text 'cầm khối cube có hình xương cá lên' --once --dry-run-motion
python3 t8_assistant.py --text 'cầm khối cube có hình xương cá lên' --once --speak
python3 t8_assistant.py --text 'xoay sang trái thêm 20 độ' --once
python3 t8_assistant.py --text 'đặt xuống' --once
python3 t8_assistant.py --text 'gắp khối cube có hình xương cá lên, xoay sang phải 30 độ và đặt xuống' --once
python3 t8_assistant.py --text 'gắp khối đỏ lên rồi đặt lên khối xanh' --once --dry-run-motion
python3 t8_assistant.py --text 'gắp khối đỏ lên rồi đặt lên khối xanh' --once
python3 t8_assistant.py --text 'gắp cube hình xương cá, đặt lên cube hình giấy vệ sinh' --once
```

Trước khi chụp ảnh để gắp qua CLI, T8 đưa tay về pose quan sát T6
`[90, 125, 0, 0, 90, 25]` và đọc lại góc để xác nhận. Lệnh gắp mở kẹp, đến vị trí vật, đóng kẹp rồi nâng tay; nó không đưa vật vào
thùng. Với nhãn `xuong_ca` rõ ràng, camera và YOLO xác nhận trực tiếp,
không cần gọi Gemini. Lệnh “trái” tăng góc servo J1 trên giá lắp hiện tại.
`đặt xuống` dùng lại pose gắp đã xác nhận, giữ góc đế sau khi xoay,
hạ tới độ cao bàn, mở kẹp rồi nâng tay ra và về pose quan sát. `thả ra` mở kẹp
ở pose hiện tại rồi về pose quan sát. T8 chỉ báo hoàn tất khi đã đọc lại pose.
Các lệnh `--once` dùng chung trạng thái `/tmp/t8_hold_state.json`.
Trong chế độ tương tác, một câu gồm gắp xương cá, xoay đế theo góc nêu rõ,
rồi đặt xuống được thực thi liên tiếp theo thứ tự. Nếu một bước lỗi, các bước
còn lại không chạy. Nếu người dùng đã mở kẹp và đặt vật bằng tay, bước chuẩn bị
đọc góc kẹp hai lần; khi xác nhận kẹp mở, T8 đồng bộ trạng thái rồi về pose chờ.
Kẹp còn đóng hoặc góc đọc không ổn định thì chuỗi mới bị chặn.

Lệnh “gắp cube A đặt lên cube B” tìm hai khối khác nhau trong cùng hai ảnh,
gồm nhãn YOLO `Fish_bone` và `Toilet_paper`,
kiểm tra IK của cả điểm gắp và điểm đặt trước khi gắp. Chỉ xếp cube cao 3 cm;
độ cao tâm cube khi đặt lên mặt đích là 0,075 m theo hiệu chuẩn hiện tại.
Nếu có nhiều ứng viên, T8 yêu cầu nói rõ màu/họa tiết; contour không thể xác
nhận họa tiết xương cá hoặc phân loại rác. `--dry-run-motion` in box, bốn góc
contour và điểm dự kiến mà không gửi lệnh robot.

Trong `T8>`, nhập `exit`/`quit`/`thoát` hoặc nhấn Ctrl+C để kết thúc. Nếu còn
giữ vật, T8 thử đặt tại XY hiện tại rồi mới thoát. Nếu đặt thất bại, kẹp vẫn
đóng và trạng thái giữ vật được lưu để xử lý ở lần chạy sau.

Nếu một bước thất bại giữa chuyển động, trạng thái chuyển sang `moving` và
các lệnh tiếp bị chặn. Xem trạng thái file và kiểm tra tay máy/vật trước khi
khôi phục bằng tay; không xóa file trong khi tay còn giữ vật. T8 không có cảm
biến lực kẹp, nên thông báo thành công xác nhận lệnh servo và góc đọc lại,
không xác nhận chắc chắn vật đang nằm trong kẹp.
