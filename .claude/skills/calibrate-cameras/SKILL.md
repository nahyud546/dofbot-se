---
name: calibrate-cameras
description: Hiệu chuẩn camera tay (hand-eye) và camera ngoài với hệ base của robot, kiểm chứng kết quả, và xử lý khi camera ngoài bị xê dịch. Dùng khi vị trí gắp/thả lệch có hệ thống, khi đổi hoặc dời camera, hoặc khi cần tọa độ mét từ camera ngoài.
---

# Hiệu chuẩn camera

Tắt T8/perception trước (skill `hardware-check`): các script này tự mở camera và cổng serial.
Thứ tự bắt buộc: camera tay trước, vì camera ngoài được hiệu chuẩn **dựa trên** camera tay.

## 1. Camera tay (hand-eye) → `config/robot/hand_eye.json`
```bash
# Đặt MỘT cube tag ngửa, cách chân robot ~15 cm phía trước. Tay tự đi ~24 pose.
python projects/vision_experiments/calibrate_hand_eye.py --cube-id 3     # thu mẫu + giải (--group/--append: nhiều vị trí)
python projects/vision_experiments/validate_hand_eye.py --cube-id 3      # cùng cube đứng yên phải cho cùng XY ở mọi pose
```
- Đạt khi mẫu kiểm định ≤ 5 mm (3D) / 4 mm (XY trên bàn). Chỉ tin trong vùng J1 đã lấy mẫu (hiện 35–140°).
- `hand_eye.center_proven.json` là mốc đã kiểm chứng bằng gắp thật: không xóa, không ghi đè. Các file
  `*.backup_*`, `*.rejected_*` là lịch sử thử nghiệm.
- Sau khi đổi hand-eye phải hiệu chuẩn lại camera ngoài.

## 2. Camera ngoài ↔ base → `config/robot/external_camera.json`
Camera tay định vị 4 góc AprilTag của từng cube trong base; camera ngoài thấy cùng tag ở pixel (1280×720).
Mỗi lần `--collect` là một cách bày cube; **bày lại cube giữa các lần**.
```bash
P=projects/vision_experiments/calibrate_external.py
python $P --reset
python $P --collect     # bộ 1: 4 cube rải rộng giữa bàn, tag ngửa lên
python $P --collect     # bộ 2: xếp tháp 2 cube (tag trên cùng ngửa) + cube lẻ ở chỗ khác
python $P --collect     # bộ 3: tháp 3 cube + cube lẻ
python $P --collect     # bộ 4–5: đổi vị trí, có ít nhất một tháp nữa
python $P --solve       # in báo cáo, ghi file, vẽ /tmp/external_calibration_overlay.png
python $P --validate    # bày vị trí MỚI rồi so camera ngoài với camera tay (mm)
```
Yêu cầu: tag ngửa và nằm phẳng; cube trong tầm camera tay (giữa bàn) **và** camera ngoài; cần ≥ 3 độ cao khác nhau,
≥ 24 điểm (6 tag-mẫu), ≥ 5 tag-mẫu. Mỗi `--collect` in tag nào được nhận, tag nào bị bỏ và vì sao.

Tiêu chí đạt (`accepted=true`): RMS chiếu lại ≤ 3 px, sai số kiểm định trung vị ≤ 6 mm. Chưa đạt thì file vẫn được
ghi với `accepted=false` và **không** được dùng; đọc phần "CHƯA ĐẠT" để biết thiếu gì. Đừng nới ngưỡng cho qua:
thu thêm bộ mẫu trải rộng hơn.

## 3. Camera ngoài bị dời
Camera ngoài là webcam, dễ bị chạm. Vị trí 4 ô màu lúc hiệu chuẩn được lưu làm mốc.
```bash
python $P --status        # tâm ô lệch bao nhiêu px; > 12 px = đã bị dời
python $P --relocalize    # giữ ống kính, giải lại pose từ MỘT bộ bày cube (≥ 3 tag)
python -m cube_vision.external_camera     # in XY (base) các cube camera ngoài đang thấy
```

## Giới hạn cần nhớ
- Độ chính xác camera ngoài không thể tốt hơn hand-eye của camera tay (vài mm).
- Từ camera ngoài, **tầng** của cube không tự suy ra chắc được (hai tầng kề chỉ khác ~1 mm ở kích thước tag):
  biết tầng thì truyền vào `locate_tag(..., layer=n)`; sai tầng làm XY lệch hàng cm.
- Ảnh khác cỡ hiệu chuẩn (ví dụ 640×480) không dùng được với `external_camera.json`.
