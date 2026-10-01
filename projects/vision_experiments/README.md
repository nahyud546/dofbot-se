# vision_experiments
Script vision độc lập (không phải ROS package):
- `apriltag_follow.py`, `face_follow_gpu.py`, `kcf_follow.py` (dời từ root).
- Model: `ai/models/detection/yolov8n-face.pt` — code resolve qua `ROBOT_ARM_ROOT` + fallback path cũ.
- Chạy: `source scripts/setup/setup_env.sh && .venv/bin/python projects/vision_experiments/face_follow_gpu.py`

## identify_cube.py — nhận diện cube và tìm tâm mặt trên
Script này chỉ kiểm tra nhận diện; nhấn `Space` không điều khiển tay. Để gắp và
phân loại, chạy `cube_sort_stage1.py` theo lệnh ở phần dưới.
Mỗi cube có 6 mặt (1 màu + 1 AprilTag + 4 ảnh rác). Các mặt nhìn thấy được gom theo hình học và quy về cùng một ID:
- ID 1 blue (tag 1 + 4 loại tái chế), ID 2 green (tag 2 + 4 loại nhà bếp),
  ID 3 red (tag 3 + 4 loại nguy hại), ID 4 yellow (tag 4 + 4 loại còn lại).
- Mã tìm các tứ giác của những mặt vật lý, gom các mặt có cạnh chung thành một cube,
  chỉnh phối cảnh từng mặt rồi chọn tín hiệu AprilTag, màu HSV hoặc DINOv2 ViT-S/14.
  Nếu các mặt cho ID khác nhau, ưu tiên AprilTag, rồi HSV, rồi DINO; báo xung
  đột trong dữ liệu. Tín hiệu từ mặt bên có thể xác định ID, nhưng tâm gắp chỉ
  lấy từ tứ giác mặt trên có cấu trúc mặt bên phía dưới.
  Không chạy ORB/RANSAC. DINO được chạy theo lô cho các mặt cần phân loại.
  Camera hiển thị liên tục; nhận dạng chạy nền và bỏ qua frame cũ khi bận.
  Mỗi cube cần ít nhất hai phiếu cùng ID trong cửa sổ 1 giây. Sau đó script giữ
  ID/class và điểm nhận diện mạnh nhất; bbox và tâm gắp theo quan sát hợp lệ mới
  nhất của chính cube đó, nên có thể theo cube khi nó di chuyển.
  Một ID khác chỉ thay khi nguồn ưu tiên hơn, hoặc điểm cao hơn trong cùng nguồn,
  và đã được xác nhận trong cửa sổ mới. Tọa độ gắp chỉ dùng khi mặt trên hiện tại
  còn mới, ID hiện tại khớp và liên kết track không mơ hồ.
- Ảnh hiển thị vẽ các mặt được gom, viền mặt trên và dấu thập tại tâm. Tham số
  `--jsonl-output /path/to/observations.jsonl` ghi `cube_id`, phiếu từng mặt,
  `top_quad_px`, `top_center_px`, `geometry_valid` và `robot_target_valid`.
  Tâm chỉ ở tọa độ pixel; `robot_target_valid` luôn là `false` vì chưa hiệu chuẩn
  camera với bàn và hệ tọa độ robot để đi TCP.
- Database mặc định `ai/datasets/trash-images/processed/vector_database_dinov2_vits14.pt`
  có 256 vector, 16 lớp, 384 chiều. Ngưỡng cosine mặc định 0.40 và không bắt
  buộc khoảng cách giữa Top 1/Top 2. Cả hai similarity được hiển thị dạng phần
  trăm và ghi trong JSONL. Có thể đặt `--sim-margin` nếu muốn lọc lớp gần nhau.
  HSV chấp nhận mặt có độ phủ ít nhất 0.40; AprilTag và HSV còn có thể cho ID
  khi chưa tìm được tứ giác mặt trên, nhưng trường hợp đó không được gắp.
  Bbox màu cam đánh dấu mặt/nhóm mặt đã thấy nhưng chưa đủ hình học gắp; bbox
  xanh lá và dấu chữ thập tím chỉ mặt trên đã xác nhận. Ứng viên contour nhỏ
  được tìm thêm ở ảnh phóng 2 lần, HSV dự phòng nhận vùng từ 200 px nhưng lọc
  màu ít bão hòa để mặt xám không bị gọi nhầm là xanh. AprilTag được thử trên
  ảnh xám gốc và ảnh tăng tương phản rồi gộp các kết quả trùng. Lý do
  `too-small`, `frame-clipped`, `no-outer-face`, `no-side-support` hoặc
  `low-image-quality` giải thích vì sao một ứng viên chưa thể gắp.
  Ảnh `processed` là ảnh tham chiếu/tăng cường, không phải bộ kiểm thử camera độc lập.
- Chạy trên máy robot (có tay + camera):
  `.venv/bin/python projects/vision_experiments/identify_cube.py --camera /dev/video2`.
- Trong cửa sổ camera, nhấn `s` để lưu frame gốc vào `/tmp/identify_cube_raw_<timestamp>.png`;
  nhấn `d` để lưu frame cùng báo cáo diện tích contour, diện tích bbox, tỉ lệ
  diện tích/bbox, cạnh nhỏ nhất, độ hỗ trợ mặt bên và lý do không chọn mặt trên;
  nhấn `q` hoặc Ctrl+C để dừng. Đặt cùng một cube ở giữa và 4 góc, nhấn `d`
  mỗi vị trí. So sánh 5 file JSON trước khi thay ngưỡng contour; nếu vùng giá
  trị cube và nhiễu chồng nhau thì giữ trạng thái có ID nhưng không cho gắp.
- Kiểm tra ảnh tĩnh không di chuyển tay: thêm `--image /path/to/frame.png --output /tmp/result.png`.
  Thêm `--contour-report /tmp/sample_center` để ghi lại frame và metrics ảnh tĩnh.
  Tùy chọn khác: `--dry-run`, `--synthetic {blue,green,red,yellow}`, `--no-trash`,
  `--device {auto,cpu,cuda}`, `--sim-thresh`, `--sim-margin`, `--jsonl-output`.
- Kiểm thử: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest projects/vision_experiments/test_identify_cube.py -q`.

## cube_sort_stage1.py — nhận dạng, kiểm tra IK và phân loại vào zone

Chạy với camera ở pose quan sát XYT:

```bash
.venv/bin/python projects/vision_experiments/cube_sort_stage1.py --camera /dev/video2
```

Script đưa tay về pose quan sát, nạp DINO lúc khởi động và nhận dạng liên tục.
Khi đã xác nhận ID và mặt trên trong 1 giây, màn hình báo có cube sẵn sàng.
Nhấn `Space` một lần trong cửa sổ camera hoặc terminal để chốt các cube còn mới,
xếp theo confidence rồi thử IK theo thứ tự. Cube không có nghiệm IK được bỏ qua;
worker gắp cube đầu tiên hợp lệ, thả vào zone tương ứng ID và về pose quan sát.
Còi chỉ kêu một lần, sau khi IK thành công và ngay trước khi tay bắt đầu di chuyển.
Nếu tất cả cube đều lỗi IK, tay đứng yên, không bíp và script tiếp tục quét.
Khi tay đang gắp, Space không tạo thêm lượt. Sau khi thả xong, cần quan sát mới
trước lượt tiếp theo; cùng một cube ở cùng vị trí vẫn có thể được gắp lại.
`--auto` dùng cùng luồng nhưng không cần Space.
Nhấn `q` để thoát; `s` để lưu ảnh camera gốc; `d` để lưu frame và metrics contour.
`--dry-run` mô phỏng một lượt
phân loại mà không kiểm tra IK thực, điều khiển tay hoặc còi.

Nếu tâm kẹp lệch khỏi cube, dùng `--pick-x-offset-mm` (dương kéo kẹp về phía
chân robot, âm đẩy ra xa) và `--pick-z-offset-mm` (dương nâng kẹp, âm hạ kẹp).
Giới hạn lần lượt là ±20 mm và ±8 mm; X mặc định +15 mm, Z mặc định 0. Bắt đầu với bước
nhỏ và đối chiếu dòng `[TARGET]` chứa pixel và tọa độ KDL thực đã ra lệnh;
đổi độ bù không sửa phép nhận diện mặt trên hoặc nghiệm IK.

DINO được nạp trước khi mở camera và tiếp tục đánh giá các mặt trong nền sau
khi AprilTag/HSV đã cho ID. Top 1/Top 2 không tự thay một ID có nguồn ưu tiên
hơn. Ảnh mặt được nắn phối cảnh và thử 0/90/180/270 độ; khi score < 0.55 hoặc
Top1–Top2 cách nhau < 0.08 thì thử thêm 45/135/225/315 độ. Bảng trên ảnh
ghi rõ hai số là DINO similarity. Điều này làm mỗi lượt nhận dạng chậm hơn
so với chế độ bỏ qua DINO cũ.
IK server dùng URDF của gói `dofbot_urdf`; nếu service ROS cũ chưa được khởi
động lại sau khi build `dofbot_info`, script báo lỗi URDF/service rõ ràng.

ID/class đã xác nhận giữ điểm tốt nhất đến khi khối mất dấu hoặc tay đã gắp xong;
bbox và tâm gắp lấy từ mặt trên mới nhất có cùng ID. Phép đổi pixel sang tọa độ robot dùng thông số
hiện có của bài color sorting. Dùng `--no-trash` để tắt DINO. Bước kiểm tra lại
zone bằng DroidCam thuộc giai đoạn tiếp theo.

Góc servo J1 của các zone là: ID1/xanh dương 30°, ID2/xanh lá 150°,
ID3/đỏ 50°, ID4/vàng 135°; pose quan sát J1=90°. Đây là góc servo tuyệt đối.
