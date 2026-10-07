# Đánh giá T8 và tác vụ tìm–gắp–xếp

## Kết luận

T8 đã dùng LLM để hiểu ngôn ngữ, lập JSON và truy xuất tri thức. Giới hạn chính
là công cụ chưa gắn đủ với trạng thái thực, thiếu quản lý tác vụ dài và xác minh
kết quả. Thêm tên hàm không tự tạo ra khả năng quan sát hay điều khiển chính xác.

Đợt này hoàn thành phạm vi **quan sát ROS 3D và dry run**. Câu “tự tìm cube đỏ rồi
đặt lên cube id 1” có thể đi đến preview có điều kiện; chưa hoàn thành tác vụ vật
lý tự xoay, gắp, tìm đích và đặt.

## Đối chiếu yêu cầu

| Phần | Hiện trạng |
|---|---|
| Hiểu câu lệnh và nguồn/đích | Gemini lập một `stack_cubes`; kiểm tra toàn bộ JSON trước thực thi |
| Tự biết chức năng đang khả dụng | Có runtime capabilities, tool `capabilities`, tri thức đã sửa cho đúng chế độ |
| Quan sát cube | Đọc scene ROS có timestamp; tool `observe_scene`, `search_object` |
| Cube đỏ và cube ID 1 | ID/model phải nhất quán; mô hình hiện tại ID3 đỏ, ID1 xanh dương |
| Nhớ nguồn khi đổi góc nhìn | Pose trong base_link, TTL 15 giây; không loại trừ bằng pixel của ảnh cũ |
| Tự xoay để bao phủ vùng tìm kiếm | Chỉ đề xuất quét hai phía; chưa kiểm chứng đường đi hoặc chạy motor |
| Dừng trước tính gắp | Kiểm tra joint đo được theo thời điểm ảnh và hai pose nhất quán |
| Tọa độ cho camera gắn trên tay | Tái sử dụng pose base từ TF động của perception; không cộng delta joint vào XYZ |
| Preview gắp và đặt lên cube | 5 TCP stage, chiều cao đích + 30 mm, kiểm tra IK/FK và giới hạn J1 khi giữ vật |
| Gắp/đặt thật và xác minh | Chưa triển khai trong backend ROS 3D; không báo holding hoặc grasp thành công giả |
| Tri thức ngoài cube/OCR | RAG/web hiện có; ROS scene không thay thế ảnh RGB/OCR, cần công cụ ảnh riêng |

## Các lỗi đã xử lý

1. Contour generic từng có thể được gán thành `cube_1..4`; fallback contour cũng
   có thể thay thế detector ngữ nghĩa/màu. Nay không còn dùng hình vuông để chứng
   minh ID, xương cá hay rác; màu phải có bằng chứng HSV hoặc detector tương ứng.
2. Quét J1 rồi dùng lại `pixel_target()` của ready pose làm sai tọa độ. Legacy
   hiện chỉ tìm trong góc nhìn hiện tại; đổi góc nhìn cần ROS TF động.
3. Chuỗi tìm→gắp→tìm→`stack_cubes` có thể gắp nguồn hai lần. `stack_cubes` nay sở
   hữu toàn bộ preview và bị từ chối nếu ghép thêm action khác.
4. Đặt dùng `STACK_Z` cố định không phản ánh chiều cao đích thực. Backend mới lấy
   TCP đích cộng chiều cao cube nguồn; hover cũng tính tương đối từ đó.
5. Dry run từng có thể đổi holding/released hoặc đi vào manager. Backend mới
   tách khỏi motion worker, không mở serial; dry run legacy cũng không dispatch
   manager và không đổi trạng thái giữ vật sau preview place/release.
6. RAG chứa tên hàm lịch sử chưa đăng ký. Các mục liên quan đã được chú thích lại;
   Gemini nhận khả năng thực của phiên. Trạng thái giữ vật từ file legacy được
   đánh dấu chưa xác minh và chỉ đọc báo cáo mới tối đa 15 giây.

## Ready pose và zone cố định

`workspaces/dofbot_ws/src/dofbot_color_sorting/color_sorting.py` bản gốc nhận màu
rồi gắp tại pose cố định `[90,53,33,32,90,30]`. Các zone dùng **vector góc servo
đích tuyệt đối**, không chỉ một alpha. Ví dụ đỏ `[117,19,66,50,90,...]`; sau mỗi
chu kỳ về `[90,130,0,0,90,0]`. `identify_grap.py` cũng đặt theo vector góc tuyệt
đối, dù góc gắp đầu vào là kết quả IK.

Vì vậy ready không cần để “reset phép cộng góc” cho zone tuyệt đối. Nó phục vụ
vùng nhìn, vị trí gắp giả định và đường đi được tác giả thiết kế. Vector đích đúng
không đảm bảo đường đi từ mọi điểm xuất phát đều an toàn.

`projects/t8_pipeline/t8_motion_worker.py:pixel_target` là công thức pixel thực
nghiệm cho góc nhìn cố định; đổi pose camera thì công thức đó không còn đủ.
Trong mô hình động đúng, tính lại:

`T_base_cube(t) = T_base_mount(q_measured(t)) × T_mount_optical × T_optical_cube(t)`.

K phụ thuộc camera/resolution/focus; extrinsic cố định giữa camera và link gắn;
TF base→camera thay đổi theo joint. Không cộng trực tiếp delta góc 5 joint vào
XYZ vì động học phi tuyến. Có thể bắt đầu từ pose khác **trong vùng đã kiểm
chứng**, với joint đo thật, TF cùng timestamp, hiệu chuẩn đạt và đường đi hợp lệ.
Không có bảo đảm cho mọi pose tùy ý.

## Kiểm chứng đợt này

- 80 test T8 qua; test transport ROS được tách thành bộ tùy chọn.
- 3 test transport ROS qua trên localhost domain 77: scene giả đã hiệu chuẩn có
  pose xác nhận/joint đứng yên; scene chưa hiệu chuẩn giữ ID nhưng không có TCP;
  timestamp scene mới không được làm mới pose cũ.
- 11 test hồi quy bridge cube 3D qua.
- IK/FK service thật cho đủ 5 stage: sai số khép kín 0,75–2,81 mm.
  Hai tọa độ pick/place giả `[-0.185,-0.03,0.047]` và
  `[-0.185,0.04,0.077]` m: sai số khép kín **1,04 mm** và **2,81 mm**.
- Test Gemini dùng mock, chưa đánh giá chất lượng model trực tuyến hoặc gọi API.
- Không chạy motor, không đo TCP thực, không kiểm tra va chạm. IK/FK cùng URDF
  chỉ kiểm tra nhất quán tính toán, không đo sai số robot ngoài đời.

## Các bước cần làm để hoàn thành tác vụ vật lý

1. **Đo hiệu chuẩn** K/distortion, hand-eye và TCP; đo sai số ở các pose ngoài tập
   fit. Cấu hình mặc định `cube_6d_urdf.yaml` hiện đánh dấu giả định URDF chưa đạt.
   Offset X 15 mm và contact Z +2 mm đang kế thừa bridge, chưa phải TCP đã đo.
2. **Controller quan sát**: danh sách pose đo kiểm, đường đi có giới hạn/collision,
   timeout/cancel, chỉ chụp sau đứng yên, giới hạn khác nhau khi có vật trong kẹp.
   Quét đối xứng không có nghĩa là đã bao phủ toàn bàn; cần kiểm tra FOV thực.
3. **Một nơi sở hữu serial và telemetry**: tránh MotionBridge khóa serial suốt
   phiên trong khi real_joint_mirror cần đọc joint; controller cần xuất joint đo
   được liên tục và trạng thái lỗi. Không dùng góc lệnh thay góc đo.
4. **State machine tác vụ**: search_source→confirm_source→locate_target→pick→
   verify_grasp→reacquire_target→place→verify_stack. Chỉ commit trạng thái sau
   phản hồi thực; retry có giới hạn, không lặp gắp khi mất phản hồi.
5. **Xác minh và giảm sai số**: tái đo nguồn trước gắp, tái đo đích trước đặt,
   kiểm tra vật trong kẹp và vị trí sau đặt, ghi chênh lệch XYZ theo pose để tìm
   backlash/TCP/hand-eye sai. Dùng tọa độ đo lại để tránh tích lũy delta.
6. **Agent và tri thức**: bổ sung tool ảnh RGB/OCR, tra manifest mô hình, xem lý do
   calibration/preflight thất bại; vòng observe→tool→verify có số bước giới hạn.
   Giữ điều khiển servo/path trong controller xác định, LLM chọn mục tiêu/task.
   Tập eval tiếng Việt nên gồm ID mơ hồ, nhiều cube cùng màu, vật bị che, đổi pose,
   mất TF, mất joint và yêu cầu bất khả thi.
