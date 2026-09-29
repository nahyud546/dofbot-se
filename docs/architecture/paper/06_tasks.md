# 06. Danh Mục Các Tác Vụ Thao Tác (Manipulation Tasks Specification)

Tài liệu này chuẩn hóa toàn bộ các tác vụ thực thi (Tasks T0 đến T8 cùng các demo cờ vua/MoveIt) của cánh tay robot Yahboom DOFBOT-SE. Mỗi tác vụ được cấu trúc đồng nhất theo 7 tiêu chí: **Input**, **Goal**, **Perception**, **Decision logic**, **Motion logic**, **Output**, và **Failure cases** nhằm phục vụ việc chuyển đổi sang lưu đồ giải thuật (flowchart), mã giả (pseudocode) và phân tích định lượng sau này.

---

## Task 0: Khởi Động & Chẩn Đoán Phần Cứng (Hardware Prereq & Bring-Up - T0)

* **Input:** Tín hiệu nguồn DC 7.4V/12V, cổng kết nối serial `/dev/ttyUSB0`, cổng camera `/dev/video0` (hoặc `/dev/video2`).
* **Goal:** Xác nhận tính toàn vẹn của kết nối USB-UART, quét kiểm tra phản hồi của toàn bộ 6 động cơ servo, khởi tạo camera UVC và đưa cánh tay về tư thế Home an toàn.
* **Perception:** Không dùng thuật toán thị giác. Chỉ chụp 1 khung hình thử nghiệm từ camera để xác thực độ sáng và chỉ số thiết bị V4L2.
* **Decision logic:** Đọc lần lượt ID servo 1 đến 6 qua `Arm_serial_servo_read()`. Bỏ qua byte rác ở lần đọc đầu tiên (tránh race-condition bộ đệm sau khi mở cổng). Nếu đủ 6 servo trả về giá trị góc hợp lệ ($0 \dots 180^\circ$ hoặc $0 \dots 270^\circ$), cấp cờ `SYSTEM_READY`.
* **Motion logic:** Phát chuỗi lệnh di chuyển chậm tới tư thế nghỉ an toàn `P_HOME` ($q = [90^\circ, 90^\circ, 90^\circ, 90^\circ, 90^\circ, 30^\circ]$) với thời gian nội suy `time_ms = 2000 ms`.
* **Output:** Trạng thái hệ thống `ALL PASS`, cánh tay đứng thẳng vuông góc, còi kêu 1 tiếng beep ngắn, LED RGB chuyển màu xanh lá.
* **Failure cases:** 
  * Cáp nguồn servo chưa bật hoặc lỏng giắc bus serial $\rightarrow$ servo không phản hồi (None), dừng hệ thống.
  * Mở nhầm webcam tích hợp của laptop $\rightarrow$ sai khung hình quan sát.

---

## Task 1: Phân Loại Màu Tại Tâm Cố Định (Fixed-Center Color Sorting - T1)

* **Input:** Một khối lập phương màu (Đỏ, Xanh lá, Xanh dương, hoặc Vàng), kích thước cạnh $30\text{ mm}$, đặt sẵn tại tâm vùng gắp trên mặt bàn (`P_BLACK_CENTER`).
* **Goal:** Nhận diện đúng nhãn màu của vật thể và điều khiển cánh tay gắp khối thả vào đúng 1 trong 4 khay màu tương ứng.
* **Perception:** Cắt vùng quan sát cục bộ ROI ($y \in [200, 480], x \in [160, 400]$), chuyển đổi BGR sang HSV, áp dụng mặt nạ `inRange`, lọc hình thái học (`MORPH_CLOSE`), tìm contour có diện tích lớn nhất thỏa mãn $S > 1000\text{ px}$. Áp dụng bộ lọc chống rung (debounce filter) tích lũy trong 10 khung hình liên tiếp.
* **Decision logic:** Nếu nhãn màu nhận diện ổn định trong 10 frame, kích hoạt trạng thái `sorting_run(color_name)`. Ánh xạ nhãn màu sang tư thế khay đích định sẵn: Đỏ ($J_1 = 117^\circ$), Xanh lá ($J_1 = 136^\circ$), Xanh dương ($J_1 = 44^\circ$), Vàng ($J_1 = 65^\circ$).
* **Motion logic:** Chuyển động vòng hở (Open-loop) theo chuỗi tư thế khớp nạp cứng:
  $$\text{P\_LOOK} \rightarrow \text{Mở kẹp } (30^\circ) \rightarrow \text{P\_BLACK\_CENTER} \rightarrow \text{Đóng kẹp } (135^\circ) \rightarrow \text{Nhấc khớp } J_2, J_3 \rightarrow \text{Xoay } J_1 \text{ tới khay} \rightarrow \text{Hạ \& Mở kẹp} \rightarrow \text{Về P\_LOOK}$$
* **Output:** Khối màu được chuyển vào khay tương ứng, cánh tay quay lại vị trí quan sát.
* **Failure cases:**
  * Ánh sáng môi trường thay đổi làm sai lệch ngưỡng HSV $\rightarrow$ nhận diện nhầm màu hoặc không đạt diện tích $1000\text{ px}$.
  * Vật đặt lệch ra ngoài tâm gắp cố định $\rightarrow$ kẹp đóng hụt hoặc va quẹt làm đổ khối màu.

---

## Task 2: Gắp & Xếp Chồng Khối Màu Nhiều Tầng (Color Grab & Stacking - T2)

* **Input:** Nhiều khối màu đặt tại các vị trí tiếp liệu trên bàn; yêu cầu xếp tháp cao từ 2 đến 4 tầng.
* **Goal:** Gắp tuần tự từng khối màu và xếp chồng chính xác lên nhau thành một cột tháp thẳng đứng.
* **Perception:** Phân đoạn màu tương tự T1 nhưng mở rộng vùng tìm kiếm trên toàn khung hình, xác định tâm khối tiếp theo cần gắp.
* **Decision logic:** Duy trì biến đếm trạng thái tầng hiện tại $L \in \{1, 2, 3, 4\}$. Với mỗi tầng mới, tự động tăng tọa độ độ cao nhả kẹp:
  $$Z_{\text{place}}(L) = Z_{\text{base\_mat}} + (L - 1) \cdot \Delta Z_{\text{cube}} + Z_{\text{safety\_offset}}$$
  với $\Delta Z_{\text{cube}} \approx 30\text{ mm}$ ($0.03\text{ m}$).
* **Motion logic:**
  1. Gắp khối tại vị trí tiếp liệu (hạ sâu $Z_{\text{pick}}$).
  2. Nhấc cao theo phương thẳng đứng vượt độ cao tối đa của tháp.
  3. Di chuyển ngang tới vị trí tâm tháp.
  4. Hạ thẳng đứng xuống độ cao tầng $Z_{\text{place}}(L)$, mở mỏ kẹp nhẹ nhàng.
  5. Rút mỏ kẹp thẳng đứng lên trên trước khi di chuyển ngang để tránh gạt đổ tháp.
* **Output:** Tháp khối màu hoàn chỉnh đứng vững trên mặt bàn.
* **Failure cases:**
  * Khối bên dưới bị xê dịch nhẹ trong quá trình nhả $\rightarrow$ tầng trên bị nghiêng và đổ tháp.
  * Mỏ kẹp mở quá rộng khi hạ ở các tầng cao $\rightarrow$ va quẹt vào các khối xung quanh.

---

## Task 3: Bám Theo Mục Tiêu Chuyển Động 2D (Pan/Tilt Target Follow - T3)

* **Input:** Đối tượng di động trong tầm nhìn của camera (khối màu, khuôn mặt người, vùng vân ảnh KCF hoặc mã AprilTag).
* **Goal:** Điều khiển 2 khớp quay của cánh tay (khớp đế Pan $J_1$ và khớp cổ tay Tilt $J_4/J_5$) để luôn giữ tâm đối tượng nằm chính giữa khung hình ($c_x = 320, c_y = 240$).
* **Perception:** Tùy chọn 1 trong 4 bộ nhận diện:
  * Color: Tâm trọng tâm contour HSV.
  * Face: Mô hình Haar Cascade / DNN trích xuất khung bao khuôn mặt.
  * KCF: Bộ theo dõi Correlation Filter bám theo vùng ảnh đặc trưng.
  * AprilTag: Giải mã góc đỉnh và tâm tag.
* **Decision logic:** Tính toán sai số vị trí điểm ảnh so với tâm quang học:
  $$e_x = 320 - c_x, \quad e_y = 240 - c_y$$
  Đưa sai số qua bộ điều khiển tỉ lệ vi tích phân PID hai kênh độc lập:
  $$\Delta \theta_1 = K_p^x e_x + K_d^x \frac{d e_x}{dt}, \quad \Delta \theta_4 = K_p^y e_y + K_d^y \frac{d e_y}{dt}$$
* **Motion logic:** Cập nhật liên tục góc quay của Servo 1 và Servo 4/5 theo chu kỳ $\Delta t \approx 30 - 50\text{ ms}$.
* **Output:** Khung hình camera bám mượt mà theo chuyển động của vật thể.
* **Failure cases:**
  * Vật thể di chuyển quá nhanh vượt quá tốc độ đáp ứng của servo ($250^\circ/\text{s}$) $\rightarrow$ mất dấu đối tượng.
  * Bộ bám KCF bị che khuất (occlusion) $\rightarrow$ bị trôi dấu vết sang nền môi trường (drift).

---

## Task 4: Nhận Dạng Cử Chỉ Tay & Thao Tác (MediaPipe Gesture Manipulation - T4)

* **Input:** Hình ảnh bàn tay người trước camera thu nhận qua MediaPipe Hands.
* **Goal:** Nhận diện cử chỉ bàn tay (Nắm tay, Xòe tay, Cử chỉ chữ V, Đếm ngón) để kích hoạt các chế độ làm việc hoặc điều khiển góc kẹp của robot mô phỏng tay người.
* **Perception:** Trích xuất 21 điểm mốc xương bàn tay 3D ($x, y, z$). Tính toán khoảng cách Euclid giữa đầu ngón tay và khớp đốt bàn, tính góc gập của từng ngón tay để phân loại cử chỉ.
* **Decision logic:**
  * Cử chỉ "Nắm tay" (Fist): Ra lệnh đóng kẹp kẹp vật.
  * Cử chỉ "Bàn tay mở" (Five): Ra lệnh mở kẹp hoàn toàn.
  * Cử chỉ "Chữ V / Victory": Kích hoạt chuỗi hành vi gắp và xếp khối.
  * Cử chỉ "Trỏ ngón": Cánh tay xoay hướng theo góc chỉ của ngón tay.
* **Motion logic:** Chuyển đổi mã cử chỉ thành mục tiêu góc khớp tương ứng hoặc gọi chuỗi quỹ đạo nội suy định sẵn.
* **Output:** Robot thực thi hành động tương thích với cử chỉ tay người theo thời gian thực.
* **Failure cases:**
  * Bàn tay bị che khuất hoặc nghiêng góc quá lớn $\rightarrow$ MediaPipe trích xuất sai landmark $\rightarrow$ nhận diện nhầm cử chỉ.

---

## Task 5: Phân Loại AprilTag & Tự Động Căn Hướng Kẹp (AprilTag Dynamic Grasping - T5)

* **Input:** Khối lập phương có dán nhãn AprilTag (họ `tag36h11`) đặt ngẫu nhiên trên bàn với góc xoay bất kỳ.
* **Goal:** Nhận diện mã ID, xác định chính xác tâm $(X, Y)$ và góc xoay Yaw của khối, tự động xoay khớp cổ tay $J_5$ song song với cạnh khối trước khi gắp.
* **Perception:** Bộ giải mã `apriltag` trích xuất 4 góc đỉnh của tag trong ảnh: $P_0, P_1, P_2, P_3$.
* **Decision logic:**
  1. Xác định góc xoay Yaw của khối trên mặt phẳng bàn:
     $$\Delta \text{Yaw} = \arctan2(y_1 - y_0, \, x_1 - x_0)$$
  2. Tính toán góc khớp xoay kẹp Joint 5:
     $$\theta_5 = \theta_1 - \Delta \text{Yaw}$$
  3. Ánh xạ tâm tag từ pixel sang tọa độ Đề-các $(X, Y)$ thông qua Homography hoặc Ray-Plane.
* **Motion logic:**
  Gửi yêu cầu IK tới dịch vụ KDL với tọa độ $(X, Y, Z_{\text{grasp}})$ và góc hướng kẹp chúc đứng. Khớp $J_5$ xoay đúng góc $\theta_5$ trước khi hạ kẹp, kẹp đóng chính xác vào hai mặt phẳng của khối mà không bị xô lệch.
* **Output:** Khối AprilTag được gắp chắc chắn và chuyển về ô đích quy định theo ID của tag.
* **Failure cases:**
  * Tag bị mờ hoặc phản quang làm thuật toán không nhận diện được ID.
  * Góc xoay $\theta_5$ vượt quá giới hạn phần cứng ($0 \dots 270^\circ$).

---

## Task 6: Phân Loại Rác Thải Với YOLOv11 (YOLOv11 Garbage Sorting - T6)

* **Input:** Các mô hình rác thải thu nhỏ đặt trong vùng quan sát của camera.
* **Goal:** Sử dụng mô hình học sâu YOLOv11 phân loại đối tượng thành 1 trong 4 nhóm rác (Tái chế, Nguy hại, Nhà bếp, Rác khác) và thả vào đúng thùng rác tương ứng.
* **Perception:** Mạng nơ-ron tích chập YOLOv11 chạy trên mô hình trọng số `best.pt` hoặc `best.onnx`. Trả về nhãn phân loại (class name), độ tin cậy (confidence score $> 0.6$) và hộp bao đối tượng (Bounding Box).
* **Decision logic:** Ánh xạ nhãn phát hiện sang 4 nhóm rác thải đô thị:
  * Tái chế (Recyclable - Thùng Xanh dương): Vỏ chai, lon kim loại, hộp giấy.
  * Nguy hại (Hazardous - Thùng Đỏ): Pin, bóng đèn, nhiệt kế.
  * Nhà bếp (Kitchen - Thùng Xanh lá): Rau củ, xương cá, vỏ trái cây.
  * Khác (Other - Thùng Xám): Đồ gốm vỡ, tàn thuốc lá.
* **Motion logic:** Xác định tâm bounding box, tính toán tọa độ gắp, gọi chuỗi chuyển động đưa vật thể lên cao và hạ vào miệng thùng rác quy định.
* **Output:** Rác thải được phân loại chính xác vào 4 thùng, phát âm thanh thông báo kết quả.
* **Failure cases:**
  * Rác đặt úp hoặc bị biến dạng làm điểm tin cậy YOLO $< 0.6$.
  * Kích thước rác vượt quá khẩu độ kẹp $40\text{ mm}$ của gripper.

---

## Task 7: Điều Khiển Bằng Giọng Nói Ngoại Tuyến (Offline Voice Action Control - T7)

* **Input:** Lệnh thoại từ người dùng qua microphone bo mạch nhận dạng giọng nói chuyên dụng hoặc qua lớp shim giả lập `Speech_Lib.py`.
* **Goal:** Tiếp nhận mã lệnh thoại, so khớp quy tắc và kích hoạt các chuyển động định sẵn hoặc chuyển tiếp trạng thái cho các bài toán gắp thả.
* **Perception:** Bộ nhận diện phần cứng xử lý âm thanh tại chỗ, trích xuất mã số nguyên (Command Code) dựa theo bảng từ điển `speech_ID.csv`.
* **Decision logic:** Bộ giải mã phân nhánh:
  * Mã 11-14: Đổi màu đèn LED RGB tương ứng (Đỏ, Xanh lá, Xanh dương, Vàng).
  * Mã 38: Bật còi báo động Buzzer.
  * Mã 39: Đưa robot về tư thế đứng vươn cao (Up pose).
  * Mã 61: Kích hoạt tác vụ phân loại màu tự động.
  * Mã 62: Kích hoạt tác vụ xếp chồng tháp khối.
* **Motion logic:** Kích hoạt các hàm chấp hành của `Arm_Lib` hoặc gọi tiến trình con thực thi tác vụ tương ứng.
* **Output:** Robot thực thi hành vi đúng theo câu lệnh nói, phát âm thanh phản hồi qua loa.
* **Failure cases:**
  * Nhiễu tạp âm môi trường làm nhận diện sai mã lệnh.
  * Xung đột tài nguyên cổng serial khi tác vụ thoại và tác vụ gắp cùng cố gắng mở kết nối.

---

## Task 8: Trợ Lý Đa Phương Thức LLM/VLM Thông Minh (Multimodal LLM Orchestration - T8)

* **Input:** Câu thoại hoặc câu lệnh văn bản tự nhiên bằng tiếng Việt/tiếng Anh (ví dụ: *"Hãy nhìn qua camera và gắp khối màu đỏ đặt sang bên trái"*), kèm khung hình ảnh từ camera robot.
* **Goal:** Phân tích ngôn ngữ tự nhiên, hiểu ngữ cảnh thị giác phức tạp (Vision-Language), lập kế hoạch hành động an toàn và điều phối các tác vụ cấp dưới thực thi trên robot thật.
* **Perception:**
  * Nhận dạng giọng nói tiếng Việt ngoại tuyến bằng **Sherpa-ONNX ASR**.
  * Chụp ảnh hiện trường `/dev/video0` hoặc `/image_raw`.
  * Mô hình đa phương thức **Gemini 3.6 Flash** tiếp nhận đồng thời cả văn bản và hình ảnh (Multi-modal Prompting).
* **Decision logic:**
  1. Nếu câu hỏi yêu cầu kiến thức thời gian thực (ví dụ thời tiết, tin tức): Tự động truy vấn công cụ tìm kiếm **Tavily Search API**.
  2. Nếu câu hỏi liên quan đến điều khiển robot: Ràng buộc mô hình LLM chỉ được phép trả về đối tượng JSON thuộc danh mục whitelist nghiêm ngặt (`none`, `color`, `stack`, `face`, `trash`, `stop`). Nghiêm cấm mô hình tự ý sinh lệnh hệ thống Linux hoặc góc servo trực tiếp.
  3. Gửi mã lệnh điều phối ($61 \dots 65$) sang bộ giám sát độc quyền `voice_task_manager.py`.
* **Motion logic:** `voice_task_manager.py` kiểm tra file khóa `/tmp/dofbot_task_manager.lock`, giải phóng thiết bị ngoại vi, khởi tạo tiến trình con ROS tương ứng và giám sát tiến trình cho tới khi hoàn tất.
* **Output:** Robot thực thi tác vụ vật lý an toàn, trợ lý phát câu trả lời giọng nói tự nhiên qua Edge-TTS.
* **Failure cases:**
  * Mất kết nối Internet khi gọi API Gemini / Tavily.
  * Người dùng ra lệnh mơ hồ ngoài danh mục hành vi cho phép $\rightarrow$ LLM từ chối thực thi và yêu cầu làm rõ.

---

## Task Phụ: Trình Diễn Cờ Vua Tự Động Với MoveIt 2 (Stockfish Chess Demo)

* **Input:** Trạng thái nước cờ trên bàn cờ vật lý (lưới $8 \times 8$).
* **Goal:** Robot tự động đấu cờ với người hoặc tự chơi (self-play) bằng thuật toán Stockfish, gắp và di chuyển các quân cờ chính xác giữa các ô cờ mà không va chạm vào các quân cờ khác trên bàn.
* **Perception:** Nhận diện trạng thái bàn cờ qua camera hoặc nạp nước đi chuẩn FEN / UCI.
* **Decision logic:** Động cơ cờ vua **Stockfish Engine** tính toán nước đi tối ưu (ví dụ: `e2e4`, `g1f3`). Node `pick_place_node.py` chuyển đổi nước đi thành tọa độ ô nguồn $(X_{\text{src}}, Y_{\text{src}})$ và ô đích $(X_{\text{dst}}, Y_{\text{dst}})$.
* **Motion logic:**
  1. Sử dụng MoveIt 2 OMPL hoạch định đường bay tiếp cận trên không.
  2. Hạ thẳng đứng (Cartesian) gắp đầu quân cờ.
  3. Nhấc bổng quân cờ lên độ cao an toàn $Z = 60\text{ mm}$ vượt qua đầu các quân cờ lân cận.
  4. Di chuyển ngang tới ô đích và hạ thẳng đứng đặt quân cờ xuống tâm ô.
  5. Nếu có tình huống ăn quân: Gắp quân bị ăn đưa ra ngoài bàn cờ trước khi chuyển quân đi tới.
* **Output:** Nước cờ hoàn tất trên bàn cờ thực tế, cập nhật trạng thái trong Planning Scene.
* **Failure cases:**
  * Sai số định vị tích lũy làm đầu kẹp chạm vào quân cờ bên cạnh gây đổ cờ.
  * Bộ giải OMPL không tìm thấy đường đi do không gian bị chiếm dụng quá dày đặc.
