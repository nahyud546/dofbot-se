# 07. Dữ Liệu & Kết Quả Thực Nghiệm (Experimental Data, Benchmarks & Validation Logs)

Tài liệu này tổng hợp toàn bộ các số liệu thực nghiệm thực tế, nhật ký thử nghiệm (raw logs), các trường hợp thành công/thất bại (**Success / Failure Logs**), thời gian lập quỹ đạo (**Planning Time**), sai số định vị không gian (**Localization Error**), sai số mặt phẳng bàn và dung sai an toàn cơ điện tử được ghi nhận trực tiếp trên hệ thống robot Yahboom DOFBOT-SE.

---

## 1. Bảng Tổng Hợp Kết Quả Thực Nghiệm Các Phân Hệ (Benchmark Matrix)

| Tác vụ / Module | Ngày đo đạc | Môi trường / Phần cứng | Tình trạng | Thời gian thực thi / Độ trễ | Sai số đo đạc / Đánh giá kỹ thuật |
| :--- | :---: | :--- | :---: | :---: | :--- |
| **KDL Kinematics Rebuild** | 2026-09-23 | Offline / ROS 2 Humble | **PASS** | Build: $3.6\text{ s}$<br>FK/IK query: $< 1.2\text{ ms}$ | Sai số vị trí roundtrip: $0.00000\text{ m}$<br>Sai số hướng: $0.00000\text{ rad}$ |
| **T1: Color Sorting Fixed** | 2026-09-23 | Robot Sonix cam + ttyUSB0 | **PARTIAL** | Perception: $33\text{ ms}$ (30 FPS)<br>Debounce: 10 frames | Nhận diện màu PASS; Motion lỗi do tuột nguồn động cơ (RX servo trả về None). |
| **T-Dynamic: Color 3D** | 2026-09-23 | Sonix cam + ttyUSB0 | **PASS (Offline)**<br>*FAIL cũ trên HW* | IK solver: $2.4\text{ ms}$<br>Random-restart: $< 18\text{ ms}$ | Phát hiện và vá 3 lỗi cốt tử: Double inversion, nghịch đảo trục X, và kẹt NR single-seed. |
| **Table Plane Z-Mapping** | 2026-09-22 | Thước đo cơ khí + TCP touch | **PASS** | 4 điểm đo thực nghiệm | Mean $Z = 0.0452\text{ m}$; độ lệch max-min: $13.0\text{ mm}$. |
| **Zone Teaching (Red)** | 2026-09-22 | TCP Contact Teach | **PASS** | 3 lượt đo vị trí | Tọa độ trung bình: $(0.109, -0.168)\text{ m}$; sai số thả rơi thực tế (drop test): $3.2\text{ mm}$. |
| **T7: Voice Bring-up** | 2026-09-24 | Shim Speech_Lib + ttyUSB0 | **PASS** | Polling rate: $20\text{ Hz}$<br>Mock inject: tức thời | Khắc phục 21 script bị hardcode đường dẫn; kiểm thử chuỗi lệnh LED 11, 12, 13 thành công. |
| **T8: LLM / VLM Pipeline** | 2026-09-24 | Gemini 3.6 Flash + Tavily | **PASS** | Gemini API: $\sim 850\text{ ms}$<br>Tavily Search: $\sim 620\text{ ms}$ | Tỷ lệ tuân thủ schema JSON whitelist: $100\%$ (tuyệt đối không sinh mã shell độc hại). |
| **SafetyGate Pre-flight** | 2026-09-21 | 16 bài kiểm tra quỹ đạo | **PASS** | Thời gian validate: $< 5\text{ ms}$ | Chặn đứng hiện tượng vung tay bất ngờ và nhảy bước góc khớp $> 0.6\text{ rad}$. |

---

## 2. Nhật Ký Chi Tiết Thực Nghiệm Động Học KDL (Kinematics Benchmark Data)

*Nguồn trích xuất: `log/2026-09-23_IK-rebuild.md` và `/tmp/kin_test2.cpp`.*

### 2.1. Kiểm thử Động học Thuận (FK Verification)
* **Vector góc khớp kiểm thử 1 (Home pose):**
  $$q = [90^\circ, 90^\circ, 90^\circ, 90^\circ, 90^\circ] \rightarrow q_{\text{rad}} = [0.0, 0.0, 0.0, 0.0, 0.0]$$
  * Tọa độ đầu ra KDL: $X = -0.0048\text{ m}, \, Y = +0.0007\text{ m}, \, Z = +0.4374\text{ m}$
  * Ma trận hướng: $\text{Roll} = 0.0, \, \text{Pitch} = 0.0, \, \text{Yaw} = 0.0$
* **Vector góc khớp kiểm thử 2 (Grasp-down pose `P_BLACK_CENTER`):**
  $$q = [90^\circ, 35^\circ, 65^\circ, 15^\circ, 90^\circ] \rightarrow q_{\text{rad}} = [0.0, -0.9599, -0.4363, -1.3090, 0.0]$$
  * Tọa độ đầu ra KDL: $X = -0.2069\text{ m}, \, Y = +0.0007\text{ m}, \, Z = +0.0528\text{ m}$
  * Ma trận hướng: $\text{Roll} = 0.0, \, \text{Pitch} = 1.1345\text{ rad} \, (\approx 65^\circ), \, \text{Yaw} = -\pi$

### 2.2. Kiểm thử Động học Nghịch & Tỷ Lệ Hội Tụ (IK Convergence Comparison)
Thực nghiệm so sánh bộ giải Newton-Raphson 1 hạt giống (bản gốc Yahboom) và bộ giải Multi-Seed + Random Restart cải tiến:

| Tọa độ mục tiêu $(X, Y, Z)\text{ [m]}$, $(\text{Pitch}, \text{Yaw})$ | Kết quả Bộ giải Gốc (1 Seed) | Kết quả Bộ giải Mới (13 Seeds + Random Restart) | Sai số vị trí cuối (Position Error) | Sai số góc Pitch (Pitch Error) |
| :--- | :---: | :---: | :---: | :---: |
| $(-0.2069, 0.0007, 0.0828)$, $P=1.04, Y=-\pi$ | Kẹt Gimbal Lock $\rightarrow$ Trả về vector 0 | **Hội tụ tại Seed 2**:<br>$[90.0^\circ, 44.7^\circ, 64.7^\circ, 11.0^\circ, 90.0^\circ]$ | $0.00000\text{ m}$ | $0.0000\text{ rad}$ |
| $(+0.2000, 0.0000, 0.0500)$ *(Lỗi dấu $+X$ cũ)* | Thất bại hoặc xoắn khớp $J_4 = 180^\circ$ | **Hội tụ cưỡng bức**:<br>$J_4 = 180^\circ$ (Chứng minh $+X$ nằm sau lưng robot) | $0.0085\text{ m}$ | $0.4500\text{ rad}$ |
| $(-0.2200, 0.0500, 0.0450)$, $P=1.04, Y=-\pi$ | Bị kẹt biên khớp | **Hội tụ qua Random Restart** (Lượt thứ 42):<br>$[90.4^\circ, 58.1^\circ, 42.5^\circ, 3.8^\circ, 44.9^\circ]$ | $\mathbf{0.0027\text{ m}} \, (2.7\text{ mm})$ | $\mathbf{0.2800\text{ rad}} \, (\approx 16^\circ)$ |

---

## 3. Dữ Liệu Thực Nghiệm Hiệu Chuẩn Bàn & Vùng Thao Tác (Table Teach Raw Data)

*Nguồn trích xuất: `dofbot_robot_arm_6dof/src/cap_vision/config/table_zones.yaml` (Đo trực tiếp ngày 2026-09-22).*

### 3.1. Phân bố độ cao mặt bàn thực tế ($Z_{\text{table}}$)
Thực hiện hạ đầu kẹp robot chạm trực tiếp vào 4 góc của thảm làm việc Yahboom, ghi nhận giá trị trục Z trong hệ quy chiếu `base_link`:

| Điểm chạm thực tế | Tọa độ $X\text{ [m]}$ | Tọa độ $Y\text{ [m]}$ | Độ cao $Z\text{ [m]}$ | Độ lệch so với trung bình ($\Delta Z$) |
| :--- | :---: | :---: | :---: | :---: |
| **Zone Green (Góc trên trái)** | $+0.036$ | $+0.161$ | $+0.051$ | $+5.8\text{ mm}$ |
| **Zone Yellow (Góc trên phải)**| $+0.128$ | $+0.165$ | $+0.046$ | $+0.8\text{ mm}$ |
| **Zone Red (Góc dưới phải)** | $+0.117$ | $-0.173$ | $+0.046$ | $+0.8\text{ mm}$ |
| **Zone Blue (Góc dưới trái)** | $+0.034$ | $-0.156$ | $+0.038$ | $-7.2\text{ mm}$ |
| **GIÁ TRỊ TRUNG BÌNH (Mean)** | — | — | $\mathbf{+0.0452\text{ m}}$ | **Độ trải rộng (Spread): $13.0\text{ mm}$** |

> [!NOTE]
> Độ chênh lệch $13\text{ mm}$ giữa điểm cao nhất ($51\text{ mm}$) và thấp nhất ($38\text{ mm}$) xuất phát từ độ nghiêng của bàn làm việc thực tế và độ võng cơ học khi vươn xa. Do đó, hệ thống chọn độ cao tham chiếu an toàn chuẩn là $Z_{\text{table}} = 0.045\text{ m}$, và khi tiếp cận gắp luôn cộng thêm khoảng đệm an toàn $Z_{\text{offset}} = +5.0\text{ mm}$.

### 3.2. Dữ liệu dạy vị trí 4 Khay Màu (Zone Centers) & Thử Nghiệm Thả Rơi
Thực hiện dạy điểm (Teach-in) vị trí trung tâm của 4 ô chứa màu:
* **Vùng Vàng (Yellow):** $(X = 0.128\text{ m}, \, Y = 0.165\text{ m})$
* **Vùng Xanh lá (Green):** $(X = 0.036\text{ m}, \, Y = 0.161\text{ m})$
* **Vùng Xanh dương (Blue):** $(X = 0.034\text{ m}, \, Y = -0.156\text{ m})$
* **Vùng Đỏ (Red - Thử nghiệm 3 lần độc lập):**
  * Lần 1: $(0.117, -0.173)\text{ m}$
  * Lần 2 (Re-teach): $(0.102, -0.161)\text{ m}$
  * Lần 3 (Drop test thực tế): $(0.108, -0.171)\text{ m}$
  * **Tọa độ chốt (Mean):** $(X = 0.109\text{ m}, \, Y = -0.168\text{ m})$
  * **Sai số định vị rơi tự do (Drop deviation):** $\Delta = \sqrt{(0.108 - 0.109)^2 + (-0.171 - (-0.168))^2} = \mathbf{3.2\text{ mm}}$ (đạt độ chính xác tuyệt đối lọt trọn trong miệng khay có kích thước $60 \times 60\text{ mm}$).

---

## 4. Dữ Liệu Kiểm Soát Quỹ Đạo & Cổng An Toàn (Safety Gate Metrics)

*Nguồn trích xuất: `hardware/safety_config.yaml` và `hardware/safety_gate.py`.*

| Tham số kiểm soát an toàn | Ngưỡng cấu hình (Threshold) | Kết quả giám sát thực tế | Trạng thái bảo vệ |
| :--- | :---: | :---: | :---: |
| **Bước nhảy góc tối đa giữa 2 waypoint** | $\Delta q \le 0.60\text{ rad} \, (\approx 34.4^\circ)$ | Cực đại ghi nhận: $0.38\text{ rad}$ | **PASS** (Triệt tiêu hiện tượng văng giật) |
| **Dung sai khớp tư thế bắt đầu (Start state)** | $\|q_0 - q_{\text{actual}}\| \le 0.05\text{ rad}$ | Thực tế dao động: $0.012 - 0.035\text{ rad}$ | **PASS** (Không giật khớp về điểm 0) |
| **Hệ số co giãn vận tốc tối đa (Velocity scale)**| $s_v \le 0.25$ | Chạy thực nghiệm: $0.10 - 0.25$ | **PASS** (Động cơ chuyển động êm) |
| **Ngưỡng dừng khẩn cấp sai số bám (Tracking error)**| $e_{\text{stop}} = 0.15\text{ rad} \, (\approx 8.6^\circ)$| Bình thường: $\le 0.04\text{ rad}$<br>Khi bị kẹt cơ học: $> 0.16\text{ rad}$ | **Kích hoạt ngắt khẩn cấp sau 1 chu kỳ đọc** |
| **Tần số đọc phản hồi trạng thái thực (Readback)** | $2.0\text{ Hz}$ | Thực tế: $1.85 - 2.10\text{ Hz}$ | Giới hạn bởi thời gian chờ UART bán song công |

---

## 5. Nhật Ký Sự Cố Chẩn Đoán Cổng Nối Tiếp & Động Cơ (Serial & Motor Bug Log)

*Nguồn trích xuất: `log/2026-09-23_T1_color-sorting.md` và `log/2026-09-23_T-dynamic-sorting.md`.*

### Sự cố 1: Hiện tượng "Servo 1/2 Bị Chết Ảo" do Race-Condition Đệm Nối Tiếp
* **Hiện tượng:** Sau khi mở cổng `/dev/ttyUSB0`, chạy lệnh đọc servo `read_servos()` thì hàm trả về `None` hoặc giá trị góc âm vô lý (ví dụ: $-73^\circ$). Nghi ngờ servo hỏng phần cứng.
* **Nguyên nhân kỹ thuật:** Chip cầu nối CH340 tồn đọng các byte rác trong bộ đệm nhận (FIFO buffer) ngay thời điểm khởi tạo cổng COM. Lượt đọc truy vấn đầu tiên bị lệch khung truyền (frame out-of-sync), dẫn tới checksum thất bại.
* **Giải pháp đã kiểm chứng:** Thực hiện quy tắc **"Bỏ qua lượt đọc đầu tiên" (Discard-First-Read)**. Từ lượt đọc thứ 2 trở đi, toàn bộ 6/6 servo phản hồi chính xác 100%:
  $$\text{Angles Readback} = [117.0^\circ, 81.0^\circ, 81.0^\circ, 92.0^\circ, 90.0^\circ, 28.0^\circ]$$

### Sự cố 2: Còi Kêu Nhưng Cánh Tay Không Nhúc Nhích (T1 Test)
* **Hiện tượng:** Lệnh gửi `sorting_run()` phát âm thanh còi Buzzer kêu bình thường, cửa sổ camera hiển thị nhận diện đúng màu Đỏ, nhưng cánh tay hoàn toàn đứng yên.
* **Nguyên nhân kỹ thuật:** Lệnh còi `Buzzer_On()` và lệnh quay động cơ `write6()` chỉ là các lệnh gửi một chiều (fire-and-forget qua `ser.write()`). Việc còi kêu chỉ chứng minh đường truyền logic TX từ laptop sang bo STM32 còn sống. Kiểm tra nguồn cấp phát hiện giắc cắm nguồn động cơ 7.4V bị lỏng, khiến vi điều khiển có điện nhưng thanh cái cấp nguồn cho các cuộn dây motor servo bị mất điện hoàn toàn.
