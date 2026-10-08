# Số đo trên phần cứng

Chỉ số đã đo. Mỗi dòng có ngày và cách đo. Số từ mô phỏng ghi rõ "mô phỏng".

## Camera tay và hand-eye

| Ngày | Đại lượng | Giá trị | Cách đo |
|---|---|---|---|
| 2026-10-06 | Vị trí camera trên arm4 | x = +40 mm, trục quang song song khâu (URDF cũ ghi −48 mm) | hiệu chuẩn 16 mẫu |
| 2026-10-06 | Ống kính (lần đầu) | fx≈896, fy≈941, k1≈−0,5 | hiệu chuẩn 16 mẫu |
| 2026-10-06 | K sau hiệu chuẩn 3 nhóm | fx/fy ≈ 935/990 | `cube_6d_calibrated.yaml` |
| 2026-10-06 | Sai số XY mặt bàn (giữ lại kiểm) | ≤ 2,3 mm | cross-validation, 52 mẫu có neo |
| 2026-10-06 | Sai số PnP-3D | 2–7 mm ở tâm; ~22 mm nhóm LEFT (cube nghiêng ~6°, cao hơn ~8 mm) | cùng bộ mẫu |
| 2026-10-07 | Kiểm chứng hand-eye 3 nhóm | tâm 1,5 mm · RIGHT tối đa 3,1 mm · LEFT 6,3 mm ở J1≈140° (≤ 2,5 mm tới J1 130°) | `validate_hand_eye.py`, tay thật |
| 2026-10-06 | Độ phân giải đọc servo | Arm_Lib cắt về độ nguyên; đếm thô (`servo_H/L`) cho 0,08° | đọc code + mô phỏng: sai số hiệu chuẩn 6–15 mm → 1–3 mm |

## Tầm với và IK

| Ngày | Đại lượng | Giá trị | Cách đo |
|---|---|---|---|
| 2026-10-06 | Dịch vụ IK ROS cũ | 312/728 điểm lưới giải được; ở z=0,047 chỉ dùng được x ∈ [−0,19; −0,11] | probe IK, không chạy motor |
| 2026-10-06 | Tầm với hình học | r = 0,06–0,30 m nếu cho góc kẹp đổi theo tầm | tính toán |

## Ô thả và tìm kiếm

| Ngày | Đại lượng | Giá trị | Cách đo |
|---|---|---|---|
| 2026-10-07 | Pose nhìn trọn ô | J2=110; J1=12 (ô 1,3) và J1=165 (ô 2,4) | tay thật |
| 2026-10-07 | Độ lặp lại tâm ô | ~2 mm; diện tích ô ~0,0053 m² | khảo sát lặp |
| 2026-10-07 | Bảng `BIN_*` so với ô thật | lệch 6–9 cm | khảo sát zone |
| 2026-10-07 | Ô 1, 2 | cần J1≈2° / 176° → ngoài `ZONE_J1_LIMITS`, bị từ chối | khảo sát zone |
| 2026-10-06 | Thời gian tìm tới khi khóa cube | 12,3 s so với 18,4 s (bắt đầu J1=40); 15,5 so với 25,2 s (J1=150) | `search_planner` FAST so với NORMAL, tay thật |
| 2026-10-07 | Xếp tháp | 4 tầng từ pose lệch: được | T8, tay thật |

## Luồng ảnh và khớp

| Ngày | Đại lượng | Giá trị | Cách đo |
|---|---|---|---|
| 2026-10-07 | Ảnh thô, QoS RELIABLE | ~15 khung/s, đứt tối đa 0,08 s (best-effort: 3–5 khung/s, đứt 1–4 s) | node probe |
| 2026-10-07 | Ảnh perception, RELIABLE | đứt tối đa 1,2 s | node probe |
| 2026-10-06 | `real_joint_mirror` | một mẫu mỗi ~1,5 s → cửa sổ `JointWindow` phải 3,5 s; look + đo mất ~5 s | tay thật |
| 2026-10-06 | Tỉ lệ sẵn sàng ở 3 pose lệch READY | ID4 đạt 85–89 %, sẵn sàng lần đầu ~2 s | tay thật |
