# cube_vision — nhận diện cube dùng chung

Mỗi cube có 3 loại mặt: **AprilTag**, **màu**, **ảnh rác**. Đừng tự viết lại; gọi các phần ở đây.

| Module | Việc | Dùng khi |
|---|---|---|
| `registry` | Bảng 4 cube (ID↔màu↔tag↔lớp rác) + profile HSV có tên (`identify`, `proven`, `runtime`) | luôn — nguồn duy nhất |
| `tag.TagDetector` | tag36h11, tuỳ chọn CLAHE hai lượt, lọc ID | cần ID chắc chắn |
| `color.square_candidates` | ô vuông theo màu HSV | cube không thấy tag |
| `trash.TrashDetector` | DINOv2 khớp mặt rác | cần nhận cube qua mặt rác |
| `identify.Identifier(profile)` | gom các phần trên | gọi một dòng |

Profile của `Identifier`: `TAG` (nhanh), `TAG_COLOR` (tag rồi màu, search-center), `FULL` (tag + màu + rác + hợp nhất).

```python
from cube_vision.identify import Identifier, TAG_COLOR
hits = Identifier(TAG_COLOR).detect(frame_bgr, want_id=3)   # [CubeDetection]
```

Quy tắc: thêm cube/màu/ngưỡng ở `registry`; đừng chép bảng sang nơi khác. Ngưỡng HSV khác nhau giữa
các profile là có chủ đích (mỗi nơi đã tinh chỉnh cho ngữ cảnh), không gộp lại.

Test: `projects/cube_vision/test_cube_vision.py` (so với `golden_legacy.json` sinh từ hàm cũ trên khung tổng hợp).
Lưu ý: `pupil_apriltags` segfault nếu tạo/huỷ nhiều Detector trong một tiến trình, nên `TagDetector` dùng chung
một instance theo (backend, số luồng).

## Cải thiện nhận diện mặt rác (DINO)
DB gốc là ảnh dataset sạch; mặt rác in trên cube nhìn qua camera tay thì khác miền ảnh nên kém hơn tag/màu.
Cách bù bằng dữ liệu thật (không đổi model):
1. Tắt T8/perception rồi chạy `collect_trash_faces.py` (lật cube cho từng lớp, SPACE lưu 8-12 mẫu mỗi lớp).
2. `eval_trash_faces.py` in độ chính xác chỉ-DB-gốc vs gốc+camera-thật (leave-one-out), nhầm lẫn và ngưỡng đề xuất.
3. `TrashDetector` tự nạp `vector_database_dinov2_vits14_real.pt` nếu có (perception và search-center đều dùng chung);
   xoá file đó để quay về DB gốc.

Đo giả lập (16 ảnh tham chiếu, ảnh mặt đã cắt rồi thu nhỏ/mờ/nhiễu/méo): DINO gốc 100% (mờ nhẹ) và 90% (nặng);
thêm `build_trash_db.py` (biến thể kiểu camera) -> 100% và 95%, nhưng điểm của nhận sai cũng tăng (0.41 -> 0.66),
nên file `*_camaug.pt` chỉ nạp khi đặt `T8_TRASH_CAMAUG=1`. Chú ý: đo trên ảnh toàn cảnh (còn nền gỗ) sẽ ra ~40% —
chất lượng phụ thuộc rất nhiều vào việc cắt đúng mặt (rectify) trước khi đưa vào DINO.

## Vật nhỏ bị bắt làm cube (mảnh hình in) — `faces.py`
Nguyên nhân: DINO chấm mảnh hình in nhỏ (nắp pin, nét vẽ) ngang ngửa mặt thật (0,5 so với 0,5), nên điểm/ngưỡng
không phân biệt được "mặt đủ" và "mảnh"; mảnh thành đối tượng "2D only" trùng ID. Cách xử lý (không chỉ lọc):
- `expected_side_prior` (pose camera + K) / `fallback_prior`: khoảng cạnh pixel của mặt 30 mm; `classify_quad` -> FACE / PART / NOISE.
- PART có nhãn DINO làm **mỏ neo**: `grow_candidates` leo đồi dựng mặt vuông quanh nó (độ trắng của vành + ủng hộ cạnh
  liên tục theo từng cạnh), `propose_faces` chạy lại DINO trên mặt đủ đã nắn phẳng và chọn theo điểm + kích thước.
  Không dựng được mặt đủ thì mảnh **không tạo đối tượng**.
- Node perception: mặt dựng được phát đi sau >=2 lần cùng nhãn cùng chỗ (mặt sai hiếm khi lặp lại), cache bị xoá khi camera dịch;
  mỗi ID chỉ giữ một đối tượng (`dedupe_pose_results`, bỏ ghost cùng ID có pose yếu/2D-only).
- Tắt/bật khi chạy: `ros2 param set /object_perception face_grow_enabled false|true`.

Đo trực tiếp trên camera thật (4 cube đứng yên, 30-40 s mỗi lần, perception+DINO trên cuda):
| | tắt (cũ) | bật (mới) |
|---|---|---|
| ghost "ID 3 2D only" / khung | 1,45 | ~0,02 |
| cube ID 4 (giấy vệ sinh) có đối tượng | 0 % | ~95 % |
| đối tượng/khung (4 cube thật) | 5-6 | 4 |
Hạn chế đã biết: mặt dựng là xấp xỉ hình vuông (mặt nhìn xiên lệch vài chục px; vị trí ID 4 dao động ~10 mm theo y);
hộp xấp xỉ chỉ gieo hạt, pose vẫn đi qua cổng tầng/nghiêng của T8 và duyệt viewer.

## Khảo sát zone — zone_survey.py (module độc lập)
Camera tay xoay nhìn các ô thả và đo tâm ô trong tọa độ base. Không import T8/ROS: robot cụ thể đi vào
qua `ZoneLayout` (điểm thả cấu hình, `plan(zone, xy)` kiểm tra tay tới được, `ik_j1`), `arm.execute("look", servo=...)`
và `scene.zone_survey(zones, expect_j1=...)`. Kết quả mỗi zone: `keep | moved | fallback | blocked`;
`blocked` = không được thả (ô đã thấy nhưng tay không tới, hoặc điểm thả cấu hình đang nằm trên ô của zone khác).
- T8 dùng qua adapter `t8_pipeline/zone_survey.py` (`t8_layout()`); CLI chỉ đo: `python projects/t8_pipeline/zone_survey.py --zones 3 4`.
- Robot/đường sort khác: tạo `ZoneLayout` của mình rồi gọi `cube_vision.zone_survey.run_survey(arm, scene, layout, zones)`.
- Test: `cube_vision/test_zone_survey.py` (layout giả), `t8_pipeline/test_zone_survey.py` (adapter + T8).

## Kiểm tra đặt cube bằng camera ngoài — placement_check.py
Không cần hiệu chuẩn camera ngoài: tự tìm 4 ô bằng mặt nạ màu (`PAD_HSV`), so ảnh TRƯỚC/SAU khi thả; vùng đổi nằm
trong ô = cube vào ô. Verdict: `in_zone | partial | outside | unseen`. Bỏ qua vùng quanh đế robot (tay đổi pose).
- Dùng độc lập: `verify_in_zone(before_bgr, after_bgr, zone)`; hoặc `PlacementVerifier(ExternalCamera("/dev/video2"))`
  với `before()` / `check(zone)`.
- T8: `--verify-placement auto|off` (mặc định auto), `--external-camera` (mặc định camera USB khác camera tay),
  `--max-retries N` (mặc định 2). Lệch/rơi → gắp lại và đặt lại (mỗi lần vẫn duyệt viewer); `unseen` không thử lại.
- Xếp tầng: chỉ cần cube nguồn nằm trên cube đích (`STACK_CONTACT_TOL_M` = 20 mm, đúng tầng), xác minh bằng scene.
- Camera ngoài phải đã ổn phơi sáng (~1 s đầu rất tối); `ExternalCamera.grab()` chờ 30 khung rồi lấy trung vị.

## Lệnh "tất cả cube" — batch_plan.py + RosTaskRunner._execute_batch
`sort_cube {"label":"all"}` và `stack_cubes {"source":"all","target":"all"|"cube_N"}` (planner/Gemini sinh ra một bước duy nhất).
- Thứ tự (`batch_plan.plan_sort/plan_stack`): IK tới được làm trước, cube cao tầng trước, gần đế trước; tháp tối đa 4 tầng.
- Chạy tuần tự, khảo sát zone MỘT lần cho cả lô, mỗi cube vẫn một vòng duyệt Space (`--batch-approval-timeout`, mặc định 180 s:
  hết hạn thì bỏ cube đó, không treo). Lỗi riêng của cube (`SKIPPABLE_CODES`) thì làm tiếp cube kế; lỗi hệ thống/Esc thì dừng cả lô và
  báo cái gì đã xong. Giới hạn tổng thời gian `batch_deadline_s` (1800 s). Mỗi bước vẫn có xác nhận camera ngoài/scene và thử lại.

## Camera ngoài ↔ base — external_camera.py
Mô hình pinhole + k1 và pose `base_T_ext` của camera ngoài (1280×720), bộ giải từ điểm base ↔ pixel (góc AprilTag do camera
tay định vị ở nhiều độ cao), `project` / `pixel_to_base(u, v, z)` / `locate_tags(frame)` / `drift_px(frame)` (so tâm ô màu
với lúc hiệu chuẩn) / `relocalize` (camera bị dời: giữ ống kính, giải lại pose). Thu mẫu thật và các lệnh:
`projects/vision_experiments/calibrate_external.py` (`--collect` nhiều bộ, `--solve`, `--validate`, `--status`, `--relocalize`).
Chỉ dùng khi `accepted` (RMS ≤ 3 px, kiểm định ≤ 6 mm, ≥ 3 độ cao). T8 chỉ lấy thêm số đo "cube cách tâm ô N mm";
kết luận đúng/sai ô vẫn theo so màu trong `placement_check`.
