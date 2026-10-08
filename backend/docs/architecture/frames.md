# Hệ tọa độ và các phép biến đổi

Ghi chú bảo trì: repo có những hệ tọa độ nào, ma trận nào nối chúng, số nằm ở file nào và tạo lại bằng lệnh gì.
Nguồn sự thật là mã, không phải file này:

- khai báo: [`projects/vision_experiments/dofbot_frames.py`](../../projects/vision_experiments/dofbot_frames.py)
- toán ghép chuỗi và mô hình camera: [`projects/cube_vision/frames.py`](../../projects/cube_vision/frames.py)

Phần "Số hiện tại" ở cuối được sinh bằng lệnh, đừng sửa tay:

```bash
python projects/vision_experiments/dofbot_frames.py --dump                       # toàn bộ bảng
python projects/vision_experiments/dofbot_frames.py --chain world wrist_optical  # một chuỗi + ma trận 4×4
python projects/vision_experiments/dofbot_frames.py --chain ext_optical wrist_optical --q 60 110 10 5 90
```

## Quy ước

- `a_T_b` là ma trận 4×4 đưa điểm viết trong hệ **b** sang hệ **a**: `p_a = a_T_b @ p_b`.
  Ghép chuỗi đọc từ trái sang phải: `a_T_c = a_T_b @ b_T_c`. Tên biến trong mã theo đúng dạng này
  (`base_T_ext`, `arm4_T_optical`).
- Đơn vị mét và radian trong mã; bảng dưới in mm và độ cho dễ đọc.
- Hệ quang học (`*_optical`): z ra trước ống kính, x sang phải ảnh, y xuống dưới ảnh. Pixel `(u, v)` gốc ở góc
  trên trái.
- Góc khớp `q` là 5 góc **servo** tính bằng độ (0–180), `q_rad = (servo − 90)·π/180`.
- `rpy` in theo quy ước URDF/TF: `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.

## Sơ đồ

```
world ≡ base_link ── arm1 ── arm2 ── arm3 ── arm4 ── arm5 ── tcp
            │         J1      J2      J3      J4  │   J5
            │                                     └── wrist_cam ── wrist_optical     camera tay
            ├── table
            ├── ext_optical                                                           webcam laptop
            └── phone_optical                                                         iPhone (DroidCam)
```

So với sơ đồ 6 hệ chuẩn của tay máy eye-in-hand, repo này khác ở ba điểm cần nhớ:

1. **Camera tay gắn trên `arm4`, không phải trên mặt bích `arm5`.** Khớp J5 xoay kẹp nhưng không xoay camera.
   "`{ee}` của camera" ở đây là `arm4`; "`{ee}` của kẹp" là `arm5`.
2. File `hand_eye.json` lưu **một** ma trận gộp `arm4_T_optical = arm4_T_wrist_cam · wrist_cam_T_wrist_optical`.
   Đồ thị tách nó ra để phần quy ước (xoay trục) không lẫn với phần hiệu chuẩn (lắp lệch).
3. `world` trùng `base_link` (`world_T_base_link = I`). Mọi tọa độ báo ra ngoài nên gọi là tọa độ `world`; muốn
   đặt world ở chỗ khác chỉ cần đổi đúng một cạnh này.

## Các chuỗi chính

```
world_T_wrist_optical(q) = world_T_base_link · base_link_T_arm4(q) · arm4_T_wrist_cam · wrist_cam_T_wrist_optical
world_T_tcp(q)           = world_T_base_link · base_link_T_arm5(q) · arm5_T_tcp
world_T_ext_optical      = world_T_base_link · base_link_T_ext_optical
world_T_phone_optical    = world_T_base_link · base_link_T_phone_optical
camA_T_camB              = inv(world_T_camA) · world_T_camB        # hai camera bất kỳ, luôn đi qua world
```

Một điểm world `P` hiện ở pixel nào của camera k: `p_optical = inv(world_T_optical_k) @ P`, rồi chiếu bằng
intrinsic của camera đó (`CameraModel.project`). Ngược lại, một pixel chỉ cho **một tia**
(`CameraModel.ray`), chưa cho điểm 3D; xem mục "Độ sâu".

## Mỗi camera cần bốn thứ

| | Camera tay (`wrist_optical`) | Webcam laptop (`ext_optical`) | iPhone (`phone_optical`) |
|---|---|---|---|
| **Extrinsic** (đang ở đâu trong world) | FK từ góc servo + hand-eye. Thay cho GPS/IMU. Tin trong J1 35–140°. | Hiệu chuẩn một lần, đứng yên. Bị chạm thì `--relocalize`. | Chưa hiệu chuẩn. Cố định: như webcam. Cầm tay: định vị lại mỗi khung bằng tag đã có trong world. |
| **Intrinsic** | `hand_eye.json` K, k1, fit chung với hand-eye (RMS 8,8 px: yếu). | `external_camera.json` K, k1, fit chung với pose (tâm ảnh kém ổn định). | Chưa có. |
| **Độ sâu** | Mặt phẳng đã biết (mặt tag ở tầng n) hoặc kích thước tag 20 mm (PnP). | Mặt phẳng đã biết. | Như webcam; nhiều camera cùng thấy thì tam giác hóa. |
| **Thời gian** | Chỉ ghép quan sát khi tay đã đứng yên và cảnh tĩnh. Chưa có đồng bộ đồng hồ giữa các luồng. | như bên | như bên; DroidCam trễ mạng ~0,1–0,3 s. |

Intrinsic nên hiệu chuẩn riêng bằng bảng ChArUco (`calibrate_intrinsics.py`, ghi
`config/robot/cameras/<tên>.json`); sau đó extrinsic chỉ còn 6 tham số pose. Hiện mới có mã, chưa camera nào
được chạy qua bước này.

### Độ sâu: vì sao một pixel chưa đủ

Camera thường chỉ cho hướng nhìn. Repo lấy độ sâu theo các cách sau, xếp từ đang dùng nhiều nhất:

- **Mặt phẳng đã biết.** Mặt tag của cube ở tầng n nằm ở `z = tag_top_z + 0,030·n` trong base. Cắt tia với mặt
  phẳng đó ra x, y (`frames.pixel_to_plane`, `ExternalCalibration.pixel_to_base`, `cube_layer.locate_cube`).
- **Kích thước vật đã biết.** Tag 20 mm → PnP cho pose 6D; khoảng cách kém chính xác khi chỉ có một góc nhìn.
- **Nhiều góc nhìn.** Camera tay nhìn cùng một tag từ vài pose, hoặc hai camera cùng thấy: ra z thật không cần giả
  thiết mặt phẳng (`cube_vision/multiview.py`, vòng tự nhìn quanh ở `vision_experiments/active_view.py`).

## File hiệu chuẩn

| File | Chứa | Tạo lại | Kiểm |
|---|---|---|---|
| `config/robot/hand_eye.json` | `arm4_T_optical`, K, k1, `tag_top_z`, `table_z`, vùng J1 | `calibrate_hand_eye.py` | `validate_hand_eye.py` |
| `config/robot/hand_eye.center_proven.json` | mốc đã kiểm bằng gắp thật: **không xóa, không ghi đè** | — | — |
| `config/robot/external_camera.json` | `base_T_ext`, K, k1, mốc ô màu chống trôi | `calibrate_external.py --collect` ×6, `--solve` | `--status`, `--validate` |
| `config/robot/cameras/<tên>.json` | K, k1, k2, `rotate`, `image_size`; khi đã đặt vào world: `base_T_optical` | `calibrate_intrinsics.py --camera <tên>`; `build_world.py --locate <tên> --save` | `build_world.py --check <tên>` |
| `data/world/latest.json` | world map: tag, cube, ô, camera (không commit) | `build_world.py --scan` | `build_world.py --show` |

## Dùng trong mã

```python
import dofbot_frames as D
g = D.build()                                    # đọc các file hiệu chuẩn
world_T_cam = g.lookup("world", "wrist_optical", q=[90, 125, 0, 0, 90])
ext_T_wrist = g.lookup("ext_optical", "wrist_optical", q)   # camera tay nằm đâu trong mắt webcam
cam = D.cameras()["ext_optical"]
uv = cam.project((g.lookup("ext_optical", "world") @ [x, y, z, 1])[:3])   # điểm world -> pixel webcam
```

Chuỗi đi qua một cạnh chưa hiệu chuẩn sẽ báo `MissingTransform` kèm lệnh cần chạy, không trả số đoán.
Đường gắp/thả đang chạy của T8 vẫn gọi trực tiếp `fk_arm4_cal(q, cal) @ cal["arm4_T_optical"]`; test
`test_dofbot_frames.py` khóa hai đường cho ra cùng một ma trận, và khóa hai bản sao hằng số URDF
(`cube_search_center_math.py`, `dofbot_ik.py`) không lệch nhau.

## World map: từ một camera di chuyển tới tọa độ mà camera khác dùng được

```
camera tay quét + tự nhìn quanh ──▶ pose 6D từng tag trong world ──▶ data/world/latest.json
                                                                          │
              iPhone / webcam thấy các tag đó ──▶ pose_from_tags ──▶ world_T_optical của camera ấy
                                                                          │
                                    world chiếu ngược vào ảnh camera ấy (vẽ đè) / RViz
```

| Bước | Lệnh | Mã |
|---|---|---|
| Intrinsic một lần cho mỗi camera | `python projects/vision_experiments/calibrate_intrinsics.py --camera phone --with-board` | `cube_vision/intrinsics.py` |
| Tay quét, tự nhìn thêm khi chưa chắc | `python projects/vision_experiments/build_world.py --scan` | `active_view.py`, `cube_vision/view_quality.py`, `cube_vision/multiview.py` |
| World tự cập nhật, tay ở pose bất kỳ, không lái tay | `python projects/vision_experiments/build_world.py --watch` (`--goto J1..J5`: đi tới một pose rồi nhìn; `--free`: nhả lực servo để tự bẻ tay) | `vision_experiments/world_watch.py` |
| Đặt camera khác vào world | `python projects/vision_experiments/build_world.py --locate phone --save` | `cube_vision/camera_pose.py` |
| Camera cố định còn nguyên chỗ không | `python projects/vision_experiments/build_world.py --check phone` | `camera_pose.drift_px` |
| Vẽ world đè lên ảnh camera, theo kịp camera dời và cube dời | `cd projects && python -m cube_vision.world_overlay --camera phone` (`--fixed`: dùng pose đã lưu; `--write-live FILE`: ghi world sống cho RViz) | `cube_vision/world_overlay.py`, `cube_vision/live_world.py` |
| Xem 3D | `ros2 launch cap_vision world_view.launch.py` | `ros/src/cap_vision/cap_vision/world_publisher.py` |

Số đo thật ngày 2026-10-08 (4 cube trên bàn, iPhone đặt sát mặt bàn cách đế ~43 cm):

- Camera tay, nhiều góc nhìn: phương ngang các góc nhìn lệch nhau 0,1–1,3 mm; độ cao đo thấp hơn thật 1,4–4,7 mm.
  Nguyên nhân: hand-eye hiện tại chỉ đạt trên mặt phẳng (`accepted_3d: false`), pose camera sai cỡ mm khi J3 ≠ 0.
  Vì vậy (a) gộp bằng trung vị PnP từng góc nhìn (`multiview.robust_tag`), không tam giác hóa: tam giác hóa với
  thị sai vài cm khuếch đại sai số pose thành z lệch tới +8 mm; (b) tag nằm ngửa đúng tầng thì lấy độ cao đã biết
  của tầng (`world_map.snap_to_layer`), tag nghiêng/dựng đứng giữ số đo 3D.
- iPhone đặt vào world bằng 3 tag: RMS chiếu lại 1,9 px; tag thứ tư không tham gia giải lệch 12 px (≈ 6 mm).
  Vị trí iPhone lặp lại trong 3 mm giữa các lần giải.
- Tag trên bàn gần đồng phẳng nên PnP có nghiệm lật (camera "chui xuống dưới bàn"); `camera_pose` tối ưu từ nhiều
  pose khởi tạo và bỏ nghiệm nằm dưới mặt bàn.

Muốn z thật tốt hơn (không cần mặt phẳng): hiệu chuẩn intrinsic camera tay bằng ChArUco rồi giải lại hand-eye
chỉ với 6 tham số pose cho tới khi đạt 3D.

`--watch` đo thật 2026-10-08: cùng một cube nhìn từ 3–5 pose tay tùy ý (J3, J4 gập, J5 xoay 40°/150°) cho tọa độ
world lệch nhau theo phương ngang trung vị 1,2 mm, lớn nhất 3,0 mm (12 lần nhìn, 3 cube). Mỗi lần nhìn dùng góc khớp
thật đọc về lúc đó; lần nhìn có J1 ngoài vùng hand-eye bị bỏ. Cube thấy ở chỗ khác chỗ đã ghi quá 15 mm là đã dời;
cube lẽ ra trong khung nhìn mà 3 lần liền không thấy (và lần nhìn đó đáng tin: có thấy tag khác hoặc ảnh nét) thì
xóa; cube ngoài khung nhìn được giữ. `world_overlay` tự nạp lại file world khi nó đổi.

Luật "nhìn chưa ổn" (ngưỡng ở đầu `view_quality.py`): tag nhỏ hơn 28 px, sát mép ảnh, PnP lệch quá 3 px, nhìn
xiên quá 65°, hướng mặt tag còn mơ hồ, J1 ngoài vùng hand-eye; và sau khi gộp: ít hơn 2 góc nhìn, hai góc nhìn
cách nhau dưới 20 mm, các góc nhìn lệch nhau quá 4 mm theo phương ngang hoặc 8 mm theo độ cao. Mỗi luật kèm gợi ý
(lại gần, vào giữa ảnh, nhìn thẳng hơn, đổi chỗ) để bộ chọn pose biết đổi gì; tối đa 4 lần nhìn thêm cho mỗi tag,
hết thì báo "CHƯA CHẮC" chứ không ép ra số.

Chế độ theo dõi (mặc định của `world_overlay`): mỗi khung, các tag mà world cho là đứng yên làm mốc để định vị lại
camera (khung sau khởi tạo từ pose khung trước nên chỉ mất vài ms). Tag lệch khỏi phép dời cứng của camera là cube
đã bị dời: nó được đo lại ngay từ khung đó (PnP một góc nhìn + độ cao tầng) và vẽ màu cam "(doi live)"; cube mới
cũng vậy; cube lẽ ra thấy mà không thấy thì vẽ xám "(khong thay)". Số đo live kém chính xác hơn số đo quét bằng
camera tay (một góc nhìn, camera ở xa), và file world gốc không bị đổi: muốn cập nhật chính thức thì `--scan` lại.
Cần ít nhất 3 tag đứng yên để biết tag nào đã dời; chỉ có 2 tag mà không khớp nhau thì báo "chưa định vị".

Camera cầm tay: không có GPS/IMU, nên mỗi khung phải thấy ít nhất 2 tag mà world đã biết chắc, tách nhau từ 40 mm;
không đủ thì báo "chưa định vị". World là ảnh chụp của một cảnh tĩnh: sau khi tay gắp/thả phải quét lại.

## Số hiện tại

Sinh ngày 2026-10-08 bằng `dofbot_frames.py --dump`.

| Hệ tọa độ | Ý nghĩa |
|---|---|
| `world` | Hệ gốc chung của mọi camera và mọi tọa độ báo ra. Chọn trùng base_link. |
| `base_link` | Đế tay máy, gắn cố định với bàn. Gốc ở tâm đế, z lên; vùng làm việc ở phía −x. |
| `arm1` | Sau khớp J1 (xoay quanh z). |
| `arm2` | Sau khớp J2 (vai, quanh y). |
| `arm3` | Sau khớp J3 (khuỷu, quanh y). |
| `arm4` | Sau khớp J4 (cổ tay gập, quanh y). CAMERA TAY GẮN Ở ĐÂY, không phải ở arm5. |
| `arm5` | Sau khớp J5 (xoay kẹp quanh z). Mặt bích gắn kẹp; J5 KHÔNG làm xoay camera. |
| `tcp` | Điểm kẹp (URDF Gripping_point_Link): mục tiêu của IK gắp/thả. |
| `wrist_cam` | Thân camera tay, trục song song arm4. |
| `wrist_optical` | Hệ quang học camera tay (ảnh 640×480): z ra trước, x phải, y xuống. |
| `table` | Mặt bàn: cùng hướng base_link, gốc hạ xuống mặt bàn (z = 0 là mặt bàn). |
| `ext_optical` | Hệ quang học webcam laptop (1280×720), đứng yên ngoài tay máy. |
| `phone_optical` | Hệ quang học iPhone qua DroidCam (1280×720 sau khi xoay ảnh). |

| Phép biến đổi | Loại | Trạng thái | Nguồn số | Giá trị (khớp ở READY nếu theo khớp) |
|---|---|---|---|---|
| `world_T_base_link` | quy ước | hằng | dofbot_frames.build | xyz = (+0.0, +0.0, +0.0) mm; rpy = (+0.0, -0.0, +0.0)° |
| `base_link_T_arm1` | FK | theo khớp | cube_search_center_math.J1_O (URDF arm1_Joint) | xyz = (+0.0, +0.0, +92.5) mm; rpy = (+0.0, -0.0, +0.0)° |
| `arm1_T_arm2` | FK | theo khớp | cube_search_center_math.J2_O (URDF arm2_Joint) | xyz = (+0.0, +0.1, +33.0) mm; rpy = (+0.0, +35.0, +0.0)° |
| `arm2_T_arm3` | FK | theo khớp | cube_search_center_math.J3_O (URDF arm3_Joint) | xyz = (+0.0, +0.6, +82.8) mm; rpy = (+0.0, -90.0, +0.0)° |
| `arm3_T_arm4` | FK | theo khớp | cube_search_center_math.J4_O (URDF arm4_Joint) | xyz = (+0.0, +0.1, +82.8) mm; rpy = (+0.0, -90.0, +0.0)° |
| `arm4_T_arm5` | FK | theo khớp | cube_search_center_math.J5_O (URDF arm5_Joint) | xyz = (-2.1, -0.0, +78.1) mm; rpy = (+0.0, -0.0, +0.0)° |
| `arm5_T_tcp` | CAD | hằng | dofbot_ik.GRIP_O (URDF Gripping_Joint) | xyz = (-2.6, +0.1, +68.1) mm; rpy = (+0.0, -90.0, +180.0)° |
| `arm4_T_wrist_cam` | hiệu chuẩn hand-eye | hằng | config/robot/hand_eye.json: arm4_T_optical · inv(wrist_cam_T_wrist_optical) | xyz = (+41.3, -0.9, +43.2) mm; rpy = (+1.4, +0.9, +1.8)° |
| `wrist_cam_T_wrist_optical` | quy ước | hằng | cube_search_center_math.MOUNT_RZ90 | xyz = (+0.0, +0.0, +0.0) mm; rpy = (+0.0, -0.0, +90.0)° |
| `base_link_T_table` | hiệu chuẩn hand-eye | hằng | config/robot/hand_eye.json: table_z (= tag_top_z − 0,030) | xyz = (+0.0, +0.0, +27.8) mm; rpy = (+0.0, -0.0, +0.0)° |
| `base_link_T_ext_optical` | hiệu chuẩn camera ngoài | hằng | config/robot/external_camera.json: base_T_ext | xyz = (-354.1, +14.3, +313.5) mm; rpy = (-141.5, +1.2, -91.6)° |
| `base_link_T_phone_optical` | hiệu chuẩn camera ngoài | hằng | config/robot/cameras/phone.json: base_T_optical | xyz = (-428.7, -28.2, +153.6) mm; rpy = (-103.1, -1.2, -86.0)° |

Chuỗi tới từng camera (q = [90.0, 125.0, 0.0, 0.0, 90.0]):

- `world_T_wrist_optical` = world_T_base_link · base_link_T_arm1(q) · arm1_T_arm2(q) · arm2_T_arm3(q) · arm3_T_arm4(q) · arm4_T_wrist_cam · wrist_cam_T_wrist_optical  
  → xyz = (-79.0, -0.3, +229.1) mm; rpy = (-144.1, +2.2, +89.4)°
- `world_T_ext_optical` = world_T_base_link · base_link_T_ext_optical  
  → xyz = (-354.1, +14.3, +313.5) mm; rpy = (-141.5, +1.2, -91.6)°
- `world_T_phone_optical` = world_T_base_link · base_link_T_phone_optical  
  → xyz = (-428.7, -28.2, +153.6) mm; rpy = (-103.1, -1.2, -86.0)°

| Camera | Cỡ ảnh | fx, fy | cx, cy | k1, k2 | Xoay luồng | Nguồn |
|---|---|---|---|---|---|---|
| `wrist_optical` | 640×480 | 935.3, 990.3 | 322.6, 229.7 | -0.444, +0.000 | 0° | config/robot/hand_eye.json: K, k1 (fit chung với hand-eye) |
| `ext_optical` | 1280×720 | 932.4, 932.4 | 623.0, 428.7 | +0.060, +0.000 | 0° | config/robot/external_camera.json: K, k1 (fit chung với pose) |
| `phone_optical` | 720×1280 | 957.4, 956.5 | 359.9, 641.7 | +0.139, -0.449 | 90° | config/robot/cameras/phone.json |

## Vùng world, cube nhận bằng mặt khác, vật bất kỳ (2026-10-08)

- **Vùng world** = hợp các vết nhìn của bộ pose quét trên mặt bàn (`active_view.scan_region`), 15 pose:
  vòng gần J2 = 125 và 110 (J3 = J4 = 0; camera cao 18–20 cm, thấy 13–32 cm quanh đế) và vòng xa
  J2 = 120, J3 = 20 (camera cao ~24 cm, nghiêng 51°, thấy 26–57 cm), mỗi vòng J1 = 42, 66, 90, 114, 133°.
  Đo được 2952 cm² (chỉ vòng gần: 844 cm²). Lưu trong world (`region`); cube/vật có tâm ngoài vùng (quá viền 15 mm)
  không được ghi; lưới chỉ vẽ trong vùng. Mọi pose nằm trong vùng hand-eye (J1 40–135°, J2 74–135°, J3 0–29°,
  J4 0–19°). Ở vòng xa tag 20 mm quá nhỏ để đo: vòng xa phục vụ mặt cube và vật khác.
- **Cube không ngửa tag**: `Identifier(FULL)` cho 4 góc mặt trên (màu hoặc hình in) → `multiview.flat_face` thử 8 thứ
  tự góc × 4 tầng với mặt 30 mm nằm phẳng. Đo thật: đúng tầng lệch 1,8–4,6 px, sai tầng 13–21 px, nên tầng tự phân
  biệt được. Ghi vào `faces` của world (không làm mốc định vị cho camera khác; tag đọc được thì tag thắng). Vị trí
  cùng cube giữa `--scan` và `--watch` lệch ≤ 2 mm. **Cần đủ sáng**: ở độ sáng 63/255 không nhận được, ở 154 nhận đủ.
  Tốc độ nhận mặt: 0,7 s/khung bằng `.venv` (GPU), ~4 s bằng python hệ thống (CPU).
- **Vật khác cube** (`build_world.py --scan`, tắt bằng `--no-objects`): nối các khung quét thành MỘT đám mây điểm
  3D trong hệ base (`cube_vision/pointcloud.py`). Cùng một điểm trên vật hiện ở nhiều khung chồng nhau; đặc trưng
  SIFT khớp giữa hai khung cho hai tia, pose camera của từng khung đã biết từ FK + hand-eye nên hai tia cắt nhau ở
  vị trí mét thật, và vật lớn hơn một khung vẫn ghép được. Đo thật (15 khung): ~1250 điểm, hai tia khớp lệch nhau
  0,3–1,5 mm; cả 4 cube hiện đúng chỗ ở độ cao ~30 mm. Sau đó bỏ điểm sát bàn và điểm thuộc cube, gom cụm, khớp hình:
  cung tròn (hoặc nhãn YOLOE là cốc/lon/chai) → hình trụ, mặt gần lấy từ đám điểm, đường kính lấy từ viền vật trong
  ảnh (YOLOE, cần `.venv`); còn lại → hộp bao **phần nhìn thấy**. World lưu `objects`: hình, tâm, đa giác đáy, chiều
  cao, và tối đa 400 điểm 3D + màu của từng vật.
  **Tay tự nhìn quanh vật** (`active_view.object_views`, `build_world.look_at_objects`): sau lượt dựng đầu, với tối đa
  3 vật nhiều điểm nhất, tay đi 8 pose an toàn trong vùng hand-eye có vật nằm giữa khung và tâm camera trải xa nhau
  (≥ 2 cm từng cặp), rồi dựng lại từ tất cả khung. Đo thật 2026-10-08: 15 khung quét → 1148 điểm; thêm 16 khung
  quanh hai vật → 13 574 điểm. Gom cụm trong lưới 3D 1 cm (trên mặt bàn thì cốc và gói khăn bị quai cốc nối liền).
  Cốc: trụ Ø 76 mm, cao 103 mm, tâm (-195, -127) mm, 5641 điểm từ 23 khung; vẽ đè lên 31 khung đều ôm đúng thân và
  miệng cốc. Gói khăn giấy: hộp 81 × 71 mm cao 80 mm, chỉ bao phần có chữ in (phần trơn không có điểm). Trước khi có
  bước nhìn quanh, hai lần quét cho cốc Ø 56 và 70 mm, cao 106 và 108 mm. Chưa đối chiếu bằng thước.
  Giới hạn: chỉ có điểm ở chỗ có hoa văn (mặt trơn một màu thì không); độ sâu nhiễu vài mm vì hai khung chỉ cách
  nhau 2–5 cm; chỉ thấy mặt quay về phía đế, nên vật dạng hộp mới có mặt trước (gói khăn giấy ra hộp 77 × 14 mm);
  bằng python hệ thống (không có YOLOE) vật không có tên và hình trụ hẹp hơn thật.
  Đã thử và bỏ: giao bóng mặt nạ trên mặt bàn (hộp lệch vài cm: không có hai khung cùng thấy đáy vật) và so ảnh nắn
  về mặt bàn (tự phơi sáng làm ô màu phẳng bị báo nhầm).
