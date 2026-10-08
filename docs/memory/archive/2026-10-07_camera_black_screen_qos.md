---
name: project-camera-black-screen-qos
description: Why the T8 approval viewer went black — best-effort QoS dropped 900 KB camera frames; fixed to RELIABLE
metadata:
  type: project
---

Viewer đen / "NO FRESH CAMERA IMAGE" (2026-10-07): `/cap_vision/image_raw` (640x480 bgr8, ~900 KB) nhận bằng best-effort
(`qos_profile_sensor_data`, perception `latest_image_qos`) bị rớt khung khi truyền: ~3-5 fps, khoảng đứt 1-4 s; viewer vẽ khung đen
khi ảnh cũ >2 s. Camera đọc trực tiếp (V4L2 hay GStreamer) ổn định 7,2 fps nên không phải phần cứng. Sửa: reliable KEEP_LAST
(`image_qos()` trong t8_ros_scene_worker, `latest_image_qos()` trong object_perception_node) -> ảnh thô ~15 fps, gap max 0,08 s;
ảnh perception gap max 1,2 s.

**Why:** đo bằng probe node ROS riêng (dấu thời gian tin nhắn đứt cùng chỗ với lúc nhận, độ trễ 0 => mất trong truyền).
**How to apply:** subscriber ảnh lớn dùng RELIABLE; `cube_sort_3d.py` vẫn best-effort (chưa đổi). Camera ngoài /dev/video2 cần ~30 khung chờ phơi sáng.
Liên quan [[project-ik-calibration-audit]].
