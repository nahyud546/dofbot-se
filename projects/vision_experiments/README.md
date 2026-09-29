# vision_experiments
Script vision độc lập (không phải ROS package):
- `apriltag_follow.py`, `face_follow_gpu.py`, `kcf_follow.py` (dời từ root).
- Model: `ai/models/detection/yolov8n-face.pt` — code resolve qua `ROBOT_ARM_ROOT` + fallback path cũ.
- Chạy: `source scripts/setup/setup_env.sh && .venv/bin/python projects/vision_experiments/face_follow_gpu.py`
