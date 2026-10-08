"""Adapter mỏng nối cube_vision.zone_survey (module độc lập) với bảng zone và IK của T8.

Toàn bộ logic nằm ở projects/cube_vision/zone_survey.py; ở đây chỉ cấp `ZoneLayout` từ
t8_motion_worker (BIN_*, zone_poses_for_target) + dofbot_ik, và giữ chữ ký cũ cho t8_ros_tasks.

CLI (cần perception ROS đang chạy và cổng tay máy rảnh):
    python projects/t8_pipeline/zone_survey.py --zones 3 4          # chỉ đo, không thả cube
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for extra in (HERE.parent, HERE.parent / "vision_experiments"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

from cube_vision import zone_survey as core  # noqa: E402
from cube_vision.zone_survey import (  # noqa: E402,F401  (tái xuất cho test/bên gọi cũ)
    DEFAULT_NAMES as ZONE_NAMES_VI, LOOK_J1_RANGE, MAX_EXTRA_VIEWS, MAX_PAD_MOVE_M,
    PAD_NOMINAL_AREA_M2, SCAN_OFFSETS_DEG, ZoneLayout, is_reliable as _reliable)

SURVEY_POSE = core.DEFAULT_POSE
SURVEY_VIEWS = core.DEFAULT_VIEWS


def t8_layout() -> ZoneLayout:
    import dofbot_ik
    import t8_motion_worker as worker
    return ZoneLayout(
        release_xy=lambda zone: [float(v) for v in dofbot_ik.fk(worker.BIN_RELEASE_POSES[int(zone)][:5])[:2]],
        configured_j1=lambda zone: float(worker.BIN_RELEASE_POSES[int(zone)][0]),
        plan=lambda zone, xy: worker.zone_poses_for_target(int(zone), xy),
        ik_j1=lambda xy: dofbot_ik.ik(xy[0], xy[1], 0.06)[0],
        views=SURVEY_VIEWS, look_pose=SURVEY_POSE)


def configured_j1(zone_id):
    return t8_layout().configured_j1(zone_id)


def release_xy(zone_id):
    return t8_layout().release_xy(zone_id)


def look_j1_toward(xy, fallback):
    return core.look_j1_toward(t8_layout(), xy, fallback)


def decide_zone(zone, entries):
    return core.decide_zone(t8_layout(), zone, entries)


def decide_zones(per_zone_entries, zones=(1, 2, 3, 4)):
    return core.decide_zones(t8_layout(), per_zone_entries, zones=zones)


def infer_missing_xy(per_zone_entries):
    """{zone: [entries]} -> {zone: [x, y]} ô suy ra từ đối xứng (chỉ để kiểm tra/in)."""
    measured = {z: tuple(e["center_xy"]) for z, entries in per_zone_entries.items()
                if (e := core.best_entry(entries))}
    return {z: v[0] for z, v in core.infer_missing(t8_layout(), measured).items()}


def run_survey(motion, backend, zones=(1, 2, 3, 4), log=print):
    return core.run_survey(motion, backend, t8_layout(), zones=zones, log=log)


def main():
    import argparse
    import json
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zones", type=int, nargs="+", default=[1, 2, 3, 4], choices=(1, 2, 3, 4))
    args = ap.parse_args()
    import t8_assistant
    from cube_vision.cameras import resolve_arg
    from t8_motion import MotionBridge
    from t8_ros_scene import RosSceneBridge
    # Giống t8_assistant: dùng perception đang chạy, không có thì tự khởi động (camera tay tự nhận);
    # chỉ dừng cái do mình bật. Không có khung camera thật thì dừng trước khi xoay tay.
    config = HERE.parents[1] / "config" / "robot" / "cube_6d_calibrated.yaml"
    # Lần khởi động đầu (nạp DINO/YOLOE) có thể mất hơn 20 s: chờ tới 60 s.
    started = t8_assistant.ensure_cube_perception(resolve_arg("auto", fallback="/dev/video0"),
                                                  timeout=60.0, ros_config=config)
    print(f"[perception] {started['reply']}")
    if not started["ok"]:
        t8_assistant.stop_owned_perception()      # không để perception mồ côi giữ camera/serial
        raise SystemExit(1)
    motion = MotionBridge()
    try:
        result = run_survey(motion, RosSceneBridge(), zones=tuple(args.zones))
    finally:
        motion.close()
        t8_assistant.stop_owned_perception()
    if result.get("error"):
        print(f"[zone] Khảo sát dừng vì lỗi hạ tầng (không phải thiếu ô/ánh sáng): {result['error']}")
    print(json.dumps({k: result[k] for k in ("targets", "measured", "restored", "surveyed_views", "error")},
                     ensure_ascii=False, indent=1))
    raise SystemExit(0 if result["restored"] else 1)


if __name__ == "__main__":
    main()
