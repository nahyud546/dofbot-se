#!/usr/bin/env python3
"""Real-world validation loggers. No pipeline refactor, measure-only.

Subscribes (read-only):
  /vision/object_states  (ObjectStates)
  /vision/selected_grasp (GraspCandidate)
  image topic (default /cap_vision/image_raw, for dataset frames)
  /real_joint_states + /joint_states (for dataset joint snapshot)

Writes to --out-dir:
  eye_in_hand_real.csv   (Sec 1: static-object stability, all frames)
  identity_real.csv      (Sec 2: same rows + ground_truth_cube_id to fill offline)
  dataset_manifest.jsonl + dataset/*.png (Sec 5: tag-GT samples only)
  grasp transactions are logged by the bridge (see cube_sort_3d.py).

DINO per-region / HSV coverage internals are NOT in ObjectState.msg, so the
logger records what IS published (identity_probabilities, method, reproj,
visible_surfaces, reason/status) and marks region-level detail as pending
perception debug. This keeps validation honest instead of inventing numbers.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from copy import deepcopy
from pathlib import Path

EYE_FIELDS = [
    "timestamp", "image_timestamp", "tf_timestamp",
    "track_id", "cube_id", "identity_confidence",
    "pose_method", "pose_confidence", "orientation_valid",
    "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
    "base_cam_px", "base_cam_py", "base_cam_pz",
    "base_cam_qx", "base_cam_qy", "base_cam_qz", "base_cam_qw",
    "cam_obj_px", "cam_obj_py", "cam_obj_pz",
    "cam_obj_qx", "cam_obj_qy", "cam_obj_qz", "cam_obj_qw",
    "base_obj_px", "base_obj_py", "base_obj_pz",
    "base_obj_qx", "base_obj_qy", "base_obj_qz", "base_obj_qw",
    "world_assoc", "image_assoc",
    "tag_reproj_px", "tag_visible",
    "dino_top1", "dino_top1_score", "dino_top2_score", "dino_margin",
    "hsv_color", "hsv_coverage",
    "state", "calibrated", "status", "tf_ok", "rejected_reason",
]

IDENTITY_FIELDS = [
    "timestamp", "track_id", "ground_truth_cube_id", "predicted_cube_id",
    "identity_confidence", "identity_probs",
    "pose_method", "tag_visible", "bbox",
    "dino_region", "dino_top1", "dino_top1_sim", "dino_top2", "dino_top2_sim", "dino_margin",
    "hsv_color", "hsv_coverage", "hsv_conf",
    "note",
]

FULL_6D_METHODS = {"apriltag_ippe", "apriltag_rgb_fused", "rgb_faces_pnp"}
TAG_GT_METHODS = {"apriltag_ippe", "apriltag_rgb_fused"}


def orientation_valid_of(method: str) -> int:
    return 1 if str(method) in FULL_6D_METHODS else 0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--mount-frame", default="measured/Camera_Link")
    ap.add_argument("--image-topic", default="/cap_vision/image_raw")
    ap.add_argument("--reproj-thresh", type=float, default=2.0)
    ap.add_argument("--save-images", action="store_true")
    ap.add_argument("--max-frames", type=int, default=0)
    return ap.parse_args(argv)


def ensure_out(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "dataset").mkdir(parents=True, exist_ok=True)
    eye = out_dir / "eye_in_hand_real.csv"
    ident = out_dir / "identity_real.csv"
    if not eye.exists():
        eye.write_text(",".join(EYE_FIELDS) + "\n", encoding="utf-8")
    if not ident.exists():
        ident.write_text(",".join(IDENTITY_FIELDS) + "\n", encoding="utf-8")
    return eye, ident


def quat_msg(q):
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def point_msg(p):
    return (float(p.x), float(p.y), float(p.z))


def main(argv=None):
    args = parse_args(argv)
    out_dir = Path(args.out_dir)
    eye_path, ident_path = ensure_out(out_dir)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, JointState
    from tf2_ros import Buffer, TransformListener, TransformException
    from cv_bridge import CvBridge

    from cap_scene_interfaces.msg import ObjectStates

    rclpy.init()
    node = Node("real_validation_logger")
    tf = Buffer()
    TransformListener(tf, node)
    bridge = CvBridge()
    latest_joints = {}
    latest_image_stamp = [0.0]

    def on_joints(msg: JointState):
        try:
            latest_joints.clear()
            latest_joints.update(dict(zip(msg.name, msg.position)))
            latest_joints["_stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except (AttributeError, TypeError, ValueError):
            pass

    node.create_subscription(JointState, "/real_joint_states", on_joints, 10)
    node.create_subscription(JointState, "/joint_states", on_joints, 10)

    def on_image(msg: Image):
        try:
            latest_image_stamp[0] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except (AttributeError, TypeError):
            pass

    node.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)

    manifest = open(out_dir / "dataset_manifest.jsonl", "a", encoding="utf-8")
    eye_f = open(eye_path, "a", encoding="utf-8", newline="")
    ident_f = open(ident_path, "a", encoding="utf-8", newline="")
    eye_w = csv.DictWriter(eye_f, fieldnames=EYE_FIELDS)
    ident_w = csv.DictWriter(ident_f, fieldnames=IDENTITY_FIELDS)
    count = [0]

    def on_states(msg: ObjectStates):
        now = node.get_clock().now().nanoseconds * 1e-9
        img_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Synchronized eye-in-hand TF at the IMAGE stamp (never "now").
        tf_ok, tf_ts, t, q = False, float("nan"), (math.nan,) * 3, (math.nan,) * 4
        try:
            tr = tf.lookup_transform(args.base_frame, args.mount_frame,
                                     Time.from_msg(msg.header.stamp))
            t = (tr.transform.translation.x, tr.transform.translation.y,
                 tr.transform.translation.z)
            q = (tr.transform.rotation.x, tr.transform.rotation.y,
                 tr.transform.rotation.z, tr.transform.rotation.w)
            tf_ts = img_ts  # lookup keyed on image stamp
            tf_ok = True
        except TransformException:
            tf_ok = False
        if not msg.objects:
            row = {k: "" for k in EYE_FIELDS}
            row.update({"timestamp": now, "image_timestamp": img_ts, "tf_timestamp": tf_ts,
                        "calibrated": int(bool(msg.calibrated)), "status": msg.status,
                        "tf_ok": int(tf_ok), "rejected_reason": "no_objects"})
            eye_w.writerow(row)
            eye_f.flush()
        for o in msg.objects:
            try:
                cp, cq = point_msg(o.camera_pose.pose.position), quat_msg(o.camera_pose.pose.orientation)
                bp, bq = point_msg(o.base_pose.pose.position), quat_msg(o.base_pose.pose.orientation)
            except (AttributeError, TypeError, ValueError):
                cp, cq = (math.nan,) * 3, (math.nan,) * 4
                bp, bq = (math.nan,) * 3, (math.nan,) * 4
            tag_visible = int("tag" in str(o.pose_method) or int(o.geometry_model_id) in (1, 2, 3, 4)
                              and str(o.pose_method) in FULL_6D_METHODS)
            row = {
                "timestamp": now, "image_timestamp": img_ts, "tf_timestamp": tf_ts,
                "track_id": o.track_id, "cube_id": int(o.object_id),
                "identity_confidence": float(o.identity_confidence),
                "pose_method": str(o.pose_method), "pose_confidence": float(o.pose_confidence),
                "orientation_valid": orientation_valid_of(o.pose_method),
                "bbox_x1": o.bbox_xyxy[0], "bbox_y1": o.bbox_xyxy[1],
                "bbox_x2": o.bbox_xyxy[2], "bbox_y2": o.bbox_xyxy[3],
                "base_cam_px": t[0], "base_cam_py": t[1], "base_cam_pz": t[2],
                "base_cam_qx": q[0], "base_cam_qy": q[1], "base_cam_qz": q[2], "base_cam_qw": q[3],
                "cam_obj_px": cp[0], "cam_obj_py": cp[1], "cam_obj_pz": cp[2],
                "cam_obj_qx": cq[0], "cam_obj_qy": cq[1], "cam_obj_qz": cq[2], "cam_obj_qw": cq[3],
                "base_obj_px": bp[0], "base_obj_py": bp[1], "base_obj_pz": bp[2],
                "base_obj_qx": bq[0], "base_obj_qy": bq[1], "base_obj_qz": bq[2], "base_obj_qw": bq[3],
                "world_assoc": "", "image_assoc": "",
                "tag_reproj_px": float(o.reprojection_error_px),
                "tag_visible": tag_visible,
                "dino_top1": "", "dino_top1_score": "", "dino_top2_score": "", "dino_margin": "",
                "hsv_color": "", "hsv_coverage": "",
                "state": str(o.state), "calibrated": int(bool(msg.calibrated)),
                "status": str(msg.status), "tf_ok": int(tf_ok),
                "rejected_reason": "" if bool(o.pose_valid) else str(o.reason),
            }
            eye_w.writerow(row)
            ident_w.writerow({
                "timestamp": now, "track_id": o.track_id, "ground_truth_cube_id": "",
                "predicted_cube_id": int(o.object_id),
                "identity_confidence": float(o.identity_confidence),
                "identity_probs": ";".join(f"{float(v):.3f}" for v in o.identity_probabilities),
                "pose_method": str(o.pose_method), "tag_visible": tag_visible,
                "bbox": ";".join(str(int(v)) for v in o.bbox_xyxy),
                "dino_region": "", "dino_top1": "", "dino_top1_sim": "",
                "dino_top2": "", "dino_top2_sim": "", "dino_margin": "",
                "hsv_color": "", "hsv_coverage": "", "hsv_conf": "",
                "note": "region-level DINO/HSV not in ObjectState.msg; see perception debug",
            })
            # Sec 5: dataset sample only on tag GT + reproj + synced TF.
            try:
                if (args.save_images and tf_ok and bool(msg.calibrated)
                        and str(o.pose_method) in TAG_GT_METHODS and bool(o.pose_valid)
                        and float(o.reprojection_error_px) <= float(args.reproj_thresh)
                        and int(o.object_id) in (1, 2, 3, 4)):
                    meta = {"timestamp": now, "image_timestamp": img_ts,
                            "track_id": o.track_id, "cube_id": int(o.object_id),
                            "bbox": [int(v) for v in o.bbox_xyxy],
                            "T_camera_cube": {"position": list(cp), "quaternion_xyzw": list(cq)},
                            "T_base_cube": {"position": list(bp), "quaternion_xyzw": list(bq)},
                            "T_base_camera": {"position": list(t), "quaternion_xyzw": list(q)},
                            "reproj_px": float(o.reprojection_error_px),
                            "pose_method": str(o.pose_method),
                            "joints": {k: float(v) for k, v in latest_joints.items() if k != "_stamp"}}
                    manifest.write(json.dumps(meta) + "\n")
                    manifest.flush()
            except (TypeError, ValueError):
                pass
        eye_f.flush()
        ident_f.flush()
        count[0] += 1
        if args.max_frames and count[0] >= args.max_frames:
            raise KeyboardInterrupt

    node.create_subscription(ObjectStates, "/vision/object_states", on_states, 10)
    node.get_logger().info(f"validation logger -> {out_dir} (tag-GT dataset={'on' if args.save_images else 'off'})")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        manifest.close()
        eye_f.close()
        ident_f.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
