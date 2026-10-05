#!/usr/bin/env python3
"""
Connects ROS 2 3D perception with the legacy T8 motion worker.
Subscribes to /vision/object_states and /vision/object_annotated.
Press Space to sort a confirmed cube.
"""
import os
import sys
import argparse
import json
import time
import math
import subprocess
import unicodedata
from pathlib import Path
import threading

# cv_bridge (ROS Humble) link voi NumPy 1.x; .venv dang dung NumPy 2.x
# -> tu re-exec bang system python de tranh crash _ARRAY_API.
if int(__import__("numpy").__version__.split(".")[0]) >= 2:
    _syspy = "/usr/bin/python3"
    print(f"[FIX] venv numpy 2.x khong tuong thich cv_bridge; re-exec bang {_syspy}...",
          flush=True)
    os.execv(_syspy, [_syspy, os.path.abspath(__file__), *sys.argv[1:]])

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Tu import kieu message (phai chay sau khi source cap_scene_interfaces)
from cap_scene_interfaces.msg import GraspCandidate as GraspCandidateMsg
from cap_scene_interfaces.msg import ObjectStates

ROOT = Path("/home/jloy/Desktop/robot-arm")
WORKER = ROOT / "projects/t8_pipeline/t8_motion_worker.py"
ROS_SETUP = ROOT / "workspaces/dofbot_ws/install/setup.bash"
GRASP_LOG = ROOT / "projects/vision_experiments/validation/grasp_execution.jsonl"


def to_ascii(text):
    """cv2.putText (Hershey) chi hien thi ASCII -> strip dau tieng Viet."""
    if text is None:
        return ""
    text = unicodedata.normalize("NFKD", str(text))
    # đ/Đ khong tach bang NFKD -> map tay.
    text = text.replace("đ", "d").replace("Đ", "D")
    return text.encode("ascii", "ignore").decode("ascii")


def log_transaction(tx):
    """Append one end-to-end grasp transaction (Sec 3). Additive, no behavior change."""
    try:
        GRASP_LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(GRASP_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(tx) + "\n")
    except OSError:
        pass

# ---- Bridge acceptance policy ----
# Perception owns classifier/PnP confidence gates and commits object_id/method.
# The bridge owns only checks that are specific to a moving robot.
FULL_6D_METHODS = {"apriltag_ippe", "apriltag_rgb_fused", "rgb_faces_pnp"}
TOP_GRASP_METHODS = {"rgb_single_face_cube"}
CONFIRM_FRAMES = 2
CONFIRM_MAX_AGE_S = 2.0
CONFIRM_MAX_POSITION_M = 0.005
CONFIRM_MAX_ROTATION_DEG = 10.0
CONFIRM_MAX_TCP_PX = 3.0
MAX_LIVE_AGE_S = 0.75
# Current CPU perception can finish more than 0.75 s after image capture.
# Keep receipt freshness strict, but allow the completed image observation.
MAX_OBSERVATION_AGE_S = 2.0
MAX_ANNOTATED_DISPLAY_AGE_S = 2.0
# Same mapping used by t8_motion_worker.cube_pick_target at the fixed READY_POSE.
PICK_K = np.array([[902.0, 0.0, 320.0],
                   [0.0, 875.4, 240.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
PICK_D = np.zeros(5, dtype=np.float64)
PICK_TARGET_Z_M = 0.047
DEFAULT_PICK_X_OFFSET_MM = 15.0  # KDL X dương kéo TCP về phía chân robot.
# rgb_geometry là position-only fallback: track được, KHÔNG grasp 6D.
CUBE_EDGE_M = 0.030
GRIPPER_MAX_WIDTH_M = 0.05


def quat_to_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        raise ValueError("quaternion rỗng")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], float)


def pose_matrix(pose_stamped):
    p, q = pose_stamped.pose.position, pose_stamped.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def camera_T_cube_from_obj(obj):
    return pose_matrix(obj.camera_pose)


def pose_ok_for_top_grasp(obj):
    """Accept semantic full-6D or a one-face metric top-grasp pose.

    The latter remains semantically ambiguous by 90 degrees, but that is a
    physical symmetry of a square cube and a parallel-jaw top grasp.
    """
    if not bool(obj.camera_pose_valid):
        return False, str(getattr(obj, "reason", "")) or "camera pose unavailable"
    if not bool(getattr(obj, "top_grasp_ready", False)):
        return False, str(getattr(obj, "reason", "")) or "top grasp not ready"
    method = str(obj.pose_method)
    if method not in FULL_6D_METHODS | TOP_GRASP_METHODS:
        return False, f"position-only ({method}); need top-face geometry"
    try:
        T = camera_T_cube_from_obj(obj)
    except (AttributeError, TypeError, ValueError):
        return False, "invalid camera transform"
    if not np.isfinite(T).all():
        return False, "invalid camera transform"
    return True, ""


def snapshot_object(obj, now_s):
    """Snapshot camera-frame pose; robot is held at cube_sort_stage1 READY_POSE."""
    candidates = grasp_candidates_for_obj(obj)
    if not candidates:
        return None
    return {
        "object_id": int(obj.object_id),
        "geometry_model_id": str(obj.geometry_model_id),
        "pose_method": str(obj.pose_method),
        "T": camera_T_cube_from_obj(obj).copy(),
        "tcp_px": np.asarray(candidates[0]["center_px"], float),
        "yaw_deg": math.degrees(candidates[0]["preferred_yaw_rad"]),
        "time_s": float(now_s),
        "count": 1,
    }


def rotation_delta_deg(first_T, second_T):
    R = first_T[:3, :3].T @ second_T[:3, :3]
    cosine = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def cube_symmetric_rotation_delta_deg(first_T, second_T):
    """Pose delta after quotienting the cube's 90-degree yaw symmetry."""
    delta = first_T[:3, :3].T @ second_T[:3, :3]
    errors = []
    for turns in range(4):
        theta = math.radians(90.0 * turns)
        rz = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                       [math.sin(theta), math.cos(theta), 0.0],
                       [0.0, 0.0, 1.0]])
        cosine = float(np.clip((np.trace(delta @ rz) - 1.0) / 2.0, -1.0, 1.0))
        errors.append(math.degrees(math.acos(cosine)))
    return min(errors)


def update_confirmation(previous, obj, now_s):
    """Confirm a camera-frame pose across adjacent fresh messages."""
    ok, reason = pose_ok_for_top_grasp(obj)
    if not ok or int(obj.object_id) not in (1, 2, 3, 4):
        return None
    current = snapshot_object(obj, now_s)
    if current is None:
        return None
    if previous is None:
        return current
    same_object = (
        previous["object_id"] == current["object_id"]
        and previous["geometry_model_id"] == current["geometry_model_id"]
        and previous["pose_method"] == current["pose_method"]
    )
    age = current["time_s"] - previous["time_s"]
    tcp_shift = float(np.linalg.norm(current["tcp_px"] - previous["tcp_px"]))
    yaw_delta = abs((current["yaw_deg"] - previous["yaw_deg"] + 45.0) % 90.0 - 45.0)
    if same_object and 0.0 < age <= CONFIRM_MAX_AGE_S and \
            tcp_shift <= CONFIRM_MAX_TCP_PX and yaw_delta <= CONFIRM_MAX_ROTATION_DEG:
        current["count"] = min(int(previous["count"]) + 1, CONFIRM_FRAMES)
    return current


def is_graspable(obj, confirmation=None, now_s=None):
    """Require a committed ID and two consistent top-grasp-capable frames."""
    if int(obj.object_id) not in (1, 2, 3, 4):
        return False, "object_id not committed"
    ok, reason = pose_ok_for_top_grasp(obj)
    if not ok:
        return False, reason
    if confirmation is not None and (float(now_s if now_s is not None else time.monotonic()) -
                                     float(confirmation.get("time_s", 0.0)) > MAX_LIVE_AGE_S):
        return False, "camera frame too old"
    if confirmation is None or (
            int(confirmation.get("object_id", 0)) != int(obj.object_id) or
            str(confirmation.get("geometry_model_id", "")) != str(obj.geometry_model_id) or
            str(confirmation.get("pose_method", "")) != str(obj.pose_method) or
            int(confirmation.get("count", 0)) < CONFIRM_FRAMES):
        return False, "waiting second consistent top-grasp frame"
    return True, ""


def grasp_candidates_for_obj(obj, pick_x_offset_mm=DEFAULT_PICK_X_OFFSET_MM):
    """Map the visible face through cube_sort_stage1's fixed-pose calibration."""
    if not math.isfinite(float(pick_x_offset_mm)) or abs(float(pick_x_offset_mm)) > 20.0:
        return []
    T = camera_T_cube_from_obj(obj)
    half = CUBE_EDGE_M / 2.0
    faces = []
    axes = np.eye(3)
    for axis_index in range(3):
        transverse = [i for i in range(3) if i != axis_index]
        for sign in (-1.0, 1.0):
            normal = T[:3, :3] @ (sign * axes[axis_index])
            facing = -float(normal[2])
            if facing < 0.65:
                continue
            local_center = sign * half * axes[axis_index]
            center = T[:3, 3] + T[:3, :3] @ local_center
            corners = []
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                point = local_center.copy()
                point[transverse[0]] += a * half
                point[transverse[1]] += b * half
                corners.append(point)
            corners_camera = (T[:3, :3] @ np.asarray(corners).T).T + T[:3, 3]
            name = ("+" if sign > 0 else "-") + "XYZ"[axis_index]
            faces.append((facing, name, center, corners_camera))
    if not faces:
        return []
    facing, name, center, corners = max(faces, key=lambda face: face[0])
    if not np.isfinite(corners).all() or np.any(corners[:, 2] <= 0):
        return []
    quad = cv2.projectPoints(corners, np.zeros(3), np.zeros(3), PICK_K, PICK_D)[0].reshape(4, 2)
    center_px = cv2.projectPoints(center.reshape(1, 3), np.zeros(3), np.zeros(3),
                                  PICK_K, PICK_D)[0].reshape(2)
    if (np.any(center_px < [0, 0]) or np.any(center_px >= [640, 480]) or
            np.any(quad < [0, 0]) or np.any(quad >= [640, 480])):
        return []

    # Keep yaw and XY equations identical to t8_motion_worker.cube_pick_target.
    edges = np.roll(quad, -1, axis=0) - quad
    dx, dy = edges[int(np.argmax(np.sum(edges * edges, axis=1)))]
    if dx * dx + dy * dy < 100.0:
        return []
    angle = math.degrees(math.atan2(float(dy), float(dx))) % 90.0
    yaw_deg = -angle if angle <= 45.0 else 90.0 - angle
    yaw_deg = max(-40.0, min(40.0, yaw_deg))
    px, py = map(float, center_px)
    radius2 = ((px - 320.0) / 320.0) ** 2 + ((py - 240.0) / 240.0) ** 2
    scale = 1.0 + 0.06 * radius2
    ux, uy = 320.0 + (px - 320.0) * scale, 240.0 + (py - 240.0) * scale
    x = (-((480.0 - uy) * (0.8 / 3000.0) + 0.15)
         + float(pick_x_offset_mm) / 1000.0)
    y = (ux - 320.0) / 4000.0
    if not (-0.30 <= x <= -0.10 and -0.12 <= y <= 0.12):
        return []
    return [{
        "surface_id": name,
        "tcp_position_base": [round(float(x), 5), round(float(y), 5), PICK_TARGET_Z_M],
        "surface_normal_base": [0.0, 0.0, 1.0],
        "approach_vector_base": [0.0, 0.0, -1.0],
        "preferred_yaw_rad": math.radians(yaw_deg),
        "gripper_width_m": CUBE_EDGE_M + 0.003,
        "quality_score": facing,
        "center_px": [float(px), float(py)],
    }]

def run_motion(command, **params):
    # Worker can import rclpy (IK) -> phai chay system python (numpy 1.x),
    # .venv numpy 2.x se crash giong bridge.
    script = f"source /opt/ros/humble/setup.bash && source {ROS_SETUP} && exec /usr/bin/python3 {WORKER}"
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/cube_sort_ros_logs")
    try:
        proc = subprocess.run(["bash", "-c", script], input=json.dumps(
            {"command": command, **params}), text=True, capture_output=True,
            cwd=ROOT, env=env, timeout=75, check=False)
    except subprocess.TimeoutExpired:
        return {"ok": False, "reply": "Het thoi gian cho chuyen dong; kiem tra tay truoc khi tiep tuc."}
    for line in reversed(proc.stdout.splitlines()):
        if line.startswith("T8_RESULT:"):
            return json.loads(line[len("T8_RESULT:"):])
    return {"ok": False, "reply": (proc.stderr or proc.stdout).strip()[-300:]}


def visible_camera_frame(node, now_s):
    """Prefer live annotated video, then live raw video, then a wait screen."""
    if node.latest_image is not None and now_s - node.latest_image_at <= MAX_ANNOTATED_DISPLAY_AGE_S:
        return node.latest_image, "annotated"
    if node.latest_raw_image is not None and now_s - node.latest_raw_at <= MAX_LIVE_AGE_S:
        return node.latest_raw_image, "raw"
    return np.zeros((480, 640, 3), np.uint8), "waiting"

class CubeSort3D(Node):
    def __init__(self):
        super().__init__("cube_sort_3d")
        self.bridge = CvBridge()
        self.latest_states = None
        self.latest_image = None
        self.latest_image_at = 0.0
        self.latest_raw_image = None
        self.latest_raw_at = 0.0
        self.confirmations = {}

        self.create_subscription(ObjectStates, "/vision/object_states", self.on_states, 10)
        self.create_subscription(Image, "/vision/object_annotated", self.on_image, 10)
        self.create_subscription(Image, "/cap_vision/image_raw", self.on_raw_image,
                                 qos_profile_sensor_data)
        self.grasp_pub = self.create_publisher(GraspCandidateMsg, "/vision/selected_grasp", 10)

        self.motion_thread = None
        # Chuoi hien thi qua cv2.putText PHẢI là ASCII (Hershey khong ho tro UTF-8).
        self.motion_status = "Ready. Nhan SPACE de gap!"
        # Case H: target đang execute không bị overwrite bởi frame tiếp theo.
        self.locked_target = None

    def on_states(self, msg):
        now_s = time.monotonic()
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now_s = self.get_clock().now().nanoseconds * 1e-9
        if not 0 <= ros_now_s - stamp_s <= MAX_OBSERVATION_AGE_S:
            self.latest_states = None
            self.confirmations.clear()
            return
        active_tracks = set()
        for obj in msg.objects:
            track_id = str(obj.track_id)
            active_tracks.add(track_id)
            sample = update_confirmation(self.confirmations.get(track_id), obj, now_s)
            if sample is None:
                self.confirmations.pop(track_id, None)
            else:
                self.confirmations[track_id] = sample
        # Retain a short occlusion history for reacquisition, but never use it
        # to grasp: only objects in the latest fresh message become candidates.
        self.confirmations = {
            track_id: (sample if track_id in active_tracks else {**sample, "count": 0})
            for track_id, sample in self.confirmations.items()
            if track_id in active_tracks or now_s - sample["time_s"] <= MAX_LIVE_AGE_S
        }
        # Publish a state snapshot only after its confirmation map is updated.
        self.latest_states = msg

    def on_image(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"annotated image decode failed: {exc}")

    def on_raw_image(self, msg):
        try:
            self.latest_raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_raw_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"raw image decode failed: {exc}")

    def publish_selected_grasp(self, obj, tcp):
        msg = GraspCandidateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.track_id = str(obj.track_id)
        msg.object_id = int(obj.object_id)
        msg.surface_id = str(tcp["surface_id"])
        msg.position_base.x, msg.position_base.y, msg.position_base.z = tcp["tcp_position_base"]
        msg.surface_normal_base.x, msg.surface_normal_base.y, msg.surface_normal_base.z = tcp["surface_normal_base"]
        msg.approach_vector_base.x, msg.approach_vector_base.y, msg.approach_vector_base.z = tcp["approach_vector_base"]
        msg.preferred_yaw_rad = float(tcp["preferred_yaw_rad"])
        msg.gripper_width_m = float(tcp["gripper_width_m"])
        msg.quality_score = float(tcp["quality_score"])
        msg.collision_score = 1.0
        msg.ik_feasible = False  # worker/MoveIt xác minh IK sau; bridge chỉ đề xuất hình học
        mode = "full6d" if str(obj.pose_method) in FULL_6D_METHODS else "top-grasp"
        msg.reason = f"stage1 fixed-pose map; {mode} {obj.pose_method}"
        self.grasp_pub.publish(msg)

    def start_sort(self, obj, tcp):
        """Lock selected TCP. Worker KHÔNG tự suy grasp từ XYZ/quaternion."""
        locked = {
            "track_id": str(obj.track_id),
            "cube_id": int(obj.object_id),
            "tcp": tcp,
            "pose_method": str(obj.pose_method),
            "pose_confidence": float(obj.pose_confidence),
            "identity_confidence": float(obj.identity_confidence),
        }
        self.locked_target = locked
        self.publish_selected_grasp(obj, tcp)

        def task(target=locked):
            t0 = time.time()
            # Pose remains in camera frame; commanded TCP uses the validated
            # stage1 fixed-pose image-to-KDL calibration.
            T_snapshot = camera_T_cube_from_obj(obj).tolist()
            self.motion_status = (
                f"LOCK {target['track_id']} cube {target['cube_id']} "
                f"TCP({target['tcp']['tcp_position_base'][0]:.3f}, "
                f"{target['tcp']['tcp_position_base'][1]:.3f}, "
                f"{target['tcp']['tcp_position_base'][2]:.3f}) "
                f"{target['tcp']['surface_id']} [{target['pose_method']}]...")
            print(
                f"[SPACE] LOCK {target['track_id']} ID={target['cube_id']} "
                f"top={target['tcp']['surface_id']} yaw="
                f"{math.degrees(target['tcp']['preferred_yaw_rad']):.1f}deg "
                f"TCP={target['tcp']['tcp_position_base']} -> checking IK",
                flush=True)
            result = run_motion(
                "sort_cube_3d", cube_id=target["cube_id"],
                track_id=target["track_id"],
                pose_method=target["pose_method"],
                tcp_position_base=target["tcp"]["tcp_position_base"],
                surface_id=target["tcp"]["surface_id"],
                surface_normal_base=target["tcp"]["surface_normal_base"],
                approach_vector_base=target["tcp"]["approach_vector_base"],
                preferred_yaw_rad=target["tcp"]["preferred_yaw_rad"],
                gripper_width_m=target["tcp"]["gripper_width_m"],
                quality_score=target["tcp"]["quality_score"])
            ok = result.get("ok", False)
            if ok:
                print(f"[PASS] IK + motion: {result.get('reply', '')}", flush=True)
            elif result.get("code") == "ik_no_solution":
                print(f"[IK FAIL] {result.get('reply', '')}; arm did not move", flush=True)
            else:
                print(f"[MOTION FAIL] {result.get('reply', '')}", flush=True)
                # Worker may have persisted phase=moving after a partial
                # trajectory. Ask its guarded recovery path to return to the
                # observation pose; that path refuses while the gripper is shut.
                recovery = run_motion("prepare")
                if recovery.get("ok"):
                    print(f"[RECOVERED] {recovery.get('reply', '')}", flush=True)
                else:
                    print(f"[RECOVERY REQUIRED] {recovery.get('reply', '')}; khong gui lenh gap tiep", flush=True)
            log_transaction({
                "timestamp": t0, "track_id": target["track_id"], "cube_id": target["cube_id"],
                "T_camera_object": T_snapshot, "pose_method": target["pose_method"],
                "pose_confidence": target["pose_confidence"],
                "identity_confidence": target["identity_confidence"],
                "orientation_valid": target["pose_method"] in FULL_6D_METHODS,
                "top_grasp_valid": target["pose_method"] in TOP_GRASP_METHODS,
                "surface_id": target["tcp"]["surface_id"],
                "surface_normal": target["tcp"]["surface_normal_base"],
                "approach_vector": target["tcp"]["approach_vector_base"],
                "tcp_position": target["tcp"]["tcp_position_base"],
                "tcp_yaw": target["tcp"]["preferred_yaw_rad"],
                "gripper_width": target["tcp"]["gripper_width_m"],
                "pick_x_offset_mm": target["tcp"]["pick_x_offset_mm"],
                "quality": target["tcp"]["quality_score"],
                "DETECTED": True, "IDENTIFIED": True, "POSE_VALID": True,
                "GRASP_SELECTED": True, "IK_OK": bool(ok),
                "APPROACH_OK": bool(ok), "GRASP_SUCCESS": bool(ok),
                "BIN_DROP_SUCCESS": bool(ok), "ok": bool(ok),
                "reached_stage": "BIN_DROP_SUCCESS" if ok else "GRASP_SELECTED",
                "failure_stage": "" if ok else "motion_or_ik",
                "worker_reply": str(result.get("reply", "")),
            })
            self.motion_status = to_ascii(f"{'OK' if ok else 'FAIL'}: {result.get('reply', '')}")
            time.sleep(2)
            self.motion_status = "Ready. Nhan SPACE de gap!"
            self.motion_thread = None
            self.locked_target = None

        self.motion_thread = threading.Thread(target=task, daemon=True)
        self.motion_thread.start()

def main(args=None):
    parser = argparse.ArgumentParser(description="Cube Sort 3D Bridge (ROS2 perception + T8 worker)")
    parser.add_argument("--skip-prepare", action="store_true",
                        help="bo qua pose-start, khong dua tay ve pose quan sat")
    parser.add_argument("--dry-run", action="store_true",
                        help="khong gui bat ky lenh motor nao, ke ca khi nhan Space")
    parser.add_argument("--pick-x-offset-mm", type=float,
                        default=DEFAULT_PICK_X_OFFSET_MM,
                        help="bù X về phía chân robot (mặc định +15 mm, giới hạn ±20 mm)")
    parsed, _ = parser.parse_known_args()
    if not math.isfinite(parsed.pick_x_offset_mm) or abs(parsed.pick_x_offset_mm) > 20.0:
        parser.error("--pick-x-offset-mm phải nằm trong [-20, 20]")
    window_name = "Cube Sort 3D Bridge"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        print(f"[GUI ERROR] Khong mo duoc cua so camera; kiem tra DISPLAY/X11: {exc}",
              flush=True)
        return 2
    startup = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(startup, "Connecting to camera...", (20, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
    cv2.imshow(window_name, startup)
    cv2.waitKey(1)
    motion_enabled = not parsed.dry_run
    prepare_error = ""
    if parsed.dry_run:
        print("[DRY] bo qua pose-start va tat motion khi nhan Space")
    elif not parsed.skip_prepare:
        # Giong cube_sort_stage1.py: tu ve pose quan sat truoc khi scan.
        print("[POSE] dua tay ve pose quan sat truoc khi scan...")
        result = run_motion("prepare")
        if not result.get("ok"):
            prepare_error = str(result.get("reply", "unknown error"))
            motion_enabled = False
            print(f"[OBSERVE ONLY] pose-start that bai: {prepare_error}")
            print("[HINT] Kiem tra kep, /dev/ttyUSB0 va worker. Cua so camera van mo; Space bi khoa.")
        else:
            print(f"[OK] pose-start: {result.get('reply')}")
    rclpy.init(args=args)
    node = CubeSort3D()
    if prepare_error:
        node.motion_status = "OBSERVE ONLY: prepare failed; Space disabled"
    elif parsed.dry_run:
        node.motion_status = "DRY RUN: camera only; Space will not move arm"

    # Run node spin in background; nuot ExternalShutdownException khi ^C
    # de khong in traceback Thread-1.
    def _spin():
        try:
            rclpy.spin(node)
        except Exception as exc:
            node.motion_status = f"ROS subscription stopped: {exc}"
            print(f"[ROS ERROR] {exc}", flush=True)
    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    print("Cube Sort 3D Bridge Started! Nhan Space de gap, q de thoat.")

    try:
        while True:
            now_s = time.monotonic()
            visible_image, image_kind = visible_camera_frame(node, now_s)
            if visible_image is not None:
                display = visible_image.copy()

                # Show status (ASCII only cho cv2.Hershey)
                cv2.putText(display, to_ascii(node.motion_status), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2, cv2.LINE_AA)

                # ID/PnP is validated by perception. Full 6D camera pose gives
                # the visible face; the READY_POSE map supplies base XY.
                candidates = []
                rejected = 0
                reject_lines = []
                if image_kind == "waiting":
                    reject_lines.append("Waiting for /cap_vision/image_raw; start cube_6d_camera.launch.py")
                elif image_kind == "raw":
                    reject_lines.append("Raw camera only; waiting for /vision/object_annotated")
                elif not motion_enabled and not parsed.dry_run:
                    reject_lines.append("OBSERVE ONLY: motion disabled")
                elif display.shape[:2] != (480, 640):
                    reject_lines.append("fixed-pose map requires 640x480 image")
                elif node.latest_states and node.latest_states.objects:
                    for obj in node.latest_states.objects:
                        if node.locked_target is not None:
                            # Case H: đang grasp thì không overwrite target.
                            if obj.track_id != node.locked_target["track_id"]:
                                continue
                        confirmation = node.confirmations.get(str(obj.track_id))
                        ok, reason = is_graspable(obj, confirmation)
                        if not ok:
                            rejected += 1
                            reject_lines.append(f"ID {obj.object_id} ({obj.track_id}): {reason}")
                            continue
                        tcps = grasp_candidates_for_obj(obj, parsed.pick_x_offset_mm)
                        if not tcps:
                            rejected += 1
                            reject_lines.append(f"ID {obj.object_id} ({obj.track_id}): no top TCP")
                            continue
                        candidates.append((obj, tcps[0]))

                lock_txt = f" LOCK:{node.locked_target['track_id']}" if node.locked_target else ""
                info = f"Ready {len(candidates)} | waiting/reject {rejected}{lock_txt}"
                cv2.putText(display, to_ascii(info), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
                # Show the concrete reason while a pose stabilizes or lacks 6D.
                for index, line in enumerate(reject_lines[:4]):
                    cv2.putText(display, to_ascii(line), (10, 90 + index * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 165, 255), 1, cv2.LINE_AA)

                cv2.imshow(window_name, display)
                key = cv2.waitKey(30) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord(' ') and node.motion_thread is None:
                    if not motion_enabled and not parsed.dry_run:
                        node.motion_status = "OBSERVE ONLY: prepare failed; Space disabled"
                        continue
                    if candidates:
                        # All candidates have passed the same safety gate; use
                        # only physical top-face quality then deterministic ID.
                        best_obj, best_tcp = sorted(
                            candidates,
                            key=lambda pair: (pair[1]["quality_score"], -int(pair[0].object_id)),
                            reverse=True)[0]
                        best_tcp["pick_x_offset_mm"] = parsed.pick_x_offset_mm
                        if parsed.dry_run:
                            node.motion_status = (f"DRY: ID {best_obj.object_id} "
                                                  f"TCP {best_tcp['tcp_position_base']}")
                            print(f"[DRY] {node.motion_status}", flush=True)
                        else:
                            node.start_sort(best_obj, best_tcp)
                    else:
                        detail = reject_lines[0] if reject_lines else "no cube detected"
                        node.motion_status = f"WAIT: {detail}"
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
