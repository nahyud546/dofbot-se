#!/usr/bin/env python3
"""System-Python ROS observation/IK worker. No serial or motor operations."""

import json
import math
import os
import sys
import time
import uuid
import textwrap
from pathlib import Path


# Pose confirmation is based on adjacent messages. Small bbox jitter can make
# a single frame miss the top-grasp confirmation even when the same cube is
# still in view. Keep the last confirmed fixed-pose grasp briefly in that case.
FIXED_GRASP_CACHE_S = 1.5
FIXED_GRASP_MAX_CENTER_SHIFT_PX = 8.0

VISION = Path(__file__).resolve().parents[1] / "vision_experiments"
sys.path.insert(0, str(VISION))


HAND_EYE_ENV = "T8_HAND_EYE_FILE"
MAX_SOURCE_LAYER = 3          # cube gắp được ở tầng 0..3 (tháp 4 cube)
MAX_TARGET_LAYER = 2          # đặt lên tầng 0..2 -> cube thứ 4 ở tầng 3
HANDEYE_CONFIRM_MAX_SHIFT_M = 0.010
HANDEYE_CONFIRM_MAX_YAW_DEG = 15.0
HANDEYE_HOLD_S = 2.0          # giữ kết quả tốt khi vài khung perception bị nhiễu
HANDEYE_HOLD_WINDOW = 5       # số mẫu tốt dùng cho trung vị
HANDEYE_HOLD_MAX_SHIFT_PX = 25.0
NOMINAL_INTRINSICS = (902.0, 875.4, 320.0, 240.0, 0.0)
CALIBRATED_CONFIG = Path(__file__).resolve().parents[2] / "config" / "robot" / "cube_6d_calibrated.yaml"


def handeye_calibration():
    """hand_eye.json đã đạt kiểm chứng, hoặc None (T8_HAND_EYE_FILE ghi đè đường dẫn)."""
    import cube_search_center_math as M
    return M.load_calibration(os.environ.get(HAND_EYE_ENV) or None)


def _intrinsics_from_yaml(path):
    import yaml
    camera = yaml.safe_load(Path(path).read_text())["camera"]
    K, dist = camera["K"], camera.get("distortion") or [0.0]
    return (float(K[0]), float(K[4]), float(K[2]), float(K[5]), float(dist[0]))


def perception_intrinsics(node):
    """((fx, fy, cx, cy, k1), nguồn): K mà perception đang dùng để giải PnP của tag.

    Thứ tự: biến môi trường T8_PERCEPTION_CONFIG (T8 đặt khi tự khởi động
    perception) -> tham số `config` của node perception (chậm: perception đang
    chiếm CPU nên có thể mất vài giây) -> file đã hiệu chuẩn -> danh định.
    """
    candidates = []
    env_path = os.environ.get("T8_PERCEPTION_CONFIG")
    if env_path:
        candidates.append(("env", env_path))
    else:
        try:
            import rclpy
            from rcl_interfaces.srv import GetParameters
            client = node.create_client(GetParameters, "/object_perception/get_parameters")
            if client.wait_for_service(timeout_sec=2.0):
                request = GetParameters.Request()
                request.names = ["config"]
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
                if future.done() and future.result() is not None and future.result().values:
                    value = future.result().values[0].string_value
                    if value:
                        candidates.append(("param", value))
        except Exception:
            pass
    candidates.append(("fallback-calibrated", str(CALIBRATED_CONFIG)))
    for source, candidate in candidates:
        if candidate and Path(candidate).is_file():
            try:
                return _intrinsics_from_yaml(candidate), f"{source}:{candidate}"
            except Exception:
                continue
    return NOMINAL_INTRINSICS, "nominal"


# pose PnP có sai số khớp tầng lớn hơn ngưỡng này thì thử dựng từ góc mặt + góc khớp (chính xác hơn)
GRAVITY_PREFER_RANGE_MM = 5.0


def locate_pose(pose, reason, stamp, joints, cal, intrinsics, faces=None):
    """Tầng + XY từ camera_T_cube (4x4 hoặc None); không bao giờ ném lỗi.

    Khi pose thiếu/không dùng được mà perception có 4 góc của một mặt (`faces`), dựng tầng/XY từ
    góc khớp + cube nằm phẳng (gravity_pose.locate_face): chạy được cả khi PnP mặt đơn thất bại.
    """
    from cube_layer import locate_cube
    servo = joints.stationary_servo(stamp)
    if servo is None:
        return {"ok": False, "pending": True,
                "reason": "khớp chưa đứng yên hoặc chưa có /real_joint_states"}
    located = None
    if pose is None:
        located = {"ok": False, "reason": reason or "chưa có pose camera"}
    else:
        try:
            located = locate_cube(pose, servo, cal, intrinsics)
        except Exception as exc:
            located = {"ok": False, "reason": f"lỗi tính tầng: {exc}"}
    if not faces:
        return located
    if located.get("ok") and float(located.get("range_error_mm") or 0.0) <= GRAVITY_PREFER_RANGE_MM:
        return located
    from gravity_pose import locate_face
    attempts = []
    for face in sorted(faces, key=lambda f: -float(f.get("conf", 0.0)) * _quad_area(f["quad"])):
        try:
            found = locate_face(face["quad"], face["axis"], servo, cal)
        except Exception as exc:
            found = {"ok": False, "reason": f"lỗi dựng từ mặt: {exc}"}
        if found.get("ok"):
            if (located.get("ok") and
                    found["range_error_mm"] >= float(located.get("range_error_mm") or 0.0)):
                break           # pose PnP vẫn khớp tầng tốt hơn; giữ nó
            found["pose_reason"] = located.get("reason", "")
            return found
        attempts.append(found.get("reason", ""))
    if located.get("ok"):
        return located
    located = dict(located)
    located["reason"] = (located.get("reason", "") + " | từ mặt: " +
                         (attempts[0] if attempts else "")).strip(" |")
    return located


def _quad_area(quad):
    pts = quad
    return abs(0.5 * sum(pts[i][0] * pts[(i + 1) % 4][1] - pts[(i + 1) % 4][0] * pts[i][1]
                         for i in range(4)))


class FaceQuadFeed:
    """Nhớ 4 góc mặt perception công bố trên /vision/face_quads theo track (vài giây)."""

    def __init__(self, ttl_s=1.5):
        self.ttl_s = ttl_s
        self.latest = {}

    def update(self, payload, now):
        try:
            data = json.loads(payload)
            tracks = data.get("tracks", {})
        except (TypeError, ValueError, AttributeError):
            return
        for tid, record in tracks.items():
            faces = [f for f in record.get("faces", [])
                     if len(f.get("quad", [])) == 4 and f.get("axis")]
            if faces:
                self.latest[str(tid)] = (now, faces)

    def faces(self, tid, now):
        entry = self.latest.get(str(tid))
        return entry[1] if entry and now - entry[0] <= self.ttl_s else []


def locate_object(obj, stamp, joints, cal, intrinsics, pose_matrix, faces=None):
    """Như locate_pose nhưng từ một ObjectState."""
    pose = None
    if bool(getattr(obj, "camera_pose_valid", False)):
        try:
            pose = pose_matrix(obj.camera_pose)
        except Exception:
            pose = None
    return locate_pose(pose, str(getattr(obj, "reason", "")), stamp, joints, cal, intrinsics, faces)


def yaw_delta_deg(a, b):
    """Hiệu yaw theo chu kỳ 90° (cube vuông đối xứng), trong [-45, 45)."""
    return (float(a) - float(b) + 45.0) % 90.0 - 45.0


class HandEyeHold:
    """Bộ lọc + xác nhận theo thời gian cho vị trí hand-eye của từng track.

    Pose RGB một mặt của perception nhấp nháy (báo nghiêng 30°+, mất khung, đổi
    track_id) dù cube nằm yên, và ở pose lạ pose cũ-kiểu "hai khung liên tiếp
    y hệt" gần như không bao giờ thoả. Ở đây: giữ vài mẫu tốt gần nhất; XY/yaw
    là trung vị (yaw theo chu kỳ 90°); xác nhận khi >= `confirm` mẫu trong
    `confirm_window_s` cùng tầng và cùng XY (<= `max_shift_m`); khung hỏng thoáng
    qua dùng lại kết quả đã giữ trong `ttl_s` giây khi cube chưa dời trong ảnh;
    track mới xuất hiện ngay chỗ track cũ (cùng ID) kế thừa mẫu của nó.
    """

    def __init__(self, ttl_s=HANDEYE_HOLD_S, window=HANDEYE_HOLD_WINDOW,
                 max_shift_px=HANDEYE_HOLD_MAX_SHIFT_PX, confirm=2,
                 confirm_window_s=1.5, max_shift_m=HANDEYE_CONFIRM_MAX_SHIFT_M,
                 max_yaw_deg=HANDEYE_CONFIRM_MAX_YAW_DEG):
        self.ttl_s, self.window, self.max_shift_px = ttl_s, window, max_shift_px
        self.confirm, self.confirm_window_s = confirm, confirm_window_s
        self.max_shift_m, self.max_yaw_deg = max_shift_m, max_yaw_deg
        self.tracks = {}

    @staticmethod
    def _centre(obj):
        box = [float(v) for v in obj.bbox_xyxy]
        return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)

    def _near(self, tid, obj, now_s):
        """Track khác cùng ID, còn mới, cùng chỗ trong ảnh (track_id vừa đổi)."""
        try:
            cx, cy = self._centre(obj)
            oid = int(obj.object_id)
        except (AttributeError, TypeError, ValueError):
            return None
        for other, entry in self.tracks.items():
            if (other != tid and entry.get("object_id") == oid
                    and now_s - entry["time_s"] <= self.ttl_s
                    and math.hypot(cx - entry["centre"][0],
                                   cy - entry["centre"][1]) <= self.max_shift_px):
                return other
        return None

    def update(self, tid, obj, loc, now_s):
        """Ghi một mẫu tốt; trả loc đã làm mượt (cùng tầng với mẫu mới nhất)."""
        entry = self.tracks.get(tid)
        if entry is None:
            donor = self._near(tid, obj, now_s)
            entry = {"samples": list(self.tracks[donor]["samples"]) if donor else []}
            self.tracks[tid] = entry
        samples = [s for s in entry["samples"] if s["loc"]["layer"] == loc["layer"]
                   and now_s - s["time_s"] <= 2 * self.ttl_s]
        samples.append({"loc": loc, "time_s": now_s})
        entry["samples"] = samples[-self.window:]
        entry.update(obj=obj, time_s=now_s, centre=self._centre(obj),
                     object_id=int(obj.object_id))
        entry["loc"] = self._smoothed(entry["samples"])
        return entry["loc"]

    @staticmethod
    def _smoothed(samples):
        import statistics
        locs = [s["loc"] for s in samples]
        out = dict(locs[-1])
        for key in ("x", "y", "range_error_mm"):
            out[key] = round(statistics.median(l[key] for l in locs), 5)
        ref = locs[-1]["yaw_deg"]
        spread = statistics.median(yaw_delta_deg(l["yaw_deg"], ref) for l in locs)
        out["yaw_deg"] = round(yaw_delta_deg(ref + spread, 0.0), 2)
        return out

    def recall(self, tid, obj, now_s):
        """(obj, loc) đã giữ nếu còn mới và cube chưa dời trong ảnh, ngược lại None."""
        entry = self.tracks.get(tid)
        if entry is None:
            donor = self._near(tid, obj, now_s)
            entry = self.tracks.get(donor) if donor else None
        if not entry or now_s - entry["time_s"] > self.ttl_s:
            return None
        try:
            cx, cy = self._centre(obj)
        except (AttributeError, TypeError, ValueError):
            return None
        if math.hypot(cx - entry["centre"][0], cy - entry["centre"][1]) > self.max_shift_px:
            return None
        return entry["obj"], entry["loc"]

    def agreeing(self, tid, now_s):
        """Số mẫu gần đây cùng tầng, cùng XY/yaw với mẫu mới nhất (>= confirm = xác nhận)."""
        entry = self.tracks.get(tid)
        if not entry or not entry["samples"]:
            return 0
        latest = entry["samples"][-1]["loc"]
        count = 0
        for sample in entry["samples"]:
            loc = sample["loc"]
            if (now_s - sample["time_s"] <= self.confirm_window_s
                    and loc["layer"] == latest["layer"]
                    and math.hypot(loc["x"] - latest["x"], loc["y"] - latest["y"]) <= self.max_shift_m
                    and abs(yaw_delta_deg(loc["yaw_deg"], latest["yaw_deg"])) <= self.max_yaw_deg):
                count += 1
        return count

    def forget(self, tid):
        self.tracks.pop(tid, None)

    def prune(self, active, now_s):
        """Bỏ mẫu quá cũ; track vắng một lúc vẫn giữ trong ttl để track mới kế thừa."""
        self.tracks = {tid: e for tid, e in self.tracks.items()
                       if now_s - e["time_s"] <= 2 * self.ttl_s}


def handeye_grasp(loc, base_grasp=None):
    """Grasp dict từ kết quả locate_cube; base_grasp (nếu có) cho các trường phụ."""
    grasp = dict(base_grasp or {"gripper_width_m": 0.033, "quality_score": 0.5})
    grasp.update(
        tcp_position_base=[loc["x"], loc["y"], loc["tcp_z"]],
        surface_id="+Z", surface_normal_base=[0.0, 0.0, 1.0],
        approach_vector_base=[0.0, 0.0, -1.0],
        preferred_yaw_rad=math.radians(float(loc["yaw_deg"])),
        center_px=list(loc["pixel"]),
        coordinate_source="handeye_tag_plane",
        layer=int(loc["layer"]), top_z=loc["top_z"],
        range_error_mm=loc["range_error_mm"])
    return grasp


def _valid_box(box):
    try:
        ok = (len(box) == 4 and box[2] - box[0] >= 10
              and box[3] - box[1] >= 10
              and 0 <= box[0] < box[2] <= 640
              and 0 <= box[1] < box[3] <= 480)
    except TypeError:
        return False
    return bool(ok)


def snapshot(timeout_s, expect=None):
    import rclpy
    from rclpy.node import Node
    from cap_scene_interfaces.msg import ObjectStates
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from cube_sort_3d import grasp_candidates_for_obj, pose_matrix
    from cube_layer import JointWindow, servo_from_joint_state

    rclpy.init()
    node = Node("t8_scene_snapshot")
    latest = []
    tracks = {}
    last_meta = {"calibrated": False, "status": ""}
    joint_report = {}
    cal = handeye_calibration()
    intrinsics, intrinsics_source = perception_intrinsics(node) if cal else (None, "")
    joints = JointWindow()
    quad_feed = FaceQuadFeed()

    def on_joints(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        joint_report.update(stamp=stamp, positions=list(msg.position))
        servo = servo_from_joint_state(list(msg.name), list(msg.position))
        if servo is not None:
            joints.add(stamp, servo)

    def observe(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not 0 <= age <= 2.0:
            return
        last_meta["calibrated"] = bool(msg.calibrated)
        last_meta["status"] = str(msg.status)
        now_m = time.monotonic()
        for obj in msg.objects:
            try:
                oid = int(obj.object_id)
            except (TypeError, ValueError):
                continue
            if oid not in (1, 2, 3, 4):
                continue
            try:
                box = [int(v) for v in obj.bbox_xyxy]
            except (TypeError, ValueError):
                continue
            if not _valid_box(box):
                continue
            pose_stamp = (obj.camera_pose.header.stamp.sec +
                          obj.camera_pose.header.stamp.nanosec * 1e-9)
            same_image_time = abs(pose_stamp - stamp) <= 0.001
            camera_current = bool(same_image_time and obj.camera_pose_valid and
                                  obj.top_grasp_ready)
            metric = bool(msg.calibrated and same_image_time and obj.base_pose_valid and
                          obj.base_pose.header.frame_id == "base_link")
            try:
                gid = int(obj.geometry_model_id)
            except (TypeError, ValueError):
                gid = 0
            tid = str(obj.track_id)
            tr = tracks.setdefault(tid, {"sightings": []})
            tr["sightings"].append({
                "mono": now_m, "oid": oid, "gid": gid, "box": box,
                "method": str(obj.pose_method),
                "idconf": float(obj.identity_confidence),
                "poseconf": float(obj.pose_confidence),
                # Fixed READY_POSE grasp mapping is independently calibrated
                # and does not require dynamic base_pose/hand-eye validity.
                "grasp": (grasp_candidates_for_obj(obj)[0]
                          if camera_current and grasp_candidates_for_obj(obj) else None),
                "base_position": (pose_matrix(obj.base_pose)[:3, 3].tolist()
                                  if metric else None),
                "base_valid": metric,
                "camera_current": camera_current,
                # hand-eye: cube chỉ có 4 góc mặt (không pose PnP) vẫn tính là quan sát hiện tại
                "confirm_current": bool(camera_current or (
                    cal is not None and quad_feed.faces(tid, now_m))),
                "top_grasp_ready": bool(obj.top_grasp_ready),
                "camera_pose_valid": bool(obj.camera_pose_valid),
                "pose_valid": bool(obj.pose_valid),
                "reason": str(obj.reason),
                "pose": (pose_matrix(obj.camera_pose)
                         if cal is not None and bool(obj.camera_pose_valid) else None),
                "stamp": stamp,
                "faces": quad_feed.faces(tid, now_m) if cal is not None else [],
            })
            tr["sightings"] = tr["sightings"][-6:]
            tr["last_seen"] = now_m
        for tid in [t for t, tr in tracks.items()
                    if now_m - tr.get("last_seen", 0) > 4.0]:
            del tracks[tid]
        objects = []
        for tid, tr in tracks.items():
            recent = [s for s in tr["sightings"] if now_m - s["mono"] <= 4.0]
            if not recent:
                continue
            last = recent[-1]
            # Median tung toa do (chong flicker): ID khoa roi moi lay vi tri.
            med = []
            for i in range(4):
                vals = sorted(s["box"][i] for s in recent)
                med.append(vals[len(vals) // 2])
            agree = (len(recent) >= 2 and recent[-1]["confirm_current"]
                     and recent[-2]["confirm_current"]
                     and recent[-1]["oid"] == recent[-2]["oid"]
                     and recent[-1]["gid"] == recent[-2]["gid"]
                     and abs(recent[-1]["box"][0] + recent[-1]["box"][2]
                             - recent[-2]["box"][0] - recent[-2]["box"][2]) / 2 <= 30
                     and abs(recent[-1]["box"][1] + recent[-1]["box"][3]
                             - recent[-2]["box"][1] - recent[-2]["box"][3]) / 2 <= 30
                     and recent[-1]["mono"] - recent[-2]["mono"] <= 4.0)
            objects.append({
                "track_id": tid, "object_id": last["oid"],
                "geometry_model_id": last["gid"],
                "bbox_xyxy": med,
                "center_px": [(med[0] + med[2]) / 2.0, (med[1] + med[3]) / 2.0],
                "pose_method": last["method"],
                "top_grasp_ready": last["top_grasp_ready"],
                "camera_pose_valid": last["camera_pose_valid"],
                "pose_valid": last["pose_valid"],
                "identity_confidence": last["idconf"],
                "pose_confidence": last["poseconf"],
                "age_s": float(now_m - last["mono"]),
                "base_pose_valid": last["base_valid"],
                "base_position": last["base_position"],
                "base_transform": None,
                "stationary": bool(agree),
                "joint_age_s": None,
                "seen_count": len(recent),
                "confirmed": bool(agree),
                "grasp": last["grasp"],
                "reason": last["reason"],
                "handeye": (locate_pose(last["pose"], last["reason"], last["stamp"],
                                        joints, cal, intrinsics, last.get("faces"))
                            if cal is not None else None),
            })
        if cal is not None and expect:
            # Kiểm tra vị trí kỳ vọng bằng pixel: hợp cả khi mặt trên là mặt màu/ảnh (pose RGB
            # nghiêng/không tin được) vì chỉ cần khung bao của cube nằm đúng chỗ đã đặt.
            from cube_layer import project_base_point
            servo_now = joints.stationary_servo(stamp)
            for obj in objects:
                for want in expect:
                    if int(want.get("object_id", -1)) != int(obj["object_id"]):
                        continue
                    if servo_now is None:             # chưa đủ lịch sử khớp đứng yên: chờ thêm
                        obj["expect_pending"] = True
                        continue
                    pixel = project_base_point(want["xyz"], servo_now, cal)
                    if pixel is not None:
                        cx, cy = obj["center_px"]
                        obj["expected_px"] = [round(pixel[0], 1), round(pixel[1], 1)]
                        obj["expected_px_error"] = round(math.hypot(cx - pixel[0], cy - pixel[1]), 1)
        robot = {"phase": "unknown", "joint_positions_rad": None,
                 "holding_verified": False, "phase_source": "unknown"}
        if joint_report:
            joint_age = node.get_clock().now().nanoseconds * 1e-9 - joint_report["stamp"]
            robot["joint_age_s"] = joint_age
            if 0 <= joint_age <= 2:
                robot["joint_positions_rad"] = joint_report["positions"]
        try:
            state_file = Path("/tmp/t8_hold_state.json")
            state_age = max(0.0, time.time() - state_file.stat().st_mtime)
            robot["phase_report_age_s"] = state_age
            if state_age <= 15.0:
                robot["phase"] = json.loads(state_file.read_text()).get("phase", "unknown")
                robot["phase_source"] = "legacy_state_file_unverified"
        except (OSError, ValueError):
            pass
        robot["handeye_ready"] = bool(
            cal is not None and joints.stationary_servo(stamp) is not None)
        latest[:] = [{"ok": True, "stamp": stamp, "age_s": age,
                      "calibrated": last_meta["calibrated"],
                      "status": last_meta["status"],
                      "handeye_available": cal is not None,
                      "perception_intrinsics": intrinsics_source,
                      "objects": objects, "robot": robot}]

    node.create_subscription(ObjectStates, "/vision/object_states", observe, 10)
    node.create_subscription(JointState, "/real_joint_states", on_joints, 10)
    node.create_subscription(String, "/vision/face_quads",
                             lambda msg: quad_feed.update(msg.data, time.monotonic()), 10)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            # Ve som khi co it nhat 1 track va MOI track deu confirmed
            # (2 sightings dong ID/box). Flicker -> doi het timeout.
            # Hand-eye needs a short stationary joint history; wait for it only
            # while that is the sole thing missing (not for geometry failures).
            if (latest and latest[0]["objects"] and
                    all(obj["confirmed"] and obj["stationary"] and
                        (obj["age_s"] > 1.0 or not ((obj.get("handeye") or {}).get("pending")
                                                    or obj.get("expect_pending")))
                        for obj in latest[0]["objects"])):
                return latest[0]
        if latest:
            now = node.get_clock().now().nanoseconds * 1e-9
            age = now - latest[0]["stamp"]
            if 0 <= age <= 2.0:
                latest[0]["age_s"] = age
                for obj in latest[0]["objects"]:
                    obj["age_s"] = age
                return latest[0]
        return {"ok": False, "reason": "Chưa có /vision/object_states mới; hãy chạy perception ROS 3D."}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def zone_survey(zones, timeout_s=6.0, expect_j1=None):
    """Đo biên thả của từng zone từ khung camera tay hiện tại (chỉ đọc, không di chuyển tay).

    zones: [{"zone_id": 1..4, "release_xy": [x, y]}] (XY base của điểm thả cấu hình).
    Chờ khớp đứng yên + một khung tươi, rồi dùng zone_locator (mặt nạ ô theo màu, chiếu điểm thả
    bằng hand-eye + khớp đo thật) để lấy biên tại góc cấu hình và độ lệch J1 tốt nhất.
    Trả {"ok", "servo", "zones": [{zone_id, seen, clipped, area, margin_cfg_mm, delta_deg,
    margin_best_mm}]}.
    """
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from cube_layer import JointWindow, servo_from_joint_state
    import zone_locator as ZL

    cal = handeye_calibration()
    if cal is None:
        return {"ok": False, "reason": "Chưa có hand-eye đã kiểm chứng; không khảo sát zone được."}
    rclpy.init()
    node = Node("t8_zone_survey")
    bridge = CvBridge()
    joints = JointWindow()
    frame = {"image": None, "stamp": None}

    def on_joints(msg):
        servo = servo_from_joint_state(list(msg.name), list(msg.position))
        if servo is not None:
            joints.add(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, servo)

    def on_image(msg):
        try:
            frame["image"] = bridge.imgmsg_to_cv2(msg, "bgr8")
            frame["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            pass

    node.create_subscription(JointState, "/real_joint_states", on_joints, 10)
    node.create_subscription(Image, "/cap_vision/image_raw", on_image, image_qos())
    table_z = float(cal["tag_top_z"]) - 0.030
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if frame["image"] is None:
                continue
            servo = joints.stationary_servo(frame["stamp"])
            if servo is None:
                continue
            if expect_j1 is not None and abs(servo[0] - float(expect_j1)) > 14.0:
                continue        # real_joint_states chưa cập nhật sau khi xoay: đừng dùng góc cũ
            image = frame["image"]
            if image.shape[:2] != (ZL.FRAME_H, ZL.FRAME_W):
                return {"ok": False, "reason": f"khung ảnh {image.shape[:2]} không phải 480x640"}
            report = []
            for zone in zones:
                zid = int(zone["zone_id"])
                xy = [float(zone["release_xy"][0]), float(zone["release_xy"][1])]
                mask = ZL.pad_mask(image, zid)
                if mask is None:
                    report.append({"zone_id": zid, "seen": False})
                    continue
                hit = ZL.find_pad(image, zid)
                delta, m_cfg, m_best = ZL.best_shift(mask, xy, servo, cal, table_z)
                center_xy, area_m2 = None, None
                try:
                    import cv2
                    from scene_geometry import pad_polygon_base
                    polygon = pad_polygon_base(mask, servo, cal, table_z)
                    if polygon is not None and len(polygon) >= 3:
                        moments = cv2.moments(polygon.astype(np.float32).reshape(-1, 1, 2))
                        if abs(moments["m00"]) > 1e-9:
                            center_xy = [round(moments["m10"] / moments["m00"], 4),
                                         round(moments["m01"] / moments["m00"], 4)]
                            area_m2 = round(abs(moments["m00"]), 5)
                except Exception:
                    pass
                report.append({"zone_id": zid, "seen": True,
                               "center_xy": center_xy, "area_m2": area_m2,
                               "clipped": bool(hit.clipped) if hit else False,
                               "area": round(float(hit.area), 0) if hit else None,
                               "margin_cfg_mm": None if m_cfg is None else round(m_cfg, 1),
                               "delta_deg": float(delta),
                               "margin_best_mm": None if m_best is None else round(m_best, 1)})
            return {"ok": True, "servo": [round(float(v), 2) for v in servo], "zones": report,
                    "image_stamp": frame["stamp"]}
        return {"ok": False, "reason": "Không có khung camera tay khi khớp đứng yên "
                                       "(kiểm tra /cap_vision/image_raw và /real_joint_states)."}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def preflight(targets, placement_correction_gripper_xy_m=None,
              handeye_correction_gripper_xy_m=None):
    from cube_3d_preflight import check_target
    from t8_motion_worker import Kinematics
    if not isinstance(targets, list) or not 1 <= len(targets) <= 6:
        raise ValueError("Preflight needs 1–6 TCP stages")
    kin = Kinematics()
    try:
        results = []
        for target in targets:
            try:
                checked = check_target(target, kin, placement_correction_gripper_xy_m,
                                       handeye_correction_gripper_xy_m)
            except Exception as exc:
                tcp = target.get("tcp_position_base", [])
                return {"ok": False,
                        "reason": (f"stage={target.get('stage')} tcp={tcp}: {exc}"),
                        "failed_stage": target.get("stage"),
                        "failed_tcp_position_base": tcp}
            results.append({"stage": target["stage"], **checked})
        if len(results) > 1 and abs(results[0]["ik_joints_deg"][0] - results[-1]["ik_joints_deg"][0]) > 45:
            return {"ok": False,
                    "reason": "source-to-target J1 exceeds 45-degree holding limit",
                    "failed_stage": results[-1]["stage"],
                    "failed_tcp_position_base": results[-1]["tcp_position_base"]}
        return {"ok": True, "stages": results, "collision_checked": False,
                "physical_tcp_validated": False}
    finally:
        kin.close()


def image_qos():
    """QoS nhận /cap_vision/image_raw. KHÔNG dùng best-effort (qos_profile_sensor_data): ảnh 900 KB bị rớt
    khung khi truyền (đo: 5 khung/s với nhiều khoảng đứt 1–4 s => viewer quá 2 s không có ảnh nên đen),
    trong khi reliable cho ảnh liên tục, khoảng cách lớn nhất 0,15 s."""
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2)


def approval_event(event, reason):
    print("T8_EVENT:" + json.dumps({"event": event, "reason": reason}, ensure_ascii=False), flush=True)


def _viewer_ascii(text):
    """cv2 Hershey chi hien ASCII; strip dau de khoi ??? nhu hinh user."""
    import unicodedata as _ud
    text = _ud.normalize("NFKD", str(text)).replace("đ", "d").replace("Đ", "D")
    return text.encode("ascii", "ignore").decode("ascii")


def live_approval_error(image_age, scene_age, shape):
    if image_age > 2.0:
        return "NO FRESH CAMERA IMAGE"
    if shape != (480, 640):
        return "CAMERA MUST BE 640x480"
    if scene_age > 2.0:
        return "NO FRESH OBJECT STATES"
    return ""


class ApprovalDisplayStatus:
    """Debounce display text; readiness still drops immediately on rejection."""
    def __init__(self):
        self.text = "WAITING FOR TARGETS"
        self.pending = None
        self.since = 0.0
        self.ready_since = None
        self.lock_key = None
        self.initialized = False

    def update(self, text, ready, now, lock_key=()):
        if not self.initialized:
            self.initialized = True
            if not ready:
                self.text = text
        if ready:
            if self.ready_since is None or lock_key != self.lock_key:
                self.ready_since, self.lock_key = now, lock_key
            if now - self.ready_since >= 1.0:
                self.text = "READY - PRESS SPACE"
                self.pending = None
                return self.text, True
            self.text = "STABILIZING TARGETS (1s)"
            return self.text, False
        self.ready_since = None
        if self.text in {"READY - PRESS SPACE", "STABILIZING TARGETS (1s)"}:
            self.text = "WAITING FOR VALID TARGETS"
        if text != self.pending:
            self.pending, self.since = text, now
        if now - self.since >= 1.0:
            self.text = text
        return self.text, False


def select_ready_targets(entries, wanted):
    """Choose one valid track per ID; rejected duplicate proposals are noise.

    A duplicate is ambiguous only when two tracks are simultaneously valid.
    Previously one valid track plus one rejected proposal locked the viewer.
    """
    matches = {}
    reject = []
    for identity in wanted:
        candidates = [(obj, reason) for obj, ready, reason in entries
                      if int(obj.object_id) == identity and ready]
        if len(candidates) == 1:
            matches[identity] = candidates[0][0]
            continue
        if len(candidates) > 1:
            reject.append(f"multiple valid tracks for ID {identity}")
            continue
        reasons = sorted(set(reason for obj, _ready, reason in entries
                             if int(obj.object_id) == identity and reason))
        reject.append(f"ID {identity}: " + ("; ".join(reasons)
                                             if reasons else "missing/stabilizing"))
    return matches, reject


def fixed_grasp_cache_matches(cached, obj, now):
    """Allow brief pose-confirmation flicker without accepting a moved cube."""
    if cached is None or now - cached["time_s"] > FIXED_GRASP_CACHE_S:
        return False
    previous = cached["obj"]
    if (int(previous.object_id) != int(obj.object_id) or
            int(previous.geometry_model_id) != int(obj.geometry_model_id)):
        return False
    try:
        old_box = [float(value) for value in previous.bbox_xyxy]
        new_box = [float(value) for value in obj.bbox_xyxy]
        if len(old_box) != 4 or len(new_box) != 4:
            return False
        dx = ((new_box[0] + new_box[2]) - (old_box[0] + old_box[2])) * 0.5
        dy = ((new_box[1] + new_box[3]) - (old_box[1] + old_box[3])) * 0.5
        return math.hypot(dx, dy) <= FIXED_GRASP_MAX_CENTER_SHIFT_PX
    except (TypeError, ValueError, AttributeError):
        return False


PROVEN_TABLE_PICK_Z = 0.047   # TCP height that grips a table-level cube
CUBE_EDGE_M = 0.030
HAND_EYE_FILE = Path(__file__).resolve().parents[2] / "config" / "robot" / "hand_eye.json"


def table_level_centre_z(path=None):
    """Cube-centre height for a cube on the table, from the hand-eye calibration."""
    try:
        data = json.loads(Path(path or HAND_EYE_FILE).read_text())
        if data.get("accepted") is not True:
            return None
        value = float(data["tag_top_z"]) - CUBE_EDGE_M / 2
        return value if math.isfinite(value) else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def metric_top_grasp(base_T, reference_z=None):
    """Top grasp from a calibrated base_link cube pose; valid at any camera pose.

    XY comes from the measured pose. Z is snapped to the proven table pick
    height plus whole cube layers, so a few mm of vision height bias cannot
    drive the gripper into the table. Returns (grasp, "") or (None, reason).
    """
    rotation = [[float(base_T[r][c]) for c in range(3)] for r in range(3)]
    position = [float(base_T[r][3]) for r in range(3)]
    up = max(range(3), key=lambda axis: abs(rotation[2][axis]))
    if abs(rotation[2][up]) < 0.90:
        return None, "cube nghiêng; không có mặt trên nằm ngang"
    side = (up + 1) % 3
    yaw = math.degrees(math.atan2(rotation[1][side], rotation[0][side]))
    yaw = (yaw + 45.0) % 90.0 - 45.0
    if reference_z is None:
        tcp_z = position[2] + .002
    else:
        layers = round((position[2] - reference_z) / CUBE_EDGE_M)
        if layers < 0 or abs(position[2] - reference_z - layers * CUBE_EDGE_M) > .010:
            return None, (f"độ cao cube z={position[2]:.3f} m không khớp tầng nào "
                          f"(tầng bàn {reference_z:.3f} m)")
        tcp_z = PROVEN_TABLE_PICK_Z + layers * CUBE_EDGE_M
    return {"surface_id": ("+" if rotation[2][up] > 0 else "-") + "XYZ"[up],
            "tcp_position_base": [round(position[0], 5), round(position[1], 5),
                                  round(tcp_z, 5)],
            "surface_normal_base": [0.0, 0.0, 1.0],
            "approach_vector_base": [0.0, 0.0, -1.0],
            "preferred_yaw_rad": math.radians(yaw),
            "gripper_width_m": CUBE_EDGE_M + .003,
            "coordinate_source": "calibrated_base_pose"}, ""


def draw_handeye_skeleton(frame, loc, servo, cal, ready):
    """Vẽ khung cube (8 đỉnh) tại vị trí hand-eye đã tính: kiểm tra bằng mắt XY/tầng/yaw.

    Dựng từ góc khớp thật + hand-eye nên vẽ được cả khi perception không có pose PnP (chỉ 4 góc
    một mặt). Trả True nếu đã vẽ.
    """
    import cv2
    import numpy as np
    from gravity_pose import wireframe_pixels
    if not (loc and loc.get("ok") and servo is not None and "base_yaw_rad" in loc):
        return False
    pts = wireframe_pixels(loc["x"], loc["y"], loc["top_z"], loc["base_yaw_rad"], servo, cal)
    if any(p is None for p in pts):
        return False
    pix = [(int(round(u)), int(round(v))) for u, v in pts]
    color = (0, 255, 0) if ready else (0, 190, 255)
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]:
        cv2.line(frame, pix[a], pix[b], color, 2, cv2.LINE_AA)
    centre = tuple(int(v) for v in np.mean(np.array(pix[:4]), axis=0))
    cv2.putText(frame, f"L{loc['layer']}", (centre[0] - 10, centre[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return True


def approval_image(state, now):
    """Prefer perception's smoothed skeleton; raw frames are only a fallback."""
    for kind in ("annotated", "raw"):
        if state.get(kind) is not None and now - state[kind + "_stamp"] <= 2.0:
            return state[kind], now - state[kind + "_stamp"]
    return None, math.inf


def approve(object_ids, operation, timeout_s, pick_x_offset_mm=15.0, notice="",
            handeye=False):
    """Wait for stable strict-ID targets and require Space before returning.

    Identity is accepted only when object_id and geometry_model_id agree in
    consecutive fresh ObjectStates messages. Colour is never consulted here.
    """
    import cv2
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    from cap_scene_interfaces.msg import ObjectStates
    from cube_layer import JointWindow, servo_from_joint_state
    from cube_sort_3d import (grasp_candidates_for_obj, is_graspable,
                              pose_matrix, update_confirmation)

    wanted = [int(value) for value in object_ids]
    if not wanted or len(wanted) > 2 or any(value not in (1, 2, 3, 4) for value in wanted):
        raise ValueError("approval cần một hoặc hai cube ID 1-4")
    if len(set(wanted)) != len(wanted):
        raise ValueError("nguồn và đích phải là hai cube ID khác nhau")
    if operation not in ("pick_hold", "stack", "stack_fixed", "sort"):
        raise ValueError("operation approval không hỗ trợ")

    def needs_metric(obj):
        if operation == "stack_fixed":
            return False
        # Fixed table mapping must not be reused for a visibly elevated cube.
        return operation == "stack" or (bool(getattr(obj, "base_pose_valid", False))
                and float(obj.base_pose.pose.position.z) > .055)

    reference_z = table_level_centre_z()

    def use_metric(obj):
        """Calibrated base pose wins whenever it is valid and current."""
        if needs_metric(obj):
            return True
        if operation == "stack_fixed" or not state["calibrated"]:
            return False
        if not (obj.base_pose_valid and obj.base_pose.header.frame_id == "base_link"):
            return False
        base_stamp = obj.base_pose.header.stamp.sec + obj.base_pose.header.stamp.nanosec * 1e-9
        return abs(base_stamp - state["scene_stamp"]) <= .001

    rclpy.init()
    node = Node("t8_target_approval")
    bridge = CvBridge()
    cal = handeye_calibration() if handeye and operation != "stack_fixed" else None
    intrinsics = perception_intrinsics(node)[0] if cal else None
    joints = JointWindow()
    hold = HandEyeHold()
    quad_feed = FaceQuadFeed()
    state = {"objects": [], "confirmations": {}, "located": {},
             "fixed_valid": {},
             "annotated": None, "annotated_stamp": -math.inf,
             "raw": None, "raw_stamp": -math.inf,
             "scene_stamp": 0.0, "scene_received": -math.inf,
             "calibrated": False, "status": "waiting"}

    def on_image(msg, kind="annotated"):
        try:
            state[kind] = bridge.imgmsg_to_cv2(msg, "bgr8")
            state[kind + "_stamp"] = time.monotonic()
        except Exception:
            pass

    def handeye_step(obj, tid, stamp, now):
        """(obj, ready, reason) cho một track ở chế độ hand-eye.

        Hoàn toàn tách khỏi bản đồ pixel cũ: vị trí/tầng/yaw đến từ locate_object,
        lọc + xác nhận theo thời gian bằng HandEyeHold (không bao giờ rơi về
        update_confirmation cũ, vốn đòi mặt trên nằm trọn ảnh ở READY_POSE).
        """
        located = locate_object(obj, stamp, joints, cal, intrinsics, pose_matrix,
                                quad_feed.faces(tid, now))
        if located.get("ok"):
            loc = hold.update(tid, obj, located, now)
            source_obj = obj
        elif located.get("pending"):
            hold.forget(tid)                 # tay đang chuyển động: mẫu cũ không còn đúng
            loc, source_obj = None, None
        else:
            recalled = hold.recall(tid, obj, now)
            source_obj, loc = recalled if recalled else (None, None)
        if loc is None:
            state["located"][tid] = located
            state["confirmations"].pop(tid, None)
            return obj, False, str(getattr(obj, "reason", "")) or located.get("reason", "")
        state["located"][tid] = loc
        strict = (int(obj.object_id) in wanted
                  and int(obj.geometry_model_id) == int(obj.object_id))
        if not strict:
            state["confirmations"].pop(tid, None)
            return obj, False, "ID/model conflict"
        agree = hold.agreeing(tid, now)
        if agree < hold.confirm:
            state["confirmations"].pop(tid, None)
            return source_obj, False, f"waiting hand-eye confirmation ({agree}/{hold.confirm})"
        state["confirmations"][tid] = {"time_s": now}
        return source_obj, True, ""

    def on_states(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not 0 <= age <= 2.0:
            state["objects"] = []
            state["confirmations"].clear()
            state["status"] = "STALE SCENE"
            return
        now = time.monotonic()
        active = set()
        accepted = []
        for obj in msg.objects:
            tid = str(obj.track_id)
            active.add(tid)
            if cal is not None:
                accepted.append(handeye_step(obj, tid, stamp, now))
                continue
            sample = update_confirmation(state["confirmations"].get(tid), obj, now)
            if sample is None:
                cached = state["fixed_valid"].get(tid)
                if operation == "stack_fixed" and fixed_grasp_cache_matches(cached, obj, now):
                    # Pose fusion may flicker briefly. Keep the last confirmed
                    # grasp while this same track remains within a small image shift.
                    accepted.append((cached["obj"], True, ""))
                    continue
                state["confirmations"].pop(tid, None)
                state["fixed_valid"].pop(tid, None)
                accepted.append((obj, False, str(obj.reason) or "no valid top grasp"))
                continue
            state["confirmations"][tid] = sample
            strict_identity = (int(obj.object_id) in wanted and
                               int(obj.geometry_model_id) == int(obj.object_id))
            ready, reason = is_graspable(obj, sample, now)
            pose_stamp = obj.camera_pose.header.stamp.sec + obj.camera_pose.header.stamp.nanosec * 1e-9
            if abs(pose_stamp - stamp) > .001:
                ready, reason = False, "camera pose timestamp mismatch"
                state["confirmations"].pop(tid, None)
            valid = strict_identity and ready
            if valid and operation == "stack_fixed":
                state["fixed_valid"][tid] = {"obj": obj, "time_s": now}
            elif operation == "stack_fixed":
                cached = state["fixed_valid"].get(tid)
                if fixed_grasp_cache_matches(cached, obj, now):
                    accepted.append((cached["obj"], True, ""))
                    continue
            accepted.append((obj, valid, reason if strict_identity else "ID/model conflict"))
        state["confirmations"] = {
            tid: sample for tid, sample in state["confirmations"].items()
            if tid in active and now - sample["time_s"] <= 0.75
        }
        state["fixed_valid"] = {
            tid: cached for tid, cached in state["fixed_valid"].items()
            if tid in active and now - cached["time_s"] <= FIXED_GRASP_CACHE_S
        }
        hold.prune(active, now)
        state["objects"] = accepted
        state["scene_stamp"] = stamp
        state["scene_received"] = now - age
        state["calibrated"] = bool(msg.calibrated)
        state["status"] = str(msg.status)

    def on_joints(msg):
        servo = servo_from_joint_state(list(msg.name), list(msg.position))
        if servo is not None:
            joints.add(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, servo)

    node.create_subscription(ObjectStates, "/vision/object_states", on_states, 10)
    node.create_subscription(Image, "/vision/object_annotated", on_image, 10)
    node.create_subscription(Image, "/cap_vision/image_raw", lambda msg: on_image(msg, "raw"),
                             image_qos())
    node.create_subscription(JointState, "/real_joint_states", on_joints, 10)
    node.create_subscription(String, "/vision/face_quads",
                             lambda msg: quad_feed.update(msg.data, time.monotonic()), 10)
    window = "T8 approval - SPACE confirm / ESC cancel"
    deadline = time.monotonic() + float(timeout_s) if float(timeout_s) > 0 else math.inf
    last_banner = None
    opened = False
    display_status = ApprovalDisplayStatus()
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
            now = time.monotonic()
            selected_image, image_age = approval_image(state, now)
            frame = selected_image.copy() if selected_image is not None else np.zeros((480, 640, 3), dtype=np.uint8)
            entries = []
            for obj, ready, reason in state["objects"]:
                oid = int(obj.object_id)
                if oid not in wanted:
                    continue
                tid = str(obj.track_id)
                confirmation = state["confirmations"].get(tid)
                fixed_cached = state["fixed_valid"].get(tid)
                cache_live = (operation == "stack_fixed" and fixed_grasp_cache_matches(
                                  fixed_cached, obj, now) and
                              fixed_cached["obj"] is obj)
                if (confirmation is None or now - confirmation["time_s"] > .75) and not cache_live:
                    ready, reason = False, reason or "confirmation expired"
                loc = state["located"].get(tid) if cal is not None else None
                if cal is not None:
                    if not (loc and loc.get("ok")):
                        ready = False
                        reason = "HAND-EYE: " + _viewer_ascii(
                            (loc or {}).get("reason") or "locating layer")
                    elif loc["layer"] > (MAX_TARGET_LAYER if operation == "stack"
                                         and oid == wanted[-1] else MAX_SOURCE_LAYER):
                        limit = (MAX_TARGET_LAYER if operation == "stack"
                                 and oid == wanted[-1] else MAX_SOURCE_LAYER)
                        ready, reason = False, (f"LAYER {loc['layer']} above verified "
                                                f"limit {limit}")
                elif needs_metric(obj) and not (state["calibrated"] and obj.base_pose_valid
                        and obj.base_pose.header.frame_id == "base_link"):
                    ready, reason = False, "stack_geometry_unavailable: calibrated base pose required"
                if cal is None and needs_metric(obj) and obj.base_pose_valid:
                    base_stamp = obj.base_pose.header.stamp.sec + obj.base_pose.header.stamp.nanosec * 1e-9
                    if abs(base_stamp - state["scene_stamp"]) > .001:
                        ready, reason = False, "stack_geometry_unavailable: stale base pose"
                entries.append((obj, ready, reason))
            if cal is not None:
                draw_servo = joints.stationary_servo(state["scene_stamp"])
                for entry_obj, entry_ready, _ in entries:
                    draw_handeye_skeleton(frame, state["located"].get(str(entry_obj.track_id)),
                                          draw_servo, cal, entry_ready)
            matches, reject = select_ready_targets(entries, wanted)
            # Stacking uses calibrated base poses; single-cube table picks use
            # the fixed READY_POSE mapping from cube_sort_3d.py.
            ready = (not reject and all(identity in matches for identity in wanted))
            live_error = live_approval_error(image_age, now - state["scene_received"], frame.shape[:2])
            if live_error:
                ready = False
            diagnostic = ("READY - PRESS SPACE" if ready else
                          (live_error or ("; ".join(sorted(set(reject))) if reject else
                           "MISSING/STABILIZING IDs " + str([oid for oid in wanted if oid not in matches]))))
            if not live_error and any("calibrated base pose required" in reason for reason in reject):
                diagnostic = f"stack_geometry_unavailable: calibrated base pose required (IDs {wanted})"
            lock_key = tuple((oid, str(matches[oid].track_id)) for oid in sorted(matches))
            banner, ready = display_status.update(diagnostic, ready, now, lock_key)
            if banner != last_banner:
                approval_event("ready" if ready else "waiting", banner)
                last_banner = banner
            readable = banner.replace("stack_geometry_unavailable: calibrated base pose required",
                                      "CALIBRATION REQUIRED (base pose)")
            lines = [f"{operation.upper()} IDs {wanted} | SPACE confirm / ESC cancel"]
            if notice:
                lines += textwrap.wrap("LAST PREFLIGHT: " + _viewer_ascii(str(notice)), 68)[:3]
            lines += textwrap.wrap(_viewer_ascii(readable), 68)[:3]
            cv2.rectangle(frame, (0, 0), (640, 22 * len(lines) + 8), (0, 0, 0), -1)
            for index, line in enumerate(lines):
                cv2.putText(frame, line, (8, 20 + 22 * index), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 255, 0) if ready else (0, 190, 255), 1, cv2.LINE_AA)
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if not opened:
                approval_event("viewer_opened", "Viewer đã khởi tạo; chọn cửa sổ, Space duyệt / Esc hủy.")
                opened = True
            # GTK does not implement WND_PROP_VISIBLE (returns -1 even open).
            # AUTOSIZE is supported by both GTK and Qt; an absent window either
            # returns -1 or raises cv2.error.
            try:
                closed = cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0
            except cv2.error:
                closed = True
            if closed:
                return {"ok": False, "code": "approval_cancelled", "reason": "Viewer đã đóng."}
            if key == 27:
                return {"ok": False, "code": "approval_cancelled",
                        "reason": "Người dùng đã hủy target bằng Esc."}
            if key != ord(" "):
                continue
            if not ready:
                # `diagnostic` describes the raw scene; during the 1s debounce
                # it can already say READY while the displayed state is still
                # STABILIZING. Report exactly what the operator sees.
                approval_event("waiting", "Space bị khóa: " + banner)
                continue
            approved = []
            for identity in wanted:
                obj = matches[identity]
                loc = state["located"].get(str(obj.track_id)) if cal is not None else None
                handeye_ok = bool(loc and loc.get("ok"))
                grasps = grasp_candidates_for_obj(obj, pick_x_offset_mm)
                if not grasps and not handeye_ok:
                    return {"ok": False, "code": "grasp_not_ready",
                            "reason": f"ID {identity} không còn grasp hợp lệ."}
                base_valid = bool(obj.base_pose_valid and
                                  obj.base_pose.header.frame_id == "base_link")
                if handeye_ok:
                    # Search-center geometry: pixel + measured joints + hand-eye,
                    # intersected with the plane of the layer the cube is on.
                    # Yaw is the base-frame heading re-expressed as seen from
                    # READY (the image yaw is only valid at READY_POSE).
                    grasp = handeye_grasp(loc, grasps[0] if grasps else None)
                elif use_metric(obj):
                    grasp = dict(grasps[0])
                    base_stamp = obj.base_pose.header.stamp.sec + obj.base_pose.header.stamp.nanosec * 1e-9
                    if abs(base_stamp - state["scene_stamp"]) > .001:
                        return {"ok": False, "code": "stack_geometry_unavailable", "reason": "Base pose đã cũ."}
                    metric, why = metric_top_grasp(pose_matrix(obj.base_pose).tolist(),
                                                   reference_z)
                    if metric is None:
                        return {"ok": False, "code": "grasp_not_ready",
                                "reason": f"ID {identity}: {why}"}
                    metric.pop("preferred_yaw_rad", None)  # keep the image-based yaw convention
                    grasp.update(metric)
                else:
                    grasp = dict(grasps[0])
                    grasp["coordinate_source"] = "fixed_ready_pose"
                approved.append({
                    "object_kind": "cube", "cube_id": identity,
                    "track_id": str(obj.track_id),
                    "geometry_model_id": int(obj.geometry_model_id),
                    "identity_source": "apriltag" if "apriltag" in str(obj.pose_method)
                                       else "geometry_3d",
                    "identity_confidence": float(obj.identity_confidence),
                    "pose_confidence": float(obj.pose_confidence),
                    "pose_method": str(obj.pose_method),
                    "bbox_xyxy": [int(v) for v in obj.bbox_xyxy],
                    "base_pose_valid": base_valid,
                    "base_position": (pose_matrix(obj.base_pose)[:3, 3].tolist()
                                      if base_valid else None),
                    "grasp": grasp,
                    "calibrated": state["calibrated"],
                })
            approved_at = time.time()
            return {"ok": True, "status": "approved",
                    "approval_token": uuid.uuid4().hex,
                    "approved_at": approved_at, "expires_at": approved_at + 15.0,
                    "scene_stamp": state["scene_stamp"], "operation": operation,
                    "objects": approved}
        return {"ok": False, "code": "approval_timeout",
                "reason": "Hết thời gian duyệt: " + diagnostic}
    except cv2.error as exc:
        return {"ok": False, "code": "viewer_unavailable", "reason": str(exc)}
    finally:
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass
        node.destroy_node()
        rclpy.shutdown()


def main():
    try:
        request = json.load(sys.stdin)
        if request.get("command") == "snapshot":
            result = snapshot(min(8.0, max(0.5, float(request.get("timeout_s", 4.0)))),
                              request.get("expect"))
        elif request.get("command") == "preflight":
            result = preflight(request.get("targets"),
                               request.get("placement_correction_gripper_xy_m"),
                               request.get("handeye_place_correction_gripper_xy_m"))
        elif request.get("command") == "approve":
            result = approve(request.get("object_ids", []),
                             request.get("operation", "pick_hold"),
                             request.get("timeout_s", 0.0),
                             request.get("pick_x_offset_mm", 15.0),
                             request.get("notice", ""),
                             bool(request.get("handeye", False)))
        elif request.get("command") == "zone_survey":
            result = zone_survey(request.get("zones", []),
                                 min(14.0, max(1.0, float(request.get("timeout_s", 6.0)))),
                                 request.get("expect_j1"))
        else:
            raise ValueError("Only snapshot/preflight/approve/zone_survey commands are supported")
    except Exception as exc:
        result = {"ok": False, "reason": str(exc)}
    print("T8_SCENE:" + json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
