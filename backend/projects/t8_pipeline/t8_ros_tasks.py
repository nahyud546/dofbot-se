"""Grounded ROS 3D previews and explicitly approved motion transactions."""

import json
import math
import sys
import time
from pathlib import Path

_PROJECTS = str(Path(__file__).resolve().parents[1])
if _PROJECTS not in sys.path:
    sys.path.insert(0, _PROJECTS)

from t8_scene import normalize_cube_label


try:
    from cube_identity import COLOR_TO_ID as COLOR_IDS, ID_NAMES_VI as COLOR_NAMES
except ImportError:
    from .cube_identity import COLOR_TO_ID as COLOR_IDS, ID_NAMES_VI as COLOR_NAMES  # type: ignore
READ_TOOLS = {"observe_scene", "robot_status", "capabilities"}
STACK_TOP_REFERENCES = {"them", "it", "that", "top", "tren cung", "trên cùng",
                        "dinh chong", "đỉnh chồng", "chong do", "chồng đó"}
STACK_HOVER_CLEARANCE_M = .015
COORDINATE_SOURCES = {"handeye": "handeye_tag_plane", "metric": "calibrated_base_pose",
                      "fixed_ready_pose": "fixed_ready_pose"}


def stack_calibration_error(scene):
    """Explain why the live ROS scene cannot support a metric stack."""
    if scene.get("calibrated"):
        return ""
    status = str(scene.get("status") or "unknown")
    if status.startswith("timestamped base-to-camera TF unavailable"):
        return ("Xếp chồng cần pose base_link cùng timestamp với ảnh. "
                "Camera đã hiệu chuẩn nhưng chưa có TF lúc chụp; kiểm tra "
                "/real_joint_states và measured/Camera_Link. ROS báo: " + status)
    return ("Xếp chồng cần pose base_link đã hiệu chuẩn; ROS hiện báo: "
            f"{status}. Cấu hình cube_6d_urdf.yaml mặc định chưa hiệu chuẩn "
            "camera/hand-eye. Hãy đo và kiểm chứng cấu hình, rồi chạy perception "
            "với config:=<file.yaml> (hoặc T8 --ros-config <file.yaml>). "
            "Nếu đã dùng cấu hình hiệu chuẩn, kiểm tra TF/joint đúng timestamp.")


def cube_id(label):
    canonical = normalize_cube_label(label)
    if canonical:
        return int(canonical[-1])
    return COLOR_IDS.get(str(label).strip().lower())


def scan_proposal():
    return {"mode": "proposal_only", "joint": 1,
            "servo_targets_deg": [90, 70, 110, 50, 130, 30, 150],
            "requires_path_validation": True,
            "capture_after_stationary": True,
            "motor_commands_sent": 0}


def capability_context(mode="ros3d_dry_run"):
    try:
        from cube_identity import identity_context
    except ImportError:
        from .cube_identity import identity_context  # type: ignore
    base_tools = ["observe_scene", "robot_status", "capabilities", "search_object",
                  "detection_mode", "vision_pick_hold", "sort_cube", "stack_cubes"]
    if mode == "legacy2d_motion":
        return {
            "mode": mode,
            "tools": ["observe_scene", "robot_status", "capabilities",
                      "search_object", "vision_pick_hold",
                      "rotate_relative", "arm_pose", "pose_start", "gripper"],
            "physical_execution_enabled": True,
            "cube_identity": identity_context(),
            "restrictions": ["color-inferred ID không được dùng để gắp/xếp",
                             "pick/stack cube_ID cần tag36h11 xác thực 2 frame",
                             "câu hỏi chỉ trả lời, không di chuyển tay"],
        }
    physical = mode == "ros3d_confirmed_motion"
    if physical:
        base_tools += ["place_held", "release_hold", "pose_start",
                       "arm_pose", "gripper"]
    return {
        "mode": mode,
        "tools": base_tools,
        "physical_execution_enabled": physical,
        "approval": "viewer_space_required" if physical else "none",
        "cube_identity": identity_context(),
        "restrictions": ["ID must come from perception; geometry alone is not identity",
                         "color-inferred ID chỉ trả lời, không gắp/xếp",
                         "scan is a proposal; no robot rotation in this mode",
                         ("pick/stack require a fresh target lock and Space approval"
                          if physical else
                          "pick/stack return IK/FK previews, not successful grasps")],
    }


CUBE_EDGE_M = 0.030
PIXEL_VERIFY_TOL_PX = 45.0     # khung bao của cube nguồn lệch tâm kỳ vọng tối đa (~10 mm trên bàn)


STACK_CONTACT_TOL_M = 0.020          # xếp tầng chỉ cần cube nguồn nằm trên cube đích (sai số tay/camera)
PLACEMENT_LABELS = {
    "in_zone": "gắp chính xác, cube nằm trong ô",
    "partial": "lệch, cube đè lên mép ô",
    "outside": "lệch, cube rơi ngoài ô",
    "missing": "ô không có cube, có thể bị văng ra ngoài khung hình",
    "unseen": "chưa xác nhận được cube trong ô",
}


class RosTaskRunner:
    MAX_PREFLIGHT_ATTEMPTS = 5

    def __init__(self, backend, motion=None, log_path=None, clock=time.monotonic,
                 placement_verifier=None, max_retries=2):
        self.backend = backend
        self.motion = motion
        self.placement_verifier = placement_verifier      # cube_vision.placement_check.PlacementVerifier
        self.max_retries = max_retries
        self.attempt = 0                        # 0 = lần đầu; >0 = thử lại sau khi cube lệch/văng (thả nhẹ tay hơn)
        self.zone_cache = None                  # kết quả khảo sát zone dùng chung cho cả lô (None = mỗi cube tự khảo sát)
        self.batch_approval_timeout = 180.0     # giây chờ Space mỗi cube trong lô; hết hạn thì bỏ cube đó, không treo
        self.batch_progress = None
        self.batch_deadline_s = 1800.0          # tổng thời gian tối đa của một lô; quá hạn thì dừng sau bước đang làm
        self.clock = clock
        self.memory = {}
        self.log_path = log_path
        self.latest = None
        self.last_stack_top_id = None

    def refresh(self):
        scene = self.backend.snapshot()
        if not scene.get("ok"):
            raise RuntimeError(scene.get("reason", "Không nhận được scene ROS 3D"))
        now = self.clock()
        for obj in scene.get("objects", []):
            identity = obj.get("object_id")
            if identity not in COLOR_NAMES or obj.get("geometry_model_id") != identity:
                continue
            if obj.get("age_s", math.inf) > 2.0 or obj.get("age_s", -1) < 0:
                continue
            # Do not retain an old metric pose when a new observation rejects it.
            self.memory[identity] = {**obj, "seen_at": now - obj["age_s"],
                                     "calibrated": bool(scene.get("calibrated"))}
        self.latest = scene
        return scene

    def describe_scene(self, scene):
        """Cube đang thấy (có pose camera hợp lệ) tách khỏi vật chỉ có màu/ID mà chưa có pose.

        Perception gắn ID cho cả ô màu trên thảm (ô đỏ/xanh) nên track không có pose không được
        coi là cube đang nằm đó.
        """
        seen, doubtful = {}, {}
        for obj in scene.get("objects", []):
            identity = obj.get("object_id")
            if identity not in COLOR_NAMES or obj.get("geometry_model_id") != identity:
                continue
            if not (0 <= obj.get("age_s", math.inf) <= 2.0):
                continue
            he = obj.get("handeye") or {}
            if he.get("ok") or obj.get("camera_pose_valid"):
                best = seen.get(identity)
                if best is None or (he.get("ok") and not (best.get("handeye") or {}).get("ok")):
                    seen[identity] = obj
            else:
                doubtful[identity] = obj
        parts = []
        for identity in sorted(seen):
            obj, he = seen[identity], seen[identity].get("handeye") or {}
            text = f"ID {identity} ({COLOR_NAMES[identity]})"
            if he.get("ok"):
                text += (f" tại x={he['x'] * 1000:.0f}, y={he['y'] * 1000:.0f} mm, "
                         f"tầng {he['layer'] + 1}")
            else:
                text += " (có pose camera nhưng chưa tính được vị trí/tầng)"
            parts.append(text)
        reply = "Quan sát mới: " + (", ".join(parts) if parts else "chưa có cube được xác thực ID.")
        unseen = [f"ID {i} ({COLOR_NAMES[i]})" for i in sorted(doubtful) if i not in seen]
        if unseen:
            reply += (" Nghi vấn (có màu/ID nhưng chưa có pose, có thể là ô màu trên thảm hoặc cube bị che): "
                      + ", ".join(unseen) + ".")
        return reply

    def resolve(self, label, allow_memory=False):
        raw = str(label or "").strip().lower()
        identity = (self.last_stack_top_id if raw in STACK_TOP_REFERENCES
                    else cube_id(label))
        if identity is None:
            return None, "Nhãn chưa có ID xác thực; hãy nêu màu hoặc cube ID 1–4."
        matches = [obj for obj in self.latest.get("objects", [])
                   if obj.get("object_id") == identity and
                   obj.get("geometry_model_id") == identity and
                   0 <= obj.get("age_s", math.inf) <= 2.0]
        if len(matches) > 1:
            # Perception gắn ID cả cho ô màu/vật không có pose (identity-only track). Chỉ track có pose
            # camera hợp lệ mới là cube thật; còn đúng một track như vậy thì không mơ hồ.
            with_pose = [obj for obj in matches
                         if obj.get("camera_pose_valid") or (obj.get("handeye") or {}).get("ok")]
            if len(with_pose) == 1:
                matches = with_pose
            else:
                return None, f"Có nhiều track cùng ID {identity}; cần xác nhận vật đích."
        if len(matches) == 1:
            return self.memory.get(identity), ""
        remembered = self.memory.get(identity)
        if allow_memory and remembered and self.clock() - remembered["seen_at"] <= 15.0:
            return remembered, ""
        return None, f"Chưa thấy cube ID {identity} ({COLOR_NAMES[identity]}) trong quan sát mới."

    def metric_error(self, obj):
        if not obj.get("calibrated"):
            return "Camera/hand-eye chưa được kiểm chứng; chưa có tọa độ base đáng tin."
        if not obj.get("base_pose_valid") or not obj.get("grasp"):
            return obj.get("reason") or "Thiếu pose base hoặc mặt gắp phù hợp."
        if not obj.get("confirmed") or not obj.get("stationary"):
            return obj.get("reason") or "Cần hai quan sát nhất quán khi tay đứng yên."
        if self.clock() - obj["seen_at"] > 15.0:
            return "Quan sát lưu trong bộ nhớ đã hết hạn."
        return ""

    def fixed_pose_error(self, obj):
        """Safety gate for the proven READY_POSE image-to-TCP mapping."""
        if not obj.get("grasp"):
            return obj.get("reason") or "Thiếu mặt gắp phù hợp ở pose quan sát."
        if not obj.get("confirmed") or not obj.get("stationary"):
            return obj.get("reason") or "Cần hai quan sát ID/pose nhất quán."
        if self.clock() - obj["seen_at"] > 2.0:
            return "Quan sát fixed-pose đã cũ; cần nhìn lại trước khi duyệt."
        return ""

    def wait_for_labels(self, labels, timeout_s=25.0):
        """Allow a freshly launched DINO/perception process to warm up."""
        deadline = self.clock() + timeout_s
        last_scene = None
        while self.clock() < deadline:
            last_scene = self.refresh()
            resolved = [self.resolve(label)[0] for label in labels]
            if all(item is not None for item in resolved):
                return last_scene
            time.sleep(0.2)
        return last_scene or self.refresh()

    def execute_approved_motion(self, intent, ent):
        """Check fixed prerequisites before preparing; Space locks the target."""
        source_label = ent.get("source") if intent == "stack_cubes" else ent.get("label")
        source_id = cube_id(source_label)
        if source_id is None:
            return {"ok": False, "code": "unknown_selector",
                    "reply": "Nguồn chưa xác định được cube ID 1-4; chưa mở viewer."}
        wanted = [int(source_id)]
        target_id = None
        if intent == "stack_cubes":
            target_raw = str(ent.get("target") or "").strip().lower()
            target_id = (self.last_stack_top_id if target_raw in STACK_TOP_REFERENCES
                         else cube_id(ent.get("target")))
            if target_id is None:
                return {"ok": False, "code": "unknown_selector",
                        "reply": "Đích chưa xác định được cube ID hoặc đỉnh chồng đã nhớ."}
            if int(target_id) == int(source_id):
                return {"ok": False, "code": "same_object",
                        "reply": "Nguồn và đích có cùng cube ID; chưa mở viewer."}
            wanted.append(int(target_id))
        operation = ("stack" if intent == "stack_cubes" else
                     "sort" if intent == "sort_cube" else "pick_hold")
        try:
            state = json.loads(Path("/tmp/t8_hold_state.json").read_text())
        except (OSError, ValueError):
            state = {}
        if state.get("phase") in {"holding", "moving"}:
            return {"ok": False, "code": "robot_not_empty",
                    "reply": "Tay đang giữ vật hoặc motion trước dở dang; cần xử lý trạng thái trước khi gắp."}
        coordinate_mode = "metric"
        calibration_scene = None
        for _ in range(3):
            try:
                calibration_scene = self.backend.snapshot()
            except Exception as exc:
                if target_id is not None:
                    raise
                # Single picks can still use the fixed map from READY_POSE.
                calibration_scene = {"ok": False, "reason": str(exc)}
            if not calibration_scene.get("ok") or calibration_scene.get("calibrated"):
                break
            status = str(calibration_scene.get("status") or "")
            # A validated camera can temporarily lack an exact-stamp TF.
            # Retry that case; a configuration known to be uncalibrated
            # cannot become metric just by waiting for another frame.
            if not (status.startswith("timestamped base-to-camera TF unavailable")
                    or status.startswith("calibration or timestamped TF required")):
                break
        if not calibration_scene.get("ok"):
            if target_id is not None:
                return {"ok": False, "code": "observation_unavailable",
                        "reply": calibration_scene.get("reason", "Chưa nhận được scene ROS 3D.")}
            calibration_scene = {"calibrated": False}
        joints_alive = (calibration_scene.get("robot") or {}).get("joint_positions_rad") is not None
        if calibration_scene.get("handeye_available") and joints_alive:
            # Pixel + measured joints + hand-eye (search-center geometry) with
            # layer detection: valid from any stationary arm pose.
            coordinate_mode = "handeye"
            print("[geometry] Hand-eye đã hiệu chuẩn: tính XY + tầng từ khớp thật, "
                  "không cần READY_POSE.", flush=True)
        elif not calibration_scene.get("calibrated"):
            # The fixed image-to-XY map was tuned at READY_POSE and does
            # not depend on dynamic hand-eye calibration.  prepare below
            # verifies the actual joints before the map can be used.
            coordinate_mode = "fixed_ready_pose"
            why = ("hand-eye đã hiệu chuẩn nhưng chưa có /real_joint_states"
                   if calibration_scene.get("handeye_available") else "chưa có hand-eye calibration")
            print(f"[geometry] {why}; dùng mapping fixed READY_POSE.", flush=True)
        # Calibrated base_link poses are valid from any stationary arm pose, so
        # the arm stays where it is. The fixed map still requires READY_POSE.
        at_ready = coordinate_mode == "fixed_ready_pose"
        print("[pose] Chuẩn bị pose quan sát trước khi duyệt target."
              if at_ready else
              "[pose] Scene đã hiệu chuẩn: giữ nguyên pose hiện tại để quan sát.", flush=True)
        try:
            prepared = (self.motion.execute("prepare") if at_ready else
                        self.motion.execute("prepare", keep_pose=True))
        finally:
            if hasattr(self.motion, "close"):
                self.motion.close()
        if not prepared.get("ok"):
            return {"ok": False, "code": "prepare_failed", "reply": prepared.get("reply", "Prepare thất bại.")}
        zone_survey_result = None
        if (intent == "sort_cube" and coordinate_mode == "handeye" and
                getattr(self.backend, "zone_check", True) and
                hasattr(self.backend, "zone_survey") and self.motion is not None):
            # Camera tay xoay hai phía đo vị trí các ô so với điểm thả cấu hình; kết quả chỉ dùng
            # cho lệnh sort này (không lưu). Luôn quay về pose xuất phát trước khi mở viewer.
            if self.zone_cache is not None:
                zone_survey_result = self.zone_cache            # khảo sát một lần cho cả lô
            else:
                print(f"[zone] Khảo sát ô thả của cube {source_id} (xoay camera tới khu vực ô)...", flush=True)
                from zone_survey import run_survey
                zone_survey_result = run_survey(self.motion, self.backend, zones=(int(source_id),))
            if not zone_survey_result["restored"]:
                return {"ok": False, "code": "zone_survey_restore_failed",
                        "reply": "Đã khảo sát zone nhưng tay không quay về được pose xuất phát; "
                                 "kiểm tra tay máy rồi thử lại."}
            blocked = next((r for r in zone_survey_result["report"]
                            if r["zone_id"] == int(source_id) and r["action"] == "blocked"), None)
            if blocked:
                # Thả theo bảng cấu hình sẽ bỏ cube vào sai ô: dừng, tay đã về pose xuất phát.
                return {"ok": False, "code": "zone_unavailable",
                        "reply": f"Không thả cube ID {source_id}: {blocked['text']}",
                        "zone_survey": zone_survey_result["report"]}
        approval_operation = "stack_fixed" if (
            operation == "stack" and coordinate_mode == "fixed_ready_pose") else operation
        # Approval and IK are one human-in-the-loop transaction.  A rejected
        # IK sample is not a completed command: reopen the live viewer so the
        # operator can move the cube, wait for a better smoothed estimate, and
        # press Space again.  Esc/closing the viewer remains the explicit exit.
        preflight_notice = ""
        fixed_mode_notice = (
            "Fixed READY_POSE chỉ tính độ cao cube đặt trực tiếp trên bàn; "
            "chỉ Space khi nguồn và mặt đích đều ở tầng bàn. "
            "Tầng tiếp theo cần scene đã hiệu chuẩn."
            if target_id is not None and coordinate_mode == "fixed_ready_pose" else "")
        preflight_attempts = 0
        previous_failure_signature = None
        same_failure_count = 0
        stack_layers = 1
        while True:
            approval = self.backend.approve(
                wanted, operation=approval_operation,
                notice=" ".join(value for value in (fixed_mode_notice, preflight_notice)
                                if value),
                **({"handeye": True} if coordinate_mode == "handeye" else {}))
            if not approval.get("ok"):
                return {"ok": False, "code": approval.get("code", "approval_failed"),
                        "reply": approval.get("reason", "Target chưa được xác nhận.")}
            if time.time() > float(approval.get("expires_at", 0)):
                print("[preflight] Target lock đã hết hạn; đang mở lại viewer.", flush=True)
                continue
            locks = approval.get("objects", [])
            if len(locks) != len(wanted) or any(
                    lock.get("object_kind") != "cube" or
                    lock.get("cube_id") != lock.get("geometry_model_id") or
                    lock.get("identity_source") not in {"apriltag", "geometry_3d"} or
                    lock.get("cube_id") != wanted[index]
                    for index, lock in enumerate(locks)):
                return {"ok": False, "code": "identity_conflict",
                        "reply": "Target duyệt không còn bằng chứng ID 3D nhất quán; chưa di chuyển."}

            source, target = locks[0], locks[1] if len(locks) == 2 else None
            expected_source = COORDINATE_SOURCES[coordinate_mode]
            if not at_ready and any(
                    lock.get("grasp", {}).get("coordinate_source") != expected_source
                    for lock in locks):
                # The lock fell back to the fixed map (no current base pose):
                # that map is only valid from READY_POSE, so go there and re-approve.
                print("[pose] Target không có pose base hiện hành; về READY_POSE rồi duyệt lại.",
                      flush=True)
                try:
                    prepared = self.motion.execute("prepare")
                finally:
                    if hasattr(self.motion, "close"):
                        self.motion.close()
                if not prepared.get("ok"):
                    return {"ok": False, "code": "prepare_failed",
                            "reply": prepared.get("reply", "Prepare thất bại.")}
                at_ready = True
                coordinate_mode = "fixed_ready_pose"
                if operation == "stack":
                    approval_operation = "stack_fixed"
                continue
            if target is not None and any(
                    lock.get("grasp", {}).get("coordinate_source") != expected_source
                    for lock in locks):
                return {"ok": False, "code": "stack_geometry_unavailable",
                        "reply": "Nguồn tọa độ target không khớp chế độ xếp đã duyệt."}
            if target is not None:
                surface = str(target.get("grasp", {}).get("surface_id", ""))
                if surface != "+Z":
                    return {"ok": False, "code": "wrong_target_face",
                            "reply": (f"Đích ID {target.get('cube_id')} đang cho mặt hông "
                                      f"{surface}, không phải mặt trên +Z. "
                                      "Cube đáy bị che (xanh dưới đỏ) không xếp được; "
                                      "hãy đặt lên ID đỉnh chồng (đỏ/ID 3 hoặc nói "
                                      "'on top') rồi Space lại.")}
            targets = []
            for stage, clearance in (("approach_pick", .020), ("pick", 0.0),
                                     ("lift_pick", .030)):
                grasp = dict(source["grasp"])
                grasp["tcp_position_base"] = list(grasp["tcp_position_base"])
                grasp["tcp_position_base"][2] += clearance
                targets.append({"stage": stage, **grasp})
            if target is not None:
                place = dict(target["grasp"])
                place["tcp_position_base"] = list(place["tcp_position_base"])
                # Target TCP already follows its measured layer in handeye mode;
                # the carried cube sits one cube height above it.
                stack_layers = int(target["grasp"].get("layer", 0)) + 1
                place["tcp_position_base"][2] += .030
                hover_clearance = STACK_HOVER_CLEARANCE_M
                hover = dict(place)
                hover["tcp_position_base"] = list(place["tcp_position_base"])
                hover["tcp_position_base"][2] += hover_clearance
                targets += [{"stage": "hover_place", **hover}, {"stage": "place", **place}]
            check = self.backend.preflight(targets)
            if not check.get("ok"):
                reason = check.get("reason", "IK/FK không đạt")
                preflight_notice = "IK/FK chưa đạt: " + reason
                preflight_attempts += 1
                print("[preflight] IK/FK chưa đạt: " + reason, flush=True)
                failed_tcp = check.get("failed_tcp_position_base") or targets[-1].get(
                    "tcp_position_base", [])
                signature = (tuple(wanted), check.get("failed_stage"),
                             tuple(round(float(v), 2) for v in failed_tcp
                                   if isinstance(v, (int, float))))
                same_failure_count = (same_failure_count + 1
                                      if signature == previous_failure_signature else 1)
                previous_failure_signature = signature
                if same_failure_count >= 2 or preflight_attempts >= self.MAX_PREFLIGHT_ATTEMPTS:
                    return {"ok": False, "code": "preflight_failed",
                            "reply": (preflight_notice + " Cùng stage/tọa độ vẫn không có "
                                      "nghiệm sau khi xác nhận lại; chưa di chuyển robot. "
                                      "Hãy đổi vị trí cube hoặc hiệu chuẩn mapping fixed-pose.")}
                print("[preflight] Chưa di chuyển robot; viewer sẽ mở lại. "
                      "Nếu đã dời cube, chờ pose mới ổn định rồi Space; Esc để hủy.",
                      flush=True)
                continue
            if time.time() > float(approval.get("expires_at", 0)):
                print("[preflight] Target lock hết hạn trong lúc kiểm tra; đang mở lại viewer.",
                      flush=True)
                continue
            break

        command = ("stack_cube_3d" if target is not None else
                   "sort_cube_3d" if intent == "sort_cube" else "pick_cube_3d")
        payload = {"approval_token": approval["approval_token"],
                   "approved_at": approval["approved_at"],
                   "preflight_ok": True, "preflight_plan": check.get("stages", []),
                   "source": source}
        if target is not None:
            payload["target"] = target
            payload["coordinate_mode"] = coordinate_mode
            payload["stack_layers"] = stack_layers
            place_stage = next((item for item in check.get("stages", [])
                                if item.get("stage") == "place"), {})
            payload["placement_offset_base_xy"] = place_stage.get(
                "placement_offset_base_xy", [0.0, 0.0])
            payload["hover_clearance_m"] = (hover_clearance
                                            if 'hover_clearance' in locals()
                                            else STACK_HOVER_CLEARANCE_M)
        if command == "sort_cube_3d":
            grasp = source["grasp"]
            payload.update(cube_id=source["cube_id"],
                           track_id=source.get("track_id", ""),
                           pose_method=source.get("pose_method", "unknown"),
                           tcp_position_base=grasp["tcp_position_base"],
                           surface_id=grasp.get("surface_id", "unknown"),
                           surface_normal_base=grasp.get("surface_normal_base", [0, 0, 1]),
                           approach_vector_base=grasp.get("approach_vector_base", [0, 0, -1]),
                           preferred_yaw_rad=grasp.get("preferred_yaw_rad", 0.0),
                           coordinate_source=grasp.get("coordinate_source", "fixed_ready_pose"),
                           gripper_width_m=grasp.get("gripper_width_m", .033),
                           quality_score=grasp.get("quality_score", 0.0))
            if zone_survey_result and zone_survey_result["targets"]:
                payload["zone_target_xy"] = {
                    str(zone): xy for zone, xy in zone_survey_result["targets"].items()}
            if self.attempt:
                payload["release_gentleness"] = min(self.attempt, 2)
        placement_ready = False
        if command == "sort_cube_3d" and self.placement_verifier is not None:
            placement_ready = bool(self.placement_verifier.before())      # ảnh camera ngoài TRƯỚC khi thả
            if not placement_ready:
                print("[verify] camera ngoài không cho ảnh: sẽ không kiểm tra được ô thả.", flush=True)
        try:
            moved = self.motion.execute(command, **payload)
        finally:
            if hasattr(self.motion, "close"):
                self.motion.close()
        moved.setdefault("actual_grasp_verified", False)
        if placement_ready and moved.get("ok") and command == "sort_cube_3d":
            check = self.placement_verifier.check(int(source["cube_id"]))
            moved["placement_check"] = check
            label = PLACEMENT_LABELS[check["verdict"]]
            target_xy = ((zone_survey_result or {}).get("targets") or {}).get(int(source["cube_id"]))
            if check.get("cube_xy") and target_xy:
                # Camera ngoài đã hiệu chuẩn: cube cách tâm ô đo được bao nhiêu (thông tin thêm, không đổi kết luận).
                offset = math.hypot(check["cube_xy"][0] - target_xy[0], check["cube_xy"][1] - target_xy[1])
                check["offset_mm"] = round(offset * 1000.0, 1)
                label += f", cách tâm ô {offset * 1000:.0f} mm"
            print(f"[verify] Zone {source['cube_id']}: {label}", flush=True)
            moved["reply"] = f"{moved.get('reply', '')} [Camera ngoài: {label}]".strip()
        if zone_survey_result and moved.get("ok") and command == "sort_cube_3d":
            zone_note = next((r["text"] for r in zone_survey_result["report"]
                              if r["zone_id"] == int(source["cube_id"])), "")
            moved["zone_survey"] = zone_survey_result["report"]
            if zone_note:
                moved["reply"] = f"{moved.get('reply', '')} [{zone_note}]".strip()
        moved["approval_token"] = approval["approval_token"]
        moved["detail"] = {"approved": locks, "preflight": check,
                           "worker": moved.get("detail", {})}
        if (moved.get("ok") and command == "stack_cube_3d" and
                coordinate_mode == "fixed_ready_pose"):
            moved["stack_verified"] = False
            moved["status"] = "executed_unverified"
            self.last_stack_top_id = None
            moved["reply"] = (
                "Robot đã chạy hết chuyển động gắp/đặt và xác nhận khớp/kẹp, "
                "nhưng camera fixed-pose chưa xác minh cube còn đứng trên đích. "
                "Chưa ghi nhận tầng hoặc cube trên cùng; hãy kiểm tra viewer. "
                "Xếp tầng tiếp theo cần scene đã hiệu chuẩn để xác minh vị trí.")
        elif moved.get("ok") and command == "stack_cube_3d":
            verification = self.verify_stack(
                source["cube_id"], target["cube_id"],
                max(approval["scene_stamp"], time.time()),
                expected=(target["grasp"]["tcp_position_base"][:2],
                          int(target["grasp"].get("layer", 0)) + 1,
                          target["grasp"].get("top_z"))
                if coordinate_mode == "handeye" else None)
            moved["detail"]["verification"] = verification
            moved["stack_verified"] = verification["ok"]
            if verification["ok"]:
                self.last_stack_top_id = source["cube_id"]
                moved["status"] = "scene_verified"
            else:
                self.last_stack_top_id = None
                moved["reply"] = (moved.get("reply", "Đã gửi chuyển động xếp.") +
                                  " Scene chưa xác minh chồng; dừng chuỗi. Lý do: " +
                                  str(verification.get("reason", "?")))
        return moved

    def verify_stack(self, source_id, target_id, after_stamp, timeout_s=8.0, expected=None):
        """Check the carried cube ended on the target.

        expected=(target_xy, layer): hand-eye mode. The target's own tag is
        covered after stacking, so look for the SOURCE at the target XY and one
        layer higher instead of requiring both cubes to be visible.
        """
        deadline = self.clock() + timeout_s
        reason = "Chưa thấy đủ nguồn và đích sau motion."
        expect = None
        if expected is not None and len(expected) >= 3 and expected[2] is not None:
            # Tâm cube nguồn khi nằm đúng trên đích: mặt trên cao top_z đích + 30 mm, tâm thấp hơn 15 mm.
            expect = [{"object_id": source_id,
                       "xyz": [expected[0][0], expected[0][1], float(expected[2]) + CUBE_EDGE_M - CUBE_EDGE_M / 2]}]
        while self.clock() < deadline:
            try:
                try:
                    scene = self.backend.snapshot(expect=expect) if expect else self.backend.snapshot()
                except TypeError:
                    scene = self.backend.snapshot()
            except Exception as exc:
                return {"ok": False, "reason": f"Quan sát sau motion thất bại: {exc}"}
            if not scene.get("ok"):
                return {"ok": False, "reason": scene.get("reason", "Scene unavailable")}
            if expected is not None:
                if scene.get("stamp", 0) > after_stamp:
                    found = [o for o in scene.get("objects", [])
                             if o.get("object_id") == source_id
                             and o.get("geometry_model_id") == source_id
                             and o.get("confirmed") and o.get("stationary")
                             and 0 <= o.get("age_s", math.inf) <= 2
                             and (o.get("handeye") or {}).get("ok")]
                    if len(found) == 1:
                        located = found[0]["handeye"]
                        offset = math.hypot(located["x"] - expected[0][0],
                                            located["y"] - expected[0][1])
                        if offset <= STACK_CONTACT_TOL_M and located["layer"] == expected[1]:
                            return {"ok": True, "source": found[0], "offset_mm": round(offset * 1000, 1),
                                    "layer": located["layer"]}
                        reason = (f"Nguồn nằm cách đích {offset * 1000:.0f} mm, tầng "
                                  f"{located['layer']} (cần ≤{STACK_CONTACT_TOL_M * 1000:.0f} mm, tầng {expected[1]}).")
                    else:
                        reason = f"Chưa thấy lại cube nguồn ID {source_id} sau motion."
                    # Mặt trên là mặt màu/ảnh thì pose RGB nghiêng, hand-eye từ chối: dùng vị trí pixel.
                    by_pixel = [o for o in scene.get("objects", [])
                                if o.get("object_id") == source_id
                                and o.get("geometry_model_id") == source_id
                                and 0 <= o.get("age_s", math.inf) <= 2
                                and o.get("expected_px_error") is not None]
                    close = [o for o in by_pixel if o["expected_px_error"] <= PIXEL_VERIFY_TOL_PX]
                    if len(close) >= 1 and not [o for o in found if (o.get("handeye") or {}).get("ok")]:
                        best = min(close, key=lambda o: o["expected_px_error"])
                        return {"ok": True, "source": best, "method": "pixel",
                                "pixel_error": best["expected_px_error"]}
                    if by_pixel:
                        reason += (f" Theo pixel, cube nguồn cách vị trí kỳ vọng "
                                   f"{min(o['expected_px_error'] for o in by_pixel):.0f}px "
                                   f"(cần ≤{PIXEL_VERIFY_TOL_PX:.0f}).")
                time.sleep(.2)
                continue
            if scene.get("calibrated") and scene.get("stamp", 0) > after_stamp:
                matches = {}
                for identity in (source_id, target_id):
                    found = [o for o in scene.get("objects", []) if o.get("object_id") == identity
                             and o.get("geometry_model_id") == identity and o.get("confirmed")
                             and o.get("stationary") and o.get("base_pose_valid")
                             and 0 <= o.get("age_s", math.inf) <= 2]
                    if len(found) == 1:
                        matches[identity] = found[0]
                if len(matches) == 2:
                    a, b = matches[source_id]["base_position"], matches[target_id]["base_position"]
                    if math.hypot(a[0] - b[0], a[1] - b[1]) <= .012 and abs(a[2] - b[2] - .030) <= .010:
                        return {"ok": True, "source": matches[source_id], "target": matches[target_id]}
                    reason = "Vị trí nguồn/đích chưa khớp một tầng cube 30 mm."
            time.sleep(.2)
        return {"ok": False, "reason": reason}

    @staticmethod
    def needs_retry(result):
        """Kết quả đã chạy xong nhưng cube không ở đích: sort lệch/rơi ngoài ô (camera ngoài), hoặc
        xếp tầng mà scene đã xác minh là cube không nằm trên đích. 'unseen' = không kết luận được: không thử lại."""
        check = result.get("placement_check")
        if result.get("ok") and check and check.get("verdict") in ("partial", "outside", "missing"):
            return ("cube vào ô rồi bị văng ra" if check.get("bounced") else PLACEMENT_LABELS[check["verdict"]])
        if RosTaskRunner.stack_failed_verification(result):
            return "cube không nằm trên cube đích"
        return ""

    @staticmethod
    def stack_failed_verification(result):
        """Scene đã kiểm và cube nguồn KHÔNG nằm trên đích. (Chế độ fixed-pose không có kiểm -> không tính.)
        Worker luôn trả status executed_unverified nên phân biệt bằng detail.verification do runner ghi."""
        return bool(result.get("ok") and result.get("stack_verified") is False
                    and "verification" in (result.get("detail") or {}))

    def _run_with_retries(self, intent, entities):
        """Một bước: chạy, kiểm tra camera ngoài/scene, lệch hoặc rơi thì thử lại tối đa max_retries lần."""
        try:
            result = self._execute(intent, entities)
        except Exception as exc:
            result = {"ok": False, "code": "observation_unavailable", "reply": str(exc)}
        retries = 0
        self.attempt = 0
        while self.motion is not None and (why := self.needs_retry(result)):
            if retries >= self.max_retries:
                result["reply"] = (f"{result.get('reply', '')} Đã thử lại {retries} lần vẫn chưa đúng "
                                   f"({why}); dừng, hãy kiểm tra bằng mắt.").strip()
                result["retries_exhausted"] = True
                break
            retries += 1
            self.attempt = retries           # lần thử lại thả nhẹ hơn: thấp hơn và chậm hơn lần trước
            print(f"[verify] {why}: gắp lại và đặt lại ({retries}/{self.max_retries}), "
                  f"thả nhẹ tay mức {min(retries, 2)}; tìm lại cube rồi duyệt viewer.", flush=True)
            try:
                result = self._execute(intent, entities)
            except Exception as exc:
                result = {"ok": False, "code": "observation_unavailable", "reply": str(exc)}
                break
        if retries:
            result["retries"] = retries
        return result

    # -- lệnh "tất cả cube" ------------------------------------------------------------------------
    @staticmethod
    def batch_kind(intent, entities):
        """'sort' | 'stack' nếu lệnh nhắm TẤT CẢ cube, không thì None."""
        from cube_vision import batch_plan as B
        if intent == "sort_cube" and B.is_all(entities.get("label")):
            return "sort"
        if intent == "stack_cubes" and B.is_all(entities.get("source")):
            return "stack"
        return None

    def _reachable(self, xy, layer):
        if xy is None:
            return None
        try:
            import dofbot_ik
            j1 = float(dofbot_ik.ik(xy[0], xy[1], 0.047 + CUBE_EDGE_M * int(layer or 0))[0])
        except Exception:  # noqa: BLE001 - NoSolution hoặc ngoài tầm
            return False
        return 10.0 <= j1 <= 170.0

    def visible_cubes(self, settle_s=8.0):
        """Cube đang thấy (mỗi ID một mục): chờ tới khi danh sách ổn định hai lần liên tiếp hoặc hết settle_s."""
        from cube_vision.batch_plan import CubeInfo
        deadline, last, stable, cubes = self.clock() + settle_s, None, 0, []
        while True:
            scene = self.refresh()
            found = {}
            for obj in scene.get("objects", []):
                identity = obj.get("object_id")
                if (identity not in COLOR_NAMES or obj.get("geometry_model_id") != identity
                        or not 0 <= obj.get("age_s", math.inf) <= 2.0):
                    continue
                he = obj.get("handeye") or {}
                pos = obj.get("base_position")
                xy = ((he["x"], he["y"]) if he.get("ok") else (tuple(pos[:2]) if pos else None))
                layer = int(he["layer"]) if he.get("ok") else 0
                found[int(identity)] = CubeInfo(int(identity), xy, layer, self._reachable(xy, layer))
            cubes = [found[k] for k in sorted(found)]
            ids = tuple(c.id for c in cubes)
            stable = stable + 1 if ids == last and ids else 0
            last = ids
            if stable >= 2 or self.clock() >= deadline:
                return cubes
            time.sleep(0.4)

    def _step(self, kind, intent, ent, index, total, label):
        print(f"[batch {index}/{total}] {label}", flush=True)
        if self.batch_progress:
            self.batch_progress(index, total, label)
        result = self._run_with_retries(intent, ent)
        if result.get("retries_exhausted"):          # thử lại hết mà cube vẫn lệch: không tính là xong
            result = {**result, "ok": False, "code": "placement_unverified"}
        mark = "xong" if result.get("ok") else "lỗi"
        print(f"[batch {index}/{total}] {mark}: {result.get('reply', '')[:160]}", flush=True)
        return result

    def _execute_batch(self, kind, intent, ent):
        """Làm tuần tự mọi cube: lập thứ tự một lần, khảo sát zone MỘT lần, mỗi cube một vòng
        duyệt Space (hết hạn thì bỏ qua cube đó, không treo), lỗi của riêng cube thì làm tiếp cube kế,
        lỗi hệ thống thì dừng cả lô và báo rõ cái gì đã xong."""
        from cube_vision import batch_plan as B
        cubes = self.visible_cubes()
        if not cubes:
            return {"ok": False, "code": "needs_observation",
                    "reply": "Chưa thấy cube nào được perception xác thực; chưa làm gì.", "scan": scan_proposal()}
        rows, skipped = [], []
        timeout_before = getattr(self.backend, "approval_timeout", 0.0)
        if not timeout_before:
            self.backend.approval_timeout = self.batch_approval_timeout
        self.zone_cache = None
        try:
            if kind == "sort":
                order, skipped = B.plan_sort(cubes)
                if not order:
                    return {"ok": False, "code": "needs_observation",
                            "reply": B.summarize("Sort tất cả", [], skipped) or "Không có cube nào tới được."}
                steps = [("sort_cube", {"label": f"cube_{i}"}, f"sort cube {i} ({COLOR_NAMES[i]}) vào zone {i}")
                         for i in order]
                survey_error = self._batch_zone_survey(order)
                if survey_error:
                    return survey_error
            else:
                target_raw = str(ent.get("target") or "").strip().lower()
                base = cube_id(target_raw) if target_raw and not B.is_all(target_raw) else None
                plan = B.plan_stack(cubes, base=base)
                skipped = plan["skipped"]
                if not plan["steps"]:
                    return {"ok": False, "code": "needs_observation",
                            "reply": f"Không xếp được: {plan.get('reason') or 'không đủ cube'}."}
                steps = [("stack_cubes", {"source": f"cube_{s}", "target": f"cube_{t}"},
                          f"xếp cube {s} ({COLOR_NAMES[s]}) lên cube {t}") for s, t in plan["steps"]]
            print(f"[batch] {len(steps)} bước: " + " | ".join(label for _, _, label in steps)
                  + (f"; bỏ qua: {skipped}" if skipped else ""), flush=True)
            top = plan["base"] if kind == "stack" else None
            deadline = self.clock() + self.batch_deadline_s
            for index, (step_intent, step_ent, label) in enumerate(steps, 1):
                if self.clock() > deadline:
                    print("[batch] Quá thời gian tối đa của lô; dừng ở đây.", flush=True)
                    skipped = list(skipped) + [(int(str(w[1].get("label", w[1].get("source"))).split("_")[1]),
                                                "chưa làm: quá thời gian tối đa của lô") for w in steps[index - 1:]]
                    break
                if kind == "stack":
                    step_ent = {**step_ent, "target": f"cube_{top}"}      # đỉnh tháp hiện tại (bước trước có thể bị bỏ)
                result = self._step(kind, step_intent, step_ent, index, len(steps), label)
                rows.append((label, result))
                if kind == "stack" and (result.get("code") == "placement_unverified"
                                        or self.stack_failed_verification(result)):
                    skipped = list(skipped) + [(int(w[1]["source"].split("_")[1]), "chưa làm: tháp ở trạng thái không rõ")
                                               for w in steps[index:]]
                    break                          # tháp ở trạng thái không rõ: dừng
                if result.get("ok") and kind == "stack":
                    top = int(step_ent["source"].split("_")[1])
                if not B.should_continue(result):
                    skipped = list(skipped) + [(int(w[1].get("label", w[1].get("source")).split("_")[1]),
                                                "chưa làm do lô dừng") for w in steps[index:]]
                    break
        except KeyboardInterrupt:
            rows.append(("bị người dùng ngắt (Ctrl+C)", {"ok": False, "reply": "dừng theo yêu cầu"}))
        finally:
            self.zone_cache = None
            self.backend.approval_timeout = timeout_before
        ok = bool(rows) and all(r.get("ok") for _, r in rows)
        label = "Sort tất cả" if kind == "sort" else "Xếp chồng tất cả"
        return {"ok": ok, "code": "batch_complete" if ok else "batch_partial",
                "reply": B.summarize(label, rows, skipped), "batch": [
                    {"step": text, "ok": r.get("ok"), "code": r.get("code"),
                     "retries": r.get("retries", 0)} for text, r in rows],
                "status": "executed_unverified" if not ok else "batch_done"}

    def _batch_zone_survey(self, cube_ids):
        """Khảo sát zone MỘT lần cho cả lô (các ô không dời trong lúc sort). None nếu ổn."""
        if not (getattr(self.backend, "zone_check", True) and hasattr(self.backend, "zone_survey")
                and self.motion is not None):
            return None
        print(f"[zone] Khảo sát ô thả một lần cho cả lô (zone {', '.join(map(str, cube_ids))})...", flush=True)
        from zone_survey import run_survey
        result = run_survey(self.motion, self.backend, zones=tuple(int(i) for i in cube_ids))
        if not result["restored"]:
            return {"ok": False, "code": "zone_survey_restore_failed",
                    "reply": "Đã khảo sát zone nhưng tay không quay về được pose xuất phát; "
                             "kiểm tra tay máy rồi thử lại."}
        self.zone_cache = result
        return None

    def execute(self, intent, entities):
        kind = self.batch_kind(intent, entities)
        if kind and self.motion is not None:
            try:
                result = self._execute_batch(kind, intent, entities)
            except Exception as exc:  # noqa: BLE001
                result = {"ok": False, "code": "observation_unavailable", "reply": f"Lô dừng do lỗi: {exc}"}
        elif kind:
            cubes = self.visible_cubes(settle_s=3.0)
            result = {"ok": True, "status": "planned", "reply": (
                "Dry run: sẽ làm tuần tự " + ", ".join(f"cube {c.id}" for c in cubes) + "; chưa điều khiển motor.")}
        else:
            result = self._run_with_retries(intent, entities)
        physical = self.motion is not None
        result.setdefault("motor_commands_sent", 0)
        result.update(dry_run=not physical, execution_enabled=physical)
        if self.log_path:
            path = Path(self.log_path)
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps({"time": time.time(), "intent": intent,
                                             "entities": entities, "result": result},
                                            ensure_ascii=False, allow_nan=False) + "\n")
            except (OSError, ValueError):
                pass
        return result

    def _execute(self, intent, ent):
        if intent == "capabilities":
            mode = "ros3d_confirmed_motion" if self.motion is not None else "ros3d_dry_run"
            return {"ok": True, "detail": capability_context(mode), "reply":
                    "T8 đọc scene ROS 3D và kiểm tra IK/FK cho gắp–xếp. " +
                    ("Mọi chuyển động cần target READY và nhấn Space xác nhận."
                     if self.motion is not None else
                     "Gắp/xếp đang ở dry run; không điều khiển motor.")}
        if intent == "detection_mode":
            if ent.get("state") == "off":
                return {"ok": True, "status": "disabled", "reply":
                        "Đã tắt đề xuất quan sát; dry run không có chuyển động quét đang chạy."}
            return {"ok": True, "status": "planned", "scan": scan_proposal(),
                    "reply": "Đề xuất quét hai phía, dừng ổn định ở mỗi pose để quan sát. "
                             "Dry run chưa xoay tay; cần kiểm chứng đường quét trước khi chạy thật."}
        simple_motion = {"pose_start": ("prepare", {}),
                         "arm_pose": ("arm_pose", {"pose": ent.get("pose")}),
                         "gripper": ("gripper", {"state": ent.get("state")})}
        if intent in simple_motion and self.motion is not None:
            command, params = simple_motion[intent]
            try:
                return self.motion.execute(command, **params)
            finally:
                if hasattr(self.motion, "close"):
                    self.motion.close()
        if intent in {"place_held", "release_hold"} and self.motion is not None:
            if intent == "place_held" and ent.get("bin", "ban") != "ban":
                return {"ok": False, "code": "unsupported_placement",
                        "reply": "ROS3D hiện chỉ hỗ trợ đặt vật đang giữ xuống XY gắp ban đầu."}
            command = "place" if intent == "place_held" else "release"
            try:
                result = self.motion.execute(command)
            finally:
                if hasattr(self.motion, "close"):
                    self.motion.close()
            result.setdefault("actual_grasp_verified", False)
            return result
        if intent not in READ_TOOLS | {"search_object", "vision_pick_hold", "sort_cube", "stack_cubes"}:
            return {"ok": False, "code": "motion_disabled", "reply":
                    f"{intent} chưa được thực thi trong chế độ ROS 3D dry run."}
        if intent == "vision_pick_hold" and ent.get("hold", True) is not True:
            return {"ok": False, "code": "unsupported_placement", "reply":
                    "ROS 3D hiện chỉ lập kế hoạch gắp–giữ hoặc stack_cubes với cube đích rõ ràng."}
        if self.motion is not None and intent in {"vision_pick_hold", "sort_cube", "stack_cubes"}:
            return self.execute_approved_motion(intent, ent)
        action_labels = ([ent.get("source"), ent.get("target")]
                         if intent == "stack_cubes" else [ent.get("label")])
        if self.motion is not None and intent in {"vision_pick_hold", "sort_cube", "stack_cubes"}:
            scene = self.wait_for_labels(action_labels)
        else:
            scene = self.refresh()
        if intent == "observe_scene":
            return {"ok": True, "detail": scene, "reply": self.describe_scene(scene)}
        if intent == "robot_status":
            return {"ok": True, "detail": scene.get("robot", {}), "reply":
                    f"Hiệu chuẩn: {'đạt' if scene.get('calibrated') else 'chưa đạt'}. "
                    f"Trạng thái được ghi nhận: {scene.get('robot', {}).get('phase', 'unknown')} "
                    "(chưa xác minh có vật trong kẹp). "
                    "T8 đang ở dry run; không điều khiển motor."}
        label = ent.get("source") if intent == "stack_cubes" else ent.get("label")
        source, reason = self.resolve(label, allow_memory=intent == "stack_cubes")
        if source is None:
            suffix = (" Perception đã được chờ khởi động nhưng chưa khóa được ID; "
                      "kiểm tra viewer/perception log và vị trí cube."
                      if self.motion is not None else
                      " Dry run chỉ đề xuất quét, chưa xoay robot.")
            return {"ok": False, "code": "needs_observation", "found": False,
                    "reply": reason + suffix,
                    "scan": scan_proposal()}
        if intent == "search_object":
            xyz = source.get("base_position")
            return {"ok": True, "found": True, "detail": source, "reply":
                    f"Đã xác thực cube ID {source['object_id']} ({COLOR_NAMES[source['object_id']]}). "
                    + (f"Tọa độ base {xyz} m." if source.get("base_pose_valid") else
                       "Chưa có tọa độ base được hiệu chuẩn; chưa thể tính gắp.")}
        error = (self.fixed_pose_error(source) if self.motion is not None
                 else self.metric_error(source))
        if error:
            return {"ok": False, "code": "pose_not_ready", "reply": error}
        targets = []
        for stage, clearance in (("approach_pick", 0.020), ("pick", 0.0), ("lift_pick", 0.030)):
            grasp = dict(source["grasp"])
            grasp["tcp_position_base"] = list(grasp["tcp_position_base"])
            grasp["tcp_position_base"][2] += clearance
            targets.append({"stage": stage, **grasp})
        detail = {"source": source, "requires_revalidation_before_motion": True}
        if intent == "stack_cubes":
            target, reason = self.resolve(ent.get("target"))
            if target is None:
                return {"ok": False, "code": "needs_observation", "reply": reason +
                        " Đã nhớ nguồn trong base_link; chưa gắp nguồn.", "scan": scan_proposal(),
                        "detail": detail}
            if target["object_id"] == source["object_id"] or target["track_id"] == source["track_id"]:
                return {"ok": False, "code": "same_object", "reply": "Nguồn và đích là cùng một cube."}
            error = (self.fixed_pose_error(target) if self.motion is not None
                     else self.metric_error(target))
            if error:
                return {"ok": False, "code": "pose_not_ready", "reply": error}
            if math.dist(source["base_position"], target["base_position"]) < 0.035:
                return {"ok": False, "code": "objects_too_close", "reply": "Nguồn và đích quá gần để lập kế hoạch xếp."}
            place = dict(target["grasp"])
            place["tcp_position_base"] = list(place["tcp_position_base"])
            # Centre of the carried 30mm cube above the support cube. Same
            # measured contact correction as the grasp planner, not fixed table Z.
            place["tcp_position_base"][2] += 0.030
            hover = dict(place)
            hover["tcp_position_base"] = list(place["tcp_position_base"])
            hover["tcp_position_base"][2] += 0.020
            targets += [{"stage": "hover_place", **hover}, {"stage": "place", **place}]
            detail["target"] = target
        check = self.backend.preflight(targets)
        if not check.get("ok"):
            return {"ok": False, "code": "preflight_failed", "reply":
                    "Preflight thất bại: " + check.get("reason", "IK/FK không đạt"),
                    "detail": {**detail, "preflight": check}}
        if self.motion is None:
            return {"ok": True, "status": "planned", "actual_grasp_verified": False,
                    "detail": {**detail, "preflight": check}, "reply":
                    "Dry run: đã xác thực nguồn" + (" và đích" if intent == "stack_cubes" else "") +
                    ", IK/FK đạt. Đã lập kế hoạch " + ("gắp–xếp" if intent == "stack_cubes" else "gắp–giữ") +
                    "; chưa gắp hoặc thả vật. Cần kiểm chứng TCP và đường đi trước khi chạy thật."}
