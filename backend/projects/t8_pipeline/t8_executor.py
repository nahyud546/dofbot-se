#!/usr/bin/env python3
# coding: utf-8
"""T8 executor: intent (đã validate) -> skill an toàn. Gemini KHÔNG chạm hardware.

Luồng: Pipeline.run() -> {intent, entities} -> Executor.execute() ->
  - ask_info/open_task/stop_task: trả lời / ủy quyền voice_task_manager (giữ nguyên)
  - rotate_relative: đọc góc hiện tại (state_service) + delta -> clamp -> write servo
  - vision_pick_hold / place_held / release_hold / gripper / light_beep:
    phase 1 (hiện tại): YOLO nhận diện + kế hoạch gắp; chưa chạy tay máy.
    phase 2: nối tọa độ ảnh vào grasp.py / Arm_Lib sau hiệu chuẩn.

Mọi skill đều kiểm tra lại params (defense in depth), không tin Gemini tuyệt đối.
"""
import re

try:
    from state_service import get_joint, CachedState
except ImportError:
    from .state_service import get_joint, CachedState  # type: ignore

JOINT_MIN, JOINT_MAX = 0, 180
DELTA_MAX = 90


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Executor:
    def __init__(self, state=None, arm_factory=None, vision=None, motion=None,
                 scene=None):
        """
        state: CachedState (giữ holding). arm_factory: fn() -> Arm_Device hoặc None (mock).
        vision: object có get_detections(label, frame) -> [{label,box,conf}] hoặc None.
        motion: MotionBridge (có thể None cho dry-run).
        scene: SceneMemory (blackboard vật đã thấy; tự tạo nếu None).
        """
        self.state = state or CachedState()
        self._arm_factory = arm_factory
        self._vision = vision
        self._motion = motion
        self.dry_run = motion is None and arm_factory is None
        self._frame, self._ctx = None, ""
        self.log = []
        if scene is None:
            try:
                from t8_scene import SceneMemory
            except ImportError:
                from .t8_scene import SceneMemory  # type: ignore
            scene = SceneMemory()
        self._scene = scene
        self._ros_runner = None
        if getattr(vision, "backend", None) == "ros3d":
            from t8_ros_tasks import RosTaskRunner
            self._ros_runner = RosTaskRunner(
                vision, motion=motion,
                log_path="/tmp/t8_ros_task_events.jsonl")

    # -- entry ------------------------------------------------------------
    def execute(self, intent, entities=None, frame=None, ctx_reply="",
                confirm_frame=None):
        ent = dict(entities or {})
        if intent in ("vision_pick_hold", "search_object") and isinstance(ent.get("label"), str):
            try:
                from cube_identity import canonical_label
            except ImportError:
                try:
                    from .cube_identity import canonical_label  # type: ignore
                except ImportError:
                    canonical_label = None
            if canonical_label is not None:
                ent["label"] = canonical_label(ent["label"])
        if self._ros_runner is not None:
            return self._ros_runner.execute(intent, ent)
        # frame (PIL Image) + ctx_reply (mô tả Gemini vision) cho skill vision
        self._frame, self._ctx = frame, ctx_reply
        self._confirm_frame = confirm_frame
        fn = {"ask_info": self._ask_info, "open_task": self._open_task,
              "stop_task": self._stop_task, "rotate_relative": self._rotate,
              "vision_pick_hold": self._pick_hold, "place_held": self._place,
              "release_hold": self._release, "gripper": self._gripper,
              "light_beep": self._light, "pose_start": self._pose_start,
              "arm_pose": self._arm_pose,
              "detection_mode": self._detection_mode,
              "search_object": self._search_object,
              "observe_scene": self._observe_scene,
              "robot_status": self._robot_status,
              "capabilities": self._capabilities}.get(intent)
        if fn is None:
            return {"ok": False, "reply": f"Intent lạ '{intent}', từ chối để an toàn.",
                    "action": "none"}
        return fn(ent)

    # -- pure / legacy ----------------------------------------------------
    def _arm_pose(self, ent):
        pose = ent.get("pose", "")
        if pose not in ("up", "down"):
            return {"ok": False, "reply": "Lệnh tay không rõ.", "action": "none"}
        if self._motion is None:
            return {"ok": True, "dry_run": True, "action": "none", "reply": f"[dry-run] Tay {pose}."}
        res = self._motion.execute("arm_pose", pose=pose)
        res["action"] = "none"
        return res
    def _pose_start(self, ent):
        if self._motion is None:
            return {"ok": True, "dry_run": True,
                    "reply": "[dry-run] Về tư thế chuẩn; chưa gửi lệnh robot.",
                    "action": "none"}
        res = self._motion.execute("prepare")
        res["action"] = "none"
        return res

    def _ask_info(self, ent):
        return {"ok": True, "reply": "", "action": "none", "passthrough": True}

    def _open_task(self, ent):
        t = ent.get("task", "")
        if t not in ("color", "stack", "face", "trash"):
            return {"ok": False, "reply": "Bài toán không rõ, nói lại giúp.", "action": "none"}
        if self.state.holding:
            return {"ok": False, "reply": f"Đang giữ {self.state.held_label}, "
                    "đặt xuống hoặc dừng trước khi mở bài mới.", "action": "none"}
        return {"ok": True, "reply": "", "action": t, "passthrough": True}

    def _stop_task(self, ent):
        return {"ok": True, "reply": "", "action": "stop", "passthrough": True}

    # -- rotate_relative (state-aware) ------------------------------------
    def _rotate(self, ent):
        try:
            joint = int(ent.get("joint", 1))
            delta = float(ent.get("delta_deg", 0))
        except (TypeError, ValueError):
            return {"ok": False, "reply": "Góc xoay không rõ.", "action": "none"}
        if joint not in (1, 2, 3, 4, 5, 6):
            return {"ok": False, "reply": "Chỉ hỗ trợ khớp 1-6.", "action": "none"}
        if delta == 0 or abs(delta) > DELTA_MAX:
            return {"ok": False, "reply": f"Chỉ xoay thêm tối đa ±{DELTA_MAX} độ/lệnh.",
                    "action": "none"}
        if self._motion is not None:
            return self._motion.execute("rotate", joint=joint, delta_deg=delta)
        if self.dry_run:
            return {"ok": True, "dry_run": True, "action": "none",
                    "reply": f"[dry-run] Đề xuất xoay J{joint} thêm {delta:g} độ; chưa đọc serial hoặc di chuyển.",
                    "plan": {"joint": joint, "delta_deg": delta}}
        if self.state.holding and joint in (1, 2, 3):
            # xoay đế khi đang giữ vật dễ va chạm -> cảnh báo nhưng vẫn cho nếu delta nhỏ
            pass
        cur = get_joint(joint)
        if cur is None and self._arm_factory is not None:
            try:
                arm = self._arm_factory()
                cur = float(arm.Arm_serial_servo_read(joint))
                try:
                    cur = float(arm.Arm_serial_servo_read(joint))
                except Exception:
                    pass
            except Exception:
                cur = None
        if cur is None:
            # Không có hardware (test laptop): trả kế hoạch dry-run rõ ràng
            sign = "+" if delta > 0 else ""
            return {"ok": True, "dry_run": True, "action": "none",
                    "reply": f"[dry-run] Đọc J{joint} hiện tại rồi xoay {sign}{delta:g} độ "
                    f"(clamp {JOINT_MIN}-{JOINT_MAX}). Cắm tay + /dev/ttyUSB0 để chạy thật.",
                    "plan": {"joint": joint, "delta_deg": delta}}
        target = clamp(cur + delta, JOINT_MIN, JOINT_MAX)
        ms = int(ent.get("speed_ms", 800))
        ms = int(clamp(ms, 200, 3000))
        moved = self._write_servo(joint, target, ms)
        if not moved:
            return {"ok": False, "reply": f"Không ghi được servo J{joint}.", "action": "none"}
        huong = "phải" if delta > 0 else "trái"
        return {"ok": True, "action": "none",
                "reply": f"Đã xoay khớp {joint} sang {huong} {abs(delta):g} độ "
                f"({cur:.0f}° → {target:.0f}°).",
                "detail": {"from": cur, "to": target}}

    def _write_servo(self, joint, angle, ms):
        if self._arm_factory is None:
            return False
        try:
            arm = self._arm_factory()
            arm.Arm_serial_servo_write(int(joint), int(round(angle)), int(ms))
            return True
        except Exception:
            return False

    # -- vision pick / hold / place ---------------------------------------
    def _detections(self, label):
        if self._vision is None or self._frame is None:
            return []
        if hasattr(self._vision, "get_cube_candidates"):
            return self._vision.get_cube_candidates(
                label, self._frame, confirm_frame=self._confirm_frame)
        return self._vision.get_detections(label, self._frame) or []

    def _pick_hold(self, ent):
        label = re.sub(r"\s+", "_", str(ent.get("label", "vat_the")).strip() or "vat_the")[:60]
        hold = ent.get("hold", True) is not False
        if self.state.holding:
            return {"ok": False, "reply": f"Đang giữ {self.state.held_label} rồi.",
                    "action": "none"}
        # cube_ID: ưu tiên tag; lệnh hành động rõ thì fallback màu theo mapping.
        if label.startswith("cube_"):
            try:
                tag_dets = self._detections(label)
            except Exception:
                tag_dets = []
            if not tag_dets:
                try:
                    from cube_identity import color_for_id, ID_NAMES_VI
                except ImportError:
                    try:
                        from .cube_identity import color_for_id, ID_NAMES_VI  # type: ignore
                    except ImportError:
                        color_for_id, ID_NAMES_VI = None, {}
                if color_for_id is not None:
                    try:
                        want = int(label.rsplit("_", 1)[1])
                        color = color_for_id(want)
                    except (ValueError, IndexError):
                        color, want = None, None
                    if color is not None and self._vision is not None and self._frame is not None:
                        try:
                            c_dets = self._vision.get_cube_candidates(
                                color, self._frame, confirm_frame=self._confirm_frame)
                        except Exception:
                            c_dets = []
                        if len(c_dets) == 1:
                            best = c_dets[0]
                            if self._motion is not None:
                                return {"ok": False, "action": "none",
                                        "code": "strict_identity_required",
                                        "detail": {"requested": label,
                                                   "color_candidate": color,
                                                   "box": best.get("box")},
                                        "reply": (f"Chỉ thấy màu {ID_NAMES_VI.get(want, color)}; "
                                                  f"chưa có bằng chứng ID 3D cho {label}. "
                                                  "Không gắp theo suy đoán màu.")}
                            return {"ok": True, "dry_run": True, "action": "none",
                                    "detail": {"box": best.get("box"), "requested": label,
                                               "color_inferred": True},
                                    "reply": f"Thấy {label} qua màu {ID_NAMES_VI.get(want, color)} "
                                             f"tại box {best.get('box')} (chưa tag, dry-run)."}
                hint = ""
                if color_for_id is not None:
                    try:
                        want = int(label.rsplit("_", 1)[1])
                        hint = (f" Nếu muốn gắp {label}, hãy xác nhận đó có phải khối màu "
                                f"{ID_NAMES_VI.get(want, '')} bạn đang thấy không, hoặc dán tag36h11 ID{want}.")
                    except (ValueError, IndexError):
                        pass
                return {"ok": False, "action": "none",
                        "reply": f"Chưa thấy tag {label} và cũng chưa thấy màu tương ứng.{hint}"}
        try:
            dets = self._detections(label)
        except Exception as exc:
            return {"ok": False, "action": "none",
                    "reply": f"YOLO không chạy được: {exc}. Chưa gắp vật."}
        if not dets:
            # Chưa có detector thật: không bịa bbox. Nếu có ảnh camera +
            # mô tả Gemini vision thì kèm theo để người dùng xác nhận bước tiếp.
            eye = ""
            if getattr(self, "_frame", None) is not None:
                eye = "Đã nhìn qua camera. "
                if getattr(self, "_ctx", ""):
                    eye += f"Gemini thấy: {self._ctx.strip()[:400]} "
            return {"ok": False, "dry_run": True, "action": "none",
                    "reply": f"{eye}Kế hoạch: tìm {label} qua camera → gắp lên và "
                    f"{'GIỮ NGUYÊN (không đặt xuống)' if hold else 'đặt theo yêu cầu'}. "
                    "Chưa xác định được một cube duy nhất bằng YOLO/HSV/contour; "
                    "chưa gắp. Đưa vật vào giữa khung hình hoặc nói rõ màu rồi thử lại.",
                    "plan": {"label": label, "hold": hold, "steps":
                             ["get_detections", "grasp_hold", "hold"]}}
        if len(dets) != 1:
            return {"ok": False, "reply":
                    "Có nhiều khối phù hợp; hãy nói rõ màu hoặc vị trí trước khi gắp."}
        best = dets[0]
        if self._motion is not None:
            return self._motion.execute("pick", label=label, box=best.get("box"),
                                        conf=best.get("conf", 0),
                                        source=best.get("source", "yolo"),
                                        geometry_verified=best.get("geometry_verified", False),
                                        image_size=list(self._frame.size))
        detector_name = {"yolo": "YOLO", "hsv": "HSV", "contour": "Contour"}.get(
            best.get("source", "yolo"), "YOLO")
        confidence = (f"conf {best['conf']:.2f}, " if best.get("source", "yolo") == "yolo"
                      else "")
        return {"ok": True, "dry_run": True, "action": "none",
                "reply": f"{detector_name} thấy {label} ({confidence}"
                         f"box {best.get('box')}). Chế độ dry-run: chưa gửi "
                         "lệnh tới tay robot.",
                "detail": {"box": best.get("box"),
                           "corners": best.get("corners"),
                           "source": best.get("source", "yolo"),
                           "holding": False}}

    def _place(self, ent):
        if self._motion is not None:
            if ent.get("bin", "ban") != "ban":
                return {"ok": False, "reply": "Chỉ hỗ trợ đặt xuống bàn tại XY hiện tại."}
            return self._motion.execute("place")
        if not self.state.holding:
            return {"ok": False, "reply": "Tay đang trống, không có gì để đặt.",
                    "action": "none"}
        b = str(ent.get("bin", "ban") or "ban")[:40]
        label = self.state.held_label
        return {"ok": True, "dry_run": True, "action": "none",
                "reply": f"[dry-run] Đề xuất đặt {label} xuống {b}; chưa gửi lệnh robot.",
                "detail": {"bin": b}}

    def _release(self, ent):
        if self._motion is not None:
            return self._motion.execute("release")
        if not self.state.holding:
            return {"ok": False, "reply": "Tay đang trống.", "action": "none"}
        label = self.state.held_label
        return {"ok": True, "dry_run": True, "action": "none",
                "reply": f"[dry-run] Đề xuất nhả {label}; chưa mở kẹp."}

    def _gripper(self, ent):
        s = ent.get("state", "")
        if s not in ("open", "close"):
            return {"ok": False, "reply": "Kẹp chỉ có mở/đóng.", "action": "none"}
        if self._motion is None:
            return {"ok": True, "dry_run": True, "action": "none",
                    "reply": f"[dry-run] Kẹp {'mở' if s=='open' else 'đóng'} (servo 6)."}
        res = self._motion.execute("gripper", state=s)
        res["action"] = "none"
        if res.get("ok"):
            self.state.mark_released() if s == "open" else self.state.mark_held("unknown")
        return res

    def _light(self, ent):
        dev, st = ent.get("device", ""), ent.get("state", "")
        if dev not in ("red", "green", "blue", "yellow", "beep") or st not in ("on", "off"):
            return {"ok": False, "reply": "Lệnh đèn/còi không rõ.", "action": "none"}
        if self._motion is None:
            return {"ok": True, "dry_run": True, "action": "none",
                    "reply": f"[dry-run] {dev} {st} (qua module speech/serial)."}
        res = self._motion.execute("light", device=dev, state=st)
        res["action"] = "none"
        return res

    # -- active perception -------------------------------------------------
    def _detection_mode(self, ent):
        st = ent.get("state", "on")
        if st not in ("on", "off"):
            return {"ok": False, "reply": "detection_mode chỉ nhận on/off.",
                    "action": "none"}
        if self._motion is None:
            return {"ok": True, "dry_run": True, "action": "none",
                    "reply": f"[dry-run] Detection mode {st}: về pose quan sát, quét chậm."}
        res = self._motion.execute("detection_mode", state=st)
        res["action"] = "none"
        return res

    def _search_object(self, ent):
        """Tìm 1 vật trong frame HIỆN TẠI (không tự xoay ở đây).

        Hàm này chỉ lookup frame và ghi bộ nhớ pixel trong cùng góc nhìn.
        Quét đổi góc nhìn cần backend ROS 3D và TF động.
        Quy định: color-inferred chỉ trả lời, không hành động.
        """
        label = re.sub(r"\s+", "_", str(ent.get("label", "")).strip() or "vat_the")[:60]
        exclude = ent.get("exclude")
        try:
            dets = self._detections(label) if not isinstance(exclude, dict) else self._vision.get_cube_candidates(
                label, self._frame, confirm_frame=self._confirm_frame,
                exclude=exclude) if self._vision is not None and self._frame is not None else []
        except Exception as exc:
            return {"ok": False, "action": "none",
                    "reply": f"Detector lỗi khi tìm {label}: {exc}."}
        if not dets:
            # Color-inferred chỉ trả lời: cube_ID không tag nhưng thấy đúng màu mapping.
            try:
                from cube_identity import color_for_id, ID_NAMES_VI
            except ImportError:
                try:
                    from .cube_identity import color_for_id, ID_NAMES_VI  # type: ignore
                except ImportError:
                    color_for_id, ID_NAMES_VI = None, {}
            if color_for_id is not None and label.startswith("cube_"):
                try:
                    want = int(label.rsplit("_", 1)[1])
                    color = color_for_id(want)
                except (ValueError, IndexError):
                    color, want = None, None
                if color is not None and self._vision is not None and self._frame is not None:
                    try:
                        c_dets = self._vision.get_cube_candidates(
                            color, self._frame, confirm_frame=self._confirm_frame)
                    except Exception:
                        c_dets = []
                    if len(c_dets) == 1:
                        return {"ok": True, "action": "none", "found": False,
                                "color_inferred": True,
                                "detail": {"label": label, "color": color,
                                           "box": c_dets[0].get("box")},
                                "reply": f"Chưa thấy tag {label} trong khung hình; chỉ thấy màu "
                                         f"{ID_NAMES_VI.get(want, color)} tại box {c_dets[0].get('box')} "
                                         f"(suy đoán {label} qua màu, chưa xác thực tag nên chỉ trả lời, "
                                         f"chưa gắp). Dán tag36h11 ID{want} để gắp chắc."}
            eye = "Đã nhìn qua camera. " if getattr(self, "_frame", None) is not None else ""
            return {"ok": False, "action": "none", "found": False,
                    "reply": f"{eye}Chưa thấy {label} trong khung hình hiện tại; cần quét thêm góc khác."}
        if len(dets) != 1:
            return {"ok": False, "action": "none", "found": False,
                    "reply": f"Thấy {len(dets)} ứng viên {label}; cần nói rõ màu/vị trí hoặc quét gần hơn."}
        best = dets[0]
        try:
            self._scene.remember(label, best)
        except Exception:
            pass
        box = best.get("box")
        return {"ok": True, "action": "none", "found": True,
                "reply": f"Đã thấy {label} tại box {box} (nguồn {best.get('source','?')}); dừng quét để tính gắp.",
                "detail": {"label": label, "box": box,
                           "center": best.get("center"),
                           "corners": best.get("corners"),
                           "source": best.get("source"),
                           "conf": best.get("conf", 0)}}

    # -- legacy observation (camera 2D, không cần ROS 3D) --------------------
    def _observe_scene(self, ent):
        """Liệt kê cube thấy trong frame hiện tại (legacy2d)."""
        if self._vision is None:
            return {"ok": False, "action": "none",
                    "reply": "Bộ nhận diện T8 chưa sẵn sàng."}
        if self._frame is None:
            return {"ok": False, "action": "none",
                    "reply": "Chưa có ảnh camera mới; thử lại với /see."}
        labels = ["khoi_do", "khoi_xanh", "khoi_xanh_duong", "khoi_vang",
                  "cube", "cube_1", "cube_2", "cube_3", "cube_4"]
        try:
            from cube_identity import color_for_id, ID_NAMES_VI
        except ImportError:
            try:
                from .cube_identity import color_for_id, ID_NAMES_VI  # type: ignore
            except ImportError:
                color_for_id, ID_NAMES_VI = None, {}
        seen, details = [], {}
        for label in labels:
            try:
                if label in ("cube", "khoi_do", "khoi_xanh",
                             "khoi_xanh_duong", "khoi_vang"):
                    dets = self._vision.get_cube_candidates(
                        label, self._frame, confirm_frame=self._confirm_frame)
                else:
                    dets = self._vision.get_cube_candidates(label, self._frame)
            except Exception:
                dets = []
            if len(dets) == 1:
                seen.append(f"{label} tại box {dets[0].get('box')}")
                details[label] = dets[0]
            elif (color_for_id is not None and label.startswith("cube_")
                    and self._frame is not None):
                try:
                    want = int(label.rsplit("_", 1)[1])
                    color = color_for_id(want)
                    c_dets = self._vision.get_cube_candidates(
                        color, self._frame, confirm_frame=self._confirm_frame)
                except Exception:
                    c_dets = []
                if len(c_dets) == 1:
                    seen.append(f"{label} (suy đoán qua màu {ID_NAMES_VI.get(want, color)}, "
                                f"chưa tag, chỉ trả lời) tại box {c_dets[0].get('box')}")
                    details[label] = {**c_dets[0], "color_inferred": True}
        if not seen:
            return {"ok": True, "action": "none", "found": False,
                    "detail": details,
                    "reply": "Đã nhìn qua camera; chưa thấy cube nào rõ trong khung hình. "
                             "Đưa vật vào giữa khung, đủ sáng rồi thử lại."}
        return {"ok": True, "action": "none", "found": True,
                "detail": details,
                "reply": "Đã nhìn qua camera, thấy: " + "; ".join(seen) + "."}

    def _robot_status(self, ent):
        holding = bool(getattr(getattr(self, "state", None), "holding", False))
        motion = "đã kết nối" if self._motion is not None else "chưa kết nối (dry-run)"
        mode = "dry-run" if getattr(self, "dry_run", False) else "legacy2d + motion"
        return {"ok": True, "action": "none",
                "detail": {"holding": holding, "motion": motion, "mode": mode},
                "reply": f"Tay máy {motion}; chế độ {mode}; "
                         f"{'đang giữ vật' if holding else 'tay trống'}."}

    def _capabilities(self, ent):
        mode = "dry-run" if getattr(self, "dry_run", False) else "legacy2d + motion"
        return {"ok": True, "action": "none",
                "detail": {"mode": mode,
                           "tools": ["observe_scene", "search_object",
                                     "vision_pick_hold", "rotate_relative",
                                     "arm_pose", "pose_start", "gripper"]},
                "reply": f"Chế độ {mode}: xem scene qua camera, tìm/gắp cube, "
                         "xoay đế, nâng/hạ tay, mở kẹp. Hỏi 'thấy gì' để liệt kê cube."}
