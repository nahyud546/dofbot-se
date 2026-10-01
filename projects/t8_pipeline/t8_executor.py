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
    def __init__(self, state=None, arm_factory=None, vision=None, motion=None):
        """
        state: CachedState (giữ holding). arm_factory: fn() -> Arm_Device hoặc None (mock).
        vision: object có get_detections(label, frame) -> [{label,box,conf}] hoặc None.
        """
        self.state = state or CachedState()
        self._arm_factory = arm_factory
        self._vision = vision
        self._motion = motion
        self._frame, self._ctx = None, ""
        self.log = []

    # -- entry ------------------------------------------------------------
    def execute(self, intent, entities=None, frame=None, ctx_reply="",
                confirm_frame=None):
        ent = dict(entities or {})
        # frame (PIL Image) + ctx_reply (mô tả Gemini vision) cho skill vision
        self._frame, self._ctx = frame, ctx_reply
        self._confirm_frame = confirm_frame
        fn = {"ask_info": self._ask_info, "open_task": self._open_task,
              "stop_task": self._stop_task, "rotate_relative": self._rotate,
              "vision_pick_hold": self._pick_hold, "place_held": self._place,
              "release_hold": self._release, "gripper": self._gripper,
              "light_beep": self._light, "pose_start": self._pose_start,
              "arm_pose": self._arm_pose}.get(intent)
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
        self.state.mark_released()
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
        self.state.mark_released()
        return {"ok": True, "action": "none",
                "reply": f"Đã đặt {label} xuống {b}.", "detail": {"bin": b}}

    def _release(self, ent):
        if self._motion is not None:
            return self._motion.execute("release")
        if not self.state.holding:
            return {"ok": False, "reply": "Tay đang trống.", "action": "none"}
        label = self.state.held_label
        self.state.mark_released()
        return {"ok": True, "action": "none",
                "reply": f"Đã nhả {label} (mở kẹp, giữ nguyên tay)."}

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
