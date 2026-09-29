#!/usr/bin/env python3
# coding: utf-8
"""Voice action runner: lệnh action -> READY (camera + SPACE) -> EXECUTING (chỉ nhận STOP).

State machine (đúng yêu cầu):
  LISTEN    : nghe mic (sherpa-vi). Lệnh đơn (đèn/còi/tay/kẹp: 11-14,38-44)
              chạy ngay. Lệnh ACTION (51-54) -> chuyển READY, KHÔNG chạy vội.
  READY     : hiện camera + chữ "READY - SPACE chạy / Q hủy" (timeout 30s).
              Chỉ phím mới xác nhận, voice lúc này không có tác dụng.
  EXECUTING : chạy motion trên worker thread. Mọi lệnh voice khác đều BỊ
              BỎ QUA (log rõ), chỉ nhận lệnh DỪNG ("dừng lại/dừng/hủy/thôi")
              hoặc phím S. Xong / dừng / lỗi -> về LISTEN + đưa tay về pose an toàn.

Ví dụ:
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/voice_action_runner.py
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/voice_action_runner.py --dry-run --no-cam
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/voice_action_runner.py --self-test
"""
import argparse
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from stt_vi import SherpaViBackend, match_command, norm, record_wav, SHERPA_MODEL_DIR  # noqa: E402

ROBOT_CAM = "/dev/video2"
ROBOT_PORT = "/dev/ttyUSB0"

# Lệnh ACTION phải qua READY+SPACE (motion dài, nguy hiểm nếu chạy nhầm).
ACTION_CODES = {
    51: ("xếp chồng", "heap_up"),
    52: ("nhảy múa", "dance"),
    53: ("gắp khối", "clamp_clock"),
    54: ("vận chuyển", "move_block"),
}
# Lệnh đơn chạy ngay ở LISTEN (nhanh, vô hại).
SIMPLE_CODES = {11, 12, 13, 14, 38, 39, 40, 41, 42, 43, 44}
# Từ dừng: kiểm tra TRƯỚC match_command, chỉ có tác dụng ở EXECUTING.
STOP_WORDS = ["dừng lại", "dung lai", "dừng", "hủy", "thôi", "stop"]


def is_stop(text):
    t = norm(text)
    return any(w in t for w in [norm(w) for w in STOP_WORDS])


class _Abort(Exception):
    pass


class Mover:
    """Motion copy từ action_voice_ctrl.py, thêm abort + dry-run.

    Mọi bước servo đều qua _servo/_sleep nên lệnh DỪNG có hiệu lực giữa các
    bước (trễ tối đa ~2s do sleep dài trong chuỗi gốc).
    """

    # pose gốc giữ nguyên
    p_mould = [90, 130, 0, 0, 90]
    p_top = [90, 80, 50, 50, 270]
    p_layer_4 = [90, 76, 40, 17, 270]
    p_layer_3 = [90, 65, 44, 17, 270]
    p_layer_2 = [90, 65, 25, 36, 270]
    p_layer_1 = [90, 48, 35, 30, 270]
    p_move_layer_4 = [90, 72, 49, 13, 270]
    p_move_layer_3 = [90, 66, 43, 20, 270]
    p_move_layer_2 = [90, 63, 34, 30, 270]
    p_move_layer_1 = [90, 53, 33, 36, 270]
    p_Yellow = [65, 22, 64, 56, 270]
    p_Red = [118, 19, 66, 56, 270]
    p_Green = [136, 66, 20, 29, 270]
    p_Blue = [44, 66, 20, 28, 270]
    p_Brown = [90, 53, 33, 36, 270]
    look_at = [90, 164, 18, 0, 90, 90]
    time_1 = 500
    time_2 = 1000
    time_sleep = 0.5

    def __init__(self, arm=None, abort_event=None, dry_run=False, fast=False, log=None):
        self.arm = arm
        self.abort = abort_event or threading.Event()
        self.dry_run = dry_run
        self.fast = fast
        self.log = log if log is not None else []

    def _msg(self, s):
        self.log.append(s)
        print(s, flush=True)

    def _check(self):
        if self.abort.is_set():
            raise _Abort()

    def _sleep(self, sec):
        self._check()
        self._msg(f"[motion] sleep {sec}s")
        if not self.dry_run:
            # ngủ từng quãng ngắn để lệnh dừng phản ứng nhanh
            t0 = time.time()
            while time.time() - t0 < sec:
                self._check()
                time.sleep(min(0.2, sec - (time.time() - t0)))
        elif self.fast:
            time.sleep(0.02)

    def _servo(self, sid, angle, t):
        self._check()
        self._msg(f"[motion] servo {sid} -> {angle} ({t}ms)")
        if not self.dry_run and self.arm is not None:
            self.arm.Arm_serial_servo_write(sid, angle, t)
        time.sleep(0.012)

    def _servo6(self, *a):
        self._check()
        self._msg(f"[motion] servo6 {a}")
        if not self.dry_run and self.arm is not None:
            self.arm.Arm_serial_servo_write6(*a)

    # -- primitives (logic y hệt bản gốc) --
    def arm_clamp_block(self, enable):
        self._servo(6, 60 if enable == 0 else 135, 400)
        self._sleep(.5)

    def arm_move(self, p, s_time=500):
        for i in range(5):
            idx = i + 1
            if idx == 5:
                self._sleep(.1)
                self._servo(idx, p[i], int(s_time * 1.2))
            elif idx == 1:
                self._servo(idx, p[i], int(3 * s_time / 4))
            else:
                self._servo(idx, p[i], int(s_time))
            self._sleep(.01)
        self._sleep(s_time / 1000)

    def arm_move_clamp(self, p, s_time=500):
        for i in range(5):
            idx = i + 1
            if idx == 5:
                self._sleep(.1)
                self._servo(idx, p[i], int(s_time * 1.2))
            else:
                self._servo(idx, p[i], s_time)
            self._sleep(.01)
        self._sleep(s_time / 1000)

    def arm_move_up(self):
        self._servo(2, 90, 1500)
        self._servo(3, 90, 1500)
        self._servo(4, 90, 1500)
        self._sleep(.1)

    def arm_move_6(self, p, s_time=500):
        for i in range(6):
            self._servo(i + 1, p[i], s_time)
            self._sleep(.01)
        self._sleep(s_time / 1000)

    def safe_pose(self):
        self._msg("[motion] về pose an toàn")
        try:
            self.arm_move_6(self.look_at, 1000)
        except _Abort:
            pass

    def go_home(self):
        """Về pose trước khi thực hiện action (không đụng gripper)."""
        self._msg("[motion] về pose trước khi chạy action")
        self.arm_move(self.p_mould, 1000)
        self._sleep(0.5)

    def heap_up(self):
        self.arm_clamp_block(0)
        self.arm_move(self.p_mould, 1000)
        self._sleep(1)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_Yellow, 1000)
        self.arm_clamp_block(1)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_layer_1, 1000)
        self.arm_clamp_block(0)
        self._sleep(.1)
        self.arm_move(self.p_mould, 1100)
        self._sleep(2)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_Red, 1000)
        self.arm_clamp_block(1)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_layer_2, 1000)
        self.arm_clamp_block(0)
        self._sleep(.1)
        self.arm_move(self.p_mould, 1100)
        self._sleep(2)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_Green, 1000)
        self.arm_clamp_block(1)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_layer_3, 1000)
        self.arm_clamp_block(0)
        self._sleep(.1)
        self.arm_move(self.p_mould, 1100)
        self._sleep(2)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_Blue, 1000)
        self.arm_clamp_block(1)
        self.arm_move(self.p_top, 1000)
        self.arm_move(self.p_layer_4, 1000)
        self.arm_clamp_block(0)
        self._sleep(.1)
        self.arm_move(self.p_mould, 1100)
        self._sleep(1)

    def dance(self):
        self._servo6(90, 90, 90, 90, 90, 90, 500)
        self._sleep(1)
        for s2, s3, s4 in [(60, 120, 60), (45, 135, 45), (60, 120, 60),
                           (90, 90, 90), (100, 80, 80), (120, 60, 60),
                           (135, 45, 45), (90, 90, 90)]:
            self._servo(2, 180 - s2, self.time_1)
            self._servo(3, s3, self.time_1)
            self._servo(4, s4, self.time_1)
            self._sleep(self.time_sleep)
        self._servo(4, 20, self.time_1)
        self._servo(6, 150, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(4, 90, self.time_1)
        self._servo(6, 90, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(4, 20, self.time_1)
        self._servo(6, 150, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(4, 90, self.time_1)
        self._servo(6, 90, self.time_1)
        self._servo(1, 0, self.time_1)
        self._servo(5, 0, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(3, 180, self.time_1)
        self._servo(4, 0, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(6, 180, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(6, 0, self.time_2)
        self._sleep(self.time_sleep)
        self._servo(6, 90, self.time_2)
        self._servo(1, 90, self.time_1)
        self._servo(5, 90, self.time_1)
        self._sleep(self.time_sleep)
        self._servo(3, 90, self.time_1)
        self._servo(4, 90, self.time_1)
        self._sleep(self.time_sleep)

    def clamp_clock(self):
        self.arm_clamp_block(0)
        self.arm_move_clamp(self.p_mould, 1000)
        self._sleep(1)
        for dest in [self.p_Yellow, self.p_Red, self.p_Green, self.p_Blue]:
            self.arm_move_clamp(self.p_top, 1000)
            self.arm_move_clamp(self.p_Brown, 1000)
            self.arm_clamp_block(1)
            self.arm_move_clamp(self.p_top, 1000)
            self.arm_move_clamp(dest, 1000)
            self.arm_clamp_block(0)
            self.arm_move_up()
            self.arm_move_clamp(self.p_mould, 1100)
            self._sleep(2)
        self._sleep(1)

    def move_block(self):
        self.arm_clamp_block(0)
        self.arm_move_clamp(self.p_mould, 1000)
        self._sleep(1)
        for layer, dest in [(self.p_move_layer_4, self.p_Yellow),
                            (self.p_move_layer_3, self.p_Red),
                            (self.p_move_layer_2, self.p_Green),
                            (self.p_move_layer_1, self.p_Blue)]:
            self.arm_move_clamp(self.p_top, 1000)
            self.arm_move_clamp(layer, 1000)
            self.arm_clamp_block(1)
            self.arm_move_clamp(self.p_top, 1000)
            self.arm_move_clamp(dest, 1000)
            self.arm_clamp_block(0)
            self._sleep(.1)
            self.arm_move_up()
            self.arm_move_clamp(self.p_mould, 1100)
            self._sleep(2)
        self._sleep(1)


class Runner:
    def __init__(self, args):
        self.args = args
        self.state = "LISTEN"
        self.pending = None  # (code, ten, fn_name)
        self.abort = threading.Event()
        self.worker = None
        self.worker_result = None
        self.log = []
        self.arm = None
        self.stt = None
        self.script = list(args.script) if args.script else None
        self.mover = Mover(arm=None, abort_event=self.abort,
                           dry_run=True, fast=True, log=self.log)
        self.cap = None

    def msg(self, s):
        self.log.append(s)
        print(s, flush=True)

    # ---------- hardware (chỉ mở khi chạy thật) ----------
    def setup(self):
        if self.args.self_test:
            self.msg("[setup] self-test: không mở serial/camera/STT")
            return
        from Arm_Lib import Arm_Device
        self.msg(f"[setup] mở serial {self.args.port} (chỉ khi EXECUTING mới lái servo)")
        self.arm = Arm_Device(com=self.args.port)
        time.sleep(0.2)
        self.mover = Mover(arm=self.arm, abort_event=self.abort,
                           dry_run=self.args.dry_run, log=self.log)
        self.msg("[setup] tải sherpa-vi...")
        self.stt = SherpaViBackend(self.args.sherpa_model)
        if not self.args.no_cam:
            import cv2
            self.cap = cv2.VideoCapture(ROBOT_CAM)
            if not self.cap.isOpened():
                print("[warn] không mở được camera, chạy không hình", flush=True)
                self.cap = None

    # ---------- nghe ----------
    def listen_text(self, exec_mode=False):
        """Trả text nghe được (mic thật hoặc kịch bản self-test)."""
        if self.script is not None:
            time.sleep(0.05)
            return self.script.pop(0) if self.script else None
        import tempfile
        win = self.args.exec_window if exec_mode else self.args.window
        path = record_wav(win, self.args.mic)
        try:
            return self.stt.transcribe(path).strip()
        finally:
            try:
                os.remove(path)
            except OSError:
                pass

    # ---------- lệnh đơn (chạy ngay ở LISTEN) ----------
    def run_simple(self, code):
        a = self.arm
        if code == 11:
            a.Arm_RGB_set(50, 0, 0) if a else self.msg("[simple] RGB đỏ")
        elif code == 12:
            a.Arm_RGB_set(0, 50, 0) if a else self.msg("[simple] RGB xanh lá")
        elif code == 13:
            a.Arm_RGB_set(0, 0, 50) if a else self.msg("[simple] RGB xanh dương")
        elif code == 14:
            a.Arm_RGB_set(50, 50, 0) if a else self.msg("[simple] RGB vàng")
        elif code == 38:
            if a:
                a.Arm_Buzzer_On(3)
            else:
                self.msg("[simple] còi")
        elif code in (39, 40):
            v = 90 if code == 39 else 50
            if a:
                for sid in (2, 3, 4):
                    a.Arm_serial_servo_write(sid, v, 1000)
                    time.sleep(0.01)
            else:
                self.msg(f"[simple] servo 2/3/4 -> {v}")
        elif code == 43:
            a.Arm_serial_servo_write(6, 180, 500) if a else self.msg("[simple] kẹp")
        elif code == 44:
            a.Arm_serial_servo_write(6, 90, 500) if a else self.msg("[simple] mở kẹp")
        time.sleep(0.5)

    # ---------- state handlers (dùng chung cho chạy thật + self-test) ----------
    def on_listen_text(self, text):
        if not text:
            return
        self.msg(f"[LISTEN] nghe: '{text}'")
        if is_stop(text):
            self.msg("[LISTEN] lệnh dừng nhưng không có action nào chạy -> bỏ qua")
            return
        code, desc, _ = match_command(text, "vi")
        if code in ACTION_CODES:
            name = ACTION_CODES[code][0]
            self.pending = (code, name, ACTION_CODES[code][1])
            self.state = "READY"
            self.msg(f"[LISTEN] lệnh ACTION '{desc}' -> READY (chờ SPACE)")
        elif code in SIMPLE_CODES:
            self.msg(f"[LISTEN] lệnh đơn '{desc}' -> chạy ngay")
            self.run_simple(code)
        elif code:
            self.msg(f"[LISTEN] mã {code} chưa hỗ trợ ở runner này -> bỏ qua")
        else:
            self.msg("[LISTEN] không khớp lệnh nào -> bỏ qua")

    def on_exec_text(self, text):
        """Ở EXECUTING: chỉ nhận STOP, còn lại bỏ qua (log rõ)."""
        if not text:
            return
        if is_stop(text):
            self.msg(f"[EXECUTING] nhận DỪNG ('{text}') -> abort action")
            self.abort.set()
        else:
            self.msg(f"[EXECUTING] bỏ qua lệnh mới ('{text}') - đang chạy action")

    def start_action(self):
        code, name, fn = self.pending
        self.abort.clear()
        self.worker_result = None

        def _run():
            try:
                self.msg(f"[worker] bắt đầu '{name}' (mã {code})")
                self.mover.go_home()
                getattr(self.mover, fn)()
                self.worker_result = "done"
                self.msg(f"[worker] '{name}' HOÀN THÀNH")
            except _Abort:
                self.msg(f"[worker] '{name}' BỊ DỪNG giữa chừng")
                self.mover.safe_pose()
                self.worker_result = "aborted"
            except Exception as e:
                self.msg(f"[worker] LỖI: {e}")
                try:
                    self.mover.safe_pose()
                except Exception:
                    pass
                self.worker_result = "error"

        self.state = "EXECUTING"
        self.worker = threading.Thread(target=_run, daemon=True)
        self.worker.start()

    def finish_action(self):
        self.worker.join(timeout=30)
        self.pending = None
        self.state = "LISTEN"
        self.msg(f"[state] về LISTEN (kết quả: {self.worker_result})")

    # ---------- vòng lặp chính (chạy thật, có camera) ----------
    def overlay(self, img, lines):
        import cv2
        y = 30
        for ln in lines:
            cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y += 30
        return img

    def grab_show(self, lines):
        if self.cap is None:
            return None
        import cv2
        ret, img = self.cap.read()
        if not ret:
            return None
        cv2.imshow("VOICE ACTION", self.overlay(img, lines))
        return cv2.waitKey(1) & 0xFF

    def run(self):
        import cv2
        self.msg("READY? Không. Bắt đầu ở LISTEN. Q = thoát.")
        try:
            while True:
                if self.state == "LISTEN":
                    key = self.grab_show(["LISTEN: noi lenh don (den/coi/tay/kep)",
                                          "lenh ACTION (xep chong/nhay...) -> READY",
                                          "Q: thoat"]) if self.cap else None
                    if key == ord("q"):
                        break
                    self.on_listen_text(self.listen_text())
                elif self.state == "READY":
                    code, name, _ = self.pending
                    t0 = time.time()
                    confirmed, cancelled = False, False
                    while time.time() - t0 < 30:
                        key = self.grab_show(
                            [f"READY: [{code}] {name}",
                             "SPACE: chay / Q: huy",
                             f"con {int(30 - (time.time() - t0))}s"]) if self.cap else None
                        if self.cap is None:
                            key = cv2.waitKey(100) & 0xFF
                        if key == 32:
                            confirmed = True
                            break
                        if key == ord("q"):
                            cancelled = True
                            break
                    if confirmed:
                        self.msg("[READY] SPACE -> EXECUTING")
                        self.start_action()
                    else:
                        self.msg("[READY] hủy/hết giờ -> LISTEN")
                        self.pending = None
                        self.state = "LISTEN"
                elif self.state == "EXECUTING":
                    key = self.grab_show(["EXECUTING: dang chay action",
                                          "moi lenh khac BI BO QUA",
                                          "noi 'dung lai' hoac phim S de dung"]) if self.cap else None
                    if key == ord("s"):
                        self.msg("[EXECUTING] phím S -> abort action")
                        self.abort.set()
                    self.on_exec_text(self.listen_text(exec_mode=True))
                    if not self.worker.is_alive():
                        self.finish_action()
        except KeyboardInterrupt:
            self.msg("dừng (Ctrl+C)")
        finally:
            self.abort.set()
            if self.worker is not None:
                self.worker.join(timeout=5)
            if self.cap is not None:
                self.cap.release()
                cv2.destroyAllWindows()

    # ---------- self-test (không phần cứng) ----------
    def self_test(self):
        ok = []
        self.msg("== SELF-TEST ==")
        # 1. lệnh action -> READY (không chạy vội)
        self.on_listen_text("xếp chồng")
        ok.append(("action -> READY", self.state == "READY" and self.pending[0] == 51))
        # 2. SPACE -> EXECUTING, worker về pose TRƯỚC khi chạy action
        self.start_action()
        ok.append(("SPACE -> EXECUTING", self.state == "EXECUTING" and self.worker.is_alive()))
        time.sleep(0.5)
        logs = "\n".join(self.log)
        ok.append(("về pose trước action",
                   "về pose trước khi chạy action" in logs
                   and logs.index("về pose trước khi chạy action") < logs.index("[motion] servo 6 -> 60")))
        # 3. lệnh khác khi đang chạy -> BỊ BỎ QUA
        time.sleep(0.3)
        st = self.state
        self.on_exec_text("đèn đỏ")
        ok.append(("lệnh khác bị bỏ qua", self.state == st and not self.abort.is_set()
                   and any("BỎ QUA" in x or "bỏ qua" in x for x in self.log)))
        # 4. lệnh dừng -> abort -> về LISTEN
        self.on_exec_text("dừng lại")
        self.finish_action()
        ok.append(("dừng -> LISTEN", self.state == "LISTEN"
                   and self.worker_result == "aborted"
                   and any("BỊ DỪNG" in x for x in self.log)))
        # 5. lệnh action khác -> READY rồi Q hủy -> LISTEN
        self.on_listen_text("nhảy múa")
        ok.append(("action2 -> READY", self.state == "READY" and self.pending[0] == 52))
        self.pending = None
        self.state = "LISTEN"
        ok.append(("hủy -> LISTEN", self.state == "LISTEN" and self.pending is None))
        # 6. lệnh đơn chạy ngay ở LISTEN
        n0 = len(self.log)
        self.on_listen_text("báo động")
        ok.append(("lệnh đơn chạy ngay", self.state == "LISTEN"
                   and any("chạy ngay" in x for x in self.log[n0:])))
        print("== KẾT QUẢ ==")
        allok = True
        for name, passed in ok:
            print(f"  [{'PASS' if passed else 'FAIL'}] {name}")
            allok = allok and passed
        print("SELF-TEST " + ("PASS" if allok else "FAIL"))
        return allok


def main():
    global ROBOT_CAM
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=ROBOT_PORT)
    ap.add_argument("--cam", default=ROBOT_CAM)
    ap.add_argument("--no-cam", action="store_true", help="chạy không cửa sổ camera")
    ap.add_argument("--dry-run", action="store_true", help="không ghi servo, chỉ log motion")
    ap.add_argument("--window", type=float, default=4, help="cửa sổ nghe ở LISTEN (giây)")
    ap.add_argument("--exec-window", type=float, default=2, help="cửa sổ nghe ở EXECUTING (giây)")
    ap.add_argument("--mic", default=None)
    ap.add_argument("--sherpa-model", default=SHERPA_MODEL_DIR)
    ap.add_argument("--script", nargs="*", default=None, help="(debug) câu giả thay mic")
    ap.add_argument("--self-test", action="store_true", help="test state machine, không phần cứng")
    args = ap.parse_args()
    ROBOT_CAM = args.cam

    r = Runner(args)
    if args.self_test:
        r.setup()
        sys.exit(0 if r.self_test() else 1)
    r.setup()
    r.run()


if __name__ == "__main__":
    main()
