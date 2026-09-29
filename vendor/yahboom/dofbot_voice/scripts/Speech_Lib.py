#!/usr/bin/env python3
# coding: utf-8
"""Speech_Lib shim tái tạo cho laptop (robot thật: /dev/video2, serial /dev/myserial->ttyUSB0).

Bản gốc Yahboom không có trong repo, file này tái tạo API dùng bởi
dofbot_voice/scripts/*.py để các file trở nên runnable:

  from Speech_Lib import Speech
  s = Speech()
  code = s.speech_read()   # int, 0 = không có lệnh mới
  s.void_write(code)       # phát câu thoại tương ứng mã
  s.close()

Luồng thật: module speech (I2C 0x0f hoặc qua serial board mở rộng)
  -> speech_read() poll mã lệnh (tra speech_ID.csv)
  -> void_write() phát loa.
Shim này ưu tiên hardware thật qua Arm_Lib serial, nếu không có thì
chạy MOCK để test logic voice->action mà không cần nói:
  - void_write(): in log thay vì phát loa.
  - speech_read(): trả 0, nhưng có thể inject mã để test bằng:
      * file /tmp/speech_mock_code chứa số (vd: echo 51 > /tmp/speech_mock_code)
      * hoặc env SPEECH_MOCK_CODE="51" (dùng 1 lần rồi xóa)
      * hoặc env SPEECH_MOCK_SEQ="11,12,38" (trả lần lượt mỗi lần poll)
"""
import os
import time

MOCK_FILE = "/tmp/speech_mock_code"


class Speech:
    def __init__(self, com="/dev/myserial", use_arm=True):
        self.com = com
        self.arm = None
        self.mock = False
        self._seq = []
        _seq_env = os.environ.get("SPEECH_MOCK_SEQ", "").strip()
        if _seq_env:
            try:
                self._seq = [int(x) for x in _seq_env.split(",") if x.strip() != ""]
            except ValueError:
                self._seq = []
        if use_arm:
            try:
                import Arm_Lib
                # Arm_Device mở serial; nếu /dev/myserial chưa có mà
                # /dev/ttyUSB0 có thì thử fallback để không crash import.
                try:
                    self.arm = Arm_Lib.Arm_Device(com=com)
                except Exception as e1:
                    if com == "/dev/myserial" and os.path.exists("/dev/ttyUSB0"):
                        self.arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
                    else:
                        raise e1
                time.sleep(0.1)
            except Exception as e:
                print(f"[Speech_Lib] MOCK mode (Arm serial unavailable: {e})")
                self.arm = None
                self.mock = True
        else:
            self.mock = True

    # -- API chính --
    def speech_read(self):
        # 1. inject test qua file (cho phép test V3 không cần mic)
        try:
            if os.path.exists(MOCK_FILE):
                with open(MOCK_FILE) as f:
                    txt = f.read().strip()
                if txt:
                    code = int(txt.split()[0])
                    # xóa sau khi đọc để không lặp vô hạn
                    try:
                        os.remove(MOCK_FILE)
                    except OSError:
                        pass
                    print(f"[Speech_Lib MOCK] injected code {code}")
                    return code
        except Exception:
            pass
        # 2. inject qua env 1 lần
        env_code = os.environ.get("SPEECH_MOCK_CODE", "").strip()
        if env_code:
            try:
                code = int(env_code.split()[0])
                del os.environ["SPEECH_MOCK_CODE"]
                print(f"[Speech_Lib MOCK] env code {code}")
                return code
            except ValueError:
                pass
        # 3. sequence
        if self._seq:
            code = self._seq.pop(0)
            print(f"[Speech_Lib MOCK] seq code {code}")
            return code
        # 4. hardware thật qua Arm serial
        if self.arm is not None:
            try:
                # Arm_Lib egg: Arm_serial_speech_read(id) -> speech_state
                res = self.arm.Arm_serial_speech_read(0)
                if res is None:
                    return 0
                return int(res)
            except Exception:
                return 0
        # 5. mock idle
        return 0

    def void_write(self, code):
        if self.arm is not None:
            try:
                # Arm_ask_speech(id) gửi 0x60+id -> board phát câu thoại
                if hasattr(self.arm, "Arm_ask_speech"):
                    self.arm.Arm_ask_speech(int(code))
                    return
            except Exception as e:
                print(f"[Speech_Lib] Arm_ask_speech({code}) failed: {e}")
        print(f"[Speech_Lib MOCK] void_write({code})")

    def close(self):
        try:
            if self.arm is not None:
                del self.arm
        except Exception:
            pass
        self.arm = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
