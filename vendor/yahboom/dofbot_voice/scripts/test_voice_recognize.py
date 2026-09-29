#!/usr/bin/env python3
# coding: utf-8
"""Test voice recognition READ-ONLY (không motion, không camera).

Nhận diện chạy ở ĐÂU?
  - Không phải mic laptop, không phải cloud.
  - Chip ASR offline trên module speech Yahboom (mic gắn trên module).
    Module tự nhận từ vựng pinyin đã nạp (vd "xiao ya", "hong deng"...)
    rồi trả về MÃ LỆNH (int) cho laptop.
  - Laptop chỉ poll mã qua 2 đường:
      1) I2C addr 0x0f, thanh ghi 0x08 (chỉ có trên board Yahboom gốc,
         laptop này KHÔNG nối I2C -> shim MOCK).
      2) Serial /dev/ttyUSB0: Arm_serial_speech_read() đọc frame 0x2A
         -> speech_state. ĐÂY LÀ ĐƯỜNG TEST TRÊN LAPTOP NÀY.

Cách test:
  1) Chạy script này, nó poll mỗi 0.2s trong N giây, in mọi mã != 0.
  2) Nói gần mic của MODULE speech (không phải mic laptop):
     - password mode: "xiao ya" (你好小亚) để đánh thức, rồi lệnh
       vd "hong deng" (đèn đỏ), "lv deng", "lan deng"...
     - loop mode (02): nói thẳng lệnh, không cần wake word.
  3) Tra mã trong speech_ID.csv (cột M2C) — script tự dịch sẵn.

Ví dụ:
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/test_voice_recognize.py --seconds 20
  # test kèm inject giả (không cần nói, kiểm tra pipeline):
  SPEECH_MOCK_SEQ="11,38" PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/test_voice_recognize.py --seconds 8 --mock
"""
import argparse
import csv
import os
import sys
import time

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "speech_ID.csv")


def load_code_map():
    """code (M2C) -> mô tả lệnh từ speech_ID.csv"""
    cmap = {}
    try:
        with open(CSV_PATH, encoding="utf-8-sig") as f:
            for row in csv.DictReader(f):
                m2c = (row.get("M2C") or "").strip()
                if not m2c:
                    continue
                try:
                    code = int(float(m2c))
                except ValueError:
                    continue
                desc = "|".join(x for x in [
                    (row.get("功能名称") or "").strip(),
                    (row.get("功能备注") or "").strip(),
                    (row.get("语音识别内容") or "").strip(),
                ] if x)
                cmap.setdefault(code, desc)
    except FileNotFoundError:
        print(f"[warn] không thấy {CSV_PATH}, chạy không dịch mã.")
    return cmap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--mock", action="store_true",
                    help="không mở serial, chỉ test pipeline bằng SPEECH_MOCK_SEQ")
    args = ap.parse_args()

    cmap = load_code_map()
    print(f"poll voice trong {args.seconds}s (0.2s/lần). Nói vào MIC CỦA MODULE speech.")
    print("Mã != 0 sẽ in ra kèm nghĩa. Ctrl+C để dừng sớm.\n")

    use_arm = not args.mock
    arm = None
    if use_arm:
        from Arm_Lib import Arm_Device
        arm = Arm_Device(com=args.port)
        time.sleep(0.3)
        print(f"[serial] đã mở {args.port} (chỉ đọc speech_state, không motion)")

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from Speech_Lib import Speech
    sp = Speech(com=args.port, use_arm=use_arm)
    print(f"[speech] mode={'REAL-serial' if sp.arm is not None else 'MOCK'}")

    hits = []
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            raw = 0
            if arm is not None:
                try:
                    raw = arm.Arm_serial_speech_read(0) or 0
                except Exception:
                    raw = 0
            code = sp.speech_read()
            for src, val in (("serial", raw), ("speech_read", code)):
                if val:
                    msg = cmap.get(int(val), "(chưa có trong speech_ID.csv)")
                    line = f"[{time.time()-t0:5.1f}s] {src} -> {val} : {msg}"
                    print(line, flush=True)
                    hits.append(line)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\ndừng sớm (Ctrl+C)")
    finally:
        try:
            sp.close()
        except Exception:
            pass
        try:
            if arm is not None:
                del arm
        except Exception:
            pass

    print(f"\n== kết quả: {len(hits)} mã nhận được ==")
    if not hits:
        print("Toàn 0 trong cửa sổ test. Nghĩa là: không ai nói / mic module xa / "
              "module chưa nạp từ vựng / module không gắn trên expansion board.")
        print("Thử lại: password mode nói 'xiao ya' trước, rồi 'hong deng'; "
              "hoặc test pipeline: SPEECH_MOCK_SEQ='11,38' ... --mock")


if __name__ == "__main__":
    main()
