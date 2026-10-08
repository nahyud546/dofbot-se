#!/usr/bin/env python3
# coding: utf-8
"""Nói TIẾNG VIỆT và/hoặc TIẾNG TRUNG vào mic laptop -> mã lệnh robot.

Engine (offline, không cần mạng lúc chạy):
  - Tiếng Việt: DUY NHẤT sherpa-onnx + zipformer-vi-30M-int8-2026-02-09.
  - Tiếng Trung: faster-whisper (mặc định small).
Song ngữ cùng lúc (mặc định): mỗi cửa sổ decode 2 lần (sherpa-vi + fw-zh)
rồi so với cả 2 bảng lệnh.
  mic laptop (arecord 16kHz mono) -> sherpa-vi / faster-whisper-zh
  -> text -> khớp VI_CMD / ZH_CMD -> code -> in + ghi /tmp/speech_mock_code
     để các script voice sẵn có (simple_voice_ctrl.py, ...) nhặt lệnh.

Ví dụ:
  # song ngữ 20s (mặc định):
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --seconds 20
  # khóa 1 ngôn ngữ (nhanh hơn, chỉ decode 1 engine):
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --seconds 20 --lang vi
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --seconds 20 --lang zh
  # xem toàn bộ câu lệnh define sẵn 2 thứ tiếng:
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --cheatsheet
  # giải mã file wav có sẵn (tự test không cần nói):
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --wav /tmp/den_do.wav
  # chỉ in, không inject vào pipeline voice:
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/stt_vi.py --seconds 20 --no-inject

Câu lệnh define sẵn: xem --cheatsheet (bảng VI + ZH bên dưới).
"""
import argparse
import os
import subprocess
import sys
import tempfile
import unicodedata

MOCK_FILE = "/tmp/speech_mock_code"
TASK_FILE = "/tmp/dofbot_task_command"
TASK_ACTIVE_FILE = "/tmp/dofbot_task_active"
def _default_sherpa_dir():
    from pathlib import Path
    root = Path(os.environ.get("ROBOT_ARM_ROOT", Path(__file__).resolve().parents[3]))
    for c in [
        root / "ai/models/sherpa-onnx-zipformer-vi-30M-int8-2026-02-09",
        root / "models/sherpa-onnx-zipformer-vi-30M-int8-2026-02-09",  # pre-restructure
        Path("/home/jloy/Desktop/robot-arm/ai/models/sherpa-onnx-zipformer-vi-30M-int8-2026-02-09"),
        Path("/home/jloy/Desktop/robot-arm/models/sherpa-onnx-zipformer-vi-30M-int8-2026-02-09"),
    ]:
        if c.exists():
            return str(c)
    return str(root / "ai/models/sherpa-onnx-zipformer-vi-30M-int8-2026-02-09")
SHERPA_MODEL_DIR = os.environ.get("SHERPA_MODEL_DIR", _default_sherpa_dir())

# từ khóa (so khớp KHÔNG dấu) -> (code, mô tả). Thứ tự = độ ưu tiên.
# ĐÃ DEDUPE: bỏ các từ đơn 1 âm tiết dễ nhầm (do/mo/nha/tha/mang/xep/mua).
# Chỉ giữ cụm rõ nghĩa >= 2 âm tiết hoặc kèm danh từ. Khớp dài-trước.
VI_CMD = [
    ("den xanh la", 12, "đèn xanh lá"),
    ("den xanh duong", 13, "đèn xanh dương"),
    ("den xanh bien", 13, "đèn xanh dương"),
    ("xanh duong", 13, "đèn xanh dương"),
    ("xanh bien", 13, "đèn xanh dương"),
    ("xanh la", 12, "đèn xanh lá"),
    ("den vang", 14, "đèn vàng"),
    ("den do", 11, "đèn đỏ"),
    ("bao dong", 38, "còi báo động"),
    ("tat coi", 38, "tắt còi"),
    ("tay len", 39, "tay lên"),
    ("tay xuong", 40, "tay xuống"),
    ("qua trai", 41, "qua trái"),
    ("qua phai", 42, "qua phải"),
    ("mo kep", 44, "mở kẹp"),
    ("nha kep", 44, "mở kẹp"),
    ("kep lai", 43, "kẹp lại"),
    ("kep chat", 43, "kẹp lại"),
    ("gap khoi", 53, "gắp khối"),
    ("vo tay", 45, "vỗ tay"),
    ("xep chong", 51, "xếp chồng"),
    ("nhay mua", 52, "nhảy múa"),
    ("van chuyen", 54, "vận chuyển"),
]

# Task commands require an exact phrase; fuzzy matching must never start a robot task.
VI_TASK_CMD = [
    ("phan loai mau", 61, "phân loại màu"),
    ("xep chong mau", 62, "xếp chồng màu"),
    ("xep trong mau", 62, "xếp chồng màu"),  # sherpa thường nghe "chồng" thành "trồng"
    ("theo doi khuon mat", 63, "theo dõi khuôn mặt"),
    ("bam theo khuon mat", 63, "theo dõi khuôn mặt"),
    ("phan loai rac", 64, "phân loại rác"),
    ("dung bai toan", 65, "dừng bài toán"),
    ("dung robot", 65, "dừng bài toán"),
]

# Tier 2 — task ĐƠN chi tiết cho Gemini (intent constrained, qua T8 pipeline).
# STT không bắn mã số cho nhóm này (có tham số: góc/label), T8 tự parse text.
# Liệt kê ở cheatsheet để người dùng biết câu nào được phép.
VI_SKILL_EXAMPLES = [
    ("xoay sang phải thêm 30 độ", "rotate_relative", "Xoay khớp 1 thêm +30° từ góc hiện tại"),
    ("xoay sang trái thêm 15 độ nữa", "rotate_relative", "Xoay khớp 1 thêm -15° từ góc hiện tại"),
    ("cầm cục cube có dán hình xương cá lên, giữ nguyên", "vision_pick_hold", "Tìm xuong_ca → gắp lên và GIỮ (không đặt)"),
    ("gắp khối đỏ lên và giữ nguyên đó", "vision_pick_hold", "Gắp khoi_do → giữ"),
    ("đặt xuống", "place_held", "Đặt vật đang giữ xuống bàn"),
    ("thả ra", "release_hold", "Nhả vật đang giữ"),
    ("mở kẹp / kẹp lại", "gripper", "Đóng/mở kẹp đơn"),
    ("thời tiết Hà Nội hôm nay", "ask_info", "Hỏi đáp + RAG nội bộ + web khi cần"),
    ("phân loại màu như thế nào", "ask_info+RAG", "Trả lời từ KB nội bộ, không gọi web"),
]


def norm(s):
    """hạ về không dấu để so khớp: 'Đên đó' -> 'den do'."""
    s = s.lower().replace("đ", "d")
    s = "".join(c for c in unicodedata.normalize("NFD", s)
                if unicodedata.category(c) != "Mn")
    return " ".join(s.split())


# Bảng TIẾNG TRUNG (chữ Hán whisper trả về + pinyin dự phòng).
# Mỗi phần tử: (mã, mô tả, [từ khóa chữ Hán...], [từ khóa pinyin không dấu...]).
ZH_CMD = [
    (11, "đèn đỏ", ["红灯", "红色", "红", "紅燈", "紅色", "紅"], ["hong deng", "hongdeng"]),
    (12, "đèn xanh lá", ["绿灯", "绿色", "绿", "綠燈", "綠色", "綠"], ["lu deng", "ludeng"]),
    (13, "đèn xanh dương", ["蓝灯", "蓝色", "蓝", "藍燈", "藍色", "藍"], ["lan deng", "landeng"]),
    (14, "đèn vàng", ["黄灯", "黄色", "黄", "黃燈", "黃色", "黃"], ["huang deng", "huangdeng"]),
    (38, "còi báo động", ["报警"], ["bao jing", "baojing"]),
    (39, "tay lên", ["向上"], ["xiang shang", "xiangshang"]),
    (40, "tay xuống", ["向下"], ["xiang xia", "xiangxia"]),
    (41, "qua trái", ["向左"], ["xiang zuo", "xiangzuo"]),
    (42, "qua phải", ["向右"], ["xiang you", "xiangyou"]),
    (43, "kẹp lại", ["夹紧"], ["jia jin", "jiajin"]),
    (44, "mở kẹp", ["松开"], ["song kai", "songkai"]),
    (45, "vỗ tay", ["鼓掌"], ["gu zhang", "guzhang"]),
    (51, "xếp chồng", ["叠罗汉"], ["die luo han", "dieluohan"]),
    (52, "nhảy múa", ["跳舞"], ["tiao wu", "tiaowu"]),
    (53, "gắp khối", ["夹方块"], ["jia fang kuai", "jiafangkuai"]),
    (54, "vận chuyển", ["搬运"], ["ban yun", "banyun"]),
]


def norm_zh_pinyin(s):
    import re
    return re.sub(r"\s+", "", norm(s))


def match_command(text, lang="auto"):
    """Trả (code, desc, kiểu khớp). lang: vi | zh | auto (cả 2 bảng).
    Ưu tiên: task exact -> skill Tier2 (không bắn mã, để T8 parse) -> servo dài-trước -> mờ."""
    import difflib
    import re
    if lang in ("vi", "auto"):
        t = norm(text)
        for kw, code, desc in VI_TASK_CMD:
            if t == kw:
                return code, desc + " [VI]", "khớp"
        # Tier 2: nhận diện để gợi ý, không bắn mã legacy (có tham số)
        # (để T8 pipeline parse intent chi tiết). Trả None để caller forward text.
        for kw, code, desc in sorted(VI_CMD, key=lambda x: -len(x[0])):
            if re.search(r"\b" + re.escape(kw) + r"\b", t):
                return code, desc + " [VI]", "khớp"
        best = None
        for kw, code, desc in sorted(VI_CMD, key=lambda x: -len(x[0])):
            if len(kw) < 5:
                continue  # bỏ fuzzy cho cụm quá ngắn
            r = difflib.SequenceMatcher(None, t, kw).ratio()
            if best is None or r > best[0]:
                best = (r, kw, code, desc)
        if best and best[0] >= 0.8:
            return best[2], best[3] + f" [VI] (khớp mờ {best[0]:.2f})", "mờ"
    if lang in ("zh", "auto"):
        for code, desc, hans, _ in ZH_CMD:
            for kw in hans:
                if kw in text:
                    return code, desc + " [ZH]", "khớp"
        tp = norm_zh_pinyin(text)
        for code, desc, _, pins in ZH_CMD:
            for py in pins:
                if py in tp:
                    return code, desc + " [ZH-pinyin]", "khớp"
    return None, None, None


def emit(code, desc, inject=True):
    print(f"-> code {code} : {desc}", flush=True)
    if inject:
        try:
            if code < 61 and os.path.exists(TASK_ACTIVE_FILE):
                print("   (đang chạy bài toán; bỏ qua lệnh servo đơn)", flush=True)
                return
            destination = TASK_FILE if code >= 61 else MOCK_FILE
            temporary = destination + f".{os.getpid()}.tmp"
            with open(temporary, "w") as f:
                f.write(str(code))
            os.replace(temporary, destination)
            print(f"   (đã gửi tới {destination})", flush=True)
        except OSError as e:
            print(f"   [warn] không gửi được lệnh: {e}", flush=True)


class SherpaViBackend:
    """Tiếng Việt DUY NHẤT bằng sherpa-onnx zipformer-vi-30M-int8."""
    def __init__(self, model_dir=SHERPA_MODEL_DIR, num_threads=4):
        import sherpa_onnx
        self.rec = sherpa_onnx.OfflineRecognizer.from_transducer(
            tokens=f"{model_dir}/tokens.txt",
            encoder=f"{model_dir}/encoder.int8.onnx",
            decoder=f"{model_dir}/decoder.onnx",
            joiner=f"{model_dir}/joiner.int8.onnx",
            num_threads=num_threads, decoding_method="greedy_search")

    def transcribe(self, wav_path):
        import wave
        import numpy as np
        with wave.open(wav_path, "rb") as wf:
            raw = wf.readframes(wf.getnframes())
        samples = np.frombuffer(raw, dtype="<i2").astype("float32") / 32768.0
        stream = self.rec.create_stream()
        stream.accept_waveform(16000, samples)
        self.rec.decode_stream(stream)
        return stream.result.text.strip()


class FWBackend:
    """Tiếng Trung bằng faster-whisper offline. Model tải 1 lần từ HF rồi cache."""
    def __init__(self, size="small"):
        from faster_whisper import WhisperModel
        self.model = WhisperModel(size, device="cpu", compute_type="int8")

    def transcribe(self, wav_path, lang="auto"):
        # lang: vi | zh | auto(None=tự đoán mỗi cửa sổ)
        kw = {} if lang == "auto" else {"language": lang}
        segs, _ = self.model.transcribe(wav_path, **kw)
        return " ".join(s.text.strip() for s in segs).strip()


def record_wav(seconds, mic=None):
    """thu mic bằng arecord ra file wav tạm 16kHz mono, trả path."""
    fd, path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    cmd = ["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1",
           "-d", str(int(seconds)), "-t", "wav", path]
    if mic:
        cmd += ["-D", mic]
    subprocess.run(cmd, check=True)
    return path


def handle_text(text, inject, lang="auto"):
    """Trả True nếu khớp lệnh. In text nghe được + kết quả."""
    if not text:
        return False
    print(f"[nghe] '{text}'", flush=True)
    code, desc, _ = match_command(text, lang)
    if code:
        emit(code, desc, inject)
        return True
    return False


def process_wav(vi_be, zh_be, path, inject, lang):
    """Decode 1 file wav. auto = sherpa-vi + fw-zh."""
    if lang == "auto":
        got = False
        if vi_be is not None:
            got = handle_text(vi_be.transcribe(path), inject, "vi") or got
        if zh_be is not None:
            got = handle_text(zh_be.transcribe(path, "zh"), inject, "zh") or got
        if not got:
            print("   (không khớp lệnh nào)", flush=True)
    elif lang == "vi":
        if not handle_text(vi_be.transcribe(path), inject, "vi"):
            print("   (không khớp lệnh nào)", flush=True)
    else:
        if not handle_text(zh_be.transcribe(path, "zh"), inject, "zh"):
            print("   (không khớp lệnh nào)", flush=True)


def print_cheatsheet():
    print("=== TIER 1: mã số offline chắc chắn (STT -> code) ===")
    print("  -- Bài toán lớn (61-65, khớp nguyên cụm) --")
    seen_task = {}
    for kw, code, desc in VI_TASK_CMD:
        seen_task.setdefault((code, desc), []).append(kw)
    for (code, desc), kws in sorted(seen_task.items()):
        print(f"  {code:>3}  {desc:<20} nói: {', '.join(sorted(set(kws)))}")
    print("  -- Servo/đèn/còi (khớp cụm dài-trước, đã bỏ từ đơn dễ nhầm) --")
    seen = {}
    for kw, code, desc in VI_CMD:
        seen.setdefault((code, desc), []).append(kw)
    for (code, desc), kws in sorted(seen.items()):
        print(f"  {code:>3}  {desc:<20} nói: {', '.join(sorted(set(kws)))}")
    print()
    print("=== TIER 2: task đơn cho Gemini (intent constrained, qua T8) ===")
    print("  (STT không bắn mã; T8 pipeline parse text -> executor an toàn)")
    for ex, intent, desc in VI_SKILL_EXAMPLES:
        print(f"  {intent:<16} nói: \"{ex}\"  ({desc})")
    print()
    print("=== BẢNG ZH (nói tiếng Trung) ===")
    for code, desc, hans, pins in ZH_CMD:
        print(f"  {code:>3}  {desc:<14} nói: {' / '.join(hans)}  (pinyin: {pins[0]})")
    print("đánh thức (module phần cứng, không qua script này): Xiǎo Yǎ 你好小亚")
    print("An toàn: Gemini chỉ được 1 intent trong ontology")
    print(" (ask_info/open_task/stop_task/rotate_relative/vision_pick_hold/place_held/release_hold/gripper/light_beep).")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0, help="nghe N giây rồi thoát (0 = tới Ctrl+C)")
    ap.add_argument("--window", type=float, default=4, help="độ dài mỗi cửa sổ thu (giây)")
    ap.add_argument("--mic", default=None, help="thiết bị arecord -D (mặc định: default)")
    ap.add_argument("--wav", default=None, help="giải mã file wav thay vì mic")
    ap.add_argument("--lang", choices=["auto", "vi", "zh"], default="auto",
                    help="auto=song ngữ cùng lúc (mặc định); vi=sherpa-duy-nhất; zh=faster-whisper")
    ap.add_argument("--cheatsheet", action="store_true", help="in bảng câu lệnh 2 thứ tiếng rồi thoát")
    ap.add_argument("--fw-model", default="small", help="tiny|base|small|medium|large-v3-turbo (đã cache sẵn trên máy)")
    ap.add_argument("--sherpa-model", default=SHERPA_MODEL_DIR, help="thư mục model sherpa-onnx tiếng Việt")
    ap.add_argument("--no-inject", action="store_true")
    args = ap.parse_args()
    inject = not args.no_inject

    if args.cheatsheet:
        print_cheatsheet()
        return

    vi_be = SherpaViBackend(args.sherpa_model) if args.lang in ("auto", "vi") else None
    zh_be = FWBackend(args.fw_model) if args.lang in ("auto", "zh") else None

    if args.wav:
        process_wav(vi_be, zh_be, args.wav, inject, args.lang)
        return

    import time
    print(f"nghe mic... (nói rõ từng cụm lệnh; vi=sherpa, zh=fw-{args.fw_model}, lang={args.lang})", flush=True)
    t0 = time.time()
    try:
        while True:
            if args.seconds and time.time() - t0 >= args.seconds:
                break
            win = args.window
            if args.seconds:
                win = min(win, max(1, args.seconds - (time.time() - t0)))
            path = record_wav(win, args.mic)
            try:
                process_wav(vi_be, zh_be, path, inject, args.lang)
            finally:
                try:
                    os.remove(path)
                except OSError:
                    pass
    except KeyboardInterrupt:
        print("\ndừng (Ctrl+C)")


if __name__ == "__main__":
    main()
