#!/usr/bin/env python3
# coding: utf-8
"""Phát âm thanh từ text bất kỳ (TTS giọng Việt) qua loa laptop.

Dùng Microsoft Edge TTS online (cần mạng), giọng mặc định vi-VN-HoaiMyNeural.
Không liên quan module speech của robot (module đó chỉ phát câu mẫu theo ID
qua Arm_ask_speech, không phát text tự do từ laptop).

Ví dụ:
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/speak_text.py "Xin chào, tôi là robot"
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/speak_text.py "Hello" --voice en-US-AvaNeural --no-play --out /tmp/hi.mp3
  PYTHONPATH="$PWD/dofbot_voice/scripts" python3 dofbot_voice/scripts/speak_text.py --list-vi
"""
import argparse
import asyncio
import os
import importlib.util
import site
import subprocess
import sys
import tempfile

DEFAULT_VOICE = "vi-VN-HoaiMyNeural"


def ensure_edge_tts():
    """Dùng bản cài user-site khi script chạy trong venv isolated."""
    if importlib.util.find_spec("edge_tts") is None:
        site.addsitedir(site.getusersitepackages())
    if importlib.util.find_spec("edge_tts") is None:
        raise RuntimeError("Thiếu edge_tts; cài bằng python -m pip install edge-tts")


async def _list_vi():
    ensure_edge_tts()
    import edge_tts
    voices = await edge_tts.list_voices()
    for v in voices:
        if (v.get("Locale") or "").startswith("vi"):
            print(f'{v["ShortName"]}  ({v.get("Gender")})')


async def _save(text, voice, rate, out):
    ensure_edge_tts()
    import edge_tts
    comm = edge_tts.Communicate(text, voice, rate=rate)
    await comm.save(out)


def play_file(path):
    # ffplay có sẵn trên máy; -nodisp để không mở cửa sổ video
    r = subprocess.run(
        ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", path],
        stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        print(f"[warn] ffplay lỗi: {r.stderr.strip()}", file=sys.stderr)
    return r.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("text", nargs="?", help="text cần phát")
    ap.add_argument("--voice", default=DEFAULT_VOICE)
    ap.add_argument("--rate", default="+0%")
    ap.add_argument("--out", default=None, help="lưu mp3 ra file (mặc định phát xong xóa)")
    ap.add_argument("--no-play", action="store_true", help="chỉ tạo file, không phát")
    ap.add_argument("--list-vi", action="store_true", help="liệt kê giọng tiếng Việt")
    args = ap.parse_args()

    if args.list_vi:
        asyncio.run(_list_vi())
        return
    if not args.text:
        ap.error("thiếu text. VD: speak_text.py \"Xin chào\"")

    keep = args.out or os.path.join(tempfile.gettempdir(), "speak_text.mp3")
    asyncio.run(_save(args.text, args.voice, args.rate, keep))
    size = os.path.getsize(keep)
    print(f"đã tạo {keep} ({size} bytes, voice={args.voice})")
    if size == 0:
        print("file rỗng, TTS thất bại", file=sys.stderr)
        sys.exit(1)
    if not args.no_play:
        print("đang phát qua loa...")
        rc = play_file(keep)
        print("phát xong" if rc == 0 else "phát lỗi")
        if not args.out:
            try:
                os.remove(keep)
            except OSError:
                pass
        if rc != 0:
            sys.exit(rc)


if __name__ == "__main__":
    main()
