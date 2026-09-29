"""Bounded Gemini + Tavily planner for the DOFBOT task manager.

Cost-smart design (v2):
  Layer 0 - Noise gate (0 API): empty / filler / too short / duplicate.
  Layer 1 - Local router (0 API): robot tasks, chitchat, help, time, math.
  Layer 2 - Cache (0 API): exact-match reply cache + Tavily result cache.
  Layer 3 - Gemini (1-2 calls max): with short history, strict search policy.
  Layer 4 - Tavily (0-1 call): double-gated, cached.

Return dict of run(): {"reply", "action", "counts", "source"}
  source in {"filtered","local","cache","llm","llm+search","llm+search(cached)","error"}
Compatible with old callers that only read reply/action/counts.
"""

import ast
import json
import os
import re
import time
import unicodedata
from dataclasses import dataclass, field
from datetime import datetime
from difflib import SequenceMatcher
from pathlib import Path


ACTIONS = {"none": None, "color": 61, "stack": 62, "face": 63,
           "trash": 64, "stop": 65}
MODEL = "gemini-3.6-flash"

# Constrained intents (mirror task_ontology.yaml). Gemini may ONLY emit these.
# Executor (t8_executor.py) re-validates before touching hardware.
INTENTS = {"ask_info", "open_task", "stop_task", "rotate_relative",
           "vision_pick_hold", "place_held", "release_hold", "gripper", "light_beep"}
# Which intents map to legacy manager actions (the rest => action none + executor skill)
INTENT_TO_ACTION = {"ask_info": "none", "open_task": "open_task:*",
                    "stop_task": "stop", "rotate_relative": "none",
                    "vision_pick_hold": "none", "place_held": "none",
                    "release_hold": "none", "gripper": "none", "light_beep": "none"}

SYSTEM = """Bạn là trợ lý tiếng Việt cho DOFBOT để bàn. AN TOÀN LÀ TRÊN HẾT.
Bạn KHÔNG điều khiển servo trực tiếp. Bạn chỉ được trả JSON duy nhất với schema:
{"reply": str ngắn gọn,
 "intent": một trong ask_info,open_task,stop_task,rotate_relative,vision_pick_hold,place_held,release_hold,gripper,light_beep,
 "entities": object tham số (có thể rỗng {}),
 "need_vision": true/false (cần nhìn camera không),
 "search_query": chuỗi tìm kiếm web hoặc ""}

LUẬT INTENT (bắt buộc):
- ask_info: hỏi đáp thuần (thời tiết, tin tức, chào hỏi, mô tả ảnh). entities={}.
- open_task: MỞ bài toán lớn, entities={"task": một trong color,stack,face,trash}.
  Chỉ dùng khi người dùng nói rõ tên bài toán ("phân loại màu", "xếp chồng màu", "theo dõi khuôn mặt", "phân loại rác").
- stop_task: "dừng bài toán/dừng robot". entities={}.
- rotate_relative: xoay THÊM delta độ từ góc HIỆN TẠI. entities={"joint":1, "delta_deg": số -90..90}.
  "xoay phải thêm 30 độ" => {"joint":1,"delta_deg":30}. "xoay trái thêm 15 độ" => {"joint":1,"delta_deg":-15}.
  Không bao giờ đoán góc tuyệt đối, không tự đặt joint khác 1 trừ khi người dùng nêu khớp.
- vision_pick_hold: gắp LÊN VÀ GIỮ (không đặt xuống). entities={"label": mô tả vật, "hold":true}.
  "cầm cục cube xương cá lên" => {"label":"xuong_ca","hold":true}. Nếu không dặn hạ/đặt => hold=true.
- place_held: "đặt xuống/hạ xuống" => entities={"bin":"ban"} (mặc định).
- release_hold: "thả ra/nhả kẹp" => entities={}.
- gripper: "mở kẹp/kẹp lại" => entities={"state":"open"|"close"}.
- light_beep: đèn/còi => entities={"device":"red|green|blue|yellow|beep","state":"on"|"off"}.
- CẤM: tự bịa intent khác, CẤM servo/shell/pose tuyệt đối, CẤM đặt hold=false khi người dùng không yêu cầu đặt.
- need_vision=true khi câu nhắc camera/ảnh ("trong ảnh", "qua camera",
  "nhìn thấy", "đọc chữ") HOẶC động từ tìm vật + vật thể/không gian
  ("cầm/gắp/nhặt/lấy/dọn/xếp" + tên vật/màu/vị trí, "cái này/kia",
  "bên trái/phải", "màu này") HOẶC intent vision_pick_hold.
- search_query: ĐỂ TRỐNG cho chào hỏi, lệnh robot, mô tả ảnh, toán, giờ/ngày.
  Chỉ điền khi CẦN thông tin mới Internet (thời tiết hôm nay, tin tức, giá 2025-2026).

Ví dụ:
Q "phân loại màu" => intent=open_task entities={"task":"color"} action suy ra color.
Q "cầm cục cube xương cá lên, giữ nguyên đó" => intent=vision_pick_hold entities={"label":"xuong_ca","hold":true}.
Q "xoay sang phải thêm 30 độ nữa" => intent=rotate_relative entities={"joint":1,"delta_deg":30}.
Q "thời tiết Hà Nội hôm nay" => intent=ask_info entities={} search_query="thời tiết Hà Nội hôm nay".
Nếu có ảnh: mô tả/OCR từ ảnh, không chắc thì nói không chắc.
"""

# ----------------------------------------------------------------------------
# Normalization + gates
# ----------------------------------------------------------------------------

def norm_text(s: str) -> str:
    s = (s or "").strip().lower()
    s = re.sub(r"\s+", " ", s)
    return s


def norm_nodau(s: str) -> str:
    s = norm_text(s).replace("đ", "d")
    s = "".join(c for c in unicodedata.normalize("NFD", s)
                if unicodedata.category(c) != "Mn")
    return s


FILLERS = {
    "ừ", "ừm", "ừm ừm", "à", "ờ", "ờm", "alo", "hả", "gì",
    "uh", "uhm", "um", "mm", "ơ", "...", ".", "!",
}

# ASR noise shorter than this (after strip) is dropped without API.
MIN_LEN = 2

# Duplicate suppression window (seconds).
DEDUP_WINDOW = 8.0

# Task keywords: (action, reply, [nodau keyword groups]).
# Order = priority. Exact task phrase wins over fuzzy servo words.
TASK_RULES = [
    ("stop", "Đã dừng bài toán.",
     ["dung bai toan", "dung robot", "dung lai", "huy bai toan", "thoat bai toan"]),
    ("color", "Mở bài toán phân loại màu.",
     ["phan loai mau", "nhan dien mau", "phan biet mau"]),
    ("stack", "Mở bài toán xếp chồng màu.",
     ["xep chong mau", "xep trong mau", "xep chong 4 khoi", "xep chong"]),
    ("face", "Mở bài toán theo dõi khuôn mặt.",
     ["theo doi khuon mat", "bam theo khuon mat", "nhan dien khuon mat"]),
    ("trash", "Mở bài toán phân loại rác.",
     ["phan loai rac", "phan biet rac"]),
]

# Labels for vision_pick_hold (nodau -> canonical label)
VISION_LABELS = [
    ("xuong ca", "xuong_ca"),
    ("xuongca", "xuong_ca"),
    ("fishbone", "xuong_ca"),
    ("giay ve sinh", "giay_ve_sinh"),
    ("toilet paper", "giay_ve_sinh"),
    ("xanh duong", "khoi_xanh_duong"),
    ("cube xanh duong", "khoi_xanh_duong"),
    ("khoi do", "khoi_do"),
    ("khoi xanh", "khoi_xanh"),
    ("khoi vang", "khoi_vang"),
    ("cube do", "khoi_do"),
    ("cube xanh", "khoi_xanh"),
    ("cube vang", "khoi_vang"),
    ("rac tai che", "rac_tai_che"),
    ("pin", "pin"),
]


def cube_label(phrase):
    t = norm_nodau(phrase)
    for keyword, label in VISION_LABELS:
        if label in ("xuong_ca", "giay_ve_sinh", "khoi_do", "khoi_xanh",
                     "khoi_xanh_duong", "khoi_vang") and keyword in t:
            return label
    if any(word in t for word in ("cube", "khoi", "lap phuong")):
        return "cube"
    return None


def try_local_stack_sequence(q):
    """Parse one explicit source cube -> target cube command without an LLM."""
    t = norm_nodau(q).strip(" .,!")
    match = re.match(r"^(?:gap|cam|nhat)\s+(.+?)\s+(?:len\s+(?:roi|va)\s+)?"
                     r"(?:dat|xep)\s+len\s+(.+)$", t)
    if match is None:
        match = re.match(r"^xep\s+(.+?)\s+len\s+(.+)$", t)
    if match is None:
        return None
    source, target = cube_label(match.group(1)), cube_label(match.group(2))
    if source is None or target is None:
        return None
    return {"source": source, "target": target}


def validate_intent(data):
    """Ép Gemini về ontology an toàn. Trả (intent, entities, action).

    - intent lạ => ask_info/none. entities sai kiểu => reset default an toàn.
    - open_task.task lạ => ask_info. rotate delta out-of-range => clamp về ±90
      (executor sẽ từ chối lần nữa nếu vẫn nguy hiểm).
    """
    intent = data.get("intent", None)
    if intent is None:
        # Model cũ / mock chỉ trả action (compat): map legacy action
        old = data.get("action", "none")
        if old in ("color", "stack", "face", "trash"):
            return "open_task", {"task": old}, old
        if old == "stop":
            return "stop_task", {}, "stop"
        return "ask_info", {}, "none"
    if not isinstance(intent, str) or intent not in INTENTS:
        return "ask_info", {}, "none"
    ent = data.get("entities")
    if not isinstance(ent, dict):
        ent = {}
    if intent == "open_task":
        t = ent.get("task", "")
        if t not in ("color", "stack", "face", "trash"):
            return "ask_info", {}, "none"
        return intent, {"task": t}, t
    if intent == "stop_task":
        return intent, {}, "stop"
    if intent == "rotate_relative":
        try:
            j = int(ent.get("joint", 1))
        except (TypeError, ValueError):
            j = 1
        try:
            d = float(ent.get("delta_deg", 0))
        except (TypeError, ValueError):
            d = 0
        j = max(1, min(6, j))
        d = max(-90.0, min(90.0, d))
        # delta 0 = vô nghĩa => coi như hỏi đáp
        if d == 0:
            return "ask_info", {}, "none"
        return intent, {"joint": j, "delta_deg": d}, "none"
    if intent == "vision_pick_hold":
        label = str(ent.get("label", "") or "")[:60].strip() or "vat_the"
        hold = ent.get("hold", True)
        hold = True if hold is not False else False
        return intent, {"label": label, "hold": hold}, "none"
    if intent == "place_held":
        b = str(ent.get("bin", "ban") or "ban")[:40]
        return intent, {"bin": b}, "none"
    if intent == "release_hold":
        return intent, {}, "none"
    if intent == "gripper":
        s = ent.get("state", "")
        if s not in ("open", "close"):
            return "ask_info", {}, "none"
        return intent, {"state": s}, "none"
    if intent == "light_beep":
        dev, st = ent.get("device", ""), ent.get("state", "")
        if dev not in ("red", "green", "blue", "yellow", "beep") or st not in ("on", "off"):
            return "ask_info", {}, "none"
        return intent, {"device": dev, "state": st}, "none"
    return "ask_info", {}, "none"


def try_local_rotate(q):
    """Parse 'xoay phải/trái thêm N độ' (0 API). Trả (intent, entities, reply) hoặc None."""
    t = norm_nodau(q)
    if "xoay" not in t and "quay" not in t:
        return None
    # joint: "khop 2" / "joint 3", mặc định 1 (đế)
    joint = 1
    m = re.search(r"(khop|joint)\s*([1-6])", t)
    if m:
        joint = int(m.group(2))
    # delta: số + "do"
    num = None
    m = re.search(r"(\d+(?:[.,]\d+)?)\s*do", t)
    if m:
        try:
            num = float(m.group(1).replace(",", "."))
        except ValueError:
            num = None
    if num is None:
        # "xoay thêm chút" không rõ góc => không đoán, để Gemini hỏi lại
        return None
    if num > 90:
        num = 90.0
    neg = any(k in t for k in ["trai", "nguoc", "am"])
    pos = any(k in t for k in ["phai", "thuan"])
    # "xoay thêm 30 độ nữa" không ghi hướng => mặc định phải (+)
    delta = -num if (neg and not pos) else num
    huong = "trái" if delta < 0 else "phải"
    reply = f"Xoay khớp {joint} sang {huong} thêm {abs(delta):g} độ từ góc hiện tại."
    return "rotate_relative", {"joint": joint, "delta_deg": delta}, reply


def try_local_hold_place(q):
    """Parse giữ/đặt/thả/kẹp (0 API)."""
    t = norm_nodau(q)
    # release trước (tránh nhầm với "mở kẹp")
    if any(k in t for k in ["tha ra", "nha kep", "nha ra", "thach ra"]):
        return "release_hold", {}, "Nhả vật đang giữ."
    if any(k in t for k in ["dat xuong", "ha xuong", "dat vat xuong", "bo xuong"]):
        return "place_held", {"bin": "ban"}, "Đặt vật đang giữ xuống."
    if any(k in t for k in ["mo kep", "mo gap"]):
        # "mở kẹp" đơn (không kèm gắp vật) => gripper
        if "gap" not in t or "giua" in t or len(t) < 12:
            return "gripper", {"state": "open"}, "Mở kẹp."
    if any(k in t for k in ["kep lai", "kep chat", "dong kep"]):
        return "gripper", {"state": "close"}, "Kẹp lại."
    # vision_pick_hold: "cầm/gắp ... lên" + "giữ"
    if any(v in t for v in ["cam ", "cam cuc", "gap ", "gap khoi", "nhat "]) and \
            any(k in t for k in ["len", "giu", "xuong ca", "fishbone"]):
        label = cube_label(t) or "vat_the"
        # nếu không dặn đặt => hold=true (đúng yêu cầu xương cá)
        return "vision_pick_hold", {"label": label, "hold": True}, \
            f"Tìm {label} rồi gắp lên và giữ nguyên (không đặt xuống)."
    return None


def try_local_motion_sequence(q):
    """Recognize an explicit pick -> rotate -> place command in one utterance."""
    t = norm_nodau(q)
    rotate = re.search(r"\b(?:xoay|quay)\b", t)
    place = re.search(r"\b(?:dat|ha|bo)\s+xuong\b", t)
    if rotate is None or place is None or rotate.start() >= place.start():
        return None
    pick_step = try_local_hold_place(t[:rotate.start()])
    rotate_step = try_local_rotate(t[rotate.start():place.start()])
    place_step = try_local_hold_place(t[place.start():])
    if (pick_step is None or pick_step[0] != "vision_pick_hold" or
            pick_step[1].get("label") != "xuong_ca" or
            rotate_step is None or rotate_step[1].get("joint") != 1 or
            place_step is None or place_step[0] != "place_held"):
        return None
    return [pick_step, rotate_step, place_step]

def try_local_light(q):
    """Parse bật/tắt đèn/còi (0 API). Trả (intent, entities, reply) hoặc None."""
    t = norm_nodau(q)
    if "den" not in t and "coi" not in t and "beep" not in t:
        return None
    dev = None
    # "den xanh duong" chứa "den xanh" -> check blue trước green
    for d, kws in [("blue", ["den xanh duong", "den blue"]),
                   ("red", ["den do"]), ("green", ["den xanh"]),
                   ("yellow", ["den vang"]), ("beep", ["coi", "beep"])]:
        if any(k in t for k in kws):
            dev = d
            break
    if dev is None:
        return None
    if any(k in t for k in ["bat ", "bat den", "bat coi", "mo den", "mo coi"]):
        st = "on"
    elif "tat " in t or "tat den" in t or "tat coi" in t:
        st = "off"
    else:
        return None  # "đèn đỏ là gì" -> không đoán, để hỏi đáp
    ten = {"red": "đỏ", "green": "xanh", "blue": "xanh dương",
           "yellow": "vàng", "beep": "còi"}[dev]
    verb = "Bật" if st == "on" else "Tắt"
    tgt = f"đèn {ten}" if dev != "beep" else "còi"
    return "light_beep", {"device": dev, "state": st}, f"{verb} {tgt}."


# ----------------------------------------------------------------------------
# General router: single source of truth local <-> Gemini (thay keyword lẻ)
# ----------------------------------------------------------------------------
# Thêm động từ/danh từ MỚI chỉ cần thêm 1 từ vào đúng nhóm dưới đây,
# không sửa logic ở nơi khác.

# Động từ cần TÌM vật qua camera (gắp/di chuyển). Đặt/thả (đồ đang giữ)
# không cần ảnh nên KHÔNG ở đây — executor check holding là đủ.
FIND_VERBS = ("cam ", "cam cuc", "gap ", "gap khoi", "nhat ", "lay ",
              "vot ", "kep lay", "xep ", "xep chong", "don ", "don dep",
              "dep ", "day ", "keo ", "di chuyen", "dua ", "dem ", "chuyen ")
ROT_VERBS = ("xoay", "quay")

GENERIC_NOUNS = ("khoi", "cuc", "cube", "vat", "cai", "hop", "chai", "pin",
                 "bong", "tui", "goi", "nap")
COLORS = ("do", "xanh", "vang", "den", "trang", "tim", "cam", "hong", "nau")
SHAPES = ("tron", "vuong", "dai", "ngan", "to", "nho", "lon")

# Từ camera tường minh (kế thừa VISION_WORDS cũ, trừ nhóm động từ đã tách).
CAMERA_WORDS = ("trong anh", "tren anh", "hinh anh", "qua camera",
                "tren camera", "tu camera", "truoc camera", "nhin thay",
                "doc chu", "doc van ban", "ocr", "vat truoc mat")

# Chỉ định không gian: cụm nhiều từ + "kia" đơn ("nay" đơn bị loại vì
# "hôm nay" là thời gian, không phải không gian).
DEIXIS_PHRASES = ("cai nay", "cai kia", "vat nay", "vat kia", "mau nay",
                  "ben trai", "ben phai", "phia truoc", "phia sau",
                  "o giua", "o truoc", "o sau", "o day", "o do", "o kia")

KNOWLEDGE_MARKS = (" la gi", " la sao", " nhu the nao", " mau gi",
                   " cai gi", " o dau", " bao nhieu", " nghia la")


def _pad(t: str) -> str:
    return " " + t + " "


def _has_find_verb(t: str) -> bool:
    tp = _pad(t)
    return any(v in tp for v in FIND_VERBS)


def _has_rot_verb(t: str) -> bool:
    return "xoay" in t or "quay" in t


def _has_camera_word(t: str) -> bool:
    return any(k in t for k in CAMERA_WORDS)


def _has_deixis(t: str) -> bool:
    if any(k in t for k in DEIXIS_PHRASES):
        return True
    tp = _pad(t)
    # "kia" đơn nhưng trừ "hôm kia" (thời gian)
    return " kia " in tp and "hom kia" not in t


def _has_object_slot(t: str) -> bool:
    """Có nhắc vật thể cụ thể/khái quát không (màu, nhãn, danh từ...)."""
    for kw, _canon in VISION_LABELS:
        if kw in t:
            return True
    tp = _pad(t)
    for c in COLORS:
        if f" mau {c} " in tp:
            return True
        for n in GENERIC_NOUNS:
            if f" {n} {c} " in tp or f" {n} mau {c} " in tp:
                return True
    for s in SHAPES:
        for n in GENERIC_NOUNS:
            if f" {n} {s} " in tp:
                return True
    return any(f" {n} " in tp or f" {n}" == tp[-len(n) - 1:] or
               tp.startswith(f" {n} ") for n in GENERIC_NOUNS)


def _object_specific(t: str) -> bool:
    """Vật thể đủ rõ để vision phân giải (nhãn/màu/hình/danh từ không kèm
    chỉ định mơ hồ). 'gắp cái kia' -> False; 'cầm khối'/'khối đỏ' -> True."""
    for kw, _canon in VISION_LABELS:
        if kw in t:
            return True
    tp = _pad(t)
    for c in COLORS:
        if f" mau {c} " in tp:
            return True
        for n in GENERIC_NOUNS:
            if f" {n} {c} " in tp or f" {n} mau {c} " in tp:
                return True
    for s in SHAPES:
        for n in GENERIC_NOUNS:
            if f" {n} {s} " in tp:
                return True
    # Danh từ chung TRƠN (không kèm "này/kia") vẫn cho qua vision —
    # Gemini nhìn ảnh sẽ mô tả, còn hơn bắt user gõ lại.
    if _has_deixis(t):
        return False
    return any(f" {n} " in tp for n in GENERIC_NOUNS)


def _is_question(q: str, t: str) -> bool:
    if "?" in q:
        return True
    if any(k in t for k in KNOWLEDGE_MARKS):
        return True
    return t.startswith("co ") or t.startswith("day la ") or " khong" in t


@dataclass
class Needs:
    """Kết quả router: đi đâu + có cần ảnh không + câu hỏi làm rõ."""
    route: str  # "local" | "llm_text" | "llm_vision" | "clarify"
    need_image: bool = False
    clarify_msg: str = ""


CLARIFY_ROTATE = ("Xoay khớp mấy, thêm bao nhiêu độ? "
                  "Ví dụ: 'xoay phải thêm 30 độ' hoặc 'khớp 2 trái 15 độ'.")
CLARIFY_OBJECT = ("Gắp vật nào? Nói màu/hình rõ hơn, ví dụ 'khối đỏ tròn'. "
                  "Nếu vật đang trước camera, gõ '/see gắp ...' để tôi nhìn.")


def assess_needs(query, has_image=False):
    """Router general duy nhất: local chắc chắn / clarify 0-call /
    llm_text / llm_vision. Assistant + Pipeline cùng dùng (1 nguồn sự thật).

    Nguyên tắc rủi ro: hỏi (question/deixis) -> dám nhìn camera;
    hành động (gắp/di chuyển) mà vật mơ hồ + chưa có ảnh -> hỏi lại,
    KHÔNG đoán, KHÔNG tốn call.
    """
    q = (query or "").strip()
    t = norm_nodau(q)

    # 1. Task lớn chính xác (match_task đã chặn câu hỏi kiến thức) -> local
    action, _reply = match_task(q)
    if action:
        return Needs("local")

    # 2. Cần nhìn camera?
    vneed = _has_find_verb(t) or _has_camera_word(t) or _has_deixis(t)
    if vneed:
        if _has_find_verb(t):
            # Hành động: vật rõ/có ảnh/hỏi khả năng -> vision;
            # vật mơ hồ + chưa ảnh -> hỏi lại, không đoán.
            if has_image or _object_specific(t) or _is_question(q, t):
                return Needs("llm_vision", True)
            return Needs("clarify", False, CLARIFY_OBJECT)
        # Hỏi/mô tả theo camera hoặc chỉ định không gian -> vision
        return Needs("llm_vision", True)

    # 3. Xoay: đủ số+độ -> local; thiếu -> hỏi lại (không đoán delta)
    if _has_rot_verb(t):
        if try_local_rotate(q):
            return Needs("local")
        return Needs("clarify", False, CLARIFY_ROTATE)

    # 4. Local chắc chắn còn lại: đặt/thả/kẹp/đèn/giờ/toán/chitchat
    h = try_local_hold_place(q)
    if h and h[0] != "vision_pick_hold":
        return Needs("local")
    if try_local_light(q):
        return Needs("local")
    if try_local_time(q) or try_local_math(q) or match_chitchat(q):
        return Needs("local")

    # 5. Còn lại mơ hồ -> 1 call Gemini text (RAG/cache xét tiếp trong run)
    return Needs("llm_text")


CHITCHAT = [
    (["chao", "hello", "hi ", "xin chao"], "Chào bạn, tôi là robot để bàn DOFBOT."),
    (["cam on", "thank"], "Không có gì, rất vui được giúp bạn."),
    (["tam biet", "bye"], "Tạm biệt, hẹn gặp lại."),
    (["ban la ai", "ten ban la gi", "gioi thieu"], "Tôi là trợ lý DOFBOT để bàn, điều khiển phân loại màu, xếp chồng, theo dõi mặt và phân loại rác."),
    (["ban lam duoc gi", "chuc nang", "giup duoc gi", "huong dan su dung"],
     "Tôi làm được: phân loại màu, xếp chồng màu, theo dõi khuôn mặt, phân loại rác, mô tả ảnh camera, trả lời câu hỏi có tìm web khi cần. Hãy nói ví dụ 'phân loại màu' hoặc 'thời tiết Hà Nội hôm nay'."),
    (["khoe khong", "suc khoe"], "Tôi vẫn ổn và sẵn sàng giúp bạn."),
    (["ok", "dong y"], "Đã rõ."),
]

# Fresh-info hints: only these topics may trigger Tavily.
FRESH_HINTS = [
    "thoi tiet", "nhiet do", "mua", "bao ", "du bao",
    "tin tuc", "tin moi", "moi nhat", "hom nay", "hien tai",
    "gia ", "gia vang", "ty gia", "chung khoan", "bitcoin",
    "lich thi dau", "ket qua bong da", "weather", "news", "price",
    "2025", "2026",
]

# Topics that must NEVER trigger search (even if Gemini asks).
NO_SEARCH_HINTS = [
    "phan loai", "xep chong", "theo doi", "khuon mat", "dung bai toan",
    "chao", "cam on", "tam biet", "ban la ai", "lam duoc gi",
    "trong anh", "tren anh", "qua camera", "nhin thay", "doc chu",
]


def is_filler(q: str) -> bool:
    t = norm_nodau(q).strip(" .!?,…")
    return t in FILLERS or len(t) < MIN_LEN


def match_task(q: str):
    """Return (action, reply) or (None, None). 0-API deterministic."""
    t = " " + norm_nodau(q) + " "
    # Câu hỏi kiến thức ("là gì/như thế nào/quy trình...") -> để RAG/LLM trả lời,
    # không mở task robot. (không yêu cầu space cuối vì có thể dính dấu ?/.)
    if any(k in t for k in ["nhu the nao", " la gi", " la sao", " gom may",
                            " quy trinh", " huong dan", " giai thich",
                            " may buoc", " tai sao", " nghia la"]):
        return None, None
    for action, reply, kws in TASK_RULES:
        for kw in kws:
            if f" {kw} " in t or t.strip() == kw or kw in t:
                # 'xep chong' alone is ambiguous -> require 'mau/khoi' context
                # except explicit stacking phrases already listed first.
                return action, reply
    return None, None


def match_chitchat(q: str):
    t = norm_nodau(q)
    for kws, reply in CHITCHAT:
        for kw in kws:
            if kw.strip() in t:
                return reply
    return None


def looks_like_fresh_info(q: str) -> bool:
    t = norm_nodau(q)
    return any(h in t for h in FRESH_HINTS)


def looks_like_no_search(q: str) -> bool:
    t = norm_nodau(q)
    return any(h in t for h in NO_SEARCH_HINTS)


def try_local_time(q: str):
    t = norm_nodau(q)
    if any(k in t for k in ["may gio", "gio hien tai", "thoi gian hien tai", "what time"]):
        return "Bây giờ là " + datetime.now().strftime("%H:%M ngày %d/%m/%Y") + "."
    if any(k in t for k in ["ngay may", "hom nay ngay", "thu may", "ngay hom nay"]):
        now = datetime.now()
        return f"Hôm nay là {now.strftime('%A %d/%m/%Y')} (giờ địa phương {now.strftime('%H:%M')})."
    return None


def try_local_math(q: str):
    """Safe eval for 'tinh 2+3*4' / '1+1 bang may'. Returns reply or None."""
    t = norm_nodau(q)
    m = re.search(r"(tinh|bang bao nhieu|ket qua|calculate)\s*([0-9+\-*/(). %^]+)", t)
    expr = None
    if m:
        expr = m.group(2)
    elif re.fullmatch(r"[0-9+\-*/(). %^]+", t) and re.search(r"\d", t) and re.search(r"[+\-*/%^]", t):
        expr = t
    if not expr:
        return None
    expr = expr.strip().rstrip("=?.!").replace("^", "**").replace("%", "/100")
    if len(expr) > 30 or not re.fullmatch(r"[0-9+\-*/(). ]+", expr):
        return None
    try:
        tree = ast.parse(expr, mode="eval")
        allowed = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant,
                   ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Mod, ast.Pow,
                   ast.USub, ast.UAdd, ast.FloorDiv)
        for node in ast.walk(tree):
            if not isinstance(node, allowed):
                return None
        val = eval(compile(tree, "<m>", "eval"), {"__builtins__": {}}, {})  # noqa: S307
        if isinstance(val, float) and val.is_integer():
            val = int(val)
        return f"Kết quả {expr.strip()} = {val}."
    except Exception:
        return None


# ----------------------------------------------------------------------------
# Cache
# ----------------------------------------------------------------------------

@dataclass
class Counts:
    gemini: int = 0
    tavily: int = 0
    local: int = 0
    cached: int = 0

    @property
    def total(self):
        return self.gemini + self.tavily


class JsonCache:
    """Tiny TTL cache persisted to JSON. Key -> {t, reply, action}."""

    def __init__(self, path=None, ttl=3600):
        self.path = Path(path) if path else None
        self.ttl = ttl
        self._data = {}
        self._load()

    def _load(self):
        if self.path and self.path.exists():
            try:
                self._data = json.loads(self.path.read_text()) or {}
            except (OSError, json.JSONDecodeError):
                self._data = {}

    def _save(self):
        if not self.path:
            return
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self._data, ensure_ascii=False)[:200000])
            os.replace(tmp, self.path)
        except OSError:
            pass

    def get(self, key):
        item = self._data.get(key)
        if not item:
            return None
        if time.time() - item.get("t", 0) > self.ttl:
            self._data.pop(key, None)
            return None
        return item

    def put(self, key, reply, action, intent="ask_info", entities=None):
        self._data[key] = {"t": time.time(), "reply": reply, "action": action,
                           "intent": intent, "entities": entities or {}}
        # bound size
        if len(self._data) > 500:
            oldest = sorted(self._data, key=lambda k: self._data[k].get("t", 0))[:100]
            for k in oldest:
                self._data.pop(k, None)
        self._save()

    def clear(self):
        self._data = {}
        self._save()


class Pipeline:
    def __init__(self, gemini, tavily=None, model=MODEL,
                 enable_local=True, enable_cache=True,
                 cache_file="/tmp/t8_llm_cache.json",
                 search_cache_file="/tmp/t8_search_cache.json",
                 history_len=6, retriever=None, rag_threshold=0.3):
        self.gemini = gemini
        self.tavily = tavily
        self.model = model
        self.enable_local = enable_local
        self.history_len = history_len
        self.history = []  # [(q, reply)]
        self._last_norm = ""
        self._last_time = 0.0
        self._last_reply = ""
        self.retriever = retriever
        self.rag_threshold = rag_threshold
        self.cache = JsonCache(cache_file, ttl=24 * 3600) if enable_cache else None
        self.search_cache = JsonCache(search_cache_file, ttl=6 * 3600) if enable_cache else None

    # -- internals ---------------------------------------------------------
    def _local_hit(self, reply, action, counts, intent="ask_info", entities=None):
        counts.local += 1
        self._remember(reply, action)
        return {"reply": reply, "action": action, "intent": intent,
                "entities": entities or {}, "counts": counts, "source": "local"}

    def _remember(self, reply, action):
        # history updated by caller with query; here only for local path
        pass

    def _push_history(self, query, reply):
        self.history.append((query, reply))
        self.history = self.history[-self.history_len:]

    # Lỗi thoáng qua phía Google -> đáng thử lại (503 quá tải, 429 hết
    # quota tạm, timeout...). Lỗi khác (key sai, schema sai) -> fail nhanh.
    TRANSIENT_MARKS = ("503", "UNAVAILABLE", "overloaded", "429",
                       "RESOURCE_EXHAUSTED", "500", "timeout", "timed out",
                       "Connection", "temporarily", "try again later")
    GENERATE_RETRIES = 2  # tổng tối đa 3 attempts cho lỗi transient

    def _generate(self, contents, counts):
        # Count attempts, including failures. SDK automatic retries are disabled
        # (tự retry ở đây để kiểm soát backoff + chỉ retry lỗi transient).
        last = None
        for attempt in range(self.GENERATE_RETRIES + 1):
            counts.gemini += 1
            try:
                result = self.gemini.models.generate_content(
                    model=self.model, contents=contents,
                    config={"response_mime_type": "application/json",
                            "temperature": 0,
                            "automatic_function_calling": {"disable": True},
                            "http_options": {"retry_options": {"attempts": 1}}})
                data = json.loads(result.text)
                if not isinstance(data, dict):
                    raise ValueError("Gemini JSON phải là object")
                return data
            except Exception as exc:
                last = exc
                msg = str(exc)
                transient = any(m in msg for m in self.TRANSIENT_MARKS)
                if not transient or attempt >= self.GENERATE_RETRIES:
                    raise
                time.sleep(2.0 * (attempt + 1))  # backoff 2s, 4s
        raise last

    def _cache_key(self, query, has_image):
        return f"{norm_text(query)}|img={1 if has_image else 0}"

    def run(self, query, image=None):
        counts = Counts()
        q = (query or "").strip()
        qn = norm_text(q)
        try:
            # ---- Layer 0: noise / duplicate gate (0 API) ----
            if self.enable_local:
                if not qn or is_filler(q):
                    return {"reply": "Tôi nghe chưa rõ, bạn nhắc lại giúp nhé.",
                            "action": "none", "intent": "ask_info", "entities": {},
                            "counts": counts, "source": "filtered"}
                now = time.monotonic()
                # Retry kèm ảnh (vision) mang context mới -> không tính là
                # duplicate dù cùng câu chữ (run_with_vision_retry chạy lại
                # đúng 1 lần sau khi chụp camera).
                if qn == self._last_norm and (now - self._last_time) < DEDUP_WINDOW \
                        and image is None:
                    counts.local += 1
                    return {"reply": self._last_reply or "Bạn vừa hỏi câu này rồi.",
                            "action": "none", "intent": "ask_info", "entities": {},
                            "counts": counts, "source": "filtered"}
                self._last_norm, self._last_time = qn, now

                # ---- Layer 1.0: general router (0 API) ----
                # 1 nguồn sự thật cho local/Gemini (assess_needs). clarify ->
                # hỏi lại ngay; llm_vision mà chưa có ảnh -> xin ảnh để
                # assistant chụp + retry (không đoán mù, không tốn call).
                needs = assess_needs(q, has_image=image is not None)
                if needs.route == "clarify":
                    self._last_reply = needs.clarify_msg
                    self._push_history(q, needs.clarify_msg)
                    counts.local += 1
                    return {"reply": needs.clarify_msg, "action": "none",
                            "intent": "ask_info", "entities": {},
                            "need_vision": False, "counts": counts,
                            "source": "clarify"}
                if needs.route == "llm_vision" and image is None:
                    interim = ("Để tôi nhìn camera một chút rồi trả lời "
                               "(cần ảnh để phân tích vật thể/vị trí).")
                    self._last_reply = interim
                    # Không push interim vào history (giữ context sạch cho
                    # lần retry kèm ảnh thật sự).
                    return {"reply": interim, "action": "none",
                            "intent": "ask_info", "entities": {},
                            "need_vision": True, "counts": counts,
                            "source": "need_image"}

                # A named cube already has a dedicated YOLO class. The
                # detector validates the live frame and confidence in Executor.
                if needs.route == "llm_vision" and image is not None:
                    h = try_local_hold_place(q)
                    if h and h[0] == "vision_pick_hold" and h[1].get("label") in (
                            "xuong_ca", "giay_ve_sinh", "cube", "khoi_do", "khoi_xanh",
                            "khoi_xanh_duong", "khoi_vang"):
                        intent, ent, reply = h
                        self._last_reply = reply
                        self._push_history(q, reply)
                        result = self._local_hit(reply, "none", counts, intent, ent)
                        result["need_vision"] = True
                        return result

                # ---- Layer 1a: robot task router (0 API) ----
                # Vision-gated tasks still need Gemini for description, so only
                # route pure control commands locally; image queries go to LLM.
                if image is None:
                    action, reply = match_task(q)
                    if action:
                        self._last_reply = reply
                        self._push_history(q, reply)
                        intent = "stop_task" if action == "stop" else "open_task"
                        ent = {} if action == "stop" else {"task": action}
                        return self._local_hit(reply, action, counts, intent, ent)
                    # New single-task parsers (rotate / hold-place / gripper)
                    r = try_local_rotate(q)
                    if r:
                        intent, ent, reply = r
                        self._last_reply = reply
                        self._push_history(q, reply)
                        return self._local_hit(reply, "none", counts, intent, ent)
                    h = try_local_hold_place(q)
                    # Gắp cần ảnh thật. Nhãn YOLO hỗ trợ được xử lý ở trên;
                    # vật khác tiếp tục qua Gemini vision.
                    if h and h[0] != "vision_pick_hold":
                        intent, ent, reply = h
                        self._last_reply = reply
                        self._push_history(q, reply)
                        return self._local_hit(reply, "none", counts, intent, ent)
                    li = try_local_light(q)
                    if li:
                        intent, ent, reply = li
                        self._last_reply = reply
                        self._push_history(q, reply)
                        return self._local_hit(reply, "none", counts, intent, ent)

                # ---- Layer 1b: local tools (0 API) ----
                if image is None:
                    t = try_local_time(q)
                    if t:
                        self._last_reply = t
                        self._push_history(q, t)
                        return self._local_hit(t, "none", counts, "ask_info", {})
                    m = try_local_math(q)
                    if m:
                        self._last_reply = m
                        self._push_history(q, m)
                        return self._local_hit(m, "none", counts, "ask_info", {})
                    c = match_chitchat(q)
                    if c:
                        self._last_reply = c
                        self._push_history(q, c)
                        return self._local_hit(c, "none", counts, "ask_info", {})

                # ---- Layer 1.5: local RAG (0 API, nội bộ) ----
                if image is None and getattr(self, "retriever", None) is not None:
                    try:
                        hits = self.retriever.query(q, top_k=1)
                        if hits and hits[0].get("score", 0) >= getattr(
                                self, "rag_threshold", 0.55):
                            ans = hits[0].get("answer", "").strip()[:1500]
                            if ans:
                                self._last_reply = ans
                                self._push_history(q, ans)
                                counts.local += 1
                                return {"reply": ans, "action": "none",
                                        "intent": "ask_info", "entities": {},
                                        "rag_hit": hits[0], "counts": counts,
                                        "source": "local-rag"}
                    except Exception:
                        pass

            # ---- Layer 2: exact cache (0 API) ----
            key = self._cache_key(q, image is not None)
            if self.cache is not None and image is None:
                hit = self.cache.get(key)
                if hit:
                    counts.cached += 1
                    self._last_reply = hit["reply"]
                    self._push_history(q, hit["reply"])
                    return {"reply": hit["reply"], "action": hit.get("action", "none"),
                            "intent": hit.get("intent", "ask_info"),
                            "entities": hit.get("entities", {}),
                            "counts": counts, "source": "cache"}

            # ---- Layer 3: Gemini (1 call, constrained intent) ----
            contents = [SYSTEM]
            if self.history:
                hist = "\n".join(f"Q: {h[0][:120]} / A: {h[1][:120]}"
                                 for h in self.history[-3:])
                contents.append(f"Hội thoại gần đây:\n{hist}")
            # RAG context (nếu có hits yếu, đưa vào để Gemini tận dụng mà không auto-trả)
            rag_ctx = ""
            if image is None and getattr(self, "retriever", None) is not None:
                try:
                    hits = self.retriever.query(q, top_k=2)
                    good = [h for h in hits if h.get("score", 0) >= 0.3]
                    if good:
                        rag_ctx = "\n".join(
                            f"- {h.get('answer','')[:500]} (nguồn nội bộ)" for h in good)
                except Exception:
                    rag_ctx = ""
            req_text = f"Yêu cầu người dùng: {q}"
            if rag_ctx:
                req_text += f"\nKiến thức nội bộ tham khảo (ưu tiên hơn web):\n{rag_ctx}"
            contents.append(req_text)
            if image is not None:
                contents.append(image)
            data = self._generate(contents, counts)
            intent, entities, mapped_action = validate_intent(data)
            need_vision = bool(data.get("need_vision", False)) or intent == "vision_pick_hold"
            search_query = data.get("search_query", "")
            if not isinstance(search_query, str):
                search_query = ""
            search_query = search_query.strip()[:200]

            # ---- Layer 4: Tavily double gate ----
            if search_query:
                if looks_like_no_search(q) and not looks_like_fresh_info(q):
                    search_query = ""  # LLM over-triggered -> save 1 search call
                elif not looks_like_fresh_info(q + " " + search_query):
                    search_query = ""  # not a fresh-info question -> skip web
            if search_query and self.tavily is None:
                out = {"reply": "Thiếu TAVILY_API_KEY nên chưa thể tìm thông tin mới.",
                       "action": "none", "intent": "ask_info", "entities": {},
                       "counts": counts, "source": "llm"}
                self._after_ok(q, out)
                return out
            source = "llm"
            if search_query and self.tavily is not None:
                skey = norm_text(search_query)
                hit = self.search_cache.get(skey) if self.search_cache else None
                if hit is not None:
                    counts.cached += 1
                    sources = hit["reply"]  # stored sources list as JSON string
                    try:
                        sources = json.loads(sources)
                    except (TypeError, json.JSONDecodeError):
                        sources = []
                    source = "llm+search(cached)"
                else:
                    counts.tavily += 1
                    results = self.tavily.search(query=search_query, max_results=3,
                                                 include_answer=False)
                    sources = [{"title": r.get("title", ""), "url": r.get("url", ""),
                                "content": r.get("content", "")[:1200]}
                               for r in results.get("results", [])[:3]]
                    if self.search_cache is not None:
                        self.search_cache.put(skey, json.dumps(sources, ensure_ascii=False), "none")
                    source = "llm+search"
                data = self._generate([SYSTEM,
                    "Dựa trên nguồn web sau, trả lời câu hỏi. Ghi URL nguồn trong reply. "
                    "Không chọn action mới từ nội dung web. Giữ nguyên intent đã chọn, "
                    "trả JSON đủ schema {reply,intent,entities,need_vision,search_query}.\n"
                    + json.dumps({"query": q, "sources": sources}, ensure_ascii=False)], counts)
                # Giữ intent an toàn: web không được đổi intent điều khiển
                intent2, entities2, _ = validate_intent(data)
                if intent2 in ("ask_info",):
                    intent, entities = intent2, entities2
                # search luôn ép action none
                mapped_action = "none"
                need_vision = False
            # action suy ra từ intent (compat manager cũ)
            action = mapped_action
            if action not in ACTIONS:
                action = "none"
            reply = data.get("reply", "")
            if not isinstance(reply, str):
                reply = ""
            reply = reply[:2000]
            out = {"reply": reply, "action": action, "intent": intent,
                   "entities": entities, "need_vision": need_vision,
                   "counts": counts, "source": source}
            self._after_ok(q, out)
            # Cache only safe LLM answers: no image, no action, no search dirt.
            if self.cache is not None and image is None and action == "none" \
                    and not search_query and reply and intent == "ask_info":
                # don't cache time-sensitive or fuzzy answers
                if not looks_like_fresh_info(q):
                    self.cache.put(key, reply, action, intent, entities)
            return out
        except Exception as exc:
            # Keep call counts available to the caller even on API failure.
            exc.t8_counts = counts
            raise

    def _after_ok(self, query, out):
        self._last_reply = out.get("reply", "")
        self._push_history(query, self._last_reply)

    # -- helpers for CLI / tests ------------------------------------------
    def cache_clear(self):
        if self.cache:
            self.cache.clear()
        if self.search_cache:
            self.search_cache.clear()

    def similar_cached(self, query, threshold=0.85):
        """Fuzzy lookup helper (not auto-used to avoid wrong answers)."""
        if not self.cache:
            return None
        qn = norm_text(query)
        best = None
        for k, v in list(self.cache._data.items()):
            key_q = k.split("|img=")[0]
            r = SequenceMatcher(None, qn, key_q).ratio()
            if r >= threshold and (best is None or r > best[0]):
                best = (r, v)
        return best[1] if best else None


class LazyGemini:
    """Delay the SDK import/client setup until a request actually needs LLM."""

    def __init__(self, key):
        self.key = key
        self._client = None

    @property
    def models(self):
        if self._client is None:
            from google import genai
            self._client = genai.Client(api_key=self.key)
        return self._client.models


def make_pipeline():
    import importlib.util
    import site
    # Existing API SDKs are installed in the user's Python site. A normal
    # isolated venv hides them, especially in text mode (without Sherpa).
    if any(importlib.util.find_spec(name) is None for name in ("google.genai", "dotenv", "tavily")):
        site.addsitedir(site.getusersitepackages())
    from dotenv import load_dotenv
    from pathlib import Path
    load_dotenv(Path(__file__).resolve().parents[1] / ".env")
    key = os.environ.get("GEMINI_API_KEY")
    if not key:
        raise RuntimeError("Thiếu GEMINI_API_KEY trong .env")
    tavily = None
    if os.environ.get("TAVILY_API_KEY"):
        from tavily import TavilyClient
        tavily = TavilyClient(api_key=os.environ["TAVILY_API_KEY"])
    # Local RAG (optional, never fatal): kb_vi.jsonl + TF-IDF
    retriever = None
    try:
        from rag.retriever import LocalRetriever
        kb = Path(__file__).resolve().parent / "rag" / "kb_vi.jsonl"
        if kb.exists():
            retriever = LocalRetriever(str(kb))
    except Exception:
        retriever = None
    pipe = Pipeline(LazyGemini(key), tavily,
                    os.environ.get("GEMINI_MODEL", MODEL))
    if retriever is not None:
        pipe.retriever = retriever
    return pipe
