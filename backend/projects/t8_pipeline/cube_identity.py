"""Single source of truth for cube ID <-> color (source code regulation).

ID1=xanh dương, ID2=xanh lá, ID3=đỏ, ID4=vàng.
- Tag-verified (AprilTag tag36h11): identity chắc nhất, ưu tiên.
- Color-inferred (HSV 2-frame, đúng 1 ứng viên) chỉ dùng để mô tả/hỗ trợ
  selector. Nó không đủ bằng chứng để thực hiện hành động theo cube ID.
"""

ID_TO_COLOR = {1: "khoi_xanh_duong", 2: "khoi_xanh", 3: "khoi_do", 4: "khoi_vang"}
COLOR_TO_ID = {"khoi_xanh_duong": 1, "blue": 1,
               "khoi_xanh": 2, "green": 2,
               "khoi_do": 3, "red": 3,
               "khoi_vang": 4, "yellow": 4}
ID_NAMES_VI = {1: "xanh dương", 2: "xanh lá", 3: "đỏ", 4: "vàng"}


# Mặt rác in trên cube -> ID cube (cùng bảng với cube_vision.registry.CUBES; có test đối chiếu).
# Alias viết không dấu (đ -> d). Khớp theo cụm dài nhất; hai lớp khác nhau cùng khớp = mơ hồ.
TRASH_FACE_CUBE_ID = {
    "newspaper": 1, "zip_top_can": 1, "book": 1, "old_school_bag": 1,
    "fish_bone": 2, "egg_shell": 2, "apple_core": 2, "watermelon_rind": 2,
    "syringe": 3, "expired_cosmetics": 3, "used_batteries": 3, "expired_tablets": 3,
    "toilet_paper": 4, "peach_pit": 4, "cigarette_butts": 4, "disposable_chopsticks": 4,
}
TRASH_FACE_ALIASES = {
    "newspaper": ("to bao", "bao giay", "bao cu", "newspaper", "bao"),
    "zip_top_can": ("lon nuoc", "lon bia", "lon nhom", "lon coca", "cai lon", "zip top can"),
    "book": ("quyen sach", "book", "sach"),
    "old_school_bag": ("cap sach cu", "cap sach", "cap di hoc", "balo", "ba lo", "school bag", "cap cu"),
    "fish_bone": ("xuong ca", "fish bone", "xuong"),
    "egg_shell": ("vo trung ga", "vo trung", "egg shell"),
    "apple_core": ("loi tao", "ruot tao", "cui tao", "apple core"),
    "watermelon_rind": ("vo dua hau", "dua hau", "watermelon"),
    "syringe": ("kim tiem", "ong tiem", "bom tiem", "xy lanh", "syringe"),
    "expired_cosmetics": ("my pham het han", "my pham", "son moi", "cosmetics", "cosmetic"),
    "used_batteries": ("pin da qua su dung", "pin da su dung", "pin da dung", "pin cu", "cuc pin",
                       "used batteries", "used battery", "batteries", "battery", "pin"),
    "expired_tablets": ("thuoc het han", "thuoc vien", "vien thuoc", "vi thuoc", "tablets", "tablet"),
    "toilet_paper": ("giay ve sinh", "cuon giay", "toilet paper"),
    "peach_pit": ("hat dao", "peach pit"),
    "cigarette_butts": ("dau loc thuoc la", "dau loc thuoc", "dau thuoc la", "tan thuoc la", "tan thuoc",
                        "cigarette butts", "cigarette"),
    "disposable_chopsticks": ("dua dung mot lan", "dua go", "dua tre", "dua an", "chopsticks", "chopstick"),
}


def _no_accents(text):
    import unicodedata as _u
    s = str(text or "").strip().lower().replace("đ", "d")
    s = "".join(c for c in _u.normalize("NFD", s) if _u.category(c) != "Mn")
    return " ".join(s.replace("_", " ").split())


def trash_face_class(text):
    """Lớp mặt rác duy nhất được nhắc trong câu ("cube hình cục pin đã qua sử dụng" -> used_batteries).

    None khi không nhắc lớp nào hoặc nhắc nhiều lớp khác nhau (mơ hồ: không đoán).
    """
    import re as _re
    t = " " + _no_accents(text) + " "
    spans = []
    for cls, aliases in TRASH_FACE_ALIASES.items():
        for alias in aliases:
            for m in _re.finditer(r"(?<![a-z0-9])" + _re.escape(alias) + r"(?![a-z0-9])", t):
                spans.append((m.start(), m.end(), cls))
    # Cụm dài hơn che cụm ngắn nằm trong nó ("cap sach" che "sach", "pin da qua su dung" che "pin").
    spans.sort(key=lambda sp: (-(sp[1] - sp[0]), sp[0]))
    taken, classes = [], set()
    for start, end, cls in spans:
        if any(start < e and end > b for b, e in taken):
            continue
        taken.append((start, end))
        classes.add(cls)
    return next(iter(classes)) if len(classes) == 1 else None


# Luồng ROS 3D gọi cube bằng hình rác ("cube xương cá" = cube ID 2). Luồng 2D cũ dùng cùng cụm từ
# cho lớp YOLO ("xuong_ca"), nên chỉ bật ánh xạ này khi backend là ros3d (t8_assistant / RosTaskRunner).
ROS3D_FACE_LABELS = False


def set_ros3d_face_labels(enabled=True):
    global ROS3D_FACE_LABELS
    ROS3D_FACE_LABELS = bool(enabled)


def trash_face_cube_id(text, need_cube_noun=False):
    """ID cube theo hình rác trong câu. need_cube_noun=True: chỉ khi câu gọi rõ "cube/khối/block"
    (luồng cũ dùng nhãn YOLO như "xuong_ca" cho vật không phải cube, không được đổi nghĩa)."""
    import re as _re
    if need_cube_noun and not ROS3D_FACE_LABELS:
        return None
    if need_cube_noun and not _re.search(r"(?<![a-z0-9])(?:cube|khoi|block)(?![a-z0-9])", _no_accents(text)):
        return None
    cls = trash_face_class(text)
    return TRASH_FACE_CUBE_ID.get(cls) if cls else None


def color_for_id(cube_id):
    try:
        return ID_TO_COLOR.get(int(cube_id))
    except (TypeError, ValueError):
        return None


def id_for_color(label):
    return COLOR_TO_ID.get(str(label or "").strip().lower())


def identity_context():
    return {"ID1": "xanh dương (khoi_xanh_duong)", "ID2": "xanh lá (khoi_xanh)",
            "ID3": "đỏ (khoi_do)", "ID4": "vàng (khoi_vang)",
            "trash_faces": {"ID1": "báo, lon nước, sách, cặp sách cũ",
                            "ID2": "xương cá, vỏ trứng, lõi táo, vỏ dưa hấu",
                            "ID3": "kim tiêm, mỹ phẩm hết hạn, pin đã qua sử dụng, thuốc hết hạn",
                            "ID4": "giấy vệ sinh, hạt đào, tàn thuốc lá, đũa dùng một lần"},
            "rule_answer_only": "câu hỏi thì color-inferred chỉ trả lời",
            "rule_action": "gắp/đặt/xếp theo ID cần object_id và geometry_model_id "
                           "được perception 3D xác nhận; không fallback từ màu"}


def resolve_for_action(label):
    """Trả (kind, mapped_label): kind=tag/color/unknown để executor fallback."""
    s = str(label or "").strip().lower()
    if s.startswith("cube_"):
        try:
            want = int(s.rsplit("_", 1)[1])
        except (ValueError, IndexError):
            return ("unknown", label)
        color = color_for_id(want)
        return ("tag_or_color", color)
    if s in COLOR_TO_ID:
        return ("color", s)
    return ("unknown", label)


def object_ids_for_label(label):
    """Tap object_id perception Terminal 1 (1=blue..4=yellow) cho 1 nhan T8.

    Tra set rong khi nhan chung chung ("cube") hoac la: nhung nhan nay khong
    khoa duoc ID roi rac, de luong legacy tu quyet (contour/YOLO).
    """
    import re as _re
    s = str(label or "").strip().lower()
    if s.startswith("cube_"):
        try:
            want = int(s.rsplit("_", 1)[1])
        except (ValueError, IndexError):
            return set()
        return {want} if want in ID_TO_COLOR else set()
    core = [_p for _p in _re.split(r"[\s_\-]+", s)
            if _p not in ("cube", "id", "khoi", "block", "mau", "color")]
    if len(core) == 1 and core[0] in ("1", "2", "3", "4"):
        return {int(core[0])}
    canon = canonical_label(label)
    if canon in COLOR_TO_ID:
        return {int(COLOR_TO_ID[canon])}
    return set()


def select_ros3d_match(objects, wanted_ids, exclude=None):
    """Chon 1 object perception da confirmed khop tap ID muon.

    Moi object la dict snapshot (object_id/confirmed/bbox_xyxy/center_px...).
    Tra None khi: khong co, chua confirmed/bbox hong, hoac mo ho (>=2 khop).
    """
    hits = []
    for obj in objects or []:
        if not isinstance(obj, dict) or not obj.get("confirmed"):
            continue
        try:
            oid = int(obj.get("object_id", 0))
        except (TypeError, ValueError):
            continue
        box = obj.get("bbox_xyxy") or [0, 0, 0, 0]
        try:
            ok_box = (len(box) == 4 and box[2] - box[0] >= 10
                      and box[3] - box[1] >= 10
                      and 0 <= box[0] < box[2] <= 640
                      and 0 <= box[1] < box[3] <= 480)
        except TypeError:
            ok_box = False
        if oid not in wanted_ids or not ok_box:
            continue
        center = obj.get("center_px") or [0.0, 0.0]
        if exclude is not None:
            ex, ey = exclude["center"]
            if abs(center[0] - ex) <= 40 and abs(center[1] - ey) <= 40:
                continue
        hits.append(obj)
    if len(hits) != 1:
        return None
    return hits[0]


# Gemini hay trả nhãn tiếng Anh thô ("blue", "red cube"...); detector chỉ hiểu
# khoi_*. Chuẩn hóa ở biên validate/executor để khỏi tìm mãi không thấy.
def canonical_label(label):
    """Map nhãn màu EN/VI rời rạc về khoi_*; giữ nguyên cube_ID/label lạ."""
    import re as _re
    s = str(label or "").strip().lower()
    if not s:
        return label
    if s.startswith("cube_"):
        digits = _re.sub(r"\D", "", s[5:])
        if digits in ("1", "2", "3", "4"):
            return f"cube_{digits}"
        return s
    face_id = trash_face_cube_id(s, need_cube_noun=True)
    if face_id is not None:                # "cube hình cục pin đã qua sử dụng" -> cube_3
        return f"cube_{face_id}"
    us = _re.sub(r"[\s\-]+", "_", s)
    tokens = set(us.split("_"))
    if "xanh_duong" in us or "blue" in tokens:
        return "khoi_xanh_duong"
    if "xanh_la" in us or "green" in tokens or "khoi_xanh" in tokens:
        return "khoi_xanh"
    if "khoi_do" in tokens or "red" in tokens:
        return "khoi_do"
    if "khoi_vang" in tokens or "yellow" in tokens:
        return "khoi_vang"
    # "do"/"vang"/"xanh" rời ("cube do", "cube xanh"): mirror VISION_LABELS —
    # xanh trơn về xanh lá (câu hỏi "xanh là lá hay dương" xử lý ở query).
    if "do" in tokens and ("cube" in tokens or "khoi" in tokens):
        return "khoi_do"
    if "vang" in tokens and ("cube" in tokens or "khoi" in tokens):
        return "khoi_vang"
    if "xanh" in tokens and ("cube" in tokens or "khoi" in tokens or "mau" in tokens):
        return "khoi_xanh"
    # Cube họa tiết lạ (syringe...) không có class riêng: về nhãn cube chung để
    # tìm bằng contour 2-frame thay vì treo vì nhãn lạ.
    if "cube" in tokens or "khoi" in tokens or "block" in tokens:
        return "cube"
    return label
