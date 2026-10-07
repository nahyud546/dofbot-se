#!/usr/bin/env python3
"""Luồng search-center-pick-place độc lập (zero-impact luồng cũ).

- File mới hoàn toàn, không dùng chung module cũ (worker/stage1/executor).
- State riêng /tmp/t8_search_center_state.json, không đọc/ghi hold_state cũ.
- Toán exact từ cube_search_center_math (FK URDF + ray-plane any-pose).
- Detector: tag36h11 ưu tiên, fallback HSV/contour 2-frame confirm.
- Quét chỉ J1: nhanh khi chưa thấy, chậm-chặt khi thấy tới center rồi mới gắp.
- Gắp tại pose dừng (live q), đặt vào zone ID 1-4 (giữ map BIN_* cũ, copy trị số).

Chạy dry-run: python3 cube_search_center.py --dry-run --camera /dev/video2
"""
from __future__ import annotations

import argparse
import faulthandler
import json
import math
import sys
import time
from pathlib import Path

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
from cube_vision import registry as _registry  # noqa: E402
import cube_search_center_math as _math_mod  # noqa: E402  (FX/CX đổi sau apply_calibration)
from cube_vision import color as _color  # noqa: E402
from cube_vision.tag import TagDetector as _SharedTagDetector  # noqa: E402
from search_planner import (FAST as FAST_TUNING, TUNINGS as SEARCH_TUNINGS,  # noqa: E402
                            SearchPlanner, SearchTuning)
import cv2
import numpy as np

from cube_search_center_math import (
    IMAGE_SIZE,
    PICK_Z_FALLBACKS,
    DESCEND_ANCHOR_ZS,
    DESCEND_MAX_DEG,
    DESCEND_MAX_LATERAL,
    DESCEND_TARGET_Z,
    DESCEND_Z_TOL,
    check_j1_hold_limit,
    check_workspace,
    fine_angle,
    fk_optical,
    j1_delta_to_zone,
    pixel_to_base,
    quad_yaw,
    ray_plane,
    zone_j1,
)

STATE_FILE = Path("/tmp/t8_search_center_state.json")
# Giữ nguyên trị số luồng cũ để tương thích IK/gripper, nhưng copy (không import).
OPEN_ANGLE = 25
CLOSE_ANGLE = 140
OBSERVE_J245 = [125.0, 0.0, 0.0, 90.0]
SWEEP_J1_MIN = 20.0
SWEEP_J1_MAX = 160.0
FAST_J1_STEP = 10.0
FAST_STEP_DELAY = 0.6
# Chậm-chặt theo user chốt: kp nhỏ, step nhỏ, ngưỡng chặt, nhiều frame.
SLOW_KP = 0.015  # deg/px
SLOW_MAX_STEP = 2.0
SLOW_DELAY = 0.5
CENTER_THRESH_X = 10.0
CENTER_THRESH_Y = 20.0  # legacy strict, giữ cho is_centered/test cũ
# F1: chỉ J1 được servo (trục X). Y không có cơ cấu chỉnh nên là advisory
# rộng ±60px — toán ray-plane exact ở mọi pixel nên không cần center Y chặt.
CENTER_Y_ADVISORY = 60.0
# F2: candidate mới phải nằm gần vị trí bám cuối (chống nhảy cube<->thảm zone).
ASSOC_RADIUS = 100.0
ASSOC_MAX_MISS = 3
# F4: cube thật ở observe chỉ 40-130px; thảm zone vỡ mảnh 150-190px.
# Muốn trash/mặt lớn hơn thì dùng gate riêng ở P1, không nới ở đây.
CUBE_QUAD_MAX = 160.0
# Verify ở approach bằng mắt thật: nhìn gần nên nới size; residual >12mm
# thì trừ thẳng vào mục tiêu giải IK lại; >50mm thì abort (sai tầng khác).
# 320 vì ở điểm gắp thấp tag 30mm nở tới ~250px (case 2026-10-06: gate 220
# loại oan cube ngay trước mắt); thẻ trắng full-frame 640px vẫn bị loại.
VERIFY_MAX_SIDE = 320.0
VERIFY_TOL = 0.012
VERIFY_ABORT = 0.050
VERIFY_IMG = "/tmp/grasp_verify.png"
CENTER_FRAMES = 5
TRACK_TIMEOUT_S = 60.0
CONFIRM_TIMEOUT_S = 120.0
GRASP_WIN = "search_center (q de thoat)"

# Copy trị số zone khớp (không import t8_motion_worker để zero-impact).
ZONE_APPROACH = {
    1: [30.0, 70.0, 0.0, 54.0, 265.0],
    2: [150.0, 70.0, 7.0, 56.0, 265.0],
    3: [50.0, 70.0, 7.0, 58.0, 265.0],
    4: [135.0, 70.0, 7.0, 54.0, 265.0],
}
ZONE_RELEASE = {
    1: [30.0, 65.7098, 0.379519, 56.1977, 265.0],
    2: [150.0, 66.6198, 7.04465, 57.3374, 265.0],
    3: [50.0, 66.7004, 7.04872, 59.2308, 265.0],
    4: [135.0, 66.5425, 7.0367, 55.4436, 265.0],
}
ZONE_LIFT = {
    1: [30.0, 89.348, 5.94074, 40.3418, 265.0],
    2: [150.0, 85.7708, 12.7032, 47.1349, 265.0],
    3: [50.0, 85.3714, 12.4169, 49.9596, 265.0],
    4: [135.0, 86.139, 13.0342, 44.3005, 265.0],
}

# Bảng màu/ID dùng chung (cube_vision.registry): xanh dương=1, xanh lá=2, đỏ=3, vàng=4,
# cùng bảng với T8. Trước đây file này map vàng=1/xanh dương=4 nên cube không có tag
# bị gán sai ID (và sai zone).
HSV_RANGES = _registry.hsv_ranges("proven")
COLOR_TO_ID = dict(_registry.COLOR_TO_ID)
ID_TO_COLOR = dict(_registry.ID_TO_COLOR)
_VISION_DETECTOR = None
_VISION_FAILS = 0


def _proven_detector():
    """Load VisionDetector luồng cũ theo đường dẫn file (read-only)."""
    global _VISION_DETECTOR
    if _VISION_DETECTOR is not None:
        return _VISION_DETECTOR
    import importlib.util
    from pathlib import Path as _P
    path = _P(__file__).resolve().parents[1] / "t8_pipeline" / "t8_vision.py"
    spec = importlib.util.spec_from_file_location("t8_proven_vision", str(path))
    if spec is None or spec.loader is None:
        raise ImportError("không load được t8_vision")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _VISION_DETECTOR = mod.VisionDetector()
    return _VISION_DETECTOR


def proven_color_candidates(bgr1, bgr2, want_id: int | None = None):
    """Detector màu đã chứng minh (HSV trực tiếp + side-associated mặt trắng).

    Luồng cũ t8_vision._side_associated bắt cube hoa văn (mặt trên trắng +
    dải màu hông) mà HSV trực tiếp của module này bỏ sót — đó là lý do màu
    chạy kém ngoài thực tế dù test ô đỏ đặc vẫn pass.
    """
    global _VISION_FAILS
    if bgr1 is None or bgr2 is None:
        return []
    if bgr1.shape[:2] != (480, 640) or bgr2.shape[:2] != (480, 640):
        return []
    try:
        from PIL import Image
    except ImportError:
        return []
    try:
        vd = _proven_detector()
    except Exception:
        _VISION_FAILS += 1
        return []
    labels = ([ID_TO_COLOR[int(want_id)]] if want_id is not None and
              int(want_id) in ID_TO_COLOR else list(HSV_RANGES.keys()))
    out = []
    for color in labels:
        try:
            rgb1 = cv2.cvtColor(bgr1, cv2.COLOR_BGR2RGB)
            rgb2 = cv2.cvtColor(bgr2, cv2.COLOR_BGR2RGB)
            pil1 = Image.fromarray(rgb1)
            pil2 = Image.fromarray(rgb2)
            cands = vd.get_cube_candidates(color, pil1,
                                           confirm_frame=lambda: pil2)
        except Exception:
            _VISION_FAILS += 1
            continue
        for c in cands or []:
            try:
                corners = c.get("corners")
                center = c.get("center")
                if corners is not None and len(corners) == 4:
                    quad = [[float(p[0]), float(p[1])] for p in corners]
                    cx, cy = float(center[0]), float(center[1])
                else:
                    box = c.get("box")
                    x1, y1, x2, y2 = (float(v) for v in box)
                    quad = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            except (TypeError, ValueError, IndexError):
                continue
            cid = COLOR_TO_ID.get(color)
            if want_id is not None and cid != int(want_id):
                continue
            src = str(c.get("source", "proven"))
            out.append({"cube_id": cid, "center": [cx, cy], "quad": quad,
                        "source": f"proven_{src}"})
    return out
# Mặc định map màu->ID để giữ tương thích BIN_POSES khi tag vắng mặt.
# Nếu dự án có map khác (cube_identity), truyền cube_id trực tiếp qua API.


# ---------------------------------------------------------------- pure logic
def sweep_next_j1(current: float, direction: int) -> tuple[float, int]:
    """Bước quét nhanh ping-pong trong [SWEEP_J1_MIN, SWEEP_J1_MAX]."""
    nxt = float(current) + direction * FAST_J1_STEP
    if nxt >= SWEEP_J1_MAX:
        return SWEEP_J1_MAX, -1
    if nxt <= SWEEP_J1_MIN:
        return SWEEP_J1_MIN, 1
    return nxt, direction


def proportional_j1(current_j1: float, cx: float) -> float:
    """Bước chậm-chặt: J1 ngược chiều err_x (camera xoay 180°, sign đã ở FK).

    Camera effective Rz90: u tăng -> Y tăng. J1 tăng (CCW từ trên) -> Y giảm
    ở elbow READY phía -X, nên step = -kp*err để đưa tâm về 320.
    """
    err = float(cx) - 320.0
    step = -SLOW_KP * err
    step = max(-SLOW_MAX_STEP, min(SLOW_MAX_STEP, step))
    return float(current_j1) + step


def is_centered(cx: float, cy: float) -> bool:
    return abs(float(cx) - 320.0) <= CENTER_THRESH_X and \
        abs(float(cy) - 240.0) <= CENTER_THRESH_Y


def is_x_centered(cx: float) -> bool:
    """Trục duy nhất có servo (J1). LOCK dựa trên cái này."""
    return abs(float(cx) - 320.0) <= CENTER_THRESH_X


def is_y_ok(cy: float) -> bool:
    """Y advisory: trong ±60px là gắp được (toán exact mọi pixel)."""
    return abs(float(cy) - 240.0) <= CENTER_Y_ADVISORY


def assoc_ok(last, cx: float, cy: float,
             radius: float = ASSOC_RADIUS) -> bool:
    """Candidate mới phải gần vị trí bám cuối (F2). Chưa bám gì -> True."""
    if last is None:
        return True
    return max(abs(float(cx) - last[0]),
               abs(float(cy) - last[1])) <= radius


def confirm_center(history: list[bool]) -> bool:
    """Chặt: cần CENTER_FRAMES True liên tiếp."""
    if len(history) < CENTER_FRAMES:
        return False
    return all(history[-CENTER_FRAMES:])


def compute_pick(u: float, v: float, quad, servo15) -> dict:
    """Tính XY exact tại pose dừng + yaw. Raise nếu ngoài workspace/không nhìn bàn."""
    P, d = pixel_to_base(float(u), float(v), servo15)
    x, y = float(P[0]), float(P[1])
    check_workspace(x, y)
    yaw = quad_yaw(quad)
    return {"x": round(x, 5), "y": round(y, 5), "z": PICK_Z_FALLBACKS[0],
            "yaw": yaw, "d": round(float(d), 4)}


def zone_pose(table: dict, cube_id: int) -> list:
    """Pose của zone với J1 lấy từ zone_j1() (có thể đã chỉnh theo ô thật trong lần chạy này)."""
    pose = list(table[int(cube_id)])
    pose[0] = float(zone_j1(int(cube_id)))
    return pose


def run_zone_check(cap, arm, cube_ids, cal, log=print) -> dict:
    """Nhìn từng zone cần dùng, đo biên thả so với ô thật và chỉnh J1 zone cho lần chạy này.

    Chỉ ghi vào ZONE_J1_RUNTIME khi zone_locator kết luận 'shifted'; mọi trường hợp
    khác (đủ biên, không thấy ô, không chắc) giữ góc cấu hình.
    """
    import zone_locator as ZL
    from cube_search_center_math import ZONE_J1_RUNTIME
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
    import dofbot_ik
    table_z = float(cal["tag_top_z"]) - 0.030

    def move(j1):
        arm.write6_array(observe_pose(float(j1)), 1300)
        time.sleep(1.6)
        for _ in range(3):
            cap.grab()
        return [float(v) for v in arm.read_joints()[:5]]

    def grab():
        for _ in range(4):
            cap.grab()
        ok, frame = cap.read()
        if not ok or frame is None:
            return None
        return frame if frame.shape[:2] == (480, 640) else cv2.resize(frame, (640, 480))

    results = {}
    for cid in cube_ids:
        release_xy = dofbot_ik.fk(ZONE_RELEASE[int(cid)][:5])[:2]
        res = ZL.survey_zone(int(cid), float(zone_j1(int(cid))), release_xy, move, grab,
                             cal, table_z, log=log)
        if res.action == "shifted":
            ZONE_J1_RUNTIME[int(cid)] = res.use_j1
        results[int(cid)] = res
    arm.write6_array(observe_pose(90.0), 1200)
    time.sleep(1.4)
    return results


def compute_place_delta(j1_pick: float, cube_id: int,
                        allow_staged: bool = False) -> dict:
    """Góc xoay exact từ J1 gắp tới J1 zone.

    Mặc định abort nếu >45° (an toàn luồng cũ). allow_staged=True thì trả
    thêm steps: chia delta thành các bước ≤40° để xoay nhiều nhịp ở độ cao
    nâng an toàn (J2=120).
    """
    from cube_search_center_math import J1_HOLD_LIMIT
    delta = j1_delta_to_zone(float(j1_pick), int(cube_id))
    if abs(float(delta)) <= J1_HOLD_LIMIT:
        return {"j1_zone": zone_j1(int(cube_id)), "delta": round(float(delta), 2),
                "staged": False, "steps": []}
    if not allow_staged:
        check_j1_hold_limit(delta)  # raise với message abort chuẩn
    steps: list[float] = []
    remaining = float(delta)
    cur = float(j1_pick)
    while abs(remaining) > 1e-9:
        step = max(-40.0, min(40.0, remaining))
        cur += step
        # wrap J1 về [0,180] servo thực tế.
        cur = max(0.0, min(180.0, cur))
        steps.append(round(cur, 1))
        remaining = j1_delta_to_zone(cur, int(cube_id))
        if len(steps) > 4:
            break
    return {"j1_zone": zone_j1(int(cube_id)), "delta": round(float(delta), 2),
            "staged": True, "steps": steps}


def split_j1_steps(j1_from: float, j1_to: float, max_step: float = 40.0) -> list[float]:
    """Chia hành trình J1 thành các mốc trung gian, mỗi bước ≤max_step."""
    from cube_search_center_math import wrap180
    delta = wrap180(float(j1_to) - float(j1_from))
    if abs(delta) < 1e-9:
        return []
    n = max(1, int(math.ceil(abs(delta) / float(max_step))))
    return [round(float(j1_from) + delta * (i + 1) / n, 1) for i in range(n)]


# ---------------------------------------------------------------- detectors
class TagDetector:
    """Tag tag36h11 cho ID 1-4, trả dict kiểu search-center (dùng cube_vision.tag)."""

    def __init__(self):
        self._det = _SharedTagDetector(ids=_registry.CUBES, nthreads=2, lazy=True,
                                       quiet=True, backends=("pupil_apriltags", "dt_apriltags"))

    def detect(self, bgr):
        """Trả [{'cube_id', 'center', 'quad', 'source'}], chỉ ID 1-4."""
        if bgr is None or bgr.shape[:2] != (480, 640):
            return []
        return [{"cube_id": t["cube_id"], "center": list(t["centroid"]),
                 "quad": t["quad"], "source": "apriltag"}
                for t in self._det.detect(bgr)]


def hsv_candidates(bgr, want_id: int | None = None):
    """Fallback HSV 1-frame (cube_vision.color) với bảng màu->ID dùng chung."""
    return _color.square_candidates(bgr, HSV_RANGES, COLOR_TO_ID, want_id)


def _quad_size(quad):
    """Kích thước bbox của quad. Trả (w, h) hoặc (0, 0) nếu quad hỏng."""
    try:
        pts = np.asarray(quad, float).reshape(-1, 2)
    except (TypeError, ValueError):
        return 0.0, 0.0
    if pts.shape != (4, 2) or not np.isfinite(pts).all():
        return 0.0, 0.0
    return (float(pts[:, 0].max() - pts[:, 0].min()),
            float(pts[:, 1].max() - pts[:, 1].min()))


def is_plausible_cube(quad, center, max_side: float = CUBE_QUAD_MAX) -> bool:
    """Gate chống false-positive trên nền lớn (thẻ trắng full-frame).

    Cube thật ở pose observe chỉ ~60-130px. Nền trắng/thảm zone vỡ mảnh
    150-190px nên max_side=160 loại ngay mà vẫn dư biên cho cube dí gần.
    """
    w, h = _quad_size(quad)
    if w < 25 or h < 25:
        return False
    if max(w, h) > max_side or w * h > 90000:
        return False
    try:
        pts = np.asarray(quad, float).reshape(4, 2)
        cx, cy = float(center[0]), float(center[1])
    except (TypeError, ValueError, IndexError):
        return False
    if pts[:, 0].min() <= 5 or pts[:, 1].min() <= 5:
        return False
    if pts[:, 0].max() >= 635 or pts[:, 1].max() >= 475:
        return False
    if not (20 <= cx <= 620 and 20 <= cy <= 460):
        return False
    return True


def detect_once(bgr, want_id: int | None = None, tag: TagDetector | None = None):
    """Tag ưu tiên, fallback HSV. Trả tối đa 1 ứng viên (hoặc [])."""
    tag = tag or TagDetector()
    tags = tag.detect(bgr)
    if want_id is not None:
        tags = [t for t in tags if t["cube_id"] == int(want_id)]
    if len(tags) == 1:
        return tags
    if tags:
        return []
    cands = hsv_candidates(bgr, want_id)
    if len(cands) == 1:
        cands[0]["source"] = "hsv_single"
        return cands
    return []


def detect_confirmed(frame_getter, want_id: int | None = None,
                       tag: TagDetector | None = None,
                       max_side: float = CUBE_QUAD_MAX,
                       closeup: bool = False):
    """Xác nhận 2 frame liên tiếp. Thứ tự: tag → proven → HSV tối giản.

    Proven (side-associated mặt trắng + width-drift ≤12px của luồng cũ) đứng
    trước HSV nội bộ vì HSV nội bộ quá lỏng với nền trắng lớn (xem ảnh abort:
    thẻ trắng full-frame từng bắn hsv_single loạn xạ). Mọi ứng viên đều qua
    gate is_plausible_cube trước khi trả về. max_side nới khi nhìn gần
    (verify ở approach) vì cube to hơn trong frame.
    closeup=True (chỉ verify ở gần): rung tay 1-2mm khiến tag 200px+ nhảy
    >12px giữa 2 frame tươi, nên gate drift/width co giãn theo size
    (10%, sàn 12px); rớt pair thì chấp nhận single-frame cùng ID (tay đã
    settled, tag khó false-positive, residual gate + workspace chặn sau).
    Luồng search giữ closeup=False nên các case F1-F4 không đổi.
    """
    tag = tag or TagDetector()
    try:
        f1 = frame_getter()
        f2 = frame_getter()
    except Exception:
        return []
    if f1 is None or f2 is None:
        return []
    # 1. Tag cả 2 frame, cùng ID, drift ≤12px.
    a = detect_once(f1, want_id, tag)
    b = detect_once(f2, want_id, tag)
    ta = [c for c in a if c.get("source") == "apriltag"]
    tb = [c for c in b if c.get("source") == "apriltag"]
    if len(ta) == 1 and len(tb) == 1 and ta[0]["cube_id"] == tb[0]["cube_id"]:
        ax, ay = ta[0]["center"]
        bx, by = tb[0]["center"]
        wa, _ha = _quad_size(ta[0]["quad"])
        wb, _hb = _quad_size(tb[0]["quad"])
        if closeup:
            scale = max(wa, wb, 1.0)
            drift_tol = max(12.0, 0.10 * scale)
            width_tol = max(12.0, 0.10 * scale)
        else:
            drift_tol = width_tol = 12.0
        if abs(ax - bx) <= drift_tol and abs(ay - by) <= drift_tol and \
                abs(wa - wb) <= width_tol and \
                is_plausible_cube(tb[0]["quad"], tb[0]["center"], max_side):
            tb[0]["confirm"] = "relaxed" if closeup else "pair"
            return [tb[0]]
        if not closeup:
            return []
    elif ta or tb:
        if not closeup:
            # Tag thấy số lượng mơ hồ (0 vs 1, hoặc >1) -> không fallback
            # màu trên cùng cặp frame để tránh gán nhầm ID.
            return []
    else:
        # 2. Proven (đã có confirm 2-frame + width-drift nội bộ).
        try:
            proven = proven_color_candidates(f1, f2, want_id)
        except Exception:
            proven = []
        if len(proven) == 1:
            if is_plausible_cube(proven[0]["quad"], proven[0]["center"],
                                 max_side):
                return proven
            return []
        if proven:
            return []
        # 3. HSV nội bộ tối giản: 1 ứng viên mỗi frame + drift + width-drift.
        ha = [c for c in a if c.get("source") == "hsv_single"]
        hb = [c for c in b if c.get("source") == "hsv_single"]
        if len(ha) == 1 and len(hb) == 1 and \
                ha[0]["cube_id"] == hb[0]["cube_id"]:
            ax, ay = ha[0]["center"]
            bx, by = hb[0]["center"]
            wa, _ = _quad_size(ha[0]["quad"])
            wb, _ = _quad_size(hb[0]["quad"])
            if abs(ax - bx) <= 12 and abs(ay - by) <= 12 and \
                    abs(wa - wb) <= 12 and is_plausible_cube(
                        hb[0]["quad"], hb[0]["center"], max_side):
                return [hb[0]]
        return []
    if closeup:
        # Rớt pair vì rung: chấp nhận single-frame cùng ID (tay settled).
        ids_a = {t["cube_id"] for t in ta}
        ids_b = {t["cube_id"] for t in tb}
        if len(ta) <= 1 and len(tb) <= 1 and (ids_a | ids_b) and \
                (not ids_a or not ids_b or ids_a == ids_b):
            solo = tb[0] if len(tb) == 1 else ta[0]
            if is_plausible_cube(solo["quad"], solo["center"], max_side):
                solo = dict(solo)
                solo["confirm"] = "single"
                return [solo]
    return []


# ---------------------------------------------------------------- state
def load_state() -> dict:
    try:
        data = json.loads(STATE_FILE.read_text())
        return data if isinstance(data, dict) else {"phase": "idle"}
    except (OSError, ValueError):
        return {"phase": "idle"}


def save_state(state: dict) -> None:
    tmp = STATE_FILE.with_name(f"{STATE_FILE.name}.{__import__('os').getpid()}.tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False))
    tmp.replace(STATE_FILE)


# ---------------------------------------------------------------- arm adapters
class ArmAdapter:
    """Wrapper mỏng quanh Arm_Lib để test thay bằng FakeArm.

    Arm_Lib in 'serial error' và trả None khi byte rác/timeout (đã biết ở
    luồng cũ) nên mọi read đều retry 5 lần, validate khoảng như
    t8_motion_worker.read_joint.
    """

    def __init__(self, arm):
        self._arm = arm

    def _read_once(self, joint: int):
        try:
            return self._arm.Arm_serial_servo_read(joint)
        except Exception:
            return None

    def _read_retry(self, joint: int) -> float:
        vmax = 270.0 if joint == 5 else 180.0
        for _ in range(5):
            v = self._read_once(joint)
            try:
                f = float(v) if v is not None else float("nan")
            except (TypeError, ValueError):
                f = float("nan")
            import math as _m
            if _m.isfinite(f) and 0.0 <= f <= vmax:
                return fine_angle(self._arm, joint, f)
            time.sleep(0.08)
        raise RuntimeError(f"không đọc được khớp {joint} sau 5 lần thử")

    def read_joints(self) -> list[float]:
        return [self._read_retry(j) for j in range(1, 6)]

    def read_j1(self) -> float:
        return self._read_retry(1)

    def write_j1(self, angle: float, ms: int = 600) -> None:
        self._arm.Arm_serial_servo_write(1, float(angle), int(ms))

    def write6(self, joints5, gripper: int, ms: int) -> None:
        self._arm.Arm_serial_servo_write6(
            float(joints5[0]), float(joints5[1]), float(joints5[2]),
            float(joints5[3]), float(joints5[4]), int(gripper), int(ms))

    def write6_array(self, joints6, ms: int) -> None:
        self._arm.Arm_serial_servo_write6_array(list(joints6), int(ms))

    def write_gripper(self, angle: int, ms: int) -> None:
        self._arm.Arm_serial_servo_write(6, int(angle), int(ms))

    def wait_settled(self, target5, tol: float = 6.0,
                     timeout_s: float = 3.0) -> dict:
        """Đợi khớp tới nơi trong tol (độ). Trả {'ok', 'err'}.

        Chống kẹp hụt khi servo còn đang chạy (J2 quăng 40° tải nặng).
        Không tới nơi sau timeout -> caller abort thay vì kẹp gió.
        """
        t0 = time.monotonic()
        err = [float("inf")] * 5
        while time.monotonic() - t0 < timeout_s:
            try:
                cur = [float(v) for v in self.read_joints()]
            except RuntimeError:
                time.sleep(0.2)
                continue
            err = [abs(c - t) for c, t in zip(cur, list(target5)[:5])]
            if max(err) <= tol:
                return {"ok": True,
                        "err": [round(v, 1) for v in err]}
            time.sleep(0.2)
        return {"ok": False, "err": [round(v, 1) for v in err]}


def observe_pose(j1: float = 90.0) -> list[float]:
    return [float(j1), *OBSERVE_J245, float(OPEN_ANGLE)]


def draw_debug_frame(frame, cands, j1: float):
    """Overlay quad/center/source lên frame cho live window --show."""
    vis = frame.copy()
    color_of = {"apriltag": (0, 255, 0), "hsv_single": (255, 255, 0)}
    for c in cands or []:
        try:
            pts = np.asarray(c["quad"], float).reshape(4, 2).astype(int)
        except (TypeError, ValueError):
            continue
        src = str(c.get("source", "?"))
        col = color_of.get(src, (0, 255, 255))
        if src.startswith("proven"):
            col = (0, 255, 255)
        cv2.polylines(vis, [pts], True, col, 2)
        cx, cy = c["center"]
        cv2.circle(vis, (int(cx), int(cy)), 4, col, -1)
        cv2.putText(vis, f"id{c.get('cube_id')}|{src}",
                    (int(cx) + 8, int(cy) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cv2.putText(vis, f"J1={j1:.0f}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    return vis


def run_search_centering(cap, arm: ArmAdapter, want_id: int | None = None,
                         timeout_s: float = TRACK_TIMEOUT_S,
                         dry_move: bool = False,
                         show: bool = False,
                         tuning: SearchTuning = FAST_TUNING) -> dict:
    """Sweep nhanh tới khi thấy, chậm-chặt tới center. Trả lock info.

    dry_move=True: không gửi lệnh motor (cho test), chỉ mô phỏng logic trên frame.
    """
    tag = TagDetector()
    t0 = time.monotonic()
    j1 = 90.0
    try:
        j1 = arm.read_j1()
    except RuntimeError:
        j1 = 90.0
    # Bắt đầu quét về phía giữa bàn (J1=90): từ biên không phải đi hết biên rồi quay lại.
    planner = SearchPlanner(tuning, j1, direction=(-1 if j1 > 90.0 else 1),
                            fx=float(_math_mod.FX), center_x=float(_math_mod.CX))
    x_hist: list[bool] = []
    last_center = None  # (cx, cy) bám cuối cho assoc gate F2
    assoc_miss = 0
    last_seen = 0.0
    last_frame = None
    last_log = 0.0
    scans = 0

    def _fresh():
        """Đọc 1 frame tươi resize sẵn. None khi lỗi camera."""
        ok, fr = cap.read()
        if not ok or fr is None:
            return None
        if (fr.shape[1], fr.shape[0]) != (640, 480):
            fr = cv2.resize(fr, (640, 480))
        return fr

    # Bắt đầu từ pose bất kỳ: nếu đã thấy cube ngay tại pose hiện tại thì giữ
    # nguyên J2-J5 (toán ray-plane dùng khớp đọc thật). Chỉ khi không thấy mới
    # về pose observe để quét J1.
    at_observe = True
    if not dry_move:
        seen_here = any(detect_confirmed(_fresh, want_id, tag) for _ in range(3))
        if seen_here:
            print("[START] đã thấy cube tại pose hiện tại; không về pose observe.",
                  flush=True)
            arm.write_gripper(OPEN_ANGLE, 500)
            time.sleep(0.6)
            at_observe = False
        else:
            arm.write6_array(observe_pose(j1), 1200)
            time.sleep(1.2)

    while time.monotonic() - t0 < timeout_s:
        # F3: confirm trên 2 frame tươi THẬT (trước đây truyền cùng 1 frame
        # 2 lần nên gate drift/width vô tác dụng với flicker đơn-frame).
        cands = detect_confirmed(_fresh, want_id, tag)
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.1)
            continue
        if (frame.shape[1], frame.shape[0]) != (640, 480):
            frame = cv2.resize(frame, (640, 480))
        last_frame = frame

        # F2: association — candidate nhảy xa vị trí bám cuối (cube<->thảm
        # zone) thì coi như không thấy; miss 3 lần liên tiếp thì bám lại.
        if cands:
            ax, ay = cands[0]["center"]
            if not assoc_ok(last_center, ax, ay):
                assoc_miss += 1
                if assoc_miss >= ASSOC_MAX_MISS:
                    last_center = None
                    assoc_miss = 0
                cands = []
        if show:
            try:
                cv2.imshow(GRASP_WIN,
                           draw_debug_frame(frame, cands, j1))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    raise TimeoutError("người dùng thoát live window (q)")
            except cv2.error:
                show = False  # headless: tắt window, giữ log + ảnh abort
        planner.j1 = j1
        if not cands:
            # FAST: chưa thấy -> bước nhanh.
            x_hist.append(False)
            assoc_miss += 1
            if assoc_miss >= ASSOC_MAX_MISS:
                last_center = None
                assoc_miss = 0
            move = planner.miss()
            j1 = move.j1
            scans += 1
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                print(f"[SCAN] chưa thấy cube (id={want_id}); J1={j1:.0f}° "
                      f"quét {scans} bước; đặt cube trước camera Sonix rồi thử lại.",
                      flush=True)
            if not dry_move and not at_observe:
                # Mất cube ở pose lạ: về observe rồi mới quét J1 (an toàn + phủ FOV).
                arm.write6_array(observe_pose(j1), 1200)
                time.sleep(1.2)
                at_observe = True
            if not dry_move:
                arm.write_j1(j1, move.ms)
                time.sleep(move.delay)
                # Xả frame cũ + ổn định auto-exposure sau bước xoay nhanh.
                for _ in range(3):
                    cap.grab()
                for _ in range(2):
                    cap.read()
            continue
        cx, cy = cands[0]["center"]
        last_seen = time.monotonic()
        last_center = [float(cx), float(cy)]
        assoc_miss = 0
        # F1: LOCK = X center 5 frame liên tiếp (trục có servo) + Y advisory.
        x_ok = is_x_centered(cx)
        y_ok = is_y_ok(cy)
        x_hist.append(x_ok)
        if confirm_center(x_hist) and y_ok:
            try:
                servo5 = arm.read_joints() if not dry_move else [j1, *OBSERVE_J245]
            except RuntimeError:
                servo5 = [j1, *OBSERVE_J245]
            return {"locked": cands[0], "servo5": [float(v) for v in servo5],
                    "j1": float(j1), "frames": len(x_hist)}
        # SLOW: thấy nhưng chưa center -> bước tỉ lệ nhỏ.
        move = planner.seen(cx)
        j1 = move.j1
        if time.monotonic() - last_log > 5.0:
            last_log = time.monotonic()
            w, h = _quad_size(cands[0]["quad"])
            print(f"[TRACK] thấy cube {cands[0]['cube_id']} tại "
                  f"({cx:.0f},{cy:.0f}) [{cands[0]['source']} {w:.0f}x{h:.0f} "
                  f"y_ok={int(y_ok)}]; J1={j1:.1f}°",
                  flush=True)
        if not dry_move:
            arm.write_j1(j1, move.ms)
            time.sleep(move.delay)
            for _ in range(2):
                cap.read()  # ổn định exposure sau bước chậm
    if last_frame is not None:
        try:
            cv2.imwrite("/tmp/search_center_abort.png", last_frame)
            print("[DEBUG] đã lưu frame lúc abort: /tmp/search_center_abort.png",
                  flush=True)
        except cv2.error:
            pass
    raise TimeoutError(f"quá thời gian center cube ({timeout_s:g}s); abort về observe")


def plan_descend(kin, x: float, y: float,
                 target_z: float = DESCEND_TARGET_Z,
                 anchors: tuple = DESCEND_ANCHOR_ZS):
    """Anchor thấp nhất có nghiệm IK + hạ J2/J3 kiểm chứng bằng FK.

    Trả (approach_joints5, grasp_joints5, info). Mọi bước đều clamp và
    verify FK: z đạt target ±8mm, trôi XY trong budget. Raise khi không an
    toàn thay vì di chuyển mù.
    """
    last = None
    failures = []
    for z_hi in anchors:
        try:
            anchor = [float(v) for v in
                      kin.ik(float(x), float(y), float(z_hi))]
        except Exception as exc:
            failures.append(f"anchor z={z_hi}: IK fail ({exc})")
            last = exc
            continue
        if len(anchor) < 5:
            failures.append(f"anchor z={z_hi}: thiếu khớp")
            last = RuntimeError(f"IK anchor trả thiếu khớp: {anchor}")
            continue
        try:
            p0 = [float(v) for v in kin.fk(anchor[:5])]
        except Exception as exc:
            failures.append(f"anchor z={z_hi}: FK fail ({exc})")
            last = exc
            continue
        dz_need = float(target_z) - p0[2]
        if dz_need >= -0.005:
            return anchor, anchor, {"mode": "anchor-low-enough",
                                    "z_hi": float(z_hi), "dz": dz_need}
        # Độ nhạy 3D theo từng trục (probe ±3°), không giả định dấu.
        sens = {}
        for axis in (1, 2):  # J2, J3
            for d in (3.0, -3.0):
                test = list(anchor)
                test[axis] += d
                try:
                    p = [float(v) for v in kin.fk(test[:5])]
                except Exception as exc:
                    last = exc
                    continue
                sens[(axis, d)] = ((p[0] - p0[0]) / d,
                                   (p[1] - p0[1]) / d,
                                   (p[2] - p0[2]) / d)
        cands = []  # (grasp5, how, deg_info)
        # 1. LSQ 2 trục phối hợp: đủ dz mà triệt drift XY.
        try:
            g, dd = _lsq_grasp(anchor, sens, dz_need)
            cands.append((g, "lsq-J2J3", dd))
        except RuntimeError:
            pass  # suy biến/quá clamp -> fallback đơn trục
        # 2. Fallback đơn trục tốt nhất (tỉ lệ trôi/dz nhỏ nhất).
        best = None
        for (axis, d), (sx, sy, sz) in sens.items():
            lat = math.hypot(sx, sy)
            if abs(sz) < 1e-5:
                continue  # trục không ăn vào z, bỏ qua
            deg_cand = dz_need / sz  # delta góc để đạt target
            if best is None or lat / abs(sz) < best[0]:
                best = (lat / abs(sz), axis, deg_cand)
        if best is not None:
            _, axis, deg = best
            grasp = list(anchor)
            grasp[axis] += deg
            cands.append((grasp, f"single-J{axis + 1}", round(deg, 2)))
        if not cands:
            failures.append(f"anchor z={z_hi}: không có trục hạ")
            last = RuntimeError(f"không có trục hạ tại anchor z={z_hi}")
            continue
        for grasp, how, deg_info in cands:
            if isinstance(deg_info, list):
                over = max(abs(v) for v in deg_info) > DESCEND_MAX_DEG
            else:
                over = abs(deg_info) > DESCEND_MAX_DEG
            if over:
                failures.append(f"anchor z={z_hi}: {how} cần hạ {deg_info}°")
                last = RuntimeError(f"cần hạ {deg_info}° vượt "
                                    f"{DESCEND_MAX_DEG:g}°")
                continue
            if not (all(0.0 <= v <= 180.0 for v in grasp[:4]) and
                    0.0 <= grasp[4] <= 270.0):
                failures.append(f"anchor z={z_hi}: {how} vượt giới hạn")
                last = RuntimeError(f"khớp hạ vượt giới hạn: {grasp}")
                continue
            try:
                p1 = [float(v) for v in kin.fk(grasp[:5])]
            except Exception as exc:
                failures.append(f"anchor z={z_hi}: {how} FK fail ({exc})")
                last = exc
                continue
            if abs(p1[2] - target_z) > DESCEND_Z_TOL:
                failures.append(f"anchor z={z_hi}: {how} "
                                f"FK z={p1[2]:.3f} lệch target")
                last = RuntimeError(f"FK hạ tới z={p1[2]:.3f}, lệch target "
                                    f"{target_z:.3f}")
                continue
            drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if drift > DESCEND_MAX_LATERAL:
                failures.append(f"anchor z={z_hi}: {how} "
                                f"trôi XY {drift * 1000:.0f}mm")
                last = RuntimeError(f"hạ trôi XY {drift * 1000:.0f}mm "
                                    "vượt giới hạn")
                continue
            info = {"mode": "descend", "z_hi": float(z_hi), "how": how,
                    "z_reached": round(p1[2], 4),
                    "drift_mm": round(drift * 1000, 1)}
            if isinstance(deg_info, list):
                info["deg"] = deg_info
                info["axis"] = "2+3"
            else:
                info["deg"] = deg_info
                info["axis"] = int(how.split("J")[1])
            return anchor, grasp, info
    raise RuntimeError(f"không hạ an toàn tới z={target_z} tại "
                       f"({x:.3f},{y:.3f}): {'; '.join(failures) or last}")


def _lsq_grasp(anchor, sens, dz_need):
    """J2+J3 phối hợp: đủ dz, min trôi XY (weighted LSQ).

    sens: {(axis, d): (sx, sy, sz)} m/độ từ probe. Raise khi suy biến
    (2 trục song song) hoặc quá clamp — caller fallback đơn trục.
    """
    cols = []
    for axis in (1, 2):
        key = (axis, 3.0)
        if key not in sens:
            raise RuntimeError("thiếu probe hạ")
        cols.append(sens[key])
    S = np.array(cols, float).T  # 3x2
    W = np.diag([1.0, 1.0, 10.0])  # z ép chặt, XY min
    A = W @ S
    b = W @ np.array([0.0, 0.0, float(dz_need)])
    # det của ATA 2x2:
    ata00 = A[0, 0] ** 2 + A[1, 0] ** 2 + A[2, 0] ** 2
    ata11 = A[0, 1] ** 2 + A[1, 1] ** 2 + A[2, 1] ** 2
    ata01 = A[0, 0] * A[0, 1] + A[1, 0] * A[1, 1] + A[2, 0] * A[2, 1]
    det = ata00 * ata11 - ata01 * ata01
    if abs(det) < 1e-12:
        raise RuntimeError("2 trục song song, không triệt drift được")
    d2 = ((A[0, 0] * b[0] + A[1, 0] * b[1] + A[2, 0] * b[2]) * ata11 -
          (A[0, 1] * b[0] + A[1, 1] * b[1] + A[2, 1] * b[2]) * ata01) / det
    d3 = (ata00 * (A[0, 1] * b[0] + A[1, 1] * b[1] + A[2, 1] * b[2]) -
          ata01 * (A[0, 0] * b[0] + A[1, 0] * b[1] + A[2, 0] * b[2])) / det
    if max(abs(d2), abs(d3)) > DESCEND_MAX_DEG:
        raise RuntimeError(f"LSQ cần ({d2:.1f},{d3:.1f})° vượt clamp")
    grasp = list(anchor)
    grasp[1] += float(d2)
    grasp[2] += float(d3)
    return grasp, [round(float(d2), 2), round(float(d3), 2)]


def solve_pick_ik(kin, x: float, y: float) -> dict:
    """Ưu tiên nghiệm direct THẤP (ngậm sâu); direct cao -> descend từ đó.

    Lỗ IK + vùng biên (vd (-0.087,-0.060) chỉ solve ở z=0.068 = cạp đỉnh
    cube 7mm) nên nghiệm direct cao hơn target+tol bị coi như anchor để hạ
    kiểm chứng, thay vì gắp luôn như trước.
    """
    last = None
    high = None  # (z, joints) direct cao nhất nhưng vẫn solve được
    for z in PICK_Z_FALLBACKS:
        try:
            joints = [float(v) for v in kin.ik(float(x), float(y), float(z))]
        except Exception as exc:
            last = exc
            continue
        if float(z) <= DESCEND_TARGET_Z + DESCEND_Z_TOL:
            return {"joints": joints, "z": float(z), "approach": None,
                    "mode": "direct"}
        high = (float(z), joints)
    if high is not None:
        z_hi, joints_hi = high
        # Thử hạ từ chính anchor direct này trước khi nhận gắp cao.
        try:
            approach, grasp, info = plan_descend(
                kin, float(x), float(y), anchors=(z_hi,))
            return {"joints": grasp, "z": DESCEND_TARGET_Z,
                    "approach": approach, "mode": info["mode"],
                    "info": info}
        except Exception as exc:
            last = exc
        # Hạ không an toàn -> trả direct cao kèm cảnh báo để caller log,
        # không abort oan khi không còn cách nào khác.
        return {"joints": joints_hi, "z": z_hi, "approach": None,
                "mode": "direct-high", "warn": str(last)}
    try:
        approach, grasp, info = plan_descend(kin, float(x), float(y))
        return {"joints": grasp, "z": DESCEND_TARGET_Z,
                "approach": approach, "mode": info["mode"], "info": info}
    except Exception as exc:
        last = exc
    raise RuntimeError(f"IK không có nghiệm tại ({x:.3f},{y:.3f}): {last}")


def verify_and_correct(cap, arm: ArmAdapter, kin, pick: dict,
                       joints, cube_id: int) -> tuple:
    """Đo residual ở pre-close bằng mắt thật, hiệu chỉnh mục tiêu.

    Model-reality có bias hệ thống mức cm (hiệu chuẩn camera chưa xong):
    nhìn lại cube bằng chính camera ở tư thế sắp kẹp (khớp tươi), tính
    lệch world XY, cộng thẳng vào mục tiêu (thấy ở đâu lệnh tới đó) rồi
    giải IK lại. Trả
    (joints2, sol2_or_None, report). Ảnh lưu VERIFY_IMG để audit hậu kiểm.
    """
    tag = TagDetector()
    try:
        servo_now = [float(v) for v in arm.read_joints()]
    except RuntimeError as exc:
        raise RuntimeError(f"không đọc khớp khi verify: {exc}")

    def _fresh():
        ok, fr = cap.read()
        if not ok or fr is None:
            return None
        if (fr.shape[1], fr.shape[0]) != (640, 480):
            fr = cv2.resize(fr, (640, 480))
        return fr

    # Xả bộ đệm V4L2: nếu không, frame đọc được là ảnh chụp TRƯỚC khi tay di
    # chuyển, ghép với khớp mới sẽ cho residual giả cỡ vài cm.
    flush = getattr(cap, "grab", None)
    for _ in range(6 if flush else 0):
        flush()
    cands = detect_confirmed(_fresh, int(cube_id), tag,
                             max_side=VERIFY_MAX_SIDE, closeup=True)
    ok, frame = cap.read()
    if ok and frame is not None:
        if (frame.shape[1], frame.shape[0]) != (640, 480):
            frame = cv2.resize(frame, (640, 480))
        try:
            cv2.imwrite(VERIFY_IMG,
                        draw_debug_frame(frame, cands, servo_now[0]))
        except cv2.error:
            pass
    if len(cands) != 1:
        dbg = ""
        try:
            okd, frd = cap.read()
            if okd and frd is not None:
                if (frd.shape[1], frd.shape[0]) != (640, 480):
                    frd = cv2.resize(frd, (640, 480))
                all_tags = tag.detect(frd)
                want = [t for t in all_tags
                        if int(t.get("cube_id", -1)) == int(cube_id)]
                dbg_parts = [f"tag_all={len(all_tags)}",
                             f"tag_want={len(want)}"]
                for t in want[:2]:
                    try:
                        w, h = _quad_size(t.get("quad"))
                        cx0, cy0 = t.get("center", (None, None))
                        dbg_parts.append(
                            f"quad={w:.0f}x{h:.0f} c=({cx0},{cy0})")
                    except Exception:
                        pass
                dbg = " debug1f: " + " ".join(dbg_parts)
        except Exception:
            pass
        raise RuntimeError("verify: không thấy lại cube ở pre-close "
                           f"(ảnh {VERIFY_IMG});{dbg} abort thay vì kẹp mù")
    cx, cy = (float(v) for v in cands[0]["center"])
    qw, qh = _quad_size(cands[0]["quad"])
    P, _d = pixel_to_base(cx, cy, servo_now)
    rx, ry = float(P[0]) - pick["x"], float(P[1]) - pick["y"]
    res = math.hypot(rx, ry)
    print(f"[VERIFY] residual {res * 1000:.0f}mm "
          f"({rx * 1000:+.0f},{ry * 1000:+.0f}) via={cands[0].get('confirm')} "
          f"px=({cx:.0f},{cy:.0f}) quad={qw:.0f}x{qh:.0f} "
          f"joints={[round(float(v), 1) for v in servo_now[:5]]} "
          f"ảnh {VERIFY_IMG}",
          flush=True)
    if res <= VERIFY_TOL:
        return joints, None, {"corrected": False,
                              "res_mm": round(res * 1000, 1)}
    if res > VERIFY_ABORT:
        raise RuntimeError(f"verify lệch {res * 1000:.0f}mm vượt "
                           f"{VERIFY_ABORT * 1000:.0f}mm; abort (sai tầng "
                           "khác, không đuổi theo)")
    tx, ty = pick["x"] + rx, pick["y"] + ry
    check_workspace(tx, ty)
    sol2 = solve_pick_ik(kin, tx, ty)
    j2 = list(sol2["joints"])
    if len(j2) >= 5:
        j2[4] = min(270, max(0, j2[0] - pick["yaw"]))
    return j2, sol2, {"corrected": True, "res_mm": round(res * 1000, 1),
                      "mode2": sol2["mode"],
                      "target2": [round(tx, 4), round(ty, 4)]}


def execute_pick_place(arm: ArmAdapter, kin, locked: dict, servo5,
                       cube_id: int, dry_move: bool = False,
                       allow_staged: bool = False,
                       cap=None) -> dict:
    """Gắp tại pose dừng + đặt vào zone.

    Mặc định abort nếu delta J1 >45°. allow_staged=True thì xoay nhiều nhịp
    (mỗi nhịp ≤40°) ở độ cao nâng an toàn J2=120 trước khi vào approach.
    cap (không dry): verify-correct bằng mắt thật ở pre-close rồi mới kẹp.
    """
    cx, cy = locked["center"]
    quad = locked["quad"]
    pick = compute_pick(float(cx), float(cy), quad, servo5)
    check_workspace(pick["x"], pick["y"])
    print(f"[PICK] model x={pick['x']:.4f} y={pick['y']:.4f} "
          f"yaw={pick['yaw']:.1f} d={pick['d']:.4f}", flush=True)
    place = compute_place_delta(float(servo5[0]), int(cube_id),
                                allow_staged=allow_staged)

    joints, z_used = None, None
    sol = solve_pick_ik(kin, pick["x"], pick["y"]) \
        if not dry_move else {"joints": [90, 50, 60, 20, 90],
                              "z": pick["z"], "approach": None,
                              "mode": "dry"}
    joints, z_used = sol["joints"], sol["z"]
    # J5 từ yaw vision (5-DOF).
    yaw = pick["yaw"]
    if len(joints) >= 5 and not dry_move:
        joints[4] = min(270, max(0, joints[0] - yaw))
    save_state({"phase": "moving", "cube_id": int(cube_id),
                "pick": pick, "place": place, "mode": sol["mode"],
                "z_used": z_used,
                "joints": [round(float(v), 1) for v in joints],
                "updated": time.time()})
    if not dry_move:
        # Xuống điểm trung gian: approach (cao, kẹp trung tính J5 solver)
        # nếu có, ngược lại xuống thẳng điểm gắp (direct). Verify NGAY tại
        # đây, trước khi xoay yaw J5 + hạ thấp: ở approach cube nhỏ hơn
        # trong frame và ngón kẹp chưa che tag (case 2026-10-06 mù ở điểm
        # gắp thấp vì tag nở >220px + kẹp che mất). Log IK để audit J1.
        pre = sol["approach"] if sol.get("approach") is not None else joints
        pre_label = ("approach" if sol.get("approach") is not None
                     else "direct")
        if sol.get("approach") is not None:
            print(f"[IK] mode={sol['mode']} z={z_used:.3f} "
                  f"approach={[round(float(v), 1) for v in pre[:5]]} "
                  f"grasp={[round(float(v), 1) for v in joints[:5]]}",
                  flush=True)
        else:
            print(f"[IK] mode={sol['mode']} z={z_used:.3f} "
                  f"direct={[round(float(v), 1) for v in joints[:5]]}",
                  flush=True)
        arm.write6(pre, OPEN_ANGLE, 1500)
        time.sleep(1.6)
        st = arm.wait_settled(pre[:5])
        print(f"[SETTLED] {pre_label} err={st['err']}", flush=True)
        if not st["ok"]:
            raise RuntimeError(f"khớp chưa tới {pre_label} {st['err']}; "
                               "abort thay vì kẹp gió")
        verify = {"corrected": False, "res_mm": None}
        sol2 = None
        if cap is not None:
            # Mắt thật đo residual ở pre-close; lệch hệ thống thì cộng
            # thẳng vào mục tiêu (thấy ở đâu lệnh tới đó) giải IK lại
            # (1 lần), rồi mới kẹp.
            joints_v, sol2, verify = verify_and_correct(
                cap, arm, kin, pick, joints, int(cube_id))
            if sol2 is not None:
                if sol2.get("approach") is not None:
                    arm.write6(sol2["approach"], OPEN_ANGLE, 1500)
                    time.sleep(1.6)
                    st2 = arm.wait_settled(sol2["approach"][:5])
                    print(f"[SETTLED] approach2 err={st2['err']}",
                          flush=True)
                    if not st2["ok"]:
                        raise RuntimeError(
                            f"khớp chưa tới approach2 {st2['err']}; abort")
                joints = joints_v
                arm.write6(joints, OPEN_ANGLE, 1500)
                time.sleep(1.6)
                st2 = arm.wait_settled(joints[:5])
                print(f"[SETTLED] grasp2 err={st2['err']}", flush=True)
                if not st2["ok"]:
                    raise RuntimeError(
                        f"khớp chưa tới điểm gắp2 {st2['err']}; abort")
                z_used = sol2["z"]
                sol = sol2
        if sol2 is None and sol.get("approach") is not None:
            # Mới đứng ở approach (chưa hiệu chỉnh): xuống điểm gắp đã áp
            # yaw J5 rồi mới kẹp. Direct (không approach) thì đã ở điểm
            # gắp từ bước pre, bỏ qua để khỏi di chuyển thừa.
            arm.write6(joints, OPEN_ANGLE, 1500)
            time.sleep(1.6)
            st = arm.wait_settled(joints[:5])
            print(f"[SETTLED] grasp err={st['err']}", flush=True)
            if not st["ok"]:
                raise RuntimeError(f"khớp chưa tới điểm gắp {st['err']}; "
                                   "abort thay vì kẹp gió")
        arm.write_gripper(CLOSE_ANGLE, 600)
        time.sleep(0.8)
        arm._arm.Arm_serial_servo_write(2, 120, 1600)
        time.sleep(1.9)
        if place.get("staged"):
            # Xoay nhiều nhịp ở độ cao nâng, mỗi nhịp ≤40°, verify readback.
            for mid_j1 in place.get("steps", []):
                arm.write_j1(float(mid_j1), 900)
                time.sleep(1.1)
                actual = arm.read_j1()
                if abs(actual - float(mid_j1)) > 12:
                    raise RuntimeError(
                        f"không xác nhận được J1 trung gian {mid_j1:.0f}° "
                        f"(đọc {actual:.0f}°)")
        approach = zone_pose(ZONE_APPROACH, cube_id)
        release = zone_pose(ZONE_RELEASE, cube_id)
        lift = zone_pose(ZONE_LIFT, cube_id)
        arm.write6_array([*approach, CLOSE_ANGLE], 1200)
        time.sleep(1.4)
        arm.write6_array([*release, CLOSE_ANGLE], 900)
        time.sleep(1.0)
        arm.write_gripper(OPEN_ANGLE, 600)
        time.sleep(0.8)
        arm.write6_array([*lift, OPEN_ANGLE], 900)
        time.sleep(1.0)
        arm.write6_array(observe_pose(90.0), 1200)
        time.sleep(1.2)
    save_state({"phase": "empty", "cube_id": int(cube_id),
                "pick": pick, "place": place, "z_used": z_used,
                "mode": sol["mode"], "verify": verify,
                "joints": [round(float(v), 1) for v in joints],
                "updated": time.time()})
    staged_txt = (f" qua {len(place.get('steps', []))} nhịp {place.get('steps', [])}"
                  if place.get("staged") else "")
    descend_txt = ""
    if sol.get("mode") == "descend":
        info = sol.get("info", {})
        descend_txt = (f" [hạ J{info.get('axis')} {info.get('deg')}° "
                       f"từ z={info.get('z_hi')} trôi {info.get('drift_mm')}mm]")
    high_txt = ""
    if sol.get("mode") == "direct-high":
        high_txt = f" [CẢNH BÁO gắp cao z={z_used:.3f}: {sol.get('warn', '')}]"
    verify_txt = ""
    if verify.get("res_mm") is not None:
        verify_txt = f" [verify res={verify['res_mm']}mm"
        if verify.get("corrected"):
            verify_txt += (f" đã hiệu chỉnh target2={verify.get('target2')} "
                           f"mode2={verify.get('mode2')}")
        verify_txt += "]"
    return {"ok": True, "pick": pick, "place": place, "z_used": z_used,
            "joints": joints,
            "reply": (f"Đã gắp ID {cube_id} tại ({pick['x']:.3f},{pick['y']:.3f}) "
                      f"z={z_used:.3f} mode={sol['mode']} "
                      f"yaw={yaw:.1f}° d={pick['d']:.3f}m{descend_txt}{high_txt}"
                      f"{verify_txt}, "
                      f"xoay J1 {place['delta']:+.1f}° "
                      f"về zone {cube_id} (J1={place['j1_zone']:.0f}°){staged_txt}.")}


def open_camera(device: str):
    # Ưu tiên V4L2 trực tiếp (giống kcf/apriltag_follow đang chạy ổn).
    # Mặc định VideoCapture(device) trên máy này rớt vào GStreamer và fail.
    candidates: list = []
    candidates.append((str(device), cv2.CAP_V4L2))
    try:
        idx = int(str(device))
        candidates.append((idx, cv2.CAP_V4L2))
    except (TypeError, ValueError):
        if str(device).startswith("/dev/video"):
            try:
                candidates.append((int(str(device).rsplit("video", 1)[1]),
                                   cv2.CAP_V4L2))
            except (ValueError, IndexError):
                pass
    candidates.append((str(device), cv2.CAP_ANY))
    last_err = ""
    for src, backend in candidates:
        try:
            cap = cv2.VideoCapture(src, backend)
        except cv2.error as exc:
            last_err = str(exc)
            continue
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok = False
        for _ in range(5):  # warm-up auto-exposure, xả frame rác
            ok, _ = cap.read()
            if ok:
                break
        if ok:
            return cap
        cap.release()
    print(f"[WARN] không mở được {device} qua V4L2 ({last_err}); "
          "thử --camera 0/1 hoặc --image ảnh tĩnh để test logic.")
    return None


def refresh_lock(cap, arm: ArmAdapter, want_id: int | None,
                 old_lock: dict, cid: int) -> dict:
    """Đọc khớp + frame tươi sau SPACE, tính lại pick trước khi gắp.

    Khớp LOCK-time đã cũ (võng tải/lượng tử hóa đọc: ±8mm ở pose xiên);
    đọc lại khớp thật + detect frame tươi, assoc với lock cũ (≤100px)
    rồi mới cho gắp. Raise khi không xác nhận lại được.
    """
    tag = TagDetector()
    try:
        servo5 = [float(v) for v in arm.read_joints()]
    except RuntimeError as exc:
        raise RuntimeError(f"không đọc khớp trước gắp: {exc}")

    def _fresh():
        ok, fr = cap.read()
        if not ok or fr is None:
            return None
        if (fr.shape[1], fr.shape[0]) != (640, 480):
            fr = cv2.resize(fr, (640, 480))
        return fr

    cands = detect_confirmed(
        _fresh, want_id if want_id is not None else int(cid), tag)
    if len(cands) != 1:
        raise RuntimeError("không thấy lại cube sau SPACE; abort về observe")
    cx, cy = cands[0]["center"]
    ox, oy = old_lock["locked"]["center"]
    if max(abs(float(cx) - float(ox)),
           abs(float(cy) - float(oy))) > ASSOC_RADIUS:
        raise RuntimeError("cube đổi vị trí sau SPACE; abort về observe")
    if int(cands[0]["cube_id"]) != int(cid):
        raise RuntimeError("ID cube đổi sau SPACE; abort về observe")
    print(f"[REFRESH] khớp tươi {[round(v, 1) for v in servo5]} "
          f"tâm ({cx:.0f},{cy:.0f})", flush=True)
    return {"locked": cands[0], "servo5": servo5,
            "j1": float(servo5[0]), "frames": 0}


def _wait_grasp_confirm(cap, lock: dict, cid: int, show: bool,
                        timeout_s: float = CONFIRM_TIMEOUT_S) -> bool:
    """Đợi người dùng nhấn SPACE để gắp, q để hủy. True = gắp.

    --show: nhấn SPACE/Enter trên live window (vẫn hiển thị quad LOCK).
    terminal: nhấn SPACE/Enter trên stdin (raw 1 phím, fallback input()).
    Hết timeout -> False (về observe an toàn).
    """
    t0 = time.monotonic()
    print(f"[CONFIRM] đã center cube {cid} tại {lock['locked']['center']} "
          f"j1={lock['j1']:.1f}. Nhấn SPACE để gắp vào zone {cid}, "
          f"q để hủy (timeout {timeout_s:.0f}s).", flush=True)
    if show:
        last = None
        while time.monotonic() - t0 < timeout_s:
            ok, frame = cap.read()
            if ok and frame is not None:
                if (frame.shape[1], frame.shape[0]) != (640, 480):
                    frame = cv2.resize(frame, (640, 480))
                last = frame
            if last is not None:
                try:
                    vis = draw_debug_frame(last, [lock["locked"]],
                                           lock["j1"])
                    cv2.putText(vis, "SPACE=gap  q=huy",
                                (10, 455), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow(GRASP_WIN, vis)
                except cv2.error:
                    break  # headless giữa chừng -> rớt về terminal
                else:
                    key = cv2.waitKey(200) & 0xFF
                    if key in (32, 13):
                        return True
                    if key in (ord("q"), ord("Q"), 27):
                        return False
                    continue
            break
    # Terminal: đọc 1 phím raw để SPACE có tác dụng ngay, không cần Enter.
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        if sys.stdin.isatty():
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while time.monotonic() - t0 < timeout_s:
                    import select
                    left = t0 + timeout_s - time.monotonic()
                    r, _, _ = select.select([sys.stdin], [], [], max(0, left))
                    if not r:
                        break
                    ch = sys.stdin.read(1)
                    if ch in (" ", "\r", "\n"):
                        return True
                    if ch in ("q", "Q", "\x1b"):
                        return False
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return False
    except (ImportError, OSError, termios.error):
        pass
    try:
        ans = input("[CONFIRM] Enter = gắp, q + Enter = hủy: ").strip().lower()
    except EOFError:
        return False
    return ans in ("", "y", "yes", "gap")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", default="auto",
                    help="auto = camera tay tự nhận (cube_vision.cameras), hoặc /dev/videoN")
    ap.add_argument("--image", default=None,
                    help="ảnh tĩnh BGR 640x480 để test logic không cần camera")
    ap.add_argument("--cube-id", type=int, default=None,
                    help="1-4, bỏ trống = cube bất kỳ (lấy ID detect được)")
    ap.add_argument("--dry-run", action="store_true",
                    help="không gửi lệnh motor, chỉ chạy logic trên camera")
    ap.add_argument("--timeout", type=float, default=TRACK_TIMEOUT_S)
    ap.add_argument("--allow-staged-rotate", action="store_true",
                    help="cho phép xoay về zone nhiều nhịp (mỗi nhịp ≤40°) "
                         "khi delta J1 vượt 45°; mặc định abort an toàn")
    ap.add_argument("--show", action="store_true",
                    help="mở live window overlay quad/center/source "
                         "(nhấn q để thoát); headless thì tự tắt")
    ap.add_argument("--search-speed", choices=sorted(SEARCH_TUNINGS), default="fast",
                    help="tốc độ quét/bám J1: fast (mặc định, nhanh hơn ~40%%) hoặc normal (số cũ)")
    ap.add_argument("--zone-check", choices=("off", "auto"), default="off",
                    help="auto: trước khi quét, nhìn ô zone của cube và chỉnh J1 thả cho lần chạy này "
                         "nếu điểm thả lệch khỏi ô (cần hand-eye chính xác ở J1 xa; mặc định off "
                         "tới khi validate_hand_eye.py xác nhận)")
    ap.add_argument("--yes", action="store_true",
                    help="bỏ qua xác nhận SPACE, tự gắp ngay sau LOCK")
    ap.add_argument("--verify", action="store_true",
                    help="bật bước nhìn lại và hiệu chỉnh ở điểm gắp (mặc định tắt: "
                         "toạ độ đã hiệu chuẩn đủ chính xác, bước này từng gây gắp hụt)")
    ap.add_argument("--no-calibration", action="store_true",
                    help="bỏ qua config/robot/hand_eye.json, dùng mô hình danh định cũ")
    ap.add_argument("--mount-pitch", type=float, default=None,
                    help="override hiệu chuẩn pitch mount camera (độ); "
                         "mặc định dùng MOUNT_PITCH_CORR_DEG trong math")
    args = ap.parse_args()
    if str(args.camera) == "auto":
        import sys as _sys
        from pathlib import Path as _Path
        _sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
        from cube_vision.cameras import resolve_arg as _resolve_camera
        args.camera = _resolve_camera(args.camera)
    faulthandler.enable()  # truy segfault native (apriltag/camera/serial)
    import cube_search_center_math as _MCM
    cal = None if args.no_calibration else _MCM.load_calibration()
    _MCM.apply_calibration(cal)
    if cal is not None:
        held = [round(float(e) * 1000, 1) for e in cal["held_out_xy_m"]]
        print(f"[CALIB] dùng hand-eye đã hiệu chuẩn ({cal['created']}; kiểm chứng XY {held} mm); "
              f"giao tia ở mặt tag z={cal['tag_top_z'] * 1000:.0f} mm", flush=True)
    else:
        print("[CALIB] CHƯA có hand-eye đạt chuẩn -> mô hình danh định (sai số cỡ cm). "
              "Chạy calibrate_hand_eye.py để gắp chính xác từ pose bất kỳ.", flush=True)
        if args.mount_pitch is not None:
            _MCM.MOUNT_PITCH_CORR_DEG = float(args.mount_pitch)
            print(f"[CALIB] mount pitch = {_MCM.MOUNT_PITCH_CORR_DEG:.1f}°")

    if args.cube_id is not None and args.cube_id not in (1, 2, 3, 4):
        ap.error("--cube-id phải là 1-4")
    cap = None
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[FAIL] không đọc được ảnh {args.image}")
            return 1
        img = cv2.resize(img, (640, 480))

        class _StaticCap:
            def read(self):
                return True, img.copy()

            def grab(self):
                return True

            def release(self):
                pass

        cap = _StaticCap()
    else:
        cap = open_camera(args.camera)
    if cap is None:
        print(f"[FAIL] không mở được camera {args.camera}")
        print("Gợi ý: thử --camera 0 (webcam laptop) hoặc --camera 1, "
              "hoặc --image /tmp/frame.png để test logic.")
        return 1
    save_state({"phase": "sweep", "updated": time.time()})
    if args.dry_run:
        print("[DRY] logic search-center, không chạm motor")

        class _DryArm(ArmAdapter):
            def __init__(self):
                self.j1 = 90.0

            def read_j1(self):
                return self.j1

            def read_joints(self):
                return [self.j1, *OBSERVE_J245]

            def write_j1(self, angle, ms=600):
                self.j1 = float(angle)

            def write6_array(self, joints6, ms):
                self.j1 = float(joints6[0])

        arm: ArmAdapter = _DryArm()  # type: ignore
        kin = None
    else:
        from Arm_Lib import Arm_Device
        arm = ArmAdapter(Arm_Device("/dev/ttyUSB0"))
        kin = _LiveKin()
    if args.zone_check == "auto" and not args.dry_run:
        cal_now = _math_mod.load_calibration()
        if cal_now is None:
            print("[ZONE] bỏ qua kiểm tra zone: chưa có hand-eye đạt kiểm chứng", flush=True)
        else:
            time.sleep(0.5)       # lệnh serial đầu tiên sau khi mở cổng hay bị rơi
            run_zone_check(cap, arm, [args.cube_id] if args.cube_id is not None else [1, 2, 3, 4],
                           cal_now)
    try:
        lock = run_search_centering(cap, arm, args.cube_id,
                                    timeout_s=args.timeout,
                                    dry_move=args.dry_run,
                                    show=args.show,
                                    tuning=SEARCH_TUNINGS[args.search_speed])
        cid = args.cube_id if args.cube_id is not None else int(lock["locked"]["cube_id"])
        print(f"[LOCK] cube {cid} center {lock['locked']['center']} "
              f"j1={lock['j1']:.1f} servo5={lock['servo5']}")
        if args.dry_run:
            pick = compute_pick(lock["locked"]["center"][0],
                                lock["locked"]["center"][1],
                                lock["locked"]["quad"], lock["servo5"])
            place = compute_place_delta(lock["j1"], cid,
                                        allow_staged=args.allow_staged_rotate)
            print(f"[DRY PICK] {pick} [DRY PLACE] {place}")
            return 0
        if not args.yes and not _wait_grasp_confirm(cap, lock, cid,
                                                     args.show):
            raise TimeoutError("hủy gắp theo yêu cầu (hoặc hết 120s "
                               "xác nhận); abort về observe")
        # Fix B: khớp/frame LOCK-time đã cũ (võng tải SPACE-delay) ->
        # đọc lại khớp thật + detect tươi rồi mới tính pick.
        lock = refresh_lock(cap, arm, args.cube_id, lock, cid)
        res = execute_pick_place(arm, kin, lock["locked"], lock["servo5"], cid,
                                 allow_staged=args.allow_staged_rotate,
                                 cap=cap if args.verify else None)
        print("[OK]", res["reply"])
        return 0
    except (TimeoutError, ValueError, RuntimeError) as exc:
        print("[ABORT]", exc)
        try:
            if not args.dry_run:
                arm.write6_array(observe_pose(90.0), 1200)
        except Exception:
            pass
        save_state({"phase": "aborted", "reply": str(exc),
                    "updated": time.time()})
        return 2
    finally:
        cap.release()
        if args.show:
            try:
                cv2.destroyWindow(GRASP_WIN)
            except cv2.error:
                pass
        try:
            if kin is not None:
                kin.close()
        except Exception:
            pass


class _LiveKin:
    """IK/FK giải tích (dofbot_ik): không cần ROS service, pitch chọn theo tầm với."""

    def __init__(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
        import dofbot_ik
        self._k = dofbot_ik

    def fk(self, joints5):
        return list(self._k.fk(joints5))

    def ik(self, x: float, y: float, z: float):
        try:
            return self._k.ik(x, y, z)
        except self._k.NoSolution as exc:
            raise RuntimeError(f"IK không có nghiệm: {exc}")

    def close(self):
        pass


if __name__ == "__main__":
    raise SystemExit(main())
