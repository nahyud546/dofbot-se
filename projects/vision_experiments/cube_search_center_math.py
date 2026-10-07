#!/usr/bin/env python3
"""Toán exact độc lập cho luồng search-center-pick-place.

Không import bất kỳ module luồng cũ nào (t8_motion_worker, stage1...).
Mọi hằng số copy trị số (không shared mutable) để đảm bảo zero-impact.

Quy ước:
- servo_deg [0..180] (J5 tới 270 cho kẹp, nhưng FK chỉ dùng J1..J5 arm).
- q_rad = (servo-90)*PI/180  (kinemarics_dofbot.cpp:40)
- Frame KDL của IK service: X- là phía bàn làm việc ở READY elbow
  (khớp poses.yaml start_scan_all khi FK đúng cho +X forward với elbow khác).
  Hàm này trả về đúng frame URDF/FK đã verify:
    [90,120,172,124,83] -> [0.204,0.001,0.0476] khớp poses.yaml picked_red_blind.
  Luồng search-center làm việc ở elbow READY [90,125,0,0,90] -> X âm,
  nên caller phải giữ nguyên dấu FK, không tự đảo dấu.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DE2RA = math.pi / 180.0
RA2DE = 180.0 / math.pi

# ---- URDF dofbot.urdf ----
J1_O = np.array([0.0, 0.0, 0.0925])
J2_O = np.array([0.0, 5e-05, 0.033])
J3_O = np.array([0.0, 0.00055, 0.08285])
J4_O = np.array([0.0, 5e-05, 0.08285])
J5_O = np.array([-0.00215, -4.5e-05, 0.078149])
CAM_O = np.array([-0.0481, -5e-05, 0.0707])

# Mount effective cho pixel raw: Rz(90°). Nominal URDF Rz270 + ảnh xoay 180°
# -> tương đương Rz90. Giữ explicit để calibrate sau thay 1 chỗ.
MOUNT_RZ90 = np.array(
    [[0.0, -1.0, 0.0, 0.0],
     [1.0, 0.0, 0.0, 0.0],
     [0.0, 0.0, 1.0, 0.0],
     [0.0, 0.0, 0.0, 1.0]],
    float,
)

# Hiệu chuẩn pitch mount camera (độ, xoay quanh trục X camera, áp sau MOUNT).
# Fit 2026-10-06: run LOCK pixel (319,202) servo [125,125,0,0,90] cho
# P=(-0.087,-0.060) nhưng kẹp hụt ~55mm dọc tia (ảnh tay + thước user);
# sàng lọc số: chỉ mount pitch +13~15° (hoặc CAM_O.x~0 / servoBias J3+10)
# tái hiện đúng "dọc tia +50mm, ngang ~0". Chọn +14° (hơi non còn hơn quá,
# vòng verify-correct ở pre-close sẽ khép nốt phần dư).
# Đổi góc tay thoải mái: đây là hằng số của bracket, không phải của pose.
MOUNT_PITCH_CORR_DEG = 14.0

# ---- Camera Sonix 640x480 (camera_info_640x480.yaml) ----
FX = 902.0
FY = 875.4
CX = 320.0
CY = 240.0
IMAGE_SIZE = (640, 480)

# ---- Bàn (table_zones.yaml) ----
Z_TABLE = 0.045

# ---- Hiệu chuẩn hand-eye (calibrate_hand_eye.py) ----
# Chưa nạp: dùng mô hình danh định ở trên và giao tia với mặt bàn (hành vi cũ).
# Đã nạp: mount đo được + giao tia ở đúng độ cao mặt tag/mặt trên cube.
CALIB_FILE = Path(__file__).resolve().parents[2] / "config" / "robot" / "hand_eye.json"
ARM4_T_OPTICAL = None
TARGET_PLANE_Z = Z_TABLE
K1 = 0.0  # méo xuyên tâm bậc 2 (chỉ khác 0 khi đã hiệu chuẩn)

# ---- Workspace cực (tay quét J1 nên cổng chữ nhật x/y loại oan cube lệch bên) ----
WORK_R_MIN = 0.05
WORK_R_MAX = 0.30
WORK_BEARING_MAX_DEG = 70.0  # = dải quét J1 20..160

# ---- Zone joint-space (copy trị số BIN_POSES, không import) ----
ZONE_J1 = {1: 30.0, 2: 150.0, 3: 50.0, 4: 135.0}

# Gate nhìn bàn: vbase.z phải đủ âm (nhìn xuống). Ngửa lên -> reject thay vì ra d âm.
VMIN_Z = -0.1

# Pick Z (wrist target, bằng luồng cũ để IK tương thích)
PICK_Z = 0.047
PICK_Z_FALLBACKS = (0.047, 0.058, 0.068)

# Lỗ IK gần chân (đo live): tại (-0.099,0.012) solver chỉ có nghiệm ở
# z=0.030 (dưới bàn, bỏ) và z>=0.072; dải gắp 0.045-0.068 FAIL toàn bộ.
# Chiến lược: IK anchor thấp nhất có nghiệm + hạ J2/J3 kiểm chứng bằng FK.
# Đo live: anchor 0.072 + hạ J2 ~-8° tới 0.058, trôi ~7-9mm (qua gate).
# Target 0.058 = ngậm 17mm thân cube (cube 0.045-0.075), đủ chắc.
DESCEND_ANCHOR_ZS = (0.072, 0.075, 0.080, 0.090, 0.100)
DESCEND_TARGET_Z = 0.058
DESCEND_MAX_DEG = 20.0
DESCEND_MAX_LATERAL = 0.015
DESCEND_Z_TOL = 0.008

# Holding limit hiện hành
J1_HOLD_LIMIT = 45.0


def load_calibration(path=None):
    """Đọc hand_eye.json; None nếu thiếu, hỏng hoặc chưa đạt kiểm chứng."""
    try:
        data = json.loads(Path(path or CALIB_FILE).read_text())
        mount = np.asarray(data["arm4_T_optical"], float).reshape(4, 4)
        fx, fy, cx, cy = (float(v) for v in data["K"])
        top_z = float(data["tag_top_z"])
        k1 = float(data.get("k1", 0.0))
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if (data.get("accepted") is not True or not np.isfinite(mount).all() or
            not np.allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-3) or
            not (400 < fx < 2000 and 400 < fy < 2000 and 0.02 < top_z < 0.12) or
            not abs(k1) <= 1.5):
        return None
    j1 = data.get("j1_correction") or {}
    scale, offset = 1.0, 0.0
    if j1.get("enabled") is True:
        try:
            scale, offset = float(j1["scale"]), float(j1.get("offset_deg", 0.0))
        except (KeyError, TypeError, ValueError):
            scale, offset = 1.0, 0.0
        if not (0.85 <= scale <= 1.25 and abs(offset) <= 5.0):
            scale, offset = 1.0, 0.0
    # Vùng J1 đã kiểm chứng: dải mẫu hiệu chuẩn thu hẹp 5 độ ở hai đầu (sai số ở rìa dải lớn hơn:
    # đo thật 4.8-6.3 mm ở J1 ~140 so với <=3 mm trong J1 40-130). Chỉ có khi hiệu chuẩn nhiều vị trí.
    j1_range = None
    env = (data.get("envelope") or {}).get("j1")
    if env and int(data.get("n_groups") or 1) > 1:
        try:
            lo, hi = float(env[0]) + J1_ENVELOPE_TRIM_DEG, float(env[1]) - J1_ENVELOPE_TRIM_DEG
            j1_range = (lo, hi) if lo < hi else None
        except (TypeError, ValueError, IndexError):
            j1_range = None
    return {"arm4_T_optical": mount, "K": (fx, fy, cx, cy), "k1": k1,
            "j1_valid_range": j1_range, "j1_scale": scale, "j1_offset_deg": offset,
            "tag_top_z": top_z, "held_out_xy_m": data.get("held_out_xy_m", []),
            "created": data.get("created", "?"),
            "held_out_errors_m": data.get("held_out_errors_m", [])}


def apply_calibration(cal) -> None:
    """Bật (cal từ load_calibration) hoặc tắt (None) mô hình đã hiệu chuẩn."""
    global ARM4_T_OPTICAL, TARGET_PLANE_Z, FX, FY, CX, CY, K1, J1_SCALE, J1_OFFSET_DEG
    if cal is None:
        ARM4_T_OPTICAL, TARGET_PLANE_Z, K1 = None, Z_TABLE, 0.0
        FX, FY, CX, CY = 902.0, 875.4, 320.0, 240.0
        J1_SCALE, J1_OFFSET_DEG = 1.0, 0.0
        return
    J1_SCALE = float(cal.get("j1_scale", 1.0))
    J1_OFFSET_DEG = float(cal.get("j1_offset_deg", 0.0))
    ARM4_T_OPTICAL = np.asarray(cal["arm4_T_optical"], float)
    TARGET_PLANE_Z = float(cal["tag_top_z"])
    FX, FY, CX, CY = cal["K"]
    K1 = float(cal.get("k1", 0.0))


def fine_angle(arm, joint: int, coarse) -> float:
    """Góc servo 0.08 độ từ giá trị thô; Arm_Lib cắt về số nguyên (lệch tới 1 độ)."""
    try:
        if int(arm.id) != joint + 0x30:
            return float(coarse)
        raw = int(arm.servo_H) * 256 + int(arm.servo_L)
    except (AttributeError, TypeError, ValueError):
        return float(coarse)
    if joint == 5:
        angle = 270.0 * (raw - 380) / (3700 - 380)
    else:
        angle = 180.0 * (raw - 900) / (3100 - 900)
        if joint in (2, 3, 4):
            angle = 180.0 - angle
    # Chỉ tin giá trị thô khi nó khớp với số nguyên thư viện vừa trả về.
    return angle if abs(angle - float(coarse)) <= 1.01 else float(coarse)


def _rpy2mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _trans(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy2mat(*rpy)
    T[:3, 3] = np.asarray(xyz, float)
    return T


def _rot_axis(axis, q: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(q), math.sin(q)
    C = 1.0 - c
    R = np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


def servo_to_q(servo_deg: float) -> float:
    return (float(servo_deg) - 90.0) * DE2RA


# Hiệu chỉnh J1: góc thật = 90 + scale*(đọc-90) + offset. Đo được trên tay thật: J1 xoay nhiều hơn
# số đọc ~12% (cube cố định cho XY trôi theo J1). Chỉ bật khi hand_eye.json có
# j1_correction.enabled (cùng cờ này bật cho IK trong dofbot_ik để lệnh và quan sát nhất quán).
J1_SCALE = 1.0
J1_OFFSET_DEG = 0.0
J1_ENVELOPE_TRIM_DEG = 5.0


def true_j1(reading_deg: float, scale: float | None = None, offset: float | None = None) -> float:
    """Góc J1 thật (độ servo) từ số đọc, theo hiệu chỉnh đang bật (hoặc tham số truyền vào)."""
    k = J1_SCALE if scale is None else scale
    off = J1_OFFSET_DEG if offset is None else offset
    return 90.0 + k * (float(reading_deg) - 90.0) + off


def fk_arm4_raw(servo15) -> np.ndarray:
    """T_base_arm4 từ góc servo ĐÃ là góc thật (không áp hiệu chỉnh J1)."""
    if len(servo15) < 5:
        raise ValueError("cần ít nhất 5 servo")
    qs = [servo_to_q(s) for s in servo15[:5]]
    T = np.eye(4)
    for origin, axis, q in (
        (J1_O, (0, 0, 1), qs[0]),
        (J2_O, (0, 1, 0), qs[1]),
        (J3_O, (0, 1, 0), qs[2]),
        (J4_O, (0, 1, 0), qs[3]),
    ):
        T = T @ _trans(origin) @ _rot_axis(axis, q)
    return T


def fk_arm4(servo15) -> np.ndarray:
    """T_base_arm4 từ 5 số đọc servo (J1 qua hiệu chỉnh nếu đang bật; J5 không ảnh hưởng arm4)."""
    if len(servo15) < 5:
        raise ValueError("cần ít nhất 5 servo")
    if J1_SCALE != 1.0 or J1_OFFSET_DEG != 0.0:
        servo15 = [true_j1(servo15[0]), *servo15[1:]]
    return fk_arm4_raw(servo15)


def fk_arm4_cal(servo15, cal) -> np.ndarray:
    """fk_arm4 với hiệu chỉnh J1 lấy từ `cal` (không phụ thuộc trạng thái toàn cục)."""
    scale, offset = float(cal.get("j1_scale", 1.0)), float(cal.get("j1_offset_deg", 0.0))
    if scale != 1.0 or offset != 0.0:
        servo15 = [true_j1(servo15[0], scale, offset), *servo15[1:]]
    return fk_arm4_raw(servo15)


def fk_optical(servo15, mount: np.ndarray | None = None,
               pitch_corr_deg: float | None = None) -> np.ndarray:
    """T_base_optical cho ray-plane."""
    if mount is None and pitch_corr_deg is None and ARM4_T_OPTICAL is not None:
        return fk_arm4(servo15) @ ARM4_T_OPTICAL
    if mount is None:
        mount = MOUNT_RZ90
    if pitch_corr_deg is None:
        pitch_corr_deg = MOUNT_PITCH_CORR_DEG
    T_arm4 = fk_arm4(servo15)
    T = T_arm4 @ _trans(CAM_O) @ np.asarray(mount, float)
    if pitch_corr_deg:
        T = T @ _rot_axis((1.0, 0.0, 0.0), float(pitch_corr_deg) * DE2RA)
    return T


def ray_plane(u: float, v: float, T_opt: np.ndarray,
              z_table: float = Z_TABLE):
    """Giao tia-mặt bàn. Trả (P_base[3], d, vbase[3]).

    Raise ValueError khi pose không nhìn bàn (vbase.z >= VMIN_Z hoặc d<=0)
    thay vì trả số âm như công thức H cố định cũ.
    """
    if not (math.isfinite(u) and math.isfinite(v)):
        raise ValueError("pixel không hữu hạn")
    if not (0 <= u < IMAGE_SIZE[0] and 0 <= v < IMAGE_SIZE[1]):
        raise ValueError("pixel ngoài ảnh 640x480")
    T_opt = np.asarray(T_opt, float)
    if T_opt.shape != (4, 4) or not np.isfinite(T_opt).all():
        raise ValueError("T_opt không hợp lệ")
    xc = (float(u) - CX) / FX
    yc = (float(v) - CY) / FY
    if K1:
        xd, yd = xc, yc
        for _ in range(8):
            scale = 1.0 + K1 * (xc * xc + yc * yc)
            xc, yc = xd / scale, yd / scale
    vc = np.array([xc, yc, 1.0])
    R = T_opt[:3, :3]
    pcam = T_opt[:3, 3]
    vbase = R @ vc
    if not np.isfinite(vbase).all() or vbase[2] >= VMIN_Z:
        raise ValueError(
            f"camera không nhìn bàn (vbase.z={vbase[2]:.3f}); "
            "dừng servo thay vì tính d âm"
        )
    d = (float(z_table) - float(pcam[2])) / float(vbase[2])
    if not math.isfinite(d) or d <= 0:
        raise ValueError(f"depth giao mặt bàn không hợp lệ d={d}")
    P = pcam + d * vbase
    return P, float(d), vbase


def pixel_to_base(u: float, v: float, servo15,
                  z_table: float | None = None):
    """API chính: pixel tâm mặt trên + khớp live -> điểm base trên mặt phẳng đích."""
    T_opt = fk_optical(servo15)
    P, d, _ = ray_plane(u, v, T_opt, TARGET_PLANE_Z if z_table is None else z_table)
    return P, d


def check_workspace(x: float, y: float) -> None:
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("XY không hữu hạn")
    r = math.hypot(x, y)
    bearing = abs(math.degrees(math.atan2(y, -x)))
    if not (WORK_R_MIN - 1e-9 <= r <= WORK_R_MAX + 1e-9 and
            bearing <= WORK_BEARING_MAX_DEG):
        raise ValueError(
            f"tâm gắp ngoài vùng làm việc "
            f"r∈[{WORK_R_MIN},{WORK_R_MAX}] m, lệch hướng ≤{WORK_BEARING_MAX_DEG:.0f}° "
            f"(r={r:.3f}, lệch={bearing:.0f}°)"
        )


def wrap180(deg: float) -> float:
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d if d != -180.0 else 180.0


# Góc J1 đo được trong lần chạy hiện tại (zone_locator); chỉ sống trong tiến trình,
# không ghi file nên lần sau luôn bắt đầu lại từ ZONE_J1.
ZONE_J1_RUNTIME: dict = {}


def zone_j1(cube_id: int) -> float:
    try:
        cid = int(cube_id)
        if cid in ZONE_J1_RUNTIME:
            return float(ZONE_J1_RUNTIME[cid])
        return float(ZONE_J1[cid])
    except (KeyError, TypeError, ValueError):
        raise ValueError("ID cube phải là 1, 2, 3 hoặc 4")


def j1_delta_to_zone(j1_pick: float, cube_id: int) -> float:
    """Góc xoay exact từ pose gắp tới zone (độ, wrap ±180)."""
    if not math.isfinite(float(j1_pick)):
        raise ValueError("J1_pick không hữu hạn")
    return wrap180(zone_j1(cube_id) - float(j1_pick))


def check_j1_hold_limit(delta: float) -> None:
    if abs(float(delta)) > J1_HOLD_LIMIT:
        raise ValueError(
            f"góc J1 nguồn-đích {delta:.1f}° vượt {J1_HOLD_LIMIT:.0f}° khi giữ vật; "
            "abort về observe thay vì xoay 2 nhịp"
        )


def quad_yaw(quad) -> float:
    """Yaw kẹp từ cạnh dài quad, cùng quy ước luồng cũ (±40°)."""
    if len(quad) != 4:
        raise ValueError("cần quad 4 điểm")
    pts = [(float(p[0]), float(p[1])) for p in quad]
    if any(not math.isfinite(v) for p in pts for v in p):
        raise ValueError("quad không hữu hạn")
    edges = [(pts[(i + 1) % 4][0] - pts[i][0],
              pts[(i + 1) % 4][1] - pts[i][1]) for i in range(4)]
    lens = [e[0] ** 2 + e[1] ** 2 for e in edges]
    if max(lens) < 100.0:
        raise ValueError("mặt trên quá nhỏ để gắp")
    # Mặt gần vuông (tag 120x102 cho yaw -26° toàn nhiễu): ước lượng góc
    # từ cạnh dài không còn ý nghĩa, mà ngón lệch >~20° sẽ húc góc cube
    # (mở 37mm không ôm nổi đường chéo hiệu dụng 40mm). Trả trung tính.
    if max(lens) / max(min(lens), 1e-9) < 1.25 ** 2:
        return 0.0
    dx, dy = max(edges, key=lambda e: e[0] ** 2 + e[1] ** 2)
    angle = math.degrees(math.atan2(dy, dx)) % 90.0
    yaw = -angle if angle <= 45.0 else 90.0 - angle
    return max(-40.0, min(40.0, yaw))
