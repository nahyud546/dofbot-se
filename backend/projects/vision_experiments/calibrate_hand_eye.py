#!/usr/bin/env python3
"""Tự hiệu chuẩn camera gắn trên tay (eye-in-hand) bằng AprilTag trên cube.

Đặt MỘT cube có tag ngửa lên, cách chân robot ~15 cm phía trước, rồi chạy:

  source /opt/ros/humble/setup.bash   # không bắt buộc, chỉ cần Arm_Lib + cv2
  /usr/bin/python3 projects/vision_experiments/calibrate_hand_eye.py \
      --camera /dev/video2 --cube-id 3

Tay tự đi qua ~24 pose quanh pose quan sát (đầu kẹp không thấp hơn 9 cm), mỗi pose
đọc khớp thật + 4 góc tag. Bộ giải tìm đồng thời: mount camera (arm4->optical),
pose tag trong base_link và hệ số tiêu cự, bằng sai số chiếu lại của 4 góc.
Một phần mẫu được giữ lại để kiểm chứng độc lập (<=5 mm mới chấp nhận).

Kết quả:
  config/robot/hand_eye.json            -> cube_search_center.py
  config/robot/cube_6d_calibrated.yaml  -> T8 (perception ROS, --ros-config)
  config/robot/hand_eye_samples.json    -> dữ liệu thô, giải lại bằng --solve-only
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import cube_search_center_math as M

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "config" / "robot"
CALIB_JSON = OUT_DIR / "hand_eye.json"
CALIB_YAML = OUT_DIR / "cube_6d_calibrated.yaml"
SAMPLES_JSON = OUT_DIR / "hand_eye_samples.json"
BASE_YAML = ROOT / "ros" / "src" / "cap_vision" / "config" / "cube_6d_urdf.yaml"
LOCK_FILE = Path("/tmp/t8_motion.lock")

TAG_SIZE_M = 0.020      # cùng giả định với object_models.yaml của perception
CUBE_SIZE_M = 0.030
READY = [90.0, 125.0, 0.0, 0.0, 90.0]
OPEN_ANGLE = 25
MIN_TIP_Z = 0.090       # đầu kẹp luôn cao hơn mặt cube (~0.06 m) >= 25 mm
ACCEPT_HELD_OUT_M = 0.005              # kiểm chứng 3D qua PnP (perception của T8)
ACCEPT_XY_M = 0.004                    # kiểm chứng XY trên bàn (search-center)
MIN_FIT, MIN_VAL = 8, 3
# Ràng buộc mềm: ảnh một tag nhỏ không xác định tốt chiều sâu/tiêu cự/chiều cao.
# Mỗi prior quy về "pixel tương đương" PRIOR_PX / sigma.
PRIOR_PX = 6.0
SIGMA_NORMAL_RAD = math.radians(1.0)   # cube nằm phẳng trên bàn ngang
SIGMA_CAM_POS_M = 0.010                # camera gần vị trí danh định đã đo
SIGMA_LOG_F = 0.05                     # tiêu cự gần ước lượng FOV hiện có
SIGMA_LOG_RATIO = 0.05                 # hiệu chỉnh fy/fx
SIGMA_K1 = 0.5                         # méo xuyên tâm bậc 2
# Mặt tag khi cube nằm trên bàn: độ cao gắp đã kiểm chứng bằng gắp thật là
# TCP z=0.047 ở tâm cube 30 mm -> mặt trên ~0.061. Ảnh xác định chiều cao kém
# (dao động ~1 cm giữa các cách chia mẫu) nên neo vào số đo vật lý này.
TAG_Z_PRIOR_M = 0.061
SIGMA_TAG_Z_M = 0.004


# ------------------------------------------------------------------ toán
def tag_points(size_m: float = TAG_SIZE_M) -> np.ndarray:
    """Góc tag trong frame tag (z ra khỏi mặt tag), cùng thứ tự detector."""
    s = float(size_m) / 2.0
    return np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]])


def to_T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def from_T(T) -> np.ndarray:
    T = np.asarray(T, float)
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


# Đo thật 2026-10-06: trong quy ước khớp của IK (bàn làm việc phía -X), camera
# nằm ở +x của arm4 (URDF ghi -x) và trục quang song song trục link.
NOMINAL_CAM_POS = np.array([0.0481, 0.0, 0.0707])


def nominal_mount(pitch_deg: float = 0.0) -> np.ndarray:
    """arm4->optical danh định: vị trí đo thật + Rz90 (+ pitch thử)."""
    T = M._trans(NOMINAL_CAM_POS) @ M.MOUNT_RZ90
    return T @ M._rot_axis((1.0, 0.0, 0.0), math.radians(pitch_deg))


def nominal_K() -> np.ndarray:
    return np.array([[902.0, 0, 320.0], [0, 875.4, 240.0], [0, 0, 1.0]])


def _dist(k1: float) -> np.ndarray:
    return np.array([float(k1), 0.0, 0.0, 0.0, 0.0])


def pnp_tag(corners, K, size_m: float = TAG_SIZE_M, k1: float = 0.0):
    """(sai số chiếu lại px, camera_T_tag) — nghiệm IPPE tốt nhất."""
    pts = tag_points(size_m).astype(np.float32)
    img = np.asarray(corners, np.float32).reshape(4, 2)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(pts, img, K, _dist(k1),
                                              flags=cv2.SOLVEPNP_IPPE)
    best = None
    for rvec, tvec in zip(rvecs, tvecs) if ok else ():
        if float(np.ravel(tvec)[2]) <= 0:
            continue
        proj = cv2.projectPoints(pts, rvec, tvec, K, _dist(k1))[0].reshape(4, 2)
        err = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1))))
        if best is None or err < best[0]:
            best = (err, to_T(rvec, tvec))
    if best is None:
        raise ValueError("PnP tag thất bại")
    return best


# Tham số: mount(6) + pose tag(6) + log f + log(fy/fx hiệu chỉnh) + k1.
N_PARAMS = 15
N_PRIORS = 11   # 2 pháp tuyến + 3 vị trí camera + f/ratio/k1/z tag + 2 neo xy tag (0 khi không neo)


def unpack(params, K0):
    """(mount, base_T_tag, K, k1) từ vector tham số."""
    K = np.asarray(K0, float).copy()
    K[0, 0] *= math.exp(params[12])
    K[1, 1] *= math.exp(params[12] + params[13])
    return to_T(params[0:3], params[3:6]), to_T(params[6:9], params[9:12]), K, float(params[14])


def n_priors(n_groups: int = 1, fit_j1: bool = False, fit_pp: bool = False) -> int:
    return N_PRIORS + 3 * (n_groups - 1) + (2 if fit_j1 else 0) + (2 if fit_pp else 0)


def group_tag_pose(params, group: int):
    """base_T_tag của nhóm `group`: nhóm 0 ở params[6:12], các nhóm sau ở params[15 + 6*(g-1):]."""
    if group == 0:
        return to_T(params[6:9], params[9:12])
    start = N_PARAMS + 6 * (group - 1)
    return to_T(params[start:start + 3], params[start + 3:start + 6])


SIGMA_J1_SCALE = 0.15          # prior yếu: dữ liệu quyết định (scale = 1 + tham số)
SIGMA_J1_OFFSET_DEG = 3.0


def j1_params(params, n_groups: int):
    """(scale, offset_deg) nếu vector tham số có chỗ cho hiệu chỉnh J1, ngược lại (1, 0)."""
    base = N_PARAMS + 6 * (n_groups - 1)
    if len(params) >= base + 2:
        return 1.0 + float(params[base]), float(params[base + 1])
    return 1.0, 0.0


SIGMA_PP_PX = 12.0             # prior cho lệch tâm ảnh (cx, cy) so với 320/240


def pp_params(params, n_groups: int, fit_j1: bool, fit_pp: bool):
    """(dcx, dcy): lệch tâm ảnh nếu vector tham số có chỗ cho nó, ngược lại (0, 0)."""
    if not fit_pp:
        return 0.0, 0.0
    base = N_PARAMS + 6 * (n_groups - 1) + (2 if fit_j1 else 0)
    return float(params[base]), float(params[base + 1])


def arm4_with_j1(servo, scale: float, offset: float):
    """T_base_arm4 khi J1 thật = 90 + scale*(đọc-90) + offset."""
    return M.fk_arm4_raw([M.true_j1(servo[0], scale, offset), *servo[1:]])


def _residuals(params, arm4_list, corner_list, K0, pts, groups=None, fit_j1=False, fit_pp=False,
               anchor=None):
    """arm4_list: ma trận T_base_arm4, hoặc (fit_j1=True) danh sách servo thô để áp hiệu chỉnh J1."""
    X, T, K, k1 = unpack(params, K0)
    groups = [0] * len(arm4_list) if groups is None else list(groups)
    n_groups = max(groups) + 1
    j1_scale, j1_offset = j1_params(params, n_groups) if fit_j1 else (1.0, 0.0)
    dcx, dcy = pp_params(params, n_groups, fit_j1, fit_pp)
    K = K.copy()
    K[0, 2] += dcx
    K[1, 2] += dcy
    if fit_j1:
        arm4_list = [arm4_with_j1(sv, j1_scale, j1_offset) for sv in arm4_list]
    poses = [T] + [group_tag_pose(params, g) for g in range(1, n_groups)]
    tag_base = [(P @ np.c_[pts, np.ones(4)].T) for P in poses]
    out = []
    for B, corners, g in zip(arm4_list, corner_list, groups):
        cam = np.linalg.inv(B @ X) @ tag_base[g]
        z = np.maximum(cam[2], 1e-4)
        x, y = cam[0] / z, cam[1] / z
        d = 1.0 + k1 * (x * x + y * y)
        out.append(np.c_[K[0, 0] * x * d + K[0, 2], K[1, 1] * y * d + K[1, 2]] - corners)
    a = anchor or {}
    cam_pos, cam_sigma = a.get("cam_pos", (NOMINAL_CAM_POS, SIGMA_CAM_POS_M))
    k1_centre, k1_sigma = a.get("k1", (0.0, SIGMA_K1))
    tag_z_v, tag_z_s = a.get("tag_z", (TAG_Z_PRIOR_M, SIGMA_TAG_Z_M))
    priors = [
        T[:2, 2] / SIGMA_NORMAL_RAD,
        (X[:3, 3] - cam_pos) / cam_sigma,
        ((T[:2, 3] - a["tag_xy"][0]) / a["tag_xy"][1]) if "tag_xy" in a else np.zeros(2),
        [params[12] / a.get("log_f_sigma", SIGMA_LOG_F), params[13] / a.get("log_ratio_sigma", SIGMA_LOG_RATIO),
         (params[14] - k1_centre) / k1_sigma, (T[2, 3] - tag_z_v) / tag_z_s]]
    for P in poses[1:]:                 # mỗi vị trí cube thêm: nằm phẳng + cùng độ cao với nhóm 0
        priors.append(P[:2, 2] / SIGMA_NORMAL_RAD)
        priors.append([(P[2, 3] - T[2, 3]) / SIGMA_TAG_Z_M])
    if fit_j1:
        priors.append([(j1_scale - 1.0) / SIGMA_J1_SCALE, j1_offset / SIGMA_J1_OFFSET_DEG])
    if fit_pp:
        priors.append([dcx / SIGMA_PP_PX, dcy / SIGMA_PP_PX])
    return np.concatenate([np.concatenate(out).ravel(), np.concatenate(priors) * PRIOR_PX])


def quick_fit(samples, K0=None, size_m: float = TAG_SIZE_M):
    """(mount, K, tag_base) ước lượng nhanh từ mọi mẫu, để dự đoán pose kế tiếp."""
    from scipy.optimize import least_squares
    K0 = nominal_K() if K0 is None else np.asarray(K0, float).reshape(3, 3)
    pts = tag_points(size_m)
    arm4 = [M.fk_arm4_raw(s["servo"]) for s in samples]
    corners = [np.asarray(s["corners"], float).reshape(4, 2) for s in samples]
    X, T, K, _ = unpack(_fit(arm4, corners, K0, pts, size_m, least_squares).x, K0)
    return X, K, T[:3, 3]


def _fit(fit_B, fit_C, K0, pts, size_m, least_squares, groups=None, fit_j1=False, fit_pp=False,
         anchor=None):
    """fit_B: ma trận T_base_arm4 (hoặc servo thô khi fit_j1=True)."""
    groups = [0] * len(fit_B) if groups is None else list(groups)
    n_groups = max(groups) + 1
    best = None
    for pitch in (0.0, 14.0, -14.0, 28.0):
        X0 = nominal_mount(pitch)
        try:
            firsts = [groups.index(g) for g in range(n_groups)]
            cam_T_tags = [pnp_tag(fit_C[i], K0, size_m)[1] for i in firsts]
        except ValueError:
            continue
        base_T = [M.fk_arm4_raw(fit_B[i]) if fit_j1 else fit_B[i] for i in firsts]
        tags0 = [from_T(B0 @ X0 @ cam_T_tag) for B0, cam_T_tag in zip(base_T, cam_T_tags)]
        x0 = np.concatenate([from_T(X0), tags0[0], [0.0, 0.0, 0.0], *tags0[1:],
                             [0.0, 0.0] if fit_j1 else [], [0.0, 0.0] if fit_pp else []])
        res = least_squares(_residuals, x0, args=(fit_B, fit_C, K0, pts, groups, fit_j1, fit_pp, anchor),
                            loss="soft_l1", f_scale=PRIOR_PX, max_nfev=400)
        if best is None or res.cost < best.cost:
            best = res
    if best is None:
        raise ValueError("không khởi tạo được bài toán (PnP thất bại)")
    return best


def undistort_normalised(u, v, K, k1):
    x, y = (float(u) - K[0, 2]) / K[0, 0], (float(v) - K[1, 2]) / K[1, 1]
    x0, y0 = x, y
    for _ in range(8):
        d = 1.0 + k1 * (x * x + y * y)
        x, y = x0 / d, y0 / d
    return x, y


def ray_plane_xy(u, v, base_T_optical, K, k1, plane_z):
    """Điểm base nơi tia qua pixel cắt mặt phẳng z = plane_z (cách runtime dùng)."""
    x, y = undistort_normalised(u, v, K, k1)
    ray = base_T_optical[:3, :3] @ np.array([x, y, 1.0])
    origin = base_T_optical[:3, 3]
    return origin + ray * (plane_z - origin[2]) / ray[2]


MAX_EXTRA_GROUP_TILT_DEG = 8.0
# Độ cao mặt tag đã được xác nhận bằng gắp thật (fit tâm bàn + search-center 2026-10-06). Dữ liệu
# nhiều vị trí không xác định được độ cao tuyệt đối (đánh đổi với tiêu cự/độ cao camera) nên
# neo chặt vào giá trị này khi có nhiều nhóm, tránh làm dịch toàn bộ vị trí ~6 mm.
ANCHOR_JSON = OUT_DIR / "hand_eye.center_proven.json"
J1_FIT_MIN_SPAN_DEG = 45.0     # chỉ tự fit hiệu chỉnh J1 khi các mẫu phủ J1 đủ rộng


def anchor_from_json(path, tag_z_sigma: float = 0.0015) -> dict | None:
    """Neo vào một hiệu chuẩn đã được kiểm chứng bằng gắp thật (hand_eye.json một nhóm, accepted).

    Dữ liệu nhiều vị trí chỉ xác định được độ lệch theo J1/bên trái-phải; tiêu cự, độ cao camera
    và độ cao mặt bàn đánh đổi lẫn nhau nên nếu để tự do thì toàn bộ vị trí dịch ~6-8 mm so với
    mô hình đã gắp đúng. Các prior chặt này (vị trí tag nhóm 0 ±0.4 mm, camera ±1 mm) giữ vùng
    giữa bàn như cũ (dịch < 1 mm) và dành dữ liệu hai bên để chỉnh phần lệch theo J1."""
    try:
        data = json.loads(Path(path).read_text())
        if data.get("accepted") is not True:
            return None
        X = np.asarray(data["arm4_T_optical"], float).reshape(4, 4)
        tag = np.asarray(data["base_T_tag"], float).reshape(4, 4)
        return {"K": tuple(float(v) for v in data["K"]), "cam_pos": (X[:3, 3], 0.001),
                "tag_xy": (tag[:2, 3], 0.0004),
                "k1": (float(data["k1"]), 0.10), "log_f_sigma": 0.01, "log_ratio_sigma": 0.01,
                "tag_z": (float(data["tag_top_z"]), tag_z_sigma)}
    except (OSError, ValueError, KeyError, TypeError):
        return None


def solve(samples, K0=None, size_m: float = TAG_SIZE_M, holdout: int = 3, fit_j1=None,
          fit_pp=None, anchor=None) -> dict:
    """Giải mount + pose tag + thông số ống kính; kiểm chứng trên mẫu giữ lại.

    samples: [{"servo": [J1..J5], "corners": [[u, v] x4], "group": int (tuỳ chọn)}]
    "group" = vị trí đặt cube: mỗi nhóm có pose tag riêng, còn mount/ống kính dùng chung,
    nên đặt cube ở nhiều chỗ (nhất là hai bên bàn) mở rộng vùng pose được hiệu chuẩn.
    Hai phép kiểm chứng độc lập, đúng với cách từng luồng dùng kết quả:
    - held_out_xy_m: pixel tâm tag + khớp -> giao tia với mặt phẳng tag (search-center).
    - held_out_errors_m: PnP 3D của tag qua mount (perception ROS của T8).
    """
    from scipy.optimize import least_squares
    if K0 is None and anchor and "K" in anchor:
        fx, fy, cx, cy = anchor["K"]
        K0 = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    K0 = nominal_K() if K0 is None else np.asarray(K0, float).reshape(3, 3)
    pts = tag_points(size_m)
    raw_groups = [int(s.get("group", 0)) for s in samples]
    order = sorted(set(raw_groups))
    groups = [order.index(g) for g in raw_groups]          # nhãn liên tục 0..G-1
    n_groups = len(order)
    # Mẫu giữ lại rải đều trong từng nhóm (mỗi mẫu thứ 4) để phủ cả dải pose.
    seen = {}
    val_idx, fit_idx = [], []
    for i, g in enumerate(groups):
        k = seen.get(g, 0)
        seen[g] = k + 1
        (val_idx if k % 4 == holdout else fit_idx).append(i)
    if len(fit_idx) < MIN_FIT or len(val_idx) < MIN_VAL:
        raise ValueError(f"cần >= {MIN_FIT} mẫu fit và >= {MIN_VAL} mẫu kiểm chứng "
                         f"(có {len(fit_idx)}/{len(val_idx)})")
    if any(sum(1 for i in fit_idx if groups[i] == g) < 3 for g in range(n_groups)):
        raise ValueError("mỗi vị trí đặt cube cần >= 3 mẫu để fit")
    servos = [list(map(float, s["servo"])) for s in samples]
    j1_all = [sv[0] for sv in servos]
    if fit_j1 is None:
        # Mặc định KHÔNG fit J1: trên dữ liệu thật hệ số ra ~1.03 và không giảm sai số XY; chỉ
        # fit khi được yêu cầu (--fit-j1) và mẫu phủ J1 đủ rộng.
        fit_j1 = False
    elif fit_j1 and (max(j1_all) - min(j1_all)) < J1_FIT_MIN_SPAN_DEG:
        fit_j1 = False
    if fit_pp is None:                          # tâm ảnh chỉ quan sát được khi tag trải rộng trong ảnh
        fit_pp = n_groups > 1
    arm4 = [M.fk_arm4_raw(sv) for sv in servos]
    corners = [np.asarray(s["corners"], float).reshape(4, 2) for s in samples]
    fit_B = [servos[i] if fit_j1 else arm4[i] for i in fit_idx]
    fit_C = [corners[i] for i in fit_idx]
    fit_G = [groups[i] for i in fit_idx]

    # Pose xoay phải đủ đa dạng, nếu không mount không xác định được.
    fit_arm4 = [arm4[i] for i in fit_idx]
    rots = np.array([cv2.Rodrigues(fit_arm4[0][:3, :3].T @ B[:3, :3])[0].ravel()
                     for B in fit_arm4[1:]])
    singular = np.linalg.svd(rots, compute_uv=False)
    if len(singular) < 2 or singular[1] < 0.10:
        raise ValueError("các pose chưa đủ đa dạng góc xoay (cần đổi cả J1 lẫn J2-J4)")

    p = _fit(fit_B, fit_C, K0, pts, size_m, least_squares, fit_G, fit_j1, fit_pp, anchor).x
    X, T, K, k1 = unpack(p, K0)
    dcx, dcy = pp_params(p, n_groups, fit_j1, fit_pp)
    K[0, 2] += dcx
    K[1, 2] += dcy
    j1_scale, j1_offset = j1_params(p, n_groups) if fit_j1 else (1.0, 0.0)
    if fit_j1:                                  # ma trận arm4 đã hiệu chỉnh J1 cho mọi mẫu
        arm4 = [arm4_with_j1(sv, j1_scale, j1_offset) for sv in servos]
    tags = [T] + [group_tag_pose(p, g) for g in range(1, n_groups)]
    fit_res = _residuals(p, fit_B, fit_C, K0, pts, fit_G, fit_j1, fit_pp, anchor)[:-n_priors(n_groups, fit_j1, fit_pp)].reshape(-1, 2)
    fit_rms = float(np.sqrt(np.mean(np.sum(fit_res ** 2, axis=1))))

    pts4 = np.c_[pts, np.ones(4)].T
    held_3d, held_xy, held_group = [], [], []
    for i in val_idx:
        tag = tags[groups[i]]
        ref = (tag @ pts4).T[:, :3]
        _, cam_T_tag = pnp_tag(corners[i], K, size_m, k1)
        seen_pts = (arm4[i] @ X @ cam_T_tag @ pts4).T[:, :3]
        held_3d.append(float(np.max(np.linalg.norm(seen_pts - ref, axis=1))))
        centre = corners[i].mean(axis=0)
        hit = ray_plane_xy(centre[0], centre[1], arm4[i] @ X, K, k1, tag[2, 3])
        held_xy.append(float(np.linalg.norm(hit[:2] - tag[:2, 3])))
        held_group.append(groups[i])
    tilts = [math.degrees(math.acos(max(-1.0, min(1.0, float(P[2, 2]))))) for P in tags]
    tilt = max(tilts)
    f = math.exp(p[12])
    # Nhóm 0 là mặt bàn chuẩn (<= 5°). Các vị trí cube khác có thể nằm trên nếp gấp/dây
    # nên được nới (<= 8°): XY runtime chỉ dùng mặt phẳng nằm ngang, còn độ chính xác đã
    # được kiểm chứng riêng bằng held_out_xy_m của từng nhóm.
    tilt_ok = tilts[0] <= 5.0 and all(t <= MAX_EXTRA_GROUP_TILT_DEG for t in tilts[1:])
    sane = bool(fit_rms <= 15.0 and 0.8 <= f <= 1.25 and tilt_ok and abs(k1) <= 1.5 and
                abs(dcx) <= 40 and abs(dcy) <= 40)
    servos = np.array([s["servo"][:4] for s in samples], float)
    return {
        "j1_scale": j1_scale, "j1_offset_deg": j1_offset, "j1_fitted": bool(fit_j1),
        "pp_offset_px": [dcx, dcy], "pp_fitted": bool(fit_pp), "group_tilt_deg": tilts,
        "arm4_T_optical": X, "base_T_tag": T, "K": K, "k1": k1, "focal_scale": f,
        "fit_rms_px": fit_rms, "held_out_errors_m": held_3d, "held_out_xy_m": held_xy,
        "held_out_group": held_group, "n_groups": n_groups,
        "n_fit": len(fit_idx), "n_val": len(val_idx),
        # Mặt phẳng runtime = nhóm 0 (cube trên bàn, đã kiểm bằng gắp thật). Các vị trí cube khác
        # có thể nằm cao/nghiêng hơn (đã thấy ~+8 mm ở một bên) nên không được trung bình vào.
        "tag_top_z": float(T[2, 3]), "table_z": float(T[2, 3] - CUBE_SIZE_M),
        "group_tag_z_m": [float(P[2, 3]) for P in tags],
        "tag_normal_tilt_deg": tilt, "rotation_singular": singular.tolist(),
        "envelope": {f"j{i + 1}": [float(servos[:, i].min()), float(servos[:, i].max())]
                     for i in range(4)},
        "accepted": bool(sane and max(held_xy) <= ACCEPT_XY_M),
        "accepted_3d": bool(sane and max(held_xy) <= ACCEPT_XY_M and
                            max(held_3d) <= ACCEPT_HELD_OUT_M),
    }


def write_outputs(result: dict, size_m: float, json_path=CALIB_JSON,
                  yaml_path=CALIB_YAML, base_yaml=BASE_YAML) -> list[str]:
    json_path = Path(json_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    K = result["K"]
    payload = {
        "accepted": result["accepted"], "accepted_3d": result["accepted_3d"],
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "arm4_T_optical": result["arm4_T_optical"].ravel().tolist(),
        "K": [K[0, 0], K[1, 1], K[0, 2], K[1, 2]], "k1": result["k1"],
        "tag_size_m": size_m,
        "tag_top_z": result["tag_top_z"], "table_z": result["table_z"],
        "base_T_tag": result["base_T_tag"].ravel().tolist(),
        "tag_normal_tilt_deg": result["tag_normal_tilt_deg"],
        "focal_scale": result["focal_scale"], "fit_rms_px": result["fit_rms_px"],
        "held_out_xy_m": result["held_out_xy_m"],
        "held_out_errors_m": result["held_out_errors_m"],
        "n_fit": result["n_fit"], "n_val": result["n_val"],
        "n_groups": result.get("n_groups", 1),
        "group_tag_z_m": result.get("group_tag_z_m"),
        # Hiệu chỉnh J1 (góc thật = 90 + scale*(đọc-90) + offset). `enabled` chỉ bật bằng
        # --enable-j1-correction sau khi đã kiểm bằng validate_hand_eye.py.
        "j1_correction": {"scale": result.get("j1_scale", 1.0),
                          "offset_deg": result.get("j1_offset_deg", 0.0),
                          "fitted": result.get("j1_fitted", False),
                          "enabled": bool(result.get("j1_enabled", False))},
        "held_out_group": result.get("held_out_group", []),
        # Vùng khớp mà các mẫu hiệu chuẩn thực sự phủ (ngoài đó là ngoại suy).
        "envelope": result.get("envelope"),
    }
    json_path.write_text(json.dumps(payload, indent=2))
    written = [str(json_path)]
    if yaml_path is not None and not result["accepted"]:
        # Không để YAML cũ tiếp tục áp dụng một hiệu chuẩn không còn đạt.
        Path(yaml_path).unlink(missing_ok=True)
    if result["accepted"] and yaml_path is not None and Path(base_yaml).is_file():
        import yaml
        cfg = yaml.safe_load(Path(base_yaml).read_text())
        full = result["accepted_3d"]
        # Luôn ghi K/k1 đã đo để perception giải PnP của tag (camera_pose, độ sâu
        # dùng chọn tầng) đúng ống kính. Chỉ khi 3D cũng đạt mới bật `calibrated`
        # (pose base của chính perception); ngược lại giữ cờ trung thực là false.
        camera = {"source": ("fixed_apriltag_eye_in_hand" if full
                             else "fixed_apriltag_eye_in_hand_intrinsics_only"),
                  "K": K.ravel().tolist(), "distortion": _dist(result["k1"]).tolist(),
                  "calibrated": bool(full), "extrinsic_calibrated": bool(full)}
        if full:
            # Perception dùng Camera_Link (= arm4 dịch CAM_O của URDF, không xoay) làm mount.
            camera["mount_T_optical"] = (M._trans(-M.CAM_O) @ result["arm4_T_optical"]).ravel().tolist()
        cfg["camera"].update(camera)
        if full:
            table = result["base_T_tag"].copy()
            table[:3, 3] -= table[:3, 2] * CUBE_SIZE_M
            cfg["table"].update(base_T_table=table.ravel().tolist(), calibrated=True)
            cfg["calibration_validation"] = {
                "method": "fixed_apriltag_eye_in_hand",
                "tool": "projects/vision_experiments/calibrate_hand_eye.py",
                "tag_size_m": size_m, "fit_rms_px": result["fit_rms_px"],
                "held_out_errors_m": result["held_out_errors_m"],
            }
        Path(yaml_path).write_text(yaml.safe_dump(cfg, sort_keys=False))
        written.append(str(yaml_path))
    return written


def report(result: dict) -> str:
    held = [round(e * 1000, 1) for e in result["held_out_errors_m"]]
    held_xy = [round(e * 1000, 1) for e in result["held_out_xy_m"]]
    X, K = result["arm4_T_optical"], result["K"]
    axis = X[:3, 2]
    lines = [
        f"fit: {result['n_fit']} mẫu, RMS chiếu lại {result['fit_rms_px']:.2f} px",
        f"kiểm chứng XY trên bàn ({result['n_val']} mẫu giữ lại): {held_xy} mm "
        f"(ngưỡng {ACCEPT_XY_M * 1000:.0f} mm) -> search-center: "
        + ("ĐẠT" if result["accepted"] else "CHƯA ĐẠT"),
        f"kiểm chứng 3D qua PnP: {held} mm (ngưỡng {ACCEPT_HELD_OUT_M * 1000:.0f} mm) "
        "-> T8 perception: " + ("ĐẠT" if result["accepted_3d"] else "CHƯA ĐẠT"),
        (f"J1 thật = 90 + {result['j1_scale']:.3f}*(đọc-90) {result['j1_offset_deg']:+.2f}° "
         f"(tự fit từ dữ liệu; {'ĐANG BẬT' if result.get('j1_enabled') else 'chưa bật'})"
         if result.get("j1_fitted") else "J1: không fit (mẫu phủ J1 chưa đủ rộng)"),
        (f"tâm ảnh lệch ({result['pp_offset_px'][0]:+.1f}, {result['pp_offset_px'][1]:+.1f}) px so với 320/240"
         if result.get("pp_fitted") else "tâm ảnh: cố định 320/240"),
        f"ống kính: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} k1={result['k1']:+.3f}",
        f"camera trong frame arm4: vị trí {np.round(X[:3, 3] * 1000, 1)} mm, "
        f"trục quang lệch {math.degrees(math.atan2(-axis[0], axis[2])):+.1f}° so với trục link",
        f"mặt tag cao z={result['tag_top_z'] * 1000:.1f} mm -> mặt bàn "
        f"z={result['table_z'] * 1000:.1f} mm; pháp tuyến tag lệch "
        f"{result['tag_normal_tilt_deg']:.1f}° so với phương đứng",
    ]
    return "\n".join("[CALIB] " + line for line in lines)


# ------------------------------------------------------------------ pose
def tip_z(servo) -> float:
    """Độ cao điểm kẹp (xấp xỉ: arm4 + 146 mm dọc trục link)."""
    T = M.fk_arm4(servo)
    return float((T @ np.array([-0.0048, 0.0, 0.14624, 1.0]))[2])


def predict_pixel(servo, tag_base, mount, K):
    cam = np.linalg.inv(M.fk_arm4(servo) @ mount) @ np.append(tag_base, 1.0)
    if cam[2] <= 0.05:
        return None
    return (K[0, 0] * cam[0] / cam[2] + K[0, 2], K[1, 1] * cam[1] / cam[2] + K[1, 2],
            float(cam[2]))


def pose_pool(tag_base, mount, K, j1_values=range(55, 126, 5)) -> list[list[float]]:
    """Mọi pose an toàn quanh READY mà tag dự đoán còn nằm gọn trong ảnh."""
    pool = []
    for j1 in j1_values:
        for j2 in range(75, 141, 5):
            for j3 in (0, 5, 10, 15, 20, 25, 30):
                for j4 in (0, 5, 10, 15, 20):
                    servo = [float(j1), float(j2), float(j3), float(j4), 90.0]
                    if tip_z(servo) < MIN_TIP_Z:
                        continue
                    px = predict_pixel(servo, tag_base, mount, K)
                    if px is None or not (95 <= px[0] <= 545 and 80 <= px[1] <= 400):
                        continue
                    if 0.08 <= px[2] <= 0.30:
                        pool.append(servo)
    return pool


def _joint_dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:4], b[:4])))


def next_pose(pool, tried, good, reach_deg: float = 22.0):
    """Pose mới xa các pose đã thử nhất, nhưng không quá xa một mẫu đã thành công.

    Mở rộng dần từ vùng mô hình còn đáng tin, nên không lao tới chỗ mất tag.
    """
    options = [p for p in pool
               if min(_joint_dist(p, t) for t in tried) >= 7.0 and
               min(_joint_dist(p, g) for g in good) <= reach_deg]
    if not options:
        return None
    return max(options, key=lambda p: min(_joint_dist(p, t) for t in tried))


def collect_loop(observe, count: int, log=print, size_m: float = TAG_SIZE_M,
                 start_pose=None, j1_values=range(55, 126, 5), group: int = 0) -> list[dict]:
    """Thu mẫu thích nghi. observe(pose) -> (servo thật, corners | None).

    start_pose: pose nhìn đầu tiên (mặc định READY; cube ở hai bên bàn dùng pose
    của pose_library); j1_values: dải J1 được thử; group: vị trí đặt cube.
    """
    start_pose = list(READY if start_pose is None else start_pose)
    servo, corners = observe(start_pose)
    if corners is None:
        raise RuntimeError("không thấy tag ở pose quan sát; dời cube vào giữa khung hình "
                           "(hoặc chọn --start-pose phù hợp vị trí cube)")
    K, mount = nominal_K(), nominal_mount()
    _, cam_T_tag = pnp_tag(corners, K, size_m)
    tag_base = (M.fk_arm4(servo) @ mount @ cam_T_tag)[:3, 3]
    samples = [{"servo": list(servo), "corners": np.asarray(corners).tolist(), "group": group}]
    tried, good = [start_pose], [start_pose]
    while len(samples) < count and len(tried) < 3 * count + 6:
        pose = next_pose(pose_pool(tag_base, mount, K, j1_values), tried, good)
        if pose is None:
            break
        tried.append(pose)
        try:
            servo, corners = observe(pose)
        except RuntimeError as exc:
            log(f"[CALIB] bỏ pose {[round(v) for v in pose[:4]]}: {exc}")
            continue
        log(f"[CALIB] pose {[round(v) for v in pose[:4]]}: "
            f"{'OK' if corners is not None else 'mất tag/rung'} "
            f"({len(samples) + (corners is not None)}/{count})")
        if corners is None:
            continue
        good.append(pose)
        samples.append({"servo": list(servo), "corners": np.asarray(corners).tolist(),
                        "group": group})
        if len(samples) >= 3 and (len(samples) <= 8 or len(samples) % 4 == 0):
            try:
                mount, K, tag_base = quick_fit(samples, size_m=size_m)
            except ValueError:
                pass
    return samples


# ------------------------------------------------------------------ phần cứng
def _detector():
    from pupil_apriltags import Detector
    return Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=1)


def detect_tag(frame, det, tag_id: int):
    tags = [t for t in det.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if int(t.tag_id) == int(tag_id)]
    if len(tags) != 1:
        return None
    return np.asarray(tags[0].corners, float).reshape(4, 2)


def grab_corners(cap, det, tag_id: int, frames: int = 3):
    """Trung bình góc tag qua vài frame tươi; None nếu rung/mất tag."""
    for _ in range(4):
        cap.grab()
    seen = []
    last = None
    for _ in range(frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            return None, last
        if (frame.shape[1], frame.shape[0]) != (640, 480):
            frame = cv2.resize(frame, (640, 480))
        last = frame
        corners = detect_tag(frame, det, tag_id)
        if corners is None:
            return None, last
        seen.append(corners)
    stack = np.array(seen)
    if float(np.max(np.abs(stack - stack.mean(axis=0)))) > 1.5:
        return None, last
    if stack.mean(axis=0).min() < 8 or stack[..., 0].max() > 632 or stack[..., 1].max() > 472:
        return None, last
    return stack.mean(axis=0), last


def read_servo(arm) -> list[float]:
    out = []
    for joint in range(1, 6):
        for _ in range(6):
            value = arm.Arm_serial_servo_read(joint)
            if value is not None and 0 <= float(value) <= (270 if joint == 5 else 180):
                out.append(M.fine_angle(arm, joint, value))
                break
            time.sleep(0.08)
        else:
            raise RuntimeError(f"không đọc được khớp {joint}")
    return out


def move_and_settle(arm, servo, ms: int = 1400) -> list[float]:
    arm.Arm_serial_servo_write6(*[float(v) for v in servo], OPEN_ANGLE, ms)
    time.sleep(ms / 1000.0 + 0.3)
    previous = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        now = read_servo(arm)
        if previous is not None and max(abs(a - b) for a, b in zip(now, previous)) <= 1.0 \
                and max(abs(a - b) for a, b in zip(now, servo)) <= 5.0:
            time.sleep(0.4)
            again = read_servo(arm)
            return [(a + b) / 2.0 for a, b in zip(now, again)]
        previous = now
        time.sleep(0.25)
    raise RuntimeError(f"khớp không ổn định tại {servo}")


def align_preview(arm, cap, det, tag_id: int, start_pose, ready_pose=None, ms: int = 1800,
                  title: str = "Can chinh cube - SPACE/Enter = bat dau, q = huy", log=print) -> None:
    """Xoay tay sang pose khởi đầu rồi cho xem camera trực tiếp để người dùng đặt cube
    vào GIỮA khung (chữ thập). Chỉ khi cube ở giữa ở pose này, các pose khác mới còn thấy tag.

    Có cửa sổ: vòng xem trực tiếp, tag đúng ID được viền xanh khi nằm trong vùng giữa; nhấn
    SPACE/Enter để bắt đầu, q/ESC để huỷ. Không có màn hình: dừng ở input()."""
    arm.Arm_serial_servo_write6_array(list(ready_pose or READY) + [OPEN_ANGLE], ms)
    time.sleep(ms / 1000.0 + 0.6)
    arm.Arm_serial_servo_write6_array([float(v) for v in start_pose] + [OPEN_ANGLE], ms)
    time.sleep(ms / 1000.0 + 0.6)
    log("[ALIGN] Tay đã ở pose khởi đầu. Đặt cube (tag ngửa lên) vào GIỮA khung hình.")
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    except cv2.error:
        input("[ALIGN] Không mở được cửa sổ xem. Chỉnh cube rồi nhấn Enter: ")
        return
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            if (frame.shape[1], frame.shape[0]) != (640, 480):
                frame = cv2.resize(frame, (640, 480))
            corners = detect_tag(frame, det, tag_id)
            h, w = frame.shape[:2]
            cv2.line(frame, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 255, 255), 1)
            cv2.line(frame, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 255, 255), 1)
            cv2.rectangle(frame, (w // 2 - 120, h // 2 - 90), (w // 2 + 120, h // 2 + 90),
                          (0, 255, 255), 1)
            if corners is None:
                note, colour = f"khong thay tag ID {tag_id}", (0, 0, 255)
            else:
                cx, cy = corners.mean(axis=0)
                centred = abs(cx - w / 2) <= 120 and abs(cy - h / 2) <= 90
                cv2.polylines(frame, [corners.astype(int)], True,
                              (0, 255, 0) if centred else (0, 165, 255), 2)
                note, colour = (("tag o giua - nhan SPACE", (0, 255, 0)) if centred else
                                ("tag chua o giua khung", (0, 165, 255)))
            cv2.putText(frame, note, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            cv2.imshow(title, frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (32, 13):
                return
            if key in (27, ord("q")):
                raise KeyboardInterrupt("người dùng huỷ ở bước căn cube")
    finally:
        cv2.destroyWindow(title)
        cv2.waitKey(1)


def collect(args) -> list[dict]:
    from Arm_Lib import Arm_Device
    from cube_search_center import open_camera
    cap = open_camera(args.camera)
    if cap is None:
        raise RuntimeError(f"không mở được camera {args.camera}")
    lock = LOCK_FILE.open("a+")
    for _ in range(60):
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            break
        except BlockingIOError:
            time.sleep(0.2)
    else:
        raise RuntimeError("cổng tay máy đang bận (/tmp/t8_motion.lock); tắt T8 trước")
    arm = Arm_Device("/dev/ttyUSB0")
    det = _detector()
    samples = []

    def observe(pose):
        servo = move_and_settle(arm, pose)
        corners, frame = grab_corners(cap, det, args.cube_id)
        if args.show and frame is not None:
            if corners is not None:
                cv2.polylines(frame, [corners.astype(int)], True, (0, 255, 0), 2)
            cv2.imshow("calibrate_hand_eye", frame)
            cv2.waitKey(1)
        return servo, corners

    try:
        import pose_library
        if args.start_pose not in pose_library.POSES:
            raise RuntimeError(f"--start-pose phải là một trong {sorted(pose_library.POSES)}")
        if args.no_align:
            if not args.yes:
                input("[CALIB] Đặt 1 cube (tag ngửa lên) trước robot ~15 cm, dọn vật khác. "
                      "Tay sẽ tự chuyển động. Enter để bắt đầu, Ctrl+C để hủy: ")
            arm.Arm_serial_servo_write6_array(READY + [OPEN_ANGLE], 1800)
            time.sleep(2.4)
        else:
            align_preview(arm, cap, det, args.cube_id, pose_library.POSES[args.start_pose])
        wide = args.start_pose != "READY"
        samples = collect_loop(observe, args.count, size_m=args.tag_size,
                               start_pose=pose_library.POSES[args.start_pose],
                               j1_values=range(20, 161, 5) if wide else range(55, 126, 5),
                               group=args.group)
    finally:
        try:
            arm.Arm_serial_servo_write6_array(READY + [OPEN_ANGLE], 1800)
            time.sleep(2.0)
        except Exception:
            pass
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
    return samples


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="auto",
                    help="auto = camera tay tự nhận (cube_vision.cameras), hoặc /dev/videoN")
    ap.add_argument("--cube-id", type=int, default=None, help="ID tag36h11 trên cube (1-4)")
    ap.add_argument("--count", type=int, default=24, help="số mẫu cần thu (>=12)")
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M)
    ap.add_argument("--start-pose", default="READY",
                    help="pose nhìn đầu tiên (READY, LEFT, RIGHT, FAR_LEFT, FAR_RIGHT, ...; xem "
                         "pose_library.POSES). Cube ở hai bên bàn không thấy từ READY.")
    ap.add_argument("--group", type=int, default=0,
                    help="số thứ tự vị trí đặt cube (0 = giữa bàn, 1, 2 = hai bên...). Mẫu của "
                         "nhóm khác nhau được giải chung mount/ống kính, mỗi nhóm một pose tag.")
    ap.add_argument("--fit-j1", action="store_true",
                    help="fit thêm hệ số/độ lệch J1 (mặc định không: dữ liệu thật không cho thấy cần)")
    ap.add_argument("--anchor", type=Path, default=ANCHOR_JSON,
                    help="hand_eye.json một nhóm đã gắp thật đúng, dùng làm neo khi có nhiều nhóm "
                         "(giữ vùng giữa bàn không dịch). 'none' để tắt")
    ap.add_argument("--no-anchor", action="store_true", help="giải tự do, không neo vào hiệu chuẩn giữa bàn")
    ap.add_argument("--enable-j1-correction", action="store_true",
                    help="bật hiệu chỉnh J1 vừa fit (cho cả hand-eye và IK). Chỉ dùng khi hai bên bàn "
                         "đều đã có mẫu và độ lệch giữa các pose giảm rõ rệt")
    ap.add_argument("--no-align", action="store_true",
                    help="bỏ bước xoay tới --start-pose và xem trực tiếp để căn cube vào giữa khung")
    ap.add_argument("--append", action="store_true",
                    help="giữ mẫu các nhóm khác trong file mẫu cũ, thay mẫu của --group này rồi giải lại tất cả")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--yes", action="store_true", help="không hỏi Enter trước khi chạy tay")
    ap.add_argument("--solve-only", type=Path, default=None,
                    help="giải lại từ file mẫu đã thu, không chạy tay")
    args = ap.parse_args()
    if str(args.camera) == "auto":
        import sys as _sys
        from pathlib import Path as _Path
        _sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
        from cube_vision.cameras import resolve_arg as _resolve_camera
        args.camera = _resolve_camera(args.camera)
    if args.solve_only is not None:
        samples = json.loads(args.solve_only.read_text())["samples"]
    else:
        if args.cube_id not in (1, 2, 3, 4) or args.count < 12:
            ap.error("cần --cube-id 1-4 và --count >= 12")
        try:
            samples = collect(args)
        except (RuntimeError, KeyboardInterrupt) as exc:
            print(f"[CALIB] DỪNG: {exc or 'người dùng hủy'}")
            return 2
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        if args.append and SAMPLES_JSON.is_file():
            kept = [s for s in json.loads(SAMPLES_JSON.read_text())["samples"]
                    if int(s.get("group", 0)) != args.group]
            print(f"[CALIB] giữ {len(kept)} mẫu của nhóm khác, thêm {len(samples)} mẫu nhóm {args.group}")
            samples = kept + samples
        SAMPLES_JSON.write_text(json.dumps({"tag_size_m": args.tag_size,
                                            "samples": samples}, indent=1))
        print(f"[CALIB] đã lưu {len(samples)} mẫu thô: {SAMPLES_JSON}")
    try:
        multi = len({int(x.get('group', 0)) for x in samples}) > 1
        anchor = None
        if multi and not args.no_anchor:
            anchor = anchor_from_json(args.anchor)
            print("[CALIB] neo vào hiệu chuẩn giữa bàn đã kiểm chứng: " +
                  (str(args.anchor) if anchor else "KHÔNG tìm thấy bản accepted, giải tự do"))
        result = solve(samples, size_m=args.tag_size, fit_j1=True if args.fit_j1 else None,
                       anchor=anchor)
    except ValueError as exc:
        print(f"[CALIB] KHÔNG GIẢI ĐƯỢC: {exc}")
        print("[CALIB] Gợi ý: đặt cube sao cho tag nằm giữa khung hình ở pose quan sát "
              "(cách chân robot 14-18 cm), đủ sáng, rồi chạy lại.")
        return 2
    result["j1_enabled"] = bool(args.enable_j1_correction and result.get("j1_fitted") and result["accepted"])
    if args.enable_j1_correction and not result["j1_enabled"]:
        print("[CALIB] KHÔNG bật hiệu chỉnh J1: chưa fit được hoặc chưa đạt kiểm chứng XY")
    print(report(result))
    for path in write_outputs(result, args.tag_size):
        print(f"[CALIB] đã ghi {path}")
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    sys.exit(main())
