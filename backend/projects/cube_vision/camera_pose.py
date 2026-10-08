"""Một camera đang ở đâu trong world, từ các tag mà world đã biết tọa độ.

Dùng cho cả hai trường hợp:
  - camera cố định (webcam, iPhone trên giá): giải một lần, lưu lại, sau đó chỉ kiểm tra có bị xê dịch không;
  - camera cầm tay: giải lại ở MỖI khung hình; không đủ tag thì trả None chứ không đoán.
Intrinsic phải có trước (`cube_vision.intrinsics`), ở đây chỉ còn 6 ẩn số pose. Thuần toán + OpenCV.
"""
from __future__ import annotations

import itertools

import cv2
import numpy as np

from .frames import CameraModel, invert

MIN_TAGS = 2                   # một tag 20 mm cho pose rất yếu (xoay bù trừ với dời); cần ≥ 2 tag tách nhau
MIN_SPREAD_M = 0.04            # các tag phải tách nhau ít nhất chừng này trong world
# Ngưỡng theo số đo thật 2026-10-08 (iPhone nhìn xiên sát bàn, world từ camera tay có z lệch ±5 mm): pose đúng cho
# RMS ~4 px (≈ 2 mm ở 0,45 m), nghiệm lật cho ~15 px. Kiểm chéo bỏ-một-tag chỉ có nghĩa khi còn lại ≥ 3 tag.
MAX_RMS_PX = 6.0
MAX_LEAVE_ONE_OUT_PX = 12.0
MIN_TAGS_FOR_LEAVE_ONE_OUT = 4
REFINE_ACCEPT_PX = 8.0         # khởi tạo từ pose trước: nếu sau tinh chỉnh còn lệch hơn mức này thì tìm lại từ đầu
MOVED_PX = 10.0                # tag lệch hơn mức này so với pose của các tag còn lại => tag đó đã bị dời
# Các tag đồng phẳng cho HAI pose camera đều khớp tạm: pose thật và pose "lật" (camera quay 180° quanh pháp tuyến mặt
# bàn). Đo thật 2026-10-08 trên iPhone cầm tay: pose thật khớp 1,8 / 3,0 px, pose lật 3,8 / 5,6 px. Chỉ nhảy sang
# nghiệm lật khi nó khớp tốt hơn rõ rệt; chênh ít thì giữ nghiệm liền mạch với khung trước.
FLIP_SWITCH_RATIO = 1.5
COPLANAR_RATIO = 0.25          # độ dày của đám điểm so với bề rộng: nhỏ hơn mức này mới coi là đồng phẳng


def _to_T(rvec, tvec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = np.asarray(tvec, float).ravel()
    return T


def _starts(points):
    """Các pose khởi tạo: camera đứng quanh đám điểm (nhiều hướng, nhiều độ cao, hai khoảng cách), nhìn vào tâm."""
    centre = points.mean(axis=0)
    out = []
    for dist in (0.25, 0.5, 0.9):
        for elev in np.radians([8.0, 25.0, 50.0, 80.0]):
            for azim in np.radians(np.arange(0.0, 360.0, 30.0)):
                eye = centre + dist * np.array([np.cos(elev) * np.cos(azim), np.cos(elev) * np.sin(azim), np.sin(elev)])
                z = (centre - eye) / dist
                x = np.cross(z, [0.0, 0.0, 1.0])
                x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
                R = np.stack([x, np.cross(z, x), z], axis=0)             # optical_R_world, ảnh "đứng thẳng"
                for roll in (0.0, np.pi / 2, np.pi, -np.pi / 2):         # camera có thể bị xoay quanh trục nhìn
                    c, s = np.cos(roll), np.sin(roll)
                    Rr = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R
                    out.append((cv2.Rodrigues(Rr)[0], (-Rr @ eye).reshape(3, 1)))
    return out


def _refine(pts, px, camera: CameraModel, optical_T_world, min_height=None):
    """Tinh chỉnh LM từ một pose cho trước (nhanh, vài ms); None khi nghiệm vô lý hoặc camera dưới `min_height`."""
    T0 = np.asarray(optical_T_world, float)
    try:
        rvec, tvec = cv2.solvePnPRefineLM(pts, px, camera.matrix(), camera.dist(),
                                          cv2.Rodrigues(T0[:3, :3])[0], T0[:3, 3].reshape(3, 1))
    except cv2.error:
        return None
    T = _to_T(rvec, tvec)
    if not np.isfinite(T).all() or (pts @ T[:3, :3].T + T[:3, 3])[:, 2].min() <= 0.02:
        return None
    if min_height is not None and float(invert(T)[2, 3]) < min_height:
        return None
    return T


def flip_start(optical_T_world, points):
    """Pose khởi tạo cho nghiệm lật của các điểm gần đồng phẳng, hoặc None khi điểm không đồng phẳng.

    Nghiệm lật = camera quay 180° quanh pháp tuyến mặt phẳng (qua trọng tâm các điểm), rồi xoay 180° quanh trục
    nhìn để ảnh giữ nguyên chiều.
    """
    pts = np.asarray(points, float)
    centre = pts.mean(axis=0)
    _, sing, vt = np.linalg.svd(pts - centre)
    if sing[1] < 1e-9 or sing[2] > COPLANAR_RATIO * sing[1]:
        return None
    n = vt[2]
    half_turn = np.eye(4)
    half_turn[:3, :3] = 2.0 * np.outer(n, n) - np.eye(3)
    half_turn[:3, 3] = centre - half_turn[:3, :3] @ centre
    return invert(half_turn @ invert(optical_T_world) @ np.diag([-1.0, -1.0, 1.0, 1.0]))


def _pnp(points, pixels, camera: CameraModel, min_height=None, init_T=None):
    """optical_T_world từ điểm world ↔ pixel (≥ 4 điểm), hoặc None.

    Các tag trên bàn gần như đồng phẳng: PnP khi đó có HAI nghiệm (nghiệm thật và nghiệm lật qua mặt phẳng), nhìn
    càng xiên càng khó phân biệt. Vì vậy tối ưu từ nhiều pose khởi tạo, bỏ nghiệm đặt camera dưới `min_height`
    (dưới mặt bàn), rồi lấy nghiệm chiếu lại khớp nhất. Có `init_T` (pose khung trước) thì thử nó trước: camera
    cầm tay giữa hai khung chỉ dời một chút nên đi thẳng tới nghiệm đúng, không cần tìm toàn cục (~1 ms thay vì ~1 s).
    """
    pts, px = np.asarray(points, np.float64), np.asarray(pixels, np.float64)
    if init_T is not None:
        T = _refine(pts, px, camera, init_T, min_height)
        if T is not None and float(np.sqrt(np.mean(_errors(pts, px, camera, T) ** 2))) <= REFINE_ACCEPT_PX:
            return T
    K, dist = camera.matrix(), camera.dist()
    best = None
    for rvec0, tvec0 in _starts(pts):
        T = _refine(pts, px, camera, _to_T(rvec0, tvec0), min_height)
        if T is None:
            continue
        err = float(np.sqrt(np.mean(_errors(pts, px, camera, T) ** 2)))
        if np.isfinite(err) and (best is None or err < best[0]):
            best = (err, T)
    return None if best is None else best[1]


def _errors(points, pixels, camera, optical_T_world) -> np.ndarray:
    cam = np.asarray(points, float) @ optical_T_world[:3, :3].T + optical_T_world[:3, 3]
    return np.linalg.norm(camera.project(cam) - np.asarray(pixels, float), axis=1)


def pose_from_tags(world_corners: dict, detections: dict, camera: CameraModel,
                   min_tags: int = MIN_TAGS, above_table: bool = True, init=None) -> dict:
    """world_corners {id: (4,3)} đã biết trong world; detections {id: (4,2)} pixel (ảnh đã xoay đúng hướng).

    Trả dict: ok, reasons, world_T_optical (khi giải được), rms_px, tags (id đã dùng), leave_one_out_px {id: px},
    spread_m. `ok` False nghĩa là KHÔNG nên dùng pose này. above_table: camera phải nằm cao hơn các tag trừ 3 cm
    (đúng với mọi camera nhìn xuống bàn) để loại nghiệm lật của các tag đồng phẳng.
    """
    ids = sorted(set(world_corners) & set(detections))
    out = {"ok": False, "reasons": [], "tags": ids, "world_T_optical": None, "rms_px": None,
           "leave_one_out_px": {}, "spread_m": 0.0}
    if len(ids) < min_tags:
        out["reasons"].append(f"chỉ thấy {len(ids)} tag đã biết trong world (cần ≥ {min_tags})")
        return out
    pts = {i: np.asarray(world_corners[i], float).reshape(4, 3) for i in ids}
    px = {i: np.asarray(detections[i], float).reshape(4, 2) for i in ids}
    centres = np.array([pts[i].mean(axis=0) for i in ids])
    out["spread_m"] = float(max(np.linalg.norm(a - b) for a in centres for b in centres)) if len(ids) > 1 else 0.0
    if len(ids) > 1 and out["spread_m"] < MIN_SPREAD_M:
        out["reasons"].append(f"các tag quá gần nhau ({out['spread_m'] * 1000:.0f} mm)")
    floor = float(min(p[:, 2].min() for p in pts.values())) - 0.03 if above_table else None
    T = _pnp(np.vstack([pts[i] for i in ids]), np.vstack([px[i] for i in ids]), camera, floor,
             None if init is None else invert(init))
    if T is None:
        out["reasons"].append("PnP không giải được")
        return out
    err = _errors(np.vstack([pts[i] for i in ids]), np.vstack([px[i] for i in ids]), camera, T)
    out["rms_px"] = float(np.sqrt(np.mean(err ** 2)))
    out["world_T_optical"] = invert(T)
    if out["rms_px"] > MAX_RMS_PX:
        out["reasons"].append(f"RMS chiếu lại {out['rms_px']:.1f} px (cần ≤ {MAX_RMS_PX:g})")
    if len(ids) >= 3:
        for held in ids:
            rest = [i for i in ids if i != held]
            T_rest = _pnp(np.vstack([pts[i] for i in rest]), np.vstack([px[i] for i in rest]), camera, floor, T)
            if T_rest is not None:
                out["leave_one_out_px"][held] = float(np.mean(_errors(pts[held], px[held], camera, T_rest)))
        worst = max(out["leave_one_out_px"].values(), default=0.0)
        if len(ids) >= MIN_TAGS_FOR_LEAVE_ONE_OUT and worst > MAX_LEAVE_ONE_OUT_PX:
            out["reasons"].append(f"bỏ một tag thì đoán lại nó lệch {worst:.1f} px (cần ≤ {MAX_LEAVE_ONE_OUT_PX:g}): "
                                  "tọa độ world của tag hoặc intrinsic chưa khớp")
    out["ok"] = not out["reasons"]
    return out


def drift_px(world_corners: dict, detections: dict, camera: CameraModel, world_T_optical) -> float | None:
    """Camera cố định còn ở chỗ cũ không: độ lệch trung vị (px) giữa tag chiếu từ world và tag đang thấy."""
    ids = sorted(set(world_corners) & set(detections))
    if not ids:
        return None
    T = invert(world_T_optical)
    err = np.concatenate([_errors(np.asarray(world_corners[i], float).reshape(4, 3),
                                  np.asarray(detections[i], float).reshape(4, 2), camera, T) for i in ids])
    return float(np.median(err))


def track(world_corners: dict, detections: dict, camera: CameraModel, prev=None,
          moved_px: float = MOVED_PX, max_rms_px: float = MAX_RMS_PX, above_table: bool = True,
          allow_global: bool = True) -> dict:
    """Định vị camera cầm tay trong khi một số tag có thể đã bị dời: chỉ các tag còn đứng yên làm mốc.

    Camera dời thì MỌI tag cùng lệch theo một phép dời cứng; một tag bị dời riêng thì lệch khỏi phép dời đó. Nên
    tìm tập tag lớn nhất (≥ MIN_TAGS) cùng giải được với RMS thấp; tag ngoài tập mà vẫn lệch quá `moved_px` là tag
    đã bị dời.

    Thứ tự thử (rẻ trước):
      1. từ `prev` (pose khung trước, world_T_optical): camera chỉ dời chút giữa hai khung nên mỗi tập tag giải trong
         ~1 ms; đây là đường chạy bình thường, kể cả khi có cube bị dời;
      2. nếu không được (camera nhảy xa, mất dấu, khung đầu) và `allow_global`: tìm toàn cục (~1 s) rồi thử lại.
         Nghiệm toàn cục trên TẤT CẢ tag bị tag dời kéo lệch, có thể nằm sai miền hội tụ, nên với ≥ 4 tag còn giải
         toàn cục riêng từng tập thiếu một tag.

    Trả dict: ok, reasons, world_T_optical, anchors [id], moved [id], rms_px, tags (id thấy được cả hai phía).
    Hai tag mà không khớp nhau thì không biết tag nào dời: ok False, không đoán.
    """
    ids = sorted(set(world_corners) & set(detections))
    out = {"ok": False, "reasons": [], "world_T_optical": None, "anchors": [], "moved": [], "rms_px": None,
           "tags": ids}
    if len(ids) < MIN_TAGS:
        out["reasons"].append(f"chỉ thấy {len(ids)} tag đã biết trong world (cần ≥ {MIN_TAGS})")
        return out
    pts = {i: np.asarray(world_corners[i], float).reshape(4, 3) for i in ids}
    px = {i: np.asarray(detections[i], float).reshape(4, 2) for i in ids}
    floor = float(min(p[:, 2].min() for p in pts.values())) - 0.03 if above_table else None

    def stack(subset):
        return np.vstack([pts[i] for i in subset]), np.vstack([px[i] for i in subset])

    def fit(subset, T):
        P, Q = stack(subset)
        return float(np.sqrt(np.mean(_errors(P, Q, camera, T) ** 2)))

    def best_for(subset, starts):
        """(rms, T) tốt nhất của một tập tag, đã so với nghiệm lật; None khi không nghiệm nào đạt."""
        P, Q = stack(subset)
        for T0 in starts:
            T = _refine(P, Q, camera, T0, floor)
            keep = None if T is None else (fit(subset, T), T)
            alt0 = flip_start(T if T is not None else T0, P)
            alt = None if alt0 is None else _refine(P, Q, camera, alt0, floor)
            if alt is not None:
                alt_rms = fit(subset, alt)
                if keep is None or alt_rms * FLIP_SWITCH_RATIO < keep[0]:
                    keep = (alt_rms, alt)
            if keep is not None and keep[0] <= max_rms_px:
                return keep
        return None

    def attempt(starts):
        for size in range(len(ids), max(MIN_TAGS, len(ids) - 3) - 1, -1):
            found = []
            for subset in itertools.combinations(ids, size):
                hit = best_for(subset, starts)
                if hit is not None:
                    found.append((hit[0], subset, hit[1]))
            if found:
                return min(found, key=lambda item: item[0])
        return None

    chosen, full_rms = None, None
    if prev is not None:
        chosen = attempt([invert(prev)])
    if chosen is None and allow_global:
        full = pose_from_tags(world_corners, detections, camera, above_table=above_table)
        full_rms = full["rms_px"]
        if full["world_T_optical"] is not None:
            chosen = attempt([invert(full["world_T_optical"])])
        if chosen is None and len(ids) >= 4:
            found = []
            for held in ids:
                subset = tuple(i for i in ids if i != held)
                P, Q = stack(subset)
                T = _pnp(P, Q, camera, floor)
                if T is not None and fit(subset, T) <= max_rms_px:
                    found.append((fit(subset, T), subset, T))
            chosen = min(found, key=lambda item: item[0]) if found else None
    if chosen is None:
        why = "" if full_rms is None else f" (RMS {full_rms:.1f} px)"
        out["reasons"].append("các tag không khớp nhau và không tách được tag nào bị dời" + why +
                              ("" if allow_global else "; chưa tìm lại toàn cục") +
                              ": cần ít nhất 3 tag đứng yên mới phân biệt được")
        return out
    rms, subset, T = chosen
    out.update(ok=True, world_T_optical=invert(T), anchors=list(subset), rms_px=rms)
    out["moved"] = [i for i in ids if i not in subset and float(np.mean(_errors(pts[i], px[i], camera, T))) > moved_px]
    return out
