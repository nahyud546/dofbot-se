"""Luật để camera "tự biết" mình nhìn chưa tốt, và nên làm gì tiếp.

Hai mức:
  assess_view   một khung của một tag: có đáng tin để dùng không, vì sao không.
  assess_fused  kết quả gộp nhiều khung (`multiview.solve_tag`): đã đủ chắc chưa hay cần nhìn thêm.
Mỗi lý do đi kèm một GỢI Ý hành động (hằng HINT_*) để bộ chọn góc nhìn biết phải đổi gì; module này không biết
tay máy nào. Ngưỡng là hằng số ở đầu file, có lý do đo được ghi bên cạnh.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import multiview as MV

MIN_SIDE_PX = 28.0            # cạnh tag ngắn hơn: góc tag lệch 1 px đã là 3–4% kích thước -> PnP rất nhiễu
EDGE_MARGIN_PX = 20.0         # sát mép ảnh: méo ống kính lớn nhất và tag dễ bị cắt
MAX_PNP_ERR_PX = 3.0
MAX_OBLIQUE_DEG = 65.0        # nhìn xiên quá: tag bị ép dẹt, một cạnh chỉ còn vài pixel
MIN_FLIP_RATIO = 2.0          # nghiệm lật khớp gần bằng nghiệm chọn -> chưa biết pháp tuyến hướng nào
MIN_VIEWS = 2
MIN_BASELINE_M = 0.035        # hai tâm camera gần nhau hơn: thị sai quá nhỏ để chốt độ sâu
MAX_POS_STD_M = 0.002
MAX_FUSED_RMS_PX = 2.5
MAX_VIEW_RMS_PX = 4.0         # một khung lệch hẳn so với phần còn lại (rung, nhận nhầm góc)

HINT_CLOSER = "closer"            # lại gần / phóng to mục tiêu
HINT_CENTRE = "centre"            # đưa mục tiêu vào giữa ảnh
HINT_FRONTAL = "frontal"          # nhìn thẳng mặt tag hơn
HINT_PARALLAX = "parallax"        # đổi vị trí camera (khác hẳn chỗ cũ) để có thị sai
HINT_RESHOOT = "reshoot"          # chụp lại (rung/mờ/nhận sai góc)
HINT_IN_ENVELOPE = "in_envelope"  # đưa khớp về vùng hand-eye đã kiểm chứng


@dataclass
class Quality:
    ok: bool
    score: float                      # 0..1, càng cao càng tốt; chỉ để xếp hạng
    reasons: list = field(default_factory=list)
    hints: list = field(default_factory=list)
    detail: dict = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.ok

    def text(self) -> str:
        return "ổn" if self.ok else "; ".join(self.reasons)


def _add(reasons, hints, reason, hint):
    reasons.append(reason)
    if hint not in hints:
        hints.append(hint)


def assess_view(view: MV.View, size_m: float = MV.TAG_SIZE_M, j1=None, j1_range=None) -> Quality:
    """Chấm một khung. `j1`/`j1_range`: góc J1 lúc chụp và vùng hand-eye tin được (nếu là camera tay)."""
    corners = np.asarray(view.corners_px, float).reshape(4, 2)
    w, h = view.camera.image_size
    reasons, hints = [], []
    sides = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
    margin = float(min(corners[:, 0].min(), corners[:, 1].min(), w - corners[:, 0].max(), h - corners[:, 1].max()))
    detail = {"side_px": float(sides.min()), "margin_px": margin}
    if sides.min() < MIN_SIDE_PX:
        _add(reasons, hints, f"tag quá nhỏ trong ảnh (cạnh {sides.min():.0f} px, cần ≥ {MIN_SIDE_PX:.0f})", HINT_CLOSER)
    if margin < EDGE_MARGIN_PX:
        _add(reasons, hints, f"tag sát mép ảnh ({margin:.0f} px)", HINT_CENTRE)
    if j1 is not None and j1_range and not (j1_range[0] <= float(j1) <= j1_range[1]):
        _add(reasons, hints, f"J1={float(j1):.0f}° ngoài vùng hand-eye {j1_range[0]:.0f}–{j1_range[1]:.0f}°",
             HINT_IN_ENVELOPE)
    candidates = MV.pnp_candidates(view, size_m)
    if not candidates:
        _add(reasons, hints, "PnP không giải được", HINT_RESHOOT)
        return Quality(False, 0.0, reasons, hints, detail)
    err, opt_T_tag = candidates[0]
    oblique = math.degrees(math.acos(min(1.0, abs(float(
        opt_T_tag[:3, 2] @ (opt_T_tag[:3, 3] / np.linalg.norm(opt_T_tag[:3, 3])))))))
    ratio = math.inf if len(candidates) < 2 else candidates[1][0] / max(err, 0.05)
    detail.update(pnp_err_px=err, oblique_deg=oblique, flip_ratio=ratio, range_m=float(np.linalg.norm(opt_T_tag[:3, 3])))
    if err > MAX_PNP_ERR_PX:
        _add(reasons, hints, f"PnP lệch {err:.1f} px (cần ≤ {MAX_PNP_ERR_PX:g})", HINT_RESHOOT)
    if oblique > MAX_OBLIQUE_DEG:
        _add(reasons, hints, f"nhìn xiên {oblique:.0f}° so với mặt tag (cần ≤ {MAX_OBLIQUE_DEG:.0f}°)", HINT_FRONTAL)
    if ratio < MIN_FLIP_RATIO:
        _add(reasons, hints, f"hướng mặt tag còn mơ hồ (nghiệm lật khớp gần bằng, tỉ số {ratio:.1f})", HINT_PARALLAX)
    score = (min(1.0, sides.min() / (2 * MIN_SIDE_PX)) * min(1.0, max(0.0, margin) / (3 * EDGE_MARGIN_PX))
             * max(0.0, 1.0 - err / (2 * MAX_PNP_ERR_PX)) * max(0.1, math.cos(math.radians(oblique))))
    return Quality(not reasons, float(score), reasons, hints, detail)


def assess_fused(fused, min_views: int = MIN_VIEWS) -> Quality:
    """Chấm kết quả gộp nhiều khung: đủ để coi là x, y, z thật chưa."""
    if not fused:
        return Quality(False, 0.0, ["chưa có khung nào dùng được"], [HINT_RESHOOT])
    reasons, hints = [], []
    std = float(np.max(fused["pos_std_m"]))
    detail = {"n_views": fused["n_views"], "baseline_m": fused["baseline_m"], "pos_std_m": std,
              "rms_px": fused["rms_px"], "flip_margin": fused["flip_margin"]}
    if fused["n_views"] < min_views:
        _add(reasons, hints, f"mới có {fused['n_views']} góc nhìn (cần ≥ {min_views}): chưa có thị sai, "
             "độ sâu chỉ dựa vào kích thước tag", HINT_PARALLAX)
    elif fused["baseline_m"] < MIN_BASELINE_M:
        _add(reasons, hints, f"các góc nhìn quá gần nhau ({fused['baseline_m'] * 1000:.0f} mm, cần ≥ "
             f"{MIN_BASELINE_M * 1000:.0f})", HINT_PARALLAX)
    if fused["rms_px"] > MAX_FUSED_RMS_PX:
        _add(reasons, hints, f"các góc nhìn không khớp nhau ({fused['rms_px']:.1f} px, cần ≤ {MAX_FUSED_RMS_PX:g})",
             HINT_RESHOOT)
    elif max(fused["view_rms_px"]) > MAX_VIEW_RMS_PX:
        _add(reasons, hints, f"một góc nhìn lệch hẳn ({max(fused['view_rms_px']):.1f} px)", HINT_RESHOOT)
    if fused["flip_margin"] < MIN_FLIP_RATIO:
        _add(reasons, hints, f"hướng mặt tag còn mơ hồ (tỉ số {fused['flip_margin']:.1f})", HINT_PARALLAX)
    if fused["n_views"] >= min_views and std > MAX_POS_STD_M:
        _add(reasons, hints, f"vị trí còn bất định ±{std * 1000:.1f} mm (cần ≤ {MAX_POS_STD_M * 1000:g})", HINT_PARALLAX)
    score = max(0.0, 1.0 - std / (4 * MAX_POS_STD_M)) * max(0.0, 1.0 - fused["rms_px"] / (2 * MAX_FUSED_RMS_PX))
    return Quality(not reasons, float(score), reasons, hints, detail)


def worst_view(fused) -> int:
    """Chỉ số góc nhìn khớp tệ nhất trong kết quả gộp (ứng viên để loại rồi gộp lại)."""
    return int(np.argmax(fused["view_rms_px"]))
