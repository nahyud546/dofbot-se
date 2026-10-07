"""Offline acceptance for auto calibration (checklist points 1-5).

Read-only: NEVER writes calibrated=true, never overwrites config. Exit 0 chi
khi tat ca gate PASS; nguoi giu data tu quyet dinh tiep theo.

    python3 src/cap_vision/cap_vision/validate_calibration.py \
        --dataset calib_ds_v1/dataset.yaml \
        --calibration calib_ds_v1/red_scene_cal_<ts>.yaml \
        --session calib_ds_v1/session/result.json   # optional
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from red_scene_core import transform  # noqa: E402  (validates 4x4)

ROWS = []


def row(name, ok, detail=""):
    ROWS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--session", default=None)
    ap.add_argument("--intrinsic", default=None)
    a = ap.parse_args()
    ds_path, cal_path = Path(a.dataset), Path(a.calibration)
    ds, cal = load(ds_path), load(cal_path)
    ds_dir = ds_path.parent
    intr_path = Path(a.intrinsic) if a.intrinsic else ds_dir / "intrinsic.yaml"

    # 1. candidate / accepted / rejected + fit / held-out.
    samples = ds.get("samples", [])
    on_disk = [s for s in samples if (ds_dir / s["image"]).exists()]
    row("samples tren dia", len(on_disk) == len(samples),
        f"{len(on_disk)}/{len(samples)}")
    fit = [s for s in samples if not s.get("validation")]
    held = [s for s in samples if s.get("validation")]
    row("fit >= 8", len(fit) >= 8, f"fit={len(fit)}")
    row("held-out >= 3", len(held) >= 3, f"held-out={len(held)}")
    if a.session:
        sess = load(a.session)
        saved, skipped = sess.get("saved", []), sess.get("skipped", [])
        row("session counts khop", len(saved) + len(skipped) >= len(samples),
            f"accepted={len(saved)} rejected={skipped}")
    for s in samples:
        meta = ds_dir / Path(s["image"]).parent / "sample.yaml"
        if not meta.exists():
            row(f"sample.yaml {s['image']}", False, "thieu meta")
            continue
        m = load(meta)
        try:
            transform(m["base_T_mount"])
        except Exception as exc:
            row(f"base_T_mount {s['image']}", False, str(exc)[:80])

    # 2. intrinsic.
    if not intr_path.exists():
        row("intrinsic file", False, f"thieu {intr_path}")
        intr = {}
    else:
        intr = load(intr_path)
        row("intrinsic calibrated flag", bool(intr.get("calibrated")), "")
        err = float(intr.get("reprojection_error_px", 9e9))
        row("reprojection < 0.5px", err < 0.5, f"{err:.4f}px")
        K = np.asarray(intr.get("camera_matrix", []), float)
        row("K hop le", K.shape == (9,) and np.isfinite(K).all()
            and 300 < K[0] < 2000 and 300 < K[4] < 2000
            and abs(K[0] - K[4]) / max(K[0], K[4]) < 0.15,
            f"fx={K[0] if K.shape == (9,) else '?'} fy={K[4] if K.shape == (9,) else '?'}")
        W, H = intr.get("image_width"), intr.get("image_height")
        cx, cy = (K[2], K[5]) if K.shape == (9,) else (None, None)
        row("principal point giua anh", cx is not None and 0.3 * W < cx < 0.7 * W
            and 0.3 * H < cy < 0.7 * H, f"cx={cx} cy={cy} WxH={W}x{H}")
        d = np.asarray(intr.get("distortion_coefficients", []), float)
        row("distortion huu han", d.size > 0 and np.isfinite(d).all()
            and bool((np.abs(d) < 1.0).all()), f"{d.tolist() if d.size else '?'}")

    # 3. mount_T_optical.
    cam = cal.get("camera", {})
    row("camera.calibrated", bool(cam.get("calibrated")), "")
    row("extrinsic_calibrated", bool(cam.get("extrinsic_calibrated")), "")
    try:
        T = transform(cam["mount_T_optical"])
        t = T[:3, 3]
        row("mount_T_optical hop le", bool(np.linalg.norm(t) <= 0.5),
            f"|t|={np.linalg.norm(t) * 1000:.1f}mm t={np.round(t * 1000, 1).tolist()}mm")
    except Exception as exc:
        row("mount_T_optical hop le", False, str(exc)[:100])

    # 4. base_T_table.
    tab = cal.get("table", {})
    row("table.calibrated", bool(tab.get("calibrated")), "")
    try:
        B = transform(tab["base_T_table"])
        n = B[:3, 2]
        o = B[:3, 3]
        row("table normal ~ +Z", bool(n[2] >= 0.90),
            f"normal={np.round(n, 3).tolist()}")
        row("table origin trong tam voi", bool(0.0 <= o[2] <= 0.10)
            and bool(np.linalg.norm(o[:2]) < 0.5),
            f"origin={np.round(o * 1000, 1).tolist()}mm")
    except Exception as exc:
        row("base_T_table hop le", False, str(exc)[:100])

    # 5. held-out errors (khong chi dua tren fit).
    val = cal.get("calibration_validation", {})
    errs = val.get("held_out_errors_m", [])
    row("held-out errors <= 5mm", len(errs) == len(held) and len(errs) > 0
        and bool(max(errs) <= 0.005),
        f"max={max(errs) * 1000:.2f}mm n={len(errs)}" if errs else "thieu")
    if intr and cam:
        same = (list(map(float, intr.get("camera_matrix", []))) == list(map(float, cam.get("K", []))))
        row("K config khop intrinsic file", bool(same),
            "" if same else "WARN: config khong copy dung intrinsic")

    n_fail = sum(1 for _, ok, _ in ROWS if not ok)
    print(f"KET LUAN: {'ACCEPT (chuyen sang validation mapping thuc te)' if n_fail == 0 else f'{n_fail} GATE FAIL - phan tich truoc, KHONG force calibrated=true'}",
          flush=True)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
