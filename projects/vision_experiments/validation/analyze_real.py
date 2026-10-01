#!/usr/bin/env python3
"""Offline analyzer for real-world validation. No ROS, numpy-only.

Inputs (from real_loggers.py + bridge):
  eye_in_hand_real.csv
  identity_real.csv          (ground_truth_cube_id filled offline)
  grasp_execution.jsonl

Outputs:
  summary.json + human-readable table on stdout + example_trace.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def quat_mean_dev(quats):
    """Mean quaternion (sign-aligned average) + per-sample angle deviations."""
    q = np.asarray(quats, float)
    q = q / (np.linalg.norm(q, axis=1, keepdims=True) + 1e-12)
    ref = q[0].copy()
    aligned = np.where((q @ ref)[:, None] < 0, -q, q)
    mean = aligned.mean(axis=0)
    mean = mean / (np.linalg.norm(mean) + 1e-12)
    dots = np.clip(np.abs(aligned @ mean), -1.0, 1.0)
    angs = np.degrees(2 * np.arccos(dots))
    return mean, float(angs.mean()), float(angs.max())


def analyze_eye(rows):
    valid = [r for r in rows if r.get("base_obj_px") not in ("", None)
             and str(r.get("rejected_reason", "")) == ""]
    pos = np.array([[float(r["base_obj_px"]), float(r["base_obj_py"]), float(r["base_obj_pz"])]
                    for r in valid], float) if valid else np.zeros((0, 3))
    out = {"n_frames": len(rows), "n_valid": len(valid),
           "tf_failures": sum(1 for r in rows if str(r.get("tf_ok")) == "0"),
           "rejected_frames": sum(1 for r in rows if str(r.get("rejected_reason", "")) != "")}
    if len(pos):
        out.update({
            "xyz_mean": pos.mean(axis=0).tolist(),
            "xyz_std": pos.std(axis=0).tolist(),
            "xyz_max_dev": float(np.linalg.norm(pos - pos.mean(axis=0), axis=1).max()),
        })
        quats = [[float(r["base_obj_qx"]), float(r["base_obj_qy"]),
                  float(r["base_obj_qz"]), float(r["base_obj_qw"])] for r in valid]
        _, rmean, rmax = quat_mean_dev(quats)
        out.update({"rot_mean_deg": rmean, "rot_max_deg": rmax})
    else:
        out.update({"xyz_mean": [], "xyz_std": [], "xyz_max_dev": float("nan"),
                    "rot_mean_deg": float("nan"), "rot_max_deg": float("nan")})
    # Switch counts in timestamp order.
    tids = [r.get("track_id", "") for r in rows if r.get("track_id")]
    cids = [r.get("cube_id", "") for r in rows if r.get("cube_id") not in ("", None)]
    meths = [r.get("pose_method", "") for r in rows if r.get("pose_method")]
    out.update({
        "track_switches": sum(1 for a, b in zip(tids, tids[1:]) if a != b),
        "id_switches": sum(1 for a, b in zip(cids, cids[1:]) if a != b),
        "method_switches": sum(1 for a, b in zip(meths, meths[1:]) if a != b),
    })
    return out


def analyze_identity(rows):
    labeled = [r for r in rows if str(r.get("ground_truth_cube_id", "")).strip() not in ("", "0", "-1")]
    n = len(labeled)
    if not n:
        return {"n_labeled": 0, "accuracy": None, "unknown_rate": None,
                "wrong_id_rate": None, "confusion": {}, "note": "fill ground_truth_cube_id first"}
    acc = unk = wrong = 0
    conf = {}
    for r in labeled:
        gt = str(r["ground_truth_cube_id"]).strip()
        pred = str(r["predicted_cube_id"]).strip()
        conf.setdefault(gt, {}).setdefault(pred, 0)
        conf[gt][pred] += 1
        if pred in ("", "0", "?", "unknown"):
            unk += 1
        elif pred == gt:
            acc += 1
        else:
            wrong += 1
    return {"n_labeled": n, "accuracy": acc / n, "unknown_rate": unk / n,
            "wrong_id_rate": wrong / n, "confusion": conf}


def analyze_grasp(path):
    stages = ["DETECTED", "IDENTIFIED", "POSE_VALID", "GRASP_SELECTED",
              "IK_OK", "APPROACH_OK", "GRASP_SUCCESS", "BIN_DROP_SUCCESS"]
    txs = []
    if path.exists():
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        txs.append(json.loads(line))
                    except ValueError:
                        pass
    funnel = {s: 0 for s in stages}
    fail_stage = {}
    for t in txs:
        reached = t.get("reached_stage", t.get("result", ""))
        for s in stages:
            if t.get(s, t.get("reached_stage", "") == s):
                funnel[s] += 1
        fs = t.get("failure_stage", "" if t.get("ok") else "unknown")
        if not t.get("ok", False):
            fail_stage[fs] = fail_stage.get(fs, 0) + 1
    return {"n_trials": len(txs),
            "n_success": sum(1 for t in txs if t.get("BIN_DROP_SUCCESS") or t.get("ok")),
            "funnel": funnel, "fail_stage": fail_stage, "trials": txs}


def pick_example(eye_rows, grasp_trials):
    if grasp_trials:
        t = grasp_trials[0]
        tid = t.get("track_id", "")
        frames = [r for r in eye_rows if r.get("track_id") == tid][:5]
        return {"transaction": t, "frames": frames}
    if eye_rows:
        return {"transaction": {}, "frames": eye_rows[:3]}
    return {"transaction": {}, "frames": []}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in-dir", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args(argv)
    in_dir = Path(args.in_dir)
    eye = read_csv(in_dir / "eye_in_hand_real.csv") if (in_dir / "eye_in_hand_real.csv").exists() else []
    ident = read_csv(in_dir / "identity_real.csv") if (in_dir / "identity_real.csv").exists() else []
    grasp = analyze_grasp(in_dir / "grasp_execution.jsonl")
    summary = {"eye_in_hand": analyze_eye(eye), "identity": analyze_identity(ident),
               "grasp": {k: v for k, v in grasp.items() if k != "trials"}}
    example = pick_example(eye, grasp.get("trials", []))
    out = Path(args.out) if args.out else in_dir / "summary.json"
    out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (in_dir / "example_trace.json").write_text(json.dumps(example, indent=2), encoding="utf-8")
    e, i, g = summary["eye_in_hand"], summary["identity"], summary["grasp"]
    print("=== EYE-IN-HAND (cube static) ===")
    print(f"frames={e['n_frames']} valid={e['n_valid']} tf_fail={e['tf_failures']} "
          f"rejected={e['rejected_frames']}")
    print(f"xyz_std={e['xyz_std']} max_dev={e.get('xyz_max_dev')}")
    print(f"rot mean={e.get('rot_mean_deg')} max={e.get('rot_max_deg')} deg")
    print(f"track_sw={e['track_switches']} id_sw={e['id_switches']} method_sw={e['method_switches']}")
    print("=== IDENTITY ===")
    print(json.dumps(i, indent=2))
    print("=== GRASP ===")
    print(f"trials={g['n_trials']} success={g['n_success']} fail_stage={g['fail_stage']}")
    print(f"wrote {out} + example_trace.json")
    # PASS/FAIL headline for Sec 1: T_base_object stable while camera moves.
    try:
        verdict = "PASS" if (e["xyz_max_dev"] == e["xyz_max_dev"] and e["xyz_max_dev"] < 0.010
                             and e["track_switches"] == 0) else "CHECK"
    except TypeError:
        verdict = "NO-DATA"
    print(f"STABILITY VERDICT: {verdict} (need max_dev<10mm, track_sw=0)")


if __name__ == "__main__":
    main()
