"""Đánh giá nhận diện mặt rác bằng DINO: độ chính xác, nhầm lẫn, và ngưỡng đề xuất.

Thuần numpy: nhận embedding đã tính (4 góc xoay mỗi ảnh) nên test được không cần model.
Cách tính điểm giống TrashDetector._rank_scores: với mỗi vector tham chiếu lấy cosine lớn nhất
qua các góc xoay, mỗi lớp lấy trung bình 3 vector gần nhất, chọn lớp cao nhất; margin = hở so
với lớp thứ hai.
"""
from __future__ import annotations

from collections import Counter

import numpy as np


def rank(query_turns, matrix, labels, mask=None):
    """(nhãn, điểm, margin) cho một ảnh: query_turns (4, dim), matrix (n, dim), mask: hàng bị bỏ qua."""
    sims = matrix @ np.asarray(query_turns).T            # (n, 4)
    best = sims.max(axis=1)
    if mask is not None:
        best = np.where(mask, -np.inf, best)
    per_class = {}
    for label in sorted(set(labels)):
        rows = np.array([i for i, l in enumerate(labels) if l == label])
        values = np.sort(best[rows])[::-1]
        values = values[np.isfinite(values)]
        if len(values):
            per_class[label] = float(values[:3].mean())
    ranked = sorted(per_class.items(), key=lambda kv: kv[1], reverse=True)
    gap = ranked[0][1] - ranked[1][1] if len(ranked) > 1 else ranked[0][1]
    return ranked[0][0], ranked[0][1], gap


def evaluate(real_turns, real_labels, base_matrix, base_labels):
    """So sánh: chỉ DB gốc  vs  DB gốc + mẫu camera thật (bỏ chính ảnh đó khi chấm, leave-one-out)."""
    real_matrix = np.asarray(real_turns)[:, 0, :]                 # góc 0 làm vector tham chiếu
    all_matrix = np.concatenate([np.asarray(base_matrix), real_matrix])
    all_labels = list(base_labels) + list(real_labels)
    out = {}
    for name, matrix, labels, use_mask in (("base_only", np.asarray(base_matrix), list(base_labels), False),
                                           ("base_plus_real", all_matrix, all_labels, True)):
        rows = []
        for i, (turns, truth) in enumerate(zip(real_turns, real_labels)):
            mask = None
            if use_mask:
                mask = np.zeros(len(labels), bool)
                mask[len(base_labels) + i] = True            # đúng ảnh đang chấm
            label, score, margin = rank(turns, matrix, labels, mask)
            rows.append((truth, label, score, margin))
        out[name] = summarise(rows)
    return out


def summarise(rows):
    correct = [r for r in rows if r[0] == r[1]]
    wrong = [r for r in rows if r[0] != r[1]]
    confusion = Counter((r[0], r[1]) for r in wrong)
    best_threshold, best_value = None, -1.0
    for t in sorted({r[2] for r in rows}):                  # ngưỡng ứng viên = đúng các điểm đã gặp
        accepted = [r for r in rows if r[2] >= t]
        precision = sum(r[0] == r[1] for r in accepted) / len(accepted)
        coverage = len(accepted) / len(rows)
        if precision >= 0.97 and coverage > best_value:     # nhận nhiều nhất mà vẫn >= 97% đúng
            best_threshold, best_value = float(np.floor(t * 1000) / 1000), coverage
    return {"n": len(rows), "accuracy": len(correct) / max(1, len(rows)),
            "mean_score_correct": float(np.mean([r[2] for r in correct])) if correct else None,
            "mean_score_wrong": float(np.mean([r[2] for r in wrong])) if wrong else None,
            "top_confusions": confusion.most_common(5),
            "threshold_for_97pct_precision": best_threshold,
            "coverage_at_that_threshold": None if best_threshold is None else round(best_value, 3)}
