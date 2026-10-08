#!/usr/bin/env python3
"""Đánh giá nhận diện mặt rác trên các crop thật đã thu (ai/datasets/trash-images/real/).

So sánh chỉ DB gốc với DB gốc + mẫu camera thật (leave-one-out: ảnh đang chấm bị loại khỏi DB),
in độ chính xác, nhầm lẫn nhiều nhất và ngưỡng cosine đề xuất. Không cần camera/tay.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cube_vision import registry, trash_eval  # noqa: E402
from cube_vision.trash import TrashDetector  # noqa: E402


def main():
    real_dir = registry.repo_root() / "ai/datasets/trash-images/real"
    crops, labels = [], []
    for folder in sorted(p for p in real_dir.glob("*") if p.is_dir()):
        if folder.name not in registry.TRASH_TO_CUBE:
            continue
        for path in sorted(folder.glob("*.png")):
            image = cv2.imread(str(path))
            if image is not None:
                crops.append(image)
                labels.append(folder.name)
    if len(crops) < 8:
        raise SystemExit(f"cần >= 8 crop trong {real_dir} (có {len(crops)}); chạy collect_trash_faces.py")
    detector = TrashDetector()
    if detector.model is None:
        raise SystemExit("DINO chưa sẵn sàng")
    # Ma trận DB gốc = các hàng chưa gồm mẫu thật (chúng ta tự thêm theo từng ảnh khi chấm).
    from cube_vision.trash import load_real_embeddings
    n_real = len(load_real_embeddings(detector.model_name, detector.dim)[0])
    base_matrix = detector.matrix.numpy()[: len(detector.labels) - n_real]
    base_labels = detector.labels[: len(detector.labels) - n_real]
    turns = detector.embed_turns(crops)
    result = trash_eval.evaluate(turns, labels, base_matrix, base_labels)
    per_class = {}
    for name in sorted(set(labels)):
        per_class[name] = labels.count(name)
    print(json.dumps({"crops": len(crops), "per_class": per_class, **result}, indent=2, ensure_ascii=False))
    gain = result["base_plus_real"]["accuracy"] - result["base_only"]["accuracy"]
    print(f"\n[EVAL] độ chính xác: chỉ DB gốc {result['base_only']['accuracy']:.0%} -> "
          f"gốc + camera thật {result['base_plus_real']['accuracy']:.0%} ({gain:+.0%})")
    print("[EVAL] ngưỡng cosine đề xuất (>=97% đúng):",
          result["base_plus_real"]["threshold_for_97pct_precision"],
          "(hiện tại TrashDetector.DEFAULT_THRESH = 0.40)")


if __name__ == "__main__":
    main()
