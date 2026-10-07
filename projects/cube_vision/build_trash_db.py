#!/usr/bin/env python3
"""Sinh thêm embedding "kiểu camera" cho DB mặt rác từ 16 ảnh tham chiếu (không cần camera/tay).

DB gốc (process_and_embed.py) chỉ gồm ảnh sạch có xoay/méo/đổi sáng/mờ; mặt in trên cube nhìn qua
camera tay chỉ rộng ~100-130 px nên còn bị nhoè pixel, nhiễu, lệch màu. Thiếu các biến dạng đó DB
nhận kém (đo giả lập: ~94% ảnh sạch nhưng ~40% khi thu nhỏ + mờ + nhiễu). Script này thêm các biến
dạng đó và ghi real/vector_database_dinov2_vits14_camaug.pt (TrashDetector tự nạp cùng DB gốc).
Chạy lại thoải mái (ghi đè file camaug); không đụng DB gốc hay processed/.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cube_vision import registry  # noqa: E402
from cube_vision.trash import TrashDetector, save_real_embeddings  # noqa: E402


def camera_like(face_224, rng, severity: float = 1.0):
    """Một biến thể giống ảnh mặt rác nhìn qua camera tay (đầu vào BGR 224x224)."""
    img = face_224
    m = severity * 0.10 * 224
    src = np.float32([[0, 0], [223, 0], [223, 223], [0, 223]])
    dst = src + rng.uniform(-m, m, (4, 2)).astype(np.float32)
    img = cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (224, 224),
                              borderMode=cv2.BORDER_REFLECT)
    side = int(rng.uniform(60, 130))                       # mặt nhỏ -> ít điểm ảnh, rồi phóng lại
    img = cv2.resize(cv2.resize(img, (side, side), interpolation=cv2.INTER_AREA), (224, 224))
    img = cv2.GaussianBlur(img, (0, 0), rng.uniform(0.3, 1.6) * severity)
    img = np.clip(img.astype(np.float32) * rng.uniform(0.7, 1.25) + rng.uniform(-25, 25), 0, 255)
    img = np.clip(img * rng.uniform(0.88, 1.12, 3), 0, 255).astype(np.uint8)
    noise = rng.normal(0, rng.uniform(2, 8) * severity, img.shape)
    return np.clip(img + noise, 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--per-class", type=int, default=48)
    ap.add_argument("--seed", type=int, default=1234)
    args = ap.parse_args()
    detector = TrashDetector()
    if detector.model is None:
        raise SystemExit("DINO chưa sẵn sàng (torch/model)")
    rng = np.random.default_rng(args.seed)
    base_dir = registry.repo_root() / "ai/datasets/trash-images/processed"
    items = []
    for name in sorted(registry.TRASH_TO_CUBE):
        folder = next((p for p in base_dir.glob("*") if p.name.lower() == name), None)
        base = None if folder is None else cv2.imread(str(folder / f"{folder.name}_base.jpg"))
        if base is None:
            print(f"[BUILD] thiếu ảnh nền cho {name}; bỏ qua")
            continue
        base = cv2.resize(base, (224, 224))
        faces = [camera_like(base, rng) for _ in range(args.per_class)]
        embeddings = detector.embed_turns(faces)[:, 0, :]
        items += [{"embedding": e, "label": name} for e in embeddings]
        print(f"[BUILD] {name}: {len(faces)} biến thể")
    print("[BUILD] đã lưu:", save_real_embeddings(items, detector.model_name, kind="camaug"))


if __name__ == "__main__":
    main()
