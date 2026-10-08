#!/usr/bin/env python3
"""Mặt nạ "vật không phải cube" trong ảnh camera tay, bằng YOLOE có sẵn của repo (gợi ý bằng chữ).

Cần `ultralytics` (chỉ có trong .venv) và trọng số `ai/models/segmentation/yoloe-11s-seg.pt`; thiếu thì `load()`
ném lỗi và bên gọi bỏ qua bước dựng vật. Đo thật (cốc đỏ, 4 khung): nhận 3/4 khung với mặt nạ gọn, không nhận nhầm
cube. Cube đã có đường riêng (tag / mặt) nên không nằm trong danh sách gợi ý.
"""
from __future__ import annotations

import os

import numpy as np

from cube_vision.registry import repo_root, ros_workspace

PROMPTS = ["cup", "mug", "bottle", "box", "can", "tool", "fruit", "object"]
MIN_CONFIDENCE = 0.25


class ObjectMasks:
    def __init__(self, prompts=PROMPTS):
        from ultralytics import YOLOE
        weights = repo_root() / "ai" / "models" / "segmentation" / "yoloe-11s-seg.pt"
        if not weights.exists():
            raise FileNotFoundError(f"thiếu trọng số {weights}")
        self.prompts = list(prompts)
        here = os.getcwd()
        try:
            os.chdir(ros_workspace())                     # ultralytics tìm mobileclip_blt.ts ở thư mục đang chạy
            self.model = YOLOE(str(weights))
            self.model.set_classes(self.prompts, self.model.get_text_pe(self.prompts))
        finally:
            os.chdir(here)

    def detect(self, frame_bgr) -> list:
        """[(nhãn, độ tin, mặt nạ HxW uint8)] của từng vật trong khung."""
        import cv2
        result = self.model.predict(frame_bgr, conf=MIN_CONFIDENCE, verbose=False)[0]
        found = []
        if result.masks is None:
            return found
        for index, polygon in enumerate(result.masks.xy):
            if len(polygon) < 3:
                continue
            mask = np.zeros(frame_bgr.shape[:2], np.uint8)
            cv2.fillPoly(mask, [np.asarray(polygon, np.int32)], 255)
            found.append((self.prompts[int(result.boxes.cls[index])], float(result.boxes.conf[index]), mask))
        return found
