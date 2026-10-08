"""YOLOE open-vocabulary instance segmentation for cubes.

Replaces the closed-set yolov8n-seg COCO weights (person/bicycle/.../toothbrush)
which have no "cube" class and segment whatever COCO happens to fire on.

Two prompt modes, matching the validation plan:
  text    — set_classes(["cube", "toy block", "colored cube"]) then predict.
  visual  — enroll a real cube (refer image + xyxy boxes) once, then every
            frame is segmented with YOLOEVPSegPredictor + stored prompts.

Visual enrollment wins when available (measured: text "cube" alone gives 0
masks on sim frames, visual prompt on the true box gives 1 tight mask).
Text stays as fallback so perception never depends on one prompt type.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

DEFAULT_TEXT_PROMPTS = ["cube", "toy block", "colored cube"]


class YOLOESegmenter:
    """Thin wrapper around ultralytics YOLOE producing ObjectInstance lists."""

    def __init__(self, weights, text_prompts=None, conf=0.5, device="auto"):
        from ultralytics import YOLOE

        self.weights = str(weights)
        self.text_prompts = list(text_prompts) if text_prompts else list(DEFAULT_TEXT_PROMPTS)
        self.conf = float(conf)
        self.device = device
        self.model = YOLOE(self.weights)
        if device != "auto":
            try:
                self.model.to(device)
            except (AttributeError, RuntimeError, ValueError):
                pass
        self._apply_text_prompts()
        self.visual_prompts = None  # {"bboxes": Nx4 float, "cls": N int}

    def _apply_text_prompts(self):
        # YOLOE only forbids a bare " " list element; multi-word prompts
        # like "toy block" are allowed and tested verbatim.
        safe = [p for p in self.text_prompts if p]
        if not safe:
            raise ValueError("need at least one text prompt")
        self.active_prompts = safe
        self.model.set_classes(safe, self.model.get_text_pe(safe))

    def enroll_visual(self, refer_image_bgr, bboxes_xyxy, class_ids=None):
        """Store visual prompts from a REAL cube frame.

        refer_image_bgr: BGR frame showing the cube(s).
        bboxes_xyxy: iterable of [x1,y1,x2,y2] in refer-image pixels.
        class_ids: one int per box (all 0 = single "cube" class).
        """
        boxes = np.asarray(list(bboxes_xyxy), dtype=np.float64).reshape(-1, 4)
        if boxes.shape[0] == 0:
            raise ValueError("visual enrollment needs at least one box")
        h, w = np.asarray(refer_image_bgr).shape[:2]
        if not np.all((boxes[:, [0, 2]] >= 0) & (boxes[:, [0, 2]] <= w)
                      ) or not np.all((boxes[:, [1, 3]] >= 0) & (boxes[:, [1, 3]] <= h)):
            raise ValueError("visual prompt box outside refer image")
        cls = (np.zeros(len(boxes), dtype=int) if class_ids is None
               else np.asarray(list(class_ids), dtype=int).reshape(-1))
        if cls.shape[0] != boxes.shape[0]:
            raise ValueError("cls must align with bboxes")
        self.visual_prompts = {"bboxes": boxes.astype(np.float32), "cls": cls}

    def save_enrollment(self, path):
        if self.visual_prompts is None:
            raise ValueError("nothing enrolled")
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({
            "bboxes": self.visual_prompts["bboxes"].tolist(),
            "cls": self.visual_prompts["cls"].tolist(),
            "text_prompts": self.text_prompts,
        }), encoding="utf-8")
        return str(path)

    def load_enrollment(self, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.visual_prompts = {
            "bboxes": np.asarray(data["bboxes"], dtype=np.float32),
            "cls": np.asarray(data["cls"], dtype=int),
        }
        if data.get("text_prompts"):
            self.text_prompts = list(data["text_prompts"])
            self._apply_text_prompts()
        return self.visual_prompts

    def _to_instances(self, result, frame_shape):
        from cap_vision.object_pipeline import ObjectInstance

        if result.masks is None:
            return []
        height, width = frame_shape[:2]
        out = []
        boxes = result.boxes
        for i, mask in enumerate(result.masks.data.cpu().numpy()):
            try:
                x1, y1, x2, y2 = (int(round(v)) for v in boxes.xyxy[i].cpu().numpy())
                conf = float(boxes.conf[i].cpu().numpy())
            except (AttributeError, IndexError, TypeError, ValueError):
                continue
            if x2 <= x1 or y2 <= y1:
                continue
            if mask.shape != (height, width):
                mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            binary = mask > 0.5
            if np.count_nonzero(binary) < 100:
                continue
            out.append(ObjectInstance((x1, y1, x2, y2), binary, conf))
        return out

    def segment(self, frame_bgr, conf=None):
        """Segment one frame. Visual prompts first, text fallback."""
        frame = np.ascontiguousarray(frame_bgr)
        threshold = self.conf if conf is None else float(conf)
        if self.visual_prompts is not None:
            try:
                from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor

                result = self.model.predict(
                    frame, visual_prompts=self.visual_prompts,
                    predictor=YOLOEVPSegPredictor,
                    conf=threshold, verbose=False)[0]
                found = self._to_instances(result, frame.shape)
                if found:
                    return found
            except (RuntimeError, ValueError, AttributeError):
                pass
        result = self.model.predict(frame, conf=threshold, verbose=False)[0]
        return self._to_instances(result, frame.shape)
