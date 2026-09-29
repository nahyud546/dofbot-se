"""YOLO detection for a single T8 camera frame; no robot motion here."""

from pathlib import Path


DEFAULT_MODEL = (Path(__file__).resolve().parents[1] / "dofbot_ws" / "src" /
                 "dofbot_yolov11" / "dofbot_yolov11" / "best.onnx")
LABEL_CLASSES = {"xuong_ca": {"fish_bone"},
                 "giay_ve_sinh": {"toilet_paper"},
                 "pin": {"used_batteries"}}
CUBE_LABELS = {"xuong_ca", "giay_ve_sinh", "cube", "khoi_do", "khoi_xanh",
               "khoi_vang", "khoi_xanh_duong"}
HSV_RANGES = {
    "khoi_do": [((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (179, 255, 255))],
    "khoi_xanh": [((35, 70, 40), (85, 255, 255))],
    "khoi_xanh_duong": [((90, 70, 40), (130, 255, 255))],
    "khoi_vang": [((20, 70, 50), (35, 255, 255))],
}


class VisionDetector:
    def __init__(self, model_path=DEFAULT_MODEL, confidence=0.5, model=None):
        self.model_path = Path(model_path)
        self.confidence = confidence
        self._model = model
        self._last_frame = None
        self._last_results = None

    def get_detections(self, label, frame):
        if frame is None:
            return []
        wanted = LABEL_CLASSES.get(label)
        if not wanted:
            return []
        if not self.model_path.is_file() and self._model is None:
            raise FileNotFoundError(f"Không có model YOLO: {self.model_path}")
        if self._model is None:
            from ultralytics import YOLO
            self._model = YOLO(str(self.model_path), task="detect")
        import numpy as np
        if frame is self._last_frame and self._last_results is not None:
            results = self._last_results
        else:
            results = self._model.predict(source=np.asarray(frame),
                                          conf=self.confidence, verbose=False)
            self._last_frame, self._last_results = frame, results
        detections = []
        for result in results:
            for box in result.boxes:
                name = str(result.names[int(box.cls.item())]).lower()
                if name not in wanted:
                    continue
                detections.append({"label": label,
                                   "class": name,
                                   "box": [int(x) for x in box.xyxy[0].tolist()],
                                   "conf": float(box.conf.item())})
        return detections

    @staticmethod
    def _rectangles(frame, label, use_color):
        """Return cube top-face quads; geometry does not establish identity."""
        import cv2
        import numpy as np
        rgb = np.asarray(frame.convert("RGB"))
        if rgb.shape[:2] != (480, 640):
            return []
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if use_color and label in HSV_RANGES:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = np.zeros((480, 640), dtype=np.uint8)
            for lower, upper in HSV_RANGES[label]:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((5, 5), np.uint8))
            edges = mask
        else:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 20, 80)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                     np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not 1000 <= area <= 30000:
                continue
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            x, y, w, h = cv2.boundingRect(approx)
            if (x < 80 or y < 60 or x + w > 560 or y + h > 420 or
                    not 0.65 <= w / h <= 1.55 or
                    area / (w * h) < 0.60):
                continue
            corners = approx.reshape(4, 2).tolist()
            found.append({"label": label, "box": [x, y, x + w, y + h],
                          "corners": corners, "center": [x + w / 2, y + h / 2],
                          "source": "hsv" if use_color else "contour",
                          "geometry_verified": True})
        return found

    def get_cube_candidates(self, label, frame, confirm_frame=None, exclude=None):
        """Use semantics first; require two matching frames for geometric fallback."""
        if label not in CUBE_LABELS or frame is None:
            return []
        def filter_excluded(candidates):
            if exclude is None:
                return candidates
            ex, ey = exclude["center"]
            return [candidate for candidate in candidates if
                    abs(candidate["center"][0] - ex) > 40 or
                    abs(candidate["center"][1] - ey) > 40]
        try:
            yolo = self.get_detections(label, frame)
        except (FileNotFoundError, ImportError, RuntimeError):
            yolo = []
        for det in yolo:
            x1, y1, x2, y2 = det["box"]
            det["center"] = [(x1 + x2) / 2, (y1 + y2) / 2]
            det["source"] = "yolo"
        yolo = filter_excluded(yolo)
        if yolo:
            return yolo
        if callable(confirm_frame):
            confirm_frame = confirm_frame()
        if confirm_frame is None:
            return []
        for use_color in (True, False) if label in HSV_RANGES else (False,):
            first = filter_excluded(self._rectangles(frame, label, use_color))
            second = filter_excluded(self._rectangles(confirm_frame, label, use_color))
            if len(first) > 1 or len(second) > 1:
                return []
            if len(first) == len(second) == 1:
                a, b = first[0], second[0]
                if (abs(a["center"][0] - b["center"][0]) <= 12 and
                    abs(a["center"][1] - b["center"][1]) <= 12 and
                    abs((a["box"][2] - a["box"][0]) -
                        (b["box"][2] - b["box"][0])) <= 12):
                    return [b]
        return []
