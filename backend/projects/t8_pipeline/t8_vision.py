"""YOLO detection for a single T8 camera frame; no robot motion here."""

from pathlib import Path


DEFAULT_MODEL = (Path(__file__).resolve().parents[2] / "workspaces" / "dofbot_ws" / "src" /
                 "dofbot_yolov11" / "dofbot_yolov11" / "best.onnx")
LABEL_CLASSES = {"xuong_ca": {"fish_bone"},
                 "giay_ve_sinh": {"toilet_paper"},
                 "pin": {"used_batteries"}}
# Color requests use HSV/YOLO; ID requests require a real tag36h11 detection
# in two frames and are never inferred from a generic rectangle.
CUBE_ID_LABELS = {f"cube_{i}" for i in (1, 2, 3, 4)}
CUBE_LABELS = {"xuong_ca", "giay_ve_sinh", "cube", "khoi_do", "khoi_xanh",
               "khoi_vang", "khoi_xanh_duong"} | CUBE_ID_LABELS
# Dai mau o vung hong (ngay duoi vien mat tren) phai dat ti le nay.
# Do 3 capture cua so [y2-0.5h, y2+0.15h]: vang that 0.42-0.47, do lac
# (muc in tren mat trang) 0.08-0.13, xanh ~0.01. Nhan mau thang với
# cach biet >= SIDE_BAND_MARGIN so voi mau dung nhi (muc in khong lat
# duoc mat hong that; 2 mau sat nhau -> mo ho -> bo).
SIDE_BAND_FRAC = 0.20
SIDE_BAND_MARGIN = 0.10
# Profile "proven" của cube_vision.registry (xanh dương S>=150 để loại bóng xanh
# nhạt của bàn): cùng giá trị với bản cũ nằm ở đây.
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
from cube_vision import registry as _registry  # noqa: E402
from cube_vision.tag import TagDetector as _SharedTagDetector  # noqa: E402

HSV_RANGES = _registry.hsv_ranges("proven")


class VisionDetector:
    def __init__(self, model_path=DEFAULT_MODEL, confidence=0.5, model=None):
        self.model_path = Path(model_path)
        self.confidence = confidence
        self._model = model
        self._last_frame = None
        self._last_results = None
        self._tag_detector = None

    def _tag_candidates(self, label, frame):
        """Detect a requested AprilTag cube ID in a legacy RGB frame."""
        if label not in CUBE_ID_LABELS or frame is None:
            return []
        try:
            import cv2
            import numpy as np
            if self._tag_detector is None:
                # Thư viện tag dùng chung (một Detector cho cả tiến trình).
                self._tag_detector = _SharedTagDetector(
                    nthreads=2, lazy=True, quiet=True,
                    backends=("pupil_apriltags", "dt_apriltags"))._ensure()
                if self._tag_detector is None:
                    return []
            rgb = np.asarray(frame.convert("RGB"))
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            wanted = int(label.rsplit("_", 1)[1])
            found = []
            for tag in self._tag_detector.detect(gray):
                if int(tag.tag_id) != wanted:
                    continue
                corners = np.asarray(tag.corners, float).reshape(4, 2)
                x1, y1 = np.floor(corners.min(axis=0)).astype(int)
                x2, y2 = np.ceil(corners.max(axis=0)).astype(int)
                center = corners.mean(axis=0)
                found.append({"label": label, "box": [int(x1), int(y1),
                                                       int(x2), int(y2)],
                              "corners": corners.tolist(),
                              "center": center.tolist(), "source": "apriltag",
                              "conf": float(min(0.99, max(0.5,
                                  getattr(tag, "decision_margin", 40.0) / 80.0))),
                              "geometry_verified": True})
            return found
        except (ImportError, RuntimeError, ValueError, cv2.error):
            return []

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
        # Mask mau sach (loc theo hue) nen cho phep epsilon lon dan khi
        # 0.04 con thua 1 dinh lech (bong/ghe mau): van uu tien 0.04 de
        # khong doi hanh vi cu. Canny thay moi do hoa in (giong contour)
        # nen giu chat 0.04 de tranh quad ao tu hoa van.
        epsilons = (0.04, 0.06, 0.08) if use_color else (0.04,)
        found = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not 1000 <= area <= 30000:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = None
            for _eps in epsilons:
                _ap = cv2.approxPolyDP(cnt, _eps * peri, True)
                if len(_ap) != 4 or not cv2.isContourConvex(_ap):
                    continue
                _x, _y, _w, _h = cv2.boundingRect(_ap)
                if (_x < 10 or _y < 10 or _x + _w > 635 or
                        not 0.65 <= _w / _h <= 1.55 or
                        area / (_w * _h) < 0.60):
                    continue
                approx = _ap
                x, y, w, h = _x, _y, _w, _h
                break
            if approx is None:
                continue
            # Khong chan vien duoi: camera cheo nen mep ban gan (workspace
            # hop le) nam sat day frame; cube cham day van lay duoc tam.
            # wh/fill/convex/area van loai cube bi cat xeo nhieu.
            corners = approx.reshape(4, 2).tolist()
            found.append({"label": label, "box": [x, y, x + w, y + h],
                          "corners": corners, "center": [x + w / 2, y + h / 2],
                          "source": "hsv" if use_color else "contour",
                          "geometry_verified": True})
        return found

    def get_cube_candidates(self, label, frame, confirm_frame=None, exclude=None):
        """Use semantics first; require two matching frames for geometric fallback.

        Cube ID requires the ROS semantic geometry model; a contour has no ID.
        """
        base = label
        if label not in CUBE_LABELS or frame is None:
            return []
        def filter_excluded(candidates):
            if exclude is None:
                return candidates
            ex, ey = exclude["center"]
            return [candidate for candidate in candidates if
                    abs(candidate["center"][0] - ex) > 40 or
                    abs(candidate["center"][1] - ey) > 40]
        if label in CUBE_ID_LABELS:
            first = filter_excluded(self._tag_candidates(label, frame))
            if callable(confirm_frame):
                confirm_frame = confirm_frame()
            second = filter_excluded(self._tag_candidates(label, confirm_frame)) \
                if confirm_frame is not None else []
            if len(first) != 1 or len(second) != 1:
                return []
            a, b = first[0], second[0]
            if (abs(a["center"][0] - b["center"][0]) > 8 or
                    abs(a["center"][1] - b["center"][1]) > 8):
                return []
            return [b]
        try:
            yolo = self.get_detections(base, frame)
        except (FileNotFoundError, ImportError, RuntimeError):
            yolo = []
        for det in yolo:
            x1, y1, x2, y2 = det["box"]
            det["center"] = [(x1 + x2) / 2, (y1 + y2) / 2]
            det["source"] = "yolo"
            det["label"] = label
        yolo = filter_excluded(yolo)
        if yolo:
            return yolo
        if base not in HSV_RANGES and base != "cube":
            return []  # A contour does not establish a fishbone/trash identity.
        if callable(confirm_frame):
            confirm_frame = confirm_frame()
        if confirm_frame is None:
            return []
        for use_color in (True,) if base in HSV_RANGES else (False,):
            first = filter_excluded(self._rectangles(frame, base, use_color))
            second = filter_excluded(self._rectangles(confirm_frame, base, use_color))
            if len(first) > 1 or len(second) > 1:
                return []
            if len(first) == len(second) == 1:
                a, b = first[0], second[0]
                if (abs(a["center"][0] - b["center"][0]) <= 12 and
                    abs(a["center"][1] - b["center"][1]) <= 12 and
                    abs((a["box"][2] - a["box"][0]) -
                        (b["box"][2] - b["box"][0])) <= 12):
                    b = dict(b)
                    b["label"] = label
                    return [b]
            # Cube hoa tiet (mat tren trang + mat hong co mau): mat mau truc
            # tiep khong dat (mat hong wide) thi ghe mat trang voi dai mau
            # nam ngay duoi no. Chi chay khi direct chang thay gi (khong ghi
            # de len truong hop direct mo ho 2+ ung vien).
            if base in HSV_RANGES and not first and not second:
                return self._side_associated(base, frame, confirm_frame,
                                             filter_excluded)
        return []

    @staticmethod
    def _side_associated(label, frame, confirm_frame, filter_excluded):
        """Nhan cube qua mat tren (trang) + dai mau o vung hong.

        Cua so [y2-0.5h, y2+0.15h] om mat hong that (nam sat vien duoi mat
        tren) va loai muc in tren mat + cube hang xom ben duoi. Nhan mau
        thang voi margin (frac >= SIDE_BAND_FRAC va hon mau nhi
        >= SIDE_BAND_MARGIN); muc in do tren mat trang cua cube vang khong
        duoc keo thanh lien ket do ao, 2 mau sat nhau thi bo. Tra
        [candidate] dung 1 mat tren on dinh 2 frame; [] neu khong/nhieu.
        Diem gap lay tam mat tren nhu detector mat mau truc tiep.
        """
        import cv2
        import numpy as np
        if label not in HSV_RANGES or frame is None or confirm_frame is None:
            return []

        def side_window_fracs(frm):
            try:
                rgb = np.asarray(frm.convert("RGB"))
            except (ValueError, TypeError):
                return []
            if rgb.shape[:2] != (480, 640):
                return []
            hsv = cv2.cvtColor(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                               cv2.COLOR_BGR2HSV)
            masks = {}
            for name, ranges in HSV_RANGES.items():
                band = np.zeros((480, 640), dtype=np.uint8)
                for lower, upper in ranges:
                    band |= cv2.inRange(hsv, np.array(lower), np.array(upper))
                masks[name] = band
            scored = []
            for top in filter_excluded(VisionDetector._rectangles(
                    frm, "cube", False)):
                x1, y1, x2, y2 = top["box"]
                r0 = max(0, int(y2 - 0.5 * (y2 - y1)))
                r1 = min(480, int(y2 + 0.15 * (y2 - y1)))
                c0, c1 = max(0, x1), min(640, x2)
                if r1 <= r0 or c1 <= c0:
                    continue
                fracs = {name: float(masks[name][r0:r1, c0:c1].mean()) / 255.0
                         for name in masks}
                scored.append((top, fracs))
            return scored

        def associated(frm):
            out = []
            for top, fracs in side_window_fracs(frm):
                ranked = sorted(fracs.items(), key=lambda kv: kv[1],
                                reverse=True)
                best, best_frac = ranked[0]
                runner = ranked[1][1] if len(ranked) > 1 else 0.0
                if (best != label or best_frac < SIDE_BAND_FRAC
                        or best_frac - runner < SIDE_BAND_MARGIN):
                    continue
                det = dict(top)
                det["label"] = label
                det["source"] = "hsv_side"
                det["geometry_verified"] = True
                out.append(det)
            return out

        first, second = associated(frame), associated(confirm_frame)
        if len(first) != 1 or len(second) != 1:
            return []
        a, b = first[0], second[0]
        if (abs(a["center"][0] - b["center"][0]) <= 12 and
                abs(a["center"][1] - b["center"][1]) <= 12 and
                abs((a["box"][2] - a["box"][0]) -
                    (b["box"][2] - b["box"][0])) <= 12):
            return [b]
        return []
