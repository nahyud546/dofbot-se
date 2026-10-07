"""AprilTag tag36h11 cho cube (một cài đặt dùng chung)."""
from __future__ import annotations

import cv2
import numpy as np

_BACKENDS = ("dt_apriltags", "pupil_apriltags")
# pupil_apriltags crashes (segfault) when several Detector objects are created and
# freed in one process, so every TagDetector shares one instance per (backend, threads).
_SHARED = {}


class TagDetector:
    """Phát hiện tag36h11.

    enhance: chạy thêm một lượt trên ảnh CLAHE (nhận diện chặt hơn ở ảnh tối) và
        gộp trùng; ids: chỉ giữ các ID này (None = tất cả); lazy: khởi tạo thư viện
        ở lần detect đầu (tránh tốn thời gian khi không dùng).
    detect() nhận ảnh xám hoặc BGR, trả [{"id","cube_id","center","centroid","corners",
    "quad","margin"}]: center là tâm do thư viện báo, centroid là trung bình 4 góc
    (search-center dùng centroid), corners là ndarray float32 (4,2).
    """

    # Mặc định ở mức lớp để subclass/test dựng bằng __new__ vẫn hoạt động.
    enhance = False
    ids = None
    nthreads = 2
    quiet = True
    backends = _BACKENDS
    detector = None
    backend = None

    def __init__(self, enhance: bool = False, ids=None, nthreads: int = 2,
                 lazy: bool = False, quiet: bool = False, backends=_BACKENDS):
        self.enhance, self.ids = enhance, (tuple(ids) if ids is not None else None)
        self.nthreads, self.quiet, self.backends = nthreads, quiet, tuple(backends)
        self.detector = None
        self.backend = None
        if not lazy:
            self._ensure()

    def _ensure(self):
        if self.detector is not None or self.backend == "unavailable":
            return self.detector
        for name in self.backends:
            try:
                module = __import__(name, fromlist=["Detector"])
            except ImportError:
                continue
            key = (name, self.nthreads)
            if key not in _SHARED:
                _SHARED[key] = module.Detector(families="tag36h11", nthreads=self.nthreads,
                                               quad_decimate=1.0, refine_edges=1)
            self.detector = _SHARED[key]
            self.backend = name
            if not self.quiet:
                print(f"[OK] apriltag backend: {name}")
            return self.detector
        self.backend = "unavailable"
        if not self.quiet:
            print("[WARN] no apriltag lib (dt_apriltags/pupil_apriltags); tag cue disabled")
        return None

    def detect(self, image):
        detector = self._ensure()
        if detector is None or image is None:
            return []
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        passes = [gray]
        if self.enhance:
            passes.append(cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray))
        out = []
        for pic in passes:
            try:
                tags = detector.detect(pic)
            except cv2.error:
                continue
            for tag in tags:
                try:
                    tag_id = int(tag.tag_id)
                except (TypeError, ValueError):
                    continue
                if self.ids is not None and tag_id not in self.ids:
                    continue
                corners = np.asarray(tag.corners, dtype=np.float32).reshape(4, 2)
                if not np.isfinite(corners).all():
                    continue
                hit = {"id": tag_id, "cube_id": tag_id,
                       "center": (float(tag.center[0]), float(tag.center[1])),
                       "corners": corners, "quad": corners.astype(float).tolist(),
                       "centroid": tuple(float(v) for v in corners.astype(float).mean(axis=0)),
                       "margin": float(getattr(tag, "decision_margin", 0.0))}
                dup = next((i for i, old in enumerate(out) if old["id"] == tag_id and
                            np.linalg.norm(np.subtract(old["center"], hit["center"])) < 8), None)
                if dup is None:
                    out.append(hit)
                elif hit["margin"] > out[dup]["margin"]:
                    out[dup] = hit
        return out
