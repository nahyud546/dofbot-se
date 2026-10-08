"""Identifier(profile): gọi lại phần rời rạc theo một cấu hình cố định.

Profile:
    TAG        chỉ AprilTag (nhanh nhất)
    TAG_COLOR  tag trước, rơi về ô màu HSV (search-center)
    FULL       tag + màu + mặt rác DINOv2 + hợp nhất (identify_cube.detect_frame)

Mọi profile trả danh sách CubeDetection cùng định dạng, nên nơi gọi không phải
biết cube được nhận từ mặt nào.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2

from . import color, registry
from .tag import TagDetector

TAG, TAG_COLOR, FULL = "TAG", "TAG_COLOR", "FULL"
PROFILES = (TAG, TAG_COLOR, FULL)


@dataclass
class CubeDetection:
    cube_id: int | None
    center: tuple[float, float] | None     # pixel (x, y) tâm mặt trên/tag
    quad: list | None                      # 4 góc [[x, y], ...]
    source: str                            # apriltag | hsv_single | fused ...
    confidence: float = 1.0
    evidence: dict = field(default_factory=dict)

    def as_candidate(self):
        """Dạng dict mà cube_search_center đang dùng."""
        return {"cube_id": self.cube_id,
                "center": None if self.center is None else [float(self.center[0]), float(self.center[1])],
                "quad": self.quad, "source": self.source}


def _unit_confidence(score):
    """identify_cube báo điểm tag theo decision margin (~80) và rác theo cosine (0-1)."""
    score = float(score or 0.0)
    return min(1.0, score / 100.0) if score > 1.0 else max(0.0, score)


class Identifier:
    def __init__(self, profile: str = TAG_COLOR, hsv_profile: str = "proven",
                 tag: TagDetector | None = None, trash=None):
        if profile not in PROFILES:
            raise ValueError(f"profile phải là một trong {PROFILES}")
        self.profile, self.hsv_profile = profile, hsv_profile
        self.ranges = registry.hsv_ranges(hsv_profile)
        self.tag = tag or TagDetector(enhance=(profile == FULL), ids=registry.CUBES,
                                      nthreads=4 if profile == FULL else 2,
                                      lazy=True, quiet=(profile != FULL))
        self.trash = trash

    def detect(self, frame_bgr, want_id: int | None = None):
        if frame_bgr is None:
            return []
        if self.profile == FULL:
            return self._full(frame_bgr, want_id)
        tags = [t for t in self.tag.detect(frame_bgr)
                if want_id is None or t["cube_id"] == int(want_id)]
        if tags and (self.profile == TAG or len(tags) == 1):
            if self.profile == TAG_COLOR and len(tags) != 1:
                return []
            return [CubeDetection(t["cube_id"], t["centroid"], t["quad"], "apriltag",
                                  min(1.0, t["margin"] / 80.0), {"margin": t["margin"]})
                    for t in tags]
        if self.profile == TAG or tags:
            return []      # TAG: không thấy; TAG_COLOR: nhiều tag -> mơ hồ, không đoán màu
        cands = color.square_candidates(frame_bgr, self.ranges, registry.COLOR_TO_ID, want_id)
        if len(cands) != 1:
            return []
        c = cands[0]
        return [CubeDetection(c["cube_id"], tuple(c["center"]), c["quad"], "hsv_single", 0.6)]

    def _full(self, frame_bgr, want_id):
        from ._paths import ensure
        ensure("vision")
        import identify_cube as legacy
        if self.trash is None:
            from .trash import TrashDetector
            self.trash = TrashDetector()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        verdicts, _ = legacy.identify_frame(frame_bgr, self.tag.detect(gray), self.trash,
                                            annotate=False)
        out = []
        for obs in verdicts:
            d = obs.to_dict()
            if want_id is not None and d["cube_id"] != int(want_id):
                continue
            centre = d["top_center_px"]
            out.append(CubeDetection(d["cube_id"], None if centre is None else tuple(centre),
                                     d["top_quad_px"], "fused:" + str(d["via"]),
                                     _unit_confidence(d["score"]), d))
        return out
