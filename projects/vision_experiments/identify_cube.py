#!/usr/bin/env python3
"""Identify a cube from its visible faces and locate its upper surface.

Each physical cube has 6 faces: 1 color face, 1 AprilTag face, 4 trash faces.
Any visible face can resolve to the same canonical cube ID (1..4).

Cube registry (matches grasp.py tag->bin map and the T6 YOLO waste groups):
    ID 1 "blue"   : HSV blue        + tag 1 + {Newspaper, Zip_top_can, Book, Old_school_bag}
    ID 2 "green"  : HSV green       + tag 2 + {Fish_bone, Egg_shell, Apple_core, Watermelon_rind}
    ID 3 "red"    : HSV red         + tag 3 + {Syringe, Expired_cosmetics, Used_batteries, Expired_tablets}
    ID 4 "yellow" : HSV yellow      + tag 4 + {Toilet_paper, Peach_pit, Cigarette_butts, Disposable_chopsticks}

Visible faces vote on the cube ID; the physical upper-face boundary provides
the image-space pick center. Conflicting IDs or uncertain geometry abstain.

Run:
    source scripts/setup/setup_env.sh
    .venv/bin/python projects/vision_experiments/identify_cube.py [--dry-run] [--camera 0]
    .venv/bin/python projects/vision_experiments/identify_cube.py --synthetic blue  # no camera needed
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

# OpenCV's bundled Qt build can point at an empty font directory.
if "QT_QPA_FONTDIR" not in os.environ and \
        Path("/usr/share/fonts/truetype/dejavu").is_dir():
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"

import cv2
import numpy as np

# --------------------------------------------------------------------------
# Registry
# --------------------------------------------------------------------------

# HSV ranges copied from projects/t8_pipeline/t8_vision.py (single source
# of truth for T8 color labels; duplicated here so this script stays
# runnable without PYTHONPATH setup).
HSV_RANGES = {
    "khoi_do": [((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (179, 255, 255))],
    "khoi_xanh": [((35, 70, 40), (85, 255, 255))],
    "khoi_xanh_duong": [((90, 70, 40), (130, 255, 255))],
    "khoi_vang": [((20, 70, 50), (35, 255, 255))],
}

CUBES = {
    1: {"name": "blue",
        "color": "khoi_xanh_duong",
        "tag_id": 1,
        "trash": {"newspaper", "zip_top_can", "book", "old_school_bag"}},
    2: {"name": "green",
        "color": "khoi_xanh",
        "tag_id": 2,
        "trash": {"fish_bone", "egg_shell", "apple_core", "watermelon_rind"}},
    3: {"name": "red",
        "color": "khoi_do",
        "tag_id": 3,
        "trash": {"syringe", "expired_cosmetics", "used_batteries",
                  "expired_tablets"}},
    4: {"name": "yellow",
        "color": "khoi_vang",
        "tag_id": 4,
        "trash": {"toilet_paper", "peach_pit", "cigarette_butts",
                  "disposable_chopsticks"}},
}

TAG_TO_CUBE = {spec["tag_id"]: cid for cid, spec in CUBES.items()}
TRASH_TO_CUBE = {cls: cid for cid, spec in CUBES.items()
                 for cls in spec["trash"]}
COLOR_TO_CUBE = {spec["color"]: cid for cid, spec in CUBES.items()}

OBSERVE_JOINTS = [90.0, 125.0, 0.0, 0.0, 90.0, 25.0]  # verified look pose
FRAME_SIZE = (640, 480)

# Contour geometry tuned from live 640x480 camera captures.
# Keep small enough faces for ID-only diagnostics, but require a larger and
# less clipped top before publishing a pick center.
CONTOUR_AREA_MIN_PX2 = 320
CONTOUR_BOX_SIDE_MIN_PX = 16
CONTOUR_AREA_BOX_RATIO_MIN = 0.16
CONTOUR_BOX_WIDTH_MAX_FRAC = 0.56
CONTOUR_BOX_HEIGHT_MAX_FRAC = 0.56
GRASP_TOP_AREA_MIN_PX2 = 750
GRASP_TOP_BOX_SIDE_MIN_PX = 24
BELOW_FACE_SUPPORT_MIN = 0.03
DISPLAY_INVALID_SCORE_MIN = 0.38
INVALID_TRASH_ID_SCORE_MIN = 0.55
COLOR_COMPONENT_AREA_MIN_PX2 = 200
COLOR_COMPONENT_BOX_SIDE_MIN_PX = 12


def repo_root() -> Path:
    env = os.environ.get("ROBOT_ARM_ROOT")
    if env and Path(env).exists():
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "workspaces").exists():
            return parent
    return Path("/home/jloy/Desktop/robot-arm")


# --------------------------------------------------------------------------
# Fusion (pure logic -> unit-testable without camera/hardware)
# --------------------------------------------------------------------------

@dataclass
class FaceCues:
    """One object region, up to three independent cues."""
    tag_id: int | None = None
    trash_class: str | None = None
    color_label: str | None = None


@dataclass
class CubeVerdict:
    cube_id: int | None
    name: str
    via: str                      # which cue(s) decided, e.g. "apriltag", "trash+color"
    conflict: str = ""            # non-empty when cues disagreed
    candidates: dict = field(default_factory=dict)
    label: str = ""
    score: float = 0.0
    reason: str = ""
    trash_top: list = field(default_factory=list)
    trash_rotation_deg: int | None = None


def fuse_cues(cues: FaceCues) -> CubeVerdict:
    """Map any visible face to its canonical cube ID.

    Priority on conflict: apriltag > trash > color (the printed sticker
    is unambiguous, the network can misfire, lighting can fool HSV).
    """
    votes: dict[str, int] = {}
    if cues.tag_id in TAG_TO_CUBE:
        votes[f"tag:{cues.tag_id}"] = TAG_TO_CUBE[cues.tag_id]
    if cues.trash_class and cues.trash_class.lower() in TRASH_TO_CUBE:
        votes[f"trash:{cues.trash_class.lower()}"] = \
            TRASH_TO_CUBE[cues.trash_class.lower()]
    if cues.color_label in COLOR_TO_CUBE:
        votes[f"color:{cues.color_label}"] = COLOR_TO_CUBE[cues.color_label]

    if not votes:
        return CubeVerdict(None, "unknown", "none", candidates={})

    # A cube ID is a label, never a ranking or a confidence score.
    ordered = sorted(votes, key=lambda key: {"tag": 0, "trash": 1, "color": 2}[key.split(":")[0]])
    winner = votes[ordered[0]]
    agreed = sorted(k for k, v in votes.items() if v == winner)
    disagreed = sorted(k for k, v in votes.items() if v != winner)
    conflict = ""
    if disagreed:
        conflict = (f"cues disagree (used {agreed}, "
                    f"ignored {disagreed})")
    # Deterministic priority for the `via` label: tag first.
    order = {"tag": 0, "trash": 1, "color": 2}
    agreed.sort(key=lambda k: order[k.split(":")[0]])
    main = agreed[0].split(":")[0]
    via = main if len(agreed) == 1 else "+".join(
        sorted({k.split(":")[0] for k in agreed},
               key=lambda k: order[k]))
    return CubeVerdict(winner, CUBES[winner]["name"], via,
                       conflict=conflict,
                       candidates={k: CUBES[v]["name"]
                                   for k, v in votes.items()})


# --------------------------------------------------------------------------
# Detectors (thin wrappers; each degrades gracefully when unavailable)
# --------------------------------------------------------------------------

class TagDetector:
    def __init__(self):
        self.detector = None
        try:
            from dt_apriltags import Detector
            backend = "dt_apriltags"
        except ImportError:
            try:
                from pupil_apriltags import Detector
                backend = "pupil_apriltags"
            except ImportError:
                print("[WARN] no apriltag lib (dt_apriltags/pupil_apriltags); "
                      "tag cue disabled")
                return
        self.detector = Detector(families="tag36h11", nthreads=4,
                                 quad_decimate=1.0, refine_edges=1)
        print(f"[OK] apriltag backend: {backend}")

    def detect(self, gray):
        if self.detector is None:
            return []
        out = []
        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        for image in (gray, enhanced):
            for tag in self.detector.detect(image):
                candidate = {"id": int(tag.tag_id),
                             "center": (float(tag.center[0]),
                                        float(tag.center[1])),
                             "corners": np.asarray(tag.corners, dtype=np.float32),
                             "margin": float(tag.decision_margin)}
                duplicate = next((i for i, old in enumerate(out)
                                  if old["id"] == candidate["id"] and
                                  np.linalg.norm(np.subtract(old["center"],
                                                             candidate["center"])) < 8), None)
                if duplicate is None:
                    out.append(candidate)
                elif candidate["margin"] > out[duplicate]["margin"]:
                    out[duplicate] = candidate
        return out


class TrashDetector:
    """DINOv2 embedding match against the trash vector database.

    Same space as ai/datasets/trash-images/process_and_embed.py:
    Rectified 224x224 face -> ImageNet normalize -> matching DINOv2 model.
    Cosine similarity over 256 stored vectors (16 classes x 16).
    """

    DB_PATH_TMPL = __file__.split("projects")[0] + "ai/datasets/trash-images/processed/vector_database_{}.pt"
    IMG_SIZE = 224
    # Provisional until independent images from /dev/video2 are available.
    DEFAULT_THRESH = 0.40
    DEFAULT_MARGIN = 0.0
    DIMS = {
        "dinov2_vits14": 384,
        "dinov2_vitb14": 768,
        "dinov2_vitl14": 1024,
        "dinov2_vitg14": 1536,
    }

    def __init__(self, thresh=DEFAULT_THRESH, device="auto",
                 margin=DEFAULT_MARGIN, model_name="dinov2_vits14"):
        self.model = None
        self.labels: list[str] = []
        self.matrix = None  # (256, dim) L2-normalized, torch cpu tensor
        self.thresh = thresh
        self.margin = margin
        self.model_name = model_name
        self.dim = self.DIMS.get(model_name, 384)
        self.last_result = {}
        self._torch = None
        self.device = None
        try:
            import torch
        except ImportError:
            print("[WARN] torch missing; trash cue disabled")
            return
        self._torch = torch
        db_path = repo_root() / self.DB_PATH_TMPL.format(model_name)
        if not db_path.is_file():
            # fallback to legacy db
            fallback = repo_root() / "ai/datasets/trash-images/processed/vector_database.pt"
            if fallback.is_file():
                db_path = fallback
            else:
                print(f"[WARN] vector DB not found at {db_path}; "
                      "trash cue disabled")
                return
        try:
            db = torch.load(str(db_path), map_location="cpu",
                            weights_only=False)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] cannot load vector DB: {exc}; trash cue disabled")
            return
        embs, labels = [], []
        for item in db:
            emb = np.asarray(item["embedding"], dtype=np.float32)
            if emb.shape != (self.dim,):
                print(f"[WARN] wrong DINO embedding shape {emb.shape} (expected {self.dim}); trash cue disabled")
                return
            embs.append(emb)
            labels.append(str(item["label"]).lower())
        if not embs or set(labels) != set(TRASH_TO_CUBE):
            print("[WARN] vector DB labels do not match cube registry; trash cue disabled")
            return
        mat = np.stack(embs)
        mat /= np.linalg.norm(mat, axis=1, keepdims=True) + 1e-9
        self.matrix = torch.from_numpy(mat)
        self.labels = labels

        try:
            model = torch.hub.load("facebookresearch/dinov2",
                                   model_name)
        except Exception as exc:  # noqa: BLE001 - offline cache etc.
            print(f"[WARN] cannot load {model_name}: {exc}; "
                  "trash cue disabled")
            self.matrix = None
            return
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        model = model.to(device)
        model.eval()
        self.model = model
        print(f"[OK] DINO trash matcher: {len(set(labels))} classes x "
              f"{len(labels) // max(1, len(set(labels)))} vectors "
              f"on {device} (thresh={thresh}, margin={margin})")

    # ImageNet stats, same as torchvision Normalize used at DB build time.
    _MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    _STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def _preprocess(self, face_bgr):
        # The caller already supplies the whole rectified upper face.
        face_bgr = cv2.resize(face_bgr, (self.IMG_SIZE, self.IMG_SIZE))
        rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        normed = (rgb - self._MEAN) / self._STD
        return self._torch.from_numpy(
            normed.transpose(2, 0, 1)).unsqueeze(0)

    def match(self, crop_bgr):
        """Match one crop; compatibility wrapper around batched inference."""
        label, score, self.last_result = self.match_many([crop_bgr])[0]
        return label, score

    def match_many(self, crops_bgr):
        """Batch face crops so several visible faces share each DINO forward pass."""
        unavailable = {"reason": "trash model unavailable", "score": 0.0,
                       "margin": 0.0, "label": ""}
        if self.model is None or self.matrix is None:
            return [(None, 0.0, unavailable.copy()) for _ in crops_bgr]
        torch = self._torch
        results = []
        for start in range(0, len(crops_bgr), 4):
            batch = crops_bgr[start:start + 4]
            turns = [np.ascontiguousarray(np.rot90(crop, k))
                     for crop in batch for k in range(4)]
            with torch.no_grad():
                tensor = torch.cat([self._preprocess(turn) for turn in turns],
                                   dim=0).to(self.device)
                embeddings = self.model(tensor).cpu().numpy()
            embeddings /= np.linalg.norm(embeddings, axis=1, keepdims=True) + 1e-9
            coarse = embeddings.reshape(len(batch), 4, -1)
            similarities = [self.matrix.numpy() @ four.T for four in coarse]
            provisional = [self._rank_scores(sims, (0, 90, 180, 270))
                           for sims in similarities]
            uncertain = [i for i, (_, score, result) in enumerate(provisional)
                         if score < 0.55 or result["margin"] < 0.08]
            if uncertain:
                fine_turns = []
                for i in uncertain:
                    crop = cv2.resize(batch[i], (self.IMG_SIZE, self.IMG_SIZE))
                    center = (self.IMG_SIZE / 2, self.IMG_SIZE / 2)
                    matrix = cv2.getRotationMatrix2D(center, 45, 1.0)
                    angled = cv2.warpAffine(crop, matrix,
                                             (self.IMG_SIZE, self.IMG_SIZE),
                                             borderMode=cv2.BORDER_REFLECT_101)
                    fine_turns.extend(np.ascontiguousarray(np.rot90(angled, k))
                                      for k in range(4))
                with torch.no_grad():
                    tensor = torch.cat([self._preprocess(turn)
                                        for turn in fine_turns], dim=0).to(self.device)
                    fine_embeddings = self.model(tensor).cpu().numpy()
                fine_embeddings /= np.linalg.norm(fine_embeddings, axis=1,
                                                   keepdims=True) + 1e-9
                for position, i in enumerate(uncertain):
                    extra = self.matrix.numpy() @ fine_embeddings[
                        position * 4:(position + 1) * 4].T
                    similarities[i] = np.concatenate((similarities[i], extra), axis=1)
            for sims in similarities:
                angles = (0, 90, 180, 270, 45, 135, 225, 315)[:sims.shape[1]]
                results.append(self._rank_scores(sims, angles))
        return results

    def _rank_scores(self, sims, angles=None):
        class_scores = {lbl: [] for lbl in set(self.labels)}
        for lbl, turn_scores in zip(self.labels, sims):
            class_scores[lbl].append(float(np.max(turn_scores)))

        best_per_class: dict[str, float] = {}
        for lbl, scores in class_scores.items():
            scores.sort(reverse=True)
            # Average the three closest references for each class.
            best_per_class[lbl] = sum(scores[:3]) / max(1, min(len(scores), 3))

        ranked = sorted(best_per_class.items(), key=lambda item: item[1],
                        reverse=True)
        label, score = ranked[0]
        gap = score - ranked[1][1]
        result = {"label": label, "score": score, "margin": gap,
                  "reason": "", "ranked": ranked[:2]}
        if angles is not None:
            winning_rows = [i for i, value in enumerate(self.labels)
                            if value == label]
            _, turn = np.unravel_index(np.argmax(sims[winning_rows]),
                                       (len(winning_rows), sims.shape[1]))
            result["rotation_deg"] = angles[int(turn)]
            result["rotations_tested"] = len(angles)
        if score < self.thresh:
            result["reason"] = "trash cosine below threshold"
            return None, score, result
        if gap < self.margin:
            result["reason"] = "trash top two classes too close"
            return None, score, result
        return label, score, result


def detect_color_regions(frame_bgr):
    """Return {color_label: binary mask} for the 4 T8 cube colors."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    masks = {}
    for label, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        masks[label] = mask
    return masks


def color_top_from_component(component_mask, edges, lines, box):
    """Recover a coloured top only when a lower cuboid edge is visible."""
    x, y, w, h = box
    contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    perimeter = cv2.arcLength(hull, True)
    quad = None
    for epsilon in (0.015, 0.025, 0.04, 0.06):
        candidate = cv2.approxPolyDP(hull, epsilon * perimeter, True)
        if len(candidate) == 4:
            quad = order_corners(candidate)
            break
    if quad is None:
        return None
    support = _below_face_support(edges, quad)
    if (support < BELOW_FACE_SUPPORT_MIN or
            float(cv2.contourArea(quad)) < GRASP_TOP_AREA_MIN_PX2 or
            min(w, h) < GRASP_TOP_BOX_SIDE_MIN_PX or x < 3 or y < 3 or
            x + w > edges.shape[1] - 3 or y + h > edges.shape[0] - 3):
        return None
    bottom = float(max(quad[:, 1]))
    has_lower_edge = any(
        abs(y2 - y1) <= 0.30 * max(1, abs(x2 - x1)) and
        abs(x2 - x1) >= 0.45 * w and
        min(x2, x + w) - max(x1, x) >= 0.40 * w and
        bottom + max(8, 0.08 * h) <= (y1 + y2) / 2 <=
        bottom + max(40, 0.85 * h)
        for x1, y1, x2, y2 in lines)
    if not has_lower_edge:
        return None
    has_side_edge = any(
        abs(y2 - y1) >= max(10, 0.15 * h) and
        min(y1, y2) <= bottom + max(8, 0.08 * h) and
        max(y1, y2) >= bottom + max(8, 0.08 * h) and
        (abs(x1 - x) <= 0.25 * w or abs(x1 - (x + w)) <= 0.25 * w or
         abs(x2 - x) <= 0.25 * w or abs(x2 - (x + w)) <= 0.25 * w)
        for x1, y1, x2, y2 in lines)
    if not has_side_edge:
        return None
    return TopFace(quad, box, float(cv2.contourArea(quad)), support)


@dataclass
class TopFace:
    corners: np.ndarray               # 4x2, clockwise from top-left
    box: tuple[int, int, int, int]
    area: float
    support: float                    # visible structure below the face
    outline_vertices: int = 4
    component_id: int = -1


@dataclass
class CubeGroup:
    faces: list[TopFace]
    top: TopFace | None
    box: tuple[int, int, int, int]
    geometry_reason: str = ""

    @property
    def top_center_px(self):
        if self.top is None:
            return None
        p = self.top.corners.astype(np.float64)
        a = np.column_stack((p[2] - p[0], p[3] - p[1]))
        try:
            t = np.linalg.solve(a, p[3] - p[0])[0]
        except np.linalg.LinAlgError:
            return None
        if not 0 < t < 1:
            return None
        return tuple(float(v) for v in p[0] + t * (p[2] - p[0]))

    @property
    def geometry_valid(self):
        return self.top_center_px is not None


@dataclass
class CubeObservation:
    group: CubeGroup
    verdict: CubeVerdict
    face_verdicts: list[tuple[TopFace, CubeVerdict]]

    @property
    def top_center_px(self):
        return self.group.top_center_px

    @property
    def geometry_valid(self):
        return self.group.geometry_valid

    def to_dict(self):
        return {"cube_id": self.verdict.cube_id,
                "name": self.verdict.name,
                "via": self.verdict.via,
                "label": self.verdict.label,
                "score": self.verdict.score,
                "reason": self.verdict.reason or self.verdict.conflict,
                "trash_top": [{"class": label, "similarity_percent":
                               round(100 * score, 1)}
                              for label, score in self.verdict.trash_top[:2]],
                "trash_rotation_deg": self.verdict.trash_rotation_deg,
                "box_px": list(self.group.box),
                "top_quad_px": (self.group.top.corners.tolist()
                                if self.group.top is not None else None),
                "top_center_px": (list(self.top_center_px)
                                  if self.top_center_px is not None else None),
                "geometry_valid": self.geometry_valid,
                "geometry_reason": self.group.geometry_reason,
                "face_votes": [{"cube_id": verdict.cube_id,
                                "via": verdict.via,
                                "label": verdict.label,
                                "score": verdict.score,
                                "reason": verdict.reason,
                                "quad_px": face.corners.tolist()}
                               for face, verdict in self.face_verdicts],
                "robot_target_valid": False}


def order_corners(points):
    """Return clockwise corners for a convex quadrilateral."""
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    points = points[np.argsort(angles)]
    start = int(np.argmin(points.sum(axis=1)))
    return np.roll(points, -start, axis=0).astype(np.float32)


def rectify_face(frame_bgr, corners, size=224):
    src = order_corners(corners)
    dst = np.float32([[0, 0], [size - 1, 0],
                      [size - 1, size - 1], [0, size - 1]])
    return cv2.warpPerspective(frame_bgr,
                               cv2.getPerspectiveTransform(src, dst),
                               (size, size))


def _below_face_support(edges, corners):
    """Check for a cube side below a candidate; flat markings lack one."""
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, corners.astype(np.int32), 255)
    outer = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                       (35, 35)))
    inner = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                       (5, 5)))
    band = cv2.subtract(outer, inner)
    mid_y = int(corners[:, 1].mean())
    band[:mid_y] = 0
    pixels = cv2.countNonZero(band)
    return cv2.countNonZero(cv2.bitwise_and(edges, band)) / max(1, pixels)


def _quad_iou(a, b, shape):
    x1, y1, w1, h1 = a.box
    x2, y2, w2, h2 = b.box
    if min(x1 + w1, x2 + w2) <= max(x1, x2) or \
       min(y1 + h1, y2 + h2) <= max(y1, y2):
        return 0.0
    mask_a = np.zeros(shape, dtype=np.uint8)
    mask_b = np.zeros(shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask_a, a.corners.astype(np.int32), 1)
    cv2.fillConvexPoly(mask_b, b.corners.astype(np.int32), 1)
    overlap = np.count_nonzero(mask_a & mask_b)
    union = np.count_nonzero(mask_a | mask_b)
    return overlap / max(1, union)


def _shared_edge(a, b):
    """Return whether two distinct planes meet along a projected edge."""
    pa, pb = a.corners, b.corners
    for i in range(4):
        aa, ab = pa[i], pa[(i + 1) % 4]
        alen = float(np.linalg.norm(ab - aa))
        for j in range(4):
            ba, bb = pb[j], pb[(j + 1) % 4]
            blen = float(np.linalg.norm(bb - ba))
            if min(alen, blen) < 12:
                continue
            tolerance = max(5.0, 0.13 * min(alen, blen))
            direction = (ab - aa) / alen
            if abs(float(np.dot(direction, (bb - ba) / blen))) < 0.96:
                continue
            offsets = [abs(float(direction[0] * (point[1] - aa[1]) -
                                 direction[1] * (point[0] - aa[0])))
                       for point in (ba, bb)]
            positions = sorted(float(np.dot(point - aa, direction))
                               for point in (ba, bb))
            overlap = min(alen, positions[1]) - max(0.0, positions[0])
            if max(offsets) < tolerance and overlap > 0.55 * min(alen, blen):
                return True
    return False


def _quad_overlap(a, b):
    area, _ = cv2.intersectConvexConvex(a.corners, b.corners)
    return float(area) / max(1.0, min(a.area, b.area))


def _group_box(faces):
    all_points = np.concatenate([face.corners for face in faces]).astype(np.float32)
    return cv2.boundingRect(all_points)


def _box_coverage(inner, outer):
    ax, ay, aw, ah = inner
    bx, by, bw, bh = outer
    overlap = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * \
        max(0, min(ay + ah, by + bh) - max(ay, by))
    return overlap / max(1, aw * ah)


def find_cube_groups(frame_bgr, diagnostics=None):
    """Keep physical face planes and group only overlapping or joined planes.

    A printed inner rectangle is grouped with its enclosing face, but cannot
    become the upper surface. A flat grid has no outer plane with a side below.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    contrast = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(cv2.GaussianBlur(contrast, (3, 3), 0), 20, 75)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                             np.ones((3, 3), np.uint8))
    _, components = cv2.connectedComponents((edges > 0).astype(np.uint8), 8)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST,
                                   cv2.CHAIN_APPROX_SIMPLE)
    enlarged = cv2.resize(contrast, None, fx=2, fy=2,
                          interpolation=cv2.INTER_CUBIC)
    fine_edges = cv2.Canny(cv2.GaussianBlur(enlarged, (3, 3), 0), 20, 75)
    fine_contours, _ = cv2.findContours(fine_edges, cv2.RETR_LIST,
                                        cv2.CHAIN_APPROX_SIMPLE)
    height, width = gray.shape
    candidates = []
    for contour, scale in ([(contour, 1) for contour in contours] +
                           [(contour, 2) for contour in fine_contours]):
        area = cv2.contourArea(contour) / (scale * scale)
        bx, by, bw, bh = cv2.boundingRect(contour)
        raw_box = (bx / scale, by / scale, bw / scale, bh / scale)
        metric = None
        if diagnostics is not None and area >= 50 and min(bw, bh) / scale >= 8:
            metric = {"area_px2": round(float(area), 1),
                      "box_px": [round(float(v), 1) for v in raw_box],
                      "area_box_ratio": round(float(area) / max(1, bw * bh /
                                                               (scale * scale)), 3),
                      "min_box_side_px": round(min(bw, bh) / scale, 1),
                      "pass": "2x" if scale == 2 else "1x",
                      "result": "candidate"}
            diagnostics.append(metric)
        box_area = (bw * bh) / (scale * scale)
        box_w = bw / scale
        box_h = bh / scale
        fill_ratio = area / max(1.0, box_area)
        if not CONTOUR_AREA_MIN_PX2 <= area <= 0.80 * width * height:
            if metric is not None:
                metric["result"] = "area-range"
            continue
        if scale == 2 and area > 3200:
            if metric is not None:
                metric["result"] = "2x-large"
            continue
        if (min(box_w, box_h) < CONTOUR_BOX_SIDE_MIN_PX or
                box_w > CONTOUR_BOX_WIDTH_MAX_FRAC * width or
                box_h > CONTOUR_BOX_HEIGHT_MAX_FRAC * height or
                fill_ratio < CONTOUR_AREA_BOX_RATIO_MIN):
            if metric is not None:
                metric["result"] = "bbox-range"
            continue
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        corners = None
        for epsilon in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.1):
            approx = cv2.approxPolyDP(hull, epsilon * perimeter, True)
            if len(approx) == 4:
                corners = order_corners(approx) / scale
                break
        if corners is None:
            if metric is not None:
                metric["result"] = "not-quadrilateral"
            continue
        x, y, w, h = cv2.boundingRect(corners)
        if (min(w, h) < CONTOUR_BOX_SIDE_MIN_PX or
                w > CONTOUR_BOX_WIDTH_MAX_FRAC * width or
                h > CONTOUR_BOX_HEIGHT_MAX_FRAC * height or
                not 0.3 <= w / h <= 3.3):
            if metric is not None:
                metric["result"] = "bbox-shape"
            continue
        side_lengths = np.linalg.norm(corners - np.roll(corners, 1, axis=0),
                                      axis=1)
        if (side_lengths.min() < CONTOUR_BOX_SIDE_MIN_PX or
                side_lengths.max() / side_lengths.min() > 4.0):
            if metric is not None:
                metric["result"] = "edge-length"
            continue
        support = _below_face_support(edges, corners)
        outline_vertices = len(cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True))
        px, py = (contour[0, 0] / scale).astype(int)
        candidate = TopFace(corners, (x, y, w, h), area, support,
                            outline_vertices, int(components[py, px]) if scale == 1 else -1)
        if any(_quad_iou(candidate, existing, gray.shape) > 0.80 and
               min(candidate.area, existing.area) / max(candidate.area, existing.area) > 0.75
               for existing in candidates):
            if metric is not None:
                metric["result"] = "duplicate"
            continue
        candidates.append(candidate)

    parents = list(range(len(candidates)))

    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    for i, a in enumerate(candidates):
        for j in range(i):
            b = candidates[j]
            if _quad_overlap(a, b) > 0.90 or (a.component_id == b.component_id and
                                             _shared_edge(a, b)):
                parents[root(i)] = root(j)
    components = {}
    for i, candidate in enumerate(candidates):
        components.setdefault(root(i), []).append(candidate)

    groups = []
    for faces in components.values():
        silhouettes = {id(face) for face in faces if face.outline_vertices >= 5 and
                       sum(other is not face and other.area < 0.8 * face.area and
                           _quad_overlap(face, other) > 0.8 for other in faces) >= 2}
        surfaces = [face for face in faces if id(face) not in silhouettes]
        physical = [face for face in surfaces if not any(
            other is not face and other.area > 1.35 * face.area and
            _quad_overlap(face, other) > 0.8 for other in surfaces)]
        if not physical:
            continue
        possible = []
        for face in physical:
            lower_sides = [other for other in physical if other is not face and
                           _shared_edge(face, other) and
                           other.corners[:, 1].mean() > face.corners[:, 1].mean() + 0.1 * face.box[3] and
                           other.area >= 0.2 * face.area]
            clipped = (face.box[0] < 3 or face.box[1] < 3 or
                       face.box[0] + face.box[2] > width - 3 or
                       face.box[1] + face.box[3] > height - 3)
            outer_width = _group_box(physical)[2]
            if (lower_sides and not clipped and
                    face.area >= GRASP_TOP_AREA_MIN_PX2 and
                    min(face.box[2:]) >= GRASP_TOP_BOX_SIDE_MIN_PX and
                    face.box[2] >= 0.5 * outer_width and
                    face.area >= 0.4 * max(f.area for f in physical)):
                possible.append((len(lower_sides), face.area, face))
        top = max(possible, key=lambda item: (item[0], item[1]))[2] if possible else None
        box = _group_box(physical)
        if (top is not None or len(physical) > 1 or
                any(f.area >= GRASP_TOP_AREA_MIN_PX2 for f in physical)):
            reason = "" if top is not None else (
                "frame-clipped" if box[0] < 3 or box[1] < 3 or
                box[0] + box[2] > width - 3 or box[1] + box[3] > height - 3
                else "too-small" if max(f.area for f in physical) < GRASP_TOP_AREA_MIN_PX2 or
                max(min(f.box[2:]) for f in physical) < GRASP_TOP_BOX_SIDE_MIN_PX
                else "no-outer-face" if len(physical) == 1
                else "no-side-support")
            groups.append(CubeGroup(physical, top, box, reason))
    groups = sorted(groups, key=lambda group: group.box[2] * group.box[3],
                    reverse=True)[:12]
    return sorted(groups, key=lambda group: (group.box[1], group.box[0]))


def find_top_faces(frame_bgr):
    """Compatibility helper for callers interested only in verified tops."""
    return [group.top for group in find_cube_groups(frame_bgr) if group.geometry_valid]


def save_contour_debug(frame_bgr, prefix):
    """Save the raw frame and measurements needed for corner tuning."""
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    raw_path = prefix.with_name(prefix.name + "_raw.png")
    report_path = prefix.with_name(prefix.name + "_contours.json")
    candidates = []
    groups = find_cube_groups(frame_bgr, diagnostics=candidates)
    recognized, _ = identify_frame(frame_bgr, tags=[], trash_detector=None,
                                   annotate=False)
    report = {
        "image_size": list(frame_bgr.shape[1::-1]),
        "thresholds": {"contour_area_min_px2": CONTOUR_AREA_MIN_PX2,
                       "candidate_box_side_min_px": CONTOUR_BOX_SIDE_MIN_PX,
                       "contour_area_box_ratio_min": CONTOUR_AREA_BOX_RATIO_MIN,
                       "contour_box_width_max_frac": CONTOUR_BOX_WIDTH_MAX_FRAC,
                       "contour_box_height_max_frac": CONTOUR_BOX_HEIGHT_MAX_FRAC,
                       "grasp_top_area_min_px2": GRASP_TOP_AREA_MIN_PX2,
                       "grasp_top_box_side_min_px": GRASP_TOP_BOX_SIDE_MIN_PX,
                       "below_face_support_min": BELOW_FACE_SUPPORT_MIN},
        "groups": [{
            "box_px": list(group.box),
            "geometry_valid": group.geometry_valid,
            "geometry_reason": group.geometry_reason,
            "faces": [{"area_px2": round(face.area, 1),
                       "box_px": list(face.box),
                       "area_box_ratio": round(face.area / max(1, face.box[2] *
                                                               face.box[3]), 3),
                       "min_edge_px": round(float(np.min(np.linalg.norm(
                           face.corners - np.roll(face.corners, 1, axis=0),
                           axis=1))), 1),
                       "below_support": round(face.support, 3),
                       "is_top": face is group.top}
                      for face in group.faces]}
            for group in groups],
        "recognized_groups": [{"cube_id": obs.verdict.cube_id,
                               "box_px": list(obs.group.box),
                               "geometry_valid": obs.geometry_valid,
                               "geometry_reason": obs.group.geometry_reason,
                               "top_center_px": obs.top_center_px,
                               "top_area_px2": (round(obs.group.top.area, 1)
                                                if obs.group.top is not None else None),
                               "top_area_box_ratio": (
                                   round(obs.group.top.area / max(1,
                                       obs.group.top.box[2] * obs.group.top.box[3]), 3)
                                   if obs.group.top is not None else None),
                               "top_min_edge_px": (
                                   round(float(np.min(np.linalg.norm(
                                       obs.group.top.corners - np.roll(
                                           obs.group.top.corners, 1, axis=0),
                                       axis=1))), 1)
                                   if obs.group.top is not None else None)}
                              for obs in recognized],
        "contours": sorted(candidates, key=lambda item: item["area_px2"],
                           reverse=True)[:300],
    }
    if not cv2.imwrite(str(raw_path), frame_bgr):
        raise OSError(f"cannot save raw frame: {raw_path}")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return raw_path, report_path


def color_on_face(face_bgr):
    """Require a nearly uniform color face, not a colored printed object."""
    crop = face_bgr[18:-18, 18:-18]
    if crop.size == 0:
        return None, 0.0
    masks = detect_color_regions(crop)
    scores = {label: np.count_nonzero(mask) / mask.size
              for label, mask in masks.items()}
    label = max(scores, key=scores.get)
    score = scores[label]
    if score < 0.40:
        return None, score
    mask = masks[label]
    saturation = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[:, :, 1]
    if np.median(saturation[mask != 0]) < 95:
        return None, score
    h, w = mask.shape
    quadrants = (mask[:h // 2, :w // 2], mask[:h // 2, w // 2:],
                 mask[h // 2:, :w // 2], mask[h // 2:, w // 2:])
    if any(np.count_nonzero(q) / q.size < 0.20 for q in quadrants):
        return None, score
    return label, score


def tag_on_face(tags, corners):
    """Associate a tag by its center and most of its projected area."""
    polygon = corners.astype(np.float32)
    accepted = []
    for tag in tags:
        tag_corners = tag.get("corners")
        if tag_corners is None or tag.get("margin", 0.0) < 15:
            continue
        quad = np.asarray(tag_corners, dtype=np.float32).reshape(4, 2)
        center = tuple(map(float, quad.mean(axis=0)))
        overlap, _ = cv2.intersectConvexConvex(polygon, quad)
        if (cv2.pointPolygonTest(polygon, center, False) >= 0 and
                overlap >= 0.55 * max(1.0, cv2.contourArea(quad))):
            accepted.append(tag)
    return accepted


def tag_fills_candidate(face, tags):
    """A contour wrapped around a tag is an inner marking, not a cube face."""
    return any(cv2.contourArea(np.asarray(tag["corners"],
                                           dtype=np.float32)) > 0.55 * face.area
               for tag in tag_on_face(tags, face.corners))


def classify_top_face(face_bgr, corners, tags, trash_detector=None):
    """One upper face selects exactly one cue: tag, uniform color, or trash."""
    on_face = tag_on_face(tags, corners)
    if on_face:
        ids = {tag["id"] for tag in on_face}
        if len(ids) != 1 or next(iter(ids)) not in TAG_TO_CUBE:
            return FaceCues(), CubeVerdict(None, "unknown", "none",
                                           reason="invalid or multiple top tags")
        tag = on_face[0]
        cues = FaceCues(tag_id=tag["id"])
        verdict = fuse_cues(cues)
        verdict.label = f"tag:{tag['id']}"
        verdict.score = tag["margin"]
        return cues, verdict

    color_label, coverage = color_on_face(face_bgr)
    if color_label is not None:
        cues = FaceCues(color_label=color_label)
        verdict = fuse_cues(cues)
        verdict.label = color_label
        verdict.score = coverage
        return cues, verdict

    if trash_detector is None:
        return FaceCues(), CubeVerdict(None, "unknown", "none",
                                       reason="trash model unavailable")
    trash_label, score = trash_detector.match(face_bgr)
    if trash_label is None or trash_label not in TRASH_TO_CUBE:
        reason = trash_detector.last_result.get("reason", "unknown trash")
        return FaceCues(), CubeVerdict(None, "unknown", "none", score=score,
                                       reason=reason,
                                       trash_top=trash_detector.last_result.get("ranked", []),
                                       trash_rotation_deg=trash_detector.last_result.get("rotation_deg"))
    cues = FaceCues(trash_class=trash_label)
    verdict = fuse_cues(cues)
    verdict.label = trash_label
    verdict.score = score
    verdict.trash_top = trash_detector.last_result.get("ranked", [])
    verdict.trash_rotation_deg = trash_detector.last_result.get("rotation_deg")
    return cues, verdict


def fuse_group(face_verdicts):
    """Identify a cube from its strongest visible cue."""
    accepted = [verdict for _, verdict in face_verdicts
                if verdict.cube_id is not None]
    if not accepted:
        reasons = [v.reason for _, v in face_verdicts if v.reason]
        dino = next((v for _, v in face_verdicts if v.trash_top), None)
        return CubeVerdict(None, "unknown", "none",
                           reason="; ".join(dict.fromkeys(reasons)) or "no face cue",
                           trash_top=dino.trash_top if dino else [],
                           trash_rotation_deg=dino.trash_rotation_deg if dino else None)
    priority = {"tag": 0, "color": 1, "trash": 2}
    best = min(accepted, key=lambda v: (priority.get(v.via, 3), -v.score))
    conflict = ", ".join(f"{v.via}:{v.label}->ID{v.cube_id}"
                         for v in accepted if v.cube_id != best.cube_id)
    dino = next((v for _, v in face_verdicts if v.trash_top), None)
    result = CubeVerdict(best.cube_id, best.name, best.via,
                         label=best.label, score=best.score,
                         conflict=conflict,
                         trash_top=dino.trash_top if dino else [],
                         trash_rotation_deg=dino.trash_rotation_deg if dino else None)
    return result


def _clamp_box(box, frame_shape):
    """Return a box whose drawn edges stay inside the image."""
    height, width = frame_shape[:2]
    x, y, w, h = map(int, box)
    x1 = max(0, min(width - 1, x))
    y1 = max(0, min(height - 1, y))
    x2 = max(0, min(width - 1, x + max(1, w)))
    y2 = max(0, min(height - 1, y + max(1, h)))
    return x1, y1, x2, y2


def _observation_sort_key(obs):
    """Prefer observations that can either be picked or explain a strong ID."""
    source = obs.verdict.via.split("+")[0]
    source_priority = {"tag": 3, "color": 2, "trash": 1}.get(source, 0)
    area = obs.group.box[2] * obs.group.box[3]
    return (obs.geometry_valid, source_priority, float(obs.verdict.score), area)


def _clean_observations(observations):
    """Suppress invalid clutter while preserving useful ID diagnostics."""
    useful = []
    for obs in observations:
        if obs.geometry_valid:
            useful.append(obs)
            continue
        if obs.verdict.cube_id is None:
            continue
        if obs.verdict.via == "trash" and obs.verdict.score < INVALID_TRASH_ID_SCORE_MIN:
            continue
        if (obs.verdict.via not in {"tag", "color", "trash"} and
                obs.verdict.score < DISPLAY_INVALID_SCORE_MIN):
            continue
        useful.append(obs)

    best_invalid_by_id = {}
    cleaned = []
    for obs in useful:
        if obs.geometry_valid or obs.verdict.cube_id is None:
            cleaned.append(obs)
            continue
        current = best_invalid_by_id.get(obs.verdict.cube_id)
        if current is None or _observation_sort_key(obs) > _observation_sort_key(current):
            best_invalid_by_id[obs.verdict.cube_id] = obs
    cleaned.extend(best_invalid_by_id.values())
    return sorted(cleaned, key=lambda obs: (obs.group.box[1], obs.group.box[0]))


def draw_observations(frame_bgr, observations, show_similarity=True):
    annotated = frame_bgr.copy()
    if not observations:
        cv2.putText(annotated, "No cube group", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    for obs in observations:
        x, y, w, h = obs.group.box
        x1, y1, x2, y2 = _clamp_box(obs.group.box, annotated.shape)
        cv2.rectangle(annotated, (x1, y1), (x2, y2),
                      (0, 200, 0) if obs.geometry_valid else (0, 165, 255), 2)
        for face, _ in obs.face_verdicts:
            cv2.polylines(annotated, [face.corners.astype(np.int32)], True,
                          (180, 180, 180), 1)
        if obs.group.top is not None:
            cv2.polylines(annotated, [obs.group.top.corners.astype(np.int32)], True,
                          (0, 200, 0) if obs.verdict.cube_id else (0, 165, 255), 2)
        if obs.top_center_px is not None:
            cv2.drawMarker(annotated, tuple(map(int, obs.top_center_px)),
                           (255, 0, 255), cv2.MARKER_CROSS, 16, 2)
        status = f"ID:{obs.verdict.cube_id or '?'} {obs.verdict.via}"
        if not obs.geometry_valid:
            status += f" {obs.group.geometry_reason or 'no-top'}"
        text_x = max(4, min(x1, annotated.shape[1] - 220))
        text_y = max(16, min(annotated.shape[0] - 8, y1 - 8 if y1 >= 24 else y2 + 18))
        cv2.putText(annotated, status, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 0) if obs.verdict.cube_id and obs.geometry_valid
                    else (0, 165, 255), 2)
        if show_similarity and obs.verdict.trash_top:
            draw_similarity_panel(annotated, obs.group.box, obs.verdict.trash_top,
                                  obs.verdict.trash_rotation_deg)
    return annotated


def identify_frame(frame_bgr, tags, trash_detector=None, annotate=True,
                   fast_known=False):
    """Classify visible physical faces and report one observation per cube."""
    grouped = []
    pending_crops = []
    pending_slots = []
    for group in find_cube_groups(frame_bgr):
        face_verdicts = []
        group_pending = []
        for face in sorted(group.faces, key=lambda f: f.area, reverse=True)[:3]:
            crop = rectify_face(frame_bgr, face.corners)
            _, verdict = classify_top_face(crop, face.corners, tags, None)
            if min(face.box[2:]) < max(28, GRASP_TOP_BOX_SIDE_MIN_PX) and verdict.cube_id is None:
                verdict.reason = "low-image-quality"
            if min(face.box[2:]) >= max(28, GRASP_TOP_BOX_SIDE_MIN_PX) and trash_detector is not None and (not fast_known or
                    verdict.cube_id is None):
                group_pending.append((len(face_verdicts), crop))
            face_verdicts.append((face, verdict))
        if not fast_known or not any(v.cube_id is not None
                                     for _, v in face_verdicts):
            for face_index, crop in group_pending:
                pending_slots.append((len(grouped), face_index))
                pending_crops.append(crop)
        grouped.append((group, face_verdicts))
    # A valid tag identifies its cube even when the side/top contours disappear
    # under perspective or glare. It does not manufacture a graspable top.
    for tag in tags:
        if (tag.get("id") not in TAG_TO_CUBE or tag.get("margin", 0) < 15 or
                tag.get("corners") is None):
            continue
        center = tag.get("center")
        if center is None and tag.get("corners") is not None:
            center = tuple(np.asarray(tag["corners"]).reshape(4, 2).mean(axis=0))
        if center is None:
            continue
        quad = np.asarray(tag["corners"], dtype=np.float32).reshape(4, 2)
        box = cv2.boundingRect(quad)
        face = TopFace(quad, box, float(cv2.contourArea(quad)), 0.0)
        verdict = fuse_cues(FaceCues(tag_id=tag["id"]))
        verdict.label = f"tag:{tag['id']}"
        verdict.score = tag["margin"]
        attached = False
        for group, face_verdicts in grouped:
            x, y, w, h = group.box
            if x <= center[0] <= x + w and y <= center[1] <= y + h:
                if not any(v.via == "tag" and v.label == verdict.label
                           for _, v in face_verdicts):
                    face_verdicts.append((face, verdict))
                attached = True
                break
        if not attached:
            grouped.append((CubeGroup([face], None, box, "no-outer-face"),
                            [(face, verdict)]))
    # Solid colour can still give an ID if the cuboid edges are too weak to
    # make a quadrilateral. The colour region alone cannot provide a pick pose.
    masks = detect_color_regions(frame_bgr)
    saturation = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[:, :, 1]
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 25, 90)
    detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                                      threshold=25, minLineLength=40,
                                      maxLineGap=12)
    lines = [] if detected_lines is None else [tuple(map(int, line.reshape(4)))
                                               for line in detected_lines]
    for label, mask in masks.items():
        n, components, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        for i in range(1, n):
            x, y, w, h, area = map(int, stats[i])
            if (area < COLOR_COMPONENT_AREA_MIN_PX2 or area > 0.80 * mask.size or
                    min(w, h) < COLOR_COMPONENT_BOX_SIDE_MIN_PX or
                    area < 0.40 * w * h):
                continue
            cx, cy = x + w / 2, y + h / 2
            component_mask = np.uint8(components == i) * 255
            if np.median(saturation[component_mask != 0]) < 95:
                continue
            top = color_top_from_component(component_mask, edges, lines,
                                           (x, y, w, h))
            existing = next(((j, group, face_verdicts)
                             for j, (group, face_verdicts) in enumerate(grouped)
                             if group.box[0] <= cx <= group.box[0] + group.box[2] and
                             group.box[1] <= cy <= group.box[1] + group.box[3]), None)
            if existing is not None:
                group_index, group, face_verdicts = existing
                if group.top is None and top is not None:
                    group.top = top
                    group.geometry_reason = ""
                    group.faces.append(top)
                    if not any(v.via == "color" and v.label == label
                               for _, v in face_verdicts):
                        verdict = fuse_cues(FaceCues(color_label=label))
                        verdict.label, verdict.score = label, area / (w * h)
                        face_verdicts.append((top, verdict))
                        if trash_detector is not None and not fast_known and min(w, h) >= max(28, GRASP_TOP_BOX_SIDE_MIN_PX):
                            pending_slots.append((group_index, len(face_verdicts) - 1))
                            pending_crops.append(rectify_face(frame_bgr, top.corners))
                continue
            quad = top.corners if top is not None else np.float32(
                [[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
            face = top or TopFace(quad, (x, y, w, h), float(area), 0.0)
            verdict = fuse_cues(FaceCues(color_label=label))
            verdict.label, verdict.score = label, area / (w * h)
            reason = "" if top is not None else (
                "frame-clipped" if x < 3 or y < 3 or x + w > mask.shape[1] - 3 or
                y + h > mask.shape[0] - 3 else
                "too-small" if area < GRASP_TOP_AREA_MIN_PX2 or
                min(w, h) < GRASP_TOP_BOX_SIDE_MIN_PX else "no-side-support")
            grouped.append((CubeGroup([face], top, (x, y, w, h), reason),
                            [(face, verdict)]))
            if trash_detector is not None and not fast_known and min(w, h) >= max(28, GRASP_TOP_BOX_SIDE_MIN_PX):
                pending_slots.append((len(grouped) - 1, 0))
                pending_crops.append(rectify_face(frame_bgr, face.corners))
    if pending_crops:
        for (group_index, face_index), (label, score, result) in zip(
                pending_slots, trash_detector.match_many(pending_crops)):
            face, existing = grouped[group_index][1][face_index]
            if existing.cube_id is not None:
                existing.trash_top = result.get("ranked", [])
                existing.trash_rotation_deg = result.get("rotation_deg")
                continue
            if label in TRASH_TO_CUBE:
                verdict = fuse_cues(FaceCues(trash_class=label))
                verdict.label = label
            else:
                verdict = CubeVerdict(None, "unknown", "none",
                                      reason=result["reason"])
            verdict.score = score
            verdict.trash_top = result.get("ranked", [])
            verdict.trash_rotation_deg = result.get("rotation_deg")
            grouped[group_index][1][face_index] = (face, verdict)
    observations = []
    for group, face_verdicts in grouped:
        observations.append(CubeObservation(group, fuse_group(face_verdicts),
                                            face_verdicts))
    # Weak inner/side contours can form a second group inside a cube whose
    # outer top is already known. Keep the outer group as the physical cube.
    observations = [obs for obs in observations if not (
        not obs.geometry_valid and any(
            other is not obs and other.geometry_valid and
            other.verdict.cube_id is not None and
            _box_coverage(obs.group.box, other.group.box) >= 0.80
            for other in observations))]
    observations = [obs for obs in observations if not (
        obs.verdict.cube_id is None and not obs.geometry_valid and
        len(obs.group.faces) == 1)]
    observations = _clean_observations(observations)
    return observations, draw_observations(frame_bgr, observations) if annotate else None


def verdict_strength(verdict):
    """Compare evidence without treating an AprilTag margin as cosine similarity."""
    sources = verdict.via.split("+")
    tier = 3 if "tag" in sources else 2 if "color" in sources else \
        1 if "trash" in sources else 0
    return tier, float(verdict.score)


@dataclass
class TimedFaceTrack:
    observation: CubeObservation
    started: float
    last_seen: float
    samples: list[CubeVerdict] = field(default_factory=list)
    sample_observations: list[CubeObservation] = field(default_factory=list)
    published: CubeVerdict | None = None
    best_observation: CubeObservation | None = None
    best_verdict: CubeVerdict | None = None
    confirmed: bool = False
    last_id_seen: float = -float("inf")
    association_ambiguous: bool = False


class OneSecondAverager:
    """Hold identity evidence while updating each cube's live geometry."""

    def __init__(self):
        self.tracks: list[TimedFaceTrack] = []

    @staticmethod
    def summarize(samples):
        valid = [v for v in samples if v.cube_id is not None]
        if not valid:
            top = max(samples, key=lambda v: v.score, default=None)
            return CubeVerdict(None, "unknown", "none", reason="no ID in 1s",
                               trash_top=top.trash_top if top else [],
                               trash_rotation_deg=top.trash_rotation_deg if top else None)
        counts = {cube_id: sum(v.cube_id == cube_id for v in valid)
                  for cube_id in {v.cube_id for v in valid}}
        winner = max(counts, key=counts.get)
        if list(counts.values()).count(counts[winner]) > 1:
            return CubeVerdict(None, "unknown", "none", reason="1s vote tied")
        matching = [v for v in valid if v.cube_id == winner]
        representative = max(matching, key=verdict_strength)
        return CubeVerdict(winner, CUBES[winner]["name"] if winner else "unknown",
                           representative.via, label=representative.label,
                           score=sum(v.score for v in matching) / len(matching),
                           reason="",
                           trash_top=representative.trash_top,
                           trash_rotation_deg=representative.trash_rotation_deg)

    def update(self, observations, now):
        self.tracks = [track for track in self.tracks
                       if now - track.last_seen < 2.0]
        old_tracks = list(self.tracks)
        def center_of(obs):
            x, y, w, h = obs.group.box
            return np.array([x + w / 2, y + h / 2])

        distances = np.array([
            [float(np.linalg.norm(center_of(obs) - center_of(track.observation)))
             for track in old_tracks] for obs in observations], dtype=float)
        ambiguous_observations = set()
        ambiguous_tracks = set()
        for row, obs in enumerate(observations):
            if len(old_tracks) < 2:
                continue
            nearest = np.sort(distances[row])[:2]
            if (nearest[0] < max(40, 0.65 * max(obs.group.box[2:])) and
                    nearest[1] - nearest[0] < 15):
                ambiguous_observations.add(row)
        for col, track in enumerate(old_tracks):
            if len(observations) < 2:
                continue
            nearest = np.sort(distances[:, col])[:2]
            if (nearest[0] < max(40, 0.65 * max(track.observation.group.box[2:])) and
                    nearest[1] - nearest[0] < 15):
                ambiguous_tracks.add(col)
        used = set()
        reports = []
        shown = []
        for row, observation in enumerate(observations):
            x, y, w, h = observation.group.box
            options = [(distances[row, i], i)
                       for i in range(len(old_tracks)) if i not in used]
            options.sort()
            distance, index = options[0] if options else (float("inf"), -1)
            ambiguous = row in ambiguous_observations or index in ambiguous_tracks
            if distance < max(40, 0.65 * max(w, h)):
                track = self.tracks[index]
                used.add(index)
            else:
                track = TimedFaceTrack(observation, now, now)
                self.tracks.append(track)
                used.add(len(self.tracks) - 1)
                ambiguous = False
            if now - track.last_seen >= 1.0:
                track.samples.clear()
                track.sample_observations.clear()
                track.started = now
            track.observation = observation
            track.last_seen = now
            track.association_ambiguous = ambiguous
            if (track.best_verdict is not None and
                    observation.verdict.cube_id == track.best_verdict.cube_id):
                track.last_id_seen = now
            track.samples.append(observation.verdict)
            track.sample_observations.append(observation)
            if now - track.started >= 1.0:
                summary = self.summarize(track.samples)
                winner_count = sum(v.cube_id == summary.cube_id
                                   for v in track.samples)
                if summary.cube_id is not None and winner_count >= min(2, len(track.samples)):
                    matching = [obs for obs in track.sample_observations
                                if obs.verdict.cube_id == summary.cube_id]
                    strongest = max(matching, key=lambda obs:
                                    verdict_strength(obs.verdict))
                    geometry_choice = max(
                        (obs for obs in matching if obs.geometry_valid),
                        key=lambda obs: verdict_strength(obs.verdict),
                        default=None)
                    previous = track.best_verdict
                    if (previous is None or
                            verdict_strength(strongest.verdict) >
                            verdict_strength(previous)):
                        track.best_observation = strongest
                        track.best_verdict = strongest.verdict
                    if (track.best_verdict is not None and geometry_choice is not None and
                          track.best_verdict.cube_id == geometry_choice.verdict.cube_id and
                          track.best_observation is not None and
                          not track.best_observation.geometry_valid):
                        track.best_observation = CubeObservation(
                            geometry_choice.group, track.best_verdict,
                            geometry_choice.face_verdicts)
                    track.confirmed = True
                    track.published = track.best_verdict
                    if observation.verdict.cube_id == track.best_verdict.cube_id:
                        track.last_id_seen = now
                track.samples.clear()
                track.sample_observations.clear()
                track.started = now
                reports.append((track, track.published or summary))
            shown.append((track, track.published or CubeVerdict(
                None, "unknown", "none", reason="collecting 1s")))
        return shown, reports


def draw_similarity_panel(frame_bgr, box, top_two, rotation_deg=None):
    """Show DINO rankings in a readable panel kept inside the camera frame."""
    header = (f"DINO similarity  {rotation_deg} deg"
              if rotation_deg is not None else "DINO similarity")
    lines = [header] + [
        f"Top {i}: {name}  {score * 100:.1f}%"
        for i, (name, score) in enumerate(top_two[:2], 1)]
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.43, 1
    text_width = max(cv2.getTextSize(line, font, scale, thickness)[0][0]
                     for line in lines)
    height, width = frame_bgr.shape[:2]
    panel_w = min(width, text_width + 16)
    panel_h = min(height, len(lines) * 19 + 10)
    x, y, _, h = box
    left = max(0, min(x, width - panel_w))
    below = y + h + 5
    top = below if below + panel_h <= height else max(0, y - panel_h - 25)
    overlay = frame_bgr.copy()
    cv2.rectangle(overlay, (left, top), (left + panel_w - 1, top + panel_h - 1),
                  (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.82, frame_bgr, 0.18, 0, frame_bgr)
    for i, line in enumerate(lines):
        color = (150, 230, 255) if i == 0 else (255, 255, 255)
        cv2.putText(frame_bgr, line, (left + 8, top + 18 + i * 19),
                    font, scale, color, thickness, cv2.LINE_AA)


def draw_tracked_frame(frame_bgr, tracked):
    annotated = draw_observations(frame_bgr, [CubeObservation(
        track.observation.group, verdict, track.observation.face_verdicts)
        for track, verdict in tracked], show_similarity=False)
    for track, verdict in tracked:
        current = track.observation
        x, y, _, _ = current.group.box
        top_two = current.verdict.trash_top or verdict.trash_top
        if top_two:
            angle = (current.verdict.trash_rotation_deg
                     if current.verdict.trash_top else verdict.trash_rotation_deg)
            draw_similarity_panel(annotated, current.group.box, top_two, angle)
        if track.association_ambiguous:
            cv2.putText(annotated, "AMBIGUOUS: wait", (x, min(annotated.shape[0] - 8, y + 65)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 165, 255), 2)
    return annotated


# --------------------------------------------------------------------------
# Arm + camera plumbing
# --------------------------------------------------------------------------

def move_to_observe(dry_run=False, verify=True):
    """Move to the observation pose and block until settled.

    This is the mandatory pose-start: vision must not run before the arm
    is confirmed at OBSERVE_JOINTS with the gripper open. Returns False
    on any failure so the caller exits instead of scanning from a bad pose.
    """
    if dry_run:
        print("[DRY] skip arm motion (would send "
              f"{OBSERVE_JOINTS} over 2000ms)")
        return True
    try:
        from Arm_Lib import Arm_Device
    except ImportError:
        print("[FAIL] Arm_Lib not importable; rerun with --dry-run "
              "or --skip-pose (vision only, no grasp)")
        return False
    try:
        arm = Arm_Device("/dev/ttyUSB0")
    except Exception as exc:  # noqa: BLE001 - hardware may be unplugged
        print(f"[FAIL] cannot reach arm on /dev/ttyUSB0: {exc}")
        return False
    for attempt in range(3):
        try:
            arm.Arm_serial_servo_write6_array(OBSERVE_JOINTS, 2000)
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] cannot send observe pose (try {attempt + 1}/3): {exc}")
            time.sleep(0.5)
            continue
        time.sleep(2.5)  # motion + 0.5s camera shake settle (eye-in-hand)
        if not verify:
            print("[OK] arm at observation pose (unverified)")
            return True
        try:
            ok = True
            for joint, expected in enumerate(OBSERVE_JOINTS[:5], 1):
                value = arm.Arm_serial_servo_read(joint)
                if value is None or abs(float(value) - expected) > 12:
                    ok = False
                    break
            if not ok:
                print(f"[WARN] pose-start not confirmed (try {attempt + 1}/3); retrying")
                continue
            print("[OK] arm at observation pose (verified)")
            return True
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] pose-start readback failed (try {attempt + 1}/3): {exc}")
            continue
    print("[FAIL] arm did not reach observation pose after 3 tries")
    return False


def open_camera(source):
    try:
        index = int(source)
    except (TypeError, ValueError):
        index = source
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        return None
    for _ in range(5):  # warm up auto-exposure
        cap.read()
    return cap


def synthetic_frame(kind):
    """Render a simple cube with a colored upper face (480x640 BGR)."""
    img = np.full((480, 640, 3), 190, dtype=np.uint8)
    bgr = {"blue": (255, 0, 0), "green": (0, 200, 0),
           "red": (0, 0, 255), "yellow": (0, 255, 255)}[kind]
    surfaces = [
        (np.int32([[220, 260], [320, 260], [320, 340], [220, 330]]),
         (70, 70, 70)),
        (np.int32([[320, 260], [420, 260], [420, 330], [320, 340]]),
         (110, 110, 110)),
        (np.int32([[250, 130], [390, 130], [420, 260], [220, 260]]), bgr),
    ]
    for polygon, color in surfaces:
        cv2.fillConvexPoly(img, polygon, color)
        cv2.polylines(img, [polygon], True, (0, 0, 0), 3)
    return img


def detect_frame(frame, tag_detector, trash_detector, fast_known=False):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = tag_detector.detect(gray)
    verdicts, _ = identify_frame(frame, tags, trash_detector, annotate=False,
                                 fast_known=fast_known)
    return verdicts


def append_jsonl(path, observations, timestamp):
    if path is None:
        return
    with path.open("a", encoding="utf-8") as stream:
        for observation in observations:
            stream.write(json.dumps({"timestamp": timestamp,
                                     **observation.to_dict()}) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--camera", default="0",
                        help="camera index or /dev/videoN (default: 0)")
    parser.add_argument("--dry-run", action="store_true",
                        help="skip arm motion (default when no serial)")
    parser.add_argument("--skip-pose", action="store_true",
                        help="skip pose-start motion (vision only, no grasp)")
    parser.add_argument("--synthetic", choices=["blue", "green", "red",
                                                "yellow"],
                        help="use a rendered cube, no camera")
    parser.add_argument("--image", type=Path,
                        help="classify one saved camera frame, no arm or camera")
    parser.add_argument("--output", type=Path,
                        default=Path("/tmp/identify_cube_demo.png"),
                        help="annotated output for --image or --synthetic")
    parser.add_argument("--jsonl-output", type=Path,
                        help="append structured cube observations (pixel geometry only)")
    parser.add_argument("--contour-report", type=Path,
                        help="with --image, save raw frame and contour metrics at this path prefix")
    parser.add_argument("--sim-thresh", type=float, default=0.40,
                        help="DINO cosine threshold for trash cue")
    parser.add_argument("--sim-margin", type=float, default=0.0,
                        help="minimum gap between DINO top two classes")
    parser.add_argument("--no-trash", action="store_true",
                        help="disable trash cue (DINO)")
    parser.add_argument("--device", default="auto",
                        choices=["auto", "cpu", "cuda"],
                        help="DINO inference device")
    parser.add_argument("--model", default="dinov2_vits14",
                        choices=["dinov2_vits14", "dinov2_vitb14", "dinov2_vitl14", "dinov2_vitg14"],
                        help="DINOv2 model architecture")
    args = parser.parse_args()

    if args.synthetic and args.image:
        parser.error("--synthetic and --image cannot be used together")
    if args.contour_report and not args.image:
        parser.error("--contour-report requires --image")
    if not 0 <= args.sim_thresh <= 1 or not 0 <= args.sim_margin <= 1:
        parser.error("--sim-thresh and --sim-margin must be in [0, 1]")

    if not (args.synthetic or args.image or args.skip_pose) and not move_to_observe(
            dry_run=args.dry_run):
        return 1
    if args.skip_pose and not (args.synthetic or args.image):
        print("[WARN] --skip-pose: scanning without pose-start (no grasp)")

    tag_detector = TagDetector()
    trash_detector = None if args.no_trash else TrashDetector(
        thresh=args.sim_thresh, device=args.device,
        margin=args.sim_margin, model_name=args.model)

    if args.synthetic or args.image:
        frame = synthetic_frame(args.synthetic) if args.synthetic else \
            cv2.imread(str(args.image))
        if frame is None:
            print(f"[FAIL] cannot read image {args.image}")
            return 1
        if args.contour_report:
            raw, report = save_contour_debug(frame, args.contour_report)
            print(f"[CONTOUR] raw={raw} metrics={report}")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = tag_detector.detect(gray)
        observations, annotated = identify_frame(frame, tags, trash_detector)
        for observation in observations:
            verdict = observation.verdict
            print(f"CUBE ID: {verdict.cube_id} ({verdict.name}) "
                  f"via {verdict.via} label={verdict.label} "
                  f"score={verdict.score:.3f} box={observation.group.box} "
                  f"top_center_px={observation.top_center_px} "
                  f"geometry_valid={observation.geometry_valid} "
                  f"reason={verdict.reason or observation.group.geometry_reason or '-'} "
                  f"DINO_Top2={[(name, round(score * 100, 1)) for name, score in verdict.trash_top[:2]]} "
                  f"DINO_rotation={verdict.trash_rotation_deg}")
        append_jsonl(args.jsonl_output, observations, time.time())
        cv2.imwrite(str(args.output), annotated)
        print(f"[OK] annotated frame -> {args.output}")
        if not observations:
            print("CUBE ID: none (no cube group)")
            return 2
        return 0

    cap = open_camera(args.camera)
    if cap is None:
        print(f"[FAIL] cannot open camera {args.camera}")
        return 1
    print("[OK] camera open 640x480. identify_cube chỉ nhận diện; "
          "để gắp hãy chạy cube_sort_stage1.py. 's': lưu ảnh, 'q': thoát.")
    averager = OneSecondAverager()
    executor = ThreadPoolExecutor(max_workers=1)
    pending = None
    pending_capture_at = 0.0
    tracked = []
    last_result_at = 0.0
    no_face_reported = False
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] frame grab failed")
                cv2.waitKey(1)
                continue
            if (frame.shape[1], frame.shape[0]) != FRAME_SIZE:
                frame = cv2.resize(frame, FRAME_SIZE)
            if pending is not None and pending.done():
                observations = pending.result()
                last_result_at = pending_capture_at
                tracked, reports = averager.update(observations, last_result_at)
                pending = None
                if not tracked and not no_face_reported:
                    print("CUBE ID: none (no cube candidate)")
                no_face_reported = not tracked
                for track, verdict in reports:
                    snapshot = track.observation
                    observation = CubeObservation(snapshot.group, verdict,
                                                  snapshot.face_verdicts)
                    print(f"CUBE ID: {verdict.cube_id} ({verdict.name}) "
                          f"via {verdict.via} label={verdict.label} "
                          f"score={verdict.score:.3f} box={observation.group.box} "
                          f"top_center_px={observation.top_center_px} "
                          f"geometry_valid={observation.geometry_valid} "
                          f"reason={verdict.reason or observation.group.geometry_reason or '-'} "
                          f"DINO_Top2={[(name, round(score * 100, 1)) for name, score in verdict.trash_top[:2]]} "
                          f"DINO_rotation={verdict.trash_rotation_deg} (ID held, geometry live)")
                    append_jsonl(args.jsonl_output, [observation], time.time())
            if pending is None:
                pending_capture_at = time.monotonic()
                pending = executor.submit(detect_frame, frame.copy(),
                                          tag_detector, trash_detector)
            if time.monotonic() - last_result_at > 2.0:
                tracked = []
            annotated = draw_tracked_frame(frame, tracked)
            cv2.imshow("identify_cube (q to quit)", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                path = Path(f"/tmp/identify_cube_raw_{int(time.time())}.png")
                cv2.imwrite(str(path), frame)
                print(f"[OK] raw camera frame -> {path}")
            if key == ord("d"):
                raw, report = save_contour_debug(
                    frame, Path(f"/tmp/identify_cube_contour_{int(time.time())}"))
                print(f"[CONTOUR] raw={raw} metrics={report}")
            if key == ord(" "):
                print("[INFO] Space không điều khiển tay trong identify_cube. "
                      "Chạy: .venv/bin/python projects/vision_experiments/"
                      "cube_sort_stage1.py --camera /dev/video2")
            if key == ord("q"):
                break
    except KeyboardInterrupt:
        print("[OK] stopped by Ctrl+C")
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
