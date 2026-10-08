"""Pose from labelled, adjacent cube faces in one RGB image.

The labels locate faces in the cube frame; the visible quadrilaterals provide
metric 2D/3D correspondences. A single unregistered square is deliberately
insufficient to fix the cube's rotation around its normal.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from cap_vision.object_pipeline import CUBE_NAMES, HSV, CubeModel, PoseEstimate, rigid


AXES = {"+X": (1, 0, 0), "-X": (-1, 0, 0),
        "+Y": (0, 1, 0), "-Y": (0, -1, 0),
        "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


def _quat_matrix(value):
    q = np.asarray(value, float).reshape(4)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError("face orientation must be a unit xyzw quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def load_face_models(path, reference_models=None):
    """Validate the owner's geometry file and return cube models.

    `object_models.yaml` may be supplied for compatibility checks; no geometry
    is silently overridden if the two declarations disagree.
    """
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("units") != "meter":
        raise ValueError("unsupported cube face geometry schema/units")
    geometry = data["geometry"]
    if not np.allclose(geometry["cube_size_m"], [0.03]*3, atol=1e-6):
        raise ValueError("expected measured 30 mm cube")
    half = float(geometry["cube_half_extent_m"])
    tag_size = float(geometry["apriltag_black_square_size_m"])
    if abs(half - 0.015) > 1e-6 or abs(tag_size - 0.020) > 1e-6:
        raise ValueError("cube half extent or AprilTag size is inconsistent")
    vertices = np.asarray(geometry["vertices_xyz_m"], float)
    if vertices.shape != (8, 3) or len({tuple(v) for v in vertices}) != 8 or not np.allclose(np.abs(vertices), half):
        raise ValueError("cube vertex list is inconsistent")
    tag_corners = np.asarray(list(geometry["apriltag_local_corners_on_plus_z_xyz_m"].values()), float)
    if tag_corners.shape != (4, 3) or not np.allclose(tag_corners[:, 2], half) or not np.allclose(np.abs(tag_corners[:, :2]), tag_size/2):
        raise ValueError("AprilTag corners are inconsistent")
    # The YAML lists tag corners in image-style top-left order. The current
    # AprilTag PnP uses the detector's established corner convention, so check
    # the same four physical points without changing their runtime ordering.
    if set(map(tuple, np.round(tag_corners, 6))) != {
            (-tag_size/2, -tag_size/2, half), (tag_size/2, -tag_size/2, half),
            (tag_size/2, tag_size/2, half), (-tag_size/2, tag_size/2, half)}:
        raise ValueError("AprilTag corner set is inconsistent")
    specs = data["cubes"]
    if {int(i) for i in specs} != set(CUBE_NAMES):
        raise ValueError("expected exactly four cube IDs")
    models = {}
    all_trash = set()
    for raw_id, spec in specs.items():
        cube_id = int(raw_id)
        if spec["name"] != CUBE_NAMES[cube_id] or not np.allclose(spec["size_m"], [2*half]*3):
            raise ValueError(f"cube {cube_id} name/size mismatch")
        faces = spec["faces"]
        if set(faces) != set(AXES):
            raise ValueError(f"cube {cube_id} needs six faces")
        labels = {}
        for axis, normal in AXES.items():
            face = faces[axis]
            n = np.asarray(normal, float)
            if (not np.allclose(face["center_xyz_m"], half*n, atol=1e-6) or
                    not np.allclose(face["normal_xyz"], n, atol=1e-6) or
                    not np.allclose(face["surface_size_m"], [2*half]*2, atol=1e-6) or
                    not np.allclose(_quat_matrix(face["orientation_xyzw"]) @ [0, 0, 1], n, atol=1e-6)):
                raise ValueError(f"cube {cube_id} face {axis} transform mismatch")
            semantic = "apriltag" if axis == "+Z" else "color" if axis == "-Z" else "trash"
            if face["semantic"] != semantic:
                raise ValueError(f"cube {cube_id} face {axis} semantic mismatch")
            if axis == "+Z":
                if int(face["apriltag_id"]) != cube_id or not np.allclose(face["black_square_size_m"], [tag_size]*2):
                    raise ValueError(f"cube {cube_id} tag mismatch")
            else:
                label = str(face["label"]).lower()
                if label in labels.values() or (semantic == "trash" and label in all_trash):
                    raise ValueError(f"duplicate face label {label}")
                labels[axis] = label
                if semantic == "trash":
                    all_trash.add(label)
        cube_T_tag = np.eye(4)
        cube_T_tag[:3, 3] = [0, 0, half]
        base = CubeModel.measured(cube_id, 2*half, tag_size, cube_T_tag)
        model = CubeModel(base.object_type, base.dimensions, base.coordinate_frame,
                          base.grasp_surfaces, base.tag_id, base.tag_size_m,
                          base.tag_T_cube, labels)
        if reference_models is not None:
            old = reference_models[cube_id]
            if (not np.isclose(old.dimensions[0], model.dimensions[0]) or
                    not np.isclose(old.tag_size_m, model.tag_size_m) or
                    not np.allclose(old.tag_T_cube, model.tag_T_cube)):
                raise ValueError(f"cube {cube_id} differs from object_models.yaml")
        models[cube_id] = model
    return models


def order_quad(points):
    points = np.asarray(points, np.float32).reshape(4, 2)
    center = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(points[:, 1]-center[1], points[:, 0]-center[0]))]
    return np.roll(points, -int(np.argmin(points.sum(axis=1))), axis=0)


def face_corners(model, axis):
    """Four cube-frame vertices in a consistent face-local cyclic order."""
    n = np.asarray(AXES[axis], float)
    up = np.array([0., 0., 1.]) if axis not in ("+Z", "-Z") else np.array([0., 1., 0.])
    right = np.cross(up, n)
    up = np.cross(n, right)
    h = model.dimensions[0] / 2
    center = h*n
    return np.array([center + h*(a*right+b*up) for a, b in
                     ((-1,-1),(1,-1),(1,1),(-1,1))], np.float64)


def semantic_patch_corners(model, axis):
    """3D corners of the visual region classified by HSV/DINO.

    The colour side fills its 30 mm face. Trash artwork is printed in the
    centred 20 mm region (the same measured span as the tag black square), as
    confirmed by the 2:3 patch/face ratio in the commissioning frames.
    """
    corners = face_corners(model, axis)
    label = model.semantic_faces.get(axis, "")
    if axis == "-Z" or label.startswith("khoi_"):
        return corners
    center = np.mean(corners, axis=0)
    scale = float(model.tag_size_m / model.dimensions[0])
    return center + scale * (corners - center)


def observation_corner_variants(model, axis):
    """Possible physical extents represented by a detected trash quad."""
    full = face_corners(model, axis)
    patch = semantic_patch_corners(model, axis)
    return [full] if np.allclose(full, patch) else [full, patch]


@dataclass(frozen=True)
class FaceObservation:
    axis: str
    corners: np.ndarray
    confidence: float


def extract_face_quads(bgr, instance, min_area=250):
    """Return face-sized quadrilaterals inside an instance mask."""
    mask = np.asarray(instance.mask, np.uint8)
    if mask.shape != bgr.shape[:2]:
        return []
    x1, y1, x2, y2 = instance.bbox
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(mask.shape[1], x2), min(mask.shape[0], y2)
    if x2 <= x1 or y2 <= y1:
        return []
    crop = bgr[y1:y2, x1:x2]
    local = mask[y1:y2, x1:x2]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 130)
    edges = cv2.bitwise_and(edges, cv2.dilate(local, np.ones((3,3), np.uint8)))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    contours += cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    found = []
    area_max = float(np.count_nonzero(local))
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        quad = cv2.approxPolyDP(contour, 0.03*perimeter, True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            continue
        area = abs(cv2.contourArea(quad))
        if not min_area <= area <= 1.05*area_max:
            continue
        poly = np.zeros(local.shape, np.uint8)
        cv2.fillConvexPoly(poly, quad.reshape(4,2), 1)
        overlap = np.count_nonzero(poly & local) / max(1, np.count_nonzero(poly))
        if overlap < 0.85:
            continue
        points = order_quad(quad.reshape(4,2) + [x1,y1])
        if min(np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)) < 14:
            continue
        if any(cv2.contourArea(cv2.convexHull(np.vstack([points, old]))) <
               1.12*max(area, cv2.contourArea(old.astype(np.float32))) for old in found):
            continue
        found.append(points)
    return sorted(found, key=lambda p: -cv2.contourArea(p))[:6]


def extract_scene_face_quads(bgr, min_area=250, max_candidates=24):
    """Find plausible printed/colour cube faces before an object mask exists.

    This is deliberately only a proposal stage.  A candidate becomes an
    object seed only after HSV or DINO assigns it one of the configured face
    labels; arbitrary squares in the scene therefore do not become cubes.
    """
    image = np.asarray(bgr)
    if image.ndim != 3 or image.shape[2] != 3:
        return []
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 35, 120)
    # Printed faces often have a pale outer edge. Closing joins its four sides.
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                             np.ones((5, 5), np.uint8), iterations=2)
    contours = list(cv2.findContours(edges, cv2.RETR_LIST,
                                     cv2.CHAIN_APPROX_SIMPLE)[0])
    # Solid colour faces can be almost textureless and therefore have weak
    # Canny edges. Add contours from the four configured HSV ranges directly.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    for ranges in HSV.values():
        selected = np.zeros((height, width), np.uint8)
        for low, high in ranges:
            selected |= cv2.inRange(hsv, np.array(low), np.array(high))
        selected = cv2.morphologyEx(selected, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))
        selected = cv2.morphologyEx(selected, cv2.MORPH_CLOSE,
                                    np.ones((7, 7), np.uint8), iterations=2)
        contours.extend(cv2.findContours(selected, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[0])
        count, labels, stats, _ = cv2.connectedComponentsWithStats(selected)
        for component in range(1, count):
            x, y, w, h, area = stats[component]
            if (area < min_area or min(w, h) < 14 or
                    area / max(1, w * h) < 0.06):
                continue
            ys, xs = np.nonzero(labels == component)
            points = np.column_stack((xs, ys)).astype(np.float32)
            contours.append(cv2.boxPoints(cv2.minAreaRect(points)).astype(np.int32)
                            .reshape(-1, 1, 2))
    found = []
    max_area = 0.18 * height * width
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 60:
            continue
        quad = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            continue
        points = order_quad(quad.reshape(4, 2))
        area = abs(cv2.contourArea(points))
        if not min_area <= area <= max_area:
            continue
        sides = np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)
        if sides.min() < 14 or sides.max() / max(1.0, sides.min()) > 3.2:
            continue
        x, y, w, h = cv2.boundingRect(points.astype(np.float32))
        if area / max(1.0, w * h) < 0.32:
            continue
        # Suppress the many near-identical contours around one printed face.
        if any(abs(cv2.contourArea(old)) > 0 and
               min(area, abs(cv2.contourArea(old))) /
               max(area, abs(cv2.contourArea(old))) >= 0.75 and
               cv2.contourArea(cv2.convexHull(np.vstack([points, old]))) <
               1.18 * max(area, abs(cv2.contourArea(old))) for old in found):
            continue
        found.append(points)
    return sorted(found, key=lambda p: -abs(cv2.contourArea(p)))[:max_candidates]


def enclosing_white_face_quad(bgr, core_quad, candidates, anchor,
                              white_s_max=70, white_v_min=140,
                              max_expansion=2.5):
    """Return a measured outer face, never an extrapolated printed contour.

    The printed graphic is the anchor.  A second, closed contour must enclose
    it and contain a substantial white annulus.  The 20/30 mm print-to-face
    ratio limits the search; failure is explicitly an unverified proposal.
    """
    core = order_quad(core_quad)
    core_area = abs(cv2.contourArea(core))
    if core_area < 250:
        return None
    h, w = bgr.shape[:2]
    ax, ay = map(float, anchor)
    if cv2.pointPolygonTest(core, (ax, ay), False) < 0:
        return None
    best = None
    for raw in candidates:
        outer = order_quad(raw)
        area = abs(cv2.contourArea(outer))
        ratio = area / core_area
        if not 1.18 <= ratio <= max_expansion ** 2:
            continue
        if cv2.pointPolygonTest(outer, (ax, ay), False) < 0:
            continue
        diagonal = float(np.linalg.norm(outer.max(axis=0) - outer.min(axis=0)))
        if np.linalg.norm(outer.mean(axis=0) - [ax, ay]) > 0.35 * diagonal:
            continue
        x, y, bw, bh = cv2.boundingRect(outer)
        if x < 1 or y < 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue
        outer_mask = np.zeros((h, w), np.uint8)
        inner_mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(outer_mask, np.rint(outer).astype(np.int32), 1)
        cv2.fillConvexPoly(inner_mask, np.rint(core).astype(np.int32), 1)
        if np.count_nonzero(inner_mask & outer_mask) < 0.8 * np.count_nonzero(inner_mask):
            continue
        ring = outer_mask.astype(bool) & ~cv2.dilate(inner_mask, np.ones((5, 5), np.uint8)).astype(bool)
        if np.count_nonzero(ring) < 100:
            continue
        hsv = cv2.cvtColor(bgr[y:y+bh, x:x+bw], cv2.COLOR_BGR2HSV)
        white = (hsv[:, :, 1] <= white_s_max) & (hsv[:, :, 2] >= white_v_min)
        coverage = float(np.mean(white[ring[y:y+bh, x:x+bw]]))
        if coverage < 0.58:
            continue
        score = coverage - 0.12 * abs(ratio - 2.25)
        if best is None or score > best[0]:
            best = (score, outer)
    return None if best is None else best[1]


def face_seed_instances(shape, labelled_quads, expansion=0.35):
    """Build coarse object proposals around semantically recognised faces.

    `labelled_quads` contains ``(cube_id, quad, confidence)``. Overlapping
    faces of the same physical cube are merged. These masks are intentionally
    coarse: their only role is to unblock face extraction and metric PnP when
    open-vocabulary segmentation misses a cube.
    """
    from cap_vision.object_pipeline import ObjectInstance

    height, width = shape[:2]
    proposals = []
    for proposal in labelled_quads:
        cube_id, raw_quad, confidence = proposal[:3]
        anchor_center = proposal[3] if len(proposal) > 3 else None
        source = proposal[4] if len(proposal) > 4 else "semantic_face"
        seed_label = proposal[5] if len(proposal) > 5 else ""
        quad = np.asarray(raw_quad, np.float32).reshape(4, 2)
        center = quad.mean(axis=0)
        expanded_quad = center + (1.0 + 2.0 * expansion) * (quad - center)
        expanded_quad[:, 0] = np.clip(expanded_quad[:, 0], 0, width - 1)
        expanded_quad[:, 1] = np.clip(expanded_quad[:, 1], 0, height - 1)
        x, y, w, h = cv2.boundingRect(expanded_quad.astype(np.int32))
        box = [x, y, x + w, y + h]
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        mask = np.zeros((height, width), np.uint8)
        cv2.fillConvexPoly(mask, np.rint(expanded_quad).astype(np.int32), 1)
        proposals.append((int(cube_id), float(confidence), anchor_center,
                          str(source), str(seed_label), expanded_quad, mask.astype(bool)))
    instances = []
    for cube_id, confidence, anchor_center, source, seed_label, quad, mask in proposals:
        # Collapse nearly identical contours, never adjacent physical faces.
        duplicate = next((item for item in instances
                          if getattr(item, "seed_cube_id", None) == cube_id and
                          np.count_nonzero(item.mask & mask) /
                          max(1, np.count_nonzero(item.mask | mask)) >= 0.70), None)
        if duplicate is not None:
            if source == "trash_outer_face" or (duplicate.proposal_source !=
                    "trash_outer_face" and confidence > duplicate.confidence):
                duplicate.mask = mask
                duplicate.proposal_source = source
                duplicate.anchor_center = anchor_center
                duplicate.confidence = confidence
                duplicate.seed_label = seed_label
                duplicate.seed_quad = quad
                ys, xs = np.nonzero(mask)
                duplicate.bbox = (int(xs.min()), int(ys.min()),
                                  int(xs.max()) + 1, int(ys.max()) + 1)
            continue
        ys, xs = np.nonzero(mask)
        instances.append(ObjectInstance((int(xs.min()), int(ys.min()),
                                         int(xs.max()) + 1, int(ys.max()) + 1), mask,
                                        float(np.clip(confidence, 0.45, 0.95)),
                                        proposal_source=str(source),
                                        anchor_center=anchor_center,
                                        seed_cube_id=cube_id,
                                        seed_label=seed_label,
                                        seed_quad=quad))
    return instances


def rectify_quad(bgr, points, size=224):
    dst = np.float32([[0,0],[size-1,0],[size-1,size-1],[0,size-1]])
    return cv2.warpPerspective(bgr, cv2.getPerspectiveTransform(order_quad(points), dst), (size,size))


class CubeFacePoseEstimator:
    def __init__(self, max_reprojection_px=2.0, ambiguity_margin_px=1.0):
        self.max_reprojection_px = max_reprojection_px
        self.ambiguity_margin_px = ambiguity_margin_px

    def estimate(self, observations, model, K, distortion):
        """Require two adjacent, distinctly labelled faces for full 6D."""
        obs = [o for o in observations if o.axis in model.semantic_faces and
               np.asarray(o.corners).shape == (4, 2) and
               np.isfinite(o.corners).all() and
               abs(cv2.contourArea(np.asarray(o.corners, np.float32))) >= 250 and
               o.confidence >= 0.55]
        obs = sorted(obs, key=lambda o: -o.confidence)
        unique = []
        for item in obs:
            if item.axis not in {o.axis for o in unique}:
                unique.append(item)
        obs = unique[:3]
        if len(obs) < 2 or not any(np.dot(AXES[a.axis], AXES[b.axis]) == 0
                                    for i,a in enumerate(obs) for b in obs[i+1:]):
            return None
        K = np.asarray(K, np.float64).reshape(3,3)
        D = np.asarray(distortion, np.float64)
        image = [order_quad(o.corners).astype(np.float64) for o in obs]
        # A visible outward-facing polygon has opposite winding in the
        # top-left-origin image coordinate system.
        candidates = []
        import itertools
        variants = [observation_corner_variants(model, o.axis) for o in obs]
        for selected_faces in itertools.product(*variants):
            object_faces = [points[::-1] for points in selected_faces]
            for shifts in itertools.product(range(4), repeat=len(obs)):
                obj = np.vstack([np.roll(points, shift, axis=0) for points, shift in zip(object_faces, shifts)])
                img = np.vstack(image)
                try:
                    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_EPNP)
                except cv2.error:
                    continue
                if not ok or tvec[2,0] <= 0:
                    continue
                try:
                    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
                except cv2.error:
                    continue
                R = cv2.Rodrigues(rvec)[0]
                if any(np.dot(R @ AXES[o.axis], tvec.ravel() + R @ (np.asarray(AXES[o.axis])*model.dimensions[0]/2)) >= 0
                       for o in obs):
                    continue
                projected = cv2.projectPoints(obj, rvec, tvec, K, D)[0].reshape(-1,2)
                error = float(np.sqrt(np.mean(np.sum((projected-img)**2, axis=1))))
                T = np.eye(4)
                T[:3,:3], T[:3,3] = R, tvec.ravel()
                candidates.append((error, rigid(T)))
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        best_error, best_T = candidates[0]
        if best_error > self.max_reprojection_px:
            return None
        for error, matrix in candidates[1:]:
            delta = best_T[:3,:3].T @ matrix[:3,:3]
            angle = np.degrees(np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1)))
            if angle > 15 and error-best_error < self.ambiguity_margin_px:
                return None
        confidence = float(np.clip(0.95 - best_error/4, 0.5, 0.95))
        return PoseEstimate(best_T, confidence, best_error, "rgb_faces_pnp", orientation_valid=True)

    def estimate_single_face(self, observation, model, K, distortion, bgr=None):
        """Recover cube geometry from one labelled face, modulo 90-degree yaw.

        A cube wireframe and its centre are invariant under that ambiguity, so
        this estimate is useful for image overlay/TCP targeting. It is marked
        orientation-invalid and must never be promoted to semantic 6D grasp
        pose until a second adjacent labelled face is observed.
        """
        if (observation.axis not in model.semantic_faces or
                np.asarray(observation.corners).shape != (4, 2) or
                observation.confidence < 0.55):
            return None
        K = np.asarray(K, np.float64).reshape(3, 3)
        D = np.asarray(distortion, np.float64)
        image = order_quad(observation.corners).astype(np.float64)
        distance = None
        if bgr is not None:
            gray = cv2.cvtColor(np.asarray(bgr), cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 40, 130)
            distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
        candidates = []
        for raw_face in observation_corner_variants(model, observation.axis):
          face = raw_face[::-1]
          for shift in range(4):
            obj = np.roll(face, shift, axis=0)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, image, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            except cv2.error:
                continue
            if not ok:
                continue
            for rvec, tvec in ((rvec, tvec),):
                if tvec[2, 0] <= 0:
                    continue
                R = cv2.Rodrigues(rvec)[0]
                # A single un-oriented artwork quad does not provide a
                # trustworthy winding sign. Keep both planar mirror branches;
                # image-edge support below chooses the physical cube side.
                projected = cv2.projectPoints(obj, rvec, tvec, K, D)[0].reshape(-1, 2)
                error = float(np.sqrt(np.mean(np.sum((projected-image)**2, axis=1))))
                T = np.eye(4)
                T[:3, :3], T[:3, 3] = R, tvec.ravel()
                edge_error = 0.0
                if distance is not None:
                    vertices = (R @ model.vertices().T).T + tvec.ravel()
                    pixels = cv2.projectPoints(vertices, np.zeros(3), np.zeros(3),
                                                K, D)[0].reshape(-1, 2)
                    samples = []
                    for a in range(8):
                        for bit in (1, 2, 4):
                            b = a ^ bit
                            if a >= b:
                                continue
                            for alpha in np.linspace(0.0, 1.0, 9):
                                point = (1-alpha)*pixels[a] + alpha*pixels[b]
                                x, y = np.rint(point).astype(int)
                                samples.append(float(distance[y, x])
                                               if 0 <= y < distance.shape[0] and
                                               0 <= x < distance.shape[1] else 25.0)
                    edge_error = float(np.mean(np.clip(samples, 0, 25)))
                candidates.append((error + 0.20 * edge_error, error, rigid(T)))
        if not candidates:
            return None
        _score, error, matrix = min(candidates, key=lambda item: item[0])
        if error > max(self.max_reprojection_px, 3.0):
            return None
        confidence = float(np.clip(0.80 - error / 5, 0.45, 0.80))
        return PoseEstimate(matrix, confidence, error, "rgb_single_face_cube",
                            orientation_valid=False,
                            symmetry_axis_local=tuple(float(v) for v in AXES[observation.axis]))
