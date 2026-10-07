"""Calibrate RGB intrinsics and eye-in-hand extrinsics from a fixed AprilTag.

The tag must remain rigidly fixed for the complete dataset.  Samples contain
an image and the timestamp-matched ``base_T_mount`` transform.  Fit and held-
out samples are separated so a low training residual cannot silently enable
hardware.  The command always writes a new scene config.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from cap_vision.red_scene_core import load_config, transform


def tag_points(size_m):
    side = float(size_m) / 2.0
    if not np.isfinite(side) or side <= 0:
        raise ValueError("tag size must be positive")
    return np.array([[-side, -side, 0.0], [side, -side, 0.0],
                     [side, side, 0.0], [-side, side, 0.0]], np.float32)


def detect_corners(image, detector, tag_id):
    tags = [tag for tag in detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            if int(tag.tag_id) == int(tag_id)]
    if len(tags) != 1:
        raise ValueError(f"expected exactly one tag {tag_id}, found {len(tags)}")
    return np.asarray(tags[0].corners, np.float32).reshape(4, 2)


def _pose(points, corners, K, distortion):
    solutions = cv2.solvePnPGeneric(points, corners, K, distortion,
                                    flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if solutions[0]:
        for rvec, tvec in zip(solutions[1], solutions[2]):
            if float(tvec[2, 0]) <= 0:
                continue
            projected = cv2.projectPoints(points, rvec, tvec, K, distortion)[0].reshape(4, 2)
            error = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1))))
            matrix = np.eye(4)
            matrix[:3, :3] = cv2.Rodrigues(rvec)[0]
            matrix[:3, 3] = tvec.ravel()
            candidates.append((error, matrix))
    if not candidates:
        raise ValueError("AprilTag PnP failed")
    return min(candidates, key=lambda item: item[0])


def solve(observations, image_size, tag_size_m, initial_K=None):
    """Return K, distortion, mount_T_camera, base_T_tag and validation data."""
    fit = [sample for sample in observations if not sample.get("validation", False)]
    held = [sample for sample in observations if sample.get("validation", False)]
    if len(fit) < 8 or len(held) < 3:
        raise ValueError("need >=8 fit and >=3 held-out tag observations")
    points = tag_points(tag_size_m)
    object_sets = [points.copy() for _ in fit]
    image_sets = [np.asarray(sample["corners"], np.float32).reshape(4, 2) for sample in fit]
    width, height = map(int, image_size)
    if initial_K is None:
        f = float(max(width, height))
        K0 = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0],
                       [0.0, 0.0, 1.0]], np.float64)
    else:
        K0 = np.asarray(initial_K, np.float64).reshape(3, 3).copy()
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
    rms, K, distortion, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        object_sets, image_sets, (width, height), K0, np.zeros(5), flags=flags)
    per_view = np.asarray(per_view).ravel()
    if (not np.isfinite(rms) or rms > 0.8 or np.max(per_view) > 1.2 or
            not 0.5 * width < K[0, 0] < 4.0 * width or
            not 0.5 * height < K[1, 1] < 4.0 * height or
            not 0.2 * width < K[0, 2] < 0.8 * width or
            not 0.2 * height < K[1, 2] < 0.8 * height):
        raise ValueError(f"intrinsic gate failed: RMS={rms:.3f}px, max={np.max(per_view):.3f}px")

    pairs = []
    reprojection = []
    for sample in observations:
        error, camera_T_tag = _pose(points, np.asarray(sample["corners"], np.float32),
                                    K, distortion)
        if error > 1.5:
            raise ValueError(f"tag reprojection {error:.3f}px > 1.5px")
        pairs.append((transform(sample["base_T_mount"]), camera_T_tag,
                      bool(sample.get("validation", False))))
        reprojection.append(error)
    train = [(mount, tag) for mount, tag, validation in pairs if not validation]
    rotations = np.array([cv2.Rodrigues(train[0][0][:3, :3].T @ mount[:3, :3])[0].ravel()
                          for mount, _ in train[1:]])
    singular = np.linalg.svd(rotations, compute_uv=False)
    if len(singular) < 2 or singular[1] < 0.15:
        raise ValueError("insufficient wrist rotation diversity for hand-eye")
    rotation, translation = cv2.calibrateHandEye(
        [mount[:3, :3] for mount, _ in train],
        [mount[:3, 3] for mount, _ in train],
        [tag[:3, :3] for _, tag in train],
        [tag[:3, 3] for _, tag in train], method=cv2.CALIB_HAND_EYE_PARK)
    mount_T_camera = np.eye(4)
    mount_T_camera[:3, :3] = rotation
    mount_T_camera[:3, 3] = translation.ravel()
    transform(mount_T_camera)

    fitted_tags = [mount @ mount_T_camera @ tag for mount, tag in train]
    u, _, vt = np.linalg.svd(np.mean([pose[:3, :3] for pose in fitted_tags], axis=0))
    base_T_tag = np.eye(4)
    base_T_tag[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    base_T_tag[:3, 3] = np.mean([pose[:3, 3] for pose in fitted_tags], axis=0)
    errors = []
    reference = points @ base_T_tag[:3, :3].T + base_T_tag[:3, 3]
    for mount, tag, validation in pairs:
        if not validation:
            continue
        candidate = mount @ mount_T_camera @ tag
        measured = points @ candidate[:3, :3].T + candidate[:3, 3]
        errors.append(float(np.max(np.linalg.norm(measured - reference, axis=1))))
    if max(errors) > 0.005:
        raise ValueError(f"held-out tag error {max(errors) * 1000:.1f}mm > 5mm")
    return {"K": K, "distortion": distortion.ravel(), "intrinsic_rms_px": float(rms),
            "per_view_rms_px": per_view.tolist(), "mount_T_camera": mount_T_camera,
            "base_T_tag": base_T_tag, "held_out_errors_m": errors,
            "reprojection_errors_px": reprojection,
            "rotation_singular": singular.tolist()}


def detector():
    try:
        from dt_apriltags import Detector
    except ImportError:
        from pupil_apriltags import Detector
    return Detector(families="tag36h11", nthreads=4, quad_decimate=1.0,
                    refine_edges=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if Path(args.output).exists():
        raise SystemExit("output exists; choose a new calibration file")
    dataset = load_config(args.dataset)
    cfg = load_config(args.config)
    target = dataset.get("target", {})
    if target.get("type") != "apriltag":
        raise SystemExit("dataset target.type must be apriltag")
    tag_id, tag_size = int(target["id"]), float(target["size_m"])
    root = Path(args.dataset).resolve().parent
    observations = []
    tag_detector = detector()
    shape = None
    for raw in dataset["samples"]:
        image = cv2.imread(str(root / raw["image"]))
        if image is None:
            raise SystemExit(f"missing image: {raw['image']}")
        if shape is None:
            shape = image.shape[:2]
        if image.shape[:2] != shape:
            raise SystemExit("mixed image resolutions")
        observations.append({**raw, "corners": detect_corners(image, tag_detector, tag_id)})
    camera = cfg["camera"]
    result = solve(observations, (shape[1], shape[0]), tag_size, camera.get("K"))
    camera.update(source="fixed_apriltag_eye_in_hand",
                  width=shape[1], height=shape[0], K=result["K"].ravel().tolist(),
                  distortion=result["distortion"].tolist(), calibrated=True,
                  mount_T_optical=result["mount_T_camera"].ravel().tolist(),
                  extrinsic_calibrated=True)
    # A tag on the upward cube face is tag_to_table_m above the support plane.
    tag_to_table = float(target.get("tag_to_table_m", 0.0))
    table = result["base_T_tag"].copy()
    table[:3, 3] -= table[:3, 2] * tag_to_table
    cfg["table"].update(base_T_table=table.ravel().tolist(), calibrated=True)
    cfg["calibration_validation"] = {
        "method": "fixed_apriltag_eye_in_hand",
        "dataset": str(Path(args.dataset).resolve()),
        "tag_id": tag_id,
        "tag_size_m": tag_size,
        "intrinsic_rms_px": result["intrinsic_rms_px"],
        "held_out_errors_m": result["held_out_errors_m"],
        "rotation_singular": result["rotation_singular"],
    }
    with open(args.output, "x", encoding="utf-8") as stream:
        yaml.safe_dump(cfg, stream, sort_keys=False)
    print(f"Saved {args.output}; intrinsic RMS={result['intrinsic_rms_px']:.3f}px; "
          f"held-out max={max(result['held_out_errors_m']) * 1000:.2f}mm")


if __name__ == "__main__":
    main()
