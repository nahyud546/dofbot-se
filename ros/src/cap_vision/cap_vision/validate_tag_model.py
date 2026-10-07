"""Detect AprilTags and overlay the configured 3D cube wireframe on one image."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from cap_vision.object_pipeline import AprilTagPoseEstimator, CubeModel


EDGES = [(a, a ^ bit) for a in range(8) for bit in (1, 2, 4) if a < (a ^ bit)]


def load_camera(path):
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    camera = data.get("camera", data)
    K = camera.get("K", camera.get("camera_matrix"))
    distortion = camera.get("distortion", camera.get("distortion_coefficients"))
    if K is None or distortion is None:
        raise ValueError("camera YAML needs K/camera_matrix and distortion coefficients")
    return np.asarray(K, float).reshape(3, 3), np.asarray(distortion, float), bool(
        camera.get("calibrated", False))


def load_models(path):
    with open(path, encoding="utf-8") as stream:
        specs = yaml.safe_load(stream).get("cubes", {})
    models = {}
    for raw_id, spec in specs.items():
        if all(spec.get(key) is not None for key in ("size_m", "tag_size_m", "cube_T_tag")):
            cube_id = int(raw_id)
            models[cube_id] = CubeModel.measured(
                cube_id, float(spec["size_m"]), float(spec["tag_size_m"]), spec["cube_T_tag"])
    return models


def detector():
    try:
        from pupil_apriltags import Detector
    except ImportError:
        from dt_apriltags import Detector
    return Detector(families="tag36h11", nthreads=4, quad_decimate=1.0,
                    decode_sharpening=0.25)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True)
    parser.add_argument("--camera", required=True, help="camera-info or scene YAML")
    parser.add_argument("--models", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"cannot read image: {args.image}")
    K, distortion, calibrated = load_camera(args.camera)
    models = load_models(args.models)
    tags = detector().detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    estimator = AprilTagPoseEstimator()
    accepted = 0
    for tag in tags:
        cube_id = int(tag.tag_id)
        model = models.get(cube_id)
        if model is None:
            continue
        pose = estimator.estimate_tag(tag.corners, model, K, distortion)
        if pose is None:
            continue
        vertices_camera = (pose.camera_T_object[:3, :3] @ model.vertices().T).T
        vertices_camera += pose.camera_T_object[:3, 3]
        pixels = cv2.projectPoints(vertices_camera, np.zeros(3), np.zeros(3),
                                   K, distortion)[0].reshape(-1, 2).astype(int)
        for first, second in EDGES:
            cv2.line(image, tuple(pixels[first]), tuple(pixels[second]), (255, 255, 0), 2)
        center = tuple(np.mean(tag.corners, axis=0).astype(int))
        label = (f"ID {cube_id} z={pose.camera_T_object[2, 3]:.3f}m "
                 f"reproj={pose.reprojection_error_px:.2f}px")
        cv2.putText(image, label, center, cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 0, 255), 1, cv2.LINE_AA)
        print(label)
        accepted += 1
    status = "CALIBRATED K" if calibrated else "UNCALIBRATED K - VALIDATION ONLY"
    cv2.putText(image, status, (8, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (0, 0, 255) if not calibrated else (0, 180, 0), 1, cv2.LINE_AA)
    if not cv2.imwrite(args.output, image):
        raise SystemExit(f"cannot write output: {args.output}")
    print(f"tags={len(tags)} accepted={accepted} calibrated={calibrated} output={args.output}")
    if args.show:
        cv2.imshow("AprilTag cube wireframe", image)
        cv2.waitKey(0)
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
