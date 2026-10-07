"""Offline calibration from stationary image + measured mount pose pairs.

Dataset YAML: pattern: [9,6], square_m: 0.015, samples: [{image: path,
base_T_mount: [16 row-major values], validation: false}, ...]. The checkerboard
stays fixed, flat on the table; +Z must point up. Image and pose must be paired
at acquisition, using capture_calibration. Requires >=8 fit + >=3 held-out poses.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from cap_vision.calibrate_intrinsic import find_corners
from cap_vision.red_scene_core import load_config, transform


def calibrate(samples, camera, pattern, square_m):
    if square_m <= 0 or min(pattern) < 3:
        raise ValueError("invalid checkerboard dimensions")
    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m
    pairs = []
    K = np.array(camera["K"]).reshape(3, 3)
    dist = np.array(camera["distortion"])
    for sample in samples:
        image = cv2.imread(str(sample["image"]))
        if image is None or image.shape[:2] != (camera["height"], camera["width"]):
            raise ValueError(f"missing/wrong resolution: {sample['image']}")
        corners = find_corners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), pattern)
        if corners is None:
            raise ValueError(f"checkerboard not found: {sample['image']}")
        ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist)
        if not ok or tvec[2, 0] <= 0:
            raise ValueError("board pose failed")
        projected = cv2.projectPoints(obj, rvec, tvec, K, dist)[0]
        rms = np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2)))
        if rms > 0.5:
            raise ValueError(f"board reprojection RMS {rms:.3f}px > 0.5")
        camera_T_board = np.eye(4)
        camera_T_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        camera_T_board[:3, 3] = tvec.ravel()
        pairs.append((transform(sample["base_T_mount"]), camera_T_board,
                      bool(sample.get("validation", False))))
    train = [(a, b) for a, b, held in pairs if not held]
    held_out = [(a, b) for a, b, held in pairs if held]
    if len(train) < 8 or len(held_out) < 3:
        raise ValueError("need >=8 training and >=3 held-out poses")
    # At least two independent rotation axes, not just yaw sweeps.
    rotations = np.array([cv2.Rodrigues(train[0][0][:3, :3].T @ a[:3, :3])[0].ravel()
                          for a, _ in train[1:]])
    singular = np.linalg.svd(rotations, compute_uv=False)
    if singular[1] < 0.15:
        raise ValueError("insufficient rotation diversity (need two axes)")
    r, t = cv2.calibrateHandEye([a[:3, :3] for a, _ in train],
        [a[:3, 3] for a, _ in train], [b[:3, :3] for _, b in train],
        [b[:3, 3] for _, b in train], method=cv2.CALIB_HAND_EYE_PARK)
    mount_T_camera = np.eye(4)
    mount_T_camera[:3, :3], mount_T_camera[:3, 3] = r, t.ravel()
    transform(mount_T_camera)
    boards = [a @ mount_T_camera @ b for a, b in train]
    # Nearest orthonormal rotation to mean of board rotations.
    u, _, vt = np.linalg.svd(np.mean([b[:3, :3] for b in boards], axis=0))
    board = np.eye(4)
    board[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    board[:3, 3] = np.mean([b[:3, 3] for b in boards], axis=0)
    if board[2, 2] < 0:
        board[:3, :3] = board[:3, :3] @ np.diag([1, -1, -1])
    errors = []
    # Compare board corners in base, not merely reprojection or fitted transforms.
    reference = obj @ (boards[0][:3, :3]).T + boards[0][:3, 3]
    for a, b in held_out:
        candidate = a @ mount_T_camera @ b
        points = obj @ candidate[:3, :3].T + candidate[:3, 3]
        errors.append(float(np.max(np.linalg.norm(points - reference, axis=1))))
    if max(errors) > 0.005:
        raise ValueError(f"held-out board error {max(errors)*1000:.1f}mm > 5mm; check board ordering/mount/kinematics")
    return mount_T_camera, board, errors


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--intrinsic", required=True)
    ap.add_argument("--output", required=True, help="new scene config, never overwrite input")
    args = ap.parse_args()
    if Path(args.output).exists():
        raise SystemExit("output exists; choose a new calibration file")
    cfg, dataset, intrinsic = map(load_config, [args.config, args.dataset, args.intrinsic])
    if not intrinsic.get("calibrated"):
        raise SystemExit("intrinsic calibration has not passed")
    c = cfg["camera"]
    c.update(K=intrinsic["camera_matrix"], distortion=intrinsic["distortion_coefficients"],
             width=intrinsic["image_width"], height=intrinsic["image_height"], calibrated=True)
    for s in dataset["samples"]:
        s["image"] = str(Path(args.dataset).resolve().parent / s["image"])
    mount, table, errors = calibrate(dataset["samples"], c, tuple(dataset["pattern"]), dataset["square_m"])
    c.update(source="checkerboard_eye_in_hand", mount_T_optical=mount.ravel().tolist(),
             extrinsic_calibrated=True)
    cfg["table"].update(base_T_table=table.ravel().tolist(), calibrated=True)
    cfg["calibration_validation"] = {"held_out_errors_m": errors,
                                     "dataset": str(Path(args.dataset).resolve())}
    with open(args.output, "x", encoding="utf-8") as stream:
        yaml.safe_dump(cfg, stream, sort_keys=False)
    print(f"Saved {args.output}; held-out max={max(errors)*1000:.2f}mm. TCP/gripper still require validation.")


if __name__ == "__main__":
    main()
