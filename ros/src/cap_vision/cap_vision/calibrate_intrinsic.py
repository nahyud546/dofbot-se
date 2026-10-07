"""Calibrate intrinsic checkerboard cho camera 640x480 (B1).

Chay offline voi anh co san (khuyen dung) hoac live capture:
    ros2 run cap_vision calibrate_intrinsic -- --images 'calib/*.png' --pattern 9x6 --square 0.025 --output src/cap_vision/config/camera_info_640x480.yaml
    ros2 run cap_vision calibrate_intrinsic -- --device 2 --count 25 --pattern 9x6 --square 0.025 --output src/cap_vision/config/camera_info_640x480.yaml

Gate dat: reproj <0.5px, fx trong 830-880, fy trong 730-770 (tu FOV 90x77 @120mm).
"""

import argparse
import glob
import sys

import cv2
import numpy as np
import yaml


def find_corners(gray, pattern):
    cols, rows = pattern
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)


def calibrate_from_images(paths, pattern, square):
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj_pts, img_pts, shape = [], [], None
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"Bo qua (khong doc duoc): {p}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_shape = gray.shape[::-1]
        if shape is not None and shape != image_shape:
            raise ValueError("Calibration images must have identical resolution")
        shape = image_shape
        c = find_corners(gray, pattern)
        if c is None:
            print(f"Khong thay checkerboard: {p}")
            continue
        obj_pts.append(objp)
        img_pts.append(c)
        print(f"OK {p}")
    if len(obj_pts) < 8:
        raise SystemExit(f"Chi co {len(obj_pts)} anh dat, can >=8 (thu 20-30 anh).")
    rms, K, dist, _, _, _, _, per_view_errors = cv2.calibrateCameraExtended(
        obj_pts, img_pts, shape, None, None)
    if not np.isfinite(rms) or not np.isfinite(K).all() or not np.isfinite(per_view_errors).all():
        raise SystemExit("calibrateCamera that bai.")
    return K, dist, float(rms), shape


def save_yaml(path, K, dist, err, shape):
    fx, fy = K[0, 0], K[1, 1]
    ok = bool(np.isfinite(err) and err < 0.5 and fx > 0 and fy > 0
              and np.isfinite(K).all() and np.isfinite(dist).all())
    data = {
        "image_width": int(shape[0]),
        "image_height": int(shape[1]),
        "camera_matrix": [float(v) for v in K.reshape(-1)],
        "distortion_coefficients": [float(v) for v in dist.reshape(-1)],
        "distortion_model": "plumb_bob",
        "calibrated": ok,
        "method": "checkerboard",
        "reprojection_error_px": round(float(err), 4),
        "notes": "Checkerboard: RMS<0.5px, finite positive focal lengths; validate held-out images.",
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    fx, fy = K[0, 0], K[1, 1]
    print(f"K=\n{K}\ndist={dist.reshape(-1)}\nreproj={err:.4f}px fx={fx:.1f} fy={fy:.1f}")
    print(f"Gate {'DAT' if ok else 'CHUA DAT (kiem tra lai anh/pattern/square)'} -> {path}")
    return 0 if ok else 2


def live_capture(device, count, pattern):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise SystemExit(f"Khong mo duoc camera {device}.")
    print(f"Nhan SPACE de chup ({count} anh), q de thoat.")
    saved, idx = [], 0
    while len(saved) < count:
        ok, frame = cap.read()
        if not ok:
            continue
        view = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = find_corners(gray, pattern)
        if c is not None:
            cv2.drawChessboardCorners(view, pattern, c, True)
        cv2.putText(view, f"{len(saved)}/{count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("calibrate_intrinsic (SPACE=chup, q=thoat)", view)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == 32 and c is not None:  # space
            p = f"/tmp/calib_{idx:02d}.png"
            cv2.imwrite(p, frame)
            saved.append(p)
            idx += 1
            print(f"Luu {p}")
    cap.release()
    cv2.destroyAllWindows()
    return saved


def main():
    ap = argparse.ArgumentParser(description="Calibrate intrinsic checkerboard.")
    ap.add_argument("--images", default="", help="Glob anh, vd 'calib/*.png'")
    ap.add_argument("--device", type=int, default=-1, help="Live capture /dev/videoN")
    ap.add_argument("--count", type=int, default=25)
    ap.add_argument("--pattern", default="9x6", help="cols x rows inner corners")
    ap.add_argument("--square", type=float, default=0.025, help="canh o co (m)")
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    cols, rows = (int(v) for v in a.pattern.lower().split("x"))
    if a.images:
        paths = sorted(glob.glob(a.images))
        if not paths:
            raise SystemExit(f"Khong thay anh: {a.images}")
        K, dist, err, shape = calibrate_from_images(paths, (cols, rows), a.square)
    elif a.device >= 0:
        paths = live_capture(a.device, a.count, (cols, rows))
        if len(paths) < 8:
            raise SystemExit("Chua du anh.")
        K, dist, err, shape = calibrate_from_images(paths, (cols, rows), a.square)
    else:
        raise SystemExit("Can --images hoac --device.")
    raise SystemExit(save_yaml(a.output, K, dist, err, shape))


if __name__ == "__main__":
    main()
