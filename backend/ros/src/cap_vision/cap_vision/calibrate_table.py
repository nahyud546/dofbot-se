"""Calibrate homography pixel -> table_frame bang cach click >= 4 diem.

Quy trinh tren ban that (plan phase 3-4):
  1. Chon goc O cua table_frame, dat 4+ vat danh dau tai (x, y) da biet (met).
     Mac dinh: ban 300x250mm, O = goc top-left.
  2. Chay script, click lan luot theo dung thu tu world_points.
  3. Nhan 's' de tinh H (RANSAC) + luu yaml; 'r' lam lai; 'q' thoat.

Khong co man hinh (SSH/headless): truyen --pixels "u1,v1 u2,v2 ...".

Vi du:
    ros2 run cap_vision calibrate_table -- --output src/cap_vision/config/homography.yaml
    ros2 run cap_vision calibrate_table -- --image /tmp/cap0.png --pixels "100,200 540,200 540,420 100,420"
"""

import argparse
import datetime

import cv2
import numpy as np
import yaml

DEFAULT_WORLD = [  # 4 goc ban 266x189mm (met, frame table_frame, O bottom-left).
    (0.00, 0.00),
    (0.266, 0.00),
    (0.266, 0.189),
    (0.00, 0.189),
]


def compute_homography(pixels, world):
    """pixels [(u,v)], world [(x,y)] -> (H 3x3, reproj_err_px)."""
    if len(pixels) < 4 or len(world) < 4:
        raise ValueError("Can it nhat 4 cap diem (dang co "
                         f"{len(pixels)} pixel / {len(world)} world).")
    if len(pixels) != len(world):
        raise ValueError("So diem pixel va world phai bang nhau.")
    src = np.asarray(pixels, dtype=np.float64)
    dst = np.asarray(world, dtype=np.float64)
    H, _ = cv2.findHomography(src, dst)
    if H is None:
        raise RuntimeError("findHomography that bai (diem thang hang?).")
    proj = (H @ np.hstack([src, np.ones((len(src), 1))]).T).T
    proj = proj[:, :2] / proj[:, 2:3]
    # Sai so chieu nguoc ra pixel de co don vi truc quan (px).
    Hinv = np.linalg.inv(H)
    back = (Hinv @ np.hstack([dst, np.ones((len(dst), 1))]).T).T
    back = back[:, :2] / back[:, 2:3]
    err = float(np.mean(np.linalg.norm(back - src, axis=1)))
    return H, err


def save_homography_yaml(path, H, err, img_size, world):
    h, w = img_size
    data = {
        "image_width": int(w),
        "image_height": int(h),
        "homography": [[float(v) for v in row] for row in H.tolist()],
        "calibrated": True,
        "reprojection_error_px": round(err, 3),
        "world_points": [[float(x), float(y)] for x, y in world],
        "notes": f"Calibrated {datetime.datetime.now():%Y-%m-%d %H:%M}."
                 " Diem world theo thu tu click pixel.",
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return data


def parse_world(text):
    if not text:
        return list(DEFAULT_WORLD)
    pts = []
    for tok in text.strip().split():
        x, y = tok.split(",")
        pts.append((float(x), float(y)))
    return pts


def parse_pixels(text):
    pts = []
    for tok in text.strip().split():
        u, v = tok.split(",")
        pts.append((float(u), float(v)))
    return pts


def interactive_collect(frame, n):
    """Click n diem, ve overlay. Tra ve [(u, v)]."""
    clicks = []

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < n:
            clicks.append((float(x), float(y)))

    cv2.namedWindow("calibrate_table: click theo thu tu world_points")
    cv2.setMouseCallback("calibrate_table: click theo thu tu world_points", on_mouse)
    print(f"Click {n} diem theo thu tu world_points. "
          "'s'=luu, 'r'=lam lai, 'q'=thoat.")
    while True:
        view = frame.copy()
        for i, (u, v) in enumerate(clicks):
            cv2.circle(view, (int(u), int(v)), 6, (0, 255, 0), -1)
            cv2.putText(view, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("calibrate_table: click theo thu tu world_points", view)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            return None
        if key == ord("r"):
            clicks.clear()
        if key == ord("s") and len(clicks) >= n:
            return clicks[:n]


def main():
    parser = argparse.ArgumentParser(description="Calibrate homography ban nap.")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--image", default="",
                        help="Dung anh co san thay vi mo camera.")
    parser.add_argument("--world", default="",
                        help='"x1,y1 x2,y2 ..." (met). Mac dinh 4 goc ban 20x24cm.')
    parser.add_argument("--pixels", default="",
                        help='Che do headless: "u1,v1 u2,v2 ..." (khong can man hinh).')
    parser.add_argument("--output", required=True, help="File homography.yaml de ghi.")
    args = parser.parse_args()

    world = parse_world(args.world)
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Khong doc duoc anh: {args.image}")
            raise SystemExit(2)
    else:
        cap = cv2.VideoCapture(args.device)
        if not cap.isOpened():
            print(f"Khong mo duoc camera index {args.device}. "
                  "Chup anh bang camera_test roi truyen --image.")
            raise SystemExit(2)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            print("Doc frame that bai.")
            raise SystemExit(2)

    h, w = frame.shape[:2]
    if args.pixels:
        pixels = parse_pixels(args.pixels)
    else:
        try:
            pixels = interactive_collect(frame, len(world))
        finally:
            cv2.destroyAllWindows()
        if pixels is None:
            print("Huy calibrate.")
            raise SystemExit(1)

    H, err = compute_homography(pixels, world)
    save_homography_yaml(args.output, H, err, (h, w), world)
    print(f"H:\n{H}")
    print(f"reprojection_error_px: {err:.2f} -> da luu {args.output}")
    print("Kiem tra lai: ros2 run cap_vision test_pixel_to_xy"
          f" -- --homography {args.output} --corners")


if __name__ == "__main__":
    main()
