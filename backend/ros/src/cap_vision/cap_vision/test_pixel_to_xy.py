"""Ham dung chung + CLI kiem tra mapping pixel -> table_frame (met).

H la ma tran 3x3 tu config/homography.yaml:
    [x*w, y*w, w]^T = H @ [u, v, 1]^T   (x, y trong frame table_frame)
cap_detector import truc tiep tu day de tranh lap lai ma tran o nhieu noi.

2 cach dung (plan phase 5 - gate thuoc truoc khi them detector):
    test_pixel_to_xy -- --homography H.yaml --pixel 321,242
    test_pixel_to_xy -- --homography H.yaml --click --image /tmp/cap0.png
Che do click: click chuot vao anh -> console in (X, Y) mm de do thuoc doi chieu.
"""

import argparse

import numpy as np
import yaml

# Vung lam viec tren mat ban (met, frame table_frame, goc O bottom-left).
# Ban 266x189mm + margin 5mm chong nhieu bien. Chinh theo ban that khi can.
WORKSPACE = {
    "x_min": -0.005,
    "x_max": 0.271,
    "y_min": -0.005,
    "y_max": 0.194,
}


def load_homography(path):
    """Doc yaml -> (H 3x3 float64, meta dict). H=None neu chua calibrate."""
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not data.get("calibrated", False):
        return None, data
    H = np.asarray(data["homography"], dtype=np.float64).reshape(3, 3)
    return H, data


def pixel_to_xy(H, u, v):
    """(u, v) pixel -> (x, y) met. Raise ValueError neu H suy bien."""
    p = H @ np.array([float(u), float(v), 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
        raise ValueError(f"Homography suy bien tai pixel ({u}, {v})")
    return float(p[0] / p[2]), float(p[1] / p[2])


def xy_to_pixel(H, x, y):
    """Nguoc lai: (x, y) met -> (u, v) pixel (de ve overlay)."""
    Hinv = np.linalg.inv(H)
    p = Hinv @ np.array([float(x), float(y), 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
        raise ValueError(f"Khong chieu nguoc duoc diem ({x}, {y})")
    return float(p[0] / p[2]), float(p[1] / p[2])


def inside_workspace(x, y, ws=None):
    """True neu (x, y) nam trong vung lam viec."""
    ws = ws or WORKSPACE
    return ws["x_min"] <= x <= ws["x_max"] and ws["y_min"] <= y <= ws["y_max"]


def report_pixel(H, u, v):
    """In 1 diem + tra ve True neu trong vung. Don vi mm de do thuoc."""
    x, y = pixel_to_xy(H, u, v)
    ok = inside_workspace(x, y)
    print(f"pixel = ({u:.0f}, {v:.0f})  ->  table = ({x * 1000:.1f}, {y * 1000:.1f}) mm"
          f"  [{'OK' if ok else 'NGOAI VUNG'}]")
    return ok


def click_loop(H, image_path):
    """Mo anh, click chuot -> in (X, Y) mm. 'q' thoat. Tra ve so diem ngoai vung."""
    import cv2  # noqa: PLC0415 - lazy de module van import duoc khi headless
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Khong doc duoc anh: {image_path}")
        raise SystemExit(2)
    bad = [0]
    print("Click vao anh de doi ra mm (so sanh voi thuoc that). Nhan 'q' de thoat.")

    def on_mouse(event, u, v, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if not report_pixel(H, float(u), float(v)):
                bad[0] += 1

    name = "test_pixel_to_xy: click -> mm (q = thoat)"
    cv2.namedWindow(name)
    cv2.setMouseCallback(name, on_mouse)
    while True:
        cv2.imshow(name, frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    return bad[0]


def main():
    parser = argparse.ArgumentParser(
        description="Kiem tra homography: pixel -> toa do ban table_frame (mm).")
    parser.add_argument("--homography", required=True,
                        help="Duong dan config/homography.yaml")
    parser.add_argument("--pixel", action="append", default=[],
                        metavar="U,V",
                        help="Diem pixel can doi (lap lai flag cho nhieu diem).")
    parser.add_argument("--corners", action="store_true",
                        help="Do them 4 goc anh de hinh dung vung phu.")
    parser.add_argument("--click", action="store_true",
                        help="Che do click chuot tren anh (can --image).")
    parser.add_argument("--image", default="",
                        help="Anh de click (vd snapshot tu camera_test).")
    args = parser.parse_args()

    H, meta = load_homography(args.homography)
    if H is None:
        print(f"CHUA CALIBRATE: {args.homography} (calibrated: false).")
        print("Chay: ros2 run cap_vision calibrate_table")
        raise SystemExit(2)

    print(f"H:\n{H}")
    if meta.get("reprojection_error_px") is not None:
        print(f"reprojection_error_px: {meta['reprojection_error_px']}")

    if args.click:
        if not args.image:
            print("Che do --click can kem --image <duong-dan-anh>.")
            raise SystemExit(2)
        raise SystemExit(1 if click_loop(H, args.image) else 0)

    pixels = []
    for item in args.pixel:
        u, v = item.split(",")
        pixels.append((float(u), float(v)))
    if args.corners:
        w, h = int(meta.get("image_width", 640)), int(meta.get("image_height", 480))
        pixels += [(0, 0), (w, 0), (w, h), (0, h)]
    if not pixels:
        print("Khong co diem nao. Them --pixel U,V, --corners hoac --click --image.")
        raise SystemExit(2)

    bad = 0
    for u, v in pixels:
        bad += not report_pixel(H, u, v)
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
