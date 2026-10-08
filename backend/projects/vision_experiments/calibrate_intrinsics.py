#!/usr/bin/env python3
"""Hiệu chuẩn thông số nội (K, k1, k2) của một camera bằng bảng ChArUco, qua ĐÚNG luồng sẽ dùng sau này.

Bảng hiện trên một màn hình phẳng, không cần in:
  - iPhone (DroidCam): bảng hiện trên màn hình laptop, cầm điện thoại quay quanh màn hình.
        python projects/vision_experiments/calibrate_intrinsics.py --camera phone --with-board
  - camera tay / webcam laptop (không tự nhìn được màn hình laptop): mở ảnh bảng trên điện thoại hoặc màn hình
    khác rồi đưa bảng đi quanh trước camera.
        python projects/vision_experiments/calibrate_intrinsics.py --png /tmp/charuco.png     # ảnh để mở trên điện thoại
        python projects/vision_experiments/calibrate_intrinsics.py --camera wrist
        python projects/vision_experiments/calibrate_intrinsics.py --camera ext

Chương trình tự giữ các khung nét và khác nhau; hãy đưa bảng tới cả 4 góc ảnh, gần và xa, nghiêng trái/phải/lên/
xuống. Kích thước thật của ô KHÔNG ảnh hưởng K (chỉ cần bảng phẳng và đúng tỉ lệ). Kết quả ghi vào
config/robot/cameras/<tên>.json.  Phím q hoặc Esc: dừng thu sớm và giải với các khung đã có.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "projects", Path(__file__).resolve().parent):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from cube_vision import cameras, intrinsics as I  # noqa: E402

BOARD = I.Board()
STRIP_PX = 70
WINDOW = "charuco"


def board_canvas(screen=(1920, 1080)):
    """Ảnh bảng vừa màn hình, chừa dải trạng thái phía dưới. Trả (ảnh BGR, số pixel mỗi ô)."""
    per = min((screen[0] - 40) // BOARD.cols, (screen[1] - STRIP_PX - 40) // BOARD.rows)
    board = cv2.cvtColor(BOARD.image(per), cv2.COLOR_GRAY2BGR)
    canvas = np.full((screen[1], screen[0], 3), 255, np.uint8)
    x0, y0 = (screen[0] - board.shape[1]) // 2, (screen[1] - STRIP_PX - board.shape[0]) // 2
    canvas[y0:y0 + board.shape[0], x0:x0 + board.shape[1]] = board
    return canvas, per


def status_strip(canvas, text, good):
    out = canvas.copy()
    cv2.rectangle(out, (0, out.shape[0] - STRIP_PX), (out.shape[1], out.shape[0]), (255, 255, 255), -1)
    cv2.putText(out, text, (30, out.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                (0, 140, 0) if good else (0, 0, 200), 2, cv2.LINE_AA)
    return out


def collect(cap, rotate, want, timeout_s, with_board, log=print):
    """Thu các khung thấy bảng, nét và khác nhau. Trả (views, cỡ ảnh)."""
    canvas = board_canvas()[0] if with_board else None
    if with_board:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    views, kept, size, covered = [], [], None, set()
    start, last_keep, message = time.time(), 0.0, "Dua camera nhin vao bang"
    while len(views) < want and time.time() - start < timeout_s:
        ok, raw = cap.read()
        if not ok:
            time.sleep(0.02)
            continue
        frame = cameras.rotate_frame(raw, rotate)
        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ids, pts = I.detect(gray, BOARD)
        good = False
        if len(ids) < I.MIN_CORNERS:
            message = "Khong thay bang (hoac thay qua it)"
        elif I.sharpness(gray) < I.MIN_SHARPNESS:
            message = "Anh mo: giu yen hon"
        elif time.time() - last_keep < 0.4 or not I.is_new_view(pts, kept, size):
            message, good = "Doi goc nhin / doi vi tri bang trong anh", True
        else:
            views.append((ids, pts))
            kept.append(pts)
            covered |= I.cells(pts, size)
            last_keep, good = time.time(), True
            message = "Da giu khung"
            log(f"  khung {len(views)}/{want}: {len(ids)} góc, phủ {len(covered)}/9 vùng ảnh")
        text = f"{len(views)}/{want} khung | phu {len(covered)}/9 vung | {message} | q: dung"
        if with_board:
            cv2.imshow(WINDOW, status_strip(canvas, text, good))
        else:
            view = frame.copy()
            for u, v in pts:
                cv2.circle(view, (int(u), int(v)), 4, (0, 255, 0), -1)
            cv2.putText(view, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, view)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    return views, size


def report(result) -> str:
    fx, fy, cx, cy = result["K"]
    lines = [f"Khung dùng: {result['intrinsics_views']}, phủ {result['intrinsics_cells']}/9 vùng ảnh, "
             f"độ nghiêng bảng {result['intrinsics_tilt_deg'][0]:.0f}–{result['intrinsics_tilt_deg'][1]:.0f}°",
             f"Ống kính: fx={fx:.1f} fy={fy:.1f} tâm=({cx:.1f}, {cy:.1f}) k1={result['k1']:+.4f} k2={result['k2']:+.4f}",
             f"RMS chiếu lại: {result['intrinsics_rms_px']:.2f} px"]
    if result["intrinsics_holdout_px"]:
        lines.append(f"Khung kiểm định (không tham gia fit): trung vị "
                     f"{np.median(result['intrinsics_holdout_px']):.2f} px")
    lines.append("ĐẠT" if result["intrinsics_accepted"] else "CHƯA ĐẠT: " + "; ".join(result["intrinsics_reasons"]))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", choices=sorted(cameras.STREAMS), help="camera cần hiệu chuẩn")
    ap.add_argument("--source", default="auto", help="/dev/videoN hoặc URL; mặc định tự tìm theo --camera")
    ap.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), help="xoay luồng (mặc định theo camera)")
    ap.add_argument("--with-board", action="store_true", help="hiện bảng toàn màn hình laptop trong lúc thu")
    ap.add_argument("--show", action="store_true", help="chỉ hiện bảng toàn màn hình (thiết bị khác thu)")
    ap.add_argument("--png", help="ghi ảnh bảng ra file (mở trên điện thoại làm bảng)")
    ap.add_argument("--views", type=int, default=30)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    if args.png:
        cv2.imwrite(args.png, BOARD.image(240, margin_px=60))
        print(f"Đã ghi {args.png} ({BOARD.cols}×{BOARD.rows} ô). Mở toàn màn hình, tắt tự xoay và tự tối màn hình.")
        return
    if args.show:
        canvas, per = board_canvas()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(WINDOW, status_strip(canvas, f"ChArUco {BOARD.cols}x{BOARD.rows}, {per} px/o | q: thoat", True))
        while cv2.waitKey(50) & 0xFF not in (ord("q"), 27):
            pass
        cv2.destroyAllWindows()
        return
    if not args.camera:
        ap.error("cần --camera, --show hoặc --png")
    spec = cameras.STREAMS[args.camera]
    source = cameras.stream_source(args.camera, args.source)
    rotate = spec["rotate"] if args.rotate is None else args.rotate
    cap = cameras.open_stream(source, spec["size"], spec["fourcc"])
    if cap is None:
        raise SystemExit(f"Không mở được camera '{args.camera}' ({source}). Có tiến trình khác đang giữ? "
                         "(DroidCam chỉ cho một kết nối.)")
    print(f"Thu từ {source} (xoay {rotate}°). Đưa bảng tới 4 góc ảnh, gần/xa, nghiêng nhiều hướng.")
    try:
        views, size = collect(cap, rotate, args.views, args.timeout, args.with_board)
    finally:
        cap.release()
    if len(views) < 4:
        raise SystemExit(f"Chỉ thu được {len(views)} khung thấy bảng: không giải được.")
    result = I.solve(views, BOARD, size)
    print(report(result))
    result.update(rotate=rotate, source=str(source))
    if not result["intrinsics_accepted"]:
        old = I.load_camera(args.camera)
        if old and old.get("intrinsics_accepted") is True:
            raise SystemExit("Giữ nguyên bản intrinsic đã đạt trước đó; không ghi bản chưa đạt.")
    print(f"Đã ghi {I.save_camera(args.camera, result)} (intrinsics_accepted={result['intrinsics_accepted']}).")


if __name__ == "__main__":
    main()
