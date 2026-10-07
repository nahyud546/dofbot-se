#!/usr/bin/env python3
"""Thu mẫu mặt rác bằng camera thật để bù khoảng cách giữa ảnh dataset và ảnh camera.

Không điều khiển tay. Với mỗi lớp rác: lật cube sao cho mặt rác đó hướng lên, xem trực tiếp,
khung xanh = đã nhận một mặt trên của cube; SPACE = lưu (crop 224x224 + embedding),
n = lớp kế, b = lớp trước, d = xoá mẫu vừa lưu của lớp, q = lưu và thoát. Lưu 8-12 mẫu mỗi lớp
ở nhiều vị trí/hướng/ánh sáng. Kết quả:
  ai/datasets/trash-images/real/<lớp>/NNN.png   (crop để xem lại)
  ai/datasets/trash-images/real/vector_database_dinov2_vits14_real.pt  (TrashDetector tự nạp)
Sau đó chạy eval_trash_faces.py để xem có cải thiện thật không (leave-one-out).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cube_vision import registry  # noqa: E402
from cube_vision.trash import TrashDetector, load_real_embeddings, save_real_embeddings  # noqa: E402

REAL_DIR = registry.repo_root() / "ai/datasets/trash-images/real"


def top_face_crops(frame, identify_cube):
    """[(quad, crop 224x224)] của các mặt trên cube thấy được trong khung."""
    out = []
    for group in identify_cube.find_cube_groups(frame):
        for face in sorted(group.faces, key=lambda f: f.area, reverse=True)[:1]:
            out.append((np.asarray(face.corners, np.float32),
                        identify_cube.rectify_face(frame, face.corners)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="auto",
                    help="auto = camera tay tự nhận (cube_vision.cameras), hoặc /dev/videoN")
    ap.add_argument("--classes", nargs="*", default=None,
                    help="chỉ thu các lớp này (mặc định: cả 16 lớp theo thứ tự cube 1-4)")
    args = ap.parse_args()
    if str(args.camera) == "auto":
        import sys as _sys
        from pathlib import Path as _Path
        _sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
        from cube_vision.cameras import resolve_arg as _resolve_camera
        args.camera = _resolve_camera(args.camera)
    import identify_cube
    from cube_search_center import open_camera
    classes = args.classes or [c for cid in sorted(registry.CUBES) for c in sorted(registry.CUBES[cid]["trash"])]
    unknown = [c for c in classes if c not in registry.TRASH_TO_CUBE]
    if unknown:
        raise SystemExit(f"lớp không tồn tại: {unknown}")
    cap = open_camera(args.camera)
    if cap is None:
        raise SystemExit(f"không mở được camera {args.camera} (tắt T8/perception trước)")
    detector = TrashDetector()
    if detector.model is None:
        raise SystemExit("DINO chưa sẵn sàng (torch/model)")
    embs, labels = load_real_embeddings(detector.model_name, detector.dim)
    items = [{"embedding": e, "label": l} for e, l in zip(embs, labels)]
    counts = {c: sum(1 for i in items if i["label"] == c) for c in classes}
    index = 0
    saved = {c: [] for c in classes}
    window = "collect_trash_faces: SPACE luu | n/b lop | d xoa | q thoat"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            if frame.shape[:2] != (480, 640):
                frame = cv2.resize(frame, (640, 480))
            name = classes[index]
            faces = top_face_crops(frame, identify_cube)
            view = frame.copy()
            for quad, _ in faces:
                cv2.polylines(view, [quad.astype(np.int32)], True, (0, 255, 0), 2)
            cid = registry.TRASH_TO_CUBE[name]
            cv2.putText(view, f"{name} (cube {cid}) da luu {counts[name]} | thay {len(faces)} mat",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("n"):
                index = (index + 1) % len(classes)
            elif key == ord("b"):
                index = (index - 1) % len(classes)
            elif key == ord("d") and saved[name]:
                path, item = saved[name].pop()
                path.unlink(missing_ok=True)
                items.remove(item)
                counts[name] -= 1
            elif key == ord(" "):
                if len(faces) != 1:
                    print(f"[COLLECT] cần đúng 1 mặt trên trong khung (đang thấy {len(faces)}); bỏ qua")
                    continue
                crop = faces[0][1]
                emb = detector.embed_turns([crop])[0, 0]
                folder = REAL_DIR / name
                folder.mkdir(parents=True, exist_ok=True)
                path = folder / f"{counts[name] + 1:03d}_{len(list(folder.glob('*.png'))) + 1}.png"
                cv2.imwrite(str(path), crop)
                item = {"embedding": emb, "label": name}
                items.append(item)
                saved[name].append((path, item))
                counts[name] += 1
                print(f"[COLLECT] {name}: {counts[name]} mẫu")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if items:
            print("[COLLECT] đã lưu:", save_real_embeddings(items, detector.model_name))
            print("[COLLECT] số mẫu:", {c: n for c, n in counts.items() if n})


if __name__ == "__main__":
    main()
