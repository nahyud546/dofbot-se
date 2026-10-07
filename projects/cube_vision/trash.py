"""Khớp mặt rác bằng embedding DINOv2 (chuyển nguyên từ identify_cube.TrashDetector)."""
from __future__ import annotations

import os

import cv2
import numpy as np

from .registry import TRASH_TO_CUBE, repo_root


def real_db_dir():
    # Không để trong processed/: process_and_embed.py xoá cả thư mục đó mỗi lần chạy.
    return repo_root() / "ai/datasets/trash-images/real"


def real_db_path(model_name: str = "dinov2_vits14", kind: str = "real"):
    return real_db_dir() / f"vector_database_{model_name}_{kind}.pt"


def load_real_embeddings(model_name: str = "dinov2_vits14", dim: int = 384):
    """([embedding], [nhãn]) từ real/vector_database_<model>_*.pt (camera thật và/hoặc ảnh giả lập
    camera), hoặc ([], []) nếu chưa có/hỏng."""
    embs, labels = [], []
    # "real" (camera thật) luôn nạp; "camaug" (giả lập camera từ build_trash_db.py) chỉ khi đặt
    # T8_TRASH_CAMAUG=1: mô phỏng cho thấy nó nâng độ chính xác ở ảnh xấu nhưng cũng làm điểm của
    # nhận sai cao lên, chưa có dữ liệu camera thật để chốt nên không bật mặc định.
    use_camaug = os.environ.get("T8_TRASH_CAMAUG") == "1"
    for path in sorted(real_db_dir().glob(f"vector_database_{model_name}_*.pt")):
        if path.name.endswith("_camaug.pt") and not use_camaug:
            continue
        try:
            import torch
            for item in torch.load(str(path), map_location="cpu", weights_only=False):
                emb = np.asarray(item["embedding"], dtype=np.float32)
                label = str(item["label"]).lower()
                if emb.shape == (dim,) and label in TRASH_TO_CUBE:
                    embs.append(emb)
                    labels.append(label)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] bỏ qua {path.name}: {exc}")
    return embs, labels


def save_real_embeddings(items, model_name: str = "dinov2_vits14", kind: str = "real") -> str:
    """Ghi [{"embedding": ndarray, "label": str}] (thay toàn bộ file của `kind`)."""
    import torch
    path = real_db_path(model_name, kind)
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save([{"embedding": np.asarray(i["embedding"], np.float32), "label": str(i["label"]).lower()}
                for i in items], str(path))
    return str(path)


class TrashDetector:
    """DINOv2 embedding match against the trash vector database.

    Same space as ai/datasets/trash-images/process_and_embed.py:
    Rectified 224x224 face -> ImageNet normalize -> matching DINOv2 model.
    Cosine similarity over 256 stored vectors (16 classes x 16).
    """

    DB_PATH_TMPL = "ai/datasets/trash-images/processed/vector_database_{}.pt"
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
        # Embedding đo bằng camera thật (collect_trash_faces.py): cùng miền ảnh với lúc chạy, nên
        # bù khoảng cách giữa ảnh dataset sạch và mặt in nhỏ nhìn qua camera tay.
        real_embs, real_labels = load_real_embeddings(self.model_name, self.dim)
        if real_embs:
            embs += real_embs
            labels += real_labels
            print(f"[OK] DINO trash: thêm {len(real_embs)} embedding camera thật")
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

    def embed_turns(self, crops_bgr):
        """(n, 4, dim) embedding chuẩn hoá của mỗi crop ở 4 góc xoay 0/90/180/270."""
        if self.model is None:
            raise RuntimeError("DINO chưa sẵn sàng")
        out = []
        for start in range(0, len(crops_bgr), 4):
            batch = crops_bgr[start:start + 4]
            turns = [np.ascontiguousarray(np.rot90(crop, k)) for crop in batch for k in range(4)]
            with self._torch.no_grad():
                tensor = self._torch.cat([self._preprocess(t) for t in turns], dim=0).to(self.device)
                emb = self.model(tensor).cpu().numpy()
            emb /= np.linalg.norm(emb, axis=1, keepdims=True) + 1e-9
            out.append(emb.reshape(len(batch), 4, -1))
        return np.concatenate(out, axis=0)

    def match(self, crop_bgr):
        """Match one crop; compatibility wrapper around batched inference."""
        label, score, self.last_result = self.match_many([crop_bgr])[0]
        return label, score

    def match_many(self, crops_bgr, refine: bool = True):
        """Batch face crops so several visible faces share each DINO forward pass.

        refine=False bỏ lượt xoay 45° cho crop không chắc (nhanh gấp ~2; dùng khi xác minh)."""
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
            if uncertain and refine:
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
