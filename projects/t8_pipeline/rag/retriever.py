#!/usr/bin/env python3
# coding: utf-8
"""Local RAG retriever: TF-IDF (sklearn) trên kb_vi.jsonl + kb_auto_zh.jsonl.

- Offline 100%, không cần mạng/GPU. Chạy tốt trên laptop.
- Chuẩn hoá không dấu (giống norm_nodau của pipeline) để query TV khớp KB.
- Hook tương lai: nếu có sentence-transformers/BGE-M3 thì thay _score bằng embedding.
API: LocalRetriever(kb_path).query(text, top_k=3) -> [{id,answer,score,source}]
"""
import json
import re
import unicodedata
from pathlib import Path

try:
    from sklearn.feature_extraction.text import TfidfVectorizer
    from sklearn.metrics.pairwise import cosine_similarity
    _HAS_SK = True
except ImportError:
    _HAS_SK = False


def nodau(s):
    s = (s or "").lower().replace("đ", "d")
    s = "".join(c for c in unicodedata.normalize("NFD", s)
                if unicodedata.category(c) != "Mn")
    return re.sub(r"\s+", " ", s).strip()


class LocalRetriever:
    def __init__(self, kb_path=None, auto_path=None):
        base = Path(kb_path) if kb_path else Path(__file__).parent / "kb_vi.jsonl"
        self.docs = []
        for p in [base, Path(auto_path) if auto_path else base.parent / "kb_auto_zh.jsonl"]:
            if p and Path(p).exists():
                for line in Path(p).read_text(encoding="utf-8").splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        d = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    # text index: q_vi + q_zh + answer (nodau)
                    idx = " ".join([d.get("q_vi", ""), d.get("q_zh", ""),
                                    d.get("answer", "")[:800]])
                    d["_idx"] = nodau(idx)
                    if d["_idx"]:
                        self.docs.append(d)
        if _HAS_SK and self.docs:
            self.vec = TfidfVectorizer(analyzer="word", ngram_range=(1, 2),
                                       min_df=1, max_features=5000)
            self.mat = self.vec.fit_transform([d["_idx"] for d in self.docs])
        else:
            self.vec, self.mat = None, None

    def __len__(self):
        return len(self.docs)

    def _fallback_score(self, q, d):
        qs, ds = set(q.split()), set(d["_idx"].split())
        if not qs:
            return 0.0
        return len(qs & ds) / max(1, len(qs))

    def query(self, text, top_k=3):
        q = nodau(text)
        if not q or not self.docs:
            return []
        if self.vec is not None:
            try:
                qv = self.vec.transform([q])
                sims = cosine_similarity(qv, self.mat)[0]
            except ValueError:
                sims = [0.0] * len(self.docs)
            order = sorted(range(len(self.docs)), key=lambda i: sims[i], reverse=True)
            out = []
            for i in order[:top_k]:
                if sims[i] <= 0:
                    continue
                d = self.docs[i]
                out.append({"id": d.get("id", ""), "answer": d.get("answer", ""),
                            "score": float(sims[i]), "source": d.get("source", "")})
            return out
        scored = [(self._fallback_score(q, d), d) for d in self.docs]
        scored.sort(key=lambda x: x[0], reverse=True)
        return [{"id": d.get("id", ""), "answer": d.get("answer", ""),
                 "score": s, "source": d.get("source", "")}
                for s, d in scored[:top_k] if s > 0]
