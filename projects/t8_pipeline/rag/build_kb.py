#!/usr/bin/env python3
# coding: utf-8
"""Build kb_vi.jsonl từ dify_knowledge_export.csv + hand-written VI entries.

Chạy lại khi CSV đổi:
  python3 LargeModel_ws/rag/build_kb.py
Không cần mạng, không cần embedding (TF-IDF ở retriever).
CSV là tiếng Trung 100% -> script trích q_zh/answer_zh làm fallback,
phần VI do kb_vi.jsonl thủ công (đã dịch các intent chính) đảm nhiệm.
"""
import csv
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CSV = ROOT / "dify_knowledge_export.csv"
OUT_AUTO = Path(__file__).resolve().parent / "kb_auto_zh.jsonl"


def split_qa(raw):
    """CSV segment_content dạng pseudo-JSON: query":"...";"answer":"..."."""
    if not raw:
        return "", ""
    s = raw.replace('""', '"')
    mq = re.search(r'query"\s*:\s*"', s)
    ma = re.search(r'answer"\s*:\s*"', s)
    if not mq:
        return "", s.strip()[:1000]
    q0 = mq.end()
    if ma:
        q = s[q0:ma.start()].rstrip(';" ').rstrip('"')
        a = s[ma.end():].strip().rstrip('"').rstrip(';').strip()
    else:
        q, a = s[q0:].strip(), ""
    return q.strip()[:500], a.strip()[:1500]


def main():
    rows = list(csv.DictReader(CSV.open(encoding="utf-8")))
    print(f"CSV rows: {len(rows)}")
    with OUT_AUTO.open("w", encoding="utf-8") as f:
        for i, r in enumerate(rows, 1):
            q, a = split_qa(r.get("segment_content", ""))
            if not q and not a:
                continue
            f.write(json.dumps({"id": f"zh{i:02d}", "q_vi": "", "q_zh": q,
                                "answer": a, "source": "dify_csv"},
                               ensure_ascii=False) + "\n")
    print(f"wrote {OUT_AUTO} (fallback ZH, retriever sẽ gộp với kb_vi.jsonl)")


if __name__ == "__main__":
    main()
