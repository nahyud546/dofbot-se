import re
from projects.t8_pipeline.t8_pipeline import norm_nodau

def is_multi_action_query(q):
    t = norm_nodau(q)
    parts = re.split(r'\b(?:va|sau do|sau do thi|roi|xong thi|tiep tuc)\b|,|\.', t)
    return len([p for p in parts if p.strip()]) >= 2
