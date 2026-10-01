import re
from t8_pipeline import try_local_hold_place, try_local_rotate, try_local_light, norm_nodau

def try_local_motion_sequence(q):
    t = norm_nodau(q)
    parts = re.split(r'\b(?:va|sau do|sau do thi|roi|xong thi|tiep tuc)\b|,|\.', t)
    steps = []
    for part in parts:
        part = part.strip()
        if not part: continue
        step = try_local_hold_place(part)
        if step:
            steps.append(step)
            continue
        step = try_local_rotate(part)
        if step:
            steps.append(step)
            continue
        step = try_local_light(part)
        if step:
            steps.append(step)
            continue
    if len(steps) >= 2:
        return steps
    return None

queries = [
    "về trạng thái pose start, xoay sang trái cho tôi",
    "hạ xuống và tiếp tục xoay sang trái thêm 30 độ cho tôi",
    "tay xuống và xoay phải 90 độ sau đố thì về vị trí xuất phát"
]

for q in queries:
    print(f"Q: {q}\nA: {try_local_motion_sequence(q)}\n")

