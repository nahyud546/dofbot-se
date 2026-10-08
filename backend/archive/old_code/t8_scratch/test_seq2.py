import re
from t8_pipeline import try_local_rotate, try_local_light, norm_nodau, try_local_hold_place as old_hold

def try_local_hold_place(q):
    t = norm_nodau(q)
    if any(k in t for k in ["dung thang", "tu the chuan", "tu the cho", "pose start", "vi tri ban dau", "vi tri cu", "vi tri xuat phat"]):
        return "pose_start", {}, "Về tư thế chuẩn."
    return old_hold(q)

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

q = "tay xuống và xoay phải 90 độ sau đố thì về vị trí xuất phát"
print(try_local_motion_sequence(q))
