import re

with open('workspaces/dofbot_robot_arm_6dof/src/cap_vision/cap_vision/object_pipeline.py', 'r') as f:
    content = f.read()

new_logic = """
def instance_similarity(inst1, inst2):
    iou = mask_iou(inst1.mask, inst2.mask)
    if iou >= 0.25:
        return iou + 1.0  # Uu tien IOU
    b1, b2 = inst1.bbox, inst2.bbox
    cx1, cy1 = (b1[0] + b1[2]) / 2, (b1[1] + b1[3]) / 2
    cx2, cy2 = (b2[0] + b2[2]) / 2, (b2[1] + b2[3]) / 2
    dist = ((cx1 - cx2)**2 + (cy1 - cy2)**2)**0.5
    diag = ((b1[2] - b1[0])**2 + (b1[3] - b1[1])**2)**0.5
    if dist < diag * 0.8:
        return 1.0 - (dist / (diag * 0.8))
    return 0.0

@dataclass
class TrackedObject:"""

content = content.replace("@dataclass\nclass TrackedObject:", new_logic)

update_method = """    def update(self, instances, timestamp):
        matches = set()
        for instance in sorted(instances, key=lambda item: -item.confidence):
            ranked = sorted(((instance_similarity(instance, track.instance), key)
                             for key, track in self.tracks.items()
                             if key not in matches and timestamp - track.last_seen <= self.max_gap),
                            reverse=True)
            if ranked and ranked[0][0] > 0.0:
                key = ranked[0][1]
                track = self.tracks[key]
                track.instance = instance"""

content = re.sub(r"    def update\(self, instances, timestamp\):.*?track\.instance = instance", update_method, content, flags=re.DOTALL)

with open('workspaces/dofbot_robot_arm_6dof/src/cap_vision/cap_vision/object_pipeline.py', 'w') as f:
    f.write(content)
