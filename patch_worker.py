import sys

path = "/home/jloy/Desktop/robot-arm/projects/t8_pipeline/t8_motion_worker.py"
with open(path, "r") as f:
    content = f.read()

# Modify the command tuple
content = content.replace('if command in ("preflight_cube_pick", "sort_cube_zone", "sort_cube_candidates"):',
                          'if command in ("preflight_cube_pick", "sort_cube_zone", "sort_cube_candidates", "sort_cube_3d"):')

old_candidates_logic = """        candidates = data.get("candidates") if command == "sort_cube_candidates" else [data]
        if not isinstance(candidates, list) or not candidates:
            raise ValueError("Cần ít nhất một cube đã xác nhận để phân loại")
        skipped = []
        for candidate_index, candidate in enumerate(candidates):
            if not isinstance(candidate, dict):
                raise ValueError("Dữ liệu cube không hợp lệ")
            try:
                cube_id, (x, y, z), joints = solve_cube_pick(
                    candidate, arm, kin, data.get("pick_x_offset_mm", 0.0),
                    data.get("pick_z_offset_mm", 0.0))
                break
            except IKNoSolution as exc:
                skipped.append({"cube_id": candidate.get("cube_id"), "reply": str(exc)})
        else:
            raise IKNoSolution("Không cube nào trong lượt có pose gắp IK hợp lệ: " +
                               "; ".join(item["reply"] for item in skipped))"""

new_candidates_logic = """        candidates = data.get("candidates") if command == "sort_cube_candidates" else [data]
        if not isinstance(candidates, list) or not candidates:
            raise ValueError("Cần ít nhất một cube đã xác nhận để phân loại")
        skipped = []
        
        if command == "sort_cube_3d":
            candidate_index = 0
            cube_id = data.get("cube_id")
            x, y, z = data.get("x"), data.get("y"), data.get("z")
            check_readback(arm, READY_POSE)
            if not gripper_is_open(arm):
                raise RuntimeError("Kẹp chưa mở tại pose quan sát")
            joints = None
            for z_try in (z, 0.058, 0.068):
                try:
                    joints = kin.ik(x, y, z_try)
                    z = z_try
                    break
                except IKNoSolution:
                    pass
            if joints is None:
                raise IKNoSolution(f"Không có giải pháp IK cho 3D pose ({x:.3f}, {y:.3f}, {z:.3f})")
            # T8 grasp: J5=J1 if not full 6DOF
            if len(joints) >= 5:
                joints[4] = joints[0]
        else:
            for candidate_index, candidate in enumerate(candidates):
                if not isinstance(candidate, dict):
                    raise ValueError("Dữ liệu cube không hợp lệ")
                try:
                    cube_id, (x, y, z), joints = solve_cube_pick(
                        candidate, arm, kin, data.get("pick_x_offset_mm", 0.0),
                        data.get("pick_z_offset_mm", 0.0))
                    break
                except IKNoSolution as exc:
                    skipped.append({"cube_id": candidate.get("cube_id"), "reply": str(exc)})
            else:
                raise IKNoSolution("Không cube nào trong lượt có pose gắp IK hợp lệ: " +
                                   "; ".join(item["reply"] for item in skipped))"""

content = content.replace(old_candidates_logic, new_candidates_logic)

# Replace the command in the big if statement near the end
content = content.replace('("pick", "preflight_stack", "preflight_cube_pick", "place_target", "sort_cube_zone", "sort_cube_candidates")',
                          '("pick", "preflight_stack", "preflight_cube_pick", "place_target", "sort_cube_zone", "sort_cube_candidates", "sort_cube_3d")')

with open(path, "w") as f:
    f.write(content)
