#!/usr/bin/env python3
"""L2 STUB planner cho T8 — thay thế model_service/Dify.

Nghe topic `asr` -> rule keyword -> JSON {"action":[...],"response":"..."}
-> validate whitelist (mirror utils/promot.py) -> pub topic `stub_plan`.

Muc dich L2 (theo docs/yahboom_task_test_plan.md muc 13):
  - Verify hop `asr -> nao -> JSON` ma KHONG can Dify/key/mang.
  - Verify moi action sinh ra deu nam trong whitelist (LLM that cung phai tuan thu).
  - Chua noi action_service (L3), chua servo.

Chay:  source /opt/ros/humble/setup.bash && python3 stub_planner.py
Test:   ros2 topic pub -1 /asr std_msgs/msg/String "{data: 'bat den do'}"
        ros2 topic echo /stub_plan
"""
import json
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Whitelist rut gon tu utils/promot.py :: action_function_library.
# model_service that bat buoc action nam trong thu vien nay.
ALLOWED = {
    "light_on", "light_off", "beep_on", "beep_off", "set_pose",
    "arm_up", "arm_down", "arm_nod", "arm_shake", "arm_applaud", "arm_dance",
    "grasp_obj", "putdown", "apriltag_sort", "track", "apriltag_remove_higher",
    "color_remove_higher", "adjust_joint", "arm_move",
    "gripper_open", "gripper_close", "grip_pose", "track_pose",
    "garbage_sort", "change_pose", "compute_pose", "return_to_orin",
    "compute_pose_order", "color_back_to_orin",
    "grasp_from_rm_list", "grasp_from_down_list",
    "cancel_apriltag_follow", "cancel_KCF_follow", "point_to",
    "seewhat", "record_video", "video_understanding",
    "check_remove", "finish_dialogue", "finishtask", "wait",
    "web_search",
}

FUNC_RE = re.compile(r"^([a-z_][a-z0-9_]*)\s*\(")


def validate(actions):
    """Rua action list ve whitelist; tra ve (clean_list, bi_loai_list)."""
    clean, dropped = [], []
    for a in actions:
        m = FUNC_RE.match(a.strip())
        if m and m.group(1) in ALLOWED:
            clean.append(a.strip())
        else:
            dropped.append(a)
    if not clean:
        clean = ["finishtask()"]
    return clean, dropped


def plan(text):
    """Rule keyword tam thay TaskDecision+TaskExecution (stub, khong phai LLM)."""
    t = text.lower()
    if "den do" in t or ("den" in t and "do" in t):
        return (["light_on('red')"], "De roi, bat den do cho ban.")
    if "den xanh" in t:
        return (["light_on('green')"], "De roi, bat den xanh cho ban.")
    if "tat den" in t:
        return (["light_off()"], "Tat den xong.")
    if "coi" in t or "bao dong" in t:
        if "tat" in t:
            return (["beep_off()"], "Tat coi xong.")
        return (["beep_on()"], "Bat coi bao dong.")
    if "nhay" in t or "mua" in t:
        return (["arm_dance()"], "Xem to nhay ne.")
    if "gat" in t:
        return (["arm_nod()"], "Gat dau dong y.")
    if "lac" in t:
        return (["arm_shake()"], "Lac dau khong dong y.")
    if "nhin" in t or "thay gi" in t or "seewhat" in t:
        return (["seewhat()"], "De to nhin mot chut.")
    if "chao" in t or "hello" in t:
        return (["finishtask()"], "Chao ban, to la robot de ban.")
    # Mac dinh: tro chuyen thuan tuy, khong action (giong rule 'hoi thoi tiet' trong prompt).
    return (["finishtask()"], f"To nghe '{text}', nhung stub L2 chua biet lam viec nay.")


class StubPlanner(Node):
    def __init__(self):
        super().__init__("stub_planner")
        self.sub = self.create_subscription(String, "asr", self.on_asr, 10)
        self.pub = self.create_publisher(String, "stub_plan", 10)
        self.get_logger().info("stub_planner san sang (L2, khong can Dify)")

    def on_asr(self, msg):
        raw_actions, response = plan(msg.data)
        actions, dropped = validate(raw_actions)
        if dropped:
            self.get_logger().warn(f"Loai action ngoai whitelist: {dropped}")
        out = json.dumps({"action": actions, "response": response},
                         ensure_ascii=False)
        self.get_logger().info(f"asr='{msg.data}' -> {out}")
        self.pub.publish(String(data=out))


def main(args=None):
    rclpy.init(args=args)
    node = StubPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
