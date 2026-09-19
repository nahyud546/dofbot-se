"""Dofbot follower: 1 class, 2 backend (sim ROS2 / tay thật Arm_Lib).

Mục tiêu: sim và hardware thật chia sẻ cùng observation/action features để
dataset record trên sim vẫn train/eval được, và policy rollout không đổi code.

- sim_ros2: sub /joint_states, gửi FollowJointTrajectory + GripperCommand.
  Cần venv có rclpy (ROS Humble py3.10). Trong venv lerobot py3.12 không có
  rclpy -> raise lỗi hướng dẫn chạy bridge ở venv ROS.
- arm_lib: wrap Yahboom Arm_Lib.Arm_Device (đã có sẵn 0.0.5 trên image).
  Đơn vị servo là ĐỘ: s1-s4,s6 0-180, s5 0-270, time_ms là thời gian chạy.
- mock=True: chạy CI/unit-test không cần hardware hay ROS.
"""

from __future__ import annotations

import logging
import math
import time
from functools import cached_property

from lerobot_robot_dofbot.config_dofbot import (
    ARM_JOINTS,
    GRIPPER_JOINT,
    DofbotFollowerConfig,
)

logger = logging.getLogger(__name__)

# Key LeRobot: "<ros_joint>.pos", đơn vị độ để khớp policy cũ.
LEROBOT_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)
LEROBOT_KEYS = tuple(f"{j}.pos" for j in LEROBOT_JOINTS)

# Home an toàn (độ) — khớp simplify-chess-game/config/home.yaml ý nghĩa.
HOME_DEG = (90.0, 90.0, 90.0, 90.0, 90.0, 30.0)


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _deg_to_gripper_rad(deg: float) -> float:
    # Quy ước nội bộ: gripper 0-180deg <-> Rlink1 0-1.57rad (mimic phi tuyến
    # nên đây chỉ là ánh xạ binary-friendly, calibrate thật ở dofbot_common).
    return _clip(deg, 0.0, 180.0) / 180.0 * 1.57


def _gripper_rad_to_deg(rad: float) -> float:
    return _clip(rad, 0.0, 1.57) / 1.57 * 180.0


try:
    from lerobot.robots.robot import Robot as _RobotBase
except ImportError:  # cho phép syntax-check khi chưa cài lerobot
    _RobotBase = object  # type: ignore[assignment,misc]


class DofbotFollower(_RobotBase):  # type: ignore[misc]
    """LeRobot Robot cho Dofbot 6DOF. Tuân thủ interface Robot (v0.6.2)."""

    config_class = DofbotFollowerConfig
    name = "dofbot_follower"

    def __init__(self, config: DofbotFollowerConfig):
        cfg_id = getattr(config, "id", None)
        try:
            super().__init__(config)  # type: ignore[call-arg]
        except Exception:  # noqa: BLE001 - fallback khi base là object/stub
            self.robot_type = self.name
            self.id = cfg_id or "dofbot_default"
            self.calibration: dict = {}
        self.config = config
        # Đảm bảo thuộc tính tối thiểu scripts lerobot-record cần.
        self.robot_type = self.name
        if not getattr(self, "id", None):
            self.id = cfg_id or "dofbot_default"
        if not hasattr(self, "calibration"):
            self.calibration: dict = {}
        self._connected = False
        self._arm = None  # Arm_Lib.Arm_Device khi backend=arm_lib
        self._ros = None  # dict node/action clients khi backend=sim_ros2
        self._cameras: dict = {}
        self._last_q_deg: list[float] = list(HOME_DEG)

    # -- features: gọi được khi chưa connect (bắt buộc theo API) --
    @cached_property
    def observation_features(self) -> dict:
        feats: dict = {k: float for k in LEROBOT_KEYS}
        for cam, cfg in (self.config.cameras or {}).items():
            h = cfg.get("height", 480) if isinstance(cfg, dict) else 480
            w = cfg.get("width", 640) if isinstance(cfg, dict) else 640
            feats[cam] = (h, w, 3)
        return feats

    @cached_property
    def action_features(self) -> dict:
        return {k: float for k in LEROBOT_KEYS}

    @property
    def is_connected(self) -> bool:
        if self.config.cameras:
            try:
                if not all(c.is_connected for c in self._cameras.values()):
                    return False
            except Exception:
                return False
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return
        backend = self.config.backend
        if self.config.mock:
            self._connected = True
            logger.info(f"{self} connected (mock, backend={backend}).")
            return
        if backend == "arm_lib":
            self._connect_arm_lib()
        elif backend == "sim_ros2":
            self._connect_ros2()
        else:
            raise ValueError(f"backend={backend!r} không hỗ trợ (chọn 'sim_ros2'|'arm_lib').")
        self._connect_cameras()
        if calibrate and not self.is_calibrated:
            self.calibrate()
        self.configure()
        self._connected = True
        logger.info(f"{self} connected (backend={backend}).")

    @property
    def is_calibrated(self) -> bool:
        # Arm_Lib không có quy trình calibrate offset như Feetech -> luôn True.
        # Sim cũng True vì joint_states đã là tuyệt đối.
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        if self.config.backend == "arm_lib" and self._arm is not None:
            try:
                # Mở torque để servo nhận lệnh (id=6 tay Dofbot).
                self._arm.Arm_serial_set_torque(1)
            except Exception as exc:  # noqa: BLE001 - hardware best-effort
                logger.warning("Arm_Lib torque enable thất bại: %s", exc)

    def get_observation(self) -> dict:
        if not self._connected:
            raise RuntimeError("Chưa connect: gọi robot.connect() trước.")
        if self.config.mock:
            obs = {k: float(v) for k, v in zip(LEROBOT_KEYS, self._last_q_deg, strict=True)}
        elif self.config.backend == "arm_lib":
            obs = self._read_arm_lib()
        else:
            obs = self._read_ros2()
        for cam, handle in self._cameras.items():
            try:
                frame = handle.async_read() if hasattr(handle, "async_read") else handle.read()
                if frame is not None:
                    obs[cam] = frame
            except Exception as exc:  # noqa: BLE001
                logger.warning("camera %s read thất bại: %s", cam, exc)
        return obs

    def send_action(self, action: dict) -> dict:
        if not self._connected:
            raise RuntimeError("Chưa connect: gọi robot.connect() trước.")
        # Strip ".pos", clip theo joint_limits_deg -> single source of truth ở config.
        q_deg: list[float] = []
        for joint in LEROBOT_JOINTS:
            key = f"{joint}.pos"
            if key not in action:
                raise KeyError(f"Thiếu {key} trong action (cần đủ {LEROBOT_KEYS}).")
            lo, hi = self.config.joint_limits_deg.get(joint, (0.0, 180.0))
            q_deg.append(_clip(float(action[key]), lo, hi))
        sent = {k: float(v) for k, v in zip(LEROBOT_KEYS, q_deg, strict=True)}
        if self.config.mock:
            self._last_q_deg = q_deg
            return sent
        if self.config.backend == "arm_lib":
            self._write_arm_lib(q_deg)
        else:
            self._write_ros2(q_deg)
        self._last_q_deg = q_deg
        return sent

    def disconnect(self) -> None:
        try:
            for cam in self._cameras.values():
                try:
                    cam.disconnect()
                except Exception:  # noqa: BLE001
                    pass
            if self._ros is not None:
                try:
                    self._ros["node"].destroy_node()
                except Exception:  # noqa: BLE001
                    pass
        finally:
            self._ros = None
            self._arm = None
            self._cameras = {}
            self._connected = False

    def __str__(self) -> str:
        return f"{self.id} DofbotFollower({self.config.backend})"

    # -- backend: Arm_Lib (tay thật) --
    def _connect_arm_lib(self) -> None:
        try:
            import Arm_Lib  # noqa: PLC0415 - chỉ có trên image Yahboom py3.10
        except ImportError as exc:
            raise ImportError(
                "Thiếu Arm_Lib (chỉ có trên image Yahboom). Cài driver hoặc "
                "dùng backend='sim_ros2'/'mock=True' để test."
            ) from exc
        self._arm = Arm_Lib.Arm_Device()
        time.sleep(0.5)
        # Đọc thử 1 servo để xác nhận bus sống.
        try:
            self._read_arm_lib()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Arm_Lib connect nhưng đọc servo thất bại: %s", exc)

    def _read_arm_lib(self) -> dict:
        assert self._arm is not None
        qs: list[float] = []
        for sid in range(1, 7):
            try:
                v = self._arm.Arm_serial_servo_read_any(sid)
            except Exception:  # noqa: BLE001
                v = None
            if v is None:
                # Giữ giá trị cuối để observation không đứt khi 1 servo miss.
                v = self._last_q_deg[sid - 1]
            qs.append(float(v))
        return {k: float(v) for k, v in zip(LEROBOT_KEYS, qs, strict=True)}

    def _write_arm_lib(self, q_deg: list[float]) -> None:
        assert self._arm is not None
        s1, s2, s3, s4, s5, s6 = (float(v) for v in q_deg)
        self._arm.Arm_serial_servo_write6(s1, s2, s3, s4, s5, s6, self.config.arm_lib_time_ms)

    # -- backend: sim ROS2 --
    def _connect_ros2(self) -> None:
        try:
            import rclpy  # noqa: PLC0415
            from rclpy.action import ActionClient  # noqa: PLC0415
            from control_msgs.action import FollowJointTrajectory  # noqa: PLC0415
            from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: PLC0415 - giữ để type-check ở venv ROS
            from sensor_msgs.msg import JointState  # noqa: PLC0415
        except ImportError as exc:
            raise ImportError(
                "backend='sim_ros2' cần rclpy + control_msgs (venv ROS Humble py3.10). "
                "Trong venv lerobot py3.12 hãy chạy bridge ROS riêng, hoặc dùng mock=True để test."
            ) from exc
        _ = (ActionClient, FollowJointTrajectory, JointTrajectoryPoint, JointState)
        rclpy.init(args=None)
        from rclpy.node import Node  # noqa: PLC0415

        node: Node = Node("dofbot_lerobot_bridge")
        latest: dict = {"msg": None}

        def _cb(msg) -> None:
            latest["msg"] = msg

        from sensor_msgs.msg import JointState as _JS  # noqa: PLC0415

        node.create_subscription(_JS, self.config.joint_states_topic, _cb, 10)
        arm_client = ActionClient(node, FollowJointTrajectory, self.config.arm_action_topic)
        # Gripper Sim là GripperCommand action; import lazy để venv thiếu vẫn import được module.
        grip_client = None
        try:
            from control_msgs.action import GripperCommand  # noqa: PLC0415

            from rclpy.action import ActionClient as _AC  # noqa: PLC0415

            grip_client = _AC(node, GripperCommand, self.config.grip_action_topic)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Không tạo grip action client (vẫn chạy arm): %s", exc)
        self._ros = {"node": node, "arm": arm_client, "grip": grip_client, "latest": latest}
        # Chờ joint_states đầu tiên tối đa 5s để fail-loud thay vì record dataset rỗng.
        t0 = time.time()
        while latest["msg"] is None and time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest["msg"] is None:
            logger.warning("Chưa thấy /joint_states sau 5s (sim/backend chưa chạy?).")

    def _read_ros2(self) -> dict:
        assert self._ros is not None
        import rclpy  # noqa: PLC0415

        node = self._ros["node"]
        latest = self._ros["latest"]
        rclpy.spin_once(node, timeout_sec=0.05)
        msg = latest["msg"]
        if msg is None:
            return {k: float(v) for k, v in zip(LEROBOT_KEYS, self._last_q_deg, strict=True)}
        pos = {n: float(p) for n, p in zip(msg.name, msg.position, strict=False)}
        out: dict = {}
        for joint, key in zip(LEROBOT_JOINTS, LEROBOT_KEYS, strict=True):
            if joint in pos:
                v = pos[joint]
                # ros2_control trả rad cho arm; gripper Rlink1 cũng rad -> đổi ra độ.
                out[key] = math.degrees(v) if joint in (*ARM_JOINTS, GRIPPER_JOINT) else float(v)
            else:
                # Fallback: giữ giá trị cuối, tránh vỡ shape observation.
                out[key] = float(self._last_q_deg[LEROBOT_JOINTS.index(joint)])
        # Cập nhật cache để lần miss sau vẫn có số.
        self._last_q_deg = [float(out[k]) for k in LEROBOT_KEYS]
        return out

    def _write_ros2(self, q_deg: list[float]) -> None:
        assert self._ros is not None
        import rclpy  # noqa: PLC0415
        from control_msgs.action import FollowJointTrajectory  # noqa: PLC0415
        from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: PLC0415
        from builtin_interfaces.msg import Duration  # noqa: PLC0415

        node = self._ros["node"]
        arm_client = self._ros["arm"]
        # Arm: độ -> rad. Gripper tách riêng qua GripperCommand nếu có.
        arm_rad = [math.radians(v) for v in q_deg[:5]]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = arm_rad
        pt.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points = [pt]
        if not arm_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Arm action server {self.config.arm_action_topic} không sẵn sàng.")
        future = arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        # Gripper: gửi position mục tiêu (rad) nếu có action server.
        grip_client = self._ros.get("grip")
        if grip_client is not None:
            try:
                from control_msgs.action import GripperCommand as _GC  # noqa: PLC0415

                g = _GC.Goal()
                g.command.position = _deg_to_gripper_rad(q_deg[5])
                if grip_client.wait_for_server(timeout_sec=1.0):
                    gf = grip_client.send_goal_async(g)
                    rclpy.spin_until_future_complete(node, gf, timeout_sec=2.0)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Gửi gripper thất bại (arm vẫn đã gửi): %s", exc)

    def _connect_cameras(self) -> None:
        if not self.config.cameras:
            return
        try:
            from lerobot.cameras import make_cameras_from_configs  # noqa: PLC0415
        except ImportError:
            logger.warning("config có cameras nhưng lerobot[cameras] chưa cài -> bỏ qua camera.")
            return
        self._cameras = make_cameras_from_configs(self.config.cameras)
        for cam in self._cameras.values():
            try:
                cam.connect()
            except Exception as exc:  # noqa: BLE001
                logger.warning("camera connect thất bại: %s", exc)
