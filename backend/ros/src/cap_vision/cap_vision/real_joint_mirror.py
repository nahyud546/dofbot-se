"""Mirror measured servo positions to /real_joint_states (read-only).

RViz robot_state_publishers need a continuous /real_joint_states source.
The fake node publishes a frozen READY pose, so RViz never follows the
real arm. This node polls the STM32 servos and publishes measured joints.

Serial safety (shared with t8_motion_worker.py):
- Takes /tmp/t8_motion.lock (LOCK_EX|NB) each cycle; skips the cycle
  while the worker owns it, so motion commands are never interleaved.
- Opens /dev/ttyUSB0, reads 6 servos, closes it every cycle. The port
  is never held open, so the worker's `fuser` guard never false-fires.
- Skips while /tmp/dofbot_task_active exists (another task owns motion).

Robustness: Arm_Lib serial reads are blocking and can wedge for many
seconds on misaligned bytes. All serial I/O therefore runs in a
dedicated daemon thread with a per-cycle time budget; the ROS timer
only publishes the last cached reading (fresh < 3 s). A wedged serial
read can never freeze ROS callbacks nor hold the t8 lock forever.

Arm_Lib readback uses the same logical degrees as the T8 IK/FK service.
Arm joints therefore use radians(readback - 90); do not invert 2/3/4 again.
Gripper mimic joints (Humble ignores <mimic>) are resolved explicitly.

Run with the real arm connected:
  ros2 launch cap_vision object_pipeline.launch.py device_index:=2 \\
      fake_joints:=false real_joints:=true
"""
import fcntl
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

LOCK_FILE = Path("/tmp/t8_motion.lock")
ACTIVE_TASK = Path("/tmp/dofbot_task_active")
SERIAL = "/dev/ttyUSB0"

ARM_JOINTS = [f"arm{i}_Joint" for i in range(1, 6)]
GRIPPER_JOINTS = ["Rlink1_Joint", "Llink1_Joint", "Rlink2_Joint",
                  "Llink2_Joint", "Rlink3_Joint", "Llink3_Joint"]
GRIPPER_SIGN = [1.0, -1.0, -1.0, 1.0, 1.0, -1.0]

JOINT_TTL_S = 5.0
READ_ATTEMPTS = 3
# Deadline moi servo. Servo 1 (khop de) cham tra loi sau khi mo cong nen
# doc no CUOI CUNG (cong da on dinh) + giu gia tri cu khi lo 1 chu ky.
SERVO_DEADLINE_S = 1.2
SERVO_ORDER = (2, 3, 4, 5, 6, 1)
WARN_REPEAT_S = 30.0

HARDWARE = None
for _parent in Path(__file__).resolve().parents:
    _cand = _parent / "hardware" / "safety_gate.py"
    if _cand.is_file():
        HARDWARE = _cand.parent
        break
if HARDWARE is None:
    raise ImportError("khong tim thay hardware/safety_gate.py tu tren cay thu muc cua node")
sys.path.insert(0, str(HARDWARE))


def fine_angle(arm, sid, coarse):
    """0.08-degree angle from the raw count; Arm_Lib truncates to an integer."""
    try:
        if int(arm.id) != sid + 0x30:
            return float(coarse)
        raw = int(arm.servo_H) * 256 + int(arm.servo_L)
    except (AttributeError, TypeError, ValueError):
        return float(coarse)
    if sid == 5:
        angle = 270.0 * (raw - 380) / (3700 - 380)
    else:
        angle = 180.0 * (raw - 900) / (3100 - 900)
        if sid in (2, 3, 4):
            angle = 180.0 - angle
    return angle if abs(angle - float(coarse)) <= 1.01 else float(coarse)


def read_servo(arm, sid, attempts=READ_ATTEMPTS):
    t0 = time.monotonic()
    for _ in range(attempts):
        if time.monotonic() - t0 > SERVO_DEADLINE_S:
            break
        try:
            value = arm.Arm_serial_servo_read(sid)
            if value is not None:
                angle = float(value)
                if math.isfinite(angle):
                    return fine_angle(arm, sid, angle)
        except Exception:
            pass
        time.sleep(0.1)
    raise RuntimeError(f"servo {sid} khong co readback (>{SERVO_DEADLINE_S}s)")


def measured_joint_rad(joint, readback_deg):
    """Convert logical Arm_Lib readback, consistent with T8 KDL FK.

    Arm_serial_servo_read already reverses the wire angles of servos 2–4.
    safety_gate's mirrored write mapping is not a readback conversion here.
    """
    if joint in ARM_JOINTS:
        return math.radians(float(readback_deg) - 90.0)
    from safety_gate import servo_deg_to_rad
    return servo_deg_to_rad(joint, readback_deg)


class RealJointMirror(Node):
    def __init__(self):
        super().__init__("real_joint_mirror")
        self.declare_parameter("port", SERIAL)
        self.declare_parameter("period_s", 0.5)
        self.port = str(self.get_parameter("port").value)
        self.pub = self.create_publisher(JointState, "/real_joint_states", 10)
        self._warned = ""
        self._warned_at = 0.0
        self._published_once = False
        self._last_published_ns = 0
        # Cache rieng tung khop: servo 1 tra loi cham/that thuong xuyen,
        # giu gia tri cu trong JOINT_TTL_S thay vi bo ca chu ky.
        self._joints = {}
        self._cache_lock = threading.Lock()
        self._stop = threading.Event()
        self._convert = measured_joint_rad
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self.create_timer(float(self.get_parameter("period_s").value),
                          self._on_timer)
        self.get_logger().info(
            f"mirroring {self.port} -> /real_joint_states (read-only, ton trong t8 lock)")

    def _warn(self, text):
        now = time.monotonic()
        # Log lap lai moi WARN_REPEAT_S de lan sau con chan doan duoc
        # (khong nen nhu truoc: cung text la im vinh vien).
        if text != self._warned or now - self._warned_at > WARN_REPEAT_S:
            self._warned, self._warned_at = text, now
            if text:
                self.get_logger().warning(text)

    def _on_timer(self):
        names = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
        now = time.monotonic()
        with self._cache_lock:
            vals = [self._joints.get(n) for n in names]
        if any(v is None or now - v[1] > JOINT_TTL_S for v in vals):
            self._warn("chua du joint that moi (cho serial/worker)")
            return
        self._warn("")
        if not self._published_once:
            self._published_once = True
            self.get_logger().info("dang publish /real_joint_states tu servo that")
        msg = JointState()
        # A cached servo position must retain its measurement time. Stamping
        # it as "now" would make stale joints look synchronized with an image.
        oldest_ns = min(v[2] for v in vals[:5])
        if oldest_ns <= self._last_published_ns:
            return
        self._last_published_ns = oldest_ns
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(oldest_ns, 1_000_000_000)
        msg.name = names
        msg.position = [float(v[0]) for v in vals]
        self.pub.publish(msg)

    def _poll_loop(self):
        import Arm_Lib
        while not self._stop.is_set():
            try:
                if ACTIVE_TASK.exists():
                    self._warn("task khac dang chay; tam dung mirror")
                else:
                    self._poll_once(Arm_Lib)
            except Exception as exc:
                self._warn(f"mirror loi: {exc}")
            time.sleep(0.2)

    def _poll_once(self, Arm_Lib):
        # Doc ca 6 servo trong 1 lan giu lock (mo cong 1 lan): servo 1 cham
        # sau khi mo cong nen doc CUOI CUNG; servo nao lo giu gia tri cu.
        raw = {}
        try:
            with LOCK_FILE.open("a+") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                try:
                    if not Path(self.port).exists():
                        raise RuntimeError(f"khong thay cong {self.port}")
                    arm = Arm_Lib.Arm_Device(self.port)
                    try:
                        # Giu timeout mac dinh cua lib (0.2s): STM32 can thoi
                        # gian hoi servo that; timeout ngan hon lam mat reply.
                        try:
                            arm.ser.reset_input_buffer()
                        except Exception:
                            pass
                        # Warmup: lan doc dau sau khi mo cong hay fail (cong
                        # chua on dinh). Doc mo servo 6 nhanh nhat roi bo.
                        time.sleep(0.3)
                        try:
                            read_servo(arm, 6)
                        except RuntimeError:
                            pass
                        for sid in SERVO_ORDER:
                            try:
                                raw[sid] = (read_servo(arm, sid),
                                            self.get_clock().now().nanoseconds)
                            except RuntimeError:
                                pass
                    finally:
                        try:
                            arm.ser.close()
                        except Exception:
                            pass
                        del arm
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)
        except BlockingIOError:
            return  # worker dang giu lock -> bo qua chu ky nay
        if not raw:
            return
        now = time.monotonic()
        with self._cache_lock:
            for sid, (value, measured_ns) in raw.items():
                joint = ARM_JOINTS[sid - 1] if sid <= 5 else "Rlink1_Joint"
                rad = float(self._convert(joint, value))
                self._joints[joint] = (rad, now, measured_ns)
                if joint == "Rlink1_Joint":
                    for n, s in zip(GRIPPER_JOINTS[1:], GRIPPER_SIGN[1:]):
                        self._joints[n] = (rad * s, now, measured_ns)


def main(args=None):
    rclpy.init(args=args)
    node = RealJointMirror()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node._stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
