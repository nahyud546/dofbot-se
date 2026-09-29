#!/usr/bin/env python3
# coding: utf-8
"""Robot state service: đọc góc khớp hiện tại + cache (cho rotate_relative state-aware).

- get_joints(): đọc 6 servo qua Arm_Lib.Arm_serial_servo_read (đọc 2 lần, bỏ lần đầu
  vì byte stale — xem log/2026-09-23). Trả [deg x6] hoặc None nếu hardware vắng.
- get_joint(joint_id): tiện cho rotate 1 khớp.
- CachedState: giữ last_read + timestamp để executor không spam serial.
- Không có hardware (laptop test) => trả None, executor chuyển sang dry-run/mock.
"""
import time

try:
    import Arm_Lib  # type: ignore
    _HAS_ARM = True
except ImportError:
    _HAS_ARM = False


def _open_arm(com="/dev/myserial"):
    import os
    try:
        return Arm_Lib.Arm_Device(com=com)
    except Exception:
        if com == "/dev/myserial" and os.path.exists("/dev/ttyUSB0"):
            try:
                return Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
            except Exception:
                return None
        return None


def read_joint_raw(arm, joint_id, tries=2):
    """Đọc 1 khớp, bỏ lần đầu (stale byte)."""
    val = None
    for _ in range(max(1, tries)):
        try:
            val = arm.Arm_serial_servo_read(int(joint_id))
        except Exception:
            val = None
    return val


def get_joints(com="/dev/myserial", tries=2):
    """Trả [j1..j6] độ hoặc None nếu không có hardware."""
    if not _HAS_ARM:
        return None
    arm = _open_arm(com)
    if arm is None:
        return None
    try:
        out = []
        for j in range(1, 7):
            v = read_joint_raw(arm, j, tries)
            if v is None:
                return None
            out.append(float(v))
        return out
    finally:
        try:
            del arm
        except Exception:
            pass


def get_joint(joint_id, com="/dev/myserial"):
    js = get_joints(com)
    if js is None:
        return None
    if 1 <= int(joint_id) <= 6:
        return js[int(joint_id) - 1]
    return None


class CachedState:
    """Cache góc + trạng thái holding để executor dùng."""

    def __init__(self, ttl=1.0):
        self.ttl = ttl
        self._joints = None
        self._t = 0.0
        self.holding = False
        self.held_label = ""

    def read(self, force=False):
        now = time.monotonic()
        if not force and self._joints is not None and (now - self._t) < self.ttl:
            return self._joints
        js = get_joints()
        if js is not None:
            self._joints, self._t = js, now
        return self._joints

    def mark_held(self, label):
        self.holding, self.held_label = True, label

    def mark_released(self):
        self.holding, self.held_label = False, ""
