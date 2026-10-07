#!/usr/bin/env python3
"""Lập kế hoạch quét tìm cube theo J1 (thuần tính toán, test được không cần tay).

Khác bản ping-pong cũ:
  * thấy cube lúc đang quét theo hướng nào cũng quay ĐÚNG phía có cube ngay
    (hướng quét được đặt lại theo vị trí cube trong ảnh), không đi tiếp rồi mới
    quay lại;
  * cube cách tâm ảnh xa thì bước lớn theo góc nhìn (bearing) thay vì bị chặn 2°;
  * mất cube thì quét ưu tiên phía lần thấy cuối vài bước rồi mới ping-pong;
  * mọi tốc độ gom trong SearchTuning (NORMAL = số cũ, FAST = nhanh hơn ~40%).
Quy ước: J1 tăng = hướng quét `direction = +1` (cùng quy ước proportional_j1:
step = -kp * (cx - 320)).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SearchTuning:
    name: str = "normal"
    sweep_min: float = 20.0
    sweep_max: float = 160.0
    sweep_step: float = 10.0       # độ/bước khi chưa thấy
    sweep_ms: int = 500
    sweep_delay: float = 0.6
    kp: float = 0.015              # độ/px khi đã thấy
    max_step: float = 2.0
    track_ms: int = 400
    track_delay: float = 0.5
    steer_far_px: float = 0.0      # >0: lệch hơn ngưỡng này thì bước theo bearing
    steer_gain: float = 0.7
    steer_max: float = 12.0
    reacquire_steps: int = 0       # số bước ưu tiên phía lần thấy cuối sau khi mất
    reacquire_scale: float = 0.5


NORMAL = SearchTuning()
FAST = SearchTuning(name="fast", sweep_step=14.0, sweep_ms=350, sweep_delay=0.35,
                    kp=0.03, max_step=4.0, track_ms=250, track_delay=0.3,
                    steer_far_px=60.0, reacquire_steps=3)
TUNINGS = {"normal": NORMAL, "fast": FAST}


@dataclass(frozen=True)
class Move:
    j1: float
    ms: int
    delay: float
    reason: str


class SearchPlanner:
    def __init__(self, tuning: SearchTuning, j1: float, direction: int = 1,
                 fx: float = 902.0, center_x: float = 320.0):
        self.t, self.j1, self.direction = tuning, float(j1), 1 if direction >= 0 else -1
        self.fx, self.center_x = fx, center_x
        self.last_side = 0
        self.reacquire_left = 0
        self._last_step = 0.0

    def _clamp(self, j1: float) -> float:
        return max(self.t.sweep_min, min(self.t.sweep_max, j1))

    def _pingpong(self, step: float) -> float:
        nxt = self.j1 + self.direction * step
        if nxt >= self.t.sweep_max:
            self.direction = -1
            return self.t.sweep_max
        if nxt <= self.t.sweep_min:
            self.direction = 1
            return self.t.sweep_min
        return nxt

    def miss(self) -> Move:
        """Chưa thấy cube: bước nhanh."""
        if self.reacquire_left > 0 and self.last_side:
            self.reacquire_left -= 1
            self.direction = self.last_side
            reason = "reacquire"
            self.j1 = self._pingpong(self.t.sweep_step * self.t.reacquire_scale)
        else:
            reason = "sweep"
            self.j1 = self._pingpong(self.t.sweep_step)
        self._last_step = 0.0
        return Move(self.j1, self.t.sweep_ms, self.t.sweep_delay, reason)

    def step_for(self, cx: float) -> float:
        err = float(cx) - self.center_x
        if self.t.steer_far_px and abs(err) > self.t.steer_far_px:
            bearing = math.degrees(math.atan(err / self.fx))
            step = -bearing * self.t.steer_gain
            return max(-self.t.steer_max, min(self.t.steer_max, step))
        step = -self.t.kp * err
        return max(-self.t.max_step, min(self.t.max_step, step))

    def seen(self, cx: float) -> Move:
        """Thấy cube ở cột ảnh cx: bước về phía cube, đặt lại hướng quét."""
        step = self.step_for(cx)
        if self._last_step and step * self._last_step < 0:
            step *= 0.5                      # vượt quá tâm: giảm bước để không dao động
        if step:
            self.direction = 1 if step > 0 else -1
            self.last_side = self.direction
        self.reacquire_left = self.t.reacquire_steps
        self._last_step = step
        self.j1 = self._clamp(self.j1 + step)
        return Move(self.j1, self.t.track_ms, self.t.track_delay, "track")
