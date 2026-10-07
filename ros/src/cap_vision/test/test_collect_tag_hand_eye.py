"""Regression for camera frames arriving faster than measured joint TF."""

from collections import deque
from types import SimpleNamespace

from cap_vision.collect_tag_hand_eye import matching_capture_image


def frame(seconds):
    sec = int(seconds)
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=int((seconds - sec) * 1e9))))


def test_older_bracketed_frame_is_accepted_when_latest_tf_is_not_ready():
    images = deque([frame(9.5), frame(9.9)], maxlen=64)
    stationary = SimpleNamespace(ready=lambda stamp, age: True)
    selected = matching_capture_image(
        images, 10.0, stationary, lambda stamp: stamp.sec == 9 and stamp.nanosec < 600_000_000)
    assert selected is images[0]


def test_old_or_moving_frame_is_rejected():
    images = deque([frame(7.0), frame(9.5)])
    stationary = SimpleNamespace(ready=lambda stamp, age: False)
    assert matching_capture_image(images, 10.0, stationary, lambda stamp: True) is None
