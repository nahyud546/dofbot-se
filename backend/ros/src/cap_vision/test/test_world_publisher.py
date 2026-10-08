import math

import numpy as np

from cap_vision import world_publisher as W


def rot(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def from_quaternion(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_quaternion_round_trips_including_half_turns():
    for axis, angle in (((0, 0, 1), 0.3), ((1, 0, 0), math.pi), ((0, 1, 0), math.pi), ((0, 0, 1), math.pi),
                        ((1, 2, 3), 2.9), ((-1, 0.2, 0.5), -1.7)):
        R = rot(axis, angle)
        q = W.quaternion(R)
        assert abs(sum(v * v for v in q) - 1.0) < 1e-9
        np.testing.assert_allclose(from_quaternion(q), R, atol=1e-9)


def test_frustum_reaches_the_image_corners_at_the_given_depth():
    lines = W.frustum_lines((900.0, 900.0, 640.0, 360.0), (1280, 720), depth=0.2)
    assert len(lines) == 8
    corner = lines[0][1]
    np.testing.assert_allclose(corner, [-640 / 900 * 0.2, -360 / 900 * 0.2, 0.2])


def test_missing_or_broken_world_file_is_none(tmp_path):
    assert W.load_world(tmp_path / "none.json") is None
    (tmp_path / "bad.json").write_text("{not json")
    assert W.load_world(tmp_path / "bad.json") is None
    (tmp_path / "ok.json").write_text('{"tags": {}}')
    assert W.load_world(tmp_path / "ok.json") == {"tags": {}}
