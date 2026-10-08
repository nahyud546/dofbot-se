import math

import pytest

import search_planner as P


def sweep_until_seen(planner, cube_j1, fov_deg=18.0, max_steps=200):
    """Mô phỏng: thấy cube khi |cube_j1 - j1| <= fov/2; trả quỹ đạo J1."""
    path = [planner.j1]
    for _ in range(max_steps):
        if abs(cube_j1 - planner.j1) <= fov_deg / 2:
            return path, True
        planner.miss()
        path.append(planner.j1)
    return path, False


def test_ping_pong_matches_the_old_sweep_rules():
    p = P.SearchPlanner(P.NORMAL, 155.0)
    assert (p.miss().j1, p.direction) == (160.0, -1)
    p = P.SearchPlanner(P.NORMAL, 25.0, direction=-1)
    assert (p.miss().j1, p.direction) == (20.0, 1)
    p = P.SearchPlanner(P.NORMAL, 90.0)
    assert p.miss().j1 == 100.0


def test_seeing_the_cube_on_the_far_side_reverses_the_sweep():
    p = P.SearchPlanner(P.FAST, 120.0, direction=1)       # sweeping toward larger J1
    p.miss()
    assert p.direction == 1
    # cube appears at the LEFT edge of the image: err<0 -> step>0 here means J1 up; use the other side
    move = p.seen(320 + 250)                              # right of centre -> J1 must go down (step = -kp*err)
    assert move.j1 < 134.0 and p.direction == -1          # reversed immediately
    move = p.seen(320 - 250)                              # then it overshoots to the other side
    assert p.direction == 1


def test_far_cube_takes_a_big_bearing_step_and_near_cube_a_small_one():
    p = P.SearchPlanner(P.FAST, 90.0)
    far = p.seen(320 + 280)
    assert 90.0 - far.j1 == pytest.approx(
        min(P.FAST.steer_max, 0.7 * math.degrees(math.atan(280 / 902.0))), abs=0.05)
    q = P.SearchPlanner(P.FAST, 90.0)
    near = q.seen(320 + 40)
    assert abs(near.j1 - 90.0) == pytest.approx(P.FAST.kp * 40, abs=1e-6)
    old = P.SearchPlanner(P.NORMAL, 90.0).seen(320 + 280)
    assert abs(old.j1 - 90.0) <= P.NORMAL.max_step + 1e-9           # old behaviour is untouched


def test_track_converges_without_oscillating():
    """Closed loop: pixel offset = fx*tan(cube_bearing - j1) (+ small camera lag ignored)."""
    cube = 63.0
    p = P.SearchPlanner(P.FAST, 90.0)
    errors = []
    for _ in range(40):
        cx = 320 + 902.0 * math.tan(math.radians(p.j1 - cube))     # cube is at lower J1 -> right of centre... sign: err = (cx-320)
        errors.append(abs(p.j1 - cube))
        p.seen(cx)
    assert errors[-1] < 0.8 and max(errors[10:]) < errors[0]
    signs = [e for e in errors[-10:]]
    assert max(signs) < 1.5                                         # settled, not hunting


def test_lost_cube_is_searched_on_the_last_seen_side_first():
    p = P.SearchPlanner(P.FAST, 100.0, direction=1)
    p.seen(320 + 200)                                             # cube was to the J1-down side
    j = p.j1
    moves = [p.miss() for _ in range(P.FAST.reacquire_steps)]
    assert all(m.reason == "reacquire" for m in moves)
    assert moves[-1].j1 < j                                       # kept going down
    assert p.miss().reason == "sweep"


def test_fast_is_faster_than_normal_and_stays_in_limits():
    assert P.FAST.sweep_ms < P.NORMAL.sweep_ms and P.FAST.sweep_delay < P.NORMAL.sweep_delay
    assert P.FAST.sweep_step > P.NORMAL.sweep_step and P.FAST.track_delay < P.NORMAL.track_delay
    p = P.SearchPlanner(P.FAST, 155.0)
    for _ in range(40):
        j = p.miss().j1
        assert P.FAST.sweep_min <= j <= P.FAST.sweep_max


def test_sweep_still_finds_a_cube_anywhere():
    for cube in (25, 60, 95, 130, 155):
        _, found = sweep_until_seen(P.SearchPlanner(P.FAST, 90.0), cube)
        assert found, cube
