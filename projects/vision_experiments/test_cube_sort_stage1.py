"""Decision and motion checks for automatic cube sorting."""

from pathlib import Path
import os
import pty
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

import cube_sort_stage1 as stage1
from cube_sort_stage1 import pick_candidate
from identify_cube import CubeObservation, CubeVerdict, identify_frame, synthetic_frame

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
import t8_motion_worker as worker


def test_terminal_space_works_without_camera_focus(monkeypatch):
    master, slave = pty.openpty()
    try:
        with os.fdopen(slave, "r") as terminal:
            monkeypatch.setattr(stage1.sys, "stdin", terminal)
            with stage1.TerminalKeys() as keys:
                os.write(master, b" ")
                assert keys.poll() == ord(" ")
    finally:
        os.close(master)


def test_pick_waits_for_confirmed_id_and_current_top_face():
    observation = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    vote = CubeVerdict(1, "blue", "color", score=0.8)
    track = SimpleNamespace(best_observation=observation, observation=observation,
                            last_seen=2.0, confirmed=True)
    assert pick_candidate([(track, vote)], 2.1) == (track, vote)
    track.confirmed = False
    assert pick_candidate([(track, vote)], 2.1) is None
    track.confirmed = True
    assert pick_candidate([(track, CubeVerdict(None, "unknown", "none"))], 2.1) is None
    assert pick_candidate([(track, vote)], 3.0) is None
    other = identify_frame(synthetic_frame("red"), [], annotate=False)[0][0]
    track.observation = other
    assert pick_candidate([(track, vote)], 2.1) is None
    track.observation = observation
    track.last_id_seen = 1.0
    assert pick_candidate([(track, vote)], 2.1) is None


def test_pick_uses_current_top_after_cube_moves():
    original = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    shifted = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    shifted.group.top.corners += [30, 0]
    track = SimpleNamespace(best_observation=original, observation=shifted,
                            last_seen=2.0, last_id_seen=2.0, confirmed=True)
    vote = CubeVerdict(1, "blue", "color", score=0.8)
    assert pick_candidate([(track, vote)], 2.1) == (track, vote)
    assert stage1.target_payload((track, vote))["top_center_px"] == \
        list(shifted.top_center_px)
    assert stage1.target_payload((track, vote))["top_center_px"] != \
        list(original.top_center_px)
    track.association_ambiguous = True
    assert pick_candidate([(track, vote)], 2.1) is None
    track.association_ambiguous = False
    track.observation = CubeObservation(
        type(original.group)(original.group.faces, None, original.group.box),
        original.verdict, original.face_verdicts)
    assert pick_candidate([(track, vote)], 2.1) is None


def test_pick_prioritizes_highest_confidence_cube():
    blue = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    red = identify_frame(synthetic_frame("red"), [], annotate=False)[0][0]
    tracks = [SimpleNamespace(best_observation=observation,
                              observation=observation, last_seen=2.0,
                              confirmed=True)
              for observation in (blue, red)]
    candidates = [(tracks[0], CubeVerdict(1, "blue", "color", score=0.78)),
                  (tracks[1], CubeVerdict(3, "red", "color", score=0.94))]
    assert pick_candidate(candidates, 2.1) == candidates[1]
    # AprilTag decision margin is not a probability and needs normalization.
    candidates[0] = (tracks[0], CubeVerdict(1, "blue", "tag", score=76.0))
    assert pick_candidate(candidates, 2.1) == candidates[0]
    candidates[0] = (tracks[0], CubeVerdict(1, "blue", "tag", score=25.0))
    assert pick_candidate(candidates, 2.1) == candidates[1]
    blocked = stage1.target_payload(candidates[1])
    assert pick_candidate(candidates, 2.1,
                          excluded_targets=[blocked]) == candidates[0]


def test_pick_calibration_offsets_have_explicit_robot_directions():
    quad = [[280, 200], [360, 200], [360, 280], [280, 280]]
    x, y, z, _ = worker.cube_pick_target([320, 240], quad, [640, 480],
                                          x_offset_mm=10, z_offset_mm=-4)
    assert (x, y, z) == (-0.204, 0.0, 0.043)
    with pytest.raises(ValueError, match="Độ bù gắp"):
        worker.cube_pick_target([320, 240], quad, [640, 480], x_offset_mm=21)


def test_sort_worker_uses_top_center_and_correct_bin():
    class Arm:
        def __init__(self):
            self.joints = worker.READY_POSE.copy()
            self.writes = []
            self.beeps = []

        def Arm_Buzzer_On(self, duration):
            self.beeps.append(duration)

        def Arm_serial_servo_read(self, joint):
            return self.joints[joint - 1]

        def Arm_serial_servo_write(self, joint, angle, _ms):
            self.joints[joint - 1] = angle
            self.writes.append((joint, angle))

        def Arm_serial_servo_write6(self, *args):
            self.joints = list(args[:6])
            self.writes.append(tuple(args[:6]))

        def Arm_serial_servo_write6_array(self, joints, _ms):
            self.joints = list(joints)
            self.writes.append(tuple(joints))

    class Kin:
        def __init__(self):
            self.calls = []

        def ik(self, x, y, z):
            self.calls.append((x, y, z))
            return [90, 50, 60, 20, 90]

    payload = {"command": "sort_cube_zone", "cube_id": 2,
               "id_confirmed": True, "geometry_valid": True,
               "top_center_px": [320, 240],
               "top_quad_px": [[280, 200], [360, 200], [360, 280], [280, 280]],
               "image_size": [640, 480]}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        arm, kin = Arm(), Kin()
        result = worker.execute(payload, arm, kin)
        assert result["ok"] and result["cube_id"] == 2
        assert kin.calls == [(-0.214, 0.0, 0.047)]
        assert arm.beeps == [1]
        assert tuple(worker.BIN_POSES[2] + [worker.CLOSE_ANGLE]) in arm.writes
        assert tuple(worker.BIN_RELEASE_POSES[2] + [worker.CLOSE_ANGLE]) in arm.writes
        assert tuple(worker.BIN_LIFT_POSES[2] + [worker.OPEN_ANGLE]) in arm.writes
        assert worker.load_state()["phase"] == "empty"


def test_sort_candidates_skips_unreachable_then_beeps_once_and_sorts():
    class Arm:
        def __init__(self):
            self.joints = worker.READY_POSE.copy()
            self.events = []

        def Arm_serial_servo_read(self, joint):
            return self.joints[joint - 1]

        def Arm_Buzzer_On(self, duration):
            self.events.append(("beep", duration))

        def Arm_serial_servo_write(self, joint, angle, _ms):
            self.joints[joint - 1] = angle
            self.events.append(("move", joint))

        def Arm_serial_servo_write6(self, *args):
            self.joints = list(args[:6])
            self.events.append(("move", 6))

        def Arm_serial_servo_write6_array(self, joints, _ms):
            self.joints = list(joints)
            self.events.append(("move", 6))

    class Kin:
        def ik(self, x, *_args):
            if x < -0.22:
                raise worker.IKNoSolution("first cube unreachable")
            return [90, 50, 60, 20, 90]

    def candidate(cube_id, cy):
        return {"cube_id": cube_id, "id_confirmed": True,
                "geometry_valid": True, "top_center_px": [320, cy],
                "top_quad_px": [[280, cy - 40], [360, cy - 40],
                                [360, cy + 40], [280, cy + 40]],
                "image_size": [640, 480]}

    candidates = [candidate(1, 100), candidate(3, 350)]
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        arm = Arm()
        result = worker.execute({"command": "sort_cube_candidates",
                                 "candidates": candidates}, arm, Kin())
        assert result["ok"] and result["cube_id"] == 3
        assert result["candidate_index"] == 1
        assert [item["cube_id"] for item in result["skipped"]] == [1]
        assert arm.events[0] == ("beep", 1)
        assert sum(event[0] == "beep" for event in arm.events) == 1
        assert worker.load_state()["phase"] == "empty"


def test_sort_candidates_all_ik_fail_without_beep_or_motion():
    class Arm:
        events = []

        def Arm_serial_servo_read(self, joint):
            return worker.READY_POSE[joint - 1]

        def Arm_Buzzer_On(self, duration):
            self.events.append(("beep", duration))

    class Kin:
        def ik(self, *_args):
            raise worker.IKNoSolution("unreachable")

    candidate = {"cube_id": 1, "id_confirmed": True,
                 "geometry_valid": True, "top_center_px": [320, 240],
                 "top_quad_px": [[280, 200], [360, 200],
                                 [360, 280], [280, 280]],
                 "image_size": [640, 480]}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"):
        arm = Arm()
        with pytest.raises(worker.IKNoSolution):
            worker.execute({"command": "sort_cube_candidates",
                            "candidates": [candidate, candidate]}, arm, Kin())
        assert arm.events == []
        assert worker.load_state()["phase"] == "empty"


@pytest.mark.parametrize("change", [{"cube_id": None}, {"id_confirmed": False},
                                    {"geometry_valid": False}])
def test_sort_worker_rejects_unconfirmed_target_without_motion(change):
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"):
        payload = {"command": "sort_cube_zone", "cube_id": 1,
                   "id_confirmed": True, "geometry_valid": True}
        payload.update(change)
        arm = SimpleNamespace(writes=[])
        with pytest.raises((ValueError, RuntimeError)):
            worker.execute(payload, arm, object())
        assert arm.writes == []


def test_sort_worker_ik_failure_keeps_state_empty_and_arm_still():
    class Arm:
        writes = []

        def Arm_serial_servo_read(self, joint):
            return worker.READY_POSE[joint - 1]

    class Kin:
        calls = 0

        def ik(self, *_args):
            self.calls += 1
            raise worker.IKNoSolution("no safe solution")

    payload = {"command": "sort_cube_zone", "cube_id": 1,
               "id_confirmed": True, "geometry_valid": True,
               "top_center_px": [320, 240],
               "top_quad_px": [[280, 200], [360, 200], [360, 280], [280, 280]],
               "image_size": [640, 480]}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        arm, kin = Arm(), Kin()
        with pytest.raises(worker.IKNoSolution):
            worker.execute(payload, arm, kin)
        assert kin.calls == 3
        assert arm.writes == []
        assert worker.load_state()["phase"] == "empty"


def test_sort_worker_does_not_treat_ik_service_error_as_unreachable_pose():
    class Arm:
        def Arm_serial_servo_read(self, joint):
            return worker.READY_POSE[joint - 1]

    class Kin:
        calls = 0

        def ik(self, *_args):
            self.calls += 1
            raise RuntimeError("IK service unavailable")

    payload = {"command": "sort_cube_zone", "cube_id": 1,
               "id_confirmed": True, "geometry_valid": True,
               "top_center_px": [320, 240],
               "top_quad_px": [[280, 200], [360, 200], [360, 280], [280, 280]],
               "image_size": [640, 480]}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        kin = Kin()
        with pytest.raises(RuntimeError, match="IK service unavailable"):
            worker.execute(payload, Arm(), kin)
        assert kin.calls == 1
        assert worker.load_state()["phase"] == "empty"


def test_preflight_cube_pick_is_silent_and_does_not_move_arm():
    class Arm:
        def __init__(self):
            self.beeps = []
            self.writes = []

        def Arm_serial_servo_read(self, joint):
            return worker.READY_POSE[joint - 1]

        def Arm_Buzzer_On(self, duration):
            self.beeps.append(duration)

    class Kin:
        def ik(self, *_args):
            return [90, 50, 60, 20, 90]

    payload = {"command": "preflight_cube_pick", "cube_id": 2,
               "id_confirmed": True, "geometry_valid": True,
               "top_center_px": [320, 240],
               "top_quad_px": [[280, 200], [360, 200], [360, 280], [280, 280]],
               "image_size": [640, 480]}
    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        arm = Arm()
        result = worker.execute(payload, arm, Kin())
        assert result["ok"] and result["cube_id"] == 2
        assert result["target_xyz"] == [-0.214, 0.0, 0.047]
        assert arm.beeps == []
        assert arm.writes == []
        assert worker.load_state()["phase"] == "empty"
        arm.beeps.clear()

        class NoSolution:
            def ik(self, *_args):
                raise worker.IKNoSolution("no safe solution")

        with pytest.raises(worker.IKNoSolution):
            worker.execute(payload, arm, NoSolution())
        assert arm.beeps == []
        assert arm.writes == []
        assert worker.load_state()["phase"] == "empty"


def test_prepare_skips_motion_when_arm_is_already_ready():
    class Arm:
        writes = []

        def Arm_serial_servo_read(self, joint):
            return worker.READY_POSE[joint - 1]

        def Arm_serial_servo_write6_array(self, *_args):
            self.writes.append("move")

    with TemporaryDirectory() as directory, \
            patch.object(worker, "STATE_FILE", Path(directory) / "state.json"), \
            patch("t8_motion_worker.time.sleep"):
        arm = Arm()
        assert worker.execute({"command": "prepare"}, arm)["ok"]
        assert arm.writes == []
        assert worker.load_state()["phase"] == "empty"


def test_stage1_can_pick_same_cube_again_on_next_space(monkeypatch):
    class Camera:
        def __init__(self):
            self.reads = 0
            self.grabs = 0

        def read(self):
            self.reads += 1
            return True, np.zeros((480, 640, 3), dtype=np.uint8)

        def grab(self):
            self.grabs += 1
            return True

        def release(self):
            pass

    class Future:
        def __init__(self, value):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class Executor:
        def __init__(self, **_kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def submit(self, function, *args, **kwargs):
            if function is stage1.run_motion:
                return Future(function(*args, **kwargs))
            events.append("scan")
            return Future([])

    class Averager:
        def update(self, _observations, _now):
            return [], [(track, verdict)]

    verdict = CubeVerdict(1, "blue", "tag", label="tag:1", score=1.0)
    observation = SimpleNamespace(
        geometry_valid=True,
        top_center_px=(320, 240),
        group=SimpleNamespace(top=SimpleNamespace(corners=np.array(
            [[280, 200], [360, 200], [360, 280], [280, 280]]))))
    track = SimpleNamespace(best_observation=observation, observation=observation,
                            last_seen=1.0, confirmed=True, published=verdict)
    camera = Camera()
    events = []
    keys = iter([-1, -1, ord(" "), ord(" "), -1, ord("q")])
    ticks = iter(i * 0.2 for i in range(1, 100))

    def motion(command, **payload):
        events.append(command)
        if command == "prepare":
            return {"ok": True}
        assert command == "sort_cube_candidates"
        assert [item["cube_id"] for item in payload["candidates"]] == [1]
        assert payload["pick_x_offset_mm"] == 15.0
        return {"ok": True, "cube_id": 1, "candidate_index": 0,
                "skipped": [], "reply": "done"}

    def next_key(_delay):
        key = next(keys)
        if key == ord(" "):
            events.append("space")
        return key

    monkeypatch.setattr(stage1, "run_motion", motion)
    monkeypatch.setattr(stage1, "TagDetector", lambda: None)
    def load_model(**_settings):
        events.append("load_model")
        return object()

    def open_camera(_camera):
        events.append("camera_open")
        return camera

    monkeypatch.setattr(stage1, "TrashDetector", load_model)
    monkeypatch.setattr(stage1, "open_camera", open_camera)
    monkeypatch.setattr(stage1, "OneSecondAverager", Averager)
    monkeypatch.setattr(stage1, "ThreadPoolExecutor", Executor)
    monkeypatch.setattr(stage1, "detect_frame", lambda *_args: [])
    def choose_ranked(_reports, _now, **kwargs):
        assert not kwargs.get("excluded_targets")
        return [(track, verdict)]

    monkeypatch.setattr(stage1, "rank_candidates", choose_ranked)
    monkeypatch.setattr(stage1, "draw_tracked_frame", lambda frame, _tracked: frame)
    monkeypatch.setattr(stage1.cv2, "imshow", lambda *_args: None)
    monkeypatch.setattr(stage1.cv2, "waitKey", next_key)
    monkeypatch.setattr(stage1.cv2, "destroyAllWindows", lambda: None)
    monkeypatch.setattr(stage1.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(sys, "argv", ["cube_sort_stage1.py"])

    assert stage1.main() == 0
    assert events.index("load_model") < events.index("camera_open")
    assert events.count("sort_cube_candidates") == 2
    assert "sort_cube_candidates" not in events[:events.index("space")]
    assert events.count("scan") > 2
    assert camera.grabs == 10


def _fake_obs(cube_id, point):
    verdict = CubeVerdict(cube_id, "blue", "color", score=0.8)
    group = SimpleNamespace(box=(point[0] - 10, point[1] - 10, 20, 20),
                            top=None)
    return SimpleNamespace(verdict=verdict, group=group,
                           top_center_px=point, geometry_valid=False)


def test_validate_observations_accepts_any_face_id_in_zone():
    zones = {"3": [[0, 0], [100, 0], [100, 100], [0, 100]]}
    obs = _fake_obs(3, (50, 50))  # geometry_valid False vẫn pass (any-face)
    ok, note = stage1.validate_observations([obs], 3, zones)
    assert ok and "trong zone" in note


def test_validate_observations_rejects_wrong_zone_and_wrong_id():
    zones = {"3": [[0, 0], [100, 0], [100, 100], [0, 100]],
             "1": [[200, 200], [300, 200], [300, 300], [200, 300]]}
    ok, _ = stage1.validate_observations([_fake_obs(3, (250, 250))], 3, zones)
    assert not ok  # đúng ID sai zone
    ok, _ = stage1.validate_observations([_fake_obs(1, (50, 50))], 3, zones)
    assert not ok  # sai ID đúng zone
    ok, _ = stage1.validate_observations([], 3, zones)
    assert not ok


def test_load_validate_zones_scales_legacy_screenshot_coords(tmp_path):
    path = tmp_path / "zones.json"
    path.write_text('{"2": [[646, 476], [535, 474], [500, 457], [603, 461]]}')
    zones = stage1.load_validate_zones(str(path))
    assert set(zones) == {"2"}
    for x, y in zones["2"]:
        assert 0 <= x < 640 and 0 <= y < 480
