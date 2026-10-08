import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dofbot_ik  # noqa: E402


@pytest.fixture(autouse=True)
def identity_j1():
    """Hermetic: ignore any hand_eye.json J1 correction on disk."""
    dofbot_ik.set_j1_correction()
    yield
    dofbot_ik.set_j1_correction()


def test_fk_matches_urdf_service_reference():
    # Values returned by the KDL service (dofbot.urdf) for the same joints.
    assert dofbot_ik.fk([90, 125, 0, 0, 90]) == pytest.approx((-0.1003, 0.0007, 0.1183), abs=1e-4)
    assert dofbot_ik.fk([90, 35, 65, 15, 90]) == pytest.approx((-0.2069, 0.0007, 0.0528), abs=1e-4)


def test_ik_closes_over_table_and_respects_limits():
    for xi in range(-26, -8):
        for yi in range(-10, 11, 2):
            for z in (0.047, 0.058, 0.067, 0.077):
                x, y = xi / 100, yi / 100
                joints = dofbot_ik.ik(x, y, z)
                assert math.dist(dofbot_ik.fk(joints), (x, y, z)) < 1e-6
                assert all(0 <= v <= 180 for v in joints[:4])
                assert joints[1] <= 100 and joints[3] <= 120


def test_lift_column_is_continuous():
    previous = None
    for step in range(13):
        joints = dofbot_ik.ik(-0.20, 0.04, 0.047 + 0.005 * step)
        if previous:
            assert max(abs(a - b) for a, b in zip(joints, previous)) < 8
        previous = joints


def test_unreachable_targets_raise():
    for target in ((-0.40, 0.0, 0.047), (0.15, 0.0, 0.047), (0.0, 0.0, 0.047)):
        with pytest.raises(dofbot_ik.NoSolution):
            dofbot_ik.ik(*target)


def test_j1_correction_roundtrips_and_matches_the_camera_side_fk():
    import math
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))
    import cube_search_center_math as M
    import dofbot_ik as D
    D.set_j1_correction(1.12, -0.8)
    M.apply_calibration({"arm4_T_optical": np.eye(4), "tag_top_z": 0.058, "K": (900, 900, 320, 240),
                         "j1_scale": 1.12, "j1_offset_deg": -0.8})
    try:
        for x, y, z in ((-0.17, 0.05, 0.047), (-0.15, -0.12, 0.077), (-0.22, 0.10, 0.047)):
            joints = D.ik(x, y, z)
            assert D.fk(joints) == pytest.approx([x, y, z], abs=1e-6)       # command/observe are inverse
            # camera-side FK and IK-side FK agree on where the arm points
            arm4 = M.fk_arm4(joints)
            assert math.atan2(arm4[1, 3], arm4[0, 3]) == pytest.approx(math.atan2(y, x), abs=0.02)
        # a reading of 90+d turns the arm by 1.12*d
        assert D.fk([90 + 20, 125, 0, 0, 90])[1] != D.fk([90 + 20 / 1.12, 125, 0, 0, 90])[1]
    finally:
        D.set_j1_correction()
        M.apply_calibration(None)
    with pytest.raises(ValueError):
        D.set_j1_correction(1.5)
