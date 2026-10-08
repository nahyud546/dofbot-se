"""Readback must use the logical servo degrees consumed by the T8 KDL service."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cap_vision.real_joint_mirror import measured_joint_rad


@pytest.mark.parametrize("joint,readback", [
    ("arm1_Joint", 88), ("arm2_Joint", 92), ("arm3_Joint", 15),
    ("arm4_Joint", 11), ("arm5_Joint", 93),
])
def test_measured_readback_matches_t8_fk_logical_angles(joint, readback):
    assert measured_joint_rad(joint, readback) == pytest.approx(
        (readback - 90) * math.pi / 180)


def test_readback_gripper_preserves_existing_mimic_mapping():
    assert measured_joint_rad("Rlink1_Joint", 30) == pytest.approx(0)
    assert measured_joint_rad("Rlink1_Joint", 180) == pytest.approx(math.pi / 2)
