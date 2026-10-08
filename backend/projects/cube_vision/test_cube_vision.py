"""Test đặc trưng: module mới phải cho đúng kết quả của các hàm cũ (golden_legacy.json)."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cube_vision import CUBES, COLOR_TO_ID, ID_TO_COLOR, hsv_ranges, registry  # noqa: E402
from cube_vision import color  # noqa: E402
from cube_vision.identify import Identifier, TAG, TAG_COLOR  # noqa: E402
from cube_vision.synthetic import FRAMES  # noqa: E402
from cube_vision.tag import TagDetector  # noqa: E402

GOLDEN = json.loads((Path(__file__).with_name("golden_legacy.json")).read_text())


def norm(o):
    if isinstance(o, np.ndarray):
        return np.round(o.astype(float), 3).tolist()
    if isinstance(o, dict):
        return {k: norm(v) for k, v in sorted(o.items())}
    if isinstance(o, (list, tuple)):
        return [norm(v) for v in o]
    if isinstance(o, (float, np.floating)):
        return round(float(o), 3)
    if isinstance(o, (int, np.integer)):
        return int(o)
    return o


def test_registry_is_consistent_with_the_runtime_yaml():
    runtime = hsv_ranges("runtime")
    assert set(runtime) == {spec["color"] for spec in CUBES.values()}
    import yaml
    data = yaml.safe_load(registry.runtime_hsv_path().read_text())["colors"]
    for name, spec in data.items():          # yaml ids must match the shared table
        assert COLOR_TO_ID[next(s["color"] for s in CUBES.values() if s["name"] == name)] == spec["id"]
    assert ID_TO_COLOR[1] == "khoi_xanh_duong" and ID_TO_COLOR[4] == "khoi_vang"
    with pytest.raises(ValueError):
        hsv_ranges("nope")


@pytest.mark.parametrize("name", sorted(FRAMES))
def test_tag_detector_matches_both_legacy_detectors(name):
    img = FRAMES[name]()
    new = TagDetector(ids=CUBES, quiet=True).detect(img)
    legacy = GOLDEN[name]["sc_tag"]
    assert sorted((t["cube_id"], [round(v, 3) for v in t["centroid"]]) for t in new) == \
        sorted((t["cube_id"], t["center"]) for t in legacy)             # search-center uses the corner mean
    enhanced = TagDetector(enhance=True, quiet=True).detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    assert sorted((t["id"], [round(v, 3) for v in t["center"]], round(t["margin"], 3)) for t in enhanced) == \
        sorted((t["id"], t["center"], t["margin"]) for t in GOLDEN[name]["ic_tag"])


@pytest.mark.parametrize("name", sorted(FRAMES))
def test_square_candidates_match_the_legacy_hsv_function_except_the_fixed_id_map(name):
    img = FRAMES[name]()
    legacy_map = {"khoi_do": 3, "khoi_xanh": 2, "khoi_xanh_duong": 4, "khoi_vang": 1}   # the old, wrong table
    got = color.square_candidates(img, hsv_ranges("proven"), legacy_map)
    assert norm(got) == GOLDEN[name]["hsv_all"]


def test_color_ids_follow_the_shared_table():
    """Blue/yellow squares are 1/4 in T8's identity table (search-center had them swapped)."""
    img = FRAMES["blue_yellow"]()
    ids = {c["cube_id"] for c in color.square_candidates(img, hsv_ranges("proven"), COLOR_TO_ID)}
    assert ids == {1, 4}
    blue_only = color.square_candidates(img, hsv_ranges("proven"), COLOR_TO_ID, want_id=1)
    assert len(blue_only) == 1 and blue_only[0]["center"][0] < 320       # the blue square on the left


def test_identifier_profiles():
    tag_only = Identifier(TAG)
    assert [d.cube_id for d in tag_only.detect(FRAMES["tag1_tag4"]())] == [1, 4]
    assert tag_only.detect(FRAMES["red_square"]()) == []                  # TAG never guesses by color
    both = Identifier(TAG_COLOR)
    assert both.detect(FRAMES["tag1_tag4"]()) == []                       # two tags: ambiguous
    assert [(d.cube_id, d.source) for d in both.detect(FRAMES["tag3"](), 3)] == [(3, "apriltag")]
    assert [(d.cube_id, d.source) for d in both.detect(FRAMES["red_square"](), 3)] == [(3, "hsv_single")]
    assert both.detect(FRAMES["blue_yellow"](), 4)[0].cube_id == 4          # yellow is ID 4
    assert both.detect(FRAMES["empty"]()) == []
    with pytest.raises(ValueError):
        Identifier("BOGUS")


@pytest.mark.parametrize("name", sorted(FRAMES))
def test_search_center_still_matches_golden_after_migration(name):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))
    import cube_search_center as SC
    img = FRAMES[name]()
    assert norm(SC.TagDetector().detect(img)) == GOLDEN[name]["sc_tag"]
    old_to_new = {3: 3, 2: 2, 4: 1, 1: 4}          # the old table had blue/yellow swapped
    expected = [dict(c, cube_id=old_to_new[c["cube_id"]]) for c in GOLDEN[name]["hsv_all"]]
    assert norm(SC.hsv_candidates(img)) == expected
    assert SC.COLOR_TO_ID == COLOR_TO_ID and SC.ID_TO_COLOR == ID_TO_COLOR


def test_identify_cube_and_t8_vision_share_the_registry():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vision_experiments"))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "t8_pipeline"))
    import identify_cube as IC
    import t8_vision as TV
    assert IC.CUBES is CUBES and IC.HSV_RANGES == hsv_ranges("identify")
    assert TV.HSV_RANGES == hsv_ranges("proven")
    from cube_vision.trash import TrashDetector
    assert IC.TrashDetector is TrashDetector
