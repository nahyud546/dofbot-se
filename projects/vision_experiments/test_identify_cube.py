"""Regression tests for cube-wide identity and physical upper-face geometry."""

import json
from copy import deepcopy

import cv2
import numpy as np
import pytest

from identify_cube import (COLOR_TO_CUBE, CUBES, CubeObservation, CubeVerdict,
                           FaceCues,
                           OneSecondAverager, TAG_TO_CUBE, TRASH_TO_CUBE,
                           TagDetector, TopFace, TrashDetector, classify_top_face,
                           color_on_face, draw_similarity_panel, find_top_faces, fuse_cues,
                           identify_frame, rectify_face, repo_root,
                           save_contour_debug, synthetic_frame, tag_fills_candidate)


@pytest.mark.parametrize("cube_id,spec", CUBES.items())
def test_all_six_faces_map_to_canonical_cube(cube_id, spec):
    assert fuse_cues(FaceCues(tag_id=spec["tag_id"])).cube_id == cube_id
    assert fuse_cues(FaceCues(color_label=spec["color"])).cube_id == cube_id
    assert len(spec["trash"]) == 4
    for label in spec["trash"]:
        assert fuse_cues(FaceCues(trash_class=label)).cube_id == cube_id


def test_registry_has_four_cubes_and_six_faces_each():
    assert len(CUBES) == len(TAG_TO_CUBE) == len(COLOR_TO_CUBE) == 4
    assert len(TRASH_TO_CUBE) == 16


def test_conflict_uses_cue_priority_not_largest_numeric_id():
    verdict = fuse_cues(FaceCues(tag_id=1, trash_class="Syringe",
                                 color_label="khoi_vang"))
    assert verdict.cube_id == 1
    assert verdict.via == "tag"
    assert verdict.conflict
    assert fuse_cues(FaceCues(trash_class="Newspaper",
                              color_label="khoi_vang")).cube_id == 1


@pytest.mark.parametrize("kind,expected", [("blue", 1), ("green", 2),
                                           ("red", 3), ("yellow", 4)])
def test_synthetic_cube_groups_three_faces_and_uses_outer_top(kind, expected):
    frame = synthetic_frame(kind)
    faces = find_top_faces(frame)
    assert len(faces) == 1
    observations, _ = identify_frame(frame, tags=[], trash_detector=None)
    assert len(observations) == 1
    obs = observations[0]
    assert obs.verdict.cube_id == expected
    assert obs.verdict.via == "color"
    assert len(obs.group.faces) == 3
    assert obs.group.top.area > 10000
    assert obs.top_center_px == pytest.approx((319, 185), abs=2)
    assert obs.to_dict()["robot_target_valid"] is False


def test_fast_known_skips_dino_when_color_already_identifies_cube():
    class Matcher:
        def match_many(self, _crops):
            raise AssertionError("DINO should not run for a known color cube")

    observations, _ = identify_frame(synthetic_frame("blue"), [], Matcher(),
                                     annotate=False, fast_known=True)
    assert observations[0].verdict.cube_id == 1


def test_fast_known_still_uses_dino_for_sticker_only_cube():
    class Matcher:
        calls = 0

        def match_many(self, crops):
            self.calls += 1
            return [("newspaper", 0.9, {"ranked": [("newspaper", 0.9)]})
                    for _ in crops]

    frame = synthetic_frame("blue")
    top = np.int32([[250, 130], [390, 130], [420, 260], [220, 260]])
    cv2.fillConvexPoly(frame, top, (220, 220, 220))
    cv2.polylines(frame, [top], True, (0, 0, 0), 3)
    matcher = Matcher()
    observations, _ = identify_frame(frame, [], matcher, annotate=False,
                                     fast_known=True)
    assert matcher.calls == 1
    assert observations[0].verdict.cube_id == 1


def test_known_color_still_exposes_background_dino_top_two():
    class Matcher:
        def match_many(self, crops):
            return [("newspaper", 0.45, {"ranked": [
                ("newspaper", 0.45), ("book", 0.44)], "reason": ""})
                    for _ in crops]

    observations, _ = identify_frame(synthetic_frame("blue"), [], Matcher(),
                                     annotate=False)
    assert observations[0].verdict.cube_id == 1
    assert observations[0].verdict.trash_top == [
        ("newspaper", 0.45), ("book", 0.44)]
    assert observations[0].to_dict()["trash_top"][0]["similarity_percent"] == 45.0
    assert "trash_rotation_deg" in observations[0].to_dict()


def test_four_visible_cubes_stay_separate_with_distinct_top_centers():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    for kind, (x, y) in zip(("blue", "green", "red", "yellow"),
                            ((60, 40), (360, 40), (60, 270), (360, 270))):
        cube = synthetic_frame(kind)[125:345, 215:425]
        cube = cv2.resize(cube, (190, 190))
        roi = frame[y:y + 190, x:x + 190]
        mask = np.any(cube != 190, axis=2)
        roi[mask] = cube[mask]
    observations, _ = identify_frame(frame, tags=[], trash_detector=None)
    assert [obs.verdict.cube_id for obs in observations] == [1, 2, 3, 4]
    assert all(obs.geometry_valid for obs in observations)
    assert len({tuple(map(round, obs.top_center_px))
                for obs in observations}) == 4


@pytest.mark.parametrize("x,y", [(6, 6), (524, 6), (6, 364),
                                  (524, 364), (265, 185)])
def test_small_cube_detected_at_five_frame_positions(x, y):
    crop = cv2.resize(synthetic_frame("blue")[125:345, 215:425], (110, 110))
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    roi = frame[y:y + 110, x:x + 110]
    mask = np.any(crop != 190, axis=2)
    roi[mask] = crop[mask]
    observations, _ = identify_frame(frame, [], annotate=False)
    assert len(observations) == 1
    assert observations[0].verdict.cube_id == 1
    assert observations[0].geometry_valid


def test_contour_report_saves_frame_and_area_to_box_metrics(tmp_path):
    frame = synthetic_frame("blue")
    raw, report = save_contour_debug(frame, tmp_path / "center")
    assert np.array_equal(cv2.imread(str(raw)), frame)
    data = json.loads(report.read_text())
    assert data["image_size"] == [640, 480]
    assert data["groups"][0]["geometry_valid"]
    assert any(face["is_top"] and face["area_box_ratio"] > 0.5
               for face in data["groups"][0]["faces"])
    assert data["recognized_groups"][0]["top_min_edge_px"] > 20
    assert data["contours"]


def test_similarity_panel_stays_inside_frame_near_bottom_right():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    draw_similarity_panel(frame, (580, 430, 55, 45),
                          [("disposable_chopsticks", 0.57),
                           ("cigarette_butts", 0.51)])
    assert np.any(frame[350:430, 350:640] != 190)
    assert np.all(frame[:300, :300] == 190)


def test_conflicting_side_tag_has_priority_without_losing_top_geometry():
    frame = synthetic_frame("blue")
    side_tag = {"id": 3, "margin": 80.0,
                "corners": np.float32([[335, 275], [375, 275],
                                       [375, 315], [335, 315]])}
    observations, _ = identify_frame(frame, [side_tag])
    assert len(observations) == 1
    assert observations[0].verdict.cube_id == 3
    assert "ID1" in observations[0].verdict.conflict
    assert observations[0].geometry_valid


def test_side_tag_can_identify_cube_when_top_has_no_color():
    frame = synthetic_frame("blue")
    top = np.int32([[250, 130], [390, 130], [420, 260], [220, 260]])
    cv2.fillConvexPoly(frame, top, (220, 220, 220))
    cv2.polylines(frame, [top], True, (0, 0, 0), 3)
    side_tag = {"id": 1, "margin": 80.0,
                "corners": np.float32([[335, 275], [375, 275],
                                       [375, 315], [335, 315]])}
    observations, _ = identify_frame(frame, [side_tag])
    assert len(observations) == 1
    assert observations[0].verdict.cube_id == 1
    assert observations[0].geometry_valid


def test_tag_contour_is_not_a_whole_cube_face():
    corners = np.float32([[100, 100], [200, 100], [200, 200], [100, 200]])
    face = TopFace(corners, (100, 100, 101, 101), 10000, 0.1)
    tag = {"id": 3, "margin": 40.0,
           "corners": np.float32([[110, 110], [190, 110],
                                  [190, 190], [110, 190]])}
    assert tag_fills_candidate(face, [tag])


def test_one_second_average_votes_and_mean_score():
    frame = synthetic_frame("blue")
    face = find_top_faces(frame)[0]
    averager = OneSecondAverager()
    samples = [CubeVerdict(1, "blue", "trash", label="newspaper", score=0.6),
               CubeVerdict(2, "green", "trash", label="fish_bone", score=0.9),
               CubeVerdict(1, "blue", "trash", label="newspaper", score=0.8)]
    group = identify_frame(frame, tags=[], trash_detector=None)[0][0].group
    for offset, sample in zip((0.0, 0.4, 1.01), samples):
        shown, reports = averager.update([CubeObservation(group, sample, [])], offset)
    assert len(reports) == 1
    assert shown[0][1].cube_id == 1
    assert reports[0][1].score == pytest.approx(0.8)
    assert shown[0][1].score == pytest.approx(0.8)
    assert averager.update([CubeObservation(group, samples[0], [])], 1.2)[1] == []


def test_track_keeps_best_score_through_conflicting_pick_window():
    frame = synthetic_frame("blue")
    group = identify_frame(frame, [], trash_detector=None, annotate=False)[0][0].group
    averager = OneSecondAverager()
    high = CubeVerdict(1, "blue", "trash", score=0.9)
    low = CubeVerdict(1, "blue", "trash", score=0.5)
    wrong = CubeVerdict(2, "green", "trash", score=0.95)
    averager.update([CubeObservation(group, high, [])], 0)
    averager.update([CubeObservation(group, low, [])], 0.4)
    shown, reports = averager.update([CubeObservation(group, low, [])], 1.1)
    assert reports[0][0].confirmed
    assert shown[0][1].score == pytest.approx(0.9)
    assert shown[0][0].best_observation.verdict.score == pytest.approx(0.9)
    averager.update([CubeObservation(group, wrong, [])], 1.5)
    _, reports = averager.update([CubeObservation(group, low, [])], 2.2)
    assert reports[0][0].confirmed
    assert reports[0][1].cube_id == 1
    assert reports[0][1].score == pytest.approx(0.9)


def test_track_keeps_identity_but_updates_live_geometry_after_motion():
    first = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    average = OneSecondAverager()
    for t in (0.0, 0.5, 1.1):
        shown, _ = average.update([first], t)
    track = shown[0][0]
    held = track.best_observation
    moved = deepcopy(first)
    x, y, w, h = moved.group.box
    moved.group.box = (x + 30, y, w, h)
    for face in moved.group.faces:
        face.corners += [30, 0]
    shown, _ = average.update([moved], 1.3)
    assert shown[0][0] is track
    assert track.best_observation is held
    assert track.observation.top_center_px[0] == pytest.approx(
        first.top_center_px[0] + 30)
    assert shown[0][1].cube_id == 1


def test_ambiguous_track_association_cannot_publish_pick_geometry():
    base = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    def at(dx):
        obs = deepcopy(base)
        x, y, w, h = obs.group.box
        obs.group.box = (x + dx, y, w, h)
        for face in obs.group.faces:
            face.corners += [dx, 0]
        return obs

    average = OneSecondAverager()
    for t in (0.0, 0.5, 1.1):
        average.update([at(-60), at(60)], t)
    shown, _ = average.update([at(0)], 1.3)
    assert shown[0][0].association_ambiguous


def test_track_switches_only_after_higher_stable_same_source_score():
    group = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0].group
    average = OneSecondAverager()
    def obs(cube_id, score):
        return CubeObservation(group, CubeVerdict(
            cube_id, CUBES[cube_id]["name"], "trash", score=score), [])
    for t, sample in [(0, obs(1, 0.7)), (0.5, obs(1, 0.7)),
                      (1.1, obs(1, 0.7))]:
        shown, _ = average.update([sample], t)
    held = shown[0][0].best_observation
    for t, sample in [(1.4, obs(2, 0.6)), (2.2, obs(2, 0.6))]:
        shown, _ = average.update([sample], t)
    assert shown[0][1].cube_id == 1
    assert shown[0][0].best_observation is held
    for t in (2.5, 3.3):
        shown, _ = average.update([obs(2, 0.8)], t)
    assert shown[0][1].cube_id == 2
    assert shown[0][0].best_observation.verdict.score == 0.8


def test_geometry_can_be_filled_without_downgrading_tag_identity():
    complete = identify_frame(synthetic_frame("blue"), [], annotate=False)[0][0]
    missing = CubeObservation(type(complete.group)(complete.group.faces, None,
                                                     complete.group.box),
                              CubeVerdict(1, "blue", "tag", score=70), [])
    visible = CubeObservation(complete.group,
                              CubeVerdict(1, "blue", "color", score=0.8), [])
    average = OneSecondAverager()
    for t in (0, 0.5, 1.1):
        average.update([missing], t)
    for t in (1.4, 2.2):
        shown, _ = average.update([visible], t)
    track, verdict = shown[0]
    assert verdict.via == "tag" and verdict.score == 70
    assert track.best_observation.geometry_valid
    assert track.best_observation.verdict.via == "tag"


def test_tag_and_color_can_identify_without_graspable_top():
    blank = np.full((480, 640, 3), 190, dtype=np.uint8)
    tag = {"id": 2, "margin": 60.0, "center": (320, 240),
           "corners": np.float32([[290, 210], [350, 210],
                                  [350, 270], [290, 270]])}
    detected, _ = identify_frame(blank, [tag], annotate=False)
    assert len(detected) == 1
    assert detected[0].verdict.cube_id == 2
    assert not detected[0].geometry_valid
    cv2.rectangle(blank, (200, 150), (300, 250), (255, 0, 0), -1)
    detected, _ = identify_frame(blank, [], annotate=False)
    assert any(obs.verdict.cube_id == 1 and not obs.geometry_valid
               for obs in detected)


def test_color_top_requires_visible_side_and_recovers_weak_contour():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    top = np.int32([[160, 170], [300, 170], [300, 300], [160, 300]])
    cv2.fillConvexPoly(frame, top, (0, 0, 230))
    flat, _ = identify_frame(frame, [], annotate=False)
    assert any(obs.verdict.cube_id == 3 and not obs.geometry_valid
               for obs in flat)
    side = np.int32([[160, 300], [300, 300], [310, 370], [170, 370]])
    cv2.fillConvexPoly(frame, side, (120, 120, 120))
    cv2.polylines(frame, [side], True, (40, 40, 40), 3)
    cv2.fillConvexPoly(frame, top, (0, 0, 230))
    cuboid, _ = identify_frame(frame, [], annotate=False)
    assert any(obs.verdict.cube_id == 3 and obs.top_center_px is not None and
               obs.top_center_px == pytest.approx((230.0, 235.0), abs=2)
               for obs in cuboid)


def test_small_cube_has_id_and_bbox_without_unsafe_pick():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    cube = synthetic_frame("blue")[120:350, 210:430]
    cube = cv2.resize(cube, None, fx=0.12, fy=0.12)
    h, w = cube.shape[:2]
    mask = np.any(cube != 190, axis=2)
    frame[200:200 + h, 250:250 + w][mask] = cube[mask]
    observations, annotated = identify_frame(frame, [], annotate=True)
    identified = [obs for obs in observations if obs.verdict.cube_id == 1]
    assert identified
    assert all(obs.group.box[2] > 0 and not obs.geometry_valid
               for obs in identified)
    assert any(obs.group.geometry_reason == "too-small" for obs in identified)
    assert not np.array_equal(frame, annotated)


def test_large_single_face_at_frame_edge_is_labeled_but_not_picked():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    cv2.rectangle(frame, (0, 100), (360, 400), (255, 0, 0), -1)
    observations, _ = identify_frame(frame, [], annotate=False)
    identified = [obs for obs in observations if obs.verdict.cube_id == 1]
    assert identified
    assert all(not obs.geometry_valid for obs in identified)
    assert any(obs.group.geometry_reason == "frame-clipped" for obs in identified)


def test_printed_inner_rectangle_does_not_replace_outer_top():
    frame = synthetic_frame("blue")
    cv2.rectangle(frame, (280, 165), (355, 235), (10, 10, 10), -1)
    cv2.rectangle(frame, (290, 175), (345, 225), (240, 240, 240), 3)
    observations, _ = identify_frame(frame, [], annotate=False)
    assert len(observations) == 1
    assert observations[0].verdict.cube_id == 1
    assert observations[0].top_center_px == pytest.approx((319, 185), abs=2)


def test_table_line_below_single_color_face_is_not_cube_side():
    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    cv2.rectangle(frame, (160, 140), (300, 270), (0, 0, 230), -1)
    cv2.line(frame, (140, 320), (320, 320), (20, 20, 20), 3)
    observations, _ = identify_frame(frame, [], annotate=False)
    assert any(obs.verdict.cube_id == 3 and not obs.geometry_valid
               for obs in observations)


def test_small_single_trash_face_can_identify_without_pick_pose():
    class Matcher:
        def match_many(self, crops):
            return [("book", 0.55, {"ranked": [("book", 0.55),
                                              ("newspaper", 0.43)]})
                    for _ in crops]

    frame = np.full((480, 640, 3), 190, dtype=np.uint8)
    cv2.rectangle(frame, (240, 200), (280, 240), (110, 110, 110), -1)
    cv2.rectangle(frame, (240, 200), (280, 240), (20, 20, 20), 2)
    observations, _ = identify_frame(frame, [], Matcher(), annotate=False)
    identified = [obs for obs in observations if obs.verdict.cube_id == 1]
    assert identified
    assert all(not obs.geometry_valid for obs in identified)
    assert any(obs.verdict.trash_top[0] == ("book", 0.55)
               for obs in identified)


def test_tag_detector_merges_original_and_contrast_detections():
    class Tag:
        tag_id = 2
        center = np.array([300.0, 200.0])
        corners = np.float32([[280, 180], [320, 180],
                              [320, 220], [280, 220]])
        decision_margin = 60.0

    class Backend:
        calls = 0

        def detect(self, image):
            self.calls += 1
            assert image.shape == (480, 640)
            return [Tag()]

    detector = TagDetector.__new__(TagDetector)
    detector.detector = Backend()
    tags = detector.detect(np.full((480, 640), 150, dtype=np.uint8))
    assert detector.detector.calls == 2
    assert len(tags) == 1 and tags[0]["id"] == 2


def test_dino_reports_top_two_even_when_gap_is_small():
    matcher = TrashDetector.__new__(TrashDetector)
    matcher.labels = ["newspaper"] * 3 + ["book"] * 3
    matcher.thresh, matcher.margin = 0.4, 0.0
    scores = np.array([[0.42], [0.42], [0.42],
                       [0.41], [0.41], [0.41]])
    label, score, result = matcher._rank_scores(scores)
    assert label == "newspaper" and score == pytest.approx(0.42)
    assert [name for name, _ in result["ranked"]] == ["newspaper", "book"]
    assert [value for _, value in result["ranked"]] == pytest.approx([0.42, 0.41])


def test_tag_wholly_on_upper_face_takes_its_id():
    frame = synthetic_frame("blue")
    face = find_top_faces(frame)[0]
    top_tag = {"id": 2, "margin": 80.0,
               "corners": np.float32([[290, 180], [340, 180],
                                      [340, 225], [290, 225]])}
    crop = rectify_face(frame, face.corners)
    cues, verdict = classify_top_face(crop, face.corners, [top_tag])
    assert cues.tag_id == 2
    assert verdict.cube_id == 2
    assert verdict.via == "tag"


def test_colored_print_is_not_a_color_face():
    face = np.full((224, 224, 3), 245, dtype=np.uint8)
    cv2.rectangle(face, (70, 70), (155, 155), (0, 0, 255), -1)
    assert color_on_face(face)[0] is None
    cues, verdict = classify_top_face(face, np.float32([[0, 0], [223, 0],
                                                        [223, 223], [0, 223]]),
                                      tags=[], trash_detector=None)
    assert cues == FaceCues()
    assert verdict.cube_id is None


def test_flat_square_on_table_is_not_a_cube():
    frame = np.full((480, 640, 3), 210, dtype=np.uint8)
    cv2.rectangle(frame, (230, 170), (390, 310), (0, 0, 0), 3)
    cv2.line(frame, (310, 170), (310, 310), (0, 0, 0), 3)
    cv2.line(frame, (230, 240), (390, 240), (0, 0, 0), 3)
    assert find_top_faces(frame) == []


@pytest.fixture(scope="module")
def trash_matcher():
    matcher = TrashDetector(device="cpu")
    if matcher.model is None:
        pytest.skip("DINO model / vector DB unavailable")
    return matcher


def test_dino_db_shape_matches_vits14(trash_matcher):
    assert len(set(trash_matcher.labels)) == 16
    assert len(trash_matcher.labels) == 256
    assert tuple(trash_matcher.matrix.shape) == (256, 384)


@pytest.mark.parametrize("name,expected", [("Fish_bone", 2),
                                           ("Used_batteries", 3),
                                           ("Toilet_paper", 4),
                                           ("Newspaper", 1)])
def test_dino_reference_images_smoke_test(trash_matcher, name, expected):
    # These are database source images, so this is only a model/DB wiring test.
    path = repo_root() / "ai/datasets/trash-images/processed" / name / \
        f"{name}_base.jpg"
    image = cv2.imread(str(path))
    assert image is not None
    label, score = trash_matcher.match(image)
    assert label == name.lower()
    assert score >= TrashDetector.DEFAULT_THRESH
    assert fuse_cues(FaceCues(trash_class=label)).cube_id == expected


def test_dino_blank_face_stays_unknown(trash_matcher):
    blank = np.full((224, 224, 3), 60, dtype=np.uint8)
    label, _ = trash_matcher.match(blank)
    assert label is None
    assert trash_matcher.last_result["rotations_tested"] == 8


def test_dino_rotated_face_keeps_class(trash_matcher):
    path = repo_root() / "ai/datasets/trash-images/processed/Egg_shell/Egg_shell_base.jpg"
    image = cv2.imread(str(path))
    assert image is not None
    label, _ = trash_matcher.match(np.ascontiguousarray(np.rot90(image)))
    assert label == "egg_shell"
    angled = cv2.warpAffine(
        image, cv2.getRotationMatrix2D((image.shape[1] / 2,
                                        image.shape[0] / 2), 45, 1),
        (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT_101)
    label, _ = trash_matcher.match(angled)
    assert label == "egg_shell"
    assert trash_matcher.last_result["rotation_deg"] in range(0, 360, 45)
