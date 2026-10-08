from pathlib import Path
import sys
from types import SimpleNamespace
from collections import deque

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cap_vision.object_perception_node import DISPLAY_CONFIRM_FRAMES, ObjectPerceptionNode
from cap_vision.object_perception_node import latest_image_qos
from rclpy.qos import HistoryPolicy, ReliabilityPolicy


class FakeDino:
    model = object()
    batch_sizes = []

    def match_many(self, crops, refine=True):
        self.batch_sizes.append(len(crops))
        return [("book", 0.9, {"margin": 0.2}) for _ in crops]


def test_camera_qos_is_constructed_without_copying_native_ros_objects():
    qos = latest_image_qos()
    assert qos.depth == 1
    assert qos.history == HistoryPolicy.KEEP_LAST
    # BEST_EFFORT làm rớt khung 900 KB (đo ~5 fps, đứt 1-4 s): perception và viewer phải dùng RELIABLE.
    assert qos.reliability == ReliabilityPolicy.RELIABLE


class FakeNode:
    dino = FakeDino()

    def get_parameter(self, name):
        values = {"trash_core_enabled": True, "trash_white_s_max": 70,
                  "trash_white_v_min": 140, "trash_face_expand_max": 1.7,
                  "dino_threshold": 0.35}
        return SimpleNamespace(value=values[name])


def test_scene_seed_uses_outer_face_not_printed_square():
    image = np.full((240, 320, 3), 90, np.uint8)
    cv2.rectangle(image, (100, 60), (190, 150), (240, 240, 240), -1)
    cv2.rectangle(image, (115, 75), (175, 135), (20, 20, 20), -1)
    seeds = ObjectPerceptionNode.scene_face_seed_instances(FakeNode(), image)
    assert FakeDino.batch_sizes[-1] == 1
    verified = [item for item in seeds if item.proposal_source == "trash_outer_face"]
    assert verified
    item = verified[0]
    assert item.mask[70, 105]  # white border belongs to the object
    assert item.mask[105, 145]  # printed core also belongs to it
    assert not item.mask[45, 80]  # table/background does not
    assert item.seed_cube_id == 1 and item.seed_label == "book"


def test_duplicate_metric_poses_keep_best_grasp_without_merging_detection_masks():
    def result(x, ready, confidence):
        pose = SimpleNamespace(camera_T_object=np.eye(4))
        pose.camera_T_object[0, 3] = x
        track = SimpleNamespace(pose=pose)
        item = SimpleNamespace(camera_pose_valid=True, top_grasp_ready=ready,
                               pose_confidence=confidence, identity_confidence=0.8)
        return track, item

    duplicate = result(0.206, False, 0.9)
    ready = result(0.200, True, 0.8)
    neighbour = result(0.230, True, 0.9)
    kept = ObjectPerceptionNode.dedupe_pose_results([duplicate, ready, neighbour])
    assert len(kept) == 2
    assert {id(track) for track, _ in kept} == {id(ready[0]), id(neighbour[0])}


def test_2d_only_observations_are_not_guessed_from_image_proximity():
    track = SimpleNamespace(pose=None)
    item = SimpleNamespace(camera_pose_valid=False, top_grasp_ready=False,
                           pose_confidence=0.0, identity_confidence=0.5)
    kept = ObjectPerceptionNode.dedupe_pose_results([(track, item), (track, item)])
    assert len(kept) == 2


def test_display_pose_waits_for_confirmation_and_rejects_single_bad_frame():
    state = SimpleNamespace(pose_display={}, _deque=deque,
                            tracker=SimpleNamespace(max_gap=3.0),
                            smoothing_preset="normal")
    good = np.eye(4)
    good[2, 3] = 0.20
    bad = good.copy()
    bad[0, 3] = 0.06
    display = ObjectPerceptionNode.display_camera_T
    assert display(state, "cube", good, 1.0) is None
    for frame in range(2, DISPLAY_CONFIRM_FRAMES):
        assert display(state, "cube", good, 1.0 + frame * 0.1) is None
    assert np.allclose(display(state, "cube", good, 1.5), good)
    assert np.allclose(display(state, "cube", bad, 1.6), good)
    assert np.allclose(display(state, "cube", good, 1.7), good)


def test_uncommitted_or_position_only_pose_does_not_draw_skeleton_or_tcp():
    item = SimpleNamespace(camera_pose_valid=True, object_id=0,
                           geometry_model_id=1, pose_method="rgb_single_face_cube")
    show = ObjectPerceptionNode.should_draw_cube_pose
    assert not show(item, {1: object()})
    item.object_id = 1
    assert show(item, {1: object()})
    item.pose_method = "rgb_geometry"
    assert not show(item, {1: object()})


def test_display_pose_reacquires_persistent_movement_without_drifting():
    state = SimpleNamespace(pose_display={}, _deque=deque,
                            tracker=SimpleNamespace(max_gap=3.0),
                            smoothing_preset="normal")
    first = np.eye(4)
    first[2, 3] = 0.20
    moved = first.copy()
    moved[0, 3] = 0.04
    display = ObjectPerceptionNode.display_camera_T
    for frame in range(DISPLAY_CONFIRM_FRAMES):
        display(state, "cube", first, 1.0 + frame * 0.1)
    for frame in range(DISPLAY_CONFIRM_FRAMES - 1):
        assert np.allclose(display(state, "cube", moved, 2.0 + frame * 0.1), first)
    assert np.allclose(display(state, "cube", moved, 2.5), moved)


def test_small_printed_patch_never_becomes_its_own_object():
    """Mảnh hình in nhỏ (nắp pin) không thành đối tượng 2D-only; mặt cỡ cube vẫn thành."""
    from cube_vision import faces

    class Node(FakeNode):
        face_tools = faces
        last_base_T_camera = None
        grown_faces = []
        grow_index = 0

        def get_parameter(self, name):
            if name in ("face_grow_enabled",):
                return SimpleNamespace(value=True)
            if name == "face_table_top_z":
                return SimpleNamespace(value=0.03)
            return super().get_parameter(name)

        K = np.eye(3)

    Node.GROWN_FACE_TTL_S = ObjectPerceptionNode.GROWN_FACE_TTL_S
    Node.GROWN_FACE_MIN_HITS = ObjectPerceptionNode.GROWN_FACE_MIN_HITS
    Node.GROW_MIN_INTERVAL_S = ObjectPerceptionNode.GROW_MIN_INTERVAL_S
    Node.grown_face_labels = ObjectPerceptionNode.grown_face_labels
    image = np.full((240, 320, 3), 90, np.uint8)
    cv2.rectangle(image, (200, 150), (214, 164), (20, 20, 20), -1)  # mảnh nhỏ
    seeds = ObjectPerceptionNode.scene_face_seed_instances(Node(), image)
    assert not seeds


def test_2d_only_part_inside_same_id_object_is_not_a_second_cube():
    def entry(box, pose_valid, object_id=3):
        track = SimpleNamespace(pose=SimpleNamespace(camera_T_object=np.eye(4)),
                                instance=SimpleNamespace(bbox=box))
        item = SimpleNamespace(camera_pose_valid=pose_valid, top_grasp_ready=pose_valid,
                               pose_confidence=0.9 if pose_valid else 0.0,
                               identity_confidence=0.8, object_id=object_id)
        return track, item

    cube = entry((100, 100, 260, 260), True)
    part = entry((150, 120, 190, 150), False)          # nắp pin nằm trong cube ID 3
    other = entry((300, 100, 360, 160), False, object_id=2)
    kept = ObjectPerceptionNode.dedupe_pose_results([cube, part, other])
    assert {id(t) for t, _ in kept} == {id(cube[0]), id(other[0])}


def test_second_object_with_same_id_far_away_is_a_ghost():
    def entry(box, pose_valid, conf, object_id=1):
        track = SimpleNamespace(pose=SimpleNamespace(camera_T_object=np.eye(4)),
                                instance=SimpleNamespace(bbox=box))
        track.pose.camera_T_object[0, 3] = box[0] / 1000.0
        item = SimpleNamespace(camera_pose_valid=pose_valid, top_grasp_ready=pose_valid and conf > 0.4,
                               pose_confidence=conf, identity_confidence=0.8, object_id=object_id)
        return track, item

    real = entry((455, 285, 613, 429), True, 0.5)
    ghost = entry((0, 131, 62, 234), True, 0.3)        # vệt màu xanh trên thảm, có pose yếu
    kept = ObjectPerceptionNode.dedupe_pose_results([ghost, real])
    assert [id(t) for t, _ in kept] == [id(real[0])]


def test_faces_of_same_cube_merge_but_other_ids_do_not():
    from cap_vision.object_pipeline import ObjectInstance

    def seed(box, cid, label="", conf=0.6):
        mask = np.zeros((300, 400), bool)
        mask[box[1]:box[3], box[0]:box[2]] = True
        return ObjectInstance(box, mask, conf, seed_cube_id=cid, seed_label=label)

    top = seed((100, 50, 200, 140), 3, "used_batteries")
    side = seed((100, 160, 200, 230), 3)               # mặt màu bên hông, cách 20 px
    other = seed((205, 60, 300, 140), 2, "fish_bone")   # cube khác ngay cạnh
    far = seed((10, 250, 40, 290), 3)                   # vệt màu cùng ID nhưng xa
    out = ObjectPerceptionNode.merge_same_cube_seeds([top, side, other, far])
    assert len(out) == 3
    merged = next(o for o in out if o.seed_cube_id == 3 and o.seed_label)
    assert merged.bbox == (100, 50, 200, 230) and merged.seed_label == "used_batteries"
