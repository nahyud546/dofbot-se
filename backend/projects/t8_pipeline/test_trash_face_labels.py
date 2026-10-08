import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cube_identity as CI
from t8_pipeline import cube_label, try_local_hold_place, try_local_stack_sequence, try_local_motion_sequence
from t8_scene import normalize_cube_label


class TrashFaceLabels(unittest.TestCase):
    def setUp(self):
        CI.set_ros3d_face_labels(True)

    def tearDown(self):
        CI.set_ros3d_face_labels(False)

    def test_mapping_is_off_outside_the_ros3d_flow(self):
        CI.set_ros3d_face_labels(False)
        self.assertIsNone(normalize_cube_label("cube hình cục pin đã qua sử dụng"))
        self.assertNotEqual(CI.canonical_label("cube pin đã qua sử dụng"), "cube_3")


    def test_table_matches_the_shared_cube_registry(self):
        from cube_vision.registry import TRASH_TO_CUBE
        self.assertEqual(CI.TRASH_FACE_CUBE_ID, TRASH_TO_CUBE)
        self.assertEqual(set(CI.TRASH_FACE_ALIASES), set(TRASH_TO_CUBE))

    def test_every_class_resolves_from_vietnamese_and_english_phrases(self):
        cases = {
            "cube hình cục pin đã qua sử dụng": 3, "pin cũ": 3, "used batteries": 3,
            "cube có hình xương cá": 2, "vỏ trứng": 2, "lõi táo": 2, "vỏ dưa hấu": 2,
            "tờ báo": 1, "lon nước": 1, "quyển sách": 1, "cặp sách cũ": 1,
            "kim tiêm": 3, "mỹ phẩm hết hạn": 3, "thuốc hết hạn": 3,
            "giấy vệ sinh": 4, "hạt đào": 4, "tàn thuốc lá": 4, "đũa dùng một lần": 4,
        }
        for phrase, expected in cases.items():
            self.assertEqual(CI.trash_face_cube_id(phrase), expected, phrase)

    def test_longest_phrase_wins_and_ambiguity_is_not_guessed(self):
        self.assertEqual(CI.trash_face_class("cặp sách cũ"), "old_school_bag")      # not "book"
        self.assertEqual(CI.trash_face_class("vỏ dưa hấu"), "watermelon_rind")
        self.assertIsNone(CI.trash_face_class("xương cá và pin cũ"))                # two classes
        self.assertIsNone(CI.trash_face_class("cube lớn màu đỏ"))                  # 'lon' (lớn) is not a can
        self.assertIsNone(CI.trash_face_class("đũa"))                              # đũa/dưa ambiguous

    def test_labels_flow_into_the_executor_resolvers(self):
        self.assertEqual(normalize_cube_label("cube hình cục pin đã qua sử dụng"), "cube_3")
        self.assertEqual(cube_label("cube hinh cuc pin da qua su dung"), "cube_3")
        self.assertEqual(CI.canonical_label("cube pin đã qua sử dụng"), "cube_3")
        # legacy 2D labels for non-cube objects keep their meaning (no cube noun -> no ID)
        self.assertNotEqual(CI.canonical_label("xuong_ca"), "cube_2")
        self.assertIsNone(normalize_cube_label("xương cá"))
        self.assertEqual(cube_label("gắp xương cá lên"), "xuong_ca")
        self.assertEqual(normalize_cube_label("cube id 2"), "cube_2")                # explicit IDs still win
        self.assertIsNone(normalize_cube_label("cube xanh dương"))                  # colours are handled elsewhere

    def test_the_failing_sentence_is_a_stack_not_an_arm_pose(self):
        sentence = "gắp cube hình cục pin đã qua sử dụng lên trên cube xanh dương"
        self.assertIsNone(try_local_hold_place(sentence))                            # was arm_pose(up)
        self.assertEqual(try_local_stack_sequence(sentence),
                         {"source": "cube_3", "target": "khoi_xanh_duong"})
        self.assertNotIn("arm_pose", [s[0] for s in (try_local_motion_sequence(sentence) or [])])

    def test_real_arm_pose_commands_still_work(self):
        for phrase, pose in (("tay lên", "up"), ("nâng tay lên", "up"), ("đứng thẳng", "up"),
                             ("tay xuống", "down"), ("hạ tay xuống", "down")):
            intent, ent, _ = try_local_hold_place(phrase)
            self.assertEqual((intent, ent.get("pose")), ("arm_pose", pose), phrase)
        # "lên trên" with nothing after it = lift (no destination), not a stack
        self.assertEqual(try_local_hold_place("gắp khối cube có hình xương cá lên trên")[0], "vision_pick_hold")
        # plain pick-and-hold keeps working
        self.assertEqual(try_local_hold_place("gắp cube xương cá lên")[0], "vision_pick_hold")

    def test_identity_context_lists_the_trash_faces_for_the_planner(self):
        self.assertIn("pin đã qua sử dụng", CI.identity_context()["trash_faces"]["ID3"])


if __name__ == "__main__":
    unittest.main()
