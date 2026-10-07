import unittest

from cube_vision import batch_plan as B
from cube_vision.batch_plan import CubeInfo as C


class Words(unittest.TestCase):
    def test_all_synonyms(self):
        for word in ("all", "ALL", "tat_ca", "tat ca", "every", "moi"):
            self.assertTrue(B.is_all(word), word)
        for word in ("cube_1", "khoi_do", "", None):
            self.assertFalse(B.is_all(word))


class Sort(unittest.TestCase):
    def test_reachable_first_then_higher_layer_then_nearest(self):
        order, skipped = B.plan_sort([C(1, (0.20, 0.0), 0, True), C(2, (0.10, 0.0), 0, True),
                                      C(3, (0.30, 0.0), 1, True), C(4, (0.12, 0.0), 0, False)])
        self.assertEqual(order, [3, 2, 1])
        self.assertEqual(skipped, [(4, "IK không tới được")])

    def test_unknown_reachability_is_still_attempted_after_known_good(self):
        order, _ = B.plan_sort([C(1, None, 0, None), C(2, (0.2, 0.0), 0, True)])
        self.assertEqual(order, [2, 1])


class Stack(unittest.TestCase):
    cubes = [C(1, (0.20, 0.10), 0, True), C(2, (0.18, 0.01), 0, True),
             C(3, (0.25, -0.08), 0, True), C(4, (0.15, 0.05), 0, True)]

    def test_base_is_the_most_central_and_tower_chains_each_top(self):
        plan = B.plan_stack(self.cubes)
        self.assertEqual(plan["base"], 2)
        self.assertEqual(plan["steps"], [(4, 2), (1, 4), (3, 1)])     # gần đế trước; mỗi bước lên đỉnh trước đó
        self.assertEqual([t for _, t in plan["steps"]], [2] + [s for s, _ in plan["steps"][:-1]])

    def test_explicit_base_is_honoured(self):
        plan = B.plan_stack(self.cubes, base=1)
        self.assertEqual(plan["base"], 1)
        self.assertEqual(plan["steps"][0][1], 1)

    def test_tower_height_is_capped_and_leftover_reported(self):
        five = self.cubes + [C(5, (0.3, 0.0), 0, True)]
        plan = B.plan_stack(five, max_height=4)
        self.assertEqual(len(plan["steps"]), 3)
        self.assertEqual(len(plan["skipped"]), 1)

    def test_stacked_or_unreachable_cubes_are_left_out(self):
        plan = B.plan_stack([C(1, (0.2, 0.0), 0, True), C(2, (0.2, 0.05), 1, True), C(3, (0.2, 0.1), 0, False),
                             C(4, (0.18, 0.02), 0, True)])
        self.assertEqual({i for i, _ in plan["skipped"]}, {2, 3})
        self.assertEqual(len(plan["steps"]), 1)

    def test_single_cube_cannot_be_stacked(self):
        plan = B.plan_stack([C(1, (0.2, 0.0), 0, True)])
        self.assertEqual(plan["steps"], [])
        self.assertTrue(plan["reason"])

    def test_missing_base_is_an_error(self):
        self.assertEqual(B.plan_stack(self.cubes[:2], base=4)["steps"], [])


class Policy(unittest.TestCase):
    def test_per_cube_failures_continue_system_failures_stop(self):
        self.assertTrue(B.should_continue({"ok": True}))
        self.assertTrue(B.should_continue({"ok": False, "code": "approval_timeout"}))
        self.assertTrue(B.should_continue({"ok": False, "code": "zone_unavailable"}))
        for code in ("approval_cancelled", "robot_not_empty", "prepare_failed", None, "viewer_unavailable"):
            self.assertFalse(B.should_continue({"ok": False, "code": code}), code)

    def test_summary(self):
        text = B.summarize("Sort tất cả", [("sort cube 4", {"ok": True}),
                                            ("sort cube 1", {"ok": False, "reply": "hết hạn"})], [(2, "IK")])
        self.assertIn("xong 1/2", text)
        self.assertIn("sort cube 1 (hết hạn)", text)
        self.assertIn("cube 2 (IK)", text)


if __name__ == "__main__":
    unittest.main()
