"""Khảo sát zone trước khi sort: quyết định, điều phối look/survey, pose zone từ vị trí ô."""
import math
import unittest

import t8_motion_worker as worker
import zone_survey as ZS
from t8_ros_tasks import RosTaskRunner
from test_handeye_mode import Backend, lock, obj, scene


def rot(xy, deg):
    t = math.radians(deg)
    return [xy[0] * math.cos(t) - xy[1] * math.sin(t), xy[0] * math.sin(t) + xy[1] * math.cos(t)]


def seen(zone, center_xy=None, margin_cfg=None, area_m2=0.0055, clipped=False):
    return {"zone_id": zone, "seen": True, "clipped": clipped, "area_m2": area_m2,
            "center_xy": center_xy if center_xy is not None else ZS.release_xy(zone),
            "margin_cfg_mm": margin_cfg, "delta_deg": 0.0, "margin_best_mm": margin_cfg}


class Decisions(unittest.TestCase):
    def test_release_point_deep_in_the_pad_keeps_the_table(self):
        item = ZS.decide_zone(3, [seen(3, margin_cfg=22.0)])
        self.assertEqual(item["action"], "keep")

    def test_displaced_pad_is_targeted_at_its_centre(self):
        centre = rot(ZS.release_xy(3), -20)             # ô dời 20° quanh đế
        item = ZS.decide_zone(3, [seen(3, centre, margin_cfg=-30.0)])
        self.assertEqual(item["action"], "moved")
        self.assertEqual(item["target_xy"], [round(centre[0], 4), round(centre[1], 4)])
        self.assertIn("thả vào tâm ô", item["text"])

    def test_pad_cut_a_lot_by_the_frame_is_not_trusted(self):
        item = ZS.decide_zone(3, [seen(3, margin_cfg=-30.0, clipped=True, area_m2=0.0030)])
        self.assertEqual(item["action"], "fallback")
        self.assertIn("khung ảnh cắt", item["text"])

    def test_pad_that_only_touches_the_frame_edge_is_trusted(self):
        item = ZS.decide_zone(3, [seen(3, rot(ZS.release_xy(3), -20), margin_cfg=-30.0,
                                       clipped=True, area_m2=0.00525)])
        self.assertEqual(item["action"], "moved")

    def test_implausible_pad_area_is_rejected(self):
        for area in (0.0003, 0.0026, 0.05):
            self.assertEqual(ZS.decide_zone(3, [seen(3, margin_cfg=-30.0, area_m2=area)])["action"],
                             "fallback")

    def test_pad_unseen_falls_back_with_a_warning(self):
        item = ZS.decide_zone(1, [{"seen": False}, None])
        self.assertEqual(item["action"], "fallback")
        self.assertIn("không thấy ô", item["text"])

    def test_pad_far_from_the_configured_point_is_a_measurement_error(self):
        far = [ZS.release_xy(2)[0] + 0.2, ZS.release_xy(2)[1]]
        item = ZS.decide_zone(2, [seen(2, far, margin_cfg=-30.0)])
        self.assertEqual(item["action"], "fallback")
        self.assertIn("nghi đo sai", item["text"])

    def test_pad_the_arm_cannot_reach_is_blocked_not_dropped_by_the_table(self):
        item = ZS.decide_zone(1, [seen(1, [-0.0052, 0.1603], margin_cfg=-50.0)])     # cần J1 ≈ 2°
        self.assertEqual(item["action"], "blocked")
        self.assertIn("giới hạn đế", item["text"])
        self.assertIsNone(item["target_xy"])

    def test_config_point_on_another_pad_is_blocked(self):
        # Log thật: ô đỏ đang nằm đúng chỗ điểm thả cấu hình của zone xanh dương, ô xanh ở nơi khác.
        red_on_blue_cfg = [ZS.release_xy(1)[0] + 0.017, ZS.release_xy(1)[1] + 0.006]
        targets, report = ZS.decide_zones({3: [seen(3, red_on_blue_cfg, margin_cfg=-40.0)],
                                           1: [{"seen": False}]}, zones=(1,))
        self.assertEqual(targets, {})
        self.assertEqual(report[0]["action"], "blocked")
        self.assertIn("sai ô", report[0]["text"])

    def test_unseen_pad_with_a_clear_config_point_still_falls_back(self):
        far_pad = rot(ZS.release_xy(3), 90)
        _, report = ZS.decide_zones({3: [seen(3, far_pad, margin_cfg=-40.0)], 1: [{"seen": False}]},
                                    zones=(1,))
        self.assertEqual(report[0]["action"], "fallback")

    def test_only_moved_zones_produce_targets(self):
        targets, report = ZS.decide_zones({3: [seen(3, rot(ZS.release_xy(3), 10), margin_cfg=-20.0)],
                                           1: [seen(1, margin_cfg=30.0)]}, zones=(1, 3, 4))
        self.assertEqual(list(targets), [3])
        self.assertEqual([r["action"] for r in report], ["keep", "moved", "fallback"])


class Symmetry(unittest.TestCase):
    """Số đo thật: xanh dương (-0.0047, 0.1603), xanh lá (-0.0132, -0.1636), đỏ (-0.0741, 0.1689)."""
    BLUE, GREEN, RED = [-0.0047, 0.1603], [-0.0132, -0.1636], [-0.0741, 0.1689]

    def test_missing_grey_is_the_mirror_of_red_across_the_blue_green_axis(self):
        entries = {1: [seen(1, self.BLUE, margin_cfg=-50.0)], 2: [seen(2, self.GREEN, margin_cfg=-50.0)],
                   3: [seen(3, self.RED, margin_cfg=-40.0)], 4: [{"seen": False}]}
        targets, report = ZS.decide_zones(entries, zones=(4,))
        item = report[0]
        self.assertTrue(item["inferred"])
        self.assertIn("suy ra từ đối xứng với zone 3", item["text"])
        x, y = targets[4]
        self.assertAlmostEqual(x, -0.0741 - 0.0085 * 0 , delta=0.012)        # cùng bên x với ô đỏ
        self.assertAlmostEqual(y, -0.1636 - (0.1689 - 0.1603), delta=0.012)  # đối xứng qua trục giữa
        self.assertLess(y, 0)

    def test_an_unreliable_grey_mask_is_replaced_by_the_inference(self):
        entries = {1: [seen(1, self.BLUE, margin_cfg=-50.0)], 2: [seen(2, self.GREEN, margin_cfg=-50.0)],
                   3: [seen(3, self.RED, margin_cfg=-40.0)], 4: [seen(4, [0.0, -0.17], area_m2=0.0015)]}
        _, report = ZS.decide_zones(entries, zones=(4,))
        self.assertEqual(report[0]["action"], "moved")
        self.assertTrue(report[0]["inferred"])

    def test_no_complete_pair_means_no_inference(self):
        entries = {1: [seen(1, self.BLUE, margin_cfg=-50.0)], 3: [seen(3, self.RED, margin_cfg=-40.0)]}
        self.assertEqual(ZS.infer_missing_xy(entries), {})

    def test_implausible_pair_is_not_used_as_the_axis(self):
        far = [self.GREEN[0], self.GREEN[1] - 0.3]
        entries = {1: [seen(1, self.BLUE, margin_cfg=-50.0)], 2: [seen(2, far, margin_cfg=-50.0)],
                   3: [seen(3, self.RED, margin_cfg=-40.0)]}
        self.assertEqual(ZS.infer_missing_xy(entries), {})

    def test_measured_zone_is_never_overridden_by_inference(self):
        entries = {1: [seen(1, self.BLUE, margin_cfg=-50.0)], 2: [seen(2, self.GREEN, margin_cfg=-50.0)],
                   3: [seen(3, self.RED, margin_cfg=-40.0)], 4: [seen(4, [-0.075, -0.17], margin_cfg=-40.0)]}
        targets, report = ZS.decide_zones(entries, zones=(4,))
        self.assertFalse(report[0].get("inferred"))
        self.assertEqual(targets[4], [-0.075, -0.17])


class FakeMotion:
    def __init__(self, fail=False):
        self.calls, self.closed, self.fail = [], 0, fail

    def execute(self, command, **payload):
        self.calls.append((command, payload))
        if command == "look":
            if self.fail:
                return {"ok": False, "reply": "không tới được"}
            return {"ok": True, "servo": payload["servo"],
                    "from_servo": [90.0, 100.0, 20.0, 10.0, 90.0]}
        return {"ok": True}

    def close(self):
        self.closed += 1

    def look_j1s(self):
        return [p["servo"][0] for c, p in self.calls if c == "look"]


class FakeScene:
    """table: {(j1 nhìn, zone): entry}; thiếu thì "không thấy ô"."""

    def __init__(self, table):
        self.table, self.requests, self.expect = table, [], []

    def zone_survey(self, zones, timeout_s=6.0, expect_j1=None):
        ids = [z["zone_id"] for z in zones]
        self.requests.append(ids)
        self.expect.append(expect_j1)
        out = [self.table[(round(expect_j1), i)] for i in ids if (round(expect_j1), i) in self.table]
        return {"ok": True, "zones": out}


class Orchestration(unittest.TestCase):
    def test_two_views_cover_all_four_zones_then_restore_the_start_pose(self):
        t3 = rot(ZS.release_xy(3), -15)
        table = {(12, 3): seen(3, t3, margin_cfg=-25.0), (12, 1): seen(1, margin_cfg=30.0),
                 (165, 2): seen(2, margin_cfg=22.0), (165, 4): seen(4, margin_cfg=-20.0)}
        motion, scene_backend, logs = FakeMotion(), FakeScene(table), []
        result = ZS.run_survey(motion, scene_backend, log=logs.append)
        self.assertEqual(motion.look_j1s(), [12.0, 165.0, 90.0])      # hai phía rồi pose xuất phát
        looks = [p["servo"] for c, p in motion.calls if c == "look"]
        self.assertEqual(looks[0][1:], [110.0, 0.0, 0.0, 90.0])         # pose nhìn hạ thấp thấy trọn ô
        self.assertEqual(looks[-1], [90.0, 100.0, 20.0, 10.0, 90.0])
        self.assertEqual(scene_backend.requests, [[1, 3], [2, 4]])
        self.assertEqual(scene_backend.expect, [12.0, 165.0])
        self.assertEqual(result["targets"][3], [round(t3[0], 4), round(t3[1], 4)])
        self.assertIn(4, result["targets"])
        self.assertNotIn(1, result["targets"])                          # đủ biên: giữ cấu hình
        self.assertTrue(result["restored"])
        self.assertGreaterEqual(motion.closed, 3)

    def test_only_the_needed_side_is_visited(self):
        table = {(12, 3): seen(3, margin_cfg=25.0)}
        motion, scene_backend = FakeMotion(), FakeScene(table)
        ZS.run_survey(motion, scene_backend, zones=(3,), log=lambda *_: None)
        self.assertEqual(motion.look_j1s(), [12.0, 90.0])
        self.assertEqual(scene_backend.requests, [[1, 3]])      # zone cùng khung cũng đo để phát hiện chồng ô

    def test_pad_seen_partially_is_re_viewed_towards_its_centre(self):
        centre = rot(ZS.release_xy(3), -5)
        partial = seen(3, centre, margin_cfg=None, clipped=True, area_m2=0.0031)
        motion = FakeMotion()
        scene_backend = FakeScene({(12, 3): partial})
        ZS.run_survey(motion, scene_backend, zones=(3,), log=lambda *_: None)
        j1s = motion.look_j1s()
        self.assertEqual(j1s[0], 12.0)
        self.assertGreaterEqual(len(j1s), 3)                # nhìn lại rồi quay về
        self.assertNotAlmostEqual(j1s[1], 12.0, delta=3.0)

    def test_pad_not_in_view_scans_both_sides(self):
        motion = FakeMotion()
        # ô đỏ chỉ thấy khi nhìn lệch +25° -> J1=37°
        scene_backend = FakeScene({(37, 3): seen(3, rot(ZS.release_xy(3), -8), margin_cfg=-20.0)})
        result = ZS.run_survey(motion, scene_backend, zones=(3,), log=lambda *_: None)
        self.assertEqual(motion.look_j1s(), [12.0, 165.0, 8.0, 37.0, 90.0])   # nhìn phía kia để suy trục, rồi -25° kẹp ở 8°, +25°
        self.assertIn(3, result["targets"])

    def test_unseen_everywhere_keeps_config_and_restores(self):
        motion = FakeMotion()
        result = ZS.run_survey(motion, FakeScene({}), zones=(2,), log=lambda *_: None)
        self.assertEqual(result["targets"], {})
        self.assertTrue(result["restored"])
        self.assertEqual(result["report"][0]["action"], "fallback")

    def test_failed_look_keeps_config_and_never_raises(self):
        result = ZS.run_survey(FakeMotion(fail=True), FakeScene({(12, 3): seen(3)}), zones=(3,),
                               log=lambda *_: None)
        self.assertEqual(result["targets"], {})
        self.assertEqual(result["report"][0]["action"], "fallback")


class WorkerZonePoses(unittest.TestCase):
    def test_pad_where_the_table_expects_it_reproduces_the_table_poses(self):
        for zone in (1, 2, 3, 4):
            planned = worker.zone_poses_for_target(zone, ZS.release_xy(zone))
            for name, table in (("approach", worker.BIN_POSES), ("release", worker.BIN_RELEASE_POSES),
                                ("lift", worker.BIN_LIFT_POSES)):
                for got, want in zip(planned[name][:4], table[zone][:4]):
                    self.assertAlmostEqual(got, want, delta=2.5, msg=(zone, name))
                self.assertEqual(planned[name][4], table[zone][4])

    def test_displaced_pad_changes_the_base_angle_and_keeps_the_shape(self):
        planned = worker.zone_poses_for_target(3, rot(ZS.release_xy(3), -15))
        self.assertNotAlmostEqual(planned["release"][0], worker.BIN_RELEASE_POSES[3][0], delta=8)
        self.assertEqual(planned["release"][0], planned["release"][0])
        # approach/lift vẫn là cùng một cung: J1 gần nhau
        self.assertAlmostEqual(planned["approach"][0], planned["release"][0], delta=3.0)

    def test_far_or_invalid_targets_are_rejected(self):
        far = [ZS.release_xy(3)[0] + 0.3, ZS.release_xy(3)[1]]
        for bad in (far, [float("nan"), 0.0], None, "x"):
            with self.assertRaises(ValueError):
                worker.zone_poses_for_target(3, bad)

    def test_pad_close_to_the_base_uses_a_feasible_tilt(self):
        # ô đỏ đo thật: (-0.077, 0.167): không có nghiệm với độ nghiêng của bảng, có với độ nghiêng IK
        planned = worker.zone_poses_for_target(3, [-0.0775, 0.1673])
        for pose in planned.values():
            self.assertTrue(10.0 <= pose[0] <= 170.0)
        self.assertAlmostEqual(planned["release"][0], planned["approach"][0], delta=1.0)

    def test_pad_needing_a_base_angle_at_the_mechanical_limit_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            worker.zone_poses_for_target(1, [-0.0052, 0.1603])        # J1 ≈ 2°
        self.assertIn("giới hạn đế", str(caught.exception))

    def test_unreachable_target_has_no_ik_solution(self):
        with self.assertRaises(ValueError):
            worker.zone_poses_for_target(3, [-0.02, 0.05], max_move_m=1.0)


class GentleRelease(unittest.TestCase):
    def tcp_z(self, pose):
        import dofbot_ik
        return float(dofbot_ik.fk(pose[:5])[2])

    def test_level_zero_keeps_the_pose(self):
        pose = worker.BIN_RELEASE_POSES[3]
        self.assertEqual(worker.gentle_release_pose(pose, 0)[0], list(pose))

    def test_each_level_releases_lower_but_never_below_the_floor_and_keeps_the_base_angle(self):
        pose = worker.BIN_RELEASE_POSES[3]
        z0 = self.tcp_z(pose)
        zs = []
        for level in (1, 2):
            new, note = worker.gentle_release_pose(pose, level)
            zs.append(self.tcp_z(new))
            self.assertAlmostEqual(new[0], pose[0], delta=1.5)
            self.assertIn("thấp hơn", note)
        self.assertLess(zs[0], z0)
        self.assertLess(zs[1], zs[0])
        self.assertGreaterEqual(zs[1], worker.RELEASE_FLOOR_Z - 0.002)

    def test_a_release_that_is_already_low_is_never_raised(self):
        pose = worker.BIN_RELEASE_POSES[1]                       # TCP z ≈ 0,044 < sàn
        new, _ = worker.gentle_release_pose(pose, 2)
        self.assertLessEqual(self.tcp_z(new), self.tcp_z(pose) + 0.001)

    def test_gentler_levels_are_slower(self):
        L = worker.GENTLE_LEVELS
        self.assertLess(L[0]["lift_ms"], L[1]["lift_ms"])
        self.assertLess(L[1]["lift_ms"], L[2]["lift_ms"])
        self.assertLess(L[0]["open_ms"], L[2]["open_ms"])


class SortIntegration(unittest.TestCase):
    def run_sort(self, zone_check):
        red = obj(3)
        backend = Backend([scene([red])], [lock(red)])
        backend.zone_check = zone_check
        centre = rot(ZS.release_xy(3), -15)
        backend.zone_survey = FakeScene({(12, 3): seen(3, centre, margin_cfg=-25.0)}).zone_survey
        motion = FakeMotion()
        original = motion.execute

        def execute(command, **payload):
            if command == "look":
                return original(command, **payload)
            motion.calls.append((command, payload))
            return {"ok": True, "status": "executed_unverified", "reply": "Đã thả cube."}
        motion.execute = execute
        result = RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "khoi_do"})
        return result, motion, centre

    def test_target_reaches_the_sort_payload_and_reply(self):
        result, motion, centre = self.run_sort(True)
        commands = [c for c, _ in motion.calls]
        self.assertEqual(commands[0], "prepare")
        self.assertEqual(commands.count("look"), 2)
        self.assertEqual(commands[-1], "sort_cube_3d")
        self.assertEqual(motion.calls[-1][1]["zone_target_xy"],
                         {"3": [round(centre[0], 4), round(centre[1], 4)]})
        self.assertIn("Zone 3", result["reply"])

    def test_blocked_zone_refuses_the_sort_and_never_reaches_the_arm(self):
        blue = obj(1)
        backend = Backend([scene([blue])], [lock(blue)])
        backend.zone_check = True
        backend.zone_survey = FakeScene({(12, 1): seen(1, [-0.0052, 0.1603], margin_cfg=-50.0)}).zone_survey
        motion = FakeMotion()
        original = motion.execute

        def execute(command, **payload):
            if command == "look":
                return original(command, **payload)
            motion.calls.append((command, payload))
            return {"ok": True, "reply": "ok"}
        motion.execute = execute
        result = RosTaskRunner(backend, motion=motion).execute("sort_cube", {"label": "cube_1"})
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "zone_unavailable")
        self.assertNotIn("sort_cube_3d", [c for c, _ in motion.calls])
        self.assertEqual(motion.look_j1s()[-1], 90.0)           # tay đã về pose xuất phát

    def test_zone_check_off_skips_the_survey(self):
        result, motion, _ = self.run_sort(False)
        self.assertNotIn("look", [c for c, _ in motion.calls])
        self.assertNotIn("zone_target_xy", motion.calls[-1][1])


if __name__ == "__main__":
    unittest.main()
