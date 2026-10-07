"""cube_vision.zone_survey độc lập với T8: layout giả, không import T8/ROS."""
import math
import unittest

from cube_vision import zone_survey as ZS

CENTRES = {1: (0.10, 0.15), 2: (0.10, -0.15), 3: (0.20, 0.15), 4: (0.20, -0.15)}


def layout(reach=lambda zone, xy: True):
    def plan(zone, xy):
        if not reach(zone, xy):
            raise ValueError("ngoài tầm tay")
        return {"release": list(xy)}
    return ZS.ZoneLayout(release_xy=lambda z: list(CENTRES[z]), configured_j1=lambda z: 90.0 + z,
                         plan=plan, ik_j1=lambda xy: math.degrees(math.atan2(xy[1], xy[0])) + 90.0)


def seen(zone, xy=None, margin=None, area=0.0053):
    return {"zone_id": zone, "seen": True, "area_m2": area, "center_xy": list(xy or CENTRES[zone]),
            "margin_cfg_mm": margin}


class Core(unittest.TestCase):
    def test_actions(self):
        lay = layout()
        self.assertEqual(ZS.decide_zone(lay, 1, [seen(1, margin=30)])["action"], "keep")
        moved = ZS.decide_zone(lay, 1, [seen(1, (0.12, 0.17), margin=-20)])
        self.assertEqual((moved["action"], moved["target_xy"]), ("moved", [0.12, 0.17]))
        self.assertEqual(ZS.decide_zone(lay, 1, [{"seen": False}])["action"], "fallback")

    def test_unreachable_measured_pad_is_blocked(self):
        item = ZS.decide_zone(layout(lambda z, xy: False), 1, [seen(1, (0.12, 0.17), margin=-20)])
        self.assertEqual(item["action"], "blocked")

    def test_config_point_covered_by_another_pad_is_blocked(self):
        entries = {3: [seen(3, (0.11, 0.155), margin=-40)], 1: [{"seen": False}]}
        _, report = ZS.decide_zones(layout(), entries, zones=(1,))
        self.assertEqual(report[0]["action"], "blocked")

    def test_survey_visits_each_side_and_restores_the_pose(self):
        calls = []

        class Arm:
            def execute(self, command, **kw):
                calls.append(kw["servo"][0])
                return {"ok": True, "from_servo": [90, 100, 20, 10, 90]}

        class Scene:
            def zone_survey(self, zones, expect_j1=None):
                return {"ok": True, "zones": [seen(z["zone_id"], margin=25) for z in zones]}

        result = ZS.run_survey(Arm(), Scene(), layout(), zones=(1, 2), log=lambda *_: None)
        self.assertEqual(calls, [12.0, 165.0, 90])
        self.assertTrue(result["restored"])
        self.assertEqual(sorted(result["measured"]), [1, 2, 3, 4])

    def test_infrastructure_failure_stops_after_one_view_and_restores(self):
        calls = []

        class Arm:
            def execute(self, command, **kw):
                calls.append(kw["servo"][0])
                return {"ok": True, "from_servo": [90, 100, 20, 10, 90]}

        class Scene:
            def zone_survey(self, zones, expect_j1=None):
                return {"ok": False, "reason": "Không có khung camera tay"}

        result = ZS.run_survey(Arm(), Scene(), layout(), zones=(1, 2), log=lambda *_: None)
        self.assertEqual(calls, [12.0, 90])                 # không xoay thêm 5 góc, rồi về pose xuất phát
        self.assertIn("khung camera", result["error"])
        self.assertTrue(result["restored"])


if __name__ == "__main__":
    unittest.main()
