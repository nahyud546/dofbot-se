"""placement_check: ảnh tổng hợp (ô màu + cube trắng), không cần camera."""
import unittest

import numpy as np

from cube_vision import placement_check as P

BG = (172, 166, 172)
PADS = {1: ((150, 140, 106), (0, 40, 130, 120)),      # BGR, (x0, y0, x1, y1)
        2: ((54, 97, 51), (510, 40, 640, 120)),
        3: ((40, 40, 180), (0, 150, 90, 240)),
        4: ((75, 80, 79), (550, 150, 640, 260))}


def scene(cubes=()):
    img = np.full((480, 640, 3), BG, np.uint8)
    for colour, (x0, y0, x1, y1) in PADS.values():
        img[y0:y1, x0:x1] = colour
    img[0:110, 220:440] = (20, 20, 20)                  # đế robot (vùng loại trừ)
    for x, y, size in cubes:
        img[y:y + size, x:x + size] = (235, 235, 235)
        img[y + size // 3:y + 2 * size // 3, x + size // 3:x + 2 * size // 3] = (60, 60, 60)
    return img


class FindPad(unittest.TestCase):
    def test_all_four_pads_are_found(self):
        for zone in PADS:
            self.assertIsNotNone(P.find_pad(scene(), zone), zone)

    def test_pad_hull_survives_a_cube_sitting_on_it(self):
        pad = P.find_pad(scene([(30, 175, 45)]), 3)
        self.assertIsNotNone(pad)
        self.assertGreater(pad.area, 5000)

    def test_missing_pad_returns_none(self):
        img = scene()
        img[150:240, 0:90] = BG
        self.assertIsNone(P.find_pad(img, 3))


def robot_decoy(img):
    """Phần xám đậm của robot to hơn ô xám thật, nằm ở nửa trên ảnh."""
    img[20:170, 230:420] = (82, 84, 83)
    return img


class Robust(unittest.TestCase):
    def test_grey_pad_is_not_confused_with_the_dark_grey_of_the_robot(self):
        pad = P.find_pad(robot_decoy(scene()), 4)
        centre = pad.hull.mean(axis=0)
        self.assertGreater(centre[0], 540)
        self.assertGreater(centre[1], 140)

    def test_hidden_grey_pad_is_inferred_from_the_other_three(self):
        img = scene()
        img[150:260, 550:640] = BG                                  # ô xám bị che hoàn toàn
        pad = P.find_pad(img, 4)
        self.assertTrue(pad.inferred)
        cx, cy = pad.hull.mean(axis=0)
        self.assertAlmostEqual(cx, 555, delta=25)   # tâm xanh lá + đỏ - xanh dương
        self.assertAlmostEqual(cy, 195, delta=25)

    def test_verdict_uses_the_real_pad_when_the_robot_looks_like_it(self):
        before = robot_decoy(scene([(280, 300, 45)]))
        after = robot_decoy(scene([(575, 180, 45)]))
        self.assertEqual(P.verify_in_zone(before, after, 4)["verdict"], "in_zone")

    def test_tall_cube_whose_top_sticks_out_behind_the_pad_is_still_in_zone(self):
        before = scene()
        after = scene()
        after[150:200, 585:625] = (235, 235, 235)                   # phần trên cube lòi ra trên mép ô
        after[200:260, 585:625] = (235, 235, 235)
        # ô xám y 150–260: cube đặt sát mép trên; thay bằng cube cao hơn ô
        after2 = scene()
        after2[120:215, 570:625] = (235, 235, 235)                  # 95 px cao: nửa dưới trong ô, nửa trên ngoài
        self.assertEqual(P.verify_in_zone(before, after2, 4)["verdict"], "in_zone")


class Verify(unittest.TestCase):
    before = scene([(280, 300, 45)])                     # cube nguồn trên bàn

    def after(self, cubes):
        return scene(cubes)                              # cube nguồn đã rời đi

    def test_cube_inside_the_pad_is_in_zone(self):
        r = P.verify_in_zone(self.before, self.after([(575, 180, 45)]), 4)
        self.assertEqual((r["verdict"], r["ok"]), ("in_zone", True))

    def test_each_zone_recognises_its_own_cube(self):
        spots = {1: (45, 55), 2: (530, 55), 3: (25, 170), 4: (575, 180)}
        for zone, (x, y) in spots.items():
            self.assertEqual(P.verify_in_zone(self.before, self.after([(x, y, 45)]), zone)["verdict"],
                             "in_zone", zone)

    def test_cube_on_the_table_is_outside(self):
        r = P.verify_in_zone(self.before, self.after([(300, 380, 45)]), 4)
        self.assertEqual((r["verdict"], r["ok"]), ("outside", False))

    def test_cube_half_off_the_pad_edge_is_partial(self):
        r = P.verify_in_zone(self.before, self.after([(522, 205, 45)]), 4)      # đè mép trái ô xám
        self.assertEqual(r["verdict"], "partial")

    def test_nothing_new_in_a_visible_pad_means_the_cube_is_missing(self):
        r = P.verify_in_zone(self.before, self.before.copy(), 4)
        self.assertEqual((r["verdict"], r["ok"]), ("missing", False))

    def test_exposure_drift_does_not_look_like_a_cube(self):
        dim = np.clip(self.before.astype(np.float32) * 0.85, 0, 255).astype(np.uint8)
        self.assertEqual(P.verify_in_zone(self.before, dim, 4)["verdict"], "missing")   # không có cube, không báo nhầm có

    def test_changes_around_the_robot_base_are_ignored(self):
        moved = self.after([(575, 180, 45)])
        moved[20:100, 250:420] = (200, 200, 200)                                # tay ở pose khác
        self.assertEqual(P.verify_in_zone(self.before, moved, 4)["verdict"], "in_zone")

    def test_pad_not_visible_is_unseen_with_reason(self):
        blank = np.full((480, 640, 3), BG, np.uint8)
        r = P.verify_in_zone(blank, blank.copy(), 3)
        self.assertEqual(r["verdict"], "unseen")
        self.assertIn("không thấy ô", r["metrics"]["reason"])

    def test_missing_frames_are_unseen(self):
        self.assertEqual(P.verify_in_zone(None, self.before, 4)["verdict"], "unseen")


class FakeCamera:
    def __init__(self, frames):
        self.frames = list(frames)

    def grab(self):
        return self.frames.pop(0) if self.frames else None


class SequenceCamera(FakeCamera):
    def sequence(self, count, interval_s):
        return [self.frames.pop(0) for _ in range(min(count, len(self.frames)))]


class Verifier(unittest.TestCase):
    def test_before_then_check(self):
        before, after = scene([(280, 300, 45)]), scene([(575, 180, 45)])
        verifier = P.PlacementVerifier(FakeCamera([before, after]), debug_dir=None)
        self.assertTrue(verifier.before())
        self.assertTrue(verifier.check(4)["ok"])

    def test_cube_that_lands_then_gets_thrown_out_is_reported_as_bounced(self):
        before = scene([(280, 300, 45)])
        landed, thrown = scene([(575, 180, 45)]), scene([(300, 380, 45)])
        verifier = P.PlacementVerifier(SequenceCamera([before, landed, thrown, thrown]), debug_dir=None)
        verifier.before()
        result = verifier.check(4)
        self.assertEqual(result["timeline"], ["in_zone", "outside", "outside"])
        self.assertTrue(result["bounced"])
        self.assertFalse(result["ok"])
        self.assertIn("bị văng", result["text"])

    def test_cube_that_stays_is_ok_on_every_sample(self):
        before = scene([(280, 300, 45)])
        stay = scene([(575, 180, 45)])
        verifier = P.PlacementVerifier(SequenceCamera([before, stay, stay, stay]), debug_dir=None)
        verifier.before()
        result = verifier.check(4)
        self.assertEqual(result["timeline"], ["in_zone"] * 3)
        self.assertFalse(result["bounced"])
        self.assertTrue(result["ok"])

    def test_cube_thrown_out_of_the_frame_is_missing_not_unseen(self):
        before = scene([(280, 300, 45)])
        verifier = P.PlacementVerifier(SequenceCamera([before, scene(), scene(), scene()]), debug_dir=None)
        verifier.before()
        self.assertEqual(verifier.check(4)["verdict"], "missing")

    def test_camera_without_frames_is_unseen(self):
        verifier = P.PlacementVerifier(FakeCamera([]), debug_dir=None)
        self.assertFalse(verifier.before())
        self.assertEqual(verifier.check(4)["verdict"], "unseen")


if __name__ == "__main__":
    unittest.main()
