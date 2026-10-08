import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cube_vision import cameras as C  # noqa: E402


def textured(seed, shape=(480, 640)):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, shape, dtype=np.uint8)
    img = cv2.GaussianBlur(img, (0, 0), 3)
    return cv2.cvtColor(cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX), cv2.COLOR_GRAY2BGR)


def shifted(frame, dx):
    return np.roll(frame, dx, axis=1)


def arm_in_view(frame):
    out = frame.copy()
    cv2.rectangle(out, (300, 200), (380, 330), (255, 255, 255), -1)      # cánh tay đi qua một vùng
    return out


def make_sysfs(root, entries):
    for node, (name, index) in entries.items():
        d = Path(root) / node
        d.mkdir(parents=True)
        (d / "name").write_text(name + "\n")
        (d / "index").write_text(str(index))
    return root


DEV = [{"path": "/dev/video0", "name": "Sonix A", "usb": "/usb/1"},
       {"path": "/dev/video2", "name": "Acer B", "usb": "/usb/2"}]


class Identify(unittest.TestCase):
    def run_identify(self, wrist_path, ext_path="/dev/video2", scene_seed=1, ext_moves=True):
        base = {p: textured(i + scene_seed) for i, p in enumerate((wrist_path, ext_path))}
        state = {"rotated": False}

        def grab(devs):
            out = {}
            for d in devs:
                f = base[d["path"]]
                if state["rotated"]:
                    f = (shifted(f, 45) if d["path"] == wrist_path else
                         (arm_in_view(f) if ext_moves else f))
                out[d["path"]] = f
            return out

        def rotate(sign):
            state["rotated"] = sign > 0

        return C.identify_wrist(DEV, grab, rotate)

    def test_the_camera_whose_whole_image_shifts_is_the_wrist(self):
        for wrist in ("/dev/video0", "/dev/video2"):
            other = "/dev/video2" if wrist == "/dev/video0" else "/dev/video0"
            result = self.run_identify(wrist, other)
            self.assertEqual(result["wrist"], wrist, result)

    def test_no_motion_anywhere_is_not_guessed(self):
        base = textured(3)
        result = C.identify_wrist(DEV, lambda devs: {d["path"]: base for d in devs}, lambda s: None)
        self.assertIsNone(result["wrist"])

    def test_two_cameras_that_both_move_are_ambiguous(self):
        base = {d["path"]: textured(i) for i, d in enumerate(DEV)}
        state = {"r": False}
        grab = lambda devs: {d["path"]: (shifted(base[d["path"]], 45) if state["r"] else base[d["path"]])  # noqa: E731
                             for d in devs}

        def rotate(sign):
            state["r"] = sign > 0

        result = C.identify_wrist(DEV, grab, rotate)
        self.assertIsNone(result["wrist"])
        self.assertIn("mơ hồ", result["reason"])


class Listing(unittest.TestCase):
    def test_only_capture_nodes_are_listed(self):
        with tempfile.TemporaryDirectory() as tmp:
            make_sysfs(tmp, {"video0": ("Acer HD", 0), "video1": ("Acer HD", 1),
                             "video2": ("Sonix", 0), "video3": ("Sonix", 1)})
            names = [(d["path"], d["name"]) for d in C.list_cameras(Path(tmp))]
        self.assertEqual(names, [("/dev/video0", "Acer HD"), ("/dev/video2", "Sonix")])


class Resolve(unittest.TestCase):
    def test_single_camera_needs_no_identification(self):
        path, source = C.resolve_wrist(DEV[:1], identify=lambda: self.fail("không được gọi"))
        self.assertEqual((path, source), ("/dev/video0", "only"))

    def test_identification_is_cached_by_name_and_survives_port_swap(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp) / "roles.json"
            calls = []

            def identify():
                calls.append(1)
                return {"wrist": "/dev/video0"}

            self.assertEqual(C.resolve_wrist(DEV, identify, cache), ("/dev/video0", "identified"))
            swapped = [{"path": "/dev/video2", "name": "Sonix A", "usb": "/usb/9"},
                       {"path": "/dev/video0", "name": "Acer B", "usb": "/usb/8"}]
            self.assertEqual(C.resolve_wrist(swapped, identify, cache), ("/dev/video2", "cache"))
            self.assertEqual(len(calls), 1)

    def test_changed_device_set_invalidates_the_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp) / "roles.json"
            C.write_cache(DEV, "/dev/video0", cache)
            other = [DEV[0], {"path": "/dev/video4", "name": "New cam", "usb": "/usb/3"}]
            self.assertEqual(C.resolve_wrist(other, None, cache), (None, "unknown"))

    def test_failed_identification_is_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = C.resolve_wrist(DEV, lambda: {"wrist": None, "reason": "x"}, Path(tmp) / "c.json")
        self.assertEqual(result, (None, "unknown"))

    def test_explicit_camera_argument_is_not_touched(self):
        self.assertEqual(C.resolve_arg("/dev/video7"), "/dev/video7")


if __name__ == "__main__":
    unittest.main()
