"""Nối khung thành đám mây điểm và tách hình dạng vật, bằng camera giả lập."""
import unittest

import cv2
import numpy as np

from cube_vision import pointcloud as PC
from cube_vision.frames import CameraModel, invert
from cube_vision.test_multiview import look_at

CAM = CameraModel("sim", (600.0, 600.0, 320.0, 240.0), (640, 480))


def poster(seed=0, size=600):
    """Ảnh hoa văn ngẫu nhiên (nhiều đặc trưng) dùng làm tấm phẳng trong cảnh."""
    rng = np.random.default_rng(seed)
    image = np.full((size, size, 3), 255, np.uint8)
    for _ in range(400):
        centre = tuple(int(v) for v in rng.integers(0, size, 2))
        colour = tuple(int(v) for v in rng.integers(0, 255, 3))
        cv2.circle(image, centre, int(rng.integers(4, 14)), colour, -1)
    return image


def render_plane(image, a_T_optical, z, extent=0.12, centre=(-0.20, 0.0)):
    """Khung camera nhìn một tấm phẳng nằm ngang ở độ cao z (tấm vuông cạnh 2*extent quanh `centre`)."""
    n = image.shape[0]
    corners = np.array([[centre[0] - extent, centre[1] - extent, z], [centre[0] + extent, centre[1] - extent, z],
                        [centre[0] + extent, centre[1] + extent, z], [centre[0] - extent, centre[1] + extent, z]])
    T = invert(a_T_optical)
    uv = CAM.project(corners @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [n, 0], [n, n], [0, n]]), uv)
    return cv2.warpPerspective(image, H, (640, 480), borderValue=(128, 128, 128))


def cylinder_points(centre, radius, height, arc=(100, 260), n=300, noise=0.001, seed=0):
    """Điểm trên mặt trụ ở nửa quay về gốc tọa độ (camera tay nhìn từ đế ra), z tính từ mặt bàn 0."""
    rng = np.random.default_rng(seed)
    toward = np.degrees(np.arctan2(-centre[1], -centre[0]))
    angles = np.radians(toward + rng.uniform(arc[0] - 180, arc[1] - 180, n))
    xyz = np.c_[centre[0] + radius * np.cos(angles), centre[1] + radius * np.sin(angles), rng.uniform(0.02, height, n)]
    return xyz + rng.normal(0.0, noise, xyz.shape)


def cloud_of(points):
    return {"points": points, "colours": np.zeros((len(points), 3), np.uint8),
            "pairs": np.tile([0, 1], (len(points), 1)), "miss_m": np.zeros(len(points))}


class Linking(unittest.TestCase):
    def test_overlapping_frames_with_known_poses_give_metric_points_on_the_surface(self):
        image, z = poster(), 0.06
        eyes = [[-0.02, -0.04, 0.26], [-0.02, 0.04, 0.26], [-0.05, 0.0, 0.30]]
        frames = [(render_plane(image, look_at(eye, [-0.20, 0.0, z]), z), CAM, look_at(eye, [-0.20, 0.0, z]))
                  for eye in eyes]
        cloud = PC.triangulate(frames)
        self.assertGreater(len(cloud["points"]), 100)
        self.assertLess(abs(float(np.median(cloud["points"][:, 2])) - z), 0.003)
        self.assertLess(float(np.percentile(np.abs(cloud["points"][:, 2] - z), 90)), 0.008)
        self.assertTrue((np.abs(cloud["points"][:, 0] + 0.20) < 0.14).all())

    def test_frames_without_overlap_or_baseline_give_no_points(self):
        z = 0.06
        T = look_at([-0.02, 0.0, 0.26], [-0.20, 0.0, z])
        same_place = [(render_plane(poster(), T, z), CAM, T)] * 2
        self.assertEqual(len(PC.triangulate(same_place)["points"]), 0)
        different = [(render_plane(poster(1), T, z), CAM, T),
                     (render_plane(poster(2), look_at([-0.02, 0.06, 0.26], [-0.20, 0.0, z]), z), CAM,
                      look_at([-0.02, 0.06, 0.26], [-0.20, 0.0, z]))]
        cloud = PC.triangulate(different)                      # chỉ còn vài điểm ở viền tấm (viền là hình học thật)
        self.assertLess(len(cloud["points"]), 20)
        self.assertEqual(PC.objects(cloud, 0.0), [])


class Shapes(unittest.TestCase):
    def test_a_cup_seen_from_one_side_becomes_a_cylinder_with_its_height(self):
        found = PC.objects(cloud_of(cylinder_points([-0.18, -0.12], 0.035, 0.095)), 0.0)
        self.assertEqual(len(found), 1)
        cup = found[0]
        self.assertEqual(cup["shape"], "cylinder")
        self.assertLess(np.linalg.norm(cup["centre"] - [-0.18, -0.12]), 0.008)
        self.assertAlmostEqual(cup["width_m"], 0.07, delta=0.012)
        self.assertAlmostEqual(cup["height_m"], 0.095, delta=0.008)

    def test_a_flat_face_becomes_a_box_not_a_cylinder(self):
        rng = np.random.default_rng(1)
        wall = np.c_[np.full(200, -0.30) + rng.normal(0, 0.001, 200), rng.uniform(-0.08, 0.0, 200),
                     rng.uniform(0.02, 0.07, 200)]
        found = PC.objects(cloud_of(wall), 0.0)
        self.assertEqual(found[0]["shape"], "box")
        self.assertAlmostEqual(found[0]["width_m"], 0.08, delta=0.01)

    def test_two_objects_are_separated_and_stray_points_do_not_bridge_them(self):
        rng = np.random.default_rng(2)
        cup, far_cup = cylinder_points([-0.18, -0.12], 0.035, 0.09), cylinder_points([-0.25, 0.10], 0.03, 0.06, seed=3)
        stray = np.c_[rng.uniform(-0.3, -0.1, 30), rng.uniform(-0.15, 0.12, 30), rng.uniform(0.02, 0.1, 30)]
        found = PC.objects(cloud_of(np.vstack([cup, far_cup, stray])), 0.0)
        self.assertEqual(len(found), 2)

    def test_table_prints_known_cubes_and_out_of_region_objects_are_dropped(self):
        cup = cylinder_points([-0.18, -0.12], 0.035, 0.09)
        flat = np.c_[np.random.default_rng(4).uniform(-0.3, -0.2, (200, 2)), np.full(200, 0.003)]
        self.assertEqual(PC.objects(cloud_of(flat), 0.0), [])
        self.assertEqual(PC.objects(cloud_of(cup), 0.0, inside=lambda xy: False), [])
        cube = np.c_[np.random.default_rng(5).uniform(-0.015, 0.015, (100, 2)) + [-0.2, 0.0], np.full(100, 0.03)]
        self.assertEqual(PC.objects(cloud_of(cube), 0.0, keep_out=[(-0.2, 0.0)]), [])

    def test_outline_in_a_frame_gives_the_true_width_and_resizing_keeps_the_near_edge(self):
        centre, radius, z = np.array([-0.20, 0.0]), 0.04, 0.05
        T = look_at([-0.02, 0.0, 0.25], [centre[0], centre[1], z])
        ring = np.linspace(0, 2 * np.pi, 80)
        rim = np.c_[centre[0] + radius * np.cos(ring), centre[1] + radius * np.sin(ring)]
        solid = np.r_[np.c_[rim, np.zeros(80)], np.c_[rim, np.full(80, 0.10)]]
        inv = invert(T)
        mask = np.zeros((480, 640), np.uint8)
        cv2.fillConvexPoly(mask, cv2.convexHull(CAM.project(solid @ inv[:3, :3].T + inv[:3, 3]).astype(np.float32))
                           .astype(np.int32), 255)
        width = PC.silhouette_width(CAM, T, mask, np.r_[centre, z])
        self.assertAlmostEqual(width, 2 * radius, delta=0.012)
        self.assertIsNone(PC.silhouette_width(CAM, T, np.full_like(mask, 255), np.r_[centre, z]))   # vật bị cắt ở mép
        blob = cylinder_points(centre, radius, 0.10, arc=(150, 210), noise=0.003)      # chỉ dải giữa mặt trước, nhiễu sâu
        item = {"points": blob, "centre": blob[:, :2].mean(axis=0), "width_m": 0.03, "shape": "box", "height_m": 0.1}
        cup = PC.cylinder_from_near_edge(item, width / 2.0)
        self.assertEqual(cup["shape"], "cylinder")
        self.assertLess(np.linalg.norm(cup["centre"] - centre), 0.01)
        self.assertEqual(cup["height_m"], 0.1)
