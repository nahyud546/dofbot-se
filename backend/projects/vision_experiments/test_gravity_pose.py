import math

import numpy as np
import pytest

import cube_layer as L
import cube_search_center_math as M
import gravity_pose as G
import test_cube_layer as T

CAL = T.CAL


def face_corners_px(xy, layer, yaw, servo, edge=0.030, noise=0.0, rng=None, z_shift=0.0, tilt=0.0):
    """Pixel của 4 góc mặt trên cube (tâm xy, tầng, yaw) qua mô hình camera thật của test."""
    z = T.TAG_TOP_Z + L.CUBE_EDGE_M * layer + z_shift
    h = edge / 2
    c, s = math.cos(yaw), math.sin(yaw)
    B = M.fk_arm4(servo) @ T.MOUNT
    inv = np.linalg.inv(B)
    out = []
    for i, (dx, dy) in enumerate(((-h, -h), (h, -h), (h, h), (-h, h))):
        lift = tilt * (dx / h)         # nghiêng quanh trục y: một nửa mặt cao hơn
        p = np.array([xy[0] + c * dx - s * dy, xy[1] + s * dx + c * dy, z + lift, 1.0])
        cam = (inv @ p)[:3]
        if cam[2] < 0.06:
            return None
        x, y = cam[0] / cam[2], cam[1] / cam[2]
        d = 1 + T.TRUE_K1 * (x * x + y * y)
        out.append([T.TRUE_K[0] * x * d + T.TRUE_K[2], T.TRUE_K[1] * y * d + T.TRUE_K[3]])
    out = np.array(out)
    if rng is not None and noise:
        out = out + rng.normal(0, noise, out.shape)
    return out


def case(rng, layer):
    servo, xy, _ = T.make_case(rng, T.TRUE_PERCEPTION_K, layer)
    return servo, xy


@pytest.mark.parametrize("layer", [0, 1, 2, 3])
def test_layer_xy_and_yaw_recovered_from_four_points(layer):
    rng = np.random.default_rng(10 + layer)
    errs, ok = [], 0
    for _ in range(25):
        servo, xy = case(rng, layer)
        yaw = rng.uniform(-0.7, 0.7)
        quad = face_corners_px(xy, layer, yaw, servo, noise=0.6, rng=rng)
        if quad is None:
            continue
        loc = G.locate_face(quad, "+X", servo, CAL)
        if not loc["ok"]:
            continue
        ok += 1
        assert loc["layer"] == layer
        errs.append(math.hypot(loc["x"] - xy[0], loc["y"] - xy[1]) * 1000)
        d = (math.degrees(loc["base_yaw_rad"] - yaw) + 45) % 90 - 45
        assert abs(d) < 4.0
    assert ok >= 18
    assert np.median(errs) < 2.0 and max(errs) < 5.0


def test_yaw_matches_the_pnp_based_locate_cube():
    rng = np.random.default_rng(3)
    checked = 0
    for _ in range(20):
        servo, xy = case(rng, 0)
        yaw = rng.uniform(-0.7, 0.7)
        quad = face_corners_px(xy, 0, yaw, servo, noise=0.4, rng=rng)
        pose = T.perceive(xy, 0, yaw, servo, T.TRUE_PERCEPTION_K, rng)
        if quad is None or pose is None:
            continue
        a = G.locate_face(quad, "+X", servo, CAL)
        b = L.locate_cube(pose, servo, CAL, T.TRUE_PERCEPTION_K)
        if not (a["ok"] and b["ok"]):
            continue
        d = (a["yaw_deg"] - b["yaw_deg"] + 45) % 90 - 45
        assert abs(d) < 5.0
        assert a["layer"] == b["layer"]
        checked += 1
    assert checked >= 10


def test_small_printed_patch_is_not_a_cube_face():
    rng = np.random.default_rng(5)
    for _ in range(10):
        servo, xy = case(rng, 0)
        quad = face_corners_px(xy, 0, 0.2, servo, edge=0.010)      # mảnh 10 mm
        if quad is None:
            continue
        loc = G.locate_face(quad, "+X", servo, CAL)
        assert not loc["ok"] and "hình vuông" in loc["reason"]


def test_tilted_face_is_rejected():
    rng = np.random.default_rng(6)
    seen = 0
    for _ in range(20):
        servo, xy = case(rng, 0)
        quad = face_corners_px(xy, 0, 0.2, servo, tilt=0.012)
        if quad is None:
            continue
        seen += 1
        assert not G.locate_face(quad, "+X", servo, CAL)["ok"]
    assert seen >= 8


def test_j1_outside_calibrated_range_is_refused():
    rng = np.random.default_rng(7)
    servo, xy = case(rng, 0)
    quad = face_corners_px(xy, 0, 0.1, servo)
    cal = dict(CAL, j1_valid_range=(80.0, 95.0))
    servo = list(servo)
    servo[0] = 130.0
    loc = G.locate_face(quad, "+X", servo, cal)
    assert not loc["ok"] and "ngoài vùng" in loc["reason"]


def test_wireframe_projects_back_onto_the_corners():
    rng = np.random.default_rng(8)
    servo, xy = case(rng, 1)
    yaw = 0.3
    quad = face_corners_px(xy, 1, yaw, servo)
    z = T.TAG_TOP_Z + L.CUBE_EDGE_M
    wire = G.wireframe_pixels(xy[0], xy[1], z, yaw, servo, CAL)
    assert all(p is not None for p in wire)
    # camera hiệu chuẩn (CAL) trùng mô hình thật của test nên 4 đỉnh mặt trên khớp quad
    top = np.array(wire[:4])
    assert np.max(np.linalg.norm(top - quad, axis=1)) < 1.5


def test_up_axis_follows_the_label_sign():
    for axis in ("+X", "-X", "+Y", "-Z", "+Z"):
        R, i = G.base_R_cube_from_face(axis, 0.4)
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.linalg.det(R) > 0
        sign = 1 if axis[0] == "+" else -1
        v = np.zeros(3)
        v[i] = sign
        assert np.allclose(R @ v, [0, 0, 1])
