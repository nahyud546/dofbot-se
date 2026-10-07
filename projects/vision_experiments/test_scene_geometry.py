import cv2
import numpy as np
import pytest

import scene_geometry as G
import zone_locator as Z
from test_zone_locator import CAL, SERVO, TABLE_Z, look_at, rect, render


def square(cx, cy, side):
    return np.array(rect(cx, cy, side, side))


def test_pad_polygon_recovers_the_metric_pad():
    cx, cy = look_at()
    mask = Z.pad_mask(render(rect(cx, cy, 0.07, 0.07)), 3)
    poly = G.pad_polygon_base(mask, SERVO, CAL, TABLE_Z)
    assert poly is not None and len(poly) >= 4
    assert np.allclose(poly.mean(axis=0), [cx, cy], atol=0.006)
    assert G.signed_distance_mm((cx, cy), poly) == pytest.approx(35.0, abs=6.0)


def test_classify_in_wrong_out_and_unknown():
    polys = {1: square(-0.10, 0.17, 0.07), 3: square(-0.16, 0.14, 0.07)}
    inside = G.classify(3, (-0.16, 0.14), polys)
    assert (inside.status, inside.zone_found) == (G.IN_ZONE, 3)
    wrong = G.classify(3, (-0.10, 0.17), polys)
    assert (wrong.status, wrong.zone_found) == (G.WRONG_ZONE, 1)
    near_edge = G.classify(3, (-0.16 + 0.035 - 0.002, 0.14), polys)            # 2 mm inside: not enough margin
    assert near_edge.status == G.OUT_OF_ZONE and near_edge.margin_mm < G.DEFAULT_MIN_MARGIN_MM
    assert G.classify(3, (0.0, 0.0), polys).status == G.OUT_OF_ZONE
    assert G.classify(3, None, polys).status == G.UNKNOWN
    assert G.classify(3, (0, 0), {}).status == G.UNKNOWN
    assert G.classify(3, (0, 0), {1: polys[1]}).status == G.UNKNOWN          # expected zone not observed


def test_verify_uses_any_verifier_and_recovery_is_only_a_stub():
    class Fake:
        def polygons(self):
            return {3: square(-0.16, 0.14, 0.07)}

        def cube_xy(self, cube_id):
            return (-0.16, 0.30)

    check = G.verify(3, Fake())
    assert check.status == G.OUT_OF_ZONE and check.source == "Fake"
    with pytest.raises(NotImplementedError):
        G.recover_cube(3, check)
    assert G.recover_cube(3, G.classify(3, (-0.16, 0.14), {3: square(-0.16, 0.14, 0.07)}))["action"] == "none"
    with pytest.raises(NotImplementedError):
        G.ExternalCameraVerifier().polygons()
