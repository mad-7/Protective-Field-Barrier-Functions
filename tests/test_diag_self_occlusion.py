"""Known-answer tests for the self-occlusion diagnostic's classification (hard rule 6)."""
import importlib.util
import math
from pathlib import Path

import numpy as np

from pfbnav.sim.depth_geometry import matrix_from_pose

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diag_self_occlusion.py"


def _load():
    spec = importlib.util.spec_from_file_location("diag_self_occlusion", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


M = _load()
FOV = math.radians(120.0)
RMAX = 5.0


def disc(r):
    return lambda b: np.full_like(np.asarray(b, dtype=float), r)


def run(depth, post_xy):
    # Sensor at the body origin looking along body +x; yaw_offset 0 so ref == body.
    return M.classify_returns(np.asarray(depth, dtype=float), FOV,
                              matrix_from_pose(0.0, [0.0, 0.0, 0.0]), 0.0,
                              mirrored=False, euclidean=False, extent=disc(0.3),
                              margin=0.005, range_max=RMAX, post_xy=post_xy)


def test_a_housing_in_every_pixel_leaves_nothing():
    r = run(np.full(64, 0.02), post_xy=(1.0, 0.0))
    assert r["returns"] == 64 and r["returns_below_housing"] == 64
    assert r["self_returns"] == 64 and r["surviving"] == 0
    assert r["post_pixels_surviving"] == 0 and r["nearest_surviving_m"] is None


def test_no_returns_at_range_max():
    r = run(np.full(64, RMAX), post_xy=(1.0, 0.0))
    assert r["returns"] == 0 and r["surviving"] == 0 and r["post_pixels_any"] == 0


def test_the_post_on_the_axis_survives():
    d = np.full(64, RMAX)
    d[31:33] = 1.0                      # the two centre pixels, perpendicular depth 1 m
    r = run(d, post_xy=(1.0, 0.0))
    assert r["returns"] == 2 and r["surviving"] == 2
    assert r["post_pixels_surviving"] == 2
    assert r["nearest_surviving_m"] < 1.01


def _row(sensor, render, post, any_, kept):
    return {"sensor": sensor, "render": render, "post": post,
            "post_pixels_any": any_, "post_pixels_surviving": kept}


def test_verdict_names_the_housing():
    rows = [_row(1, "all", True, 0, 0), _row(1, "no_own_model", True, 3, 3),
            _row(1, "no_robot", True, 3, 3)]
    v = M.verdict(rows)
    assert v["H1_housing"] and not v["H2_post_not_rendered"]
    assert v["proposed_fix_sees_post_on_every_sensor"]


def test_verdict_names_an_unrendered_post():
    rows = [_row(1, "all", True, 0, 0), _row(1, "no_own_model", True, 0, 0),
            _row(1, "no_robot", True, 0, 0)]
    v = M.verdict(rows)
    assert v["H2_post_not_rendered"] and not v["H1_housing"]
