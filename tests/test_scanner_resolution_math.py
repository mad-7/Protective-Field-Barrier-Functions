"""Known-answer tests for the resolution / near-clipping experiment's evaluation (rule 6)."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.sim.depth_geometry import cast_to_wall, matrix_from_pose

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diag_scanner_resolution.py"
FOV = math.radians(120.0)
R_MAX = 5.0
NORMAL = 0.0                               # a wall ahead, along +x


def _load():
    spec = importlib.util.spec_from_file_location("diag_scanner_resolution", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


M = _load()
SENSORS = [matrix_from_pose(math.radians(a), [0.325, 0.0, 0.16]) for a in (60.0, -60.0)]


def captures(res, distance, *, scale=1.0):
    base, wall = [], []
    for m in SENSORS:
        depth = cast_to_wall(FOV, res, m, NORMAL, distance, mirrored=True,
                             euclidean=False, far=R_MAX)
        hit = depth < R_MAX
        depth = np.where(hit, depth * scale, depth)
        base.append({"depth": [R_MAX] * res, "matrix": m, "fov": FOV})
        wall.append({"depth": depth.tolist(), "matrix": m, "fov": FOV})
    return base, wall


def test_an_exact_capture_has_zero_error():
    base, wall = captures(80, 1.0)
    res, rng = M.face_residuals(base, wall, NORMAL, 1.0, R_MAX)
    assert res.size > 10
    assert np.abs(res).max() < 1e-9
    assert np.all(rng > 0.0)


def test_a_one_percent_long_depth_reads_as_ten_millimetres_per_metre():
    # Depth 1 % long moves each point 1 % further along its own ray from the sensor, so
    # the error per metre of range from that sensor is exactly the cos of the ray's
    # angle to the wall normal times 10 mm/m -- bounded above by 10 mm/m.
    base, wall = captures(256, 1.0, scale=1.01)
    res, rng = M.face_residuals(base, wall, NORMAL, 1.0, R_MAX)
    per_m = res / rng
    assert np.all(per_m > 0.0) and np.all(per_m <= 0.010 + 1e-12)


def test_resolution_mismatch_between_captures_is_refused():
    base, _ = captures(80, 1.0)
    _, wall = captures(256, 1.0)
    with pytest.raises(ValueError, match="resolution"):
        M.face_residuals(base, wall, NORMAL, 1.0, R_MAX)


def test_summarise_reports_the_tail_and_the_rate():
    res = np.array([0.001, 0.002, 0.003, 0.010])
    rng = np.array([1.0, 1.0, 1.0, 1.0])
    s = M.summarise(res, rng)
    assert s["pixels"] == 4
    assert s["median_mm"] == pytest.approx(2.5)
    assert s["max_abs_mm"] == pytest.approx(10.0)
    assert s["median_mm_per_m"] == pytest.approx(2.5)
    assert M.summarise(np.array([]), np.array([])) == {"pixels": 0}


# ----------------------------------------------------------------------
# The pre-declared rule
# ----------------------------------------------------------------------
def _table(f):
    return {f"{r}@{n}": {"median_mm_per_m": f(r, n)}
            for r in (80, 256, 512) for n in (0.01, 0.05, 0.1)}


def test_an_error_that_falls_with_resolution_is_sampling():
    v = M.verdict(_table(lambda r, n: 10.0 * 80 / r), (80, 256, 512), (0.01, 0.05, 0.1))
    assert v["cause"] == "sampling"
    assert v["best_setting"].startswith("512@")


def test_an_error_that_moves_with_near_clipping_is_depth_conversion():
    v = M.verdict(_table(lambda r, n: 10.0 * 0.01 / n), (80, 256, 512), (0.01, 0.05, 0.1))
    assert v["cause"] == "depth_conversion"


def test_an_error_neither_lever_moves_is_neither():
    v = M.verdict(_table(lambda r, n: 10.0 + 0.2 * (r / 512)), (80, 256, 512),
                  (0.01, 0.05, 0.1))
    assert v["cause"] == "neither"


def test_a_change_below_the_scatter_does_not_count_as_moving():
    # 1.2 -> 0.39 mm/m more than halves the error but moves it by only 0.81 mm/m, under
    # the ~1 mm/m scatter, so it is not a cause.
    v = M.verdict(_table(lambda r, n: 1.2 * (80 / r) ** 0.6), (80, 256, 512),
                  (0.01, 0.05, 0.1))
    assert v["cause"] == "neither"
