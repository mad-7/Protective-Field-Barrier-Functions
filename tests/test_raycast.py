"""Known answers for the exact ray caster (hard rule 6)."""
import math

import numpy as np
import pytest

from pfbnav.eval.raycast import (
    bin_bearings,
    ray_hit,
    ray_ranges,
    rays_hit,
    sector_scan,
    to_body_frame,
)

SQUARE = np.array([[1.0, -1.0], [3.0, -1.0], [3.0, 1.0], [1.0, 1.0]])


def test_straight_ahead_and_oblique_hits():
    assert ray_hit(0.0, SQUARE) == pytest.approx(1.0)
    assert ray_hit(math.atan2(0.5, 1.0), SQUARE) == pytest.approx(math.hypot(1.0, 0.5))
    assert ray_hit(math.pi, SQUARE) == math.inf                    # behind
    assert ray_hit(math.pi / 2, SQUARE) == math.inf                # passes beside


def test_ranges_cap_at_range_max_and_take_the_nearest_polygon():
    near = SQUARE - [0.5, 0.0]
    r = ray_ranges([0.0, math.pi], [SQUARE, near], 5.0)
    assert r[0] == pytest.approx(0.5) and r[1] == 5.0


def test_bin_bearings_are_centred_and_ascending():
    b = bin_bearings(120)
    assert len(b) == 120 and np.all(np.diff(b) > 0)
    assert b[0] == pytest.approx(-math.pi + math.pi / 120)


def test_body_frame_of_a_rotated_pose():
    poly = to_body_frame([[1.0, 3.0]], (1.0, 1.0, math.pi / 2))
    assert np.allclose(poly, [[2.0, 0.0]])


def test_the_vectorised_caster_equals_the_exact_one():
    rng = np.random.default_rng(3)
    polys = [c + rng.uniform(-0.3, 0.3, (rng.integers(3, 7), 2)) for c in rng.uniform(-3, 3, (6, 2))]
    th = rng.uniform(-math.pi, math.pi, 400)
    assert rays_hit(th, polys, 5.0) == pytest.approx(ray_ranges(th, polys, 5.0), abs=1e-12)
    assert np.all(rays_hit(th, [], 5.0) == 5.0)


def test_the_sector_scan_reads_the_nearest_point_at_its_bearing():
    """A wall 1 m ahead, perpendicular: in the sector straddling straight ahead the nearest ray is
    the one closest to 0 rad, reported at that ray's bearing with range 1 / cos(bearing); sectors
    with no hit read range_max at their centre."""
    wall = np.array([[1.0, -10.0], [1.02, -10.0], [1.02, 10.0], [1.0, 10.0]])
    r, b = sector_scan(120, [wall], 5.0, 13)
    assert np.all(np.diff(b) > 0)
    k = int(np.argmin(np.abs(b)))
    width = 2 * math.pi / 120
    assert abs(b[k]) <= width / 26 + 1e-12 and r[k] == pytest.approx(1.0 / math.cos(b[k]))
    behind = np.cos(bin_bearings(120)) < 0
    assert np.all(r[behind] == 5.0) and np.allclose(b[behind], bin_bearings(120)[behind])


def test_the_sector_scan_sees_a_corner_between_bin_centres():
    """Positive control: a box corner pointing at the robot, placed midway between two bin centres,
    1 m out. One ray per centre misses the vertex and reads the faces behind it; the sector scan
    reports a range within one ray spacing of the vertex's 1.0 m."""
    width = 2 * math.pi / 120
    mid = bin_bearings(120)[60] + width / 2                         # a sector boundary
    v = np.array([math.cos(mid), math.sin(mid)])
    n = np.array([-v[1], v[0]])
    corner = np.array([v, v + 0.3 * (v + n), v + 0.6 * v, v + 0.3 * (v - n)])
    centre_rays = ray_ranges(bin_bearings(120), [corner], 5.0)
    r, _ = sector_scan(120, [corner], 5.0, 13)
    assert centre_rays.min() > 1.0 + 0.02                           # the vertex is unobserved
    assert r.min() == pytest.approx(1.0, abs=0.005)
