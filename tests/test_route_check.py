"""Known answers for the route-level field check (`pfbnav/world/route_check.py`, ruled 2026-10-13).

The field is the emulator's own (`FieldGeometry` from `configs/base.yaml`) at the 0.40 m/s band,
the traversal speed every stratum derives today. Each geometric answer is checked against the
field's support in the separating direction, and the whole test once more against an independent
reading -- points on an obstacle's boundary compared with `FieldGeometry.extent` along their
bearing, which is how the emulator itself decides a breach from a scan.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.route_check import FieldShape, clearance_to_field, inside_normals, sample_route
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
FIELDS = FieldGeometry(FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).extent,
                       certified_params_from_config(CFG))
BAND = 0.40
FIELD = FieldShape.moving(FIELDS, BAND)


def rect(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def h(angle):
    return float(FIELDS.support(np.array([angle]), BAND, 0.0, False)[0])


@pytest.mark.parametrize("delta", [-0.002, 0.002])
def test_a_wall_beside_a_straight_route_breaches_exactly_inside_the_lateral_reach(delta):
    w = h(math.pi / 2) + delta
    samples = sample_route([[-1.0, 0.0], [1.0, 0.0]], 0.02)
    gap = clearance_to_field(samples, [rect(-5.0, w, 5.0, w + 0.1)], FIELD)
    assert gap.min() == pytest.approx(delta, abs=1e-9)
    assert (gap.min() < 0) == (delta < 0)


@pytest.mark.parametrize("delta", [-0.002, 0.002])
def test_a_wall_ahead_of_the_routes_end_breaches_exactly_inside_the_depth(delta):
    d = h(0.0) + delta
    samples = sample_route([[-1.0, 0.0], [0.0, 0.0]], 0.02)
    gap = clearance_to_field(samples, [rect(d, -2.0, d + 0.1, 2.0)], FIELD)
    assert gap.min() == pytest.approx(delta, abs=1e-9)


def test_winding_does_not_matter():
    w = h(math.pi / 2) - 0.01
    samples = sample_route([[-1.0, 0.0], [1.0, 0.0]], 0.05)
    ccw = clearance_to_field(samples, [rect(-5.0, w, 5.0, w + 0.1)], FIELD)
    cw = clearance_to_field(samples, [rect(-5.0, w, 5.0, w + 0.1)[::-1]], FIELD)
    assert np.allclose(ccw, cw)


def test_far_obstacles_are_ignored():
    samples = sample_route([[0.0, 0.0], [1.0, 0.0]], 0.1)
    gap = clearance_to_field(samples, [rect(10.0, 10.0, 11.0, 11.0)], FIELD)
    assert np.isinf(gap).all()


# The extent along 7200 bearings (0.05 deg), computed once: the emulator's own boundary, read at
# the nearest bearing. Off by well under a millimetre at these ranges, inside the 2 mm band below.
EXTENT_GRID = FIELDS.extent(np.linspace(-math.pi, math.pi, 7200, endpoint=False), BAND, 0.0, False)


def _breached_by_scan(x, y, yaw, polygon, n=2000):
    """The emulator's reading: is any boundary point of the polygon inside the field extent
    along its own bearing? Dense boundary points stand in for the scan's returns."""
    v = np.asarray(polygon, dtype=float)
    pts = np.vstack([a + np.linspace(0, 1, n)[:, None] * (b - a)
                     for a, b in zip(v, np.roll(v, -1, axis=0))])
    rel = pts - [x, y]
    c, s = math.cos(-yaw), math.sin(-yaw)
    body = np.column_stack([c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]])
    r = np.hypot(body[:, 0], body[:, 1])
    bearing = np.arctan2(body[:, 1], body[:, 0])
    k = np.rint((bearing + math.pi) / (2 * math.pi) * EXTENT_GRID.size).astype(int)
    return bool(np.any(r < EXTENT_GRID[k % EXTENT_GRID.size]))


def test_the_separating_axis_test_agrees_with_the_scan_reading():
    """Independent cross-check over random shelf corners near random poses."""
    rng = np.random.default_rng(7)
    agree = disagree_near_tangency = 0
    for _ in range(200):
        yaw = rng.uniform(-math.pi, math.pi)
        samples = sample_route([[0.0, 0.0], [0.001 * math.cos(yaw), 0.001 * math.sin(yaw)]], 1.0)
        cx, cy = rng.uniform(-0.9, 0.9, 2)
        poly = rect(cx, cy, cx + rng.uniform(0.05, 0.6), cy + rng.uniform(0.05, 0.6))
        gap = float(clearance_to_field(samples, [poly], FIELD)[0])
        if abs(gap) < 0.002:                     # within the boundary sampling's resolution
            disagree_near_tangency += 1
            continue
        assert (gap < 0) == _breached_by_scan(0.0, 0.0, yaw, poly), (yaw, poly, gap)
        agree += 1
    assert agree > 150


def test_inside_normals_point_into_the_bends():
    left_turn = inside_normals([[0, 0], [1, 0], [1, 1]])
    assert left_turn.tolist() == [[0.0, 1.0], [-1.0, 0.0]]
    right_turn = inside_normals([[0, 0], [1, 0], [1, -1]])
    assert right_turn.tolist() == [[0.0, -1.0], [-1.0, 0.0]]
    straight = inside_normals([[0, 0], [1, 0], [2, 0]])
    assert not straight.any()
    # A collinear vertex is not a bend: the first segment looks past it to the real one.
    assert inside_normals([[0, 0], [1, 0], [2, 0], [2, 1]])[0].tolist() == [0.0, 1.0]


def test_samples_cover_every_segment_and_take_both_tangents_at_a_vertex():
    s = sample_route([[0, 0], [1, 0], [1, 1]], 0.25)
    corner = (np.isclose(s.x, 1.0) & np.isclose(s.y, 0.0))
    assert sorted(np.round(s.yaw[corner], 6).tolist()) == [0.0, round(math.pi / 2, 6)]
    steps = np.hypot(np.diff(s.x), np.diff(s.y))
    assert steps.max() <= 0.25 + 1e-12


def test_the_inside_offset_moves_toward_the_bend():
    s = sample_route([[0, 0], [1, 0], [1, 1]], 0.5).offset(0.05)
    assert np.allclose(s.y[:3], 0.05)          # first segment displaced to its left


def test_location_classes_name_start_goal_bend_and_straight():
    from pfbnav.world.route_check import location_classes
    s = sample_route([[0, 0], [4, 0], [4, 4]], 0.1)
    labels = location_classes(s, start_window=0.5, goal_window=1.0, bend_window=0.3)
    at = {(round(float(x), 2), round(float(y), 2)): lab for x, y, lab in zip(s.x, s.y, labels)}
    assert at[(0.0, 0.0)] == "start" and at[(0.5, 0.0)] == "start"
    assert at[(2.0, 0.0)] == "straight" and at[(3.8, 0.0)] == "bend" and at[(4.0, 0.2)] == "bend"
    assert at[(4.0, 3.0)] == "goal" and at[(4.0, 4.0)] == "goal" and at[(4.0, 2.0)] == "straight"


# --- the oracle tracker (criterion amended 2026-10-15) -------------------------------------------

def test_band_caps_take_the_fastest_clear_band():
    from pfbnav.world.route_check import band_caps
    clear = np.array([[True, True, True, False],      # band 0 (0.10)
                      [True, True, False, False],     # band 1 (0.30)
                      [True, False, False, False]])   # band 2 (0.40)
    assert band_caps(clear, [0.10, 0.30, 0.40]).tolist() == [0.40, 0.30, 0.10, 0.0]


def test_the_oracle_brakes_into_the_goal_at_the_deceleration_limit():
    from pfbnav.world.route_check import oracle_speeds
    s = np.linspace(0.0, 2.0, 201)
    v = oracle_speeds(s, np.full(s.size, 0.40), decel=1.0)
    assert v[-1] == 0.0
    expected = np.minimum(0.40, np.sqrt(2.0 * 1.0 * (2.0 - s)))
    assert np.allclose(v, expected)


def test_the_oracle_slows_ahead_of_a_low_cap_and_not_after_it():
    from pfbnav.world.route_check import oracle_speeds
    s = np.linspace(0.0, 4.0, 401)
    caps = np.where((s >= 2.0) & (s <= 2.5), 0.10, 0.40)
    v = oracle_speeds(s, caps, decel=1.0, v_goal=0.40)
    i = int(np.argmin(np.abs(s - 1.99)))            # 1 cm before the slow stretch
    assert v[i] == pytest.approx(math.sqrt(0.10 ** 2 + 2 * 1.0 * 0.01), abs=1e-9)
    assert np.all(v[(s >= 2.0) & (s <= 2.5)] <= 0.10 + 1e-12)
    assert v[int(np.argmin(np.abs(s - 2.6)))] == pytest.approx(0.40)   # no acceleration limit


def test_stop_runs_count_separate_breached_stretches():
    from pfbnav.world.route_check import stop_runs
    assert stop_runs([False, True, True, False, True]) == 2
    assert stop_runs([True, False]) == 1 and stop_runs([False, False]) == 0 and stop_runs([]) == 0


def test_traverse_time_known_answers():
    from pfbnav.world.route_check import traverse_time
    s = np.linspace(0.0, 2.0, 101)
    assert traverse_time(s, np.full(s.size, 0.40)) == pytest.approx(5.0)
    # Constant deceleration from 0.40 m/s to rest over 0.08 m takes 0.40 s.
    d = np.linspace(0.0, 0.08, 1001)
    v = np.sqrt(np.maximum(0.0, 0.40 ** 2 - 2 * 1.0 * d))
    assert traverse_time(d, v) == pytest.approx(0.40, rel=1e-3)
    assert math.isinf(traverse_time(s, np.where(np.abs(s - 1.0) < 0.05, 0.0, 0.4)))


def test_a_field_travelling_sideways_is_the_emulators_field_at_that_heading():
    """A holonomic base may travel at any heading to its body: the shape must be the emulator's own
    field for that heading, and travelling along the body's x axis must be the default."""
    side = FieldShape.moving(FIELDS, BAND, heading=math.pi / 2)
    assert np.allclose(side.support, FIELDS.support(FIELDS.normal_angles, BAND, math.pi / 2, False))
    assert np.allclose(FieldShape.moving(FIELDS, BAND, heading=0.0).support, FIELD.support)
    # Known answer: travelling +x with the body facing -y, a wall along the route on the -y side is
    # straight AHEAD OF THE BODY (abeam of the travel), so the separating direction is the body's
    # +x axis: it breaches exactly inside the support along body angle 0, at heading pi/2.
    s = sample_route([[-1.0, 0.0], [1.0, 0.0]], 0.02)
    body_turned = type(s)(s.x, s.y, s.yaw - math.pi / 2, s.inside)   # body faces -y, travels +x
    w = float(FIELDS.support(np.array([0.0]), BAND, math.pi / 2, False)[0])
    gap = clearance_to_field(body_turned, [rect(-5.0, -w - 0.1 - 0.002, 5.0, -w - 0.002)],
                             FieldShape.moving(FIELDS, BAND, heading=math.pi / 2))
    assert gap.min() > 0 and gap.min() == pytest.approx(0.002, abs=1e-6)


def test_the_coarse_first_test_decides_breaches_exactly_and_bounds_clearance_from_below():
    """`coarse_stride` must give the same breach decision as the full test, the same depth for
    every breach, and never more clearance than the full test for a clear pose."""
    rng = np.random.default_rng(11)
    for _ in range(60):
        yaw = rng.uniform(-math.pi, math.pi)
        n = 40
        s = sample_route([[0.0, 0.0], [math.cos(yaw), math.sin(yaw)]], 1.0 / n)
        cx, cy = rng.uniform(-0.8, 0.8, 2)
        poly = rect(cx, cy, cx + rng.uniform(0.05, 0.6), cy + rng.uniform(0.05, 0.6))
        full = clearance_to_field(s, [poly], FIELD)
        fast = clearance_to_field(s, [poly], FIELD, coarse_stride=8)
        assert np.array_equal(full < 0, fast < 0)
        assert np.allclose(fast[full < 0], full[full < 0])
        finite = np.isfinite(full) & (full >= 0)
        assert np.all(fast[finite] <= full[finite] + 1e-12) and np.all(fast[finite] >= 0)
