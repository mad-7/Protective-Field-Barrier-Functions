"""Known answers for the free-orientation oracle (`pfbnav/world/oracle.py`, ruled 2026-10-15)."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.oracle import (
    Limits,
    closure,
    fastest_path_time,
    reach,
    rotation_time,
    state_tables,
    turning_speed_limit,
    yaw_grid,
)
from pfbnav.world.route_check import FieldShape, clearance_to_field, sample_route
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
LIM = Limits(v_max=0.40, vy_max=0.50, w_max=1.5, alpha_max=3.0, budget=0.80 * 0.617,
             lever_arm=0.228 + 0.1585, decel=1.0)


def test_the_yaw_grid_starts_at_the_start_yaw_and_closes_the_turn():
    g = yaw_grid(0.3, math.radians(15))
    assert g.size == 24 and g[0] == pytest.approx(0.3)
    assert np.allclose(np.diff(np.unwrap(g)), math.radians(15))
    with pytest.raises(ValueError):
        yaw_grid(0.0, math.radians(7))


def test_rotating_in_place_stays_inside_a_clear_run_and_wraps_round():
    clear = np.array([1, 1, 0, 1, 1, 1, 0, 1], dtype=bool)
    seed = np.zeros(8, dtype=bool)
    seed[4] = True
    assert closure(clear, seed).tolist() == [0, 0, 0, 1, 1, 1, 0, 0]
    seed = np.zeros(8, dtype=bool)
    seed[0] = True                                   # the run 7, 0, 1 wraps round
    assert closure(clear, seed).tolist() == [1, 1, 0, 0, 0, 0, 0, 1]
    seed = np.zeros(8, dtype=bool)
    seed[2] = True                                   # a blocked seed reaches nothing
    assert not closure(clear, seed).any()


def test_a_route_blocked_at_the_tangent_yaw_is_traversable_turned():
    c = np.ones((6, 8), dtype=bool)
    c[2:4, 0] = False                                # the tangent-locked oracle would stop here
    r = reach(c)
    assert r.traversable and r.stops == 0 and r.start_valid and r.goal_valid


def test_continuity_forbids_jumping_across_a_blocked_orientation():
    c = np.zeros((4, 8), dtype=bool)
    c[:2, 0] = True                                  # reached: yaw 0 only
    c[2, 4] = True                                   # next: only yaw 4, cut off by blocked 1-3
    c[3, 4] = True
    r = reach(c)
    assert not r.traversable and r.stops == 1 and r.first_death == 2


def test_stops_count_separate_dead_stretches_and_validity_is_reported():
    c = np.ones((9, 4), dtype=bool)
    c[2:4, :] = False                                # stretch one
    c[6, :] = False                                  # stretch two
    r = reach(c)
    assert r.stops == 2 and r.first_death == 2 and r.deaths == (2, 6) and not r.traversable
    c = np.ones((3, 4), dtype=bool)
    c[0, 0] = False
    assert not reach(c).start_valid
    c = np.ones((3, 4), dtype=bool)
    c[-1, :] = False
    out = reach(c)
    assert not out.goal_valid and out.stops == 1


def test_speed_limits_known_answers():
    assert turning_speed_limit(0.0, 0.0, LIM, 0.02) == pytest.approx(0.40)       # the box
    diag = turning_speed_limit(math.pi / 4, 0.0, LIM, 0.02)                     # the budget
    assert diag == pytest.approx(LIM.budget / math.sqrt(2.0))
    k = math.radians(15) / 0.02
    turning = turning_speed_limit(0.0, k, LIM, 0.02)
    expected = min(LIM.budget / (1.0 + LIM.lever_arm * k), 0.40, LIM.w_max / k,
                   math.sqrt(LIM.alpha_max * 0.02 / k))
    assert turning == pytest.approx(expected)
    assert turning < 0.10            # turning a grid step every 2 cm forces band 0


def test_rotation_time_known_answers():
    w = min(LIM.w_max, LIM.budget / LIM.lever_arm)                 # the budget binds: 1.28 rad/s
    assert w < LIM.w_max
    big = math.pi / 2
    assert rotation_time(big, LIM) == pytest.approx(big / w + w / LIM.alpha_max)
    small = 0.1                                                    # never reaches w: triangle
    assert rotation_time(small, LIM) == pytest.approx(2.0 * math.sqrt(small / LIM.alpha_max))
    assert rotation_time(0.0, LIM) == 0.0


def _straight(n=201, length=2.0):
    s = np.linspace(0.0, length, n)
    return s, np.zeros(n)


def test_a_straight_route_at_one_yaw_takes_the_braking_profile_time():
    s, phi = _straight()
    yaws = yaw_grid(0.0, math.radians(45))
    R = np.zeros((s.size, yaws.size), dtype=bool)
    R[:, 0] = True
    caps = np.where(R, 0.40, 0.0)
    v = np.minimum(0.40, np.sqrt(2.0 * (s[-1] - s)))
    v[0] = 0.0
    expected = float(np.sum(2.0 * np.diff(s) / (v[:-1] + v[1:])))
    assert fastest_path_time(s, phi, yaws, R, caps, LIM) == pytest.approx(expected)


def test_an_in_place_rotation_costs_its_time_and_needs_clear_orientations_on_the_way():
    s, phi = _straight()
    yaws = yaw_grid(0.0, math.radians(45))          # 8 orientations
    R = np.zeros((s.size, yaws.size), dtype=bool)
    R[:100, 0] = True                               # yaw 0 until half way
    R[99:, 2] = True                                # yaw 90 deg from just before half way
    R[99, 1] = True                                 # 45 deg passable at the turn point
    caps = np.where(R, 0.40, 0.0)
    t = fastest_path_time(s, phi, yaws, R, caps, LIM)
    straight = fastest_path_time(s, phi, yaws, np.where(np.arange(8) == 0, True, False)[None, :]
                                 .repeat(s.size, 0), np.full((s.size, 8), 0.40), LIM)
    # The search may turn while crawling (one grid step per sample) or stop and rotate in place:
    # whichever is faster. Either way the turn costs time, and never more than stopping to rotate
    # (the in-place option is always available here, so it bounds the answer from above).
    assert math.isfinite(t) and t > straight
    assert t <= straight + rotation_time(math.pi / 2, LIM) + 2.0 * math.sqrt(0.01 / LIM.decel)
    R[99, 1] = False                                # the way round is blocked: no path
    assert math.isinf(fastest_path_time(s, phi, yaws, R, caps, LIM))


def test_state_tables_at_the_tangent_yaw_match_the_tangent_locked_check():
    cfg = load_config(REPO / "configs" / "base.yaml")
    fields = FieldGeometry(FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"]).extent,
                           certified_params_from_config(cfg))
    samples = sample_route([[0.0, 0.0], [1.0, 0.0]], 0.05)
    wall = [np.array([[-2.0, 0.27], [3.0, 0.27], [3.0, 0.4], [-2.0, 0.4]])]
    yaws = yaw_grid(0.0, math.radians(30))           # index 0 = the tangent
    cache: dict = {}

    def shape_for(heading):
        return cache.setdefault(heading, FieldShape.moving(fields, 0.10, heading=heading))

    table = state_tables(samples, wall, yaws, shape_for)
    locked = clearance_to_field(samples, wall, FieldShape.moving(fields, 0.10))
    assert np.allclose(table[:, 0], locked)
    # Known answer: turned +90 deg (index 3) the body's FRONT (body angle 0) faces the wall instead
    # of its left side, travel still along the wall (no lead toward it), so the clearance drops by
    # exactly the footprint's support difference h(0) - h(pi/2): the long axis now faces the wall.
    # Since A20 the footprint term is the footprint swept through the yaw interval reachable from
    # rest, so the difference is that swept support's.
    from pfbnav.safety.certified import yaw_interval
    # A21: the moving field's yaw interval carries e(0) more, +-2 e(0) at zero yaw rate.
    lo, hi = yaw_interval(0.0, fields.params)
    h = fields.swept_interval(np.array([0.0, math.pi / 2]), 2.0 * lo, 2.0 * hi)
    assert np.allclose(table[:, 3], table[:, 0] - (h[0] - h[1]), atol=1e-9)


# --- the orientation set includes the route's own directions (found 2026-10-15) -------------------

def test_the_orientation_set_holds_the_start_and_every_route_direction():
    from pfbnav.world.oracle import yaw_set
    yaws, start = yaw_set(0.2, math.radians(15), extra=[0.0, math.pi / 2, 0.2 + math.radians(15)])
    assert yaws[start] == pytest.approx(0.2)
    assert np.any(np.isclose(yaws, 0.0)) and np.any(np.isclose(yaws, math.pi / 2))
    assert np.all(np.diff(yaws) > 0) and yaws.size == 24 + 2          # a duplicate is not added


def test_a_tight_straight_clear_only_along_the_route_is_traversable_with_its_direction():
    """The artefact of the first free run, reproduced: a corridor 2 mm wider than the band-0
    field's lateral reach at the route's own direction, entered from open floor with the body at
    11.5 deg. A grid anchored at the start yaw (nearest orientation 3.5 deg off the route) breaches
    in the corridor; with the route's direction in the set the robot turns in the open and passes."""
    from pfbnav.world.oracle import reach, yaw_set
    cfg = load_config(REPO / "configs" / "base.yaml")
    fields = FieldGeometry(FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"]).extent,
                           certified_params_from_config(cfg))
    w = float(fields.support(np.array([math.pi / 2]), 0.10, 0.0, False)[0]) + 0.002
    walls = [np.array([[1.0, w], [3.0, w], [3.0, w + 0.1], [1.0, w + 0.1]]),
             np.array([[1.0, -w - 0.1], [3.0, -w - 0.1], [3.0, -w], [1.0, -w]])]
    samples = sample_route([[0.0, 0.0], [4.0, 0.0]], 0.02)
    cache: dict = {}

    def shape_for(h):
        return cache.setdefault(h, FieldShape.moving(fields, 0.10, heading=h))

    start_yaw = 0.2
    grid = yaw_grid(start_yaw, math.radians(15))
    assert not reach(state_tables(samples, walls, grid, shape_for) >= 0).traversable
    yaws, start = yaw_set(start_yaw, math.radians(15), extra=[0.0])
    r = reach(state_tables(samples, walls, yaws, shape_for) >= 0, start_index=start)
    assert r.start_valid and r.traversable


def test_mission_time_on_an_uneven_orientation_set_uses_the_real_angles():
    from pfbnav.world.oracle import yaw_set
    s, phi = _straight()
    yaws, start = yaw_set(0.0, math.radians(90), extra=[math.radians(30)])   # 0, 30, 90, 180, -90
    j30 = int(np.argmin(np.abs(yaws - math.radians(30))))
    R = np.zeros((s.size, yaws.size), dtype=bool)
    R[:100, start] = True
    R[99:, j30] = True                                # at sample 99 rotate 30 deg in place
    caps = np.where(R, 0.40, 0.0)
    t = fastest_path_time(s, phi, yaws, R, caps, LIM, start_index=start)
    straight_caps = np.zeros_like(caps)
    straight_caps[:, start] = 0.40
    R0 = np.zeros_like(R)
    R0[:, start] = True
    t0 = fastest_path_time(s, phi, yaws, R0, straight_caps, LIM, start_index=start)
    # It either turns one 30-deg step while crawling or stops and rotates 30 deg in place; both are
    # bounded by stopping to rotate 30 deg (not 90, the uniform step this set no longer has).
    assert t0 < t <= t0 + rotation_time(math.radians(30), LIM) + 2.0 * math.sqrt(0.01 / LIM.decel)


# --- the generator's validity rules ---------------------------------------------------------------

def _validity(path, walls, start_yaw):
    from pfbnav.world.oracle import route_validity
    cfg = load_config(REPO / "configs" / "base.yaml")
    fields = FieldGeometry(FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"]).extent,
                           certified_params_from_config(cfg))
    cache: dict = {}

    def shape_for(h):
        return cache.setdefault(h, FieldShape.moving(fields, 0.10, heading=h))

    return route_validity(path, walls, start_yaw, shape_for, sample_spacing=0.02,
                          yaw_step=math.radians(15), heading_round=math.radians(1.0),
                          coarse_stride=16), fields


def _box(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def test_validity_classes_known_answers():
    from pfbnav.world.oracle import GOAL_INVALID, RESIDUAL, ROUTE_OK, START_INVALID
    path = [[0.0, 0.0], [4.0, 0.0]]
    assert _validity(path, [_box(0.0, 3.0, 4.0, 3.5)], 0.0)[0] == ROUTE_OK
    # A wall 0.05 m ahead of the start, across the route's initial direction: breached at the start.
    assert _validity(path, [_box(0.30, -0.5, 0.35, 0.5)], 0.0)[0] == START_INVALID
    # A wall just beyond the goal, across the approach, closer than any orientation's band-0 reach
    # ahead (the support ahead is at least the footprint's smallest extent + margin): goal invalid.
    assert _validity(path, [_box(4.10, -1.0, 4.20, 1.0)], 0.0)[0] == GOAL_INVALID
    # A gap mid-route narrower than the band-0 field abeam at EVERY orientation: residual.
    _status, fields = _validity(path, [], 0.0)
    narrowest = float(np.min(fields.support(fields.normal_angles, 0.10, 0.0, False)))
    w = narrowest - 0.01
    walls = [_box(1.8, w, 2.2, w + 0.1), _box(1.8, -w - 0.1, 2.2, -w)]
    assert _validity(path, walls, 0.0)[0] == RESIDUAL


# --- the rotation sweep of tangent tracking (ruled 2026-10-16) -------------------------------------
def test_tracking_yaw_rates_known_answers():
    """speed x (the tangent's turn over the window) / window: zero on straights, a quarter turn
    spread over the window near the vertex."""
    from pfbnav.world.oracle import tracking_yaw_rates
    path = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 3.0]])
    s = sample_route(path, 0.02)
    w = tracking_yaw_rates(s, window=0.8, speed=0.10)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(s.x), np.diff(s.y)))])
    near = np.abs(arc - 3.0) < 0.39
    far = np.abs(arc - 3.0) > 0.41
    assert np.allclose(w[near], 0.10 * (math.pi / 2) / 0.8)
    assert np.allclose(w[far], 0.0)
    straight = sample_route(np.array([[0.0, 0.0], [5.0, 0.0]]), 0.02)
    assert np.allclose(tracking_yaw_rates(straight, 0.8, 0.10), 0.0)


def test_rate_levels_round_up():
    from pfbnav.world.oracle import rate_levels
    assert np.allclose(rate_levels([0.0, 0.01, 0.05, 0.051, 0.19], 0.05),
                       [0.0, 0.05, 0.05, 0.10, 0.20])


def test_the_rotation_sweep_only_ever_shrinks_a_states_clearance():
    """The swept field contains the rotation-blind one, so every state's clearance with the
    tracking yaw rates is at most its clearance without; at a bend some state is strictly
    tighter, so the sweep is really applied (the positive control)."""
    from pfbnav.world.oracle import tracking_yaw_rates, yaw_set
    cfg = load_config(REPO / "configs" / "base.yaml")
    fields = FieldGeometry(FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"]).extent,
                           certified_params_from_config(cfg))
    band0 = certified_params_from_config(cfg).band_top_speeds[0]
    cache = {}

    def shape_for(heading, yaw_rate=0.0):
        key = (round(heading, 9), round(yaw_rate, 9))
        if key not in cache:
            cache[key] = FieldShape.moving(fields, band0, heading=heading, yaw_rate=yaw_rate)
        return cache[key]

    path = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])
    samples = sample_route(path, 0.02)
    corner = [np.array([[2.32, -0.6], [2.6, -0.6], [2.6, -0.32], [2.32, -0.32]])]
    yaws, _ = yaw_set(0.0, math.radians(15), extra=np.unique(np.round(samples.yaw, 12)))
    blind = state_tables(samples, corner, yaws, shape_for)
    rates = tracking_yaw_rates(samples, 0.8, 1.5)             # a large rate, to see it act
    swept = state_tables(samples, corner, yaws, shape_for, yaw_rates=rates, rate_step=0.05)
    assert np.all(swept <= blind + 1e-12)
    assert np.any(swept < blind - 1e-4)
