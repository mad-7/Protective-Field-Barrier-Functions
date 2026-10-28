"""The declared route cap and the privileged oracle tracker (ruled 2026-10-20): known answers on
synthetic routes, before either is applied to a real run (hard rule 6)."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.control.route_cap import OracleTracker, RouteCapped, cap_twist
from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.planning.route_profile import (
    ProfileSpec,
    RouteProfile,
    anticipate,
    decelerate_backward,
    fastest_schedule,
    oracle_schedule,
    route_cap,
)
from pfbnav.safety.certified import FieldGeometry, band_speed_limit
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.oracle import Limits, fastest_path_time
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
PARAMS = certified_params_from_config(CFG)
FIELDS = FieldGeometry(FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).extent,
                       PARAMS)
SPEC = ProfileSpec.from_config(CFG, PARAMS)
LIMITS = [band_speed_limit(b, PARAMS) for b in range(len(PARAMS.band_top_speeds))]
LIM = Limits(v_max=0.5, vy_max=0.5, w_max=1.5, alpha_max=1.4, budget=0.80 * 0.617,
             lever_arm=0.228 + 0.158, decel=1.0)


def _box(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


# --------------------------------------------------------------------------- the pure pieces
def test_anticipate_takes_the_minimum_over_the_next_lead_metres():
    arc = np.array([0.0, 0.1, 0.2, 0.3, 0.4])
    v = np.array([5.0, 4.0, 1.0, 3.0, 2.0])
    assert anticipate(arc, v, 0.0).tolist() == v.tolist()
    assert anticipate(arc, v, 0.15).tolist() == [4.0, 1.0, 1.0, 2.0, 2.0]
    assert anticipate(arc, v, 10.0).tolist() == [1.0, 1.0, 1.0, 2.0, 2.0]


def test_the_backward_pass_lets_the_robot_brake_to_every_later_cap():
    arc = np.linspace(0.0, 1.0, 11)
    v = np.full(11, 0.5)
    v[-1] = 0.0
    out = decelerate_backward(arc, v, 0.5)
    for i in range(10):
        assert out[i] ** 2 <= out[i + 1] ** 2 + 2 * 0.5 * 0.1 + 1e-12
    assert out[-2] == pytest.approx(math.sqrt(0.1))     # sqrt(2 a ds)
    assert out[0] == 0.5                                 # far enough back to be unaffected


def test_cap_twist_never_speeds_a_command_up_and_keeps_its_direction():
    cmd = np.array([0.3, 0.4, 1.0])                      # 0.5 m/s
    out = cap_twist(cmd, 0.25, 0.2)
    assert math.hypot(out[0], out[1]) == pytest.approx(0.25)
    assert math.atan2(out[1], out[0]) == pytest.approx(math.atan2(0.4, 0.3))
    assert out[2] == 0.2
    slow = np.array([0.01, 0.0, -0.05])
    assert cap_twist(slow, 0.25, 0.2).tolist() == slow.tolist()
    assert cap_twist(np.array([0.0, 0.0, -3.0]), 0.0, 0.5).tolist() == [0.0, 0.0, -0.5]


def test_the_schedule_is_the_oracle_programme_step_for_step():
    """`fastest_schedule` repeats `fastest_path_time` (which may not be edited: A18's closure); on
    random tables, with in-place rotations forced by blocked states, the times agree exactly."""
    rng = np.random.default_rng(7)
    for _ in range(25):
        n, nj = 40, 8
        arc = np.cumsum(np.r_[0.0, rng.uniform(0.0, 0.05, n - 1)])
        yaws = np.linspace(-math.pi, math.pi, nj, endpoint=False)
        tangents = np.full(n, rng.uniform(-math.pi, math.pi))
        R = rng.random((n, nj)) > 0.25
        R[0, 0] = True
        caps = np.where(R, rng.choice(LIMITS, size=(n, nj)), 0.0)
        want = fastest_path_time(arc, tangents, yaws, R, caps, LIM)
        got = fastest_schedule(arc, tangents, yaws, R, caps, LIM)
        if math.isinf(want):
            assert got is None
        else:
            assert got is not None and got.time == pytest.approx(want, rel=1e-12)
            assert got.speed[0] == 0.0 and got.speed[-1] == 0.0
            assert all(R[i, got.depart[i]] and R[i, got.arrive[i]] for i in range(n))


# --------------------------------------------------------------------------- the cap on a route
def _corridor_with_a_pinch(pinch_gap):
    """A straight 6 m route along y = 0 between walls 0.80 m either side, with a pinch (posts
    `pinch_gap` m from the centreline, both sides) for 0.4 m at x = 3."""
    path = np.array([[0.0, 0.0], [6.0, 0.0]])
    walls = [_box(-1.0, 0.80, 7.0, 0.90), _box(-1.0, -0.90, 7.0, -0.80)]
    if pinch_gap < 0.80:
        walls += [_box(2.8, pinch_gap, 3.2, 0.80), _box(2.8, -0.80, 3.2, -pinch_gap)]
    return path, walls


def test_an_open_corridor_is_capped_at_the_top_band_speed_limit():
    path, walls = _corridor_with_a_pinch(0.80)          # no pinch at all
    p = route_cap(path, walls, FIELDS, PARAMS, SPEC)
    assert (p.band == 2).all()
    assert p.speed.max() == pytest.approx(LIMITS[2])
    assert p.length == pytest.approx(6.0)


@pytest.mark.parametrize(("gap", "band"), [(0.315, 1), (0.30, 0)])
def test_a_pinch_lowers_the_band_there_and_the_cap_falls_before_it(gap, band):
    """Abeam, the top band's field reaches 0.322 m, band 1's 0.309 and band 0's 0.299 (A21): posts
    0.315 m from the centreline leave band 1, posts at 0.30 only band 0."""
    path, walls = _corridor_with_a_pinch(gap)
    p = route_cap(path, walls, FIELDS, PARAMS, SPEC)
    at = p.index(3.0)
    assert p.band[at] == band and p.speed[at] == pytest.approx(LIMITS[band])
    # Anticipation: the cap is already down before the pinch begins (x = 2.8) ...
    assert p.speed[p.index(2.8 - 0.05)] < LIMITS[2]
    # ... and a robot at the cap can brake to every later cap at the declared deceleration.
    for i in range(p.arc.size - 1):
        assert p.speed[i] ** 2 <= p.speed[i + 1] ** 2 + 2 * SPEC.decel * (p.arc[i + 1] - p.arc[i])             + 1e-9
    # Far from the pinch the corridor is open again.
    assert p.band[p.index(0.5)] == 2 and p.band[p.index(5.5)] == 2


def test_the_cap_ignores_what_it_is_not_given():
    """Non-privileged: the cap is computed from the obstacles it is handed (the mapped ones); an
    obstacle left out changes nothing."""
    path, walls = _corridor_with_a_pinch(0.80)
    with_post = walls + [_box(3.0, -0.05, 3.1, 0.05)]  # an 'unmapped' post on the route
    assert route_cap(path, walls, FIELDS, PARAMS, SPEC).band.tolist() == \
        route_cap(path, walls, FIELDS, PARAMS, SPEC).band.tolist()
    blocked = route_cap(path, with_post, FIELDS, PARAMS, SPEC)
    assert (blocked.band[blocked.index(2.9):blocked.index(3.05)] == -1).all()


def test_a_profile_round_trips_through_json():
    path, walls = _corridor_with_a_pinch(0.34)
    p = route_cap(path, walls, FIELDS, PARAMS, SPEC)
    q = RouteProfile.from_dict(p.to_dict())
    assert q.kind == p.kind and np.array_equal(q.speed, p.speed) and np.array_equal(q.band, p.band)
    with pytest.raises(ValueError):
        RouteProfile.from_dict({**p.to_dict(), "version": 0})


def test_the_schedule_on_an_open_route_is_traversable_and_faces_along_it():
    path, walls = _corridor_with_a_pinch(0.80)
    s = oracle_schedule(path, walls, 0.0, FIELDS, PARAMS, LIM, SPEC, yaw_step=math.radians(15),
                        heading_round=math.radians(1.0), yaw_rates=[0.2, 0.1])
    assert s.traversable and s.yaw_offset is not None
    assert s.speed[0] == pytest.approx(0.0) and s.speed.max() <= LIMITS[2] + 1e-12


def test_a_route_through_an_obstacle_falls_back_and_says_so():
    path, walls = _corridor_with_a_pinch(0.80)
    s = oracle_schedule(path, walls + [_box(3.0, -0.05, 3.1, 0.05)], 0.0, FIELDS, PARAMS, LIM,
                        SPEC, yaw_step=math.radians(15), heading_round=math.radians(1.0),
                        yaw_rates=[0.2, 0.1])
    assert s.traversable is False and (s.yaw_offset == 0.0).all()


# --------------------------------------------------------------------------- the controllers
def _input(remaining, points, stop=False):
    scan = ScanStream.of(np.full(4, 10.0), np.linspace(-math.pi, math.pi, 4, endpoint=False),
                         np.ones(4, bool))
    pts = np.asarray(points, dtype=float)
    return ClassicalInput.of(stop_active=stop, velocity=np.zeros(3), scan=scan,
                             route=RouteView.of(pts, pts[-1], pts[-1], remaining), dt=0.05)


class _Fast:
    name, input_kind = "fast", "classical"
    ceilings = np.array([0.5, 0.5, 1.5])

    def reset(self):
        pass

    def command(self, inp):
        return np.array([0.5, 0.0, 1.5])


def _flat_profile(speed, yaw_rate, length=4.0):
    arc = np.linspace(0.0, length, 201)
    return RouteProfile(kind="cap", arc=arc, speed=np.full(arc.size, speed),
                        yaw_rate=np.full(arc.size, yaw_rate), band=np.ones(arc.size, int))


def test_a_capped_controller_goes_no_faster_than_the_profile_where_it_is():
    arc = np.linspace(0.0, 4.0, 401)
    speed = np.where(arc < 2.0, 0.40, 0.10)
    prof = RouteProfile(kind="cap", arc=arc, speed=speed, yaw_rate=np.full(arc.size, 0.3),
                        band=np.ones(arc.size, int))
    c = RouteCapped(_Fast(), lambda ep: prof)
    c.begin_episode({"plan_length_m": 4.0, "scenario_seed": 1})
    early = c.command(_input(3.5, [[0.0, 0.0], [1.0, 0.0]]))       # s = 0.5
    late = c.command(_input(1.0, [[0.0, 0.0], [1.0, 0.0]]))        # s = 3.0
    assert early.tolist() == pytest.approx([0.40, 0.0, 0.3])
    assert late.tolist() == pytest.approx([0.10, 0.0, 0.3])
    assert c.episode_report() == {"route_cap_steps": 2, "route_cap_capped_steps": 2}


def test_the_cap_passes_the_episode_and_the_report_through_to_its_base():
    """B6 seeds its sampler from the scenario: the identity must reach it through the cap, and its
    counts must reach the episode record (2026-10-27)."""
    class Seeded(_Fast):
        def begin_episode(self, episode):
            self.seen = dict(episode)

        def episode_report(self):
            return {"base_count": 7}

    base = Seeded()
    c = RouteCapped(base, lambda ep: _flat_profile(0.2, 0.3))
    c.begin_episode({"plan_length_m": 4.0, "scenario_seed": 11})
    assert base.seen == {"plan_length_m": 4.0, "scenario_seed": 11}
    assert c.episode_report() == {"base_count": 7, "route_cap_steps": 0,
                                  "route_cap_capped_steps": 0}


def test_a_profile_for_a_different_route_is_refused():
    c = RouteCapped(_Fast(), lambda ep: _flat_profile(0.2, 0.3, length=4.0))
    with pytest.raises(ValueError, match="different route"):
        c.begin_episode({"plan_length_m": 4.1, "scenario_seed": 1})
    c2 = RouteCapped(_Fast(), lambda ep: _flat_profile(0.2, 0.3))
    with pytest.raises(RuntimeError):
        c2.command(_input(3.0, [[0.0, 0.0], [1.0, 0.0]]))


def test_the_tracker_follows_the_route_and_returns_to_it():
    t = OracleTracker(lambda ep: _flat_profile(0.2, 1.0), ceilings=[0.5, 0.5, 1.5], k_cross=1.0,
                      k_yaw=2.0, yaw_tolerance=0.05, min_speed=0.018,
                      creep_yaw_rate=0.053)
    t.begin_episode({"plan_length_m": 4.0})
    # On the route, facing along it: straight ahead at the profile's speed, no turn.
    on = t.command(_input(3.0, [[0.0, 0.0], [1.0, 0.0]]))
    assert on.tolist() == pytest.approx([0.2, 0.0, 0.0])
    # 0.1 m to the right of the route (the route lies to the robot's LEFT): the command leans
    # left, no faster than the profile.
    off = t.command(_input(3.0, [[0.0, 0.1], [1.0, 0.1]]))
    assert off[1] > 0.0 and math.hypot(off[0], off[1]) <= 0.2 + 1e-12
    # Body turned +0.3 rad from the route (the route appears at -0.3 rad): it turns back.
    turned = t.command(_input(3.0, [[0.0, 0.0], [math.cos(-0.3), math.sin(-0.3)]]))
    assert turned[2] < 0.0


def test_the_tracker_rotates_in_place_where_the_schedule_stops_to_turn():
    arc = np.linspace(0.0, 4.0, 401)
    prof = RouteProfile(kind="schedule", arc=arc, speed=np.zeros(arc.size),
                        yaw_rate=np.full(arc.size, 0.5), band=np.zeros(arc.size, int),
                        yaw_offset=np.full(arc.size, math.pi / 2), traversable=True)
    t = OracleTracker(lambda ep: prof, ceilings=[0.5, 0.5, 1.5], k_cross=1.0, k_yaw=2.0,
                      yaw_tolerance=0.05, min_speed=0.018,
                      creep_yaw_rate=0.053)
    t.begin_episode({"plan_length_m": 4.0})
    cmd = t.command(_input(3.0, [[0.0, 0.0], [1.0, 0.0]]))
    assert cmd[:2].tolist() == [0.0, 0.0] and cmd[2] == pytest.approx(0.5)
    assert t.episode_report()["oracle_rotating_steps"] == 1


def test_a_stopped_tracker_asks_for_the_creep_regime():
    """Stopped, the tracker keeps to the creep regime -- at most the creep speed and creep yaw
    rate -- so its restart is judged on the at-rest field (found 2026-10-20)."""
    t = OracleTracker(lambda ep: _flat_profile(0.2, 1.0), ceilings=[0.5, 0.5, 1.5], k_cross=1.0,
                      k_yaw=2.0, yaw_tolerance=0.05, min_speed=0.018, creep_yaw_rate=0.053)
    t.begin_episode({"plan_length_m": 4.0})
    cmd = t.command(_input(3.0, [[0.0, 0.0], [math.cos(-0.3), math.sin(-0.3)]], stop=True))
    assert math.hypot(cmd[0], cmd[1]) <= 0.018 + 1e-12 and abs(cmd[2]) <= 0.053 + 1e-12
    assert t.episode_report()["oracle_stopped_steps"] == 1


@pytest.mark.parametrize(("gap", "allowance"), [(0.35, 0.2), (0.31, 0.15)])
def test_the_yaw_allowance_is_the_largest_ladder_rate_keeping_band_0_traversable(gap, allowance):
    """Ruled 2026-10-20 ("the largest yaw rate keeping band 0 admissible, rounded down"), per route:
    a pinch 0.31 m from the centreline admits band 0 swept at 0.15 rad/s but not at 0.20; an open
    one admits the top of the ladder. The ladder's order as given does not matter, and the tracker's
    yaw-rate cap is the allowance."""
    path, walls = _corridor_with_a_pinch(gap)
    s = oracle_schedule(path, walls, 0.0, FIELDS, PARAMS, LIM, SPEC, yaw_step=math.radians(15),
                        heading_round=math.radians(1.0), yaw_rates=[0.05, 0.2, 0.1, 0.15])
    assert s.traversable and s.yaw_allowance == allowance
    assert (s.yaw_rate == allowance).all() and s.schedule_time is not None
