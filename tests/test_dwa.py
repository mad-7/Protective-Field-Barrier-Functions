"""Known answers for B3, holonomic DWA (hard rule 6). The footprint is a circle of radius 0.25 m so
that every clearance is a number one can write down; the parameters are explicit, not the
configured (untuned) ones. The actuation numbers are the configured ones (a_max 0.5, alpha_max 1.4,
a_decel_max 1.0, the 0.5 / 0.5 / 1.5 box); the wheel budget is set out of reach unless a test is
about it."""
import math
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import bearings, box, make_input, ray_scan

from pfbnav.config import load_config
from pfbnav.control.base import ClassicalController
from pfbnav.control.dwa import (
    DWAParams,
    HolonomicDWA,
    cell_max_table,
    crop_route,
    project_route,
    rollout,
)
from pfbnav.envs.controller_input import ClassicalInput, RouteView

P = {"max_speed": 0.5, "window_time": 0.15, "sim_time": 1.5, "sim_granularity": 0.025,
     "angular_sim_granularity": 0.1, "vx_samples": 5, "vy_samples": 5, "wz_samples": 7,
     "safety_margin": 0.05, "clearance_cap": 0.3, "w_path": 1.0, "w_progress": 1.0,
     "w_heading": 0.3, "w_clearance": 1.0}
ACT = {"v_max": 0.5, "vy_max": 0.5, "w_max": 1.5, "a_max": 0.5, "alpha_max": 1.4,
       "a_decel_max": 1.0, "wheel_budget": 10.0, "lever_arm": 0.3865}
RADIUS = 0.25
DV = 0.5 * 0.15         # the window's translational reach, m/s
DW = 1.4 * 0.15         # its yaw reach, rad/s


def circle(psi):
    return np.full(np.shape(psi), RADIUS)


def dwa(act=None, **over):
    return HolonomicDWA(DWAParams(**{**P, **over}), circle, **{**ACT, **(act or {})})


def route_input(points, *, velocity=(0.0, 0.0, 0.0), scan=None):
    pts = np.asarray(points, dtype=float)
    seg = float(np.hypot(*np.diff(pts, axis=0).T).sum())
    base = make_input(scan=scan, velocity=velocity)
    return ClassicalInput.of(stop_active=False, velocity=velocity, scan=base.scan,
                             route=RouteView.of(pts, pts[1], pts[-1], seg), dt=0.05)


STRAIGHT = [[0.0, 0.0], [10.0, 0.0]]


def find(samples, vx, vy, wz):
    hit = np.flatnonzero(np.all(np.abs(samples - [vx, vy, wz]) < 1e-9, axis=1))
    assert len(hit) == 1, (vx, vy, wz)
    return int(hit[0])


# --- the window ---------------------------------------------------------------------------------

def test_the_window_at_rest_is_the_reach_disc_by_the_yaw_interval():
    """From rest: translations within a_max T = 0.075 m/s (a disc: the limiter bounds the change's
    magnitude), yaw rates within alpha_max T = 0.21 rad/s. A 5 x 5 grid keeps 13 points of the disc
    (offsets in units of 0.0375 with i^2 + j^2 <= 4), times 7 yaw rates."""
    s = dwa().window(np.zeros(3))
    assert len(s) == 13 * 7
    assert np.all(np.hypot(s[:, 0], s[:, 1]) <= DV + 1e-12)
    assert np.abs(s[:, 2]).max() == pytest.approx(DW)
    for v in ((DV, 0.0), (-DV, 0.0), (0.0, DV), (0.0, -DV)):
        assert np.any(np.all(np.abs(s[:, :2] - v) < 1e-12, axis=1)), v
    assert not np.any(np.all(np.abs(np.abs(s[:, :2]) - DV) < 1e-12, axis=1))   # no corners


def test_the_window_is_cut_by_the_box_and_max_speed_and_centres_on_the_estimate():
    s = dwa().window(np.array([0.47, 0.0, 0.3]))
    assert s[:, 0].max() == pytest.approx(0.5) and s[:, 0].min() == pytest.approx(0.395)
    assert np.all(np.hypot(s[:, 0], s[:, 1]) <= 0.5 + 1e-12)
    assert s[:, 2].min() == pytest.approx(0.3 - DW) and s[:, 2].max() == pytest.approx(0.3 + DW)
    s = dwa(max_speed=0.3).window(np.array([0.25, 0.2, 0.0]))
    assert len(s) and np.all(np.hypot(s[:, 0], s[:, 1]) <= 0.3 + 1e-12)


def test_the_window_keeps_inside_the_enforced_wheel_budget():
    s = dwa({"wheel_budget": 0.1}).window(np.zeros(3))
    use = np.abs(s[:, 0]) + np.abs(s[:, 1]) + 0.3865 * np.abs(s[:, 2])
    assert np.all(use <= 0.1 + 1e-12)
    find(s, DV, 0.0, 0.0)                                       # 0.075 <= 0.1: kept
    assert not np.any(np.all(np.abs(s - [DV, 0.0, DW]) < 1e-9, axis=1))   # 0.156: dropped


def test_the_states_are_as_many_as_the_granularities_need():
    """At rest the fastest sample moves 0.075 x 1.5 = 0.1125 m (4.5 granules of 0.025) and turns
    0.21 x 1.5 = 0.315 rad (3.15 of 0.1): five intervals. At 0.45 m/s the window reaches 0.5 m/s:
    0.75 m, thirty."""
    c = dwa()
    rest = c.state_times(c.window(np.zeros(3)))
    assert len(rest) == 6 and rest[-1] == pytest.approx(1.5)
    fast = c.window(np.array([0.45, 0.0, 0.0]))
    times = c.state_times(fast)
    assert len(times) == 31
    step = times[1] - times[0]
    assert np.hypot(fast[:, 0], fast[:, 1]).max() * step <= 0.025 + 1e-12
    assert np.abs(fast[:, 2]).max() * step <= 0.1 + 1e-12


# --- the geometry -------------------------------------------------------------------------------

def _rk4(twist, times, dt=1e-3):
    vx, vy, wz = twist

    def f(s):
        return np.array([vx * math.cos(s[2]) - vy * math.sin(s[2]),
                         vx * math.sin(s[2]) + vy * math.cos(s[2]), wz])

    out, s, t = [], np.zeros(3), 0.0
    for target in times:
        while t < target - 1e-12:
            h = min(dt, target - t)
            k1 = f(s)
            k2 = f(s + h / 2 * k1)
            k3 = f(s + h / 2 * k2)
            k4 = f(s + h * k3)
            s = s + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            t += h
        out.append(s.copy())
    return np.array(out)


def test_the_closed_form_rollout_matches_integration_of_the_body_twist():
    twists = np.array([[0.3, 0.1, 0.0], [0.3, -0.2, 0.7], [0.0, 0.0, 1.2], [0.2, 0.1, 1e-9],
                       [-0.1, 0.4, -1.5]])
    times = np.linspace(0.0, 1.5, 16)
    x, y, th = rollout(twists, times)
    for i, tw in enumerate(twists):
        ref = _rk4(tw, times)
        assert np.allclose(x[i], ref[:, 0], atol=1e-10)
        assert np.allclose(y[i], ref[:, 1], atol=1e-10)
        assert np.allclose(th[i], ref[:, 2], atol=1e-12)


def test_the_route_projection_known_answers():
    route = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    dist, arc, heading = project_route(route, np.array([[0.5, 0.2], [1.3, 0.5], [1.0, 1.5],
                                                        [-0.4, 0.3]]))
    assert dist.tolist() == pytest.approx([0.2, 0.3, 0.5, 0.5])
    assert arc.tolist() == pytest.approx([0.5, 1.5, 2.0, 0.0])
    assert heading.tolist() == pytest.approx([0.0, math.pi / 2, math.pi / 2, 0.0])
    assert np.allclose(crop_route(route, 1.25), [[0, 0], [1, 0], [1, 0.25]])
    assert np.allclose(crop_route(route, 5.0), route)
    assert np.allclose(crop_route(np.array([[0, 0], [0, 0], [2, 0]]), 1.0), [[0, 0], [1, 0]])


def test_the_cell_table_never_under_states_the_footprint_and_over_states_it_by_one_cell():
    def ellipse_ish(psi):
        return 0.3 + 0.05 * np.cos(2 * np.asarray(psi))

    c = HolonomicDWA(DWAParams(**P), ellipse_ish, **ACT)
    psi = np.random.default_rng(0).uniform(-3 * math.pi, 3 * math.pi, 20000)
    e = c.rollout_extent(psi)
    assert np.all(e >= ellipse_ish(psi) - 1e-15)
    assert np.all(e <= ellipse_ish(psi) + 0.1 * 2 * math.pi / 7200 + 1e-15)
    assert cell_max_table(ellipse_ish).max() == pytest.approx(0.35)


# --- scoring ------------------------------------------------------------------------------------

def test_on_a_clear_straight_route_from_rest_the_terms_and_the_choice():
    """Nothing within reach: no clearance cost. (0.075, 0, 0) ends 0.1125 m along the route, on it,
    aligned: total -0.1125, the least; (0, 0.075, 0) ends 0.1125 m off it at arc 0; a turning sample
    ends on the arc (x, y) = 0.075 (sin a, 1 - cos a) / 0.21, a = 0.315 rad."""
    c = dwa()
    ev = c.evaluate(route_input(STRAIGHT))
    s = ev.samples
    ahead, side, turn = find(s, DV, 0, 0), find(s, 0, DV, 0), find(s, DV, 0, DW)
    assert [ev.costs[k][ahead] for k in ("path", "progress", "heading", "clearance")] == \
        pytest.approx([0.0, 0.1125, 0.0, 0.0], abs=1e-12)
    assert ev.total[ahead] == pytest.approx(-0.1125)
    assert ev.costs["path"][side] == pytest.approx(0.1125)
    assert ev.costs["progress"][side] == pytest.approx(0.0, abs=1e-12)
    a = DW * 1.5
    x, y = DV * math.sin(a) / DW, DV * (1 - math.cos(a)) / DW
    assert ev.costs["path"][turn] == pytest.approx(y)
    assert ev.costs["progress"][turn] == pytest.approx(x)
    assert ev.costs["heading"][turn] == pytest.approx(a)
    assert ev.total[turn] == pytest.approx(y - x + 0.3 * a)
    assert ev.best == ahead
    assert c.command(route_input(STRAIGHT)).tolist() == pytest.approx([DV, 0.0, 0.0], abs=1e-12)


def test_at_speed_on_a_straight_route_it_takes_the_ceiling():
    cmd = dwa().command(route_input(STRAIGHT, velocity=(0.45, 0.0, 0.0)))
    assert cmd.tolist() == pytest.approx([0.5, 0.0, 0.0], abs=1e-12)


def test_the_clearance_term_known_answer():
    """A wall 0.5 m to the right: its nearest returns are the bins at -88.5 and -91.5 deg, at
    0.5 / cos(1.5 deg); standing still, the clearance is that minus 0.25, the cost 0.3 minus it."""
    scan = ray_scan([box(-3.0, -0.6, 3.0, -0.5)])
    ev = dwa().evaluate(route_input(STRAIGHT, scan=scan))
    still = int(np.argmin(np.abs(ev.samples).sum(axis=1)))
    assert np.abs(ev.samples[still]).max() < 1e-12
    gap = 0.5 / math.cos(math.radians(1.5)) - RADIUS
    assert ev.costs["clearance"][still] == pytest.approx(0.3 - gap)


def test_the_route_is_cropped_so_a_returning_leg_cannot_pull_the_projection_forward():
    """A hairpin: out 0.3 m along +x, back along y = 0.2. Uncropped, an end state at (0.1, 0.15)
    projects onto the return leg (0.05 m away) at arc 0.7; cropped to the outward leg, onto it at
    arc 0.1. (The configured crop, 2 x 0.5 x 1.5 = 1.5 m, would contain a hairpin this small; a
    0.3 m crop shows the rule.)"""
    route = np.array([[0.0, 0.0], [0.3, 0.0], [0.3, 0.2], [0.0, 0.2]])
    _, arc_full, _ = project_route(route, np.array([[0.1, 0.15]]))
    _, arc_crop, _ = project_route(crop_route(route, 0.3), np.array([[0.1, 0.15]]))
    assert arc_full[0] == pytest.approx(0.7) and arc_crop[0] == pytest.approx(0.1)


# --- obstacles, invalid bins, blocking ----------------------------------------------------------

def test_a_wall_ahead_admits_exactly_the_samples_that_stop_short_of_the_margin():
    """A wall 0.6 m ahead, estimate 0.25 m/s: a sample collides when it ends within 0.25 + 0.05 of
    the wall, i.e. beyond x = 0.30 at 1.5 s: vx > 0.2. The window's vx grid is 0.175 ... 0.325 in
    steps of 0.0375, so only vx = 0.175 survives (with vy = 0, the disc's edge), at every yaw rate."""
    scan = ray_scan([box(0.6, -3.0, 0.7, 3.0)])
    ev = dwa().evaluate(route_input(STRAIGHT, scan=scan, velocity=(0.25, 0.0, 0.0)))
    ok = ev.samples[ev.admissible]
    assert set(np.round(ok[:, 0], 6)) == {0.175}
    assert np.allclose(ok[:, 1], 0.0) and len(ok) == 7
    assert np.all(np.isinf(ev.total[~ev.admissible]))
    assert np.all(np.isnan(ev.costs["path"][~ev.admissible]))


def test_blocked_it_commands_zero_and_counts_the_step():
    """The same kind of wall 0.55 m ahead at 0.4 m/s: every sample in the window (vx >= 0.325)
    reaches it within the horizon. Zero is commanded (the limiter brakes at a_decel_max) and the
    step is counted, reported, and forgotten at reset."""
    scan = ray_scan([box(0.55, -3.0, 0.65, 3.0)])
    c = dwa()
    inp = route_input(STRAIGHT, scan=scan, velocity=(0.4, 0.0, 0.0))
    assert c.evaluate(inp).best is None
    assert c.command(inp).tolist() == [0.0, 0.0, 0.0]
    assert c.act(inp).tolist() == [0.0, 0.0, 0.0]
    assert c.episode_report() == {"dwa_steps": 2, "dwa_blocked_steps": 2}
    c.command(route_input(STRAIGHT))
    assert c.episode_report() == {"dwa_steps": 3, "dwa_blocked_steps": 2}
    c.reset()
    assert c.episode_report() == {"dwa_steps": 0, "dwa_blocked_steps": 0}


def test_an_invalid_bin_is_an_obstacle_at_the_footprint_boundary_never_free_space():
    """Bin 60 (+1.5 deg) unobserved on an otherwise empty scan, from rest on a straight route: the
    shared convention (ruled 2026-10-27) puts an obstacle at the footprint's edge on its bearing, so
    no admissible sample has any forward component, and the robot does not drive ahead as it does
    with the bin valid and empty (hard rule 5). (A purely lateral sample passes the point
    tangentially, dipping ~0.09 mm into it between rollout states: the rollout's resolution,
    sim_granularity, not a gap in the rule.)"""
    b = bearings()[60]
    assert b == pytest.approx(math.radians(1.5))
    c = dwa()
    ev = c.evaluate(route_input(STRAIGHT, scan=ray_scan([], invalid=[60])))
    ok = ev.samples[ev.admissible]
    assert len(ok) and np.all(ok[:, 0] <= 1e-12)
    assert not np.any(ev.admissible[ev.samples[:, 0] > 1e-12])
    cmd = c.command(route_input(STRAIGHT, scan=ray_scan([], invalid=[60])))
    assert cmd[0] <= 1e-12
    assert c.command(route_input(STRAIGHT)).tolist() == pytest.approx([DV, 0.0, 0.0], abs=1e-12)
    points, unobserved = c.obstacle_points(route_input(STRAIGHT, scan=ray_scan([], invalid=[60])))
    r = np.hypot(points[:, 0], points[:, 1])
    assert np.all(unobserved) and r.tolist() == pytest.approx([RADIUS])



def test_inside_the_margin_it_may_move_away_but_not_nearer():
    """A wall 0.27 m to the right: the nearest return is 0.0201 m from the footprint, inside the
    0.05 margin, so the returns' threshold is that clearance. Moving right (toward) is rejected;
    moving left (away) is admitted, so the robot is not frozen."""
    scan = ray_scan([box(-3.0, -0.4, 3.0, -0.27)])
    ev = dwa().evaluate(route_input(STRAIGHT, scan=scan))
    s = ev.samples
    assert not ev.admissible[find(s, 0.0, -DV, 0.0)]
    assert ev.admissible[find(s, 0.0, DV, 0.0)]
    assert ev.admissible[find(s, 0.0375, 0.0375, 0.0)]
    assert np.all(~ev.admissible[s[:, 1] < -1e-12])


def test_pruning_far_points_changes_no_decision():
    """Points beyond every state's reach plus the footprint plus max(margin, cap) are dropped; the
    evaluation with every point kept is identical."""

    class Unpruned(HolonomicDWA):
        def obstacle_points(self, inp, reach=None):
            return super().obstacle_points(inp, 100.0)

    rng = np.random.default_rng(1)
    for _ in range(6):
        boxes = []
        for _ in range(4):
            x, y = rng.uniform(-2.0, 2.0, 2)
            if math.hypot(x, y) < 0.7:
                continue
            boxes.append(box(x, y, x + rng.uniform(0.05, 0.5), y + rng.uniform(0.05, 0.5)))
        inp = route_input(STRAIGHT, scan=ray_scan(boxes, invalid=[int(rng.integers(120))]),
                          velocity=(rng.uniform(0.0, 0.4), rng.uniform(-0.1, 0.1), 0.0))
        a = dwa().evaluate(inp)
        b = Unpruned(DWAParams(**P), circle, **ACT).evaluate(inp)
        assert np.array_equal(a.admissible, b.admissible)
        assert np.array_equal(a.total, b.total)


# --- construction and refusals ------------------------------------------------------------------

def test_it_refuses_impossible_parameters_and_the_untyped_input():
    with pytest.raises(ValueError, match="max_speed"):
        dwa(max_speed=0.6)
    with pytest.raises(ValueError, match="sim_time"):
        dwa(sim_time=0.5)                       # below w_max / 2 alpha_max = 0.536 s
    with pytest.raises(ValueError, match="vx_samples"):
        dwa(vx_samples=1)
    with pytest.raises(ValueError, match="w_progress"):
        dwa(w_progress=0.0)
    with pytest.raises(ValueError, match="safety_margin"):
        dwa(safety_margin=-0.01)
    with pytest.raises(ValueError, match="integer"):
        DWAParams.from_block({**P, "wz_samples": 7.5})
    assert DWAParams.from_block({k: float(v) for k, v in P.items()}) == DWAParams(**P)
    with pytest.raises(TypeError, match="ClassicalInput"):
        dwa().command(np.zeros(82))


def test_the_configured_b3_is_a_classical_controller_under_the_cap_and_the_recovery():
    from pfbnav.control.recovery import Recovery
    from pfbnav.control.route_cap import RouteCapped

    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    b3 = HolonomicDWA.from_config(cfg)
    assert isinstance(b3, ClassicalController) and b3.name == "B3_holonomic_dwa"
    assert b3.ceilings.tolist() == [0.5, 0.5, 1.5]
    assert b3.wheel_budget == pytest.approx(0.95 * 0.617)
    capped = RouteCapped(b3, lambda _episode: None)
    wrapped = Recovery.from_config(capped, cfg)
    assert wrapped.name == b3.name
    act = b3.act(make_input())
    assert act.shape == (3,) and np.all(np.abs(act) <= 1.0)
    # The blocked count reaches the episode record through the cap and the recovery.
    report = wrapped.episode_report()
    assert report["dwa_steps"] == 1 and report["dwa_blocked_steps"] == 0
    assert report["route_cap_steps"] == 0
