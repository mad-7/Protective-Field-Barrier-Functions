"""Known answers for B2, holonomic Regulated Pure Pursuit (hard rule 6). The footprint is a circle of
radius 0.25 m so that every clearance is a number one can write down; the parameters are explicit,
not the configured (untuned) ones."""
import math
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import box, make_input, ray_scan

from pfbnav.config import load_config
from pfbnav.control.base import ClassicalController
from pfbnav.control.regulated_pure_pursuit import RegulatedPurePursuit, RPPParams
from pfbnav.envs.controller_input import ClassicalInput, RouteView

P = {"desired_speed": 0.5, "lookahead_time": 1.5, "min_lookahead": 0.3, "max_lookahead": 0.9,
     "min_radius": 0.9, "proximity_distance": 0.6, "approach_distance": 0.6, "min_speed": 0.05,
     "min_approach_speed": 0.05, "k_omega": 1.5, "proximity_gain": 1.0}
RADIUS = 0.25


def circle(psi):
    return np.full(np.shape(psi), RADIUS)


def rpp(**over):
    return RegulatedPurePursuit(RPPParams(**{**P, **over}), circle, v_max=0.5, vy_max=0.5,
                                w_max=1.5)


def route_input(points, *, velocity=(0.0, 0.0, 0.0), scan=None, remaining=None):
    pts = np.asarray(points, dtype=float)
    seg = np.hypot(*np.diff(pts, axis=0).T).sum()
    base = make_input(scan=scan, velocity=velocity)
    return ClassicalInput.of(stop_active=False, velocity=velocity, scan=base.scan,
                             route=RouteView.of(pts, pts[1], pts[-1],
                                                float(seg if remaining is None else remaining)),
                             dt=0.05)


def test_on_a_clear_straight_route_it_drives_at_the_desired_speed_toward_the_lookahead():
    c = rpp().command(route_input([[0, 0], [10, 0]]))
    assert c.tolist() == pytest.approx([0.5, 0.0, 0.0])


def test_the_lookahead_grows_with_the_estimated_speed_and_is_clipped():
    r = rpp()
    assert r.lookahead_distance(route_input([[0, 0], [10, 0]])) == pytest.approx(0.3)
    assert r.lookahead_distance(route_input([[0, 0], [10, 0]], velocity=(0.4, 0.0, 0.0))) == \
        pytest.approx(0.6)
    assert r.lookahead_distance(route_input([[0, 0], [10, 0]], velocity=(0.0, 1.0, 0.0))) == \
        pytest.approx(0.9)


def test_curvature_regulation_known_answer():
    """A 90 deg turn 0.3 m ahead; at 0.4 m/s the lookahead is 0.6 m, the arc turns pi/2 over it:
    R = 0.6 / (pi / 2) = 0.382 m, factor R / 0.9, speed 0.5 x 0.4244 = 0.2122 m/s, toward the
    lookahead point (0.3, 0.3)."""
    inp = route_input([[0, 0], [0.3, 0], [0.3, 5.0]], velocity=(0.4, 0.0, 0.0))
    c = rpp().command(inp)
    expected = 0.5 * (0.6 / (math.pi / 2)) / 0.9
    assert math.hypot(c[0], c[1]) == pytest.approx(expected)
    assert math.atan2(c[1], c[0]) == pytest.approx(math.pi / 4)
    assert c[2] == pytest.approx(1.5 * math.pi / 4)


def test_proximity_regulation_known_answer():
    """A wall 0.55 m ahead: the nearest return is the bin straddling 0 deg, at 0.55 / cos(1.5 deg);
    its gap to the 0.25 m footprint is 0.3002 m, half the proximity distance: half speed."""
    scan = ray_scan([box(0.55, -3.0, 0.6, 3.0)])
    inp = route_input([[0, 0], [10, 0]], scan=scan)
    r = rpp()
    gap = 0.55 / math.cos(math.radians(1.5)) - RADIUS
    assert r.clearance(inp) == pytest.approx(gap)
    assert r.command(inp)[0] == pytest.approx(0.5 * gap / 0.6)


def test_the_proximity_gain_known_answer():
    """Nav2's cost_scaling_gain: below the distance the factor is gain x d / D; at and beyond it, 1."""
    r = rpp(proximity_gain=0.5)
    assert r.proximity_factor(0.3) == pytest.approx(0.5 * 0.3 / 0.6)
    assert r.proximity_factor(0.6) == 1.0 and r.proximity_factor(2.0) == 1.0
    assert r.proximity_factor(0.0) == 0.0
    scan = ray_scan([box(0.55, -3.0, 0.6, 3.0)])
    inp = route_input([[0, 0], [10, 0]], scan=scan)
    gap = 0.55 / math.cos(math.radians(1.5)) - RADIUS
    assert r.command(inp)[0] == pytest.approx(0.5 * 0.5 * gap / 0.6)
    for bad in (0.0, 1.2):
        with pytest.raises(ValueError, match="proximity_gain"):
            rpp(proximity_gain=bad)


def test_an_unobserved_bin_is_no_clearance_and_the_speed_falls_to_its_floor():
    scan = ray_scan([], invalid=[40])
    inp = route_input([[0, 0], [10, 0]], scan=scan)
    r = rpp()
    assert r.clearance(inp) == 0.0
    assert r.command(inp)[0] == pytest.approx(0.05)


def test_the_approach_slows_it_to_the_goal_and_never_below_its_floor():
    r = rpp()
    assert r.command(route_input([[0, 0], [0.3, 0]]))[0] == pytest.approx(0.25)
    assert r.command(route_input([[0, 0], [0.3, 0]], remaining=0.0))[0] == pytest.approx(0.05)


def test_the_least_factor_governs_and_the_floor_applies_to_curvature_and_proximity():
    """Proximity 0.1 (gap 0.06 m) under a gentle route: 0.5 x 0.1 = 0.05, at the floor; a smaller
    gap still gives the floor, not less."""
    for wall in (0.31, 0.27):
        scan = ray_scan([box(wall, -3.0, wall + 0.05, 3.0)])
        assert rpp().command(route_input([[0, 0], [10, 0]], scan=scan))[0] == \
            pytest.approx(0.05, abs=2e-3)


def test_it_refuses_impossible_parameters_and_the_untyped_input():
    with pytest.raises(ValueError):
        rpp(min_lookahead=1.0, max_lookahead=0.5)
    with pytest.raises(ValueError):
        rpp(min_speed=0.6)
    with pytest.raises(ValueError):
        RegulatedPurePursuit(RPPParams(**{**P, "desired_speed": 0.6}), circle, v_max=0.5,
                             vy_max=0.5, w_max=1.5)
    with pytest.raises(TypeError, match="ClassicalInput"):
        rpp().command(np.zeros(82))


def test_the_configured_b2_is_a_classical_controller_under_the_cap_and_the_recovery():
    from pfbnav.control.recovery import Recovery
    from pfbnav.control.route_cap import RouteCapped

    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    b2 = RegulatedPurePursuit.from_config(cfg)
    assert isinstance(b2, ClassicalController) and b2.name == "B2_regulated_pure_pursuit"
    wrapped = Recovery.from_config(RouteCapped(b2, lambda _episode: None), cfg)
    assert wrapped.name == b2.name
    act = b2.act(make_input())
    assert act.shape == (3,) and np.all(np.abs(act) <= 1.0)
