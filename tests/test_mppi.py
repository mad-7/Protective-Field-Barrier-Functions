"""Known answers for B6, field-aware MPPI (hard rule 6; A16). The field is the CERTIFIED layer's
(`FieldGeometry`), so every breach count is checked against `FieldGeometry.select`, never against a
number copied from the table. The parameters are explicit, not the configured (untuned) ones, except
where a test is about the configured values."""
import math
import time
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import box, make_input, ray_scan

from pfbnav.config import load_config
from pfbnav.control.base import ClassicalController
from pfbnav.control.mppi import (
    TABLE_PAD_M,
    FieldAwareMPPI,
    MPPIParams,
    diamond,
    diamond_to_bearing,
    extent_of_support,
    field_table,
    inverse_cos,
)
from pfbnav.envs.controller_input import ClassicalInput, RouteView
from pfbnav.eval.tuning import flatten_numeric
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import (
    ActuationLimits,
    WheelBudget,
    clamp_velocity_with_status,
    limit_acceleration,
    project_to_wheel_budget,
)
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
PARAMS = certified_params_from_config(CFG)
FOOTPRINT = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
EXACT = FieldGeometry(FOOTPRINT.extent, PARAMS)          # the layer's own (1440 normals)
DT = float(CFG["sim"]["dt"])
LIMITS = ActuationLimits.from_config(CFG)
TAU = float(CFG["estimation"]["velocity"]["filter_tau"])
# Candidate 0 (ruled 2026-10-27), at 256 samples to keep the suite fast.
P = {"horizon_steps": 30, "samples": 256, "temperature": 0.01, "noise_std": (0.1, 0.1, 0.1),
     "iterations": 1, "route_tracking": 1.0, "goal_progress": 10.0, "field_breach": 10.0,
     "smoothness": 0.01}


@pytest.fixture(scope="module")
def table():
    return field_table(FOOTPRINT.extent, PARAMS)        # built once per process


def mppi(table, **over) -> FieldAwareMPPI:
    a, r = CFG["actuation"], CFG["robot"]
    b = a["wheel_budget"]
    return FieldAwareMPPI(MPPIParams(**{**P, **over}), table, FOOTPRINT.extent, dt=DT,
                          v_max=float(a["v_max"]), vy_max=float(a["vy_max"]),
                          w_max=float(a["w_max"]),
                          budget_limit=float(b["filter_fraction"]) * float(b["v_rim"]),
                          lever_arm=float(r["half_wheelbase"]) + float(r["half_track"]),
                          a_max=LIMITS.a_max, alpha_max=LIMITS.alpha_max,
                          a_decel_max=LIMITS.a_decel_max, estimate_alpha=DT / (TAU + DT))


def route_input(points, scan=None, *, velocity=(0.0, 0.0, 0.0), stop=False) -> ClassicalInput:
    pts = np.asarray(points, dtype=float)
    length = float(np.hypot(*np.diff(pts, axis=0).T).sum())
    base = make_input(scan=scan)
    return ClassicalInput.of(stop_active=stop, velocity=velocity, scan=base.scan,
                             route=RouteView.of(pts, pts[1], pts[-1], length), dt=DT)


def constant(command, horizon=30, k=1):
    return np.tile(np.asarray(command, dtype=float), (k, horizon, 1))


# ------------------------------------------------------------------ pieces
def test_feasible_commands_are_the_bases_box_and_budget():
    """The rollouts move as the base would: `clamp_velocity_with_status` then the filter-share
    wheel budget (`project_to_wheel_budget`), on random commands far outside both."""
    limits, budget = ActuationLimits.from_config(CFG), WheelBudget.from_config(CFG)
    cmds = np.random.default_rng(0).normal(0.0, 1.0, (300, 3))
    expected = np.array([project_to_wheel_budget(clamp_velocity_with_status(c, limits).applied,
                                                 budget.filter_limit, budget.lever_arm).applied
                         for c in cmds])
    got = FieldAwareMPPI.feasible(mppi(field_table(FOOTPRINT.extent, PARAMS)), cmds)
    np.testing.assert_allclose(got, expected, rtol=0.0, atol=1e-15)


def test_the_limiter_is_the_bases():
    """`limit` is `limit_acceleration` (a_max speeding up, a_decel_max slowing, alpha_max on the yaw
    rate) on random commands and previous commands, in either direction (ruled 2026-10-27)."""
    rng = np.random.default_rng(1)
    cmds, prevs = rng.normal(0.0, 0.3, (400, 3)), rng.normal(0.0, 0.3, (400, 3))
    expected = np.array([limit_acceleration(c, q, DT, LIMITS) for c, q in zip(cmds, prevs,
                                                                               strict=True)])
    got = mppi(field_table(FOOTPRINT.extent, PARAMS)).limit(cmds, prevs)
    np.testing.assert_allclose(got, expected, rtol=0.0, atol=1e-15)


def test_a_rollout_respects_the_limiter_from_the_command_executing(table):
    """From rest, a constant 0.4 m/s request: the applied speed rises by a_max dt per step until it
    reaches 0.4 (the rollout cannot jump to it); from 0.4, a zero request falls by a_decel_max dt."""
    c = mppi(table)
    v = c.applied_sequence(constant((0.4, 0.0, 0.0), 30), np.zeros(3))[0]
    up = LIMITS.a_max * DT
    expected = np.minimum(up * np.arange(1, 31), 0.4)
    np.testing.assert_allclose(v[:, 0], expected, atol=1e-12)
    v = c.applied_sequence(constant((0.0, 0.0, 0.0), 30), np.array([0.4, 0.0, 0.0]))[0]
    down = LIMITS.a_decel_max * DT
    np.testing.assert_allclose(v[:, 0], np.maximum(0.4 - down * np.arange(1, 31), 0.0), atol=1e-12)
    w = c.applied_sequence(constant((0.0, 0.0, 1.0), 5), np.zeros(3))[0]
    np.testing.assert_allclose(w[:, 2], LIMITS.alpha_max * DT * np.arange(1, 6), atol=1e-12)


def test_the_field_is_sized_by_the_certified_composition(table):
    """max(|estimate|, |command executing|), the estimate predicted by the estimator's filter: an
    estimate of 0.3 m/s along +y while the rollout commands 0.1 m/s along +x sizes the first state's
    field from the estimate (0.3 + alpha (0.1 x - 0.3 y) ... still the larger, heading ~ +y); a slow
    estimate under a fast command is sized by the command. The yaw rate is the larger magnitude."""
    c = mppi(table)
    a = DT / (TAU + DT)
    v = constant((0.1, 0.0, 0.2), 3)
    speed, heading, yaw = c.field_motion(v, np.array([0.0, 0.3, 0.5]))
    e1 = np.array([0.0, 0.3, 0.5]) + a * (np.array([0.1, 0.0, 0.2]) - np.array([0.0, 0.3, 0.5]))
    assert speed[0, 0] == pytest.approx(math.hypot(e1[0], e1[1]))
    assert heading[0, 0] == pytest.approx(math.atan2(e1[1], e1[0]))
    assert yaw[0, 0] == pytest.approx(abs(e1[2]))
    speed, heading, yaw = c.field_motion(constant((0.4, 0.0, 0.0), 3), np.zeros(3))
    assert speed[0].tolist() == pytest.approx([0.4] * 3) and not heading.any() and not yaw.any()


def test_a_stop_resets_the_start_to_the_brakes_step(table):
    """After a step on which the layer held the robot (stop_active), the rollouts start where the
    base then is: the limiter's step toward zero from the command executing before."""
    c = mppi(table)
    c.begin_episode({"scenario_seed": 2})
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan())
    for _ in range(20):
        c.command(inp)
    before = c._before.copy()
    c.command(route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan(), stop=True))
    assert c.held_steps == 1
    assert c._before.tolist() == pytest.approx(c.limit(np.zeros(3), before).tolist())


def test_the_diamond_angle_known_answers_monotone_and_inverse():
    assert diamond(np.array(1.0), np.array(0.0)) == 0.0
    assert diamond(np.array(0.0), np.array(1.0)) == 1.0
    assert diamond(np.array(-1.0), np.array(0.0)) == 2.0
    assert diamond(np.array(0.0), np.array(-1.0)) == -1.0
    assert diamond(np.array(1.0), np.array(1.0)) == 0.5
    psi = np.linspace(-math.pi + 1e-6, math.pi - 1e-6, 2001)
    d = diamond(np.cos(psi), np.sin(psi))
    assert np.all(np.diff(d) > 0.0)
    np.testing.assert_allclose(diamond_to_bearing(d), psi, atol=1e-12)


def test_the_extent_formula_is_the_layers():
    """`extent_of_support` is `FieldGeometry.extent`'s formula: equal on the layer's own support."""
    b = np.linspace(-math.pi, math.pi, 97)
    h = EXACT.support(EXACT.normal_angles, 0.3, 0.4, False, yaw_rate=0.7)
    np.testing.assert_allclose(extent_of_support(h, inverse_cos(b, EXACT.normal_angles))[0],
                               EXACT.extent(b, 0.3, 0.4, False, yaw_rate=0.7), atol=1e-12)


def test_the_table_is_never_below_the_certified_field(table):
    """CONSERVATIVE (ruled 2026-10-27): the padded table against `FieldGeometry.select` (1440
    normals) over a dense sweep -- 400 random states over every band, heading and yaw rate, plus the
    at-rest rows, each at 2,880 bearings -- is never below the exact field. The pad is 3 mm, 1.5 x
    the unpadded interpolant's worst undershoot (-2.0 mm over 5,400 states, 2026-10-27); the
    over-statement stays within 13.5 mm, and within 9 mm abeam (60-120 deg off the travel direction),
    where the narrowest aisle leaves about 21 mm."""
    assert TABLE_PAD_M == table.pad == 0.003
    rng = np.random.default_rng(11)
    b = np.linspace(-math.pi, math.pi, 2880, endpoint=False) + 1e-4
    low, high, abeam = [], [], []
    for _ in range(400):
        s, h = rng.uniform(0.0, PARAMS.speed_cap), rng.uniform(-math.pi, math.pi)
        w = rng.uniform(-PARAMS.yaw_cap, PARAMS.yaw_cap)
        rest = s <= PARAMS.standstill_enter and abs(w) <= PARAMS.standstill_yaw_enter
        d = table.extent(np.full(b.shape, s), np.full(b.shape, h), np.full(b.shape, w), b) \
            - EXACT.select(b, s, h, rest, yaw_rate=w)
        rel = np.abs(np.remainder(b - h + math.pi, 2.0 * math.pi) - math.pi)
        side = (rel > math.radians(60)) & (rel < math.radians(120))
        low.append(d.min())
        high.append(d.max())
        abeam.append(d[side].max())
    for _ in range(60):
        w = rng.uniform(-PARAMS.standstill_yaw_enter, PARAMS.standstill_yaw_enter)
        d = table.extent(np.zeros(b.shape), np.zeros(b.shape), np.full(b.shape, w), b) \
            - EXACT.select(b, 0.0, 0.0, True, yaw_rate=w)
        low.append(d.min())
        high.append(d.max())
    assert min(low) >= 0.0
    assert max(high) <= 0.0135 and max(abeam) <= 0.009


def test_the_band_and_rest_rows_follow_the_layer(table):
    """The band is chosen from the speed reachable within T (A20), and rest needs both standstill
    ENTER thresholds: at 0.35 m/s the band-2 field (as the layer), slower moves band 0."""
    assert EXACT.band_for(0.35) == 2 and EXACT.band_for(0.02) == 0
    rows = table.state_rows(np.array([0.35, 0.02, 0.005, 0.005]), np.zeros(4),
                            np.array([0.0, 0.0, 0.0, 0.5]))[0]
    n = table.heading_cells * len(table.yaw_knots)
    assert rows[0] // n == 2 and rows[1] // n == 0
    assert rows[2] >= table.rest_base and rows[3] < table.rest_base   # a spin is moving


def test_params_from_the_config_and_their_refusals():
    block = CFG["baselines"]["mppi"]
    p = MPPIParams.from_block(block)
    # samples x horizon fixed by the real-time measurement on the training host (2026-10-29).
    assert (p.horizon_steps, p.samples, p.iterations) == (20, 1024, 1)
    # Candidate 0, ruled 2026-10-27.
    assert p.noise_std == (0.10, 0.10, 0.10) and p.temperature == 0.01
    assert (p.goal_progress, p.smoothness, p.field_breach, p.route_tracking) == (10.0, 0.01, 10.0,
                                                                                 1.0)
    assert block["compute_measurement"] == "runs/realtime/REALTIME_20261029T095844Z_seed0"
    assert MPPIParams.from_flat(**flatten_numeric(block)) == p
    with pytest.raises(ValueError, match="declared"):
        MPPIParams.from_block({**block, "model": "unicycle"})
    with pytest.raises(ValueError, match="integer"):
        MPPIParams.from_block({**block, "samples": 30.5})
    with pytest.raises(ValueError, match="weights"):
        MPPIParams.from_block({**block, "weights": {**block["weights"], "heading": 1.0}})
    with pytest.raises(ValueError, match="temperature"):
        MPPIParams(**{**P, "temperature": 0.0})
    with pytest.raises(ValueError, match="noise_std"):
        MPPIParams(**{**P, "noise_std": (0.1, 0.1)})


def test_rollout_known_answers(table):
    c = mppi(table)
    s = c.rollout(constant((0.2, 0.0, 0.0)))[0]
    np.testing.assert_allclose(s[:, 0], 0.2 * DT * np.arange(1, 31), atol=1e-15)
    assert not s[:, 1:].any()
    s = c.rollout(constant((0.0, 0.0, 1.0)))[0]                      # spin on the spot
    np.testing.assert_allclose(s[:, 2], DT * np.arange(1, 31), atol=1e-15)
    assert np.abs(s[:, :2]).max() == 0.0
    s = c.rollout(constant((0.0, 0.1, 0.0)))[0]                      # body +y is left
    np.testing.assert_allclose(s[:, 1], 0.1 * DT * np.arange(1, 31), atol=1e-15)


# ------------------------------------------------------------------ the cost terms
def test_cost_terms_known_answers(table):
    """On an open, straight route along +x (weights from P):
      * forward at 0.2 m/s: no tracking error, progress 0.2 x 30 x 0.05 = 0.3 m -> -10 x 0.3,
        smoothness 0.01 x 0.2^2 (only the first command changes, from rest);
      * sideways at 0.1 m/s: tracking sum over t of (0.005 t)^2 = 2.5e-5 x 9455, no progress,
        smoothness 0.01 x 0.1^2."""
    c = mppi(table)
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan())
    v = np.concatenate([constant((0.2, 0.0, 0.0)), constant((0.0, 0.1, 0.0))])
    t = c.cost_terms(v, inp)
    assert t["route_tracking"] == pytest.approx([0.0, 2.5e-5 * 9455], abs=1e-12)
    assert t["goal_progress"] == pytest.approx([-3.0, 0.0], abs=1e-12)
    assert t["smoothness"] == pytest.approx([0.01 * 0.04, 0.01 * 0.01], abs=1e-15)
    assert t["field_breach"].tolist() == [0.0, 0.0] and not t["rejected"].any()


def test_progress_is_measured_along_a_bending_route(table):
    """A route turning left 0.1 m ahead: driving +y from (0, 0) never gets onto it, while the
    projection of x = 0.3, y = 0 on the route [(0,0), (0.1,0), (0.1,5)] is at arc 0.1 + 0 ... the
    nearest point (0.1, 0) at arc 0.1 m."""
    c = mppi(table, horizon_steps=30)
    inp = route_input([[0.0, 0.0], [0.1, 0.0], [0.1, 5.0]], scan=ray_scan())
    t = c.cost_terms(constant((0.2, 0.0, 0.0)), inp)                 # ends at x = 0.3
    assert t["goal_progress"][0] == pytest.approx(-10.0 * 0.1, abs=1e-12)


def test_a_rollout_that_breaches_the_certified_field_is_penalised_as_the_layer_counts(table):
    """A post 0.55 m dead ahead, seen by the two bins either side of 0 deg. Moving at 0.45 m/s
    (band 2, A20) both points lie inside the band field at each of the 4 states: 8 pairs, 80 at
    weight 10. At rest the at-rest field (0.40 m at most) holds none. The layer's own count is
    computed here, and every point is first checked to lie more than 10 mm from the boundary (76 mm
    as built), so the table's error cannot decide the answer. (A wall across the route leaves some
    bin within 3 mm of the boundary: found by that check, 2026-10-27.)"""
    c = mppi(table, horizon_steps=4)
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan([box(0.55, -0.04, 0.6, 0.04)]))
    pts = c.scan_points(inp)
    v = np.concatenate([constant((0.45, 0.0, 0.0), 4), constant((0.0, 0.0, 0.0), 4)])
    exact = []
    for cmd, at_rest in (((0.45, 0.0, 0.0), False), ((0.0, 0.0, 0.0), True)):
        n = 0
        for t in range(1, 5):
            x = cmd[0] * DT * t
            b, r = np.arctan2(pts[:, 1], pts[:, 0] - x), np.hypot(pts[:, 0] - x, pts[:, 1])
            ext = EXACT.select(b, cmd[0], 0.0, at_rest)
            assert np.abs(r - ext).min() > 0.010
            n += int((r < ext).sum())
        exact.append(n)
    terms = c.cost_terms(v, inp)
    assert exact == [8, 0]
    assert terms["field_breach"].tolist() == [80.0, 0.0]
    assert not terms["rejected"].any()


def test_a_point_inside_the_footprint_rejects_the_rollout(table):
    """A wall 0.35 m ahead: driving into it puts the wall inside the footprint (0.286 m ahead)
    within the horizon -- rejected, weight zero; backing away is not."""
    c = mppi(table)
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan([box(0.35, -2.0, 0.45, 2.0)]))
    v = np.concatenate([constant((0.4, 0.0, 0.0)), constant((-0.2, 0.0, 0.0))])
    terms = c.cost_terms(v, inp)
    assert terms["rejected"].tolist() == [True, False]
    w = c.weights(terms)
    assert w is not None and w.tolist() == [0.0, 1.0]


def test_an_invalid_bin_is_an_obstacle_on_the_footprint_boundary(table):
    """Hard rule 5 under the shared convention (ruled 2026-10-27): an invalid bin dead ahead is a
    point on B6's own footprint boundary on its bearing. Standing still touches it without entering
    it, so the zero rollout is NOT rejected; driving toward it is; backing away is not. From rest
    the robot does not drive ahead, as it does with the bin valid and empty."""
    blind = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan(invalid=[60]))
    c = mppi(table)
    pts = c.scan_points(blind)
    b = blind.scan.bearings[60]
    r = float(c.unobserved_edge(np.array([b]))[0])
    assert float(FOOTPRINT.extent(np.array([b]))[0]) <= r <= float(FOOTPRINT.extent(
        np.array([b]))[0]) + 0.001
    assert np.isclose(pts, [r * math.cos(b), r * math.sin(b)]).all(axis=1).any()
    v = np.concatenate([constant((0.0, 0.0, 0.0)), constant((0.2, 0.0, 0.0)),
                        constant((-0.2, 0.0, 0.0))])
    assert c.cost_terms(v, blind)["rejected"].tolist() == [False, True, False]
    c.begin_episode({"scenario_seed": 4})
    x = []                                     # the robot's travel toward the point, as applied
    for _ in range(10):
        c.command(blind)
        x.append((x[-1] if x else 0.0) + c.previous[0] * DT)
    assert c.all_rejected_steps == 0 and max(x) <= 1e-9 and x[-1] < 0.0     # it backs away
    clear = mppi(table)
    clear.begin_episode({"scenario_seed": 4})
    go = np.array([clear.command(route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan()))
                   for _ in range(10)])
    assert go[-1, 0] > 0.01                               # it starts forward (the limiter paces it)


def test_every_rollout_rejected_commands_zero_and_is_counted(table):
    """A wall inside the footprint all round leaves no admissible rollout: the command is zero, the
    nominal cleared, the step counted as `mppi_all_rejected`, and the command executing becomes the
    limiter's step toward zero."""
    boxed = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan([box(-0.25, -0.15, 0.25, 0.15)]))
    c = mppi(table)
    c.begin_episode({"scenario_seed": 4})
    assert c.command(boxed).tolist() == [0.0, 0.0, 0.0]
    assert c.all_rejected_steps == 1 and not c.nominal.any()
    assert c.episode_report()["mppi_all_rejected"] == 1


# ------------------------------------------------------------------ the controller
def test_determinism_under_a_fixed_seed_and_order_independence(table):
    """Seeded from the episode: the same scenario gives bit-identical commands, whatever ran
    before it in the process; another scenario draws other noise."""
    inp = route_input([[0.0, 0.0], [10.0, 0.0]],
                      scan=ray_scan([box(-3.0, 0.45, 5.0, 0.6), box(-3.0, -0.6, 5.0, -0.45)]))

    def run(c, seed, steps=6):
        c.reset()
        c.begin_episode({"scenario_seed": seed})
        return np.array([c.command(inp) for _ in range(steps)])

    c = mppi(table, samples=64)
    first = run(c, 5)
    run(c, 6)
    again = run(c, 5)
    other = run(mppi(table, samples=64), 6)
    assert np.array_equal(first, again)
    assert np.array_equal(run(mppi(table, samples=64), 5), first)
    assert not np.array_equal(first, other)
    assert c.episode_report()["mppi_seed_key"] == [0xB6, 5]


def test_a_straight_open_route_goes_straight(table):
    """Nothing in the scan, a straight route ahead: after 40 steps it drives forward near the
    ceiling, without drifting sideways or turning."""
    c = mppi(table)
    c.begin_episode({"scenario_seed": 3})
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan())
    cmds = np.array([c.command(inp) for _ in range(40)])
    tail = cmds[-10:].mean(axis=0)
    assert tail[0] > 0.3 and abs(tail[1]) < 0.05 and abs(tail[2]) < 0.05
    assert np.all(np.abs(cmds) <= c.ceilings + 1e-12)
    assert c.all_rejected_steps == 0


def test_the_configured_candidate_0_drives(table):
    """Candidate 0 (ruled 2026-10-27; it replaced initial values that did not move the robot, the
    finding of that day) drives forward on an open, straight route at the configured 1024 x 20,
    under the acceleration limiter (0.4 m/s is 16 steps away at a_max)."""
    c = FieldAwareMPPI.from_config(CFG)
    c.begin_episode({"scenario_seed": 3})
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan())
    cmds = np.array([c.command(inp) for _ in range(30)])
    assert cmds[-10:, 0].mean() > 0.3


def test_the_interface(table):
    c = mppi(table)
    assert isinstance(c, ClassicalController) and c.name == "B6_field_aware_mppi"
    inp = route_input([[0.0, 0.0], [10.0, 0.0]], scan=ray_scan())
    a = c.act(inp)
    assert np.allclose(a, np.clip(c.issued / c.ceilings, -1.0, 1.0))
    assert np.allclose(c.previous, c.limit(c.issued, np.zeros(3)))   # what the base applies
    with pytest.raises(TypeError):
        c.command(np.zeros(82))
    wrong = ClassicalInput.of(stop_active=False, velocity=(0, 0, 0), scan=inp.scan,
                              route=inp.route, dt=0.1)
    with pytest.raises(ValueError, match="control period"):
        c.command(wrong)
    with pytest.raises(ValueError, match="non-negative"):
        c.begin_episode({"scenario_seed": -1})


def test_per_step_time(table, capsys):
    """The real-time criterion is p95 <= 50 ms on the TRAINING host (ruled 2026-10-27), measured by
    scripts/measure_realtime.py. This prints the time on this host for the configured 1024 x 20 in a
    0.9 m aisle and in the open; no figure from a laptop is a property of the training host, so
    nothing is asserted about it. What is asserted: only points within reach (the horizon's travel
    plus the largest field) are used."""
    c = FieldAwareMPPI.from_config(CFG)
    aisle = route_input([[0.0, 0.0], [10.0, 0.0]],
                        scan=ray_scan([box(-3.0, 0.45, 5.0, 0.6), box(-3.0, -0.6, 5.0, -0.45)]))
    r = aisle.scan.ranges
    assert len(c.scan_points(aisle)) == int((r <= c.reach).sum())
    assert c.reach == pytest.approx(PARAMS.speed_cap * c.p.horizon_steps * DT + table.max_extent)
    times = {}
    for name, inp in (("aisle", aisle), ("open", route_input([[0, 0], [10, 0]], ray_scan()))):
        c.reset()
        c.command(inp)
        start = time.perf_counter()
        for _ in range(3):
            c.command(inp)
        times[name] = (time.perf_counter() - start) / 3
    with capsys.disabled():
        print(f"\nB6 per step ({c.p.samples} x {c.p.horizon_steps}): aisle {1e3 * times['aisle']:.0f} ms, open "
              f"{1e3 * times['open']:.0f} ms (criterion p95 <= 50 ms on the training host)")
    assert all(math.isfinite(t) for t in times.values())
