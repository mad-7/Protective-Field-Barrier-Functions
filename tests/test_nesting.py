"""NESTING (S2, ruled 2026-10-19): every stop that begins from a MOVING state -- translating or
spinning -- ends with the at-rest field inside the field in force at the trigger, at every reachable
rest pose. Exhaustive over speeds (all three bands), yaw rates (spins on the spot included), travel
directions relative to the body, and limiter trajectories to rest. Beside the monotonicity tests in
`tests/test_certified_layer.py`. No simulator.

Reachable rest poses: from the state, over the reach time T the BODY-FRAME velocity changes at
a_max in one of 12 directions (or not at all) and the yaw rate at +-alpha_max (or not at all), then
both brake -- the limiter's bang-bang extremes. The velocity turns with the body (a holonomic base's
command is body-frame), and every twist is projected into the wheel budget as the base projects it;
each state is projected too, so no state the base cannot be in is tested. At each rest pose the at-rest field (yaw rate zero) is placed and
its boundary compared radially with the field in force; the worst excess must not exceed
GRID_TOLERANCE_M, which covers only this test's own discretisation (the 0.1 deg comparison grid,
interpolation that can only understate the field, and the 4 ms trajectory step).

Positive control: from AT-REST states -- excluded by the ruling, since in a static world a
stationary robot with a clear field cannot be triggered -- nesting fails by more than 10 mm, so the
check is shown able to see a violation.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.eval.containment import in_frame
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
P = certified_params_from_config(CFG)
FIELDS = FieldGeometry(FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).extent, P)
GRID = np.linspace(-math.pi, math.pi, 3600, endpoint=False)
GRID_TOLERANCE_M = 3e-4
DT = 0.004
REST = FIELDS.select(GRID, 0.0, 0.0, True)
REST_PTS = np.column_stack([REST * np.cos(GRID), REST * np.sin(GRID)])
_B = CFG["actuation"]["wheel_budget"]
BUDGET = float(_B["filter_fraction"]) * float(_B["v_rim"])
LEVER = float(CFG["robot"]["half_wheelbase"]) + float(CFG["robot"]["half_track"])


def in_budget(u, w):
    """Scale (u, w) into |ux| + |uy| + L|w| <= budget, direction preserved, as the base does."""
    use = abs(u[0]) + abs(u[1]) + LEVER * abs(w)
    return (u * (BUDGET / use), w * (BUDGET / use)) if use > BUDGET else (u, w)


def rest_poses(v0, w0):
    out = []
    for ang in np.radians(np.arange(0, 360, 30)):
        for alpha in (-P.alpha_max, 0.0, P.alpha_max):
            for push in (True, False):
                acc = np.array([math.cos(ang), math.sin(ang)]) * (P.a_max if push else 0.0)
                pos, th, v, w, t = np.zeros(2), 0.0, np.asarray(v0, dtype=float), w0, 0.0
                while t < P.reach_time - 1e-12:
                    v, w = in_budget(v + acc * DT,
                                     float(np.clip(w + alpha * DT, -P.yaw_cap, P.yaw_cap)))
                    c, sn = math.cos(th), math.sin(th)
                    pos = pos + DT * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                    th, t = th + w * DT, t + DT
                while np.hypot(*v) > 1e-9 or abs(w) > 1e-9:
                    s = np.hypot(*v)
                    v = v * max(s - P.a_brake * DT, 0.0) / s if s > 0 else v
                    w = math.copysign(max(abs(w) - P.alpha_brake * DT, 0.0), w)
                    c, sn = math.cos(th), math.sin(th)
                    pos = pos + DT * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                    th += w * DT
                out.append((pos[0], pos[1], th))
    return out


def worst_excess(speed, heading, yaw_rate, at_rest=False):
    v0, yaw_rate = in_budget(speed * np.array([math.cos(heading), math.sin(heading)]), yaw_rate)
    speed = float(np.hypot(*v0))
    in_force = FIELDS.select(GRID, speed, heading, at_rest, yaw_rate=yaw_rate)
    worst = -np.inf
    for pose in rest_poses(v0, yaw_rate):
        q = in_frame(REST_PTS, pose, (0.0, 0.0, 0.0))
        r, b = np.hypot(q[:, 0], q[:, 1]), np.arctan2(q[:, 1], q[:, 0])
        worst = max(worst, float(np.max(r - np.interp(b, GRID, in_force, period=2 * math.pi))))
    return worst


SPEEDS = [0.03, 0.10, 0.20, 0.30, 0.43]                   # band 1 / 1 / 1 / 2 / 2 by reach
HEADINGS = [0.0, math.radians(45), math.radians(90), math.pi]
YAW_RATES = [0.0, 0.3, -1.0, 1.5]


@pytest.mark.parametrize("speed", SPEEDS)
def test_every_stop_from_a_translating_state_nests(speed):
    worst = max(worst_excess(speed, h, w) for h in HEADINGS for w in YAW_RATES)
    assert worst <= GRID_TOLERANCE_M, f"speed {speed}: the at-rest field leaves by {worst:.5f} m"


@pytest.mark.parametrize("yaw_rate", [0.07, 0.3, 0.7, 1.5, -0.7])
def test_every_stop_from_a_spin_on_the_spot_nests(yaw_rate):
    """A spin is a moving state (ruled 2026-10-19: at rest needs the yaw rate below its
    threshold too), so its stop must nest."""
    assert yaw_rate > P.standstill_yaw_exit or -yaw_rate > P.standstill_yaw_exit
    assert worst_excess(0.0, 0.0, yaw_rate) <= GRID_TOLERANCE_M


def test_positive_control_at_rest_states_do_not_nest():
    """Excluded by the ruling, and the reason the exclusion is needed: no bounded at-rest field
    that carries the reach from rest can contain copies of itself shifted by that reach."""
    assert worst_excess(0.0, 0.0, 0.0, at_rest=True) > 0.010


def test_the_rotational_standstill_thresholds_follow_the_translational_ones():
    """Derived (2026-10-19), rounded down: the translational thresholds over the footprint's
    outermost radius, so at rest no body point moves faster than the translational threshold."""
    psi = np.radians(np.arange(0.0, 360.0, 0.05))
    radius = float(FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"]).extent(psi).max())
    assert P.standstill_yaw_enter <= P.standstill_enter / radius
    assert P.standstill_yaw_exit <= P.standstill_exit / radius
    assert P.standstill_yaw_exit == pytest.approx(P.standstill_exit / radius, abs=2e-4)


def _creep_excess(params, speed, heading, yaw_rate):
    """How far, radially, the footprint swept from an at-rest state (creeping at `speed`) to rest
    reaches beyond that state's at-rest field, with the margins removed so the reach alone is
    tested; body-frame velocity, wheel budget, as above."""
    import dataclasses

    from pfbnav.eval.containment import footprint_boundary
    bare = dataclasses.replace(params, margin=1e-9, lateral_margin=1e-9)
    table = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
    fields = FieldGeometry(table.extent, bare)
    boundary = footprint_boundary(table.extent, 360)
    field = fields.select(GRID, 0.0, 0.0, True, yaw_rate=yaw_rate)
    worst = -np.inf
    v0 = speed * np.array([math.cos(heading), math.sin(heading)])
    for ang in np.radians(np.arange(0, 360, 30)):
        for alpha in (-P.alpha_max, 0.0, P.alpha_max):
            acc = P.a_max * np.array([math.cos(ang), math.sin(ang)])
            pos, th, v, w, t = np.zeros(2), 0.0, v0.copy(), yaw_rate, 0.0
            poses = []
            while t < P.reach_time - 1e-12:
                v, w = in_budget(v + acc * DT, float(np.clip(w + alpha * DT, -P.yaw_cap,
                                                              P.yaw_cap)))
                c, sn = math.cos(th), math.sin(th)
                pos = pos + DT * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                th, t = th + w * DT, t + DT
                poses.append((pos[0], pos[1], th))
            while np.hypot(*v) > 1e-9 or abs(w) > 1e-9:
                s = np.hypot(*v)
                v = v * max(s - P.a_brake * DT, 0.0) / s if s > 0 else v
                w = math.copysign(max(abs(w) - P.alpha_brake * DT, 0.0), w)
                c, sn = math.cos(th), math.sin(th)
                pos = pos + DT * np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])
                th += w * DT
                poses.append((pos[0], pos[1], th))
            for pose in poses[::4] + [poses[-1]]:
                q = in_frame(boundary, pose, (0.0, 0.0, 0.0))
                r, b = np.hypot(q[:, 0], q[:, 1]), np.arctan2(q[:, 1], q[:, 0])
                worst = max(worst, float(np.max(r - np.interp(b, GRID, field,
                                                              period=2 * math.pi))))
    return worst


def test_the_at_rest_field_contains_creep_from_the_exit_threshold():
    """Ruled condition (2026-10-19): the at-rest field is sized for the motion reachable from every
    state classified at rest -- from the standstill EXIT threshold, not zero -- so creep at 0.02 m/s
    (and a spin up to the rotational exit) is contained. Margins removed: the reach alone."""
    for heading in np.radians([0.0, 45.0, 90.0, 180.0]):
        for w in (0.0, P.standstill_yaw_exit, -P.standstill_yaw_exit):
            assert _creep_excess(P, P.standstill_exit, heading, w) <= 1.5e-3


def test_negative_control_an_at_rest_field_sized_from_zero_does_not_contain_creep():
    import dataclasses
    from_zero = dataclasses.replace(P, standstill_enter=0.0, standstill_exit=1e-9)
    assert _creep_excess(from_zero, P.standstill_exit, 0.0, 0.0) > 2e-3
