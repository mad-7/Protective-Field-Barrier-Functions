"""Known answers and brute-force containment for the reachable-motion field (`pfbnav/safety/
reach.py`; ruled 2026-10-18, option a). The safety property is containment: everything the
footprint sweeps from the stop trigger to rest lies inside the field in force at the trigger."""
import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.reach import (
    ReachFields,
    ReachParams,
    band_depth,
    box_budget_speed_cap,
    reach_speed,
    rest_reach,
    swerve,
    yaw_excursion,
    yaw_interval,
)
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
P = ReachParams(reach_time=0.15, a_max=1.0, alpha_max=3.0, a_brake=1.0, alpha_brake=3.0,
                speed_cap=0.5073, yaw_cap=1.5, rest_speed=0.02)


def test_the_speed_cap_of_the_box_and_the_wheel_budget():
    # |vx| <= 0.4, |vy| <= 0.5, |vx| + |vy| <= 0.586: the vertex (0.086, 0.5)
    assert box_budget_speed_cap(0.4, 0.5, 0.586) == pytest.approx(math.hypot(0.086, 0.5))
    assert box_budget_speed_cap(0.4, 0.4, 2.0) == pytest.approx(math.hypot(0.4, 0.4))
    assert box_budget_speed_cap(0.4, 0.5, 0.3) == pytest.approx(0.3)
    rp = ReachParams.from_config(CFG)
    assert rp.reach_time == pytest.approx(0.15)                  # 0.10 response + 0.05 step
    assert rp.speed_cap == pytest.approx(math.hypot(0.95 * 0.617 - 0.5, 0.5))
    assert rp.yaw_cap == pytest.approx(1.5)                      # w_max binds (budget: 1.517)


def test_reach_speed_rest_reach_and_band_depth_known_answers():
    assert reach_speed(0.10, P) == pytest.approx(0.25)
    assert reach_speed(0.45, P) == pytest.approx(0.5073)          # capped
    # from 0.02 m/s: 0.003 + 0.01125 over T, then 0.17^2 / 2 braking
    assert rest_reach(P) == pytest.approx(0.02 * 0.15 + 0.5 * 0.0225 + 0.17 ** 2 / 2)
    assert rest_reach(dataclasses.replace(P, rest_speed=1e-12)) == pytest.approx(0.0225)  # a T^2
    assert band_depth(0.30, P) == pytest.approx(0.30 * 0.15 + 0.09 / 2)


def test_yaw_excursion_known_answers():
    # from zero: alpha T^2 / 2 + (alpha T)^2 / (2 alpha) = alpha T^2
    assert yaw_excursion(0.0, P) == pytest.approx(3.0 * 0.0225)
    # from 0.30: 0.045 + 0.03375 over T, then 0.75^2 / 6 braking
    assert yaw_excursion(0.30, P) == pytest.approx(0.045 + 0.03375 + 0.75 ** 2 / 6)
    # against: -0.045 + 0.03375 over T, then 0.15^2 / 6 -> still behind the start: 0
    assert yaw_excursion(-0.30, P) == 0.0
    # A20 sizes the sweep from |omega|: symmetric, the larger excursion on both sides
    assert yaw_interval(0.30, P) == pytest.approx((-0.1725, 0.1725))
    assert yaw_interval(-0.30, P) == pytest.approx((-0.1725, 0.1725))
    lo, hi = yaw_interval(0.0, P)
    assert lo == pytest.approx(-hi)
    # capped: from 1.4 the rate reaches 1.5 after 1/30 s
    tc = 0.1 / 3.0
    assert yaw_excursion(1.4, P) == pytest.approx(1.4 * tc + 1.5 * tc * tc + 1.5 * (0.15 - tc)
                                                  + 1.5 ** 2 / 6)


@pytest.mark.parametrize("w0", [-1.5, -0.6, -0.3, -0.05, 0.0, 0.05, 0.3, 0.9, 1.45])
def test_yaw_excursion_is_the_brute_force_maximum(w0):
    """Every bang-bang yaw-acceleration profile over T (switching once), then braking: the
    largest positive heading reached matches the closed form, and none exceeds it."""
    dt = 1e-4
    best = 0.0
    steps = round(P.reach_time / dt)             # an integer count: no extra step from float sums
    for switch in np.linspace(0.0, P.reach_time, 31):
        w, th = w0, 0.0
        for k in range(steps):
            acc = P.alpha_max if k * dt >= switch else -P.alpha_max
            w = float(np.clip(w + acc * dt, -P.yaw_cap, P.yaw_cap))
            th += w * dt
            best = max(best, th)
        while abs(w) > 1e-9:
            w = math.copysign(max(abs(w) - P.alpha_brake * dt, 0.0), w)
            th += w * dt
            best = max(best, th)
    assert best <= yaw_excursion(w0, P) + 2e-4
    assert best >= yaw_excursion(w0, P) - 2e-3


def _fields(variant, margin=1e-6):
    """The assessment-era field: limits P (a_max 1.0, alpha_max 3.0) with the bands that went with
    them (0.10, 0.30, 0.40) -- the variant and containment tests are about the construction, so
    they keep one coherent parameter set rather than mixing it with A20's bands."""
    params = dataclasses.replace(certified_params_from_config(CFG), margin=margin,
                                 lateral_margin=margin, band_top_speeds=(0.10, 0.30, 0.40),
                                 # pre-A20: the top band then stopped at 0.40 (the A20 check that
                                 # it cover the reachable cap would refuse it; P carries the reach)
                                 speed_cap=0.40)
    return ReachFields(FieldGeometry(TABLE.extent, params), P, variant)


def _world(v, th):
    """A body-frame velocity in the trigger frame: it turns with the body (a holonomic base's
    command is body-frame). Until 2026-10-19 these trajectories held it fixed in the trigger frame,
    which hid the drift of a body that turns while it translates (`certified.coupling`)."""
    c, s = math.cos(th), math.sin(th)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _swept_points(v0, w0, accel, alpha, dt=2e-3):
    """Footprint boundary points swept from the trigger to rest, in the trigger's body frame:
    over T the velocity changes at `accel` (m/s^2, a 2-vector, |accel| <= a_max) and the yaw rate
    at `alpha`; then the velocity brakes along a straight line at a_brake and the yaw at
    alpha_brake (what the limiter does with a zero command)."""
    psi = np.radians(np.arange(0.0, 360.0, 2.0))
    e = TABLE.extent(psi)
    fp = np.column_stack([e * np.cos(psi), e * np.sin(psi)])
    pos, th = np.zeros(2), 0.0
    v, w = np.asarray(v0, dtype=float), float(w0)
    pts = [fp.copy()]
    t = 0.0
    while t < P.reach_time:
        v = v + np.asarray(accel) * dt
        s = np.hypot(*v)
        if s > P.speed_cap:
            v *= P.speed_cap / s
        w = float(np.clip(w + alpha * dt, -P.yaw_cap, P.yaw_cap))
        pos, th, t = pos + _world(v, th) * dt, th + w * dt, t + dt
        c, s_ = math.cos(th), math.sin(th)
        pts.append(fp @ np.array([[c, s_], [-s_, c]]) + pos)
    while np.hypot(*v) > 1e-9 or abs(w) > 1e-9:
        s = np.hypot(*v)
        v = v * max(s - P.a_brake * dt, 0.0) / s if s > 0 else v
        w = math.copysign(max(abs(w) - P.alpha_brake * dt, 0.0), w)
        pos, th = pos + _world(v, th) * dt, th + w * dt
        c, s_ = math.cos(th), math.sin(th)
        pts.append(fp @ np.array([[c, s_], [-s_, c]]) + pos)
    return np.vstack(pts)


def _contained(rf, pts, speed, heading, yaw_rate, at_rest, tol=1.5e-3):
    """Every swept point within the field selected at the trigger (radially), to `tol` m: the
    support grid and the 2-deg footprint sampling cost about a millimetre."""
    bearing = np.arctan2(pts[:, 1], pts[:, 0])
    reach = rf.select(bearing, speed, heading, at_rest, yaw_rate=yaw_rate)
    return float(np.max(np.hypot(pts[:, 0], pts[:, 1]) - reach)) <= tol


# Trajectories from the 2026-10-18 traces and around them: (v0, w0, accel, alpha).
ALONG = [((0.14, 0.0), -0.30, (1.0, 0.0), -3.0),       # narrow 10, step 78: turning harder
         ((0.16, 0.0), 0.30, (1.0, 0.0), 3.0),
         ((0.07, 0.0), 0.13, (1.0, 0.0), 0.0),          # narrow 12: band-0 speed-up
         ((0.25, 0.0), 0.0, (0.0, 0.0), 3.0),
         ((0.0, 0.0), 0.0, (1.0, 0.0), 3.0)]


@pytest.mark.parametrize("v0, w0, accel, alpha", ALONG)
def test_both_variants_contain_acceleration_along_the_travel_direction(v0, w0, accel, alpha):
    for variant in ("ruled", "full"):
        rf = _fields(variant)
        speed = math.hypot(*v0)
        at_rest = speed <= rf.params.standstill_enter
        heading = math.atan2(v0[1], v0[0])
        assert _contained(rf, _swept_points(v0, w0, accel, alpha), speed, heading, w0, at_rest), \
            (variant, v0, w0)


SWERVE = [((0.20, 0.0), 0.0, (0.0, 1.0), 0.0),          # sideways at a_max
          ((0.25, 0.0), 0.2, (0.6, 0.8), -3.0),
          ((0.0, 0.0), 0.0, (0.0, -1.0), 3.0),          # from rest: covered by both
          ((0.10, 0.0), -0.4, (-0.6, 0.8), 3.0)]


@pytest.mark.parametrize("v0, w0, accel, alpha", SWERVE)
def test_the_full_variant_contains_a_swerve(v0, w0, accel, alpha):
    rf = _fields("full")
    speed = math.hypot(*v0)
    assert _contained(rf, _swept_points(v0, w0, accel, alpha), speed, 0.0, w0,
                      speed <= rf.params.standstill_enter)


def test_the_ruled_variant_does_not_contain_a_swerve_while_moving():
    """Why the 'full' variant was adopted (A20): moving at 0.25 m/s, a sideways command at a_max
    leaves the along-heading field (about 41 mm sideways against the at-rest field's 29 mm, which
    every moving field contains), and the full field contains it."""
    pts = _swept_points((0.25, 0.0), 0.0, (0.0, 1.0), 0.0)
    assert not _contained(_fields("ruled"), pts, 0.25, 0.0, 0.0, False)
    assert _contained(_fields("full"), pts, 0.25, 0.0, 0.0, False)


def test_a_field_without_reach_does_not_contain_the_traced_turn():
    """Negative control (the 2026-10-18 finding): a field sized from the rates at the decision held
    constant -- no reach (a_max, alpha_max -> 0), as before A20 -- does not contain the turn the
    robot makes when the command keeps accelerating; the reach field does."""
    params = dataclasses.replace(certified_params_from_config(CFG), margin=1e-6,
                                 lateral_margin=1e-6)
    constant = dataclasses.replace(params, a_max=1e-9, alpha_max=1e-9, alpha_brake=3.0)
    pts = _swept_points((0.14, 0.0), -0.30, (1.0, 0.0), -3.0)
    bearing = np.arctan2(pts[:, 1], pts[:, 0])
    reach = FieldGeometry(TABLE.extent, constant).select(bearing, 0.14, 0.0, at_rest=False,
                                                         yaw_rate=-0.30)
    assert float(np.max(np.hypot(pts[:, 0], pts[:, 1]) - reach)) > 5e-3
    assert _contained(_fields("full"), pts, 0.14, 0.0, -0.30, False)


def test_the_at_rest_field_is_the_infimum_of_the_moving_fields():
    rf = _fields("ruled", margin=0.10)
    rest = rf.rest_support(rf.normal_angles)
    for variant in ("ruled", "full"):
        rf = ReachFields(rf.fields, P, variant)
        for top in rf.params.band_top_speeds:
            for heading in np.radians([0.0, 37.0, 90.0, 180.0]):
                for w in (-1.0, -0.3, 0.0, 0.3, 1.0):
                    s = rf.support(rf.normal_angles, top, heading, False, yaw_rate=w)
                    assert np.all(s >= rest - 1e-12)


def test_full_is_never_smaller_than_ruled_and_both_grow_with_the_band():
    ruled, full = _fields("ruled", 0.10), _fields("full", 0.10)
    full = ReachFields(ruled.fields, P, "full")
    prev = None
    for top in ruled.params.band_top_speeds:
        r = ruled.support(ruled.normal_angles, top, 0.3, False, yaw_rate=0.2)
        f = full.support(full.normal_angles, top, 0.3, False, yaw_rate=0.2)
        assert np.all(f >= r - 1e-12)
        if prev is not None:
            assert np.all(r >= prev - 1e-12)
        prev = r


def test_the_swerve_bound_dominates_sampled_velocity_changes():
    """Brute force over velocity changes |delta| <= a T: the displacement's support never exceeds
    the bound, at any direction relative to travel."""
    rng = np.random.default_rng(3)
    V = 0.30
    rho = P.a_max * P.reach_time
    nus = np.radians(np.arange(0.0, 360.0, 5.0))
    bound = swerve(np.cos(nus), V, P)
    for _ in range(400):
        speed = rng.uniform(0.0, V - rho)
        d = rng.normal(size=2)
        d *= rng.uniform(0.0, rho) / np.hypot(*d)
        # constant acceleration toward the change over T, then straight braking
        v0 = np.array([speed, 0.0])
        vT = v0 + d
        disp = v0 * P.reach_time + 0.5 * d * P.reach_time + vT * np.hypot(*vT) / (2 * P.a_brake)
        support = disp[0] * np.cos(nus) + disp[1] * np.sin(nus)
        assert np.all(support <= bound + 1e-12)


@pytest.mark.parametrize("v0, w0, accel, alpha", [
    ((0.02, 0.0), 0.0, (-1.0, 0.0), 0.0),            # creeping at the standstill exit, reversing
    ((0.0, 0.02), 0.0, (0.6, -0.8), -3.0),
    ((0.014, -0.014), 0.0, (0.0, 1.0), 3.0),
    ((0.0, 0.0), 0.8, (0.0, 0.0), 3.0),              # turning on the spot: at rest by speed
    ((0.0, 0.0), -1.2, (0.7, 0.7), -3.0),
])
def test_the_at_rest_field_contains_the_reach_from_any_at_rest_state(v0, w0, accel, alpha):
    """At rest the composed speed can be up to the standstill exit (hysteresis), in a direction
    the field does not trust, and the yaw rate anything: the at-rest field contains it all."""
    rf = _fields("full")
    assert _contained(rf, _swept_points(v0, w0, accel, alpha), 0.0, 0.0, w0, True)


def test_reach_from_rest_counts_the_standstill_speed():
    """Negative control: an at-rest field sized from zero speed does not contain a robot creeping
    at the standstill exit that then accelerates (the recovery creeps at exactly that speed)."""
    rf = _fields("full")
    zero = ReachFields(rf.fields, dataclasses.replace(P, rest_speed=1e-12), "full")
    pts = _swept_points((0.02, 0.0), 0.0, (1.0, 0.0), 0.0)
    assert _contained(rf, pts, 0.0, 0.0, 0.0, True)
    assert not _contained(zero, pts, 0.0, 0.0, 0.0, True, tol=1e-4)



def test_the_adopted_field_is_the_assessed_full_variant():
    """A20 adopted the FULL construction in `pfbnav/safety/certified.py`; the assessment's variant
    (this module's `ReachFields`) must give the same field at the configured limits."""
    params = certified_params_from_config(CFG)
    adopted = FieldGeometry(TABLE.extent, params)
    assessed = ReachFields(adopted, ReachParams.from_config(CFG), "full")
    n = adopted.normal_angles
    for top in params.band_top_speeds:
        for heading in np.radians([0.0, 45.0, 180.0]):
            for w in (-0.8, 0.0, 0.25):
                assert np.allclose(adopted.support(n, top, heading, False, yaw_rate=w),
                                   assessed.support(n, top, heading, False, yaw_rate=w),
                                   rtol=0.0, atol=1e-12)
    for w in (0.0, 0.6):
        assert np.allclose(adopted.support(n, 0.0, 0.0, True, yaw_rate=w),
                           assessed.support(n, 0.0, 0.0, True, yaw_rate=w), rtol=0.0, atol=1e-12)
    for speed in (0.0, 0.02, 0.03, 0.2, 0.3, 0.45):
        assert adopted.band_for(speed) == params.band_top_speeds.index(assessed.band_for(speed))
