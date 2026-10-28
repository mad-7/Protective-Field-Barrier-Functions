"""The PFB's field (WP2 step 1, ruled 2026-10-23): known answers, and the CROSS-CHECK against the
certified layer's field. The two modules are independent (hard rule 3); a test may import both."""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.safety import certified as cert
from pfbnav.safety import pfb_field as pf
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import certified_params_from_config

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
CP = certified_params_from_config(CFG)
FP = pf.field_params_from_config(CFG)
CERT = cert.FieldGeometry(TABLE.extent, CP)
PFB = pf.PFBField(TABLE.extent, FP)
BEARINGS = -math.pi + (np.arange(120) + 0.5) * 2 * math.pi / 120
RNG = np.random.default_rng(3)


# --------------------------------------------------------------------------- the parameters
def test_the_same_keys_give_the_same_constants():
    assert FP.reach_time == CP.reach_time and FP.a_brake == CP.a_brake
    assert FP.a_max == CP.a_max and FP.alpha_max == CP.alpha_max
    assert FP.alpha_brake == CP.alpha_brake and FP.yaw_cap == pytest.approx(CP.yaw_cap, abs=1e-15)
    assert FP.speed_cap == pytest.approx(CP.speed_cap, abs=1e-15)
    assert FP.margin == CP.margin and FP.lateral_margin == CP.lateral_margin
    assert FP.rest_speed == CP.rest_speed and FP.band_tops == CP.band_top_speeds
    assert FP.rest_enter == CP.standstill_enter and FP.rest_yaw == CP.standstill_yaw_exit
    assert FP.rest_yaw_enter == CP.standstill_yaw_enter


@pytest.mark.parametrize(("vx", "vy", "budget"), [(0.5, 0.5, 0.58615), (0.4, 0.5, 0.58615),
                                                  (0.3, 0.2, 1.0), (0.5, 0.5, 0.4)])
def test_the_planar_speed_cap_matches(vx, vy, budget):
    assert pf.planar_speed_cap(vx, vy, budget) == pytest.approx(
        cert.box_budget_speed_cap(vx, vy, budget), abs=1e-15)


@pytest.mark.parametrize("w", [0.0, 0.05, 0.3, 1.0, 1.49, 2.0, -0.7])
def test_the_scalar_terms_match(w):
    assert pf.heading_reach(w, FP) == pytest.approx(cert.yaw_excursion(abs(w), CP), abs=1e-15)
    assert pf.rest_creep_reach(FP) == pytest.approx(cert.rest_reach(CP), abs=1e-15)
    for v in (0.0, 0.1, 0.3, 0.51):
        assert pf.travel_depth(v, FP) == pytest.approx(cert.band_depth(v, CP), abs=1e-15)
        assert pf.turning_drift(v, w, FP) == pytest.approx(cert.coupling(v, w, CP), abs=1e-15)
        c = np.linspace(-1.0, 1.0, 41)
        assert np.allclose(pf.swerve_reach(c, v, FP), cert.swerve(c, v, CP), atol=1e-15)


@pytest.mark.parametrize("speed", [0.0, 0.01, 0.025, 0.026, 0.2, 0.225, 0.226, 0.4, 0.5074])
def test_the_band_choice_matches(speed):
    assert pf.band_of(speed, FP) == cert.band_index(cert.reach_speed(speed, CP),
                                                    CP.band_top_speeds)


# --------------------------------------------------------------------------- the sweep
@pytest.mark.parametrize("e", [0.0, 0.1, 1.2])
def test_the_closed_form_sweep_is_the_sampled_one(e):
    """The swept support against brute force: the hull rotated through 801 angles in [-e, e]. The
    closed form is the exact supremum, so it is never below the samples and above them only by the
    sampling gap."""
    n = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    exact = PFB.footprint.support(n, e)
    phis = np.linspace(-e, e, 801)
    sampled = np.max([(PFB.footprint.radius[None, :]
                       * np.cos(n[:, None] - phi - PFB.footprint.angle[None, :])).max(axis=1)
                      for phi in phis], axis=0)
    assert np.all(exact >= sampled - 1e-12)
    assert np.all(exact - sampled <= PFB.footprint.outer_radius * (1 - math.cos(e / 800)) + 1e-9)


@pytest.mark.parametrize("e", [0.0, 0.0315, 0.2, 0.9])
def test_the_sweep_matches_the_certified_derivation(e):
    n = CERT.normal_angles
    assert np.allclose(PFB.footprint.support(n, e), CERT.swept_interval(n, -e, e), atol=1e-12)


# --------------------------------------------------------------------------- the field
def test_the_band_fields_are_the_certified_fields():
    """PFB-(b) IS the certified trigger set's complement: at every band top, heading and yaw rate,
    the PFB's field equals the certified field (Q1, C1)."""
    for _ in range(60):
        heading = RNG.uniform(-math.pi, math.pi)
        w = RNG.choice([0.0, RNG.uniform(-1.5, 1.5)])
        band = int(RNG.integers(0, len(FP.band_tops)))
        top = FP.band_tops[band]
        assert np.allclose(PFB.support(CERT.normal_angles, top, heading, w),
                           CERT.support(CERT.normal_angles, top, heading, False, yaw_rate=w),
                           atol=1e-12)
        assert np.allclose(PFB.band_extent(BEARINGS, band, heading, w),
                           CERT.extent(BEARINGS, top, heading, False, yaw_rate=w), atol=1e-12)


def test_the_field_the_qp_uses_is_the_certified_field_too():
    """The QP's rows go through `support_and_partials` / `extent_and_partials`, not `support`; that
    path must be the certified field as well (found 2026-10-26: its turning-drift term used the
    sweep's E(|w|) + E(0) and was up to 6 mm larger). Every band, heading and yaw rate, and at rest;
    and its partials against central finite differences."""
    for _ in range(60):
        heading = RNG.uniform(-math.pi, math.pi)
        w = float(RNG.choice([0.0, RNG.uniform(-1.5, 1.5)]))
        band = int(RNG.integers(0, len(FP.band_tops)))
        top = FP.band_tops[band]
        h, d_head, d_w = PFB.support_and_partials(top, heading, w)
        assert np.allclose(h, CERT.support(CERT.normal_angles, top, heading, False, yaw_rate=w),
                           atol=1e-12)
        rho = PFB.extent_and_partials(BEARINGS, top, heading, w)[0]
        assert np.allclose(rho, CERT.extent(BEARINGS, top, heading, False, yaw_rate=w), atol=1e-12)
        step = 1e-6
        fd_head = (PFB.support(PFB.normals, top, heading + step, w)
                   - PFB.support(PFB.normals, top, heading - step, w)) / (2 * step)
        mag = abs(w)
        if 2 * step < mag < 1.5 - 2 * step:                        # |w| away from 0 and the cap
            fd_w = (PFB.support(PFB.normals, top, heading, mag + step)
                    - PFB.support(PFB.normals, top, heading, mag - step)) / (2 * step)
            smooth = np.abs(fd_w - d_w) < 1e-3                     # kinks of the maxima excepted
            assert smooth.mean() > 0.95
        smooth = np.abs(fd_head - d_head) < 1e-3
        assert smooth.mean() > 0.95
    for w in (0.0, 0.03, 0.06, 0.4):
        h, _, _ = PFB.support_and_partials(0.0, 0.0, w, at_rest=True)
        assert np.allclose(h, CERT.support(CERT.normal_angles, 0.0, 0.0, True, yaw_rate=w),
                           atol=1e-12)


@pytest.mark.parametrize("w", [0.0, 0.03, 0.06, 0.4])
def test_the_at_rest_field_is_the_certified_one(w):
    assert np.allclose(PFB.extent(BEARINGS, 0.0, 0.0, w, at_rest=True),
                       CERT.extent(BEARINGS, 0.0, 0.0, True, yaw_rate=w), atol=1e-12)


def test_the_continuous_field_never_exceeds_the_band_field():
    """PFB-(a), the field at the speed itself, lies inside the field of the band that speed selects:
    the gap between them is what (b) closes, and (a) is H3's ablation."""
    for _ in range(40):
        speed = RNG.uniform(0.0, FP.speed_cap)
        v = pf.reachable(speed, FP)
        band = pf.band_of(speed, FP)
        heading, w = RNG.uniform(-math.pi, math.pi), RNG.uniform(-1.0, 1.0)
        assert np.all(PFB.extent(BEARINGS, v, heading, w)
                      <= PFB.band_extent(BEARINGS, band, heading, w) + 1e-12)


def test_the_field_grows_with_speed_and_yaw_rate_and_contains_the_rest_field():
    heading = 0.3
    rest = PFB.extent(BEARINGS, 0.0, heading, 0.0, at_rest=True)
    prev = rest
    for v in (0.05, 0.15, 0.3, 0.5):
        cur = PFB.extent(BEARINGS, v, heading, 0.0)
        assert np.all(cur >= prev - 1e-12)
        prev = cur
    lo, hi = PFB.extent(BEARINGS, 0.3, heading, 0.1), PFB.extent(BEARINGS, 0.3, heading, 1.0)
    assert np.all(hi >= lo - 1e-12)


@pytest.mark.parametrize(("v", "w"), [(0.10, 0.0), (0.30, 0.4), (0.51, 1.0)])
def test_the_support_ahead_and_behind_are_the_known_sums(v, w):
    """Known answers from the construction itself. Straight ahead (c = 1): the swept footprint, the
    travel depth D(V), the turning drift and the forward margin. Straight behind (c = -1): the swept
    footprint, the swerve allowed backwards, the drift, the at-rest reach and the lateral margin. (The
    field's farthest point is NOT straight ahead: with a rectangular footprint it lies off-axis,
    where the front corners add to the depth -- the certified field's shape too.)"""
    heading = 0.7
    sweep = pf.heading_reach(w, FP) + pf.heading_reach(0.0, FP)
    n = np.array([heading, heading + math.pi])
    swept = PFB.footprint.support(n, sweep)
    k = pf.turning_drift(v, w, FP)
    ahead = swept[0] + pf.travel_depth(v, FP) + k + max(FP.margin - FP.lateral_margin,
                                                       pf.rest_creep_reach(FP)) + FP.lateral_margin
    behind = (swept[1] + float(pf.swerve_reach(np.array([-1.0]), v, FP)[0]) + k
              + pf.rest_creep_reach(FP) + FP.lateral_margin)
    got = PFB.support(n, v, heading, w)
    assert got[0] == pytest.approx(ahead, abs=1e-12)
    assert got[1] == pytest.approx(max(behind, float(PFB.rest_support(n[1:], w)[0])), abs=1e-12)


@pytest.mark.parametrize("e", [0.0, 1e-6, 0.0315, 0.5, 2.9, 3.2])
def test_the_fast_sweep_is_the_all_vertices_sweep(e):
    """The supporting-vertex search plus range-maximum table (WP2 step 2's speed fix: 297 -> 2 ms)
    against the closed form over every hull vertex."""
    n = PFB.normals
    assert np.allclose(PFB.footprint.support(n, e), PFB.footprint.support_all_vertices(n, e),
                       atol=1e-15)


def test_the_polygon_search_is_the_brute_force_minimum():
    """The active normal found through the boundary polygon (2026-10-24, the per-step budget) is the
    minimum over every facing normal: rho agrees to 1e-12 at arbitrary bearings over every regime,
    heading and yaw rate, the active normal agrees except where two give the same rho (a vertex),
    and no field needed the brute-force path."""
    field = pf.PFBField(TABLE.extent, FP)
    rng = np.random.default_rng(17)
    for _ in range(120):
        regime = int(rng.integers(-1, len(FP.band_tops)))
        heading, w = rng.uniform(-math.pi, math.pi), rng.uniform(0.0, 0.6)
        at_rest = regime < 0
        v = 0.0 if at_rest else FP.band_tops[regime]
        h, _, _ = field.support_and_partials(v, heading, w, at_rest)
        b = rng.uniform(-math.pi, math.pi, 200)
        k_fast = field._active_normal(b, h)
        k_ref = field._active_normal_brute(b, h)
        rho_fast = h[k_fast] / np.cos(b - field.normals[k_fast])
        rho_ref = h[k_ref] / np.cos(b - field.normals[k_ref])
        assert rho_fast == pytest.approx(rho_ref, abs=1e-12)
        assert np.mean(k_fast == k_ref) > 0.97
    assert field.brute_force_evaluations == 0


def test_a_support_that_is_not_a_polygon_falls_back_to_the_brute_force_minimum():
    """Positive control for the guard: one support value pulled in makes its neighbours' lines miss
    the boundary; the vertices are then not a valid polygon, the brute-force minimum answers, and
    the evaluation is counted."""
    field = pf.PFBField(TABLE.extent, FP)
    h, _, _ = field.support_and_partials(FP.band_tops[1], 0.3, 0.2)
    h = h.copy()
    h[100] -= 0.05
    b = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    k = field._active_normal(b, h)
    assert field.brute_force_evaluations == 1
    assert np.array_equal(k, field._active_normal_brute(b, h))
