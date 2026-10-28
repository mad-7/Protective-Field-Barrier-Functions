"""The cached field evaluation returns what the direct computation returns (hard rule 6).

`FieldGeometry` caches two things that cannot change once it is built: the footprint's
support on the support grid, and 1 / cos(psi - n) for each bearing grid it is asked for.
Both were rebuilt on every call, which is why the emulator cost 124 ms p95 per step --
more than the physics (`tests/test_step_loop_sim.py`, 2026-09-23).

Caching a safety layer's geometry is only acceptable if it changes no number, so this
compares against the arithmetic written out in full, at every band, at rest and moving,
over headings and bearing grids, and checks the cache cannot be written through or grow
without bound.
"""
import math

import numpy as np
import pytest

from pfbnav.safety.certified import (
    CertifiedParams,
    FieldGeometry,
    band_depth,
    coupling,
    rest_reach,
    swerve,
    yaw_interval,
)

PARAMS = CertifiedParams(
    response_time=0.10, a_brake=1.0, restart_delay=2.0, n_field_sets=3,
    margin=0.10, lateral_margin=0.05, standstill_enter=0.01, standstill_exit=0.02,
    band_top_speeds=(0.10, 0.30, 0.51), warning_scale=1.5, warning_extra=0.05,
    warning_extra_abeam=0.02, alpha_brake=1.4,
    # A20 (2026-10-18): the reachable-motion parameters, as configured
    reach_time=0.15, a_max=0.5, alpha_max=1.4, speed_cap=0.5074, yaw_cap=1.5,
    standstill_yaw_enter=0.0296, standstill_yaw_exit=0.0592)


def footprint(psi):
    """An asymmetric footprint: a rectangle with a bump, so nothing is hidden by symmetry."""
    p = np.asarray(psi, dtype=float)
    hx, hy = 0.2854, 0.2086
    rect = np.minimum(hx / np.maximum(np.abs(np.cos(p)), 1e-9),
                      hy / np.maximum(np.abs(np.sin(p)), 1e-9))
    return rect + 0.03 * np.exp(-((np.degrees(p) + 35.0) / 8.0) ** 2)


@pytest.fixture(scope="module")
def fields():
    return FieldGeometry(footprint, PARAMS)


def direct_h_fp(fields, angles):
    """The footprint's support straight from the boundary: no cache."""
    a = np.asarray(angles, dtype=float)
    n = np.column_stack([np.cos(a), np.sin(a)])
    return (fields._boundary @ n.T).max(axis=0)


def direct_support(fields, angles, band_top_speed, heading, at_rest):
    """The A20 support function written out (2026-10-18), using neither cache: the rotation sweep
    from the hull (`swept_interval`, which reads no cache for a non-empty interval), the reach
    terms from their formulas."""
    a = np.asarray(angles, dtype=float)
    lo, hi = yaw_interval(0.0, PARAMS)
    rest = fields.swept_interval(a, lo, hi) + rest_reach(PARAMS) + PARAMS.lateral_margin
    if at_rest:
        return rest
    c = np.cos(a - heading)
    lead = np.maximum(0.0, c)
    travel = np.maximum(band_depth(band_top_speed, PARAMS) * lead,
                        swerve(c, band_top_speed, PARAMS))
    # A21 (S2 nesting, 2026-10-19): the yaw interval carries e(0) more (+-2 e(0) at zero yaw rate),
    # the margin term is at least the at-rest reach, and the turning-while-translating drift.
    e = 2.0 * hi
    return np.maximum(fields.swept_interval(a, -e, e) + travel
                      + coupling(band_top_speed, 0.0, PARAMS)
                      + np.maximum((PARAMS.margin - PARAMS.lateral_margin) * lead,
                                   rest_reach(PARAMS))
                      + PARAMS.lateral_margin,
                      rest)


def direct_extent(fields, bearings, band_top_speed, heading, at_rest):
    """The extent, written out: rho(psi) = min over facing normals of h(n) / (n . u)."""
    b = np.asarray(bearings, dtype=float)
    h = direct_support(fields, fields.normal_angles, band_top_speed, heading, at_rest)
    cos = np.cos(b[:, None] - fields.normal_angles[None, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(cos > 1e-9, h[None, :] / cos, np.inf)
    return ratio.min(axis=1)


BEARING_GRIDS = {
    "filter_120": np.radians(np.arange(-180.0, 180.0, 3.0)),
    "policy_36": np.radians(np.arange(-180.0, 180.0, 10.0)),
    "odd_7": np.radians(np.array([-179.0, -91.3, -0.2, 0.0, 44.7, 91.3, 179.9])),
}


@pytest.mark.parametrize("grid", sorted(BEARING_GRIDS))
@pytest.mark.parametrize("band", PARAMS.band_top_speeds)
@pytest.mark.parametrize("heading_deg", [-180.0, -35.0, 0.0, 47.5, 179.0])
def test_the_moving_field_matches_the_direct_computation(fields, grid, band, heading_deg):
    b = BEARING_GRIDS[grid]
    got = fields.extent(b, band, math.radians(heading_deg), False)
    want = direct_extent(fields, b, band, math.radians(heading_deg), False)
    assert np.allclose(got, want, rtol=0.0, atol=1e-12)


@pytest.mark.parametrize("grid", sorted(BEARING_GRIDS))
def test_the_at_rest_field_matches_the_direct_computation(fields, grid):
    b = BEARING_GRIDS[grid]
    got = fields.extent(b, 0.0, 0.0, True)
    want = direct_extent(fields, b, 0.0, 0.0, True)
    assert np.allclose(got, want, rtol=0.0, atol=1e-12)


def test_the_footprint_support_matches_on_and_off_the_support_grid(fields):
    on = fields.footprint_support(fields.normal_angles)
    assert np.allclose(on, direct_h_fp(fields, fields.normal_angles), atol=1e-12)
    off = np.radians(np.array([-123.4, -0.7, 15.0, 88.8]))
    assert np.allclose(fields.footprint_support(off), direct_h_fp(fields, off), atol=1e-12)


def test_a_copy_of_the_support_grid_hits_the_cache_with_the_same_answer(fields):
    same = np.array(fields.normal_angles, copy=True)
    assert np.array_equal(fields.footprint_support(same),
                          fields.footprint_support(fields.normal_angles))


def test_the_cache_cannot_be_written_through(fields):
    first = fields.footprint_support(fields.normal_angles)
    first[:] = -99.0
    assert np.all(fields.footprint_support(fields.normal_angles) > 0.0)


def test_repeated_calls_on_one_grid_are_identical(fields):
    b = BEARING_GRIDS["filter_120"]
    a = fields.select(b, 0.25, 0.3, False)
    for _ in range(5):
        assert np.array_equal(fields.select(b, 0.25, 0.3, False), a)


def test_the_bearing_cache_is_bounded(fields):
    for k in range(12):
        fields.extent(np.radians(np.arange(-180.0, 180.0, 3.0)) + 1e-6 * k, 0.30, 0.0, False)
    assert len(fields._inv_cos) <= fields._inv_cos_max


def test_lateral_reach_is_unchanged_by_the_cache(fields):
    for band in PARAMS.band_top_speeds:
        for heading in (0.0, 1.0, -2.5):
            got = fields.lateral_reach(band, heading, False)
            want = float(direct_support(
                fields, np.array([heading + math.pi / 2, heading - math.pi / 2]),
                band, heading, False).max())
            assert got == pytest.approx(want, abs=1e-12)
