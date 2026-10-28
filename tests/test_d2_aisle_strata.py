"""D2 consistency test (WP1 item 1) on the certified field -- since A20 (2026-10-18) the field
sized for the motion REACHABLE before braking acts.

The field descends from the WP1 ruling (the footprint swept along the braking segment, `margin`
ahead, `lateral_margin` abeam and behind) and now contains the reach within T: the band from
v + a_max T, the yaw interval, the swerve an isotropic limiter allows, and at rest the isotropic
reach from the standstill speed. The narrow floor is F = 0.1735 m (A20). Known answers first, then the
requirements on the real configuration: measured footprint table, half_width 0.2086 (the
robot is 0.417 m wide), certified parameters, configured strata, and G1b's committed peak
cross-track read from preregistration/.

Scans are synthetic -- exact ranges to walls from the body origin -- so these test the
geometry of fields and strata, not a scanner model.
"""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.safety.certified import (
    CertifiedParams,
    FieldGeometry,
    StandstillDetector,
    band_depth,
    band_index,
    field_breached,
)
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.world.strata import (
    binned_radial_margin,
    certified_params_from_config,
    max_offset_at_rest,
    max_traversal_speed,
    moving_field_fits,
    narrow_floor,
    rest_clearance_floor,
    rest_offset_floor,
    sector_min_penalty,
    sector_min_range,
    strata_from_config,
    usable_band_top,
    wrap_to_pi,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
TABLE = FootprintTable.from_csv(REPO / CFG["robot"]["footprint_table"])
HALF_WIDTH = float(CFG["robot"]["half_width"])
PARAMS = certified_params_from_config(CFG)
FIELDS = FieldGeometry(TABLE.extent, PARAMS)
RANGE_MAX = float(CFG["scanners"]["range_max"])
MAX_CROSS_TRACK = float(yaml.safe_load((REPO / "preregistration" / "gates.yaml").read_text())
                        ["G1b_closed_loop_tracking"]["max_cross_track"])
SCAN = np.radians(np.arange(-180.0, 180.0, 0.25))


def _aisle_scan(clearance, offset=0.0):
    """Exact ranges from the body origin to walls at y = +(w - offset) and y = -(w + offset),
    w = half_width + clearance; `offset` moves the robot towards the +y wall."""
    w = HALF_WIDTH + clearance
    s = np.sin(SCAN)
    with np.errstate(divide="ignore"):
        r = np.where(s > 1e-12, (w - offset) / s,
                     np.where(s < -1e-12, (w + offset) / -s, np.inf))
    return np.minimum(r, RANGE_MAX), np.ones_like(r, dtype=bool)


def _circle(r):
    return lambda psi: np.full_like(np.asarray(psi, dtype=float), r)


P = CertifiedParams(response_time=0.1, a_brake=1.0, restart_delay=2.0, n_field_sets=3,
                    margin=0.1, lateral_margin=0.05, standstill_enter=0.01,
                    standstill_exit=0.02, band_top_speeds=(0.1, 0.3, 0.5),
                    warning_scale=1.5, warning_extra=0.05, warning_extra_abeam=0.02,
                    alpha_brake=1.4, reach_time=0.15, a_max=0.5, alpha_max=1.4,
                    speed_cap=0.5, yaw_cap=1.5,
    standstill_yaw_enter=0.0296, standstill_yaw_exit=0.0592)
CIRCLE = FieldGeometry(_circle(0.2), P)


# ---------------------------------------------------------------- field geometry
def test_band_selection_known_answers():
    assert band_index(0.0, P.band_top_speeds) == 0
    assert band_index(0.1, P.band_top_speeds) == 0        # top is inclusive
    assert band_index(0.1001, P.band_top_speeds) == 1
    assert band_index(0.5, P.band_top_speeds) == 2
    with pytest.raises(ValueError, match="exceeds"):
        band_index(0.51, P.band_top_speeds)


def test_band_depth_known_answer():
    """D(V) = V T + V^2 / (2 a_b), T = response 0.10 + one 0.05 control step (A20)."""
    assert band_depth(0.3, P) == pytest.approx(0.3 * 0.15 + 0.09 / 2)


def test_the_field_ahead_abeam_behind():
    """A disc of 0.2 m (rotation changes nothing), band topped at 0.30, A21 (S2 nesting).
    AHEAD: e + D + coupling + margin (the forward margin 0.05 beats the at-rest reach 13.1 mm, plus
    the lateral 0.05). BEHIND: e + swerve + coupling + at-rest reach + lateral margin -- the moving
    field contains the at-rest field at every reachable rest pose, a brief reversal included.
    ABEAM: the same on both sides, wider than the at-rest field."""
    from pfbnav.safety.certified import coupling, rest_reach, swerve
    d, cpl, rr = band_depth(0.3, P), coupling(0.3, 0.0, P), rest_reach(P)
    ahead, left, behind, right = CIRCLE.extent(np.array([0.0, math.pi / 2, math.pi,
                                                         -math.pi / 2]), 0.3, 0.0, False)
    assert ahead == pytest.approx(0.2 + d + cpl + 0.10, abs=2e-5)
    back = float(swerve(np.array([-1.0]), 0.3, P)[0])
    assert behind == pytest.approx(0.2 + back + cpl + rr + 0.05, abs=2e-5)
    assert left == pytest.approx(right, abs=1e-12)
    assert left > 0.2 + rr + 0.05


def test_oblique_bearings_lie_on_the_support_boundary():
    """The extent is the support function inverted on the emulator's grid of normals:
    rho(psi) = min over facing normals of h(n) / cos(psi - n). Checked against the same inversion
    on a grid fourteen times finer (`support` evaluated directly): the grid errs only toward a
    larger field, by less than half a millimetre."""
    fine = np.linspace(-math.pi, math.pi, 20160, endpoint=False)
    h = CIRCLE.support(fine, 0.5, 0.0, False)
    for psi in (math.pi / 4, 3 * math.pi / 4, -math.pi / 3, 1.9):
        c = np.cos(psi - fine)
        brute = float(np.min(h[c > 1e-9] / c[c > 1e-9]))
        got = CIRCLE.extent(np.array([psi]), 0.5, 0.0, False)[0]
        assert got == pytest.approx(brute, abs=5e-4)
        assert got >= brute - 1e-9        # discretisation errs towards a larger field


def test_the_field_turns_with_the_travel_direction():
    from pfbnav.safety.certified import coupling
    d = band_depth(0.3, P)
    left = CIRCLE.extent(np.array([math.pi / 2]), 0.3, math.pi / 2, False)[0]
    assert left == pytest.approx(0.2 + d + coupling(0.3, 0.0, P) + 0.10, abs=2e-5)


def test_rearward_travel_puts_the_depth_behind():
    """Travelling -x, the depth and `margin` lead toward -x; the body's +x side trails with the
    reversal reach, the drift and the at-rest reach and margin (A21)."""
    from pfbnav.safety.certified import coupling, rest_reach, swerve
    d, cpl = band_depth(0.3, P), coupling(0.3, 0.0, P)
    behind, ahead = CIRCLE.extent(np.array([math.pi, 0.0]), 0.3, math.pi, False)
    assert behind == pytest.approx(0.2 + d + cpl + 0.10, abs=2e-5)
    back = float(swerve(np.array([-1.0]), 0.3, P)[0])
    assert ahead == pytest.approx(0.2 + back + cpl + rest_reach(P) + 0.05, abs=2e-5)


def test_rest_field_is_the_reach_and_the_lateral_margin_all_round():
    """The at-rest field is the INFIMUM of the moving fields (supervisor 2026-09-27): the footprint
    grown by the isotropic reach from the standstill speed (A20; 13.1 mm at a_max 0.5) and the
    lateral margin, all round. A 0.2 m circle therefore reaches 0.2631."""
    from pfbnav.safety.certified import rest_reach
    assert rest_reach(P) == pytest.approx(0.02 * 0.15 + 0.5 * 0.5 * 0.0225 + 0.095 ** 2 / 2)
    want = 0.2 + rest_reach(P) + 0.05
    assert np.allclose(CIRCLE.extent(SCAN, 0.0, 0.0, True), want, atol=2e-5)
    assert np.allclose(CIRCLE.select(SCAN, 0.0, 1.0, at_rest=True), want, atol=2e-5)


def test_select_uses_the_band_the_reachable_speed_selects():
    """0.2 m/s reaches 0.275 within T: band 1 (top 0.30); 0.24 reaches 0.315: band 2 (A20)."""
    got = CIRCLE.select(np.array([0.0]), 0.2, 0.0, at_rest=False)
    assert got == pytest.approx(CIRCLE.extent(np.array([0.0]), 0.3, 0.0, False))
    got = CIRCLE.select(np.array([0.0]), 0.24, 0.0, at_rest=False)
    assert got == pytest.approx(CIRCLE.extent(np.array([0.0]), 0.5, 0.0, False))


@pytest.mark.parametrize("speed", [0.10, 0.30, 0.51])
def test_the_sideways_reach_grows_with_the_band_and_starts_at_the_at_rest_field(speed):
    """Since A20 the sideways reach GROWS with the band: an isotropic limiter can swerve, and the
    faster band brakes the swerve longer (it was constant, e(90) + lateral_margin, under the
    WP1 field). Every band contains the at-rest field (the infimum; since A21 strictly, by the
    nesting terms), and every band is the support toward the side."""
    for heading in (0.0, 0.7, math.pi):
        reach = FIELDS.lateral_reach(speed, heading, at_rest=False)
        rest = FIELDS.lateral_reach(0.0, heading, at_rest=True)
        assert reach >= rest - 1e-12
        sides = np.array([heading + math.pi / 2, heading - math.pi / 2])
        assert reach == pytest.approx(float(FIELDS.support(sides, speed, heading, False).max()))
        slower = [t for t in PARAMS.band_top_speeds if t < speed]
        if slower:
            assert reach >= FIELDS.lateral_reach(max(slower), heading, False) - 1e-12
    # Since A21 band 0 exceeds the at-rest field sideways too (nesting): 296.5 against 279.5 mm.
    assert FIELDS.lateral_reach(PARAMS.band_top_speeds[0], 0.0, False) == pytest.approx(
        0.29649, abs=1e-4)
    assert FIELDS.lateral_reach(0.0, 0.0, True) == pytest.approx(0.27949, abs=1e-4)


def test_an_invalid_bin_breaches_the_field():
    """Hard rule 5: unobserved space is never free."""
    r, v = _aisle_scan(1.0)
    v[10] = False
    assert field_breached(r, v, FIELDS.extent(SCAN, 0.0, 0.0, True))


def test_parameters_are_validated():
    with pytest.raises(ValueError, match="ascending"):
        CertifiedParams(0.1, 1.0, 2.0, 2, 0.1, 0.05, 0.01, 0.02, (0.3, 0.3), 1.5, 0.05, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)
    with pytest.raises(ValueError, match="lateral_margin"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.2, 0.01, 0.02, (0.3,), 1.5, 0.05, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)
    with pytest.raises(ValueError, match="standstill"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.05, 0.02, 0.01, (0.3,), 1.5, 0.05, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)


def test_the_warning_field_may_not_be_smaller_than_or_equal_to_the_protective_field():
    """A warning field that is not strictly larger can never warn first, so it is an error
    rather than a setting (supervisor 2026-09-26)."""
    with pytest.raises(ValueError, match="smaller"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.05, 0.01, 0.02, (0.3,), 0.9, 0.05, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)
    with pytest.raises(ValueError, match="smaller"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.05, 0.01, 0.02, (0.3,), 1.5, -0.01, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)
    # Abeam, the warning field equals the protective field unless the abeam extension is
    # positive, and then it cannot warn before it (2026-09-28, D2).
    with pytest.raises(ValueError, match="never warn"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.05, 0.01, 0.02, (0.3,), 1.5, 0.05, 0.0, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)
    # A warning field that reached less far ahead than to the side would be backwards.
    with pytest.raises(ValueError, match="ahead"):
        CertifiedParams(0.1, 1.0, 2.0, 1, 0.1, 0.05, 0.01, 0.02, (0.3,), 1.5, 0.01, 0.02, 3.0, 0.15, 0.5, 1.4, 0.3, 1.5, 0.0296, 0.0592)


# ------------------------------- D2, 2026-09-28: the warning field's abeam extension, derived
def test_the_abeam_warning_extension_is_strictly_inside_the_narrow_gap():
    """The ruled relation, derived from the geometry rather than restated as 29.1 mm.

    In the narrowest aisle, with the robot at the committed 50 mm tracking bound, the gap
    between the protective field's abeam reach and the near wall is what the warning field has
    to fit into. With the old 0.05 extension all round it did not: the probe measured the robot
    inside the warning field for 170 of 170 steps, so the near-miss metric was saturated there.
    """
    narrow = min(s.min_clearance for s in strata_from_config(CFG))
    top = PARAMS.band_top_speeds[-1]
    wall = HALF_WIDTH + narrow - MAX_CROSS_TRACK
    gap = wall - FIELDS.lateral_reach(top, 0.0, False)
    # A21 (2026-10-19): 39.6 mm at F = 0.1987 under the nesting field, top band (41.0 mm at
    # 0.1735 under A20; 29.1 mm at 0.13 under the WP1 field).
    assert gap == pytest.approx(0.0396, abs=1e-4), "the 39.6 mm gap has moved"
    assert PARAMS.warning_extra_abeam < gap, (
        f"warning_extra_abeam {PARAMS.warning_extra_abeam} does not fit the {gap:.4f} m gap")
    # And so the warning field no longer covers the wall at the tracking bound.
    abeam = np.array([math.pi / 2])
    reach = float(FIELDS.support(abeam, top, 0.0, False, warning=True)[0])
    assert reach < wall


def test_the_warning_field_ahead_is_unchanged_by_the_abeam_ruling():
    """D2 kept the extension AHEAD as it was: depth scale x D + warning_extra, and warning_extra
    again as margin -- only abeam and behind changed. Under A20 the footprint term is the swept
    footprint (the yaw interval from rest) and D the reach-time depth."""
    from pfbnav.safety.certified import yaw_interval
    top = PARAMS.band_top_speeds[-1]
    ahead = np.array([0.0])
    from pfbnav.safety.certified import coupling
    lo, hi = yaw_interval(0.0, PARAMS)
    h = float(FIELDS.swept_interval(ahead, 2 * lo, 2 * hi)[0])     # A21: +-2 e(0)
    was = (h + PARAMS.warning_scale * band_depth(top, PARAMS) + PARAMS.margin
           + 2 * PARAMS.warning_extra + coupling(top, 0.0, PARAMS))
    assert float(FIELDS.support(ahead, top, 0.0, False, warning=True)[0]) == pytest.approx(was)


# ---------------------------------------------------------------- standstill hysteresis
def test_standstill_hysteresis_and_transition_log():
    det = StandstillDetector(P)
    speeds = [0.0, 0.015, 0.025, 0.015, 0.012, 0.009, 0.015, 0.03]
    states = [det.update(v) for v in speeds]
    # Starts at rest; 0.015 is inside the band, so still at rest; 0.025 leaves rest;
    # 0.015 and 0.012 stay moving; 0.009 comes to rest; 0.015 stays at rest; 0.03 moves.
    assert states == [True, True, False, False, False, True, True, False]
    assert det.transitions == [(2, "moving"), (5, "rest"), (7, "moving")]
    det.reset()
    assert det.at_rest and det.transitions == []


# ---------------------------------------------------------------- D2 on the real config
def test_the_robot_is_0417_m_wide():
    assert 2 * HALF_WIDTH == pytest.approx(0.4172)


def test_narrow_floor_follows_a21():
    """A21 (2026-10-19): F = the smallest clearance with the three D2 gaps >= 0 (binned) under the
    nesting field, 0.1687, plus the recorded 0.03 buffer (`scripts/compute_narrow_floor.py`). It
    was 0.1735 under A20; ruling 4's formula (0.13) no longer binds."""
    narrowest = min(strata_from_config(CFG), key=lambda s: s.min_clearance)
    assert narrowest.min_clearance == pytest.approx(0.1987)
    assert narrowest.min_clearance > narrow_floor(PARAMS.lateral_margin, MAX_CROSS_TRACK,
                                                  float(CFG["world"]["narrow_floor_slack"]))
    names = {s.name: (s.min_clearance, s.max_clearance) for s in strata_from_config(CFG)}
    assert names["narrow"] == pytest.approx((0.1987, 0.2687))
    assert names["medium"] == pytest.approx((0.2687, 0.40))
    assert names["wide"] == pytest.approx((0.40, 0.80))


def test_narrow_floor_exceeds_lateral_margin_plus_max_cross_track():
    """Ruling 4's test."""
    narrowest = min(s.min_clearance for s in strata_from_config(CFG))
    assert narrowest > PARAMS.lateral_margin + MAX_CROSS_TRACK


def test_the_rest_floor_is_a_knife_edge():
    floor = rest_clearance_floor(FIELDS, HALF_WIDTH)
    field = FIELDS.extent(SCAN, 0.0, 0.0, True)
    above, v = _aisle_scan(floor + 5e-4)
    below, _ = _aisle_scan(floor - 5e-4)
    assert not field_breached(above, v, field)
    assert field_breached(below, v, field)


def test_a_stationary_robot_centred_in_the_narrowest_aisle_triggers_no_stop():
    """D2's required test."""
    narrowest = min(strata_from_config(CFG), key=lambda s: s.min_clearance)
    scan, valid = _aisle_scan(narrowest.min_clearance)
    assert not field_breached(scan, valid, FIELDS.select(SCAN, 0.0, 0.0, at_rest=True))


def test_every_stratum_clears_the_rest_floor_strictly():
    floor = rest_clearance_floor(FIELDS, HALF_WIDTH)
    for stratum in strata_from_config(CFG):
        assert stratum.min_clearance > floor


def test_rest_floor_as_recorded():
    """0.0709 on the S300 footprint under A20 (2026-10-18): the swept footprint's sideways
    extent + the reach from the standstill speed (13.1 mm) + lateral_margin 0.05, less half_width.
    It was 0.0509 under the WP1 field, and 0.1009 while the at-rest field wrongly carried `margin`
    0.10 all round
    (fixed 2026-09-27), and 0.1007 before that on the pod footprint."""
    assert rest_clearance_floor(FIELDS, HALF_WIDTH) == pytest.approx(0.0709, abs=1e-4)


def test_traversal_speed_bisection_known_answer():
    """Disc of 0.2 m, half-width 0.2 (A21). Below about 0.0796 of clearance not even band 0's
    sideways reach fits; above it the sideways reach grows with the band top (swerve and drift),
    so the bisection finds an intermediate top at 0.08 and 0.09 m; at 0.10 every band fits."""
    assert max_traversal_speed(CIRCLE, 0.2, 0.06) == 0.0
    assert max_traversal_speed(CIRCLE, 0.2, 0.08) == pytest.approx(0.24393, abs=1e-4)
    assert max_traversal_speed(CIRCLE, 0.2, 0.09) == pytest.approx(0.43302, abs=1e-4)
    assert max_traversal_speed(CIRCLE, 0.2, 0.10) == pytest.approx(0.5)
    assert moving_field_fits(0.5, CIRCLE, 0.2, 0.10)


def test_usable_band_top():
    assert usable_band_top(0.45, P) == pytest.approx(0.3)
    assert usable_band_top(0.5, P) == pytest.approx(0.5)
    assert usable_band_top(0.08, P) is None


# Traversal speeds (ruling 3), re-derived on the Minkowski field and logged in the
# decision log. Pinned: a change to the fields, footprint or strata that moves them fails
# here until the log is updated.
# Re-derived 2026-09-20 on v_max 0.40 (the G1b re-run under the wheel budget): the top
# band moved 0.50 -> 0.40, so every stratum's traversal speed moves with it. The strata
# themselves did not move: the Minkowski field's sideways reach does not grow with speed,
# so each aisle still admits its top band rather than a speed the geometry picks.
# Re-derived 2026-10-18 under A20 (bands 0.10 / 0.30 / 0.51; the reach field; narrow at F): every
# stratum admits the top band, centred and at the tracking offset -- the narrow floor was chosen
# so (scripts/compute_narrow_floor.py), with 40 mm binned to spare at the offset.
TRAVERSAL = {  # stratum: (centred v*, band; at the 0.05 m tracking offset v*, band)
    "narrow": (0.51, 0.51, 0.51, 0.51),
    "medium": (0.51, 0.51, 0.51, 0.51),
    "wide": (0.51, 0.51, 0.51, 0.51),
}


@pytest.mark.parametrize("name", sorted(TRAVERSAL))
def test_traversal_speed_per_stratum_as_recorded(name):
    stratum = next(s for s in strata_from_config(CFG) if s.name == name)
    centred = max_traversal_speed(FIELDS, HALF_WIDTH, stratum.min_clearance)
    offset = max_traversal_speed(FIELDS, HALF_WIDTH, stratum.min_clearance,
                                 offset=MAX_CROSS_TRACK)
    v0, band0, v1, band1 = TRAVERSAL[name]
    assert centred == pytest.approx(v0, abs=2e-4)
    assert usable_band_top(centred, PARAMS) == band0
    assert offset == pytest.approx(v1, abs=2e-4)
    assert usable_band_top(offset, PARAMS) == band1


def test_narrow_margin_at_the_tracking_offset_as_recorded():
    """The near-side gap between the moving field and the wall, at the committed tracking offset in
    the narrowest aisle, m: 0.0396 under A21 at F = 0.1987, top band (0.0410 at 0.1735 under A20;
    0.0291 at 0.13 under the WP1 field)."""
    narrowest = min(s.min_clearance for s in strata_from_config(CFG))
    top = PARAMS.band_top_speeds[-1]
    gap = (HALF_WIDTH + narrowest - MAX_CROSS_TRACK) - FIELDS.lateral_reach(top, 0.0, False)
    assert gap == pytest.approx(0.0396, abs=1e-4)


def test_the_narrowest_aisle_admits_the_top_band_in_a_real_scan():
    narrowest = min(strata_from_config(CFG), key=lambda s: s.min_clearance)
    for offset in (0.0, MAX_CROSS_TRACK):
        scan, valid = _aisle_scan(narrowest.min_clearance, offset)
        assert not field_breached(
            scan, valid, FIELDS.select(SCAN, PARAMS.band_top_speeds[-1], 0.0, at_rest=False))


# ------------------------------------------------- Sector-min binning (supervisor 2026-09-26)
# The scan the layer receives reports the NEAREST point in each sector, so the derivation has
# to name what that costs instead of hiding it in the slack.
FILTER_BINS = int(CFG["scanners"]["filter_bins"])
BIN_WIDTH = 2 * math.pi / FILTER_BINS
SCAN_BINS = -math.pi + BIN_WIDTH * (np.arange(FILTER_BINS) + 0.5)


def test_wrap_to_pi_known_answers():
    got = wrap_to_pi([0.0, math.pi, -math.pi, 3 * math.pi, -3 * math.pi / 2])
    assert got == pytest.approx([0.0, math.pi, math.pi, math.pi, math.pi / 2])


def test_a_bin_straddling_the_normal_reads_the_perpendicular_distance():
    """Known answer: the nearest point of the wall is the foot of the perpendicular, so a
    sector containing the wall's normal reports exactly the wall distance."""
    got = sector_min_range([math.pi / 2], 0.4, math.pi / 2, BIN_WIDTH)
    assert got[0] == pytest.approx(0.4)


def test_a_bin_beside_the_normal_reads_its_nearer_edge():
    """Known answer: bin centred one full width off the normal, so its near edge is at
    half a width; the range is d / cos(half width), not d / cos(a full width)."""
    centre = math.pi / 2 + BIN_WIDTH
    got = sector_min_range([centre], 0.4, math.pi / 2, BIN_WIDTH)
    assert got[0] == pytest.approx(0.4 / math.cos(BIN_WIDTH / 2))
    # ... and strictly nearer than the bin-centre range, which is what trips a stop early.
    assert got[0] < 0.4 / math.cos(BIN_WIDTH)


def test_bins_facing_away_from_the_wall_see_nothing():
    assert math.isinf(sector_min_range([-math.pi / 2], 0.4, math.pi / 2, BIN_WIDTH)[0])


def test_the_sector_min_penalty_is_positive_and_shrinks_as_the_aisle_widens():
    """It costs margin (the scan reads short), and it costs less the further the wall is."""
    penalties = [sector_min_penalty(FIELDS, SCAN_BINS, HALF_WIDTH + c, 0.0, 0.0, True,
                                    BIN_WIDTH)
                 for c in (0.12, 0.13, 0.20, 0.40)]
    assert all(p > 0 for p in penalties)
    assert penalties == sorted(penalties, reverse=True)


def test_the_sector_min_penalty_as_recorded():
    """Pinned at rest, at the narrow floor and 10 mm above it: 0.34 / 0.33 mm under A20 (it was
    0.43 / 0.37 mm at 0.12 / 0.13 under the WP1 field). The term the ruling named, DERIVED
    exactly; a fraction of what the real scan eats, which is why the buffer is asserted against the
    MEASURED total."""
    narrow = min(s.min_clearance for s in strata_from_config(CFG))
    at_floor = sector_min_penalty(FIELDS, SCAN_BINS, HALF_WIDTH + narrow, 0.0, 0.0, True,
                                  BIN_WIDTH)
    above = sector_min_penalty(FIELDS, SCAN_BINS, HALF_WIDTH + narrow + 0.01, 0.0, 0.0, True,
                               BIN_WIDTH)
    assert at_floor == pytest.approx(0.00034, abs=5e-5)
    assert above == pytest.approx(0.00033, abs=5e-5)
    # The measured scan penalty is about 1 mm (2026-09-27); the buffer dominates it.
    measured_scan_penalty_m = 0.001
    assert float(CFG["world"]["narrow_floor_slack"]) > 3 * measured_scan_penalty_m


def test_the_binned_margin_as_geometry_predicts_it():
    """The geometry side of the comparison, checkable without the simulator: at the narrow floor,
    at rest and centred, 127.5 mm; the top band centred 89.1 mm, at the offset 38.5 mm (A21)."""
    narrow = min(s.min_clearance for s in strata_from_config(CFG))
    top = PARAMS.band_top_speeds[-1]
    got = [binned_radial_margin(FIELDS, SCAN_BINS, HALF_WIDTH + narrow - off, band, 0.0, rest,
                                BIN_WIDTH)
           for off, band, rest in ((0.0, 0.0, True), (0.0, top, False),
                                   (MAX_CROSS_TRACK, top, False))]
    assert got == pytest.approx([0.1275, 0.0891, 0.0385], abs=5e-4)


# Measured against the real scan packet at the TRUE 0.13 m narrow floor, the wall FACES placed
# at the clearance (`pytest -m sim tests/test_d2_real_scan.py -s`, 2026-09-28, at the freeze
# commit). These are MEASUREMENTS: hard rule 15 forbids replacing them with what the model says,
# and they are what the model is judged against.
#
# History, because each earlier figure was wrong in a different way: 9.5 mm (2026-09-23) was
# taken against the old, wider at-rest field and compared with the wall centre; 69.5 / 19.4 mm
# (2026-09-27) were right about the field but measured an aisle 10 mm too narrow, the harness
# having placed each wall's centre at the clearance. The true-floor figures are exactly 10.0 mm
# larger than those, in all three cases -- the half-thickness the harness was losing.
# RE-MEASURED 2026-10-20 under A21 + A22, at the 0.1987 m narrow floor, wall faces at the clearance
# (`pytest -m sim tests/test_d2_real_scan.py -s` at 6207c7d; only the decision log uncommitted).
# The WP1 figures at 0.13 m were 79.5 / 79.5 / 29.4 mm. The at-rest and the centred moving margin
# no longer coincide: A20/A21's moving field reaches further abeam than the at-rest field (the
# swerve, the drift, the nesting terms), so the at-rest field is now STRICTLY inside it there.
MEASURED_MARGIN_AT_REST_M = 0.1283          # centred, at rest
MEASURED_MARGIN_TOP_BAND_CENTRED_M = 0.0901  # centred, moving at the top band
MEASURED_MARGIN_TOP_BAND_OFFSET_M = 0.0401   # displaced by the committed peak cross-track


def test_the_at_rest_field_is_strictly_inside_the_moving_one_abeam_as_measured():
    """The at-rest field is the infimum of the moving fields. Under WP1 the two touched abeam (79.5
    mm both, 2026-09-28); under A20/A21 the moving field reaches further abeam, and the scan says
    so: 128.3 mm standing, 90.1 mm centred at the top band. The geometry's lateral reaches differ
    by the same amount, to the scan's +-2 mm."""
    assert MEASURED_MARGIN_AT_REST_M > MEASURED_MARGIN_TOP_BAND_CENTRED_M
    top = PARAMS.band_top_speeds[-1]
    gap = FIELDS.lateral_reach(top, 0.0, False) - FIELDS.lateral_reach(0.0, 0.0, True)
    assert (MEASURED_MARGIN_AT_REST_M - MEASURED_MARGIN_TOP_BAND_CENTRED_M) == pytest.approx(
        gap, abs=0.002)


@pytest.mark.parametrize(("label", "offset", "speed", "at_rest", "measured"), [
    ("at rest, centred", 0.0, 0.0, True, MEASURED_MARGIN_AT_REST_M),
    ("top band, centred", 0.0, None, False, MEASURED_MARGIN_TOP_BAND_CENTRED_M),
    ("top band, offset", MAX_CROSS_TRACK, None, False, MEASURED_MARGIN_TOP_BAND_OFFSET_M),
])
def test_the_binned_model_is_conservative_against_the_real_scan(label, offset, speed, at_rest,
                                                                measured):
    """The model must never promise MORE margin than the simulator delivers.

    That is the only direction that matters: a model that over-states the margin would let a
    stratum be declared admissible on geometry the scan does not support. Under-stating is safe.

    Measured 2026-10-20 at the 0.1987 m floor (A21): 128.3 mm at rest, 90.1 mm centred at the top
    band, 40.1 mm at the committed peak cross-track (2026-09-28 at 0.13 m: 79.5 / 79.5 / 29.4 mm).
    The model (127.5 / 89.1 / 38.5 mm) lands below every one. The
    gap is not error in the ranges -- abeam the model matches to +-0.5 mm -- but in where the
    minimum sits: the model treats each wall as an infinite line, and its minimum wanders to
    oblique bearings (+34.5 deg in the offset case) where a finite wall seen from corner-mounted
    sensors reads further away than an infinite one. Hence conservative, and hence a bound rather
    than a prediction.
    """
    narrow = min(s.min_clearance for s in strata_from_config(CFG))
    face = HALF_WIDTH + narrow - offset
    band = PARAMS.band_top_speeds[-1] if speed is None else speed
    modelled = binned_radial_margin(FIELDS, SCAN_BINS, face, band, 0.0, at_rest, BIN_WIDTH)
    assert modelled <= measured + 1e-9, (
        f"{label}: the model promises {modelled * 1000:.2f} mm where the scan measured "
        f"{measured * 1000:.2f} mm; a model that over-states the margin is unsafe")
    assert measured - modelled < 0.005, (
        f"{label}: the model is {(measured - modelled) * 1000:.2f} mm below the measurement, "
        "wider than the gap recorded on 2026-09-28; the scan or the geometry has moved")


def test_the_measured_margins_clear_the_real_scan_in_both_states():
    """D2's requirement on the corrected field, re-measured as the 2026-09-27 ruling directed:
    positive margin at rest AND moving, centred AND at the tracking bound, at the true floor."""
    for margin in (MEASURED_MARGIN_AT_REST_M, MEASURED_MARGIN_TOP_BAND_CENTRED_M,
                   MEASURED_MARGIN_TOP_BAND_OFFSET_M):
        assert margin > 0.0
    # The tracking bound costs 50 mm of the moving margin (90.1 -> 40.1 mm): the narrow floor's own
    # arithmetic, confirmed by the scan rather than by the geometry that set it. (Under WP1 the
    # at-rest and moving margins coincided, so the at-rest one stood in; now the moving one binds.)
    assert (MEASURED_MARGIN_TOP_BAND_CENTRED_M - MEASURED_MARGIN_TOP_BAND_OFFSET_M) == pytest.approx(
        MAX_CROSS_TRACK, abs=0.002)


# ------------------------------------- The at-rest field is the binding case, not the moving one
def test_the_at_rest_field_stays_the_infimum_and_the_floor_clears_both():
    """The at-rest field stays the infimum of the moving fields (2026-09-27), so a stop taken
    anywhere the controller may be can be cleared. Under A21 the moving requirement at the tracking
    offset (the top band, 0.1591 m) exceeds the at-rest one (0.1209 m) -- the swerve, the drift and
    the nesting terms -- and the narrow floor F = 0.1987 clears both with the buffer."""
    narrow = min(s.min_clearance for s in strata_from_config(CFG))
    at_rest = rest_offset_floor(FIELDS, HALF_WIDTH, MAX_CROSS_TRACK)
    top = PARAMS.band_top_speeds[-1]
    moving = FIELDS.lateral_reach(top, 0.0, False) - HALF_WIDTH + MAX_CROSS_TRACK
    assert at_rest == pytest.approx(0.12089, abs=1e-4)
    assert moving == pytest.approx(0.15914, abs=1e-4)
    assert at_rest <= moving + 1e-12, "the at-rest field is no longer the infimum"
    assert narrow > moving, "the narrow floor no longer clears the moving requirement"
    assert max_traversal_speed(FIELDS, HALF_WIDTH, narrow, offset=MAX_CROSS_TRACK) > 0.0


def test_how_far_off_centre_a_robot_may_stop_per_stratum():
    """Known answer: half_width + clearance - at-rest reach. Every stratum now admits a stop
    anywhere the controller is permitted to be; narrow admitted only 29 mm against the 50 mm
    permitted while the at-rest field was wrongly the larger one."""
    admitted = {s.name: max_offset_at_rest(FIELDS, HALF_WIDTH, s.min_clearance)
                for s in strata_from_config(CFG)}
    # A21 (2026-10-19), at the new strata (the at-rest field is unchanged by A21)
    assert admitted["narrow"] == pytest.approx(0.1278, abs=1e-4)
    assert admitted["medium"] == pytest.approx(0.1978, abs=1e-4)
    assert admitted["wide"] == pytest.approx(0.3291, abs=1e-4)
    for name, value in admitted.items():
        assert value > MAX_CROSS_TRACK, f"{name} cannot absorb a stop at the tracking bound"
