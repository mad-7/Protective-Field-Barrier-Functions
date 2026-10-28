"""Known-answer tests for the scan packet, on synthetic geometry (hard rule 6).

No simulator: the packet is a pure format, and these fix it before `lua/scanner_v2.lua`
is written to satisfy it. The geometry cases have answers that can be worked out by hand
-- a wall at a known distance, a corner, a sector nothing covered.
"""
import math

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.scan_packet import (
    FORMAT_VERSION,
    HEADER_LEN,
    ScanPacket,
    assert_current,
    bin_bearings,
    decode_scan,
    encode_scan,
)

CFG = load_config("configs/base.yaml")
POLICY = int(CFG["scanners"]["policy_bins"])
FILTER = int(CFG["scanners"]["filter_bins"])
R_MIN = float(CFG["scanners"]["range_min"])
R_MAX = float(CFG["scanners"]["range_max"])
DT = float(CFG["sim"]["dt"])


def wall_ranges(bearings, distance, *, range_max=R_MAX):
    """Exact range to a wall parallel to +x at y = +`distance`, along each bearing.

    A bearing pointing away from the wall sees nothing, which is a measurement of free
    space out to `range_max`, not an absence of measurement.
    """
    b = np.asarray(bearings, dtype=float)
    s = np.sin(b)
    with np.errstate(divide="ignore"):
        r = np.where(s > 1e-12, distance / s, np.inf)
    return np.minimum(r, range_max)


def packet(sim_time=1.0, policy_ranges=None, policy_valid=None, filter_bearings=None,
           filter_ranges=None, filter_valid=None, range_min=R_MIN, range_max=R_MAX):
    fb = bin_bearings(FILTER) if filter_bearings is None else filter_bearings
    pr = np.full(POLICY, R_MAX) if policy_ranges is None else policy_ranges
    fr = np.full(FILTER, R_MAX) if filter_ranges is None else filter_ranges
    pv = np.ones(POLICY, bool) if policy_valid is None else policy_valid
    fv = np.ones(FILTER, bool) if filter_valid is None else filter_valid
    return encode_scan(sim_time, pr, pv, fb, fr, fv,
                       range_min=range_min, range_max=range_max)


def decode(values):
    return decode_scan(values, policy_bins=POLICY, filter_bins=FILTER,
                       range_min=R_MIN, range_max=R_MAX)


# ----------------------------------------------------------------------
# Layout
# ----------------------------------------------------------------------
def test_the_packet_is_the_size_the_configuration_implies():
    assert len(packet()) == HEADER_LEN + 2 * POLICY + 3 * FILTER == 6 + 72 + 360


def test_round_trip_preserves_every_field():
    fb = bin_bearings(FILTER)
    pr = wall_ranges(bin_bearings(POLICY), 1.25)
    fr = wall_ranges(fb, 1.25)
    pv = np.ones(POLICY, bool)
    fv = np.ones(FILTER, bool)
    p = decode(encode_scan(2.5, pr, pv, fb, fr, fv, range_min=R_MIN, range_max=R_MAX))
    assert isinstance(p, ScanPacket)
    assert p.sim_time == pytest.approx(2.5)
    assert p.policy_ranges == pytest.approx(pr)
    assert p.filter_ranges == pytest.approx(fr)
    assert p.filter_bearings == pytest.approx(fb)
    assert p.policy_valid.all() and p.filter_valid.all() and p.fully_observed


def test_decoded_arrays_are_read_only():
    p = decode(packet())
    with pytest.raises(ValueError):
        p.policy_ranges[0] = 0.0


# ----------------------------------------------------------------------
# Bearings
# ----------------------------------------------------------------------
def test_policy_bearings_are_sector_centres_and_are_not_transmitted():
    p = decode(packet())
    b = p.policy_bearings
    assert b.size == POLICY
    # 36 bins over a full turn: 10 deg each, first centred 5 deg past -180.
    assert math.degrees(b[0]) == pytest.approx(-175.0)
    assert math.degrees(b[18]) == pytest.approx(5.0)
    assert np.allclose(np.diff(b), math.radians(10.0))


def test_bearings_lie_in_the_half_open_turn():
    b = bin_bearings(FILTER)
    assert b.min() > -math.pi and b.max() <= math.pi


def test_filter_bearings_must_ascend():
    fb = bin_bearings(FILTER).copy()
    fb[5], fb[6] = fb[6], fb[5]
    with pytest.raises(ValueError, match="ascending"):
        decode(packet(filter_bearings=fb))


def test_filter_bearings_outside_the_turn_are_refused():
    fb = bin_bearings(FILTER) + 2 * math.pi
    with pytest.raises(ValueError, match=r"\(-pi, pi\]"):
        decode(packet(filter_bearings=fb))


# ----------------------------------------------------------------------
# Geometry: known answers
# ----------------------------------------------------------------------
def test_a_wall_abeam_reads_its_perpendicular_distance_at_ninety_degrees():
    fb = bin_bearings(FILTER)
    p = decode(packet(filter_ranges=wall_ranges(fb, 0.8)))
    i = int(np.argmin(np.abs(fb - math.pi / 2)))
    # 120 bins: the nearest bearing to +90 deg is 1.5 deg off, so the range is
    # 0.8 / sin(88.5 deg), not 0.8 exactly -- the geometry, not a tolerance.
    assert p.filter_ranges[i] == pytest.approx(0.8 / math.sin(fb[i]))
    assert p.filter_ranges[i] == pytest.approx(0.8, abs=5e-4)


def test_bearings_pointing_away_from_the_wall_are_free_to_range_max():
    fb = bin_bearings(FILTER)
    p = decode(packet(filter_ranges=wall_ranges(fb, 0.8)))
    ahead = int(np.argmin(np.abs(fb)))           # +x, parallel to the wall
    behind = int(np.argmin(np.abs(np.abs(fb) - math.pi)))
    assert p.filter_ranges[ahead] == pytest.approx(R_MAX)
    assert p.filter_ranges[behind] == pytest.approx(R_MAX)
    assert p.filter_valid[ahead] and p.filter_valid[behind]   # observed, and free


def test_a_corner_reads_the_nearer_of_the_two_walls():
    fb = bin_bearings(FILTER)
    left = wall_ranges(fb, 0.8)                                  # y = +0.8
    front = wall_ranges(fb - math.pi / 2, 1.2)                   # x = +1.2, rotated in
    p = decode(packet(filter_ranges=np.minimum(left, front)))
    i45 = int(np.argmin(np.abs(fb - math.pi / 4)))
    assert p.filter_ranges[i45] == pytest.approx(min(left[i45], front[i45]))
    assert p.filter_ranges[i45] < R_MAX


def test_range_min_is_the_self_filter_floor():
    fr = np.full(FILTER, R_MAX)
    fr[0] = R_MIN - 1e-3                      # inside the robot: not a measurement
    with pytest.raises(ValueError, match="outside"):
        decode(packet(filter_ranges=fr))


def test_a_range_beyond_range_max_is_refused():
    fr = np.full(FILTER, R_MAX)
    fr[3] = R_MAX + 0.01
    with pytest.raises(ValueError, match="outside"):
        decode(packet(filter_ranges=fr))


# ----------------------------------------------------------------------
# Validity: unobserved is not free (hard rule 5)
# ----------------------------------------------------------------------
def test_an_uncovered_sector_is_invalid_and_its_range_is_not_checked():
    # A blind spot between the pods: no ray covered bins 10-14, so whatever number sits
    # in those slots means nothing and must not be read as free space.
    fv = np.ones(FILTER, bool)
    fv[10:15] = False
    fr = np.full(FILTER, R_MAX)
    fr[10:15] = 0.0                           # below range_min, but never measured
    p = decode(packet(filter_ranges=fr, filter_valid=fv))
    assert not p.filter_valid[10:15].any()
    assert p.filter_valid[:10].all() and p.filter_valid[15:].all()
    assert not p.fully_observed


def test_free_space_and_unobserved_space_are_different_records():
    free = decode(packet())
    fv = np.ones(FILTER, bool)
    fv[0] = False
    blind = decode(packet(filter_valid=fv))
    # Same range in that bin, opposite meaning.
    assert free.filter_ranges[0] == pytest.approx(blind.filter_ranges[0])
    assert free.filter_valid[0] and not blind.filter_valid[0]


def test_validity_must_be_zero_or_one():
    values = packet()
    values[HEADER_LEN + POLICY] = 0.5
    with pytest.raises(ValueError, match="policy validity"):
        decode(values)


# ----------------------------------------------------------------------
# Disagreement between scanner and environment
# ----------------------------------------------------------------------
def test_a_short_packet_is_refused():
    with pytest.raises(ValueError, match="floats"):
        decode(packet()[:-1])


def test_a_foreign_format_version_is_refused():
    values = packet()
    values[0] = FORMAT_VERSION + 1
    with pytest.raises(ValueError, match="version"):
        decode(values)


def test_declared_bin_counts_must_match_the_configuration():
    values = packet()
    values[2] = POLICY + 1
    with pytest.raises(ValueError, match="bins"):
        decode(values)


def test_a_packet_taken_with_other_range_limits_is_refused():
    with pytest.raises(ValueError, match="range"):
        decode(packet(range_max=R_MAX + 1.0))


def test_a_non_finite_value_is_refused():
    values = packet()
    values[HEADER_LEN + 2] = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        decode(values)


def test_a_negative_simulation_time_is_refused():
    with pytest.raises(ValueError, match="simulation time"):
        decode(packet(sim_time=-0.05))


# ----------------------------------------------------------------------
# One tick
# ----------------------------------------------------------------------
def test_a_packet_from_this_tick_is_accepted():
    p = decode(packet(sim_time=3.0))
    assert_current(p, 3.0, DT)
    assert_current(p, 3.0 + 0.4 * DT, DT)      # within half a step


def test_a_packet_from_the_previous_tick_is_refused():
    p = decode(packet(sim_time=3.0))
    with pytest.raises(ValueError, match="did not run this tick"):
        assert_current(p, 3.0 + DT, DT)


def test_a_packet_from_the_future_is_refused_too():
    p = decode(packet(sim_time=3.0 + DT))
    with pytest.raises(ValueError, match="did not run this tick"):
        assert_current(p, 3.0, DT)


def test_assert_current_rejects_a_non_positive_dt():
    p = decode(packet())
    with pytest.raises(ValueError):
        assert_current(p, 1.0, 0.0)
