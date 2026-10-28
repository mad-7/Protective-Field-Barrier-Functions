"""Known-answer tests for position-aware coverage (hard rule 5, hard rule 6).

The geometry is chosen so the answer can be worked out by hand: a rectangular robot, and
scanners at the positions the platform actually has (centreline, 240 deg) and the one the
supervisor chose (diagonal corners, 270 deg).
"""
import math

import numpy as np
import pytest

from pfbnav.sim.coverage import (
    SensorView,
    bin_mask,
    inside_footprint,
    observed,
    unobserved_extent,
    visible,
)

HX, HY = 0.2854, 0.1789            # the chassis half-extents
RMAX = 5.0


def rectangle(hx, hy):
    """Extent function of an axis-aligned rectangle centred on the origin."""
    def extent(psi):
        c, s = np.abs(np.cos(psi)), np.abs(np.sin(psi))
        with np.errstate(divide="ignore"):
            return np.minimum(np.where(c > 1e-12, hx / c, np.inf),
                              np.where(s > 1e-12, hy / s, np.inf))
    return extent


RECT = rectangle(HX, HY)
# The inherited envelope: the centreline pods stick out to 0.352 m fore and aft.
POD_RECT = rectangle(0.352, HY)


def centreline_240():
    """The inherited layout: two 240 deg pods on the centreline, as four 120 deg sensors."""
    return [SensorView(0.325, 0.0, math.radians(a), math.radians(120.0), RMAX)
            for a in (60.0, -60.0)] + \
           [SensorView(-0.325, 0.0, math.radians(a), math.radians(120.0), RMAX)
            for a in (120.0, -120.0)]


def corners_270():
    """The chosen layout: 270 deg scanners flush on two diagonal corners."""
    return [SensorView(HX, HY, math.radians(45.0), math.radians(270.0), RMAX),
            SensorView(-HX, -HY, math.radians(-135.0), math.radians(270.0), RMAX)]


def corners_240():
    return [SensorView(HX, HY, math.radians(45.0), math.radians(240.0), RMAX),
            SensorView(-HX, -HY, math.radians(-135.0), math.radians(240.0), RMAX)]


# ----------------------------------------------------------------------
# Visibility of single points
# ----------------------------------------------------------------------
def test_a_point_in_front_of_a_forward_sensor_is_visible():
    s = SensorView(0.0, 0.0, 0.0, math.radians(120.0), RMAX)
    assert visible(s, np.array([[1.0, 0.0]])).tolist() == [True]


def test_a_point_outside_the_field_of_view_is_not():
    s = SensorView(0.0, 0.0, 0.0, math.radians(120.0), RMAX)
    # 61 deg off the axis of a 120 deg sensor.
    p = np.array([[math.cos(math.radians(61)), math.sin(math.radians(61))]])
    assert visible(s, p).tolist() == [False]


def test_a_point_beyond_range_is_not():
    s = SensorView(0.0, 0.0, 0.0, math.radians(120.0), 1.0)
    assert visible(s, np.array([[1.5, 0.0]])).tolist() == [False]


def test_the_robot_hides_what_is_behind_it():
    # A corner sensor looking across the robot at a point on the far side.
    s = SensorView(HX, HY, math.radians(-90.0), math.radians(270.0), RMAX)
    far_side = np.array([[0.0, -HY - 0.1]])
    assert visible(s, far_side, None).tolist() == [True]          # geometry alone
    assert visible(s, far_side, RECT).tolist() == [False]         # the robot is in the way


def test_a_sensor_inside_the_envelope_can_still_see_out():
    # Scanner bodies are part of the envelope, so rays start inside it: leaving is fine,
    # only coming back in blocks.
    s = SensorView(HX - 0.01, 0.0, 0.0, math.radians(120.0), RMAX)
    assert visible(s, np.array([[1.0, 0.0]]), RECT).tolist() == [True]


def test_inside_footprint_known_answer():
    pts = np.array([[0.0, 0.0], [HX - 0.01, 0.0], [HX + 0.01, 0.0], [0.0, HY + 0.01]])
    assert inside_footprint(pts, RECT).tolist() == [True, True, False, False]


# ----------------------------------------------------------------------
# The inherited layout: blind beside the robot
# ----------------------------------------------------------------------
def test_centreline_pods_miss_points_beside_the_robot():
    # Checked by hand on 2026-09-21: at mid-body, 0.21-0.56 m out is seen by neither pod.
    pts = np.array([[0.0, w] for w in (0.21, 0.30, 0.40, 0.50, 0.56)])
    assert not observed(centreline_240(), pts).any()
    assert observed(centreline_240(), np.array([[0.0, 0.57], [0.0, 1.0]])).all()


def test_centreline_pods_invalidate_the_abeam_bins_and_keep_the_axial_ones():
    mask = bin_mask(centreline_240(), 120, POD_RECT, depth=1.0, occlusion=False)
    b = np.degrees(-math.pi + 2 * math.pi * (np.arange(120) + 0.5) / 120)
    assert not mask[np.abs(b - 90.0) < 5].any()        # beside the robot, left
    assert not mask[np.abs(b + 90.0) < 5].any()        # and right
    assert mask[np.abs(b) < 5].all()                   # ahead
    assert mask[np.abs(np.abs(b) - 180.0) < 5].all()   # behind


def test_the_blind_zone_reaches_where_the_hand_calculation_says():
    # Apex 0.563 m from the centreline at mid-body, so 0.563 - 0.1789 beyond the chassis.
    r = unobserved_extent(centreline_240(), POD_RECT, depth=1.0, occlusion=False)
    assert r["reach_m"] == pytest.approx(0.563 - HY, abs=0.02)
    assert r["area_m2"] > 0.1


# ----------------------------------------------------------------------
# The chosen layout: nothing outside the chassis is blind
# ----------------------------------------------------------------------
def test_diagonal_270_degree_scanners_see_everything_outside_the_chassis():
    r = unobserved_extent(corners_270(), RECT, depth=1.0)
    assert r["area_m2"] == 0.0
    assert bin_mask(corners_270(), 120, RECT, depth=1.0).all()
    assert bin_mask(corners_270(), 36, RECT, depth=1.0).all()


def test_diagonal_240_degree_scanners_leave_the_other_two_corners_blind():
    mask = bin_mask(corners_240(), 120, RECT, depth=1.0)
    b = np.degrees(-math.pi + 2 * math.pi * (np.arange(120) + 0.5) / 120)
    # The front-right corner sits at about -32 deg: blind for 240 deg units.
    corner = math.degrees(math.atan2(-HY, HX))
    assert not mask[np.abs(b - corner) < 3].all()
    assert unobserved_extent(corners_240(), RECT, depth=1.0)["reach_m"] > 0.1


def test_bin_mask_rejects_nonsense():
    with pytest.raises(ValueError):
        bin_mask(corners_270(), 0, RECT, depth=1.0)
    with pytest.raises(ValueError):
        bin_mask(corners_270(), 36, RECT, depth=0.0)


def test_occlusion_by_crossings_agrees_with_a_brute_force_march():
    # The crossing count must say what walking along each sight line would say.
    from pfbnav.sim.coverage import inside_footprint as inside
    rng = np.random.default_rng(3)
    s = SensorView(HX, HY, math.radians(45.0), math.radians(270.0), RMAX)
    pts = rng.uniform(-1.0, 1.0, size=(300, 2))
    pts = pts[~inside(pts, RECT)]
    fast = visible(s, pts, RECT)
    geo = visible(s, pts, None)
    for p, f, g in zip(pts, fast, geo, strict=True):
        if not g:
            assert not f
            continue
        line = np.array([HX, HY]) + np.outer(np.linspace(0, 1, 2000), p - np.array([HX, HY]))
        ins = inside(line, RECT)
        out_at = np.flatnonzero(~ins)
        blocked = bool(out_at.size and ins[out_at[0]:].any())
        assert f == (not blocked)
