"""Known-answer tests for the polygon footprint geometry (hard rule 6).

No simulator. Every shape here has radii that can be worked out by hand, so a geometry
error shows up as failing arithmetic rather than as a plausible number in the pods report.
"""
import math

import numpy as np
import pytest

from pfbnav.sim.footprint import (
    contains_origin,
    convex_hull,
    outermost_contact,
    polygon_from_support,
    quadrant_radii,
    radial_extent,
    support_of_points,
    symmetry_report,
)


def _rectangle(a: float, b: float):
    """Corners of a rectangle with half-extents a along x and b along y."""
    return [[a, b], [-a, b], [-a, -b], [a, -b]]


# ----------------------------------------------------------------------
# convex_hull
# ----------------------------------------------------------------------
def test_hull_of_a_square_is_its_four_corners():
    hull = convex_hull(_rectangle(1.0, 1.0))
    assert len(hull) == 4
    assert sorted(map(tuple, np.round(hull, 9))) == sorted(
        map(tuple, np.round(_rectangle(1.0, 1.0), 9))
    )


def test_interior_points_do_not_appear_in_the_hull():
    hull = convex_hull(_rectangle(1.0, 1.0) + [[0.0, 0.0], [0.5, 0.5]])
    assert len(hull) == 4


def test_hull_rejects_too_few_points():
    with pytest.raises(ValueError, match="at least 3"):
        convex_hull([[0.0, 0.0], [1.0, 1.0]])


def test_hull_rejects_the_wrong_shape():
    with pytest.raises(ValueError):
        convex_hull([0.0, 1.0, 2.0])


# ----------------------------------------------------------------------
# contains_origin
# ----------------------------------------------------------------------
def test_a_centred_rectangle_contains_the_origin():
    assert contains_origin(convex_hull(_rectangle(1.0, 0.5)))


def test_an_offset_rectangle_does_not():
    offset = [[p[0] + 5.0, p[1]] for p in _rectangle(1.0, 0.5)]
    assert not contains_origin(convex_hull(offset))


# ----------------------------------------------------------------------
# radial_extent
# ----------------------------------------------------------------------
@pytest.mark.parametrize("bearing, expected", [
    (0.0, 1.0),                 # straight ahead, to the x face
    (math.pi, 1.0),             # straight back
    (math.pi / 2, 0.5),         # abeam, to the nearer y face
    (-math.pi / 2, 0.5),
])
def test_radial_extent_of_a_rectangle_on_its_axes(bearing, expected):
    hull = convex_hull(_rectangle(1.0, 0.5))
    assert radial_extent(hull, bearing) == pytest.approx(expected, abs=1e-9)


def test_radial_extent_to_a_corner_is_the_half_diagonal():
    hull = convex_hull(_rectangle(1.0, 0.5))
    corner = math.atan2(0.5, 1.0)
    assert radial_extent(hull, corner) == pytest.approx(math.hypot(1.0, 0.5), abs=1e-9)


def test_radial_extent_of_a_unit_square_matches_the_closed_form():
    """For a square of half-side 1, the extent is 1 / max(|cos|, |sin|)."""
    hull = convex_hull(_rectangle(1.0, 1.0))
    for bearing in np.linspace(-math.pi, math.pi, 37):
        expected = 1.0 / max(abs(math.cos(bearing)), abs(math.sin(bearing)))
        assert radial_extent(hull, bearing) == pytest.approx(expected, abs=1e-9)


# ----------------------------------------------------------------------
# quadrant_radii
# ----------------------------------------------------------------------
def test_a_square_has_equal_radii_in_every_quadrant():
    radii = quadrant_radii(convex_hull(_rectangle(1.0, 1.0)), samples=181)
    for values in radii.values():
        assert values["inscribed"] == pytest.approx(1.0, abs=1e-6)
        assert values["circumscribed"] == pytest.approx(math.sqrt(2.0), abs=1e-3)


def test_a_rectangle_is_inscribed_by_its_shorter_half_extent():
    radii = quadrant_radii(convex_hull(_rectangle(1.0, 0.5)), samples=361)
    for values in radii.values():
        assert values["inscribed"] == pytest.approx(0.5, abs=1e-6)
        assert values["circumscribed"] == pytest.approx(math.hypot(1.0, 0.5), abs=1e-3)


def test_the_measured_robot_envelope_as_a_plain_rectangle():
    """The configured envelope, 0.3520 x 0.1970, as a sanity anchor on real numbers."""
    radii = quadrant_radii(convex_hull(_rectangle(0.3520, 0.1970)), samples=361)
    for values in radii.values():
        assert values["inscribed"] == pytest.approx(0.1970, abs=1e-6)
        assert values["circumscribed"] == pytest.approx(0.4034, abs=1e-3)


def test_a_cross_shaped_outline_is_not_its_bounding_rectangle():
    """Why a polygon is wanted at all.

    A robot with protrusions fore-aft and sideways but cut corners has a smaller
    circumscribed radius than the rectangle that bounds it.
    """
    cross = [[0.35, 0.05], [0.05, 0.20], [-0.05, 0.20], [-0.35, 0.05],
             [-0.35, -0.05], [-0.05, -0.20], [0.05, -0.20], [0.35, -0.05]]
    radii = quadrant_radii(convex_hull(cross), samples=721)
    circumscribed = max(v["circumscribed"] for v in radii.values())
    assert circumscribed == pytest.approx(0.3536, abs=2e-3)
    assert circumscribed < math.hypot(0.35, 0.20)   # the bounding rectangle's corner


def test_quadrant_radii_reject_an_origin_outside_the_outline():
    offset = [[p[0] + 5.0, p[1]] for p in _rectangle(1.0, 0.5)]
    with pytest.raises(ValueError, match="outside the footprint"):
        quadrant_radii(convex_hull(offset))


def test_quadrant_radii_reject_too_few_samples():
    with pytest.raises(ValueError, match="at least 2"):
        quadrant_radii(convex_hull(_rectangle(1.0, 1.0)), samples=1)


# ----------------------------------------------------------------------
# symmetry_report
# ----------------------------------------------------------------------
def test_a_symmetric_outline_reports_no_asymmetry():
    report = symmetry_report(quadrant_radii(convex_hull(_rectangle(1.0, 0.5)), samples=181))
    assert report["fore_aft_difference"] == pytest.approx(0.0, abs=1e-6)
    assert report["left_right_difference"] == pytest.approx(0.0, abs=1e-6)


def test_a_nose_heavy_outline_reports_a_fore_aft_difference():
    nose = [[1.5, 0.5], [-1.0, 0.5], [-1.0, -0.5], [1.5, -0.5]]
    report = symmetry_report(quadrant_radii(convex_hull(nose), samples=361))
    assert report["fore_aft_difference"] == pytest.approx(
        math.hypot(1.5, 0.5) - math.hypot(1.0, 0.5), abs=2e-3
    )
    assert report["left_right_difference"] == pytest.approx(0.0, abs=1e-6)


# ----------------------------------------------------------------------
# Support-function probe: bisection and polygon reconstruction
# ----------------------------------------------------------------------
def _plate(parts, thickness):
    """Contact function of a plate against parts given by projection intervals, with
    checkCollision's surface semantics: a part wholly inside the plate is not touched."""
    half = thickness / 2.0

    def contact(d):
        for lo, hi in parts:
            overlaps = lo - half <= d <= hi + half
            swallowed = d - half < lo and hi < d + half
            if overlaps and not swallowed:
                return True
        return False
    return contact


def test_outermost_contact_finds_the_outer_face_of_the_outermost_part():
    t = 0.006
    # A chassis and a wheel whose projections do not overlap: the gap between them is the
    # case a thin plate passes through, and the scan must still report the wheel.
    contact = _plate([(-0.28, 0.28), (0.31, 0.3473)], t)
    lo, hi = outermost_contact(contact, 0.5, -0.8, 0.005, 0.0005)
    assert lo - t / 2 <= 0.3473 < hi - t / 2
    assert hi - lo <= 0.0005


def test_outermost_contact_does_not_skip_a_part_as_thick_as_the_plate():
    t = 0.006
    for start in np.linspace(0.30, 0.31, 23):      # every phase of the scan grid
        contact = _plate([(-0.1, 0.1), (0.2500, 0.2560)], t)
        lo, hi = outermost_contact(contact, start, -0.8, 0.005, 0.0005)
        assert lo - t / 2 <= 0.2560 < hi - t / 2


def test_outermost_contact_can_skip_a_part_thinner_than_the_plate():
    """The limitation the fine retry exists for. A 0.8 mm part with the scan phased as
    the probe phases it (start = outer face + t + half the part) is stepped over, and the
    scan reports the part behind it instead."""
    t, w = 0.006, 0.0008
    c = 0.25
    contact = _plate([(-0.1, 0.1), (c - w / 2, c + w / 2)], t)
    lo, hi = outermost_contact(contact, c + w / 2 + t, -0.8, 0.005, 0.0005)
    assert hi - t / 2 < c                        # missed: this is the 0.1 m part
    lo, hi = outermost_contact(contact, c + w / 2 + t, -0.8, 0.0005, 0.0005)
    assert lo - t / 2 <= c + w / 2 < hi - t / 2   # the fine step finds it


def test_outermost_contact_refuses_a_start_inside_the_geometry():
    with pytest.raises(ValueError, match="scan start"):
        outermost_contact(_plate([(-0.3, 0.3)], 0.006), 0.2, -0.8, 0.005, 0.0005)


def test_outermost_contact_reports_nothing_when_nothing_is_touched():
    assert outermost_contact(lambda d: False, 0.5, -0.5, 0.005, 0.0005) is None


def test_polygon_from_support_recovers_a_rectangle():
    bearings = np.radians(np.arange(-180, 180, 2.0))
    corners = [(0.3, 0.2), (-0.3, 0.2), (-0.3, -0.2), (0.3, -0.2)]
    poly = polygon_from_support(bearings, support_of_points(corners, bearings))
    assert radial_extent(poly, 0.0) == pytest.approx(0.3, abs=1e-9)
    assert radial_extent(poly, math.pi / 2) == pytest.approx(0.2, abs=1e-9)
    assert np.max(np.linalg.norm(poly, axis=1)) == pytest.approx(math.hypot(0.3, 0.2),
                                                                  abs=1e-9)


def test_polygon_from_support_is_conservative_for_a_disc():
    # A disc of radius r sampled every 10 deg gives the circumscribed 36-gon, whose
    # vertices sit at r / cos(5 deg): never inside the disc, at most 0.4 % outside.
    r = 0.25
    bearings = np.radians(np.arange(0, 360, 10.0))
    poly = polygon_from_support(bearings, np.full(len(bearings), r))
    norms = np.linalg.norm(poly, axis=1)
    assert np.all(norms >= r - 1e-12)
    assert norms.max() == pytest.approx(r / math.cos(math.radians(5.0)), rel=1e-9)


def test_polygon_from_support_refuses_an_unbounded_sampling():
    with pytest.raises(ValueError, match="gap"):
        polygon_from_support(np.radians([0.0, 30.0, 60.0]), [0.2, 0.2, 0.2])
