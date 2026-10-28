"""Known answers for the step-0 trace reading (`pfbnav/eval/trace.py`)."""
import math

import numpy as np
import pytest

from pfbnav.eval.trace import (
    bins_inside,
    first_segment_bearing_body_deg,
    nearest_to_footprint,
    quarter_turn_class,
    scan_points,
    signed_angle_deg,
)


def test_signed_angles_wrap_to_the_half_open_turn():
    assert signed_angle_deg(math.radians(10), math.radians(350)) == pytest.approx(20.0)
    assert signed_angle_deg(math.pi, 0.0) == pytest.approx(180.0)
    assert signed_angle_deg(-math.pi, 0.0) == pytest.approx(180.0)


@pytest.mark.parametrize("error, label", [
    (0.0, "0"), (8.2, "0"), (-14.9, "0"), (90.0, "+90"), (101.0, "+90"), (-88.0, "-90"),
    (180.0, "180"), (-171.0, "180"), (190.0, "180"), (45.0, "other"), (-130.0, "other"),
])
def test_errors_are_classified_by_the_nearest_quarter_turn(error, label):
    assert quarter_turn_class(error) == label


def test_the_first_segment_is_measured_from_the_robots_heading():
    path = [(0.0, 0.0), (0.0, 0.0), (0.0, 2.0), (3.0, 2.0)]   # a repeated first vertex is skipped
    assert first_segment_bearing_body_deg(path, 0.0) == pytest.approx(90.0)
    assert first_segment_bearing_body_deg(path, math.pi / 2) == pytest.approx(0.0)
    assert first_segment_bearing_body_deg(path, math.pi) == pytest.approx(-90.0)
    assert first_segment_bearing_body_deg([(1.0, 1.0)], 0.0) is None


def _disc(radius):
    """A circular footprint: signed distance |q| - radius."""
    return lambda q: np.hypot(q[:, 0], q[:, 1]) - radius


def test_scan_points_put_an_unobserved_bin_at_the_minimum_range():
    pts = scan_points([2.0, 5.0], [0.0, math.pi / 2], [True, False], 0.05)
    assert pts[0] == pytest.approx([2.0, 0.0])
    assert pts[1] == pytest.approx([0.0, 0.05], abs=1e-12)


def test_the_nearest_point_to_a_disc_footprint_is_known():
    bearings = np.radians([-60.0, 30.0, 90.0])
    near = nearest_to_footprint(_disc(0.2), [0.9, 0.5, 0.7], bearings, [True] * 3, 0.05)
    assert near["distance_m"] == pytest.approx(0.3)
    assert near["bearing_deg"] == pytest.approx(30.0)
    assert near["valid"]
    # inside the footprint: negative; an unobserved bin counts at range_min and wins
    near = nearest_to_footprint(_disc(0.2), [0.15, 0.5, 9.0], bearings, [True, True, False], 0.05)
    assert near["distance_m"] == pytest.approx(-0.15)
    assert near["bearing_deg"] == pytest.approx(90.0) and not near["valid"]
    near = nearest_to_footprint(_disc(0.2), [0.15, 0.5, 9.0], bearings, [True] * 3, 0.05)
    assert near["distance_m"] == pytest.approx(-0.05)


def test_bins_inside_a_field_are_listed_with_their_depth():
    bearings = np.radians([0.0, 90.0, 180.0])
    found = bins_inside([0.25, 0.40, 1.0], bearings, [True, True, False], [0.30, 0.30, 0.30])
    assert [round(f["bearing_deg"]) for f in found] == [0, 180]
    assert found[0]["by_m"] == pytest.approx(0.05) and found[0]["valid"]
    assert found[1]["by_m"] is None and not found[1]["valid"]
    assert bins_inside([1.0], [0.0], [True], [0.3]) == []

