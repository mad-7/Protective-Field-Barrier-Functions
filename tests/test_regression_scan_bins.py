"""Known answers for the scan-bin check of the wall-removal regression (ruled 2026-10-13)."""
import math

import numpy as np
import pytest

from pfbnav.eval.regression import (
    bins_on_shape,
    distance_to_triangles_2d,
    packet_bin_differences,
    wall_bin_check,
)

RANGE_MAX = 5.0


def ring(r, t=0.05):
    """A square wall ring of half-size r and half-thickness t, as plan triangles."""
    out = []
    for (x0, y0, x1, y1) in ((r - t, -r - t, r + t, r + t), (-r - t, -r - t, -r + t, r + t),
                             (-r - t, r - t, r + t, r + t), (-r - t, -r - t, r + t, -r + t)):
        out += [np.array([[x0, y0], [x1, y0], [x1, y1]]), np.array([[x0, y0], [x1, y1], [x0, y1]])]
    return out


WALL = ring(7.55)


def test_distance_to_triangles():
    tri = [np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])]
    d = distance_to_triangles_2d([[0.2, 0.2], [2.0, 0.0], [-1.0, -1.0], [1.0, 1.0]], tri)
    assert d == pytest.approx([0.0, 1.0, math.sqrt(2.0), math.sqrt(2.0) / 2])


def test_a_return_on_the_wall_is_a_wall_bin_and_one_short_of_it_is_not():
    # Robot at (0, 3) facing +y: body bearing 0 points at the wall's inner face at y = 7.5.
    pose = (0.0, 3.0, math.pi / 2)
    bearings = np.array([0.0, 0.0, math.pi / 2, 0.0])
    ranges = np.array([4.5, 4.0, 1.0, RANGE_MAX])
    valid = np.array([True, True, True, True])
    on = bins_on_shape(pose, ranges, bearings, valid, WALL, range_max=RANGE_MAX, tol=0.01)
    assert on.tolist() == [True, False, False, False]      # wall / short of it / aside / no return


def test_an_invalid_bin_views_nothing():
    on = bins_on_shape((0.0, 3.0, math.pi / 2), [4.5], [0.0], [False], WALL,
                       range_max=RANGE_MAX, tol=0.01)
    assert not on.any()


def test_a_sector_bin_counts_if_any_part_of_its_arc_is_on_the_wall():
    # Centre bearing misses the wall's corner region; the arc's edge reaches it.
    pose = (7.0, 7.0, 0.0)
    r = 0.6
    centre = math.radians(-60.0)          # the point at the centre bearing is inside the ring
    half = math.radians(30.0)             # the arc sweeps up to bearing -30 deg ... and -90 deg
    narrow = bins_on_shape(pose, [r], [centre], [True], WALL, range_max=RANGE_MAX, tol=0.01)
    wide = bins_on_shape(pose, [r], [centre], [True], WALL, range_max=RANGE_MAX, tol=0.01,
                         sector_half_width=half)
    assert not narrow.any() and wide.all()


def _packet(pose, filter_ranges, policy_ranges):
    n_f, n_p = len(filter_ranges), len(policy_ranges)
    return {"pose": pose,
            "filter_bearings": np.linspace(-math.pi, math.pi, n_f, endpoint=False) + math.pi / n_f,
            "filter_ranges": np.asarray(filter_ranges, dtype=float),
            "filter_valid": np.ones(n_f, dtype=bool),
            "policy_ranges": np.asarray(policy_ranges, dtype=float),
            "policy_valid": np.ones(n_p, dtype=bool)}


def _scene(pose, with_wall):
    """Ideal ranges from a robot in an empty room bounded by the ring (or by nothing)."""
    n_f, n_p = 120, 36
    pkt = _packet(pose, np.full(n_f, RANGE_MAX), np.full(n_p, RANGE_MAX))
    if with_wall:
        x, y, yaw = pose
        inner = 7.55 - 0.05          # the ring's inner face; the robot stands inside it
        for k, b in enumerate(pkt["filter_bearings"]):
            c, s = math.cos(yaw + b), math.sin(yaw + b)
            hits = [(math.copysign(inner, d) - p) / d for p, d in ((x, c), (y, s))
                    if abs(d) > 1e-12]
            hit = min(h for h in hits if h > 0)
            if hit < RANGE_MAX:
                pkt["filter_ranges"][k] = hit
        # policy bin = minimum of the filter bins whose bearing falls in its sector
        edges = -math.pi + 2 * math.pi * np.arange(n_p + 1) / n_p
        for j in range(n_p):
            inside = (pkt["filter_bearings"] >= edges[j]) & (pkt["filter_bearings"] < edges[j + 1])
            pkt["policy_ranges"][j] = pkt["filter_ranges"][inside].min()
    return pkt


def test_removing_the_wall_changes_exactly_the_predicted_bins():
    pose = (0.0, 3.5, math.pi / 2)
    a, c = _scene(pose, True), _scene(pose, False)
    report = wall_bin_check([a, a], [c, c], WALL, range_max=RANGE_MAX, tol=0.01)
    assert report["exact"], report["mismatches"]
    assert report["packets_viewing"] == 2 and report["bins_predicted"] > 0


def test_a_bin_that_changed_without_viewing_the_wall_fails_the_check():
    pose = (0.0, 3.5, math.pi / 2)
    a, c = _scene(pose, True), _scene(pose, False)
    behind = int(np.argmin(np.abs(a["filter_bearings"] - math.pi)))   # looks away from the wall
    c["filter_ranges"][behind] = 1.0
    report = wall_bin_check([a], [c], WALL, range_max=RANGE_MAX, tol=0.01)
    assert not report["exact"]
    assert behind in report["mismatches"][0]["differ_not_predicted"]


def test_a_wall_bin_that_did_not_change_fails_the_check():
    pose = (0.0, 3.5, math.pi / 2)
    a = _scene(pose, True)
    report = wall_bin_check([a], [a], WALL, range_max=RANGE_MAX, tol=0.01)
    assert not report["exact"] and report["mismatches"][0]["predicted_not_differing"]


def test_identical_packets_far_from_the_wall_pass_and_differences_are_bitwise():
    far = _scene((0.0, 0.0, math.pi / 2), True)          # wall 7.5 m away, beyond range
    assert wall_bin_check([far], [far], WALL, range_max=RANGE_MAX, tol=0.01)["exact"]
    other = dict(far, filter_ranges=far["filter_ranges"] + 1e-12)
    assert packet_bin_differences(far, other)["filter"].all()
