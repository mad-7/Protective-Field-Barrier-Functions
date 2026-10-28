"""Known-answer tests for the coverage map's cell classification (hard rule 6)."""
import math

import numpy as np
import pytest

from pfbnav.eval.coverage_map import (
    CONFOUNDED,
    INVALID,
    OBSERVED,
    VIOLATION,
    bin_index,
    bins_spanned,
    classify,
    grid_cells,
    tally,
)

TOL = 0.02


def test_a_valid_bin_that_reads_the_post_is_observed():
    assert classify([True], [0.49], [5.0], 0.50, TOL) == OBSERVED


def test_a_valid_bin_that_reads_past_the_post_is_a_violation():
    # Free space reported where the post stands: the failure hard rule 5 forbids.
    assert classify([True], [5.0], [5.0], 0.50, TOL) == VIOLATION


def test_an_invalid_bin_is_never_a_violation():
    assert classify([False], [5.0], [5.0], 0.50, TOL) == INVALID


def test_a_bin_already_short_without_the_post_is_confounded_not_observed():
    assert classify([True], [0.30], [0.30], 0.50, TOL) == CONFOUNDED


def test_the_tolerance_boundary():
    assert classify([True], [0.52], [5.0], 0.50, TOL) == OBSERVED
    assert classify([True], [0.5201], [5.0], 0.50, TOL) == VIOLATION


def test_a_neighbouring_spanned_bin_reporting_the_post_is_enough():
    # The measured case (COVMAP-s300_corners_20260922T212716Z): the axis bin empty, the
    # post's near face in the bin next door. The scan does report the obstacle.
    assert classify([True, True], [5.0, 0.49], [5.0, 5.0], 0.50, TOL) == OBSERVED


def test_every_spanned_bin_silent_is_still_a_violation():
    assert classify([True, True], [5.0, 5.0], [5.0, 5.0], 0.50, TOL) == VIOLATION


def test_a_valid_neighbour_decides_when_the_axis_bin_is_invalid():
    assert classify([False, True], [5.0, 0.49], [5.0, 5.0], 0.50, TOL) == OBSERVED
    assert classify([False, True], [5.0, 5.0], [5.0, 5.0], 0.50, TOL) == VIOLATION
    assert classify([False, False], [5.0, 5.0], [5.0, 5.0], 0.50, TOL) == INVALID


def test_classify_refuses_mismatched_or_empty_input():
    with pytest.raises(ValueError):
        classify([True], [1.0, 2.0], [5.0, 5.0], 0.50, TOL)
    with pytest.raises(ValueError):
        classify([], [], [], 0.50, TOL)


def test_bins_spanned_covers_the_posts_angular_width():
    # The measured cell: a 10 mm radius post at 0.24 m subtends +-2.39 deg, so it reaches
    # from 90.60 to 95.38 deg; 120 bins are 3 deg wide, so bins 90 ([90, 93)) and 91.
    b = math.radians(92.9928)                      # 0.007 deg inside bin 90
    spanned = bins_spanned(b, 0.24, 0.01, 120)
    assert spanned[0] == bin_index(b, 120) == 90
    assert set(spanned) == {90, 91}


def test_bins_spanned_includes_the_bins_between_the_edges():
    # Pressed against the footprint a post is wide: at 0.12 m it subtends +-4.78 deg.
    spanned = bins_spanned(math.radians(92.9928), 0.12, 0.01, 120)
    assert set(spanned) == {89, 90, 91, 92}
    assert spanned[0] == 90


def test_bins_spanned_wraps_at_the_seam():
    assert set(bins_spanned(math.radians(179.9), 0.24, 0.01, 120)) == {119, 0}


def test_bins_spanned_collapses_to_one_bin_when_the_post_is_far():
    b = math.radians(46.5)                         # mid-bin, so no edge is clipped
    assert bins_spanned(b, 5.0, 0.01, 120) == [bin_index(b, 120)]


def test_bins_spanned_refuses_nonsense():
    with pytest.raises(ValueError):
        bins_spanned(0.0, 0.0, 0.01, 120)


def test_bin_index_matches_the_packet_binning():
    assert bin_index(-math.pi + 1e-9, 120) == 0
    assert bin_index(0.0, 120) == 60
    assert bin_index(math.pi, 120) == 119
    assert bin_index(math.radians(90.0), 36) == 27


def _disc(r):
    return lambda psi: np.full_like(np.asarray(psi, dtype=float), r)


def test_grid_cells_lie_between_the_footprint_edge_and_the_depth():
    cells = grid_cells(_disc(0.3), depth=1.0, spacing=0.05, clearance=0.015)
    r = np.hypot(cells[:, 0], cells[:, 1])
    assert r.min() > 0.315 and r.max() <= 1.3 + 1e-12
    # A ring from 0.315 to 1.3 m sampled every 5 cm: area / cell area.
    expected = math.pi * (1.3 ** 2 - 0.315 ** 2) / 0.05 ** 2
    assert len(cells) == pytest.approx(expected, rel=0.03)


def test_grid_cells_reject_nonsense():
    with pytest.raises(ValueError):
        grid_cells(_disc(0.3), depth=1.0, spacing=0.0, clearance=0.0)


def test_tally_counts_and_refuses_unknown_classes():
    assert tally([OBSERVED, OBSERVED, INVALID]) == {OBSERVED: 2, INVALID: 1,
                                                    VIOLATION: 0, CONFOUNDED: 0}
    with pytest.raises(ValueError):
        tally(["free"])
