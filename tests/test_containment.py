"""Known answers for the containment arithmetic (`pfbnav/eval/containment.py`; A20)."""
import math

import numpy as np
import pytest

from pfbnav.eval.containment import containment_excess, footprint_boundary, in_frame, rest_margin


def disc(r):
    return lambda psi: np.full_like(np.asarray(psi, dtype=float), r)


def test_in_frame_known_answers():
    pts = np.array([[1.0, 0.0], [0.0, 1.0]])
    # the same pose: unchanged
    assert in_frame(pts, (2.0, 3.0, 0.4), (2.0, 3.0, 0.4)) == pytest.approx(pts)
    # a body 1 m ahead of the frame, turned +90 deg: its +x point lands at (1, 1)
    got = in_frame(pts, (1.0, 0.0, math.pi / 2), (0.0, 0.0, 0.0))
    assert got == pytest.approx(np.array([[1.0, 1.0], [0.0, 0.0]]), abs=1e-12)
    # a frame turned +90 deg sees a world point straight ahead of it at its own +x
    got = in_frame(np.array([[0.0, 0.0]]), (0.0, 2.0, 0.0), (0.0, 0.0, math.pi / 2))
    assert got == pytest.approx(np.array([[2.0, 0.0]]), abs=1e-12)


def test_a_disc_sliding_ahead_is_contained_exactly_up_to_the_field():
    """A 0.2 m disc slides 0.1 m along +x; the swept set reaches 0.3 m ahead. A field of 0.32 m
    contains it by 0.02 m; one of 0.25 m is exceeded by 0.05 m."""
    boundary = footprint_boundary(disc(0.2), 720)
    poses = [(x, 0.0, 0.0) for x in np.linspace(0.0, 0.1, 11)]
    assert containment_excess(boundary, poses, (0.0, 0.0, 0.0),
                              lambda b: np.full(b.shape, 0.32)) == pytest.approx(-0.02, abs=1e-9)
    assert containment_excess(boundary, poses, (0.0, 0.0, 0.0),
                              lambda b: np.full(b.shape, 0.25)) == pytest.approx(0.05, abs=1e-9)


def test_a_rotating_rectangle_reaches_its_corner_radius():
    """A 0.2 x 0.1 half-size rectangle rotated in place: its corners sweep a circle of radius
    hypot(0.2, 0.1); a field of that radius contains it, one 1 mm smaller does not."""
    hx, hy = 0.2, 0.1

    def rect(psi):
        return np.minimum(hx / np.maximum(np.abs(np.cos(psi)), 1e-12),
                          hy / np.maximum(np.abs(np.sin(psi)), 1e-12))
    boundary = footprint_boundary(rect, 3600)
    poses = [(0.0, 0.0, th) for th in np.linspace(0.0, math.pi / 2, 181)]
    corner = math.hypot(hx, hy)
    assert containment_excess(boundary, poses, (0.0, 0.0, 0.0),
                              lambda b: np.full(b.shape, corner)) <= 1e-9
    assert containment_excess(boundary, poses, (0.0, 0.0, 0.0),
                              lambda b: np.full(b.shape, corner - 0.001)) > 0.0


def test_rest_margin_known_answers():
    assert rest_margin([0.5, 0.31, 0.4], [True] * 3, [0.3, 0.3, 0.3]) == pytest.approx(0.01)
    assert rest_margin([0.5, 0.29, 0.4], [True] * 3, [0.3, 0.3, 0.3]) == pytest.approx(-0.01)
    assert rest_margin([0.5, 9.0], [True, False], [0.3, 0.3]) == -math.inf
