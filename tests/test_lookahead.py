"""Known-answer tests for path tracking (hard rule 6). Pure; no simulator.

Two things carry real weight here: the arc-length projection, because the progress reward
is defined on it (the observation ruling of 2026-09-23), and its monotonicity, because a
path that passes near itself would otherwise let the tracker jump backwards -- which reads
as negative progress and pays a policy to drive in circles.
"""
import math

import numpy as np
import pytest

from pfbnav.planning.lookahead import (
    PathTracker,
    arc_lengths,
    point_at,
    project,
    to_body,
)

STRAIGHT = np.array([[0.0, 0.0], [10.0, 0.0]])
ELL = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]])
HAIRPIN = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 0.4], [0.0, 0.4]])


def test_arc_lengths_known_answers():
    assert list(arc_lengths(STRAIGHT)) == [0.0, 10.0]
    assert list(arc_lengths(ELL)) == [0.0, 5.0, 10.0]
    diag = np.array([[0.0, 0.0], [3.0, 4.0]])
    assert arc_lengths(diag)[-1] == pytest.approx(5.0)


def test_point_at_interpolates_and_clamps():
    assert list(point_at(ELL, 2.5)) == [2.5, 0.0]
    assert list(point_at(ELL, 7.5)) == [5.0, 2.5]
    assert list(point_at(ELL, -3.0)) == [0.0, 0.0]
    assert list(point_at(ELL, 99.0)) == [5.0, 5.0]


def test_projection_on_a_straight_path():
    p = project(STRAIGHT, [3.0, 2.0])
    assert p.s == pytest.approx(3.0)
    assert p.distance == pytest.approx(2.0)
    assert list(p.point) == [3.0, 0.0]


def test_projection_at_a_corner_takes_the_nearer_leg():
    p = project(ELL, [5.6, 0.4])
    assert p.distance == pytest.approx(0.6, abs=1e-9)
    assert p.s == pytest.approx(5.4, abs=1e-9)


def test_projection_before_the_start_and_after_the_end_clamps():
    assert project(STRAIGHT, [-4.0, 0.0]).s == pytest.approx(0.0)
    assert project(STRAIGHT, [14.0, 0.0]).s == pytest.approx(10.0)


def test_projection_from_s_ignores_what_is_behind():
    assert project(ELL, [4.0, 0.2], from_s=0.0).s == pytest.approx(4.0)
    assert project(ELL, [4.0, 0.2], from_s=6.0).s == pytest.approx(6.0)


def test_a_window_is_needed_as_well_as_from_s_on_a_hairpin():
    """The hairpin's legs are 0.4 m apart but 7 m apart along the path. Searching forward
    without a window projects a robot on the first leg onto the return leg and pays it for
    7 m it never travelled."""
    point = [2.0, 0.35]
    assert project(HAIRPIN, point).s > 5.0, "the globally nearest point is on the return leg"
    assert project(HAIRPIN, point, from_s=1.0).s > 5.0, "forward alone still leaps"
    windowed = project(HAIRPIN, point, from_s=1.0, window=1.0)
    assert windowed.s == pytest.approx(2.0, abs=1e-9)


def test_the_tracker_never_goes_backwards_on_a_hairpin():
    tracker = PathTracker(HAIRPIN, lookahead=0.8)
    history = []
    for x in np.linspace(0.0, 5.0, 26):          # out along the first leg
        history.append(tracker.update([x, 0.0, 0.0]).s)
    assert history == sorted(history)
    assert history[-1] == pytest.approx(5.0, abs=0.01)
    for x in np.linspace(5.0, 0.0, 26):          # and back along the second
        state = tracker.update([x, 0.4, math.pi])
        history.append(state.s)
        assert state.progress >= -1e-9, "progress went backwards"
    assert history == sorted(history)


def creep(tracker, xs, y=0.0, yaw=0.0):
    """Update the tracker along a series of poses, as the step loop would (2 cm a step)."""
    state = None
    for x in xs:
        state = tracker.update([x, y, yaw])
    return state


def test_progress_is_the_arc_length_gained():
    tracker = PathTracker(ELL, lookahead=0.8)
    creep(tracker, np.arange(0.0, 2.01, 0.02))
    assert tracker.s == pytest.approx(2.0)
    assert tracker.update([2.0, 0.0, 0.0]).progress == pytest.approx(0.0)
    assert tracker.update([2.5, 0.0, 0.0]).progress == pytest.approx(0.5)


def test_a_leap_further_than_the_window_is_not_paid_as_progress():
    """A teleport -- a reset, a simulator hiccup -- must not read as metres travelled."""
    tracker = PathTracker(ELL, lookahead=0.8, window=1.0)
    state = tracker.update([9.0, 0.0, 0.0])
    assert state.progress == pytest.approx(1.0), "progress is capped by the window"
    assert state.s == pytest.approx(1.0)


def test_cross_track_is_the_distance_from_the_path():
    tracker = PathTracker(STRAIGHT, lookahead=0.8)
    assert creep(tracker, np.arange(0.0, 3.01, 0.02), y=0.25).cross_track ==         pytest.approx(0.25)
    assert creep(tracker, np.arange(3.0, 4.01, 0.02), y=-0.4).cross_track ==         pytest.approx(0.4)


def test_the_lookahead_sits_one_lookahead_ahead_and_stops_at_the_end():
    tracker = PathTracker(STRAIGHT, lookahead=0.8)
    state = creep(tracker, np.arange(0.0, 2.01, 0.02))
    assert list(state.lookahead_world) == pytest.approx([2.8, 0.0])
    assert not state.at_end
    end = creep(tracker, np.arange(2.0, 9.91, 0.02))
    assert list(end.lookahead_world) == pytest.approx([10.0, 0.0])
    assert end.at_end
    assert end.remaining == pytest.approx(0.1, abs=1e-9)


def test_the_lookahead_follows_the_path_around_a_corner():
    tracker = PathTracker(ELL, lookahead=0.8)
    state = creep(tracker, np.arange(0.0, 4.61, 0.02))
    # 0.8 m along the path from s = 4.6 turns the corner at s = 5.
    assert list(state.lookahead_world) == pytest.approx([5.0, 0.4])


# ----------------------------------------------------------------------
# Body-frame encoding
# ----------------------------------------------------------------------
def test_to_body_straight_ahead_and_abeam():
    assert to_body([1.0, 0.0], [0.0, 0.0, 0.0]) == pytest.approx((1.0, 0.0, 1.0))
    assert to_body([0.0, 2.0], [0.0, 0.0, 0.0]) == pytest.approx((2.0, 1.0, 0.0))
    assert to_body([0.0, -2.0], [0.0, 0.0, 0.0]) == pytest.approx((2.0, -1.0, 0.0))
    assert to_body([-3.0, 0.0], [0.0, 0.0, 0.0]) == pytest.approx((3.0, 0.0, -1.0))


def test_to_body_turns_with_the_robot():
    # The robot faces +y; a point at world +y is then straight ahead.
    assert to_body([0.0, 1.0], [0.0, 0.0, math.pi / 2]) == pytest.approx((1.0, 0.0, 1.0))
    # And a point at world +x is on its right.
    assert to_body([1.0, 0.0], [0.0, 0.0, math.pi / 2]) == pytest.approx((1.0, -1.0, 0.0))


def test_to_body_is_offset_by_the_robots_position():
    assert to_body([4.0, 3.0], [1.0, 3.0, 0.0]) == pytest.approx((3.0, 0.0, 1.0))


def test_to_body_at_zero_distance_is_defined_not_nan():
    d, sin_b, cos_b = to_body([2.0, 2.0], [2.0, 2.0, 0.7])
    assert d == 0.0 and math.isfinite(sin_b) and math.isfinite(cos_b)
    assert (sin_b, cos_b) == (0.0, 1.0)


def test_the_encoded_pair_is_a_unit_vector():
    for point in ([3.0, 1.0], [-2.0, 5.0], [0.5, -7.0]):
        _, sin_b, cos_b = to_body(point, [1.0, 2.0, 0.3])
        assert math.hypot(sin_b, cos_b) == pytest.approx(1.0)


def test_the_tracker_encodes_the_goal_as_well_as_the_lookahead():
    tracker = PathTracker(ELL, lookahead=0.8)
    state = tracker.update([0.0, 0.0, 0.0])
    assert state.s == pytest.approx(0.0)
    assert state.goal_body[0] == pytest.approx(math.hypot(5.0, 5.0))
    assert state.lookahead_body[0] == pytest.approx(0.8)
    assert list(state.goal_world) == [5.0, 5.0]


# ----------------------------------------------------------------------
# Refusals
# ----------------------------------------------------------------------
def test_a_one_point_path_is_refused():
    with pytest.raises(ValueError):
        PathTracker(np.array([[0.0, 0.0]]), lookahead=0.8)


def test_a_non_positive_lookahead_or_window_is_refused():
    with pytest.raises(ValueError):
        PathTracker(STRAIGHT, lookahead=0.0)
    with pytest.raises(ValueError):
        PathTracker(STRAIGHT, lookahead=0.8, window=0.0)


def test_reset_puts_the_tracker_back_and_clamps():
    tracker = PathTracker(STRAIGHT, lookahead=0.8)
    tracker.update([5.0, 0.0, 0.0])
    tracker.reset()
    assert tracker.s == 0.0
    tracker.reset(99.0)
    assert tracker.s == pytest.approx(tracker.length)
