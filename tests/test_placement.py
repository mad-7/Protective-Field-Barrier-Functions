"""Known-answer tests for route-relative placement (A13; hard rule 6). Pure."""
import math

import numpy as np
import pytest

from pfbnav.world.baking import Obstacle
from pfbnav.world.layouts import rectangle
from pfbnav.world.placement import (
    NO_BYPASS,
    TOO_CLOSE_TO_MAPPED,
    PlacementRule,
    convex_clearance,
    convex_overlap,
    expected_engaged_share,
    place_on_route,
    square_beside,
    surface_offset,
)

STRAIGHT = np.array([[0.0, 0.0], [10.0, 0.0]])
HALL = (-5.0, -5.0, 15.0, 5.0)


def rule(**kw):
    base = {"offset_max": 0.6, "first_offset_max": 0.6, "arc_from_start": 2.0,
            "arc_to_goal": 1.0, "min_goal_distance": 1.0, "margin": 0.15,
            "size": (0.25, 0.60), "attempts": 200}
    return PlacementRule(**{**base, **kw})


def always(_obstacles):
    return True


# ---------------------------------------------------------------- geometry, known answers
def test_a_square_beside_a_straight_route_sits_exactly_at_its_offset():
    """The ruled measure is to the nearest SURFACE, never the centre."""
    for offset, side in ((0.0, 1), (0.25, 1), (0.6, -1)):
        poly = square_beside(STRAIGHT, 5.0, offset, side, 0.4)
        assert surface_offset(poly, STRAIGHT) == pytest.approx(offset, abs=1e-9)
        centre_y = float(poly.mean(axis=0)[1])
        assert centre_y == pytest.approx(side * (offset + 0.2))


def test_left_is_left_of_the_direction_of_travel():
    assert square_beside(STRAIGHT, 5.0, 0.3, 1, 0.4).mean(axis=0)[1] > 0
    reversed_route = STRAIGHT[::-1].copy()
    assert square_beside(reversed_route, 5.0, 0.3, 1, 0.4).mean(axis=0)[1] < 0


def test_a_route_that_crosses_the_obstacle_has_zero_offset():
    poly = rectangle(4.0, -0.5, 5.0, 0.5)
    assert surface_offset(poly, STRAIGHT) == 0.0


def test_the_recorded_offset_is_the_measured_one_where_the_route_bends_back():
    """Sampled 0.5 m from one leg, but a hairpin brings the other leg within 0.1 m."""
    hairpin = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 1.3], [0.0, 1.3]])
    poly = square_beside(hairpin, 5.0, 0.5, 1, 0.6)        # spans y 0.5 .. 1.1
    assert surface_offset(poly, hairpin) == pytest.approx(0.2, abs=1e-9)


def test_convex_overlap_and_clearance_known_answers():
    a = rectangle(0, 0, 1, 1)
    assert convex_overlap(a, rectangle(0.5, 0.5, 2, 2))
    assert not convex_overlap(a, rectangle(1.5, 0, 2, 1))
    assert convex_clearance(a, rectangle(1.5, 0, 2, 1)) == pytest.approx(0.5)
    assert convex_clearance(a, rectangle(2, 2, 3, 3)) == pytest.approx(math.sqrt(2))
    # A thin bar crossing a square without either holding the other's vertices: SAT catches it.
    assert convex_overlap(a, rectangle(-1, 0.4, 2, 0.6))


def test_the_rule_refuses_impossible_values():
    with pytest.raises(ValueError, match="first_offset_max"):
        rule(first_offset_max=0.7)
    with pytest.raises(ValueError, match="size range"):
        rule(size=(0.6, 0.25))


# ---------------------------------------------------------------- the placement
def test_every_placement_respects_the_ruled_ranges():
    rng = np.random.default_rng(0)
    for _ in range(50):
        got = place_on_route(rng, STRAIGHT, (10.0, 0.0), [], HALL, 2, rule(), always)
        assert got.complete and len(got.obstacles) == 2
        for poly_offset, arc in zip(got.offsets_m, got.arcs_m, strict=True):
            assert 0.0 <= poly_offset <= 0.6 + 1e-9
            assert 2.0 - 0.3 <= arc <= 9.0 + 0.3     # centre's projection: within half a size
        for o in got.obstacles:
            assert not o.mapped


def test_offsets_and_sides_are_uniform():
    rng = np.random.default_rng(1)
    offsets, sides = [], []
    for _ in range(600):
        got = place_on_route(rng, STRAIGHT, (10.0, 0.0), [], HALL, 1, rule(), always)
        offsets.append(got.offsets_m[0])
        sides.append(np.sign(got.obstacles[0].polygon.mean(axis=0)[1]))
    assert np.mean(np.array(offsets) <= 0.3) == pytest.approx(0.5, abs=0.06)
    assert np.mean(np.array(sides) > 0) == pytest.approx(0.5, abs=0.06)


def _side_and_size(poly):
    return int(np.sign(poly.mean(axis=0)[1])), float(np.linalg.norm(poly[1] - poly[0]))


def test_the_cap_changes_the_first_obstacles_offset_and_nothing_else():
    """Ruled 2026-09-30: "first" is an index, not a location. The capped obstacle's position,
    side and size are drawn exactly as the second's, so with the same random stream, capping it
    at 0.3 m halves its offset and leaves everything else where it was (known answer: the offset
    is `cap * u` for the same u)."""
    for seed in range(40):
        wide = place_on_route(np.random.default_rng(seed), STRAIGHT, (10.0, 0.0), [], HALL, 1,
                              rule(first_offset_max=0.6), always)
        capped = place_on_route(np.random.default_rng(seed), STRAIGHT, (10.0, 0.0), [], HALL, 1,
                                rule(first_offset_max=0.3), always)
        assert capped.attempts == wide.attempts == 1
        assert capped.arcs_m == pytest.approx(wide.arcs_m, abs=1e-12)
        assert _side_and_size(capped.obstacles[0].polygon) == pytest.approx(
            _side_and_size(wide.obstacles[0].polygon))
        assert capped.offsets_m[0] == pytest.approx(wide.offsets_m[0] / 2, abs=1e-12)


def test_the_capped_obstacle_lies_along_the_route_like_the_others():
    """The same condition as a distribution: with the ruled cap, the capped obstacle (index 0)
    and the uncapped one (index 1) are spread alike along the window [2, 9] m and alike left and
    right. Mean of U[2, 9] is 5.5 with a standard error of 0.08 over 600 draws."""
    rng = np.random.default_rng(3)
    arcs = {0: [], 1: []}
    left = {0: [], 1: []}
    for _ in range(600):
        got = place_on_route(rng, STRAIGHT, (10.0, 0.0), [], HALL, 2,
                             rule(first_offset_max=0.3), always)
        assert got.offsets_m[0] <= 0.3 + 1e-9
        for k in (0, 1):
            arcs[k].append(got.arcs_m[k])
            left[k].append(_side_and_size(got.obstacles[k].polygon)[0] > 0)
    for k in (0, 1):
        assert np.mean(arcs[k]) == pytest.approx(5.5, abs=0.25)
        assert np.mean(np.array(arcs[k]) < 5.5) == pytest.approx(0.5, abs=0.06)
        assert np.mean(left[k]) == pytest.approx(0.5, abs=0.06)


def test_a_rejected_obstacle_is_resampled_and_the_rejection_counted():
    """Feasibility is decided in the TRUE world; the obstacle is re-sampled, never the layout."""
    calls = {"n": 0}

    def reject_twice(_obstacles):
        calls["n"] += 1
        return calls["n"] > 2

    got = place_on_route(np.random.default_rng(2), STRAIGHT, (10.0, 0.0), [], HALL, 1,
                         rule(), reject_twice)
    assert got.complete and got.rejections[NO_BYPASS] == 2 and got.attempts == 3


def test_obstacles_keep_their_margin_from_mapped_geometry():
    """A shelf 0.2 m to the route's left leaves no room for a margin-respecting obstacle there,
    so over many placements every one lands on the right and the left draws are refused."""
    shelf = Obstacle(rectangle(0.0, 0.2, 10.0, 1.0), name="shelf")
    rng = np.random.default_rng(3)
    refused = 0
    for _ in range(40):
        got = place_on_route(rng, STRAIGHT, (10.0, 0.0), [shelf], HALL, 1, rule(), always)
        assert got.complete
        assert convex_clearance(got.obstacles[0].polygon, shelf.polygon) >= 0.15
        assert got.obstacles[0].polygon.mean(axis=0)[1] < 0, "placed in the shelf's margin"
        refused += got.rejections[TOO_CLOSE_TO_MAPPED]
    assert refused > 0, "no left-side draw was ever refused in 40 placements"


def test_exhausting_the_attempts_is_reported_not_hidden():
    got = place_on_route(np.random.default_rng(4), STRAIGHT, (10.0, 0.0), [], HALL, 1,
                         rule(attempts=5), lambda _o: False)
    assert not got.complete and got.obstacles == [] and got.rejections[NO_BYPASS] == 5


def test_a_route_too_short_for_the_exclusions_places_nothing():
    short = np.array([[0.0, 0.0], [2.5, 0.0]])
    got = place_on_route(np.random.default_rng(5), short, (2.5, 0.0), [], HALL, 1, rule(), always)
    assert not got.complete and got.obstacles == []


def test_the_same_seed_places_the_same_obstacles():
    a = place_on_route(np.random.default_rng(6), STRAIGHT, (10.0, 0.0), [], HALL, 2, rule(), always)
    b = place_on_route(np.random.default_rng(6), STRAIGHT, (10.0, 0.0), [], HALL, 2, rule(), always)
    for x, y in zip(a.obstacles, b.obstacles, strict=True):
        assert np.array_equal(x.polygon, y.polygon)


# ---------------------------------------------------------------- the ruled numbers
def test_the_ruled_rule_engages_62_5_percent_short_of_the_ruled_90():
    """The inconsistency in the 2026-09-29 ruling, as a known answer.

    All offsets uniform on [0, 0.6] m with 1-2 obstacles gives P(any within 0.3 m) =
    1 - (0.5 + 0.25) / 2 = 0.625, so the ruled 90 % validity check fails by construction.
    Capping the FIRST obstacle at 0.3 m makes every scenario engaged and keeps offsets spread
    to 0.6 m through the second.
    """
    assert expected_engaged_share(0.6, 0.6, 0.3, (1, 2)) == pytest.approx(0.625)
    assert expected_engaged_share(0.3, 0.6, 0.3, (1, 2)) == pytest.approx(1.0)
    # And the sampler agrees with the arithmetic.
    rng = np.random.default_rng(7)
    engaged = []
    for _ in range(800):
        n = int(rng.integers(1, 3))
        got = place_on_route(rng, STRAIGHT, (10.0, 0.0), [], HALL, n, rule(), always)
        engaged.append(min(got.offsets_m) <= 0.3)
    assert np.mean(engaged) == pytest.approx(0.625, abs=0.05)
