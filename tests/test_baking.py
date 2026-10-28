"""Known-answer tests for map baking (hard rule 6). Pure; no simulator.

The bake decides what the global planner believes, so an error here is invisible at run
time and shows up as "the planner would not use that aisle". Every case has an answer
worked out by hand or by area.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.world.baking import (
    Obstacle,
    bake,
    cell_overlap,
    inside_convex,
    planner_radii,
    polygon_distance,
    soft_cost,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
RADII = {"inscribed": 0.2086, "circumscribed": 0.33742, "lethal": 0.2386, "soft": 0.36742}
UNIT_SQUARE = np.array([[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]])


def rotated_square(centre, side, angle_deg):
    """A square of `side` rotated about its centre: the case a bounding box gets wrong."""
    h = side / 2
    corners = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    a = math.radians(angle_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return corners @ rot.T + np.asarray(centre, dtype=float)


# ----------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------
def test_polygon_distance_known_answers():
    d = polygon_distance([[1.5, 2.5], [0.5, 1.5], [1.5, 1.5], [0.0, 0.0]], UNIT_SQUARE)
    assert d[0] == pytest.approx(0.5)            # straight above the top edge
    assert d[1] == pytest.approx(0.5)            # straight left of the left edge
    assert d[2] == pytest.approx(0.0)            # inside
    assert d[3] == pytest.approx(math.hypot(1.0, 1.0))   # diagonal from the near corner


def test_inside_convex_counts_the_boundary_as_inside():
    got = inside_convex([[1.5, 1.5], [1.0, 1.5], [2.0, 2.0], [0.999, 1.5]], UNIT_SQUARE)
    assert list(got) == [True, True, True, False]


def test_a_cell_sharing_only_an_edge_is_not_occupied():
    # Cell centre 0.75 with 0.5 cells spans [0.5, 1.0]; the square starts at x = 1.0.
    assert not cell_overlap(UNIT_SQUARE, [[0.75, 1.25]], 0.5)[0]
    assert cell_overlap(UNIT_SQUARE, [[1.25, 1.25]], 0.5)[0]


def test_an_axis_aligned_square_marks_exactly_its_own_cells():
    m = bake([Obstacle(UNIT_SQUARE)], (0.0, 0.0, 4.0, 4.0), 0.5, RADII)
    assert int(m.occupied.sum()) == 4            # 1 m^2 at 0.5 m cells, edges not counted
    assert m.shape == (8, 8)


def test_a_rotated_square_marks_far_fewer_cells_than_its_bounding_box():
    """The v1 error, made concrete: a 45 deg shelf's box is twice its area."""
    side, res = 1.0, 0.1
    poly = rotated_square((2.5, 2.5), side, 45.0)
    m = bake([Obstacle(poly)], (0.0, 0.0, 5.0, 5.0), res, RADII)
    cells = int(m.occupied.sum())
    # Bounds, not a fudge: every cell whose centre is inside is marked (at least the area),
    # and the extra can only be cells the boundary passes through (at most the perimeter
    # divided by the cell, plus the four corners).
    area_cells = side ** 2 / res ** 2
    rim_cells = 4 * side / res + 4
    assert area_cells <= cells <= area_cells + rim_cells, cells
    box_side = poly[:, 0].max() - poly[:, 0].min()
    box_cells = (box_side / res) ** 2
    assert cells < 0.8 * box_cells, f"{cells} cells against a box's {box_cells:.0f}"


def test_every_cell_whose_centre_lies_inside_an_obstacle_is_occupied():
    poly = rotated_square((2.5, 2.5), 1.0, 30.0)
    m = bake([Obstacle(poly)], (0.0, 0.0, 5.0, 5.0), 0.1, RADII)
    rows, cols = np.nonzero(~m.occupied)
    centres = m.world_of(rows, cols)
    assert not inside_convex(centres, poly).any()


# ----------------------------------------------------------------------
# Tiers
# ----------------------------------------------------------------------
def test_soft_cost_is_one_at_the_lethal_radius_and_zero_at_the_soft_one():
    assert soft_cost([0.0], 0.2, 0.4)[0] == pytest.approx(1.0)
    assert soft_cost([0.2], 0.2, 0.4)[0] == pytest.approx(1.0)
    assert soft_cost([0.3], 0.2, 0.4)[0] == pytest.approx(0.5)
    assert soft_cost([0.4], 0.2, 0.4)[0] == pytest.approx(0.0)
    assert soft_cost([5.0], 0.2, 0.4)[0] == pytest.approx(0.0)


def test_soft_cost_refuses_a_soft_radius_inside_the_lethal_one():
    with pytest.raises(ValueError):
        soft_cost([0.1], 0.4, 0.2)


def test_the_lethal_tier_is_exactly_the_cells_within_the_lethal_radius():
    m = bake([Obstacle(UNIT_SQUARE)], (0.0, 0.0, 4.0, 4.0), 0.05, RADII)
    assert np.array_equal(m.lethal, m.clearance_m <= RADII["lethal"])
    assert m.cost[m.lethal].min() == pytest.approx(1.0)


def test_clearance_is_infinite_when_the_layout_has_no_mapped_obstacle():
    m = bake([], (0.0, 0.0, 2.0, 2.0), 0.5, RADII)
    assert np.isinf(m.clearance_m).all()
    assert not m.lethal.any() and not m.occupied.any()
    assert m.cost.max() == pytest.approx(0.0)


def test_the_planner_radii_come_from_the_config_and_the_measured_footprint():
    radii = planner_radii(CFG, circumscribed=0.33742)
    assert radii["inscribed"] == pytest.approx(CFG["robot"]["half_width"])
    assert radii["lethal"] == pytest.approx(
        CFG["robot"]["half_width"] + CFG["planner"]["lethal_margin"])
    assert radii["soft"] == pytest.approx(0.33742 + CFG["planner"]["soft_margin"])


# ----------------------------------------------------------------------
# Unmapped obstacles
# ----------------------------------------------------------------------
def test_an_unmapped_obstacle_is_absent_from_every_layer_and_is_recorded():
    mapped = Obstacle(UNIT_SQUARE, name="shelf_a")
    hidden = Obstacle(UNIT_SQUARE + np.array([2.0, 0.0]), mapped=False, name="pallet")
    both = bake([mapped, hidden], (0.0, 0.0, 6.0, 6.0), 0.1, RADII)
    alone = bake([mapped], (0.0, 0.0, 6.0, 6.0), 0.1, RADII)
    assert np.array_equal(both.occupied, alone.occupied)
    assert np.array_equal(both.lethal, alone.lethal)
    assert np.allclose(both.clearance_m, alone.clearance_m)
    assert both.unmapped == ["pallet"]
    assert alone.unmapped == []


# ----------------------------------------------------------------------
# Indexing
# ----------------------------------------------------------------------
def test_cell_and_world_round_trip():
    m = bake([], (-1.0, -2.0, 3.0, 4.0), 0.25, RADII)
    row, col = m.cell_of([0.3], [1.1])
    centre = m.world_of(row, col)[0]
    assert abs(centre[0] - 0.3) <= 0.125 and abs(centre[1] - 1.1) <= 0.125
    back = m.cell_of([centre[0]], [centre[1]])
    assert back[0][0] == row[0] and back[1][0] == col[0]


def test_a_point_outside_the_map_is_an_error_not_a_clamped_cell():
    m = bake([], (0.0, 0.0, 1.0, 1.0), 0.25, RADII)
    with pytest.raises(ValueError, match="outside the baked map"):
        m.cell_of([1.5], [0.5])


def test_free_for_centre_follows_the_lethal_tier():
    m = bake([Obstacle(UNIT_SQUARE)], (0.0, 0.0, 4.0, 4.0), 0.05, RADII)
    assert not m.free_for_centre([1.5], [1.5])[0]          # inside the obstacle
    assert not m.free_for_centre([0.95], [1.5])[0]          # 0.05 m off it: within lethal
    assert m.free_for_centre([0.5], [1.5])[0]               # 0.5 m off it: free


def test_the_grid_covers_bounds_that_do_not_divide_by_the_resolution():
    m = bake([], (0.0, 0.0, 1.01, 1.0), 0.25, RADII)
    assert m.shape == (4, 5)                                # rounded outwards, never short


def test_bake_refuses_nonsense():
    with pytest.raises(ValueError):
        bake([], (0.0, 0.0, 0.0, 1.0), 0.25, RADII)
    with pytest.raises(ValueError):
        bake([], (0.0, 0.0, 1.0, 1.0), 0.0, RADII)
    with pytest.raises(ValueError):
        bake([], (0.0, 0.0, 1.0, 1.0), 0.25, {"lethal": 0.4, "soft": 0.2})


def test_an_obstacle_needs_at_least_three_points():
    with pytest.raises(ValueError):
        Obstacle(np.array([[0.0, 0.0], [1.0, 0.0]]))
