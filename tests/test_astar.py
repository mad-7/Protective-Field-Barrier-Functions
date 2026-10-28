"""Known-answer tests for the global planner (hard rule 6). Pure; no simulator.

Two things matter beyond "it finds a path": every failure has a NAME rather than an
invented straight line (v1 returned one from a `try/except`, so an unreachable goal looked
like a plan through a shelf), and the simplified path never shaves a lethal corner.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.planning.astar import (
    EXPANSION_LIMIT,
    GOAL_LETHAL,
    GOAL_OUTSIDE,
    OK,
    START_LETHAL,
    START_OUTSIDE,
    UNREACHABLE,
    line_cells,
    octile,
    plan,
    plan_from_config,
    segment_is_clear,
)
from pfbnav.world.baking import Obstacle, bake

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
RADII = {"inscribed": 0.2086, "circumscribed": 0.33742, "lethal": 0.2386, "soft": 0.36742}
RES = 0.1
BOUNDS = (0.0, 0.0, 5.0, 5.0)


def empty_map(res=RES):
    return bake([], BOUNDS, res, RADII)


def wall(x0, x1, y0, y1, **kw):
    return Obstacle(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]), **kw)


# ----------------------------------------------------------------------
# Pieces
# ----------------------------------------------------------------------
def test_octile_known_answers():
    assert octile((0, 0), (0, 3)) == pytest.approx(3.0)
    assert octile((0, 0), (3, 3)) == pytest.approx(3 * math.sqrt(2.0))
    assert octile((0, 0), (1, 3)) == pytest.approx(2.0 + math.sqrt(2.0))


def test_line_cells_covers_a_diagonal_without_skipping_clipped_cells():
    cells = line_cells((0.5, 0.5), (2.5, 2.5))
    assert cells[0] == (0, 0) and cells[-1] == (2, 2)
    # A 45 deg line through cell corners touches the cells on both sides of each corner.
    assert (0, 1) in cells and (1, 0) in cells
    assert all(abs(r - c) <= 1 for r, c in cells)


def test_line_cells_on_an_axis_is_the_row_itself():
    assert line_cells((0.5, 0.5), (0.5, 3.5)) == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_line_cells_is_symmetric_in_its_endpoints():
    a = set(line_cells((0.5, 0.5), (3.5, 1.5)))
    b = set(line_cells((3.5, 1.5), (0.5, 0.5)))
    assert a == b


# ----------------------------------------------------------------------
# Planning
# ----------------------------------------------------------------------
def test_an_empty_map_gives_the_straight_line_after_simplification():
    r = plan(empty_map(), (0.5, 0.5), (4.5, 4.5), soft_weight=1.0)
    assert r.ok and r.status == OK
    assert len(r.path_cells) == 2, "a clear diagonal needs no intermediate waypoints"
    assert r.waypoints_before_simplification > 2
    assert r.length_m == pytest.approx(math.hypot(4.0, 4.0), abs=2 * RES)


def test_the_path_starts_at_the_start_cell_and_ends_at_the_goal_cell():
    m = empty_map()
    r = plan(m, (0.35, 1.25), (4.15, 3.05), soft_weight=1.0)
    assert np.allclose(r.path_world[0], m.world_of(*m.cell_of([0.35], [1.25]))[0])
    assert np.allclose(r.path_world[-1], m.world_of(*m.cell_of([4.15], [3.05]))[0])


def test_a_wall_forces_a_detour_around_its_end():
    m = bake([wall(2.0, 2.2, 0.0, 3.5, name="wall")], BOUNDS, RES, RADII)
    r = plan(m, (0.5, 0.5), (4.5, 0.5), soft_weight=1.0)
    assert r.ok
    assert r.length_m > 4.0, "a straight line would cross the wall"
    # It must pass beyond the wall's end (y > 3.5) somewhere along the way.
    assert r.path_world[:, 1].max() > 3.5
    assert all(segment_is_clear(m, a, b)
               for a, b in zip(r.path_cells[:-1], r.path_cells[1:], strict=True))


def test_a_goal_walled_off_is_unreachable_not_a_straight_line():
    """v1's failure mode: the fallback returned a line straight through the wall."""
    m = bake([wall(2.0, 2.2, 0.0, 5.0, name="divider")], BOUNDS, RES, RADII)
    r = plan(m, (0.5, 2.5), (4.5, 2.5), soft_weight=1.0)
    assert r.status == UNREACHABLE
    assert len(r.path_world) == 0 and math.isinf(r.cost)
    assert r.expansions > 0


def test_a_start_or_goal_in_the_lethal_tier_is_named():
    m = bake([wall(2.0, 2.2, 0.0, 5.0)], BOUNDS, RES, RADII)
    assert plan(m, (2.1, 2.5), (0.5, 0.5), soft_weight=1.0).status == START_LETHAL
    assert plan(m, (0.5, 0.5), (2.1, 2.5), soft_weight=1.0).status == GOAL_LETHAL


def test_a_start_or_goal_off_the_map_is_named():
    m = empty_map()
    assert plan(m, (-1.0, 0.5), (4.5, 4.5), soft_weight=1.0).status == START_OUTSIDE
    assert plan(m, (0.5, 0.5), (9.0, 4.5), soft_weight=1.0).status == GOAL_OUTSIDE


def test_an_expansion_limit_is_named_and_returns_no_path():
    r = plan(empty_map(), (0.5, 0.5), (4.5, 4.5), soft_weight=1.0, max_expansions=5)
    assert r.status == EXPANSION_LIMIT
    assert len(r.path_world) == 0
    assert r.expansions <= 5


def test_the_planner_will_not_squeeze_diagonally_between_two_lethal_cells():
    m = bake([wall(2.0, 2.4, 0.0, 2.4, name="lower"),
              wall(2.4, 2.8, 2.4, 5.0, name="upper")], BOUNDS, RES, RADII)
    r = plan(m, (0.5, 0.5), (4.5, 4.5), soft_weight=1.0)
    if r.ok:
        for a, b in zip(r.path_cells[:-1], r.path_cells[1:], strict=True):
            assert segment_is_clear(m, a, b)
    else:
        assert r.status == UNREACHABLE


def test_the_soft_tier_pushes_the_path_off_the_wall():
    """With the soft weight at zero the cheapest path hugs the lethal boundary; with it on,
    the planner pays a little length to move away."""
    # Start and end INSIDE the soft band: the wall's face is at y = 2.0 and the soft radius
    # is 0.367 m, so a path at y = 1.75 is in the band and a straight run stays there.
    m = bake([wall(0.0, 5.0, 2.0, 2.2, name="long_wall")], BOUNDS, RES, RADII)
    hugging = plan(m, (0.5, 1.75), (4.5, 1.75), soft_weight=0.0, simplify_path=False)
    pushed = plan(m, (0.5, 1.75), (4.5, 1.75), soft_weight=5.0, simplify_path=False)
    assert hugging.ok and pushed.ok
    # The MINIMUM y, not the maximum: both paths start and end at 1.75, so the maximum is
    # pinned by the endpoints and says nothing about what happens between them.
    assert hugging.path_world[:, 1].min() == pytest.approx(1.75, abs=RES)
    assert pushed.path_world[:, 1].min() < hugging.path_world[:, 1].min() - RES
    # ... and it pays for that in length, which is what "discourages, never forbids" means.
    assert pushed.length_m > hugging.length_m


def test_simplification_never_crosses_a_lethal_cell():
    m = bake([wall(1.0, 1.2, 0.0, 3.0, name="a"), wall(3.0, 3.2, 2.0, 5.0, name="b")],
             BOUNDS, RES, RADII)
    r = plan(m, (0.5, 0.5), (4.5, 4.5), soft_weight=1.0)
    assert r.ok
    for a, b in zip(r.path_cells[:-1], r.path_cells[1:], strict=True):
        assert segment_is_clear(m, a, b)
    assert len(r.path_cells) < r.waypoints_before_simplification


def test_simplification_does_not_change_reachability_or_the_endpoints():
    m = bake([wall(1.0, 1.2, 0.0, 3.0), wall(3.0, 3.2, 2.0, 5.0)], BOUNDS, RES, RADII)
    plain = plan(m, (0.5, 0.5), (4.5, 4.5), soft_weight=1.0, simplify_path=False)
    short = plan(m, (0.5, 0.5), (4.5, 4.5), soft_weight=1.0)
    assert plain.ok and short.ok
    assert plain.path_cells[0] == short.path_cells[0]
    assert plain.path_cells[-1] == short.path_cells[-1]
    assert short.length_m <= plain.length_m + 1e-9


def test_the_same_request_gives_the_same_plan():
    """Determinism within a process is claimed, so the heap's ties must not decide."""
    m = bake([wall(2.0, 2.2, 0.0, 3.5)], BOUNDS, RES, RADII)
    first = plan(m, (0.5, 0.5), (4.5, 0.5), soft_weight=1.0)
    for _ in range(3):
        again = plan(m, (0.5, 0.5), (4.5, 0.5), soft_weight=1.0)
        assert again.path_cells == first.path_cells
        assert again.cost == first.cost


def test_the_cost_of_a_free_straight_run_is_its_length():
    m = empty_map()
    r = plan(m, (0.55, 0.55), (0.55, 3.55), soft_weight=1.0, simplify_path=False)
    assert r.ok
    assert r.cost == pytest.approx(3.0, abs=RES)


def test_plan_from_config_uses_the_configured_weight():
    m = bake([wall(0.0, 5.0, 2.0, 2.2)], BOUNDS, RES, RADII)
    from_cfg = plan_from_config(m, (0.5, 1.0), (4.5, 1.0), CFG, simplify_path=False)
    direct = plan(m, (0.5, 1.0), (4.5, 1.0),
                  soft_weight=float(CFG["planner"]["soft_weight"]), simplify_path=False)
    assert from_cfg.path_cells == direct.path_cells


def test_a_negative_soft_weight_is_refused():
    with pytest.raises(ValueError):
        plan(empty_map(), (0.5, 0.5), (1.0, 1.0), soft_weight=-1.0)
