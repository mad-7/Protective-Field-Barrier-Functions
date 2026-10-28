"""Every built world fits on the scene's floor (ruled 2026-10-10), and the checks that say so.

Known answers first (hard rule 6), then the guard itself over every built scenario set. The
guard reads `scenarios/test/` too once A14 builds it, as the ruling asked: an extent check is
geometry, not tuning (hard rule 2), and nothing there is modified (hard rule 1).

Until the floor is enlarged (`sim.floor_half_extent_m` 7.5 today) the guard FAILS on every set,
correctly: the halls reach 14.2 m.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.world.baking import Obstacle
from pfbnav.world.extent import (
    fits_on_floor,
    floor_covers,
    foreign_shapes_on,
    triangle_overlaps_square,
    world_reach,
)
from pfbnav.world.layouts import Layout
from pfbnav.world.scenarios import Scenario, world_of

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")
SETS = sorted((REPO / "scenarios").rglob("*.json"))


def _square(cx, cy, half):
    return [[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half],
            [cx - half, cy + half]]


def _layout(mapped=(), unmapped=(), bounds=(0.0, 0.0, 14.0, 10.0)):
    return Layout(obstacles=[*mapped, *unmapped], bounds=bounds)


# --- known answers -----------------------------------------------------------------------

def test_the_reach_of_a_bare_hall_is_its_far_corner():
    assert world_reach(_layout(), (0.0, 0.0)) == pytest.approx(math.hypot(14.0, 10.0))
    assert world_reach(_layout(), (7.0, 5.0)) == pytest.approx(math.hypot(7.0, 5.0))


def test_an_unmapped_obstacle_outside_the_bounds_sets_the_reach():
    """Item (b) of the ruling: every unmapped obstacle counts, not only the hall bounds."""
    stray = Obstacle(np.array(_square(20.0, 0.0, 0.5)), mapped=False, name="box_0")
    assert world_reach(_layout(unmapped=[stray]), (0.0, 0.0)) == pytest.approx(
        math.hypot(20.5, 0.5))


def test_a_mapped_obstacle_outside_the_bounds_sets_the_reach():
    shelf = Obstacle(np.array(_square(0.0, -20.0, 1.0)), mapped=True, name="shelf_0")
    assert world_reach(_layout(mapped=[shelf]), (0.0, 0.0)) == pytest.approx(
        math.hypot(1.0, 21.0))


def test_fitting_is_the_disc_inside_the_square():
    assert fits_on_floor(17.2, 0.01, 17.5)
    assert not fits_on_floor(17.2, 0.01, 7.5)
    assert fits_on_floor(17.49, 0.01, 17.5) and not fits_on_floor(17.5, 0.01, 17.5)


def test_seven_by_seven_tiles_cover_the_ruled_floor_and_nine_do_not():
    seven = [(5.0 * i, 5.0 * j, 2.5, 2.5) for i in range(-3, 4) for j in range(-3, 4)]
    nine = [(5.0 * i, 5.0 * j, 2.5, 2.5) for i in range(-1, 2) for j in range(-1, 2)]
    assert floor_covers(seven, 17.5) == []
    assert floor_covers(nine, 7.5) == []
    assert floor_covers(nine, 17.5), "nine tiles must not pass as a 35 m floor"


def test_a_missing_tile_an_overlap_and_a_stray_tile_are_each_reported():
    """Positive controls for the floor check: each defect alone must be caught."""
    seven = [(5.0 * i, 5.0 * j, 2.5, 2.5) for i in range(-3, 4) for j in range(-3, 4)]
    gap = [t for t in seven if t[:2] != (15.0, -10.0)]
    assert any("cover" in p for p in floor_covers(gap, 17.5))
    doubled = [*seven[:-1], (seven[0][0] + 1.0, seven[0][1], 2.5, 2.5)]
    assert any("overlap" in p for p in floor_covers(doubled, 17.5))
    stray = [*seven, (20.0, 0.0, 2.5, 2.5)]
    assert any("past" in p for p in floor_covers(stray, 17.5))


def test_triangle_against_square_known_answers():
    assert triangle_overlaps_square([[0, 0], [1, 0], [0, 1]], 5.0)            # inside
    assert triangle_overlaps_square([[-9, -9], [9, -9], [0, 9]], 5.0)         # around it
    assert not triangle_overlaps_square([[6, 0], [8, 0], [7, 1]], 5.0)        # beside it
    assert not triangle_overlaps_square([[5, -1], [6, 0], [5, 1]], 5.0)       # touching an edge
    # Corners overlap on both axes but the hypotenuse separates them: only the edge normal
    # can tell, so this is the case that proves the third family of axes is there.
    assert not triangle_overlaps_square([[5.5, 4.6], [4.6, 5.5], [6, 6]], 5.0)


def test_a_perimeter_wall_on_the_floor_is_foreign_and_one_beyond_it_is_not():
    """The shape found in the scene on 2026-10-10: a 0.1 m wall ring at +-7.55 m."""
    def ring(r, t=0.05):
        out = []
        for (x0, y0, x1, y1) in ((r - t, -r - t, r + t, r + t), (-r - t, -r - t, -r + t, r + t),
                                 (-r - t, r - t, r + t, r + t), (-r - t, -r - t, r + t, -r + t)):
            out += [[[x0, y0], [x1, y0], [x1, y1]], [[x0, y0], [x1, y1], [x0, y1]]]
        return out
    shapes = {"/Wall/Cuboid": ring(7.55), "/fence": ring(17.6)}
    assert foreign_shapes_on(shapes, 17.5) == ["/Wall/Cuboid"]
    assert foreign_shapes_on(shapes, 7.5) == []


# --- the guard ---------------------------------------------------------------------------

def test_there_are_sets_to_guard():
    names = {p.relative_to(REPO).as_posix() for p in SETS}
    assert {"scenarios/tune/tune_narrow.json", "scenarios/pilot/pilot_narrow.json",
            "scenarios/train/train_narrow.json"} <= names


@pytest.mark.parametrize("path", SETS, ids=lambda p: p.relative_to(REPO).as_posix())
def test_every_world_in_the_set_fits_on_the_floor(path):
    h = float(CFG["sim"]["floor_half_extent_m"])
    spawn = float(CFG["sim"]["robot_spawn_radius_m"])
    worst, outside = 0.0, []
    for record in json.loads(path.read_text(encoding="utf-8"))["scenarios"]:
        scenario = Scenario(**record)
        reach = world_reach(world_of(scenario, CFG), scenario.start)
        worst = max(worst, reach)
        if not fits_on_floor(reach, spawn, h):
            outside.append(scenario.seed)
    print(f"\n{path.name}: worst reach {worst:.2f} m against a floor of +-{h} m")
    assert not outside, (f"{len(outside)} worlds reach past the floor (+-{h} m; worst "
                         f"{worst:.2f} m + spawn {spawn} m), e.g. seeds {outside[:5]}")
