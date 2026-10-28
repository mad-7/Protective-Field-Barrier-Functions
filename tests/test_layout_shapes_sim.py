"""A generated layout, built in a real instance (`pytest -m sim`). (WP1)

What only the simulator can answer about option 1 (2026-09-23): that the shapes go in where
the polygons say, that reading them back agrees (the per-episode check, item 4), that the
scanners actually see them at scan-plane height, that an unmapped obstacle is sensed and
counted while the planner's map has no trace of it, and that stopping the simulation takes
every shape with it -- which is what keeps handles from accumulating over an episode-shaped
workload (item 5).

Nothing is saved; the scene is the pinned one and is left as it was found.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.layout_shapes import (
    LayoutShapes,
    Pose2D,
    transform_placing,
    unmapped_absent_from_bake,
)
from pfbnav.sim.scanner import Scanner
from pfbnav.world.baking import bake, planner_radii
from pfbnav.world.layouts import generate

pytestmark = pytest.mark.sim

REPO = Path(__file__).resolve().parents[1]
SEED = 1_000_005
SETTLE = 10


@pytest.fixture(scope="module")
def layout(cfg):
    """A generated layout with one unmapped obstacle, and its baked map."""
    lay = generate(SEED, cfg, stratum="medium", unmapped_count=1, split="tune")
    table = FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"])
    radii = planner_radii(cfg, circumscribed=table.circumscribed)
    baked = bake(lay.obstacles, lay.bounds, float(cfg["planner"]["resolution"]), radii)
    return lay, baked


@pytest.fixture
def shapes(cfg, sim_client):
    built = LayoutShapes(sim_client, cfg)
    try:
        yield built
    finally:
        built.remove()


def place_around_robot(sim_client, cfg, lay, built) -> Pose2D:
    """Put the layout so one of its aisle centres sits where the robot already stands."""
    sim = sim_client.sim
    ref = sim_client.object(cfg["sim"]["objects"]["base_ref"])
    pos = [float(v) for v in sim.getObjectPosition(ref, sim.handle_world)]
    yaw = float(sim.getObjectOrientation(ref, sim.handle_world)[2])
    from pfbnav.world.scenarios import aisle_centres

    lane = aisle_centres(lay)[1]
    inner = lay.free_space()
    start = (0.5 * (inner[0] + inner[2]), lane, 0.0)
    transform = transform_placing(start, (pos[0], pos[1], yaw))
    built.instantiate(lay.obstacles, transform, floor_z=0.0)
    return transform


def test_every_shape_goes_in_where_its_polygon_says(cfg, sim_client, layout, shapes):
    """The per-episode check (item 4), run against a real scene."""
    lay, _baked = layout
    place_around_robot(sim_client, cfg, lay, shapes)
    problems = shapes.verify(lay.obstacles)
    assert problems == [], problems
    assert shapes.report()["alive"] == len(lay.obstacles)


def test_the_unmapped_obstacle_is_in_the_scene_and_absent_from_the_map(cfg, sim_client,
                                                                      layout, shapes):
    lay, baked = layout
    place_around_robot(sim_client, cfg, lay, shapes)
    assert lay.unmapped, "this layout was asked for an unmapped obstacle"
    for obstacle in lay.unmapped:
        assert obstacle.name in shapes.handles
        assert bool(sim_client.sim.isHandle(shapes.handles[obstacle.name]))
    assert unmapped_absent_from_bake(lay, baked) == []


def test_the_scanners_see_the_layout(cfg, sim_client, layout, shapes):
    """A shelf that the scanners cannot see is furniture, not an obstacle."""
    lay, _baked = layout
    scanner = Scanner(sim_client, cfg)
    sim_client.stop()
    scanner.install()
    sim_client.start(settle_steps=SETTLE)
    try:
        sim_client.step()
        empty = scanner.read(sim_time=sim_client.time())
        place_around_robot(sim_client, cfg, lay, shapes)
        sim_client.step()
        built = scanner.read(sim_time=sim_client.time())
        near_empty = int(np.sum(np.asarray(empty.filter_ranges) < 4.0))
        near_built = int(np.sum(np.asarray(built.filter_ranges) < 4.0))
        print(f"\nbins reading under 4 m: {near_empty} before, {near_built} after")
        assert near_built > near_empty, "the layout is invisible to the scanners"
        assert float(np.min(built.filter_ranges)) < float(np.min(empty.filter_ranges))
    finally:
        scanner.remove()
        sim_client.stop()
        sim_client.start(settle_steps=SETTLE)


def test_stopping_the_simulation_takes_every_shape_with_it(cfg, sim_client, layout):
    """The episode's cleanup, and why handles cannot accumulate across resets (item 5)."""
    lay, _baked = layout
    built = LayoutShapes(sim_client, cfg)
    place_around_robot(sim_client, cfg, lay, built)
    handles = list(built.handles.values())
    assert all(bool(sim_client.sim.isHandle(h)) for h in handles)
    sim_client.stop()
    sim_client.start(settle_steps=SETTLE)
    assert not any(bool(sim_client.sim.isHandle(h)) for h in handles)
    built.handles = {}          # they are gone; the bookkeeping follows


def test_repeated_reset_and_rebuild_does_not_accumulate_shapes(cfg, sim_client, layout):
    """Five episodes' worth: the scene must not grow (v1 accumulated at ~70k steps)."""
    lay, _baked = layout
    sim = sim_client.sim
    counts = []
    for _ in range(5):
        sim_client.stop()
        sim_client.start(settle_steps=SETTLE)
        built = LayoutShapes(sim_client, cfg)
        place_around_robot(sim_client, cfg, lay, built)
        counts.append(len([h for h in sim.getObjectsInTree(sim.handle_scene,
                                                           sim.sceneobject_shape, 0)
                           if str(sim.getObjectAlias(h)).startswith("layout_")]))
    assert len(set(counts)) == 1, f"shapes accumulated across resets: {counts}"
    assert counts[0] == len(lay.obstacles)
    sim_client.stop()
    sim_client.start(settle_steps=SETTLE)


def test_instantiating_twice_without_removing_is_refused(cfg, sim_client, layout, shapes):
    lay, _baked = layout
    transform = place_around_robot(sim_client, cfg, lay, shapes)
    with pytest.raises(RuntimeError, match="remove"):
        shapes.instantiate(lay.obstacles, transform)


def test_verify_catches_a_shape_that_moved(cfg, sim_client, layout, shapes):
    """The check must be able to fail, or it says nothing when it passes."""
    lay, _baked = layout
    place_around_robot(sim_client, cfg, lay, shapes)
    name = next(iter(shapes.handles))
    handle = shapes.handles[name]
    pos = [float(v) for v in sim_client.sim.getObjectPosition(handle,
                                                              sim_client.sim.handle_world)]
    sim_client.sim.setObjectPosition(handle, [pos[0] + 0.05, pos[1], pos[2]],
                                     sim_client.sim.handle_world)
    problems = shapes.verify(lay.obstacles)
    assert any(p["name"] == name and p["field"] == "position" for p in problems), problems
