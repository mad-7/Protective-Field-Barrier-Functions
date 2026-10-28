"""The scene's floor is what the config says, and nothing else stands on it (`pytest -m sim`).

The floor's half of the extent guarantee (`pfbnav/world/extent.py`; ruled 2026-10-10). The fast
guard shows every built world fits inside `sim.floor_half_extent_m` at any placement yaw; these
read the scene back (hard rules 13 and 15) and show that square really is floor:

  * the tiles under `sim.objects.floor` tile [-h, h]^2 exactly, axis-aligned, every top at z = 0;
  * no tile can register contact, on any tile, for any part of the robot: every tile carries the
    centre tile's flags, and a probe resting on each tile's surface is unseen by both obstacle
    sets the contact check uses. The tiles are NOT collidable (measured 2026-10-13), so the floor
    is outside both sets by its flag, not only the wheels' set by exclusion. The positive control
    is a collidable target overlapping the probe by half its width, which both sets must see.
    (Overlapping, not enclosing: the control's first version put the probe wholly inside a larger
    target and saw nothing on any tile, 2026-10-13 -- read as the check comparing surfaces, which
    this version tests.) Without it,
    "unseen" would read the same if the probe or the collections were broken (it failed that way
    on 2026-10-13, when the test assumed the tiles were collidable);
  * no shape outside the robot and the floor stands over the square, because a hall can be
    placed anywhere on it;
  * the robot's scene pose is within `sim.robot_spawn_radius_m` of the origin, the spawn the
    fast guard assumes.

Hard rule 18: the whole operating envelope, not the patch round the origin every earlier probe
used. The scene is left as it was found.
"""
from __future__ import annotations

import math

import pytest

from pfbnav.sim.scene_geometry import FLAT_TOL, floor_tiles, shapes_outside, world_triangles
from pfbnav.world.extent import floor_covers, foreign_shapes_on

pytestmark = pytest.mark.sim

PROBE_M = 0.02       # m, the probe cube's side; it straddles the tile's top surface


def test_the_floor_tiles_the_declared_square_flat_at_z_zero(cfg, sim_client):
    sim = sim_client.sim
    tiles = floor_tiles(sim, sim_client.object(cfg["sim"]["objects"]["floor"]))
    h = float(cfg["sim"]["floor_half_extent_m"])
    print(f"\n{len(tiles)} tiles; declared floor +-{h} m")
    assert floor_covers([t[:4] for t in tiles.values()], h) == []
    assert max(abs(t[4]) for t in tiles.values()) < FLAT_TOL, "a tile's top is not at z = 0"


def test_no_tile_can_register_contact_and_every_tile_matches_the_centre(cfg, sim_client):
    sim = sim_client.sim
    floor = sim_client.object(cfg["sim"]["objects"]["floor"])
    robot = sim_client.object(cfg["sim"]["objects"]["robot_tree"])
    tiles = floor_tiles(sim, floor)
    flags = {h: sim_client.special_flags(h) for h in tiles}
    centre = min(tiles, key=lambda h: math.hypot(tiles[h][0], tiles[h][1]))
    assert all(f == flags[centre] for f in flags.values()), f"tile flags differ: {flags}"

    probe = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, [PROBE_M] * 3, 0))
    target = int(sim.createPrimitiveShape(sim.primitiveshape_cuboid, [PROBE_M] * 3, 0))
    try:
        for shape in (probe, target):
            sim.setObjectInt32Param(shape, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(shape, sim.shapeintparam_respondable, 0)
        sim.setObjectSpecialProperty(target, int(sim.objectspecialproperty_collidable))
        everything = sim_client.collection_of_everything_except(robot, probe)
        no_floor = sim_client.collection_of_everything_except(robot, floor, probe)
        control_missed, tile_seen = [], []
        for cx, cy, _hx, _hy, top in tiles.values():
            sim.setObjectPosition(probe, [cx, cy, top], sim.handle_world)
            sim.setObjectPosition(target, [cx + PROBE_M / 2, cy, top], sim.handle_world)
            if not (sim.checkCollision(probe, everything)[0]
                    and sim.checkCollision(probe, no_floor)[0]):
                control_missed.append((cx, cy))
            sim.setObjectPosition(target, [cx, cy, top + 50.0], sim.handle_world)
            if sim.checkCollision(probe, everything)[0] or sim.checkCollision(probe, no_floor)[0]:
                tile_seen.append((cx, cy))
    finally:
        sim.removeObjects([probe, target])
    print(f"\ntile flags {flags[centre]} on all {len(tiles)} tiles")
    assert not control_missed, f"control: a collidable target here went unseen: {control_missed}"
    assert not tile_seen, f"a contact set sees these tiles: {tile_seen}"


def test_nothing_but_the_robot_and_the_floor_stands_on_the_floor(cfg, sim_client):
    sim = sim_client.sim
    floor = sim_client.object(cfg["sim"]["objects"]["floor"])
    robot = sim_client.object(cfg["sim"]["objects"]["robot_tree"])
    shapes = {name: world_triangles(sim, h)
              for name, h in shapes_outside(sim, floor, robot).items()}
    h = float(cfg["sim"]["floor_half_extent_m"])
    print(f"\nshapes outside the robot and the floor: {sorted(shapes)}")
    assert foreign_shapes_on(shapes, h) == [], "a hall can be placed over these"


def test_the_robot_spawns_where_the_guard_assumes(cfg, sim_client):
    sim = sim_client.sim
    ref = sim_client.object(cfg["sim"]["objects"]["base_ref"])
    x, y, _z = (float(v) for v in sim.getObjectPosition(ref, sim.handle_world))
    print(f"\nbase_ref at ({x * 1e3:+.3f}, {y * 1e3:+.3f}) mm")
    assert math.hypot(x, y) <= float(cfg["sim"]["robot_spawn_radius_m"])
