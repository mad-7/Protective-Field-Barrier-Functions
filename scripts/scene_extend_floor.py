"""Remove /Wall and enlarge the floor from 3 x 3 to 7 x 7 tiles, as ONE scene edit (ruled
2026-10-10 and 2026-10-13). The RESEARCHER runs it.

    python scripts/scene_extend_floor.py --config configs/base.yaml --seed 0 --dry-run
    python scripts/scene_extend_floor.py --config configs/base.yaml --seed 0

**The ruling:** 35 x 35 m, the nine existing 5 m tiles kept byte-for-byte and forty added around
them, so a tile stays centred on the origin (an even count would put a seam under the start).
A scene-only edit under hard rule 12. This script makes it through the API rather than the GUI,
so every tile is an exact copy of the centre tile and the result is read back, not assumed:

  1. refuse unless the scene on disk is the pinned one (`create_run_dir` checks its hash);
  2. inventory the scene and dump every property of the floor's tiles (`sim.getProperties`);
  3. find what else stands over the 35 m square: a hall can be placed anywhere on it, so a shape
     there is an obstacle no scenario recorded. `/Wall` -- a 0.25 m configurable-wall ring at
     +-7.6 m, found 2026-10-10 -- is **removed, its whole tree** (ruled 2026-10-13), its properties
     kept in `wall_removed.json`; **anything else there is refused**, unruled;
  4. copy the centre tile (its collision element and its visible element, `copyPasteObjects`
     with nothing stripped) forty times under `sim.objects.floor`, at 5 m spacing, same height;
  5. check in memory: the 49 tiles tile +-17.5 m exactly; each copy's properties equal the
     template's except identity and placement; the nine originals' properties are unchanged;
  6. save over `sim.scene`, **reload the saved file**, and repeat every check on what was saved,
     plus an inventory diff: exactly the eighty new objects added, exactly the /Wall tree
     removed, nothing moved or resized, and nothing left standing on the floor.

`--dry-run` does 1-3 and reports, changing nothing. The old scene stays in git; the regression
(`scripts/regress_floor.py`) reads it from there.

Writes `runs/diag/FLOOR_EDIT_<timestamp>_seed<seed>/`: inventory_before.json, properties_before
.json, foreign_shapes.json, and after an edit inventory_after.json, inventory_diff.json,
properties_after.json, summary.json with the new scene's SHA-256 to pin.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.scene_geometry import (
    floor_tiles,
    property_differences,
    property_dump,
    shapes_outside,
    world_triangles,
)
from pfbnav.utils.runio import create_run_dir, file_sha256
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.extent import floor_covers, foreign_shapes_on, triangle_overlaps_square

REPO_ROOT = Path(__file__).resolve().parents[1]
TILE_M = 5.0                     # m, the ResizableFloor_5_25 tile's side
OLD_COUNT, NEW_COUNT = 3, 7      # tiles per side, before and after (ruled 2026-10-10)
NEW_HALF_EXTENT_M = NEW_COUNT * TILE_M / 2.0   # 17.5 m
GRID_TOL_M = 1e-6
WALL_PATH = "/Wall"              # removed in the same edit (ruled 2026-10-13)
# Property names that legitimately differ between a copy and its template, or across a reload:
# which object it is, and where it is. Matched as substrings, case-insensitively.
# `meshes`, `modelHash` and `vortex.materialUniqueId` were added on 2026-10-14, when the first edit
# refused on them: across the nine ORIGINAL tiles -- byte-identical geometry, one `mesh_sha256` --
# `meshes` are sequential mesh handles (10003176, 10003179, ...), `vortex.materialUniqueId`
# sequential ids (859, 863, ...) and `modelHash` differs on every tile
# (runs/diag/FLOOR_EDIT_20261013T214913Z_seed0/properties_before.json). Per-object identifiers,
# then; the geometry itself is compared through `mesh_sha256`, which is not ignored.
IDENTITY = ("handle", "uid", "alias", "name", "parent", "children", "objectid", "persistent",
            "meshes", "modelhash", "uniqueid")
PLACEMENT = ("pose", "position", "quaternion", "orientation", "matrix")


def _inventory_module():
    spec = importlib.util.spec_from_file_location(
        "scene_inventory", REPO_ROOT / "scripts" / "diag_scene_inventory.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["scene_inventory"] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, indent=2, default=float), encoding="utf-8")


def floor_tree_dump(sim, floor: int) -> dict[str, dict[str, Any]]:
    """Every object under the floor (tiles and their visible elements), by alias path."""
    return {str(sim.getObjectAlias(int(h), 2)): property_dump(sim, int(h))
            for h in sim.getObjectsInTree(floor, sim.handle_all, 1)}


def foreign_report(sim, floor: int, robot: int) -> list[dict[str, Any]]:
    """What stands over the 35 m square besides the robot and the floor, described fully enough
    to rule on: its tree, flags, plan extent, whether it encloses the origin, and scripts."""
    out = []
    shapes = shapes_outside(sim, floor, robot)
    triangles = {name: world_triangles(sim, h) for name, h in shapes.items()}
    for name in foreign_shapes_on(triangles, NEW_HALF_EXTENT_M):
        h = shapes[name]
        tris = triangles[name]
        xyz = np.vstack(tris)
        plan = np.linalg.norm(xyz[:, :2], axis=1)
        parent = int(sim.getObjectParent(h))
        siblings = ([int(x) for x in sim.getObjectsInTree(parent, sim.handle_all, 1)]
                    if parent != -1 else [])
        scripts = []
        for s in siblings:
            if int(sim.getObjectType(s)) == int(sim.sceneobject_script):
                code = sim.getStringProperty(s, "code", {"noError": True})
                scripts.append({"alias": str(sim.getObjectAlias(s, 2)),
                                "code": code if code is None else str(code)})
        flags = int(sim.getObjectSpecialProperty(h))
        out.append({
            "shape": name,
            "parent": str(sim.getObjectAlias(parent, 2)) if parent != -1 else None,
            "collidable": bool(flags & int(sim.objectspecialproperty_collidable)),
            "measurable": bool(flags & int(sim.objectspecialproperty_measurable)),
            "detectable": bool(flags & int(sim.objectspecialproperty_detectable)),
            "respondable": int(sim.getObjectInt32Param(h, sim.shapeintparam_respondable)),
            "static": int(sim.getObjectInt32Param(h, sim.shapeintparam_static)),
            "triangles": len(tris),
            "plan_min_xy_m": xyz[:, :2].min(axis=0).tolist(),
            "plan_max_xy_m": xyz[:, :2].max(axis=0).tolist(),
            "z_range_m": [float(xyz[:, 2].min()), float(xyz[:, 2].max())],
            # Nearest and farthest vertex from the origin in plan; a ring round the origin has a
            # large nearest distance AND covers no small square at the origin.
            "plan_vertex_distance_m": [float(plan.min()), float(plan.max())],
            "covers_origin": any(triangle_overlaps_square(t, 0.05) for t in tris),
            "on_old_floor": bool(foreign_shapes_on({name: tris}, OLD_COUNT * TILE_M / 2.0)),
            "scripts_beside_it": scripts,
        })
    return out


def check_old_floor(tiles: dict[int, tuple]) -> tuple[int, float]:
    """The nine tiles at 5 m spacing round the origin; returns the centre tile and its top z."""
    if len(tiles) != OLD_COUNT ** 2:
        raise SystemExit(f"expected {OLD_COUNT ** 2} floor tiles, found {len(tiles)}")
    if floor_covers([t[:4] for t in tiles.values()], OLD_COUNT * TILE_M / 2.0):
        raise SystemExit("the existing tiles do not tile +-7.5 m; this is not the scene ruled on")
    centre = [h for h, t in tiles.items() if math.hypot(t[0], t[1]) < GRID_TOL_M]
    if len(centre) != 1:
        raise SystemExit("no tile is centred on the origin")
    return centre[0], tiles[centre[0]][4]


def verify(sim, floor: int, template_props: dict[str, Any], visible_template_props: dict[str, Any],
           originals_before: dict[str, dict[str, Any]], copies: list[str]) -> list[str]:
    """Every check of step 5; returns the problems (empty = good)."""
    problems = []
    tiles = floor_tiles(sim, floor)
    problems += floor_covers([t[:4] for t in tiles.values()], NEW_HALF_EXTENT_M)
    if max(abs(t[4]) for t in tiles.values()) >= 1e-6:
        problems.append("a tile's top is not at z = 0")
    now = floor_tree_dump(sim, floor)
    for path, before in originals_before.items():
        if path not in now:
            problems.append(f"original {path} is gone")
            continue
        diff = property_differences(before, now[path], ignore=IDENTITY)
        if diff:
            problems.append(f"original {path} changed: {diff}")
    for path in copies:
        template = visible_template_props if "visible" in path.rsplit("/", 1)[-1].lower() \
            else template_props
        if path not in now:
            problems.append(f"copy {path} is missing")
            continue
        diff = property_differences(template, now[path], ignore=IDENTITY + PLACEMENT)
        if diff:
            problems.append(f"copy {path} differs from its template in {diff}")
    added = sorted(set(now) - set(originals_before) - set(copies))
    if added:
        problems.append(f"unexpected objects under the floor: {added}")
    return problems


def remove_wall(client: SimClient, cfg: dict[str, Any],
                foreign: list[dict[str, Any]]) -> tuple[dict[str, Any], list[str]]:
    """Remove the whole /Wall tree, in memory (ruled 2026-10-13). Refuses -- nothing changed --
    unless the shapes on the floor-to-be are exactly the wall's. Returns the removed objects'
    properties and their paths. Shared with `scripts/regress_floor.py`, which applies the same
    edit in memory within one scene load (ruled 2026-10-14)."""
    sim = client.sim
    floor = client.object(cfg["sim"]["objects"]["floor"])
    robot = client.object(cfg["sim"]["objects"]["robot_tree"])
    wall = client.object(WALL_PATH)
    wall_tree = [int(h) for h in sim.getObjectsInTree(wall, sim.handle_all, 0)]
    wall_paths = sorted(str(sim.getObjectAlias(h, 2)) for h in wall_tree)
    others = [f["shape"] for f in foreign if not f["shape"].startswith(WALL_PATH + "/")]
    if others or not foreign:
        raise SystemExit(
            f"the floor-to-be holds {[f['shape'] for f in foreign]}; the ruling removes {WALL_PATH} "
            f"and nothing else, so {others or 'its absence'} needs a ruling first. Nothing changed.")
    props = {str(sim.getObjectAlias(h, 2)): property_dump(sim, h) for h in wall_tree}
    sim.removeObjects(wall_tree)
    if bool(sim.isHandle(wall)) or foreign_report(sim, floor, robot):
        raise SystemExit("the wall is still in the scene after removal; nothing was saved")
    return props, wall_paths


def add_tiles(sim, floor: int, centre: int) -> tuple[list[str], dict[str, Any], dict[str, Any]]:
    """Copy the centre tile (collision and visible elements, nothing stripped) into the forty
    new grid positions under the floor, in memory. Returns the copies' paths and the two
    templates' properties. Shared with `scripts/regress_floor.py`."""
    visible = [int(h) for h in sim.getObjectsInTree(centre, sim.sceneobject_shape, 1 | 2)]
    if len(visible) != 1:
        raise SystemExit(f"the centre tile has {len(visible)} child shapes, expected one")
    template_props = property_dump(sim, centre)
    visible_template_props = property_dump(sim, visible[0])
    copies: list[str] = []
    k = (NEW_COUNT - 1) // 2
    for i in range(-k, k + 1):
        for j in range(-k, k + 1):
            if max(abs(i), abs(j)) <= (OLD_COUNT - 1) // 2:
                continue
            element, shown = (int(h) for h in sim.copyPasteObjects([centre, visible[0]], 0))
            if int(sim.getObjectParent(shown)) != element:
                raise SystemExit("the copied visible element is not under its copied tile")
            sim.setObjectParent(element, floor, True)
            sim.setObjectPosition(element, [TILE_M * i, TILE_M * j,
                                            float(sim.getObjectPosition(centre, sim.handle_world)[2])],
                                  sim.handle_world)
            tag = f"c{i + k}{j + k}"
            sim.setObjectAlias(element, f"ResizableFloor_5_25_element_{tag}")
            sim.setObjectAlias(shown, f"ResizableFloor_5_25_visibleElement_{tag}")
            copies += [str(sim.getObjectAlias(element, 2)), str(sim.getObjectAlias(shown, 2))]
    return copies, template_props, visible_template_props


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--dry-run", action="store_true",
                        help="report the floor and what stands on the 35 m square; change nothing")
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    inventory = _inventory_module()
    scene = REPO_ROOT / cfg["sim"]["scene"]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    # Refuses unless the scene on disk is the pinned one: the edit starts from what was ruled on.
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"FLOOR_EDIT_{stamp}", args.seed, cfg)
    old_sha = file_sha256(scene)

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.stop()
    client.load_scene(str(scene))
    sim = client.sim
    floor = client.object(cfg["sim"]["objects"]["floor"])
    robot = client.object(cfg["sim"]["objects"]["robot_tree"])

    before_inventory = inventory.inventory(client)
    _write(run_dir / "inventory_before.json",
           {"scene": cfg["sim"]["scene"], "scene_sha256": old_sha, "objects": before_inventory})
    tiles = floor_tiles(sim, floor)
    centre, top = check_old_floor(tiles)
    originals = floor_tree_dump(sim, floor)
    _write(run_dir / "properties_before.json", originals)
    foreign = foreign_report(sim, floor, robot)
    _write(run_dir / "foreign_shapes.json", foreign)

    print(f"scene {cfg['sim']['scene']} ({old_sha[:12]}...): {len(tiles)} tiles, centre tile "
          f"{sim.getObjectAlias(centre, 2)}, top z {top:+.2e} m")
    print(f"shapes standing over the {2 * NEW_HALF_EXTENT_M:.0f} m square besides the robot and "
          f"the floor: {len(foreign)}")
    for item in foreign:
        print(f"  {item['shape']} (parent {item['parent']}): collidable {item['collidable']}, "
              f"measurable {item['measurable']}, respondable {item['respondable']}, "
              f"{item['triangles']} triangles")
        print(f"    plan x {item['plan_min_xy_m'][0]:+.3f}..{item['plan_max_xy_m'][0]:+.3f}, "
              f"y {item['plan_min_xy_m'][1]:+.3f}..{item['plan_max_xy_m'][1]:+.3f}, "
              f"z {item['z_range_m'][0]:.3f}..{item['z_range_m'][1]:.3f} m; vertices "
              f"{item['plan_vertex_distance_m'][0]:.2f}..{item['plan_vertex_distance_m'][1]:.2f} m "
              f"from the origin; covers the origin: {item['covers_origin']}")
        for s in item["scripts_beside_it"]:
            first = (s["code"] or "<code not readable>").strip().splitlines()[:3]
            print(f"    script {s['alias']}: {' | '.join(first)}")
    if args.dry_run:
        client.stop()
        _write(run_dir / "summary.json", {"dry_run": True, "scene_sha256": old_sha,
                                          "tiles": len(tiles), "foreign_shapes": foreign})
        print(f"dry run: nothing changed -> {run_dir.relative_to(REPO_ROOT).as_posix()}")
        return 0
    wall_props, wall_paths = remove_wall(client, cfg, foreign)
    _write(run_dir / "wall_removed.json", wall_props)
    print(f"removed {wall_paths}")
    copies, template_props, visible_template_props = add_tiles(sim, floor, centre)

    problems = verify(sim, floor, template_props, visible_template_props, originals, copies)
    if problems:
        client.stop()
        _write(run_dir / "summary.json", {"saved": False, "problems": problems})
        raise SystemExit("in-memory check failed, NOT saved:\n  " + "\n  ".join(problems[:20]))

    sim.saveScene(str(scene))
    client.load_scene(str(scene))     # read back what was SAVED, not what is in memory
    sim = client.sim
    floor = client.object(cfg["sim"]["objects"]["floor"])
    robot = client.object(cfg["sim"]["objects"]["robot_tree"])
    after_inventory = inventory.inventory(client)
    new_sha = file_sha256(scene)
    _write(run_dir / "inventory_after.json",
           {"scene": cfg["sim"]["scene"], "scene_sha256": new_sha, "objects": after_inventory})
    report = inventory.diff(before_inventory, after_inventory)
    _write(run_dir / "inventory_diff.json", report)
    _write(run_dir / "properties_after.json", floor_tree_dump(sim, floor))
    problems = verify(sim, floor, template_props, visible_template_props, originals, copies)
    if sorted(report["added"]) != sorted(copies):
        problems.append(f"the inventory added {len(report['added'])} objects, expected the "
                        f"{len(copies)} copies")
    if sorted(report["removed"]) != wall_paths:
        problems.append(f"the inventory removed {report['removed']}, expected exactly {wall_paths}")
    if report["moved"] or report["resized"]:
        problems.append(f"moved {report['moved']}, resized {report['resized']}")
    if foreign_report(sim, floor, robot):
        problems.append("a shape stands on the saved floor")
    client.stop()

    summary = {"saved": True, "old_scene_sha256": old_sha, "new_scene_sha256": new_sha,
               "copies": copies, "inventory_diff": {k: len(v) if isinstance(v, list) else v
                                                    for k, v in report.items()},
               "problems": problems, "ok": not problems}
    _write(run_dir / "summary.json", summary)
    print(f"\nsaved and reloaded: {len(copies) // 2} tiles added; inventory diff: added "
          f"{len(report['added'])}, removed {len(report['removed'])}, moved {len(report['moved'])},"
          f" resized {len(report['resized'])}")
    if problems:
        print("READ-BACK FAILED -- do not commit; restore the scene with git:")
        for p in problems[:20]:
            print(f"  {p}")
        return 1
    print(f"read-back OK. New scene SHA-256 (pin as sim.scene_sha256):\n  {new_sha}")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
