"""Regression check: does turning on the wheel spheres' collidable flag change the dynamics?

Ruling 1 on the WP0 results report: one G1b profile must give bit-identical trajectories
before and after the flag edit, and a difference stops the work. Two invocations of the
simulator are not bit-identical even with no change at all -- the G1b smoke test and the
full re-run differ from the first logged row (runs G1b_20260918T104143Z and ..104317Z) --
while repetitions inside one invocation always have been. So the check runs in one
process:

  1. load the scene, confirm the four spheres' flag is off;
  2. run the profile twice (A, B) -- the determinism control; A and B must be identical;
  3. turn the flag on through the API, and confirm a collection now sees a sphere;
  4. run the profile again (C); A and C must be identical.

The scene file is not modified: the researcher makes the same edit in the GUI afterwards,
and G0 then runs on the saved file.

Usage:
    python scripts/regress_collidable_flag.py --config configs/gates/g1b_rerun.yaml --seed 0
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.regression import compare_trajectories
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import ChassisGeometry, SceneObjects, body_point_in_ref
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
SPHERE_MARK = "wheel_respondable"
# A profile that loads all four wheels, rotates and translates: the most exposed to any
# change in the wheels' contact handling.
PROFILE, SPEED = "translate_while_rotating", 0.30
DISTANCE_M = 5.0
# A plate at body +y 0.204 m cuts the left spheres (to 0.2083) and misses the wheel meshes
# (to 0.1958); see tests/test_collection_semantics.py.
PLATE_OFFSET_M, PLATE_THICKNESS_M = 0.204, 0.006


def _g1b():
    spec = importlib.util.spec_from_file_location(
        "g1b_script", REPO_ROOT / "scripts" / "gate_g1b_tracking.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["g1b_script"] = module
    spec.loader.exec_module(module)
    return module


def collection_sees_sphere(client: SimClient, cfg, shapes: list[int]) -> bool:
    """Whether a collection of the robot's shapes registers a plate cutting only the spheres."""
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = client.sim
    client.start(settle_steps=20)
    try:
        base_ref = client.object(objects.base_ref)
        plate = int(sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [PLATE_THICKNESS_M, 3.0, 3.0]))
        sim.setObjectInt32Param(plate, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(plate, sim.shapeintparam_respondable, 0)
        sim.setObjectOrientation(plate, base_ref, [0.0, 0.0, geom.yaw_offset + math.pi / 2])
        sim.setObjectPosition(plate, base_ref, body_point_in_ref(
            [0.0, PLATE_OFFSET_M, 0.5], geom.yaw_offset).tolist())
        seen = bool(sim.checkCollision(plate, client.entity_for(shapes))[0])
        sim.removeObjects([plate])
    finally:
        client.stop()
    return seen


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    g1b = _g1b()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"REGRESS_FLAG_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))
    objects = SceneObjects.from_config(cfg)

    client.start(settle_steps=1)
    physics = client.physics_settings()
    shapes = client.shapes_in_tree(client.object(objects.robot_tree))
    spheres = {h: str(client.sim.getObjectAlias(h, 2)) for h in shapes
               if SPHERE_MARK in str(client.sim.getObjectAlias(h, 2))}
    before_flags = {spheres[h]: client.special_flags(h)["collidable"] for h in spheres}
    client.stop()
    print(f"physics in force: {physics}")
    if len(spheres) != 4:
        raise SystemExit(f"expected 4 wheel spheres, found {len(spheres)}: {spheres}")
    if any(before_flags.values()):
        raise SystemExit(f"a sphere's flag is already on in the saved scene ({before_flags}); "
                         "this check needs the unedited scene")

    # Every run is preceded by the same plate check, so A, B and C share one history and
    # a difference in C can only come from the flag.
    names = []
    for rep in (0, 1):
        seen_before = collection_sees_sphere(client, cfg, shapes)
        print(f"-- before the edit, run {'AB'[rep]}")
        g1b.run_once(client, cfg, PROFILE, SPEED, DISTANCE_M, run_dir, rep)
        names.append(f"{PROFILE}_{SPEED:.2f}_rep{rep}.csv")

    for h in spheres:
        client.set_collidable(h, True)
    seen_after = collection_sees_sphere(client, cfg, shapes)
    print("-- after the edit, run C")
    g1b.run_once(client, cfg, PROFILE, SPEED, DISTANCE_M, run_dir, 2)
    names.append(f"{PROFILE}_{SPEED:.2f}_rep2.csv")
    client.stop()

    control = compare_trajectories(run_dir / names[0], run_dir / names[1])
    edit = compare_trajectories(run_dir / names[0], run_dir / names[2])
    report = {
        "physics": physics, "profile": PROFILE, "speed": SPEED,
        "spheres": list(spheres.values()),
        "collection_sees_sphere": {"before": seen_before, "after": seen_after},
        "control_A_vs_B": control, "edit_A_vs_C": edit,
        "result": ("PASS" if control["identical"] and edit["identical"]
                   and not seen_before and seen_after else "STOP"),
    }
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\ncollection sees a sphere: before {seen_before}, after {seen_after} "
          "(expected False, True)")
    print(f"determinism control, A vs B: identical={control['identical']}")
    print(f"flag edit, A vs C:          identical={edit['identical']}")
    for label, r in (("A vs B", control), ("A vs C", edit)):
        if not r["identical"]:
            worst = {k: v for k, v in r["max_abs_difference"].items() if v}
            print(f"   {label}: first difference {r['first_difference']}; max |diff| {worst}")
    print(f"\n{report['result']}  ->  {run_dir}")
    if report["result"] == "STOP":
        print("Ruling 1: stop and report. Do not edit the scene until this is understood.")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
