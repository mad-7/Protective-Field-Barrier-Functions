"""Report every robot part that protrudes past the chassis, and the polygon footprint.

Requested at the WP0 review (D3, outstanding) and extended at the second review: how many
objects protrude, where they sit, how big they are, whether the envelope is symmetric, and
the inscribed and circumscribed radii of the polygon footprint in each body quadrant.

This is a **report, not a gate**. It asserts only that the measurement is self-consistent
and prints everything else, because which parts are allowed to protrude is a fact about the
robot rather than a property that can pass or fail.

Two cautions carried over from earlier measurements, both learned the hard way:

  * Per-shape extents come from each shape's own bounding box transformed into the body
    frame. That is exact for a shape whose box is axis-aligned there and conservative for
    one that is rotated -- a wheel's box over-states it by a few centimetres. The *polygon*
    inherits the same conservatism. `tests/test_footprint_probe.py` measures the true
    envelope with collision queries and is the authority where the two disagree.
  * `objbbox` is the object's own geometry; `modelbbox` would return the whole robot for
    any object in the tree. v1's map baker used the latter, which is correct for a
    standalone obstacle and wrong here.

Usage:
    python scripts/report_pods.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.footprint import convex_hull, quadrant_radii, symmetry_report
from pfbnav.sim.robot import (
    ChassisGeometry,
    SceneObjects,
    bbox_corners_in_body,
    chassis_extent_in_body,
)
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

# A part counts as protruding when it reaches past the chassis by more than this, in
# metres. Below it the difference is inside the measurement's own conservatism.
PROTRUSION_TOL_M = 0.002


def survey(client: SimClient, cfg: dict[str, Any]) -> dict[str, Any]:
    """Measure every robot shape in the body frame and build the polygon footprint."""
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = client.sim
    base_ref = client.object(objects.base_ref)
    chassis_handle = client.object(objects.chassis)

    lo, hi = client.object_bbox(chassis_handle)
    chassis = chassis_extent_in_body(
        lo, hi, sim.getObjectMatrix(chassis_handle, base_ref), geom.yaw_offset
    )

    parts, all_corners = [], []
    for handle in client.shapes_in_tree(client.object(objects.robot_tree)):
        lo, hi = client.object_bbox(handle)
        matrix = sim.getObjectMatrix(handle, base_ref)
        corners = bbox_corners_in_body(lo, hi, matrix, geom.yaw_offset)
        all_corners.extend(corners[:, :2].tolist())
        half = np.max(np.abs(corners), axis=0)
        centre = corners.mean(axis=0)
        parts.append({
            "name": str(sim.getObjectAlias(handle, 2)),
            "centre_body": [round(float(v), 4) for v in centre],
            "size": [round(float(v), 4) for v in (corners.max(axis=0) - corners.min(axis=0))],
            "reach_x": float(half[0]),
            "reach_y": float(half[1]),
            "beyond_chassis_x": float(half[0] - chassis[0]),
            "beyond_chassis_y": float(half[1] - chassis[1]),
        })

    for part in parts:
        part["protrudes"] = bool(
            part["beyond_chassis_x"] > PROTRUSION_TOL_M
            or part["beyond_chassis_y"] > PROTRUSION_TOL_M
        )
    parts.sort(key=lambda p: max(p["beyond_chassis_x"], p["beyond_chassis_y"]), reverse=True)

    hull = convex_hull(all_corners)
    radii = quadrant_radii(hull, samples=361)
    return {
        "chassis_half_extents": [round(v, 4) for v in chassis],
        "configured_envelope": [geom.half_length, geom.half_width],
        "shape_count": len(parts),
        "protruding_count": sum(1 for p in parts if p["protrudes"]),
        "parts": parts,
        "hull_vertices_body": [[round(float(x), 4), round(float(y), 4)] for x, y in hull],
        "quadrant_radii": radii,
        "symmetry": symmetry_report(radii),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"PODS_{stamp}", args.seed, cfg)

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))
    try:
        client.start(settle_steps=20)
        report = survey(client, cfg)
    finally:
        client.stop()

    (run_dir / "pods_report.json").write_text(json.dumps(report, indent=2, default=float))

    chassis = report["chassis_half_extents"]
    print(f"\nchassis half-extents      {chassis[0]:.4f} x {chassis[1]:.4f} m")
    print(f"configured envelope       {report['configured_envelope'][0]:.4f} x "
          f"{report['configured_envelope'][1]:.4f} m")
    print(f"robot shapes              {report['shape_count']}, of which "
          f"{report['protruding_count']} protrude past the chassis by more than "
          f"{PROTRUSION_TOL_M} m\n")
    print(f"{'beyond x':>9} {'beyond y':>9} {'reach x':>8} {'reach y':>8} "
          f"{'size (x,y,z)':>24}  part")
    for p in report["parts"]:
        if not p["protrudes"]:
            continue
        print(f"{p['beyond_chassis_x']:+9.4f} {p['beyond_chassis_y']:+9.4f} "
              f"{p['reach_x']:8.4f} {p['reach_y']:8.4f} "
              f"{tuple(p['size'])!s:>24}  {p['name']}")

    print("\npolygon footprint, per body quadrant (m):")
    print(f"{'quadrant':>12} {'inscribed':>10} {'at deg':>7} {'circumscribed':>14} {'at deg':>7}")
    for name, v in report["quadrant_radii"].items():
        print(f"{name:>12} {v['inscribed']:10.4f} {v['inscribed_bearing_deg']:7.1f} "
              f"{v['circumscribed']:14.4f} {v['circumscribed_bearing_deg']:7.1f}")

    sym = report["symmetry"]
    print(f"\nsymmetry: fore-aft difference {sym['fore_aft_difference']:.4f} m, "
          f"left-right {sym['left_right_difference']:.4f} m")
    print(f"hull has {len(report['hull_vertices_body'])} vertices  ->  {run_dir}")
    print("\nThese are bounding-box derived and conservative for rotated parts; "
          "tests/test_footprint_probe.py measures the true envelope by collision query.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
