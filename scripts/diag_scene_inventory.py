"""Inventory a scene, and diff two inventories. For verifying what a scene edit changed.

Hard rule 12: a scene edit is verified by reading back WHAT CHANGED before it is committed.
For a geometry edit that means measuring the geometry (`probe_scanner_layout.py`); for an
edit that is supposed to change nothing measurable -- moving the camera, say -- it means
showing that nothing measurable changed. This records every object's identity, type, world
pose and bounding box, and `--against` compares two such records.

Usage:
    python scripts/diag_scene_inventory.py --config configs/base.yaml --seed 0 \
        --out docs/scene/inventory_before.json
    python scripts/diag_scene_inventory.py --config configs/base.yaml --seed 0 \
        --out docs/scene/inventory_after.json \
        --against docs/scene/inventory_before.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import file_sha256

REPO_ROOT = Path(__file__).resolve().parents[1]
# Below this an object counts as unmoved: the simulator reports poses in single precision
# in places, and a scene reload can differ in the last bit without anything having moved.
POSE_TOL_M = 1e-6
POSE_TOL_RAD = 1e-6


def inventory(client: SimClient) -> dict[str, Any]:
    """Every object in the scene, by path: type, world pose, and bounding box where it has one."""
    sim = client.sim
    out: dict[str, Any] = {}
    for handle in (int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)):
        path = str(sim.getObjectAlias(handle, 2))
        entry: dict[str, Any] = {
            "type": int(sim.getObjectType(handle)),
            "position": [float(v) for v in sim.getObjectPosition(handle, sim.handle_world)],
            "orientation": [float(v) for v in sim.getObjectOrientation(handle,
                                                                       sim.handle_world)],
        }
        half = sim.getVector3Property(handle, "bbHSize", {"noError": True})
        if half is not None:
            entry["half_size"] = [float(v) for v in half]
        out[path] = entry
    return out


def diff(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    """What changed between two inventories: objects added, removed, moved or resized."""
    added = sorted(set(after) - set(before))
    removed = sorted(set(before) - set(after))
    moved, resized = [], []
    for path in sorted(set(before) & set(after)):
        a, b = before[path], after[path]
        dp = max((abs(x - y) for x, y in zip(a["position"], b["position"], strict=True)),
                 default=0.0)
        do = max((abs(x - y) for x, y in zip(a["orientation"], b["orientation"],
                                             strict=True)), default=0.0)
        if dp > POSE_TOL_M or do > POSE_TOL_RAD:
            moved.append({"path": path, "position_delta_m": dp, "orientation_delta_rad": do})
        if a.get("half_size") and b.get("half_size"):
            ds = max(abs(x - y) for x, y in zip(a["half_size"], b["half_size"], strict=True))
            if ds > POSE_TOL_M:
                resized.append({"path": path, "half_size_delta_m": ds})
    return {"added": added, "removed": removed, "moved": moved, "resized": resized,
            "unchanged": len(set(before) & set(after)) - len(moved) - len(resized),
            "identical": not (added or removed or moved or resized)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--scene", help="a scene to inventory instead of the configured one")
    parser.add_argument("--out", required=True)
    parser.add_argument("--against", help="an earlier inventory to compare with")
    args = parser.parse_args()

    cfg = load_config(args.config)
    scene = REPO_ROOT / (args.scene or cfg["sim"]["scene"])
    if not scene.is_file():
        raise SystemExit(f"{scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.stop()
    client.load_scene(str(scene))
    try:
        record = {"scene": str(scene.relative_to(REPO_ROOT)),
                  "scene_sha256": file_sha256(scene), "objects": inventory(client)}
    finally:
        client.stop()

    out = REPO_ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(record, indent=2, default=float))
    print(f"{len(record['objects'])} objects from {record['scene']} "
          f"({record['scene_sha256'][:12]}...) -> {out}")

    if args.against:
        earlier = json.loads((REPO_ROOT / args.against).read_text())
        report = diff(earlier["objects"], record["objects"])
        print(f"\nagainst {args.against} ({earlier['scene_sha256'][:12]}...):")
        print(f"  added {len(report['added'])}, removed {len(report['removed'])}, "
              f"moved {len(report['moved'])}, resized {len(report['resized'])}, "
              f"unchanged {report['unchanged']}")
        for item in report["moved"][:10]:
            print(f"  moved: {item['path']} by {item['position_delta_m']:.6f} m, "
                  f"{item['orientation_delta_rad']:.6f} rad")
        for path in report["added"][:10]:
            print(f"  added: {path}")
        for path in report["removed"][:10]:
            print(f"  removed: {path}")
        print("  IDENTICAL: nothing measurable changed" if report["identical"]
              else "  the scene's measurable content CHANGED")
        (out.with_name(out.stem + "_diff.json")).write_text(
            json.dumps(report, indent=2, default=float))
        return 0 if report["identical"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
