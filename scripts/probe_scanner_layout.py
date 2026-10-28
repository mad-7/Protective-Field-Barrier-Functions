"""Read back a scanner layout from a scene: sensors, mounting, scan plane, protrusion, mass.

Hard rule 12: a scene edit is verified by reading back what changed before it is
committed. This reads a **candidate** scene -- by default the S300 layout waiting in
`scenes/pending/` -- and reports what the supervisor's conditions on it need
(2026-09-21):

  1. **flush mounting:** each scanner model's bounding box in the body frame, and how far it
     protrudes beyond the chassis corners;
  2. **scan plane:** each sensor's height above the floor, the wheel tops (the hidden wheel
     spheres' centres plus their radius) and the chassis top, and whether the plane clears
     them -- which decides whether the planar occlusion model over-states occlusion and
     what low obstacles pass under the scan;
  4. **mass:** each scanner model's mass as found (the `mass` property of its shapes), for
     the physics regression.

Also: every vision sensor's pose in the body frame, field of view and ray count as found,
the scripts inside each scanner model and whether they are enabled, and whether any
object aliased `LiDAR` (the inherited pods) is still in the robot.

The candidate scene is loaded as given and never saved. The run's provenance records the
candidate's path and SHA-256 -- not the pinned scene's -- so the record says exactly what
was read.

Usage (one instance on sim.port):
    python scripts/probe_scanner_layout.py --config configs/base.yaml --seed 0
    python scripts/probe_scanner_layout.py --config configs/base.yaml --seed 0 \
        --scene scenes/warehouse_v2.ttt
"""
from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.scanner import rays_for, sensor_view_from_matrix
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = "scenes/pending/warehouse_v2_s300.ttt"
CHASSIS_HALF = (0.2854, 0.1789)        # m, the chassis alone (docs/decision_log.md, WP0)


# ---------------------------------------------------------------------------------------
# Pure geometry
# ---------------------------------------------------------------------------------------
def quat_rotate(q: list[float], v: np.ndarray) -> np.ndarray:
    """Rotate (n, 3) vectors by a CoppeliaSim quaternion [qx, qy, qz, qw]."""
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return np.asarray(v, dtype=float) @ r.T


def box_corners(half: list[float], pose: list[float]) -> np.ndarray:
    """The 8 corners of a box of half sizes `half` at `pose` [x y z qx qy qz qw]."""
    signs = np.array(list(itertools.product((-1, 1), repeat=3)), dtype=float)
    local = signs * np.asarray(half, dtype=float)
    return quat_rotate(pose[3:], local) + np.asarray(pose[:3], dtype=float)


def apply_matrix(m: list[float], pts: np.ndarray) -> np.ndarray:
    r = np.asarray(m, dtype=float).reshape(3, 4)
    return np.asarray(pts, dtype=float) @ r[:, :3].T + r[:, 3]


def ref_to_body(pts: np.ndarray, yaw_offset: float) -> np.ndarray:
    """(n, 3) points from youBot_ref's frame into the body frame (rotate by -yaw)."""
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    p = np.asarray(pts, dtype=float)
    return np.column_stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]])


def protrusion(xy_extent: tuple[float, float], chassis_half: tuple[float, float]) -> dict:
    """How far a body-frame |x|, |y| extent reaches past the chassis, m (0 if within)."""
    return {"beyond_x_m": max(0.0, xy_extent[0] - chassis_half[0]),
            "beyond_y_m": max(0.0, xy_extent[1] - chassis_half[1])}


def shape_dynamics(sim: Any, shape: int) -> dict[str, Any]:
    """Whether a shape takes part in dynamics, read two ways and compared (hard rule 13).

    A mass only matters if the shape is dynamic: a static shape parented to the robot is
    carried kinematically and adds nothing to its inertia. Read through the property API
    and the older int parameters; a disagreement is reported, not resolved here.
    """
    opt = {"noError": True}
    props = int(sim.getObjectSpecialProperty(shape))
    out: dict[str, Any] = {
        "handle": shape, "alias": str(sim.getObjectAlias(shape)),
        "property_dynamic": sim.getBoolProperty(shape, "dynamic", opt),
        "property_respondable": sim.getBoolProperty(shape, "respondable", opt),
        "property_mass_kg": sim.getFloatProperty(shape, "mass", opt),
        "param_static": int(sim.getObjectInt32Param(shape, sim.shapeintparam_static)),
        "param_respondable": int(sim.getObjectInt32Param(shape, sim.shapeintparam_respondable)),
        "special": {name: bool(props & int(getattr(sim, f"objectspecialproperty_{name}")))
                    for name in ("collidable", "measurable", "detectable", "renderable")}}
    if out["property_dynamic"] is not None:
        out["dynamic_reads_agree"] = bool(out["property_dynamic"]) == (out["param_static"] == 0)
    return out


# ---------------------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--scene", default=DEFAULT_SCENE,
                        help="the candidate scene to read back (relative to the repo)")
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()

    base = load_config(args.config)
    base = apply_display(base, args.display)
    seed_everything(args.seed)
    scene = REPO_ROOT / args.scene
    if not scene.is_file():
        raise SystemExit(f"{scene} does not exist")
    digest = file_sha256(scene)
    # Provenance must name the scene actually read: the candidate, with its own hash.
    cfg = copy.deepcopy(base)
    cfg["sim"]["scene"] = args.scene
    cfg["sim"]["scene_sha256"] = digest
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"LAYOUT_{stamp}", args.seed, cfg)
    yaw = float(base["robot"]["yaw_offset"])
    range_max = float(base["scanners"]["range_max"])
    density = float(base["scanners"]["rays_per_degree"])

    client = SimClient(SimConnection.from_config(base, instance_id=args.instance))
    client.connect()
    sim = client.sim
    client.load_scene(str(scene))
    report: dict[str, Any] = {"scene": args.scene, "scene_sha256": digest,
                              "pinned_scene_sha256": base["sim"]["scene_sha256"]}
    try:
        root = client.object(base["sim"]["objects"]["robot_tree"])
        ref = client.object(base["sim"]["objects"]["base_ref"])
        in_robot = [int(h) for h in sim.getObjectsInTree(root, sim.handle_all, 0)]
        report["inherited_pods_present"] = [
            h for h in in_robot if str(sim.getObjectAlias(h)) == "LiDAR"]

        sensors = []
        for h in (int(x) for x in sim.getObjectsInTree(root, sim.sceneobject_visionsensor, 0)):
            fov = float(sim.getObjectFloatParam(h, sim.visionfloatparam_perspective_angle))
            view = sensor_view_from_matrix([float(v) for v in sim.getObjectMatrix(h, ref)],
                                           fov, yaw, range_max)
            model = h
            parent = int(sim.getObjectParent(h))
            while parent not in (root, -1):
                model, parent = parent, int(sim.getObjectParent(parent))
            sensors.append({
                "handle": h, "alias": str(sim.getObjectAlias(h)), "model": model,
                "x_m": view.x, "y_m": view.y, "heading_deg": math.degrees(view.heading),
                "fov_deg": math.degrees(fov),
                "rays_as_found": int(sim.getObjectInt32Param(
                    h, sim.visionintparam_resolution_x)),
                "rays_at_configured_density": rays_for(math.degrees(fov), density),
                "world_z_m": float(sim.getObjectPosition(h, sim.handle_world)[2])})
        report["sensors"] = sensors

        models = []
        for model in sorted({s["model"] for s in sensors}):
            shapes = [int(x) for x in sim.getObjectsInTree(model, sim.sceneobject_shape, 0)]
            corners = []
            mass = 0.0
            for sh in shapes:
                half = [float(v) for v in sim.getVector3Property(sh, "bbHSize")]
                pose = [float(v) for v in sim.getPoseProperty(sh, "bbPose")]
                local = box_corners(half, pose)
                corners.append(ref_to_body(apply_matrix(
                    [float(v) for v in sim.getObjectMatrix(sh, ref)], local), yaw))
                mass += float(sim.getFloatProperty(sh, "mass"))
            pts = np.vstack(corners) if corners else np.zeros((0, 3))
            ext = (float(np.abs(pts[:, 0]).max()), float(np.abs(pts[:, 1]).max())) \
                if len(pts) else (0.0, 0.0)
            scripts = [int(x) for x in sim.getObjectsInTree(model, sim.sceneobject_script, 0)]
            parent = int(sim.getObjectParent(model))
            models.append({
                "parent": {"handle": parent, "alias": str(sim.getObjectAlias(parent)),
                           "type": int(sim.getObjectType(parent))},
                "dynamics": [shape_dynamics(sim, sh) for sh in shapes],
                "handle": model, "alias": str(sim.getObjectAlias(model)),
                "shapes": len(shapes), "mass_kg": mass,
                "body_bbox_min_m": pts.min(axis=0).tolist() if len(pts) else None,
                "body_bbox_max_m": pts.max(axis=0).tolist() if len(pts) else None,
                "abs_extent_xy_m": list(ext),
                "protrusion": protrusion(ext, CHASSIS_HALF),
                "scripts": [{"handle": s, "disabled": bool(sim.getBoolProperty(
                    s, "scriptDisabled"))} for s in scripts]})
        report["scanner_models"] = models

        wheels = [h for h in in_robot if "wheel_respondable" in str(sim.getObjectAlias(h))]
        wheel_tops = []
        for h in wheels:
            half = [float(v) for v in sim.getVector3Property(h, "bbHSize")]
            z = float(sim.getObjectPosition(h, sim.handle_world)[2])
            wheel_tops.append(z + max(half))
        chassis = client.object(base["sim"]["objects"]["chassis"])
        ch_half = [float(v) for v in sim.getVector3Property(chassis, "bbHSize")]
        ch_pose = [float(v) for v in sim.getPoseProperty(chassis, "bbPose")]
        ch_world = apply_matrix([float(v) for v in sim.getObjectMatrix(chassis, sim.handle_world)],
                                box_corners(ch_half, ch_pose))
        plane = [s["world_z_m"] for s in sensors]
        report["scan_plane"] = {
            "sensor_world_z_m": plane,
            "wheel_top_world_z_m": max(wheel_tops) if wheel_tops else None,
            "chassis_top_world_z_m": float(ch_world[:, 2].max()),
            "clears_wheel_tops": bool(wheel_tops) and min(plane) > max(wheel_tops),
            "clears_chassis_top": min(plane) > float(ch_world[:, 2].max())}
    finally:
        client.stop()
        client.load_scene(str(REPO_ROOT / base["sim"]["scene"]))   # back to the pinned scene

    if file_sha256(scene) != digest:
        raise SystemExit("the candidate scene changed while it was read; it is never saved")
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"candidate scene {args.scene}  sha256 {digest[:12]}...")
    print(f"inherited LiDAR pods still present: {report['inherited_pods_present'] or 'none'}")
    for s in sensors:
        print(f"  sensor {s['alias']:<24} at ({s['x_m']:+.3f}, {s['y_m']:+.3f}) m, facing "
              f"{s['heading_deg']:+7.1f} deg, {s['fov_deg']:.1f} deg, z {s['world_z_m']:.3f} m, "
              f"{s['rays_as_found']} rays as found -> {s['rays_at_configured_density']}")
    for m in models:
        p = m["protrusion"]
        print(f"  model {m['alias']:<24} {m['shapes']} shapes, mass {m['mass_kg']:.3f} kg, "
              f"|x| {m['abs_extent_xy_m'][0]:.4f} |y| {m['abs_extent_xy_m'][1]:.4f} m, beyond "
              f"chassis {p['beyond_x_m'] * 1000:.1f} / {p['beyond_y_m'] * 1000:.1f} mm, "
              f"{len(m['scripts'])} scripts; parent {m['parent']['alias']} "
              f"(type {m['parent']['type']})")
        for dyn in m["dynamics"]:
            print(f"    shape {dyn['alias']}: dynamic {dyn['property_dynamic']} "
                  f"(static param {dyn['param_static']}, agree "
                  f"{dyn.get('dynamic_reads_agree')}), respondable "
                  f"{dyn['property_respondable']} / {dyn['param_respondable']}, mass "
                  f"{dyn['property_mass_kg']} kg, {dyn['special']}")
    sp = report["scan_plane"]
    print(f"scan plane z {[round(z, 3) for z in sp['sensor_world_z_m']]} m; wheel tops "
          f"{sp['wheel_top_world_z_m']}; chassis top {sp['chassis_top_world_z_m']:.3f} m; "
          f"clears wheels {sp['clears_wheel_tops']}, clears chassis {sp['clears_chassis_top']}")
    print(f"-> {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
