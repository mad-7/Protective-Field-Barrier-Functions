"""Measure the robot's footprint by collision query at many bearings, shape by shape.

Ruling 8 on the D7/G1b report: classify every shape that protrudes past the chassis as
collidable or visual-only, and report the envelope both ways. It also replaces the
bounding-box polygon of `scripts/report_pods.py`, which the spherical wheels made neither
accurate nor reproducible (a sphere's box over-states its reach by up to sqrt(3)).

Method. A thin, large, static, non-respondable plate is placed perpendicular to body
bearing b at offset d. `sim.checkCollision` tests *surface* intersection, so the plate
touches a part exactly when d lies within half a plate thickness of the part's projection
onto n(b). Scanning inward from outside the part's bounding box, in steps no larger than
the plate thickness, finds the first contact; bisection then brackets the outer face, and
the support value h(b) = max over the part of p . n(b) follows. The robot is at rest and
the simulation is not stepped during the probe.

The first version filled the half-space beyond d with a solid block, on the reasoning
that contact would then be monotone in d. It reported *no* contact for every shape,
including the chassis, because a solid that swallows a part whole intersects none of its
surfaces. The chassis check caught it and nothing was reported.

Resolution. Per-shape support values, which decide protrusion, are taken every
`--shape-step-deg` (default 10 deg). The two envelopes are measured every
`--bearing-step-deg` (default 2 deg), each against a collection of its shapes at once.

"Collidable" means what the contact check sees. `RobotContact` checks collections, and
a collection skips an object whose collidable flag is off, while the same object passed
as a single handle is checked regardless (tests/test_collection_semantics.py, measured
2026-09-18). So the classification is the flag, and the collidable envelope is measured
against a collection. Per-shape values, which decide protrusion, are measured by single
handle for every shape that registers at all, flag on or off, and a shape's
`registers_by_handle` is reported beside its flag. The first complete run classified by
handle instead and so called all 36 shapes collidable; the flag is what matters.
The respondable flag is reported alongside, because a shape can be seen by the contact
check yet pass through obstacles in the dynamics, or the reverse.

Envelopes, each as a support function and the polygon it implies:
  * collidable -- what the contact check can register; the footprint the PFB must cover
    for rule 4 to hold;
  * all shapes -- collidable by probe plus visual-only by bounding box; what a camera
    would call the robot.

Usage:
    python scripts/probe_footprint.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.footprint import (
    outermost_contact,
    polygon_from_support,
    quadrant_radii,
    radial_extent,
    support_of_points,
    symmetry_report,
)
from pfbnav.sim.robot import (
    ChassisGeometry,
    SceneObjects,
    bbox_corners_in_body,
    body_point_in_ref,
)
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

# The probe plate. Thin along the probe direction; large enough across and in height to
# cover the whole robot at any bearing. SCAN_STEP must not exceed the thickness, or a
# part thinner than the step could fall between two samples (see outermost_contact).
PLATE_THICKNESS_M = 0.006
PLATE_SPAN_M = 3.0
PLATE_HEIGHT_M = 3.0
PLATE_CENTRE_Z_M = 0.5
SCAN_STEP_M = 0.005
# Retry step for a part the normal step missed: a part thinner than the plate registers
# only in bands as wide as it is thin (see _probe_with_retry).
FINE_STEP_M = 0.0005
# Bisection tolerance, metres: the width of the bracket on every support value.
TOL_M = 0.0005
# A shape protrudes when it reaches past the chassis by more than this at some bearing.
PROTRUSION_TOL_M = 0.002
# The earlier axis sweeps (tests/test_footprint_probe.py, 2026-09-17), which put the
# collidable envelope's outer faces in these brackets. The new probe must agree.
AXIS_BRACKETS = {"front": (0.3495, 0.3520), "left": (0.1945, 0.1970)}


def _make_plate(sim) -> int:
    plate = int(sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [PLATE_THICKNESS_M, PLATE_SPAN_M, PLATE_HEIGHT_M]))
    sim.setObjectInt32Param(plate, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(plate, sim.shapeintparam_respondable, 0)
    return plate


def _support_by_probe(sim, plate, base_ref, yaw_offset, entity, bearing, start, stop,
                      step=SCAN_STEP_M):
    """Upper bound on an entity's support value at `bearing`, or None if never touched."""
    # Orientation is fixed for the whole scan; only the position moves.
    sim.setObjectOrientation(plate, base_ref, [0.0, 0.0, float(yaw_offset + bearing)])

    def contact_at(offset: float) -> bool:
        point = [offset * math.cos(bearing), offset * math.sin(bearing), PLATE_CENTRE_Z_M]
        sim.setObjectPosition(plate, base_ref, body_point_in_ref(point, yaw_offset).tolist())
        return bool(sim.checkCollision(plate, entity)[0])

    found = outermost_contact(contact_at, start, stop, step, TOL_M)
    if found is None:
        return None
    _, outside = found
    return outside - PLATE_THICKNESS_M / 2.0


def _probe_with_retry(sim, plate, base_ref, yaw_offset, entity, bearing, start, stop):
    """(support, how): 'probe' at the normal step, 'probe-fine' after a fine retry, or
    (None, 'missed').

    A part thinner than the plate along the probe direction can sit wholly inside the
    plate, where no surface is cut; it then registers only in two bands as wide as the
    part is thin, which the normal step can jump. The fine retry catches anything thicker
    than FINE_STEP_M. This was learned on 2026-09-18, when an arm cylinder registered
    along +x and nowhere else at 5 mm steps.
    """
    value = _support_by_probe(sim, plate, base_ref, yaw_offset, entity, bearing, start, stop)
    if value is not None:
        return value, "probe"
    value = _support_by_probe(sim, plate, base_ref, yaw_offset, entity, bearing, start, stop,
                              FINE_STEP_M)
    if value is not None:
        return value, "probe-fine"
    return None, "missed"


def probe(client: SimClient, cfg: dict[str, Any], bearings: np.ndarray,
          shape_bearings: np.ndarray) -> dict[str, Any]:
    """Classify every shape, measure the chassis and the envelope at `bearings`, and the
    shapes that could protrude at `shape_bearings` (a subset of `bearings`)."""
    geom = ChassisGeometry.from_config(cfg)
    objects = SceneObjects.from_config(cfg)
    sim = client.sim
    base_ref = client.object(objects.base_ref)
    chassis = client.object(objects.chassis)
    shapes = client.shapes_in_tree(client.object(objects.robot_tree))
    if chassis not in shapes:
        raise SystemExit("the chassis is not among the robot's shapes; check sim.objects")
    shapes = [chassis] + [h for h in shapes if h != chassis]
    coarse_index = [int(np.argmin(np.abs(bearings - b))) for b in shape_bearings]
    if not np.allclose(bearings[coarse_index], shape_bearings):
        raise SystemExit("the per-shape bearings must be a subset of the envelope bearings")
    margin = PLATE_THICKNESS_M

    def scan(entity, corners, b):
        top = float(support_of_points(corners, [b])[0])
        bottom = -float(support_of_points(corners, [b + math.pi])[0])
        return _probe_with_retry(sim, plate, base_ref, geom.yaw_offset, entity, float(b),
                                 top + margin, bottom - margin)

    plate = _make_plate(sim)
    records = []
    try:
        for handle in shapes:
            name = str(sim.getObjectAlias(handle, 2))
            lo, hi = client.object_bbox(handle)
            corners = bbox_corners_in_body(lo, hi, sim.getObjectMatrix(handle, base_ref),
                                           geom.yaw_offset)
            respondable = bool(sim.getObjectInt32Param(handle, sim.shapeintparam_respondable))
            # Collidable means what the contact check sees. RobotContact checks the chassis
            # as a single handle, which ignores the flag -- and the chassis's flag is off in
            # this scene -- and every other shape through a collection, which honours it.
            collidable_flag = client.special_flags(handle)["collidable"]
            bbox_fine = support_of_points(corners, bearings)
            record = {
                "handle": handle, "name": name, "is_chassis": handle == chassis,
                "respondable": respondable, "corners": corners,
                "support_bbox": bbox_fine[coarse_index],
                "support_probe": None, "how": [], "candidate": False,
                "collidable_flag": collidable_flag,
                "collidable": collidable_flag or handle == chassis,
            }
            records.append(record)

            if handle == chassis:
                # The chassis is measured at every envelope bearing: it is the reference
                # every other shape's protrusion is measured against.
                found = [scan(handle, corners, b) for b in bearings]
                if any(v is None for v, _ in found):
                    raise SystemExit("the chassis was missed at some bearing; the probe is "
                                     "misplaced, so nothing here is a measurement")
                chassis_fine = np.array([v for v, _ in found])
                record["registers_by_handle"] = True
                record["support_probe"] = chassis_fine[coarse_index]
                record["how"] = [how for _, how in found]
            else:
                # Whether it registers at all as a single handle (which ignores the flag),
                # along +x or +y: a part thin along one axis is wide along the other.
                record["registers_by_handle"] = any(
                    scan(handle, corners, b)[0] is not None for b in (0.0, math.pi / 2))
                # Its box contains it, so a box inside the chassis at every bearing means
                # the part cannot protrude, and needs no per-bearing probe.
                record["candidate"] = bool(np.any(bbox_fine > chassis_fine + PROTRUSION_TOL_M))
                if record["registers_by_handle"] and record["candidate"]:
                    values = []
                    for b, box_value in zip(shape_bearings, record["support_bbox"]):
                        value, how = scan(handle, corners, b)
                        values.append(box_value if value is None else value)
                        record["how"].append("bbox" if value is None else how)
                    record["support_probe"] = np.array(values)
            kind = "collidable " if record["collidable"] else "flag off   "
            resp = "respondable" if respondable else "           "
            extra = ""
            if record["candidate"]:
                fine = record["how"].count("probe-fine")
                box = record["how"].count("bbox")
                extra = f"  [probed; fine retries {fine}, bbox fallbacks {box}]"
            print(f"   {kind} {resp} {name}{extra}")

        # The collidable envelope, against all collidable shapes at once. A collection skips
        # the chassis (its flag is off), so the chassis's own per-bearing values, measured
        # above by handle, are folded in after the scan.
        members = [r for r in records if r["collidable"]]
        entity = client.entity_for([r["handle"] for r in members])
        all_corners = np.vstack([r["corners"] for r in members])
        envelope_support, envelope_how = [], []
        for b in bearings:
            value, how = scan(entity, all_corners, b)
            if value is None:
                raise SystemExit(f"the collidable set was not touched at bearing "
                                 f"{math.degrees(b):.1f} deg")
            envelope_support.append(value)
            envelope_how.append(how)
        envelope_support = np.maximum(np.array(envelope_support), chassis_fine)
        # A part that needed a fine retry or a box fallback is thin somewhere, and the
        # envelope scan, at the normal step, could jump it at a bearing in between. Its
        # box is added so that the envelope cannot under-state it.
        thin = [r for r in members if any(h in ("probe-fine", "bbox") for h in r["how"])]
        if thin:
            thin_corners = np.vstack([r["corners"] for r in thin])
            envelope_support = np.maximum(envelope_support,
                                          support_of_points(thin_corners, bearings))
    finally:
        sim.removeObjects([plate])
    return {"records": records, "configured": (geom.half_length, geom.half_width),
            "collidable_support": envelope_support, "envelope_how": envelope_how,
            "thin_shapes": [r["name"] for r in thin]}


def envelope(bearings: np.ndarray, support: np.ndarray) -> dict[str, Any]:
    """The polygon and radii implied by a sampled support function."""
    poly = polygon_from_support(bearings, support)
    radii = quadrant_radii(poly, samples=181)
    return {
        "polygon": poly,
        "half_length_front": radial_extent(poly, 0.0),
        "half_length_rear": radial_extent(poly, math.pi),
        "half_width_left": radial_extent(poly, math.pi / 2),
        "half_width_right": radial_extent(poly, -math.pi / 2),
        "circumscribed": float(np.max(np.linalg.norm(poly, axis=1))),
        "quadrant_radii": radii,
        "symmetry": symmetry_report(radii),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--bearing-step-deg", type=float, default=2.0,
                        help="bearing spacing of the envelopes; the polygon over-states "
                             "corners by at most 1/cos(step/2) - 1 (0.015 %% at 2 deg)")
    parser.add_argument("--shape-step-deg", type=float, default=10.0,
                        help="bearing spacing of the per-shape support values that decide "
                             "which shapes protrude")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = apply_display(cfg, args.display)
    seed_everything(args.seed)
    bearings = np.radians(np.arange(-180.0, 180.0, args.bearing_step_deg))
    shape_bearings = np.radians(np.arange(-180.0, 180.0, args.shape_step_deg))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "gates", f"FOOTPRINT_{stamp}",
                             args.seed, cfg)

    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))
    try:
        client.start(settle_steps=20)
        physics = client.physics_settings()
        print(f"physics in force: {physics}; shapes every {args.shape_step_deg} deg, "
              f"envelopes every {args.bearing_step_deg} deg")
        result = probe(client, cfg, bearings, shape_bearings)
    finally:
        client.stop()

    records = result["records"]
    chassis = next(r for r in records if r["is_chassis"])
    chassis_h = chassis["support_probe"]
    for r in records:
        probed = r["support_probe"] is not None
        r["source"] = "probe" if probed else "bbox"
        r["support_used"] = r["support_probe"] if probed else r["support_bbox"]
        r["beyond_chassis_m"] = float(np.max(r["support_used"] - chassis_h))
        r["protrudes"] = r["beyond_chassis_m"] > PROTRUSION_TOL_M and not r["is_chassis"]
        r["protrudes_by_bbox"] = (float(np.max(r["support_bbox"] - chassis_h))
                                  > PROTRUSION_TOL_M and not r["is_chassis"])

    collidable_h = result["collidable_support"]
    visual = [r for r in records if not r["collidable"]]
    all_h = collidable_h.copy()
    if visual:
        visual_corners = np.vstack([r["corners"] for r in visual])
        all_h = np.maximum(all_h, support_of_points(visual_corners, bearings))
    collidable = envelope(bearings, collidable_h)
    everything = envelope(bearings, all_h)

    with (run_dir / "support.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["bearing_deg", "collidable_envelope_m", "all_shapes_envelope_m"])
        for i, b in enumerate(bearings):
            writer.writerow([round(math.degrees(b), 3), collidable_h[i], all_h[i]])
    with (run_dir / "support_per_shape.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        header = ["bearing_deg"]
        for r in records:
            source = r["source"]
            header.append(f"{r['name']} [{source}]")
        writer.writerow(header)
        for i, b in enumerate(shape_bearings):
            writer.writerow([round(math.degrees(b), 3)] + [r["support_used"][i] for r in records])

    def summary(env: dict[str, Any]) -> dict[str, Any]:
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in env.items()}

    front = collidable["half_length_front"]
    left = collidable["half_width_left"]
    report = {
        "physics": physics,
        "bearing_step_deg": args.bearing_step_deg,
        "shape_step_deg": args.shape_step_deg,
        "plate_thickness_m": PLATE_THICKNESS_M,
        "tolerance_m": TOL_M,
        "configured_envelope": {"half_length": result["configured"][0],
                                "half_width": result["configured"][1]},
        "agreement_with_axis_sweeps": {
            "front": {"probe": front, "bracket_2026_09_17": AXIS_BRACKETS["front"]},
            "left": {"probe": left, "bracket_2026_09_17": AXIS_BRACKETS["left"]},
        },
        "shape_count": len(records),
        "collidable_count": sum(r["collidable"] for r in records),
        "shapes": [{**{k: r[k] for k in ("name", "is_chassis", "collidable", "collidable_flag",
                                         "registers_by_handle", "respondable",
                                         "candidate", "source", "beyond_chassis_m",
                                         "protrudes", "protrudes_by_bbox")},
                    "fine_retries": r["how"].count("probe-fine"),
                    "bbox_fallbacks": r["how"].count("bbox")}
                   for r in records],
        "thin_shapes_boxed_into_envelope": result["thin_shapes"],
        "envelope_fine_retries": result["envelope_how"].count("probe-fine"),
        "envelope_collidable": summary(collidable),
        "envelope_all_shapes": summary(everything),
    }
    (run_dir / "footprint_probe.json").write_text(json.dumps(report, indent=2, default=float))

    print()
    print(f"{len(records)} shapes, {report['collidable_count']} collidable. Protruding past "
          f"the chassis by > {PROTRUSION_TOL_M} m (by probe, or by bounding box):")
    print(f"{'class':<12} {'respond.':<9} {'beyond':>8} {'bbox?':>6}  shape")
    for r in sorted(records, key=lambda r: -r["beyond_chassis_m"]):
        if r["protrudes"] or r["protrudes_by_bbox"]:
            kind = "collidable" if r["collidable"] else "flag off"
            resp = "yes" if r["respondable"] else "no"
            by_box = "yes" if r["protrudes_by_bbox"] else "no"
            print(f"{kind:<12} {resp:<9} {r['beyond_chassis_m']:+8.4f} {by_box:>6}  "
                  f"{r['name']}")
    hl, hw = result["configured"]
    print()
    print(f"configured rectangle: half_length {hl:.4f}, half_width {hw:.4f} m")
    print(f"cross-check against the 2026-09-17 axis sweeps: front {front:.4f} "
          f"(bracket {AXIS_BRACKETS['front']}), left {left:.4f} "
          f"(bracket {AXIS_BRACKETS['left']})")
    for label, env in (("collidable", collidable), ("all shapes", everything)):
        print()
        print(f"envelope, {label} (upper bounds, within {TOL_M} m):")
        print(f"   front {env['half_length_front']:.4f}  rear {env['half_length_rear']:.4f}  "
              f"left {env['half_width_left']:.4f}  right {env['half_width_right']:.4f}  "
              f"circumscribed {env['circumscribed']:.4f} m")
        for name, v in env["quadrant_radii"].items():
            print(f"   {name:>12}: inscribed {v['inscribed']:.4f} @ "
                  f"{v['inscribed_bearing_deg']:+.0f} deg, circumscribed "
                  f"{v['circumscribed']:.4f} @ {v['circumscribed_bearing_deg']:+.0f} deg")
        print(f"   symmetry: fore-aft {env['symmetry']['fore_aft_difference']:.4f} m, "
              f"left-right {env['symmetry']['left_right_difference']:.4f} m")
    print()
    print(f"-> {run_dir}  ({physics['engine_name']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
