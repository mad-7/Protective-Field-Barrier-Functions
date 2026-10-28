"""Why did the coverage map see nothing? Self-occlusion against an unrendered post. (WP1)

The "before" coverage map (COVMAP-centreline_20260922T161234Z) read range_max in every
bin at every one of 7,852 cells: no return survived, even from a post 0.25 m in front of
a pod. Two hypotheses, and this tells them apart:

  H1  **housing** -- since d7da848 each sensor renders everything, the robot included; if
      the sensor sits inside its own pod's casing, every pixel hits the casing a few cm
      away, every return lies inside the footprint, and the self-filter drops them all.
  H2  **post** -- a primitive created while the simulation runs is not rendered by the
      sensors, so there is nothing to see.

Each sensor is rendered directly from Python (the Lua is not installed; the scanner-model
scripts are disabled, confirmed by read-back) under three render sets, with the post absent
and present, the post 0.4 m along that sensor's own axis:

  all            everything, the robot included (what the Lua does now)
  no_own_model   everything except that sensor's own scanner model (the proposed fix)
  no_robot       everything except the whole robot (the post alone, isolates H2)

For each render: pixels with a return, the nearest raw depth, how many returns fall inside
the footprint + self_filter_margin (dropped as the robot), how many survive, and whether
any survivor lies on the post.

Reading: H1 if `all` has (nearly) every pixel returning within a few cm and nothing
surviving, while `no_own_model` sees the post. H2 if the post is absent from `no_robot`.
Both can hold.

The post and the collections exist in memory only; the scene is never saved and its SHA-256
is checked before and after (hard rule 12). Please do not zoom, pan or save while it runs.

Usage (one instance on sim.port, under a minute):
    python scripts/diag_self_occlusion.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.depth_geometry import apply_matrix, sensor_points
from pfbnav.sim.scanner import Scanner
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

POST_DIAMETER_M = 0.02         # as in diag_coverage_map.py
POST_HEIGHT_M = 1.0
POST_AHEAD_M = 0.4             # along each sensor's own axis, from the sensor
ON_POST_TOLERANCE_M = 0.03     # a surviving return this close to the post's axis is the post
HOUSING_M = 0.05               # a raw depth below this is "the sensor's own casing" scale
SETTLE_STEPS = 10
RENDER_SETS = ("all", "no_own_model", "no_robot")


# ---------------------------------------------------------------------------------------
# Pure: classify one sensor's depth row
# ---------------------------------------------------------------------------------------
def classify_returns(depth: np.ndarray, fov: float, matrix: list[float], yaw_offset: float,
                     *, mirrored: bool, euclidean: bool,
                     extent: Callable[[np.ndarray], Any], margin: float,
                     range_max: float, post_xy: tuple[float, float] | None) -> dict[str, Any]:
    """What one depth row contains, filtered exactly as lua/scanner_v2.lua filters it.

    `matrix` is sensor -> robot_ref; points are rotated by -yaw_offset into the body frame.
    A return is a pixel reading below range_max; it is **self** if its body-frame range is
    within the footprint extent at its bearing + `margin`, else it **survives**.
    """
    d = np.asarray(depth, dtype=float).ravel()
    ret = d < range_max - 1e-6
    out: dict[str, Any] = {"pixels": int(d.size), "returns": int(ret.sum()),
                           "min_depth_m": float(d.min()) if d.size else None,
                           "returns_below_housing": int((d < HOUSING_M).sum())}
    pts = apply_matrix(matrix, sensor_points(d, fov, mirrored=mirrored,
                                             euclidean=euclidean))[ret]
    c, s = math.cos(-yaw_offset), math.sin(-yaw_offset)
    x = c * pts[:, 0] - s * pts[:, 1]
    y = s * pts[:, 0] + c * pts[:, 1]
    r = np.hypot(x, y)
    b = np.arctan2(y, x)
    self_ret = r <= np.asarray(extent(b), dtype=float) + margin
    keep = ~self_ret
    out["self_returns"] = int(self_ret.sum())
    out["surviving"] = int(keep.sum())
    out["nearest_surviving_m"] = float(r[keep].min()) if keep.any() else None
    if post_xy is not None:
        on = np.hypot(x - post_xy[0], y - post_xy[1]) <= ON_POST_TOLERANCE_M
        out["post_pixels_any"] = int(on.sum())
        out["post_pixels_surviving"] = int((on & keep).sum())
    return out


def verdict(rows: list[dict[str, Any]]) -> dict[str, bool]:
    """H1 and H2 from the per-sensor results, as stated in the module docstring."""
    def get(sensor, render, post):
        return next(r for r in rows if r["sensor"] == sensor and r["render"] == render
                    and r["post"] == post)
    sensors = sorted({r["sensor"] for r in rows})
    h1 = all(get(s, "all", True)["post_pixels_surviving"] == 0
             and get(s, "no_own_model", True)["post_pixels_surviving"] > 0 for s in sensors)
    h2 = any(get(s, "no_robot", True)["post_pixels_any"] == 0 for s in sensors)
    fix_sees_post = all(get(s, "no_own_model", True)["post_pixels_surviving"] > 0
                        for s in sensors)
    return {"H1_housing": h1, "H2_post_not_rendered": h2,
            "proposed_fix_sees_post_on_every_sensor": fix_sees_post}


# ---------------------------------------------------------------------------------------
def body_to_ref(x: float, y: float, yaw_offset: float) -> tuple[float, float]:
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    return c * x - s * y, s * x + c * y


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = apply_display(cfg, args.display)
    seed_everything(args.seed)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    hash_before = file_sha256(scene)
    yaw = float(cfg["robot"]["yaw_offset"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SELFOCC_{stamp}", args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    client.load_scene(str(scene))
    sim = client.sim
    scanner = Scanner(client, cfg)
    sensors = scanner.discover()

    def own_model(h: int) -> int:
        """The sensor's ancestor just below the robot root (as Scanner.scanner_models)."""
        parent = int(sim.getObjectParent(h))
        while parent != root:
            h, parent = parent, int(sim.getObjectParent(parent))
        return h
    root = client.object(cfg["sim"]["objects"]["robot_tree"])
    ref = client.object(cfg["sim"]["objects"]["base_ref"])
    scanner.scanner_models()           # raises if a sensor is not inside the robot

    report: dict[str, Any] = {"scene_sha256": hash_before, "sensors": []}
    rows: list[dict[str, Any]] = []
    post = None
    try:
        scanner._silence_scanner_model_scripts()
        report["legacy_scripts"] = scanner.legacy_report()
        for s in sensors:
            h = s["handle"]
            sim.setExplicitHandling(h, 1)
            sim.setObjectInt32Param(h, sim.visionintparam_resolution_x, s["rays"])
            sim.setObjectFloatParam(h, sim.visionfloatparam_far_clipping, scanner.range_max)
            if int(sim.getObjectInt32Param(h, sim.visionintparam_resolution_x)) != s["rays"]:
                raise SystemExit(f"sensor {h}: resolution did not take")
        client.start(settle_steps=SETTLE_STEPS)
        ref_z = float(sim.getObjectPosition(ref, sim.handle_world)[2])

        # Render sets, created while running (they vanish at stop).
        def everything_except(*roots: int) -> int:
            col = int(sim.createCollection(0))
            sim.addItemToCollection(col, sim.handle_all, -1, 0)
            for rt in roots:
                sim.addItemToCollection(col, sim.handle_tree, int(rt), 1)
            return col
        no_robot = everything_except(root)

        post = int(sim.createPrimitiveShape(sim.primitiveshape_cylinder,
                                            [POST_DIAMETER_M, POST_DIAMETER_M, POST_HEIGHT_M],
                                            0))
        props = int(sim.getObjectSpecialProperty(post))
        report["post_special_properties"] = {
            name: bool(props & int(getattr(sim, f"objectspecialproperty_{name}")))
            for name in ("collidable", "measurable", "detectable", "renderable")}
        report["post_parent"] = int(sim.getObjectParent(post))
        park = [50.0, 50.0, 0.5 * POST_HEIGHT_M - ref_z]         # far outside range_max

        for s in sensors:
            h, v = s["handle"], s["view"]
            model = own_model(h)
            no_own = everything_except(model)
            entity = {"all": -1, "no_own_model": no_own, "no_robot": no_robot}
            px, py = (v.x + POST_AHEAD_M * math.cos(v.heading),
                      v.y + POST_AHEAD_M * math.sin(v.heading))
            rx, ry = body_to_ref(px, py, yaw)
            near = float(sim.getObjectFloatParam(h, sim.visionfloatparam_near_clipping))
            report["sensors"].append({
                "handle": h, "model": model, "alias": str(sim.getObjectAlias(h)),
                "model_alias": str(sim.getObjectAlias(model)), "x": v.x, "y": v.y,
                "heading_deg": math.degrees(v.heading), "near_clipping_m": near,
                "entity_to_render_as_found": int(sim.getObjectInt32Param(
                    h, sim.visionintparam_entity_to_render)),
                "post_body_xy": [px, py]})
            for with_post in (False, True):
                if with_post:
                    sim.setObjectPosition(post, [rx, ry, 0.5 * POST_HEIGHT_M - ref_z], ref)
                else:
                    sim.setObjectPosition(post, park, ref)
                for render in RENDER_SETS:
                    sim.setObjectInt32Param(h, sim.visionintparam_entity_to_render,
                                            entity[render])
                    got = int(sim.getObjectInt32Param(h, sim.visionintparam_entity_to_render))
                    if got != entity[render]:
                        raise SystemExit(f"sensor {h}: entity_to_render {entity[render]} "
                                         f"read back {got}")
                    sim.handleVisionSensor(h)
                    buf, res = sim.getVisionSensorDepth(h, 1)
                    depth = np.array(struct.unpack(f"<{len(buf) // 4}f", buf))
                    if depth.size != int(res[0]) * int(res[1]):
                        raise SystemExit(f"sensor {h}: {depth.size} depths for {res}")
                    row = classify_returns(
                        depth, math.radians(s["fov_deg"]),
                        [float(m) for m in sim.getObjectMatrix(h, ref)], yaw,
                        mirrored=scanner.pixel_mirror, euclidean=scanner.depth_is_euclidean,
                        extent=scanner.footprint.extent, margin=scanner.self_filter_margin,
                        range_max=scanner.range_max, post_xy=(px, py))
                    row.update(sensor=h, render=render, post=with_post)
                    rows.append(row)
        sim.removeObjects([post])            # while running (it would vanish at stop)
        post = None
        client.stop()
    finally:
        if post is not None and bool(sim.isHandle(post)):
            sim.removeObjects([post])
        scanner.remove()
        client.stop()
        client.load_scene(str(scene))        # discard every in-memory change

    if file_sha256(scene) != hash_before:
        raise SystemExit("the scene file changed during the run; it is never saved")
    report["rows"] = rows
    report["verdict"] = verdict(rows)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"post special properties: {report['post_special_properties']}")
    for s in report["sensors"]:
        print(f"\nsensor {s['handle']} ({s['alias']} in {s['model_alias']}) at "
              f"({s['x']:+.3f}, {s['y']:+.3f}) facing {s['heading_deg']:+.0f} deg; near clip "
              f"{s['near_clipping_m']:.4f} m; entity as found {s['entity_to_render_as_found']}")
        print(f"  {'render':<13} {'post':<5} {'returns':>7} {'<5cm':>5} {'min d':>7} "
              f"{'self':>5} {'kept':>5} {'nearest':>8} {'on post':>7} {'kept':>5}")
        for r in (r for r in rows if r["sensor"] == s["handle"]):
            nearest = "-" if r["nearest_surviving_m"] is None else \
                f"{r['nearest_surviving_m']:.3f}"
            print(f"  {r['render']:<13} {r['post']!s:<5} {r['returns']:>7} "
                  f"{r['returns_below_housing']:>5} {r['min_depth_m']:>7.3f} "
                  f"{r['self_returns']:>5} {r['surviving']:>5} {nearest:>8} "
                  f"{r['post_pixels_any']:>7} {r['post_pixels_surviving']:>5}")
    print(f"\nverdict: {report['verdict']}")
    print(f"-> {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
