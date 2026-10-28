"""Where does the scanner's ~1 % long reading come from? Resolution against near clipping.

`SCANCHECK_20260921T213302Z` settled the scanner's frame and pixel convention and left one
error: ranges read ~1 % long measured from the sensor (median ~10 mm/m, worse at oblique
incidence), always in the unsafe direction. No combination of a depth scale, a depth
offset and a field-of-view error explained it (best fit: 9 mm RMS, 42 mm worst). Two
mechanisms remain that a single parameter cannot capture, and each has a lever:

  * **pixel sampling** -- a 1.5 deg pixel covers a patch of a slanted wall whose depth
    varies across it, and the depth reported need not be the depth at the pixel centre.
    If so the error shrinks as the pixels shrink: **resolution** 80 -> 256 -> 512.
  * **depth conversion** -- CoppeliaSim renders a perspective depth buffer and converts it
    to metres using the near and far clipping planes; a near plane close to the sensor
    squeezes the buffer's precision. If so the error moves with **near clipping**.

This measures the front wall -- the worst case in the last run -- at 1.0 and 2.0 m, under
every combination, from the raw capture (the convention is settled: mirrored order,
perpendicular depth), and times a step at each resolution because the cost is per sensor
but the check should show it rather than assume it.

**Decision rule, set before the run:**
  * the median error per metre falls by half or more from 80 to 512 px -> sampling:
    adopt the resolution that brings it within the scanner check's 10 mm, then confirm
    with the full multi-wall check;
  * it moves by half or more across the near-clipping values -> depth conversion: adopt
    the setting that minimises it, then confirm the same way;
  * neither moves it by more than ~1 mm/m (the scatter seen so far) -> neither lever;
    the fallback is a conservative range correction, which is the supervisor's decision.

The wall, the scanner script and the sensor settings exist in memory only; the scene is
never saved and its SHA-256 is checked before and after (hard rule 12). Please do not
zoom, pan or save the scene while it runs.

Usage (one instance on sim.port, nothing else running):
    python scripts/diag_scanner_resolution.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.depth_geometry import apply_matrix, sensor_points
from pfbnav.sim.scanner import Scanner
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

RESOLUTIONS = (80, 256, 512)
NEAR_CLIPPINGS_M = (0.01, 0.05, 0.1)
DISTANCES_M = (1.0, 2.0)
WALL_BODY_DEG = 0.0            # the front wall: the worst case in SCANCHECK_20260921T213302Z
SETTLE_STEPS = 10
TIMED_STEPS = 50
SHORTENED_BY_M = 0.02
HALF_LENGTH_M = 3.0            # the check's wall is 6 m long
END_MARGIN_M = 0.1


def _check_module():
    """The multi-wall check's wall placement and capture, reused rather than copied."""
    path = REPO_ROOT / "scripts" / "diag_scanner_check.py"
    spec = importlib.util.spec_from_file_location("diag_scanner_check", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------------------
# Evaluation (pure)
# ---------------------------------------------------------------------------------------
def face_residuals(baseline: list[dict], withwall: list[dict], normal_rad: float,
                   distance: float, range_max: float, *, mirrored: bool = True,
                   euclidean: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """(n . p - d, |p - sensor|) in metres for every pixel that sees the wall's face.

    The second array is each point's range from its own sensor, so an error that scales
    with range can be read as mm per metre whatever the sensor's distance to the wall.
    """
    res, rng = [], []
    n = np.array([math.cos(normal_rad), math.sin(normal_rad)])
    t = np.array([-n[1], n[0]])
    for base, wall in zip(baseline, withwall, strict=True):
        b = np.asarray(base["depth"])
        w = np.asarray(wall["depth"])
        if b.shape != w.shape:
            raise ValueError("baseline and wall captures differ in resolution")
        sees = (b - w > SHORTENED_BY_M) & (w < range_max - 1e-6)
        if not sees.any():
            continue
        m = np.asarray(wall["matrix"], dtype=float).reshape(3, 4)
        pts = apply_matrix(m.ravel(), sensor_points(
            w, wall["fov"], mirrored=mirrored, euclidean=euclidean))[sees]
        face = np.abs(pts[:, :2] @ t) <= HALF_LENGTH_M - END_MARGIN_M
        pts = pts[face]
        res.extend((pts[:, :2] @ n - distance).tolist())
        rng.extend(np.linalg.norm(pts - m[:, 3], axis=1).tolist())
    return np.asarray(res), np.asarray(rng)


def summarise(res: np.ndarray, rng: np.ndarray) -> dict[str, Any]:
    """Median and tail of the error, absolute and per metre of range from the sensor."""
    if res.size == 0:
        return {"pixels": 0}
    return {"pixels": int(res.size),
            "median_mm": float(np.median(res) * 1000),
            "p95_abs_mm": float(np.percentile(np.abs(res), 95) * 1000),
            "max_abs_mm": float(np.abs(res).max() * 1000),
            "median_mm_per_m": float(np.median(res / rng) * 1000)}


def verdict(table: dict[str, dict[str, Any]], resolutions: tuple[int, ...],
            nears: tuple[float, ...]) -> dict[str, Any]:
    """Apply the pre-declared rule to the median error per metre.

    `table` maps "res@near" to a summary pooled over distances. A lever "moves" the error
    when its best setting is at most half its worst; a change under 1 mm/m is scatter.
    """
    def per_m(r: int, n: float) -> float:
        return table[f"{r}@{n}"]["median_mm_per_m"]

    by_res = {r: float(np.median([per_m(r, n) for n in nears])) for r in resolutions}
    by_near = {n: float(np.median([per_m(r, n) for r in resolutions])) for n in nears}

    def moves(values: dict) -> bool:
        lo, hi = min(values.values(), key=abs), max(values.values(), key=abs)
        return abs(hi) - abs(lo) > 1.0 and abs(lo) <= 0.5 * abs(hi)

    sampling = moves(by_res)
    conversion = moves(by_near)
    if sampling and not conversion:
        cause = "sampling"
    elif conversion and not sampling:
        cause = "depth_conversion"
    elif sampling and conversion:
        cause = "both"
    else:
        cause = "neither"
    best = min(table, key=lambda k: abs(table[k]["median_mm_per_m"]))
    return {"by_resolution_mm_per_m": by_res, "by_near_clipping_mm_per_m": by_near,
            "cause": cause, "best_setting": best}


# ---------------------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    check = _check_module()
    scene = REPO_ROOT / cfg["sim"]["scene"]
    hash_before = file_sha256(scene)
    range_max = float(cfg["scanners"]["range_max"])
    yaw_offset = float(cfg["robot"]["yaw_offset"])
    normal_ref = math.radians(WALL_BODY_DEG) + yaw_offset
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SCANRES_{stamp}", args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    report: dict[str, Any] = {"resolutions": list(RESOLUTIONS),
                              "near_clippings_m": list(NEAR_CLIPPINGS_M),
                              "distances_m": list(DISTANCES_M),
                              "wall_body_deg": WALL_BODY_DEG, "cells": {}}
    raw: dict[str, Any] = {}
    scanner = None
    wall = None
    try:
        for res_x in RESOLUTIONS:
            for near in NEAR_CLIPPINGS_M:
                key = f"{res_x}@{near}"
                client.load_scene(str(scene))          # a clean scene for every setting
                scanner = Scanner(client, cfg)
                scanner.install(resolution_x=res_x)
                robot = client.object(cfg["sim"]["objects"]["base_ref"])
                tree = client.object(cfg["sim"]["objects"]["robot_tree"])
                sensors = [int(h) for h in client.sim.getObjectsInTree(
                    tree, client.sim.sceneobject_visionsensor, 0)]
                original_near = [float(client.sim.getObjectFloatParam(
                    h, client.sim.visionfloatparam_near_clipping)) for h in sensors]
                for h in sensors:
                    client.sim.setObjectFloatParam(
                        h, client.sim.visionfloatparam_near_clipping, near)

                client.start(settle_steps=SETTLE_STEPS)
                # CoppeliaSim does not always store the near plane it is given -- 0.01 read
                # back as 0.009 (2026-09-21) -- and the Lua sets the far plane at start. So
                # the value that counts is the one in force while rendering, read back now,
                # and it is what the results are recorded against (hard rule 13).
                near_in_force = [float(client.sim.getObjectFloatParam(
                    h, client.sim.visionfloatparam_near_clipping)) for h in sensors]
                if max(near_in_force) - min(near_in_force) > 1e-9:
                    raise SystemExit(f"the sensors disagree about their near plane: "
                                     f"{near_in_force}")
                status = scanner.read_status()
                if int(status["resolution_x"]) != res_x:
                    raise SystemExit(f"asked for {res_x} rays, the scanner reports "
                                     f"{status['resolution_x']}")
                timing = check.time_steps(client, TIMED_STEPS)
                client.step()
                baseline = check.capture_raw(client, sensors, robot)
                cell: dict[str, Any] = {"step_ms": timing, "original_near_m": original_near,
                                        "near_requested_m": near,
                                        "near_in_force_m": near_in_force[0],
                                        "by_distance": {}}
                pooled_res, pooled_rng = [], []
                for d in DISTANCES_M:
                    wall = check.place_wall(client, robot, normal_ref, d)
                    client.step()
                    client.step()
                    captured = check.capture_raw(client, sensors, robot)
                    check.remove_wall(client, wall)
                    wall = None
                    r, g = face_residuals(baseline, captured, normal_ref, d, range_max)
                    cell["by_distance"][f"{d:.1f}"] = summarise(r, g)
                    pooled_res.append(r)
                    pooled_rng.append(g)
                    raw[f"{key}@{d:.1f}"] = {"baseline": baseline, "wall": captured}
                client.stop()
                cell.update(summarise(np.concatenate(pooled_res), np.concatenate(pooled_rng)))
                report["cells"][key] = cell
                scanner.remove()
                scanner = None
                print(f"  {res_x:4d} px, near {near:5.2f} m (in force "
                      f"{near_in_force[0]:.4f}): {cell['pixels']:4d} px, "
                      f"median {cell['median_mm']:+6.2f} mm ({cell['median_mm_per_m']:+6.2f} mm/m), "
                      f"p95 {cell['p95_abs_mm']:6.2f}, max {cell['max_abs_mm']:6.2f} mm, "
                      f"step {timing['mean_ms']:.1f} ms")
    finally:
        if wall is not None:
            check.remove_wall(client, wall)
        if scanner is not None:
            scanner.remove()
        client.stop()
        client.load_scene(str(scene))           # leave the instance on the pinned scene

    if file_sha256(scene) != hash_before:
        raise SystemExit("the scene file changed during the run; it is never saved")
    report["scene_sha256"] = hash_before
    # The near-clipping lever only tests anything if the planes in force actually differ.
    # When they do not, near clipping is reported UNTESTED and the resolution result stands
    # on its own; the data is kept either way (the first full run discarded it, 2026-09-21).
    near_moved = all(
        len({round(report["cells"][f"{r}@{n}"]["near_in_force_m"], 6)
             for n in NEAR_CLIPPINGS_M}) == len(NEAR_CLIPPINGS_M)
        for r in RESOLUTIONS)
    report["near_clipping_tested"] = near_moved
    report["verdict"] = verdict(report["cells"], RESOLUTIONS, NEAR_CLIPPINGS_M)
    if not near_moved:
        v = report["verdict"]
        v["cause"] = "sampling" if v["cause"] in ("sampling", "both") else "not_sampling"
        v["near_clipping"] = ("untested: CoppeliaSim held the near plane at the same value "
                              "for every setting")

    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    (run_dir / "raw_capture.json").write_text(json.dumps(raw))
    stamp_display(run_dir, client.display_status())

    v = report["verdict"]
    print("\nmedian error per metre of range from the sensor:")
    print("  by resolution:     " + ", ".join(
        f"{r} px {x:+.2f}" for r, x in v["by_resolution_mm_per_m"].items()))
    print("  by near clipping:  " + ", ".join(
        f"{n} m {x:+.2f}" for n, x in v["by_near_clipping_mm_per_m"].items()))
    if not report["near_clipping_tested"]:
        print(f"  near clipping: {v['near_clipping']}")
    print(f"\ncause by the pre-declared rule: {v['cause']}; smallest error at {v['best_setting']}")
    print(f"-> {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
