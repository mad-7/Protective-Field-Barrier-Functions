"""Does `lua/scanner_v2.lua` report the geometry that is actually there? (WP1)

Walls are placed in memory at known perpendicular distances around the robot -- ahead,
to its left, to its right and behind, in the **body frame** -- and the scan is compared
against the exact answer. The comparison is the **plane relation**: a point p on a wall
with unit normal n at distance d satisfies n . p = d, whatever sensor saw it and wherever
that sensor sits on the robot.

Two readings of each wall are checked:

  * **the raw capture** -- every sensor's depth row and pose relative to `youBot_ref`, read
    back after the step -- reconstructed under all four pixel conventions
    (`pfbnav/sim/depth_geometry.py`), in `youBot_ref`'s frame;
  * **the packet** the Lua published, in the body frame, under the convention in
    `configs/base.yaml` -- what the environment will consume.

History (2026-09-21):
  * `SCANCHECK_20260921T144640Z`: a wall on a line tilted ~60 deg -- a mirrored pixel order.
  * `SCANCHECK_20260921T151356Z`: mirrored/perpendicular won on every wall by one to two
    orders of magnitude, leaving a residual of ~+10 mm (median) that no single scale,
    offset or angle parameter explained. That run also exposed that the scanner reported
    bearings in `youBot_ref`'s frame, a quarter turn from the body frame; the walls it
    called left/front/behind were chassis front/right/left.

This version therefore:
  * places walls in the body frame, including both sides, so abeam is really abeam;
  * places each wall **after** the robot has settled, and records the robot's pose at
    placement and at capture, so settling cannot masquerade as scanner error;
  * repeats each wall at three distances, so an error proportional to distance (a scale)
    separates from a constant one (an offset);
  * scores only points on the wall's face, excluding its end faces.

Also checked: that the scan never sees the robot itself, what was found of the v1 LiDAR
scripts, the coverage, and the scanner's per-step cost.

Pre-declared acceptance: every wall at every distance within 10 mm in the packet, and no
return inside the footprint in the wall-free baseline.

The walls and the scanner script exist in memory only and are removed afterwards; the
scene is never saved, and its SHA-256 is checked before and after (hard rule 12).
**Please do not zoom, pan or save the scene while it runs**: a saved camera change alters
the scene file, and the next run then refuses the scene (2026-09-21).

Usage (one instance on sim.port, nothing else running):
    python scripts/diag_scanner_check.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.profiling import summarise_timings
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.depth_geometry import apply_matrix, sensor_points, wall_residuals
from pfbnav.sim.scanner import Scanner
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

SETTLE_STEPS = 10
TIMED_STEPS = 100
WALL_DISTANCES_M = (0.6, 1.0, 2.0)
WALL_LENGTH_M = 6.0
WALL_HEIGHT_M = 1.0
WALL_THICKNESS_M = 0.05
END_MARGIN_M = 0.1             # points within this of a wall's end are not on its face
WALLS_BODY_DEG = {"front": 0.0, "left": 90.0, "right": -90.0, "behind": 180.0}
SHORTENED_BY_M = 0.02          # a reading counts as "sees the wall" past this change
FACING_DEG = 60.0              # only bearings within this of the normal meet the wall
ACCEPT_RESIDUAL_M = 0.010      # pre-declared: every wall within 10 mm
SELF_MARGIN_M = 0.05           # a baseline return within the footprint + this is the robot
CONVENTIONS = [(m, e) for m in (False, True) for e in (False, True)]


def label(mirrored: bool, euclidean: bool) -> str:
    return f"{'mirrored' if mirrored else 'direct'}/{'euclidean' if euclidean else 'perpendicular'}"


# ---------------------------------------------------------------------------------------
# Simulator side
# ---------------------------------------------------------------------------------------
def place_wall(client: SimClient, robot: int, normal_ref_rad: float, distance: float) -> int:
    """A static wall `distance` m along a normal given in `youBot_ref`'s frame."""
    sim = client.sim
    wall = int(sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [WALL_LENGTH_M, WALL_THICKNESS_M, WALL_HEIGHT_M], 0))
    sim.setObjectOrientation(wall, [0.0, 0.0, normal_ref_rad - math.pi / 2], robot)
    c = distance + 0.5 * WALL_THICKNESS_M
    sim.setObjectPosition(wall, [c * math.cos(normal_ref_rad), c * math.sin(normal_ref_rad),
                                 0.5 * WALL_HEIGHT_M], robot)
    return wall


def remove_wall(client: SimClient, wall: int | None) -> None:
    """Remove a wall if it still exists.

    A wall placed while the simulation runs is deleted by CoppeliaSim when the run stops,
    so by cleanup time its handle may already be gone; `sim.isHandle` says so without the
    error that aborted the cleanup on 2026-09-21 and left the scanner installed.
    """
    if wall is not None and bool(client.sim.isHandle(wall)):
        client.sim.removeObjects([wall])


def robot_pose(client: SimClient, robot: int) -> list[float]:
    """World x, y, z (m) and yaw (rad) of `youBot_ref`."""
    sim = client.sim
    p = sim.getObjectPosition(robot, sim.handle_world)
    e = sim.getObjectOrientation(robot, sim.handle_world)
    return [float(p[0]), float(p[1]), float(p[2]), float(e[2])]


def capture_raw(client: SimClient, sensors: list[int], robot: int) -> list[dict[str, Any]]:
    """Each sensor's depth row, pose relative to `youBot_ref` and field of view.

    Read after the step, without handling the sensors again: the Lua handled them during
    sensing, so this is the buffer it binned -- the same data, not a second rendering.
    """
    sim = client.sim
    out = []
    for h in sensors:
        buf, res = sim.getVisionSensorDepth(h, 1)
        depth = struct.unpack(f"<{len(buf) // 4}f", buf)
        if len(depth) != int(res[0]) * int(res[1]):
            raise SystemExit(f"sensor {h}: depth buffer holds {len(depth)} values for a "
                             f"{res[0]}x{res[1]} image")
        out.append({"handle": h, "depth": list(depth),
                    "matrix": [float(v) for v in sim.getObjectMatrix(h, robot)],
                    "fov": float(sim.getObjectFloatParam(
                        h, sim.visionfloatparam_perspective_angle))})
    return out


def time_steps(client: SimClient, n: int) -> dict[str, Any]:
    samples = []
    for _ in range(n):
        t0 = time.perf_counter_ns()
        client.sim.step()
        samples.append((time.perf_counter_ns() - t0) / 1e6)
    return summarise_timings(samples)


# ---------------------------------------------------------------------------------------
# Evaluation (pure)
# ---------------------------------------------------------------------------------------
def on_face(points: np.ndarray, normal_rad: float, half_length: float) -> np.ndarray:
    """Whether each point lies along the wall's face rather than past its ends."""
    t = np.array([-math.sin(normal_rad), math.cos(normal_rad)])
    return np.abs(np.asarray(points)[:, :2] @ t) <= half_length - END_MARGIN_M


def raw_hypotheses(baseline: list[dict], withwall: list[dict], normal_ref_rad: float,
                   distance: float, range_max: float,
                   half_length: float = 0.5 * WALL_LENGTH_M) -> dict[str, Any]:
    """Residuals of the wall's face pixels under every convention, in `youBot_ref`'s frame.

    Pixels are chosen by what changed between the two captures, not by any convention,
    so the selection cannot favour the hypothesis being tested; the end-face exclusion is
    applied per convention to that convention's own points.
    """
    per: dict[str, Any] = {}
    for mirrored, euclidean in CONVENTIONS:
        residuals = []
        selected = 0
        for base, wall in zip(baseline, withwall, strict=True):
            b = np.asarray(base["depth"])
            w = np.asarray(wall["depth"])
            sees = (b - w > SHORTENED_BY_M) & (w < range_max - 1e-6)
            selected += int(sees.sum())
            if not sees.any():
                continue
            pts = apply_matrix(wall["matrix"], sensor_points(
                w, wall["fov"], mirrored=mirrored, euclidean=euclidean))[sees]
            pts = pts[on_face(pts, normal_ref_rad, half_length)]
            residuals.extend(wall_residuals(pts, normal_ref_rad, distance).tolist())
        r = np.asarray(residuals)
        # `selected` is the same for every convention (what changed between captures);
        # `pixels` is what was scored after that convention's own end-face exclusion.
        per[label(mirrored, euclidean)] = (
            {"selected": selected, "pixels": int(r.size), "max_abs_m": float(np.abs(r).max()),
             "median_m": float(np.median(r))} if r.size
            else {"selected": selected, "pixels": 0})
    fitted = {k: v for k, v in per.items() if v["pixels"]}
    best = min(fitted, key=lambda k: fitted[k]["max_abs_m"]) if fitted else None
    return {"conventions": per, "best": best}


def packet_residuals(baseline, withwall, normal_body_rad: float, distance: float,
                     half_length: float = 0.5 * WALL_LENGTH_M) -> dict[str, Any]:
    """The published packet's filter stream against the wall, in the body frame."""
    b, r = withwall.filter_bearings, withwall.filter_ranges
    off = np.abs((b - normal_body_rad + math.pi) % (2 * math.pi) - math.pi)
    sees = ((baseline.filter_ranges - r > SHORTENED_BY_M) & withwall.filter_valid
            & (off <= math.radians(FACING_DEG)))
    pts = np.column_stack([r * np.cos(b), r * np.sin(b)])
    sees &= on_face(pts, normal_body_rad, half_length)
    if not sees.any():
        return {"bins": 0}
    res = r[sees] * np.cos(b[sees] - normal_body_rad) - distance
    rows = [{"bearing_deg": float(math.degrees(x)), "range_m": float(y), "residual_m": float(z)}
            for x, y, z in zip(b[sees], r[sees], res, strict=True)]
    return {"bins": int(sees.sum()), "max_abs_m": float(np.abs(res).max()),
            "median_m": float(np.median(res)), "by_bearing": rows}


def distance_fit(medians: dict[float, float]) -> dict[str, float] | None:
    """Median residual against wall distance: slope (a scale error) and intercept (an
    offset). Needs two distances or more."""
    if len(medians) < 2:
        return None
    d = np.array(sorted(medians))
    m = np.array([medians[k] for k in d])
    slope, intercept = np.polyfit(d, m, 1)
    return {"slope_mm_per_m": float(slope * 1000), "intercept_mm": float(intercept * 1000)}


def self_hits(packet, table: FootprintTable) -> dict[str, Any]:
    """Valid returns inside the robot's own footprint (+ margin) in a wall-free scan."""
    b, r, v = packet.filter_bearings, packet.filter_ranges, packet.filter_valid
    inside = v & (r <= table.extent(b) + SELF_MARGIN_M)
    return {"count": int(inside.sum()),
            "bearings_deg": [float(math.degrees(x)) for x in b[inside]],
            "nearest_return_m": float(r[v].min()) if v.any() else None}


# ---------------------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    hash_before = file_sha256(scene)
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    range_max = float(cfg["scanners"]["range_max"])
    yaw_offset = float(cfg["robot"]["yaw_offset"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SCANCHECK_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    client.load_scene(str(scene))
    scanner = Scanner(client, cfg)
    report: dict[str, Any] = {
        "walls_body_deg": WALLS_BODY_DEG, "distances_m": list(WALL_DISTANCES_M),
        "yaw_offset_rad": yaw_offset,
        "configured_convention": label(scanner.pixel_mirror, scanner.depth_is_euclidean),
        "accept_residual_m": ACCEPT_RESIDUAL_M}
    captures: dict[str, Any] = {}
    wall = None
    try:
        client.start(settle_steps=SETTLE_STEPS)
        report["step_without_scanner_ms"] = time_steps(client, TIMED_STEPS)
        client.stop()

        robot = client.object(cfg["sim"]["objects"]["base_ref"])
        tree = client.object(cfg["sim"]["objects"]["robot_tree"])
        sensors = [int(h) for h in client.sim.getObjectsInTree(
            tree, client.sim.sceneobject_visionsensor, 0)]

        scanner.install()
        report["legacy_scripts"] = scanner.legacy_report()

        client.start(settle_steps=SETTLE_STEPS)
        report["scanner_status"] = scanner.read_status()
        report["step_with_scanner_ms"] = time_steps(client, TIMED_STEPS)
        client.step()
        captures["baseline"] = {"packet": scanner.read(sim_time=client.time()),
                                "raw": capture_raw(client, sensors, robot)}
        client.stop()

        for name, body_deg in WALLS_BODY_DEG.items():
            for d in WALL_DISTANCES_M:
                client.start(settle_steps=SETTLE_STEPS)
                placed_at = robot_pose(client, robot)
                wall = place_wall(client, robot, math.radians(body_deg) + yaw_offset, d)
                client.step()
                client.step()
                key = f"{name}@{d:.1f}"
                captures[key] = {"packet": scanner.read(sim_time=client.time()),
                                 "raw": capture_raw(client, sensors, robot),
                                 "pose_at_placement": placed_at,
                                 "pose_at_capture": robot_pose(client, robot)}
                # Removed while the simulation still runs. An object created during a
                # run is deleted by CoppeliaSim when the run stops, so removing it after
                # stop() finds an invalid handle (2026-09-21, run 3's crash).
                remove_wall(client, wall)
                wall = None
                client.stop()
    finally:
        remove_wall(client, wall)
        scanner.remove()
        client.stop()
        client.load_scene(str(scene))       # leave the instance on the pinned scene

    if file_sha256(scene) != hash_before:
        raise SystemExit("the scene file changed during the check; it is never saved")
    report["scene_sha256"] = hash_before

    base = captures["baseline"]
    report["self_hits"] = self_hits(base["packet"], table)
    report["coverage"] = {
        "policy_valid": int(base["packet"].policy_valid.sum()),
        "filter_valid": int(base["packet"].filter_valid.sum()),
        "fully_observed": bool(base["packet"].fully_observed)}
    report["walls"] = {}
    for name, body_deg in WALLS_BODY_DEG.items():
        normal_body = math.radians(body_deg)
        entry: dict[str, Any] = {"by_distance": {}}
        medians: dict[float, float] = {}
        for d in WALL_DISTANCES_M:
            cap = captures[f"{name}@{d:.1f}"]
            drift = np.array(cap["pose_at_capture"]) - np.array(cap["pose_at_placement"])
            packet = packet_residuals(base["packet"], cap["packet"], normal_body, d)
            entry["by_distance"][f"{d:.1f}"] = {
                "raw": raw_hypotheses(base["raw"], cap["raw"], normal_body + yaw_offset, d,
                                      range_max),
                "packet": packet,
                "robot_drift_mm_mrad": [float(x * 1000) for x in drift]}
            if packet.get("bins"):
                medians[d] = packet["median_m"]
        entry["distance_fit"] = distance_fit(medians)
        report["walls"][name] = entry
    report["binning_cost_ms"] = (report["step_with_scanner_ms"]["mean_ms"]
                                 - report["step_without_scanner_ms"]["mean_ms"])
    walls_ok = all(v["packet"].get("bins", 0) > 0
                   and v["packet"]["max_abs_m"] <= ACCEPT_RESIDUAL_M
                   for w in report["walls"].values() for v in w["by_distance"].values())
    report["accepted"] = bool(walls_ok and report["self_hits"]["count"] == 0)

    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    (run_dir / "raw_capture.json").write_text(json.dumps(
        {k: {kk: vv for kk, vv in v.items() if kk != "packet"} for k, v in captures.items()}))
    stamp_display(run_dir, client.display_status())

    print(f"\nconfigured convention: {report['configured_convention']}, bearings in the "
          f"body frame (yaw_offset {yaw_offset:.4f} rad)")
    print(f"sensor axes (body frame): {report['scanner_status'].get('sensor_axes_deg')}")
    print(f"legacy LiDAR scripts: {report['legacy_scripts']}")
    for name, w in report["walls"].items():
        print(f"\n{name} wall (body {WALLS_BODY_DEG[name]:+.0f} deg):")
        for d, v in w["by_distance"].items():
            best = v["raw"]["best"]
            b = v["raw"]["conventions"].get(best, {}) if best else {}
            p = v["packet"]
            drift = v["robot_drift_mm_mrad"]
            print(f"  {d} m  raw best {best} (max {b.get('max_abs_m', float('nan')) * 1000:6.2f} mm)"
                  f"  | packet {p.get('bins', 0):3d} bins, max "
                  f"{p.get('max_abs_m', float('nan')) * 1000:6.2f} mm, median "
                  f"{p.get('median_m', float('nan')) * 1000:+6.2f} mm"
                  f"  | robot drift {drift[0]:+.1f},{drift[1]:+.1f} mm, yaw {drift[3]:+.2f} mrad")
        if w["distance_fit"]:
            f = w["distance_fit"]
            print(f"  median vs distance: {f['slope_mm_per_m']:+.2f} mm/m, "
                  f"intercept {f['intercept_mm']:+.2f} mm")
    sh = report["self_hits"]
    print(f"\nself-hits in the wall-free baseline: {sh['count']} "
          f"(nearest return {sh['nearest_return_m']})")
    c = report["coverage"]
    print(f"coverage: policy {c['policy_valid']}/{scanner.policy_bins}, "
          f"filter {c['filter_valid']}/{scanner.filter_bins}")
    display = client.display_status()
    note = "" if display["consistent"] else "  (display state changed: timing not comparable)"
    print(f"step: {report['step_without_scanner_ms']['mean_ms']:.2f} ms without, "
          f"{report['step_with_scanner_ms']['mean_ms']:.2f} ms with the scanner "
          f"-> binning {report['binning_cost_ms']:+.2f} ms{note}")
    print(f"\nACCEPTED: {report['accepted']}  (every wall at every distance within "
          f"{ACCEPT_RESIDUAL_M * 1000:.0f} mm, no self-hits)")
    print(f"-> {run_dir}")
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
