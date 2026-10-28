"""Coverage map: is every cell around the robot observed, or honestly flagged? (WP1)

A ~20 mm post is swept over a dense grid from the footprint edge out to
`scanners.coverage_depth` (1.0 m) beyond it, at scan-plane height -- the post runs from the
floor to 1 m, so it crosses the scan plane whatever its height. At each cell the published
scan is read and the cell classified (`pfbnav/eval/coverage_map.py`): **observed**,
**invalid** (flagged unobserved), **violation** (a valid bin reading past the post: free
space reported where there is an obstacle), or **confounded** (the bin already read nearer
without the post).

**Pre-declared pass criterion (supervisor, 2026-09-21): zero violations, on both the
policy (36-bin) and the filter (120-bin) stream.** Invalid cells are not failures: on the
inherited centreline layout the zone beside the robot is expected to be flagged invalid,
and saying so is the point of the validity fix. The map is run on both layouts and the
before/after pair is a paper figure (`coverage_map.png`, `.pdf`).

Why a dense sweep (standing rule, 2026-09-21): every earlier scanner check put its walls at
>= 0.6 m, just outside the 0.563 m blind zone, and missed it. Coverage and boundary tests
sweep the region that matters densely, not at convenient distances.

Also recorded: each sensor's pose in the body frame, the scan-plane height, the validity
masks and what the layout leaves unseen, what was found of scanner-model scripts.

The post and the scanner script exist in memory only and are removed afterwards; the
scene is never saved and its SHA-256 is checked before and after (hard rule 12). Please
do not zoom, pan or save the scene while it runs.

Usage (one instance on sim.port, nothing else running):
    python scripts/diag_coverage_map.py --config configs/base.yaml --seed 0 --label centreline
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from pfbnav.config import load_config
from pfbnav.eval.coverage_map import (
    CLASSES,
    VIOLATION,
    bins_spanned,
    classify,
    grid_cells,
    tally,
)
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.scanner import Scanner
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

POST_DIAMETER_M = 0.02         # "a ~20 mm probe post" (supervisor)
POST_HEIGHT_M = 1.0            # floor to 1 m: crosses the scan plane at any mounting height
POST_CLEARANCE_M = 0.005       # beyond the post's radius, so it never touches the robot
SEEN_TOLERANCE_M = 0.02        # reading up to post distance + this counts as seeing it
DEFAULT_SPACING_M = 0.025
SETTLE_STEPS = 10


def body_to_ref(x: float, y: float, yaw_offset: float) -> tuple[float, float]:
    """A body-frame point in youBot_ref's frame (the body frame is ref rotated by yaw)."""
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    return c * x - s * y, s * x + c * y


def draw(cells: np.ndarray, classes: list[str], scanner: Scanner, title: str,
         path_stem: Path) -> None:
    """The coverage map as a figure: cells by class, the footprint, the sensors."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colours = {"observed": "#1D9E75", "invalid": "#EF9F27", "violation": "#E24B4A",
               "confounded": "#888780"}
    fig, ax = plt.subplots(figsize=(6.0, 6.0))
    c = np.array(classes)
    for name in CLASSES:
        sel = c == name
        if sel.any():
            # Plotted in the body frame with forward up: x_plot = -y, y_plot = x.
            ax.scatter(-cells[sel, 1], cells[sel, 0], s=2.0, c=colours[name], marker="s",
                       linewidths=0, label=f"{name} ({int(sel.sum())})")
    b = np.radians(np.arange(-180.0, 181.0, 1.0))
    e = np.asarray(scanner.footprint.extent(b))
    ax.plot(-e * np.sin(b), e * np.cos(b), color="#2C2C2A", linewidth=1.0)
    for s in scanner.sensors:
        v = s["view"]
        ax.plot(-v.y, v.x, marker="o", color="#2C2C2A", markersize=4)
        for edge in (v.heading - 0.5 * v.fov, v.heading + 0.5 * v.fov):
            ax.plot([-v.y, -v.y - 0.25 * math.sin(edge)], [v.x, v.x + 0.25 * math.cos(edge)],
                    color="#2C2C2A", linewidth=0.6, linestyle=":")
    ax.set_aspect("equal")
    ax.set_xlabel("-y (m), body frame: left is left")
    ax.set_ylabel("x (m), forward up")
    ax.set_title(title, fontsize=10)
    ax.legend(loc="upper right", fontsize=7, markerscale=4)
    fig.tight_layout()
    fig.savefig(path_stem.with_suffix(".png"), dpi=200)
    fig.savefig(path_stem.with_suffix(".pdf"))
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--label", required=True,
                        help="which layout this is, for the run name and the figure")
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--spacing", type=float, default=DEFAULT_SPACING_M,
                        help="m between grid cells")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = apply_display(cfg, args.display)
    seed_everything(args.seed)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    hash_before = file_sha256(scene)
    yaw = float(cfg["robot"]["yaw_offset"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"COVMAP-{args.label}_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    client.load_scene(str(scene))
    scanner = Scanner(client, cfg)
    depth = scanner.coverage_depth
    cells = grid_cells(scanner.footprint.extent, depth, args.spacing,
                       0.5 * POST_DIAMETER_M + POST_CLEARANCE_M)
    print(f"{len(cells)} cells at {args.spacing * 1000:.0f} mm spacing, footprint edge to "
          f"{depth} m beyond it")

    report: dict[str, Any] = {"label": args.label, "spacing_m": args.spacing,
                              "depth_m": depth, "post_diameter_m": POST_DIAMETER_M,
                              "seen_tolerance_m": SEEN_TOLERANCE_M, "cells": len(cells)}
    rows: list[dict[str, Any]] = []
    post = None
    t0 = time.time()
    try:
        scanner.install()
        report["legacy_scripts"] = scanner.legacy_report()
        report["coverage_geometry"] = scanner.coverage_report()
        client.start(settle_steps=SETTLE_STEPS)
        report["scanner_status"] = scanner.read_status()
        ref = client.object(cfg["sim"]["objects"]["base_ref"])
        ref_z = float(client.sim.getObjectPosition(ref, client.sim.handle_world)[2])
        sensor_z = [float(client.sim.getObjectPosition(s["handle"], client.sim.handle_world)[2])
                    for s in scanner.sensors]
        report["scan_plane"] = {"sensor_world_z_m": sensor_z, "base_ref_world_z_m": ref_z}

        client.step()
        baseline = scanner.read(sim_time=client.time())
        post = int(client.sim.createPrimitiveShape(
            client.sim.primitiveshape_cylinder,
            [POST_DIAMETER_M, POST_DIAMETER_M, POST_HEIGHT_M], 0))
        post_z = 0.5 * POST_HEIGHT_M - ref_z          # floor to 1 m, relative to base_ref

        for k, (x, y) in enumerate(cells):
            rx, ry = body_to_ref(float(x), float(y), yaw)
            client.sim.setObjectPosition(post, [rx, ry, post_z], ref)
            client.step()
            packet = scanner.read(sim_time=client.time())
            dist = math.hypot(x, y)
            bearing = math.atan2(y, x)
            row: dict[str, Any] = {"x": float(x), "y": float(y), "distance_m": dist,
                                   "bearing_deg": math.degrees(bearing)}
            for stream, n, ranges, valid, base in (
                    ("filter", scanner.filter_bins, packet.filter_ranges, packet.filter_valid,
                     baseline.filter_ranges),
                    ("policy", scanner.policy_bins, packet.policy_ranges, packet.policy_valid,
                     baseline.policy_ranges)):
                # The post is not a point: judge it over every bin it spans, the bin
                # holding its axis first (measured 2026-09-22; see bins_spanned).
                spanned = bins_spanned(bearing, dist, 0.5 * POST_DIAMETER_M, n)
                row[f"{stream}_bin"] = spanned[0]
                row[f"{stream}_range_m"] = float(ranges[spanned[0]])
                row[f"{stream}_bins_spanned"] = " ".join(str(i) for i in spanned)
                row[f"{stream}_spanned_ranges_m"] = " ".join(
                    f"{float(ranges[i]):.4f}" for i in spanned)
                row[f"{stream}_class"] = classify([bool(valid[i]) for i in spanned],
                                                  [float(ranges[i]) for i in spanned],
                                                  [float(base[i]) for i in spanned],
                                                  dist, SEEN_TOLERANCE_M)
            rows.append(row)
            if (k + 1) % 500 == 0:
                rate = (k + 1) / (time.time() - t0)
                print(f"  {k + 1}/{len(cells)} cells, {rate:.1f} per s")
        client.sim.removeObjects([post])            # while running (it would vanish at stop)
        post = None
        client.stop()
    finally:
        if post is not None and bool(client.sim.isHandle(post)):
            client.sim.removeObjects([post])
        scanner.remove()
        client.stop()
        client.load_scene(str(scene))           # leave the instance on the pinned scene

    if file_sha256(scene) != hash_before:
        raise SystemExit("the scene file changed during the run; it is never saved")
    report["scene_sha256"] = hash_before
    report["wall_seconds"] = time.time() - t0
    for stream in ("filter", "policy"):
        counts = tally(r[f"{stream}_class"] for r in rows)
        report[f"{stream}_counts"] = counts
        report[f"{stream}_violations"] = [
            {k: r[k] for k in ("x", "y", "distance_m", "bearing_deg",
                               f"{stream}_bin", f"{stream}_range_m",
                               f"{stream}_bins_spanned", f"{stream}_spanned_ranges_m")}
            for r in rows if r[f"{stream}_class"] == VIOLATION]
    report["passed"] = (report["filter_counts"][VIOLATION] == 0
                        and report["policy_counts"][VIOLATION] == 0)

    with (run_dir / "cells.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())
    blind = report["coverage_geometry"]
    draw(cells, [r["filter_class"] for r in rows], scanner,
         f"{args.label}: filter stream, {len(cells)} cells, "
         f"{report['filter_counts'][VIOLATION]} violations; geometric blind area "
         f"{blind['unobserved_area_m2']:.3f} m2", run_dir / "coverage_map")

    print(f"\nlayout {args.label}: {len(scanner.sensors)} sensors")
    for s in blind["sensors"]:
        print(f"  sensor at ({s['x']:+.3f}, {s['y']:+.3f}) m, facing {s['heading_deg']:+7.1f} deg, "
              f"{s['fov_deg']:.1f} deg wide, {s['rays']} rays")
    print(f"scan plane: sensors at world z {[round(z, 3) for z in sensor_z]} m")
    print(f"scanner-model scripts: {report['legacy_scripts']}")
    print(f"geometry leaves {blind['unobserved_area_m2']:.3f} m2 unseen, reaching "
          f"{blind['unobserved_reach_m']:.3f} m beyond the footprint; valid bins: policy "
          f"{blind['policy_valid']}/{scanner.policy_bins}, filter "
          f"{blind['filter_valid']}/{scanner.filter_bins}")
    for stream in ("filter", "policy"):
        print(f"{stream:>6}: {report[f'{stream}_counts']}")
    print(f"\nPASSED (zero violations on both streams): {report['passed']}")
    print(f"-> {run_dir}  (figure: coverage_map.png / .pdf)")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
