"""The traversability precondition, and the unavoidable-stop floor, measured per stratum.

Supervisor, 2026-09-26 and 2026-09-27. For each aisle stratum, drive the aisle at the
stratum's derived traversal speed, twice:

  * **offset 0** -- an ideal centreline tracker. Its protective-stop rate is the stratum's
    **unavoidable-stop floor**: no controller can do better, because zero cross-track error is
    the best case. H1 is reported as stops above this floor, beside the raw count.
  * **offset = G1b's committed peak cross-track** -- the worst position a controller inside its
    tolerance may occupy. If the robot is **permanently** inside the field there, the stratum
    is inadmissible and the floor must be raised. The field is never shrunk.

**The certified layer is evaluated but does not override**, as in G0's `env_step_loop` context
and for the same reason: a stop zeroes the command, the traverse ends, distance goes to zero
and stops-per-100-m diverges. What is wanted is how many stops the geometry forces over a
distance actually travelled.

**Why the offset is held closed-loop** (corrected 2026-09-28). The first version drove
open-loop and justified it by G1b's 0.3 mm peak cross-track -- but G1b is CLOSED-loop; open-loop
is what G1 measured, and G1 failed. Open-loop, the robot drifted 2.4 mm sideways over 3 m on
2026-09-27 and 7.6 mm on 2026-09-28, so the traverse was not at the offset it claimed, and drift
away from the wall would have made the test easier than stated. The offset is now held by the
same proportional heading and cross-track law G1b validated, with its committed gains, and a
positive control refuses a traverse that strays from its line by more than `LINE_HELD_TOL_M`.
"A correctly tracking robot at the cross-track bound" is then what is measured.

The aisle is two thin walls in the WORLD frame (unlike `tests/test_d2_real_scan.py`, where they
are placed about `base_ref` for a stationary robot), long enough for the whole traverse. They
exist in memory only and are removed afterwards; the scene is never saved.

**Each wall's FACE is placed at the stratum's clearance** (corrected 2026-09-28). A clearance is
measured to the aisle's surface, and until this date the walls were placed by their CENTRE at
that clearance, so every aisle was 10 mm narrower than the stratum it claimed to test. That
produced the saturated warning field measured on 2026-09-27 and 2026-09-28: the warning field
reaches 0.2795 m abeam, 0.9 mm past a face at 0.2786 m, where a true 0.13 m aisle puts the face
at 0.2886 m. The same wall-thickness artefact had already been found once in the EXPECTATION
(2026-09-27); this is the CONSTRUCTION.

Usage:
    python scripts/probe_traversability.py --config configs/base.yaml --seed 0
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
import yaml

from pfbnav.config import load_config
from pfbnav.safety.certified import CertifiedLayer, FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.robot import YouBotBase, body_point_in_ref, world_to_body, wrap_angle
from pfbnav.sim.scanner import Scanner
from pfbnav.sim.velocity_estimate import (
    PoseDifferenceEstimator,
    VelocityEstimateConfig,
)
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import (
    certified_params_from_config,
    max_traversal_speed,
    strata_from_config,
    usable_band_top,
)
from pfbnav.world.traversability import TraverseResult, precondition_report

REPO_ROOT = Path(__file__).resolve().parents[1]
PREREG = REPO_ROOT / "preregistration" / "gates.yaml"

# The aisle: long enough that the robot never runs out of wall, and tall enough to cross the
# 0.221 m scan plane.
WALL_LENGTH_M = 8.0
WALL_THICKNESS_M = 0.02
WALL_HEIGHT_M = 0.60
# How far each traverse drives. Long enough for the stop rate per 100 m to mean something, short
# enough that four traverses fit in a few minutes.
TRAVERSE_M = 3.0
SETTLE_STEPS = 10
# How far the nearest measured return may sit from the wall FACE the probe built before the
# traverse is treated as not having seen it. Measured 2026-09-27 over six traverses at three
# clearances: the residual against the face is within +-0.5 mm and does not scale with distance.
# 5 mm is ten times that and still catches a wall that is not there at all.
WALL_SEEN_TOL_M = 0.005
# A cap on the step loop, so a traverse that is not making progress ends rather than hanging.
MAX_STEP_FACTOR = 3.0
# Half the wall's thickness: the wall's centre sits this far BEHIND its face, so that the face is
# where the stratum says the aisle's surface is.
HALF_THICKNESS_M = 0.5 * WALL_THICKNESS_M
# How far the robot may stray from the line it starts on before the traverse is refused as not
# being at its stated offset. G1b holds forward travel at 0.40 m/s to 0.34 mm peak with the same
# law; 2 mm is six times that and a quarter of what open-loop drift reached.
LINE_HELD_TOL_M = 0.002


def build_aisle(client: SimClient, cfg: dict[str, Any], clearance: float,
                offset: float) -> list[int]:
    """Two world-fixed walls forming an aisle around the robot's current pose, along body +x.

    The robot sits at per-side clearance `clearance`, displaced by `offset` toward the +y wall:
    the near FACE is `half_width + clearance - offset` from the body origin, the far face
    `half_width + clearance + offset`. Each wall's centre is placed half a thickness behind its
    face. Returns the wall handles.
    """
    sim = client.sim
    yaw = float(cfg["robot"]["yaw_offset"])
    half_width = float(cfg["robot"]["half_width"])
    base_ref = client.object(cfg["sim"]["objects"]["base_ref"])
    ref_z = float(sim.getObjectPosition(base_ref, sim.handle_world)[2])
    w = half_width + clearance
    walls = []
    for y in (w - offset + HALF_THICKNESS_M, -(w + offset + HALF_THICKNESS_M)):
        wall = int(sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            [WALL_LENGTH_M, WALL_THICKNESS_M, WALL_HEIGHT_M], 0))
        # Placed relative to base_ref once, then detached to the world so the robot drives
        # ALONG them rather than carrying them with it.
        x_ref, y_ref, z_ref = body_point_in_ref(
            [0.5 * TRAVERSE_M, y, 0.5 * WALL_HEIGHT_M - ref_z], yaw)
        sim.setObjectOrientation(wall, [0.0, 0.0, yaw], base_ref)
        sim.setObjectPosition(wall, [x_ref, y_ref, z_ref], base_ref)
        pose = sim.getObjectPose(wall, sim.handle_world)
        sim.setObjectParent(wall, -1, True)
        sim.setObjectPose(wall, pose, sim.handle_world)
        walls.append(wall)
    return walls


def traverse(client: SimClient, cfg: dict[str, Any], scanner: Scanner, base: YouBotBase,
             stratum_name: str, clearance: float, speed: float, offset: float,
             run_dir: Path) -> TraverseResult:
    """One drive along one aisle, counting breached steps and the stops that would have fired."""
    dt = float(cfg["sim"]["dt"])
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    params = certified_params_from_config(cfg)
    layer = CertifiedLayer(params, float(cfg["robot"]["half_length"]),
                           float(cfg["robot"]["half_width"]), table.extent)

    client.stop()
    scanner.install()
    client.start(settle_steps=SETTLE_STEPS)
    walls = build_aisle(client, cfg, clearance, offset)
    base.bind()

    # Driven until the distance is covered, not for a step count derived from the commanded
    # speed: the wheel budget means the realised speed is lower (G1b: 0.362 m/s realised at
    # 0.40 commanded), so a step count would silently cut the traverse short. The smoke check
    # on 2026-09-27 travelled 2.65 m of an intended 3.0 m for exactly this reason.
    nominal_steps = math.ceil(TRAVERSE_M / (speed * dt))
    max_steps = int(MAX_STEP_FACTOR * nominal_steps)
    rows: list[dict[str, Any]] = []
    breached_steps = 0
    # The SAME estimator the step loop feeds the emulator: pose differencing over the body
    # frame, filtered. A hand-rolled world-frame difference would hand the layer a heading
    # rotated by `yaw_offset` and select a different band, so the probe would not be measuring
    # what an episode does.
    estimator = PoseDifferenceEstimator(VelocityEstimateConfig.from_config(cfg))
    pose = np.asarray(base.pose(), dtype=float)
    estimator.update((pose[0], pose[1], pose[2]))
    previous = pose[:2].copy()
    distance = 0.0
    # The line to hold, fixed in the world at the start pose: the aisle was built around it, so
    # holding it IS holding the offset. The law and gains are G1b's (configs/base.yaml).
    gains = cfg["tracking"]
    origin = pose[:2].copy()
    yaw0 = float(pose[2])
    along = np.array([math.cos(yaw0), math.sin(yaw0)])
    normal = np.array([-along[1], along[0]])
    worst_cross_track = 0.0
    nearest = math.inf
    try:
        for step in range(max_steps):
            if distance >= TRAVERSE_M:
                break
            x_now, y_now, yaw_now = base.pose()
            cross = float(np.dot(np.array([x_now, y_now]) - origin, normal))
            worst_cross_track = max(worst_cross_track, abs(cross))
            desired = speed * along - float(gains["k_cross_track"]) * cross * normal
            vx, vy = world_to_body(desired, yaw_now)
            wz = float(gains["k_heading"]) * wrap_angle(yaw0 - yaw_now)
            base.command_body_velocity([vx, vy, wz])
            client.step()
            packet = scanner.read(sim_time=client.time())
            pose = np.asarray(base.pose(), dtype=float)
            velocity = estimator.update((pose[0], pose[1], pose[2]))
            distance += float(np.linalg.norm(pose[:2] - previous))
            previous = pose[:2].copy()
            state = layer.step(client.time(), velocity,
                               packet.filter_ranges, packet.filter_bearings,
                               packet.filter_valid, intended_direction=(1.0, 0.0))
            breached_steps += int(state["breached"])
            ranges = np.asarray(packet.filter_ranges, dtype=float)
            valid = np.asarray(packet.filter_valid, dtype=bool)
            min_range = float(ranges[valid].min()) if valid.any() else math.inf
            nearest = min(nearest, min_range)
            rows.append({
                "step": step, "t": round(step * dt, 4),
                "x": pose[0], "y": pose[1], "yaw": pose[2],
                "distance_m": distance,
                "breached": int(state["breached"]),
                "stop_triggered": int(state["stop_triggered"]),
                "field_id": state["field_id"], "warning": int(state["warning"]),
                "min_range_m": min_range,
                "cross_track_m": cross,
                "vx": velocity[0], "vy": velocity[1],
                "speed_est": float(math.hypot(velocity[0], velocity[1])),
            })
    finally:
        base.halt()
        for wall in walls:
            if bool(client.sim.isHandle(wall)):
                client.sim.removeObjects([wall])
        scanner.remove()
        client.stop()

    name = f"{stratum_name}_offset{round(offset * 1000):03d}mm"
    with (run_dir / f"{name}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # **Two positive controls** (hard rule 14; hard rule 15's second half). "No steps inside the
    # field" reads the same whether the aisle was clear or never there, and whether the robot was
    # at its offset or had wandered from it. So the probe must show (1) that it saw the wall it
    # built, where it built it, on the FIRST step -- before any motion can move the robot -- and
    # (2) that the robot held the line the aisle was built around.
    expected = float(cfg["robot"]["half_width"]) + clearance - offset
    first = float(rows[0]["min_range_m"]) if rows else math.inf
    if not math.isfinite(first) or abs(first - expected) > WALL_SEEN_TOL_M:
        raise SystemExit(
            f"{stratum_name} at offset {offset:.3f} m: the first-step nearest return was "
            f"{first:.4f} m, but the near wall face was built at {expected:.4f} m. The probe did "
            "not see its own aisle, so a clear result would mean nothing.")
    if worst_cross_track > LINE_HELD_TOL_M:
        raise SystemExit(
            f"{stratum_name} at offset {offset:.3f} m: the robot strayed {worst_cross_track * 1000:.2f}"
            f" mm from its line (limit {LINE_HELD_TOL_M * 1000:.1f} mm), so the traverse was not at "
            "the offset it reports.")
    print(f"   controls: first return {first:.4f} m vs face {expected:.4f} m; "
          f"worst cross-track {worst_cross_track * 1000:.2f} mm; nearest over the traverse "
          f"{nearest:.4f} m")

    return TraverseResult(
        stratum=stratum_name, clearance_m=clearance, speed_m_s=speed, offset_m=offset,
        steps=len(rows), steps_breached=breached_steps, stops=layer.stops,
        distance_m=distance)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    add_display_flag(parser)
    parser.add_argument("--strata", nargs="*", default=None,
                        help="narrow a run to these strata; the result is then PARTIAL")
    args = parser.parse_args()

    cfg = apply_display(load_config(args.config), args.display)
    seed_everything(args.seed)
    criteria = yaml.safe_load(PREREG.read_text(encoding="utf-8"))["G1b_closed_loop_tracking"]
    offset = float(criteria["max_cross_track"])

    strata = strata_from_config(cfg)
    if args.strata:
        unknown = [s for s in args.strata if s not in {x.name for x in strata}]
        if unknown:
            raise SystemExit(f"unknown strata {unknown}")
        strata = [s for s in strata if s.name in args.strata]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"TRAVERSABILITY_{stamp}",
                             args.seed, cfg)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        raise SystemExit(f"scene {scene} does not exist")

    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    client.load_scene(str(scene))
    scanner = Scanner(client, cfg)
    base = YouBotBase.from_config(client, cfg)

    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    params = certified_params_from_config(cfg)
    fields = FieldGeometry(table.extent, params)
    half_width = float(cfg["robot"]["half_width"])

    results: list[TraverseResult] = []
    try:
        for stratum in strata:
            derived = max_traversal_speed(fields, half_width, stratum.min_clearance,
                                          offset=offset)
            speed = usable_band_top(derived, params)
            if speed is None:
                raise SystemExit(
                    f"stratum {stratum.name} admits no band at all ({derived:.4f} m/s); "
                    "that is a strata question, not a probe failure")
            print(f"\n=== {stratum.name}: clearance {stratum.min_clearance:.3f} m, "
                  f"derived speed {derived:.4f} -> band top {speed:.2f} m/s")
            for off in (0.0, offset):
                label = "ideal centreline" if off == 0.0 else f"offset {off * 1000:.0f} mm"
                print(f"-- {label}")
                result = traverse(client, cfg, scanner, base, stratum.name,
                                  stratum.min_clearance, speed, off, run_dir)
                results.append(result)
                print(f"   steps {result.steps}, inside the field "
                      f"{result.fraction_inside:.3f}, stops {result.stops}, "
                      f"distance {result.distance_m:.3f} m, "
                      f"stops/100 m {result.stops_per_100m}")
    finally:
        client.stop()

    report = precondition_report(results)
    report.update(offset_m=offset, traverse_m=TRAVERSE_M,
                  display=client.display_status(),
                  physics=cfg["sim"]["physics"])
    if args.strata:
        report["result"] = "PARTIAL"
        report["partial_strata"] = list(args.strata)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")

    print("\nunavoidable-stop floor per 100 m (ideal centreline tracker):")
    for name, floor in report["unavoidable_stop_floor_per_100m"].items():
        print(f"  {name:<8} {floor}")
    if args.strata:
        print("PARTIAL: not every stratum was probed, so this is not a precondition verdict.")
    elif report["passed"]:
        print("precondition PASSED: no stratum is permanently inside the field")
    else:
        print(f"precondition FAILED: {report['inadmissible_strata']} "
              "-- raise the floor, never shrink the field")
    print(f"-> {run_dir}")
    return 0 if report["passed"] or args.strata else 1


if __name__ == "__main__":
    raise SystemExit(main())
