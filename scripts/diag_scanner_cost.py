"""What the scanners cost per step, and which lever makes them cheaper (WP1).

The step-floor diagnosis (2026-09-19, `runs/diag/STEPFLOOR-gui_20260919T234447Z_seed0`)
measured a 20.2 ms step with the display off, of which the two LiDAR pods are ~9.4 ms and
everything else ~10.8 ms. The pods are the robot's scanners and stay in the scene -- the
forward pod sets `robot.half_length`, and with it the footprint table, the lethal radius,
the certified field and the D2 strata -- so the question is how a scan is produced, not
whether. Today they render four 256-wide vision sensors (1024 rays per step) while
`scanners.policy_bins` (36) and `filter_bins` (120) consume 156 bearings, and nothing
reads them at all.

Levers measured here, all with the display off and real-time off (settled by the
step-floor diagnosis), three interleaved passes, each configuration from a freshly loaded
scene:

  implicit_<res>     resolution_x set on every sensor, handled implicitly by the main
                     script as now, nothing read. Says how cost scales with rays.
  explicit_<res>     sim.setExplicitHandling(sensor, 1): the main script leaves the
                     sensors alone and nothing handles them. The cost of having sensors
                     that are never processed.
  explicit_<res>_read  explicit, and every step handles each sensor and reads its depth
                     buffer over the remote API -- the per-call scanner, which is the
                     case the batched step exists to remove. Read timed separately.
  pipeline_<res>     the scene's own LiDAR scripts enabled, so the scan is binned
                     server-side exactly as the platform does it, and the client reads
                     one 144-byte buffer signal per step. This is the closest thing to
                     what `lua/scanner_v2.lua` will cost, and the one to compare against.
  no_scanners        pods removed. A reference, not a configuration: the robot needs
                     scans. It bounds what any scanner design can save.

The scanners are two stock Hokuyo URG-04LX-UG01 (Fast) pods: two vision sensors each,
5 m range (`lua/Lidar_Scripts/child_script.lua`, `maxScanDistance = 5`, matching
`scanners.range_max`). The scene's copies of those scripts are **disabled**, so the cost
measured by the step-floor diagnosis was rendering alone, with no binning in it. The v1
child script bins to 36 in a per-pixel Lua loop and publishes `youBot_lidar`; v2 needs
36 + 120 bins with bearings and validity, so that loop is a cost to measure, not to keep.

The resolutions come from `pfbnav.sim.scanning.resolution_for_bins` and each sensor's own
field of view, not from numbers written here: the sensors' as-built width, the width that
carries `filter_bins` over a full turn, and that width oversampled by two.

Reported per configuration: ms per `sim.step()`, ms per scan read, and their sum -- the
per-step cost the environment inherits. Nothing is adopted here: the scanner design
(resolution, handling, and whether the scan is assembled server-side in
`lua/scanner_v2.lua` rather than read sensor by sensor) is a WP1 decision, and a per-call
read measured here is the case the batched step exists to remove.

The scene is changed in memory only and reloaded from disk afterwards; it is never saved,
and the file's SHA-256 is checked before and after (hard rule 12).

Usage (one instance on sim.port, GUI or headless):
    python scripts/diag_scanner_cost.py --config configs/base.yaml --seed 0 --mode gui
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.profiling import summarise_timings
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.sim.scanning import rays_per_step, resolution_for_bins
from pfbnav.utils.runio import create_run_dir, file_sha256, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

WARMUP_STEPS = 5
TIMED_STEPS = 200
SCAN_SIGNAL = "youBot_lidar"     # set by lua/Lidar_Scripts/child_script.lua
OVERSAMPLE = 2.0          # a stream needing exact bearings gets two rays per bin
EXPLICIT_BIT = 1          # sim.setExplicitHandling: bit 0 is explicit handling


def sensors(client: SimClient) -> list[dict[str, Any]]:
    """The scanners' vision sensors, with the geometry that sets their ray count."""
    sim = client.sim
    found = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_visionsensor, 0):
        h = int(h)
        found.append({
            "handle": h,
            "resolution_x": int(sim.getObjectInt32Param(h, sim.visionintparam_resolution_x)),
            "resolution_y": int(sim.getObjectInt32Param(h, sim.visionintparam_resolution_y)),
            "perspective": bool(sim.getObjectInt32Param(
                h, sim.visionintparam_perspective_operation)),
            "fov_deg": math.degrees(float(sim.getObjectFloatParam(
                h, sim.visionfloatparam_perspective_angle))),
            "near_clipping": float(sim.getObjectFloatParam(
                h, sim.visionfloatparam_near_clipping)),
            "far_clipping": float(sim.getObjectFloatParam(
                h, sim.visionfloatparam_far_clipping)),
        })
    if not found:
        raise SystemExit("no vision sensors in the scene; the scanners are what this measures")
    return found


def apply_resolution(client: SimClient, handles: list[int], resolution_x: int) -> None:
    """Set every sensor's horizontal resolution. The simulation must be stopped."""
    for h in handles:
        client.sim.setObjectInt32Param(h, client.sim.visionintparam_resolution_x,
                                       int(resolution_x))
        applied = int(client.sim.getObjectInt32Param(
            h, client.sim.visionintparam_resolution_x))
        if applied != int(resolution_x):
            raise SystemExit(f"sensor {h}: asked for {resolution_x} rays, it reports {applied}")


def apply_explicit(client: SimClient, handles: list[int], explicit: bool) -> None:
    for h in handles:
        client.sim.setExplicitHandling(h, EXPLICIT_BIT if explicit else 0)
        applied = int(client.sim.getExplicitHandling(h)) & EXPLICIT_BIT
        if bool(applied) != explicit:
            raise SystemExit(f"sensor {h}: explicit handling did not take ({applied})")


def settings_in_force(client: SimClient) -> dict[str, Any]:
    """The simulator settings this configuration actually ran under.

    Recorded because the first scanner run read ~6 ms above the step-floor run's
    display-off figure, which is what an unapplied `display_enabled` would look like: a
    setting the script asked for but never checked (2026-09-19).
    """
    sim = client.sim
    return {
        "realtime_simulation": bool(sim.getBoolParam(sim.boolparam_realtime_simulation)),
        "display_enabled": bool(sim.getBoolParam(sim.boolparam_display_enabled)),
        "headless": bool(sim.getBoolParam(sim.boolparam_headless)),
        "dynamic_step_size": float(sim.getFloatParam(sim.floatparam_dynamic_step_size)),
        "simulation_time_step": float(
            sim.getFloatParam(sim.floatparam_simulation_time_step)),
    }


def read_scan(client: SimClient, handles: list[int]) -> int:
    """Handle each sensor and read its depth buffer; returns the bytes transferred."""
    sim = client.sim
    total = 0
    for h in handles:
        sim.handleVisionSensor(h)
        depth, _resolution = sim.getVisionSensorDepth(h, 1)
        total += len(depth)
    return total


def scanner_scripts(client: SimClient) -> list[int]:
    """Script objects inside the LiDAR pods: the scene's own scanner pipeline."""
    sim = client.sim
    scripts = []
    for root in (int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
                 if str(sim.getObjectAlias(h)) == "LiDAR"):
        for h in sim.getObjectsInTree(root, sim.sceneobject_script, 0):
            scripts.append(int(h))
    return scripts


def set_scripts_enabled(client: SimClient, handles: list[int], enabled: bool) -> None:
    """Enable or disable script objects. The simulation must be stopped.

    Through the `scriptDisabled` property: CoppeliaSim 4.10 deprecates the int32
    parameter `scriptintparam_enabled` in favour of properties, and writing it is silently
    refused (measured 2026-09-19, `SCANNER-gui_...` aborted on "enabled did not take").
    The same deprecated read is what reported every script as disabled in the step-floor
    inventory, so that reading is not evidence either way.
    """
    for h in handles:
        client.sim.setBoolProperty(h, "scriptDisabled", not enabled)
        applied = not bool(client.sim.getBoolProperty(h, "scriptDisabled"))
        if applied != enabled:
            raise SystemExit(f"script {h}: scriptDisabled did not take (enabled={applied})")


def remove_scanners(client: SimClient) -> int:
    """Remove the LiDAR pod subtrees. In memory only, for the reference measurement."""
    sim = client.sim
    roots = [int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
             if str(sim.getObjectAlias(h)) == "LiDAR"]
    if not roots:
        raise SystemExit("no object aliased 'LiDAR'; the scene or the alias has changed")
    doomed: set[int] = set()
    for root in roots:
        doomed.update(int(x) for x in sim.getObjectsInTree(root, sim.handle_all, 0))
        doomed.add(root)
    sim.removeObjects(sorted(doomed))
    return len(doomed)


def time_configuration(client: SimClient, name: str, spec: dict[str, Any]) -> dict[str, Any]:
    """Apply one configuration to the stopped scene, then time steps and scan reads."""
    if spec["strip"]:
        handles: list[int] = []
        scripts: list[int] = []
        removed = remove_scanners(client)
    else:
        found = sensors(client)
        handles = [s["handle"] for s in found]
        removed = 0
        apply_resolution(client, handles, spec["resolution"])
        apply_explicit(client, handles, spec["explicit"])
        scripts = scanner_scripts(client)
        set_scripts_enabled(client, scripts, spec["pipeline"])

    client.start(settle_steps=0)
    in_force = settings_in_force(client)
    if in_force["realtime_simulation"]:
        raise SystemExit(f"{name}: real-time pacing is on; it would set the step at 50 ms")
    for _ in range(WARMUP_STEPS):
        client.step()
        if spec["read"]:
            read_scan(client, handles)
        if spec["pipeline"]:
            client.sim.getBufferSignal(SCAN_SIGNAL)
    if spec["pipeline"]:
        # A stale script that fails on load would otherwise be read as a cheap step.
        if client.sim.getBufferSignal(SCAN_SIGNAL) is None:
            raise SystemExit(f"{name}: the scanner scripts are enabled but no "
                             f"'{SCAN_SIGNAL}' signal appeared; check the simulator log")
        execs = sum(int(client.sim.getObjectInt32Param(h, client.sim.scriptintparam_execcount))
                    for h in scripts)
        if execs == 0:
            raise SystemExit(f"{name}: the scanner scripts never executed")

    step_ms, scan_ms = [], []
    scan_bytes = 0
    for _ in range(TIMED_STEPS):
        t0 = time.perf_counter_ns()
        client.sim.step()
        t1 = time.perf_counter_ns()
        if spec["read"]:
            scan_bytes = read_scan(client, handles)
        if spec["pipeline"]:
            scan_bytes = len(client.sim.getBufferSignal(SCAN_SIGNAL) or b"")
        t2 = time.perf_counter_ns()
        step_ms.append((t1 - t0) / 1e6)
        scan_ms.append((t2 - t1) / 1e6)
    client.stop()

    total = [a + b for a, b in zip(step_ms, scan_ms, strict=True)]
    result = {
        "spec": spec, "sensors": len(handles), "objects_removed": removed,
        "scanner_scripts": len(scripts), "settings_in_force": in_force,
        "rays_per_step": rays_per_step([spec["resolution"]] * len(handles)) if handles else 0,
        "scan_bytes": scan_bytes,
        "step": summarise_timings(step_ms), "total": summarise_timings(total),
        "scan_read": (summarise_timings(scan_ms)
                      if (spec["read"] or spec["pipeline"]) else None),
        "step_samples_ms": step_ms, "total_samples_ms": total,
    }
    print(f"  {name:<22} step {result['step']['mean_ms']:6.2f}  scan "
          f"{(result['scan_read']['mean_ms'] or 0.0) if result['scan_read'] else 0.0:6.2f}"
          f"  total {result['total']['mean_ms']:6.2f} ms  ({result['rays_per_step']} rays, "
          f"{result['scan_bytes']} B, display {in_force['display_enabled']})")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--mode", choices=("headless", "gui"), required=True)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--passes", type=int, default=3)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    scene = REPO_ROOT / cfg["sim"]["scene"]
    scene_hash_before = file_sha256(scene)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"SCANNER-{args.mode}_{stamp}",
                             args.seed, cfg)

    client = SimClient(SimConnection.from_config(cfg, instance_id=args.instance))
    client.connect()
    sim = client.sim
    if bool(sim.getBoolParam(sim.boolparam_headless)) != (args.mode == "headless"):
        raise SystemExit(f"--mode {args.mode} does not match the instance")
    client.load_scene(str(scene))

    found = sensors(client)
    bins = max(int(cfg["scanners"]["policy_bins"]), int(cfg["scanners"]["filter_bins"]))
    fovs = {round(s["fov_deg"], 6) for s in found}
    if len(fovs) != 1:
        raise SystemExit(f"the sensors have different fields of view {sorted(fovs)}; the "
                         "resolution sweep assumes one scanner geometry")
    fov = fovs.pop()
    as_built = max(s["resolution_x"] for s in found)
    needed = resolution_for_bins(fov, bins)
    oversampled = resolution_for_bins(fov, bins, oversample=OVERSAMPLE)
    sweep = sorted({as_built, oversampled, needed}, reverse=True)
    print(f"{len(found)} sensors, {fov:.1f} deg each, as built {as_built} rays; "
          f"{bins} bins over 360 deg need {needed} rays per sensor "
          f"({oversampled} at x{OVERSAMPLE:g})")

    def spec(resolution: int, *, explicit: bool = False, read: bool = False,
             pipeline: bool = False, strip: bool = False) -> dict[str, Any]:
        return {"resolution": resolution, "explicit": explicit, "read": read,
                "pipeline": pipeline, "strip": strip}

    plan: list[tuple[str, dict[str, Any]]] = []
    for res in sweep:
        plan.append((f"implicit_{res}", spec(res)))
    for res in sweep:
        plan.append((f"explicit_{res}", spec(res, explicit=True)))
        plan.append((f"explicit_{res}_read", spec(res, explicit=True, read=True)))
    for res in sweep:
        plan.append((f"pipeline_{res}", spec(res, pipeline=True)))
    plan.append(("no_scanners", spec(0, strip=True)))

    # Settled by the step-floor diagnosis: real-time pacing off, display off.
    def prepare() -> None:
        sim.setBoolParam(sim.boolparam_realtime_simulation, False)
        if args.mode == "gui":
            sim.setBoolParam(sim.boolparam_display_enabled, False)
            if bool(sim.getBoolParam(sim.boolparam_display_enabled)):
                raise SystemExit("the display did not switch off; every figure here would "
                                 "carry rendering that the step-floor run did not")

    report: dict[str, Any] = {
        "mode": args.mode, "instance": args.instance, "passes": args.passes,
        "timed_steps": TIMED_STEPS, "bins_required": bins, "fov_deg": fov,
        "resolution_as_built": as_built, "resolution_needed": needed,
        "resolution_oversampled": oversampled, "sensors_as_found": found,
        "configurations": {},
    }
    per_pass: dict[str, list[dict[str, Any]]] = {name: [] for name, _ in plan}
    try:
        for p in range(args.passes):
            print(f"pass {p}")
            for name, spec in plan:
                client.load_scene(str(scene))
                prepare()
                per_pass[name].append(time_configuration(client, name, spec))
    finally:
        client.stop()
        client.load_scene(str(scene))       # leave the instance on the unmodified scene

    for name, runs in per_pass.items():
        steps = [x for r in runs for x in r["step_samples_ms"]]
        totals = [x for r in runs for x in r["total_samples_ms"]]
        report["configurations"][name] = {
            "passes": len(runs), "spec": runs[0]["spec"],
            "rays_per_step": runs[0]["rays_per_step"], "scan_bytes": runs[0]["scan_bytes"],
            "step": summarise_timings(steps), "total": summarise_timings(totals),
            "total_mean_ms_by_pass": [r["total"]["mean_ms"] for r in runs],
            "scan_read_mean_ms": ([r["scan_read"]["mean_ms"] for r in runs]
                                  if runs[0]["scan_read"] else None),
            "settings_in_force": runs[0]["settings_in_force"],
            "scanner_scripts": runs[0]["scanner_scripts"],
            "step_samples_ms": steps, "total_samples_ms": totals,
        }

    if file_sha256(scene) != scene_hash_before:
        raise SystemExit("the scene file changed during the measurement; it is never saved")
    report["scene_sha256"] = scene_hash_before

    configs = report["configurations"]
    baseline = configs[f"implicit_{as_built}"]["total"]["mean_ms"]
    stripped = configs["no_scanners"]["total"]["mean_ms"]
    report["scanner_cost_as_built_ms"] = baseline - stripped
    report["cheapest_reading_configuration"] = min(
        ((n, c["total"]["mean_ms"]) for n, c in configs.items()
         if c["spec"]["read"] or c["spec"]["pipeline"]),
        key=lambda x: x[1], default=None)

    samples = {k: {"step": c.pop("step_samples_ms"), "total": c.pop("total_samples_ms")}
               for k, c in configs.items()}
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())
    (run_dir / "samples_ms.json").write_text(json.dumps(samples))

    print(f"\n{'configuration':<22}{'rays':>6}{'step':>8}{'scan':>8}{'total':>8}  by pass")
    for name, c in configs.items():
        scan = c["scan_read_mean_ms"]
        by_pass = " ".join(f"{m:6.2f}" for m in c["total_mean_ms_by_pass"])
        print(f"{name:<22}{c['rays_per_step']:>6}{c['step']['mean_ms']:8.2f}"
              f"{(sum(scan) / len(scan) if scan else 0.0):8.2f}"
              f"{c['total']['mean_ms']:8.2f}  {by_pass}")
    print(f"\nscanners as built cost {report['scanner_cost_as_built_ms']:.2f} ms/step "
          f"(no scanners: {stripped:.2f} ms)")
    print(f"cheapest configuration that actually reads a scan: "
          f"{report['cheapest_reading_configuration']}")
    print("Nothing is adopted here: the scanner design is a WP1 decision, and a per-call "
          "read is what the batched step exists to remove.")
    print(f"-> {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
