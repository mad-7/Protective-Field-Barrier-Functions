"""Rendering changes the step's cost but not its result: one G1b profile, display on and off.

`sim.display_enabled` is false by default because rendering costs ~7 ms of a ~27 ms step
(2026-09-19). That is only allowed if it changes nothing else, so this runs the same G1b
profile twice **in one process** -- determinism is claimed within a process only -- with
the display on and then off, and compares the trajectories field by field. Anything but
bit-identical means the display is not a free saving and the default must be revisited.

The order is display-on first, so a difference cannot be blamed on the first run warming
something up: the cheaper configuration runs second and still has to reproduce the
expensive one exactly.

Usage (one GUI instance; a headless one has no display and the script refuses):
    python scripts/regress_display_off.py --config configs/base.yaml --seed 0
"""
from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.regression import compare_trajectories
from pfbnav.sim.client import SimClient, SimConnection
from pfbnav.utils.runio import create_run_dir, stamp_display
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]

PROFILE = "forward"
SPEED_M_S = 0.30          # mid-grid: fast enough to move, inside every ceiling
DISTANCE_M = 5.0          # G1b's committed distance


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "scripts" / f"{name}.py")
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load scripts/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instance", type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    seed_everything(args.seed)
    g1b = _load("gate_g1b_tracking")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"DISPLAY_{stamp}", args.seed, cfg)

    base = SimConnection.from_config(cfg, instance_id=args.instance)
    client = SimClient(dataclasses.replace(base, display_enabled=True))
    client.connect()
    if bool(client.sim.getBoolParam(client.sim.boolparam_headless)):
        raise SystemExit("this instance is headless: there is no display to compare")
    client.load_scene(str(REPO_ROOT / cfg["sim"]["scene"]))

    results = {}
    try:
        for label, display in (("display_on", True), ("display_off", False)):
            client.connection = dataclasses.replace(base, display_enabled=display)
            out = run_dir / label
            out.mkdir()
            r = g1b.run_once(client, cfg, PROFILE, SPEED_M_S, DISTANCE_M, out, 0)
            r["display_in_force"] = client.display_in_force
            results[label] = r
            print(f"{label:<12} display in force {client.display_in_force}  "
                  f"peak {r['max_abs_cross_track']:.6f} m  heading "
                  f"{r['final_heading_error_deg']:.4f} deg  steps {r['steps']}")
    finally:
        client.stop()

    name = f"{PROFILE}_{SPEED_M_S:.2f}_rep0.csv"
    comparison = compare_trajectories(run_dir / "display_on" / name,
                                      run_dir / "display_off" / name)
    report = {"profile": PROFILE, "speed_m_s": SPEED_M_S, "distance_m": DISTANCE_M,
              "runs": results, "comparison": comparison,
              "identical": bool(comparison["identical"])}
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float))
    stamp_display(run_dir, client.display_status())

    print(f"\ntrajectories identical: {comparison['identical']}")
    if not comparison["identical"]:
        print(f"first difference: {comparison['first_difference']}")
        print(f"largest difference per column: {comparison['max_abs_difference']}")
    print(f"-> {run_dir}")
    return 0 if comparison["identical"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
