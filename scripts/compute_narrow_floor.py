"""The narrow floor F under A20 (ruled 2026-10-18): the smallest per-side clearance at which all three
D2 gaps are >= 0, binned, under the FULL reachable-motion construction at the new limits (a_max 0.5
m/s^2, alpha_max 1.4 rad/s^2, bands [0.10, 0.30, 0.51] indexed by reachable speed), plus the recorded
0.03 m buffer. Then the strata: narrow [F, F + 0.07], medium [F + 0.07, 0.40], wide [0.40, 0.80].
Ruled stop: F + 0.07 > 0.30. Offline, seconds.

    python scripts/compute_narrow_floor.py --config configs/base.yaml --seed 0

The gaps are those of `scripts/assess_reach_admissibility.py` (A1 at rest centred, A2 at rest at
the tracking offset, A3 moving at the offset in every band a covered speed selects), found by
bisection on the clearance. The binned gap is what the layer evaluates (120-bin sector minimum);
the geometric one is reported beside it.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.reach import ReachFields, ReachParams
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import certified_params_from_config

REPO_ROOT = Path(__file__).resolve().parents[1]
A_MAX = 0.5                     # m/s^2, ruled (A20)
ALPHA_MAX = 1.4                 # rad/s^2, ruled (A20)
BANDS = (0.10, 0.30, 0.51)      # m/s, ruled (A20), indexed by reachable speed
BUFFER = 0.03                   # m, the recorded narrow buffer (slack)
NARROW_SPAN = 0.07              # m, narrow [F, F + 0.07]
STOP_ABOVE = 0.30               # m, ruled: stop and report if F + 0.07 exceeds it
TOL = 1e-5                      # m, bisection tolerance on the clearance


def _assess():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "assess", REPO_ROOT / "scripts" / "assess_reach_admissibility.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--no-record", action="store_true", help="print only")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    assess = _assess()
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    params = dataclasses.replace(certified_params_from_config(cfg), band_top_speeds=BANDS,
                                 alpha_brake=ALPHA_MAX)
    reach = ReachParams.from_config(cfg, a_max=A_MAX, alpha_max=ALPHA_MAX)
    rf = ReachFields(FieldGeometry(table.extent, params), reach, "full")
    half_width = float(cfg["robot"]["half_width"])
    offset = float(yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml")
                                  .read_text(encoding="utf-8"))
                   ["G1b_closed_loop_tracking"]["max_cross_track"])
    n_bins = int(cfg["scanners"]["filter_bins"])
    bin_width = 2.0 * math.pi / n_bins
    bins = -math.pi + bin_width * (np.arange(n_bins) + 0.5)

    def worst(clearance: float) -> float:
        return assess.gaps(rf, half_width, clearance, offset, bins, bin_width)["worst_gap_m"]

    lo, hi = 0.0, 0.40
    if worst(hi) < 0.0:
        raise SystemExit("no clearance up to 0.40 m admits the narrow requirements")
    while hi - lo > TOL:
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if worst(mid) >= 0.0 else (mid, hi)
    floor0 = hi
    g = assess.gaps(rf, half_width, floor0, offset, bins, bin_width)
    F = floor0 + BUFFER
    at_F = assess.gaps(rf, half_width, F, offset, bins, bin_width)
    strata = {"narrow": [round(F, 4), round(F + NARROW_SPAN, 4)],
              "medium": [round(F + NARROW_SPAN, 4), 0.40], "wide": [0.40, 0.80]}
    stop = F + NARROW_SPAN > STOP_ABOVE
    print(f"bands {BANDS} by reachable speed; a_max {A_MAX}, alpha_max {ALPHA_MAX}, "
          f"T {reach.reach_time:.2f} s, speed cap {reach.speed_cap:.4f} m/s")
    print(f"band structure: {g['bands']}")
    print(f"zero-gap clearance {floor0:.4f} m (binding: "
          + ", ".join(f"{k} {v['binned'] * 1e3:+.2f}/{v['geometric'] * 1e3:+.2f} mm"
                      for k, v in g.items() if k.startswith("A") and "binned" in v)
          + "; A3 " + " ".join(f"{t}:{v['binned'] * 1e3:+.2f}/{v['geometric'] * 1e3:+.2f}"
                               for t, v in g["A3_moving_offset"].items()) + ")")
    print(f"F = {floor0:.4f} + {BUFFER} = {F:.4f} m; at F the worst gap is "
          f"{at_F['worst_gap_m'] * 1e3:.1f} mm (binned)")
    print(f"strata: {strata}; F + 0.07 = {F + NARROW_SPAN:.4f} "
          f"{'> 0.30: STOP AND REPORT' if stop else '<= 0.30: proceed'}")
    report = {"a_max": A_MAX, "alpha_max": ALPHA_MAX, "bands": BANDS, "variant": "full",
              "reach_time_s": reach.reach_time, "speed_cap": reach.speed_cap,
              "zero_gap_clearance_m": floor0, "buffer_m": BUFFER, "F_m": F,
              "gaps_at_zero_gap_clearance": g, "gaps_at_F": at_F, "strata": strata,
              "stop": stop}
    if args.no_record:
        return 0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"NARROW_FLOOR_A20_{stamp}",
                             args.seed, cfg)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
