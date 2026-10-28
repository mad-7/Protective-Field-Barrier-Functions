"""Impact assessment (1) of the reachable-motion field (ruled 2026-10-18, option a): D2 and
traversability for the NARROW stratum, at rest and moving, centred and at the 50 mm tracking
offset, at alpha_max in {3.0, 2.0, 1.5} and a_max in {1.0, 0.5}, for both translation variants
("ruled": reach along the travel direction; "full": also the swerve an isotropic limiter allows).
Offline, seconds; no simulator.

    python scripts/assess_reach_admissibility.py --config configs/base.yaml --seed 0

**Narrow admissibility, declared before it is computed** (the D2 requirements the strata were
locked on, applied to the new field at the narrow floor, clearance 0.13 m per side):
  A1  a stationary robot centred in the narrowest aisle is clear of its at-rest field;
  A2  a robot at rest at the committed tracking offset (G1b's max_cross_track) is clear, so a
      stop anywhere the controller may be can be cleared;
  A3  the moving field fits at the tracking offset for every band a robot can be in while
      travelling the aisle at a covered speed (straight, zero yaw rate).
Each is a near-side gap, m, > 0: geometric (the field's exact support toward a wall parallel to
travel) AND binned (what the layer evaluates: the 120-bin sector-minimum scan against the field's
reach per bin). Admissibility holds when all six gaps are positive. Also reported, not part of
the criterion: the warning field's abeam extension against the A3 gap (D2 asks for it strictly
inside), the band structure under reach (which speeds any band covers; whether band 0 is
reachable at all), and medium/wide for context.

Ruled consequence: narrow admissibility breaks -> stop and report (raise the floor, never shrink
the field; the strata are the supervisor's ruling).
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from pfbnav.config import load_config
from pfbnav.safety.certified import FieldGeometry
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.safety.reach import VARIANTS, ReachFields, ReachParams, reach_speed
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.strata import (
    binned_radial_margin,
    certified_params_from_config,
    strata_from_config,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
ALPHAS = (3.0, 2.0, 1.5)       # rad/s^2, ruled sensitivity
ACCELS = (1.0, 0.5)            # m/s^2, ruled sensitivity


def covered_speeds(rf: ReachFields) -> dict:
    """The band structure under reach: the highest composed speed any band covers, and the
    composed speeds that select each band (empty when none can)."""
    tops = rf.params.band_top_speeds
    rho = rf.reach.a_max * rf.reach.reach_time
    out, lower = {}, 0.0
    for top in tops:
        hi = top - rho                                  # v + a T <= top
        out[f"{top:.2f}"] = None if hi < lower else [round(lower, 4), round(hi, 4)]
        lower = max(lower, hi)
    return {"bands": out, "max_covered_speed": round(max(tops[-1] - rho, 0.0), 4),
            "speed_cap": round(rf.reach.speed_cap, 4),
            "uncovered_above": round(tops[-1] - rho, 4)
            if reach_speed(rf.reach.speed_cap, rf.reach) > tops[-1] else None}


def gaps(rf: ReachFields, half_width: float, clearance: float, offset: float,
         bins: np.ndarray, bin_width: float) -> dict:
    """The three requirements' near-side gaps, geometric and binned, m."""
    cov = covered_speeds(rf)
    moving_tops = [float(t) for t, span in cov["bands"].items() if span is not None]
    rest_reach_lat = rf.lateral_reach(0.0, 0.0, True)
    out = {
        "A1_rest_centred": {"geometric": half_width + clearance - rest_reach_lat,
                            "binned": binned_radial_margin(rf, bins, half_width + clearance,
                                                           0.0, 0.0, True, bin_width)},
        "A2_rest_offset": {"geometric": half_width + clearance - offset - rest_reach_lat,
                           "binned": binned_radial_margin(rf, bins,
                                                          half_width + clearance - offset,
                                                          0.0, 0.0, True, bin_width)},
    }
    a3 = {}
    for top in moving_tops:
        a3[f"{top:.2f}"] = {
            "geometric": half_width + clearance - offset - rf.lateral_reach(top, 0.0, False),
            "binned": binned_radial_margin(rf, bins, half_width + clearance - offset, top, 0.0,
                                           False, bin_width)}
    out["A3_moving_offset"] = a3
    worst = [out["A1_rest_centred"]["geometric"], out["A1_rest_centred"]["binned"],
             out["A2_rest_offset"]["geometric"], out["A2_rest_offset"]["binned"]]
    worst += [v for g in a3.values() for v in g.values()]
    out["admissible"] = bool(moving_tops) and min(worst) > 0.0
    out["worst_gap_m"] = min(worst)
    out["bands"] = cov
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--no-record", action="store_true",
                        help="print only; create no run directory (a preview)")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    table = FootprintTable.from_csv(REPO_ROOT / cfg["robot"]["footprint_table"])
    fields = FieldGeometry(table.extent, certified_params_from_config(cfg))
    half_width = float(cfg["robot"]["half_width"])
    offset = float(yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml")
                                  .read_text(encoding="utf-8"))
                   ["G1b_closed_loop_tracking"]["max_cross_track"])
    n_bins = int(cfg["scanners"]["filter_bins"])
    bin_width = 2.0 * math.pi / n_bins
    bins = -math.pi + bin_width * (np.arange(n_bins) + 0.5)
    strata = {s.name: s.min_clearance for s in strata_from_config(cfg)}
    warning_abeam = float(cfg["certified"]["warning_extra_abeam"])

    # The present field, for reference: same requirements, computed the same way.
    present = {"A1": half_width + strata["narrow"] - fields.lateral_reach(0.0, 0.0, True),
               "A2": half_width + strata["narrow"] - offset
               - fields.lateral_reach(0.0, 0.0, True),
               "A3": half_width + strata["narrow"] - offset
               - fields.lateral_reach(fields.params.band_top_speeds[-1], 0.0, False)}
    print(f"present field (narrow, {strata['narrow']:.2f} m): A1 {present['A1'] * 1e3:.1f} mm, "
          f"A2 {present['A2'] * 1e3:.1f} mm, A3 {present['A3'] * 1e3:.1f} mm")

    results = []
    for alpha, a, variant in itertools.product(ALPHAS, ACCELS, VARIANTS):
        rp = ReachParams.from_config(cfg, a_max=a, alpha_max=alpha)
        rf = ReachFields(fields, rp, variant)
        row = {"alpha_max": alpha, "a_max": a, "variant": variant,
               "reach_time_s": rp.reach_time,
               "narrow": gaps(rf, half_width, strata["narrow"], offset, bins, bin_width)}
        for name in ("medium", "wide"):
            g = gaps(rf, half_width, strata[name], offset, bins, bin_width)
            row[name] = {"admissible": g["admissible"], "worst_gap_m": g["worst_gap_m"]}
        n = row["narrow"]
        a3 = n["A3_moving_offset"]
        a3_min = min((min(g.values()) for g in a3.values()), default=float("nan"))
        row["warning_inside_A3_gap"] = bool(a3) and warning_abeam < a3_min
        results.append(row)
        print(f"alpha {alpha:.1f} a {a:.1f} {variant:5s}: narrow "
              f"{'ADMISSIBLE' if n['admissible'] else 'BREAKS':10s} worst "
              f"{n['worst_gap_m'] * 1e3:+6.1f} mm | A1 {n['A1_rest_centred']['geometric'] * 1e3:+6.1f}"
              f"/{n['A1_rest_centred']['binned'] * 1e3:+6.1f} A2 "
              f"{n['A2_rest_offset']['geometric'] * 1e3:+6.1f}"
              f"/{n['A2_rest_offset']['binned'] * 1e3:+6.1f} A3 "
              + " ".join(f"{t}:{g['geometric'] * 1e3:+.1f}/{g['binned'] * 1e3:+.1f}"
                         for t, g in a3.items())
              + f" | bands {n['bands']['bands']} max covered {n['bands']['max_covered_speed']}"
              f" | medium {'ok' if row['medium']['admissible'] else 'BREAKS'}"
              f" wide {'ok' if row['wide']['admissible'] else 'BREAKS'}")

    report = {"criterion": "A1, A2, A3 near-side gaps > 0, geometric and binned (declared in "
                           "the script's docstring before computing)",
              "narrow_clearance_m": strata["narrow"], "tracking_offset_m": offset,
              "half_width_m": half_width, "present_field_gaps_m": present, "results": results}
    if args.no_record:
        return 0
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"REACH_ADMISSIBILITY_{stamp}",
                             args.seed, cfg)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, default=float),
                                          encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
