"""Do the tune runs already show the test set's success collapse, on their obstacle routes? Read-only,
seconds, no simulator, never touches a test set (2026-11-08).

    python scripts/split_tune_by_obstacles.py --config configs/base.yaml --seed 0 \
        --run B1=runs/tuning/B1_pure_pursuit_20261104T183811Z_seed0/candidate_13 \
        --run B2=runs/tuning/B2_regulated_pure_pursuit_20261030T111716Z_seed0/candidate_23 \
        --run B3=runs/tuning/B3_holonomic_dwa_20261104T141250Z_seed0/candidate_11 \
        --run B6=runs/tuning/B6_field_aware_mppi_20261104T141227Z_seed0/candidate_11 \
        --run B7=runs/tuning/B7_alpha_20261102T195759Z_seed0/candidate_0

The test's unmapped_static sets put obstacles on EVERY route; the three tune sets on 17 of 60. Each run's
episodes are split by whether their scenario has an unmapped obstacle (`unmapped_count`, joined on
`scenario_sha256`), through the same `outcome_summary` as every evaluation. Each run went through the
ordinary tuning code, not `scripts/evaluate_test.py`'s filter switch, so agreement between the obstacle
routes here and the test says the scenarios explain the test's success rates; disagreement points at the
sealed runner. Monitoring only: tune-set evidence, 17 routes, cited as a check and never as a result.
"""
from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from pfbnav.config import load_config
from pfbnav.eval.runner import outcome_summary

REPO_ROOT = Path(__file__).resolve().parents[1]
TUNE_SETS = ("tune_narrow", "tune_medium", "tune_wide")


def obstacle_flags(manifests: Sequence[Mapping[str, Any]]) -> dict[str, bool]:
    """{scenario_sha256: has an unmapped obstacle}. Refuses a scenario hash seen twice."""
    flags: dict[str, bool] = {}
    for manifest in manifests:
        for s in manifest["scenarios"]:
            key = str(s["scenario_sha256"])
            if key in flags:
                raise ValueError(f"scenario {key[:12]} appears twice")
            flags[key] = int(s["unmapped_count"]) > 0
    return flags


def split_records(records: Sequence[Mapping[str, Any]], flags: Mapping[str, bool]
                  ) -> dict[str, list[Mapping[str, Any]]]:
    """Episodes with and without obstacles. Every episode must belong to a known scenario, and every
    known scenario must have exactly one episode: a run of another set is refused, not half-read."""
    keys = [str(r["scenario_sha256"]) for r in records]
    if sorted(keys) != sorted(flags):
        raise ValueError("the run's episodes are not one per scenario of these sets")
    out: dict[str, list[Mapping[str, Any]]] = {"obstacles": [], "clear": []}
    for r, k in zip(records, keys, strict=True):
        out["obstacles" if flags[k] else "clear"].append(r)
    return out


def row(records: Sequence[Mapping[str, Any]], dt: float) -> dict[str, Any]:
    o = outcome_summary(list(records), dt)
    return {"episodes": o["episodes"], "success_rate": o["success_rate"], "counts": o["counts"],
            "stops": o["protective_stops"], "per_100m_planned": o["stops_per_100m_planned"],
            "driven_over_planned": (o["distance_m"] / o["planned_length_m"]
                                    if o["planned_length_m"] else None),
            "moving_mps": o["moving_speed_mps"], "latched": o["latched_step_share"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", action="append", required=True, help="ARM=<run or candidate directory>")
    args = parser.parse_args()
    dt = float(load_config(args.config)["sim"]["dt"])
    flags = obstacle_flags([json.loads((REPO_ROOT / "scenarios" / "tune" / f"{n}.json")
                                       .read_text(encoding="utf-8")) for n in TUNE_SETS])
    print(f"tune sets: {sum(flags.values())} of {len(flags)} scenarios have unmapped obstacles")
    for item in args.run:
        arm, _, path = item.partition("=")
        log = REPO_ROOT / path / "episodes.jsonl"
        records = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines() if x]
        parts = split_records(records, flags)
        for name, part in parts.items():
            r = row(part, dt)
            print(f"{arm} {name}: n {r['episodes']}, success {r['success_rate']:.3f}, {r['counts']}, "
                  f"stops {r['stops']}, per 100 m planned {r['per_100m_planned']:.2f}, driven/planned "
                  f"{r['driven_over_planned']:.2f}, moving {r['moving_mps']:.3f} m/s, "
                  f"latched {r['latched']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
