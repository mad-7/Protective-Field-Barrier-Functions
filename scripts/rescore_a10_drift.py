"""Re-score A10's drift criterion for a finished probe run, on signed drift.

The first A10 probe (DSTEP_20260918T202513Z_seed0) compared drift *magnitudes* with the
frozen step's, which let a drift of the same size in the opposite direction count as "no
difference". This reads that run's own per-candidate `lateral.csv` files, recomputes the
drift with its sign (`signed_heading_drift_deg`), checks that the magnitudes reproduce the
ones the run recorded, and re-applies A10 with `drift_band_holds`.

The run directory is only read -- nothing under runs/ is modified (hard rule 1). The
correction is written to docs/corrections/<run name>.json and .md, next to nothing else,
and cited from docs/decision_log.md.

Usage:
    python scripts/rescore_a10_drift.py --run runs/diag/DSTEP_20260918T202513Z_seed0
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from pfbnav.eval.gates import drift_band_holds, heading_drift_deg, signed_heading_drift_deg

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "corrections"

# The magnitudes recomputed from the CSV must reproduce the recorded ones to this, in deg;
# anything larger means the CSV and the summary disagree and the re-score is not valid.
REPRODUCE_TOL_DEG = 1e-6


def drift_from_csv(path: Path, dt: float) -> tuple[float, float]:
    """(signed drift, magnitude) in degrees from one open-loop `lateral.csv`."""
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} is empty")
    yaw = [float(r["yaw"]) for r in rows]
    wz = {float(r["cmd_wz"]) for r in rows}
    if wz != {0.0}:
        raise ValueError(f"{path}: the drift run commands a yaw rate {sorted(wz)}; expected 0")
    return signed_heading_drift_deg(yaw, 0.0, dt), heading_drift_deg(yaw, 0.0, dt)


def rescore(run: Path) -> dict:
    summary = json.loads((run / "summary.json").read_text())
    provenance = json.loads((run / "provenance.json").read_text())
    band = float(summary["drift_band_deg"])
    frozen = f"{float(summary['frozen_step_s']):.3f}"
    dt = float(next(iter(summary["candidates"].values()))["physics"]["simulation_time_step"])

    rows = {}
    for key, c in summary["candidates"].items():
        signed, magnitude = drift_from_csv(run / f"step_{key}" / "lateral.csv", dt)
        recorded = float(c["lateral_drift_deg"])
        if abs(magnitude - recorded) > REPRODUCE_TOL_DEG:
            raise SystemExit(f"step {key}: recomputed |drift| {magnitude} does not reproduce "
                             f"the recorded {recorded}; not re-scoring")
        rows[key] = {"signed_drift_deg": signed, "recorded_magnitude_deg": recorded,
                     "g1b_pass_with_ceilings": bool(c["g1b"]["result"] == "PASS"
                                                    and c["g1b_ceilings_unchanged"]),
                     "g0_zero_lag": bool(c["g0_zero_lag"]),
                     "recorded_drift_within_band": bool(c["drift_within_band"]),
                     "recorded_passes_a10": bool(c["passes_a10"])}

    reference = rows[frozen]["signed_drift_deg"]
    passing = []
    for key, r in rows.items():
        r["signed_gap_deg"] = r["signed_drift_deg"] - reference
        r["drift_within_band"] = drift_band_holds(r["signed_drift_deg"], reference, band)
        r["passes_a10"] = (r["g1b_pass_with_ceilings"] and r["g0_zero_lag"]
                           and r["drift_within_band"])
        if r["passes_a10"] and key != frozen:
            passing.append(float(key))
    return {
        "run": run.name, "run_commit": provenance["commit"], "run_dirty": provenance["dirty"],
        "scene_sha256": provenance.get("scene_sha256"), "band_deg": band,
        "frozen_step_s": float(frozen), "reference_signed_drift_deg": reference,
        "recorded_largest_passing_step_s": summary["largest_passing_step_s"],
        "corrected_largest_passing_step_s": max(passing) if passing else None,
        "candidates": rows,
    }


def as_markdown(r: dict) -> str:
    lines = [
        f"# Correction: A10 drift criterion, {r['run']}",
        "",
        (f"Run commit `{r['run_commit'][:7]}` (dirty: {r['run_dirty']}), scene "
        f"`{(r['scene_sha256'] or '')[:8]}...`. The run's files are unchanged; this re-scores "
        "its own `lateral.csv` files with `scripts/rescore_a10_drift.py`."),
        "",
        ("**Defect.** The probe compared drift magnitudes, `abs(|d| - |d_ref|) <= band`. "
        "The band (0.64 deg) was measured on three drifts of one sign, so A10's criterion "
        "is on signed drifts: `abs(d - d_ref) <= band` (`drift_band_holds`). Test: "
        "`tests/test_gate_metrics.py::test_drift_band_fails_for_a_reversed_sign_drift_of_"
        "similar_size`."),
        "",
        (f"Reference (frozen step {r['frozen_step_s']} s): {r['reference_signed_drift_deg']:+.3f} "
        f"deg. Band: {r['band_deg']} deg."),
        "",
        ("| step s | signed drift deg | gap to frozen deg | band (recorded) | band (corrected) "
        "| A10 (recorded) | A10 (corrected) |"),
        "|---|---|---|---|---|---|---|",
    ]
    frozen = f"{r['frozen_step_s']:.3f}"
    for key, c in r["candidates"].items():
        if key == frozen:
            # The baseline is not a candidate; its G1b result is reported separately.
            lines.append(f"| {key} | {c['signed_drift_deg']:+.3f} | reference | - | - | "
                         f"baseline | baseline |")
            continue
        lines.append(
            f"| {key} | {c['signed_drift_deg']:+.3f} | {c['signed_gap_deg']:+.3f} | "
            f"{c['recorded_drift_within_band']} | {c['drift_within_band']} | "
            f"{'pass' if c['recorded_passes_a10'] else 'FAIL'} | "
            f"{'pass' if c['passes_a10'] else 'FAIL'} |")
    lines += [
        "",
        (f"Largest passing step: recorded {r['recorded_largest_passing_step_s']} s, corrected "
        f"{r['corrected_largest_passing_step_s']} s (None: 0.005 s stays)."),
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    args = parser.parse_args()
    run = Path(args.run)
    result = rescore(run)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for suffix, text in ((".json", json.dumps(result, indent=2)),
                         (".md", as_markdown(result))):
        out = OUT_DIR / f"{run.name}{suffix}"
        if out.exists():
            raise SystemExit(f"{out} exists; refusing to overwrite a correction")
        out.write_text(text, encoding="utf-8")
    print(as_markdown(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
