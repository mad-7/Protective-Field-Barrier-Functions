"""Relabel the 2026-10-13 traced episodes under the 2026-10-16 outcome rule (supervisor's check).

Reads each trace's final record (read-only; nothing under runs/ is modified) and reports, per
episode, the recorded label, the old rule's label (which must reproduce it) and the new rule's.
Expected: exactly two labels change, `TRACE_20261013T215044Z` trace 2 and `TRACE_20261013T221601Z`
trace 13, from `timeout` to `unrecoverable_stop`. Seconds; no simulator.

    python scripts/relabel_traces.py --config configs/base.yaml --seed 0

Writes `runs/diag/RELABEL_<timestamp>_seed<seed>/summary.json`.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.envs.reward import RewardWeights
from pfbnav.eval.relabel import relabel
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
TRACE_RUNS = ("TRACE_20261013T215044Z_seed0", "TRACE_20261013T220231Z_seed0",
              "TRACE_20261013T221601Z_seed0")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    finals = []
    for run in TRACE_RUNS:
        for path in sorted((REPO_ROOT / "runs" / "diag" / run).glob("trace_*.jsonl")):
            last = json.loads(path.read_text(encoding="utf-8").splitlines()[-1])
            finals.append({"name": f"{run}/{path.name}", **last})
    if len(finals) != 16:
        raise SystemExit(f"expected the 16 traced episodes, found {len(finals)}")
    rows = relabel(finals, RewardWeights.from_config(cfg))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"RELABEL_{stamp}", args.seed, cfg)
    changed = [r for r in rows if r["changed"]]
    summary = {"episodes": len(rows), "reproduced_by_old_rule": sum(r["reproduced"] for r in rows),
               "changed": changed, "rows": rows}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    for r in rows:
        mark = "CHANGED" if r["changed"] else ""
        print(f"{r['name']:45s} recorded {r['recorded']:18s} old {r['old_rule']:18s} "
              f"new {r['new_rule']:18s} {mark}")
    print(f"{summary['reproduced_by_old_rule']}/{len(rows)} reproduced by the old rule; "
          f"{len(changed)} changed -> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
