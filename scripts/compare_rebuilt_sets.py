"""Check that rebuilt scenario sets reproduce a committed build exactly. Seconds; no simulator.

    python scripts/compare_rebuilt_sets.py --config configs/base.yaml --seed 0 --against <git-rev>

For a rebuild made only to re-stamp the sets (the scenario identity moved, the generator's output
did not): every built set under scenarios/tune, scenarios/pilot and scenarios/train is compared,
scenario by scenario, with the same file at `<git-rev>`, ignoring only `scenarios_sha256`
(`pfbnav/eval/set_compare.py`). Writes `runs/diag/COMPARE_SETS_<timestamp>_seed<seed>/summary.json`
and exits non-zero on any difference.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.set_compare import compare_sets
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--against", required=True, help="the git revision of the previous build")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    results = {}
    for split in ("tune", "pilot", "train"):
        for path in sorted((REPO_ROOT / "scenarios" / split).glob("*.json")):
            rel = path.relative_to(REPO_ROOT).as_posix()
            old = json.loads(subprocess.run(["git", "show", f"{args.against}:{rel}"],
                                            cwd=REPO_ROOT, capture_output=True, text=True,
                                            encoding="utf-8", check=True).stdout)
            new = json.loads(path.read_text(encoding="utf-8"))
            results[rel] = compare_sets(old, new)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"COMPARE_SETS_{stamp}", args.seed, cfg)
    identical = all(r["identical"] for r in results.values())
    (run_dir / "summary.json").write_text(json.dumps({"against": args.against,
                                                      "identical": identical, "sets": results},
                                                     indent=2), encoding="utf-8")
    for rel, r in results.items():
        print(f"{rel:45s} {r['scenarios']:4d} scenarios "
              f"{'identical' if r['identical'] else str(len(r['differences'])) + ' DIFFERENCES'}")
    print(f"{'ALL IDENTICAL' if identical else 'DIFFERENCES FOUND'} -> "
          f"{run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if identical else 1


if __name__ == "__main__":
    raise SystemExit(main())
