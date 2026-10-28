"""Oracle mission time, old against new (A20, ruled 2026-10-19). Offline, seconds.

    python scripts/compare_mission_time.py --config configs/base.yaml --seed 0 \
        --old <pre-A20 info run dir> --new <A20 info run dir>

Both are `scripts/quantify_oracle_free.py --phase info` runs at lethal 0.10, soft 0.03: the OLD at
the last pre-A20 code (28afe21) on the pre-A20 sets, the NEW at the A20 code (v_max 0.50) on the
rebuilt sets. See `pfbnav/eval/mission_time.py` for what is compared and why only the wide pairs
are the same routes.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.mission_time import matched, per_stratum, read_routes
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
LETHAL, SOFT = 0.10, 0.03


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--old", required=True, help="the pre-A20 info run directory")
    parser.add_argument("--new", required=True, help="the A20 info run directory")
    args = parser.parse_args()
    cfg = load_config(args.config)
    seed_everything(args.seed)
    old = read_routes(Path(args.old) / "routes.csv", LETHAL, SOFT)
    new = read_routes(Path(args.new) / "routes.csv", LETHAL, SOFT)
    report = {"lethal": LETHAL, "soft": SOFT, "old_run": args.old, "new_run": args.new,
              "old": per_stratum(old), "new": per_stratum(new), "matched": matched(old, new)}
    for s in sorted(set(report["old"]) | set(report["new"])):
        o, n = report["old"].get(s, {}), report["new"].get(s, {})
        m = report["matched"].get(s)

        def fmt(v, unit=""):
            return "-" if v is None else f"{v:.2f}{unit}"
        print(f"{s:7s}: old {o.get('traversable', 0)}/{o.get('routes', 0)} traversable, median "
              f"{fmt(o.get('median_time_s'), ' s')} ({fmt(o.get('median_s_per_m'), ' s/m')}); new "
              f"{n.get('traversable', 0)}/{n.get('routes', 0)}, median "
              f"{fmt(n.get('median_time_s'), ' s')} ({fmt(n.get('median_s_per_m'), ' s/m')})"
              + (f"; matched {m['pairs']} pairs, new/old median {m['median_ratio']:.3f} "
                 f"[{m['min_ratio']:.3f}, {m['max_ratio']:.3f}]" if m else ""))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "analysis", f"MISSION_TIME_{stamp}", args.seed,
                             cfg)
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
