"""L1 seed 9000001's decline, read from its logs (ruled 2026-11-03, option b). Read-only, seconds.

    python scripts/diag_training_decline.py --config configs/experiments/L1_sac_shaped.yaml --seed 9000001 --run runs/train/L1_sac_shaped_seed9000001

The supervisor asked: (1) align the success blocks with the curriculum's stage boundaries -- does the
decline start at the switch to the full distribution (40 %)? (2) the replay buffer's composition
after that switch: the share of stall-truncated transitions and of successes. The buffer is not
saved with a run; it is FIFO over the last `training.buffer_size` transitions, so it is
reconstructed exactly from `episodes.jsonl` (`pfbnav.eval.training_diag.buffer_composition`).
Prints, and writes `runs/diag/DECLINE_<timestamp>_seed<seed>/summary.json`:
  * success per 100k steps, with the stages each bin saw, and the stage boundaries in steps;
  * success per stage x stratum, and in the full stage with and without unmapped obstacles;
  * the buffer's composition at each stage switch and every 200k steps.
No code or hyperparameter change follows from this; a defect found is a bug fix, not a lever.
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.eval.training_diag import buffer_composition, success_by_timestep
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
BIN = 100_000          # steps per success bin
EVERY = 200_000        # steps between buffer snapshots


def _share(eps):
    return round(sum(e["outcome"] == "success" for e in eps) / len(eps), 4) if eps else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run", required=True, help="the training run's directory")
    args = parser.parse_args()
    cfg = load_config(args.config)
    run = REPO_ROOT / args.run
    episodes = [json.loads(line) for line in
                (run / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if line]
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    total, buffer = int(summary["total_timesteps"]), int(cfg["training"]["buffer_size"])
    switches = [round(float(s["from_fraction"]) * total)
                for s in summary["curriculum"]["stages"]][1:]
    print(f"{len(episodes)} episodes, {total} steps, buffer {buffer}; stage switches at {switches}")

    bins = success_by_timestep(episodes, list(range(0, total + 1, BIN)))
    print("success per 100k steps (bin ends at step):")
    for b in bins:
        s = "-" if b["success"] is None else f"{b['success']:.3f}"
        print(f"  {b['to']:>8}: {b['episodes']:>4} episodes, success {s}  {','.join(b['stages'])}")

    groups = defaultdict(list)
    for e in episodes:
        groups[(e.get("curriculum_stage"), e.get("stratum"))].append(e)
    per_stratum = {f"{st}/{sr}": {"episodes": len(v), "success": _share(v)}
                   for (st, sr), v in sorted(groups.items(), key=lambda kv: str(kv[0]))}
    print("success per stage/stratum:")
    for k, v in per_stratum.items():
        print(f"  {k}: {v}")
    full = [e for e in episodes if e.get("curriculum_stage") == "full"]
    unmapped = {"with": {"episodes": sum(int(e["unmapped_count"]) > 0 for e in full),
                         "success": _share([e for e in full if int(e["unmapped_count"]) > 0])},
                "without": {"episodes": sum(int(e["unmapped_count"]) == 0 for e in full),
                            "success": _share([e for e in full if int(e["unmapped_count"]) == 0])}}
    print(f"full stage, unmapped obstacles: {unmapped}")

    snapshots = sorted(set(switches) | set(range(EVERY, total + 1, EVERY)))
    buffers = [buffer_composition(episodes, at, buffer) for at in snapshots]
    print("replay buffer composition (shares of its transitions; success terminals = goal "
          "rewards in it):")
    for b in buffers:
        shares = ", ".join(f"{k} {v:.3f}" for k, v in b["shares"].items())
        print(f"  at {b['at']:>8}: {shares}; unfinished {b['unfinished']:.3f}; "
              f"success terminals {b['success_terminals']}")

    out_dir = create_run_dir(REPO_ROOT / "runs" / "diag",
                             f"DECLINE_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}", args.seed, cfg)
    (out_dir / "summary.json").write_text(json.dumps({
        "run": args.run, "stage_switches": switches, "success_per_100k": bins,
        "per_stage_stratum": per_stratum, "full_stage_unmapped": unmapped,
        "buffer": buffers}, indent=2), encoding="utf-8")
    print(f"-> {out_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
