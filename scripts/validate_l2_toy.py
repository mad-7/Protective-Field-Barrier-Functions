"""L2's toy positive control, run to completion, for one declared PID candidate (A28b).

The fast suite runs the toy (`pfbnav/training/l2_toy.py`) for 1,000 steps on seed 0 with candidate 1.
This runs one seed and one candidate (`pfbnav.control.sac_lagrangian.PID_CANDIDATES`) for `--steps`
(default 5,000).

**Criteria, unchanged** (declared 2026-10-26; the A28a dual-ascent update FAILED them on seeds 1 and 3,
2026-10-27): over the last 20 episodes, SAC-Lagrangian's mean stops per episode are at most 1.25 d and
at most half of plain SAC's, plain SAC's are at least 2 d (the constraint binds), and the multiplier
stays within [0, lambda_max]. A candidate PASSES if all four hold on each of seeds 0-3. **Selection
(A28b):** the first candidate, in declared order (1, 2, 3), that passes is L2's; if none passes, stop
and report. Also reported, descriptively: the fraction of 20-episode windows whose mean meets d (and
1.25 d), over every full window and over those ending at episode 250 or later.

The researcher runs it (RL training beyond 1,000 steps, CLAUDE.md). No simulator; CPU, about 3-4 min
per seed on a laptop. Writes runs/validation/L2_TOY_PID<candidate>_<stamp>_seed<seed>/summary.json.

    python scripts/validate_l2_toy.py --config configs/base.yaml --seed 0 --candidate 1
"""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pfbnav.config import load_config
from pfbnav.control.sac import make_sac
from pfbnav.control.sac_lagrangian import PID_CANDIDATES, make_sac_lagrangian
from pfbnav.training.l2_toy import D, StopToy, T, mean_action, toy_cfg
from pfbnav.utils.runio import create_run_dir

REPO_ROOT = Path(__file__).resolve().parents[1]
LAST = 20
LATE = 250             # first episode of the "late" windows (the A28a cycle had settled into its
                       # period by then)


def verdict(plain_costs: list[int], lag_costs: list[int], multipliers: list[float],
            cap: float) -> dict:
    """The four declared criteria on one seed's episode costs and multiplier trace."""
    plain = float(np.mean(plain_costs[-LAST:]))
    lag = float(np.mean(lag_costs[-LAST:]))
    checks = {"binds": plain >= 2.0 * D, "meets_limit": lag <= 1.25 * D,
              "halves_plain": lag <= 0.5 * plain,
              "multiplier_in_range": all(0.0 <= m <= cap for m in multipliers)}
    return {"plain_last": plain, "lagrangian_last": lag, "checks": checks,
            "verdict": "PASS" if all(checks.values()) else "FAIL"}


def window_fractions(costs: list[int], limit: float, window: int = LAST,
                     late: int = LATE) -> dict:
    """Descriptive (A28b): the fraction of `window`-episode moving means at or below `limit` and
    1.25 `limit`, over every full window and over the windows ending at episode index `late` or
    later (0-based). None where there is no such window."""
    c = np.asarray(costs, dtype=float)
    if len(c) < window:
        return {"all": None, "all_125": None, "late": None, "late_125": None, "windows": 0}
    means = np.convolve(c, np.ones(window) / window, mode="valid")   # means[i] ends at i+window-1
    tail = means[max(0, late - window + 1):]

    def frac(m, bound):
        return float(np.mean(m <= bound + 1e-12)) if len(m) else None
    return {"all": frac(means, limit), "all_125": frac(means, 1.25 * limit),
            "late": frac(tail, limit), "late_125": frac(tail, 1.25 * limit),
            "windows": len(means)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="recorded for provenance")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--candidate", type=int, required=True, choices=sorted(PID_CANDIDATES),
                        help="the declared PID candidate (A28b), in order 1, 2, 3")
    parser.add_argument("--steps", type=int, default=5000)
    args = parser.parse_args()
    if args.steps <= 1000:
        raise SystemExit("--steps above 1,000: the fast suite already covers 1,000")
    cfg = load_config(args.config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    toy = toy_cfg(args.candidate)
    run_dir = create_run_dir(REPO_ROOT / "runs" / "validation",
                             f"L2_TOY_PID{args.candidate}_{stamp}", args.seed,
                             {**cfg, "l2_toy": toy})
    start = time.time()
    plain_env, lag_env = StopToy(), StopToy()
    plain = make_sac(plain_env, toy, seed=args.seed)
    plain.learn(args.steps)
    lag = make_sac_lagrangian(lag_env, toy, seed=args.seed, ruled=False)
    lag.learn(args.steps)
    multipliers = [e["multiplier"] for e in lag.dual_history] + [lag.lagrange_multiplier]
    out = {"seed": args.seed, "steps": args.steps, "episode_steps": T, "cost_limit": D,
           "candidate": args.candidate, "pid_gains": list(PID_CANDIDATES[args.candidate]),
           "settings": toy["lagrangian"],
           **verdict(plain_env.episode_costs, lag_env.episode_costs, multipliers,
                     float(toy["lagrangian"]["multiplier_max"])),
           "window_fractions": window_fractions(lag_env.episode_costs, D),
           "final_multiplier": lag.lagrange_multiplier,
           "mean_action": {"plain": mean_action(plain), "lagrangian": mean_action(lag)},
           "episode_costs": {"plain": plain_env.episode_costs, "lagrangian": lag_env.episode_costs},
           "multiplier_per_episode": multipliers[:-1], "wall_s": time.time() - start}
    (run_dir / "summary.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    fr = out["window_fractions"]
    print(f"{out['verdict']} (candidate {args.candidate}, seed {args.seed}): L2 "
          f"{out['lagrangian_last']:.2f} stops/episode (d = {D}), plain {out['plain_last']:.2f}, "
          f"lambda {lag.lagrange_multiplier:.3f}; windows meeting d {fr['all']:.2f} "
          f"(from episode {LATE}: {fr['late']:.2f})")
    print(f"-> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
