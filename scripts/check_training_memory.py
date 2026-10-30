"""The training memory check (ruled 2026-10-30). The RESEARCHER runs it (~1 h on the training host,
one instance, the GPU): no training run of 1M steps or more until it passes.

    python scripts/check_training_memory.py --config configs/experiments/L1_sac_shaped.yaml --seed 9200000

L1's first pilot grew the training process 1,971 -> 3,562 MB over 300k steps, inconclusive between
one-off costs (CUDA's context, measured +528 MB on the host; the first update's cuBLAS/cuDNN
workspaces) and a leak. This trains exactly as `scripts/train.py` does -- the same `run()`, the
stall truncation, the learner on `training.device` -- for `training.memory_check.total_timesteps`
(100k), sampling both processes' private memory every `training.memory_sample_every` steps.

**The criterion, declared before running** (`training.memory_check`, `pfbnav.utils.memory.levels_off`):
"growth beyond the CUDA start-up cost levels off within the first 10 % of the run". The baseline is
the first sample at or after `settled_from` (3,000: after `learning_starts` and the first thousand
updates); the mark is 10 % of the run (10k steps); PASS when the training process grows at most
`max_growth_mb` (100 MB) from the mark to the end, and the samples reach the end. One column, one
check. Under the null (no leak) the expected false-alarm rate is ~0: allocator fluctuations are
tens of MB, against a 100 MB bound; the slope after the mark and the growth before it are reported
beside the verdict so a near miss is visible. The simulator's growth is reported against the A20
reference (+1.7 MB / 100 episodes), not gated.

**Positive control (hard rule 15):** before training, 256 MiB is allocated and written in this
process; the sampler must see 230-290 MB of growth, then the block is freed. A sampler that reads
the wrong process, or the wrong counter, fails here and the check stops before training.

**Seeds.** A diagnostic, never a pilot: the seed must be in the pilot range and must NOT be one of
G4's `pilot_training_seeds`, so the check never previews a pilot seed's run. 9200000 is suggested.

Writes `runs/diag/MEMTRAIN_<timestamp>_seed<seed>/`: everything `train.py` writes (episodes,
memory_trend.jsonl, checkpoints, summary.json) plus `memory_check.json` with the verdict.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from pfbnav.config import load_config
from pfbnav.utils.cli import add_display_flag, apply_display
from pfbnav.utils.memory import MemoryRecord, levels_off
from pfbnav.utils.runio import create_run_dir
from pfbnav.utils.seeding import seed_everything

REPO_ROOT = Path(__file__).resolve().parents[1]
CONTROL_MIB = 256
CONTROL_RANGE_MB = (230.0, 290.0)


def _train_module():
    spec = importlib.util.spec_from_file_location("train", REPO_ROOT / "scripts" / "train.py")
    if spec is None or spec.loader is None:
        raise SystemExit("cannot load scripts/train.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def seed_problem(seed: int, gates: dict) -> str | None:
    """Why `seed` may not be used for the memory check, or None."""
    lo, hi = gates["layout_sets"]["seed_ranges"]["pilot"]
    if not lo <= seed <= hi:
        return f"seed {seed} is outside the pilot range [{lo}, {hi}]: diagnostics draw from it"
    if seed in gates["G4_learned_pilot"]["pilot_training_seeds"]:
        return f"seed {seed} is a G4 pilot training seed: the check never previews a pilot"
    return None


def positive_control() -> dict:
    """Allocate and write CONTROL_MIB MiB; the private-memory reading must move by about that."""
    probe = MemoryRecord(port=None)
    before = probe.snapshot()["python"]
    block = bytearray(CONTROL_MIB * 2 ** 20)
    block[::4096] = b"\x01" * len(block[::4096])                  # touch every page
    after = probe.snapshot()["python"]
    del block
    if before is None or after is None:
        return {"passed": False, "reason": f"memory not measurable: {probe.status}"}
    grew = after - before
    passed = CONTROL_RANGE_MB[0] <= grew <= CONTROL_RANGE_MB[1]
    return {"passed": passed, "allocated_mib": CONTROL_MIB, "measured_growth_mb": grew,
            "accepted_range_mb": list(CONTROL_RANGE_MB)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--instance", type=int, default=0)
    add_display_flag(parser)
    args = parser.parse_args()
    cfg = apply_display(load_config(args.config), args.display)
    gates = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml").read_text(encoding="utf-8"))
    problem = seed_problem(args.seed, gates)
    if problem:
        raise SystemExit(problem)
    check = cfg["training"]["memory_check"]
    control = positive_control()
    print(f"positive control: {control}", flush=True)
    if not control["passed"]:
        raise SystemExit("the positive control failed: the sampler does not measure this process")
    train = _train_module()
    seed_everything(args.seed)
    sets = train.load_training_sets(cfg, args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = create_run_dir(REPO_ROOT / "runs" / "diag", f"MEMTRAIN_{stamp}", args.seed, cfg)
    total = int(check["total_timesteps"])
    summary = train.run(cfg, args.seed, run_dir, total, sets, args.instance)
    samples = [json.loads(line) for line in
               (run_dir / "memory_trend.jsonl").read_text(encoding="utf-8").splitlines() if line]
    verdict = levels_off(samples, total_timesteps=total, settled_from=int(check["settled_from"]),
                         mark_fraction=float(check["mark_fraction"]),
                         max_growth_mb=float(check["max_growth_mb"]))
    sims = [s for s in samples if s.get("sim") is not None]
    out = {"criterion": dict(check), "positive_control": control, "verdict": verdict,
           "sim": {"first_mb": sims[0]["sim"] if sims else None,
                   "last_mb": sims[-1]["sim"] if sims else None,
                   "episodes": summary["episodes"],
                   "reference": "A20: CoppeliaSim +1.7 MB/100 episodes (reported, not gated)"},
           "training_summary": {k: summary[k] for k in ("timesteps", "episodes", "outcomes",
                                                        "stall_truncation", "completed", "error")}}
    (run_dir / "memory_check.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    word = "PASS" if verdict["passed"] else "FAIL"
    print(f"{word}: {verdict['reason']} -> {run_dir.relative_to(REPO_ROOT).as_posix()}")
    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
