"""Build the validation set: fresh scenarios for comparing baselines with and without the PFB.

    python scripts/build_validation_set.py --config configs/base.yaml --seed 0

Writes six sets under `scenarios/validation/` (narrow, medium and wide, each mapped-only and with unmapped
obstacles), `--count` scenarios each (default 30), from seeds 2,500,000 up. These seeds were never used for
tuning a baseline or choosing alpha, and are disjoint from every other set (checked). Offline, no
simulator; the researcher runs it because it can take more than five minutes with the validity checks.

It refuses seeds outside the tune range, overlapping seeds, and an output under `scenarios/test/`. Each
set is checked with `verify()` before it is left on disk.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.scenarios import build_set, verify
from pfbnav.world.validation_set import (
    CELLS,
    FIRST_SEED,
    TUNE_RANGE,
    cell_name,
    cell_seeds,
    overlaps,
    truncate,
    used_seeds,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = REPO_ROOT / "scenarios"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--count", type=int, default=30, help="scenarios kept per cell")
    parser.add_argument("--headroom", type=int, default=15,
                        help="extra seeds asked for per cell, since some seeds build no scenario")
    parser.add_argument("--workers", type=int, default=4, help="scenario-building processes")
    parser.add_argument("--first-seed", type=int, default=FIRST_SEED)
    parser.add_argument("--out-dir", default="scenarios/validation")
    args = parser.parse_args()

    cfg = load_config(args.config)
    out_dir = (REPO_ROOT / args.out_dir).resolve()
    if (SCENARIOS / "test") in (out_dir, *out_dir.parents):
        raise SystemExit("never write under scenarios/test/")
    used = used_seeds(SCENARIOS)
    plan = [(i, *cell) for i, cell in enumerate(CELLS)]
    for i, *_ in plan:
        seeds = cell_seeds(i, args.count, args.headroom, args.first_seed)
        if not (TUNE_RANGE[0] <= seeds[0] and seeds[-1] <= TUNE_RANGE[1]):
            raise SystemExit(f"seeds {seeds[0]}-{seeds[-1]} are outside the tune range {TUNE_RANGE}")
        clash = overlaps(seeds, used)
        if clash:
            raise SystemExit(f"seeds already used by another set: {dict(list(clash.items())[:5])}")
    seed_everything(args.seed)
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, stratum, condition, rate in plan:
        seeds = cell_seeds(i, args.count, args.headroom, args.first_seed)
        print(f"-- {cell_name(stratum, condition)}: seeds {seeds[0]}-{seeds[-1]}, rate {rate}", flush=True)
        manifest = build_set(seeds, cfg, "tune", episode_rate=rate, stratum=stratum,
                             workers=args.workers)
        manifest = truncate(manifest, args.count)
        checked = verify(manifest, cfg)
        if not checked["ok"]:
            raise SystemExit(f"{cell_name(stratum, condition)} fails verification: "
                             f"{checked['problems'][:3]}")
        path = out_dir / f"{cell_name(stratum, condition)}.json"
        path.write_text(json.dumps(manifest, indent=2, default=float), encoding="utf-8")
        lengths = [s["path_length_m"] for s in manifest["scenarios"]]
        print(f"   kept {manifest['built']}; unmapped share {manifest['episode_rate_achieved']:.2f}; "
              f"routes {min(lengths):.1f}-{max(lengths):.1f} m (mean {sum(lengths) / len(lengths):.1f}); "
              f"verify ok -> {path.relative_to(REPO_ROOT).as_posix()}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
