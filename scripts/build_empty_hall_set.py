"""Build the empty-hall scenario set (perimeter walls, no shelves): the first rung of the baseline ladder.

    python scripts/build_empty_hall_set.py --config configs/worlds/empty_hall.yaml --seed 0

Writes `scenarios/free/empty_hall.json` (tune-range seeds, 2900000 up: development, never a test set) and
checks it with `verify()` before leaving it. Offline, no simulator, about a minute. Evaluate it with the
same config: `scripts/evaluate.py --config configs/worlds/empty_hall.yaml --sets scenarios/free/empty_hall.json`.

It refuses a config with shelf rows (use configs/worlds/empty_hall.yaml) and a seed range outside the tune
range (hard rule 2: test layouts are never used for development).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.utils.seeding import seed_everything
from pfbnav.world.free_hall import build_free_set
from pfbnav.world.scenarios import verify

REPO_ROOT = Path(__file__).resolve().parents[1]
TUNE_RANGE = (2_000_000, 2_999_999)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--first-seed", type=int, default=2_900_000,
                        help="first layout seed of the set (a tune-range seed)")
    parser.add_argument("--count", type=int, default=20, help="scenarios to build")
    parser.add_argument("--out", default="scenarios/free/empty_hall.json")
    args = parser.parse_args()
    cfg = load_config(args.config)
    if tuple(int(v) for v in cfg["world"]["layout"]["rows"]) != (0, 0) or "free_hall" not in cfg["world"]:
        raise SystemExit("this builds an empty hall: use configs/worlds/empty_hall.yaml "
                         "(world.layout.rows [0, 0] and world.free_hall)")
    seeds = range(args.first_seed, args.first_seed + args.count)
    if not (TUNE_RANGE[0] <= seeds[0] and seeds[-1] <= TUNE_RANGE[1]):
        raise SystemExit(f"seeds {seeds[0]}-{seeds[-1]} are outside the tune range {TUNE_RANGE}")
    seed_everything(args.seed)
    manifest = build_free_set(seeds, cfg, split="tune")
    checked = verify(manifest, cfg)
    if not checked["ok"]:
        raise SystemExit(f"the set fails verification right after building: {checked['problems'][:3]}")
    out = (REPO_ROOT / args.out).resolve()
    if (REPO_ROOT / "scenarios" / "test") in out.parents:
        raise SystemExit("never write under scenarios/test/")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(manifest, indent=2, default=float), encoding="utf-8")
    lengths = [s["path_length_m"] for s in manifest["scenarios"]]
    print(f"built {manifest['built']} of {manifest['requested']}; failures {manifest['failures']}; "
          f"routes {min(lengths):.1f}-{max(lengths):.1f} m (mean {sum(lengths) / len(lengths):.1f}); "
          f"verify ok -> {out.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
