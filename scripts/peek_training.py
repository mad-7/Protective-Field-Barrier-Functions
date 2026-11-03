"""A read-only look at a training run's episodes: outcomes per curriculum stage, success in ten
consecutive blocks of episodes, and G4's learning criterion as `gate_g4.py` will read it. Seconds.

    python scripts/peek_training.py --run runs/train/L1_sac_shaped_seed9000001

Reads `episodes.jsonl` only. A monitor, not the gate: the verdict is `gate_g4.py`'s, which also
needs the checkpoint evaluation and the controls. Writes nothing.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any, Sequence

import yaml

from pfbnav.eval.g4 import learning_criterion

REPO_ROOT = Path(__file__).resolve().parents[1]


def success_by_block(episodes: Sequence[dict[str, Any]], blocks: int) -> list[dict[str, Any]]:
    """Episodes in episode order, cut into `blocks` consecutive parts (the last takes the
    remainder); per block its episode range, size and success share."""
    ordered = sorted(episodes, key=lambda e: e["episode"])
    n = len(ordered)
    size = n // blocks if blocks else 0
    out = []
    for b in range(blocks):
        part = ordered[b * size: n if b == blocks - 1 else (b + 1) * size]
        if part:
            out.append({"first": part[0]["episode"], "last": part[-1]["episode"], "n": len(part),
                        "success": sum(e["outcome"] == "success" for e in part) / len(part)})
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, help="a runs/train/ directory")
    args = parser.parse_args()
    run = REPO_ROOT / args.run
    episodes = [json.loads(line) for line in
                (run / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if line]
    print(f"{len(episodes)} episodes")
    stages = sorted({e.get("curriculum_stage") for e in episodes}, key=str)
    for stage in stages:
        mine = [e for e in episodes if e.get("curriculum_stage") == stage]
        counts = Counter(e["outcome"] for e in mine)
        print(f"  stage {stage}: {len(mine)} episodes, success {counts['success'] / len(mine):.3f}, "
              f"{dict(counts)}")
    print("success in ten consecutive blocks of episodes:")
    for b in success_by_block(episodes, 10):
        print(f"  episodes {b['first']}-{b['last']} ({b['n']}): {b['success']:.3f}")
    g4 = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml")
                        .read_text(encoding="utf-8"))["G4_learned_pilot"]["learning"]
    print("G4 learning:", learning_criterion(
        episodes, window_fraction=float(g4["window_fraction"]),
        min_last_success=float(g4["min_last_success"]), stage=g4.get("stage")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
