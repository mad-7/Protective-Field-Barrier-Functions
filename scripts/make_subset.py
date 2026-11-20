"""Cut a few scenarios out of a built set, for watching one route at a time while tuning by hand.

    python scripts/make_subset.py scenarios/tune/tune_narrow.json --indices 3 7 --out runs/scratch/narrow_3_7.json

Or keep every scenario without unmapped obstacles (the "mapped obstacles only" rung of the baseline ladder):

    python scripts/make_subset.py scenarios/tune/tune_wide.json --mapped-only --out runs/scratch/wide_mapped.json

then `python scripts/evaluate.py ... --sets runs/scratch/narrow_3_7.json --draw-path`. The output is the
same manifest with only those scenarios, so `verify()` still checks each one against the generator. Test
sets are refused (hard rule 2: test layouts are never used for debugging). Writes nothing under
`scenarios/`.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def subset(manifest: dict, indices: list[int]) -> dict:
    """The manifest with only `indices` (positions in its scenario list); raises on a bad index."""
    scenarios = manifest["scenarios"]
    bad = [i for i in indices if not 0 <= i < len(scenarios)]
    if bad:
        raise ValueError(f"indices {bad} are outside 0..{len(scenarios) - 1}")
    out = dict(manifest)
    out["scenarios"] = [scenarios[i] for i in indices]
    out["built"] = out["requested"] = len(indices)
    out["failures"] = []
    out["locked"] = False
    return out


def mapped_only_indices(manifest: dict) -> list[int]:
    """Positions of the scenarios with no unmapped obstacle."""
    return [i for i, sc in enumerate(manifest["scenarios"]) if int(sc.get("unmapped_count", 0)) == 0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--indices", type=int, nargs="+")
    group.add_argument("--mapped-only", action="store_true",
                       help="every scenario whose obstacles are all in the planner's map")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    src = (REPO_ROOT / args.source).resolve()
    if (REPO_ROOT / "scenarios" / "test") in src.parents:
        raise SystemExit("test layouts are never used for debugging (hard rule 2)")
    out = (REPO_ROOT / args.out).resolve()
    if (REPO_ROOT / "scenarios") in out.parents:
        raise SystemExit("write the subset outside scenarios/ (for example runs/scratch/)")
    manifest = json.loads(src.read_text(encoding="utf-8"))
    out.parent.mkdir(parents=True, exist_ok=True)
    indices = mapped_only_indices(manifest) if args.mapped_only else args.indices
    out.write_text(json.dumps(subset(manifest, indices), indent=2), encoding="utf-8")
    print(f"{len(indices)} scenarios -> {out.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
