"""Cut a few scenarios out of a built set, for watching one route at a time while tuning by hand.

    python scripts/make_subset.py scenarios/tune/tune_narrow.json --indices 3 7 --out runs/scratch/narrow_3_7.json

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("--indices", type=int, nargs="+", required=True)
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
    out.write_text(json.dumps(subset(manifest, args.indices), indent=2), encoding="utf-8")
    print(f"{len(args.indices)} scenarios -> {out.relative_to(REPO_ROOT).as_posix()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
