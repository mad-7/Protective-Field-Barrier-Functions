"""Freeze the preregistration: record what it says, and what the layout sets are made by.

Run by the RESEARCHER, once, after the confirmatory hypotheses, the endpoints and the gate
criteria are all committed (hard rule 1: the preregistration is never written by Claude). Then
commit `preregistration/LOCK.json` on its own.

    python scripts/freeze_prereg.py
    python scripts/freeze_prereg.py --relock A13     (after an amendment; see below)

What the lock records, and why each is there:

  * **Every file in `preregistration/`**, hashed as text with line endings normalised
    (`pfbnav.utils.hashing`). Raw-byte hashes differ between this laptop's CRLF checkout and the
    training host's LF one, so a lock written here would have failed its own test there.
  * **Paths in POSIX form.** `Path.relative_to` gives backslashes on Windows, and on Linux
    `preregistration\\gates.yaml` is a single filename, not a path.
  * **`generator_sha256` and `scenarios_sha256`** -- ruled 2026-09-26 (Q2): the generator's code
    is part of a layout set's identity, so the lock pins it, and a changed generator after the
    freeze fails the suite until an amendment says why.
  * **The commit and a clean tree.** A lock of files that differ from any commit could not be
    reproduced, so a dirty tree is refused.

**Re-locking after an amendment.** `--relock A13` replaces an existing lock, and only if
`AMENDMENTS.md` carries a heading for that amendment -- so the lock cannot move without the record
that says why. The replaced lock is kept inside the new one under `supersedes`, so the history of
what was frozen when is never lost.

It refuses to freeze over a placeholder (`TO_BE_COPIED_FROM_THE_PLAN`): locking one would
preregister the absence of something rather than the thing.

Test sets are NOT in this lock. They do not exist at the freeze (the order ruled 2026-09-28 is
freeze, then G2, then generate the sets) and each is locked by `write_set(..., lock=True)`,
carrying its own identity triple, which `verify()` checks against the hashes pinned here.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from pfbnav.utils.hashing import text_sha256
from pfbnav.world.layouts import generator_sha256
from pfbnav.world.scenarios import scenarios_sha256

ROOT = Path(__file__).resolve().parents[1]
PREREG = ROOT / "preregistration"
LOCK = PREREG / "LOCK.json"
PLACEHOLDER = "TO_BE_COPIED_FROM_THE_PLAN"


def git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True,
                          check=True).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--relock", metavar="AMENDMENT", default=None,
                        help="replace the existing lock, citing the amendment that changed it")
    args = parser.parse_args()
    previous = None
    if LOCK.exists():
        if not args.relock:
            raise SystemExit("Already frozen. Changes need an amendment in AMENDMENTS.md and a "
                             "dated entry in docs/decision_log.md, then --relock <amendment>.")
        amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
        if not re.search(rf"^## {re.escape(args.relock)} - ", amendments, flags=re.MULTILINE):
            raise SystemExit(f"AMENDMENTS.md has no '## {args.relock} - ' heading; a lock may "
                             "only move with the amendment that explains it.")
        previous = json.loads(LOCK.read_text(encoding="utf-8"))
    elif args.relock:
        raise SystemExit("Nothing to re-lock: the preregistration has never been frozen.")
    if git("status", "--porcelain"):
        raise SystemExit("The working tree is not clean; commit first, so the lock matches a "
                         "commit that can be checked out again.")
    files = [p for p in sorted(PREREG.iterdir()) if p.is_file() and p.name != LOCK.name]
    required = {"gates.yaml", "endpoints.yaml", "AMENDMENTS.md"}
    missing = required - {p.name for p in files}
    if missing:
        raise SystemExit(f"cannot freeze without {sorted(missing)}")
    for p in files:
        if PLACEHOLDER in p.read_text(encoding="utf-8"):
            raise SystemExit(f"{p.name} still contains {PLACEHOLDER}; copy the plan in first")

    lock = {
        "frozen_utc": datetime.now(timezone.utc).isoformat(),
        "commit": git("rev-parse", "HEAD"),
        "hash": "sha256 of the text with line endings normalised to LF",
        "sha256": {p.relative_to(ROOT).as_posix(): text_sha256(p) for p in files},
        "generator_sha256": generator_sha256(),
        "scenarios_sha256": scenarios_sha256(),
    }
    if previous is not None:
        lock["relocked_for"] = args.relock
        # The whole chain, newest first: nothing that was once frozen is forgotten.
        lock["supersedes"] = previous
    LOCK.write_text(json.dumps(lock, indent=2) + "\n", encoding="utf-8")
    print(f"Frozen {len(files)} files at {lock['commit'][:10]} -> "
          f"{LOCK.relative_to(ROOT).as_posix()}")
    print(f"generator_sha256 {lock['generator_sha256'][:12]}, "
          f"scenarios_sha256 {lock['scenarios_sha256'][:12]}")
    print("Now: git add preregistration/LOCK.json && git commit, on its own.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
