"""Apply amendment A34 to preregistration/ (the RESEARCHER runs this; hard rule 1), after A32.

    python docs/pending/apply_A34.py

Appends the A34 section of docs/pending/A34_amendment.md to preregistration/AMENDMENTS.md and makes
its edits in endpoints.yaml (block H1prime), in UTF-8. Refuses unless A32 is applied and A34 is not,
and unless every text it replaces occurs exactly once; on any mismatch nothing is written. Then:
commit the two files, and `python scripts/freeze_prereg.py --relock A34`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A34_amendment.md"

STATEMENT = "  statement: B7 triggers fewer protective stops than B1, B2 and B6\n"
REPORTING = "  reporting: success rate and mission time beside it for every comparison in the same table\n"
EDITS = [
    (STATEMENT,
     STATEMENT + ("  claim: the filter removes protective stops from a classical planner -- not a "
                  "throughput or success improvement (A34)\n")),
    (REPORTING,
     ("  reporting: success rate and mission time reported descriptively beside it for every "
      "comparison in the same table; no inference on them (A34)\n"
      "  secondary:                       # A34 (ruled 2026-11-06), declared before the test run; "
      "outside the confirmatory family\n"
      "    - B7 vs B2, the paired within-planner contrast -- protective stops per 100 m, latched "
      "share of steps, moving speed (mean speed while not latched)\n"
      "    - the stops endpoint's denominator -- per 100 m of distance driven (the primary's own, "
      "recovery included; timeouts pooled, not censored) and per 100 m on completed routes only, "
      "beside the per-planned-length version (A14)\n")),
]


def section() -> str:
    """The amendment text between the draft's two `---` rules."""
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A34 - "):
        raise SystemExit("the draft's A34 section was not found between its `---` lines")
    return parts[1].strip("\n") + "\n"


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A32 - " not in amendments:
        raise SystemExit("A32 is not applied yet; nothing written")
    if "## A34 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A34; nothing written")
    endpoints = (PREREG / "endpoints.yaml").read_text(encoding="utf-8")
    for old, new in EDITS:
        if endpoints.count(old) != 1:
            raise SystemExit(f"endpoints.yaml: a text to replace occurs {endpoints.count(old)} "
                             f"times, not once ({old.strip()[:60]!r}); nothing written")
        endpoints = endpoints.replace(old, new)
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + section())
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints)
    print("A34 applied: AMENDMENTS.md, endpoints.yaml. Commit them, then "
          "`python scripts/freeze_prereg.py --relock A34`.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
