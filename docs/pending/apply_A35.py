"""Apply amendment A35 to preregistration/ (the RESEARCHER runs this; hard rule 1), after A34.

    python docs/pending/apply_A35.py

Appends the A35 section of docs/pending/A35_amendment.md to preregistration/AMENDMENTS.md and makes
its edits in endpoints.yaml and gates.yaml, in UTF-8. Refuses unless A34 is applied and A35 is not,
and unless every text it replaces occurs the stated number of times; on any mismatch nothing is
written. Then: commit the three files, and `python scripts/freeze_prereg.py --relock A35`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A35_amendment.md"

ENDPOINTS = [   # (old, new, occurrences)
    ("  endpoint: protective_stops_per_100m\n",
     ("  endpoint: protective_stops_per_100m_planned   # A35 (2026-11-07): per 100 m of PLANNED route "
     "length; driven distance is secondary\n"), 2),
    (("  H1_estimand: rate ratio of protective stops, total stops / total distance per arm, L5 over "
     "the comparator; scan-fault stops excluded (A25)\n"),
     ("  H1_estimand: rate ratio of protective stops, total stops / total PLANNED route length per arm "
     "(A35), L5 over the comparator; scan-fault stops excluded (A25)\n"), 1),
    (("    - the stops endpoint's denominator -- per 100 m of distance driven (the primary's own, "
     "recovery included; timeouts pooled, not censored) and per 100 m on completed routes only, "
     "beside the per-planned-length version (A14)\n"),
     ("    - the stops endpoint's denominator -- per 100 m of distance driven (recovery included; "
     "timeouts pooled, not censored; secondary since A35) and per 100 m on completed routes only, "
     "beside the primary per 100 m of planned route (A35)\n"), 1),
]
GATES = [
    ("  reduction_order: [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1]  # A28, confirmed A28a\n",
     ("  reduction_order: [L5-v2, L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1]  # A28, confirmed "
     "A28a; L5-v2 at the head (A35)\n"), 1),
    ("    pilot: [9000000, 9999999]\n",
     ("    pilot: [9000000, 9999999]\n"
     "    # A35 (2026-11-07): 9100000-9100999 is carved out -- CONFIRMATORY training seeds\n"
     "    # (G4_learned_pilot.confirmatory_training_seeds); no diagnostic, pilot or layout draws from it.\n"),
     1),
    ("  integrity:                                 # every one must hold\n",
     ("  pilot_training_range: [9000000, 9000099]   # A35: pilot TRAINING seeds come from here only\n"
     "  confirmatory_training_seeds: [9100001, 9100002, 9100003, 9100004, 9100005, 9100006, 9100007,\n"
     "                                9100008, 9100009, 9100010]   # A35: K = 10, fixed, never changed\n"
     "  integrity:                                 # every one must hold\n"), 1),
]


def section() -> str:
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A35 - "):
        raise SystemExit("the draft's A35 section was not found between its `---` lines")
    return parts[1].strip("\n") + "\n"


def _apply(name: str, text: str, edits) -> str:
    for old, new, n in edits:
        if text.count(old) != n:
            raise SystemExit(f"{name}: a text to replace occurs {text.count(old)} times, not {n} "
                             f"({old.strip()[:60]!r}); nothing written")
        text = text.replace(old, new)
    return text


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A34 - " not in amendments:
        raise SystemExit("A34 is not applied yet; nothing written")
    if "## A35 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A35; nothing written")
    endpoints = _apply("endpoints.yaml", (PREREG / "endpoints.yaml").read_text(encoding="utf-8"),
                       ENDPOINTS)
    gates = _apply("gates.yaml", (PREREG / "gates.yaml").read_text(encoding="utf-8"), GATES)
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + section())
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints)
    with open(PREREG / "gates.yaml", "w", encoding="utf-8") as f:
        f.write(gates)
    print("A35 applied: AMENDMENTS.md, endpoints.yaml, gates.yaml. Commit them, then "
          "`python scripts/freeze_prereg.py --relock A35`.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
