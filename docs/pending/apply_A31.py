"""Apply amendment A31 to preregistration/ (the RESEARCHER runs this; hard rule 1).

    python docs/pending/apply_A31.py

Appends the A31 section of docs/pending/A31_amendment.md to preregistration/AMENDMENTS.md and makes
its two edits in endpoints.yaml and gates.yaml, in UTF-8. Every edit checks that the text it replaces
is there exactly once and that A31 is not applied yet; on any mismatch nothing is written. Then:
commit the three files, and `python scripts/freeze_prereg.py --relock A31`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A31_amendment.md"

ENDPOINTS_OLD = "    trigger: L1's or L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot)\n"
ENDPOINTS_NEW = ("    trigger: L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot); L1's G4 "
                 "outcome does not trigger it (A31)\n")
GATES_OLD = "      for the seed-CV discussion; if it fails, stop and report\n"
GATES_NEW = ("      for the seed-CV discussion; if it fails, stop and report. A31 (2026-11-03): for L5 a\n"
             "      second failure triggers the pivot (endpoints.yaml); for L1 a second failure is\n"
             "      disclosed and L1 still trains its confirmatory seeds unconditionally as a comparator.\n")


def section() -> str:
    """The amendment text between the draft's two `---` rules."""
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A31 - "):
        raise SystemExit("the draft's A31 section was not found between its two `---` lines")
    return parts[1].strip("\n") + "\n"


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    endpoints = (PREREG / "endpoints.yaml").read_text(encoding="utf-8")
    gates = (PREREG / "gates.yaml").read_text(encoding="utf-8")
    if "## A31 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A31; nothing written")
    for name, text, old in (("endpoints.yaml", endpoints, ENDPOINTS_OLD),
                            ("gates.yaml", gates, GATES_OLD)):
        if text.count(old) != 1:
            raise SystemExit(f"{name}: the text to replace occurs {text.count(old)} times, not once; "
                             "nothing written")
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + section())
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints.replace(ENDPOINTS_OLD, ENDPOINTS_NEW))
    with open(PREREG / "gates.yaml", "w", encoding="utf-8") as f:
        f.write(gates.replace(GATES_OLD, GATES_NEW))
    print("A31 applied: AMENDMENTS.md, endpoints.yaml, gates.yaml. Commit them, then "
          "`python scripts/freeze_prereg.py --relock A31`.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
