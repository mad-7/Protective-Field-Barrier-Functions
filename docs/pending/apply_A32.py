"""Apply amendment A32 to preregistration/ (the RESEARCHER runs this; hard rule 1), after A31.

    python docs/pending/apply_A32.py

Appends the A32 section of docs/pending/A32_amendment.md to preregistration/AMENDMENTS.md and makes
its edits in gates.yaml and endpoints.yaml, in UTF-8. Refuses unless A31 is applied and A32 is not, and
unless every text it replaces occurs exactly once; on any mismatch nothing is written. Then: commit the
three files, and `python scripts/freeze_prereg.py --relock A32`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A32_amendment.md"
PAD = " " * 45 + "#   "

GATES = [
    ("  pilot_training_seeds: [9000000, 9000001, 9000002]   # from the pilot range only. Pilot seeds\n",
     "  pilot_training_seeds: [9000000, 9000001, 9000002, 9000003]   # from the pilot range only. Pilot seeds\n"),
    (PAD + "9000002. The pivot counts the restarted pilot only.\n",
     PAD + "9000002. The pivot counts the restarted pilot only.\n"
     + PAD + "A32 (2026-11-03): 9000003 added. L5's first pilot\n"
     + PAD + "(9000001) crashed on a PFB defect: an integrity\n"
     + PAD + "failure, kept, not counted; L5 runs at 9000002,\n"
     + PAD + "its second seed 9000003.\n"),
    ("    integrity: fix; a fresh pilot seed; the failed run is kept\n",
     ("    integrity: fix; a fresh pilot seed; the failed run is kept; an integrity failure is not\n"
     "      counted toward the pivot (A32)\n")),
    ("      disclosed and L1 still trains its confirmatory seeds unconditionally as a comparator.\n",
     ("      disclosed and L1 still trains its confirmatory seeds unconditionally as a comparator.\n"
     "      A32 (2026-11-03): the same for L2 and L4; only L5 triggers the pivot. An exploratory arm\n"
     "      (A1, L5-nowp, L2-d005, L2-d020) that fails G4 twice is dropped and its failure reported.\n")),
]
L2_SENS = ("  L2_sensitivity: L2 at d = 0.05 and d = 0.2 stops per episode (L2-d005, L2-d020) -- "
           "protective stops, success and mission time beside L2 at d = 0.1; outside the Holm "
           "family; first in the cut order (A28)\n")
ENDPOINTS = [
    (("    trigger: L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot); L1's G4 outcome "
     "does not trigger it (A31)\n"),
     ("    trigger: L5's pilot fails G4 twice (both pilot seeds of G4_learned_pilot); no other arm's G4 "
     "outcome triggers it (A31, A32)\n")),
    (L2_SENS,
     L2_SENS + "  B7_alpha_sensitivity: B7 at alpha 1, 2 and 5 on the test set -- protective stops per "
     "100 m, success and mission time; the frozen alpha 1.0 was selected on the tune set where "
     "success was nearly flat (0.50-0.52, the horizon capped it), so stops decided; outside the "
     "confirmatory family (A32)\n"),
]


def section() -> str:
    """The amendment text between the draft's two `---` rules."""
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A32 - "):
        raise SystemExit("the draft's A32 section was not found between its `---` lines")
    return parts[1].strip("\n") + "\n"


def _apply(name: str, text: str, edits: list[tuple[str, str]]) -> str:
    for old, new in edits:
        if text.count(old) != 1:
            raise SystemExit(f"{name}: a text to replace occurs {text.count(old)} times, not once "
                             f"({old.strip()[:60]!r}); nothing written")
        text = text.replace(old, new)
    return text


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A31 - " not in amendments:
        raise SystemExit("A31 is not applied yet (apply_A31.py first); nothing written")
    if "## A32 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A32; nothing written")
    gates = _apply("gates.yaml", (PREREG / "gates.yaml").read_text(encoding="utf-8"), GATES)
    endpoints = _apply("endpoints.yaml", (PREREG / "endpoints.yaml").read_text(encoding="utf-8"),
                       ENDPOINTS)
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + section())
    with open(PREREG / "gates.yaml", "w", encoding="utf-8") as f:
        f.write(gates)
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints)
    print("A32 applied: AMENDMENTS.md, gates.yaml, endpoints.yaml. Commit them, then "
          "`python scripts/freeze_prereg.py --relock A32`.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
