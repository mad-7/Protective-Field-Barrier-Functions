"""Apply A37 (HF_hosts) to preregistration/ (the RESEARCHER runs this; hard rule 1).

    python docs/pending/apply_A37.py

Appends the A37 section of docs/pending/A37_amendment.md to preregistration/AMENDMENTS.md and adds HF_hosts to the
`exploratory` block of endpoints.yaml, in UTF-8. Refuses unless A36 is applied and A37 is not, and unless the text it
extends occurs exactly once; on any mismatch nothing is written. Then commit the two files by name, and
`python scripts/freeze_prereg.py --relock A37`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A37_amendment.md"

ANCHOR = "  L2_sensitivity: "
LINE = ("  HF_hosts: B9 (B3 + PFB) versus B3 and B8 (B6 + PFB) versus B6, protective stops per 100 m planned on each half of "
        "the test set, paired within one scene load; proposed after HF-c was read, so exploratory; low-count comparisons "
        "flagged, not read as verdicts (A37)\n")


def section() -> str:
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A37 - "):
        raise SystemExit("the draft's A37 section was not found between its `---` lines")
    return parts[1].strip("\n") + "\n"


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A36 - " not in amendments:
        raise SystemExit("A36 is not applied yet; nothing written")
    if "## A37 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A37; nothing written")
    endpoints = (PREREG / "endpoints.yaml").read_text(encoding="utf-8")
    if endpoints.count(ANCHOR) != 1:
        raise SystemExit(f"endpoints.yaml: the anchor occurs {endpoints.count(ANCHOR)} times, not 1; nothing written")
    endpoints = endpoints.replace(ANCHOR, LINE + ANCHOR)
    text = section()
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + text)
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints)
    print("A37 applied: AMENDMENTS.md, endpoints.yaml. Commit them by name, then: "
          "python scripts/freeze_prereg.py --relock A37")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
