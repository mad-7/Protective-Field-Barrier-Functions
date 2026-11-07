"""Apply A14b (the test size) and A36 (HF-c) to preregistration/ (the RESEARCHER runs this; hard rule 1).

    python docs/pending/apply_A36.py

Appends the A14b and A36 sections of docs/pending/A36_amendment.md to preregistration/AMENDMENTS.md, sets
the test size in scenario_sets.yaml and appends the HF-c family to endpoints.yaml, in UTF-8. Refuses unless
A35 is applied and neither A14b nor A36 is, and unless every text it replaces occurs exactly once; on any
mismatch nothing is written. Then commit the three files and this draft by name, and
`python scripts/freeze_prereg.py --relock A36`.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PREREG = REPO / "preregistration"
DRAFT = REPO / "docs" / "pending" / "A36_amendment.md"

SCENARIO_SETS = [
    ("  test: pending_A14\n",
     ("  test:                             # A14b (2026-11-07, the researcher's decision)\n"
      "    per_cell: 50                    # per stratum x condition: 3 x 2 cells, 300 scenarios, locked\n"),
     1),
]

HFC_BLOCK = """
filter_efficacy_family:            # A36 (2026-11-07, the researcher's decision); separate from confirmatory_family
  members: [HFc]                   # one member: no correction
  alpha: 0.05

HFc:                               # A36: the PFB's efficacy on a classical planner
  statement: B7 (RPP + PFB) triggers fewer protective stops than B2 (RPP alone)
  claim: the filter removes protective stops from a classical planner -- stop removal only; success, mission time, latched share and moving speed reported descriptively beside it, no inference (as A34)
  comparison: B7 vs B2
  endpoint: protective_stops_per_100m_planned   # A35
  scan_faults: as H1 (A25)
  population: test, condition unmapped_static, all clearance strata pooled; B1, B2, B3, B6, B7 on each scenario within one scene load
  analysis: >-
    the A14a rate ratio, B7 over B2, continuity 0.5 on each arm's total; both arms run once, so the
    interval has the scenario part only, t quantile with n - 1 df; two-sided 95 %; rejects in B7's
    favour when the log-scale interval lies wholly below 0; reported as the rate ratio with its interval
  calibration: >-
    before the test is read: at equal rates, 150 pairs, pilot route lengths, 10,000 replications, the
    one-sided rejection rate on each side at most 0.025 + 3 MC SE, at rates {0.5, 1, 2, 5, 52.8} per 100 m
    and difficulty CV {0.5, 1.0} (configs/analysis/hfc_calibration.yaml); any gated cell failing -> stop and
    report, HF-c not tested; 0.25 per 100 m and CV 1.5 reported, not gated (A36)
  success: the comparison significant in the predicted direction
  disclosure: decided after the development-set B2 and B7 rates were seen (B2 52.8, B7 0.47 per 100 m planned); the test set untouched (A36)
"""


def section() -> str:
    parts = DRAFT.read_text(encoding="utf-8").split("\n---\n")
    if len(parts) < 3 or not parts[1].lstrip().startswith("## A14b - "):
        raise SystemExit("the draft's A14b/A36 sections were not found between its `---` lines")
    body = parts[1].strip("\n") + "\n"
    if "\n## A36 - " not in body:
        raise SystemExit("the draft's A36 section is missing")
    return body


def _apply(name: str, text: str, edits) -> str:
    for old, new, n in edits:
        if text.count(old) != n:
            raise SystemExit(f"{name}: a text to replace occurs {text.count(old)} times, not {n} "
                             f"({old.strip()[:60]!r}); nothing written")
        text = text.replace(old, new)
    return text


def main() -> int:
    amendments = (PREREG / "AMENDMENTS.md").read_text(encoding="utf-8")
    if "## A35 - " not in amendments:
        raise SystemExit("A35 is not applied yet; nothing written")
    if "## A14b - " in amendments or "## A36 - " in amendments:
        raise SystemExit("AMENDMENTS.md already has A14b or A36; nothing written")
    sets = _apply("scenario_sets.yaml",
                  (PREREG / "scenario_sets.yaml").read_text(encoding="utf-8"), SCENARIO_SETS)
    endpoints = (PREREG / "endpoints.yaml").read_text(encoding="utf-8")
    if "\nHFc:" in endpoints or "filter_efficacy_family" in endpoints:
        raise SystemExit("endpoints.yaml already has HFc; nothing written")
    endpoints = endpoints.rstrip("\n") + "\n" + HFC_BLOCK
    text = section()
    sep = "" if amendments.endswith("\n\n") else ("\n" if amendments.endswith("\n") else "\n\n")
    with open(PREREG / "AMENDMENTS.md", "w", encoding="utf-8") as f:
        f.write(amendments + sep + text)
    with open(PREREG / "scenario_sets.yaml", "w", encoding="utf-8") as f:
        f.write(sets)
    with open(PREREG / "endpoints.yaml", "w", encoding="utf-8") as f:
        f.write(endpoints)
    print("A14b and A36 applied: AMENDMENTS.md, scenario_sets.yaml, endpoints.yaml. Commit them by name, "
          "then: python scripts/freeze_prereg.py --relock A36")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
