# CoDIT regular paper draft: Protective-Field Barriers

`pfb_codit.tex` (IEEE conference class, double column, target <= 6 pages) and `references.bib`.

**Build** (needs `IEEEtran.cls`, standard in TeX Live / MiKTeX / Overleaf):
`pdflatex pfb_codit && bibtex pfb_codit && pdflatex pfb_codit && pdflatex pfb_codit`.

## Before submitting
1. **Supervisor approval.** Every result is TUNE-SET, pilot-seed evidence, labelled "monitoring and framing
   only" in the preregistered study. Publishing it as preliminary is a decision for the supervisor; no test
   layout may be used for any number in the paper (hard rule 2).
2. **Red `[...]` placeholders** (`\tbd`): authors and affiliation; B1, B2 and L1 per-planned-length rates
   and L1's latched share / moving speed (`scripts/resummarize_eval.py` on B1 candidate_13, B2 candidate_23,
   `runs/eval/L1_sac_shaped_20261104T124625Z_seed0`); the PFB's p95 compute (`runs/realtime/REALTIME_20261029T095844Z_seed0`
   pfb part); Fig. 1 (to be made).
3. **The four references marked VERIFY** at the end of `references.bib` (Coulter 1992, Macenski 2023 RPP,
   Williams 2017 MPPI, Macenski 2020 Nav2): check volume, pages and titles against the publishers.
   The other 22 are copied verbatim from `paper/MDPI_template/references.bib`.
4. Check the page count in the PDF; trim Related Work first if over six pages.

## Where each number comes from (docs/decision_log.md, entries of the dates given)
| Claim | Source |
|---|---|
| B2 86.1 -> B7 0.59 stops/100 m driven; latched 42 % -> 0.05 % | 2026-11-07 "Moving speed and latched share", "All arms under the A34 denominators" |
| B7 and B2 0.057 m/s moving on the 13 timeouts | 2026-11-06 "REVIEWED: B2 vs B7 speed" |
| 17 of 30 B7 timeouts blocked = the oracle's 17 failures | 2026-11-03 "B7's timeouts classified" |
| Table I (planned / driven / latched / moving) | 2026-11-07 tables (resummarize_eval, peek_tuning) |
| L1 G4: 0.006 / 0.061, checkpoint 1/60, 0/60; replay-buffer collapse | 2026-11-04 "L1's two pilot seeds", 2026-11-03 decline diagnosis |
| L5 G4: 0.463, 31/60; latched 0.15 %; drives 1.6x | 2026-11-06 "L5 pilot seed 9000002: G4 PASS", 2026-11-07 denominators |
| B6 50/60, 3 stops; B7 3; L5 6 | 2026-11-07 "All arms under the A34 denominators" |
| Compute p95 B1-B3, B6 | 2026-11-07 compute entry; 2026-10-29 real-time measurement |
| alpha = 1.0 from {1,2,5,10,20}, the selection rule | 2026-11-03 "alpha 1.0 FROZEN" |
| Field definition, barrier row, QP | `pfbnav/safety/pfb_field.py`, `pfbnav/safety/pfb.py` docstrings |
| T = 0.15 s, box 0.5/0.5/1.5, dt 0.05 | `configs/base.yaml`; `paper/claims_trace.md` |

Commit this folder on its own, never together with code (`SESSION_HANDOFF.md`).
