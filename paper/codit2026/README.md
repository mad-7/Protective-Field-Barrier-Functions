# CoDIT regular paper draft: Protective-Field Barriers for classical local planners

`pfb_codit.tex` (IEEE conference class, double column, <= 6 pages; compiles to 4) and `references.bib`.
**Scope (researcher, 2026-11-07):** the PFB as a safety filter for CLASSICAL local planners only (B1, B2,
B3, B6, B7). Learned planners are deliberately left out until their results are reviewed.

**Build** (needs `IEEEtran.cls`): `pdflatex pfb_codit && bibtex pfb_codit && pdflatex pfb_codit && pdflatex pfb_codit`.

## Before submitting
1. **Supervisor approval.** Every result is TUNE-SET evidence ("monitoring and framing only" in the
   preregistered study). Publishing it as preliminary is the supervisor's decision; no test layout may be
   used for any number in the paper (hard rule 2).
2. **Red `[...]` placeholders** (`	bd`): authors and affiliation; B1's and B2's per-planned-length stop
   rates (`scripts/resummarize_eval.py` on `runs/tuning/B1_pure_pursuit_20261104T183811Z_seed0/candidate_13`
   and `runs/tuning/B2_regulated_pure_pursuit_20261030T111716Z_seed0/candidate_23`); the PFB's p95 compute
   (`runs/realtime/REALTIME_20261029T095844Z_seed0`, pfb part). Optional: a figure (one narrow-aisle episode,
   B2 against B7, with stop locations).
3. **Four references marked VERIFY** at the end of `references.bib` (Coulter 1992, Macenski 2023, Williams
   2017, Macenski 2020). The rest are copied verbatim from `paper/MDPI_template/references.bib`.

## Where each number comes from (docs/decision_log.md, entries of the dates given)
| Claim | Source |
|---|---|
| Table I: success, stops planned/driven, latched share, moving speed | 2026-11-07 "Moving speed and latched share", "All arms under the A34 denominators" |
| Table II: B2 and B7 per stratum | B7 alpha tuning console, B2-vs-B7 monitoring (`runs/tuning/B7_alpha_20261102T195759Z_seed0`), 2026-11-03 |
| Table III: alpha sensitivity (success, stops, fallback steps) | 2026-11-03 "alpha 1.0 FROZEN" |
| B7 = B2 moving speed 0.057 m/s on 13 routes; B2 27 % latched, 69 stops | 2026-11-06 "REVIEWED: B2 vs B7 speed" |
| 17 blocked = the oracle's 17 failures; 13 horizon timeouts, 68-97 % covered; 30/43 | 2026-11-03 "B7's timeouts classified" |
| 28 of 30 B7 timeouts without a stop (2000050: 2, 2000053: 1) | same entry, per-timeout table |
| tuned B2 slows below 1.18 m clearance; 0.73 m in 10 s vs 4.34 / 4.20 m | 2026-11-06 bracket positions; 2026-11-07 compute entry |
| alpha 1 brakes at a wall 0.95 m ahead at 0.3 m/s; filtered at alpha 2 | 2026-11-03 "the frozen alpha brakes where 2.0 filtered" (tests/test_pfb.py) |
| B1 latched 83-90 % at every candidate | 2026-11-07 moving-speed table |
| B6 50/60, 3 stops; B7 3 | 2026-11-07 denominators table |
| Compute p95 | 2026-11-07 compute entry; 2026-10-29 real-time measurement (B6) |
| Field, barrier row, QP | `pfbnav/safety/pfb_field.py`, `pfbnav/safety/pfb.py` docstrings |
| T = 0.15 s; box 0.5/0.5/1.5; dt 0.05; two S300 scanners | `configs/base.yaml`; `paper/claims_trace.md`; the researcher (scanner count) |

Commit this folder on its own, never together with code (`SESSION_HANDOFF.md`).
