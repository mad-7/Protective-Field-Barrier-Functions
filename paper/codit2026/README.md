# CoDIT regular paper draft: Protective-Field Barriers for classical local planners

`pfb_codit.tex` (IEEE conference class, double column, <= 6 pages; compiles to 6 with four figures),
`references.bib`, `figures/`, `results_test.json` (held-out values behind Tables I-II and Fig. 3a-b) and
`results_tune.json` (development-set values behind Fig. 3c, each with its run and decision-log source). Written in academic style (researcher, 2026-11-07).

**Figures** (from the project's own geometry, configuration and runs; nothing typed by hand except
`results_test.json` and `results_tune.json`; the host can check them against the runs):
```
FIG_OUT=paper/codit2026/figures FIG_TEXT_WIDTH=7.16 FIG_SCOPE=classical python paper/figures/scripts/fig1_architecture.py
FIG_OUT=paper/codit2026/figures FIG_TEXT_WIDTH=7.16 python paper/figures/scripts/fig2_field_geometry.py
FIG_OUT=paper/codit2026/figures FIG_TEXT_WIDTH=7.16 python paper/figures/scripts/fig5_pfb_step.py
FIG_OUT=paper/codit2026/figures FIG_TEXT_WIDTH=7.16 FIG_SCOPE=classical python paper/codit2026/fig_codit_results.py [--from-runs]
```
Printed order: Fig. 1 architecture (classical scope); Fig. 2 the fields from `pfb_field.py`; Fig. 3 the planners'
stops, latched time and success (held-out) and B7 against alpha (development set); Fig. 4 one PFB step from the
emulator on a tuning route. `paper/codit2026/fig_codit_results.py` supersedes the tune-set version in
`paper/figures/scripts/`, which is left unchanged.
**Scope (researcher, 2026-11-07):** the PFB as a safety filter for CLASSICAL local planners only (B1, B2,
B3, B6, B7). Learned planners are deliberately left out until their results are reviewed.

**Build** (needs `IEEEtran.cls`): `pdflatex pfb_codit && bibtex pfb_codit && pdflatex pfb_codit && pdflatex pfb_codit`.

## Before submitting
1. **This is a DRAFT; submission is the authors' decision** (the researcher, the supervisor and the
   co-authors; supervisor ruling 2026-11-07). The held-out classical results are now the basis. Labels: anything
   still from the tuning set says development-set; B7 vs B2 is confirmatory; B7 vs B1 and B6 are exploratory; B7
   vs B6 reads "indistinguishable at 18 vs 10 events", never "matches"; the phrase "preregistered but exploratory"
   is not used. Success and time are descriptive. No tuning used a test layout (hard rule 2).
2. **Values not in the decision log, confirmed 2026-11-08** by the main session from the printout of the run's
   `report.json` on the host (`descriptive.<arm>.pooled`): the mean mission times (B3 131.2 on 2 successes, B6 129.6,
   B7 169.0 on 3) and the step count 1,815 of 598,139. RPP's request and the PFB's correction both have p95 0.075
   m/s on the obstacle half; these are 5 mm/s histogram bins reported at the upper edge. The paper states the request's.
3. **Everything on the half without obstacles is exploratory or descriptive;** the confirmatory comparison is the
   obstacle half's B7 vs B2 only. B7 vs B3 was not declared: counts only.
4. **References marked VERIFY** in `references.bib` (Coulter 1992, Macenski 2023, Williams 2017, Macenski 2020,
   and the `verify` fields of Raja 2024, IEC 61496-3, Clarabel, Agrawal 2017, Reis 2020; the last two are taken from the reference list of Garg et al. 2024). The rest are copied verbatim from
   `paper/MDPI_template/references.bib`.

## Where each number comes from (docs/decision_log.md, entries of the dates given)
| Claim | Source |
|---|---|
| Table I, Fig. 3a-b, rate ratios (B7/B2, B7/B1, B7/B6), 98 %, 26 % further, 146 to 0 unrecoverable, 5.9 m per episode on 10.8 m routes, request p95 0.075, median correction 0.025, 37.6 %, infeasible 0.3 % | 2026-11-08 "The sealed classical test (unmapped_static): HF-c PASS"; `results_test.json`; run `runs/test/HFC_ANALYSIS_20261108T171312Z_seed0` (host) |
| Table II (half without obstacles), its rate ratios, median correction 0.005, 0.4 %, mission times, the declared positive control | 2026-11-08 "The sealed classical test (mapped_only)"; `results_test.json`; run `runs/test/HFC_ANALYSIS_mapped_only_20261108T223147Z_seed0` (host) |
| Mean mission times of successes; 1,815 of 598,139 steps | `report.json` of the same run, read from the host printout; see "Before submitting" item 2 |
| Disclosures: test size and B7 vs B2 fixed before the test set was built or read, after development rates seen (52.8, 0.47); calibration before the read | 2026-11-07 "DECIDED (the researcher)", `docs/pending/A36_amendment.md`; 2026-11-08 "HF-c calibration PASSED" |
| Development set by obstacle presence: success on 17 obstacle / 43 clear routes, moving speeds, RPP stops 224 to 0 | 2026-11-08 "The success collapse is the population" |
| Development-set rates 52.8 and 0.47; 633 m; 17 of 60 routes with obstacles | 2026-11-07 "All arms under the A34 denominators"; `scripts/split_tune_by_obstacles.py` docstring |
| Fig. 3c and the gain paragraph: alpha sensitivity (success, stops, fallback steps) | 2026-11-03 "alpha 1.0 FROZEN" |
| 17 blocked = the oracle's 17 failures; 13 horizon timeouts, 68-97 % covered; 30/43 | 2026-11-03 "B7's timeouts classified" |
| 28 of 30 B7 timeouts without a stop (2000050: 2, 2000053: 1) | same entry, per-timeout table |
| tuned B2 slows below 1.18 m clearance; 0.73 m in 10 s vs 4.34 / 4.20 m | 2026-11-06 bracket positions; 2026-11-07 compute entry |
| alpha 1 brakes at a wall 0.95 m ahead at 0.3 m/s; filtered at alpha 2 | 2026-11-03 "the frozen alpha brakes where 2.0 filtered" (tests/test_pfb.py) |
| B1 latched 83-90 % at every candidate | 2026-11-07 moving-speed table |
| B7's three stops in two blocked episodes (2000050, 2000053); retraced: QP infeasible next to the obstacle, maximal braking, field not kept clear | 2026-11-07 "Offline diagnostics, first read (item 3)", 3b; "Diagnostics item 3 COMPLETE" (trace `runs/diag/TRACE2_20261107T190227Z`, training host) |
| B1: five retraced failures, unrecoverable stop after 6-8 stops, cross-track p95 1-8 cm | same entries, 3e (trace `runs/diag/TRACE2_20261107T191325Z`, training host) |
| Compute p95 | 2026-11-07 compute entry; 2026-10-29 real-time measurement (B6) |
| CBF background: safe set, CBF vs candidate, QP, pointwise optimality, sampling | Garg et al. 2024 (checked against the PDF: eqs. 2-4, 8; Def. 2.1; Secs. 1.1, 3.4, 7.2) |
| Candidate CBF, validity not shown; gamma = alpha dt = 0.05; saturation stricter than the discrete condition; measured margins; non-smooth field, regime switching, up to 120 rows; myopia as a possible reading; deployed field is the band of the regime (band tops) | 2026-11-09 "The PFB reviewed against the CBF theory"; researcher-approved wording 2026-11-09; `pfbnav/safety/pfb.py`, `pfb_field.py` |
| QP infeasible on 0.15 % of B7's steps, half without obstacles (777 of 514,175) | 2026-11-08 "The sealed classical test (mapped_only)"; `report.json` printout |
| Field, barrier row, QP | `pfbnav/safety/pfb_field.py`, `pfbnav/safety/pfb.py` docstrings |
| T = 0.15 s; box 0.5/0.5/1.5; dt 0.05; two S300 scanners | `configs/base.yaml`; `paper/claims_trace.md`; the researcher (scanner count) |

Commit this folder on its own, never together with code (`SESSION_HANDOFF.md`).
