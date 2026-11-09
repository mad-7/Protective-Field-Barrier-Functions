# Journal draft: Protective-Field Barriers (full study)

`pfb_journal.tex` (generic single-column preprint layout: `article` class, `natbib` with `plainnat`; no
publisher template), `references.bib`, `figures/`, `results_test.json` (held-out classical values),
`results_tune.json` (development-set and pilot values) and `scripts/`. About 27 pages. Written 2026-11-07;
held-out classical results added 2026-11-08.

**Scope (researcher, 2026-11-07):** the full study, classical planners (B1, B2, B3, B6, B7) and the
learned SAC arms (L1-L5).

## Binding rules (supervisor, 2026-11-07; relayed for the held-out results, 2026-11-08)
1. **DRAFT; never submitted by anyone but the authors.** Submission is the authors' decision.
2. **Three labels.** HELD-OUT: the classical planners on the sealed test set. DEVELOPMENT-SET: the tune set.
   PILOT: learned arms on pilot-range seeds. **No learned arm has run on the test set and none may be
   described as held-out**: H1 and H2 are untested and keep red `[pending]` placeholders.
3. Wording: "indistinguishable at 18 vs 10 events" and "at 2 vs 9 events" on the test set, "at 3 vs 3" on the
   development set (never "B6 matches B7"); H1prime is a declared exploratory contingency, no pivot occurred;
   never "preregistered but exploratory".
4. Stated plainly as NOT supported: B7 completing obstacle routes (147 of 150 time out); the PFB beating MPPI
   in either condition; on clear routes B3 and B6 complete more than B7. Success and time are descriptive (A34).
5. Disclosed: the test size (A14b) and HF-c as confirmatory (A36) were fixed before the test set was built or
   read but after the development-set rates were seen, and the size before L5's second pilot seed.
6. Name (researcher, 2026-11-09): the PFB is a candidate discrete-time CBF or a CBF-based safety filter, never simply a
   CBF; validity is not shown and no safety guarantee is claimed. The deployed field is piecewise in speed (per band);
   the continuous-in-speed field is PFB-(a), the H3 ablation.
7. No invented numbers or references. VERIFY notes live in the bib field `verify`, which `plainnat`
   ignores, so they never print. One citation (`iso10218cbf2026`, author unknown) is withheld in the text
   with a red marker until its authors are verified.

## Build
From this folder (MiKTeX or TeX Live):
```
pdflatex pfb_journal
bibtex pfb_journal
pdflatex pfb_journal
pdflatex pfb_journal
```

## Figures
Regenerated from the project's own geometry, configuration and runs (repository root):
```
python paper/journal/scripts/fig_architecture_journal.py   # Figure 1 (schematic, vector)
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig2_field_geometry.py
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig4_layout.py
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig5_pfb_step.py
python paper/journal/scripts/fig_journal_results.py
python paper/journal/scripts/fig_journal_heldout.py
```
Figure 1 (`fig_architecture.pdf`) is a schematic drawing in full scope (classical and learned planners); `fig1_architecture.pdf` (the shared script's version) is kept but unused. `fig3_coverage.pdf` is copied from
`paper/MDPI_template/figures/`: its script reads a footprint table from a commit of the old repository
(`git show ce2306d...`), which this repository does not have.

## Where each number comes from (docs/decision_log.md, entries of the dates given)
| Claim | Source |
|---|---|
| Tables 2-4, Fig. 5 (HELD-OUT, both halves; rate ratios; PFB corrections; declared control) | 2026-11-08 "The sealed classical test (unmapped_static)" and "(mapped_only)"; `runs/test/HFC_ANALYSIS_20261108T171312Z_seed0`, `runs/test/HFC_ANALYSIS_mapped_only_20261108T223147Z_seed0` (training host). Checked 2026-11-08 by the host session against the researcher's printouts of both `report.json` files: every printed value matches; the run files themselves were not read from the laptop. Driven and planned metres and the fractions are derived as stops / rate (half 2's B6 driven +-2 m) |
| Half 1 mission times (B3 131.2, B6 129.6, B7 169.0 s) | relayed 2026-11-08; `paper/codit2026/results_test.json` (not in the log's table) |
| Tune split, 17 obstacle / 43 clear routes | 2026-11-08 "The success collapse is the population" |
| Calibration gate (0.0297; highest 0.0274; ungated cells) | 2026-11-08 "HF-c calibration PASSED" |
| HF-c, A14b, disclosures | 2026-11-07 "DECIDED"; `docs/pending/A36_amendment.md`; applied in `da0952f`, relocked `1d1ebf0`, test sets built `6afb838` (on origin/main) |
| Power and test size (H1 max 0.48; H2 at +20 pp, SD <= 0.3) | 2026-11-07 "A14b refined grid" |
| Table 5 (DEVELOPMENT-SET, all arms: success, stops per 100 m planned / driven / completed, latched, moving speed) | 2026-11-07 "Offline diagnostics, first read (item 3)", table 3c/3d; "All arms under the A34 denominators" |
| Table 6 (B2 vs B7 by stratum, development set) | as the CoDIT draft: B7 alpha tuning, `runs/tuning/B7_alpha_20261102T195759Z_seed0` (2026-11-03) |
| Fig. 6c and text, alpha grid (success, stops driven, fallback steps) | 2026-11-03 "alpha 1.0 FROZEN"; `results_tune.json` |
| B7 = B2 moving speed 0.057 m/s on 13 routes; B2 27 % latched, 69 stops | 2026-11-06 "REVIEWED: B2 vs B7 speed" |
| RPP regulation from 1.18 m; 0.73 m vs 4.34 / 4.20 m in 10 s | 2026-11-06 ruling (4); 2026-11-07 compute entry |
| 17 blocked = privileged tracker's 17 failures; 13 long-route timeouts 68-97 %; 30/43 | 2026-11-03 "B7's timeouts classified" (via the CoDIT README) |
| B7's 3 stops: QP infeasible next to the obstacle, braking | 2026-11-07 diagnostics 3b |
| B1: 24 candidates, latched 83-90 %; 5 traces, 6-8 stops, cross-track 1-8 cm | 2026-11-06 "B3 and B1 tuning finished"; 2026-11-07 moving-speed table; diagnostics 3e |
| B2 fails >= 75 % of routes by the restop cycle | 2026-11-01 (CLAUDE.md, plan of record) |
| B3/B6 share the sampler seed; 93 % positions | 2026-11-07 "B6 TUNED"; ruling 2026-11-07 item 1 |
| Table 7 (G4, pilot) | 2026-10-30 (L1 9000000); 2026-11-04 (L1 9000001/9000002); 2026-11-03 (L5 9000001 crash, A32); 2026-11-06 (L5 9000002 PASS) |
| L5 training 924/1,754, 0 unrecoverable; checkpoint per stratum 6/11/14; 29 timeouts | 2026-11-06 "L5 pilot seed 9000002" |
| L5 requests 0.43-0.63 m/s, 45-99 % >= 0.45; filter alters ~100 % of steps; B7 50/89 %; wandering 1.6x (1.73/1.45/1.61), 4-6 m detours | 2026-11-07 diagnostics 3a; "L5's checkpoint and the oracle under the A34 denominators" |
| Compute p95 (PFB 27.9 / 28.0 ms, B6 34.5 / 37.1 ms) | 2026-11-07 diagnostics 3d; 2026-10-29 real-time measurement |
| Throughput 21.1 vs ~33 steps/s; G2 117 steps/s at N = 4 | 2026-11-06 L5 provenance; 2026-10-30/11-04 L1 runs; 2026-10-29 G2 PASS |
| Primary per planned length (A35); K = 10, seeds 9100001-9100010 | 2026-11-07 rulings; `preregistration/endpoints.yaml`, `gates.yaml` |
| Power: no feasible size reaches 80 % at plausible rates; test-size rule | 2026-11-07 "A14b power grid"; ruling items 2-3 |
| k_proximity 0.10113 (derived); 2M steps, curriculum, stall truncation; L2 d = 1.0 / 100 m | 2026-11-01 ruling; 2026-10-30 A30 rulings; `configs/base.yaml` |
| Tune set: 60 scenarios, 633 m, 17 with unmapped obstacles; training set mean 10.67 m | `scenarios/tune/*.json` (read 2026-11-07); 2026-10-30 (L_ref 10.671616 m) |
| Section 3, general CBF definitions | Garg et al. 2024, Annual Reviews in Control 57, 100945 (eqs. 1-4, 8, 93; Defs. 2.1, 2.2, 3.3, 4.1, 4.2, 6.1, 7.2, 7.4, 7.5; Secs. 1.1, 3.4, 4.1.5, 5.1, 5.2, 7.2, 9), read from the PDF 2026-11-09; eq. 93 restated in the h >= 0 convention |
| Field, barrier rows, motion fractions, QP, fallback | `pfbnav/safety/pfb_field.py`, `pfbnav/safety/pfb.py`, `configs/base.yaml` `filter:` |
| Platform, sensing, certified layer, verification findings | `paper/claims_trace.md` (MDPI draft trace), `configs/base.yaml`, CLAUDE.md |

## Open before any submission
- Every `verify` field in `references.bib` (checked against the publisher's record).
- Red markers: data/code availability, acknowledgements and AI-use disclosure, the withheld citation.
- The learned arms: Tables 8 and 9 (H1, H2) and every `[pending]` cell of Table 7 (L2, L4, L5 9000003).

Commit this folder on its own, never together with code.
