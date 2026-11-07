# Journal draft: Protective-Field Barriers (full study)

`pfb_journal.tex` (generic single-column preprint layout: `article` class, `natbib` with `plainnat`; no
publisher template), `references.bib`, `figures/`, `results_tune.json` (the values behind the results
figure and tables) and `scripts/fig_journal_results.py`. About 24 pages. Written 2026-11-07.

**Scope (researcher, 2026-11-07):** the full study, classical planners (B1, B2, B3, B6, B7) and the
learned SAC arms (L1-L5). Learned arms are reported as PILOT results (G4) or as placeholders.

## Binding rules (supervisor, 2026-11-07)
1. **DRAFT; never submitted by anyone but the authors.** Submission is the authors' decision.
2. **Every number is a DEVELOPMENT-SET (tune set) result**; held-out (test) numbers are red
   `[held-out: pending]` / `[pending]` placeholders (`\pending`). No test layout is used anywhere (hard rule 2).
3. Wording: "B6 and B7 are indistinguishable at 3 versus 3 events" (never "B6 matches B7"); "H1prime is a
   declared exploratory contingency; held-out evaluation pending"; never "preregistered but exploratory".
4. Learned arms: L5's pilot seed 9000002 passed G4; L1 failed G4 on three pilot seeds (9000000 under the
   pre-A30 rules, 9000001 and 9000002 under A30). L2, L4 and L5 9000003 pilots: `[running]` placeholders.
5. No invented numbers or references. VERIFY notes live in the bib field `verify`, which `plainnat`
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
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig1_architecture.py
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig2_field_geometry.py
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig4_layout.py
FIG_OUT=paper/journal/figures FIG_TEXT_WIDTH=6.3 python paper/figures/scripts/fig5_pfb_step.py
python paper/journal/scripts/fig_journal_results.py
```
`fig1` is drawn in full scope (learned and classical controllers). `fig3_coverage.pdf` is copied from
`paper/MDPI_template/figures/`: its script reads a footprint table from a commit of the old repository
(`git show ce2306d...`), which this repository does not have.

## Where each number comes from (docs/decision_log.md, entries of the dates given)
| Claim | Source |
|---|---|
| Table 2 (all arms: success, stops per 100 m planned / driven / completed, latched, moving speed) | 2026-11-07 "Offline diagnostics, first read (item 3)", table 3c/3d; "All arms under the A34 denominators" |
| Table 3 (B2 vs B7 by stratum) | as the CoDIT draft: B7 alpha tuning, `runs/tuning/B7_alpha_20261102T195759Z_seed0` (2026-11-03) |
| Fig. 5c, alpha grid (success, stops driven, fallback steps) | 2026-11-03 "alpha 1.0 FROZEN"; `results_tune.json` |
| B7 = B2 moving speed 0.057 m/s on 13 routes; B2 27 % latched, 69 stops | 2026-11-06 "REVIEWED: B2 vs B7 speed" |
| RPP regulation from 1.18 m; 0.73 m vs 4.34 / 4.20 m in 10 s | 2026-11-06 ruling (4); 2026-11-07 compute entry |
| 17 blocked = privileged tracker's 17 failures; 13 long-route timeouts 68-97 %; 30/43 | 2026-11-03 "B7's timeouts classified" (via the CoDIT README) |
| B7's 3 stops: QP infeasible next to the obstacle, braking | 2026-11-07 diagnostics 3b |
| B1: 24 candidates, latched 83-90 %; 5 traces, 6-8 stops, cross-track 1-8 cm | 2026-11-06 "B3 and B1 tuning finished"; 2026-11-07 moving-speed table; diagnostics 3e |
| B2 fails >= 75 % of routes by the restop cycle | 2026-11-01 (CLAUDE.md, plan of record) |
| B3/B6 share the sampler seed; 93 % positions | 2026-11-07 "B6 TUNED"; ruling 2026-11-07 item 1 |
| Table 4 (G4) | 2026-10-30 (L1 9000000); 2026-11-04 (L1 9000001/9000002); 2026-11-03 (L5 9000001 crash, A32); 2026-11-06 (L5 9000002 PASS) |
| L5 training 924/1,754, 0 unrecoverable; checkpoint per stratum 6/11/14; 29 timeouts | 2026-11-06 "L5 pilot seed 9000002" |
| L5 requests 0.43-0.63 m/s, 45-99 % >= 0.45; filter alters ~100 % of steps; B7 50/89 %; wandering 1.6x (1.73/1.45/1.61), 4-6 m detours | 2026-11-07 diagnostics 3a; "L5's checkpoint and the oracle under the A34 denominators" |
| Compute p95 (PFB 27.9 / 28.0 ms, B6 34.5 / 37.1 ms) | 2026-11-07 diagnostics 3d; 2026-10-29 real-time measurement |
| Throughput 21.1 vs ~33 steps/s; G2 117 steps/s at N = 4 | 2026-11-06 L5 provenance; 2026-10-30/11-04 L1 runs; 2026-10-29 G2 PASS |
| Primary per planned length (A35); K = 10, seeds 9100001-9100010 | 2026-11-07 rulings; `preregistration/endpoints.yaml`, `gates.yaml` |
| Power: no feasible size reaches 80 % at plausible rates; test-size rule | 2026-11-07 "A14b power grid"; ruling items 2-3 |
| k_proximity 0.10113 (derived); 2M steps, curriculum, stall truncation; L2 d = 1.0 / 100 m | 2026-11-01 ruling; 2026-10-30 A30 rulings; `configs/base.yaml` |
| Tune set: 60 scenarios, 633 m, 17 with unmapped obstacles; training set mean 10.67 m | `scenarios/tune/*.json` (read 2026-11-07); 2026-10-30 (L_ref 10.671616 m) |
| Field, barrier rows, motion fractions, QP, fallback | `pfbnav/safety/pfb_field.py`, `pfbnav/safety/pfb.py`, `configs/base.yaml` `filter:` |
| Platform, sensing, certified layer, verification findings | `paper/claims_trace.md` (MDPI draft trace), `configs/base.yaml`, CLAUDE.md |

## Open before any submission
- Every `verify` field in `references.bib` (checked against the publisher's record).
- Red markers: data/code availability, acknowledgements and AI-use disclosure, the withheld citation.
- Held-out tables (Tables 5 and 6) and every `[pending]` / `[running]` cell.

Commit this folder on its own, never together with code.
