# Decision request — before the freeze (2026-09-28)

Your 2026-09-27 ruling on the narrow stratum is implemented and measured. Five decisions stand
between here and freezing `preregistration/`; each is answerable in a line, and each has a
recommendation so a "yes" is enough.

## Where things stand

- **Ruling 1, the field.** You were right, and it was worse than the abeam example: the moving
  field was smaller than the at-rest field over **~80 % of bearings at every band, by up to
  62.6 mm**. The at-rest field is now `e(ψ) + lateral_margin`, exactly the infimum of the moving
  fields. Monotonicity in speed is asserted per bearing and per heading; worst violation
  **0.000000 mm**.
- **Ruling 2, the floor.** 0.13 m holds. The at-rest and moving requirements now coincide at
  0.10091 m, and a robot may stop up to **79.1 mm** off-centre in narrow against the 50 mm the
  controller is permitted (it was 29.1 mm).
- **Rulings 3, 4 and 6** are in: the restart condition, `unrecoverable_stop` as its own outcome,
  and the standing rule as hard rule 15.
- **The traversability precondition passes on the real scan** in all three strata: 0 of 170 steps
  inside the field at both offsets, and an **unavoidable-stop floor of 0.0 per 100 m** everywhere.
- **D2 re-measured in both states**, as you directed:

  | narrow floor 0.13 m | measured | model |
  |---|---|---|
  | at rest, centred | 69.5 mm | 68.7 mm |
  | top band, centred | 69.5 mm | 68.7 mm |
  | top band, at the 50 mm bound | 19.4 mm | 16.1 mm |

  The two centred figures are identical — ruling 1 confirmed by measurement, independently of
  the geometry that predicts it — and 69.5 − 19.4 = 50.1 mm recovers the committed cross-track
  to 0.1 mm. The model sits 0.8–3.3 mm *below* the measurement everywhere, so it is conservative
  and is now tested as a bound (`model ≤ measured`), never as a prediction.

---

## Decision 1 — the narrow slack has lost its justification

**The 9.6 mm "scan penalty" the 0.03 slack was set against does not exist.** It was the test
walls' **half-thickness**: a 0.02 m cuboid is placed by its centre and presents its surface
10 mm nearer, and the scanner measures the surface. A positive control added to the
traversability probe found it — a shortfall of 9.5–10.3 mm that **did not scale with distance**
across 0.28–0.60 m, so it could not be the ~1 % pixel sampling that had been assumed. Against the
wall's face the residual is **−0.27 to +0.50 mm**.

The real scan costs **about 1 mm**. So 0.03 covers it roughly thirty times over, not three.
Two tests had passed on the same artefact, each with a tolerance that happened to equal it; both
are fixed.

This also rewrites the worked example you gave for hard rule 15. The measured 9.6 mm beat the
derived 2.6 mm and was still wrong, because nothing checked what the measurement was pointing
at. The rule now carries a second half: **every probe carries a positive control.**

**Options.** (a) Keep 0.03 as a deliberate safety buffer, recorded as chosen rather than
derived. (b) Justify it from something else — wall placement tolerance in generated layouts is
the obvious candidate — which means measuring that. (c) Reduce it.

**Recommendation: (a).** It is the safe direction, the strata are already admissible, and nothing
measured argues for less. It only needs recording honestly as a choice.

## Decision 2 — the warning field saturates in the narrow stratum

A robot tracking at the committed 50 mm bound in a narrow aisle is **inside the warning field for
100 % of the traverse**. The warning field reaches 0.3095 m abeam (`lateral_margin` 0.05 +
`warning_extra` 0.05); the aisle leaves 0.2886 m. It costs nothing in the primary metric, but the
secondary near-miss metric is **uninformative in narrow** — one entry and full dwell, every
episode, for every method.

**Options.** (a) Accept it as a true statement: a narrow aisle is permanently a near miss.
(b) Give the warning field a separate, smaller abeam extension. (c) Drop the near-miss metric in
the narrow stratum only.

**Recommendation: (b)**, abeam extension 0.02 m, marked VERIFY. It keeps the metric
discriminating in narrow without changing it elsewhere, and `warning_extra` is already VERIFY.
But (a) is defensible, and this is a reporting question rather than a safety one.

## Decision 3 — the estimate the 2 % rule compares

Ruling 5 says "fewer than 2 % of narrow episodes". Whether that means the observed proportion or a
confidence bound changes what the rule does:

| pilot size | 0 events rules out | 2 events: observed / can't rule out |
|---|---|---|
| 50 | 5.8 % | — |
| 100 | 2.95 % | — |
| **150** | **1.98 %** | 1.33 % / up to **4.14 %** |

Taken literally, a pilot passes while unable to exclude a rate well above 2 %.

**Recommendation: compare the exact one-sided 95 % upper bound, with a pilot of 150 narrow
episodes** — the smallest round size at which a clean pilot actually rules out 2 %. The two
errors are not symmetric: adding a co-primary that was not needed costs some power; withholding
one that was needed leaves a compressed primary unprotected, which is the failure the rule exists
for.

This is my reading of the ruling's intent rather than its words, so it is the one I most need you
to confirm.

## Decision 4 — whose episodes trigger the rule

No controller exists yet, so the pilot runs after the methods do — on the disjoint pilot seeds,
before any test episode is evaluated, with a mechanical rule, so the endpoint is still fixed in
advance.

**Recommendation: the rule triggers if ANY method in the confirmatory comparison (L1, L2, L4, L5,
A1) reaches the threshold**, rather than the proportion pooled over all of them. Endpoints must be
identical across methods, and one method's unrecoverable stops must not be averaged away by the
others'.

## Decision 5 — the hypotheses are not in the preregistration

**This is the most important one.** Ruling 5 widens "the Holm family", but **the family is not
defined anywhere in this repository**. H1 is cited in the requirements and H6 was added "to the
hypothesis list" by amendment A3, but the list itself — H1–H5, the comparisons they make, the
family Holm is applied over — lives outside `preregistration/`.

So the preregistration as it stands **locks the gate thresholds but not the confirmatory
hypotheses or the multiplicity correction**: the part of a study plan that most needs locking.

**Recommendation: copy H1–H5, their comparisons and the Holm family into
`preregistration/endpoints.yaml` before the freeze**, with H6 kept exploratory and outside the
family as A3 has it. A test already refuses to let the preregistration be frozen over a
placeholder, so this cannot be skipped by accident.

**What I need from you:** where the current hypothesis list is written down, so it can be copied
exactly rather than reconstructed.

---

## One behaviour to be aware of — no decision needed

The restart condition means a stopped robot **cannot reverse while an intrusion sits within
0.31 m ahead of it**: the moving field's rear margin is `margin` (0.10), larger than its abeam
margin, so the intrusion is inside the field of the direction it wants to travel. It leaves
**abeam** instead, which is the departure a narrow aisle needs. This reads like a bug and is
correct under ruling 3 as worded; both behaviours are pinned by tests so neither is "fixed" later
by accident. If you intended "never drive toward an intrusion" to allow driving directly away,
that is a change to the restart condition, and I would want it as a ruling.

## What is waiting on these

Decisions 3–5 complete `endpoints.yaml`; Decisions 1–2 are recorded in the config. Then the freeze,
then **G2 on the training host**, then the tune and test scenario sets, then SAC.
