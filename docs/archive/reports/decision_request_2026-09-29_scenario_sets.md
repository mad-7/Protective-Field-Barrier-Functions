# Decision request — the scenario sets (2026-09-29)

The preregistration is frozen and G0 passes with all six cases, so the next step is to generate
and lock the tune and test sets. The script exists (`scripts/build_scenario_sets.py`); it reads
seed ranges and rates from the frozen preregistration and **refuses to run until two decisions
are made**. Both change the test sets, and the preregistration is frozen, so both go through one
amendment (A13) and a re-lock.

Nothing has been generated. Every measurement below used the **pilot** seed range; no test seed
has been touched (hard rule 2).

---

## Decision 1 — unmapped obstacles rarely meet the route

Unmapped obstacles are placed uniformly over the hall's free floor, *before* any route exists. A
route is a thin line through a 14 × 10 m hall, so most obstacles never come near it. Measured on
40 pilot scenarios per stratum at `unmapped_static` (rate 1.0):

| stratum | scenarios with ANY unmapped obstacle within 0.3 m of the route | within 0.6 m | nearest obstacle, median |
|---|---|---|---|
| narrow | **15 %** (6/40) | 22 % | 1.75 m |
| medium | **8 %** (3/40) | 12 % | 2.17 m |
| wide | **12 %** (5/40) | 15 % | 2.16 m |

0.3 m is roughly where an obstacle enters the robot's protective field as it follows its plan
(half-width 0.21 m plus the 0.05 m abeam margin).

**Why it matters.** H1's whole test population is `unmapped_static`. In about **88 %** of it the
robot never meets an unmapped obstacle, so H1 mostly measures behaviour on mapped geometry, and
the `unmapped_static` condition mostly behaves like a second `mapped_only` condition. The
2026-09-23 ruling to log each obstacle's offset anticipated reporting H1 against offset. That is
still possible, but the engaged subgroup would be small: about 1 scenario in 9.

**Options.**

- **(a) Keep uniform placement.** No amendment, but the unmapped condition is diluted as above.
- **(b) Place unmapped obstacles relative to the route, in every split.** After planning, put the
  obstacles at a declared distribution of lateral offsets along the route — for example uniform
  on [0, 0.6] m — keeping the existing rules of at least 2 m from the start and 1 m from the
  goal. This changes the frozen generator (A13, re-lock). It must apply to train and tune too, or
  the policies would train on a different obstacle distribution from the one they are tested on.
- **(c) Keep placement, but reject `unmapped_static` scenarios whose nearest obstacle is too far.**
  Also a code change, and it selects *layouts* as well as obstacle positions, which is a subtler
  bias than (b).

**Recommendation: (b).** The condition would then test what it is named for, by construction, and
H1 can still be reported against offset — now across a declared range rather than whatever
uniform placement happens to produce.

## Decision 2 — the set sizes

**They are not preregistered anywhere.** A test set's size fixes the power of H1 and H2, so the
script will not choose it. What the choice has to balance:

**Compute.** Plans average 10.6 m, about 600 steps per episode at the realised 0.35 m/s. Each test
scenario is run by the confirmatory methods B1–B4 once and L1–L5 with five seeds each: 29 episodes.

| per (condition × stratum) cell | test scenarios | confirmatory evaluation steps |
|---|---|---|
| 30 | 180 | 3.2 M |
| **50** | **300** | **5.3 M** |
| 100 | 600 | **10.6 M** |

The requirements budget **5–9 M steps for all evaluation**. These figures are a lower bound: they
assume episodes that never stop, and stopped or timed-out episodes run longer. 100 per cell does
not fit before any exploratory analysis is counted.

**Power for H2** (Monte Carlo of the frozen analysis: paired Wilcoxon on seed-averaged success,
worst-case Holm level α/2, and "difference ≥ +15 pp AND significant"):

| per cell (H2 pairs) | true +15 pp | true +20 pp | true +25 pp |
|---|---|---|---|
| 30 (60) | 0.52 | 0.92 | 1.00 |
| 50 (100) | 0.48 | 0.96 | 1.00 |
| 100 (200) | 0.50 | 0.99 | 1.00 |

**A property of H2's frozen criterion that no sample size can fix:** if the true effect is exactly
+15 pp, the observed difference falls below +15 pp about half the time, so power stays near 50 %
at any size. More scenarios only buy power for true effects above the threshold. This is not a
proposal to change H2 — the criterion is ruled and frozen — but it is worth knowing when reading
the result. The model behind these figures is deliberately simple (L4 success uniform on
[0.3, 0.8] per scenario), so treat them as orders of magnitude, not guarantees.

**Recommendation:**

- **Test: 50 per cell** — 300 test scenarios; H1 has 150 pairs pooled over strata, H2 has 100.
  It fits the budget at 5.3 M steps, with the pilot (about 0.8 M) and exploratory analyses still
  to be costed against the remainder.
- **Tune: 20 per stratum** — disposable, rate 0.25 as preregistered.
- **Pilot: the 30 narrow scenarios ruled in D3, at `unmapped_static`** — an unrecoverable stop
  needs something to stop for. Please confirm the condition; the ruling fixed the size but not
  the condition. This also depends on Decision 1: under uniform placement only about 15 % of
  narrow pilot scenarios would put an obstacle in the robot's way.

---

## What A13 would contain

A `scenario_sets` block — test per cell, tune per stratum, pilot size and condition — plus, if
Decision 1 is (b), the placement rule and its offset distribution. The researcher commits it and
re-freezes with `scripts/freeze_prereg.py`. Then one command builds everything and locks the test
sets. A `--dry-run` first writes the sets under `runs/diag/`, unlocked, reporting yield and
engagement for every set, so the plan can be checked before anything is locked.
