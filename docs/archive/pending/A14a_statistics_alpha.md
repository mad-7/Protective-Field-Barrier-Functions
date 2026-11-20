## A14a - 2026-10-26 - The statistical analysis; the PFB gain's grid and selection rule

**Decided by the supervisor:**
- the statistical analysis on 2026-10-02 and 2026-10-03 (rate ratio, two-level bootstrap, the
  correction, the degrees of freedom, the decision tree);
- the alpha grid and its selection rule on 2026-10-24;
- the instruction to preregister both, as A14a, before any tuning, on 2026-10-26.

All are recorded in `docs/decision_log.md`. **A14b** (the seed count and the test size) follows,
from the decision tree after Step 0 and G2. No test layout was built, read or used. No method has
been trained or evaluated.

**Why.** `endpoints.yaml` still specifies a two-sided Wilcoxon signed-rank test, paired by scenario,
with learned methods seed-averaged over five seeds and a paired detectability floor in H1's success
criterion.
- The H1 power analysis (2026-10-02) found that this test leans against a seed-averaged L5 when the
  comparator runs once.
- It also found that shared seed effects inflate learned-versus-learned null rejections.
- The supervisor ruled its replacement, and the replacement passed calibration (2026-10-03).

**What changes.**

1. **H1's estimand: the rate ratio.** For L5 over a comparator: (L5's total protective stops / L5's
   total distance) / (the comparator's total stops / its total distance).
   - Scan-fault stops are excluded (A25).
   - 0.5 is added to each arm's total stops (a declared continuity correction).
2. **The interval.** A two-level bootstrap of the log rate ratio:
   - scenarios are resampled jointly across arms (the pairing);
   - a learned arm's seeds are resampled within the arm, independently of every other arm's;
   - a classical arm resamples scenarios only.

   Its variance is computed in closed form for the linearised statistic (`pfbnav/eval/rate_ratio.py`;
   checked against brute-force resampling, and against the full bootstrap within -2 % to +7 %).
   - **Correction:** each learned arm's seed part of the variance is scaled by K/(K-1), and the
     interval uses a t quantile.
   - **Learned against classical:** df = K - 1 of the learned arm (flat; adopted 2026-10-03 after
     passing calibration over the whole grid).
   - **Learned against learned:** Welch-Satterthwaite df over the scenario part (n - 1) and each
     seed part (K - 1).
   - **Declared fallback, not in force:** seeds as units (per-seed log rate ratios against the
     classical arm over the same scenarios, a one-sample t with K - 1 df). If that fails
     calibration too: stop and report.
   - A comparison rejects in L5's favour when the two-sided interval at the comparison's Holm level
     lies wholly below 0 on the log scale.
3. **The confirmatory claim is method-level** (the two-level interval). The policy-scoped interval
   (seeds held fixed) is reported as secondary, never substituted.
4. **H1's logic is unchanged:** intersection-union over B1, B2, B3, B6, L1, L2, L3. Every comparison
   must reject; there is no correction within H1.
   - **The detectability floor is removed from H1's success criterion.** It is redundant given a
     valid test, and kept as a reported quantity: the 95th percentile of the effect under
     within-scenario arm-label swaps.
5. **H2 inherits the analysis.** The estimand is the narrow-stratum success-rate difference, L5 minus
   L4, with the same two-level corrected interval on the mean; L4 and L5 train at the same seed
   count, so it is Welch-Satterthwaite. H2's success criterion is unchanged: a difference of at
   least +15 pp, and significant after Holm.
6. **Family and level:** H1, H2, and H1N if the narrow rule adds it, under Holm's step-down at
   familywise alpha = 0.05. Power is planned at the most stringent step, alpha/3.
7. **The decision tree** (2026-10-03, extended) decides A14b's seed count and test size, and is
   preregistered here as written in the decision log:
   - **Step 0 (planning inputs):**
     - measured on the disposable tune-unmapped set: the stop rate at its lower 80 % bound, the seed
       CV at its upper 80 % bound, and the comparator gaps at the bound nearer the margin;
     - the effect of interest stays a 30 % reduction against the closest comparator;
     - L5's own measured effect is never a power input.
   - **Branch 1:** corrected, 10 seeds, power >= 0.80 at <= 100 per cell gives 10 seeds.
   - **Branch 2:** else, a count <= 20 reaching 0.80, if G2 fits.
   - **Branch 3:** else, H1's method-level claim is restricted to the classical comparators
     (B1, B2, B3, B6; B4 retired by A16), L1-L3 are reported policy-scoped, and H1 is amended.
   - **Branch 4:** sizes {150, 200} per cell for the unmapped-static cells only, subject to G2.
   - **Branch 5:** if nothing affordable reaches 0.80:
     - H1, its effect size and the analysis stay unchanged;
     - the achieved power is preregistered in A14b;
     - a non-significant result is inconclusive;
     - the rate ratio with its interval is the primary result.
   - **H2:** stays method-level at the chosen seed count, with its achieved power preregistered.
   - **Seed count:** L5's count is the larger of H1's and H2's requirements, capped at 20, subject to
     G2. L4 trains at the same count.
8. **The PFB gain alpha** (ruled 2026-10-24), tuned before L5's pilot and then frozen for L3, L4,
   L5 and B7:
   - **Grid:** {1, 2, 5, 10, 20} 1/s.
   - **Comparator:** B7 (B2's tuned parameters plus the PFB, nothing else).
   - **Set:** the tune set at the training unmapped rate.
   - **Selection:** among the settings whose success rate is within 5 percentage points of the
     best, the fewest protective stops per 100 m; ties broken by mission time.

**In the preregistration:** `endpoints.yaml` only (its `test`, `unit`, `reporting`, H1's `success`
and an `analysis` and `tuning` block), as in `docs/pending/endpoints_A14a.yaml`. `gates.yaml` is
unchanged. No generator or scenario code changes, so nothing is rebuilt. The lock is renewed
(`--relock A14a`).
