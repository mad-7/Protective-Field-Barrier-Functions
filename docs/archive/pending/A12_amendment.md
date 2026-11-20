
---

## A12 - 2026-09-28 - Endpoints, the confirmatory hypotheses, and the narrow-stratum rule

**Decided by the supervisor on 2026-09-27 and 2026-09-28** (ruling 5 on the narrow stratum, and
rulings D3-D5 on the before-the-freeze request). **Made before any policy has trained**: no
controller exists yet, so no method's behaviour can have informed it.

**What changed.** A new file, `preregistration/endpoints.yaml`, copied exactly from the
supervisor's consolidated text. It brings into the preregistration what had lived only in the
research plan: the confirmatory hypotheses **H1 and H2**, the conditional co-primary **H1N**, the
**Holm** family they form, the exploratory hypotheses H3-H6, the test (two-sided Wilcoxon signed
rank, paired by scenario), the unit of analysis, and the reporting rules. Until now the
preregistration locked the gate thresholds but not the hypotheses or the multiplicity correction,
which are the part of a study plan that most needs locking.

**Two clarifications ruled with it** (2026-09-28, D5):

1. **H1's comparators are B1-B4 and L1-L3.** B5 belongs to H5 and L4 to H2, and neither is an H1
   comparator.
2. **H1 and H1N use intersection-union logic**, with Holm applied **only across hypotheses**. Every
   comparison within H1 must reject at alpha, with no correction inside H1; the family Holm
   corrects over is {H1, H2}, widened by H1N only if the narrow rule triggers.

**The narrow-stratum rule.** Pilot the narrow stratum with **150 narrow episodes per method** on the
pilot seed range, **30 per seed** (five seeds). For each method in the confirmatory comparisons --
**B1, B2, B3, B4, L1, L2, L3, L4, L5**; A1 is excluded as exploratory -- compute the **exact one-sided
95 % Clopper-Pearson upper bound** on the proportion of `unrecoverable_stop` outcomes. If that bound
is **at or above 0.02 for any listed method**, H1N joins the confirmatory family as a co-primary
(narrow completion rate, with distance to first stop as its secondary). **At n = 150 the rule passes
only with zero events**: zero in 150 gives an upper bound of 1.98 %, one in 150 gives 3.12 %. The
pilot runs after the methods exist and before any test episode is evaluated.

**Why an upper bound rather than the observed proportion.** The two errors are not symmetric.
Adding a co-primary that was not needed costs some power; withholding one that was needed leaves a
compressed primary unprotected, which is the failure the rule exists to prevent. A pilot of 150 at
the observed proportion would have kept the sole primary with 2 events, which cannot exclude 4.14 %.

**Why any method rather than the pooled rate.** Endpoints must be identical across methods, and one
method's unrecoverable stops must not be averaged away by the others'.

**What was already known at the time.** That the at-rest field had been corrected to the infimum of
the moving fields (2026-09-27), removing the narrow-aisle deadlock in geometry; that the
traversability precondition passed on the real scan with an unavoidable-stop floor of **0.0 per
100 m in every stratum**; and that D2 re-measured 69.5 mm of margin at rest and 19.4 mm at the
tracking bound. On that evidence an unrecoverable stop needs something the precondition does not
model -- an unmapped obstacle, or a controller that stops with an intrusion inside the minimum
margin -- so the rule may well not trigger. It is declared anyway, because whether it triggers is a
fact about the methods, and the methods do not exist yet.
