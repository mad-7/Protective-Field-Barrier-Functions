## A16 - 2026-10-15 - Comparators: B4 and B5 retired; B6 (field-aware MPPI) and B7 (RPP + PFB) added

**Decided by the supervisor on 2026-10-15**, before WP3 baseline implementation and before any test
evaluation, so each controller is implemented and tuned once. (A14 stays reserved for the test-set
size.)

**Why.** Two gaps in the comparator set, raised before any baseline exists. (1) No classical
controller knew the certified fields, so H1 could not answer "could a hand-designed controller that
avoids the fields do as well without learning?". (2) The attribution analysis -- does the PFB on a
classical controller give the same benefit as training with it? -- used plain Pure Pursuit, not the
strongest reasonable nominal controller.

**What changed.**
- **B4 ("PP + field switching only", the industrial status quo) is retired.** With the certified
  layer beneath every controller it is identical to B1: a design error. **B1 carries the "industrial
  status quo" label** -- a path tracker relying on the safety scanners.
- **B5 (PP + PFB, no learning) is retired**, superseded by B7.
- **B6 = holonomic MPPI with a field-aware cost**, an H1 comparator in B4's place. The cost uses the
  certified layer's actual discrete band fields, not the PFB's continuous field: the hand-designed
  approach is to avoid triggering the fields that stop you, and the PFB field would make MPPI a soft
  PFB. **Declared now:** cost terms -- route tracking, goal progress, a field-breach penalty, hard
  rejection of rollouts intersecting scan points, control smoothness; horizon, sample count and
  temperature (initial values in `configs/base.yaml`, `baselines.mppi`). All tuned under the
  equal-tuning-budget rule on the tune set only. The same route, certified layer, 120-bin scan stream
  and classical recovery as every baseline. Implemented in our stack with vectorised rollouts, as DWA
  is; Nav2 is not run.
- **B7 = Regulated Pure Pursuit + PFB, no learning**, in B5's place: in H5 (exploratory) and in H3's
  granularity study. **H5 is extended to report protective stops as well as mission time**, since
  attribution concerns both.
- **Standing rule: method IDs are never reused** (`configs/methods.yaml`, CLAUDE.md hard rule 20).
  B4 and B5 were never implemented, but frozen documents name them; redefining them would make the
  record ambiguous.

**The classical set after A16**, from the research plan's experimental matrix (B2 and B3 confirmed by
the researcher, 2026-10-15): **B1** holonomic Pure Pursuit, the industrial status quo; **B2** Regulated
Pure Pursuit, Nav2-style, slowing on curvature and near obstacles; **B3** holonomic DWA with a
three-dimensional dynamic window over (vx, vy, omega); **B6** field-aware MPPI; **B7** RPP + PFB. B4
and B5 are retired and their IDs never reused. **All of them share** the same route, the certified
layer beneath, the 120-bin scan stream, the non-square actuation box with the wheel-budget
projection, the classical recovery behaviour, and an equal tuning budget on the tune set only.

**In `preregistration/endpoints.yaml`.** H1 and H1N comparators `[B1, B2, B3, B6, L1, L2, L3]`;
`narrow_rule.methods` `[B1, B2, B3, B6, L1, L2, L3, L4, L5]`; H5 = L5 versus B7, mission time and
protective stops.

**Power.** B6 enters the power analysis fully. H1 keeps seven comparators, so its structure and the
power analysis's shape are unchanged. Step 0 measures the inter-comparator gaps on the tune-unmapped
set including B6, so B6 must be implemented and tuned before that measurement (which waits on trained
learned seeds anyway). Making the strongest comparator secondary to protect power was considered and
rejected: it would leave the objection unanswered, and it is the scope-narrowing move branch 5
forbids. B6 is expected to become the closest comparator, so the 30 % effect of interest will be
measured against it.

**Not affected:** G4, whose positive control is B1.
