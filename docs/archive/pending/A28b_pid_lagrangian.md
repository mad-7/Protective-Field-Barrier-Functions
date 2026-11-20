## A28b - 2026-10-27 - L2's multiplier: PID-Lagrangian, three declared gain candidates, selection by the toy control

**Decided by the supervisor on 2026-10-27** (ruling on the L2 toy control's FAIL), recorded in
`docs/decision_log.md`. No test layout was built, read or used. No method has been trained, tuned or
evaluated. The PID candidates below were declared before any toy run with them; the only toy run with
candidate 1 was the fast suite's own 1,000 steps, a regression test, not the control.

**Why.** A28a's projected dual ascent failed the toy control: at 5,000 steps, seeds 0-3 (commit
`171f366`), the last 20 episodes gave 0.00, 3.20, 0.60 and 1.35 stops per episode against
<= 1.25 at d = 1. It met d on average (0.86-1.06 over episodes 250-1,250), but it ran a limit cycle,
with a period of ~300-400 episodes and lambda between 0.01 and 0.27. Only 69-77 % of 20-episode
windows met 1.25 d. The last iterate -- the policy that is evaluated -- therefore depended on the
cycle's phase.
- Option (a), accepting this, was rejected: L2's final checkpoint would depend on the phase, which
  biases H1.
- Option (c), judging on a time average, was rejected: it rewrites a declared criterion after a
  failure, and the evaluated policy is the last iterate.

**What changes** (`endpoints.yaml`, `declared.L2_multiplier`, new `declared.L2_pid_candidates`):

1. **The update** -- a PID-Lagrangian (Stooke, Achiam and Abbeel, ICML 2020), once per completed
   episode k, from c_bar_k, the mean protective stops per episode over the last 20 completed
   episodes:
   - e = c_bar_k - d;
   - I = max(0, I + e), the integral projected as in Stooke et al.'s Algorithm 2, I_0 = 0;
   - lambda = min(100, max(0, K_P e + K_I I + K_D max(0, c_bar_k - c_bar_{k-1}))).

   lambda_0 = 0. lambda is held, with I = 0, until 20 episodes have completed. The first update has
   no derivative term (there is no previous mean). The per-gradient-step update (eta 3e-4) is retired.
2. **Three candidates**, (K_P, K_I, K_D) in declared order:
   - 1 = (0.1, 2e-3, 0), PI;
   - 2 = (0.1, 2e-3, 1.0), PID;
   - 3 = (0.2, 2e-3, 1.0), PID with the stronger proportional term.

   Units: reward units per stop per (stop per episode).
   **Derivation** (as ruled: the integral alone reaches the oscillating run's level of ~0.1-0.3
   within ~100 episodes):
   - In the A28a runs the summed excess over the first 100 updates was 88-116 (mean ~104), so
     K_I = 0.2 / 104 ~ 2e-3. Its lambda's 90th percentile was 0.18-0.20.
   - K_P: a typical excess (|e| median 0.9) moves lambda by 0.1 or 0.2 at once.
   - K_D: the 95th-percentile rise of c_bar per episode (0.1) adds ~0.1, while the mean rise (0.02)
     biases lambda by only ~0.02.
3. **Selection.** The toy control is rerun per candidate: 5,000 steps, seeds 0-3, criteria
   unchanged. Those criteria are: the last 20 episodes at most 1.25 d at d = 1 and at most half of
   plain SAC's; plain SAC at least 2 d; lambda in [0, 100].
   - L2's gains are those of the FIRST candidate, in declared order, that passes on all four seeds.
   - The fraction of 20-episode windows meeting d is reported.
   - If none passes: stop and report.
   - The same gains apply to L2, L2-d005 and L2-d020. Until the selection, every L2 config's
     `pid_candidate` is OPEN, and training refuses it.

**Note for the record.** The ruling writes the integral as K_I sum(e). Stooke et al.'s Algorithm 2
projects the integral onto >= 0 at every update (I <- (I + e)+). That projection is implemented, as
"Stooke et al.'s form". Without it, the integral would wind below zero during long stretches under
the limit, and lambda would stay at 0 until it unwound. This is the windup that the cap at 0 already
prevented in A28a's dual ascent. The reference is in `paper/MDPI_template/references.bib`, marked
VERIFY.

**What does not change.** d = 0.1 a priori; L2-d005 and L2-d020 exploratory and first in the cut
order; the 20-episode window; lambda_max 100; units consistency with d; everything else in A28 and
A28a.
