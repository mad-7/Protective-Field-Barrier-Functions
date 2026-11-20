# WP2 -- the Protective-Field Barrier (PFB): design note and decision request

Drafted 2026-10-23, after the freeze took effect (G4 controls sound). **No filter code is written.**
The filter equations need the supervisor's ruling recorded in `docs/decision_log.md` before any
implementation (hard rule 7). This note proposes the structure and asks seven questions (section 9).
Each comes with a recommendation.

## 1. Role, and what must stay true

**The PFB is the method's contribution.** It is a LiDAR-sector control barrier function whose zero
set is a CONTINUOUS, speed- and direction-dependent protective field. It sits in the step loop's
filter slot (`pfb_stage`, `pfbnav/envs/step_loop.py`) and modifies the nominal command minimally.
The certified-layer emulator still runs beneath it, unchanged.

**Invariants, from existing rules and rulings:**
- **Pure functions** in `pfbnav/safety/pfb.py`: no simulator, environment or `certified.py` import
  (hard rule 3, enforced by `tests/test_architecture.py`). The field construction is therefore
  written independently. TESTS may import both modules, and they cross-check the two (section 8).
- **Filter off is bit-identical** (`pfb_stage` returns the same object; the existing test).
- **The settled construction**, the same physics the certified layer was corrected to:
  - **reach:** A20, T = response time + one step, and the speed reachable within T;
  - **rotation sweep:** the footprint turned through the yaw reachable within T;
  - **nesting:** A21, the moving field carries e(0), the rest reach and the turning drift;
  - **lag-safe composition:** the speed and yaw rate that size the field are the larger of the
    estimate and the applied command (ruled 2026-10-16), and here also the candidate command.
- **The velocity is the estimated actual one**, never the command or odometry (ruling 3, D7/G1b).
- **Unobserved space is never free** (hard rule 5): an invalid bin is an obstacle at the footprint.
- **Actuation limits enter the QP** as the mecanum wheel-speed polytope, not a post-hoc clamp (A6).
- **Collisions are judged only by simulator contact** (hard rule 4); the PFB is judged by protective
  stops and contact.

## 2. Inputs, each step

- The 120-bin filter stream: ranges r_i, bearings psi_i and validity.
- The velocity estimate v_hat = (vx, vy, w).
- The command the base applied last step, u_prev (after the limiter).
- The nominal command u_nom from the controller (learned policy or classical baseline).
- The certified layer's latch flag, read-only (section 7).

## 3. The barrier

**One row per bin** within reach (bins further than the largest field reach are dropped):

    h_i(x, u) = r_i - rho(psi_i ; V(u), theta(u), w(u)) - delta_i

- **`rho(psi ; V, theta, w)`:** the boundary distance, along bearing psi, of the field for travel at
  speed V in direction theta (body frame) with yaw rate w. It is built exactly as the certified
  field's support function:
  - the swept footprint over +-(e(|w|) + e(0));
  - the travel term max(D(V)·lead, swerve);
  - the coupling term, plus the margins ahead and abeam;
  - the at-rest infimum.
  But V is the CONTINUOUS speed (see Q1), not a band top.
- **`V(u)`:** the speed reachable within T from max(|v_hat|, |u_prev|, |u|), i.e. reach_speed, A20.
- **`delta_i`:** the sector-binning penalty, if any (derived per the D2 analysis; measured about
  1 mm). Q6 asks whether to carry it.

**The discrete-time CBF condition** (dt 0.05 s):

    h_i(x_{k+1}, u) >= (1 - alpha·dt) · h_i(x_k, u_prev)

- r_i at k+1 is predicted from the candidate command: the point's body-frame position moves by
  -(u_xy + w × p_i)·dt.
- rho at k+1 is evaluated for u.
- alpha is the class-K gain, the only tuned PFB parameter (Q4).

**Why it is nonconvex, and what is done about it.** rho depends on u nonlinearly (speed, direction,
yaw rate). It is LINEARISED about the nominal command (one SQP step), the QP is solved, and the
TRUE constraint is checked at the solution. If any row is violated, one more linearisation about the
solution is made, then the fallback (section 6). Gradients come from analytic derivatives of the
support function and are verified by finite differences (Q3).

## 4. The QP

    min_u   (u - u_nom)^T W (u - u_nom)
    s.t.    the CBF rows (section 3), linearised
            the wheel polytope: |vx| + |vy| + (l + w_t)|w| <= 0.95 v_rim (the filter's share, A6)
            the axis box (v_max, vy_max, w_max)
            the limiter envelope: u reachable from u_prev in one step (a_max, a_decel_max,
            alpha_max), so the QP's output is exactly what will be applied

- W weights translation and rotation in comparable units (Q5).
- Solver: OSQP (`filter.qp_solver`), warm-started. Three to six variables and at most 120 rows, well
  within the budget; G2 measures the cost.

## 5. Bands: how the PFB relates to the certified layer (the central question, Q1)

The certified layer stops the robot whenever a valid range enters the field of the BAND its reachable
speed selects, a field sized for the band's TOP speed. A PFB that keeps only its continuous field
clear (rho at V itself) leaves a gap: between V and the band top, the certified field is larger, so
the robot can be stopped although the PFB is satisfied. The options:

- **(a) Continuous field at V.** The purest statement of the method, and the smoothest barrier. Stops
  remain possible from the band discretisation alone, and the PFB cannot claim "no stops" even in
  principle.
- **(b) The field of the band V selects** (piecewise). It dominates the certified field exactly, but
  is discontinuous in V. It can be handled exactly by solving one QP per band, with |u| restricted to
  that band's speed range, and taking the best feasible: at most three small QPs.
- **(c) A continuous envelope dominating the band fields:** smooth, and slightly more conservative
  than (b) near band edges.

**Recommendation: (b)**, solved per band. It is exact, the barrier then guarantees the certified layer
never triggers on observed static geometry (up to prediction error), and the "continuous field"
contribution stays in rho's dependence on direction and yaw rate within a band. (a) is kept as an
ablation if wanted.

## 6. Infeasibility, unobserved bins, the latch

- **Infeasible QP:** the fallback is the maximum-deceleration command toward zero within the limiter
  (and the polytope), flagged `pfb_infeasible` and counted per episode. The filter never claims a
  feasibility it does not have.
- **Unobserved bin:** it enters as r_i = 0, so its row is unsatisfiable and the QP falls back. That is
  consistent with the layer, which stops on it anyway.
- **While the certified latch is set** (Q7): the PFB passes only commands the restart condition could
  release (the creep regime, receding twists, A22/A24), or zero. It never pushes against a held stop.

## 7. What it reads from the certified layer

Only the latch flag, through the step loop. The layer stays a black box beneath (hard rule 3). The
PFB's parameters come from `filter:` in the config. Where a quantity is the SAME physical constant
as the certified layer's (a_max, alpha_max, a_decel_max, the footprint table, margins), it is read
from the same config key, never restated (rule 16). The response time is the PFB's own: one control
step, versus the certified 0.10 s (Q2).

## 8. Tests (before any training with the filter)

- **Known answers per term** of rho, and a CROSS-CHECK against `certified.FieldGeometry` in the test
  only: the PFB's band field equals the certified field at band tops, and its continuous field is
  never larger than the band field.
- **Finite-difference checks** of every analytic gradient (hard rule, mathematical code).
- **The QP:**
  - u_nom feasible means u = u_nom (minimal modification);
  - random scans never yield a violated TRUE constraint without the infeasible flag;
  - the polytope and the limiter envelope always hold.
- **Closed loop in the emulator:** an ideal tracker plus the PFB over the tune routes, with the
  certified layer underneath. Expected certified stops on static geometry: zero, under (b). Every
  stop that occurs is attributed (prediction error, unobserved bins, infeasibility).
- **The acceptance test of ruling 3:** the filter is fed a velocity that differs from the command,
  and its field follows the velocity.
- **The declared sensitivity check:** velocity noise and one-step latency (`estimation.velocity`).
- **Filter-off bit-identity** (exists), and filter-on determinism within one scene load.

## 9. Decisions requested

- **Q1. The field the barrier keeps clear:** (a) continuous at V, (b) the band field per band (three
  small QPs), or (c) a smooth dominating envelope. *Recommended: (b).*
- **Q2. The PFB's response time:** one control step (the filter acts on the next command), with
  T_pfb = dt + the estimate's lag. Or reuse the certified 0.10 s? *Recommended: its own, dt plus a
  declared lag allowance, with the sensitivity check covering the latency.*
- **Q3. The nonconvexity:** linearise about u_nom, check the true constraint, one re-linearisation,
  then the fallback. Or sample candidate directions? *Recommended: linearise and check.*
- **Q4. alpha:** one value for every method that carries the PFB (L5, B7, L3), tuned on the tune
  layouts under the equal-tuning-budget rule, before L5's pilot. *Recommended.*
- **Q5. W:** diag(1, 1, l²), so a unit of yaw rate costs what the footprint's lever arm moves.
  *Recommended.*
- **Q6. The binning penalty delta_i:** carry the derived sector-minimum term (about 1 mm), or none?
  *Recommended: carry it; it is a property of the sensor the barrier reads.*
- **Q7. Behaviour under the certified latch:** pass only commands the restart condition could
  release, else zero. *Recommended.*

**Order once ruled:** the rho construction with its cross-check tests; then the QP with known
answers; then the emulator closed loop on tune routes; then the simulator acceptance tests; then
alpha tuning; then L5's pilot (G4).
