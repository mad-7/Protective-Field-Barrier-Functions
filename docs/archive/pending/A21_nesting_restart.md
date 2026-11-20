## A21 - 2026-10-19 - Nesting for stops begun in motion; a rigid-body restart condition; the narrow floor by rule

**Decided by the supervisor on 2026-10-19** in two rulings, both recorded in `docs/decision_log.md`
before any change. No test layout was built, read or used. No method has been trained or evaluated.

**Erratum to A20:** A20's text names its narrow-floor run as `runs/analysis/NARROW_FLOOR_A20_20261019T221401Z_seed0`. The
run is `runs/analysis/NARROW_FLOOR_A20_20261018T202400Z_seed0`.

**Why.** A20's containment test (2026-10-19) held containment at every stop, but a spinning stop
ended 3.54 mm inside the minimum margin.
- The at-rest field the robot stops in carries its own reach, so after a turning stop it reached
  beyond the field in force at the trigger.
- A stop could create its own deadlock in a field that was clear when it triggered.
- The restart condition ignored rotation entirely.

**What changed.**

1. **Nesting (S2).** Every stop that begins from a MOVING state (translating or spinning) ends with
   the at-rest field inside the field in force at the trigger, at every reachable rest pose.
   - **The moving field carries:**
     - the at-rest field's own rotational reach on top of the reachable turn, so the yaw interval
       is +-(e(|omega|) + e(0));
     - at least the at-rest reach in every direction;
     - a term the exhaustive nesting test showed necessary, the drift of a body that turns while it
       translates, D(V) * 2 sin(e(|omega|) / 2) (a holonomic base's command turns with the body).
   - **At-rest states are excluded.** In a static world a stationary robot with a clear at-rest field
     cannot be triggered by static obstacles. An at-rest trigger needs something entering the field
     (dynamic, out of scope), where holding the latch is correct.
     - A counter of at-rest triggers is kept in every static-world run (G4 controls, pilots,
       evaluation). Expected zero; any occurrence is reported as a construction gap.
   - **At-rest classification needs both the estimate and the applied command below the standstill
     threshold, rotation included.** The rotational thresholds are the translational ones over the
     footprint's outermost radius, so a spin is a moving state.
   - **The supervisor's first wording was impossible, and the error was the supervisor's.** It would
     have required the at-rest field, which carries the reach from rest, to nest inside itself
     shifted by that reach.
   - **S1 was rejected:** releasing on a standing field leaves the obstacle in the reach shell, and
     any first move re-triggers.
2. **The restart condition in rigid-body form.** A restart is permitted only if BOTH rules permit it:
   - **the centre rule:** v . p <= 0 for every breaching point, along its bearing;
   - **the closest-point rule:** (v + omega x q) . (p - q) <= 0, with q the footprint point nearest
     p, in the leading field for the commanded twist (its rotation sweep included).
   - It is never more permissive than the 2026-09-28 rule, and stricter in about 2.5 % of random
     translation cases, each a genuine sideways approach. This supersedes "reproduces the current
     rule exactly".
3. **The narrow floor by its rule.**
   - F = the smallest per-side clearance with all three D2 gaps >= 0 (binned) under the field
     above, + 0.03 m: **0.1687 + 0.03 = 0.1987 m.**
   - **The strata: narrow [0.1987, 0.2687], medium [0.2687, 0.40], wide [0.40, 0.80].**
   - The ruling named 0.1926, computed before the coupling term.
   - Run: `runs/analysis/NARROW_FLOOR_A20_20261019T221401Z_seed0` (the script keeps its A20 name).

**In the preregistration** (copies of the frozen files with only these edits, to apply verbatim):
- `endpoints.yaml`: H2's population becomes "narrow (0.1987-0.2687 m per side; A21)".
  File: `docs/pending/endpoints_A21.yaml`.
- `scenario_sets.yaml`: the bypass clearance's stated value becomes 0.1987 m (read from the strata;
  only the text changes). File: `docs/pending/scenario_sets_A21.yaml`.
- **`scenarios_sha256` changes** (`certified.py` and `strata.py` are in the generator's closure),
  and the strata change the narrow and medium layouts, so **the sets are REBUILT.** The lock is
  renewed (`--relock A21`).

**Checks, in the ruled order:** the rebuild and the oracle residual check; the containment test in
the simulator, with its margins-removed control declared before running; then the G4 controls.
