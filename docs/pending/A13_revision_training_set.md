

### A13, revised 2026-10-03 - The training set, and the tune-unmapped measurement set

**Decided by the supervisor on 2026-10-03**, before any policy trains. `preregistration/
scenario_sets.yaml` gains two blocks under `scenario_sets`; nothing else in A13 changes.

**The training set.** Equal thirds by stratum, 667 scenarios each (2,001 in all), from the train seed
range at the preregistered train rate of 0.25, with A13's route-relative placement and bypass rule.
**Training seed k visits the whole set in its own permutation, and every method's seed k uses that
same permutation**: seeds carry all training randomness, including order, or the method-level
inference -- seeds as the replicates of a method -- is miscalibrated. **No curriculum in v2.** The
permutation is a function of k alone (`pfbnav/training/order.py`), and every training run records
its hash.

**The tune-unmapped set** is disposable: 20 scenarios per stratum from the tune seed range at an
unmapped rate of 1.0 -- the H1 population's -- for the measurement the extended decision tree plans
on (Step 0): the seed CV, the stop rate and the inter-comparator gaps, at conservative 80 % bounds.
The tune sets' own rate of 0.25 would dilute the stop rate relative to `unmapped_static`. It is
allocated after the existing tune sets, so no tune seed moves, and its building is logged as a tune
regeneration.
