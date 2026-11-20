
---

## A13 - 2026-09-29 - Route-relative unmapped obstacles; the tune and pilot sets

**Decided by the supervisor on 2026-09-29** (rulings on the scenario-sets request). **Made before
any scenario set exists and before any policy trains.** The measurements that prompted it were
taken on pilot seeds only; no test seed has been generated.

**What changed.** A new file, `preregistration/scenario_sets.yaml`, and one change to the frozen
scenario module, `pfbnav/world/scenarios.py`, which is why the lock moves with this amendment.

**Placement.** Unmapped obstacles were placed uniformly over the hall's free floor before any route
existed. Measured on 40 pilot scenarios per stratum at `unmapped_static`, only 8-15 % put any
obstacle within 0.3 m of the route (narrow 15 %, medium 8 %, wide 12 %). H1's entire population is
`unmapped_static`, so roughly 88 % of it would not have tested the manipulation its condition is
named for.

Placement is now **route-relative in every split** -- train, tune and test -- so training sees the
distribution it is tested on. After the route is planned, each obstacle's offset to its nearest
**surface**, never its centre, is uniform on [0, 0.6] m; its side is uniform left or right; its
position is uniform over the route's arc length, excluding the first 2 m and the last 1 m. An
obstacle is kept only if the goal remains reachable in the true world -- mapped plus unmapped -- at
the lethal inflation radius. A refused obstacle is **re-sampled, never the layout**; if a route
cannot hold its obstacles within 200 draws, the next endpoint pair of the same layout is tried. The
rejection rate is logged per stratum and by cause. Because placement now depends on the route, each
scenario records its unmapped obstacles' geometry, and its `layout_sha256` covers them.

**The validity check.** At least 90 % of `unmapped_static` test scenarios must have an obstacle
surface within 0.3 m of the planned route, verified by a dry run before any test set is locked; the
build script refuses to lock a set that fails it.

**Option (c) was rejected** -- keeping uniform placement and discarding scenarios whose obstacles
landed far from the route -- because it would select layouts, not only obstacle positions.

**The tune set.** 20 scenarios per stratum, at the tune rate of 0.25, with route-relative placement.

**The pilot.** 150 **distinct** narrow scenarios from the pilot seed range, at `unmapped_static`.
Each learned method runs scenario *i* with training seed *i* mod 5, so 30 scenarios per seed and
150 distinct scenarios per method; each classical baseline runs all 150 once. This corrects the
construction implied by "30 per seed": deterministic baselines on 30 scenarios would give only 30
episodes, and a learned method's episodes sharing a scenario are clustered, which the
Clopper-Pearson bound -- a bound on independent trials -- does not allow.

**Not in this amendment.** The test-set size is **A14**, fixed after G2 has run on the training host
(on pilot seeds) and after the Monte Carlo of H1's power. The order is recorded in
`docs/decision_log.md`.

**One inconsistency, resolved before commit (supervisor, 2026-09-30).** As first ruled, every
offset is uniform on [0, 0.6] m, which engages only 62.5 % of scenarios with 1-2 obstacles
(1 - (0.5 + 0.25) / 2) -- below the 90 % check by construction. The two parts of the ruling had
not been checked against each other. Resolved by option (a): **the first obstacle's offset is
uniform on [0, 0.3] m** (`first_obstacle_offset_max_m: 0.3`) and any further obstacle's on
[0, 0.6] m, so every scenario is engaged and the 90 % check stands. Two conditions:

* **"First" is an index, not a location.** The capped obstacle's along-route position, side and
  size are drawn exactly as for the second obstacle, independent of the cap; otherwise close
  obstacles would sit systematically at one end of the route. A known-answer test pins this
  (with the same random stream, the cap changes the offset and nothing else), and the dry run
  reports where each index came to rest, since feasibility refusals can still shift it.
* **The offset analysis is descriptive and exploratory**, reported against the per-scenario
  minimum surface offset, and is not a confirmatory endpoint. Its distribution is shaped by
  design: the cap guarantees one obstacle within 0.3 m.

The same rule applies in train, tune and test.
