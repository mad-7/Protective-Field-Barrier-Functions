# Related-work map: where the PFB sits

Found with Consensus on 2026-11-23. Papers read in full are listed in `README.md` (the four earlier ones plus six read
on 2026-11-23: Olayemi 2026 PA-CBF, Thumm 2023, Mestres, Oh 2026, Muhammad 2021, Liu 2017, Song 2022); every other paper below is
abstract level only, so a claim about it is a claim about its abstract. Seven searches were run, three rate-limited; none
is a systematic review. Titles link to Consensus pages.

## The claim this map tests

A continuous, speed- and direction-dependent LiDAR barrier filter (the PFB) placed beneath any controller keeps it out of
a certified layer's latched protective stops, and the cost of those stops (availability) is the metric, beside success,
timeouts and mission time. Classical planners and a learned planner are both evaluated under the same filter.

## Clusters

| # | Cluster | Representative work | What it measures | Relation to the PFB |
|---|---|---|---|---|
| A | Discrete-time, beam-wise LiDAR CBF filters | [Olayemi et al. 2026, PA-CBF](https://consensus.app/papers/details/w:83693e3f-13a0-4505-b91b-c3b4fb201b09/?utm_source=claude_desktop); [Keyumarsi 2023](https://consensus.app/papers/details/w:8be64316-3e6b-4d89-9955-822cce149b06/?utm_source=claude_desktop); [Harms 2025, composite CBF](https://consensus.app/papers/details/w:2ece8c70-9204-4987-9513-b596d3a0de10/?utm_source=claude_desktop); [Xu 2025](https://consensus.app/papers/details/w:bad4f574-5ce7-4bc6-bc42-61689ce8b213/?utm_source=claude_desktop) | Collisions, smoothness, success | **Closest prior art to the filter itself.** PA-CBF uses beam-wise discrete-time barrier constraints on LiDAR. Our filter must be described as an application of this family, not a new barrier |
| B | CBF inside or around classical planners | [Exact-Safe MPPI 2026](https://consensus.app/papers/details/w:5f89bd10-816f-49cb-bdda-87fcd02c1640/?utm_source=claude_desktop); [GS-MPPI 2025](https://consensus.app/papers/details/w:a5b320d5-fca9-4283-a869-7ffaedee4878/?utm_source=claude_desktop); [D-CBF MPC 2023](https://consensus.app/papers/details/w:b510f500-f425-4e53-b987-297108a0f6ce/?utm_source=claude_desktop); [LiDAR CBF MPC 2025](https://consensus.app/papers/details/w:adcb1bd3-b33f-43a8-872d-5f64ff733d35/?utm_source=claude_desktop); [Safari 2026](https://consensus.app/papers/details/w:373e310f-4739-4437-86c5-7ccc10a82c5e/?utm_source=claude_desktop) | Collisions, goal-reaching, compute time | CBF is built into the planner. We keep the planner unchanged and wrap it; this is the contrast for B6 vs B8 |
| C | Known failure of CBF filters: stalls and undesired equilibria | [Mestres et al. 2026](https://consensus.app/papers/details/w:ff84f622-f586-4024-975f-cd86c1c295a1/?utm_source=claude_desktop); [Chen et al. 2026](https://consensus.app/papers/details/w:e8ff4d2e-76bc-4373-b9eb-c6ba9ac97289/?utm_source=claude_desktop) | Limit cycles, undesired equilibria | **Explains our B3+PFB stalls on unmapped obstacles.** We should cite it and report timeouts beside stops, not hide them |
| D | CBF / shield filters with RL navigation | [CBF-RL](https://consensus.app/papers/details/w:34afa5a8-83c4-47ad-b628-a1c168296b8e/?utm_source=claude_desktop); [DODGER](https://consensus.app/papers/details/w:ff768661-4f42-4929-bc27-70c2102e294c/?utm_source=claude_desktop); [Kong 2025](https://consensus.app/papers/details/w:8ff96af7-c4f7-45a0-8f2d-322b1f4ac07c/?utm_source=claude_desktop); [Marzari 2025](https://consensus.app/papers/details/w:a92fb836-33b9-4a4e-aae4-e9fb591be36d/?utm_source=claude_desktop); [Son 2026](https://consensus.app/papers/details/w:316b3256-c243-42b2-8df3-161f47afed2f/?utm_source=claude_desktop); [Dawood 2024](https://consensus.app/papers/details/w:e0fd0404-1072-4584-be02-b666e9f99c55/?utm_source=claude_desktop); [Guerrier 2025](https://consensus.app/papers/details/w:9c085229-688b-4bca-a545-2d43381cec93/?utm_source=claude_desktop) | Collisions, success, convergence speed | A crowded area. Filter-in-training is established. We add a latching certified layer beneath |
| E | Theory of RL under a safety filter | [Oh et al. 2026](https://consensus.app/papers/details/w:a9a27138-ed27-4b38-97f7-43c6379b8e45/?utm_source=claude_desktop); [Banerjee 2024, DMPS](https://consensus.app/papers/details/w:a4a9ae0e-cec8-4ca7-983a-fddd71c9ebf4/?utm_source=claude_desktop); [Kim 2024](https://consensus.app/papers/details/w:3a210fbd-f06f-4bf0-ae2f-dc0ee65e0a0e/?utm_source=claude_desktop) | Safety, asymptotic return | Supports the in-loop design: a sufficiently permissive filter need not cost asymptotic performance (Oh et al.). Gives a theoretical defence for L5 |
| F | Counting safety interventions | [Thumm et al. 2023](https://consensus.app/papers/details/w:aeddd4ee-f0ec-4bfa-b95e-4d38ee9851d4/?utm_source=claude_desktop) | Failsafe interventions (abrupt braking) | **Closest to our metric.** They reduce failsafe interventions in provably safe RL on Safety Gym and a human-robot task. Our setting differs: a latching safety-rated layer on an AMR with restart dynamics, and a stop metric per 100 m |
| G | Industrial safety fields and AMR safety | [Muhammad 2021, adaptive warning fields](https://consensus.app/papers/details/w:c4b23d4b-2004-422f-b637-f3e878151e34/?utm_source=claude_desktop); [Liu 2017](https://consensus.app/papers/details/w:61be2c6a-998c-4570-a906-589e12d2049b/?utm_source=claude_desktop); [Tiusanen 2020, standards](https://consensus.app/papers/details/w:dd12f78c-c731-4016-b216-148739dcc9dd/?utm_source=claude_desktop); [ILIAD](https://consensus.app/papers/details/w:3af9a130-a261-4c25-80ec-1af394ac379a/?utm_source=claude_desktop); [dead zones 2026](https://consensus.app/papers/details/w:03203de3-1474-429e-a856-335a5bc24ad8/?utm_source=claude_desktop) | Efficiency of fields, stopping, fleet downtime | Treat the protective field as given or shape it; none couples it to a CBF filter. Muhammad 2021 and Liu 2017 note that primitive safety fields cost efficiency, which is our motivation |

## What the map says about novelty

- **Not novel:** a discrete-time CBF filter on LiDAR (cluster A), a filter in the RL loop (cluster D), a theoretical case
  for it (cluster E).
- **Possibly novel, as far as these searches show:** (1) the protected object is a latching, speed-banded certified layer
  with a restart condition, and the metric is its protective stops per 100 m; (2) the same filter is evaluated under
  classical and learned controllers; (3) the finding that training beneath a latching layer collapses without the filter
  (L1 latched about half of all steps in the 100k smoke runs; exploratory).
- **Nearest threat to novelty:** Thumm et al. 2023 (intervention counts in RL) and PA-CBF 2026 (the filter). A reviewer who
  knows both will ask what the combination adds. The answer must be the certified layer and the availability evidence.

## Full-text findings (seven papers, 2026-11-23)

- **PA-CBF (Olayemi 2026) is the closest prior art to the filter.** Same row form, h_{k+1} >= (1 - gamma dt) h_k, beam-wise,
  QP with slack, SAC beneath. Differences: constant margin 0.3 m (no speed or direction dependence), unicycle, forward sector
  of 32 beams, range-rate and a predictive horizon, no certified layer and no stop metric; one table, 100 episodes, no
  intervals. It is unclear from the text whether the filter was in the training loop. Its code is public.
- **Thumm 2023 is the closest prior art to the metric.** Failsafe interventions, with long recovery from the stopped state,
  per episode; reward penalty was fragile (no effect at -0.01, shield avoided at -0.1); PID-Lagrangian failed on the harder
  task; 10 seeds, 2M steps, bootstrap intervals. Their shield uses exact obstacle positions, not LiDAR.
- **Mestres: filters can stall.** Undesired equilibria exist even for convex safe sets and can be stable; limit cycles in
  dimension above two. Continuous-time analysis, so our B3 + PFB stalls are explained by analogy, not proof.
- **Oh 2026: motivation, not guarantee.** The theorem needs a perfect, stationary, least-restrictive filter and safe
  starts. Ours is conservative (about 3% of steps infeasible in L5) and sits above a stateful latch. Their recipe, the most
  permissive filter, does not match our own alpha tuning (checked 2026-11-23 against `pfbnav/safety/pfb.py` and the decision
  log of 2026-11-03): the row is g >= (1 - alpha dt) max(h, 0), so larger alpha is less restrictive (dt 0.05 s, alpha 1 is
  0.95 and alpha 20 is 0). On B7, 60 tune scenarios, seed 0, grid {1, 2, 5, 10, 20}: stops per 100 m 0.59, 2.94, 9.63,
  11.98, 12.93, success 0.50 to 0.517, time 155 to 158 s. More permissive meant more stops and no visible gain; frozen at 1.0,
  the lowest value tried.
- **Muhammad 2021 and Liu 2017: industrial context.** Level-0 AGV safety is a static protection field with a discretised
  warning field and ISO 3691-4; efficiency is read as fewer slowdowns and stops (Muhammad) or higher average speed (Liu).
  Liu: production robots keep certified scanners because software safety is hard to demonstrate, which supports wrapping
  the certified layer instead of replacing it. Neither counts stops with numbers.

- **Song 2022: the closest prior art to L5's framing, not to its mechanism.** SAC with a CBF filter in the loop, LiDAR only,
  with SAC, Lagrangian SAC and a hand-crafted CBF as comparators, the same shape as our L1, L2, L5 set. The CBF is a learned
  linear function of reciprocal ranges fitted on collision labels (no geometric field), the action is forward-only, the
  metric is reward and collisions, and the evidence is 4,000 steps with 5 seeds in two corridors. No stops, no certified
  layer. Its 1/d scan representation targets the same near-range sensitivity as our `scan_exponent` variant.

**Consequences for the study:** describe the PFB as an application of the beam-wise discrete CBF family with a field that
mirrors the certified layer; add a constant-margin beam-CBF comparator (PA-CBF-like) to test that the speed- and
direction-dependent geometry is what removes stops; replace the near-100% "modified" share with an intervention magnitude;
meet Thumm's evidence standard (seeds, intervals, penalty sensitivity); keep the L1, L2, L5 comparator set (Song uses the same shape, so it is the expected one) and say how ours differs.

## Claims we can and cannot make

- Can: the filter reduces protective stops of the emulated certified layer, if the clean reruns confirm it, and success,
  timeouts and mission time are reported beside the stops.
- Cannot: a new barrier function, a collision-safety guarantee beyond the emulator, or superiority over other safe-RL
  methods until L2 and L3 are run.
- Must cite: Mestres 2026 for the stalls; Oh 2026 and CBF-RL for filter-in-training; Thumm 2023 for intervention counts;
  Olayemi 2026 (PA-CBF) as the beam-wise discrete-time filter family.

## Not yet searched (rate limit, then scope)

- Learned planners versus classical planners on stops or availability specifically.
- Safety-rated field standards beyond the review by Tiusanen (IEC 61496, ISO 3691-4 text itself).
- Whether anyone reports protective-stop latch dynamics for RL (my searches found none; absence in a thin search is weak
  evidence).
