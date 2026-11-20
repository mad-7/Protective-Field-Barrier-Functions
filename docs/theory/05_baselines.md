# 5. Baselines and why they are used

All baselines share: the same A* route, the certified layer beneath, the 120-bin scan stream,
the same actuation box and wheel budget, the declared classical recovery, a map-based route
speed cap, and an equal tuning budget (N = 24 random-search candidates on the tune set only).

## Classical controllers (no learning)

| ID | Method | Why it is a baseline | Tuned parameters (see `configs/baselines/`) |
|---|---|---|---|
| B1 | Holonomic Pure Pursuit | The industrial status quo: a path tracker that relies on the safety scanner to stop it | cruise_speed, k_omega, goal_slowdown, lookahead |
| B2 | Regulated Pure Pursuit (Nav2 style) | The standard open-source tracker that slows on curvature and near obstacles; the host for B7 | desired_speed, lookahead_time, min/max_lookahead, min_radius, proximity_distance, approach_distance, min_speed, min_approach_speed, k_omega, proximity_gain |
| B3 | Holonomic DWA | The standard sampling local planner (Nav2 DWB defaults as candidate 0) | max_speed, window_time, sim_time, safety_margin, clearance_cap, w_path, w_heading, w_clearance (samples and granularity fixed) |
| B6 | Field-aware MPPI | The strongest hand-designed alternative: it avoids the certified layer's own fields in its cost, answering "why not just plan around the field?" | temperature, noise_std (3), cost weights (4); samples 1024 and horizon 20 fixed by the 50 ms real-time measurement |

## Classical + PFB (the integration study)

| ID | Method | Purpose |
|---|---|---|
| B7 | B2 + PFB | Attribution: what the PFB adds on a standard tracker (H5) |
| B8 | B6 + PFB | PFB on the strongest planner (exploratory) |
| B9 | B3 + PFB | PFB on a host that completes routes but still stops (exploratory) |

PFB parameters are the frozen ones (`filter:` in `configs/base.yaml`); the host parameters
are the tuned ones, unchanged.

## Learned controllers (SAC, shared hyperparameters)

| ID | Method | Why |
|---|---|---|
| L1 | SAC with proximity shaping, no filter | Standard learned local planner |
| L2 | SAC-Lagrangian, cost limit on stops | Learned method that treats stops as a constraint, the strongest learned alternative to the PFB |
| L3 | L1 wrapped post hoc by the PFB | Separate, labelled baseline: filter not seen in training |
| L4 | SAC trained with PFB, with shaping | Ablation of shaping |
| L5 | SAC trained with PFB, no shaping | The proposed method |

Retired IDs (B4, B5) stay retired and are never reused (`configs/methods.yaml`).

## Why this set

- B1 to B3 and B6 span tracker, regulated tracker, sampling planner and optimisation-based
  planner: the classes a practitioner would pick.
- B6 and L2 are the two baselines most likely to reduce stops without a barrier filter, so
  they bound the claim.
- B7 to B9 isolate the PFB's effect: same host with and without it.
