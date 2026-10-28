# Realism audit of the certified-layer emulator

Started 2026-10-20, as ruled. The supervisor's three questions for each feature:

- What does the emulator do?
- What is its industrial counterpart (a safety laser scanner with switchable field sets, and a
  safety controller with speed monitoring)?
- Is the emulator **more conservative than hardware** (yes / no / equal / unknown)?

Values are `configs/base.yaml`'s; the reasoning behind each is in `docs/decision_log.md`.

**"Conservative"** means more stops, larger fields or later restarts than a certified installation
would produce. A "no" is a place where the emulator is kinder than hardware, and each one is listed
again at the end.

**Status: first pass.** Rows marked *open* have no evidence yet. They are filled in before the
write-up, and none of them changes the emulator (the freeze of 2026-10-20 admits only blocking
defects).

## Field construction

| Feature | Emulator | Industrial counterpart | More conservative than hardware? |
|---|---|---|---|
| Field sizing (A20) | Contains every footprint pose reachable before braking acts. Reach time T = response 0.10 s + one control step. The band's top speed sets the reachable speed (`v + a_max T`). Then braking at `a_brake` 1.0 m/s². | IEC 61496-3 / ISO 13855 protective-field sizing: S = K·T + C, with stopping distance from the vehicle's measured braking. | **Yes.** It assumes worst-case acceleration through the whole response time, at the band's TOP speed, in every direction the limiter allows (the swerve term). Installations usually size from the nominal speed per field. |
| Rotation sweep | The footprint swept through the yaw reachable within T, from the measured yaw rate and `alpha_max` 1.4 rad/s². | Usually absent. Scanners' fields are static polygons per field set, and rotation is covered by an extra margin chosen at commissioning. | **Yes, it shapes the yaw rate; it does not cap it at 0.2 rad/s** (evidence 2026-10-26, simulator, PFB above the ideal tracker, 60 tune routes, `runs/analysis/PFB_SIM_ACCEPTANCE_20261026T171339Z_seed0`; \|yaw rate\| rad/s, median / p90 / max). Band 2: bends 0.15 / 0.38 / 0.64, straights 0.00 / 0.26 / 0.67. Band 1: bends 0.14 / 0.39 / 0.72. Band 0: bends 0.00 / 0.20 / 0.60. The PFB trades yaw for clearance: in the one traced heading-attributed stop, a requested 1.38 rad/s was filtered to 0.62. The emulator agrees (band 2 bends 0.12 / 0.40 / 0.63, `PFB_ACCEPTANCE_20261026T190701Z_seed0`). The 0.2 rad/s figure below was the oracle schedule's own cap (withdrawn 2026-10-24). |
| Nesting (A21, S2) | A moving field also carries the at-rest field's own rotational reach (+e(0)), at least the rest reach in every direction, and the drift of a body that turns while it translates. A stop begun in motion therefore ends with the at-rest field clear. | No direct counterpart. Field-set switching on hardware is often staggered so the next set is free, which is the same intent. | **Yes.** It adds 9-11 mm in every direction (the departure shell). |
| Discrete bands | Three field sets, selected by reachable speed: tops 0.10 / 0.30 / 0.51 m/s. | Speed-indexed field-set switching (typically 4-16 sets, selected by the safety controller from encoder speed). | **Unknown / probably yes.** There are fewer sets than typical, so the field in force is sized for its band's top speed, not the speed driven. Band 0's usable speed is 0.025 m/s. *Open: compare with a 4- and 8-set layout.* |
| Margins | `margin` 0.10 m ahead; `lateral_margin` 0.05 m abeam and behind. The at-rest field is the infimum of the moving fields. | ISO 13855 supplement C (measurement error and reflectors), typically 0.1 m or more ahead. | **Equal / slightly less** abeam. *Open: the S300's own tolerance (+-30 mm at 5 m) against the 0.05 m.* |
| Warning field | `warning_scale` 1.5 x the depth, + 0.05 m ahead, 0.02 m abeam. Reported, never acts. | A warning field that usually triggers a slow-down. | **Not applicable.** The emulator does not slow on it. The route cap (2026-10-20) provides the slow-down for classical baselines, from the map. |
| Unobserved space | A bin with no valid return breaches every field (hard rule 5). Dropout reads 0.0, never range_max. | Scanner self-test: contamination or a blinded window triggers a safe state. | **Equal or yes.** |

## Stop and restart

| Feature | Emulator | Industrial counterpart | More conservative than hardware? |
|---|---|---|---|
| Response time | 0.10 s from breach to braking command. | Scanner response (typically 60-120 ms for the S300) plus the safety controller's cycle. | **Equal.** *Open: the training-host loop latency.* |
| Braking | The limiter's deceleration `a_decel_max` 1.0 m/s², the same as the field's `a_brake`. | A safe stop (STO / SS1) with the drives' braking. The measured joint authority was 2.3-3.0 m/s². | **Yes.** It brakes at less than half the measured authority, and the field is sized for that. |
| Latch and restart delay | The stop latches. Restart needs the delay (2.0 s) and the condition below. | Restart interlock with a manual or automatic restart after the field is clear (IEC 61496). | **Equal.** |
| Restart condition (A22) | The WHOLE field the commanded twist selects must be clear, plus the two approach rules. At or below the standstill exits the at-rest field governs (the creep regime). | Automatic restart once the active field set is clear. Leaving a tight spot uses a slower field set (manoeuvring sets). | **Yes.** It checks the field of the motion asked for, not the one in force. |
| Receding release (A24) | Held at rest with an intrusion inside the at-rest field (the end of every creep stop), the robot is released after the restart delay for a creep twist under which every point inside strictly recedes; those points are exempt only while the command keeps receding at creep. Unobserved bins still block. | A reset by an operator, then a reduced-speed manoeuvring mode to move clear. | **No, less conservative than an operator reset**: ours releases automatically. It is consistent with the modelled restart delay, and it never approaches anything. |
| Standstill detection | Translational enter 0.01 / exit 0.02 m/s. Rotational enter 0.0296 / exit 0.0592 rad/s. From the velocity estimate and the applied command. | Safe standstill monitoring (SSM), from safety-rated encoders. | **Unknown.** *Open: SSM thresholds on comparable AGVs are 5-20 mm/s.* |

## Sensing and state

| Feature | Emulator | Industrial counterpart | More conservative than hardware? |
|---|---|---|---|
| Scanners | Two S300 at opposite corners, merged into 120 bins with exact bearings. Range 0.05-5.0 m, self-filtered. | The same sensors, with field evaluation inside each scanner. | **Unknown.** Bin merging takes the minimum per bin, which is conservative. Beam divergence and reflectivity are not modelled. *Open.* |
| Scan plausibility (A25) | Each packet is checked against the last accepted one, bounded by one step of motion (+-2 bins, +-6 cm); a corrupt render is re-rendered up to twice, then every bin is unobserved (a fail-safe stop), counted as a scan fault and kept out of H1's stop count. | Scanner self-diagnostics: an internal fault, contamination or a blinded window drives the outputs to the safe state. | **Equal** in effect (a fault means a stop). It exists because the SIMULATED renderer can corrupt under CPU starvation; a real S300 has no such mode. |
| Velocity estimate | Pose differences, low-pass tau 0.10 s. The band and the sweep take the larger of the estimate and the applied command. | Safety-rated encoder speed per wheel, typically unlagged. | **Yes for the field** (the larger of the two); the estimate itself lags. |
| Unmapped obstacles | Sensed (above the scan plane), never in the planner's map. | The same. | **Equal.** |
| Collision | Ground-truth simulator contact only (hard rule 4). | Physical contact. | **Equal.** |

## Above the safety layer (declared, not part of the emulator)

| Feature | Emulator | Industrial counterpart | More conservative than hardware? |
|---|---|---|---|
| Classical recovery | Creep at 0.018 m/s, 0.30 m per attempt, at most 5 attempts. | Vendor recovery behaviours (back-off, manoeuvring mode). | **Not applicable** (a baseline component). |
| Route cap (2026-10-20) | The fastest band whose field is clear of the MAPPED world, per point of the route, plus a yaw-rate cap. Anticipated over 0.10 m and deceleration-limited at 0.5 m/s². Every classical baseline gets it; learned arms do not. | Map-based speed zones and slow-down areas in fleet managers, and Regulated Pure Pursuit's proximity regulation. | **Not applicable.** It shapes the controllers, not the safety layer. |

## Consequence: the achievable speed (recorded 2026-10-20, A23; not fixed under the freeze)

**An ideal tracker averages about 0.12 m/s under A20-A22.** The privileged oracle tracker needs a
median 88 s, 8.4 s/m, over the 60 tune routes.
**WITHDRAWN 2026-10-24 (supervisor): an artefact of the oracle schedule's 0.20 rad/s yaw cap, not a property of the fields. With the PFB, the ideal tracker reached the goal on 48 of the 60 tune routes (the other 12 stall on unmapped obstacles it cannot see) at a median 0.37 m/s (0.28-0.42; 2.7 s/m against the oracle's 8.4), with zero unattributed certified stops and no contact (runs/analysis/PFB_ACCEPTANCE_20261024T174707Z_seed0, emulator, sector scan). The text below is kept as recorded.**

**Rotation dominates.** Band 0, swept at the yaw rate, stays traversable only up to 0.20 rad/s
(0.10-0.20 per route, rounded down). In the narrowest aisle alone it would admit about 0.5 rad/s;
the binding places are shelf ends. Hardware field sets carry no rotation sweep; a commissioning
margin covers rotation.

**So the fields are more conservative than typical industrial configurations, especially in
rotation.** The results report the oracle mission time as a reference line.

## Places where the emulator is kinder than hardware (to be listed in the write-up)

- **The receding release (A24):** it releases automatically where hardware needs an operator reset.
  It is bounded: creep speed, strictly receding from everything inside, after the restart delay.

Candidates still under the *open* rows:

- abeam margin against scanner tolerance;
- standstill thresholds;
- beam divergence.
