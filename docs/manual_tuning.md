# Tuning a classical baseline by hand

Use this when a baseline misbehaves (stops, wanders, never reaches the goal) and you want to find out
why and adjust it. The directions below follow from what each parameter does; they are expectations to
test, not measured effects.

## The loop

1. **Pick the scenarios that show the problem.** Run the baseline on a set, find the indices of the bad
   episodes in `runs/eval/<run>/episodes.jsonl`, and cut them out:
   `python scripts/make_subset.py scenarios/tune/tune_narrow.json --indices 3 7 --out runs/scratch/a.json`
   (test sets are refused).
2. **Watch it.** `python scripts/evaluate.py --config configs/base.yaml --seed 0 --controller <ctrl>
   --sets runs/scratch/a.json --draw-path`. The planned route is blue, the robot's track red, start green,
   goal yellow. If the red track leaves the blue line, the controller is not following it; if it stops
   on the line, look at the certified layer (below).
3. **Read what happened.** A step ring buffer dumps the recent steps (command, applied command, certified
   state) under `runs/eval/<run>/ring/` on stationary triggers, contacts and unrecoverable stops.
4. **Change one parameter**, in `configs/baselines/<id>.yaml` only (`params:`), by a modest factor
   (about 20-30 %). Rerun the same subset.
5. **Then rerun the full tune set** before believing it. A change that fixes one route often breaks
   others.
6. **Record it** in `docs/decision_log.md`: which parameter, old and new value, why, and the tune-set result.

## Rules that keep the comparison fair

- Tune on the tune set only, never on `scenarios/test/`.
- The baselines were tuned with an equal budget (24 candidates each). Hand-tuning one baseline gives it
  more effort than the others. Either give every baseline the same hand-tuning effort, or report the
  hand-tuned values as a separate, labelled variant. Do not quietly replace the tuned values.
- Never retune the PFB's alpha, the certified layer, the recovery or the actuation limits to make a
  baseline look better: they are shared by every method.

## Symptoms and parameters

The first thing to decide is whether the robot is **not following the route** (a controller problem) or
**standing still** (a latched protective stop, the recovery, or a controller that has slowed to zero).

| Symptom | Likely parameters (direction) |
|---|---|
| Stops often near shelf corners (many protective stops) | Slow down where the certified field bites: B1 `cruise_speed` down; B2 `proximity_distance` up, `proximity_gain` down, `min_radius` up; B3 `safety_margin` and `w_clearance` up, `max_speed` down; B6 `field_breach` weight up |
| Cuts corners or swings wide | Lookahead: B1 `lookahead`; B2 `lookahead_time`, `min_lookahead`, `max_lookahead`; B3 `w_path` up; B6 `route_tracking` up |
| Oscillates or snakes along the route | Lookahead too short or gain too high: B1/B2 `k_omega` down, lookahead up; B3 `w_heading` down; B6 `smoothness` up, `noise_std` down |
| Very slow, times out | B1 `cruise_speed` up; B2 `desired_speed`, `min_speed` up and `proximity_distance` down; B3 `max_speed` up, `w_clearance` down; B6 `goal_progress` up |
| Stops short of the goal or creeps at the end | B1 `goal_slowdown` down; B2 `approach_distance` down, `min_approach_speed` up |
| B3 picks poor commands | `window_time` and `sim_time` (reach of the search), then the weights; the sample counts and granularity are resolution, not tuned |
| B6 does not move or jitters | `temperature` (low = near-greedy, high = averaging), `noise_std`; `samples` (1024) and `horizon_steps` (20) are fixed by the 50 ms real-time limit |

## Telling a stop from a controller problem

A robot that "can rotate but doesn't" is usually in one of: the 2.0 s certified restart delay, the 1.0 s
recovery wait, the recovery creep (0.018 m/s, translation only), or standing inside its own field (a
rotation would sweep the footprint into the obstacle, so the restart condition refuses it). Check the ring
dump for the certified state before changing any controller parameter: tuning a controller cannot fix a
stop it did not cause, and the fix is usually a slower approach, not a different path.
