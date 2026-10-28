# Free-orientation oracle, phase decide (one run)

2271 routes from tune_narrow, tune_medium, tune_wide, tune_unmapped_narrow, tune_unmapped_medium, tune_unmapped_wide, pilot_narrow, train_narrow, train_medium, train_wide.

Lethal margin 0.07 (ruled). Free-orientation oracle: a continuous yaw path within the yaw-rate, yaw-acceleration and wheel-budget limits, field along the velocity, footprint rotating with the body; criterion: zero routes with a band-0 breach.

*The unavoidable-stop floor is platform-specific: a holonomic base with full knowledge of its route (the oracle). A1's floor is the tangent-locked oracle's.*

| soft | plannable | quant. fails | start-invalid | goal-invalid | residual | bend-stop routes | free floor /100 m | A1 (tangent) floor /100 m | residual rule |
|---|---|---|---|---|---|---|---|---|---|
| 0.03 | 2271/2271 | 0 | 20 | 0 | 0 | 0 | 0.083 | 1.012 | re-sample |
| 0.05 | 2271/2271 | 0 | 20 | 0 | 0 | 0 | 0.083 | 0.935 | re-sample |
| 0.08 | 2271/2271 | 0 | 20 | 0 | 0 | 0 | 0.082 | 0.774 | re-sample |
| 0.10 | 2271/2271 | 0 | 20 | 0 | 0 | 0 | 0.081 | 0.769 | re-sample |
| 0.13 | 2271/2271 | 0 | 20 | 0 | 0 | 0 | 0.081 | 0.818 | re-sample |
| 0.20 | 2271/2271 | 0 | 20 | 0 | 1 | 0 | 0.083 | 0.791 | re-sample |
| 0.30 | 2271/2271 | 0 | 20 | 0 | 1 | 0 | 0.082 | 0.805 | re-sample |

**Chosen soft margin (fewest bend stops, plannability kept):** 0.03. **Residual rule** (<= 2% in every stratum): re-sample.

Per stratum at soft 0.03:

| stratum | routes | start-invalid | goal-invalid | residual (share) | bend-stop routes | free floor /100 m | A1 floor /100 m |
|---|---|---|---|---|---|---|---|
| medium | 707 | 0 | 0 | 0 (0.0%) | 0 | 0.000 | 0.053 |
| narrow | 857 | 20 | 0 | 0 (0.0%) | 0 | 0.222 | 2.649 |
| wide | 707 | 0 | 0 | 0 (0.0%) | 0 | 0.000 | 0.000 |
