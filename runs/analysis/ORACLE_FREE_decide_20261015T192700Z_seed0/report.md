# Free-orientation oracle, phase decide (one run)

2271 routes from tune_narrow, tune_medium, tune_wide, tune_unmapped_narrow, tune_unmapped_medium, tune_unmapped_wide, pilot_narrow, train_narrow, train_medium, train_wide.

Lethal margin 0.07 (ruled). Free-orientation oracle: a continuous yaw path within the yaw-rate, yaw-acceleration and wheel-budget limits, field along the velocity, footprint rotating with the body; criterion: zero routes with a band-0 breach.

*The unavoidable-stop floor is platform-specific: a holonomic base with full knowledge of its route (the oracle). A1's floor is the tangent-locked oracle's.*

| soft | plannable | quant. fails | start-invalid | goal-invalid | residual | bend-stop routes | free floor /100 m | A1 (tangent) floor /100 m | residual rule |
|---|---|---|---|---|---|---|---|---|---|
| 0.03 | 2271/2271 | 0 | 20 | 1 | 3 | 1 | 0.100 | 1.012 | re-sample |
| 0.05 | 2271/2271 | 0 | 20 | 1 | 9 | 0 | 0.124 | 0.935 | re-sample |
| 0.08 | 2271/2271 | 0 | 20 | 1 | 14 | 0 | 0.143 | 0.774 | re-sample |
| 0.10 | 2271/2271 | 0 | 20 | 1 | 6 | 0 | 0.110 | 0.769 | re-sample |
| 0.13 | 2271/2271 | 0 | 20 | 2 | 5 | 0 | 0.109 | 0.818 | re-sample |
| 0.20 | 2271/2271 | 0 | 20 | 2 | 4 | 0 | 0.103 | 0.791 | re-sample |
| 0.30 | 2271/2271 | 0 | 20 | 2 | 9 | 0 | 0.121 | 0.805 | re-sample |

**Chosen soft margin (fewest bend stops, plannability kept):** 0.05. **Residual rule** (<= 2% in every stratum): re-sample.

Per stratum at soft 0.05:

| stratum | routes | start-invalid | goal-invalid | residual (share) | bend-stop routes | free floor /100 m | A1 floor /100 m |
|---|---|---|---|---|---|---|---|
| medium | 707 | 0 | 0 | 1 (0.1%) | 0 | 0.013 | 0.105 |
| narrow | 857 | 20 | 1 | 4 (0.5%) | 0 | 0.274 | 2.388 |
| wide | 707 | 0 | 0 | 4 (0.6%) | 0 | 0.054 | 0.000 |
