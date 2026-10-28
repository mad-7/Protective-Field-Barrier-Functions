# Soft margin under the oracle criterion (one run)

2271 routes from tune_narrow, tune_medium, tune_wide, tune_unmapped_narrow, tune_unmapped_medium, tune_unmapped_wide, pilot_narrow, train_narrow, train_medium, train_wide.

Lethal margin 0.07 (ruled). Oracle tracker: fastest clear band at every point, deceleration-limited; criterion: zero routes with a band-0 breach.

| soft | plannable | quant. fails | start-invalid | goal-invalid | residual | bend-breach routes | unavoidable stops /100 m | mission time / constant (median) | 50 mm inside (info) | residual rule |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.03 | 2271/2271 | 0 | 16 | 12 | 187 | 187 | 1.012 | 1.008 | 2141 | stop |
| 0.05 | 2271/2271 | 0 | 15 | 10 | 187 | 167 | 0.935 | 1.008 | 2179 | stop |
| 0.08 | 2271/2271 | 0 | 15 | 13 | 150 | 144 | 0.774 | 1.008 | 1986 | stop |
| 0.10 | 2271/2271 | 0 | 17 | 14 | 143 | 141 | 0.769 | 1.008 | 1885 | stop |
| 0.13 | 2271/2271 | 0 | 18 | 10 | 161 | 160 | 0.818 | 1.008 | 1684 | stop |
| 0.20 | 2271/2271 | 0 | 15 | 11 | 161 | 155 | 0.791 | 1.008 | 1717 | stop |
| 0.30 | 2271/2271 | 0 | 18 | 14 | 159 | 161 | 0.805 | 1.007 | 1731 | stop |

**Chosen soft margin (fewest bend breaches, plannability kept):** 0.1. **Residual rule** (<= 2% in every stratum): stop.

Per stratum at soft 0.1:

| stratum | routes | start-invalid | goal-invalid | residual (share) | bend-breach routes | stops /100 m | time ratio (median) | 50 mm inside |
|---|---|---|---|---|---|---|---|---|
| medium | 707 | 0 | 1 | 4 (0.6%) | 4 | 0.065 | 1.008 | 569 |
| narrow | 857 | 17 | 13 | 139 (16.2%) | 137 | 1.957 | 1.008 | 770 |
| wide | 707 | 0 | 0 | 0 (0.0%) | 0 | 0.000 | 1.008 | 546 |
