# Per-step cost profile (not a gate)

Commit `f78b04f95ef7561788d165b7dda8a2ea74aed992` (dirty: True), scene `340dbc5d...`, engine ODE, 400 steps per instance. Components above ~2 ms per step on average are flagged.

## 1 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.031 | 0.026 | 0.057 | 0.153 |  |
| command | 6.290 | 5.616 | 11.024 | 16.431 | above ~2 ms |
| step | 49.051 | 45.262 | 84.452 | 108.945 | above ~2 ms |
| contact | 5.157 | 3.261 | 6.698 | 603.982 | above ~2 ms |
| pose | 3.141 | 2.721 | 5.943 | 8.035 | above ~2 ms |
| velocity | 0.085 | 0.070 | 0.159 | 0.344 |  |
| torque_cap | 6.093 | 5.414 | 10.900 | 17.342 | above ~2 ms |
| **whole step** | 69.848 | 63.362 | 119.303 | 673.955 | |

## 2 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.037 | 0.034 | 0.064 | 0.424 |  |
| command | 6.930 | 6.542 | 10.462 | 20.670 | above ~2 ms |
| step | 59.147 | 54.995 | 89.838 | 196.395 | above ~2 ms |
| contact | 5.881 | 3.694 | 6.461 | 748.525 | above ~2 ms |
| pose | 3.497 | 3.241 | 5.307 | 16.731 | above ~2 ms |
| velocity | 0.095 | 0.086 | 0.140 | 3.839 |  |
| torque_cap | 6.798 | 6.282 | 10.551 | 21.667 | above ~2 ms |
| **whole step** | 82.385 | 75.176 | 120.209 | 824.351 | |

## 3 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.028 | 0.025 | 0.057 | 0.157 |  |
| command | 5.396 | 4.728 | 10.298 | 20.687 | above ~2 ms |
| step | 46.941 | 43.014 | 88.109 | 127.713 | above ~2 ms |
| contact | 5.312 | 2.754 | 5.983 | 961.458 | above ~2 ms |
| pose | 2.746 | 2.372 | 5.189 | 8.484 | above ~2 ms |
| velocity | 0.075 | 0.066 | 0.143 | 0.303 |  |
| torque_cap | 5.228 | 4.608 | 9.944 | 20.732 | above ~2 ms |
| **whole step** | 65.726 | 57.861 | 124.688 | 1033.313 | |

## 4 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.032 | 0.030 | 0.051 | 0.219 |  |
| command | 6.339 | 6.218 | 9.054 | 26.618 | above ~2 ms |
| step | 59.012 | 59.338 | 74.624 | 245.423 | above ~2 ms |
| contact | 5.709 | 3.629 | 5.282 | 880.884 | above ~2 ms |
| pose | 3.197 | 3.099 | 4.688 | 15.539 | above ~2 ms |
| velocity | 0.087 | 0.077 | 0.134 | 3.973 |  |
| torque_cap | 6.051 | 5.877 | 8.520 | 24.014 | above ~2 ms |
| **whole step** | 80.427 | 79.401 | 97.537 | 951.355 | |
