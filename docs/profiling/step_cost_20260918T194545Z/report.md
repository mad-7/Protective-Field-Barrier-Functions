# Per-step cost profile (not a gate)

Commit `486495cfcee6dda06e442cc9ce27881aed6c2fc9` (dirty: False), scene `340dbc5d...`, engine ODE, 400 steps per instance, 3 interleaved passes. Machine: 8 logical cores, 4 physical, Intel64 Family 6 Model 142 Stepping 12, GenuineIntel. Components above ~2 ms per step on average are flagged. Per-step samples are in samples/.

## 1 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.036 | 0.033 | 0.064 | 0.348 |  |
| command | 6.643 | 6.226 | 10.152 | 18.027 | above ~2 ms |
| step | 56.165 | 53.531 | 76.120 | 150.305 | above ~2 ms |
| contact | 6.641 | 3.562 | 5.702 | 1223.583 | above ~2 ms |
| pose | 3.317 | 3.020 | 5.174 | 10.380 | above ~2 ms |
| velocity | 0.090 | 0.085 | 0.144 | 0.513 |  |
| torque_cap | 6.356 | 5.928 | 9.605 | 17.217 | above ~2 ms |
| **whole step** | 79.248 | 72.979 | 105.328 | 1295.199 | |

Aggregate steps/s by pass: 12.8, 12.5, 12.6

- spike: pass 0, instance 0, contact, steps [0]
- spike: pass 1, instance 0, contact, steps [0]
- spike: pass 2, instance 0, contact, steps [0]

## 2 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.039 | 0.036 | 0.065 | 0.257 |  |
| command | 7.751 | 7.268 | 12.211 | 39.359 | above ~2 ms |
| step | 68.139 | 64.413 | 100.945 | 133.737 | above ~2 ms |
| contact | 7.762 | 4.228 | 6.979 | 1493.899 | above ~2 ms |
| pose | 3.914 | 3.654 | 6.187 | 16.780 | above ~2 ms |
| velocity | 0.099 | 0.090 | 0.167 | 1.047 |  |
| torque_cap | 7.528 | 6.937 | 12.069 | 24.398 | above ~2 ms |
| **whole step** | 95.233 | 86.627 | 137.177 | 1579.401 | |

Aggregate steps/s by pass: 20.9, 20.8, 21.3

- spike: pass 0, instance 0, contact, steps [0]
- spike: pass 0, instance 1, contact, steps [0]
- spike: pass 1, instance 0, contact, steps [0]
- spike: pass 1, instance 1, contact, steps [0]
- spike: pass 2, instance 0, contact, steps [0]
- spike: pass 2, instance 1, contact, steps [0]

## 3 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.033 | 0.029 | 0.057 | 0.895 |  |
| command | 6.326 | 6.308 | 9.486 | 18.451 | above ~2 ms |
| step | 57.004 | 59.029 | 73.921 | 119.096 | above ~2 ms |
| contact | 6.430 | 3.723 | 5.509 | 1290.715 | above ~2 ms |
| pose | 3.212 | 3.180 | 4.868 | 11.070 | above ~2 ms |
| velocity | 0.085 | 0.078 | 0.135 | 1.149 |  |
| torque_cap | 6.001 | 6.003 | 8.694 | 16.542 | above ~2 ms |
| **whole step** | 79.091 | 79.555 | 98.374 | 1357.024 | |

Aggregate steps/s by pass: 34.0, 46.9, 35.2

- spike: pass 0, instance 0, contact, steps [0]
- spike: pass 0, instance 1, contact, steps [0]
- spike: pass 0, instance 2, contact, steps [0]
- spike: pass 1, instance 0, contact, steps [0]
- spike: pass 1, instance 1, contact, steps [0]
- spike: pass 1, instance 2, contact, steps [0]
- spike: pass 2, instance 0, contact, steps [0]
- spike: pass 2, instance 1, contact, steps [0]
- spike: pass 2, instance 2, contact, steps [0]

## 4 instance(s)

| component | mean ms | p50 ms | p95 ms | max ms | flag |
|---|---|---|---|---|---|
| budget | 0.036 | 0.033 | 0.063 | 1.357 |  |
| command | 6.757 | 6.559 | 9.836 | 41.272 | above ~2 ms |
| step | 63.145 | 61.493 | 76.562 | 3045.789 | above ~2 ms |
| contact | 6.758 | 3.837 | 5.505 | 1445.086 | above ~2 ms |
| pose | 3.425 | 3.269 | 5.074 | 63.713 | above ~2 ms |
| velocity | 0.093 | 0.083 | 0.158 | 4.012 |  |
| torque_cap | 6.500 | 6.301 | 9.374 | 23.742 | above ~2 ms |
| **whole step** | 86.715 | 82.364 | 102.186 | 3079.591 | |

Aggregate steps/s by pass: 40.9, 49.2, 49.3

- spike: pass 0, instance 0, contact, steps [0]
- spike: pass 0, instance 1, contact, steps [0]
- spike: pass 0, instance 2, contact, steps [0]
- spike: pass 0, instance 3, contact, steps [0]
- spike: pass 1, instance 0, contact, steps [0]
- spike: pass 1, instance 1, contact, steps [0]
- spike: pass 1, instance 2, contact, steps [0]
- spike: pass 1, instance 3, contact, steps [0]
- spike: pass 2, instance 0, contact, steps [0]
- spike: pass 2, instance 1, contact, steps [0]
- spike: pass 2, instance 1, pose, steps [22]
- spike: pass 2, instance 2, contact, steps [0]
- spike: pass 2, instance 3, contact, steps [0]
