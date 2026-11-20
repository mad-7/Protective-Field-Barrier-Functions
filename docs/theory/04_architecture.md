# 4. Proposed architecture

```
 map + goal
    |
    v
 [ A* global planner ]  --> route (waypoints)
    |
    v
 [ Local controller ]   --> u_nom = (vx, vy, w)      B1 PP | B2 RPP | B3 DWA | B6 MPPI | SAC (L1-L5)
    |
    v
 [ PFB filter ]         --> u = argmin |u - u_nom|_W  s.t. barrier rows      (the contribution)
    |                        (identity when disabled)
    v
 [ Certified safety layer ]  --> pass u, or latched protective stop           (independent, always on)
    |
    v
 [ Actuation: box, wheel budget, acceleration limiter ]  --> youBot (CoppeliaSim)
    ^
    |  LiDAR scan (120-bin filter stream, with validity flags), velocity estimate
```

## Roles

| Block | Knows about | Authority |
|---|---|---|
| Planner | map | route only |
| Controller | route, scan | proposes `u_nom` |
| **PFB** | scan, velocity estimate, certified field geometry | minimally modifies `u_nom` to keep every return outside the field |
| Certified layer | scan, velocity estimate | stops, latches, restarts |
| Simulator contact check | ground truth | defines collisions (never the scan) |

## Properties of the architecture

- **Controller-agnostic.** The PFB only sees `u_nom`. Any controller can sit above it; this
  is what allows the same filter to be inserted into every baseline.
- **Identity when off.** With `filter.enabled: false` the PFB is the identity map
  (verified by a bit-identity test), so a baseline is exactly the baseline.
- **Independence.** `certified.py` and `pfb.py` import neither one another nor shared field
  code (`tests/test_architecture.py`). The PFB reads the certified layer's configuration
  keys, never its code, so the two cannot drift silently.
- **Same stream.** All controllers and the PFB read the same scan stream, the same recovery
  behaviour (classical set), and the same actuation limits.

## Controlled variable

Every comparison differs in exactly one thing: whether the PFB sits between controller and
certified layer. Primary metric: protective stops per 100 m. Secondary: success, mission
time, collisions (contact only).
