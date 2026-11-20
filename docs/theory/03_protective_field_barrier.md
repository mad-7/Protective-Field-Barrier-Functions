# 3. The Protective-Field Barrier (PFB)

The PFB is the proposed safety mechanism. Note on naming: it is the **Protective-Field
Barrier**, a deterministic CBF. It is not a probabilistic barrier function.

## Definition

The PFB is a discrete-time CBF filter whose **safe set is the complement of the certified
layer's own trigger set**, with the discrete band fields replaced by a continuous, speed- and
direction-dependent protective field `rho`.

For a candidate command `u = (vx, vy, w)` and each valid scan return `i` within reach, with
`r_i(u)` and `psi_i(u)` the return's range and bearing in the body frame after travelling at
`u` for `tau = dt + lag_allowance`:

```
g_i(u) = r_i(u) - rho( psi_i(u); theta(u), W(u) ) - delta   >=   (1 - a dt) * max( h_i(now), 0 )
```

- `rho(psi; V, theta, w)`: the boundary distance of the protective field along bearing psi.
  The field is the footprint swept over the reachable heading change, extended by the travel
  over the reach time plus braking, `D(V) = V T + V^2 / (2 a_b)`, plus margins
  (`pfbnav/safety/pfb_field.py`). It is built from the same configuration keys as the
  certified layer but by independent code.
- `theta(u)`: field heading, along `u` where `u` dominates the velocity estimate.
- `W(u) = max(|w_hat|, |w|)`: yaw rate the field is swept for.
- `delta`: the measured binning penalty at its upper bound (0.5 mm).
- `max(h_i(now), 0)`: a regime that is breached now must be clear after one step. The filter
  never enters a breached regime.

The filter solves, per regime (at-rest field, or the band selected by reachable speed), the QP

```
min  (u - u_nom)^T diag(1, 1, rho_out^2) (u - u_nom)
s.t. linearised rows g_i(u) >= ...  (tangents in translation, chords in yaw rate),
     wheel polytope, axis box, limiter's one-step envelope, regime speed range.
```

The true rows are checked at the solution with no tolerance; one re-linearisation is allowed.
The best feasible regime wins. If none is feasible, the maximum-braking command is issued and
labelled `brake` (or `infeasible`, counted).

`alpha` is frozen at 1.0 /s for all PFB arms (selected on the tune set, grid low edge,
reported).

## Why this removes protective stops

The certified layer triggers when a valid point lies inside its field for the current band.
The PFB constrains the command so that, after one step, no valid point lies inside the field
of the regime the robot will be in. If a feasible command exists, the certified layer is
never triggered by that step, so no protective stop occurs. Where no feasible command exists
the robot brakes, and the certified layer remains the backstop. The PFB never removes or
weakens the certified layer.

## Under the certified latch

While the certified layer is latched, only a command that its restart condition could release
passes (a creep that recedes from every point inside the at-rest field, or a twist whose own
band field is clear). Otherwise the output is zero.

## Hypotheses it serves

H1: protective stops per 100 m (rate ratio) of the PFB-equipped learned method vs baselines.
H2: PFB in the training loop vs filter bolted on afterwards. H5: attribution on a classical
host (B7). Exploratory: the PFB beneath MPPI (B8) and DWA (B9).
