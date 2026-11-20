# 2. Control Barrier Functions

## Definition

Consider a control-affine system `x_dot = f(x) + g(x) u`, with `x` in X and `u` in U. Let
`h : X -> R` be continuously differentiable and define the **safe set**

```
C = { x : h(x) >= 0 }.
```

`h` is a **Control Barrier Function (CBF)** for C if there is an extended class-K function
`alpha` (here `alpha(h) = a h`, a > 0) such that for all x in a neighbourhood of C

```
sup_{u in U}  [ L_f h(x) + L_g h(x) u ]  >=  - alpha(h(x)).
```

**Forward invariance.** If `h` is a CBF and the controller keeps
`L_f h + L_g h u >= -alpha(h)` at every x, then C is forward invariant: a trajectory that
starts in C stays in C (Ames et al., 2017).

## As a safety mechanism: the CBF filter

A CBF turns safety into a constraint on the command, solved in a minimally invasive way:

```
u* = argmin_u  (u - u_nom)^T W (u - u_nom)
     s.t.  L_f h + L_g h u >= -alpha(h)        # barrier condition
           u in U                              # actuation limits
```

`u_nom` is whatever the controller wanted. When the barrier condition is inactive, `u* = u_nom`.

## Discrete-time form (what is implemented)

The controller runs at a fixed step `dt`, so the condition used is

```
h(x_{k+1}(u)) >= (1 - a dt) h(x_k),     0 < a dt <= 1.
```

For a predicted state that depends on the candidate command `u` through a reach horizon
`tau = dt + lag`, this is evaluated on a declared grid of motion fractions and tightened by
a measured margin so that the grid minimum lower-bounds the dense minimum
(`filter.motion_fractions`, `filter.motion_grid_margin_m`).

## Assumptions and limits

- The guarantee is relative to the model: kinematic prediction, bounded estimation lag,
  declared actuation limits. It is not a guarantee about the real world.
- If the QP is infeasible, no invariance claim holds for that step. The filter then brakes at
  maximum rate and the event is counted and reported.
- An unobserved region is never treated as free space. An invalid scan bin is an obstacle at
  the footprint's boundary on its bearing.
