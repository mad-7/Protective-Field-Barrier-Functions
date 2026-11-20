# 1. The certified safety layer

## Definition

A **certified safety layer** is a component that sits between a motion controller and the
actuators and satisfies three properties:

1. **Independence.** It uses its own sensing channel (a safety LiDAR) and its own logic. It
   shares no code, state or tuning with the controller it supervises.
2. **Fixed, declared protective fields.** Its trigger set is a finite family of
   pre-configured regions of the plane, selected by a discrete variable (speed band and
   direction), not computed from the controller's intent.
3. **Fail-safe action only.** Its only authority is to stop. It never steers, re-plans or
   modifies a command except by replacing it with a braking command.

Formally, let the robot's footprint be F, the speed bands be b = 0..B-1 with top speeds
v_0 < ... < v_{B-1}, and let `field(b, theta)` be the closed region the layer protects in
band b for travel direction theta (the footprint swept along the braking segment, plus
margins). With scan points P_k (body frame, valid bins only) and the estimated velocity
v_hat_k, the layer's state machine is

```
b_k      = band(reachable speed at step k)
trigger  = exists p in P_k : p in field(b_k, theta_k)          # or any invalid bin
latched  = trigger  or  (latched_prev and not restart_ok)
u_out    = 0 (braking within a_brake)   if latched
           u_cmd                        otherwise
restart_ok = field clear for restart_delay  and  no breaching point approaches
             (v . p <= 0 for every breaching p)
```

A **protective stop** is one entry into the latched state. The primary metric of this project
is protective stops per 100 m.

## What it is in this project

`pfbnav/safety/certified.py` emulates an industrial safety scanner plus safety controller
(SICK S300 class). Parameters (`configs/base.yaml`, key `certified`): response time 0.10 s,
braking 1.0 m/s^2, restart delay 2.0 s, three speed bands with tops 0.10 / 0.30 / 0.51 m/s,
ahead margin 0.10 m, lateral margin 0.05 m. It runs beneath **every** controller, learned or
classical.

## What it is not

It does not claim functional-safety compliance (no SIL/PL argument). It is an emulator that
reproduces the behaviour that matters for the research question: a discrete, conservative,
latching stop. Collisions are never judged by it; they are judged only by simulator contact
checks.

## Why it matters for the contribution

Because its fields are discrete and sized for the worst case of each band, it stops a robot
that a continuous, command-aware analysis would show to be safe. Each such stop costs at
least the restart delay plus recovery. The PFB (chapter 3) removes those stops without
altering or bypassing the certified layer.
