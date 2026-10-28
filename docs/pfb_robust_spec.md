# Robust PFB against a bounded motion disturbance — specification

Status: **specification only.** No filter is implemented. Written for D10 (supervisor
review, 2026-09-18). The hypothesis it serves is **H6, exploratory** (amendment A3).

---

## 1. What the measurements showed

> **Superseded as numbers, 2026-09-18.** Every figure in this section was measured on
> Bullet. D7 froze the physics at ODE, where lateral drift at 0.12 m/s is 0.47° instead of
> 20.55°. The framing in §2–§4 stands; the bound must be refitted from G1b on ODE (§5.2).

The platform does not execute the velocity it is commanded. Two effects, both measured,
both reproducible, and both *deterministic functions of the command* rather than noise:

**Speed deficit.** Realised speed falls short of commanded, with a knee between 0.2 and
0.3 m/s. Forward profile, open loop, 5 m:

| commanded | realised | deficit |
|---|---|---|
| 0.10 m/s | 0.093 | 7.3% |
| 0.20 | 0.180 | 9.8% |
| 0.30 | 0.241 | 19.8% |
| 0.50 | 0.315 | 36.0% |

**Yaw coupling.** A commanded lateral velocity produces an uncommanded yaw rate. Lateral
profile, open loop, 5 m, expressed as yaw per metre of travel:

| commanded lateral speed | yaw coupling |
|---|---|
| 0.06 m/s | 0.045 rad/m |
| 0.12 | 0.072 |
| 0.18 | 0.083 |
| 0.24 | 0.160 |
| 0.30 | loss of control |

Forward motion shows 0.003 rad/m at 0.20 m/s — two orders of magnitude smaller. The
coupling **reverses sign with the strafe direction at matched magnitude** (+20.55° for
+0.12 m/s, −20.35° for −0.12 m/s), so it is odd in the commanded lateral velocity and is
not an asymmetry of the robot.

Cause, established by elimination: the mecanum mixing matches Modern Robotics exactly, the
wheel order matches KUKA's own driver configuration, the roller angles match specification,
the geometry is measured, and the sign vector is confirmed by motion. What remains is
CoppeliaSim's wheel emulation — a sphere with a free 45° joint, roughly 3% off in the
differential between diagonal pairs. Forward motion does not exercise the free joint;
lateral motion does.

## 2. Why "bounded disturbance" rather than "noise" or "model error"

Three framings were available and two are wrong.

Treating it as **noise** is wrong: it is repeatable to three significant figures and
sign-deterministic. A stochastic bound would be both loose and unfalsifiable.

Treating it as **model error to be corrected** — fitting a compensator and cancelling it —
is wrong for this project: it would tune the plant model to a simulator artefact, and any
safety claim would then rest on the fidelity of that fit rather than on the barrier.

Treating it as a **bounded disturbance carried explicitly by the barrier** is right: the
barrier stops claiming the robot goes exactly where commanded, and instead claims safety
for *every* realised velocity inside a stated set. The bound is measured, the claim is
checkable, and a platform change alters the bound rather than invalidating the argument.

## 3. Disturbance model

Let `u = (v_x, v_y, ω)` be the commanded body velocity and `v = (ẋ, ẏ, ω)` the realised
one. Define the disturbance as the difference, in the body frame:

```
d(u) = v_realised(u) − u
```

**Structure to be fitted**, from the measurements above:

```
d_speed(u)  = −κ(‖u_xy‖) · u_xy              speed deficit, along the commanded direction
d_yaw(u)    =  γ(‖u_xy‖) · (u_y / ‖u_xy‖)    yaw coupling, odd in the lateral component
```

with `κ` and `γ` monotone in speed over the operating range. The forward measurements pin
`γ ≈ 0` for `u_y = 0`; the lateral measurements pin `γ` at five speeds. **The intermediate
command directions are unmeasured and must be, before the bound can be claimed:** the
diagonal profile has been run at one speed only.

**The bound** is the pair

```
κ̄(s) = max over directions of κ at speed s
γ̄(s) = max over directions of |γ| at speed s
```

fitted with a stated margin over the measurement resolution, and **monotone by
construction** so that interpolation between measured speeds is conservative.

### 3a. Two components, split before refitting (ruling 5, WP0 results report)

The speed shortfall on ODE has two causes with different natures, and the bound treats
them separately:

1. **Saturation -- deterministic, and removable.** The wheels are velocity-controlled with
   a 2.5 N m torque cap and top out near 12.34 rad/s, i.e. 0.617 m/s at the rim (G1-ODE,
   2026-09-18). A command whose fastest wheel needs more than that is simply not executed.
   It is a property of the command, not of the ground, and the wheel-speed budget
   (`actuation.wheel_budget`, ruling 2) keeps every command below it: nominal commands
   use at most 0.80 of v_rim, the filter's output at most 0.95.
2. **Residual slip -- the robust bound's target.** Below saturation the robot still falls
   short, with wheel odometry matching the command: the wheels turn as asked and the
   sphere-floor contact slips. This grows with rim speed and depends on direction.

**Measured (G1b re-run, realised-speed map; rim speed = |vx| + |vy| + (l+w)|w|):**

| profile | rim 0.40 | rim 0.50 | highest point | saturated? |
|---|---|---|---|---|
| forward | 9.5 % | 12.0 % | 12.0 % at rim 0.50 | no |
| lateral | 12.3 % | 16.5 % | 16.5 % at rim 0.50 | no |
| diagonal_45 | 11.5 % (rim 0.42) | -- | 16.2 % at rim 0.57; **27.8 % at rim 0.71** | only the last |
| translate_while_rotating | 10.7 % (rim 0.42) | 15.3 % (rim 0.52) | 21.5 % at rim 0.62 | borderline |

**Expected reduction, stated before refitting.** With the budget enforced no wheel is
asked for more than 0.95 x 0.617 = 0.586 m/s, so the saturation component should vanish.
The worst-case shortfall should then fall from **27.8 %** (diagonal, saturated) to about
**16-17 %** -- the largest shortfall measured on unsaturated commands (lateral at rim 0.50,
16.5 %; diagonal at rim 0.57, 16.2 %) -- i.e. roughly 11 of the diagonal's 28 points are
saturation. translate_while_rotating at 0.50 m/s (21.5 %, rim 0.62, at the ceiling) should
likewise drop towards 16-17 %. Forward and lateral up to 0.50 m/s are below the filter
share and should not change. If the refit shows a worst case well above 17 %, either v_rim
is lower under some load than measured (it is a torque cap, so load-dependent) or slip
is not the only remaining cause; either would be reported, not absorbed into the bound.

**The robust bound (§3, κ̄) is refitted to the residual slip only**, from runs made with
the budget in force. Saturation does not enter it: the budget removes it by construction,
and a bound that carried it would make the barrier conservative against a disturbance the
system no longer has.

**Caveat on the slip itself.** It arises at the contact of CoppeliaSim's *hidden sphere*
wheel proxies (`wheel_respondable_*`, confirmed by the researcher 2026-09-18) with the
floor, not at a mecanum roller. It is a property of the simulated platform, and the
thesis reports it as such.

## 4. What the robust PFB does with it

The nominal PFB enforces, per LiDAR sector, that the stopping distance along the sector
bearing stays within the free range:

```
h(x, u) = r_ψ − e(ψ) − [ v·t_react + v²/(2·a_brake) + margin ]  ≥ 0
```

The robust form replaces the *commanded* velocity with the worst case over the disturbance
set, at three points:

1. **Speed.** The stopping-distance term uses the **estimated actual** speed, never the
   command (ruling 3, A9). The shortfall `κ` is part of the disturbance bound, not a term
   to ignore: it is the gap between what a candidate command predicts and what the robot
   does, so it bounds the error of any prediction the QP makes from a command, and it must
   not be used to justify a larger `a_brake`. Measured on ODE (G1b, first run): 8–22 % at
   0.40 m/s depending on direction; the re-run's realised-speed map is what the bound is
   fitted to.
2. **Direction.** The sector bearing at which the robot is actually approaching an obstacle
   differs from the commanded bearing by up to `arctan(γ̄·L)` over a lookahead `L`. The
   barrier must hold for **every** bearing in that cone, not just the commanded one, which
   in practice means evaluating `h` over a widened sector.
3. **Yaw.** An uncommanded yaw rate rotates the footprint during the stop. The footprint
   extent `e(ψ)` must be taken as the maximum over the yaw excursion reachable within the
   stopping time, `γ̄ · v · t_stop`.

The filter then solves the same QP as the nominal one, against the widened constraint set.
Feasibility is not automatic: a widened cone can make the QP infeasible in a tight aisle
where the nominal one is feasible, and the fallback when it does is part of the
specification still to be written.

## 5. What must be measured before this can be implemented

1. **The diagonal directions.** `γ` is measured only at `u_y = 0` and `u_x = 0`. The
   structure in §3 assumes it scales with `u_y/‖u_xy‖`; that is an assumption, not a
   measurement.
2. **The disturbance under closed-loop control.** Every number above is open-loop. G1b
   measures the same platform with the tracker engaged, and the residual disturbance under
   closed loop is what a deployed filter would actually face. **It may be far smaller**, in
   which case the robust form is cheap; the bound should be fitted to it, not to §1.
3. **Whether the bound survives the D7 physics freeze.** The probe may change `κ` and `γ`
   substantially. Fitting before the freeze would waste the work.

## 6. H6 as stated

> **H6 (exploratory).** With the wheel-speed budget enforced, the platform's residual
> motion disturbance — the residual slip (§3a; saturation excluded, since the budget removes
> it) and the lateral yaw coupling — can be bounded by a monotone function of commanded
> speed and direction, and a PFB carrying that bound retains its
> safety guarantee at a quantifiable cost in conservatism — measured as the increase in
> protective stops per 100 m against the nominal filter.

Exploratory because it was formulated after the measurements that motivate it. It is not
eligible for confirmatory testing on the test split, and the comparison it implies —
robust PFB against nominal PFB — is an addition to the ablation set, not a replacement for
any committed comparison.
