# Decision Log

Format: date · decision · reason · alternatives considered · affects.

## 2026-09-21 · Repository strategy
- **Decision:** Same Git repository; v1 tagged `v1-chapter-results`; v2 developed on branch `v2` in a new package `pfbnav/`. v1 code is not present in the v2 working tree.
- **Reason:** v1 results must stay reproducible; v2 changes environment, sensing, actuation, reward, and evaluation.
- **Alternatives:** in-place refactor (rejected: silently alters v1); separate repository (acceptable, loses shared history).

## 2026-09-21 · Holonomic actuation as primary
- **Decision:** (v_x, v_y, omega) primary; unicycle retained as ablation A1.
- **Reason:** platform fidelity; filter gains lateral authority; removes kinematic confound from H1/H2.
- **Condition:** Gate G1 must pass; otherwise revert to unicycle primary.

## 2026-09-21 · Ground-truth contact
- **Decision:** Contact from `sim.checkCollision` on the chassis shape; LiDAR thresholds never define collisions.
- **Reason:** v1 used 0.20 m from centre, below the chassis half-length.

## 2026-09-21 · 360-degree sensing
- **Decision:** Two corner-mounted scanners; explicit validity flags per bin.
- **Reason:** holonomic motion makes the v1 rear blind zone a safety confound.

## 2026-09-16 (WP0) - Acceleration limiting is isotropic in the plane
- **Decision:** `actuation.a_max` bounds the magnitude of the change in the linear
  velocity vector (vx, vy) per step; `actuation.alpha_max` bounds the yaw-rate change
  independently. The limiter runs after the velocity-box clamp.
- **Reason:** a single scalar `a_max` describes a direction-independent limit. Clipping
  vx and vy separately would let a diagonal command accelerate by a_max * sqrt(2) and
  would bend the direction of the velocity change towards an axis, which the PFB reads
  as a different braking direction. Clamping first keeps the limited command inside the
  velocity box, because the box is convex and the output lies on the segment between the
  previous command and the clamped one.
- **Alternatives:** per-axis rate limits (rejected: anisotropic, and inconsistent with a
  single scalar in the config); wheel-torque limits (rejected: not observable through the
  velocity-control interface used by every baseline).
- **Affects:** `pfbnav/sim/robot.py`, every controller, gate G1.

## 2026-09-16 (WP0) - Entities of the ground-truth contact check
- **Decision:** `sim.checkCollision(chassis_shape, everything_except_the_robot_subtree)`.
  The floor stays inside the obstacle collection.
- **Reason:** restricting entity 1 to the chassis shape keeps the wheels' floor contact
  out; subtracting the robot's own subtree from entity 2 keeps the wheels and arm links,
  which rest against the chassis, from registering as collisions. The floor is kept in
  because a chassis touching the floor means the robot has tipped or dropped, which is a
  real failure and must be counted, not hidden.
- **Alternatives:** chassis vs `sim.handle_all` (rejected: self-contact); excluding the
  floor as well (rejected: hides tip-over).
- **Affects:** `pfbnav/sim/contact.py`, collision counts, every headline metric.

## 2026-09-16 (WP0) - Scene object paths and base_ref yaw offset are configuration
- **Decision:** `sim.objects` (pose reference, chassis shape, model root, wheel joints in
  `[fl, fr, rl, rr]` order) and `robot.yaw_offset` live in `configs/base.yaml`, all marked
  VERIFY. `robot.wheel_sign` and `robot.yaw_offset` keep neutral defaults ([1, 1, 1, 1]
  and 0.0) rather than v1's measured values ([-1, -1, -1, -1] and +pi/2).
- **Reason:** those two v1 values describe the v1 scene's youBot instance, not the v2
  scene, which does not exist yet. Copying them in would encode an unverified assumption
  as a default; a neutral default fails visibly in gate G1 instead of quietly producing a
  robot that drives backwards.
- **Affects:** `configs/base.yaml`, `pfbnav/sim/robot.py`, gate G1.

## 2026-09-16 (WP0) - The chassis-geometry test loads the configured scene
- **Decision:** `tests/test_chassis_geometry.py` loads `sim.scene` rather than measuring
  whatever scene happens to be open, and fails if that file is missing.
- **Reason:** `robot.half_length` and `robot.half_width` propagate into the PFB zero set
  and the planner inflation, so the numbers recorded in the config must be traceable to a
  named scene file.
- **Affects:** `tests/test_chassis_geometry.py`.

## 2026-09-17 (WP0) - Chassis footprint and base_ref yaw offset measured from the scene
- **Decision:** `robot.half_length = 0.2854`, `robot.half_width = 0.1789`,
  `robot.yaw_offset = 1.5707963` (pi/2). The VERIFY markers on the two half-extents are
  removed; `wheel_sign` stays VERIFY.
- **Measurement:** `pytest -m sim tests/test_chassis_geometry.py` against
  `scenes/warehouse_v2.ttt` on 2026-09-17. The `/youBot` bounding box in its own frame is
  min [-0.081, -0.1785, -0.2851], max [0.081, 0.1785, 0.2851] m; that frame is X-up
  (shape y -> body x, shape z -> body y, shape x -> body z), so base_ref is a quarter turn
  from chassis forward, as it was in v1. Resulting full chassis size 0.571 x 0.358 x 0.162 m
  against the youBot datasheet's 0.580 x 0.380 x 0.140 m.
- **Reason:** the scene's geometry is what every rollout, contact check, and PFB evaluation
  actually sees, so the measured values are preferred to the datasheet nominals
  (0.29 / 0.19) that previously stood in the config.
- **Alternatives:** keeping the datasheet values (rejected: not what the simulator uses);
  padding the half-extents upward for safety (rejected: `filter.margin` 0.05 m and
  `certified.margin` 0.10 m already carry the safety allowance, and a hidden pad inside
  the footprint would double-count it and make the reported footprint disagree with the
  scene).
- **Open:** which of the two equal-area alignments is forward is a kinematic question, not
  a geometric one; gate G1 confirms it by driving the robot. The chosen alignment is the
  one whose long axis is body +x.
- **Affects:** `configs/base.yaml`, `pfbnav/safety/pfb.py` footprint, `planner.inflation`
  (circumscribed radius falls from 0.347 to 0.337 m, so the 0.35 m setting still holds),
  gate G1.

## 2026-09-17 (WP0) - Frame alignment is measured, not inferred
- **Decision:** `tightest_yaw_offset` in `pfbnav/sim/robot.py` scans the yaw offset and
  reports the one that makes the chassis box axis-aligned; the chassis-geometry test prints
  it and both equal-area alignments.
- **Reason:** the first version of the test only hinted at a rotated frame when the
  *configured* half-extents looked swapped within 0.01 m. On the 2026-09-17 run the width
  was 0.0111 m out, the hint stayed silent, and the quarter-turn error had to be inferred
  by hand from the printed extents. A diagnostic that only fires when the value it is
  checking is already nearly right is not a diagnostic.
- **Note:** for a rectangular footprint the two alignments have exactly equal area, so
  which one the scan returns depends on the grid; both are always reported.
- **Affects:** `pfbnav/sim/robot.py`, `tests/test_chassis_geometry.py`,
  `tests/test_robot_kinematics.py`.

## 2026-09-17 (WP0) - Chassis geometry confirmed; 0.001 rad residual accepted
- **Confirmation:** re-run of `pytest -m sim tests/test_chassis_geometry.py` after the
  config fix: both tests pass, measured half_length 0.2854 m and half_width 0.1789 m at
  `yaw_offset = pi/2`, difference from the config 0.0000 m on both.
- **Finding:** the alignment diagnostic puts the tightest alignment at 1.5698 rad, not
  pi/2 = 1.5708. The 0.0010 rad gap is ten times the 1e-4 rad scan resolution, so it is a
  real residual rotation between the chassis shape frame and base_ref, not discretisation
  noise. It also explains the ~0.4 mm excess seen in the first run, which could have been
  either a rotation or a translation: at pi/2 the axis-aligned box is 0.2854 x 0.1789 m
  against the tightest 0.2852 x 0.1786 m.
- **Decision:** keep `yaw_offset = 1.5707963` (pi/2) rather than 1.5698.
- **Reason:** pi/2 is the designed quarter turn and 0.001 rad (0.057 deg) is below the
  level at which the scene's geometry is meaningful. The pi/2 value also over-states the
  footprint by 0.2 / 0.3 mm rather than under-stating it, which is the safe direction for a
  safety footprint. Consequences elsewhere are negligible: a 0.057 deg heading bias against
  G1's 5 deg drift threshold, and 0.5 mm/s of spurious lateral velocity at v_max against
  G1's 10 per cent speed-error threshold.
- **Not resolved:** whether the residual lives in the base_ref dummy's placement, in the
  chassis mesh, or in a small out-of-plane tilt. The available data cannot separate them
  and G1 cannot either, since the effect is far below its thresholds. Revisit only if G1
  shows an unexplained heading bias.
- **Affects:** `configs/base.yaml` (unchanged as a result), gate G1.

## 2026-09-17 (WP0) - Wheel joint paths corrected from the scene hierarchy
- **Decision:** `sim.objects.wheels` becomes
  `/youBot/Joint/Rectangle0/rollingJoint_fl`, `.../rollingJoint_fr`,
  `/youBot/rollingJoint_rl`, `/youBot/rollingJoint_rr`.
- **Reason:** the front wheels hang off the youBot's passive front-axle joint
  `/youBot/Joint`, not off the model root, so the previous `/youBot/rollingJoint_fl` and
  `_fr` did not exist and `YouBotBase.bind()` would have failed at the first rollout.
  Established from a subtree listing of the scene on 2026-09-17, not guessed.
- **Note:** every `rollingJoint_*` carries the stock mecanum roller Script that drives its
  `slippingJoint_*`. Those scripts are part of the wheel model and must not be removed;
  v1's `check_directions.py` note about running with "the Lua script removed" referred to
  something else.
- **Gap this exposed:** no test resolved `sim.objects.wheels`, because the chassis and
  contact tests only use `chassis`, `base_ref` and `robot_tree`. Added
  `tests/test_scene_objects.py`, which resolves every configured path, reports all misses
  at once, and asserts the four wheel paths are four distinct joints -- two paths aliasing
  to one handle would command a wheel twice and leave another dead, which a
  square-driving smoke test would not notice.
- **Affects:** `configs/base.yaml`, `pfbnav/sim/robot.py` binding, gate G1.

## 2026-09-17 (WP0) - Open: the contact check watches the chassis shape only
- **Observation:** the wheels are separate shapes on their own links, outside the
  `/youBot` chassis shape that `ChassisContact` checks and that `robot.half_length` /
  `half_width` describe. The youBot datasheet width (0.380 m) exceeds the measured chassis
  width (0.358 m), which suggests the wheels stand proud of the chassis by roughly a
  centimetre a side. If so, a side obstacle can strike a wheel without touching the
  chassis, and hard rule 4's contact definition would not see it.
- **Status:** not acted on. The overhang is inferred from a datasheet, so
  `tests/test_chassis_geometry.py::test_wheels_against_the_chassis_footprint` measures the
  chassis, wheel and whole-tree envelopes and reports the difference. The decision about
  what to do waits on that measurement.
- **Options if the overhang is real:** widen the footprint to the wheel envelope while
  keeping the contact entity as the chassis; or check chassis against everything and the
  wheels against everything-but-the-floor, unioning the results, which would preserve
  tip-over detection while catching wheel strikes.
- **Affects:** `pfbnav/sim/contact.py`, `configs/base.yaml`, `pfbnav/safety/pfb.py` footprint.

## 2026-09-17 (WP0) - Simulator-test fixtures carry no optional work
- **Decision:** the session-scoped `sim_client` fixture in `tests/conftest.py` does only
  what is needed to return a working client: connect, load the scene, start. Diagnostics
  and convenience readouts go in individual tests.
- **Reason:** a version print added to the fixture used
  `sim.stringparam_application_version`, which does not exist in CoppeliaSim 4.10. It
  errored all nine sim-marked tests at setup and cost a full run cycle for a line that
  served no purpose -- the build was already known. Work in a session fixture is a single
  point of failure for every test behind it.
- **Environment of record:** CoppeliaSim 4.10 Edu, Bullet 2.83, ZMQ remote API on port
  23000, Python 3.13.13, pytest 9.1.1.
- **Affects:** `tests/conftest.py`.

## 2026-09-17 (WP0) - Contact path verified; bounding-box envelope is an upper bound
- **Verified:** `pytest -m sim` passes 9/9 against `scenes/warehouse_v2.ttt`. The obstacle
  collection holds 52 objects and leaks nothing from the 64-object robot subtree; the
  chassis is clear at spawn; a cuboid placed inside the chassis is detected and the reading
  clears when it is removed. Every `sim.objects` path resolves and the four wheel joints
  are four distinct joints.
- **Finding (positive):** obstacle collections built from an "all objects" rule are
  evaluated lazily -- an object created after `bind()` is detected without rebinding. WP1
  scenario generation therefore does not need to rebind after spawning obstacles.
- **Finding (method error, mine):** `test_wheels_against_the_chassis_footprint` bounds each
  shape's own axis-aligned box and then bounds the rotated corners. That is exact for the
  chassis, whose box is axis-aligned in the body frame to 0.001 rad, but conservative for
  the wheels, whose frames are rolled about their axles. It reported the wheels reaching
  0.3133 m along body +x where `half_wheelbase + wheel_radius` = 0.278 m is the geometric
  maximum, so at least 0.035 m of that figure is inflation and the +0.0648 m width figure
  is contaminated the same way. The numbers are upper bounds and are labelled as such.
- **Open:** the *direction* of the finding stands -- the chassis measures 0.1789 m
  half-width against a datasheet 0.19 m to the wheel's outer face -- so a side obstacle may
  strike a wheel without touching the chassis, which `ChassisContact` would not count.
  Under-counting collisions is the dangerous direction for the project's headline metric.
- **Next:** `tests/test_footprint_probe.py` sweeps a thin plate along each body axis and
  asks `sim.checkCollision` where the chassis, the wheels and the whole robot stop
  responding. That measures the same quantity with the API that defines a collision,
  at 0.0025 m resolution. The decision on whether to widen the contact entity waits on it.
- **Affects:** `pfbnav/sim/contact.py`, `configs/base.yaml`, `pfbnav/safety/pfb.py` footprint.

## 2026-09-17 (WP0) - Probe sweep: orientation bug, and what the guard assertions bought
- **What happened:** the first run of `tests/test_footprint_probe.py` set the probe plate's
  position relative to base_ref but never its orientation, so the slab kept the
  world-aligned orientation it was created with and its thin axis did not point along the
  swept body axis. Chassis and wheels never responded; the whole-robot group responded at
  what was merely the last offset tried.
- **Why it did not become a wrong number:** the test asserts that the chassis responds
  somewhere in a range that starts inside the chassis, so it failed loudly instead of
  reporting 0.3030 m as an edge. A probe-based measurement has no internal check that the
  probe is where it is believed to be, so that assertion is the measurement's only
  guarantee and is not optional.
- **Fix:** `_make_plate` now sets the plate's orientation relative to base_ref to
  [0, 0, yaw_offset], aligning its axes with the body frame. The sweep additionally records
  the full contact profile, asserts it is a clean inside-to-outside step with no re-entry,
  and prints it, so a misplaced probe shows up as a pattern rather than as a plausible
  number.
- **Affects:** `tests/test_footprint_probe.py`.

## 2026-09-17 (WP0) - Chassis half-width cross-validated by a second method
- **Result:** a collision probe swept laterally in the CoppeliaSim sandbox touches the
  chassis at plate-centre offsets up to 0.180 m and misses from 0.190 m. With a 0.003 m
  plate half-thickness that puts the chassis face in (0.177, 0.187] m, containing the
  0.1789 m obtained from the bounding-box transform. Two methods that share no code and no
  assumptions agree, so `robot.half_width` is confirmed rather than merely measured once.
- **Reporting bug found and fixed:** the probe test printed the outer face as
  `edge + half_plate`, which overstates it by a plate thickness (0.183 m against a true
  0.1789 m). Contact at offset o means face > o - half, and the first miss gives
  face <= o + step - half, so the face is now reported as that bracket. Differences between
  groups measured with the same plate are unaffected, since the thickness cancels.
- **Added:** the probe test now asserts its own chassis edge against the bounding-box value
  in the config. Two independent measurements of one edge disagreeing means one is wrong,
  which must stop the run rather than be averaged away.
- **Affects:** `tests/test_footprint_probe.py`, `configs/base.yaml` (confirmed, unchanged).

## 2026-09-17 (WP0) - Collision definition widened to chassis + wheels
- **Measurement:** a collision probe swept along each body axis at 0.0025 m gives, in the
  body frame: chassis face 0.1789 m lateral and 0.2854 m forward; wheels' outer face
  (0.1945, 0.1970] lateral and (0.2770, 0.2795] forward; arm reaching (0.3495, 0.3520]
  forward. The wheels therefore stand about 17 mm proud of the chassis laterally and do
  not protrude forward; the arm protrudes about 65 mm forward.
- **Method confirmed by three independent checks:** the wheels' forward face lands on
  `half_wheelbase + wheel_radius` = 0.278 m exactly; the whole-robot forward profile shows
  a gap whose first segment ends at the chassis face, bracketing 0.2854 m; and the arm's
  reach matches the bounding-box estimate 0.3501 m, the one case where that method is
  accurate.
- **Decision (researcher):** a ground-truth collision is the union of two checks --
  the chassis against every object outside the robot subtree including the floor, and the
  wheels against the same set minus the floor. Class renamed `ChassisContact` ->
  `RobotContact`; `sim.objects.floor` added to the config.
- **Reason:** with a chassis-only entity, a wall in the 17 mm band between the chassis
  face and the wheels' outer face is a real collision that reports clear. Under-counting
  collisions biases the project's headline safety metric in the dangerous direction. The
  floor stays in the chassis set so a tip-over is still counted, and out of the wheel set
  because the wheels rest on it by design.
- **Decision (researcher):** the arm will be parked inside the base envelope in the scene,
  rather than widening the footprint to 0.3501 m. The PFB footprint is a symmetric
  rectangle, so covering a forward-only protrusion would inflate the rear by the same
  65 mm and make the robot behave as if it were 13 cm longer than it is.
- **Config:** `robot.half_length` / `half_width` now describe the robot envelope, not the
  chassis. `half_width` 0.1789 -> 0.1970, the upper bound of the measured wheel bracket,
  because under-stating a safety footprint is the unsafe direction. `half_length` stays
  0.2854: the chassis governs there.
- **Cost:** two `checkCollision` calls per physics step instead of one. Relevant to gate G2.
- **Affects:** `pfbnav/sim/contact.py`, `pfbnav/sim/client.py`, `pfbnav/sim/robot.py`
  (SceneObjects), `configs/base.yaml`, `pfbnav/safety/pfb.py` footprint, gate G2.

## 2026-09-17 (WP0) - Forward overhang misattributed to the arm; diagnostic added
- **Correction:** the ~65 mm of collidable geometry ahead of the chassis was reported as
  "the arm". The researcher confirms the arm folds within the base, so that attribution was
  a guess and it was wrong.
- **Current hypothesis:** `/youBot/LiDAR/body`. The LiDAR sits at +0.325 m along body
  forward in the `/youBot` frame, it is a shape, and the forward sweep profile shows a gap
  between the chassis segment (ending 0.2875) and a separate segment (0.2975 to 0.35) --
  the signature of a pod mounted ahead of the bumper rather than something folded on top.
  Not asserted; to be confirmed by measurement.
- **Fix to the method, not just the answer:** an envelope measurement says how far
  something protrudes but not what it is, and attributing it wrongly sends the fix to the
  wrong place. `test_which_parts_reach_past_the_configured_footprint` now lists every robot
  shape by its overhang against the configured envelope, so the protruding part is named.
- **Open decision:** if the LiDAR pod is confirmed, either it counts as collidable robot
  geometry (`half_length` -> ~0.352, which also inflates the rear by 65 mm because the PFB
  footprint is a symmetric rectangle) or it is moved back inside the chassis outline in the
  scene. Moving it is preferred for the same reason the arm was parked.
- **Noted from v1:** `bake_occupancy_map.py` discovers obstacles as all scene shapes minus
  the `/youBot` and `/goal_dummy` subtrees, minus aliases containing "floor" or "ground".
  This project's contact check is structural only, naming the floor explicitly as
  `sim.objects.floor`. v1 also reads `objfloatparam_modelbbox_*`, the whole model's box;
  the chassis measurement here uses `objbbox`, the object's own geometry, which is why it
  returns the chassis and not the entire robot.
- **Affects:** `tests/test_chassis_geometry.py`, `configs/base.yaml` (pending), WP1 baking.

## 2026-09-17 (WP0) - LiDAR pods are robot geometry; contact entity made structural
- **Confirmed:** the forward overhang is `/youBot/LiDAR/body`, sitting 0.325 m along body
  forward and reaching (0.3495, 0.3520] m. Not the arm, which folds within the base.
- **Decision (researcher):** the LiDAR stays outside the chassis and counts as part of the
  robot. A second LiDAR was fitted behind the robot at the same stand-off on 2026-09-17, so
  the envelope is symmetric fore and aft.
- **Consequence for the contact entity:** the LiDAR is robot geometry that an obstacle can
  strike, but it was not in the contact entity, which is the same defect just fixed for the
  wheels and four times larger (65 mm against 17 mm). Rather than add "LiDAR" to a list,
  the split is now structural: the chassis against everything outside the robot subtree
  including the floor, and *every other shape in the robot subtree* against the same set
  minus the floor. This supersedes the chassis-plus-wheels choice recorded earlier today.
- **Reason for structural over enumerated:** a list of parts goes stale. The second LiDAR
  was covered by this rule the moment it was added, with no code change; an enumerated list
  would have silently excluded it until someone remembered.
- **Config:** `robot.half_length` 0.2854 -> 0.3520, the upper bound of the measured LiDAR
  bracket, matching how `half_width` was chosen. The envelope now describes the chassis,
  the wheels and the sensor pods.
- **Consequence flagged, not fixed:** the circumscribed radius is now
  hypot(0.3520, 0.1970) = 0.4034 m, so `planner.inflation: 0.35` no longer satisfies its
  own stated constraint of being at least the circumscribed radius. Marked VERIFY in the
  config; WP1 sets it.
- **Method gap closed:** the probe sweep measured only +x and +y, so a rear pod would never
  have appeared. It now sweeps -x and -y as well. The robot is not symmetric a priori, and
  the forward LiDAR had no counterpart at all until today.
- **Affects:** `pfbnav/sim/contact.py`, `configs/base.yaml`, `tests/test_contact.py`,
  `tests/test_contact_sim.py`, `tests/test_footprint_probe.py`, `planner.inflation` (WP1).

## 2026-09-17 (WP0) - Envelope confirmed symmetric; contact covers both sensor pods
- **Verified:** 15 of 16 sim tests pass. The rear LiDAR is at the same stand-off as the
  front: both the forward and rear sweeps put the whole-robot face in (0.3495, 0.3520] m,
  and left and right both in (0.1945, 0.1970] m. `half_length = 0.3520` and
  `half_width = 0.1970` are therefore exact rather than conservative.
- **Regression tests pass:** a wall at 0.188 m lateral is caught by the attachments check
  via `swedishWheel_rl`, and a wall at 0.320 m forward via `/youBot/LiDAR[0]/body`. Both
  print that the chassis-only check would have reported clear. Those are the two
  under-counts fixed today, demonstrated rather than argued.
- **Cross-checks:** the probe puts the chassis face at (0.2845, 0.2870] fore/aft and
  (0.1770, 0.1795] lateral, containing the bounding-box values 0.2854 and 0.1789; the
  wheels' fore/aft face at (0.2770, 0.2795] contains `half_wheelbase + wheel_radius`
  = 0.278. Three methods agree.
- **Process failure, mine:** two edits in a multi-replacement patch silently failed to
  match, leaving `test_chassis_fits_inside_the_configured_footprint` with its previous
  assertion (`abs(d_length) <= 0.01`, comparing the chassis against what is now the
  envelope) and the probe report with stale text. Only the config edits in that batch had
  their match counts asserted. Edits that change an assertion must fail loudly when the
  pattern does not match, or a test silently keeps checking the wrong thing while the
  surrounding prose says otherwise.
- **Fixed:** the chassis assertion is now signed -- the chassis must not *exceed* the
  envelope, being smaller is the normal case -- and the frame alignment is asserted
  against the 0.005 rad accepted residual, which the measured 0.0010 rad satisfies.
- **Affects:** `tests/test_chassis_geometry.py`, `tests/test_footprint_probe.py`.

## 2026-09-17 (WP0) - Wheel layout measured; half_track corrected
- **Measured** (wheel joint positions in the body frame, m): fl (+0.228, +0.1585),
  fr (+0.228, -0.1586), rl (-0.228, +0.1585), rr (-0.228, -0.1586), axles at z = 0.0504.
- **Result:** every `[fl, fr, rl, rr]` label matches its physical quadrant, so the mecanum
  mixing drives the wheels it believes it is driving. `half_wheelbase` 0.228 is exact;
  `half_track` 0.158 -> 0.1585 (measured; 0.1 mm left/right asymmetry ignored). L+W moves
  from 0.386 to 0.3865, a 0.13 per cent change in the yaw term.
- **Corroboration:** the body frame derived from the *chassis bounding box* turns the wheel
  joint positions into a symmetric rectangle about the origin. Two unrelated pieces of
  geometry agree on `yaw_offset = pi/2`, which no single measurement could establish.
- **Cross-check, not acted on:** the axles sit 0.0504 m above base_ref against
  `wheel_radius = 0.05`. That is consistent with base_ref at floor level, but it is not a
  measurement of the radius. Wheel radius scales every commanded velocity, so gate G1's
  mean relative speed error is where it gets settled; a 0.0504 m radius would show as a
  0.8 per cent speed bias, well inside G1's 10 per cent threshold but not zero.
- **Made permanent:** `tests/test_scene_objects.py::test_wheel_layout_matches_the_configuration`
  asserts the quadrant of each wheel and its distance from the configured lever arms.
  Swapped labels produce a robot that moves and still closes a square while mixing the
  wrong wheels, so this cannot be left to a smoke test.
- **Affects:** `configs/base.yaml`, `pfbnav/sim/robot.py` (`ref_point_in_body`),
  `tests/test_scene_objects.py`, `tests/test_robot_kinematics.py`.

## 2026-09-17 (WP0) - Single-object collections do not work for collision checks
- **Confirmed behaviour of CoppeliaSim 4.10:** a collection created with
  `createCollection(0)` holding exactly one object reports no collision where the same
  object, passed as a raw handle, reports one. `getCollectionObjects` confirms the member
  is present: `members [33]`, `raw 1, collection 0`.
- **Handled:** `SimClient.entity_for` returns the handle itself for a single object and
  builds a collection only for two or more. Multi-item collections behave correctly -- the
  33-shape attachments collection detects strikes on both `swedishWheel_rl` and
  `LiDAR[0]/body`.
- **Open:** whether a multi-item collection could silently drop its *first* member. If it
  did, `RobotContact` would under-count. A two-item collection test settles it; until then
  the risk is noted rather than dismissed, because a silent under-count on the collision
  path is the worst failure this project can have.
- **Affects:** `pfbnav/sim/client.py`, `pfbnav/sim/contact.py`.

## 2026-09-17 - Supervisor review of WP0: items 1-3 accepted, D1-D5 recorded

### D1 - A* uses two-tier inflation; the global planner is not footprint-complete
- **Decision:** lethal inflation = inscribed radius + 0.03 m; a soft cost decays out to the
  circumscribed radius + margin. The global planner is intentionally not
  footprint-complete; the local planner and the filters handle the rest.
- **Config:** `planner.inflation: 0.35` is replaced by `planner.lethal_margin: 0.03` and
  `planner.soft_margin: 0.03` (VERIFY -- D1 left the soft margin open). Margins are stored
  rather than summed radii, because the inscribed radius *is* `robot.half_width` (0.1970)
  and the circumscribed radius is hypot(half_length, half_width) = 0.4034; a copied number
  would go stale the next time the envelope is measured. Lethal works out at 0.2270 m,
  soft at 0.4334 m.
- **Supersedes** the note added earlier today that `inflation: 0.35` violated its own
  stated constraint. The constraint itself was the wrong shape: one radius cannot be both
  admissible for planning and conservative for safety.
- **Affects:** `configs/base.yaml`, `pfbnav/planning/` (WP1), `pfbnav/world/baking.py` (WP1).

### D2 - Aisle strata defined by per-side clearance
- **Decision:** `robot.half_width` takes the measured value (already done today: 0.1970 m,
  the wheels' outer face). Aisle strata in the layout generator are defined by *per-side
  clearance*, not by aisle width. The certified layer's lateral margin must be strictly
  below the narrowest per-side clearance, or the robot would trip a protective stop merely
  by standing in the narrowest aisle.
- **Required test (blocked on WP1):** a stationary robot centred in the narrowest aisle
  must trigger no protective stop. It cannot be written yet -- `pfbnav/safety/certified.py`
  is a WP1 interface stub and the layout generator does not exist -- so it is recorded here
  as a gate on WP1 rather than left implicit.
- **Constraint to carry:** `certified.margin` is 0.10 m today. Once the narrowest stratum
  is fixed, that value must be checked against it, and the check belongs in the test above.
- **Affects:** `pfbnav/world/` (WP1), `pfbnav/safety/certified.py` (WP1), `configs/base.yaml`.

### D3 - Pod positions and envelope symmetry reported; rectangle envelope kept
- **Reported:** the forward LiDAR pod `/youBot/LiDAR/body` sits at +0.325 m along body x and
  reaches (0.3495, 0.3520] m. A second pod was fitted behind at the same stand-off on
  2026-09-17. The probe sweep measures the whole-robot face at (0.3495, 0.3520] m both
  forward and rearward, and at (0.1945, 0.1970] m both left and right: the envelope is
  symmetric on both axes to within the 0.0025 m sweep resolution.
- **Decision:** keep the rectangle envelope. The symmetry is what makes it honest -- a
  rectangle covering a forward-only protrusion would have inflated the rear by the same
  65 mm for nothing.
- **Affects:** `pfbnav/safety/pfb.py` footprint, `configs/base.yaml`.

### D4 - Gate G0 (contact validation) before G1
- **Decision:** implement and pass G0 before G1, per the `G0_contact_validation` block
  committed in `preregistration/gates.yaml`: five positive cases at 0.1 m/s approach,
  detection lag at most 1 step against a geometric prediction, a
  `parallel_pass_gap_10mm` negative case with zero tolerated false contacts, in both the
  `gate_script` and `env_step_loop` contexts.
- **Reference used for the geometric prediction:** `sim.checkDistance` between the robot
  and the obstacle, which is independent of `sim.checkCollision`. Predicting the contact
  step from the commanded speed instead would fold in the acceleration limiter and the
  wheel radius, making the gate a test of the model rather than of the contact check.
- **Known limitation, not a relaxation:** the `env_step_loop` context cannot run until the
  Gymnasium environment exists in WP1. The script therefore reports that context as
  NOT_RUN and the overall verdict as INCOMPLETE rather than PASS. G0 is not passed until
  both contexts pass.
- **Affects:** `scripts/gate_g0_contact.py`, `pfbnav/eval/gates.py`, WP1 environment.

### D5 - The claim that v1 "invented" false contacts was overstated
- **What was claimed:** that v1's 0.20 m LiDAR collision threshold "both missed real
  contacts and invented false ones".
- **Evidence for the first half (holds):** v1 declared a collision when the minimum binned
  range fell below `COLLISION_THRESH = 0.20` m from the robot centre
  (`YouBot/YouBotEnv.py:101`, applied at line 808 as `min_range_m < COLLISION_THRESH`). The
  chassis front face measures 0.2854 m from centre. A head-on wall contact therefore
  returns about 0.2854 m and is never declared. v1 also had a single front scanner with a
  rear blind zone, so rear contact was invisible in principle.
- **Evidence for the second half (does not hold as stated):** a false contact needs the
  measured range below 0.20 m with nothing touching. The robot's half-extent is below
  0.20 m only on the side faces -- beyond roughly 63 degrees from forward, falling to
  0.1789 m (chassis) and 0.1958 m (wheels) at 90 degrees. So an obstacle 0.196 to 0.200 m
  abeam is not touching yet is below threshold. That band is at most 21 mm, and about 4 mm
  where the wheels are. The geometry makes false contacts *possible*; it does not show they
  *occurred*. Their frequency depends on how often v1's policies passed within 2 cm of a
  wall abeam, and v1 inflated obstacles by `ROBOT_RADIUS = 0.50` m in its planner, which
  would tend to keep the robot much further out.
- **Correction:** the claim is narrowed to "v1's threshold misses real contacts, and admits
  false ones in a narrow lateral band whose realised rate is unmeasured". The published
  supervisor summary is corrected to match.
- **To settle it empirically:** a Run Request is prepared (not run) to replay the v1
  checkpoints on a branch cut from tag `v1-chapter-results` with a ground-truth contact
  logger recording both verdicts per step. Read-only with respect to the checkpoints
  (hard rule 10).
- **Affects:** `docs/decision_log.md`, the published WP0 summary, a future v1 re-evaluation.

## 2026-09-17 (WP0) - G0 and G1 implemented; environment and lint brought up to date
- **Implemented:** `scripts/gate_g0_contact.py` and `scripts/gate_g1_fidelity.py`, both
  reading their thresholds from `preregistration/gates.yaml` and writing a timestamped run
  directory through `create_run_dir` (`runs/gates/G0_<stamp>_seed<n>/`). The docstring's
  `G0_<timestamp>/` form is honoured inside the experiment name so that the project's
  never-overwrite guarantee still applies.
- **New metrics, all with known-answer tests first (hard rule 6):** `pfbnav/eval/gates.py`
  provides `unwrap_angles`, `path_length`, `steady_state_start`, `relative_speed_error`,
  `heading_drift_deg`, `detection_lag_steps`, `count_contact_steps`; 32 synthetic cases in
  `tests/test_gate_metrics.py`.
- **G1 reports a diagnosis, not only a verdict:** it logs the body velocity twice, once
  from the pose of the reference frame and once reconstructed from the wheel joints. The
  two share no code path, so a robot moving against its command with both readouts
  agreeing means `robot.wheel_sign` is wrong, while a disagreement means the frame
  convention or the lever arms are. That distinction is the point of the gate.
- **Environment:** `pip install -e ".[dev]"` had never been run. `pfbnav` is importable
  only because the working directory happens to be on the path, so `python scripts/...`
  fails outright with ModuleNotFoundError. Three interpreters are present on this machine
  (3.10.10 for this session, miniconda at 3.13, and the 3.13.13 that the researcher's
  pytest runs report), so the editable install must be done with the interpreter that will
  run the gates.
- **Lint:** `ruff check .` ran for the first time and reported 20 findings, now zero. Eleven
  were modernisation fixes. Ten were ISC004, implicit string concatenation inside list
  literals -- syntactically indistinguishable from a forgotten comma, which is a failure
  mode that has already cost this project two silent bugs, so they are now parenthesised
  rather than suppressed.
- **Open question raised, not resolved:** `pod_corner` in the committed G0 case list is
  ambiguous. "Pod" was this session's coinage for the LiDAR housing, but it is also the
  standard warehouse term for a mobile shelving unit, in which case the case means an
  obstacle corner rather than a robot part. The case is implemented as a box turned 45
  degrees so that a corner leads, positioned to meet the forward LiDAR housing, which
  satisfies both readings; the intended one should be confirmed.
- **Affects:** `scripts/`, `pfbnav/eval/gates.py`, `tests/test_gate_metrics.py`,
  `pyproject.toml` environment assumptions.

## 2026-09-17 (WP0) - `pod_corner` resolved; environment settled
- **Resolved:** "pod" means the LiDAR housing (researcher, 2026-09-17), so `pod_corner` is
  a robot part being struck, parallel to `wheel_edge`, not an obstacle corner. The case was
  implemented as a 45-degree box meeting the housing's front *face*, which was the wrong
  contact feature; it is now a 0.04 m post stood at the housing's front-outer corner so
  that the corner leads.
- **Placed by measurement, not by path:** `measure_forward_part` finds the robot shape with
  the greatest forward reach and takes its lateral half-extent. Hard-coding
  `/youBot/LiDAR/body` would have broken the day the second housing was fitted, when
  CoppeliaSim renamed the first to `LiDAR[0]`.
- **Environment settled:** the researcher will use Python 3.10.10, the same interpreter as
  this session. `python -m pip install -e ".[dev]"` has been run into it, so
  `python scripts/gate_g0_contact.py` now imports `pfbnav` without a PYTHONPATH prefix.
  `ruff check .` passes with zero findings.
- **Affects:** `scripts/gate_g0_contact.py`, `pfbnav/sim/client.py` (`object_bbox`).

## 2026-09-17 (WP0) - Gate G0 run 1: FAIL. wheel_sign measured as [-1, -1, -1, -1]
- **Result:** `runs/gates/G0_20260917T151954Z_seed0` (and an identical earlier run at
  T151611Z). All five positive cases failed; the negative case "passed" vacuously.
- **What happened:** the robot drove backwards on every command. Commanded +0.100 m/s along
  body +x, realised -0.0922 m/s; commanded +0.100 m/s along body +y, realised -0.0920 m/s.
  Every `min_gap` therefore equals its starting value, because the robot retreated from
  each obstacle instead of approaching it. `parallel_pass_gap_10mm` reported zero contacts
  while driving 1.02 m *away* from the wall and off the end of it, so that pass carries no
  information and is not evidence of anything.
- **Diagnosis:** both translation axes inverted together. Negating all four wheel speeds
  negates (vx, vy, wz) as a set, which is exactly that signature. The alternative
  explanation, `yaw_offset` being pi out, produces an identical signature in body
  coordinates -- but the wheel-layout test measured the joints named fl and fr at body +x,
  and a frame pi out would have put the front wheels at the rear. The frame is right, so
  the sign vector is wrong.
- **Config:** `robot.wheel_sign` [1, 1, 1, 1] -> [-1, -1, -1, -1]. The VERIFY marker is
  removed: this is now measured, not assumed. It matches what the v1 scene required, which
  was an expectation this project deliberately refused to encode until it was measured.
- **Bug found in my own diagnostic, which would have mis-attributed this:** G1's
  `diagnose()` treated "pose and wheels disagree" as pointing at the frame convention.
  That is backwards. `body_from_wheels` divides by the *configured* sign vector, so a wrong
  sign vector makes the wheel reconstruction return the command that was intended while the
  pose shows the opposite -- disagreement is the expected signature of a sign error, and
  both faults produce it. The logic moved to `pfbnav.eval.gates.diagnose_direction`, a pure
  function with six known-answer cases including this run's exact numbers, and it now names
  both candidates and says which test separates them instead of picking one.
- **Also noted, not acted on:** the realised speed was 7.8 per cent below the command at
  0.1 m/s. Inside G1's 10 per cent threshold, but not comfortably, and G1 measures at
  0.5 m/s where roller friction matters less. Worth reading the number rather than the
  verdict.
- **Provenance:** both runs record `commit: HEAD, dirty: true` -- the repository has no
  commits yet, so gate results are not currently traceable to a tree state. That should be
  fixed before any result is cited.
- **Affects:** `configs/base.yaml`, `pfbnav/eval/gates.py`, `scripts/gate_g1_fidelity.py`,
  `tests/test_gate_metrics.py`.

## 2026-09-17 (WP0) - Gate G0 run 2: all gate_script cases pass; verdict INCOMPLETE
- **Result:** `runs/gates/G0_20260917T153135Z_seed0`. All six cases pass in the
  `gate_script` context. Overall verdict INCOMPLETE, not PASS, because `gates.yaml`
  requires `env_step_loop` as well and that context awaits the WP1 environment.
- **Detection lag is 0 on every positive case**, against the committed allowance of 1 step:
  `sim.checkCollision` fires on the same step that `sim.checkDistance`, an independent
  primitive, first reports the surfaces meeting.
- **Each case struck the part it was aimed at:** front wall -> `/youBot/LiDAR[0]/body`;
  rear wall -> `/youBot/LiDAR[1]/body` (which also settles which housing is which);
  side wall and wheel_edge -> `swedishWheel_fl`; pod_corner -> `/youBot/LiDAR[0]/body` at
  its corner. `wheel_edge` did not touch the chassis, so the 2.8 mm slab clearance held and
  the case tested what it claims to.
- **The widened contact entity is now demonstrated on moving approaches, not only static
  probes.** `wheel_edge` is a collision through a wheel with the chassis clear, and
  `rear_wall` one through a sensor housing: the 17 mm and 65 mm under-counts found earlier
  today, caught as real collisions at 0.1 m/s.
- **`wheel_sign = [-1, -1, -1, -1]` confirmed by motion:** +0.0927 m/s realised against
  +0.100 m/s commanded, in the commanded direction, on both translation axes.
- **The negative case is now informative.** In run 1 it reported zero contacts while driving
  away from the wall; here it travels +1.023 m alongside it at a 10.8 mm gap and still
  reports zero. Lateral drift over that metre was 1.2 mm, about 0.07 degrees.
- **Carried into G1:** the realised speed is 7.3 per cent below command at 0.1 m/s, inside
  the 10 per cent criterion but not comfortably; and yaw direction is still unverified,
  since every G0 case commanded zero yaw rate.
- **Affects:** `configs/base.yaml` (wheel_sign confirmed), gate G1 next per D4.

## 2026-09-17 (WP0) - Gate G1: FAIL on all four profiles. Cause is wheel slip, not control
- **Result:** `runs/gates/G1_20260917T153746Z_seed0`, FAIL. Committed criteria are
  max_speed_error 0.10 and max_heading_drift 5.0 deg over 5 m.

  | profile | wheels (err) | body speed | slip | drift |
  |---|---|---|---|---|
  | forward | 0.5000 (0.0004) | 0.315 | 37% | 26.2 deg |
  | lateral | 0.2982 (0.0002) | 0.228 | 24% | 97.7 deg |
  | diagonal_45 | 0.5752 (0.0002) | 0.212 | 63% | 8.3 deg |
  | translate_while_rotating | 0.4928 (0.0004) | 0.315 | 36% | 177.9 deg |

- **The actuation chain is correct.** The velocity reconstructed from the wheel joints
  tracks the command to within 0.0004 relative on every profile, and the direction is right
  on all four, which independently confirms `wheel_sign = [-1,-1,-1,-1]`. The entire loss
  sits between the wheel hubs and the floor: steady sliding, reached within about 20 steps
  and flat thereafter, not a transient and not a controller error.
- **The preregistered fallback does not address this failure.** `gates.yaml` offers
  "unicycle primary; holonomic becomes caveated ablation", which assumes the failure is
  specific to holonomy. It is not: the `forward` profile alone slips 37 per cent, and a
  unicycle youBot would use the same wheels on the same floor. Changing the actuation model
  would not recover the tracking. This should be put to the supervisor before the fallback
  is invoked, because invoking it would concede the contribution's main platform claim for
  no benefit.
- **Consequence beyond the gate:** `filter.a_brake` and `certified.a_brake` are both
  1.0 m/s^2. If the contact cannot sustain the traction to reach 0.5 m/s, its ability to
  *decelerate* at 1.0 m/s^2 is in doubt, and both the PFB's stopping distance and the
  certified layer's speed-banded fields rest on that number. This is safety-relevant, not
  cosmetic.
- **Ruled out first, because it is mine:** `SimClient.start()` sets
  `floatparam_simulation_time_step` from the config on every start, overriding whatever the
  scene was authored with. Coarsening the physics step is a known cause of exactly this
  slip signature, so the scene's native step must be read before anything else is
  investigated.
- **Other candidates, in the order they can be checked:** the mecanum roller emulation
  (`slippingJoint_*`) not turning; friction or mass properties of `wheel_respondable_*`
  against the floor; and load distribution across the passive front axle, which the 26 deg
  of yaw on a straight-line profile would be consistent with.
- **Noted:** v1 commanded the same `V_MAX = 0.5` m/s on this robot but had no fidelity
  gate, so it never measured tracking. This slip may be a long-standing property of the
  model that v1's agents simply learned around, rather than anything new.
- **Added:** `--speed-scale` on the G1 script, which reports `DIAGNOSTIC` rather than a
  gate verdict for any value other than 1.0, and `mean_slip_fraction` per profile, so the
  slip-versus-speed curve can be measured without restating the criteria.
- **Affects:** `preregistration/gates.yaml` fallback adequacy, `configs/base.yaml`
  a_brake values, `scripts/gate_g1_fidelity.py`, WP1 environment design.

## 2026-09-17 (WP0) - G1 slip: wheel radius ruled out, slip is genuine and speed-dependent
- **Hypothesis tested and rejected:** that `robot.wheel_radius = 0.05` was wrong and the
  "slip" was a unit-conversion error in the kinematics. The forward ratio implied an
  effective radius of 0.0320 m, which fitted the forward profile to 1.5 per cent. Direct
  measurement refutes it: `wheel_respondable_fl` and `_rl` are 0.1 x 0.1 x 0.1 m spheres
  centred at world z = 0.05, with `base_ref` at z = -0.0004. The radius is 0.05 m exactly,
  as configured. `robot.wheel_radius` is now measured rather than inherited.
- **Also ruled out:** that `SimClient.start()` coarsened the physics. The scene's native
  step is 0.05 s with a 0.005 s dynamics step on Bullet, so the override is a no-op.
- **Therefore the loss is genuine contact slip**, and it scales with speed:

  | commanded | wheels | body | slip |
  |---|---|---|---|
  | 0.100 m/s (G0) | 0.100 | 0.0927 | 7.3% |
  | 0.500 m/s (G1) | 0.4928 | 0.3152 | 36.0% |

  Both points fit slip_fraction = 0.73 * v to within 0.5 percentage points, so the absolute
  loss grows with v^2 -- drag-like, not a constant offset.
- **Consequence if that law holds:** the committed `max_speed_error` of 0.10 is satisfiable
  only below about 0.137 m/s, while `actuation.v_max` is 0.5. The platform would be running
  roughly 3.6 times faster than it can track to the preregistered criterion. That is a
  prediction, testable by a speed sweep, not yet a conclusion.
- **Method note, mine:** the roller-velocity check was asked for in the wrong state (the
  robot was stationary) and would in any case have been uninformative, because ideal
  mecanum kinematics give zero roller motion during pure forward translation. But the wheel
  being a *sphere* means the mecanum behaviour is produced by the child script on each
  rollingJoint, not by roller geometry, so reasoning from ideal kinematics was unsound from
  the start. The script's contents are the next thing to read.
- **Still unexplained:** 26 degrees of heading drift on a straight-line profile. Symmetric
  slip cannot cause veer, so there is an asymmetry on top of the slip.
- **Open for the supervisor:** whether to pursue the physics, lower `actuation.v_max` to a
  trackable speed and re-run G1, or treat commanded velocity as a request and rely on
  measured velocity throughout. The third option is close to what the PFB already does,
  since it takes velocity as an input, but `filter.a_brake` and `certified.a_brake` at
  1.0 m/s^2 remain unverified and a platform that cannot reach 0.5 m/s may not be able to
  shed it at 1.0 m/s^2 either.
- **Affects:** `configs/base.yaml` (`actuation.v_max`, both `a_brake` values),
  `preregistration/gates.yaml` fallback adequacy, WP1 environment design.

## 2026-09-17 (WP0) - The mecanum emulation is stock and correct; the slip is inherent
- **Read the wheel script.** Each `rollingJoint_*/Script` runs, in `sysCall_actuation`,
  `sim.setObjectOrientation(slipping|sim.handleflag_reljointbaseframe, {-pi/4,0,0}, rolling)`:
  the stock CoppeliaSim mecanum emulation, re-aiming a free "slipping" joint to -45 degrees
  every step so the respondable sphere can spin about the roller axis. It is running and it
  is doing what it should. The emulation is not broken.
- **Therefore the slip is a property of the wheel model, not a fault.** A sphere with one
  driven axis and one free axis is only partially constrained; it rolls only to the extent
  friction enforces it. Reasoning from ideal mecanum roller kinematics, as this log did
  earlier today, does not apply to a sphere-based emulation.
- **Implication for v1:** the model is stock and unmodified, and v1 commanded the same
  `V_MAX = 0.5` m/s on it, so v1 very probably ran at about 0.32 m/s while believing it
  commanded 0.5, and never measured the difference. Distance-based metrics computed from
  pose are unaffected -- protective stops per 100 m counts real metres -- but any v1 claim
  about achieved speed, or any timing derived from commanded velocity, carries this bias.
  Worth stating in the thesis rather than leaving implicit.
- **Friction values not yet read:** CoppeliaSim 4.10 deprecates `getEngineFloatParam` in
  favour of a properties API whose names are not known here. They are readable from the GUI
  (shape dynamic properties). Deferred, because the friction value only matters if the
  platform's physics is to be changed mid-project, which is a larger decision than the gate.
- **Next measurement:** a speed sweep of the forward profile, which answers the question
  that actually has to be decided -- what speed the platform can track -- independently of
  the cause. Prediction from the two points held so far: slip_fraction = 0.73 * v, giving
  7.3, 14.6, 21.9 and 36.5 per cent at 0.1, 0.2, 0.3 and 0.5 m/s.
- **Affects:** `configs/base.yaml` (`actuation.v_max`), thesis statements about v1 speeds.

## 2026-09-17 (WP0) - Speed sweep: prediction falsified, a knee at 0.2-0.3 m/s
- **Measured** (forward profile, 5 m each, `--speed-scale` diagnostic runs):

  | commanded | slip | predicted 0.73v | speed error | heading drift |
  |---|---|---|---|---|
  | 0.100 | 0.0735 | 0.0730 | 0.0733 | 0.34 deg |
  | 0.200 | 0.0981 | 0.1460 | 0.0980 | 0.84 deg |
  | 0.300 | 0.1978 | 0.2190 | 0.1980 | 16.69 deg |
  | 0.500 | 0.3600 | 0.3650 | 0.3576 | 26.16 deg |

- **The linear law is falsified.** It fits the endpoints but over-predicts at 0.2 m/s by a
  half (0.146 against 0.098 measured). Slip is not proportional to speed: there is a knee
  between 0.2 and 0.3 m/s, below which the contact rolls and above which it slides. The
  earlier conclusion that `v_max` would have to fall to 0.137 m/s was wrong, and wrong in
  the pessimistic direction; the forward criterion holds up to about 0.2 m/s.
- **The heading drift is a slip artefact, not an asymmetry.** 0.34 and 0.84 degrees over 5 m
  at 0.1 and 0.2 m/s: the robot travels straight. The 26 degrees at 0.5 m/s appears only
  once the contact slides, with the same knee. This closes the unexplained veer recorded
  earlier today: the robot is not crooked.
- **Not acted on, deliberately.** Two reasons not to set `actuation.v_max` from this sweep:
  0.2 m/s passes at 98 per cent of its threshold on a single seed, which is no margin for a
  preregistered value; and forward is not the binding constraint. In the full G1 run,
  lateral at its own 0.3 m/s limit gave 23.4 per cent error and 97.7 degrees of drift
  against forward's 19.8 per cent and 16.7 degrees at the same speed. Mecanum lateral
  motion loads the free roller axis directly, so its knee is lower, and a `v_max` chosen
  from the forward curve would pick a speed the robot cannot strafe at.
- **Next:** the same sweep on the lateral profile, which is what actually bounds the
  velocity box. Only then is there a basis for a `v_max` / `vy_max` proposal to the
  supervisor.
- **Affects:** `configs/base.yaml` (`actuation.v_max`, `vy_max`), reward time cost,
  certified-layer speed bands, episode length budgets.

## 2026-09-17 (WP0) - Lateral sweep: holonomic fidelity fails at every speed
- **Measured** (lateral profile, 5 m each; `vy_max` is 0.3 so scale s commands 0.3s):

  | commanded | slip | speed error | heading drift | speed ok | drift ok |
  |---|---|---|---|---|---|
  | 0.060 | 0.0474 | 0.0474 | 12.85 deg | yes | NO |
  | 0.120 | 0.0845 | 0.0844 | 20.55 deg | yes | NO |
  | 0.180 | 0.1109 | 0.1109 | 23.66 deg | NO | NO |
  | 0.240 | 0.1584 | 0.1585 | 45.94 deg | NO | NO |
  | 0.300 | -- | 0.2337 | 97.74 deg | NO | NO |

- **Heading drift fails at every lateral speed tested**, including 0.06 m/s -- a fifth of
  `vy_max`, where the speed error is a comfortable 4.7 per cent. Drift shows no sign of
  approaching the 5 degree limit as speed falls. This is qualitatively unlike forward,
  where drift collapsed to 0.34 degrees once the contact stopped sliding. Lateral motion on
  this platform is systematically crooked, independent of speed.
- **Correction to this log's earlier entry.** It recorded that the preregistered fallback
  "does not address this failure" because the forward profile failed too. That judgement
  was made on 0.5 m/s data alone. With both curves in hand it is wrong: forward passes both
  criteria at or below 0.2 m/s, lateral fails drift at every speed, so the failure *is*
  holonomy-specific and the committed fallback -- "unicycle primary; holonomic becomes
  caveated ablation" -- is the applicable response, conditional on `v_max` coming down to
  about 0.2 m/s.
- **Mechanism, hypothesis only:** the front wheels hang off `/youBot/Joint`, a passive
  axle, while the rear pair is rigidly mounted. Lateral motion needs the front and rear
  pairs to generate opposing longitudinal forces that cancel; a compliant front axle cannot
  match a rigid rear one, leaving a net yaw torque. That would be speed-independent, which
  is what the data shows. Not tested.
- **Consideration recorded, not acted on:** G1 drives open-loop constant commands by
  design, and a closed-loop controller would correct heading. Thirteen degrees over 5 m may
  be recoverable in practice. The committed criterion is open-loop and it fails; it is not
  reinterpreted here.
- **Before the fallback can be adopted it needs its own evidence.** "Unicycle primary"
  means vx and wz together, which is the `translate_while_rotating` profile -- measured only
  at 0.5 m/s, where it failed with 177.9 degrees of drift. Whether the unicycle regime is
  itself viable at the reduced speed is unmeasured, and adopting a fallback that has not
  been shown to work would replace one unverified claim with another.
- **Affects:** `preregistration/gates.yaml` G1 verdict, `configs/base.yaml`
  (`actuation.mode`, `v_max`, `vy_max`), ablation A1's status, WP1 environment design.

## 2026-09-17 (WP0) - G1 complete: FAIL as specified; neither actuation mode qualifies
- **All four profiles, across speed, against the committed criteria (speed error <= 0.10,
  heading drift <= 5.0 deg over the profile):**

  | profile | speeds passing |
  |---|---|
  | forward | 0.1 and 0.2 m/s (0.0980 error, 0.84 deg drift at 0.2) |
  | lateral | none: drift 12.85 deg at 0.06 m/s rising to 97.74 at 0.30 |
  | translate_while_rotating | none: 7.95 deg at 0.1 m/s, 24.98 at 0.2, 177.91 at 0.5 |
  | diagonal_45 | measured only at 0.5 m/s; 0.6054 error, 8.34 deg |

- **Verdict: G1 FAILS.** Only the forward profile passes, and only at or below 0.2 m/s.
  Neither the holonomic mode nor the committed unicycle fallback meets the criteria as
  written.
- **Specification gap identified, not resolved and not relaxed.**
  `max_heading_drift_deg: 5.0` is stated as an absolute bound "over the profile distance".
  For the three pure-translation profiles that is unambiguous: it bounds *unwanted*
  rotation, and forward at 0.84 deg passes cleanly. For `translate_while_rotating`, which
  commands 181.7 deg of rotation at scale 0.2, a 5 deg absolute bound is a 2.75 per cent
  relative requirement -- about 3.6 times stricter than the 10 per cent speed criterion
  applied to the same run, and the only criterion in the file that tightens as the
  commanded motion grows. At that speed the robot tracks yaw to 4.38 per cent and speed to
  5.28 per cent, which are comparable. Whether the asymmetry was intended is the
  supervisor's decision. It changes the verdict only at 0.1 m/s; 0.2 m/s fails on both
  readings at 13.07 per cent.
- **Consequence of a 0.1 m/s operating point, if that is where this lands:** a 5 m traverse
  takes 50 s, or 1000 steps at dt = 0.05. `reward.max_steps` is 1500, so an episode would
  cover about 7.5 m of travel, which is short for warehouse navigation, and G2's compute
  projection was made on a different assumption. Lowering `v_max` is not a local change.
- **Not changed:** `actuation.mode`, `v_max`, `vy_max`. All three are preregistration-
  adjacent and the choice among the available responses -- reduce speed, investigate the
  platform physics, amend the drift criterion for rotating profiles, or re-specify G1 as an
  open-loop plant test with closed-loop control assumed -- belongs to the supervisor.
- **Affects:** `preregistration/gates.yaml`, `configs/base.yaml` (`actuation.*`,
  `reward.max_steps`), ablation A1's status, G2's budget, WP1 environment design.

## 2026-09-17 (WP0) - Braking diagnostic written; a_brake has never been measured
- **Why:** `filter.a_brake` and `certified.a_brake` are both 1.0 m/s^2, and the PFB's
  stopping distance is `v * t_react + v^2 / (2 * a_brake) + margin`. That expression is the
  safety argument and the number under it has never been measured. G1 showed the platform
  loses 36 per cent of its commanded speed to wheel slip at 0.5 m/s; a contact that cannot
  deliver the traction to reach a speed may not deliver it to shed one either.
- **A diagnostic, not a gate.** `preregistration/gates.yaml` has no braking criterion and
  hard rule 1 forbids adding one, so `scripts/diag_braking.py` reports measurements and
  their ratio to the configured `a_brake` and issues no verdict. A criterion would have to
  be committed to the preregistration first.
- **Method.** Accelerate under the normal limiter, hold 40 steps, then command zero with
  `rate_limit=False`. Without that bypass the measurement would return `actuation.a_max`,
  the limiter's own ramp rate, rather than anything about the platform. `YouBotBase`
  already carried that flag for exactly this purpose.
- **Metric chosen by how the PFB uses it:** `effective_deceleration = v^2 / (2 d)`, the
  constant deceleration that would cover the measured distance, because the barrier needs
  to know how much *distance* a speed costs, not how much time. Mean deceleration over time
  is reported alongside; the gap between the two says how far from constant the brake is.
- **Bias found and pinned while writing the tests.** Computing the braking distance by
  summing `speed * dt` over the samples is a rectangular integral: it carries half a step
  of extra distance and reports the deceleration about 9 per cent low. On a safety margin
  that is not acceptable, so the script measures travelled path instead, and
  `test_integrating_the_speed_series_instead_of_the_path_biases_the_answer` pins the 0.909
  figure so the cheap version cannot creep back in.
- **What the PFB currently assumes, for comparison with whatever comes back:** braking
  distances of 0.005, 0.020, 0.045 and 0.125 m at 0.1, 0.2, 0.3 and 0.5 m/s.
- **Affects:** `configs/base.yaml` (`filter.a_brake`, `certified.a_brake`),
  `pfbnav/safety/pfb.py` stopping-distance model, `pfbnav/safety/certified.py` speed bands.

## 2026-09-17 (WP0) - Braking measured: a_brake = 1.0 is conservative by about 2.5x
- **Corrected results** from `runs/gates/BRAKE_20260917T225315Z_seed0`, forward axis. The
  figures the script printed on the night were wrong; see the bug below. Recomputed from
  the logged positions:

  | entry speed | braking distance | distance the PFB assumes | effective decel | ratio |
  |---|---|---|---|---|
  | 0.093 m/s | 0.0019 m | 0.0043 m | 2.27 m/s^2 | 2.3 |
  | 0.180 m/s | 0.0066 m | 0.0163 m | 2.46 m/s^2 | 2.5 |
  | 0.241 m/s | 0.0110 m | 0.0291 m | 2.63 m/s^2 | 2.6 |
  | 0.315 m/s | 0.0167 m | 0.0496 m | 2.96 m/s^2 | 3.0 |

- **`a_brake = 1.0` is conservative, not optimistic.** The platform stops in about a third
  of the distance the PFB budgets. This is the safe direction, and it is the opposite of
  what the slip result led me to expect: a contact that slips under acceleration still
  stops promptly, because the velocity-controlled joints clamp the wheels.
- **Measurement bug, mine, found by inspecting the raw trace rather than trusting the
  summary.** A sample logged at step k records the pose *after* step k has been simulated,
  so the travel produced by the first braking step lies between the last pre-brake sample
  and the first post-brake one. The script started the path at the first post-brake sample
  and discarded it. At 0.315 m/s that was 11.8 mm of a 16.7 mm stop: the distance came out
  70 per cent short and the deceleration correspondingly inflated, to 2137 m/s^2 in the
  worst row. The absurdity of that number is what prompted the check; a subtler bias would
  have passed as a finding.
- **Fixed and pinned:** the slicing moved into `pfbnav.eval.gates.braking_distance`, which
  starts one sample before the onset and raises if asked to start at zero, with tests that
  assert both the correct value and the 0.294 ratio the naive version produces.
- **Caveats on the numbers.** Only two or three samples fall inside the transient at 20 Hz,
  so the *distance* is measured directly from logged positions but the deceleration
  *profile* is unresolved; the effective value is nevertheless the quantity the barrier
  needs. The apparent rise in deceleration with entry speed is consistent with more of the
  stop falling inside the first unresolved step at higher speeds, and should not be read as
  speed-dependent braking. Yaw during braking was at most 0.04 degrees.
- **Not changed.** `filter.a_brake` and `certified.a_brake` stay at 1.0. Raising them
  towards the measured 2.3 would shrink the protective fields and reduce protective stops,
  which is a change to the headline metric's denominator and a safety parameter both; it
  belongs to the supervisor. Note also that the simulated brake is unphysically abrupt --
  velocity-controlled joints with high force limits -- so matching a_brake to it would tie
  the safety argument to a simulator artefact rather than to a plausible robot.
- **Affects:** `configs/base.yaml` (`filter.a_brake`, `certified.a_brake`),
  `pfbnav/safety/pfb.py`, `pfbnav/safety/certified.py`, `scripts/diag_braking.py`,
  `pfbnav/eval/gates.py`.

## 2026-09-18 (WP0) - Mixing verified against Modern Robotics; lateral fault is in the plant
- **Source:** the Northwestern Mobile Manipulation Capstone page for the KUKA youBot,
  supplied by the researcher, which states r = 0.0475 m, 2l = 0.47 m, 2w = 0.30 m for the
  real robot and refers to Modern Robotics ch. 13.4 for the chassis kinematics.
- **Our mecanum mixing agrees with Modern Robotics eq. 13.10 to 1.4e-14** over 2000 random
  twists. Pinned as `test_mixing_matches_modern_robotics_*` in
  `tests/test_robot_kinematics.py`. This matters because until now the kinematics had only
  been checked against v1, which shares their lineage: agreeing with v1 could only confirm
  that a common assumption was applied consistently, not that it was correct. Note also
  that Modern Robotics numbers the wheels counter-clockwise (fl, fr, rr, rl) while this
  project uses (fl, fr, rl, rr); the test reorders rather than assuming they match.
- **The scene is not dimensionally the real youBot:**

  | parameter | real youBot (MR) | this scene (measured) | difference |
  |---|---|---|---|
  | wheel radius | 0.0475 m | 0.0500 m | +5.3% |
  | half wheelbase | 0.2350 m | 0.2280 m | -3.0% |
  | half track | 0.1500 m | 0.1585 m | +5.7% |

  The configuration keeps the measured values, which is correct -- the simulation is what is
  being measured -- but a thesis claim that the platform *is* a youBot should note the few
  per cent discrepancy in the model.
- **Consequence for the G1 diagnosis:** the kinematics are now verified against an
  authoritative external reference and the geometry is measured from the scene, so the
  lateral yaw is not a modelling error on our side. It is a property of the simulated
  plant, which narrows the search to the wheel emulation and the chassis structure.
- **Noted:** the capstone simulates the chassis *kinematically* at a 0.01 s timestep with
  no physics, which is why that course never encounters wheel slip. Driving the base
  kinematically is an available option here too, but it would remove the very plant
  imperfection a safety-filter study exists to handle, and would make the PFB's braking
  model exact by construction. Recorded as an option, not a recommendation.
- **Affects:** `tests/test_robot_kinematics.py`, thesis statements about platform fidelity.

## 2026-09-18 (WP0) - Roller angles correct; front axle is a live rocker
- **Roller angles are right.** The four wheel scripts set the slipping joint base frame to
  `{-pi/4, 0, 0}` for fl and rr and `{+pi/4, 0, pi}` for fr and rl -- opposite roller angles
  with mirrored mounting, exactly Modern Robotics' specification (gamma = -45 deg for wheels
  1 and 3, +45 deg for 2 and 4). The hypothesis that the scene had copied one script to all
  four wheels is eliminated.
- **`/youBot/Joint` is a revolute joint in dynamic mode** (type 10, mode 5), carrying
  `Rectangle0` and with it both front wheels, while the rear pair is rigidly mounted on the
  chassis. That is the one structural asymmetry left between forward and lateral loading.
- **Working hypothesis:** mecanum wheels carry substantial internal forces even at constant
  velocity, each pushing at plus or minus 45 degrees with the components cancelling. At the
  front those forces act through a rocker that cannot resist a roll moment, so the front
  pair's normal loads redistribute, their traction becomes unequal and the cancellation
  breaks, leaving a steady yaw. Forward motion loads the rocker symmetrically, which is why
  it stays clean. Prediction: the rocker deflects during a strafe and not during a forward
  run.
- **Conventions now confirmed three independent ways.** The KUKA youBot User Manual
  (Locomotec, v1.01) gives the base frame as x forward, y left, z up with positive z
  rotation counter-clockwise, and its `youbot-base.cfg` numbers the wheels
  BaseLeftFront=1, BaseRightFront=2, BaseLeftBack=3, BaseRightBack=4 -- this project's
  `[fl, fr, rl, rr]`. Note that Modern Robotics instead numbers counter-clockwise
  (fl, fr, rr, rl), which is why the agreement test reorders rather than assuming.
- **No published speed limit.** The manual defers it: "The operator has to perform a risk
  analysis for the chosen velocity and pay load." It does record that the platform "may
  loose traction, slide, and crash" on a tilted plane, so traction loss is a acknowledged
  characteristic of the real platform rather than purely a simulation artefact.
- **Added:** `--log-joints` on the G1 script, logging any named joint's position every step,
  so the rocker prediction can be tested with the profiles that already exist.
- **Affects:** `scripts/gate_g1_fidelity.py`, the G1 diagnosis.

## 2026-09-18 (WP0) - Rocker ruled out; lateral yaw is a symmetric, handed property of the wheel model
- **Rocker hypothesis rejected.** `/youBot/Joint` deflects 0.055 degrees peak-to-peak during
  a 5 m strafe and 0.011 during the forward control run. It is dynamically live but
  effectively rigid, and cannot redistribute load enough to matter. The mechanism proposed
  in the previous entry is wrong.
- **The yaw reverses with the strafe direction, at matched magnitude:** +20.55 deg for
  vy = +0.120 m/s, -20.35 deg for vy = -0.120 m/s, against +0.84 deg for the forward
  control at the same speed. The effect is symmetric and handed, which rules out a fixed
  asymmetry -- no mass offset, no geometry error, no chassis compliance.
- **Method note:** `heading_drift_deg` returns an absolute value, so the two runs both
  summarised as about +20 degrees. The sign had to be recovered from the raw yaw series.
  A metric that discards the sign cannot answer a question about direction, and this test
  existed only to answer one; worth remembering before reusing that number.
- **Consistency check.** Both strafe runs also show about 0.92 m of displacement
  perpendicular to the commanded direction. That is not a second effect: a 5 m path turning
  0.358 rad is a circular arc of radius 14.0 m, whose chord deviates by R(1 - cos theta) =
  0.89 m. The robot strafes along a gentle arc and the numbers close.
- **Magnitude of the underlying error.** A parasitic yaw rate of 0.0086 rad/s at
  vy = 0.12 m/s requires a differential of 0.067 rad/s across the diagonal wheel pairs
  against a commanded 2.4 rad/s: about 3 per cent. That is the scale of a modelling
  imperfection, not a gross error, and is what a sphere-plus-free-joint approximation of a
  mecanum wheel would be expected to produce.
- **Conclusion.** Every layer above the physics is now verified: the mixing matches Modern
  Robotics exactly, the wheel order matches KUKA's own driver configuration, the roller
  angles match the specification, the geometry is measured, and the sign vector is
  confirmed by motion. The lateral yaw is a property of CoppeliaSim's mecanum emulation in
  this scene, not an error in this project's code or configuration.
- **Affects:** G1's verdict and what can be done about it; no code or configuration change
  follows from this entry.

## 2026-09-18 - Supervisor decisions on the WP0 update: D6-D12

### D6 - G1 stands as FAIL; G1b added rather than re-specifying it
- **Decision:** G1 is not re-specified. Its FAIL stands and is reported. A new gate G1b
  measures closed-loop tracking, with thresholds the supervisor has committed to
  `preregistration/gates.yaml`. Both results appear in the thesis. The unicycle fallback
  detaches from G1 and attaches to G1b.
- **Why this is the right call and my suggestion was not:** re-specifying a gate after
  seeing it fail is indistinguishable, from outside, from moving the goalposts. Keeping
  both measurements answers the question I raised -- whether open-loop fidelity is the right
  requirement -- as evidence rather than as an argument, because the reader sees what the
  platform does under each.
- **Read from the committed G1b block:** proportional heading and cross-track controller
  with gains in `configs/base.yaml`; all four profiles at 0.10, 0.20, 0.30, 0.40, 0.50 m/s
  over 5 m; max cross-track 0.05 m, RMS 0.02 m, final heading error 5 deg; three
  repetitions, all passing; `v_max` is the highest speed passing every profile in every
  repetition.
- **Affects:** `scripts/gate_g1b_tracking.py` (new), `configs/base.yaml` (gains),
  thesis structure.

### D7 - One-day physics probe before G1b, then frozen
- **Decision:** a time-boxed one-day probe of physics engine, timestep and substeps, roller
  friction and wheel mass, on the lateral profile only. Adopt the best configuration,
  freeze it, record it. No tuning after G1b runs.
- **Why the ordering matters:** tuning physics after seeing a gate result would make the
  gate a measurement of the tuning. Freezing first, and recording the frozen values, keeps
  G1b a measurement of the platform.
- **Scope note:** engine, timestep and substeps are simulator settings; friction and mass
  are scene object properties. None currently live in `configs/`, so whatever is adopted
  must be recorded explicitly or it will not survive a scene reload.
- **Affects:** the scene, `configs/base.yaml` (to record the frozen settings), G1b.

### D8 - v_max is an output of G1b, not an input
- **Decision:** `actuation.v_max` is set by G1b's outcome, and every dependent constant
  updates in a single commit.
- **Dependents identified so far:** `actuation.vy_max`, `reward.time_cost`,
  `reward.max_steps` (episode length in metres), observation scaling for the velocity
  channels, the PFB's stopping distance through entry speed, the certified layer's speed
  bands, and G2's compute projection. A partial update would leave the configuration
  internally inconsistent in ways no single test would catch.
- **Affects:** `configs/base.yaml`, WP1 environment, G2.

### D9 - a_brake stays 1.0; the limiter enforces it by construction
- **Decision:** `filter.a_brake` and `certified.a_brake` stay at 1.0 m/s^2. A symmetric
  deceleration limit is added to the acceleration limiter so the assumption holds by
  construction rather than by measurement.
- **Correction to this log and to the summary:** the braking diagnostic measured 2.3 to
  3.0 m/s^2, and the entry of 2026-09-17 presented that as the platform's braking capacity.
  That reading is wrong. The joints are velocity-controlled with high force limits, so
  commanding zero clamps the wheels almost instantly; what was measured is the joint
  controller's authority, not the tyre-ground braking physics a real robot would be limited
  by. The number is an artefact of the actuation model and must not be used to justify a
  larger `a_brake`.
- **Consequence:** with the limiter bounding deceleration at `a_decel_max`, no controller
  can command a stop faster than the PFB assumes, so the stopping-distance model is
  conservative by construction and no longer depends on what the simulated contact happens
  to deliver.
- **Affects:** `configs/base.yaml` (`actuation.a_decel_max`), `pfbnav/sim/robot.py`
  (`limit_acceleration`), the published summary's wording.

### D10 - Lateral coupling becomes a bounded disturbance; robust PFB specified, not built
- **Decision:** characterise the lateral yaw coupling as a bounded disturbance, fitted over
  speed and command direction, and specify a robust PFB that carries that bound. Add H6 to
  the hypothesis list as exploratory. Do not implement the filter; produce the
  specification.
- **Why this is better than what I proposed:** I offered "accept it and rely on closed-loop
  control", which treats a measured, reproducible, sign-reversing coupling as noise. It is
  not noise: it is a deterministic function of the commanded velocity, and a barrier
  function can carry a bound on it explicitly. That converts a platform defect into a
  stated robustness property.
- **Measured so far, lateral profile, 5 m:** 0.045 rad/m at 0.06 m/s, 0.072 at 0.12, 0.083
  at 0.18, 0.160 at 0.24, and loss of control at 0.30. Sign reverses with the strafe
  direction at matched magnitude. Forward motion shows 0.003 rad/m at 0.20 m/s. The fit
  needs the intermediate command directions, which no run has covered.
- **Affects:** `docs/pfb_robust_spec.md` (new), the hypothesis list, `pfbnav/safety/pfb.py`
  (later).

### D11 - Commit, then re-run; three repetitions for every gate
- **Decision:** commit the repository, then re-run G0 and G1 so results are citable. Every
  gate now requires three repetitions, all passing.
- **Standing problem this fixes:** every gate run to date records `commit: HEAD,
  dirty: true`, because the repository had no commits. No result so far is traceable to a
  tree state, which is why none of them should be cited as they stand.
- **Note on repetitions:** `gates.yaml` states `repetitions: 3` for G1b only. The extension
  to G0 and G1 is a supervisor decision recorded in `preregistration/AMENDMENTS.md` rather
  than by editing the committed criteria.
- **Affects:** the repository, `scripts/gate_g0_contact.py`, `scripts/gate_g1_fidelity.py`.

### D12 - The platform is youBot-derived, and the difference is tabulated
- **Decision:** add a scene-versus-datasheet dimension table and describe the platform as
  youBot-derived rather than as a youBot.

  | quantity | KUKA / Modern Robotics | this scene (measured) | difference |
  |---|---|---|---|
  | wheel radius | 0.0475 m | 0.0500 m | +5.3% |
  | half wheelbase | 0.2350 m | 0.2280 m | -3.0% |
  | half track | 0.1500 m | 0.1585 m | +5.7% |
  | chassis half-length | 0.2900 m (datasheet 580 mm) | 0.2854 m | -1.6% |
  | chassis half-width | 0.1900 m (datasheet 380 mm) | 0.1789 m | -5.8% |

  Conventions, by contrast, match exactly: base frame x forward, y left, z up with positive
  z counter-clockwise, and wheel order fl, fr, rl, rr as in KUKA's own `youbot-base.cfg`.
- **Affects:** the published summary, thesis wording, `docs/`.

## 2026-09-18 (WP0) - D6-D12 implemented; two Run Requests prepared

- **Written:** `scripts/gate_g1b_tracking.py` (D6), `scripts/report_pods.py` (the
  outstanding D3 report, extended with the polygon footprint), `pfbnav/sim/footprint.py`
  and `docs/pfb_robust_spec.md` (D10). Tests: 240 fast, ruff clean.
- **D9 implemented.** `actuation.a_decel_max` added at 1.0 m/s^2 and `limit_acceleration`
  now applies it whenever a command reduces speed, `a_max` otherwise. A test in
  `tests/test_infrastructure.py` asserts `a_decel_max <= filter.a_brake` and
  `<= certified.a_brake`, so the PFB's stopping-distance model cannot silently become
  optimistic through a configuration edit. The published summary's wording is corrected:
  the 2.3-3.0 m/s^2 figure is the velocity-controlled joints' authority, not braking
  physics.
- **D12 implemented:** the scene-versus-datasheet table is in this log under D12 and in the
  published summary, with the platform described as youBot-derived.
- **Two bugs caught by known-answer tests before they reached a report.** `radial_extent`
  had a sign error in the ray-segment solve, from crossing with the wrong vector; it would
  have produced plausible but wrong per-quadrant radii. And the synthetic constant-brake
  test showed that integrating the speed series rectangularly biases the deceleration 9 per
  cent low, which is why `braking_distance` measures travelled path instead. Neither would
  have been visible in the output.
- **`preregistration/AMENDMENTS.md` could not be written.** Hard rule 1 and the
  `Edit(preregistration/**)` deny in `.claude/settings.json` both block it. The content is
  staged at `docs/AMENDMENTS.staged.md` with a header giving the `git mv` that puts it in
  place. Five amendments recorded: A1 G1b added and the fallback moved, A2 three
  repetitions for every gate, A3 H6 exploratory, A4 physics frozen before G1b, A5 v_max as
  an output of G1b.
- **D11 partially deferred.** The repository commit was deferred at the researcher's
  instruction, so every gate result still records `commit: HEAD, dirty: true` and none is
  citable. The three-repetition requirement is recorded in the staged amendments; the G0
  and G1 scripts do not yet loop repetitions internally, so three runs means three
  invocations until they do.
- **Gains committed for G1b** as its criteria require: `tracking.k_cross_track` 1.0 and
  `tracking.k_heading` 2.0, both marked VERIFY. They have never been run. If G1b fails,
  separating a controller-tuning failure from a platform limit is the first thing to do,
  and D7's no-tuning rule applies to the physics, not to these.
- **Affects:** `configs/base.yaml`, `pfbnav/sim/robot.py`, `pfbnav/sim/client.py`,
  `pfbnav/sim/footprint.py`, `pfbnav/eval/gates.py`, `scripts/`, `docs/`.

## 2026-09-18 (WP0) - Pods report delivered; its polygon is not trustworthy as it stands
- **Delivered** `runs/gates/PODS_20260918T004205Z_seed0/pods_report.json`. 36 robot shapes,
  **14 protruding** past the chassis by more than 2 mm.
- **The sensor housings are cleanly measured**, because they are boxes: centres at
  x = +0.325 and -0.325 m, reaching 0.3501 and 0.3500 m, lateral half-extent 0.0255 m.
  Fore-aft symmetric to 0.1 mm. These are the parts that set the envelope's length.
- **The wheels are not cleanly measured, and this invalidates the polygon.** A
  `wheel_respondable` is a *sphere*, so its bounding box is a cube, and the eight corners of
  a rotated cube reach sqrt(3) times further from the centre than the sphere does. Measured
  lateral reach came out 0.2437 m where the collision probe says 0.1958 m -- over-stated by
  48 mm. Worse, each wheel has settled at a different rotation, so the four report reaches
  spanning 0.2829 to 0.3133 m for parts that are geometrically identical, and a re-run would
  give different numbers again. **The polygon is neither accurate nor reproducible.**
- **Consequently the per-quadrant radii are conservative and should not be used as they
  stand:** inscribed 0.2241 to 0.2327 m against a true lateral envelope of 0.1970, and
  circumscribed 0.3520 to 0.3625 m against a true corner of about 0.351. The apparent
  quadrant asymmetry (front-left 0.3520 against rear-right 0.3625) is sphere orientation,
  not robot geometry; the reported symmetry figures, 7.3 mm fore-aft and 3.1 mm left-right,
  are the same artefact.
- **What would fix it:** extend `tests/test_footprint_probe.py` to sweep the plate at many
  bearings rather than four, giving a polygon measured by collision query. That method
  already agrees with geometry where both can be checked -- it put the wheels' forward face
  at 0.278 m, exactly `half_wheelbase + wheel_radius`. Deferred rather than done, because
  D7 may change the platform and the sweep should be run once on the frozen configuration.
- **Direction of the error is safe but the magnitude is wrong.** Over-stating the inscribed
  radius makes D1's lethal inflation more conservative, not less, so nothing unsafe follows
  from using these numbers in the interim -- but they must not be quoted as measurements of
  the robot.
- **Affects:** `scripts/report_pods.py`, `pfbnav/sim/footprint.py`, D1's inflation radii.

## 2026-09-18 (WP0) - D7 probe: nothing varied yet; platform confirmed deterministic
- **Both probe runs report the identical configuration** -- engine 0 (Bullet), simulation
  step 0.05 s, dynamics step 0.005 s -- and returned 20.56 and 20.55 degrees of lateral
  drift. No axis was varied, or what was varied had no measurable effect. Friction and mass
  are not captured in the summary, so if those were changed it cannot be told from the
  record, which is itself a reason to script the axes that can be scripted.
- **Useful by-product: run-to-run variation is 0.01 degrees** on drift and zero to four
  decimal places on speed error. The platform is deterministic, so A2's three repetitions
  will catch configuration mistakes rather than measurement noise. Worth knowing before
  committing 60 runs to G1b.
- **Fixed the reason it did not get swept.** `SimClient.set_physics` now sets the dynamics
  engine and substep size and *verifies they took*, raising if the build does not provide a
  requested engine -- a run attributed to the wrong engine is worse than no run. Exposed as
  `--engine` and `--dynamic-step` on the G1 script. Friction and mass remain manual, since
  4.10 moved them to a properties API this project has not established.
- **Affects:** `pfbnav/sim/client.py`, `scripts/gate_g1_fidelity.py`, the D7 protocol.

## 2026-09-18 (WP0) - D7 engine sweep: the yaw coupling is a Bullet artefact
- **Lateral profile, 0.12 m/s, 5 m, dynamics step 0.005 s, everything else unchanged:**

  | engine | heading drift | speed error | distance | run |
  |---|---|---|---|---|
  | 0 Bullet 2.83 (baseline) | 20.55 deg | -- | 5 m | G1_20260918T003830Z |
  | 1 ODE | **0.47 deg** | 2.2 % | 5.00 m | G1_20260918T090550Z |
  | 3 Newton | 3.12 deg | 3.1 % | 5.00 m | G1_20260918T090751Z |
  | 4 MuJoCo | 160.3 deg | 65 % | 2.21 m (timed out) | G1_20260918T091125Z |

  `G1_20260918T090953Z` has config and provenance but no summary: an aborted run, engine
  not recorded. Excluded.
- **Consequence for the cause established earlier.** The mixing, wheel order, roller angles
  and geometry were all verified independently of physics, so the 20 degrees was attributed
  to "CoppeliaSim's wheel emulation". That is now narrowed: it is the emulation *under
  Bullet*. The same scene under ODE drifts 0.47 degrees. The earlier attribution was right
  in what it excluded and too broad in what it blamed.
- **This is a finding about simulator dependence, not only a configuration choice**, and is
  recorded as such: the holonomic fidelity of the same model varies by a factor of 40
  across engines and fails outright on one. Any result reported on this platform carries
  the engine as a stated condition.
- **ODE is the provisional choice**, pending the substep and full-speed checks on it. Not
  yet frozen. Provenance for all four runs is dirty/uncommitted (D11 deferred by the
  researcher), so the numbers are reproducible from the recorded configs but not citable
  against a commit.
- **Affects:** D7, `docs/pfb_robust_spec.md` §1 (the disturbance bound was fitted to Bullet
  and must be re-measured on the frozen engine), `configs/base.yaml` once frozen.

## 2026-09-18 (WP0) - D7 closed: physics frozen at ODE, dynamics step 0.005 s
- **Substep sweep on ODE, lateral, 5 m:**

  | dynamics step | speed | heading drift | speed error | run |
  |---|---|---|---|---|
  | 0.005 s | 0.12 m/s | 0.47 deg | 2.2 % | G1_20260918T090550Z |
  | 0.002 s | 0.12 m/s | 1.11 deg | 3.2 % | G1_20260918T091911Z |
  | 0.001 s | 0.12 m/s | 0.84 deg | 3.4 % | G1_20260918T092052Z |
  | 0.005 s | 0.30 m/s | 2.79 deg | 8.5 % | G1_20260918T092302Z |

- **Solver sensitivity is 0.64 deg of drift**, with no trend as the step shrinks, so it
  reads as solver variation rather than a convergence error. That is slightly above the
  0.5 deg I proposed in advance as "robust", but all three step sizes pass the G1 criteria
  by a wide margin. The band is reported as the platform's solver uncertainty and not
  hidden by picking the best-looking value.
- **Kept 0.005 s because it is the scene's own default and the cheapest**, not because it
  gave the smallest drift. It happens to do both. Anyone reading this should know that
  0.002 s would have reported 1.11 deg.
- **At full lateral speed (0.30 m/s), ODE holds the line where Bullet lost control:** 2.79
  deg of drift. The speed error of 8.5 % is close to G1's 10 % limit and is all ground slip,
  since wheel odometry agrees with the command to 0.006 %. On ODE the speed deficit, not
  the yaw, is the binding effect near the top of the range.
- **Frozen in `configs/base.yaml` under `sim.physics`** and applied by `SimClient.start()`
  on every start, overriding the scene file, which still says Bullet. An unavailable engine
  raises. The G1 script's `--engine`/`--dynamic-step` flags still work but now mark the run
  DIAGNOSTIC when they depart from the frozen values. Tests: `tests/test_frozen_physics.py`.
- **Not tuned: friction and wheel mass.** D7 allowed them. The engine change alone brought
  lateral drift under the threshold, and every further axis is another degree of freedom
  fitted to the gate. Left at scene defaults, and recorded here as untouched.
- **G1 as recorded stays FAIL (D6).** It was measured on Bullet, the physics then in force.
  A G1 re-run on ODE would be a new result reported alongside it, not a replacement.
- **Affects:** every simulator script and test; `docs/pfb_robust_spec.md` §1, whose numbers
  are Bullet's and must be re-measured under G1b.

## 2026-09-18 (WP0) - G1b result: PASS, v_max_from_gate 0.40 m/s, set by our own clamp
- **Run:** `runs/gates/G1b_20260918T093210Z_seed0`, ODE / 0.005 s (recorded in summary),
  60 runs, all CSVs present. Provenance dirty/uncommitted (D11 deferred by researcher).
- **Verdict under the committed criteria: PASS, v_max_from_gate = 0.40 m/s.** 0.10-0.40 pass
  every profile in every repetition. 0.50 fails on `diagonal_45` (peak 75.9 mm, RMS 69.2 mm)
  and `translate_while_rotating` (peak 62.2 mm, RMS 36.6 mm).
- **The three repetitions are bit-identical** to the printed precision. They confirm
  determinism and add no further information.
- **Cause of the 0.50 failures: `clamp_velocity`, not physics.** It clips each axis
  independently to the box (v_max 0.5, vy_max 0.3). A diagonal at 0.50 m/s needs 0.354 m/s
  on each axis; vy is clipped to 0.30, so the commanded direction bends by about 5 degrees
  and the tracker's correction, being lateral too, is clipped with it. Cross-track error
  climbs monotonically to a 76 mm plateau. `translate_while_rotating` at 0.50 hits the vy
  clip whenever the rotating reference puts more than 0.3 m/s on the lateral axis, which
  shows up as a cross-track oscillation at the rotation period. At 0.40 the same effect is
  already present on that profile (19.5 mm peak) but within the limits.
- **Lateral "passes" at 0.40 and 0.50 without being tested there.** The command is clipped
  to 0.30 m/s in both, realised 0.275 m/s, identical to the 0.30 run. The gate result
  covers lateral motion up to vy_max only.
- **Realised speed (steady state, after 1 m), not a G1b criterion, reported for D8:**
  forward 0.40 -> 0.362, 0.50 -> 0.440; diagonal 0.40 -> 0.335; lateral 0.30 -> 0.275;
  translate_while_rotating 0.40 -> 0.312 m/s.
- **Nothing changed in response.** Clipping per axis contradicts the limiter's own design,
  which scales the acceleration step uniformly to preserve direction, and changing it would
  plausibly move v_max. But changing it after the gate would be tuning after G1b, which D7
  forbids. Referred to the supervisor together with the D8 update.
- **Affects:** D8 (`actuation.v_max` and dependents, held pending the supervisor),
  `pfbnav/sim/robot.py::clamp_velocity`.

## 2026-09-18 (WP0) - Amendments file in place
- `preregistration/AMENDMENTS.md` placed by the researcher (A1-A5, A4 outcome, clamp item
  under "Open"). The staged copy in `docs/` no longer exists; the preregistration file is
  the only version.

## 2026-09-18 (WP0) - Supervisor rulings on the D7/G1b report: implementation
- **Order set by the supervisor:** commit; A6 and the clamp fix; G1b re-run and G1-ODE;
  footprint probe; D8 in one commit. Baseline committed as `a7c1a63` (the first commit;
  G1 and G1b first-run provenance predates it and stays uncitable).
- **A6-A9 drafted** in `docs/AMENDMENTS_A6-A9.staged.md` for the researcher to paste into
  `preregistration/`, with a `gates.yaml` addition (`lateral_extra_speeds`, reworded
  `outcome`). The clamp fix is held out of any commit until A6 is committed.
- **Ruling 1, clamp.** `clamp_velocity_with_status` scales (vx, vy) by one factor, clips
  omega separately, and reports both; `YouBotBase` counts every binding. Known-answer
  tests include G1b's failing diagonal (0.50 m/s at 45 deg -> (0.3, 0.3), still 45 deg)
  and a 2000-sample check that output is inside the box and parallel to input. The WP2
  target (wheel-speed polytope as QP constraints) is recorded in A6 and in the PFB
  interface docstring.
- **Rulings 2 and 4, re-run grid.** The G1b script now reads `lateral_extra_speeds` and
  refuses to run without it; refuses a box below the speeds tested; derives v_max and
  vy_max lowest-first; logs realised speed and clamp bindings per run; writes
  `realised_speed_map.csv`. `configs/gates/g1b_rerun.yaml` lifts the box to 0.50/0.50 as a
  measurement ceiling. With the box lifted, the clamp should not bind on the grid except
  momentarily at 0.50 m/s under tracker corrections; the counts will show it.
- **Ruling 3, actual speed.** Neither the PFB nor the certified layer is implemented
  (both raise NotImplementedError), so there is no bug to report yet. Both interfaces now
  state that `velocity` is the estimated actual velocity and neither the command nor
  wheel odometry: G1b showed wheel odometry matching the command to 0.006 % while the
  robot fell 8-22 % short, so odometry would inherit the command's error. Using the
  command is not uniformly conservative either: it over-states speed in steady motion but
  under-states it during every deceleration, when the certified layer would pick too
  small a field. **Acceptance test for WP1/WP2:** feed each layer a velocity that differs
  from the command and assert the field and D(s) follow the velocity.
- **Ruling 5.** G1 run directories are now named by engine (`G1-ODE_<stamp>`).
- **Ruling 6.** `physics_settings()` records the engine name; G1, G1b and the footprint
  probe print it with every verdict. Newton external-validity check added via A8.
- **Ruling 7.** `sim.physics.solver_sensitivity_drift_deg: 0.64` in `configs/base.yaml`;
  G1 records and prints it with every drift figure.
- **Ruling 8.** `scripts/probe_footprint.py`: per-shape support function by half-space
  bisection at 180 bearings, collidable defined empirically by `checkCollision` (hard
  rule 4), visual-only shapes by bounding box, envelope reported both ways. Pure helpers
  `bisect_support`, `polygon_from_support`, `support_of_points` with known-answer tests.
- **Affects:** `pfbnav/sim/robot.py`, `pfbnav/sim/client.py`, `pfbnav/sim/footprint.py`,
  `pfbnav/eval/gates.py`, `pfbnav/safety/{pfb,certified}.py` (docstrings only),
  `scripts/gate_g1b_tracking.py`, `scripts/gate_g1_fidelity.py`,
  `scripts/probe_footprint.py`, `configs/base.yaml`, `configs/gates/g1b_rerun.yaml`,
  `docs/pfb_robust_spec.md`.

## 2026-09-18 (WP0) - G1b re-run (A6): PASS, ceilings at the top of the tested grid
**Superseded: measured before the wheel budget** (label added 2026-09-19; the result is
kept -- see the 2026-09-19 entry "G1b under the wheel budget").

- **Run:** `runs/gates/G1b_20260918T104317Z_seed0`, commit `04ae863`, clean, ODE / 0.005 s.
  66 runs, repetitions bit-identical. First citable gate result.
- **Verdict: PASS. v_max_from_gate 0.50, vy_max_from_gate 0.50 m/s -- both the top of the
  grid, so right-censored:** "passes everything tested", not "fails above". Weakest
  profile translate_while_rotating, peak 6.5 / 13.4 / 25.6 mm at 0.30 / 0.40 / 0.50 m/s,
  roughly doubling per 0.1 m/s against a 50 mm limit.
- **The A6 diagnosis is confirmed.** diagonal_45 at 0.50: 75.9 mm (first run, per-axis
  clip) -> 0.18 mm. translate_while_rotating at 0.50: 62.2 -> 25.6 mm. Lateral realised
  0.418 m/s at 0.50 commanded, where the clip had held it at 0.275.
- **Clamp bindings at 0.50 m/s are rounding, not shaping:** forward 232, lateral 237,
  translate_while_rotating 5 steps, smallest scale factor 0.99975. The tracker's command
  sits on the box edge and floating point tips it over. A tolerance on the counter would
  keep these from reading as real bindings; not changed, since it alters no result.
- **Realised-speed map (A9), realised m/s at commanded 0.10/0.20/0.30/0.40/0.50:**
  forward 0.098/0.192/0.280/0.362/0.440; lateral 0.098/0.190/0.275/0.351/0.418 (0.35 ->
  0.314, 0.45 -> 0.386); diagonal_45 0.097/0.186/0.265/0.335/0.361; translate_while_rotating
  0.096/0.186/0.268/0.339/0.392. Worst shortfall: diagonal at 0.50, 28 %.

## 2026-09-18 (WP0) - G1-ODE (A7): FAIL, reported alongside the Bullet FAIL
- **Runs:** `G1-ODE_20260918T124531Z_seed0`, `..124712Z_seed1`, `..124854Z_seed2`, commit
  `04ae863`, clean. Same verdict in all three.
- **Per profile (at the configured limits, open loop):** forward 0.50 m/s speed error
  12.0 % (FAIL, limit 10 %), drift 0.16-0.52 deg; lateral 0.30 m/s 8.5 %, drift 2.81-2.85
  deg (PASS); diagonal_45 (0.50, 0.30) 33.7 % (FAIL); translate_while_rotating 12.6 %,
  drift 44.1-44.8 deg (FAIL). Drift quoted with the +/-0.64 deg solver band (A8).
- **Repetitions are not bit-identical across separate invocations** (forward drift 0.156 /
  0.166 / 0.520 deg; translate_while_rotating 44.82 / 44.81 / 44.07), unlike G1b's
  in-process repetitions. The spread is within or near the solver band. A fresh scene load
  per invocation is the likely difference; not investigated.
- **Wheel-speed saturation found.** Only diagonal_45 shows wheel odometry disagreeing with
  the command (21 %): its wheels are commanded 16.0 rad/s and reach 12.33-12.34 rad/s. The
  other profiles command at most 12.32 rad/s and get it. So the wheel joints saturate near
  **12.34 rad/s, i.e. 0.617 m/s at the rim** (r = 0.05 m); source in the scene not yet
  identified (joint max velocity or motor torque), VERIFY. Consequence: the mecanum
  polytope |vx| + |vy| + (l+w)|wz| <= ~0.617 m/s binds inside the 0.50/0.50 box --
  (0.50, 0.50) needs 1.0 m/s at the rim. G1b's diagonal at 0.50 m/s (0.707 m/s rim) was
  saturated too, which is part of its 28 % shortfall; it still tracked because
  saturation of the two driven wheels is symmetric. This is ruling 1's WP2 target, now
  with a measured number, and it bears on D8.
- **Interpretation.** The forward and translate failures are the realised-speed shortfall
  at 0.50 m/s (open loop has no way to correct it); the diagonal failure is saturation.
  None changes A1: the fallback attaches to G1b, which passed.

## 2026-09-18 (WP0) - Footprint probe, first attempt: method error, nothing reported
- **Runs:** `FOOTPRINT_20260918T125143Z_seed0`, `..125239Z_seed0`; config and provenance
  only, no results.
- **Cause, mine:** the probe filled the half-space beyond each offset with a solid block,
  on the reasoning that contact would be monotone. `checkCollision` tests *surface*
  intersection, so a block that swallows a shape whole touches none of its surfaces: every
  shape, the chassis included, read as not collidable. The chassis guard stopped the run
  before any number was written.
- **Fix:** back to a thin plate (6 mm), which always cuts surfaces. `outermost_contact`
  scans inward from outside each bounding box in 5 mm steps (<= plate thickness, so no
  part can fall between samples) and bisects to 0.5 mm. The 2 deg envelope is measured
  against a collection of all collidable shapes; per-shape values, which decide
  protrusion, every 10 deg. The run cross-checks itself against the 2026-09-17 axis
  sweeps (front 0.3495-0.3520, left 0.1945-0.1970).
- **What should have caught it:** a fake-simulator test with surface semantics, now in
  `tests/test_probe_footprint_fake.py` -- it fails the block method and passes the plate.
- **Also:** narrowed G1b runs now report PARTIAL, not FAIL with the fallback message.

## 2026-09-18 (WP0) - Footprint probe, second attempt: stopped on a thin arm part
- **Run:** `FOOTPRINT_20260918T130509Z_seed0`, commit `328bcbb`, clean; config and
  provenance only. 23 of 36 shapes classified collidable -- chassis, all four
  `wheel_respondable_*`, both LiDAR bodies among them -- before it stopped on
  `.../ME_Arm2_13_sub0/Cylinder`: registered along +x, missed at another bearing.
- **Cause, mine again:** the docstring of `outermost_contact` claimed that with step <=
  plate thickness no part can be skipped. That holds only for parts at least as thick as
  the plate. A thinner part can sit wholly inside the plate, cutting no surface, and
  registers only in two bands as wide as it is thin, which a 5 mm step can jump. The
  fake-simulator test had not applied the swallowing rule to its thin-part case.
- **Fix:** a miss is retried at 0.5 mm; a part thinner than that falls back to its bounding
  box, flagged per bearing, and any part needing either has its box added to the envelope
  so the envelope cannot under-state it. Only shapes whose box reaches past the chassis
  are probed per bearing (the rest cannot protrude); the chassis is probed at every
  envelope bearing as the reference. Plate orientation is set once per scan.
- **Tests:** the thin-part test now uses surface semantics and asserts the skip happens
  (the limitation, stated) and that the fine step recovers it; the fake robot gains a
  0.8 mm fin (found by the retry, and verified to be missed by the coarse scan) and a
  0.2 mm fin (box fallback).
- **Run time:** the second attempt took over 14 minutes for 23 shapes, about 16 ms per
  remote call. The new version should be faster; the estimate is revised to 15-25 min.

## 2026-09-18 (WP0) - Footprint probe, third attempt: complete; collections honour the collidable flag
- **Run:** `FOOTPRINT_20260918T133012Z_seed0`, commit `7fff68d`, clean, ODE. All 36 shapes,
  no fine retries or box fallbacks needed. Self-check against the 2026-09-17 axis sweeps:
  front 0.3503 in (0.3495, 0.3520], left 0.1958 in (0.1945, 0.1970]. Symmetric to 0.2 mm.
- **14 shapes protrude past the chassis** -- the same count as the bounding-box report:
  2 LiDAR bodies (+65 mm), 4 `swedishWheel_*` meshes (+45-64 mm), 4 `wheel_respondable_*`
  spheres (+44-59 mm), 4 `intermediateLink_*` (+39-54 mm).
- **The probe's two measurements disagreed, and the disagreement is a finding.** Per shape,
  the probe passes a single handle to `checkCollision`; for the envelope it passes a
  collection. The wheel spheres reach 0.2085 m sideways by handle; the envelope, by
  collection, stops at 0.1958 m, the visual wheel mesh. Confirmed in isolation by
  `tests/test_collection_semantics.py` (sim, run once at one pose): a plate at +y 0.204 m
  touches `wheel_respondable_fl/rl` and `intermediateLink_fl/rl` as handles and *not* in a
  collection. Their collidable flag is off; the `swedishWheel_*` meshes' flag is on.
  **`checkCollision` ignores the collidable flag for an explicit handle and honours it
  inside a collection.**
- **Classification (ruling 8), by the flag, which is what the contact check uses:**
  collidable -- 2 LiDAR bodies, 4 `swedishWheel_*`; not collidable -- 4 `wheel_respondable_*`
  (respondable: they carry the physics), 4 `intermediateLink_*` (neither collidable nor
  respondable). My probe's "collidable" column classified by handle and so marked all 36
  collidable; it is wrong on these 8 and must not be quoted.
- **Envelope both ways** (10 deg, per-shape; agrees with the 2 deg collection scan):
  * flagged collidable -- front 0.3503, rear 0.3502, sides 0.1958/0.1959, circumscribed
    0.3511 m (at +-4 deg and +-176 deg: the LiDAR housings);
  * all shapes -- front 0.3503, rear 0.3502, **sides 0.2083/0.2085** (wheel spheres),
    circumscribed 0.3512 m.
- **Against the configured rectangle (0.3520 x 0.1970):** half_length over-states by 1.7
  mm; half_width over-states the flagged envelope by 1.1 mm and **under-states the wheel
  spheres by 11.5 mm**. The rectangle's corner radius, 0.4034 m, over-states the true
  circumscribed radius by 52 mm, which D1's soft band inherits.
- **Safety-relevant consequence for hard rule 4.** `RobotContact` builds its robot side as a
  collection, so it cannot see the wheel spheres. The spheres are what ODE uses for the
  wheels' physics and reach 12.5 mm past the mesh the contact check does see. A lateral
  scrape in that band -- a sphere pressing a shelf upright -- would be physical contact
  with no contact registered, and would not count as a collision. Not changed: which shapes
  define a collision is a decision for the supervisor (options in the session summary).
- **Also:** the envelope is a convex outline by construction (a support function), so the
  cross-shaped robot's re-entrant corners are filled in. Conservative for e(psi).

## 2026-09-18 (WP0) - Rulings on the WP0 results report: step 1 tooling (collision flag)
- **Ruling 1, option (a):** the four `wheel_respondable_*` spheres get the collidable flag.
- **The bit-identical before/after check cannot span two invocations.** The G1b smoke test
  and the full re-run (`G1b_20260918T104143Z`, `..104317Z`, same commit, same config)
  differ from the first logged row: 17 um in x at row 0, up to 4.2 mm along-track and
  0.08 mm cross-track by the end. Repetitions *within* one invocation have always been
  bit-identical. A GUI edit between two invocations would therefore fail the ruling's test
  with or without any effect of the flag. `scripts/regress_collidable_flag.py` keeps the
  bit-identical standard and moves it into one process: plate check, run A; plate check,
  run B (determinism control, must equal A); flag on via the API, plate check (a
  collection must now see a sphere), run C (must equal A). It does not save the scene.
  Profile translate_while_rotating at 0.30 m/s, which loads all four wheels. The cause of
  the cross-invocation difference is not investigated; recorded as a property of the
  platform (CoppeliaSim 4.10 / ODE).
- **G0's geometric reference had the same flag problem.** It measured `checkDistance`
  against a collection, which honours the *measurable* flag -- also off on the spheres.
  With only the collidable flag turned on, contact would fire on the spheres while the
  reference still measured to the mesh 12.5 mm further in: a false detection lag. The
  reference is now taken per handle over exactly the shapes whose collidable flag is on,
  so it tracks the contact definition whatever the measurable flag says.
- **Validation (ruling 1), G0 only:** every step, every robot shape is also checked as a
  single handle (flags ignored) and compared with `RobotContact` (`contact_agreement`,
  known-answer tests). A disagreement fails the run's exit status and is listed per step.
  Never in the environment's step loop: it costs one call per shape per step.
- **G0 runs on `configs/gates/g0_rerun.yaml` (half_width 0.2085).** The negative case
  places its wall 10 mm outside the configured half_width; at 0.1970 that wall would sit
  1.5 mm inside the now-counted spheres and fail by construction, testing the stale config
  rather than the contact check. 0.2085 is D8's value; the overlay exists only because G0
  precedes D8 in the ruled order.
- **Provenance:** every run now records `scene` and `scene_sha256`, hashed before the run
  directory is created (a missing scene leaves no empty run). G0 also records every robot
  shape's collidable/measurable/detectable flags and the physics in force.
- **Guard:** `tests/test_collection_semantics.py::test_wheel_spheres_are_collidable_in_the_scene`
  (sim) fails if the saved scene loses the flag.

## 2026-09-18 (WP0) - Wheel ceiling (ruling 2): not a velocity limit; torque cap suspected
- **Read from the scene** (`tests/test_wheel_joint_limits.py`, sim, read-only): all four
  rolling joints are velocity-controlled (`dynctrlmode` 4, no motion profile), maximum
  torque **2.5 N m**, `maxvel` 6.28 rad/s.
- **`maxvel` is not in force**: the wheels routinely run at 10-12.3 rad/s. So the 12.34
  rad/s ceiling is not a joint velocity limit.
- **The 2.5 N m torque cap is the candidate**: a velocity-controlled motor that cannot
  exert more than its cap settles where that torque balances the load. If so, the ceiling
  depends on load (payload, floor friction, how many wheels share the work), and v_rim is
  a property of this robot and floor rather than a constant -- which is what VERIFY on
  v_rim should mean.
- **Decisive check, prepared:** `scripts/diag_wheel_torque.py` drives G1's saturating
  diagonal command and logs every wheel's velocity and torque; torque at 2.5 N m on the
  plateau confirms the torque cap. Nothing is raised or changed (ruling 2).

## 2026-09-18 (WP0) - Rulings 2-6 on the WP0 results report (branch wp0-budget-footprint)
- **Held off master** until G0 is back: the ruled order is scene edit + regression, G0,
  then the budget, the footprint table and D8. Merged after G0.
- **Wheel sphere confirmed hidden** by the researcher: `wheel_respondable_fl` is a pure
  sphere on a hidden layer. The visible mecanum meshes do no physics. Ruling 1 stands;
  the write-up states that the counted contact envelope is 12.5 mm wider than the real
  wheel, conservative against the real platform (`docs/writeup_notes.md`).
- **Ruling 2, wheel budget:** `WheelBudget`, `project_to_wheel_budget` (one scale factor on
  (vx, vy, wz), never clipping), `wheel_budget_use` = |vx| + |vy| + (l+w)|wz|, shown equal
  to the fastest wheel's rim speed from the robot's own inverse kinematics to 1e-12.
  `actuation.wheel_budget`: v_rim 0.617 VERIFY, nominal 0.80, filter 0.95, with the
  authority reasoning in the config. `YouBotBase` enforces the filter share last and
  counts bindings; nominal controllers project into the nominal share themselves (the
  WP1 environment, before the filter). Box and budget are convex, so the acceleration
  limiter cannot leave them (tested over 2000 random commands).
- **Ruling 3, footprint table:** `pfbnav/safety/footprint_table.py` (pure) holds e(psi)
  at each whole degree with a periodic cubic spline giving e and de/dpsi.
  `footprint_table_from_polygon` raises every knot by the spline's worst undershoot of the
  true outline -- checked on a dense grid *plus every vertex bearing*, since e peaks in a
  kink at vertices and a uniform grid missed one by 0.17 mm -- plus a 1 um guard. Tests:
  circle, rectangle, never-below property on a 20011-point grid, finite-difference
  derivative, periodic wrap, CSV round trip. `scripts/build_footprint_table.py` builds the
  committed table from a probe run and refuses a run from another scene hash or a dirty
  tree. The table awaits the probe re-run on the edited scene.
- **Probe classification fixed:** collidable = the flag (what collections honour);
  `registers_by_handle` reported beside it. The fake simulator now applies both halves of
  the measured rule.
- **Ruling 4, velocity estimate:** `pfbnav/sim/velocity_estimate.py`, midpoint-heading pose
  differencing of `youBot_ref`, first-order filter (tau 0.10 s VERIFY), configurable noise
  and latency (committed off), documented as privileged. Known-answer tests: exact on a
  line, within (w dt)^2/24 on an arc, first-order step response, exactly one step of
  latency, seeded noise at the configured spread.
- **Ruling 5, H6 split** in `docs/pfb_robust_spec.md` §3a with the expected reduction
  stated before refitting: worst-case shortfall 27.8 % -> about 16-17 % once saturation is
  budgeted away; the bound is refitted to residual slip only.
- **Ruling 6:** `docs/writeup_notes.md`.
- **G2 cannot be prepared as specified.** `scripts/gate_g2_throughput.py` is a stub, and
  its specification measures environment steps with random actions and with SAC updates;
  the Gymnasium environment is WP1. Reported to the supervisor rather than substituted.

## 2026-09-18 (WP0) - Scene history repaired; first flag regression run
- **An unreviewed scene edit went into `ba8199d`.** The researcher saved the scene from the
  GUI at 14:47 local (spheres' collidable flag on) while checking the sphere; my
  `git add -A` swept the changed `.ttt` into `ba8199d` without review, and its commit
  message does not mention it. Scene SHA-256: original `2f7a8b8e...` (`a7c1a63`),
  flags-on `c89392ff...` (`ba8199d`, `63018b4`).
- **The researcher then undid the edit by hand** and ran the regression check
  (`REGRESS_FLAG_20260918T161931Z`, commit `63018b4`, dirty, scene `d0f1d15c...`). The undo
  left `intermediateLink_fl`'s collidable flag on (read by
  `tests/test_collection_semantics.py`), so a collection already saw geometry at the
  sphere's position before the edit and the script reported STOP, correctly.
- **Its dynamics result stands as evidence, not as the check:** runs A, B and C were
  bit-identical over all 378 rows -- turning the four sphere flags on changed nothing in
  the dynamics. It is not the ruled check because it ran on an accidental scene from a
  dirty tree. (`REGRESS_FLAG_20260918T161652Z`, the attempt before, stopped at the
  saved-scene flag check and ran nothing.)
- **Repair:** the original scene is restored byte for byte from `a7c1a63` and committed;
  the regression check is re-run on it; the researcher then makes the edit (four
  Collidable ticks, nothing else) and it is committed on its own. The `d0f1d15c` file is
  kept outside the repository. From now on the scene is staged by name, never by `-A`.

## 2026-09-18 (WP0) - Ruling 1 done: flag regression PASS; spheres collidable in the scene
- **Regression check PASS** (`REGRESS_FLAG_20260918T162434Z`, commit `bcbd57d`, clean, original
  scene `2f7a8b8e...`, ODE): collection saw a sphere before False / after True; A vs B and A
  vs C bit-identical over 378 rows x 14 columns (translate_while_rotating, 0.30 m/s).
  Turning the four sphere flags on changes nothing in the dynamics.
- **Scene edited by the researcher:** Collidable ticked on the four `wheel_respondable_*`,
  nothing else. Read back with `tests/test_collection_semantics.py`: spheres collidable and
  seen by a collection; `intermediateLink_*` not collidable; `swedishWheel_*` unchanged.
  New scene SHA-256 `340dbc5d75972436833d016e025e5294937b48af7ef9ccf69500124a6805dcdd`.
- **Note:** the researcher's `pytest` resolves to Python 3.13.13, while `python` -- which runs
  every script -- is 3.10.10. Sim tests should be run as `python -m pytest` so tests and
  scripts share one interpreter.

## 2026-09-18 (WP0) - G0 re-run after ruling 1: gate_script context passes 3/3; INCOMPLETE
- **Runs:** `G0_20260918T163047Z_seed0`, `..163306Z_seed1`, `..163719Z_seed2`; commit `61e4494`,
  clean, scene `340dbc5d...`, ODE, overlay `configs/gates/g0_rerun.yaml` (half_width 0.2085).
- **Every case passes in every repetition:** detection lag 0 steps on all five positive
  cases; the negative case records 0 contacts at a minimum gap of 0.0100 m. Verdict
  INCOMPLETE, as committed: `env_step_loop` needs the WP1 environment.
- **The spheres count now:** side_wall_lateral is struck first by `wheel_respondable_rl`.
  The negative case's 10.0 mm minimum gap, with its wall placed from half_width 0.2085,
  independently confirms the probe's sphere reach.
- **Handle/collection validation agrees on every step of all 18 case runs:** no shape the
  contact check cannot see touched an obstacle before one it can.
- **16 shapes have the collidable flag off, not 4** as I told the researcher to expect:
  the four `intermediateLink_*`, the arm links, `Rectangle0`, and the chassis `/youBot`
  itself. As built in the original scene. The chassis still counts, since RobotContact
  checks it as a single handle; but G0's geometric reference used flag-on shapes only and
  so omitted the chassis. Fixed (the chassis is now in the reference set). No re-run: the
  chassis led in no case -- the housings lead by 65 mm fore and aft, the wheels sideways --
  so the minimum distance was never the chassis's.
- **Across invocations:** wheel_edge was struck by `swedishWheel_fl` at step 39 on seed 1
  and by `swedishWheel_rl` at step 40 on seeds 0 and 2, the known cross-invocation
  non-determinism; same verdict.

## 2026-09-18 (WP0) - Wheel ceiling confirmed a torque limit (ruling 2)
- **Run:** `runs/diag/WHEEL_TORQUE_20260918T164140Z_seed0`, commit `61e4494`, scene
  `340dbc5d...`, ODE. G1's diagonal command (0.50, 0.30): two wheels asked 16 rad/s, two
  asked 4 rad/s.
- **Saturated wheels (fr, rl): 12.32 rad/s at |torque| 2.500 N m -- the cap.** Unsaturated
  wheels (fl, rr): 4.000 rad/s as commanded, on 0.77-0.89 N m.
- **So the ceiling is the 2.5 N m torque cap, not a velocity limit.** v_rim = 0.617 m/s is
  where full torque balances the load, and moves with load; `wheel_budget.v_rim` keeps its
  VERIFY for that reason, and is not raised (ruling 2). A budget share below 1 (nominal
  0.80, filter 0.95) also leaves margin for a heavier load pulling the ceiling down.

## 2026-09-18 (WP0) - Footprint probe counts the chassis by handle
- The chassis's collidable flag is off in the scene, but RobotContact checks it as a single
  handle, so it counts. The probe had exited on a flag-off chassis and measured its
  envelope against a collection, which skips it. Now: `collidable` = flag, or the chassis;
  the raw flag is reported as `collidable_flag`; the chassis's per-bearing values, measured
  by handle, are folded into the collection envelope. The fake robot's chassis is flag-off
  too, as in the scene.

## 2026-09-18 (WP0) - D8: v_max, vy_max, envelope, footprint table and planner radii
- **Footprint probe on the edited scene:** `FOOTPRINT_20260918T164846Z_seed0`, commit
  `f47362a`, clean, scene `340dbc5d...`, ODE; 36 shapes, 21 collidable (flag on, or the
  chassis by handle); no retries or fallbacks. Collidable envelope: front 0.3503, rear
  0.3502, left 0.2086 (upper bound 0.20856), right 0.2085, circumscribed 0.3511 m;
  symmetric to 0.1 mm. The four `intermediateLink_*` protrude 39-54 mm past the chassis
  with their flag off, inside the spheres and meshes. The all-shapes envelope (0.3551 m
  circumscribed) takes flag-off parts from bounding boxes and is not used.
- **Changes, in one commit:**
  * `robot.half_width` 0.1970 -> **0.2086**. The ruling said 0.2085; the probe's upper
    bound on the left is 0.20856, and the config's convention is the upper bound, since
    under-stating a safety footprint is the unsafe direction. A 0.1 mm departure in the
    conservative direction, flagged for the supervisor.
  * `robot.footprint_table` -> `configs/footprint/youbot_envelope_1deg.csv`: 360 knots,
    periodic spline, built from the probe run above. Every knot carries a 0.75 mm spline
    margin (the polygon's vertices are kinks the spline would otherwise undercut), so the
    table reads sides 0.2093, front 0.3510, circumscribed 0.3519 m -- uniformly 0.75 mm
    conservative. Tests pin the table to the scene hash, so a scene change fails them
    until the probe is re-run.
  * `actuation.vy_max` 0.3 -> **0.5**; `v_max` stays **0.5**. Both are axis ceilings inside
    the wheel budget (ruling 2), and commanded ceilings rather than speeds reached (A9);
    G1b passed everything to 0.50, the top of the tested grid (write-up notes).
  * Planner (D1): lethal radius half_width + 0.03 = **0.2386 m** (ruling: 0.2385, see
    above); soft band to the table's circumscribed radius + 0.03 = **0.3819 m**
    (ruling: 0.3512 + margin; the table's 0.3519 includes the spline margin). The
    circumscribed radius now comes from the table, not hypot(half_length, half_width),
    which over-stated the corners by 52 mm.
- **Dependents reviewed (A5):** `reward.time_cost`, `reward.max_steps`, the observation's
  velocity scaling, the PFB's stopping distance, the certified layer's speed bands and
  G2's projection. v_max is unchanged at 0.5, so none that depends on it changes. The
  observation scaling and the certified bands are not implemented yet (WP1/WP2) and will
  read vy_max and the table from the config. `robot.half_length` stays 0.3520 (not ruled;
  it over-states the probe's 0.3503 by 1.7 mm, the conservative direction).
- **Stale by design:** `configs/gates/g0_rerun.yaml` (half_width 0.2085) now differs from
  base by 0.1 mm; kept, because the G0 re-run runs name it. `configs/gates/g1b_rerun.yaml`
  now repeats base's box and is kept for the same reason.

## 2026-09-18 - WP0 COMPLETE
Closed by the supervisor on 2026-09-18. Gate verdicts, as reported:

| gate | verdict | run(s) | note |
|---|---|---|---|
| G0 contact validation | **INCOMPLETE** -- `gate_script` context PASS 3/3; `env_step_loop` not run | `G0_20260918T163047Z` / `..163306Z` / `..163719Z` | lag 0 on all positive cases, 0 false contacts, handle/collection validation agrees on every step; `env_step_loop` needs the WP1 environment |
| G1 holonomic fidelity (Bullet) | **FAIL**, stands (D6, A1) | G1 runs of 2026-09-17 | measured on the physics then in force |
| G1-ODE (A7) | **FAIL**, reported alongside | `G1-ODE_20260918T124531Z` / `..124712Z` / `..124854Z` | speed-shortfall failures an open-loop test cannot correct; diagonal also wheel-saturated |
| G1b closed-loop tracking, first run | PASS at v_max 0.40 -- **superseded**, produced under the clamp defect (A6) | `G1b_20260918T093210Z` | kept on record |
| G1b re-run (A6) | **PASS**, v_max = vy_max = 0.50 m/s, "passes everything tested" -- **superseded: measured before the wheel budget** (2026-09-19) | `G1b_20260918T104317Z` | top of the tested grid; the box exceeded the feasible set |
| G2 throughput | **DEFERRED to WP1** (option a) | -- | specification needs the environment; a profiling pass (not a gate) replaces it for now |

Also closed in WP0: D7 physics freeze (ODE, 0.005 s), the collision-flag regression check
(PASS, `REGRESS_FLAG_20260918T162434Z`), the footprint probe and table, and D8.

**Confirmations at closure:**
- `robot.half_width` 0.2086 and the soft band's origin at the table's 0.3519 m **stand**.
- "Measured values go into the config as the upper bound of their bracket" is now an
  explicit convention in CLAUDE.md.
- Determinism claims are scoped to one process. **WP5 requirement:** all controllers for a
  given scenario run inside one process, so pairing holds (`docs/requirements.md`).
- `v_rim` stays 0.617 m/s VERIFY, a load-dependent torque-balance point. A per-episode
  counter of wheels at the 2.5 N m torque cap is added (`YouBotBase.record_torque_cap`,
  `count_at_torque_cap` with known-answer tests) and is required in training and
  evaluation logs; if saturation appears, the budget fractions drop.
- G0 asserts the chassis is among the checked handles, every way it must be.
- **Scene handling is a standing rule** (CLAUDE.md hard rule 12): never staged implicitly;
  edits deliberate, read back, committed alone. `sim.scene_sha256` pins the scene; every
  run refuses another hash (`SceneMismatch`) and a test compares the committed scene with
  the pin. Both scene hashes are in `docs/reproducibility.md`.
- **G2 deferred to WP1.** A profiling pass of per-step cost (budget projection, command,
  physics step, contact check, pose read, velocity estimate, torque-cap read) across 1-4
  instances, written to `docs/profiling/`, never `runs/gates/`, with no PASS/FAIL;
  components above ~2 ms per step on average are flagged
  (`scripts/profile_step_cost.py`, `pfbnav/eval/profiling.py` with known-answer tests).
- **Current work package: WP1** (CLAUDE.md), starting with the D2 consistency test, G0's
  `env_step_loop` context, and the environment's per-step budget.

## 2026-09-18 (WP1) - D2 consistency test: aisle strata derived from the no-stop requirement
- **Requirement (D2):** a stationary robot centred in the narrowest aisle triggers no
  protective stop. The robot is 0.417 m wide (half_width 0.2086) with the measured
  footprint table.
- **Certified field geometry added** (`pfbnav/safety/certified.py`, pure; first WP1 piece of
  the certified layer): speed selects a band (`band_index`), each band's field is sized for
  the band's *top* speed, `field_depth` = v t_r + v^2/(2 a_b) + margin, and the boundary
  lies at e(psi) + depth along each bearing -- the PFB barrier's radial form. Invalid bins
  breach (hard rule 5). The footprint is passed in, so the module still imports nothing
  from the package (the architecture test forbids any name containing "pfb").
- **New config, VERIFY:** `certified.band_top_speeds` [0.10, 0.30, 0.50] m/s for the three
  field sets. Band 0 is the stationary robot's field; its top speed sets the D2 floor, so
  it is a safety-design choice for the supervisor, proposed here with its consequence.
- **Derivation** (`pfbnav/world/strata.py`): band-0 depth 0.10 x 0.10 + 0.10^2/2 + 0.10 =
  **0.115 m**; the field reaches max_psi (e + d0)|sin psi| = **0.3243 m** sideways; minus
  half_width gives the **per-side clearance floor 0.1157 m** (aisle 0.6486 m). The robot
  is taken aligned with the aisle; a turned robot's field reaches towards the 0.352 m
  circumscribed radius, which no narrow aisle admits -- a planner matter.
- **Strata proposed, VERIFY** (`world.aisle_strata`, per side): narrow [0.12, 0.20), medium
  [0.20, 0.40), wide [0.40, 0.80] m -- aisles from 0.657 m. Only the narrow floor is
  derived; the band edges are a proposal.
- **Tests** (`tests/test_d2_aisle_strata.py`): known answers for band selection, depth,
  lateral reach and invalid bins; the floor is a knife edge (no stop at +0.5 mm, stop at
  -0.5 mm); the required test on the configured narrowest stratum; every stratum strictly
  above the floor; `certified.margin` (0.10) and the band-0 depth (0.115) strictly below
  the narrowest clearance (D2's constraint to carry).
- **Next (WP1):** G0's `env_step_loop` context and the environment's per-step budget both
  need the environment; the budget is set from the profiling pass.

## 2026-09-18 (WP1) - Instance ports spaced by 2
- The researcher launches parallel instances on 23000, 23002, 23004, 23006. `sim.port_stride`
  (2) makes instance i listen on port + 2 i; `port_for_instance` takes the stride (tests
  updated). Spacing leaves the port beside each instance free, for ZeroMQ add-on versions
  that open a second port next to their own.

## 2026-09-18 (WP1) - Profiling pass (not a gate): per-step cost across 1-4 instances
**Measured with GUI rendering in force (~7 ms per step included), and with all four
instances open even for the 1-instance pass** (noted 2026-09-20). The per-step figures
are therefore upper bounds on what the same work costs with `sim.display_enabled:
false`; the component *shares* stand.

- **Output:** `docs/profiling/step_cost_20260918T182131Z/`, commit `f78b04f`, scene
  `340dbc5d...`, ODE, 400 steps per instance, ports 23000/23002/23004/23006. Provenance
  records `dirty: true`: the only untracked file at launch was the aborted attempt's
  directory (`step_cost_20260918T181001Z`, committed as a record in `d367d14`); no code
  difference.
- **Mean ms per step (1 / 2 / 3 / 4 instances):** physics step 49 / 59 / 47 / 59; command
  (4 joint writes) 6.3 / 6.9 / 5.4 / 6.3; torque-cap read (4 joint reads) 6.1 / 6.8 / 5.2 /
  6.1; contact check (2 calls) 5.2 / 5.9 / 5.3 / 5.7; pose (2 calls) 3.1 / 3.5 / 2.7 / 3.2;
  budget projection 0.03; velocity estimate 0.08-0.10. Whole step 70 / 82 / 66 / 80.
- **Flagged above ~2 ms:** every component that talks to the simulator -- command, contact,
  pose, torque-cap -- and the physics step itself. The two pure components are two orders
  of magnitude below the flag. A remote call costs about 1.5 ms (pose: 2 calls, 3.1 ms;
  command and torque-cap: 4 calls each, ~6 ms), so the robot-side I/O is ~21 ms of a
  ~70 ms step: about 30 %.
- **Throughput:** about 14 / 24 / 46 / 50 steps/s summed over 1 / 2 / 3 / 4 instances. The
  3-instance pass ran faster per instance than the 1-instance pass, so run-to-run variation
  is of the order of the differences between passes; one pass per instance count does not
  resolve scaling finely.
- **Anomaly:** the contact check's maximum is 0.6-1.0 s in every pass while its p95 is
  5-7 ms -- a rare, very slow call, probably the first check after collections are built.
  The script saved summaries, not per-step samples, so where the spike falls cannot be
  checked from this output.
- **Not measured:** the two LiDAR scanners, the policy and the PFB QP -- none exists yet.
  The environment's step will cost more than this.
- **Torque cap:** no wheel reached the 2.5 N m cap in any pass (0.2 m/s sinusoid; expected).

## 2026-09-18 (WP1) - Certified field made directional; D2 re-derived; traversal speeds
- **Rulings 2-4 on the WP1 report.** The radial field (full depth at every bearing) made
  narrow aisles untraversable above 0.10 m/s and would have made H1 measure our field
  geometry. The field is now directional, in the PFB's form: closing speed along bearing
  psi is s = v_top c(psi - phi), c = max(0, cos), and the boundary lies at
  `e + m_lat + s t_r + s^2/(2 a_b) + (m - m_lat) c` when moving -- the full depth plus
  margin m along travel, the fixed lateral margin m_lat = **0.05 m** at right angles and
  behind -- and at `e + m` (omnidirectional margin only, m = 0.10) at rest.
- **Interpretation to confirm:** "the omnidirectional margin" is read as the existing
  `certified.margin` (0.10), which also remains the along-travel margin when moving. So a
  robot at rest carries 0.10 m sideways, and 0.05 m once moving.
- **New VERIFY:** `certified.standstill_speed` 0.01 m/s, the estimated speed at or below
  which the robot counts as at rest. It must sit above the velocity estimate's noise at rest.
- **D2 re-derived:** the at-rest field reaches 0.3093 m sideways, so the at-rest floor is
  **0.1007 m** per side. The narrow floor is set by ruling 4 as lateral margin 0.05 + G1b's
  committed peak cross-track 0.05 + slack 0.02 = **0.12 m** (`world.narrow_floor_slack`),
  and the at-rest floor sits below it. Tests: known answers for the directional field;
  the rest floor is a knife edge; the stationary no-stop test on the narrowest stratum; the
  floor exceeds lateral margin + max cross-track (read from preregistration/).
- **Traversal speed per stratum (ruling 3) -- the status quo's cost:**

  | stratum (min clearance) | centred: v*, usable band | at 0.05 m tracking offset: v*, band |
  |---|---|---|
  | narrow (0.12 m) | 0.454 m/s -> band 0.30 | **0.083 m/s -> no band** |
  | medium (0.20 m) | 0.50 -> band 0.50 | 0.50 -> band 0.50 |
  | wide (0.40 m) | 0.50 -> band 0.50 | 0.50 -> band 0.50 |

  **Finding:** a robot at the edge of its committed tracking tolerance in the narrowest
  aisle has no band whose field fits. The ruled floor keeps the *lateral* margin clear at
  that offset, but band 0's forward depth still has a sideways component at oblique
  bearings, larger than the 0.02 m slack. Centred, the narrow stratum runs in band 0.30.
  The values are pinned by tests so that a change to the fields, footprint or strata
  that moves them fails until this record is updated.

## 2026-09-18 (WP1) - Rulings on the WP1 report: compute path, A10 drafted, budget status
- **Compute (ruling 1):** 7.5M steps do not fit six weeks at ~50 steps/s; target >= 100
  steps/s aggregate with the full environment. Spike diagnosis first: the profiler now
  saves every per-step sample, repeats the 1-4 instance sweep over interleaved passes,
  records core counts, and lists non-physics samples above 50 ms by step (`spikes`, with
  a known-answer test). Then the dynamics-step probe upward: A10 drafted in
  `docs/AMENDMENT_A10.staged.md` for the researcher to paste; `scripts/probe_dynamics_step.py`
  runs 0.005 / 0.010 / 0.025 s in one process against G1b's committed thresholds (ceilings
  unchanged) and the 0.64 deg drift band (lateral drift at 0.12 m/s, D7's speed -- set
  explicitly, since vy_max is now 0.5), times the bare physics step, reports the largest
  passing step, and refuses to run until A10 is committed. It adopts nothing.
- **Budget (ruling 6):** I/O <= 5 ms and step <= 100 ms p95 are interim; the binding
  requirement is >= 100 steps/s aggregate. Recorded in `docs/requirements.md` with the
  inconsistency: 100 ms p95 is 10 steps/s per instance, 40 over four.
- **Batched step (ruling 5):** approved; next. The per-call path stays as the reference and
  G0 runs through both; bit-for-bit equality over a multi-step trajectory in one process;
  same handles, chassis and wheel spheres included. **Scene pin:** a script stored inside
  the `.ttt` changes the file's bytes, so `sim.scene_sha256` covers it; files in `lua/` are
  covered by the commit hash and dirty flag. The gap is a scene script that is a stale copy
  of a `lua/` file -- to be closed when the batched step is built, by loading its code
  from `lua/` at run time rather than keeping a second copy in the scene.

## 2026-09-18 (WP1) - Contact-check spike located: first call after the collections are built
- **Run:** `docs/profiling/step_cost_20260918T194545Z/`, commit `486495c`, clean, scene
  `340dbc5d...`, ODE; 3 interleaved passes of the 1-4 instance sweep, 400 steps each, every
  sample saved. **Machine: 4 physical / 8 logical cores** (Intel family 6 model 142).
- **The spike is the first contact check of each run: step 0 in all 30 instance-runs, and
  nowhere else.** 0.6-1.5 s once, then 3-7 ms per call. The collections RobotContact
  builds at bind() are evidently prepared lazily on first use. Cost: about one second per
  episode reset, not per step; it can be paid inside reset() (a warm-up check) so it never
  lands in a control step. Not changed yet: it belongs with the batched step.
- **A separate machine-wide stall:** three of four instances took ~3.0 s over the same step
  (344-345) in 4-instance pass 0 -- simultaneous across processes, so the host, not the
  physics. It explains that pass's lower aggregate (40.9 steps/s against 49.2 and 49.3).
- **Aggregate steps/s by pass (1 / 2 / 3 / 4 instances):** 12.8, 12.5, 12.6 / 20.9, 20.8,
  21.3 / 34.0, 46.9, 35.2 / 40.9, 49.2, 49.3. Four instances on four physical cores reach
  ~49 steps/s; the 3-instance pass-to-pass spread (34-47) is as large as the difference
  between 3 and 4 instances. Physics step 56-68 ms mean; robot I/O ~23 ms (command 6.6,
  torque-cap 6.4, contact 3.8 median, pose 3.3). Against the >= 100 steps/s target, the
  physics step is the larger lever (A10) and the I/O the second (the batched step).

## 2026-09-18 (WP1) - Certified field rebuilt as a Minkowski sum; hysteresis; budget resolved
- **Supervisor's correction (WP1 report):** the oblique sideways reach was a shape error,
  not a cost. The field is now the Minkowski sum of the footprint with the braking segment
  along the travel direction u, offset by a stadium (a 0.05 m disc swept +-0.05 m along u):
  support h(n) = h_fp(n) + D max(0, n.u) + 0.05 |n.u| + 0.05. Ahead e + D + 0.10, abeam
  e + 0.05 at every speed, **behind e + 0.10** (the at-rest margin, as corrected), rearward
  travel putting the depth behind; at rest e + 0.10 all round. Boundary distances come
  from the support function (min over 1440 normals), which can only over-state the field.
  `FieldGeometry` replaces the radial functions.
- **Tests:** ahead/abeam/behind known answers; oblique bearings against an independent
  brute-force Minkowski sum of point clouds (agreement within 0.5 mm, never below); the
  field turns with travel; rearward travel; **maximum sideways extent = e(+-90 deg) + 0.05,
  identical at 0.1, 0.3 and 0.5 m/s** (the supervisor's required test).
- **Standstill hysteresis:** `StandstillDetector`, enter at <= 0.01 m/s, exit above
  0.02 m/s (both VERIFY), transitions logged with their step; known-answer test.
- **Narrow numbers re-derived:** sideways reach 0.2593 m at every speed. Every stratum now
  admits the top band (0.50), centred and at the 0.05 m tracking offset; at that offset in
  the narrowest aisle, 19.3 mm remains between field and wall. The earlier "no band fits"
  was the shape error. At-rest floor unchanged, 0.1007 m. No slack change needed.
- **Budget resolved (ruling 6):** >= 100 steps/s aggregate is binding; the per-step figure
  is derived from it and the instance count -- 4 instances => <= 40 ms p95 per step. The
  earlier 100 ms figure is withdrawn (`docs/requirements.md`).

## 2026-09-18 (WP1) - A10 as approved: probe updated
- A10 approved with additions, now in `docs/AMENDMENT_A10.staged.md` for pasting: fourth
  candidate 0.050 s; a G0 criterion (five positive cases, zero detection lag, at the
  candidate step, at the highest tested approach speed); physics ms/step and aggregate
  steps/s recorded per candidate; the dynamics step is the only change -- 20 Hz control,
  n = 0.05 / step physics steps (10, 5, 2, 1), asserted by the probe.
- **Interpretation to confirm:** "the highest tested approach speed" is read as **0.50
  m/s**, the highest speed any gate tests, rather than G0's committed 0.10 m/s -- a larger
  step moves the robot further per physics step, so fast approach is where contact could
  be stepped over.
- `scripts/probe_dynamics_step.py` runs, per candidate: the bare physics-step timing; the
  profiler's 4-instance measurement at that step (`profile(..., dynamic_step)`); G1b's grid;
  G0's positive cases at 0.50 m/s with the handle validation; the lateral drift run.

## 2026-09-19 (WP1) - A10 closed: 0.005 s stays; dynamics-step tuning is a closed question
- **Run:** `runs/diag/DSTEP_20260918T202513Z_seed0`, commit `cc04c63`, clean, scene
  `340dbc5d...`, ODE. All four candidates complete. Closed by the supervisor 2026-09-19.
- **Verdict: no larger step passes A10; 0.005 s stays.** Every candidate fails criterion 3,
  the drift band, on signed drift: reference +0.831 deg at 0.005 s; 0.010 s -0.177
  (gap 1.008), 0.025 s -0.802 (gap 1.634), 0.050 s -1.323 (gap 2.154); band 0.64 deg.
- **Defect in the probe, recorded next to the run.** The probe compared drift
  *magnitudes* (`abs(|d| - |d_ref|)`), so a drift of the same size in the opposite
  direction scored as "no difference", and its summary reports `largest_passing_step_s:
  0.05`. The band was measured on three drifts of one sign (+0.47, +1.11, +0.84 deg, D7;
  checked on the raw yaw series), so A10's criterion is on signed drifts. The run's files
  are unchanged. **Correction:** `docs/corrections/DSTEP_20260918T202513Z_seed0.{md,json}`,
  produced by `scripts/rescore_a10_drift.py` from the run's own `lateral.csv` files; it
  first checks that the recomputed magnitudes reproduce the recorded ones (to 1e-6 deg).
  **Code fix:** `signed_heading_drift_deg` and `drift_band_holds` in `pfbnav/eval/gates.py`;
  `heading_drift_deg` is now the magnitude of the signed value, and its docstring forbids
  comparing two of them; G1 records the signed drift; the probe applies the band to signed
  drifts. **Reversed-sign test:**
  `tests/test_gate_metrics.py::test_drift_band_fails_for_a_reversed_sign_drift_of_similar_size`
  (and the probe's three cases); the re-scorer is tested on a synthetic run
  (`tests/test_rescore_a10.py`).
- **Finding: physics is not the bottleneck.** Physics ms per step with four instances:
  58.9 / 61.4 / 51.2 / 49.8 at n = 10 / 5 / 2 / 1 physics steps per control step;
  aggregate 47.6 / 42.7 / 50.0 / 51.0 steps/s. A tenfold cut in physics work saves ~10 ms
  of a ~55 ms step. Even a passing step would have bought ~7 %.
  **This throughput table was measured with GUI rendering in force (~7 ms per step) and is
  not to be cited as-is** (noted 2026-09-20): it compares candidates like for like, which
  is what A10 needed, so **A10's verdict is unaffected**, but the absolute ms and steps/s
  belong to a configuration the environment will not run. Any throughput claim comes from
  the aggregate measurement with `display_enabled: false`.
- **Finding: tunnelling was not limiting.** G0's five positive cases were detected with
  zero lag at 0.50 m/s at every candidate, handle/collection validation agreeing on every
  step; G1b passed with ceilings 0.50 at every larger step. The step fails on drift only.
- **Dynamics-step tuning is therefore closed.** It is neither safe (drift) nor useful
  (throughput), and is not reopened for compute reasons.

## 2026-09-19 (WP1) - G1b under the wheel budget: re-run approved; WP0 result superseded
- **Observed in the A10 probe (one repetition, 0.005 s):** translate_while_rotating at
  0.50 m/s ends at a final heading error of 5.053 deg against the 5.0 limit; every other
  cell passes. The WP0 re-run (`G1b_20260918T104317Z`, commit `04ae863`) predates the
  wheel budget (`56fef1c`, WP0 results ruling 2): its runs carry no `budget_steps`.
- **Mechanism.** Under the budget the same cell's peak cross-track *fell* (25.65 ->
  12.66 mm) while its final heading error *rose* (0.27 -> 5.05 deg); realised speed fell
  0.392 -> 0.313 m/s, with the budget binding on 324 steps. The projection preserves
  direction: it scales (vx, vy, omega) by one factor, so the heading correction shrinks
  together with the translation. Holding the line gets easier (slower), holding the
  heading harder (less omega). **Motivation for the WP2 polytope constraint**
  (`docs/requirements.md`): a QP over the wheel-speed polytope can trade translation for
  omega instead of shrinking both.
- **Ruling (supervisor, 2026-09-19):** the WP0 result is labelled "superseded: measured
  before the wheel budget" and kept. G1b is re-run on the current controller, 0.005 s,
  three repetitions. v_max = 0.40 is **not** established from a 0.053 deg margin on one
  repetition: the spread across repetitions is reported, and if the repetitions straddle
  the limit the verdict at 0.50 is FAIL with the margin stated. If v_max lands at 0.40,
  band tops, per-stratum traversal speeds and the narrow floor are re-derived in one commit.
- **Implementation.** Repetitions inside one process are bit-identical, so a spread needs
  separate invocations: three invocations (`--seed 0/1/2`, `--instance 0/1/2`, in
  parallel), each running the committed three repetitions, aggregated by
  `scripts/aggregate_g1b.py` (`pfbnav/eval/tracking_spread.py`: per-cell min/max/spread,
  worst margin, invocations within, status pass/straddle/fail; a straddling cell fails its
  speed; ceilings lowest-first as in A6). The aggregator refuses invocations that differ in
  commit, dirty state, scene, criteria or physics. `gate_g1b_tracking.py` gains
  `--instance` and records the wheel budget in its summary.
- **Methods chain** (for the write-up): clamp defect (first run, v_max 0.40, superseded,
  A6) -> budget absent (re-run, 0.50/0.50, superseded) -> current controller (this re-run).
  `docs/writeup_notes.md`.

## 2026-09-19 (WP1) - Fixed-cost diagnosis approved, before the batched step
- **Approved, time-boxed to one day.** `scripts/diag_step_floor.py`, one instance, ms per
  `sim.step()` with the dynamics step held at 0.005 s, per configuration: the warehouse
  scene as loaded, real-time off, real-time on, idle fps 0, display off (GUI only),
  dynamics off, youBot alone (with the floor it rests on), fully empty scene; plus a bare
  remote call as the I/O reference. Run once headless and once with the GUI (`--mode`,
  checked against `sim.boolparam_headless`). It verifies stepping mode is in force
  (`sim.setStepping` level > 0) after connect, after every scene reload and at the end,
  and inventories the scene's scripts (type, enabled, executions per step), vision and
  proximity sensors, including any v1 leftovers. The scene is modified in memory only,
  reloaded afterwards and never saved; its file hash is checked before and after.
- **Read from the ZMQ server add-on (CoppeliaSim 4.10), to be tested rather than
  assumed:** `sim.setStepping` is cumulative and per client -- our client enables it on
  connect and again on every `start()`, so the level climbs; harmless while > 0, and now
  recorded. Each `sim.step()` returns only after a full main-loop pass.
- **Decision rule, set in advance (supervisor):** floor = the cheapest representative
  configuration (the warehouse scene with the robot and frozen physics; stripped scenes
  and physics-off locate the cost but never set the floor), headless binding. **Below ~15
  ms per step -> proceed to the batched step and keep the 100 steps/s target. Otherwise
  stop optimising and re-plan the matrix at WP1 close, in this order: more instances, then
  total_timesteps, then seeds for A1 and L2 -- never L1, L4, L5.** Coded as
  `floor_decision` (`pfbnav/eval/profiling.py`, `STEP_FLOOR_THRESHOLD_MS = 15.0`), with
  known-answer tests, before the diagnosis runs.

## 2026-09-19 (WP1) - Every step-cost measurement so far was taken with the GUI
- **Stated by the researcher, 2026-09-19:** all previous tests ran on GUI instances, not
  headless. So the profiling pass (`docs/profiling/`), the A10 probe's throughput figures
  and the ~50 ms per-step floor all include whatever the GUI costs per step. Nothing
  measured so far describes a headless instance.
- **Consequence for the diagnosis:** a GUI run is the like-for-like one, and the mode that
  binds the decision rule is the mode training will run in -- a launch decision, not a
  property of the diagnosis. `scripts/diag_step_floor.py` records the mode it measured in
  (`decision.measured_in_mode`) and no longer labels a GUI run non-binding.
- **Rendering is switchable at run time**, so staying with the GUI costs nothing if it is
  the cost: `display_off` (`sim.boolparam_display_enabled = False` inside a GUI instance)
  is the in-GUI equivalent of headless and is in the representative set, so it can set the
  floor. If the floor falls there, the environment sets it on start through a `sim` config
  flag, and instances keep being launched exactly as they are now.
- **Headless is then only a confirmation** that nothing else about the launch mode matters,
  and is not needed to decide the path.

## 2026-09-19 (WP1) - Fixed-cost diagnosis: the step is ~27 ms, and 10.5 ms of it is dead v1 LiDAR
- **Runs:** `runs/diag/STEPFLOOR-gui_20260919T232851Z_seed0` (first pass, superseded: one
  pass per configuration in a fixed order, which made `youbot_alone` read *slower* than the
  full scene), `..233658Z` (three interleaved passes, fresh scene per configuration) and
  `..234447Z` (adds the combined configurations). Commits `fcbaee9`, `fec1d70`, `aa35262`,
  clean, scene `340dbc5d...`, GUI confirmed by `sim.boolparam_headless`, scene file
  byte-identical before and after each run.
- **Per-step cost, warehouse scene, one instance, dynamics step 0.005 s** (mean ms per
  `sim.step()`, three passes agreeing to ~0.5 ms):

  | configuration | 233658Z | 234447Z | isolates |
  |---|---|---|---|
  | as_loaded | 26.8 | 27.5 | baseline (real-time already off in the scene) |
  | realtime_on | 49.9 | 49.9 | wall-clock pacing; not in use |
  | idle_fps_0 | 27.1 | 28.3 | no effect |
  | display_off | 19.9 | 20.2 | rendering ~7 ms |
  | dynamics_off | 20.8 | 21.3 | physics ~6.5 ms |
  | no_lidar | 17.0 | 17.9 | the v1 LiDAR ~9.6 ms |
  | youbot_alone | 24.4 | 25.5 | warehouse furniture ~2 ms |
  | no_lidar_display_off | -- | **10.8** | the two savings are additive |
  | no_lidar_display_off_no_furniture | -- | 10.2 | furniture is 0.6 ms once LiDAR is gone |
  | empty_scene | 8.0 | 8.0 | irreducible cost of a step |

- **The largest single cost in the step is dead weight.** The scene inventory found **four
  256x1 vision sensors in two duplicated `/youBot/LiDAR` subtrees inside the youBot model**
  (14 objects), left from v1; v2's scanners do not exist yet and nothing reads them. A
  vision sensor that no script handles explicitly is rendered by the main script every
  step, so they cost ~9.6 ms per step for nothing. Physics costs ~6.5 ms, less than the
  leftovers.
- **Neither the launch mode nor real-time pacing is the problem.** Rendering costs ~7 ms
  and is switchable at run time, so GUI instances are fine and headless would buy nothing;
  `realtime_simulation` is already off in the scene (turning it on costs 50 ms/step, which
  is what real-time pacing at 20 Hz means); `idle_fps` does nothing while running.
- **Decision rule as measured: floor 20.2 ms (`display_off`) > 15 ms -> replan_matrix.**
  With the v1 LiDAR removed as well, the adoptable floor is **10.8 ms
  (`no_lidar_display_off`) -> batched_step**. Removing the LiDAR is a deliberate scene edit
  (hard rule 12) and the supervisor's decision, so the script keeps it out of the
  representative set and reports it separately; a variant that also strips the warehouse
  furniture is not adoptable and was wrongly allowed to set that separate figure in
  `..234447Z` (reported 10.2 ms / `..._no_furniture`). Fixed after the run; the adoptable
  figure is 10.8 ms and the verdict is unchanged.
- **Also recorded:** stepping mode was in force (level 1) in every timed configuration;
  the level returns to 0 whenever the simulation stops and `SimClient.start` re-enables it,
  so the climbing level noted on 2026-09-19 does not happen in practice. No script
  executed during any timed step, and all twelve report disabled -- consistent with the
  hidden spheres doing the wheel physics, and contradicting the comment in
  `configs/base.yaml` that the rollingJoint scripts drive the slipping joints (out of
  scope; reported, not changed).
- **CORRECTION, same day, before anything was acted on: the LiDAR pods are the robot's
  scanners, not leftovers, and must not be removed.** `robot.half_length` 0.3520 is set by
  the forward pod at `/youBot/LiDAR/body`; the rear pod was fitted on 2026-09-17 so that
  the envelope is symmetric; `configs/base.yaml` already specifies `scanners.policy_bins`
  36 and `filter_bins` 120; and `docs/migration_v1_to_v2.md` has "Hokuyo Lua child script
  -> `lua/scanner_v2.lua`, rewrite (WP1), dual scanners". Removing them would also
  invalidate `half_length` and therefore the footprint table, the lethal radius, the
  certified field and the D2 strata. I read "nothing reads them today" as "nothing needs
  them"; that was wrong. The configuration is renamed `no_scanners` and is explicitly not
  adoptable: what it measures is the **budget the WP1 scanner design has to fit**.
- **What the measurement actually says, restated.** With the display off, a step costs
  20.2 ms, of which the scanners as currently built are 9.4 ms and everything else is
  10.8 ms. The scanners render **four 256x1 vision sensors = 1024 rays per step** while
  the observation and filter streams need **36 + 120 = 156 bearings**. Nothing reads the
  output yet, so all of it is currently wasted, and even once `scanner_v2.lua` exists the
  cost is set by how the scan is produced, not by what the policy consumes.
- **Open, for the supervisor:** (a) **scanner implementation is now a WP1 compute
  question, not a deletion:** measure step cost against vision-sensor resolution (256 down
  to the bins actually needed), explicit versus implicit handling, and any cheaper scan
  method, with the pods and `half_length` untouched; (b) set the display off from the
  environment at start (a `sim` config flag), keeping it switchable for debugging; (c) the
  batched-step decision then rests on whether scanning fits the ~4 ms that 15 ms per step
  leaves once the other 10.8 ms is paid. **Still to check:** the earlier profiling pass
  measured 56 ms for a single instance's step with four instances open, against ~27 ms
  here with one; idle instances appear to inflate each other, which matters for the
  aggregate target and needs re-measuring.

## 2026-09-20 (WP1) - Scanner cost: partial run, then stopped; the rule's verdict does not turn on it
- **Run:** `scripts/diag_scanner_cost.py`, commit `55884fd`, one GUI instance. Pass 0
  completed nine configurations and then aborted on `script 91: enabled did not take`.
  Stopped there by the researcher's decision, with the supervisor to be informed: the
  remaining measurement cannot change the decision (see below).
- **Measured, pass 0 (four sensors, 120 deg each, 256 rays as built; 120 bins over 360 deg
  would need 41 rays per sensor):**

  | configuration | step ms | scan ms | total | rays |
  |---|---|---|---|---|
  | implicit_256 | 26.3 | -- | 26.3 | 1024 |
  | implicit_81 | 25.9 | -- | 25.9 | 324 |
  | implicit_41 | 25.2 | -- | 25.2 | 164 |
  | explicit_256 (nothing handled) | 20.1 | -- | 20.1 | 1024 |
  | explicit_256_read | 19.9 | 14.4 | 34.4 | 1024 |
  | explicit_81_read | 19.8 | 14.7 | 34.6 | 324 |
  | explicit_41_read | 20.2 | 17.0 | 37.2 | 164 |

- **Rendering cost is per sensor, not per ray.** Cutting rays 6x (1024 -> 164) saves ~1.1
  ms; not handling the sensors at all saves ~6.2 ms, about 1.5 ms per sensor. Sizing the
  sensors to the bins the streams consume is therefore nearly pointless as an optimisation,
  and `pfbnav/sim/scanning.resolution_for_bins` is kept for sizing the scan correctly, not
  for saving time.
- **A per-call scan is unaffordable, and it is latency, not data.** Four handles plus four
  depth reads is eight round trips: 14-17 ms per step, and the *smallest* buffer (656 B)
  was the *slowest* (17.0 ms). At ~1.5-1.8 ms per remote call this is round-trip cost.
  The scan must be assembled server-side and returned once -- which is what
  `lua/Lidar_Scripts/child_script.lua` already does for v1 (36 bins, 144 B, signal
  `youBot_lidar`) and what `lua/scanner_v2.lua` must do for 36 + 120 bins with bearings
  and validity.
- **Two defects found and fixed (`aaf5372`), both from the deprecated int32-parameter
  API.** `sim.setObjectInt32Param(script, scriptintparam_enabled, 1)` is silently refused
  in 4.10; the live route is the `scriptDisabled` property. **The step-floor inventory's
  claim that all twelve scripts are disabled and never execute came from that same
  deprecated read and is retracted** -- it is not evidence either way. The script now also
  records the settings in force per configuration and refuses to run if the display did
  not switch off: pass 0's figures sit ~6 ms above the step-floor run's display-off
  numbers, which is what an unapplied `display_enabled` looks like.
- **Researcher's clarifications (2026-09-20), recorded because they close two open
  questions:** `showLines = false` in the v1 child script disables only the drawn beams,
  for memory; detection is unaffected. The stock Hokuyo pod uses two vision sensors to
  cover 240 deg, so four sensors across two pods is the model's own geometry, not a
  redundancy to trim.
- **Why the rest of the measurement was skipped.** The pre-set rule compares the floor
  against 15 ms per `sim.step()`. Everything except the scanners costs 10.8 ms
  (`no_scanners_display_off`, step-floor run); the scanners are required and their
  rendering is a fixed ~1.5 ms per sensor; so any working configuration is at least
  ~17-20 ms and the rule's verdict is **replan_matrix** whatever the Lua binning costs.
  The binning cost is still unknown and still matters for the environment's per-step
  budget -- it is measured by the profiling pass once `scanner_v2.lua` and the environment
  exist, which `docs/requirements.md` already requires. `scripts/diag_scanner_cost.py`
  stays in the repo, fixed and unrun, if that cost is wanted earlier.
- **Tension to put to the supervisor, not resolved here.** The 15 ms threshold was a proxy
  for reaching 100 steps/s aggregate. With the display off (20.2 ms per step) and the
  batched step replacing ~23 ms of per-call I/O with one call, a step would be ~23 ms:
  ~43 steps/s per instance, and the profiling pass scaled linearly across instances
  (12.6 / 10.5 / 12.9 / 11.6 steps/s per instance at 1 / 2 / 3 / 4 instances). Four instances would
  then be ~170 steps/s, above the target. So the rule says stop and re-plan, while the
  arithmetic says the batched step may still reach the goal the rule was protecting. The
  rule was set in advance and is not reinterpreted here; the supervisor decides.

## 2026-09-20 (WP1) - Deviation from the pre-set step-floor rule, declared and bounded
- **The rule, as set in advance on 2026-09-19 and NOT amended:** floor < ~15 ms per
  `sim.step()` -> batched step, target kept; otherwise stop optimising and re-plan the
  matrix at WP1 close. **Measured floor 20.2 ms, so the rule returns `replan_matrix`.**
  It stands in the record exactly as written; nothing here rewrites it.
- **Deviation (supervisor, 2026-09-20): build the batched step and measure aggregate
  throughput with four instances before re-planning.** Reason: the 15 ms threshold was a
  **proxy** for >= 100 steps/s aggregate, chosen before anyone knew the floor decomposes
  into ~7 ms of rendering that can simply be switched off and a fixed scanner cost that
  cannot. A proxy that was set without that knowledge should not decide the question its
  goal can now be measured against directly.
- **Pre-declared decision point, set before the measurement exists:** measured aggregate
  **>= 100 steps/s -> proceed to the environment, no re-plan**; **< 100 -> re-plan the
  matrix at WP1 close in the committed order** (more instances, then `total_timesteps`,
  then seeds for A1 and L2; never L1, L4, L5), **with no further optimisation**.
- **Time-box: two days. If the aggregate is not measured by then, re-plan regardless.**
- **The measured step must be the full one:** batched I/O, the server-side scan including
  its binning loop, the contact check, the pose read, the velocity estimate and the budget
  projection. A step missing any of these is not the step training pays for.
- **The 170 steps/s figure is a projection, not a result.** It comes from 20.2 ms per step
  with the display off, plus a batched call replacing ~23 ms of per-call I/O, times four
  instances at the observed near-linear scaling. It is the hypothesis this measurement
  tests, and it is not to be quoted as a measurement anywhere.
- **Order (supervisor):** the approved G1b re-run first, since `v_max` and everything
  derived from it may change; then the batched step and the aggregate measurement.

## 2026-09-20 (WP1) - Display off by default; the display state is provenance
- **`sim.display_enabled: false`** in `configs/base.yaml`. `SimClient.start()` applies it
  and **reads it back**, raising if the instance disagrees; a headless instance has no
  display to switch and records `false` rather than pretending the write landed. Every
  run's `provenance.json` now carries `display_enabled`.
- **Rendering must change cost and nothing else**, so `scripts/regress_display_off.py`
  runs one G1b profile twice **in one process** -- display on first, then off -- and
  compares the trajectories field by field; anything but bit-identical means the default
  is not free and must be revisited. Determinism is claimed within a process only, which
  is why both runs share one. Tests: `tests/test_display_flag.py`, including a fake whose
  write silently does nothing.
- **To watch a run**, set `sim.display_enabled: true`; the run records that it was on.

## 2026-09-20 - Standing rule: a deprecated read, or a silent write, is not evidence
- **Rule.** A value read through a deprecated API, or a setting written without
  confirmation, carries no evidential weight. Settings are established by **read-back**
  and, where the effect is observable, **by the effect**: real-time pacing was confirmed
  by the step cost moving to 50 ms, which is what 20 Hz wall-clock pacing means. Findings
  asserted from an unverified read are withdrawn, not downgraded.
- **Why.** Two failures on 2026-09-19/20, both from the same deprecated route:
  `sim.setObjectInt32Param(script, scriptintparam_enabled, 1)` is silently refused in
  CoppeliaSim 4.10 (the live property is `scriptDisabled`), and the matching read is what
  produced the retracted claim that all twelve of the scene's scripts are disabled and
  never execute. A third, still open: the first scanner run's figures sit ~6 ms above the
  step-floor run's display-off numbers, which is what an unapplied `display_enabled` looks
  like -- the script asked and never checked. All three are now read back.
- **Applies to:** `pfbnav/sim/client.py` (`apply_display`, `set_physics`, `assert_timestep`
  already did this), the diagnostics under `scripts/`, and anything that configures the
  simulator in WP1 and later.

## 2026-09-20 (WP1) - A display refusal aborted the G1b re-run at run 53 of 66; made non-fatal
- **What happened.** With `sim.display_enabled: false` newly in force, all three G1b
  invocations died partway: `SimError: asked for display_enabled=False on 127.0.0.1:23002,
  the instance reports True`, at run 53 of 66 (seed 1). The first 52 runs applied the
  setting without complaint, so this is not "the instance refuses while stopped": something
  re-enabled rendering mid-run.
- **Cause: unexplained.** The likeliest candidate is interaction with the GUI window (the
  visualisation toggle, or clicking into the scene view), since the runs were watched. Not
  reproduced, not confirmed, and recorded as unexplained rather than guessed at.
- **My design error, and the fix.** Rendering is a performance setting that must change
  cost and nothing else, so refusing it makes a run slower, not wrong -- and raising threw
  away 45 minutes of a gate. `SimClient.apply_display` now **counts and reports** a
  refusal (hard rule 9's "explicit status that is logged and counted") instead of raising:
  `display_in_force` records what actually happened, `display_refusals` counts the
  refusals, and `display_consistent` goes false if the state changed part-way through a
  session. G1b writes all three into its summary. What the client must never do -- and
  still does not -- is claim a setting took when the read-back says otherwise (hard rule 13).
- **Consequence for the re-run.** No results were lost that mattered: the three invocations
  are re-run from the start. If the display flips again the run now finishes and reports
  it, and `scripts/regress_display_off.py` settles whether a flip could have changed a
  trajectory at all (it must not).

## 2026-09-20 (WP1) - Open question: does the >= 100 steps/s target follow from its own premise?
- **The premise, as written** (WP1 report ruling 1; `docs/requirements.md`; A10's "known at
  the time"): "the training matrix (25 runs x 300k = 7.5M steps) does not fit six weeks at
  the measured ~50 steps/s". `preregistration/gates.yaml` G2 carries the same ingredients:
  `wallclock_budget_weeks: 6`, `n_training_runs: 25`, and `training.total_timesteps:
  300000` in `configs/base.yaml`.
- **The arithmetic does not support it.** Six weeks is 3,628,800 s, so 7.5M steps fit if
  the aggregate rate is **>= 2.07 steps/s**:

  | aggregate | 7.5M steps |
  |---|---|
  | 2 steps/s | 43.4 days |
  | 10 | 8.7 days |
  | 25 | 3.5 days |
  | 50 (measured, GUI, 4 instances) | **1.74 days** |
  | 100 (the target) | 0.87 days |

  At the measured rate the matrix's stepping is about **42 hours**, not six weeks. The
  target is ~50x the rate the stated justification requires.
- **What could still make the budget bind, none of it measured:** SAC gradient updates
  (300k per run x 25 runs = 7.5M updates) -- and **torch here is CPU-only** (2.11.0+cpu,
  `cuda.is_available()` False) on **4 physical cores** that four CoppeliaSim instances
  already occupy; the per-step cost of the scanner binning, the PFB QP and the observation
  assembly, none of which exist yet; evaluation over the scenario sets across controllers
  and seeds (WP5); and tuning sweeps (WP3/WP4). The measured 50 steps/s is `sim.step()`
  plus per-call I/O with **no scanners, no policy and no filter**.
- **Why this matters now.** The compute work since 2026-09-18 -- A10, the step-floor
  diagnosis, the scanner measurement, the batched step and its aggregate test -- all serves
  a target whose justification may be arithmetically wrong. If the six-week budget is not
  threatened by stepping, the remaining compute work is optional and WP1's environment can
  start immediately; G2, already preregistered with the six weeks and 25 runs, measures the
  real end-to-end rate once the environment and SAC exist and decides then, which is what
  it was deferred to WP1 to do.
- **Researcher's clarification (2026-09-20), which decides most of it: training will run
  on a separate GPU machine, not on this one.** Every throughput figure in this project --
  the profiling pass, A10's table, the step-floor diagnosis, the 50 steps/s in the premise
  -- was measured on the **development laptop**: 4 physical cores, CPU-only torch, four
  CoppeliaSim instances sharing those cores. None of it describes the machine that will
  train, so neither the premise nor the >= 100 steps/s target derived from it can be
  settled here. The CPU-only-torch concern above applies to this laptop alone.
- **Consequence.** A per-step budget and an instance count are properties of the training
  host: the instance sweep and the aggregate measurement belong on that machine, which is
  what G2 (`wallclock_budget_weeks: 6`, `n_training_runs: 25`, `parallel_instances:
  [1, 2, 3, 4]`) already specifies, and G2 was deferred to WP1 precisely so it could run
  once the environment and SAC exist. The findings that **do** carry over are the ones
  about shape rather than speed: the dynamics step is closed (A10), rendering changes cost
  and not results, a per-call scan costs 14-17 ms of round trips wherever it runs, and the
  scan must therefore be assembled server-side.
- **Not acted on.** The target is the supervisor's and stands until they rule. Raised as a
  question about the premise, with the arithmetic and the unmeasured terms above; the
  batched step is paused rather than abandoned pending that ruling.

## 2026-09-20 (WP1) - The >= 100 steps/s target was an arithmetic error; replaced by >= 25 on the training host
- **The error, stated by the supervisor:** "the >= 100 steps/s target came from my
  arithmetic error (7.5M steps over 3.6M seconds is 2.1 steps/s, not 57)". Six weeks is
  3,628,800 s; 7.5M steps over that window need **2.1 steps/s**, and at the measured ~50
  steps/s the training matrix's stepping takes **~42 hours**, not six weeks. The premise
  in the WP1 report (ruling 1), in `docs/requirements.md` and in A10's "known at the time"
  was wrong by a factor of ~27.
- **Replacement requirement (approved 2026-09-20): >= 25 steps/s aggregate on the
  training host**, with the full environment. Derived from ~20-25M total simulator steps
  -- 7.5M training (25 runs x 300k), 5-9M evaluation, 3-4M tuning, plus 20% contingency --
  against the six-week window: ~20M steps need 5.5 steps/s to fill the window end to end,
  and 25 steps/s completes them in ~9.3 days, leaving the rest for analysis, failures and
  re-runs. Both numbers are recorded so the derivation reproduces. The old figure is
  marked RESOLVED in `docs/requirements.md` with the derivation and the correction.
- **Unmeasured term G2 must cover: 7.5M SAC gradient updates**, one per environment step.
  No update has been timed anywhere in WP0 or WP1, and on the training host they run
  alongside the simulator instances: G2's aggregate is the environment **and** the
  learner, not `sim.step()` alone.
- **Standing rule (supervisor):** every numeric requirement in `docs/requirements.md`
  carries its derivation -- inputs, arithmetic, window -- and **a requirement whose
  derivation does not reproduce is challengeable**: recompute it, say so, and do not treat
  it as binding until it is re-derived or re-ruled. Recorded at the head of that file.
- **What the wrong target cost, and what survives it.** Two days went to A10, the
  step-floor diagnosis, the scanner measurement and the batched-step plan. What survives
  is shape rather than speed: the dynamics step is closed at 0.005 s (A10); a per-call
  scan costs 14-17 ms of round trips on any host, so the scan is assembled server-side;
  scanner cost is per sensor, not per ray; rendering changes cost and not results.

## 2026-09-20 (WP1) - WP1 environment starts; scan packet mandatory, batching only if free
- **Approved (supervisor):** start the environment now.
- **Mandatory design: one server-side scan packet per tick.** A per-call scan is
  unaffordable on any host (14-17 ms of round trips, measured 2026-09-20, independent of
  buffer size), and **the bins must come from one tick** -- a scan stitched from sensors
  read at different times is not a scan of one instant. `lua/scanner_v2.lua` bins inside
  the simulator and returns 36 policy bins and 120 filter bins with their bearings and
  validity flags, in one packet.
- **Batching contact, pose and torque: only if it falls out naturally.** Not a goal of its
  own. If it does, it carries the condition set with the batched step (WP1 report, ruling
  5): the per-call path stays as the reference, and a **bit-for-bit equality test over a
  multi-step trajectory in one process** compares the two.
- **G2 runs on the training machine** once the environment and SAC exist.

## 2026-09-20 (WP1) - Host identity in every run's provenance
- **Why.** A timing figure is a property of the machine. Every throughput number in WP0
  and WP1 came from the development laptop (`DESKTOP-5MIDQ97`, 4 physical cores, 8 logical,
  7.7 GB, torch 2.11.0+cpu with no GPU), while training runs on a separate GPU machine,
  and nothing in the run directories said so.
- **`provenance.json` now carries `host`:** hostname, OS, Python, CPU model, logical and
  physical cores, RAM, GPU (via torch, or whether `nvidia-smi` is present) and `shared`.
  `shared` is **declared** in `configs/base.yaml` (`host.shared`), not detected: whether
  someone else was using the machine is not something a process can see, and a contended
  run's timings are not comparable with an uncontended one's. Absent config leaves it
  `None` -- "nobody said" and "not shared" must not be the same record.
  `pfbnav/utils/runio.py::host_identity`, `tests/test_host_provenance.py`.

## 2026-09-20 (WP1) - The aborted G1b attempt is discarded entirely
- **Void, by the supervisor's instruction:** `runs/gates/G1b_20260920T004112Z_seed0`,
  `..004127Z_seed1`, `..004137Z_seed2`. All three died on the display refusal (seed 1 at
  run 53 of 66, the others earlier) and **none wrote a `summary.json`**, so none can enter
  the aggregator even by accident. They are left on disk unmodified (hard rule 1) and are
  not to be cited, quoted or partially aggregated.
- **The re-run reports the spread across all three invocations**, and **if they straddle
  the 5.0 deg limit the verdict at 0.50 m/s is FAIL with the margin stated** -- already
  how `pfbnav/eval/tracking_spread.py` scores a straddling cell.
- **What the aborted attempt suggested, and which proves nothing on its own:** seed 1
  reproduced the A10 probe exactly at `translate_while_rotating` 0.50 m/s -- final heading
  5.05 deg against the 5.0 limit, peak cross-track 13.0 mm. One invocation, discarded.

## 2026-09-20 (WP1) - G1b under the wheel budget: PASS at v_max 0.40; derived values re-set
- **Runs:** `G1b_20260920T211601Z_seed0`, `..211603Z_seed1`, `..211623Z_seed2`, aggregated
  by `G1b-SPREAD_20260920T215108Z_seed0`. Commit `ced9e61`, clean, scene `340dbc5d...`,
  host `DESKTOP-5MIDQ97`, three invocations x the committed three repetitions. The
  aggregator checked the invocations agree on commit, dirty state, scene, criteria and
  physics before combining them.
- **Verdict: PASS, v_max 0.40 m/s, vy_max 0.50 m/s.** 0.10-0.40 pass in every invocation;
  0.50 fails on one cell:

  > `translate_while_rotating` at 0.50: final heading error **5.033-5.052 deg** against
  > 5.0, **0 of 3 invocations within**, worst margin **-0.052 deg**; peak cross-track
  > 12.9-13.0 mm, realised 0.313 m/s, the wheel budget binding on 324 steps in all three.

  **It does not straddle -- all three miss** -- so the supervisor's straddle rule
  (2026-09-19) did not have to be invoked; this is a clean failure at 0.50.
- **Repetitions inside each invocation were bit-identical** (all three invocations), as
  the determinism claim requires; the spread across invocations is 0.019 deg on the
  deciding cell.
- **Display refusals: 3-4 per invocation, `consistent: false` in all three.** The same
  unexplained mid-run flip as earlier in the day, now counted rather than fatal. **It
  cannot have changed these results:** `DISPLAY_20260920T211342Z_seed0` compared one G1b
  profile with the display on and off in one process and found the trajectories
  **bit-identical** (362 rows, no differing field).
- **Re-derived on v_max 0.40, in this commit:**
  | quantity | was | now | why |
  |---|---|---|---|
  | `actuation.v_max` | 0.50 | **0.40** | D8 writes the gate's output |
  | `actuation.vy_max` | 0.50 | 0.50 | lateral passes the grid and both extensions |
  | `certified.band_top_speeds` | [0.10, 0.30, 0.50] | **[0.10, 0.30, 0.40]** | a band topped at 0.50 would size a field for a speed that cannot be commanded; the lower tops are unchanged, nothing measured moved them |
  | traversal speed, every stratum | 0.50 | **0.40** | the top band moved; the aisles did not |
  | narrow floor | 0.12 | **0.12** | lateral margin 0.05 + G1b's committed peak cross-track 0.05 + slack 0.02: the *threshold*, not a measured peak, so v_max does not enter it |
  | slack at the tracking offset, narrowest aisle | 19.3 mm | **19.3 mm** | the Minkowski field's sideways reach is e(+-90 deg) + lateral margin at every speed |
  | at-rest clearance floor | 0.1007 | **0.1007** | unchanged; still below the 0.12 floor |
- **What did not move, and why that is the point.** The field's sideways reach does not
  grow with speed, so lowering the top band changes the stopping depth *ahead* and
  nothing about how the robot fits an aisle. Every stratum still admits its top band
  centred and at the 0.05 m tracking offset.
- **Tests re-pinned:** `tests/test_d2_aisle_strata.py` (traversal 0.40, the slack and the
  real-scan check now read the top band from the config rather than the literal 0.50),
  `tests/test_footprint_config.py` (the box).
- **An assumption removed rather than worked around.** `tests/test_robot_kinematics.py`
  asserted `vy_max <= v_max`. Nothing in the kinematics needs that -- the clamp scales
  each axis against its own ceiling -- and the measurement now contradicts it. Replaced
  by the real invariant: each axis ceiling is positive and within the wheel budget's
  filter share. **The non-square box is for the supervisor to confirm:** squaring vy_max
  down to 0.40 would discard a measured capability, and the budget, not the box, is the
  binding constraint.
- **For WP2:** the failing cell is a rotating profile, and the mechanism is the recorded
  one -- the direction-preserving projection scales omega down with translation, so
  heading suffers while cross-track improves. **5.05 deg at 0.50 m/s is the baseline the
  polytope constraint has to beat.**

## 2026-09-20 (WP1) - G1b closed; consequences of the non-square box recorded
- **Closed by the supervisor:** v_max 0.40 and vy_max 0.50 are sanctioned, the non-square
  box is approved, and the removal of the `vy_max <= v_max` test assumption is approved.
- **Corner test added.** `tests/test_wheel_budget.py` now pins the box's own corner:
  (0.40, 0.50, 0) uses 0.90 m/s at the rim against a nominal share of 0.4936, so it is
  outside the budget and must be projected; the projection shortens it along its own
  direction (heading unchanged at 51.3 deg off the nose, all three axes scaled by one
  factor) rather than clipping an axis. A second case adds rotation and pins the turn
  ratio. The corner is reachable precisely because the box is non-square, so it is the
  case worth testing.
- **Two consequences recorded in `docs/requirements.md`:**
  * **WP1.** The policy's normalised action scales **each axis by its own ceiling** --
    a in [-1, 1]^3 -> (a0 v_max, a1 vy_max, a2 w_max) -- and the asymmetry is stated in
    the MDP section. One shared ceiling would let the same normalised magnitude ask for
    more lateral than forward motion, biasing exploration toward sideways travel for no
    physical reason.
  * **WP3.** Every baseline (holonomic PP, RPP, DWA) commands through the same
    non-square box and the same budget projection. A controller with more lateral
    authority than another is being compared on its actuation limits rather than its
    navigation, and the comparison would favour whichever commands the most lateral
    motion.
- **Write-up note:** the certified band edges are this platform's demonstrated limit, not
  a vehicle specification or a standard. `band_top_speeds` is bounded above by v_max 0.40,
  which is what G1b could demonstrate on this youBot under the wheel budget -- not a
  manufacturer's rating and not a figure from IEC 61496 or ISO 3691-4. The band
  *structure* follows the certified-layer pattern; the *numbers* are ours, so any claim
  about protective stops is a claim about this platform's envelope.
- **Display refusals are now counted in every run, not only gates.**
  `SimClient.display_status()` reports in force / refusals / consistent / **starts**, and
  `runio.stamp_display` merges it into `provenance.json`; wired into every script that
  drives the simulator and writes a run. `starts` is recorded beside the count so a
  refusal rate can be read against the number of chances to refuse. **Trigger, set now
  and not acted on: stop and diagnose if a refusal becomes fatal, or if the count rises
  with run length.** Added to the limitations list in `docs/writeup_notes.md` as known,
  unexplained simulator behaviour shown not to affect trajectories.
- **Next and only critical-path item: the WP1 environment.**

## 2026-09-21 (WP1) - Scan packet v2: the stamp is the state's time, not the pass's
- **Found by the tick check, on the first simulator run of the scanner**
  (`scripts/diag_scanner_check.py`, HEAD `0192d24`): "scan packet is +0.0500 s from the
  current step" -- exactly one control period, every time.
- **The scanner itself worked**: `lua/scanner_v2.lua` loaded through `sim.createScript`,
  initialised, ran, and published a packet that decoded cleanly against the contract. The
  only failure was the freshness check, which is the check doing its job.
- **Cause.** `sysCall_sensing` runs *after* the step's physics, but the simulator's clock
  during sensing still reads the time the step **started**. The scan therefore described
  the state at t + dt while carrying the stamp t, and the client -- reading after
  `sim.step()` returns -- was already at t + dt.
- **Fix: the packet carries the time of the state it describes**, so the Lua stamps
  `sim.getSimulationTime() + sim.getSimulationTimeStep()`. Loosening the tolerance was the
  alternative and was rejected: it would have hidden a genuinely stale packet, which is
  the thing the check exists to catch. **FORMAT_VERSION 1 -> 2**, because the meaning of a
  field changed even though the layout did not; nothing had consumed v1.
- **A mismatch the run caught late is now caught by the fast tests.** Bumping Python's
  constant while leaving the Lua's at 1 would have failed only once an instance was
  running; `tests/test_scanner_client.py` now asserts that the Lua declares the same
  FORMAT_VERSION, uses the same three signal names, and stamps the state time.

## 2026-09-21 (WP1) - First scanner check: the scan runs but its geometry is wrong; four fixes
- **Run:** `runs/diag/SCANCHECK_20260921T144640Z_seed0`, commit `68442d0`, clean, display
  off with no refusals.
- **Working:** `lua/scanner_v2.lua` loads, initialises, publishes every tick, covers all
  36 and 120 bins, and costs ~4.5 ms per step to bin.
- **Failing:** a wall 1.0 m to the left came back on a straight line tilted ~60 deg from
  where it is (`range x sin(bearing)` 0.16 m at 13 deg, 2.21 m at 49 deg, against 1.0).
  The "depth is euclidean" alternative did not fit either (0.84 m worst), so this was not
  the depth question anticipated beforehand.
- **Leading hypothesis, not yet a finding: mirrored pixel order.** Reflecting points across
  a sensor's axis turns a line at angle a into one at 2 x axis - a, and the sensors' axes
  are at 30 deg -- so a 0 deg wall reads as 60 deg. `tests/test_depth_geometry.py`
  reproduces the signature exactly on a synthetic sensor. It is carried in the
  configuration as `scanners.pixel_mirror: true`, VERIFY, and the next check settles it
  from the raw capture rather than by re-running until something fits.
- **What the scanner reported about the pods, measured:** four sensors, 120 deg each, axes
  at +30, -30, +150 and -150 deg. **Each pod covers 180 deg, not 240** -- its two sensors
  are 60 deg apart and overlap by 60 deg -- and **the two pods meet exactly abeam, at
  +-90 deg, with no overlap**. Abeam is where each sensor is at the edge of its field of
  view, and it is the direction the certified field's 0.05 m lateral margin faces; the
  next check scores it separately.
- **Four fixes, before the re-run:**
  1. **The convention is a parameter, identified from data.** `pfbnav/sim/depth_geometry.py`
     turns a depth row into points under any of the four conventions (mirrored or not;
     perpendicular or euclidean), with known-answer tests showing each true convention
     reconstructs a synthetic wall exactly and every wrong one misses by > 5 cm. The Lua
     uses `scanners.pixel_mirror` / `depth_is_euclidean` from the configuration.
  2. **The robot never sees itself.** Each sensor now renders everything except the whole
     robot tree. Without it the scan could report the chassis, the wheels or the other pod
     as obstacles 0.2-0.4 m away, past `range_min`. (v1 excluded only its own pod.)
  3. **The v1 LiDAR scripts are handled at run time, never in the scene.** Their state is
     read through the live `scriptDisabled` property; a live one is disabled, read back,
     and re-enabled on removal. A disable that does not take raises -- a legacy script left
     running would bin the same sensors again and rewrite their image.
  4. **The ray count came out 81, not 80.** The scene stores the 120 deg angle in single
     precision (120.0000033 deg) and the ceiling rounded the noise up. The count is now
     rounded to four decimals before the ceiling.
  Also: a return beyond `range_max` from the body origin is dropped rather than recorded.
  The far clipping plane is perpendicular to the axis, so off-axis returns can lie beyond
  it -- and the decoder correctly refuses a valid range above `range_max`.
- **The re-run** places three walls itself (left, front, behind), reconstructs the raw
  capture under all four conventions per wall, scores the published packet under the
  configured one, checks the wall-free baseline for returns inside the footprint, and
  reports abeam separately. Pre-declared acceptance: every wall within 10 mm, no
  self-hits. The robot is not moved and the scene is never saved.

## 2026-09-21 (WP1) - Scanner check 2: convention settled; bearings a quarter turn off; three corrections
- **Run:** `runs/diag/SCANCHECK_20260921T151356Z_seed0`, commit `152904a`, clean. The run
  started and ended on the pinned scene (its own before/after hash check passed).
- **The pixel convention is settled: mirrored order, perpendicular depth.** On every wall,
  from the raw capture: median residual ~10 mm for mirrored/perpendicular, against
  0.65-0.85 m for direct order and ~0.15 m for euclidean depth. `scanners.pixel_mirror`
  and `depth_is_euclidean` are no longer VERIFY.
- **But ~10 mm remains, and it is not explained.** The residual rises with depth with a
  zero intercept (fitted per sensor: +/-0.03 mm), and k x (the normal's component along
  the sensor axis) came out ~9.4 mm/m on every wall -- one systematic parameter, not
  noise. Fitting single causes to the raw capture did not find it: a depth scale 1.4 %
  long cut the RMS from 15.4 to 8.1 mm but left a 43 mm worst case; an offset, a tangent
  scale, a half-pixel shift and a field-of-view error did worse. **Recorded as open.**
  Its direction matters: a range read long places an obstacle further away than it is,
  which is the unsafe direction for the certified field and the PFB.
- **The worst cases were partly the check's fault:** pixels hitting the wall's 5 cm end
  faces read up to ~50 mm off the plane and set the maxima. The check now scores the
  wall's face only.
- **The scanner reported bearings in `youBot_ref`'s frame, not the body frame -- a
  quarter turn off.** `robot.yaw_offset` is pi/2 (youBot_ref's +x is a quarter turn from
  chassis-forward, as in v1) and the Lua never applied it. Every bearing was 90 deg off;
  the walls the check called left/front/behind were chassis front/right/left. Fixed: the
  Lua rotates every point, every coverage axis and every reported axis by -yaw_offset
  before taking bearings, refuses to start without a yaw_offset, and the client passes it.
  The plane relation itself was frame-consistent (walls and scan both in youBot_ref), so
  the convention result stands.
- **Correction to what I told the researcher about the pods.** I said each pod covers
  180 deg and the pods meet exactly abeam with no overlap. **Wrong -- I had paired the
  sensors by axis angle.** The measured sensor positions put the pods at youBot_ref
  y = +-0.325 m (chassis fore and aft), each carrying sensors 120 deg apart, so **each pod
  covers 240 deg, as the researcher said**, and in the body frame the pods overlap by
  60 deg on each side: **abeam (+-90 deg) is covered twice**, not at a seam. The forward
  pod at body x = +0.325 is consistent with `robot.half_length` 0.3520 being set by it.
- **Six legacy LiDAR scripts were found live and disabled** at run time through the
  `scriptDisabled` property, then restored. That settles the question the deprecated read
  got wrong: they were running.
- **No self-hits:** the wall-free baseline's nearest return was 5.0 m (range_max) with the
  robot excluded from rendering. Coverage 36/36 and 120/120.
- **The display was refused once, and the refusal coincided with the researcher zooming
  the scene view.** That is the first observation consistent with the untested GUI-
  interaction hypothesis (limitations, 2026-09-20). One coincidence, not a diagnosis. The
  step timings of this run are not comparable (display state changed mid-run) and are not
  used.
- **The scene file was changed after the run**, at 16:16 against the run's 16:14 finish:
  the camera zoom was saved into `scenes/warehouse_v2.ttt` (SHA-256 9f08660f...). The next
  run refused it, as the pinning is meant to. Restored from git to the pinned 340dbc5d...,
  with the modified file kept in the session scratchpad. No scene edit was intended and
  none was committed (hard rule 12).
- **Next check (this commit):** walls in the body frame on all four sides, each placed
  **after** the robot has settled with its pose recorded at placement and capture, at
  0.6, 1.0 and 2.0 m so a scale separates from an offset, scored on the wall's face only.

## 2026-09-21 (WP1) - Scanner check 3 crashed in cleanup: walls placed mid-run vanish at stop
- **What happened.** The check now places each wall after the robot has settled, i.e.
  while the simulation runs. CoppeliaSim deletes objects created during a run when the run
  stops, so `removeObjects([wall])` after `stop()` failed with "found invalid handles"; the
  cleanup block then tried the same removal, failed again, and skipped the rest -- the
  scanner script was left installed and the legacy LiDAR scripts disabled **in memory**.
  The scene file was never touched (nothing saves it); the next run reloads it from disk.
- **Void:** the partial run directory from this attempt holds only `config.yaml` and
  `provenance.json` -- no summary, nothing to cite.
- **Fix.** Walls are removed while the simulation still runs, and cleanup goes through
  `remove_wall`, which checks `sim.isHandle` first, so a wall the simulator already
  deleted cannot abort the rest of the cleanup. Tested on a fake that raises exactly the
  simulator's error for an invalid handle.
- **Also recorded, same day:** the attempt before this one failed while importing scipy
  with "the paging file is too small": the laptop (7.7 GB) was out of virtual memory with
  several GUI instances open. Not a code fault; closing the extra instances is the remedy.

## 2026-09-21 (WP1) - Scanner check 3: frame and convention confirmed; readings ~1 % long, FAIL
- **Run:** `runs/diag/SCANCHECK_20260921T213302Z_seed0`, commit `0fd693f`, clean, pinned
  scene, display off with **0 refusals over 14 starts** (the GUI was not touched).
- **Pre-declared acceptance (10 mm on every wall at every distance): FAIL.** Only the side
  walls at 0.6 m pass (9.1 mm worst). Packet median / worst, body frame:

  | wall | 0.6 m | 1.0 m | 2.0 m | median vs distance |
  |---|---|---|---|---|
  | front | +8 / 19 mm | +16 / 33 mm | +33 / 63 mm | 17.8 mm/m, -2.4 mm |
  | left | +6 / 9 mm | +11 / 17 mm | +23 / 31 mm | 12.1 mm/m, -1.4 mm |
  | right | +6 / 9 mm | +11 / 17 mm | +23 / 31 mm | 12.1 mm/m, -1.3 mm |
  | behind | +8 / 19 mm | +16 / 33 mm | +33 / 63 mm | 17.8 mm/m, -2.4 mm |

- **Settled:** the body-frame fix (sensor axes now +-60 and +-120 deg; front = behind and
  left = right to 0.1 mm, as a symmetric robot must give); the convention, again on all 12
  placements; settling ruled out (robot drift 0.0 mm between placement and capture); no
  self-hits; full coverage. The scanner costs ~6 ms per step (43.4 against 37.4 ms, with
  the display steady -- a valid timing this time).
- **Not settled: ranges read ~1 % long, from the sensor.** Median ~10 mm per metre of range
  from the sensor at every off-axis angle, larger at oblique incidence. **Not a simple
  parameter:** fitted to the raw capture, a depth scale leaves 9.4 mm RMS / 44 mm worst;
  scale + field of view + offset together still 9.2 / 42 mm.
- **Why it matters:** the error is always in the unsafe direction (an obstacle reads
  further away than it is). Where the certified field's boundary sits -- ~0.26 m abeam
  (0.05 m lateral margin), ~0.57 m ahead at 0.40 m/s -- it is ~3-6 mm typical and up to
  19 mm ahead: a material share of a 50 mm margin.
- **Next: one experiment before any correction** (`scripts/diag_scanner_resolution.py`):
  the front wall at 1.0 and 2.0 m under resolution 80 / 256 / 512 px x near clipping
  0.01 / 0.05 / 0.10 m. **Rule, set before the run:** the median error per metre halving
  or better (and moving > 1 mm/m) with resolution -> pixel sampling, adopt a resolution;
  with near clipping -> depth conversion, adopt that setting; with neither -> fall back to
  a conservative range correction, which is the supervisor's decision because it changes
  what the safety layers see. Either adoption is then confirmed by the full multi-wall
  check.

## 2026-09-21 (WP1) - Resolution experiment, first attempt: the near plane is not stored as given
- The experiment stopped at its first setting: "sensor 93: near clipping 0.01 did not
  take (reads 0.008999999612569809)". CoppeliaSim accepted the write and stored a
  different value -- not single-precision noise (0.01 in float32 is 0.0099999998). The
  read-back caught it, which is what hard rule 13 requires.
- **Changed:** the exact value is not what the experiment needs; three *different* near
  planes are. The script now reads the plane back **after the simulation starts** (the
  Lua sets the far plane at start, and the near plane may move with it), records the value
  in force beside the value requested, refuses sensors that disagree, and refuses a run in
  which the planes in force do not actually differ -- the lever must move for the result to
  mean anything. The analysis is recorded against the values in force.

## 2026-09-21 (WP1) - Resolution experiment: the range error is pixel sampling; 512 rays adopted
- **Run:** `runs/diag/SCANRES_20260921T214435Z_seed0`, commit `7c9253b`. **The script aborted before writing its
  summary** -- it treated an untestable near-clipping lever as fatal and discarded the
  resolution data with it. **The record of this run is the researcher's console output**,
  transcribed here; the run directory holds only config and provenance. Fixed in this
  commit: the data is always saved and near clipping is reported untested.
- **Result, front wall (the worst case) at 1.0 and 2.0 m pooled:**

  | rays per sensor | median | per metre from sensor | p95 | worst | step |
  |---|---|---|---|---|---|
  | 80 | +20.1 mm | +14.2 mm/m | 50.3 mm | 62.9 mm | 40-42 ms |
  | 256 | +6.2 mm | +4.4 mm/m | 15.1 mm | 19.8 mm | 39-47 ms |
  | 512 | +3.1 mm | +2.2 mm/m | 7.5 mm | 9.8 mm | 51-54 ms |

- **Cause, by the pre-declared rule: pixel sampling.** The error falls in exact proportion
  to pixel width -- 3.2x finer (80 -> 256) cut it 3.2x, 2x finer (256 -> 512) cut it 2.0x.
  Each pixel's depth corresponds to a point slightly off its centre on a slanted surface,
  always on the far side. It is not a depth-conversion error.
- **Near clipping: untested.** CoppeliaSim held the near plane at 0.009 m for every value
  requested (0.01, 0.05, 0.10), so the lever never moved; the script's check said so. The
  three settings per resolution were therefore repeats, and they agree to 0.01 mm.
- **Adopted: `scanners.rays_per_sensor: 512`** -- the lowest tested count whose worst case
  is within the scanner check's 10 mm (9.8 mm) -- with its derivation in the config (the
  standing rule on numeric requirements) and VERIFY until the multi-wall check passes at
  512. What the bins need (80, two rays per bin) becomes a floor the client enforces.
- **Correction to a finding of 2026-09-20.** "Scanner cost is per sensor, not per ray" held
  for **rendering** only. With binning, 512 rays cost ~10 ms more a step than 80 on this
  laptop, because the Lua transformed one pixel per call. It now builds each sensor's
  returns into one list and transforms them with a single `sim.multiplyVector` -- the same
  arithmetic, one call per sensor. The step cost at 512 is re-measured by the confirming
  check.
- **What 512 rays means at the distances that matter:** at ~2.2 mm/m, the error at the
  certified field's boundary (~0.26 m abeam, ~0.57 m ahead) is ~1 mm -- against ~4-8 mm at
  80 rays. Still in the unsafe direction, now small beside the 50 mm margins.

## 2026-09-21 (WP1) - The scanners leave a blind zone beside the robot; the layout goes to the supervisor
- **Finding (geometry, from the scanner's own reported pose):** the two 240 deg Hokuyo pods
  sit on the centreline at body x = +-0.325 m. Each sees 120 deg either side of its own
  facing, so the front pod's rearmost ray and the rear pod's foremost ray meet only
  **0.563 m** from the centreline at mid-body. **Anything nearer, beside the robot, is seen
  by neither pod** -- checked point by point: at x = 0, lateral 0.21 / 0.26 / 0.30 / 0.40 /
  0.50 / 0.56 m are invisible to both; 0.57 m and beyond, both see it. Blind area outside
  the chassis 0.176 m^2, reaching 0.38 m beyond it.
- **It sits on the certified field's boundary.** The field reaches e(+-90) + 0.05 = 0.26 m
  abeam; the blind zone spans 0.21-0.56 m there. With holonomic motion the robot can move
  sideways straight into it.
- **The scanner reports that space as observed -- a hard rule 5 violation in my code.**
  `lua/scanner_v2.lua` marks a bin valid when the bin's bearing from the body origin lies
  inside a sensor's field of view, i.e. as if every sensor sat at the origin. The sensors
  are 0.325 m from it, so abeam bins read free space where nothing was measured.
- **Why every check missed it:** each test wall was at >= 0.6 m, just beyond 0.563 m.
- **Correction:** I told the researcher abeam is "covered twice". True only beyond 0.56 m;
  closer in it is covered by neither pod. And the scan excludes the robot from rendering,
  so real occlusion by the chassis -- which would only enlarge the gap -- is not modelled.
- **Layouts compared (exact geometry, blind area outside the chassis / furthest reach):**
  centreline 240 deg (now) 0.176 m^2 / 0.38 m; diagonal corners 240 deg 0.163 m^2 /
  0.24 m (the gap moves to the other two corners); **diagonal corners 270 deg: none.** A
  rectangle's corner faces 270 deg of open space, so two opposite-corner scanners cover
  everything outside it only if each sees >= 270 deg; no mounting of a 240 deg unit can.
  CoppeliaSim ships a 270 deg model ("SICK S300 Fast"), a real certified safety scanner.
- **Not decided here.** Changing the layout is a scene edit with a re-measurement chain
  (footprint probe and table -- the forward pod sets `half_length` 0.352 today -- lethal
  and soft radii, certified field, D2 strata and traversal speeds, scanner check). The
  validity bug is fixed on any layout; on the current one, fixing it makes the gap visible
  as a permanent breach abeam rather than hiding it. Both go to the supervisor.

## 2026-09-21 (WP1) - Supervisor's decision: S300-type layout; position-aware validity lands first
- **Decision (supervisor):** option (a), two 270 deg scanners of the SICK S300 type on
  diagonal corners -- implementing the v2 plan's section 2.2 design; the centreline pods were
  the inherited platform. **In the write-up: "an S300-type layout", never that the system
  uses certified sensing.**
- **Conditions:** (1) flush with the chassis corners, minimal protrusion; (2) model
  self-occlusion -- each scanner sees the robot, self-returns filtered with the footprint
  table from that sensor's pose; scan-plane height recorded, with whether it clears the
  wheel tops, and the low obstacles below it listed as a limitation; (3) a coverage-map
  test -- a ~20 mm post swept densely from the footprint edge to 1 m at scan-plane height,
  every cell observed or flagged invalid, never free without observation, on both
  layouts, the before/after maps a paper figure; (4) a pre-declared physics regression --
  G1b's binding cells (translate_while_rotating at 0.40 and 0.50, lateral at 0.50), three
  repetitions, any verdict change -> full G1b re-run; the S300 model mass recorded as found.
- **Order:** validity fix + coverage map on the current layout; the scene edit committed
  alone with its hash; footprint probe; table, radii, field, D2 strata and traversal speeds
  re-derived in one commit; coverage map on the new layout; G0 (3 reps); the G1b
  regression; then the environment.
- **The researcher had already made the scene edit** (S300 Fast on the corners) when the
  decision arrived. To keep the order, the edited scene (SHA-256 8c772a64...) is held in
  `scenes/pending/` (git-ignored: out of every run and every commit) with a copy in the
  session scratchpad, and the pinned scene is back in place for the "before" map. The file
  changed once while being captured (1232c060... then 8c772a64...): CoppeliaSim saved again
  between two reads. The researcher is to save any final version to the pending path, not
  over `scenes/warehouse_v2.ttt`.

## 2026-09-21 (WP1) - Position-aware validity; the robot rendered and self-filtered
- **Validity** now comes from `pfbnav/sim/coverage.py`: a bin is valid only if its whole
  sector, from the footprint edge to `scanners.coverage_depth` (1.0 m) beyond it, is seen
  by some sensor from where that sensor is -- field of view, range, and occlusion by the
  footprint (a sight line that has left the robot and re-enters it is blocked; counted as
  polygon crossings, checked against a brute-force march). Computed once at installation
  from the sensors' real poses and sent to the Lua as per-bin masks; the Lua's angle-only
  `markCoverage` is gone. Known answers: the inherited layout loses the abeam bins and keeps
  the axial ones, with a blind reach of 0.563 - 0.179 m; diagonal 270 deg sensors lose
  nothing; diagonal 240 deg sensors leave the other two corners blind.
- **The robot is rendered** (entity to render: everything), so it occludes as the renderer
  decides, and **returns inside the footprint + `self_filter_margin` (5 mm) are dropped** as
  the robot's own. The footprint travels to the Lua sampled per degree.
- **The planar occlusion model over-states occlusion** -- the footprint includes parts below
  and above the scan plane -- so the mask errs towards invalid. Which parts are below the
  plane is settled when the scan-plane height is measured on the new layout.
- **Ray count by density:** `scanners.rays_per_degree` 4.2666667 (512 over 120 deg) replaces
  `rays_per_sensor`, because the measured error scales with pixel width; a 135 deg S300
  sensor gets 576. Per sensor, floor-checked against the bins.
- **Scanner-model scripts are found by ancestry, not by name** -- the model each sensor
  belongs to is its ancestor just below the robot root -- so the S300 model's own scripts
  are disabled at run time exactly as the Hokuyo pods' were.

## 2026-09-22 (WP1) - Layout read-back, G1b regression comparator, dense-sweep rule
- **`scripts/probe_scanner_layout.py`** reads a candidate scene (default: the S300 layout in
  `scenes/pending/`) without saving it, and records what the conditions need: each sensor's
  body-frame pose, field of view and ray count as found; each scanner model's body-frame
  bounding box and its protrusion beyond the chassis (condition 1); the scan-plane height
  against the wheel tops and the chassis top (condition 2); each model's mass as found
  (condition 4); the model's scripts; whether the inherited `LiDAR` pods are still present.
  Provenance names the candidate and its SHA-256, not the pinned scene, so the record says
  what was read. This is the hard-rule-12 read-back before the scene edit is committed.
- **`scripts/g1b_regression.py`** compares the binding cells (translate_while_rotating at
  0.40 and 0.50, lateral at 0.50) from three narrowed invocations with the closed spread
  `G1b-SPREAD_20260920T215108Z`. Verdict is PASS only if every run is within every limit, so
  fail <-> straddle is not a change but pass <-> anything else is. **Any change -> full
  G1b re-run** (pre-declared). It refuses invocations that disagree in commit, dirty state
  or scene, criteria or physics that differ from the reference, and a scene equal to the
  reference's (then there is nothing to regress). Known-answer tests in
  `tests/test_tracking_spread.py`.
- **Standing rule (hard rule 14):** coverage and boundary tests sweep geometry densely
  across the region that matters, not at convenient distances.
- **Write-up wording:** "an S300-type layout", never certified sensing (`writeup_notes.md`,
  Limitations), with the below-the-plane limitation to be filled from the read-back.

## 2026-09-22 (WP1) - Defect: every sensor blinded by its own casing (d7da848); fixed
- **Found by the dense coverage map** (`COVMAP-centreline_20260922T161234Z`, commit d97a693,
  pinned scene): 0 observed, 4,139 filter / 3,715 policy violations out of 7,852 cells --
  every bin read range_max at every cell. FAILED. The validity masks were right (abeam
  invalid, axial valid); the ranges were empty. Kept on record, not deleted.
- **Diagnosis** (`scripts/diag_self_occlusion.py`, `SELFOCC_20260922T163041Z`, commit
  b56bfca): with the robot rendered, all 512 pixels of each of the four sensors read
  0.0094 m -- the pod's own casing, just past the 0.009 m near plane -- so every return fell
  inside the footprint and the self-filter dropped it. Rendering everything except the
  sensor's own model, each sensor saw a 20 mm post 0.4 m out on its axis (near face read
  0.390 m, 8 pixels kept); the post is rendered (renderable flag set, seen with the whole
  robot excluded). H1 (housing) confirmed, H2 (post not rendered) rejected. On this layout
  nothing else of the robot is in any sensor's view (own-model-excluded and robot-excluded
  renders identical); on the S300 corners the chassis may be, which is what the footprint
  filter is for.
- **Cause:** d7da848 set `entity_to_render = -1` (render everything) to model
  self-occlusion; no check after it put an obstacle near the robot through the Lua path.
- **Fix:** each sensor renders a collection of everything except its own scanner model (its
  ancestor just below the robot root, as `Scanner.scanner_models` finds it), confirmed by
  read-back; the chassis and the other scanners stay rendered, and the footprint filter
  stays. Reported in the status packet (`render`, `own_models`). A contract test forbids
  `entity_to_render, -1`.
- **Verification:** the coverage map on the current layout, re-run (the "before" map).
- **Verified 2026-09-22** (`COVMAP-centreline_20260922T163600Z`, commit ce2306d, pinned
  scene, clean): **PASS, 0 violations** on both streams; filter 4,139 observed / 3,713
  invalid, policy 3,715 / 4,137. Observed readings sit 6-10 mm short of the post's axis
  (its radius is 10 mm). Of the invalid cells, 3,499 (filter) still read the post: bins are
  invalid when any part of their sector is unseen, so the flag is conservative. The 214
  truly unseen cells (0.134 m2 at 25 mm spacing) match the geometric blind area (0.146 m2)
  and its reach to ~0.56 m abeam. This is the "before" figure.

## 2026-09-22 (WP1) - Scene: the S300-type layout replaces the centreline pods
- **The scene edit is the researcher's** (supervisor decision (a), 2026-09-21): the two
  inherited Hokuyo pods are gone; two `SickS300` models sit on diagonal corners, each with
  two 135 deg sensors, so 270 deg per scanner. Pinned scene SHA-256
  0ae35a50290d097857bb3cd828858f70a736ae0fa2102f0fac1e69b452164790 replaces 340dbc5d...,
  and `sim.scene_sha256` moves in this commit (hard rule 12). The previous scene stays in
  git history at 4af47c1.
- **Read back before committing** (`scripts/probe_scanner_layout.py`,
  `LAYOUT_20260922T170201Z`, commit 96a5e58, on the candidate in `scenes/pending/`):
  - **Sensors:** front-right model at (+0.226, -0.147) m covering bearings -180..+90 deg,
    rear-left at (-0.226, +0.145) m covering 0..270 deg; four sensors, 135 deg each, 256
    rays as found (the Lua sets 576 at the configured density). No `LiDAR` object left.
  - **Condition 1, flush mounting:** each model reaches 12.4 / 12.3 mm beyond the chassis
    ends and 39.3 / 37.1 mm beyond its sides (oriented bounding boxes, so an upper bound on
    a rounded shape) -- about 10 mm beyond the wheels laterally. The front unit was moved
    back 20 mm on this reading, to match the rear (0.2464 -> 0.2264 m); the read-back was
    repeated on the moved scene. The real outline is the footprint probe's business.
  - **Condition 2, scan plane:** z = 0.2207 m, clearing the wheel tops (0.0999 m) and the
    chassis top (0.1764 m). Nothing of the robot but the other S300 (z 0.104-0.257 m) lies
    in the plane, so the footprint occlusion in the mask is conservative here. Obstacles
    below 0.221 m are unseen: a limitation, in `writeup_notes.md`.
  - **Condition 4, mass as found:** 17.0 kg per model, but each shape is **static and not
    respondable** (property and int-parameter reads agree, hard rule 13), i.e. carried
    kinematically by the chassis and contributing no mass or inertia. The 17 kg is recorded
    as found; it is not edited, and no mass change enters the physics regression. The shapes
    are collidable, measurable, detectable and renderable, so contact checks and the
    footprint probe include them.
  - Three scripts per model, all live as found; the scanner disables them at run time by
    ancestry and restores them afterwards.
- **Known stale until the re-derivation commit:** the footprint table still describes the
  pod layout, so `test_the_table_was_built_from_the_configured_scene` FAILS on purpose from
  this commit until `scripts/probe_footprint.py` re-measures and the table, radii, field,
  D2 strata and traversal speeds are re-derived in one commit.

## 2026-09-22 (WP1) - Re-derivation on the S300 footprint (one commit)
Source: `scripts/probe_footprint.py` on the pinned S300 scene (0ae35a50),
`FOOTPRINT_20260922T170810Z_seed0`, commit 09aed30, clean, ODE, 2 deg bearing step,
0.0005 m tolerance; table rebuilt by `scripts/build_footprint_table.py`.

- **The scanners no longer set the envelope.** Each S300 reaches 0.3205 / 0.3193 m at
  +-30 deg from the body origin, *inside* the wheel corners (0.3372 m), and its sideways
  reach stays inside the wheels. The envelope is the chassis fore and aft and the wheels
  sideways and at the corners.
- **half_length 0.3520 -> 0.2854 m** (probe front 0.28534, rear 0.28514, rounded up as the
  measurement bracket's upper bound): 67 mm shorter at each end, and no longer set by a
  sensor pod.
- **half_width 0.2086 m, unchanged** (probe left 0.20856, right 0.20850): the same hidden
  wheel spheres.
- **Footprint table** rebuilt: inscribed 0.20945 (was 0.20925), circumscribed 0.33742 (was
  0.35185), spline margin 0.95 mm. The robot is shorter and a fifth of a millimetre wider.
- **Planner radii** (derived, not stored): lethal 0.2386 unchanged; soft
  0.33742 + 0.03 = 0.3674, was 0.3819 -- the soft band shrinks by 14.5 mm.
- **D2:** at-rest clearance floor 0.10091 m (was 0.1007), still strictly below the narrow
  floor 0.12, so the strata stand: narrow 0.12-0.20, medium 0.20-0.40, wide 0.40-0.80 m per
  side. Slack at the tracking offset in the narrowest aisle 0.0191 m (was 0.0193).
- **Traversal speeds unchanged**: 0.40 m/s in every stratum, each capped by v_max rather
  than by the field fitting, and the usable band top is 0.40 everywhere.
- **The certified bands are untouched** (band_top_speeds 0.10 / 0.30 / 0.40): they are set
  by v_max, which the layout does not change.
- **Recorded numbers updated in the same commit** (`tests/test_d2_aisle_strata.py`,
  `tests/test_footprint_config.py`), and `tests/test_scanner_client.py`'s fake sim now
  carries the S300 poses: with two 270 deg scanners on diagonal corners **no bin is
  unobserved and the geometric blind area is 0.0**. The inherited pods' blind zone stays
  as a fixture there and as pure geometry in `tests/test_coverage.py`, so the report can
  still be shown to say "unobserved" when there is something unobserved.
- **Limitation recorded** (`docs/writeup_notes.md`): the scan plane is at 0.221 m; obstacles
  below it are unseen by the safety layers, and appear as collisions with no protective
  stop.

## 2026-09-22 (WP1) - S300 coverage map: no blind zone; a bin-edge defect in the classifier
- **`COVMAP-s300_corners_20260922T212716Z`** (commit 75142e9, pinned S300 scene, clean,
  display off, 0 refusals, 7,680 cells, 193 s): **0 invalid cells on both streams** -- the
  geometric blind area is 0.0 and every bin is valid, which is what the layout change was
  for -- and 7,678 observed on the filter stream. The status packet confirms 4 sensors of
  135 deg at 576 rays each, rendering everything except their own model.
- **Two filter cells scored "violation", and the classifier was wrong, not the scan.**
  Both sat within 0.01 deg of a bin edge ((-0.0124, +0.2376) at 92.9928 deg, bin edge at
  93; (+0.0376, -0.2374) at -81.0058 deg, edge at -81), and in both the **policy stream
  saw the post at 0.228 m**. A 20 mm post at 0.24 m subtends +-2.39 deg against 3 deg
  filter bins, so its visible near face fell in the bin next to the one holding its axis;
  asking only the axis bin scored the scan as reporting free space where it had in fact
  reported the post.
- **Fix:** `bins_spanned` (pure, tested) returns every bin the post's angular extent
  overlaps, the axis bin first, wrapping at the seam and including the bins between the
  edges; `classify` now takes one flag, range and baseline per spanned bin -- observed if
  any valid spanned bin reports the post, invalid if all are flagged unobserved, confounded
  if none can tell, violation otherwise. `diag_coverage_map.py` records the spanned bins
  and their ranges for every violation, so a future one is diagnosable from the file alone.
- The run stands as recorded (FAILED under the classifier of the day). The S300 map is
  re-run under the fixed classifier before the "after" figure is used.
- **Environment note:** the fast suite failed sporadically with `numpy MemoryError` while
  CoppeliaSim was open (1.3 GB free of 7.7 GB). Not a code failure; the suite is green with
  memory free. Worth remembering when a run and the suite are asked for at once.
- **Re-run under the fixed classifier** (`COVMAP-s300_corners_20260922T214324Z`, commit
  a37b9d9, pinned scene, clean, display off, 196 s): **PASS -- 7,680 / 7,680 observed on
  both streams, 0 invalid, 0 violations.** Condition 3 is met on the S300 layout, and the
  before/after pair is the paper figure: 3,713 invalid and 214 truly blind cells
  (0.134 m2 measured, 0.146 m2 geometric, 60/120 filter bins valid) with the centreline
  pods, against nothing unobserved and 120/120 valid on the corners.

## 2026-09-22 (WP1) - `configs/gates/g0_rerun.yaml` superseded
- The overlay pinned `half_width: 0.2085` because G0 ran before D8 wrote the measured value
  into `base.yaml`. D8 has since written **0.2086** (the probe's upper bound, re-confirmed
  on the S300 scene, FOOTPRINT_20260922T170810Z: left 0.20856), so the overlay is now 0.1 mm
  **below** base: it under-states the robot, the unsafe direction, and the opposite of what
  it was written for.
- Marked superseded in the file itself and in `scripts/gate_g0_contact.py`'s usage line,
  which now names `configs/base.yaml`. The file is kept, not deleted: the WP0 runs
  (G0_20260918*) cite it, and a run's config must stay readable.

## 2026-09-22 (WP1) - G0 case placement on the S300 layout (supervisor ruling)
Ruled 2026-09-22: run G0 as preregistered (option (a)), with the placement fixed; nothing
in `preregistration/` changes.

- **Placement by the envelope, not by "the forward-most shape".** That phrase stopped being
  well defined when the leading part became bearing-dependent: on this robot the chassis
  leads dead ahead (support 0.2852 m at 0 deg against the front scanner's 0.2767), the
  front S300 leads from about -10 to -45 deg (0.2983 at -10, 0.3205 at -30), and a wheel
  leads beyond that. `outermost_bearing` takes the argmax of the footprint table within
  +-90 deg of straight on, and the corner post is placed at that point.
- **The magnitude bug.** `chassis_extent_in_body` returns half-extents, so the old
  placement mirrored the leading part's lateral offset: with the centreline pods that was
  harmless, but on the S300 layout it would have put the corner post on the front-LEFT
  corner while the housing sits front-RIGHT, and the case would have passed while striking
  the wrong part. `first_to_strike` now takes the obstacle as a box and the drive as an
  axis, keeps only the shapes that overlap the obstacle across the drive, and returns the
  one reaching furthest along it.
- **Every positive case now asserts the shape it strikes.** The expectation is measured
  from the scene, not written down; `summarise_case` fails the case when the struck shape
  is not the one it was placed for, whatever the detection lag. "A case that contacts the
  wrong part while passing is worse than one that errors" (supervisor).
- **`--dry-run`** measures the robot, writes `placement.json` (each case's obstacle box,
  drive direction and the shape it must strike) and stops without driving, so the coverage
  question below can be answered before the gate runs.
- Known-answer tests on synthetic **asymmetric** geometry, `tests/test_g0_placement.py`
  (15): a housing on one side only, at one height; the mirrored post strikes something
  else; a low slab can only be struck by a wheel; a diagonal drive and an unreachable
  obstacle are refused; a case striking another shape fails with a perfect lag.
- **Hard rule 15** (supervisor): gate overlays never pin measured constants.
- **Open: coverage of the four part classes (the supervisor's condition 4).** From the
  probe's per-shape supports at 10 deg steps, the S300 housings **never lead** at any
  bearing: the wheel meshes lead everywhere except dead ahead and dead astern, where the
  chassis platform leads, and the housings' best (0.3205 m at -30 deg) sits 15 mm inside
  the wheel corner (0.3357). No floor-standing obstacle can therefore strike a housing
  first. Reported to the supervisor with the numbers; adding a case is an amendment and
  needs their ruling.

## 2026-09-23 (WP1) - G0 placement measured on the scene; housing coverage waived
- **Bounding boxes were not good enough, and said so loudly.** The first dry run put
  `wheel_respondable_fl` in front of everything: its bounding box reaches 0.3133 m forward
  and 0.2437 m sideways, while the contact probe measures that same wheel at 0.2085 m
  sideways (FOOTPRINT_20260922T170810Z). Expectations now come from each shape's **mesh
  vertices**, densified to 10 mm (`densify`), because a primitive's mesh can be as coarse
  as eight corners and the corner post's corridor is only 56 mm across.
- **The corner case is approached along its own bearing.** Placed at the envelope's
  outermost forward point (0.3374 m at -35.0 deg) but driven into along +x, the post is met
  by the chassis at 0.2854 m, not by the corner: along +x the chassis leads in that
  corridor. Driving along the outermost bearing, the wheel corner leads it by 1.2 mm
  (0.3375 against the chassis corner's 0.3363). The robot is holonomic, so the approach is
  a translation at the committed speed; `first_to_strike` now accepts any horizontal drive
  and tests the corridor the obstacle sweeps out.
- **Measured placement** (`--dry-run`, `G0_20260923T000934Z`, pinned scene, commit cf3ce33):

  | case | strikes | class |
  | --- | --- | --- |
  | front_wall_head_on | `ME_Platfo2_sub1/Rectangle` | chassis |
  | rear_wall | `youBot` | chassis |
  | side_wall_lateral | `wheel_respondable_rl` | wheel sphere |
  | pod_corner | `swedishWheel_fr` | wheel mesh |
  | wheel_edge | `swedishWheel_fl` | wheel mesh |

  `side_wall_lateral` strikes the wheel sphere at 0.2085 m, which is what set
  `robot.half_width` (0.2086 is that measurement rounded up), so the re-measurement did not
  move this case.
- **Housing coverage waived.** The supervisor's condition 4 said to stop if a part class is
  uncovered: the S300 housings never lead, sitting about 15 mm inside the wheel corners
  everywhere, so no floor-standing obstacle can strike one first (an obstacle existing only
  above 0.10 m would, as the housings span 0.104-0.257 m and the wheels end at 0.100 m).
  The researcher waived the stop on 2026-09-23 and G0 runs without a housing case. Recorded
  here and in the run's report; adding such a case remains an amendment for the supervisor.
- **Names are historical labels.** `pod_corner` no longer strikes a pod, and the wall cases
  strike the chassis rather than a housing. The preregistered names stand unchanged
  (`preregistration/` is never edited); what each case probes is recorded with the results.
- **G0 re-run on the S300 scene: every case passes in `gate_script`; overall INCOMPLETE.**
  Three invocations (`G0_20260923T001328Z_seed0`, `...T001729Z_seed1`, `...T002143Z_seed2`),
  commit 4b7cbfd, clean, pinned scene 0ae35a50, ODE, display off with 0 refusals.
  All five positive cases fire with **detection lag 0 steps** (limit 1) at the step the
  geometric gap reaches zero, with no contact on the step before; the negative case records
  **0 false contacts** at a minimum gap of 0.01008-0.01011 m (held 10 mm off the envelope);
  the handle-versus-collection validation **agrees on every step** of every case, 37-222
  steps each. The verdict stays INCOMPLETE because `gates.yaml` requires two contexts and
  `env_step_loop` needs the environment (WP1). That is unchanged and is not a failure.
- **What each case struck, as the placement predicted** (recorded per run in
  `placement.json`): front and rear walls the chassis (`ME_Platfo2_sub1/Rectangle`,
  `youBot`), the side wall the rear-left wheel sphere, the corner post the front-right
  wheel mesh, the low slab the front-left wheel mesh. Every case struck the shape it was
  placed for; the assertion added on 2026-09-22 passed on all three invocations.
- **Several shapes touch on the same step in three cases, which the case geometry implies:**
  a 1.2 m wall abeam meets both left wheel spheres at once, the 1.2 m low slab meets the
  front-left and rear-left wheels at once, and the front wall meets three nested chassis
  shapes at once. Only `pod_corner` isolates a single shape. Worth knowing when reading a
  `part` field: it names one struck shape, not the only one.
- **Housing coverage: still none, by the researcher's waiver of 2026-09-23.** The four part
  classes the supervisor listed are covered as chassis (2 cases), wheel sphere (1), wheel
  mesh (2), sensor housing (0). Adding a raised-obstacle case for the housings remains an
  amendment for the supervisor.
- **G1b physics regression after the scene change: no verdict changed; G1b stands.**
  Three narrowed invocations (`G1b_20260923T003812Z_seed0`, `...T003828Z_seed1`,
  `...T003838Z_seed2`, commit ad12e54, clean, scene 0ae35a50, PARTIAL by design) compared
  with the closed spread `G1b-SPREAD_20260920T215108Z` (scene 340dbc5d) by
  `scripts/g1b_regression.py`: translate_while_rotating@0.40 PASS -> PASS,
  translate_while_rotating@0.50 FAIL -> FAIL, lateral@0.50 PASS -> PASS. Repetitions were
  identical within each invocation. **No full re-run is triggered.**
- **The numbers barely moved, as predicted from the S300 shapes being static and
  non-respondable** (no mass or inertia added). Worst-case metric ranges, before -> after:
  twr@0.40 heading 2.3587-2.3615 -> 2.3630 deg, peak cross-track 0.0109-0.0110 -> 0.0109 m;
  twr@0.50 heading 5.0333-5.0523 -> 5.0479 deg (limit 5, still failing by 0.048);
  lateral@0.50 heading 0.2916-0.2976 -> 0.2934 deg. The spread across the three new
  invocations is zero to four decimal places, narrower than the old run's, so the
  0.50 m/s failure is if anything better established than before.
- **Carried forward:** `configs/gates/g1b_rerun.yaml` now only repeats base's box and falls
  under hard rule 15. It is kept until the runs citing it are no longer live, then retired
  the way `g0_rerun.yaml` was.
  - **Withdrawn 2026-09-26, before the WP1 item 4 re-run.** The premise was wrong: D8 had
    already written v_max 0.40 into base.yaml, so the overlay's 0.50 does *not* repeat
    base's box, and `gate_g1b_tracking.py` refuses to start when the box is below the
    tested grid. Nor is it the thing hard rule 15 forbids: 0.50 is not a measured constant
    but a measurement ceiling placed at or above every speed the gate tests, so that the
    clamp cannot decide the result (A6). Retiring it would have broken the gate. It stays,
    and the note is corrected here rather than deleted so the reasoning stays on the
    record.

## 2026-09-23 (WP1 item 1) - D2 against the real scan packet
- **`tests/test_d2_real_scan.py` (sim-marked, 5 tests)** puts the robot in an aisle built
  from two walls about `base_ref` and asks D2's question of the scan the certified layer
  will actually receive, rather than of a synthetic one. It covers what only the simulator
  can answer: binning takes the NEAREST point in each 3 deg sector (shorter than the
  bin-centre range, the direction that trips a stop), the sensors judge visibility from the
  corners, and ranges carry pixel-sampling error.
- **Result on the pinned S300 scene, narrowest stratum (0.12 m per side, aisle 0.657 m):**
  no stop at rest, every bin observed, and the walls measured where they were built (within
  10 mm). **The tightest bin clears the at-rest field by 9.5 mm**, against the 19.1 mm the
  geometry predicts (0.3286 - 0.30945): the nearest-point-in-sector rule eats about half the
  margin. Moving at the top band the tightest bin clears by 59.5 mm centred and **9.4 mm at
  G1b's committed peak cross-track** (0.05 m). An aisle 10 mm below the at-rest floor does
  trip a stop, so the test can fail when it should.
- **Defect found while writing it (mine, in the test):** a cuboid is created with its long
  axis on ref +x while the aisle runs along body +x, a quarter turn away. Placed without
  turning them, the "walls" crossed the robot's path instead of flanking it and the scan
  read 0.215 m abeam instead of 0.329 m. The first version of the test would have reported
  a D2 failure that was entirely its own. Walls are now turned by `yaw_offset` explicitly.
- The synthetic derivation in `tests/test_d2_aisle_strata.py` stands unchanged; this is the
  consistency check on top of it. WP1 item 1 is closed.

## 2026-09-23 (WP1 item 3) - The environment's per-step budget, and a stale figure corrected
- **The budget is derived, not written down.** `pfbnav/eval/step_budget.py`:
  `per_step_budget_s = instances / aggregate_steps_per_s`, from `env.step_budget` in
  `configs/base.yaml` (target 25 steps/s on the training host, 4 instances planned). At
  four instances that is **160 ms p95 per step**; 120 ms at three, 40 ms at one. A test
  asserts no millisecond figure is stored in the config.
- **`docs/requirements.md` said 40 ms at four instances. It was wrong.** That figure divided
  by the `>= 100 steps/s` target which was itself withdrawn as an arithmetic error on
  2026-09-20; the correction did not reach this line and it stood for three days. Corrected
  with both withdrawn figures named, per the standing derivation rule.
- **`scripts/check_step_budget.py`** scores a profiling pass against the derived budget and
  reports what moved between two passes. On `step_cost_20260918T194545Z` (laptop, GUI
  rendering in force, commit 486495c): at 4 instances the step's p95 is 102 ms, **64 % of
  its 160 ms budget**, worst-pass aggregate **40.9 steps/s**; at 3 instances 82 % and
  34.0 steps/s; at 2 and 1 the aggregate is 20.8 and 12.5 steps/s, below the target, which
  is what the arithmetic predicts rather than a fault. The step is carried by `sim.step`
  (76.6 ms p95, 72 % of it), then `command` 9.8, `torque_cap` 9.4, `contact` 5.5 ms.
- **None of this is a verdict on the requirement**, which binds on the training host with
  the learner running; the script says so in its own output. What it catches is a change in
  the shape of the step -- the scan packet will show up as an added component.
- **Open, and blocking G2:** `preregistration/gates.yaml` carries no
  `G2_compute_budget.min_steps_per_second`. CLAUDE.md requires a gate's thresholds to be
  committed before it runs, and `preregistration/` is never edited here (hard rule 1), so
  the researcher commits it. `tests/test_step_budget.py` skips with that reason and turns
  into a consistency check the moment the key appears.

## 2026-09-23 (WP1) - The certified-layer emulator implemented (environment, slice 1)
- **`CertifiedLayer` was a declared interface raising `NotImplementedError("WP1")`.** It is
  now implemented, pure and simulator-free, and tested on synthetic scans
  (`tests/test_certified_layer.py`, 14 known-answer tests). The environment's step loop
  needs it: the emulator runs beneath every controller, and its stops are the primary
  metric.
- **The sequence it emulates:** the speed band comes from the ESTIMATED velocity (never the
  command, never wheel odometry -- ruling 3 on the D7/G1b report); a valid range inside the
  selected field, or any unobserved bin (hard rule 5), breaches it; a breach TRIGGERS a stop
  that becomes ACTIVE `response_time` later, so the robot keeps its command through the
  latency the field is sized to cover; the stop is LATCHED while breached and for
  `restart_delay` after the field last came clear. `override` zeroes the command only while
  a stop is active; how fast the base then slows is the platform's business, and `a_brake`
  sizes fields rather than commanding brakes.
- **A stop is counted once**, on the trigger, not once per step and not again when a breach
  returns during the restart delay: the metric is protective stops per 100 m, so a miscount
  here would be a miscount of the result.
- **Travel direction is held when the speed estimate falls below the standstill threshold**,
  rather than swinging with noise: the field is directional, and a meaningless bearing would
  put the braking depth in a meaningless place.
- **`warning`** reports the field of the band ABOVE the one in force being breached: an
  early indication on the same geometry, since no separate warning field is specified. It
  is reported and never acts. **Defect caught by its own test:** the first version compared
  against the next band top above the current *speed*, which is the current band's own top,
  so it could never warn. Open for the supervisor if a separate warning field is wanted.
- **Default footprint:** if none is passed, the configured rectangle's circumscribed radius,
  which over-states the robot rather than guessing a shape. The environment will pass the
  measured table.

## 2026-09-23 (WP1) - The step loop (environment, slice 2)
- **`pfbnav/envs/step_loop.py`**: the simulator-facing half of the environment, and the
  context `gates.yaml` calls `env_step_loop`. Order per step: the emulator decides on the
  scan already in hand (the world as last sensed, which is all a real controller has); a
  stop overrides the command before anything is applied; the base clamps, projects into the
  wheel budget and rate-limits what remains and reports what it *applied*; one physics step;
  then sensing -- one scan packet (36 + 120 bins), the pose, the ground-truth contact check;
  then the velocity estimate, which is what the next step's band selection uses.
- **It owns no task, reward or observation**, so it could be written and exercised before
  the scenarios and map that WP1 still owes. `reset` means the loop's own state, not the
  robot's pose.
- **Counters** carry the primary metric: protective stops (counted on the trigger),
  stopped steps, contact steps, warning steps, steps with an invalid bin, distance, and
  over-budget steps. `stops_per_100m` returns **None** until the robot has moved: 0 stops
  over 0 m is not a rate.
- **Tested** by `tests/test_step_loop.py` (5, pure) and `tests/test_step_loop_sim.py` (7,
  sim-marked): one fresh scan per step stamped for its own tick, the applied command
  reported rather than the requested one, contact checked inside the loop, and the emulator
  stopping a creeping robot at a wall -- triggered once, active, command zeroed, no contact.
- **Fixture fix:** `tests/conftest.py` now stops the simulation before loading the scene. A
  run interrupted mid-test leaves the simulation running, and `sim.loadScene` then refuses
  ("simulation is not stopped"), which failed every sim-marked test for a reason that had
  nothing to do with the tests.
- **Measured, and it matters: the certified layer costs 124 ms p95 per step** (whole step
  243 ms p95, over the 160 ms budget on all 10 steps, on the laptop with rendering off).
  `sim.step` is 91 ms of it and the scan packet only 11 ms, so the emulator -- our own
  Python -- is now the largest single cost in the loop, larger than the physics. It
  evaluates the field over a sampled support function twice a step (the field and the
  warning band). This is a real regression risk for G2 and is the next thing to fix; the
  budget accounting caught it on its first run, which is what it is for.

## 2026-09-23 (WP1) - The emulator's field evaluation cached: 124 ms -> 1.5 ms a call
- **Measured cost, not guessed:** the step loop's first sim run put the certified layer at
  124 ms p95 per step, more than `sim.step` (91 ms) and ten times the scan packet (11 ms).
- **Two things were rebuilt on every call that cannot change once the object is built:**
  the footprint's support on the support grid -- a 2880 x 1440 product, already computed in
  `__init__` and never used -- and `1 / cos(psi - n)` for the bearing grid, which in the
  step loop is one of the two scan streams and never changes. The bearing cache is bounded
  (4 grids) and both caches hand back copies, so no caller can write through them.
- **Result:** 124 ms -> 11 ms with the support cached, -> **1.5 ms** with the cosine matrix
  cached as well, measured on the filter stream with the heading changing every call. The
  emulator is no longer the largest cost in the step.
- **No number changed**, which is the only condition under which a safety layer's geometry
  may be cached: `tests/test_certified_field_cache.py` (54 cases) compares the cached path
  with the arithmetic written out in full -- every band, at rest and moving, five headings,
  three bearing grids including one that is neither uniform nor aligned -- to within 1e-12,
  on an ASYMMETRIC footprint so nothing is hidden by symmetry. The D2, strata and certified
  suites are unchanged and still pass.

## 2026-09-23 (WP1) - The v2 observation layout, ruled: 82-D (hard rule 7)
Ruled by the supervisor, 2026-09-23. v1's 45-D unicycle observation is not a guide.

| block | channels | contents |
| --- | --- | --- |
| policy scan | 36 | ranges, normalised by `scanners.range_max` |
| validity | 36 | one flag per policy bin |
| velocity | 3 | (vx, vy, wz), body frame, from the pose-difference estimator |
| lookahead | 3 | (d_wp, sin beta_wp, cos beta_wp) to the tracked waypoint |
| goal | 3 | (d_g, sin beta_g, cos beta_g) |
| certified | 1 | `stop_active` |
| **total** | **82** | |

- **Angles as (sin, cos), distances separately.** A bearing fed as one number jumps by 2 pi
  across the seam; the pair is continuous everywhere, and the decomposition is the same for
  the goal and the lookahead so the policy sees one encoding, not two.
- **The lookahead sub-state was missing from the proposal and is required:** the arc-length
  progress reward is defined on the projection onto the planned polyline, so a policy
  without the waypoint cannot see what it is being paid for. **The no-planner ablation drops
  these three channels, giving 79-D.**
- **Certified state: `stop_active` only.** `field_id` and `warning` are excluded.
  - *Why `stop_active` is in:* state observability. While a stop is latched, commands have
    no effect; without the flag those transitions are unexplainable to the agent, which
    would be learning from a partially observed process for no reason.
  - *Why the rest is out:* including the layer's state would make **H1 circular** --
    SAC-without-PFB would also learn to avoid stops, so a difference in stops per 100 m
    would no longer isolate the PFB; it would be **unfair to the baselines**, which cannot
    see it; and it would **couple the policy to a layer that is deliberately independent**
    (hard rule 3 keeps `certified.py` and `pfb.py` apart, and the same separation applies
    to what the policy may read).
  - **Declared ablation:** one L5 seed trained without `stop_active`, reported. It is
    declared now so it cannot become a post-hoc explanation later.
- **Validity as 36 separate channels, not a sentinel range.** Hard rule 5: unobserved space
  is never free, and a sentinel makes "unobserved" indistinguishable from "an obstacle at
  that range". On the S300 layout every bin is currently valid, so the flags would be
  constant inputs with untrained weights -- therefore **declared sensor-dropout
  augmentation at a low rate during training**, so the channels carry signal and the
  weights are trained. Without it the sensor-failure case is untested and must be stated in
  limitations.
- **The filter stream is untouched:** 120 bins with exact bearings, for the safety layers,
  independent of the policy's 36. The policy never reads the filter stream.
- **Velocity from the pose-difference estimator**, never the command and never wheel
  odometry, carrying the declared noise and latency settings (ruling 4; A9).
- **Pose-agnostic, as in v1:** no absolute x, y or theta in the observation. A policy given
  its world pose can memorise a layout instead of reacting to what it senses, and the test
  layouts are the point of the experiment.
- **Action:** a in [-1, 1]^3 scales per axis by its own ceiling -- (a0 * v_max,
  a1 * vy_max, a2 * w_max), v_max 0.40 and vy_max 0.50 -- never one shared speed; the box
  corner is projected into the wheel budget with direction preserved.

## 2026-09-23 (WP1) - Map baking: exact footprints, unmapped obstacles excluded
- **`pfbnav/world/baking.py`** turns a layout's obstacle polygons into the grid the global
  planner searches. Pure; it reads the layout, never the scene -- baking from the simulator
  would make "what the planner was told" and "what is there" the same thing by construction,
  and the unmapped-obstacle case is exactly the difference between them.
- **Exact footprints (the v1 rewrite).** v1 marked each obstacle's axis-aligned bounding
  box. A cell is occupied here only when the polygon actually meets the cell's square, by a
  separating-axis test. Measured in the tests: a 1 m square rotated 45 deg marks 144 cells at
  0.1 m resolution -- its area (100) plus the boundary rim -- where its bounding box would
  mark 200. A shelf at an angle no longer closes aisles the robot can drive.
- **A cell that only shares an edge with an obstacle is not occupied.** Counting a touch as
  overlap would inflate every obstacle by a ring of cells, which is the bounding-box error
  in another form; clearance and the lethal tier carry the safety, and they are measured
  from the polygon itself.
- **Unmapped obstacles are dropped from every layer and listed by name** in the result, so a
  run states how many the planner was not told about instead of leaving the omission
  invisible. Tested by baking the same layout with and without one: the grids are identical.
- **Two tiers, from D1:** lethal = cells within `inscribed + lethal_margin` (0.2386 m today)
  of an obstacle, where the inscribed radius is `robot.half_width` -- no heading lets the
  centre sit there; soft = a linear cost from 1 at that boundary to 0 at
  `circumscribed + soft_margin` (0.36742 m), discouraging and never forbidding. Radii are
  derived from the config and the measured footprint at bake time, never stored: a copied
  number goes stale at the next measurement (D1).
- **Linear soft decay** because nothing measured says otherwise; it is a stated choice, not
  a tuned one.
- **18 known-answer tests** (`tests/test_baking.py`): distances by hand, the rotated-square
  bound above, every cell whose centre is inside an obstacle marked, the lethal tier equal
  to the clearance threshold, the unmapped comparison, indexing round trips, and refusals
  (empty bounds, zero resolution, a soft radius inside the lethal one, a two-point polygon).

## 2026-09-23 (WP1) - A* over the baked map: statuses instead of a fallback
- **`pfbnav/planning/astar.py`**: 8-connected A* on cell centres, octile heuristic scaled
  by the resolution (admissible, so the first path found is optimal on the grid), with
  line-of-sight simplification. The global planner searches centres, not the footprint: D1
  makes it deliberately not footprint-complete.
- **Every failure has a name** (`start_outside_map`, `goal_outside_map`, `start_in_lethal`,
  `goal_in_lethal`, `unreachable`, `expansion_limit`), the path is empty unless the status
  is `ok`, and nothing is invented. v1 wrapped the search in `try/except` and returned a
  straight line on failure, so an unreachable goal looked like a plan through a shelf
  (hard rule 9). Tested by walling a goal off: the result is `unreachable` with an infinite
  cost and no path.
- **Cost:** a move costs its length times `1 + soft_weight * cost(cell entered)`; lethal
  cells are never entered, and a diagonal between two lethal cells is refused so a path
  cannot squeeze through a corner that does not exist.
- **`planner.soft_weight: 1.0` added to `configs/base.yaml`** (VERIFY): D1 fixed the two
  tiers but not what the soft tier costs the search. At 1.0 a cell on the lethal boundary
  is twice as expensive as free space, so the planner accepts a detour of up to twice the
  distance to leave the band -- "discourages, never forbids" with a number attached. To be
  tuned on `scenarios/tune` only (hard rule 2) once plans are run.
- **Simplification uses a supercover traversal, not Bresenham.** Bresenham visits one cell
  per step and skips cells a line merely clips, so a path simplified on it can shave the
  corner of a lethal cell. `line_cells` walks the grid lines and includes both cells at
  every crossing; every returned segment is asserted clear in the tests.
- **Ties are broken by insertion order** (a counter in the heap key), so one map and one
  request give one plan: determinism within a process is claimed, and a bare heap would
  not honour it.
- **19 known-answer tests** (`tests/test_astar.py`): octile distances by hand, supercover
  coverage and symmetry, a clear diagonal simplifying to two waypoints, a wall forcing a
  detour past its end, each named failure, the diagonal squeeze refused, the soft tier
  pushing a path off a wall and paying in length for it, simplification never crossing a
  lethal cell and never lengthening the path, and repeatability.
- **Two test defects of mine, both caught here:** `zip(x, x[1:], strict=True)` can never be
  equal-length, and the soft-tier test measured the path's maximum y, which is pinned by
  its endpoints and says nothing about what happens between them.
- **`scripts/diag_plot_map.py`** draws what the planner believes: obstacles as given, the
  cells the bake marks occupied, the lethal tier, the soft tier, and the plan. Unmapped
  obstacles are drawn as dashed outlines only. The default figure
  (`docs/figures/map_and_plan.png`, 12 x 8 m, 5 cm cells, 160 x 240) puts the unmapped
  pallet in the corridor the plan uses, so the figure shows the planner routing **straight
  through an obstacle it was never told about** -- the case the local planner and the
  safety layers exist for. 208 waypoints simplify to 4 over 10.88 m. The layout is
  hand-built and the title says so; it is replaced when the generator lands.

## 2026-09-23 (WP1) - Lookahead tracking: monotonic projection with a search window
- **`pfbnav/planning/lookahead.py`**: arc-length projection onto the planned polyline, the
  lookahead point, and the body-frame encoding the observation needs. Pure; the tracker is
  the only object, and it holds one number -- how far along the path the robot has come.
- **The progress reward is defined on this projection** (observation ruling, 2026-09-23),
  so `progress` is the arc length gained since the previous update and nothing else. A
  robot rounding a shelf is paid for following the plan rather than penalised for moving
  sideways relative to the goal.
- **Monotonic forward search was not enough, and the test said so.** On a hairpin whose legs
  are 0.4 m apart but 7 m apart along the path, searching forward from the current arc
  length still projects a robot on the outbound leg onto the return leg: it would be paid 7 m
  for moving a centimetre. The projection now takes a **window** as well -- it searches
  `[s, s + window]` -- so a leap cannot be counted. `planner.track_window: 1.0` m: at
  v_max 0.40 and dt 0.05 the robot covers 0.02 m a step, so a metre is fifty steps of slack
  and far below the arc length to any doubled-back leg. A teleport (a reset, a hiccup) is
  clamped to the window instead of being banked as progress, which a test pins.
- **Encoding:** the lookahead and the goal each as `(distance, sin bearing, cos bearing)` in
  the body frame; at zero distance the bearing is reported as straight ahead rather than a
  NaN a network would propagate.
- **22 known-answer tests** (`tests/test_lookahead.py`): arc lengths and interpolation,
  projection on a straight path, at a corner, before the start and past the end, the
  hairpin case above, the tracker never going backwards over an out-and-back, progress as
  arc length, the lookahead turning a corner and stopping at the end, the body-frame
  encoding under rotation and translation, the unit-vector property, and the refusals.
  The tracker's tests step it as the step loop will (2 cm a step), not in leaps.

## 2026-09-23 (WP1) - Layouts are generated and instantiated into the scene (option 1)
- **The question the figure exposed:** `scripts/diag_plot_map.py`'s layout is hand-built in
  the script, not read from `scenes/warehouse_v2.ttt` -- which is an empty hall with walls
  more than 5 m from the robot. A layout must exist in two places: as polygons for the bake,
  and as geometry in the simulator for the scanners and the contact check to see. v1 avoided
  the question by baking FROM the scene (all shapes minus `/youBot` and `/goal_dummy`).
- **Decision (researcher, 2026-09-23): option 1.** The generator emits polygons; the bake
  uses them; the environment instantiates the same polygons into the scene at episode reset
  and removes them at the end. One source of truth, in the generator.
- **Why not bake from the scene (v1's way):** it makes "what the planner was told" and "what
  is actually there" the same thing by construction, so an unmapped obstacle cannot be
  expressed at all -- and the gap between the two is what the local planner and the safety
  layers exist for. An unmapped obstacle is now just a polygon flagged `mapped=False`:
  instantiated in the simulator, absent from the bake, counted in the run.
- **Why not author layouts in scenes:** the scene and its description would drift apart
  silently, and D2 defines the aisle strata as a property of the layout generator, which
  implies systematically varied arrangements rather than a hand-built hall.
- **Consequences to carry:**
  - the scene file stays fixed and pinned by its SHA-256, so hard rule 12 stays simple: no
    layout ever changes `scenes/warehouse_v2.ttt`;
  - shapes are created and removed per episode, which costs wall clock at reset and must be
    measured against the episode (1,500 steps at 0.05 s is 75 s, so a 1 s reset is ~1.3 %);
  - **a test must assert the instantiated geometry matches the baked polygons**, or the two
    can drift within a single episode -- the failure this decision exists to prevent;
  - obstacles are static and non-respondable, as the S300 bodies are, unless a case needs
    otherwise: they must be seen by the scanners and counted by the contact check, not
    pushed around.

## 2026-09-23 (WP1) - Layout sets: splits, unmapped obstacles, and what runs must verify
Six rulings (supervisor, 2026-09-23), on top of the option-1 decision recorded above.

1. **Split policy approved, with identity carried in metadata.** Layouts are generated from
   disjoint seed ranges per split, recorded in `preregistration/`, and the test set is
   frozen by a lock. **A layout set's identity is (seed, layout hash, generator code
   hash)**: the generator's own hash travels in the set's metadata and every run verifies
   it, so a change to the generator cannot silently produce a different world under the same
   seed. Tests assert both seed ranges and generated layout hashes are disjoint across
   splits. **Tune layouts are disposable** and may be regenerated with a log entry; **test
   layouts may not**, absent a recorded amendment.
2. **Unmapped obstacles: a low declared rate in train, a fixed rate per test stratum.**
   Proposed ~20-30 % of training episodes containing at least one, refined to whatever the
   generator makes natural; test strata are fixed at 0 % (mapped-only) and 100 %
   (unmapped-static). All preregistered before any policy trains. **The train rate is a
   shared environment hyperparameter, identical across L1, L2, L4, L5 and A1, never varied
   per method** -- otherwise a method's advantage could come from the world it was trained
   in rather than from the method. Reasoning: **zero in train makes the test a distribution
   shift**, so a drop in performance would not separate "cannot handle unmapped obstacles"
   from "has never seen one"; **a high rate teaches a general distrust prior** that is not
   industrially realistic, since a real AMR's map is mostly right.
3. **Per-episode logging: the number of unmapped obstacles and each one's lateral offset
   from the planned path.** H1 is then reportable as a function of how far the map
   understates the world, not only as a stratum mean -- an obstacle 0.1 m off the path and
   one 1.5 m off are not the same test, and averaging them hides the gradient.
4. **The geometry check runs per episode, at reset:** every instantiated shape matches its
   polygon in pose and extent, and every `mapped=False` polygon is absent from the baked
   grid while present in the scene. Not a one-off test: drift within a single episode is the
   failure option 1 exists to prevent.
5. **Reset wall clock is recorded per episode in the run summary from the start**, and G2
   measures it. **Watch for handle or memory accumulation across resets:** v1 hit that class
   of failure at ~70k steps, and creating and removing shapes every episode is exactly the
   pattern that causes it.
6. **Measured, not assumed: `sim.checkCollision` DOES see a non-respondable shape inside a
   collection.** `tests/test_unmapped_obstacle_contact.py` (sim-marked, 4 cases) puts a
   static block overlapping the robot with `respondable` on and off: contact registers in
   both cases (`part=chassis`, names `/youBot`, `/Cuboid`), a block 2 m away registers in
   neither, and with `respondable` off the robot does not move when stepped -- it passes
   through rather than being shoved. So an unmapped obstacle can be non-respondable *and*
   still counted by the ground-truth contact check, which hard rule 4 makes the sole
   definition of a collision. `respondable` governs the dynamics solver; `collidable`
   governs collision queries; they are independent.

## 2026-09-23 (WP1) - The layout generator
- **`pfbnav/world/layouts.py`** generates a hall of shelf rows at an aisle clearance sampled
  from a D2 stratum, so a layout states its stratum by construction: rows are spaced at
  `2 * (half_width + clearance)` and the metadata carries the clearance, the aisle width and
  the stratum name. A test measures the gap between the generated shelves rather than
  trusting the metadata.
- **Identity, as ruled:** metadata carries `(seed, layout_sha256, generator_sha256)`. The
  layout hash is over the geometry (rounded to a micrometre, so the same world hashes the
  same whatever floating-point path produced it) and includes each obstacle's `mapped` flag;
  the generator hash is over this module's source, so a change to the generator cannot
  silently produce a different world under the same seed.
- **Unmapped obstacles** are placed by rejection sampling clear of everything mapped, by a
  configured margin, and flagged `mapped=False`. The generator never decides HOW MANY -- the
  rate is preregistered and shared across methods -- so the caller passes a count.
- **`distance_to_path`** measures an unmapped obstacle's shortest distance to the planned
  polyline, which is what the per-episode log needs (ruling 3).
- **Defect found by drawing it (2026-09-23).** With shelf rows sampled at an absolute
  2.4-4.0 m in a 14 m hall, the rows were short against the hall, open floor ran beside them,
  and the plan crossed it diagonally: **a narrow-aisle layout that never made the robot use
  a narrow aisle**, which would have made the D2 strata decorative. Rows are now a FRACTION
  of the usable span (`shelf_length_fraction`, 0.70-0.92) and alternate which end they leave
  open, so a traverse threads the aisles.
- **Still open, and it belongs to scenario generation, not here:** even with full-length
  rows, a start and goal in the open leave the plan free to go around the block rather than
  through an aisle. **Endpoints must be chosen so the stratum binds** -- start and goal in
  different aisles, or on opposite sides of a row -- or a layout's stratum will not describe
  the run that happened. Recorded as a requirement on `pfbnav/world/scenarios.py`.
- **23 known-answer tests** (`tests/test_layouts.py`): aisle width and stratum arithmetic,
  the measured gap between rows, the robot fitting every aisle generated, shelves inside the
  walls, same-seed reproducibility, hash sensitivity to geometry and to the `mapped` flag,
  seed-range disjointness and geometric disjointness across splits, unmapped obstacles clear
  of mapped geometry and absent from the baked grid, `distance_to_path`, and a hall too
  shallow for its aisles raising rather than generating something silently wrong. One test
  skips until `layout_sets.seed_ranges` is preregistered.

## 2026-09-23 (WP1) - Scenario generation: endpoints that make the stratum bind
- **`pfbnav/world/scenarios.py`**: a scenario is a generated layout, a start pose, a goal,
  and the identity a run verifies. Pure; the planner is used at generation time to check the
  task is the task it claims to be.
- **A scenario is kept only if its plan threads an aisle.** `aisle_rectangles` gives the gaps
  BETWEEN shelf rows where the rows overlap in x -- end aisles are excluded, because
  travelling along one is going round the block -- and `aisle_travel` measures how much of
  the plan lies inside them. A candidate is rejected below `world.layout.min_aisle_travel`
  (1.5 m). Endpoints are placed inside aisles along the shelves' own span, not in the open
  cross-aisle at the hall's end.
- **Clearance alone was not enough, and the first version proved it.** Accepting a plan whose
  tightest clearance was near the layout's per-side clearance passed a "wide" scenario whose
  path merely clipped a shelf corner (0.259 m in a 0.564 m aisle) while never entering an
  aisle, and rejected every narrow layout. The measure is now aisle travel, on the plan.
- **Reproducibility trap found by `verify` (2026-09-23).** `generate` sampled its stratum
  from the rng only when none was given, so regenerating a scenario with its recorded
  stratum skipped that draw, shifted every later draw, and produced a different world -- a
  set failed verification against itself. The draw is now unconditional and the explicit
  stratum overrides it, so `generate(seed, cfg)` and the same call with the stratum it would
  have sampled give the same world. This is exactly the failure the identity triple exists
  to catch, caught before any set was built.
- **Sets** carry what was requested AND what was achieved: `episode_rate_requested` against
  `episode_rate_achieved`, which is what a paper reports. An empty set reports `None` rather
  than 0, because 0/0 is not a rate. Failures are listed by seed with a named status.
- **The lock works as ruled:** `write_set(..., lock=True)` marks a set locked, and writing
  over a locked set raises -- test layouts need a recorded amendment, not a rerun. Tune sets
  are rewritable.
- **`verify` regenerates and compares** the layout and generator hashes per scenario plus the
  module hash for the set, listing every mismatch rather than raising, so a report can say
  which scenarios moved.
- **25 known-answer tests** (`tests/test_scenarios.py`), including the rate honoured over
  4,000 draws, the lock, verification catching both a moved world and a changed generator,
  and an impossible aisle-travel requirement refused by name rather than met with an easier
  scenario.

## 2026-09-23 (WP1) - The observation and the reward, implemented (hard rule 7)
- **`pfbnav/envs/observation.py`** implements the ruled 82-D layout and nothing else: 36
  scan, 36 validity, 3 velocity, 3 lookahead, 3 goal, 1 `stop_active`; 79-D without the
  lookahead block for the no-planner ablation. `slices` names each block, so a diagnostic
  can report a channel rather than an index.
- **Scaling (VERIFY, part of the layout under hard rule 7):** ranges by
  `scanners.range_max`; velocity by its OWN axis ceilings (0.40 / 0.50 / 1.5), the same
  asymmetry the action uses; the lookahead distance by the lookahead itself, so a robot
  tracking normally sees ~1; the goal distance by `env.observation.goal_scale` (18 m, above
  the hall's 17.2 m diagonal) so distant goals stay distinguishable instead of saturating.
  Distances clip at 1 and never wrap. Every channel is bounded, and the Box bounds are
  derived from the same table.
- **Validity stays a separate block**, and a test asserts the range channel is untouched by
  the flag: "unobserved" and "an obstacle at that range" must remain different inputs
  (hard rule 5).
- **`dropout`** implements the declared augmentation: a dropped bin is flagged invalid AND
  its range is set to `range_max`, so the range channel cannot leak what the sensor would
  have seen. It copies its inputs, and at rate 0 it does not touch the rng -- so switching
  the augmentation off cannot shift a training run's random stream.
- **`pfbnav/envs/reward.py`** reads the `reward:` block and returns the total with a
  breakdown that sums to it exactly, with every term present on every step so a log has
  fixed columns. **A protective stop is charged once, on the trigger**, matching how the
  emulator counts it: charging per stopped step would pay a policy to break a latch it
  cannot break, and would tie the penalty to the restart delay rather than to the stop.
- **Intervention is measured on the axes' own ceilings**, so a lateral difference is not
  confused with a forward one; the box clamp, the wheel budget and a protective stop all
  reach it the same way, because the policy is charged on what it got, not what it asked.
- **`episode_outcome` names the ending**, and arriving while in contact counts as a
  collision: a run that ends in contact is not a success, whatever else happened that step.
- **34 known-answer tests** (`tests/test_observation.py`, `tests/test_reward.py`).

## 2026-09-25 (WP1) - Instantiating a layout into the scene
- **`pfbnav/sim/layout_shapes.py`** builds a generated layout's polygons as shapes: mapped
  and unmapped alike, static and non-respondable, at heights that cross the 0.221 m scan
  plane (`shelf_height` 1.6 m, `unmapped.height` 0.80 m -- an unmapped obstacle must be
  SENSED though unmapped; one below the plane would test the limitation, not the method).
- **The layout moves, not the robot.** A scenario's start is a pose in the layout's frame,
  and placing it in the world would mean teleporting a dynamic robot every episode.
  `transform_placing` returns the rigid transform that puts the start where the robot
  already stands after `start()`; the environment keeps it and converts poses back into
  layout coordinates. Layout coordinates stay exactly what the generator produced.
- **Shapes are created while the simulation runs, so a stop takes them with it.** That is
  the episode's cleanup, and it is why handles cannot accumulate: measured over five
  reset-and-rebuild cycles, the scene holds exactly the layout's shape count every time
  (item 5; v1 accumulated at ~70k steps). `report()` counts what exists so a run can watch
  it anyway.
- **The per-episode check (item 4) is implemented and exercised in a real scene:** `verify`
  reads each shape's position, yaw and extent back out, maps them into layout coordinates,
  and compares them with the polygon they came from; `unmapped_absent_from_bake` is the pure
  half. A test nudges one shape by 50 mm and asserts `verify` reports it -- a check that
  cannot fail says nothing when it passes.
- **Measured: the scanners see the layout.** In the empty hall no filter bin reads under
  4 m; with the layout built, 110 do. A shelf the scanners cannot see would be furniture,
  not an obstacle.
- **A non-rectangular polygon is refused, not approximated** (`rectangle_of`): a shape that
  does not match its polygon is precisely the drift this module exists to prevent.
- **17 tests**: 10 pure (the transform round-tripping, rigidity, rectangles including a
  rotated one, refusals) and 7 sim-marked (placement, read-back, the unmapped obstacle in
  the scene and absent from the map, the scanners seeing it, cleanup at stop, no
  accumulation over five resets, double-instantiation refused, and the moved-shape check).
- **The S300 models' own scripts error on every sensing step, and always have** (reported by
  the researcher, 2026-09-25): `[/youBot/SickS300[0]/Script]:52: attempt to perform
  arithmetic on a nil value`. The model's demo script calls `sim.readVisionSensor` and reads
  `u1[1]`/`u1[2]` as the auxiliary packet's resolution; these sensors carry no
  coordinate-extraction filter, so the packet lacks those fields and `u1[2]-1` is arithmetic
  on nil. **The model's own scanning therefore never worked in this scene** -- it was never a
  fallback we declined to use, which is worth stating beside the server-side design.
- **No measured run is affected:** `Scanner.install()` disables every script inside a scanner
  model by ancestry, read back, before `start()`, and restores them afterwards. The errors
  appear only when the simulation runs without the scanner installed (the GUI's Play button,
  or sim-marked tests that do not install it).
- **Coupling now on record:** that script's `sysCall_init` sets the sensors' far clipping,
  their 135 deg field of view AND the joint angles that orient them (-/+67.5 deg). If it ever
  ran before our coverage masks were computed, the sensors would move after their poses were
  measured. It cannot today -- we disable it before the simulation starts -- but the
  dependency is real and is why the masks are computed from the poses read at install time.
- **Open, for the researcher:** disabling those scripts in the scene would be a deliberate
  scene edit (hard rule 12) and is safe -- our Lua sets far clipping and resolution every run,
  and the saved sensor orientations already match what the script would set. Left as is for
  now; nothing measured depends on it.

## 2026-09-25 (WP1) - The Gymnasium environment
- **`pfbnav/envs/nav_env.py`** wires the parts that already existed and holds no geometry,
  reward arithmetic or observation layout of its own: scenario -> layout -> bake -> plan ->
  tracker -> shapes -> step loop -> observation -> reward.
- **An episode's reset**: stop, start (the robot returns to its scene pose), instantiate the
  layout placed so the scenario's start lands where the robot already is, **verify every
  shape against its polygon** (item 4) and refuse the episode by name if any differs, read
  the first scan. `info` carries the unmapped count, **each obstacle's offset from the plan**
  (item 3) and the **reset wall clock** (item 5); measured at ~2.6 s, which is 3 % of a
  1,500-step episode.
- **A scenario whose layout regenerates to a different hash is refused** (`ScenarioUnusable`)
  rather than run: the identity triple is checked at the moment it matters.
- **Two defects found by driving it, both mine:**
  - **The layout was placed a quarter turn off.** `YouBotBase.pose()` returns the BODY yaw
    (base_ref plus `robot.yaw_offset`), but the transform was built from base_ref's raw
    orientation, so the plan was aligned a quarter turn from chassis-forward and the robot
    drove sideways relative to its own route: 0.000 m of progress and 0.201 m of cross-track
    over 25 steps. Fixed to the body convention.
  - **An unmapped obstacle could sit on the start.** Seed 1000005 put one 0.023 m from the
    plan AT the start, so the robot spawned in contact and the episode ended on step 1. An
    obstacle ON the route is the case the safety layers exist for; one AT the start is not a
    test. Scenario generation now rejects an endpoint pair unless every unmapped obstacle is
    at least `min_arc_from_start` (2.0 m) along the plan and `min_distance_to_goal` (1.0 m)
    from the goal, and records each obstacle's **arc position** beside its offset, so H1 can
    be read against where the map understates the world as well as by how much.
- **Measured end to end** (`tests/test_nav_env_sim.py`, 11 sim tests): following the plan
  into the unmapped obstacle, the emulator stops the robot -- applied speed 0.192, 0.142,
  0.092, 0.042, 0.000 m/s as the base decelerates under its own acceleration limit -- and
  **there is no contact**. The stop zeroes the command; it does not teleport the base to
  rest, which is why the test asserts deceleration rather than an instant zero.
- **Resets do not accumulate**: layout shapes per scenario across repeated resets are
  {scenario 0: [8, 8], scenario 1: [7, 7]}, and each reset's count equals that layout's own
  obstacle count.
- **The no-planner ablation runs** at 79 channels through the same environment.

## 2026-09-25 (WP1) - G0's `env_step_loop` context implemented
- **`scripts/gate_g0_contact.py --contexts`** now runs each case in either context.
  `env_step_loop` drives the same six cases through `pfbnav/envs/step_loop.py` -- the loop
  episodes run -- so the contact check is validated in the order the environment calls it:
  command, physics step, scan packet, pose, contact. The per-handle validation of ruling 1
  is done around the loop, not inside it, so the loop stays the one episodes run.
- **The certified layer is evaluated but may not override in that context.** A positive case
  must reach contact for the gate to validate anything, and the emulator would stop the
  robot first; `StepLoop` now honours `certified.enabled`, always evaluating and reporting
  the layer and overriding the command only when it is on. The step at which the layer
  WOULD have stopped is recorded per case (`certified_would_stop_step`). Off is for this
  gate and for a declared ablation, never for a result.
- **`--cases` runs a subset as a smoke check**, and a subset is written to the summary as
  `PARTIAL` with the case names: a partial run cannot be read later as a verdict.
- **Smoke check passed** (`G0_20260925T182943Z`, two cases in `env_step_loop`):
  `front_wall_head_on` lag 0 with the expected shape struck, `parallel_pass_gap_10mm` 0
  false contacts at a 0.0102 m minimum gap, handle-versus-collection agreement on every step
  of both, and the emulator would have stopped at steps 15 and 2 respectively. Step cost in
  this context is 58-64 ms p95, which includes the scan packet the gate_script context does
  not read.

## 2026-09-25 (WP1) - Camera saved into the scene; a display flag
- **The researcher saved the scene with the camera pulled back** so the whole robot is
  visible. A saved scene is a new file and a new hash (0ae35a50 -> 20974276), which every
  run refuses until the config follows (hard rule 12).
- **Verified by inventory, not by assertion.** `scripts/diag_scene_inventory.py` records
  every object's path, type, world pose and bounding box; the pinned scene was extracted
  from git, inventoried, and compared with the saved one: **111 objects, 0 added, 0 removed,
  0 moved, 0 resized**. Only the view changed. Committed alone with the hash.
- **The footprint table still records the hash it was built from**, and a hash comparison
  alone would now fail although the geometry is identical. Rather than overwrite that
  header (which would falsify the table's provenance) or re-run a 30-minute probe to
  measure the same robot again, the **equivalence is recorded with its evidence** in
  `docs/scene/equivalence.json`, and `tests/test_footprint_config.py` consults it: the
  table is accepted on a different hash only when a recorded equivalence covers the pair
  and the cited diff exists. A scene edit that moved anything would not be covered and
  would still fail.
- **`--display {config,on,off}`** added to the gates and diagnostics
  (`pfbnav/utils/cli.py`), so a run can be watched without editing the committed config.
  It is safe to offer because rendering changes no physics -- trajectories were checked
  bit-identical with it on and off -- and every run records what was in force in its
  provenance, so a watched run stays labelled as one. Default `config`, which keeps
  rendering off: ~7 ms of a ~27 ms step.

## 2026-09-25 (WP1) - G0 PASSES in both contexts, three seeds
- **Result: PASS** (`runs/gates/G0_20260925T185322Z_seed0`, `...T195422Z_seed1`,
  `...T195847Z_seed2`). Commit `62073bb`, clean, pinned scene `20974276`, on the laptop.
  Six cases in each of the two committed contexts, three invocations: **36 case-runs, all
  passed, no failing case in either context**. This is G0's first non-`INCOMPLETE` result;
  every earlier G0 run lacked the `env_step_loop` context and was recorded as such.
- **Against the committed criteria** (`preregistration/G0_contact_validation`, read from
  the file, not restated): the five positive cases each reached contact at
  **detection lag 0** (limit 1), and `parallel_pass_gap_10mm` held a **0.0101 m** minimum
  gap with **0 false contacts** (limit 0). Identical in all three invocations.
- **Each case struck the shape it was placed for** (the supervisor's 2026-09-22 assertion),
  including the two that are only reachable by a specific part: `pod_corner` met the front
  right swedish wheel at the outermost bearing, `wheel_edge` the front left wheel. The one
  seed-to-seed difference is cosmetic: seed 0's `wheel_edge` also touched
  `wheel_respondable_fl` on the contact step, seeds 1 and 2 did not. The expected shape is
  in the struck set either way.
- **The per-handle validation agrees with the collection check on every step of every
  case**, in both contexts (ruling 1, 2026-09-24): 0 disagreements over 2,460 validated steps.
- **`part` is blank in the `env_step_loop` context.** The step loop reports the contacting
  shapes, not the part classification, so that diagnostic column is empty there. The verdict
  does not use it -- the shape assertion uses the per-handle `struck` set, which is present
  in both contexts -- but the column should not be read as "no part".

### What the certified layer would have done (reported, never hidden)
- Four of the five positive cases: the layer **would have stopped at step 15**, contact came
  at step 36. With the layer on, none of those contacts happens.
- **`wheel_edge`: the layer would never have stopped.** This is correct behaviour and a
  real limitation, not a defect. The case's slab is **0.012 m** high, deliberately low
  enough to pass under the chassis (underside 0.0148 m) so that only a wheel can reach it;
  the scan plane is at **0.221 m**. A scanning safety layer cannot see it, so neither the
  certified emulator nor the PFB can prevent this contact class **by construction**. It
  belongs in the thesis as a stated limit of the contribution, and it is the reason G0
  defines collisions by simulator contact rather than by LiDAR (hard rule 4).
- **`parallel_pass_gap_10mm`: the layer would have stopped at step 2.** A 10 mm pass
  alongside a wall is inside any protective field, so the negative case is a
  contact-truth case, not a "safe manoeuvre" case. Worth carrying into D2: the aisle strata
  must be derived so that legitimate travel is not permanently inside the field, or
  stops per 100 m measures the aisle rather than the controller.

### Step cost (informative only)
- Whole-step p95 in the `env_step_loop` context: **44-60 ms** across cases and seeds, with
  rendering **on** (the researcher watched; `display.in_force: true`, 13 starts, 0 refusals).
  This is the laptop, not the training host, so per `docs/requirements.md` it is not
  evidence about the >= 25 steps/s requirement -- G2 measures that on the training host.
  It does sit inside the 160 ms per-step budget that 4 instances imply.

## 2026-09-26 (WP1 item 4) - G1b re-run: v_max stays 0.40, vy_max stays 0.50
- **Result: PASS** (`runs/gates/G1b-SPREAD_20260926T193955Z_seed0`, over
  `G1b_20260926T180549Z_seed0`, `...T180547Z_seed1`, `...T180544Z_seed2`). The invocations
  ran at commit `d765d97`, clean, on the pinned S300 scene `20974276`; the aggregate was
  taken at `6cf8518`. **198 episodes, 189 passed**, ~41 min per invocation with three
  instances in parallel.
- **Why a full re-run was owed.** The last full grid ran on 2026-09-20 at `ced9e61` against
  the **legacy-lidar** scene `340dbc5d`. The 2026-09-23 work was a narrowed regression (two
  profiles, two speeds) on the intermediate scene `0ae35a50`, which is not a verdict. Every
  constant derived from `v_max` therefore rested on a scene the project no longer runs.
- **`v_max = 0.40`, `vy_max = 0.50`, unchanged.** No band top, traversal speed or narrow
  aisle floor needs re-deriving, so D8's single dependent commit is not triggered.
- **The boundary is decisive, not marginal.** `translate_while_rotating` final heading:
  **2.3585-2.3592 deg at 0.40** (limit 5.0, margin +2.641, 3/3 invocations within) and
  **5.0474-5.0518 deg at 0.50** (margin -0.052, **0/3** within). The spread inside the
  failing cell is 0.0044 deg, a hundredth of its distance to the limit. **No cell anywhere
  in the grid straddles a limit**, so the straddle-FAILS rule is not invoked. Cross-track
  was never close in any cell; heading is the only binding metric. Lateral passed at every
  grid speed and both extensions (0.35, 0.45), so `vy_max` stays at the top of the grid.

### The box did not decide the result, and this was measured rather than assumed
- `clamp_translation_steps` is **not** zero (4,059 steps, concentrated in forward@0.50 and
  lateral@0.50, where the commanded speed equals the box). Read alone that looks like the
  A6 failure mode. It is not: the minimum `clamp_scale` over those cells is **0.999999**
  (forward@0.50), **0.999979** (lateral@0.50), **1.000000** (diagonal@0.50) -- the box
  removes at most two parts in 100,000 of the commanded vector. A6 is satisfied in
  substance, not merely by the script's `v_max >= max(speeds)` guard.
- **What bites is the wheel budget**, as ruling 2 intends: minimum `budget_scale`
  **0.829** (diagonal@0.50), **0.819** (twr@0.40), **0.656** (twr@0.50). The failing cell
  fails while the budget holds it at 66% of its commanded vector, which is why it realises
  0.313 m/s when commanded 0.50. **The 0.50 failure is a capability limit, not a clamp
  artifact** -- the robot cannot translate at 0.5 and rotate at once inside its wheel
  budget. This is the interpretation the gate would have gotten wrong if only the binding
  counts had been reported.
- **Free cross-check on the scene change.** The per-cell clamp and budget binding counts are
  *identical* to the 2026-09-20 legacy-lidar run (budget 2,493 / 2,664 / 2,916 in the same
  cells; clamp 15 steps in twr@0.50 in both). The S300 scanners, being static and
  non-respondable, added nothing dynamically -- now established on the full grid rather
  than on the three cells the 2026-09-23 regression could reach.

### Two record-keeping points
- **Every instance refused `display_enabled=false` mid-run** (5, 3 and 4 refusals of 66
  starts; `consistent: false`, `in_force: false` at the end), as one did on 2026-09-20.
  The cause is still unexplained. No verdict is affected -- rendering is trajectory-neutral
  (`scripts/regress_display_off.py`) and repetitions stayed bit-identical within each
  invocation -- but the files could not say *which* episodes rendered, because display was
  recorded only per run, and the hand-built dict had dropped `starts`, the field that exists
  so refusals can be read against the chances to refuse. Both fixed in `6cf8518`; the next
  run localises it. **Carried to the supervisor:** it has now appeared in two gates, on
  every instance. Harmless here, but G2 measures wall clock, so the cause should be known
  before G2 runs.
- **Two aggregate directories exist for one verdict.** `G1b-SPREAD_20260926T185254Z_seed0`
  records `dirty: true` -- an uncommitted display-reporting edit was in the tree when it
  ran. The aggregator checks the *invocations'* dirty flag, not its own, so it did not
  refuse. The numbers were unaffected (the aggregator reads only the three committed-clean
  summaries and does not import the edited file), and the re-run at `6cf8518` is
  **byte-for-byte identical** in both `summary.json` and all 66 rows of `spread.csv`. The
  clean one is the artifact of record; the first is kept, not deleted (hard rule 10).
  In the report, `commit` names the code that produced the measurement (`d765d97`) while
  the directory's provenance names when the aggregation ran (`6cf8518`); both are wanted.

## 2026-09-27 (WP1) - The narrow floor derived term by term, and what it does not fix
Supervisor rulings on the 2026-09-26 weekly report, taken before Q1-Q3 as instructed.

- **Slack raised 0.02 -> 0.03, narrow floor 0.12 -> 0.13 m** per side, as ruled. Aisle width
  at the floor moves 0.657 -> 0.677 m. Medium (0.20) and wide (0.40) edges unmoved, so the
  band edges are **narrow 0.13-0.20, medium 0.20-0.40, wide 0.40-0.80**.
- **The sector-min term is now derived exactly, not assumed.** `strata.sector_min_range`
  computes what a bin reports for a straight wall under nearest-point-in-sector binning
  (d / cos of the largest in-sector deviation from the wall normal), and
  `sector_min_penalty` differences that against the perpendicular margin. At the old floor
  it costs **2.60 mm**; at 0.13 it costs **1.41 mm**, and it shrinks as the aisle widens.
- **The term the ruling named is about a quarter of what the scan actually eats.** D2 against
  the real scan packet measured the tightest bin clearing by 9.5 mm where the geometry
  predicted 19.1 mm -- a 9.6 mm loss. Sector-min binning accounts for 2.6 mm of that; ~1 %
  range error is ~3.3 mm at a 0.33 m wall, and the ~3.7 mm residual is sensor parallax and
  mesh sampling. **The slack is therefore justified against the measured 9.6 mm total (0.03
  covers it about three times over), never against the derived term alone**, and a test
  asserts that relation rather than the prettier one.

### The raise does not make the narrow stratum admissible, because the binding case is at rest
- **Moving, the narrow stratum was already admissible at 0.12.** The moving field's sideways
  reach is 0.25951 m, so with the committed 0.05 m tracking offset it needs only
  0.25951 - 0.2086 + 0.05 = **0.1009 m** per side. Raising 0.12 -> 0.13 widens the buffer; it
  does not change a verdict.
- **At rest the field is wider abeam** -- `margin` 0.10 all round, against `lateral_margin`
  0.05 when moving -- so its reach is 0.30951 m and its floor for a robot displaced by the
  committed peak cross-track is **0.15091 m per side, above the new 0.13 floor**. At 0.13 a
  robot may come to rest at most **29.1 mm** off-centre while the controller is permitted
  **50 mm** (`max_offset_at_rest`: narrow 0.0291, medium 0.0991, wide 0.2991 m).
- **A stop taken off-centre in a narrow aisle cannot be cleared.** A latched stop holds
  *while the field is breached* and for `restart_delay` after it comes clear; the at-rest
  field stays breached, and a latched stop zeroes the command, so the robot cannot recentre
  to clear it. The episode ends by timeout. This is the emulator behaving correctly.
- **Admitting a stop anywhere the controller may legitimately be** would put the narrow floor
  at 0.15091 + 0.03 = **0.181 m**, leaving a narrow stratum 0.181-0.20 -- 19 mm wide, which
  is barely a stratum and sits where the contribution is supposed to show.
- Raised with the supervisor before the strata lock. Recorded in `configs/base.yaml` beside
  the strata and pinned by `test_the_at_rest_floor_for_an_off_centre_robot_exceeds_the_narrow_floor`,
  which fails if the geometry ever stops saying so in either direction.

### Held until the floor is settled
The traversability precondition harness and the per-stratum unavoidable-stop floor are
specified but not built: both measure against a floor that may move to 0.181, and the
unavoidable-stop floor in the narrow stratum is dominated by exactly the at-rest mechanism
above. Building them first would measure a stratum that is about to change.

## 2026-09-27 (WP1) - Q3, Q4, Q6, Q7 of the 2026-09-26 rulings

### Q6: the emulator has its own warning field (hard rule 7 entry)
- **Was:** `warning` reported the protective field of the **band above** the one in force.
  Two holes follow from that and neither is a detail: the **top band could never warn**
  (there is no band above it), and **a stationary robot could never be warned** (at rest the
  field is band-independent). The fastest state and the state D2 is written about were exactly
  the two that could not warn.
- **Now:** a configured warning field per band. Depth `D_w = warning_scale * D(v) +
  warning_extra`; at rest, where there is no depth to scale, the margin is extended by
  `warning_extra` all round. `warning_scale: 1.5`, `warning_extra: 0.05` m, both **VERIFY** --
  a real S300 configures warning zones independently of its protective zones and these are
  proposed, not measured.
- **`CertifiedParams` validates that the warning field is strictly larger** than the
  protective one, and **the two warning parameters have no defaults**: a field size that
  decides when the robot is warned may not be inherited silently from a dataclass default, so
  every construction site states it.
- **The near-miss metric counts ENTRIES, not steps.** `warning_entries` counts rising edges
  and `warnings_per_100m` puts it on the same footing as the primary metric, returning None
  before the robot has moved for the same reason. `warning_steps` is kept but measures dwell,
  which is a different quantity and must not be summed into a count of near misses.
  A protective breach is always inside the warning field too, so it counts as an entry while
  `warning` itself goes quiet: "warned" means warned and not yet stopped.
- Reported and never acts: not in the observation (the 82-D layout keeps only `stop_active`)
  and never an override.

### Q3: the dropout mechanism changed (hard rule 7 entry)
- **Was:** a flat per-bin rate, and a dropped bin's range was set to `range_max`.
- **Now, as ruled:** a **blocked arc per episode** -- probability 0.10, a contiguous 1-6 bins
  (10-60 deg of the ring), held for the whole episode -- plus **per-bin per-step**
  invalidation at 0.01. The arc wraps, because bin 0 is not a physical boundary. The arc is
  the failure that matters (a smeared lens, tape, a mount that sees the payload) and one a
  policy cannot average away over a few steps.
- **A dropped bin now reads range 0.0, not `range_max`.** `range_max` reads as "nothing out
  there" -- unobserved space presented as free, which is the one thing hard rule 5 forbids.
  0.0 reads as "something right here", the safe direction, and cannot be mistaken for a real
  return because the validity channel beside it is 0.
- **Training only, and `training` defaults to False**, so an evaluation or a gate cannot
  augment by omission: opting in is a decision, opting out must not require one. Never
  applied to the filter's 120-bin stream -- degrading the safety layers' input would make them
  measure better than they are.
- **The arc is drawn unconditionally at reset**, even at probability 0, so the random stream
  does not branch on the outcome. This is the failure `world.layouts.generate` hit on
  2026-09-23, where a skipped draw made two runs with the same seed visit different worlds.

### Q7: retry the display setting; G2 accounts for rendering
- `SimClient.apply_display` now tries up to **3** times. `display_retries` counts attempts
  beyond the first; `display_refusals` counts only starts still wrong after the last attempt,
  so a start that takes on the retry costs a retry and not a refusal -- which matters because
  escalation is triggered on refusals as a fraction of starts. `display_status` carries
  `retries` and `refusal_fraction` (None before any start: 0/0 is not "no refusals").
- **Escalate to a one-day diagnosis only** if `refusal_fraction` exceeds 0.10 or rendered and
  non-rendered throughput differ materially. The cause is deliberately not hunted.
- **G2's specification now carries the rendering criteria**: throughput reported **split** by
  rendered and non-rendered episodes, never pooled, and a **FAIL above
  `max_rendered_episode_fraction` (0.10)**. G2 itself stays unimplemented: it measures
  throughput with SAC updates enabled and SAC does not exist, so building half of it would
  produce a number that is not the one the gate is about. Its stub now says so and names the
  thresholds it needs committed.

### Q4: confirmed, and the evidence is in the write-up notes
- v_max 0.40 with vy_max 0.50 confirmed. `docs/writeup_notes.md` carries the
  clamp-versus-budget table as the evidence that the asymmetry is physical: the box removes at
  most two parts in 100,000 of the commanded vector while the wheel budget removes up to a
  third, so the 0.50 failure is a capability limit. **The binding counts are to be reported
  with the scales**; the counts alone invert the reading.
- Also recorded in the write-up notes: G0's `wheel_edge` as the concrete demonstration of the
  below-the-scan-plane limitation, and the display refusal's 2026-09-26 recurrence with the
  ruled retry policy.

## 2026-09-27 (WP1) - Q5: G0 case six, and preregistration deciding the case list
- **Case six, `raised_housing`** (amendment A11): a slab spanning the robot's side, underside
  at **0.19 m**, top at 0.40 m, approached abeam at the committed 0.10 m/s. The chassis and
  wheels pass beneath it, so a scanner housing is what meets it. Heights from the S300
  read-back of 2026-09-22 -- wheel tops 0.0999, chassis top 0.1764, housings z 0.104-0.257,
  scan plane 0.2207 -- so the underside clears the chassis by 13.6 mm, sits inside the
  housings' span, and the slab **crosses the scan plane**. That last part is the point: this
  obstacle is SEEN, where `wheel_edge` cannot be, so the two cases probe opposite regimes.
- **The slab spans the whole side rather than being centred on a housing's x**, so the case
  does not assume which housing leads; whichever shape above the underside reaches furthest
  abeam is struck, and `expected_shapes` records it from densified mesh vertices as the other
  cases do.
- **Preregistration now decides which cases run.** `declared_cases` reconciles the script's
  case table against `positive_cases`: a case defined in code but not declared is **not run**
  and is reported as pending; a case declared but unbuildable is **fatal**, because passing the
  remainder while a declared case never ran would be a false verdict. Before this, the table
  lived only in the script, so a sixth case would have run the moment it was written and its
  result would have looked like part of a preregistered gate. Case six therefore stays inert
  until A11 and the amended `gates.yaml` are committed.
- **A dry run comes before the gate run.** Whether a housing is really the first shape struck
  is a measurement, not a claim: the stowed arm also stands above 0.19 m and could lead. The
  dry run reports the struck shape per case without driving anything, and the supervisor's
  2026-09-22 standing instruction is to report those shapes before running.

## 2026-09-27 (WP1) - The preregistration takes the layout sets, G2's thresholds and A11
- **Committed by the researcher** (hard rule 1: never edited here). `layout_sets` carries the
  four seed ranges, the identity triple, the locks, the unmapped rates with their count
  distribution and placement rule, and the ruled dropout mechanism; `G2_compute_budget` gains
  `min_steps_per_second: 25` and `max_rendered_episode_fraction: 0.10`; `G0_contact_validation`
  gains `raised_housing` and `case_required_shapes`. A11 is appended to `AMENDMENTS.md`.
- **Verified against the code rather than read:** ranges disjoint with each probe seed naming
  one split (1.5M train, 2.5M tune, 3.5M test, 9.5M pilot); `min_steps_per_second` equal to
  `env.step_budget.aggregate_steps_per_s`; the dropout triple and the unmapped count, size,
  height and margin equal to `configs/base.yaml`. The gate now reconciles to six positive cases
  with nothing pending.
- **`case_required_shapes` was an inert criterion** -- declared in the file and read by nothing,
  which is worse than absent because it reads as enforced. Now checked twice: at placement,
  where the MEASURED shape must satisfy the prefix, so a dry run catches a case that would
  strike the arm instead of a housing before anything drives; and at verdict, against what was
  actually STRUCK. The first is geometry, the second is the run, and both must agree.

### Two ways the preregistration could be damaged silently, now closed
- **Duplicate keys.** Committing A11 appended a second `positive_cases` instead of replacing
  the first, so the block declared five cases and six at once. **PyYAML takes the last without
  a word**, so it loaded as the six we wanted and every test passed: only load order decided
  what the gate's criteria were. `tests/test_preregistration.py` now loads `gates.yaml` through
  a strict loader that refuses duplicate mapping keys.
- **Encoding.** The first attempt to append A11 used `>>` in PowerShell, which writes
  **UTF-16LE**; glued onto a UTF-8 file it left 3,117 NUL bytes, and git reclassified
  `AMENDMENTS.md` as binary, so its history stopped being readable as a diff. Restored from
  HEAD and re-appended as UTF-8. A test now asserts both preregistration files are UTF-8 with
  no NUL bytes. **Standing note: never use `>>` in PowerShell on a UTF-8 file in this repo**;
  `python -c "open(..., 'a', encoding='utf-8')"` or `Add-Content -Encoding UTF8` instead.

## 2026-09-27 (WP1) - The at-rest field was wrong; the strata were never the problem
Supervisor ruling of 2026-09-27, rejecting all three options in the decision request: the
fault was in the field construction, not the aisle. **Hard rule 7 entry: this changes the
certified layer's field equations.**

### Ruling 1: the field must be monotone non-decreasing in speed, in every direction
- **It was not.** The at-rest field carried `margin` (0.10) all round while a moving field
  carried `lateral_margin` (0.05) abeam. Measured over the real footprint before the fix, the
  moving field was **smaller than the at-rest field over ~80 % of bearings at every band, by
  up to 62.6 mm**. The robot was protected *less* the faster it went. What made this visible
  was the narrow-aisle deadlock, but that was a symptom.
- **Fixed:** the at-rest field is now `e(psi) + lateral_margin`, the **infimum** of the moving
  fields. Minimising the moving support over speed and heading sends `D(v) -> 0` and
  `|nu| -> 0`, leaving exactly that, so it is the largest field that keeps monotonicity --
  not a conservative guess.
- **Asserted per bearing and per heading**, on the real footprint and on a disc, across the
  whole band ladder, for the protective and the warning field alike; worst violation now
  **0.000000 mm**. A further test pins that each moving field *touches* the at-rest field
  abeam, so the infimum is exact rather than merely a bound. These exist before the strata
  lock, as ruled.

### Ruling 2: the narrow floor stays at 0.13 m
- The at-rest requirement at the committed cross-track is now **0.10091 m**, identical to the
  moving requirement, where it was 0.15091 m before. 0.13 clears both with buffer.
- How far off-centre a robot may come to rest, by stratum: narrow **79.1 mm**, medium 149.1,
  wide 349.1 -- all above the 50 mm the controller is permitted. Narrow admitted 29.1 mm
  before. **The deadlock is gone.**

### Ruling 3: the restart condition
- Motion resumes only when the **band-0 field along the intended travel direction** is clear.
  The loop passes the requested command's direction; the field and the band still come from
  the estimated velocity, never the command.
- This is what makes the smaller at-rest field safe: a stopped robot **may be approached** to
  the minimum margin, and **may never drive toward** an intrusion. Without it, shrinking the
  at-rest field would have bought the robot the right to set off at something 0.05 m away.
- **The at-rest margin 0.05 is marked VERIFY.** It is close for a person; the number is
  proposed, not measured, and the restart condition is what carries the safety argument.
- **A consequence worth pinning, because it reads like a bug:** "away" is not a free
  direction. The moving field's rear margin is `margin` (0.10), larger than its abeam margin,
  so a robot cannot *reverse* while an intrusion sits within 0.31 m ahead -- it leaves
  **abeam** instead, which is the departure the narrow aisle actually needs. Both are tested.

### Ruling 4: unrecoverable stops are their own outcome
- An episode that exhausts its steps **while a stop is latched and the restart is blocked** is
  `unrecoverable_stop`, not `timeout`. A timeout is a controller that was too slow; an
  unrecoverable stop is a robot that could not legally move again, and pooling them would hide
  the failure the safety layers cause. Collision and success still outrank it.
- No recovery mode is added to the emulator: real recovery is supervised.

### Ruling 6: the standing rule, now hard rule 15
- **Where a derived quantity and a measured one both exist, the parameter is set from the
  measurement and the test asserts against the measurement; the derivation explains the
  number, it never sets it.** Recorded in CLAUDE.md with the worked example: derived
  sector-min term **2.6 mm** against measured total **9.6 mm**, so a slack set from the
  derivation would have been about four times too small. The old rule 15 (gate overlays never
  pin measured constants) becomes rule 16.
- Applied immediately: the 9.5 mm real-scan figure was measured against the OLD at-rest field
  and **may not be carried across or replaced from geometry**. Its test is skipped with that
  reason until D2 is re-run against the real scan packet, which the same ruling requires. The
  purely geometric quantities are re-pinned (sector-min penalty 0.43 mm at 0.12, 0.37 mm at
  0.13; binned margin 68.7 mm at 0.12).

### A test that had been passing for the wrong reason
`test_travel_direction_comes_from_the_velocity_and_survives_a_slow_step` drove along **+x**,
where the heading it was guarding (0 rad) is the same one `atan2(0, 0)` returns, and asserted a
breach the old oversized at-rest field delivered by itself. It tested nothing. Rewritten to
travel along **+y**, so the two headings differ by 90 degrees, and to check the field's depth
after the robot resumes.

## 2026-09-27 (WP1) - The traversability precondition harness
Supervisor 2026-09-26, unblocked by the field correction of 2026-09-27.

- **`pfbnav/world/traversability.py`** (pure) carries `TraverseResult` and the report;
  **`scripts/probe_traversability.py`** drives the simulator. 14 known-answer tests
  (hard rule 6) before anything is measured.
- **Two traverses per stratum**, both at the stratum's derived band top: **offset 0**, an ideal
  centreline tracker whose stop rate is the stratum's **unavoidable-stop floor**, and **offset
  = G1b's committed peak cross-track**, the worst position a controller inside its tolerance
  may occupy. A stratum permanently inside the field at the offset is **inadmissible**.
- **The floor comes only from the zero-offset traverse**, and two ideal traverses for one
  stratum is an error rather than a silent choice of the last: a displaced robot is not the
  best case, and admitting one would make the floor depend on the tracking tolerance instead of
  on the aisle.
- **`stops_above_floor` is deliberately not clamped at zero.** A negative value means a
  controller incurred fewer stops than the ideal centreline tracker, which is possible -- the
  tracker follows the centreline and a controller may take a line the geometry treats more
  kindly. Clamping would present a real result as "no better than the floor". It is reported
  beside the raw count, never instead of it.
- **The criterion is "permanently", not "mostly", and the fraction is never collapsed into the
  boolean.** A stratum 90 % inside the field passes the ruled criterion while being close to
  useless, so `fraction_inside` is always reported. A threshold below 1.0 would be a
  preregistered number, not a default chosen here.
- **A traverse that did not run reports None, not zero**, and is not admissible: a probe that
  failed to start must not read as a clean stratum, for the same reason `stops_per_100m` is
  None before the robot moves.
- **The layer is evaluated but does not override**, as in G0's `env_step_loop` context: a stop
  zeroes the command, the traverse ends, distance goes to zero and stops-per-100-m diverges.
- **The robot is driven open-loop** at the offset rather than tracked to it. The precondition is
  a property of the stratum, not of a controller; G1b measures peak cross-track at 0.3 mm over
  5 m at these speeds, three orders below the 50 mm under test, so nothing a tracker does can
  be mistaken for geometry.
- **The probe uses `PoseDifferenceEstimator`, not a hand-rolled pose difference.** `pose()` is
  world-frame; differencing it directly would hand the emulator a heading rotated by
  `yaw_offset`, select a different band, and measure something an episode never does. Three API
  mismatches in the first draft (`seed_everything`'s module, `YouBotBase.command`, `halt`) were
  caught by the type checker before the script cost anyone a run.

## 2026-09-27 (WP1) - The traversability smoke check, and two defects it exposed
- **Smoke check on the narrow stratum** (`runs/diag/TRAVERSABILITY_20260927T221720Z_seed0`,
  commit `d2be3fb`, clean, pinned scene, display off, PARTIAL as narrowed runs must be).
  Both traverses: **0 steps inside the protective field, 0 stops**. The precondition holds for
  narrow, which is what the corrected at-rest field predicted.
- **Verified against the geometry rather than accepted.** Centred, the near wall is 0.3386 m;
  at the 50 mm offset, 0.2886 m. The moving protective field reaches 0.2595 m abeam and the
  warning field 0.3095 m. So the prediction is: no breach either way, **no warning centred, a
  warning throughout at the offset** -- and that is exactly what the run produced, 0/150 warning
  steps centred against 150/150 at the offset. The probe and the theory agree.

### The pass was one column away from being unfalsifiable
- **The probe recorded no range**, so "0 steps inside the field" read the same whether the aisle
  was clear or the walls had never been built, never rendered, or landed off the scan plane.
  Only the warning column distinguished the two traverses, and only because the offset case
  happened to sit inside the warning field. A centred-only run would have looked identical to a
  probe that saw nothing. This is the failure hard rule 14 exists for.
- **Fixed with a positive control.** Every step now records `min_range_m`, and a traverse whose
  nearest return over the whole drive differs from the wall it built by more than
  `WALL_SEEN_TOL_M` (0.02 m, generous against ~1 % range error and sector-min shortening)
  **raises** rather than returning a clean result. A probe that cannot see its own aisle is not
  evidence about the aisle.

### The traverse was 12 % short and said so nowhere
- The step count came from the COMMANDED speed, but the wheel budget means the realised speed is
  lower (G1b: 0.362 m/s realised at 0.40 commanded), so the smoke check travelled **2.65 m of an
  intended 3.0 m**. The per-100-m rates were unaffected, being normalised, but a "3 m traverse"
  that is 2.65 m is a quiet inaccuracy in the record.
- The loop now drives **until the distance is covered**, with a step cap at three times the
  nominal count so a traverse that is not making progress ends rather than hanging.

### Carried to the supervisor
**A correctly tracking robot in the narrow stratum is inside the WARNING field for 100 % of the
traverse.** The warning field reaches 0.3095 m abeam (`lateral_margin` 0.05 + `warning_extra`
0.05) against the 0.2886 m the aisle leaves at the tracking bound. The unavoidable-stop floor is
0, so this costs nothing in the primary metric, but the secondary near-miss metric will be
saturated in narrow: one entry, 100 % dwell, every episode. Either that is the true statement --
a narrow aisle is permanently a near miss -- or `warning_extra` wants a separate, smaller value
abeam. Both are defensible; it is a ruling, and `warning_extra` is already VERIFY.

## 2026-09-27 (WP1) - The precondition PASSES; and the 9.6 mm scan penalty never existed

### The precondition
- **PASSED on all three strata** (`runs/diag/TRAVERSABILITY_20260927T222532Z_seed0`, commit
  `c084484`, clean, pinned scene, display off, 6 starts, 0 refusals). Every traverse: **0 of 170
  steps inside the protective field, 0 stops, 3.0125 m covered** (the distance fix worked). The
  **unavoidable-stop floor is 0.0 per 100 m in narrow, medium and wide.**
- So the corrected at-rest field holds against the real scan, not only in geometry, and the
  strata are admissible as they stand at the 0.13 m floor.

### What the positive control found, one run after it was added
- The control compares the nearest measured return against the wall the probe built. It passed --
  but only after revealing a **systematic 9.5-10.3 mm shortfall**, consistent across six
  traverses and three clearances from 0.28 m to 0.60 m.
- **It does not scale with distance**, so it is not the ~1 % pixel sampling that had been
  assumed (1 % would be 2.8 mm at 0.28 m and 6.0 mm at 0.60 m). It is a near-constant 10 mm.
- **It is the test walls' half-thickness.** A cuboid 0.02 m thick is placed by its CENTRE and
  presents its surface 10 mm nearer; the scanner measures the surface. Against the wall's **near
  face** the residual is **-0.27 to +0.50 mm, mean +0.08 mm**. The scanner is accurate to better
  than half a millimetre here.

### Consequences, and they reach a standing rule
- **The 0.0096 m "scan penalty" does not exist.** The 2026-09-23 D2 figure -- the tightest bin
  clearing by 9.5 mm where geometry predicted 19.1 mm -- was measuring the wall, not the sensor.
  What the scan actually costs is **about 1 mm**: sub-millimetre range residual plus the derived
  sector-min term (0.43 mm at 0.12, 0.37 mm at 0.13).
- **The narrow floor's slack is no longer explained.** 0.03 was justified as covering 9.6 mm
  three times over; it covers the real ~1 mm about thirty times. **It stays as ruled** -- a wider
  buffer is the safe direction and lowering it is the supervisor's call -- but if it is to be
  derived rather than chosen it needs a different account (layout tolerance, wall placement error
  in generated worlds). Raised with the supervisor.
- **Two tests had passed on the same artefact, each with a tolerance that happened to equal it.**
  `test_the_aisle_is_measured_where_it_was_built` compared against the wall centre with
  `RANGE_TOL_M = 0.01` -- exactly the half-thickness -- so every reading sat a full tolerance
  below its own assertion. Both now compare against the face, with the tolerance cut to 2 mm; the
  probe's control is cut from 20 mm to 5 mm, ten times the measured residual.
- **Hard rule 15 gains its second half.** The rule said the measurement sets the parameter and
  the derivation only explains it. This case is the measurement beating the derivation **and
  being wrong anyway**, because nothing had checked what it was pointing at. The rule now reads:
  *a measurement is only worth that authority if something can show it measured the right thing;
  every probe carries a positive control.* The worked example in CLAUDE.md is rewritten around
  what actually happened rather than around the figure it originally cited.

## 2026-09-28 (WP1) - D2 re-measured against the real scan, in both states
`pytest -m sim tests/test_d2_real_scan.py` on commit `4770b86`: **5 passed**, including the
face-based assertion at the tightened 2 mm tolerance, which had never been checked against the
simulator in that form.

| case (narrow floor 0.13 m) | measured | model | residual |
|---|---|---|---|
| at rest, centred | **69.5 mm** | 68.66 | +0.84 mm |
| top band, centred | **69.5 mm** | 68.66 | +0.84 mm |
| top band, at the 50 mm tracking bound | **19.4 mm** | 16.13 | +3.27 mm |

- **The two centred figures are identical, and that is ruling 1 confirmed by measurement.** The
  at-rest field is the infimum of the moving fields, and abeam the moving field touches it, so a
  centred robot has the same margin standing still as at the top band. The simulator says so
  independently of the geometry that predicts it.
- **69.5 - 19.4 = 50.1 mm**, the committed peak cross-track to within 0.1 mm: the narrow floor's
  own arithmetic, confirmed by the scan rather than by the geometry that set it.
- **The model is conservative, and only that direction matters.** It lands 0.8-3.3 mm BELOW the
  measurement in every case. The gap is not range error -- abeam the model matches to +-0.5 mm --
  but where the minimum sits: the model treats each wall as an infinite line, and its minimum
  wanders to oblique bearings (+34.5 deg in the offset case) where a finite wall seen from
  corner-mounted sensors reads further away than an infinite one. So `binned_radial_margin` is a
  **bound, not a prediction**, and the test now asserts `model <= measured`: a model that
  over-stated the margin could declare a stratum admissible on geometry the scan does not
  support.
- **The skipped test is restored, pinned to the measurement** (hard rule 15), with a second
  assertion that the model-to-measurement gap has not widened beyond the recorded 3.3 mm --
  which would mean the scan or the geometry had moved. The fast suite now has **one** skip left,
  the preregistration freeze.

### A prediction of mine that was wrong, recorded because it nearly misread the result
Before the run I said to expect ~79 mm and that ~69 mm would mean "something is still comparing
to the wall centre". That was backwards: the margin is *measured from the face*, so 69.5 mm is
the correct value and 79 mm is what a zero-thickness wall would give. Had the number come back at
79 mm I would have called the correction confirmed when it was not. The arithmetic that matters
is `face - reach = 0.3286 - 0.2595 = 69.1 mm`, and the run gave 69.5.

## 2026-09-28 (WP1) - Rulings before the freeze: D1-D5 and the restart condition
Supervisor rulings on `docs/reports/decision_request_2026-09-28_before_the_freeze.md`.
**Hard rule 7 entry: the certified layer's field equations change (trailing margin, warning
field).**

### D1 - the narrow slack stays 0.03, as a deliberate buffer
Recorded in `configs/base.yaml` as a chosen number, not a derived one, with the history: its
original motivation, a 9.6 mm scan penalty, was a wall half-thickness artefact, and the real scan
costs about 1 mm. The floor is not reverted. **Positive controls on every probe are endorsed as hard
rule 15's second half.**

### D2 - the warning field gets a separate abeam extension, derived from the geometry
- `warning_extra_abeam: 0.02` (VERIFY) extends the warning field abeam, behind and at rest;
  `warning_extra: 0.05` (VERIFY) still extends it ahead. The warning field's reach **ahead is
  unchanged to the fifth decimal** (0.66629 m at the top band); only abeam and behind shrink.
- **The ruled relation, asserted as a test:** the abeam extension must be strictly less than the
  gap between the protective field's abeam reach and the wall at the 50 mm tracking bound in the
  narrowest aisle. The gap is derived in the test, not restated: **29.09 mm**. 0.02 leaves 9.1 mm,
  and the warning field abeam now reaches 0.2795 m against a wall at 0.2886 m, so it no longer
  saturates. That is a derivation; per hard rule 15 it is to be confirmed by re-running the narrow
  traversability probe, which measured 170/170 warning steps under the old field.
- Validation: the abeam extension must be positive (else the warning field equals the protective
  field abeam and cannot warn first), and the ahead extension may not be smaller than it.

### D3, D4, D5 - endpoints and hypotheses
- `docs/pending/endpoints.yaml` holds the supervisor's consolidated text **exactly**, for the
  researcher to commit as `preregistration/endpoints.yaml` (hard rule 1). Strict-parsed, UTF-8, no
  placeholder. Its note -- "at n = 150 the rule passes only with zero events" -- was checked, not
  assumed: zero in 150 gives an exact one-sided 95 % upper bound of 1.98 %, one in 150 gives 3.12 %.
- D3: exact one-sided 95 % Clopper-Pearson upper bound against 2 %; **150 narrow episodes per
  method, 30 per seed** on the pilot seeds.
- D4: the rule triggers on **any** method in a confirmatory comparison -- **B1-B4, L1-L5**; A1 is
  excluded as exploratory. My draft had listed L1, L2, L4, L5 and A1; superseded.
- **D5, two clarifications ruled now:**
  1. **H1's comparators are B1-B4 and L1-L3.** B5 belongs to H5, and L4 to H2.
  2. **H1 and H1N use intersection-union logic** -- every comparison inside the hypothesis must
     reject at alpha, with no correction within it -- and **Holm is applied only across
     hypotheses**, over {H1, H2}, widened by H1N only if the narrow rule triggers.
- `docs/pending/A12_amendment.md` rewritten to the rulings as made, for the researcher to append.

### The restart condition, changed
- **The moving field's trailing margin is now `lateral_margin` (0.05)**, where it was `margin`
  (0.10): the fore/aft term takes `max(0, nu)` where it took `|nu|`. The field now carries `margin`
  ahead only, `lateral_margin` abeam and behind.
- **Read as a change to the field, not only to the restart check -- and this reading is mine, so it
  is flagged.** The ruling's words admit both. Under the restart-only reading, a robot reversing away
  from an intrusion 0.26-0.31 m ahead would be released by the 0.05 trailing check and then, once
  moving, re-stopped on the next step by the unchanged 0.10 rear of the full field -- every
  `restart_delay`, each time counted as a protective stop in the primary metric. The ruling's own
  instruction to re-run the monotonicity test only makes sense if the protective field changes. So
  the protective field changes, and a test now drives the robot away after release and asserts no
  further stop.
- **Consequence to be aware of:** the protective field behind a MOVING robot shrinks by 50 mm. An
  object 0.26-0.31 m behind the direction of travel no longer triggers a stop.
- **Motion restarts when** (1) the band-0 leading field -- the band-0 field in the half-plane ahead
  of the intended motion -- is clear, and (2) `v . p <= 0` for every point `p` breaching the band-0
  field, an unobserved bin counting as breaching along its bearing (hard rule 5). With "leading" as
  that half-plane, (2) follows from (1); it is computed anyway, as ruled, so the rule stays correct
  if the leading region is ever narrowed. Stopping while moving still uses the full field.
- **One place the implementation is stricter than the ruling's words, deliberately and flagged:**
  the latch also keeps its existing requirement that the field the robot stands in is clear. With
  something inside the minimum margin behind the intended motion, the ruled condition alone would
  release the robot, and on the very next step -- still at rest -- the breach would fire a new stop:
  release, re-stop, every `restart_delay`. Closer than the minimum margin is a real breach, and
  recovery from it is supervised (ruling 4); the episode ends as `unrecoverable_stop`.
- **Monotonicity re-run** on the new field, protective and warning, per bearing and heading:
  worst violation **0.000000 mm**. An independent brute-force Minkowski construction (disc, swept
  segment, a now one-sided margin core, ring) agrees with the support-function field at oblique
  bearings. The two pinned restart tests are updated: leaving abeam still allowed; reversing away
  **flipped** from blocked to allowed.

### Found on the way, before it could cost anything: identity hashes depended on line endings
`generator_sha256` and `scenarios_sha256` hashed raw source bytes. This laptop checks the repository
out with CRLF and a Linux training host would get the committed LF bytes, so `layouts.py` hashed to
`493fda...` here and `76a6b4...` there: a test set locked here would have failed `verify()` on the
training host as though the generator had changed. The existing `scripts/freeze_prereg.py` had the
same bug, wrote Windows backslash paths into the lock (on Linux a single filename, not a path), and
did not record `generator_sha256` as the Q2 ruling requires. All fixed in `96dedfb`: hashes are of
the text with line endings normalised, nothing else; the lock uses POSIX paths and pins both module
hashes and the commit; the freeze refuses a dirty tree and a placeholder. No scenario set had been
written, so nothing was invalidated.

## 2026-09-28 (WP1) - G0 dry run clean; the probe's aisle was 10 mm too narrow
### G0 with case six: the placement is right
`runs/gates/G0_20260928T222256Z_seed0`, `--dry-run`, commit `187728b`, clean, pinned scene.
**`raised_housing` is placed to strike `/youBot/SickS300[1]`**, a scanner housing, satisfying the
preregistered `case_required_shapes` prefix; the stowed arm did not lead. The other five cases are
placed on exactly the shapes they struck in the 2026-09-25 PASS. The one result that could have
forced an amendment to a just-frozen file did not.

### The narrow probe re-run: two surprises, one cause each
`runs/diag/TRAVERSABILITY_20260928T222331Z_seed0`: 0 of 170 steps inside the protective field at
both offsets, floor 0.0 -- but **the warning field was still entered on 170 of 170 steps at the
offset**, where the D2 derivation said 0, and the nearest return fell 4.2 mm short of the wall
face, where the day before it had been within 0.5 mm.

- **The warning fires from step 0, so it is not drift: the aisle is built 10 mm too narrow.** A
  clearance is measured to the aisle's surface, but the probe -- and `tests/test_d2_real_scan.py` --
  placed each wall's CENTRE at the clearance, putting its face 10 mm inside the stratum. The
  warning field reaches 0.2795 m abeam: 0.9 mm past that face (0.2786 m), and 9.1 mm short of
  where a true 0.13 m aisle puts it (0.2886 m). **The D2 derivation was right; the probe was
  measuring a 0.12 m aisle.** This is the wall half-thickness a third time: found in the
  expectation on 2026-09-27, it was still in the construction.
- **The 4.2 mm is open-loop drift.** World x is lateral here; the robot strayed 2.4 mm sideways
  over 3 m on 2026-09-27 and 7.6 mm on 2026-09-28, and the nearest return falls with it. The
  probe's docstring had justified driving open-loop by G1b's 0.3 mm peak cross-track -- but G1b is
  CLOSED-loop, and open-loop is what G1 measured and failed. Drift away from the wall would have
  made the traverse easier than stated, which is the unsafe direction.

### Fixed
- Both harnesses place each wall's **face** at the stratum clearance, its centre half a thickness
  behind.
- The probe **holds its line closed-loop** with G1b's law and committed gains, so "a correctly
  tracking robot at the cross-track bound" is what is measured.
- **Two positive controls** now: the FIRST step's nearest return must match the face as built
  (before any motion can move the robot), and the worst cross-track over the traverse must stay
  within `LINE_HELD_TOL_M` (2 mm, six times G1b's peak). Either failing refuses the traverse.

### What stands, and what must be re-measured
- **Everything measured still holds, a fortiori.** The precondition passed, and D2 measured 69.5 mm
  and 19.4 mm, all in an aisle 10 mm narrower than the floor. The pinned margins are relabelled as
  what they are -- margins of a 0.12 m aisle -- and the model test keeps the geometry of that
  measurement, so it still compares like with like.
- **To re-measure** with the corrected harness: the D2 margins at the true 0.13 floor (a skipped
  test says so; expected about 10 mm larger, but hard rule 15 pins nothing until it is measured),
  and the narrow probe, where the warning field should now clear the wall at the offset.
- **The freeze does not depend on either.** Nothing in `preregistration/` states these margins or
  the warning geometry, and the generator is untouched by the harness fix.

## 2026-09-28 (WP1) - The preregistration is frozen; the warning field measured
- **Frozen** by the researcher at `5b8c9f8`, the lock recording commit `853ce00`: `gates.yaml`,
  `endpoints.yaml` (byte-identical to the supervisor's ruled text) and `AMENDMENTS.md` through A12,
  hashed as text with POSIX paths, plus `generator_sha256` `940c4b5e...` and `scenarios_sha256`
  `654ac61c...`. The three freeze-gated tests now run and pass: files unchanged, no placeholder,
  and the generator is the one the lock pinned.
- **The narrow probe, with the corrected harness** (`TRAVERSABILITY_20260928T223451Z_seed0`, at the
  freeze commit, clean): 0 of 170 steps inside the protective field at either offset, floor 0.0,
  and **0 of 170 warning steps at the offset** -- where the 0.05 extension gave 170/170. D2's
  derivation of the abeam warning extension is now a measurement.
- **Both positive controls passed with room:** the first-step return sat +0.42 mm (centred) and
  +0.36 mm (offset) from the wall face as built, against a 5 mm tolerance; the worst cross-track
  over the traverse was **0.37 mm**, against 2 mm -- the same as G1b's 0.34 mm with the same law,
  and the nearest return over the whole traverse equals the first, so nothing drifted.

## 2026-09-28 (WP1) - D2 pinned at the true narrow floor; the fast suite has no skips left
`pytest -m sim tests/test_d2_real_scan.py -s` at the freeze commit, wall faces at the clearance:
**5 passed**.

| narrow floor 0.13 m | 2026-09-27 (aisle 10 mm too narrow) | **2026-09-28, true floor** | model |
|---|---|---|---|
| at rest, centred | 69.5 mm | **79.5 mm** | 78.71 mm |
| top band, centred | 69.5 mm | **79.5 mm** | 78.71 mm |
| top band, 50 mm offset | 19.4 mm | **29.4 mm** | 28.12 mm |

- **Exactly +10.0 mm in all three cases** -- the wall half-thickness the old harness was losing, now
  measured rather than inferred. 79.5 - 29.4 = 50.1 mm still recovers the committed cross-track.
- **The model stays conservative at the true geometry**: +0.79, +0.79 and +1.28 mm below the
  measurement. The offset gap is smaller than the +3.27 mm seen in the too-narrow aisle.
- Pinned from the measurement (hard rule 15), and the waiting test is removed: the fast suite is
  **884 passed with no skips** -- the three freeze-gated tests now run and pass.

### The display retry, first seen working
The D2 run logged `asked for display_enabled=False ... the instance reports True (attempt 1 of 3)`
and no refusal after it: the setting took on the retry. Under the pre-2026-09-26 client that start
would have counted as a refusal and rendered for its whole run; now it costs one retry and records
none. The first evidence from a real instance that the Q7 ruling's mechanism does what it was for.

## 2026-09-28 (WP1) - G0 PASSES with six cases, under the frozen criteria
`runs/gates/G0_20260928T224313Z_seed0`, `...T224805Z_seed1`, `...T225248Z_seed2`: commit `43aa482`,
clean, pinned scene, both contexts. **The first gate verdict under the frozen preregistration**: each
summary's criteria are identical to the locked `gates.yaml`.

- **42 case-runs, all passed.** Six positive cases at **detection lag 0** in both contexts and every
  seed (limit 1); `parallel_pass_gap_10mm` held 0.0101 m with **0 false contacts**. Every case struck
  the shape it was placed for. Handle-versus-collection validation agreed on all **2,700 steps**.
- **Case six (`raised_housing`, A11) struck `/youBot/SickS300[1]` and nothing else**, in both contexts
  and all three seeds, satisfying `case_required_shapes`. The stowed arm never reached it.
- **What the certified layer would have done, now across both regimes:**
  - `raised_housing` -- an obstacle the scanners SEE (it crosses the 0.221 m plane): the layer would
    have stopped at **step 15**, contact came at step 39. **Prevented, with 24 steps (1.2 s) to
    spare.**
  - `wheel_edge` -- an obstacle the scanners cannot see (0.012 m, below the plane): **never stopped**.
    Unchanged, and still the concrete demonstration of the limitation.
  The two cases between them now show what the safety layers can prevent and what they cannot, from
  the same gate run.
- The other four positive cases: would-stop at step 15, as on 2026-09-25; the negative case at step 2.
  The field changes of 2026-09-27/28 (at-rest infimum, trailing margin) moved none of these.
- Display: 15 starts per invocation, 0 refusals, 0 retries.

## 2026-09-29 (WP1) - The scenario sets: built for, not built; two decisions first
- **`scripts/build_scenario_sets.py`** and the pure **`pfbnav/world/set_plan.py`** (12 known-answer
  tests). Seed ranges and rates come from the frozen preregistration, never restated; each set draws
  a disjoint seed block, allocated over the whole plan so a set's seeds do not depend on which others
  are built alongside it; every set is `verify()`-ed straight after building; only test sets are
  locked; engagement is reported for every set. It refuses: without preregistered sizes, before the
  freeze, with a generator the lock did not pin, and on a dirty tree. `--dry-run` writes under
  `runs/diag/`, unlocked.
- **Nothing generated.** All measurements on the pilot seed range; no test seed touched.

### What the measurements found (pilot seeds)
- **Yield is fine**: 87-100 % per stratum at rate 1.0 over 30-40 seeds (an early 4/6 in narrow was
  small-sample noise). Build time 0.5-2.4 s per scenario.
- **Unmapped obstacles rarely meet the route.** Placed uniformly over free floor before any route
  exists, only **8-15 %** of `unmapped_static` scenarios put any obstacle within 0.3 m of the plan
  (narrow 15 %, medium 8 %, wide 12 %; nearest-obstacle median ~2 m). H1's whole population is
  `unmapped_static`, so ~88 % of it would not test the manipulation the condition is named for.
- **The set sizes are not preregistered anywhere**, and are bounded by compute: at ~600 steps per
  episode and 29 confirmatory episodes per scenario, 100 per cell is 10.6 M steps against a 5-9 M
  budget for ALL evaluation; 50 per cell is 5.3 M.
- **H2's frozen criterion caps its power near 50 % when the true effect equals +15 pp**, at any
  size: the observed difference falls below the threshold half the time. Size buys power only for
  true effects above it (Monte Carlo: 0.96 at +20 pp with 50 per cell).
- Also noted: `scenarios.py` is locked, so even the one conditional draw that makes a seed's
  start and goal differ between rate 0.0 and 1.0 cannot be changed without an amendment. No
  preregistered hypothesis compares across conditions, so independent seeds per condition are used
  and it does not matter.

Put to the supervisor: `docs/reports/decision_request_2026-09-29_scenario_sets.md` -- placement
(recommended: route-conditioned, in every split) and sizes (recommended: 50 per test cell, 20 per
tune stratum, the 30 narrow pilot at `unmapped_static`). Both need A13 and a re-lock.

## 2026-09-29 (WP1) - Rulings on the scenario sets: A13 now, A14 after G2 and the power analysis
### The order, as ruled
1. **A13 now**: route-relative placement and its checks, the tune size, the pilot construction.
2. Dry-run the tune and pilot sets; report engagement and rejection rates per stratum.
3. **G2 on the training host**, on pilot seeds.
4. **The H1 power analysis** (Monte Carlo of the frozen analysis).
5. **A14**: the test-set size, the smallest of {50, 75, 100} per cell giving H1 power >= 0.80,
   subject to throughput. The 5-9 M evaluation figure was the supervisor's rough estimate, not a
   budget; the binding requirement is >= 25 steps/s.

### A13 implemented
- **`pfbnav/world/placement.py`** (pure; 14 known-answer tests). After the route is planned, each
  unmapped obstacle is a square facing the route: offset to its nearest SURFACE uniform on
  [0, 0.6] m, side uniform, position uniform over arc length excluding the first 2 m and the last
  1 m, kept clear of mapped geometry by the preregistered 0.15 m margin, its centre at least 1 m
  from the goal (preregistered). Kept only if the goal is reachable in the true world, mapped plus
  unmapped, at the lethal inflation radius; a refused obstacle is re-sampled, never the layout, and
  a route that cannot hold its obstacles in 200 draws hands over to the next endpoint pair of the
  same layout. The offset RECORDED is measured exactly from polygon to route afterwards, since a
  bending route can come closer than the sampled offset.
- **`scenarios.py`** (the locked module, so the lock moves with A13): the generator now builds the
  MAPPED layout only; placement follows the route; each scenario records its obstacles' polygons,
  the placement draws and the refusals by cause; `layout_sha256` covers mapped plus unmapped. A new
  status, `no_route_admits_its_unmapped_obstacles`. **`world_of`** rebuilds a scenario's full world
  from the regenerated mapped layout plus the recorded obstacles and checks the hash; the
  environment's reset and `verify` both use it, so a locked set is rebuilt from what it records,
  not by re-running the placement. `layouts.py` is untouched: `generator_sha256` does not move.
- **The build script** builds tune and pilot until A14, refuses a config that disagrees with the
  preregistered placement, applies the 90 % validity check (reported in dry runs, refusing the lock
  of a failing test set), and reports rejection rates per set by cause.
- **The pilot**: 150 distinct narrow scenarios; `learned_seed_for(i) = i mod 5`, tested to give 30
  distinct scenarios per seed. Why, as ruled: deterministic baselines on 30 scenarios yield only 30
  episodes, and a learned method's episodes sharing a scenario are clustered, which the
  Clopper-Pearson bound does not allow.
- **`freeze_prereg.py --relock A13`**: a lock can now move, but only if AMENDMENTS.md carries that
  amendment's heading, and the replaced lock is kept inside the new one under `supersedes`.

### One inconsistency in the ruling, put back before A13 is committed
Offsets uniform on [0, 0.6] m for every obstacle engage only **62.5 %** of scenarios
(`1 - (0.5 + 0.25) / 2` with 1-2 obstacles; confirmed by sampling), while the same ruling requires
**>= 90 %**. The check would fail every test set by construction. `first_offset_max` is exposed
(config and draft preregistration, currently 0.6 as ruled); capping the first obstacle at 0.3 m
engages every scenario while the second still spreads offsets to 0.6 m. One line from the
supervisor settles it.

### State of the suite
One test fails, by design: `test_the_generator_is_the_one_the_lock_pinned`, because `scenarios.py`
changed and its hash no longer matches the lock. It clears when A13 is committed and the lock moves
with `--relock A13`. Everything else passes.

H2's power note is in `docs/writeup_notes.md`, as ruled.

## 2026-09-30 - A13's cap on the first obstacle: option (a), with two conditions

**Ruled by the supervisor**, who noted the conflict was theirs: the two parts of the 2026-09-29
ruling had not been checked against each other, and the 62.5 % arithmetic is right.

- **The first obstacle's offset is uniform on [0, 0.3] m**, any further obstacle's on [0, 0.6] m.
  Every scenario is engaged; the 90 % validity check stands. `route_relative.first_offset_max` is
  0.3 in `configs/base.yaml` and `first_obstacle_offset_max_m` 0.3 in the draft preregistration.
- **"First" is an index, not a location.** Its position along the route, side and size are drawn
  exactly as for the second obstacle, independent of the cap. `place_on_route` already drew them
  identically (only the offset's upper limit depends on the index); this is now pinned by a
  known-answer test (same random stream, cap 0.6 vs 0.3: same position, side and size, offset
  exactly halved) and a distribution test (both indices uniform over the window, half left).
  Feasibility refusals can still shift where a close obstacle comes to rest -- a close one is
  refused more often in a tight stretch -- so the build script now reports the mean position along
  the route by index (`position_by_index`). Reported, not corrected.
- **The offset analysis is descriptive and exploratory**, against the per-scenario minimum surface
  offset, and is not a confirmatory endpoint; its distribution is shaped by design. Recorded in the
  draft preregistration (`offset_analysis`) and in `docs/writeup_notes.md`.
- The same rule applies in train, tune and test.

## 2026-10-02 - A13 dry run, and the bypass rule that replaces "reachable in the true world"

**The dry run** (`runs/diag/SCENARIO_SETS_20261002T125110Z_seed0`, commit ba86918, clean): tune
20/20 per stratum and pilot 150/150, all engaged, nothing lost; placement refused 95 % of draws in
narrow layouts, nearly all `too_close_to_mapped`; capped and uncapped obstacles came to rest at
0.59 and 0.62 of route length.

**The finding.** The ruled feasibility test -- the goal reachable in the true world -- was met by
detours through other aisles, which a stack that plans once on the mapped map and never replans
cannot take. Planning in the true world confined to a corridor around the route, at the lethal
radius: 93/150 pilot scenarios (62 %) impassable at +-0.75 m, 43/150 (29 %) at +-1.5 m; positive
control (mapped world, same corridor) 150/150. A retry at the narrow floor gave 71 % but its control
failed in 48/150 (the 0.05 m planner grid closes aisles that sit exactly at the floor) and was
WITHDRAWN. A diagnostic was stopped by Claude past ten minutes (hard rule 11).

**Ruled by the supervisor (option a)**, who noted the reachability test was theirs:
- **Clearance: the narrow floor.** The gap past an obstacle is an aisle and meets the same test:
  0.13 m per side beyond the footprint. The lethal radius (0.03 m) would put the obstacle inside the
  0.05 m abeam protective field -- an unavoidable stop, the geometry-dominated case H1 excludes. The
  field's abeam margin is constant, so the floor admits the bypass at any speed band.
- **W = 1.0 m**, with a fallback declared in advance: 1.5 m only if a tune (20/stratum) or pilot
  (150) set cannot be built in full within the re-sampling cap. Rejections are acceptable;
  re-sample the obstacle, never the layout. Measure W = 0.75, 1.0, 1.5 per stratum first.
- **Conditions:** obstacles judged jointly; where obstacles land reported per stratum (aisle /
  mouth / open floor); the exclusion recorded as a limitation (no scenario needs replanning;
  option (b), global replanning, is the future work).

**Implemented** (`pfbnav/world/bypass.py`, wired into `make_scenario`):
- A grid of cell centres over the corridor at 0.01 m; exact distance to every polygon; a cell is
  admissible at `half_width + narrow min_clearance - resolution/2`; a bypass exists if start and
  goal share an 8-connected component. The radius is READ from `robot.half_width` and the narrow
  stratum, never restated.
- **The one approximation, chosen not ruled (`VERIFY` in the config):** the half-cell tolerance is
  what lets a gap of exactly the floor through at any grid alignment -- without it the narrowest
  mapped aisles fail the control, as the 0.05 m attempt did. Its cost: a gap up to 5 mm per side
  below the floor may be admitted, out of the floor's deliberate 0.03 m buffer. A dense sweep
  (0.115-0.145 m in 1 mm steps, five alignments) pins both bounds.
- **Positive control per route:** the corridor must pass with no unmapped obstacle before any is
  placed; a failing route is set aside and counted (`bypass_control_failures`,
  `BYPASS_CONTROL_FAILED`). It passed on every smoke-test route.
- **Joint:** the callback receives every obstacle placed so far plus the candidate; a test pins two
  individually passable obstacles closing a gap together.
- The rejection cause is now `no_bypass_within_corridor`.
- `build_scenario_sets.py`: `--bypass-width` and `--probe` (dry runs only; probes add 150
  `unmapped_static` scenarios for medium and wide from pilot seeds after `pilot_narrow`'s block,
  which does not move); every set reports the control, built-in-full, and obstacle locations
  (`location_of`: aisle between two rows / mouth within `end_clearance` of an aisle end / open
  floor -- the end strips by the walls are 2.9-3.6 m wide, so "open" is open). A dry run now
  tolerates a scenario module that differs from the lock, recorded in its summary, because a
  revision is measured before its amendment is written. A non-dry build refuses until the bypass
  rule is preregistered.

### The W sweep (2026-10-02): W = 1.0 m stands; the fallback was not needed

Three dry runs at commit 4384969, seed 0, clean (`runs/diag/SCENARIO_SETS_20261002T141652Z_seed0`,
`...T143913Z...`, `...T150119Z...` for W = 0.75, 1.0, 1.5), each with `--probe` (150
`unmapped_static` scenarios for medium and wide from pilot seeds 9000150-9000449).
- **Every set built in full at every W**, none lost to placement, and the positive control set
  aside no route. The ruled fallback to 1.5 m does not apply.
- At W = 1.0: draws per placed obstacle 50 / 35 / 4 (narrow / medium / wide); refusals for no
  bypass 3.6 / 10.8 / 18.6 % of draws; most refusals are `too_close_to_mapped`. At least 12/150
  narrow and 6/150 medium scenarios moved to another endpoint pair (draws > 200 per obstacle).
- **Locations at W = 1.0** (aisle / mouth / open floor): narrow 0 / 3 / 97 %, medium 4 / 17 / 79 %,
  wide 54 / 7 / 39 %. No narrow obstacle CAN sit in an aisle: 0.25 m obstacle + 0.15 m margin +
  a 0.677 m bypass = 1.08 m against the widest narrow aisle's 0.82 m. "An unmapped obstacle in
  the narrow stratum" means one on the open floor of a narrow-aisled layout.
- Capped and uncapped obstacles came to rest alike along the route (W = 1.0: 0.65 / 0.64 narrow,
  0.59 / 0.60 medium, 0.58 / 0.60 wide).

Drafted for the researcher: `docs/pending/A13_revision_bypass.md` (appended to A13, which is the
last amendment, so A13's text carries the rule and its cost without editing committed lines) and
`docs/pending/scenario_sets.yaml` (feasibility text replaced; a `bypass` block whose W the build
script checks against the config).

## 2026-10-02 - Order changed: the H1 power analysis now; its model, ruled; two properties found

**Reordered by the supervisor:** the H1 power analysis now, then SAC, then G2 on the training host
(pilot seeds), then A14. G2 measures throughput with SAC updates running, so it cannot come first.

**Results note, ruled:** stratum and obstacle location are confounded (narrow encounters 97 % open
floor, wide 54 % inside aisles). Per-stratum H1 breakdowns report location alongside; the paper
claims nothing for unmapped obstacles inside narrow aisles. In `docs/writeup_notes.md`.

**The power model, ruled:** comparator rates {2, 5, 10} per 100 m, L5 at 0.7 x; Poisson counts with
mean rate x route length / 100, lengths from the pilot set; shared gamma difficulty (mean 1, CV
{0.5, 1.0, 1.5}) on all arms; learned arms carry a per-seed gamma multiplier (CV 0.3) and are
averaged over 5 seeds, classical arms run once; Pratt's zero handling (to be recorded in A14 as a
clarification of "Wilcoxon signed-rank"); primary scenario all seven at the margin, secondary one at
the margin and six at 2 x L5; the seven comparisons simulated jointly. **Decision rule, declared
before running:** the smallest of {50, 75, 100} per cell with power >= 0.80 at rate 5, CV 1.0, all
seven at the margin; if 100 does not reach it, stop and report.

**Implemented:** `pfbnav/eval/power.py` (pure), `tests/test_power.py`,
`configs/analysis/h1_power.yaml`, `scripts/power_h1.py`. The Wilcoxon is SciPy's (normal
approximation, tie correction, no continuity correction -- SciPy's default at these sizes), pinned to
a hand-worked Pratt example. Alpha and the comparators are read from `endpoints.yaml`, the lengths
from `scenarios/pilot/pilot_narrow.json` (hash recorded). Choices of mine, flagged for the supervisor:
- **Holm level.** H1's level depends on H2 and on whether H1N joins. The decision is evaluated at
  both worst cases, alpha/2 (as the H2 Monte Carlo of 2026-09-29 did) and alpha/3; a disagreement
  is reported, not resolved.
- **The paired detectability floor** is part of H1's success criterion but is defined nowhere in the
  repository. Not modelled; the power reported is an upper bound on H1's success probability.
- **The per-seed multiplier** is read as constant across scenarios (a seed's policy is better or
  worse everywhere).

**Two properties of the frozen analysis under this model**, found by the positive control
(n = 225 pairs, 3000 replicates, rate 5, CV 1.0; scratch probe, the full run records its own):
1. **Shared seed effects inflate the null rejection rate.** The unit is the scenario, but a learned
   arm's seed effect is shared by every scenario, so the test sees these five seeds rather than the
   method. Learned vs learned at EQUAL rates, seed CV 0.3: 27 % rejections in each direction, not
   2.5 %. (Seed CV 0: 2.8 / 2.2 %.)
2. **A single classical run against L5's five-seed average leans against L5.** At equal mean rates
   most single-run counts are zero, the seed average is rarely zero, the paired differences are not
   symmetric about zero, and the signed-rank test -- not a test of means -- reads L5 as worse: 0.0 %
   in L5's favour, 49 % against. At the 30 % margin, power against ONE classical comparator is
   0.09-0.16; H1 needs all four. (Classical vs classical: 2.9 / 2.1 %.)

A 50-replicate smoke run gave power 0.00-0.06 at the decision point for every size: H1 as frozen
very likely cannot reach 0.80 at 100 per cell under this model. Both properties are pinned as
known-answer tests.

### The power run (2026-10-02): STOP AND REPORT -- no size reaches 0.80

`runs/power/H1_POWER_20261002T210848Z_seed0` (commit be2909e, clean, seed 0, 4000 replicates; pilot
lengths hash b69e9981, matching the committed set).
- **Decision point** (rate 5, CV 1.0, all seven at the margin): power 0.004 / 0.006 / 0.015 at
  alpha/2 for 50 / 75 / 100 per cell (alpha/3: 0.003 / 0.005 / 0.011). No size chosen; nothing
  above 100 tried, as ruled.
- **Nowhere in the primary grid** does power reach 0.80: best 0.58 (rate 10, CV 0.5, 100/cell).
  At rate 2 it is 0.000 everywhere: about 0.2 expected stops on a 10 m route.
- **The bottleneck is the four classical comparisons.** At the decision point each learned
  comparison rejects 0.77-0.88; each classical one 0.07-0.14.
- **Under the null** (equal rates, decision point, alpha 0.05): classical vs L5 rejects AGAINST L5
  38 / 49 / 56 % at 50 / 75 / 100 per cell and in L5's favour 0.3 %; learned vs learned rejects
  22-29 % each way. Both GROW with size: they are biases of the test under this model, not noise.
  The first means the frozen test can conclude L5 is worse than a classical baseline that is in
  truth equal.
Reported to the supervisor with options; nothing decided here.

## 2026-10-02 - Rulings on the H1 power report: rate ratio, two-level bootstrap, recalibrated rerun

**Ruled by the supervisor:**
- **Property 1 -> option (a).** H1's estimand becomes the **rate ratio** (total stops / total
  distance per arm), with inference from a **bootstrap CI on its log**, scenarios resampled jointly
  across arms. A14 records that this replaces the signed-rank test; the Pratt ruling is moot.
- **Property 2.** The per-seed multiplier constant across scenarios is the correct reading. The
  confirmatory claim is **method-level**: a **two-level bootstrap** (seeds within learned arms,
  scenarios jointly across arms; classical arms resample scenarios only). The **policy-scoped**
  version (seeds fixed) is reported as secondary. **H2 inherits** the two-level bootstrap on the
  success-rate difference.
- **Option (c) rejected:** evaluation is deterministic, so repeats add nothing; longer routes would
  change the locked scenario distribution.
- **Detectability floor:** defined as the 95th percentile of the effect under within-scenario
  arm-label swaps; redundant in H1's success criterion given a valid test, so **removed from the
  criterion** and kept as a reported quantity. (Implemented with the evaluation code, with its own
  known-answer test.)
- **Holm level for all power calculations: alpha/3.**
- **Rerun:** calibration first (equal rates, each direction, every candidate; a candidate exceeding
  nominal on either side is excluded); candidates T1 (policy-scoped), T2 (two-level, 5 seeds), T3
  (two-level, 10 seeds); seed CV {0.15, 0.3, 0.5}; normal approximation on the log rate ratio for
  the grid, verified against the full bootstrap at a few cells; the 0.5 continuity correction
  declared and checked at equal rates. **Decision rule, declared before running:** T2 if power
  >= 0.80 at rate 5, scenario CV 1.0, seed CV 0.3, all seven at the margin, at some size <= 100;
  else T3 (five more seeds per learned arm in every confirmatory comparison, subject to G2); else
  stop and report. T1 reported in every case.

**Implemented** (`pfbnav/eval/rate_ratio.py`, `tests/test_rate_ratio.py`,
`configs/analysis/h1_rate_ratio.yaml`, `scripts/power_h1_rate_ratio.py`):
- **The normal approximation's variance is the bootstrap's own, in closed form** for the linearised
  statistic. With residuals e_ki = s_ki / (S + 0.5) - d_i / D, col/row their sums over seeds /
  scenarios and t their total: seeds fixed, |col|^2 - t^2/n; seeds resampled,
  |e|^2 + c|col|^2 + (1 - 1/n)(|row|^2 + c t^2) - t^2 with c = 1 - 1/K; covariance through the shared
  scenarios col_a . col_b - t_a t_b / n. Brute-force resampling of the weights matches it to 1 %.
  Against the FULL bootstrap: seeds fixed within 1 %; seeds resampled scatter -2 % to +7 % over six
  datasets (five seeds give 126 distinct resamples and the log curves across them) -- the run's
  bootstrap check reports what that does to rejections.
- **Continuity correction (0.5 on each arm's total), declared:** it is not symmetric when exposure
  differs. L5 summed over 5 seeds against a single classical run, at equal rates: a lean TOWARD L5
  of log((5S + 0.5) / 5S) - log((S + 0.5) / S); -0.174 at S = 2 stops per run. `continuity_bias`
  computes it; the calibration reports the measured mean log rate ratio at equal rates beside it.
- **Choices of Claude's, flagged:** (1) the exclusion tolerance -- nominal + 3 Monte Carlo SEs
  (10 000 calibration replicates; 324 checks, so a bare "> nominal" would exclude a calibrated test
  by noise); (2) calibration also at seed CV 0, because T1's null ("these trained policies are
  equal") holds only there -- at seed CV > 0 and equal METHOD rates T1 rejecting is correct for its
  estimand; (3) data simulated once per cell with 10 seeds, T1/T2 using the first five (common random
  numbers).
- **Smoke run (101 replicates, scratch; noisy):** all three candidates exceeded the tolerance
  somewhere. T2 at rate 10, CV 1.5, seed CV 0.5 rejected ~7 % in L5's favour against nominal 0.8 %:
  a two-level bootstrap over five seeds under-states the seed variance ((K-1)/K, and normal rather
  than K-1-df quantiles). The full run decides; if it confirms, the outcome is "stop and report".

### The rate-ratio rerun (2026-10-02): STOP AND REPORT -- every candidate fails calibration

`runs/power/H1_RR_20261002T220803Z_seed0` (commit 02dcef0, clean, seed 0). Level alpha/3; nominal
0.00833 per side; exclusion tolerance 0.01106 (nominal + 3 MC SE at 10 000 replicates).
- **Calibration at equal rates:** all three candidates excluded. T2 exceeds in 204 of 432 checks
  (worst 0.054), T3 in 126 (worst 0.027); both only at seed CV >= 0.15 and worse as seed CV, rate and
  size grow, i.e. as the seed component dominates the variance. At seed CV 0 both are conservative
  (T2 0.003-0.005 classical, ~0.0001 learned). The seed level is under-stated: resampling K seeds
  gives (K-1)/K of their variance, and normal quantiles ignore that the seed component has K-1
  degrees of freedom. T1 at its own null (seed CV 0): learned 0.008-0.011, classical 0.010-0.016 --
  slightly above tolerance in 22 cells, mostly at rate 2.
- **Power at the decision point** (rate 5, CV 1.0, seed CV 0.3, all at the margin): T2 0.052 /
  0.080 / 0.102, T3 0.085 / 0.159 / 0.220, T1 0.299 / 0.454 / 0.563 at 50 / 75 / 100 per cell.
- **Power is capped by seeds, not scenarios.** With infinitely many scenarios, the seed component
  alone needs K >= 2 (z_{1-alpha/6} + z_{0.8})^2 cv^2 / log(0.7)^2 seeds per learned arm for a learned
  comparison: 3.7 / 14.8 / 41.2 at seed CV 0.15 / 0.3 / 0.5 (half that against a classical arm). At
  seed CV 0.3 neither 5 nor 10 seeds can reach 0.80 at any test size, before any calibration fix.
- **The normal approximation holds:** against the full bootstrap at five cells, per-comparison
  verdicts agree in 92.7-99.9 % of cases and all-seven power differs by at most 0.05.
- **The continuity correction does not bias the equal-rate case** (mean log rate ratio within
  +-0.002 of zero at seed CV 0). The earlier fixed-count prediction of a lean toward L5 was wrong as a
  bias prediction: averaged over Poisson counts, the 0.5 cancels the log's own 1/(2S) downward bias.
Reported to the supervisor; nothing decided here.

## 2026-10-03 (WP4 started early) - SAC: the method-agnostic core, while A14 waits

Started at the researcher's request while the supervisor rules on the H1 power report. Only what
every learned method shares; nothing that a pending ruling or WP2 decides.

**Built:**
- `pfbnav/control/base.py`: the `Controller` protocol every controller presents (reset, act on
  the observation, a normalised action in [-1, 1]^3).
- `pfbnav/control/sac.py`: `make_sac` builds SB3's SAC from the `training:` block and nothing
  else -- every hyperparameter is required, so none falls back to a library default nobody chose;
  `SACController` turns a checkpoint into a deterministic controller (the mean action).
- `pfbnav/training/record.py`: **`EpisodeRecorder`** resolves a real clash -- `NavEnv` puts its
  episode record under `info["episode"]`, which SB3 reads expecting `{r, l, t}` (its logger would
  fail; its `Monitor` would silently overwrite the record). The record moves to
  `info["episode_record"]` with return and length added. **`EpisodeLog`** appends one JSON line per
  finished episode as it happens, and refuses an existing file.
- `scripts/train.py`: one experiment config, one seed, `runs/train/<experiment>_seed<seed>/`
  (refused if it exists: hard rule 10); checkpoints every `training.checkpoint_every`; a summary
  with steps per second. It verifies the training set before use.
- `configs/base.yaml` `training:` gains `train_freq 1` and `gradient_steps 1` (one update per
  step, as docs/requirements.md budgets 7.5M updates), `ent_coef auto`, `net_arch [256, 256]` and
  `device auto` (SB3 defaults, marked VERIFY), `checkpoint_every 25000`, and `scenarios: OPEN`.
- Tests (`tests/test_training.py`, synthetic environment with NavEnv's spaces): every
  hyperparameter required and mapped; the recorder's hand-off; SAC learning with the log's line
  count and timesteps exact; bit-identical learning from the same seed within one process, and a
  different seed differing; a checkpoint as a deterministic controller; the script's refusals.

**What it refuses, and why:**
- **`filter.enabled: true`.** `pfb_filter` is still the WP2 stub and the environment does not call
  it, so L4 and L5 trained now would silently train without the PFB. L3 (L1's checkpoint with the
  PFB applied post hoc) and G2 (which measures the full loop, PFB included) wait on it too.
- **`training.scenarios: OPEN`.** No training set exists; its size and stratum mix are unruled.

**Open, for the supervisor:** the training set (size, stratum mix, whether every method shares
one scenario sequence per seed); L2's definition, which appears nowhere in the repository; and
whether WP2 (the PFB) comes before any training, since L4, L5, L3 and G2 all need it.

## 2026-10-03 - Rulings on the rate-ratio rerun; the DECISION TREE, recorded before measurement

**Ruled by the supervisor:**
- **(a) adopted:** the seed-level variance is scaled by K/(K-1), with t quantiles and
  Welch-Satterthwaite degrees of freedom for learned-vs-learned comparisons. Adoption requires
  passing the same calibration gate (equal rates, at most nominal + 3 MC SE on each side).
  Calibration and power are re-run for corrected T2 and T3, and for **H2** under the same
  two-level corrected analysis; H2's earlier scenario-level power figures no longer apply.
- **(c) the seed CV is measured on the TUNE set, not the pilot:** every learned method's seeds on
  every tune scenario, all strata, fully crossed; seed and scenario fitted as crossed random
  effects; planning uses the **upper 80 % confidence bound**, not the point estimate. The pilot
  keeps its job of counting unrecoverable stops.
- **Order:** corrected calibration and power now; then SAC; then the seed-CV measurement on the
  tune set and G2 on the training host; then apply the tree and write A14.

**The decision tree, as ruled, recorded here before any measurement:**
1. If corrected **T3 with 10 seeds** reaches power >= 0.80 at the middle point at a size <= 100
   per cell, adopt it: **10 seeds for every learned arm in a confirmatory comparison**.
2. Else, if some seed count **<= 20** reaches 0.80 and **G2 shows the budget fits**, adopt that
   count (L1, L2, L4, L5 train it; L3 reuses L1; A1 stays at 5).
3. Else **option (d)**: H1's method-level claim is restricted to **B1-B4**, with L5's seeds raised
   as that comparison requires; the learned comparisons L1-L3 are reported **policy-scoped (T1)**;
   the H1 statement is amended accordingly.

**H2 follows the same tree**; if it reaches branch 3 it becomes policy-scoped.

### The correction, implemented (2026-10-03)

- **Split, exactly:** the uncorrected two-level variance is the scenario part (the policy-scoped
  variance, seeds held) plus each arm's seed part, E_u Var_w =
  |e|^2 + (1 - 1/n)|row|^2 - (|col|^2 + (1 - 1/n) t^2) / K (zero for an arm run once). Tested: the
  parts add to the two-level formula, which brute-force resampling already matched.
- **Corrected interval:** each learned arm's seed part x K/(K-1); t quantile with Welch-Satterthwaite
  df over the scenario part (n - 1 df) and each learned seed part (K - 1 df). **A reading of the
  ruling, flagged:** the ruling names WS df for learned-vs-learned; the same WS formula is applied
  to learned-vs-classical, where it combines the scenario part with ONE seed part (it tends to K - 1
  when the seed part dominates and to n - 1 when the scenario part does). The stricter reading --
  df = K - 1 outright for learned-vs-classical -- would be more conservative when scenario variance
  dominates.
- **Positive control:** learned vs learned at equal rates, seed CV 0.3, 5 seeds, at 0.05: the
  uncorrected interval rejects 0.039 / 0.037 per side, the corrected one 0.015 / 0.015.
- **H2** uses the same machinery on success: the mean's linearisation (s - M)/(K n) is exact, and
  brute-force resampling of a success-rate difference matches it to 1 %. Its model -- L4 success per
  scenario uniform on [0.3, 0.8] (the 2026-09-29 model), L5 + delta, and a per-seed LOGIT shift
  constant across scenarios -- is a choice of Claude's, flagged; the shift's SD is swept over
  {0.15, 0.3, 0.5}. **H2's middle point is not ruled:** at a true +15 pp the ">= +15 pp" criterion
  caps power near 0.5 whatever the design, so it cannot be +15 pp; the run reports +20 pp, SD 0.3
  for orientation only.
- `scripts/power_corrected.py` (config `configs/analysis/power_corrected.yaml`): candidates C5, C10,
  C15, C20 (corrected, seeds per learned arm) and T1; the ruled calibration gate for H1 and H2; the H1
  grid with power for all seven and for the four classical comparisons (branch 3); the H2 grid; and
  the tree evaluated at each swept seed CV for orientation only -- the tree is applied after the tune-
  set measurement, at its upper 80 % bound.
- Smoke run (100 replicates, scratch; calibration meaningless at that size): H1 corrected 20 seeds
  ~0.5 at the middle point at 100 per cell; H2 at +20 pp, corrected 10 seeds ~0.9.

## 2026-10-03 - Rulings on six questions: training set, L2, WP2 order, df, H2 model and middle point

**Ruled by the supervisor:**
1. **Training set:** equal thirds by stratum, ~2,000 scenarios. **Seed index k draws its own
   permutation of the 2,000; every method's seed k uses that same permutation.** Seeds must carry
   all training randomness, including order, or the method-level inference is miscalibrated.
   **No curriculum in v2.**
2. **L2 is SAC-Lagrangian:** cost = protective stops triggered per episode; constraint: expected
   cost per episode <= d, enforced by a Lagrange multiplier with dual ascent and a separate cost
   critic; reward = the unshaped task reward (no proximity term, no stop penalty). d is chosen on
   the TUNE set from {0.05, 0.1, 0.2} under the equal-tuning-budget rule: the best tune success
   among settings meeting their limit. Built on the same SB3 SAC, sharing every hyperparameter;
   **validated first on a toy problem where the constraint is known to bind.**
3. **WP2 (the PFB) is next.** L1 may train in parallel only on a **frozen environment**: the
   environment code hash is recorded in provenance before L1's first seed; WP2 adds the filter only
   behind its enable flag; a test shows filter-off behaviour bit-identical before and after WP2
   within one process. **Any filter-off behaviour change invalidates prior L1 runs.** L1's pilot
   seed runs against **G4, whose criteria are declared first**; then the remaining seeds.
4. **Welch-Satterthwaite against classical comparators** (scenario n-1 and seed K-1 parts) is
   adopted **if it passes calibration across the whole grid**, especially the high-scenario /
   low-seed and low-scenario / high-seed corners; otherwise **fall back to flat K-1**.
5. **H2 model accepted with changes:** the seed logit-SD sweep extended to {0.15, 0.3, 0.5, 1.0} (v1
   seeds were near-bimodal); a **low-baseline condition**, L4 success uniform on [0.05, 0.40],
   alongside [0.3, 0.8]; 0/1 outcomes drawn per seed per scenario (evaluation is deterministic).
6. **H2's middle point:** true effect +20 pp, L4 baseline [0.3, 0.8], seed SD = the measured upper
   80 % bound on the logit scale from the tune-set measurement. **Until then the full grid is
   reported with no decisive cell.**

**Order:** the corrected calibration and power run; freeze the environment and start L1's pilot;
WP2.

**Applied to the corrected run in progress** (`runs/power/CORRECTED_20261003T153838Z_seed0`, commit
e159ee1): its H1 part stands -- it computes the Welch-Satterthwaite rule across the whole grid,
corners included, which is what ruling 4 tests. Its H2 part is superseded by ruling 5 and is rerun
alone with the extended grid (`--parts h2`). The flat K-1 fallback is implemented now, so that if
the H1 calibration fails a corner, the fallback can be run without a further build.

### The corrected run (2026-10-03): WS fails against classical comparators; flat K-1 in force

`runs/power/CORRECTED_20261003T153838Z_seed0` (commit e159ee1, clean, seed 0; level alpha/3,
nominal 0.00833 per side, gate 0.01106). Its H2 part is superseded by ruling 5 and not reviewed.
- **Learned vs learned (Welch-Satterthwaite, as ruled) PASSES** for every corrected candidate:
  worst 0.0100 (C15), C5 0.0079, C10 0.0089, C20 0.0092, both sides.
- **Learned vs classical FAILS**, on L5's side only, at the low-scenario / high-seed corner the
  supervisor named (rate 10, seed CV 0.5): C5 22 of 108 cells over (worst 0.0154), C10 6 (0.0122),
  C15 5 (0.0137), C20 4 (0.0125). The high-scenario / low-seed corner (rate 2, CV 1.5, seed CV 0 and
  0.15) passes, conservatively: 0.003-0.011. The opposite side never exceeds 0.0091.
- **Ruling 4 applied:** flat K-1 for learned vs classical (`classical_df: flat_k_minus_1`). Caveat,
  stated before the rerun: where the seed part dominates, WS df already approaches K-1, so at K = 20
  the two rules are close in exactly the failing corner; the rerun's calibration decides.
- **Power at the middle point is short of 0.80 under every branch, and seeds do not limit it.**
  At rate 5, 100 per cell: seed CV 0.15 -- C10 0.50, C20 0.73 (all seven and classical-only alike),
  T1 0.69; seed CV 0.3 -- C10 0.14, C20 0.43 (classical-only 0.52), T1 0.57. T1 carries no seed
  variance in its inference and still reaches only 0.57-0.69, so no seed count lifts the middle
  point to 0.80: the limit is the information per scenario (~0.5 expected stops on a ~10 m route
  at rate 5) and the 100-per-cell cap. At rate 10, seed CV 0.15, C10 reaches 0.86 and C20 0.98. The
  true stop rate matters as much as the seed CV. Reported to the supervisor.

## 2026-10-03 - The decision tree EXTENDED (branch 3 below 0.80), recorded before any measurement

**Ruled by the supervisor**, answering how the tree treats a branch-3 outcome below 0.80. Recorded
here before the tune-set measurement or any training run.

**Step 0 -- plan on measured inputs at conservative bounds:**
- **stop rate** at its **lower** 80 % bound; **seed CV** at its **upper** 80 % bound;
- **comparator configuration** from the measured inter-comparator gaps, each at the bound
  **nearer the margin**;
- the effect of interest stays a **30 % reduction against the closest comparator**; **L5's own
  measured effect is never used as a power input**;
- the measurement runs on a **disposable tune-unmapped set**: tune seed range, unmapped rate 1.0,
  20 per stratum, logged as a tune regeneration -- because the 0.25 tune rate dilutes the stop
  rate relative to the H1 population (unmapped_static).

**Branches 1-3 unchanged**, evaluated on Step 0's inputs:
1. corrected T3 (10 seeds) >= 0.80 at <= 100 per cell -> 10 seeds for every learned arm in a
   confirmatory comparison;
2. else some seed count <= 20 reaching 0.80, if G2 shows the budget fits -> that count (L1, L2,
   L4, L5 train it; L3 reuses L1; A1 stays at 5);
3. else option (d): H1's method-level claim restricted to B1-B4, L5's seeds raised as that
   comparison requires, L1-L3 reported policy-scoped (T1), H1 statement amended.

**Branch 4:** if branch 3 is below 0.80 at 100 per cell, extend sizes to **{150, 200} per cell
for the three unmapped_static cells only** (mapped_only stays at H2's requirement); adopt the
smallest reaching 0.80, **subject to G2 showing training plus evaluation fits six weeks**.

**Branch 5:** if nothing affordable reaches 0.80: **keep H1, its 30 % effect and the analysis
unchanged**; preregister the **achieved power** in A14; declare that a **non-significant result
is inconclusive**; report the **rate ratio with its CI as the primary result**, with the power
stated beside the verdict. **Never enlarge the effect size to obtain power.**

**Calibration fallbacks:** flat K-1 is in force (learned vs classical). If it fails at the
low-scenario / high-seed corner, the next fallback is **seeds as units**: per-seed log rate ratios
against the classical arm over the same scenarios, a one-sample t with K-1 df. **If that fails,
stop and report.**

**Then:** the environment freeze, L1's pilot against G4, WP2.

## 2026-10-03 - The environment frozen for L1; the training set and its per-seed order

Per the supervisor's ruling 3 (L1 trains in parallel with WP2 only on a frozen environment) and
ruling 1 (the training set).

- **The PFB's slot added BEFORE the freeze** (`pfbnav/envs/step_loop.py`, `pfb_stage`): beneath the
  policy and above the certified layer. Off, it returns the command ARRAY ITSELF -- no copy, no
  arithmetic -- and the filter is never called; WP2 fills only the "on" branch. Adding the slot
  after L1 had trained would itself have changed the environment. The loop now refuses to be built
  with `filter.enabled: true` (before any I/O) until WP2 exists: a loop "with the filter" that ran
  without it would train or evaluate a method under its name. The certified layer's
  `intended_direction` and `override` now take the slot's output, which off is the same object.
- **The environment's identity** (`pfbnav/envs/freeze.py`): the import closure of
  `pfbnav.envs.nav_env` (27 files today: envs, sim, world, planning, certified, footprint table,
  step budget, hashing) plus the Lua scanner and the footprint table the config names, hashed as
  text with line endings normalised. Defined by the closure, not a list, so when WP2 wires
  `pfb.py` into the step loop the identity moves on its own. **Pinned** in `configs/base.yaml`
  `env.frozen_code_sha256` = d61a536e...; `tests/test_freeze.py` fails on drift and
  `scripts/train.py` refuses any other identity, recording it in `environment.json` per run.
- **The before/after test** (`tests/test_step_loop_sim.py`, simulator): the same 60 commands run
  twice in one process, through the loop as built and with the slot bypassed (the pre-WP2 path);
  every pose, velocity, applied command, scan and certified flag must match bit for bit. Positive
  control: a sequence differing in one command must NOT match. **Re-freezing** (WP2 will need it)
  requires this test to pass on the simulator and a decision-log entry; a filter-off behaviour
  change invalidates prior L1 runs.
- **The training set:** `scripts/build_scenario_sets.py` plans `train_{narrow,medium,wide}` (667
  each, train seed range, rate 0.25) once `scenario_sets.train` is preregistered, and
  `tune_unmapped_{...}` (20 each, tune seeds, rate 1.0, after the tune blocks) for Step 0. Drafts:
  `docs/pending/scenario_sets.yaml` and `docs/pending/A13_revision_training_set.md`.
- **The per-seed order** (`pfbnav/training/order.py`): seed k's permutation of the combined set is a
  function of k alone, from its own named stream; every method's seed k uses it; its hash is
  recorded per run. No curriculum.
- **G4 drafted** for the supervisor (`docs/pending/G4_draft.md`): integrity, "it learned", and a
  deterministic tune-set check, thresholds PROPOSED. It needs an evaluation runner that does not
  exist yet.

### The flat-K-1 H1 rerun and the extended H2 grid (2026-10-03): both calibrate

Both at commit 0329941, clean, seed 0; level alpha/3, gate 0.01106 per side.
`runs/power/CORRECTED_20261003T181435Z_seed0` holds only config and provenance (a start that did
not compute); nothing to review, left in place (hard rule 1).

**H1, flat K-1 against classical** (`CORRECTED_20261003T181634Z`): **every corrected candidate
passes**, so flat K-1 is adopted for learned vs classical and the seeds-as-units fallback is not
needed. Worst per candidate: C5 0.0079, C10 0.0089, C15 0.0106, C20 0.0099. The corner that failed
under WS (rate 10, CV 0.5, seed CV 0.5) is now 0.0036-0.0099 on L5's side. T1 at its own null (seed
CV 0): learned 0.0118, classical up to 0.0153 at rate 2 -- slightly above the gate in sparse cells;
T1 is secondary and reported only.
- **Power at the middle point** (rate 5, CV 1.0, all at the margin), 50 / 75 / 100 per cell, all
  seven / classical only: seed CV 0.15 -- C10 0.04-0.30 / 0.05-0.32, C20 0.15-0.62 / same;
  seed CV 0.3 -- C10 0.02-0.10 / 0.04-0.18, C20 0.10-0.37 / 0.12-0.44; T1 0.30-0.57 / 0.35-0.66.
  Flat K-1 costs power against WS (C20 0.37 vs 0.43 at seed CV 0.3, 100/cell). At rate 10, seed CV
  0.15: C20 0.97, C10 0.76. **Branches 1-3 do not reach 0.80 at <= 100 per cell at the middle
  point; branch 4 (150, 200) is outside this grid** and is evaluated on Step 0's measured inputs.

**H2, the ruled grid** (`CORRECTED_20261003T181806Z`; both baselines, seed SD to 1.0): **every
corrected candidate passes** (worst 0.0082); T1 at its own null 0.0099. At +20 pp, power is nearly
independent of size -- the seeds limit it -- and the two baselines agree within 0.03:
- seed SD 0.15: C10 0.98-1.00; 0.3: C10 0.92-0.95; 0.5: C10 0.79-0.83, C15 0.87-0.89; 1.0: C10
  0.30-0.34, C20 0.64-0.69, T1 0.60-0.62.
- So H2 reaches branch 1 (10 seeds) unless the measured seed SD is near 0.5 or above; at 1.0 no
  count <= 20 reaches 0.80 and H2 would go to branch 3 (policy-scoped). At +15 pp power is ~0.48 at
  any size, as "Reading H2" says. No decisive cell until the tune-set measurement (ruled).

## 2026-10-03 - Ruled: calibration settled; H2 stays method-level; one seed count for L5 and L4

**Recorded, as ruled by the supervisor:**
- **Flat K-1 against classical comparators adopted** (passes the whole grid, conservative at the
  corner); **Welch-Satterthwaite adopted for learned vs learned**; **seeds-as-units declared but
  not needed**.
- **H1 proceeds to branch 4 on Step 0's measured inputs.**

**Ruled, recorded before any measurement, revising the earlier H2 ruling:** H2's confirmatory test
**stays method-level at the chosen seed count**, with its **achieved power preregistered** and
**branch 5's interpretation applying** (a non-significant result is inconclusive). The
**policy-scoped version is reported as secondary, never substituted.** Reason: with high seed
variance, a claim about specific trained policies says little about the method, and narrowing the
scope to pass is what branch 5 forbids. (This replaces "if H2 reaches branch 3 it becomes
policy-scoped".)

**Seed count:** decided **once for L5**, as the **larger of what H1 and H2 require, capped at 20,
subject to G2**; **L4 trains at the same count**. If both hypotheses end in branch 5, use the
**largest count the budget affords**.

**Next:** the environment freeze, L1's pilot against G4, then WP2.

### The freeze confirmed on the simulator (2026-10-03)

`pytest -m sim tests/test_step_loop_sim.py -k bit_identical`, run by the researcher against a live
instance (Python 3.13.13, pytest 9.1.1): **1 passed** in 35 s. The loop as built (PFB slot present,
filter off) and the loop with the slot bypassed produced identical poses, velocities, applied
commands, scans and certified flags over 60 commands in one process, and the positive control (one
changed command) did not match. The environment L1 trains on is frozen at
`env.frozen_code_sha256` d61a536e....

## 2026-10-03 - The evaluation runner (first use: G4's checkpoint check)

- `pfbnav/eval/runner.py`: `run_episodes` runs any `Controller` once on each scenario index of any
  environment that ends an episode with `info["episode"]`, in one process, failing loudly on an
  episode without an outcome; `outcome_summary` gives outcome counts and shares, the contact rate
  (any contacted step, not only a "collision" ending) and the **pooled** stop rate -- total stops /
  total distance x 100, the rate-ratio estimand's numerator and denominator, never an average of
  per-episode rates. Known-answer test: 3 stops over 20 m = 15 per 100 m, where the per-episode mean
  would give 9.64.
- `scripts/evaluate.py`: one checkpoint (deterministic) over named sets; refuses `scenarios/test/`
  (hard rule 2), an environment that is not the frozen one, `filter.enabled` and unverifiable sets;
  records the checkpoint's SHA-256 and the environment identity; writes episodes as they finish and
  a summary overall, per set and per stratum.
- The freeze check is one function now (`freeze.freeze_problem`), used by training and evaluation.

## 2026-10-03 - G4 ruled (amendment A15); the controls, B1 and the gate built; a standing rule

**Ruled by the supervisor:** G4 is a generic gate on every learned method's pilot seed (L1, L2,
L4, L5, A1). Criterion 1 (integrity) as drafted; criterion 2: last-10 % training success >= 0.20
AND >= +0.15 over the first 10 %; criterion 3, the 60 tune scenarios at rate 0.25, deterministic
policy: success >= 0.30, contact <= 10 % (the "no outcome above 90 %" criterion dropped: it fails
excellent policies, and the success floor already excludes standing still); criterion 4 recorded
as drafted plus per-stratum success. **Controls before any verdict:** zero action must FAIL
criterion 3 (all timeouts, no successes); holonomic Pure Pursuit must PASS it; otherwise the gate is
broken. Failure handling: integrity -> fix, fresh pilot seed, failed run kept; learning/checkpoint
with integrity intact -> one second pilot seed, unchanged (pass: G4 passes, first failure recorded
for the seed-CV discussion; fail: stop and report); a hyperparameter change after a failure applies
to every learned method and restarts the pilot. **Standing rule** (CLAUDE.md hard rule 17): pilot
seeds only from the pilot range, never in H1/H2; every confirmatory seed trains unconditionally and
stays in the analysis.

**Built:**
- Drafts for the researcher: `docs/pending/G4_gates_block.yaml` (appended to `gates.yaml`; strict-
  parses with it) and `docs/pending/A15_G4.md`. **Numbered A15**, because the decision log reserves
  A14 for the test size (ruled 2026-09-29); flagged. Pilot training seeds: 9000000 and 9000001.
- `pfbnav/eval/g4.py` (pure): learning, checkpoint, integrity, controls and the verdict, controls
  first; known-answer tests at and either side of every threshold.
- `pfbnav/control/pure_pursuit.py`: **B1, holonomic Pure Pursuit on the observation alone** --
  translate toward the lookahead point at `cruise_speed`, slow linearly inside `goal_slowdown`,
  turn to face the point at `k_omega` -- and the zero-action control. `configs/base.yaml`
  `baselines.pure_pursuit` (0.25 m/s, 1.5 /s, 0.5 m; UNTUNED, VERIFY). Not part of the
  environment's identity.
- `scripts/evaluate.py` takes `--controller sac | pure_pursuit | zero`.
- `scripts/train.py` writes the learner's losses as CSV (`sb3/progress.csv`) for the non-finite
  check and records any exception (re-raised) in its summary.
- `scripts/gate_g4.py`: reads the training run, the pilot's checkpoint evaluation and the two
  control runs; refuses evaluations on other sets or another environment, and a training seed
  outside G4's pilot seeds; thresholds from `gates.yaml` only.

## 2026-10-09 - G4's controls: zero action behaves; Pure Pursuit FAILS -- the gate is BROKEN

Both at commit 8f232f1, clean, seed 0, rendering on, frozen environment d61a536e..., G4's three
tune sets (60 scenarios).
- **Zero action** (`runs/eval/zero_action_20261005T200808Z_seed0`): 60/60 timeouts, 0 successes,
  0 contact, 0.12 m moved in all. Behaves as required.
- **Pure Pursuit, B1** (`runs/eval/B1_pure_pursuit_20261006T074216Z_seed0`): **0 successes**;
  50 unrecoverable stops, 8 collisions, 2 timeouts; contact 13 %; 46 stops per 100 m; 3.4 m moved
  per episode on average. **G4 requires it to pass; per the ruling the gate is BROKEN and no pilot
  verdict may be read.**
- Patterns: stuck episodes mostly show exactly 2 stops -- the first released, the second never --
  in MAPPED worlds (most have no unmapped obstacle). **7 of 8 collisions on step 1, 0 m moved**,
  where zero action on the same scenarios had none: the first command itself makes contact.
  29 of 60 plans pass a mapped surface closer than the at-rest abeam field
  (`path_min_clearance_m` < half_width + lateral_margin = 0.2586 m; the planner's lethal radius is
  0.2386), but stuck episodes occur on both sides of that line, so it is not the whole story.
- Hypotheses, not findings: (a) the planner's 0.03 m lethal margin against the field's 0.05 m
  abeam margin -- a plan tracked exactly breaches the at-rest field near shelf corners, and a
  latched stop cannot clear where the field it stands in is breached; (b) Pure Pursuit's turning
  (k_omega 1.5) swings the long axis across a narrow passage; (c) something on the first step that
  makes contact, possibly the turn itself.
- **Diagnostic** `scripts/diag_episode_trace.py`: per-step pose, commands, the certified layer's
  report and contact names for chosen scenarios, capturing the step loop's state in-process only
  (the frozen environment is untouched and checked). Traces requested for narrow tune indices 2,
  3, 6 and 15.

### Found 2026-10-09 (the researcher, watching the rendered control): halls extend off the floor

The scene's floor is nine 5 m ResizableFloor tiles, x and y in [-7.5, 7.5] m
(`docs/scene/inventory_after_camera.json`). Each episode `transform_placing` moves and rotates the
14 x 10 m layout so the scenario's start lands on the robot, which stands at the origin facing +y.
So a hall can reach up to its diagonal (17.2 m) from the origin, and does: **every one of the 60
tune halls extends off the floor (4-38 % of its area)**; across every built set (tune, pilot,
train) the hall reaches up to **14.22 m**. Nothing checked this: every probe and gate ran near the
origin.

**Effect on G4's Pure Pursuit control:** 11 of the 60 episodes stopped within 0.5 m of the floor
edge, most at world y = +7.2 to +7.3 (the robot drives a long aisle along +y and stops where its
front meets the edge; locations estimated by mapping distance travelled onto the plan). A second,
separate cause stops it mid-floor, early (indices 3, 15, 22, 27, 31, 35: 0.07-2 m travelled, 6+ m
from any edge), and the step-1 collisions are unexplained; the per-step traces are still needed.

**Fix options, for the supervisor:** (A) enlarge the floor to cover any placement -- at least
2 x 17.2 = 34.4 m square, i.e. 35 x 35 m (7 x 7 tiles) -- a scene edit under hard rule 12 (alone,
read back, `sim.scene_sha256` updated), the environment code untouched; or (B) teleport the robot
to the scenario start with the hall fixed on the floor -- an environment-code change, a re-freeze,
and against the "nothing is teleported" design. Recommended: (A). Either way the controls rerun,
and nothing trained before the fix is used.


### Ruled 2026-10-10 (supervisor): the floor, the carry-over check, the second cause, a standing rule

Recorded before any measurement under it.

1. **Option A, 35 x 35 m.** The nine existing 5 m tiles stay byte-for-byte; forty more are added
   around them (7 x 7, a tile still centred on the origin). **Not 40 x 40:** an even count puts a
   seam under the origin and changes the start's contact geometry. A scene-only edit under hard
   rule 12; `sim.scene_sha256` re-pinned; `sim.floor_half_extent_m: 17.5` added. The guard test
   is approved, extended to (a) the test sets once built and (b) every unmapped obstacle, not only
   the hall bounds. Confirm that no training run predates the fix.
2. **Before carry-over**, in one process, old scene then new: G1b `translate_while_rotating` at
   0.40, G0 `side_wall_lateral`, and one traversability probe run. Bit-identical -> G0, G1b and
   traversability carry over; any difference -> stop and report.
3. **The second cause.** Zero action had no contacts, so it is not spawn overlap; the first
   command makes the contact. Hypothesis: a heading mismatch in hall placement (cf. the earlier
   youBot_ref 90-degree offset). The per-step traces must report at step 0 the measured pose
   against the scenario's start pose (position and yaw), the route's first segment bearing in the
   body frame, the first commanded twist, and the struck shape. Check whether the step-0 yaw error
   clusters near +-90 or 180 degrees, and whether the six early mid-floor stops share the cause.
4. **Standing rule, added to CLAUDE.md (hard rule 18):** tests and probes cover the whole
   operating envelope, including the world's spatial extent, not only the region a probe was built
   for.
5. Then rerun both G4 controls on the new scene.

**No training run predates the fix:** `runs/train/` does not exist in this repository
(2026-10-10). The L1 pilot has been on hold since the controls failed, and no Run Request for
`scripts/train.py` has been issued.

### Found 2026-10-10, from the recorded episodes: every hall after the first was placed around the previous episode's end pose

Found while preparing the step-0 trace of item 3 above, **from data already on disk**, before any
new run. Every episode record carries the transform its hall was placed with
(`shapes.transform`). Inverting it at the scenario's start gives the world pose the environment
*assumed* the robot stood at:

- **Zero action (60 episodes): always (0.000, 0.000) at +90.05 deg.** The robot's scene pose.
- **Pure Pursuit: (0, 0) at +90.06 deg for episode 0 only.** For every later episode the assumed
  pose is where the robot ENDED the previous episode: episode 1 assumed y = +5.764 m (episode 0
  travelled 5.77 m along +y), episode 2 y = +3.710 (episode 1: 3.71 m), episode 3 y = +7.160
  (episode 2: 7.16 m), episode 16 y = +0.067 (episode 15: 0.07 m), and so on for all 59.
  Assumed headings ran 84.5 to 98.2 deg, so **the yaw error is within +-8 deg; it does not cluster
  near +-90 or 180.** The error is in position, up to 7.2 m.

**Mechanism.** `NavEnv.reset` read the robot's pose (`_robot_world_pose_when_stopped`) and built
the transform from it **before** `StepLoop.reset` stopped and restarted the simulation. After the
first episode the simulation is still running at that point, so the read returns the end pose of
the episode just finished; the restart then returns the robot to the origin, and the hall is built
around a pose the robot no longer occupies. The robot starts somewhere else in its hall: beside or
inside a shelf (the step-1 collisions, although the scenario starts lie 0.39 to 1.82 m from every
layout surface), or facing a wall (the early stops). Offline, from the scenario files, the routes'
first segments lie dead ahead of the start heading in 54 of 60 scenarios (the other 6 within
+-52 deg), so the scenarios themselves are not mis-oriented.

**Why nothing caught it.** Zero action never moves, so every one of its placements was right; as
the gate's negative control it could not see this. The sim tests restart the simulator between
tests, and the one test that resets twice moves the robot 5 mm in between. No check compared the
robot's measured pose at step 0 with the scenario's start: the comparison item 3 asks for.

**Consequences.** The G4 Pure Pursuit control (B1_pure_pursuit_20261006T074216Z) is invalid as a
measurement of the controller: 59 of its 60 episodes ran in mis-placed halls. The floor-edge
reading from 2026-10-09 is partly confounded too: the robot always drove from the origin, and the
assumed end poses show it stopping at most at y = +7.16, where its front meets the 7.5 m floor
edge, which supports the floor finding; but which hall surface was where is unknown for those
episodes. The overhang itself is placement geometry and is unaffected: the floor fix stands. No
training run exists, so nothing trained is affected. Every multi-episode use of `NavEnv`
(training, evaluation, the trace) carried the bug; the gates and probes do not use `NavEnv`.

**Fix (environment code, so a re-freeze; for the supervisor's acknowledgement).**
- The transform is built inside `after_start`, from the pose read **after** the restart: where
  the robot actually stands when the episode begins.
- **Every reset now checks it** (the step-0 comparison of item 3, built into the environment):
  the measured pose at step 0, mapped into the layout, must equal the scenario's start within
  `env.start_pose_tolerance_m` / `_rad` (0.01 m, 0.01 rad), or the reset raises. The errors go
  into every episode record (`start_position_error_m`, `start_yaw_error_rad`).
- A sim test drives the robot more than 0.5 m, resets, and asserts the start error. Its positive
  control is the distance driven, so it cannot pass with the robot still at the origin.
- `env.frozen_code_sha256` is re-pinned; the filter-off bit-identity test reruns on the
  simulator as the freeze requires.

### Found 2026-10-10, preparing the floor edit: a perimeter wall stands inside the 35 m floor

The scene holds an object no entry in this log mentions: `/Wall` (a dummy at (0.975, -0.675)
carrying a script, `/Wall/Script`) with one shape, `/Wall/Cuboid`, whose frame is at
(7.55, 0, 0.125) and whose bounding box is 15.2 x 15.2 x 0.25 m (both pinned inventories,
`docs/scene/inventory_pinned.json` and `inventory_after_camera.json`). The likeliest reading is
a grouped perimeter wall, 0.1 m thick and **0.25 m tall, around +-7.6 m**: the edge of today's
floor. A shape whose box covered the origin solidly would put the robot in contact at every
start, and zero action recorded none. 0.25 m is above the scan plane (scanners at z = 0.18 m),
so the scanners would see it.

**What it would mean, if confirmed:**
- **It, rather than the floor's edge, is likely what stopped the robot at y = +7.16 m:** the
  front scanner's field meeting a wall 0.3 m ahead. The floor-edge reading of 2026-10-09 needs
  this correction.
- **It would stand inside the 35 m floor and cut through most halls.** Enlarging the floor alone
  would leave the halls' overhanging parts on floor and the wall running across them: an
  obstacle no scenario records, mapped or unmapped.
- No gate or probe used it: G0 and traversability build their own obstacles, G1b drives 5 m, and
  at the origin the wall is beyond the scanners' 5 m range.

**Not acted on.** The ruling covers the floor only. `scripts/scene_extend_floor.py --dry-run`
reports every shape standing over the 35 m square: plan extent, height, flags, whether it covers
the origin, and the scripts beside it. The edit itself **refuses while any such shape remains**,
so the floor cannot be enlarged with the wall still in place. **For the supervisor:** remove
`/Wall` (the halls carry their own perimeter walls), or move it outside +-17.5 m, or something
else. `tests/test_floor_sim.py::test_nothing_but_the_robot_and_the_floor_stands_on_the_floor`
fails until it is resolved.


### Reviewed 2026-10-13: the step-0 fix holds; the early stops are geometry, not placement; the wall confirmed; the floor is not collidable

All runs at `377647c`, clean.

**The re-freeze, on the simulator.** `tests/test_step_loop_sim.py` 8 passed, the filter-off
bit-identity test among them, as the freeze requires.

**The stale-pose fix, in 16 traced episodes** (`runs/diag/TRACE_20261013T215044Z`, `..220231Z`,
`..221601Z`; the 7 step-1 collisions and the 6 early mid-floor stops of the void control, plus
narrow 2 and medium 1, each reset following an episode that moved): **step-0 error 0.0 mm and
0.00 deg in all 16, no contact at reset, no contact at all.** The step-1 collisions were the
stale pose, all of them. The route's first segment lay within +-2.3 deg of the heading in every
episode.

**But 14 of the 16 still end `unrecoverable_stop` (2 timeouts), and none reaches its goal.** The
early stops are a second, independent mechanism, read from the traces and replayed offline
against the layouts (the field geometry is the emulator's own `FieldGeometry`; the scan is
replaced by ray casts against the generator's polygons):
- **The trigger.** Pure Pursuit cruises at 0.25 m/s, so the band-0.30 moving field. In all 16
  first stops that field is breached by a shelf by **3 to 41 mm**, in 14 of them at **+-32 to 42
  deg**, where the front corners point (the moving field reaches 0.45-0.48 m there). The robot
  passes a shelf end or face with its centre ~0.25-0.30 m off the face. One (narrow 14) is the
  unmapped obstacle, as designed.
- **Why it is permanent.** At rest the at-rest field is clear in all 16, but the restart (ruled
  2026-09-28) needs the band-0 LEADING field along the commanded direction to be clear, and Pure
  Pursuit only ever commands toward the lookahead point, i.e. forward past the shelf. Restart
  blocked until timeout: 0 restarts in 1,373-1,460 stopped steps.
- **Why it happens so often.** (1) The planner lets the path come within the lethal radius
  `half_width + lethal_margin` = 0.2386 m of a surface, which is inside the at-rest field abeam
  (`half_width + lateral_margin` = 0.2586 m) -- hypothesis (a) of 2026-10-09, now confirmed by a
  stop at 0.264 m abeam (wide 17). (2) Pure Pursuit steers straight at a point 0.8 m along the
  path, so it cuts every bend toward its inside shelf (heading 4-12 deg off the path at the stops).
- **Slowing down would not cure it.** Replayed against the band-0.10 field, 4 of the 16 stop poses
  are still breached and the rest clear by at most 48 mm (most by under 20).
- **Not the floor and not the wall:** every one of these stops is within 6.6 m of the origin,
  inside today's floor.
- **The same mechanism shows in the sim suite**, independently:
  `test_following_the_plan_advances_along_it` now fails (0.45 m gained, needs 0.5): the tracker
  is stopped at a shelf corner after 0.45 m on a WIDE scenario whose only unmapped obstacle sits
  6.32 m along the route. `test_the_certified_layer_sits_beneath_the_policy` still passes, but
  for that same shelf stop rather than the obstacle its docstring names (the docstring's "0.03 m
  off, 2.2 m along" predates A13). Both are left as they are until the ruling.

**Consequence for G4: rerunning the controls on the new scene with B1 as configured would fail
the same way.** That is a question about B1 and the planner's margins, not about the floor.

**The wall, confirmed** (`runs/diag/FLOOR_EDIT_20261013T214913Z`, dry run): `/Wall/Cuboid` is a
closed ring, 48 triangles, plan x and y in [-7.6, +7.6] m, z 0 to 0.25 m, its vertices 10.68-10.75 m
from the origin; it does not cover the origin. Collidable, measurable, detectable, respondable,
static. `/Wall/Script` is CoppeliaSim's configurable-wall model script (`configUi`: X/Y size,
thickness, height). The edit refuses while it stands; it needs a ruling.

**The floor is not collidable** (same dry run, every tile's `sim.getProperties`): each
`ResizableFloor_5_25_element` is respondable but `collidable False`, `measurable False`,
`detectable False`; each visible element is non-respondable and `collidable False`. A collection
honours the collidable flag, so **the floor is in neither contact set**: the wheels' exclusion of
the floor tree is moot, and `pfbnav/sim/contact.py`'s statement that the chassis is checked
against the floor ("touching it means...") has never held on this scene. Found by the positive
control in `tests/test_floor_sim.py`, which assumed collidable tiles and saw nothing on any of them;
the test now checks that every tile carries the centre tile's flags and that no contact set sees a
tile, with a collidable target at each spot as the control. The forty new tiles copy these flags.
Whether chassis-floor contact should count is a question for the supervisor; nothing is changed.

**Also from the sim run:** the nine tiles tile +-7.5 m with tops at z = -9.1e-8 m; the robot spawns
0.12 mm from the origin (`sim.robot_spawn_radius_m` 0.01 holds).


### Ruled 2026-10-13 (supervisor): re-freeze, the wall, the planner/field defect, recovery, the floor

Recorded before any measurement under it.

1. **Re-freeze acknowledged** (`env.frozen_code_sha256` 3f4c7a83...). **No training run exists**
   (`runs/train/` absent; no Run Request for `scripts/train.py` was ever issued) and **the Pure
   Pursuit control run `B1_pure_pursuit_20261006T074216Z` is void.** The reset's step-0 pose check
   **raises and stops the run, never warns and continues**: `NavEnv.reset` raises `RuntimeError`;
   `pfbnav/eval/runner.py` and `scripts/evaluate.py` catch nothing, and `scripts/train.py` records
   the error and re-raises. Pinned by
   `tests/test_runner.py::test_a_failed_step0_check_stops_the_run_rather_than_skipping_the_episode`.
   **Why the earlier gates were unaffected:** the stale pose needs CHAINED `NavEnv` resets, i.e. a
   reset after an episode that moved, inside one environment. No gate or probe uses `NavEnv`:
   G0's `gate_script` context, G1b, D2 and the traversability probe start the simulation fresh and
   build their geometry around the pose read AFTER that start; G0's `env_step_loop` context uses
   `StepLoop` with an obstacle placed in `after_start`; the step-loop tests likewise. The `NavEnv`
   sim tests restart the simulator between tests, and the one that reset twice moved 5 mm in
   between. The zero-action control never moved, so its placements were right. Only the Pure
   Pursuit control and the 2026-10-13 traces chained resets after motion.
2. **Remove `/Wall`**, in the same scene edit as the floor. In the regression, trajectories and
   contact outcomes must be bit-identical; scan packets may differ only in the bins that viewed
   the old wall, and the regression must confirm that exactly those bins differ.
3. **The planner/field inconsistency is an environment defect** (lethal 0.2386 m inside the
   0.2586 m at-rest field; from the D1 ruling). **(a) approved by criterion:** an ideal tracker
   following every planned route exactly at its stratum's traversal speed, field oriented along the
   path tangent, produces zero field breaches; no stratum may lose plannability (grid quantisation
   included). Choose the smallest lethal and soft margins meeting both, quantified offline on the
   built tune, pilot and training plans. Also report, information only, the breach rate at 50 mm
   cross-track toward the inside of bends. Report before the values are ruled; then amendment,
   rebuild of all built sets, relock.
   **(b) approved as a declared recovery behaviour for ALL classical baselines B1-B5 identically:**
   when stopped and the restart is blocked beyond a declared wait after the restart delay, move at
   <= 0.10 m/s in a restart-permitted direction maximising clearance (v . p <= 0 for every
   breaching point), for at most a declared distance, then resume; attempts per episode capped.
   Stops still count in H1; recoveries are counted and reported; a deadlock recovery cannot clear is
   an `unrecoverable_stop`. Learned policies get no scripted recovery; the asymmetry is stated in the
   methods. **(c) alone rejected.**
4. **Keep the floor non-collidable:** the wheel spheres are collidable and permanently touch the
   floor, so the contact metric concerns obstacles only. Correct `contact.py`'s documentation and add
   a test asserting the contact target set (hall geometry and unmapped obstacles, floor excluded).

**Order:** wall removal and floor enlargement as one scene edit; regression; offline margin
quantification and the route-level traversability report; the supervisor's ruling on values;
amendment, rebuild, relock; classical recovery; both G4 controls.

### Implemented 2026-10-13, under that ruling

- **Item 1:** the test above.
- **Item 4:** `contact.py`'s documentation corrected (the floor is in neither contact set because
  every tile is non-collidable; the wheels' exclusion is redundant). That file is in the frozen
  environment, so its text hash moved: **RE-FROZEN again, c63a1083...**, a docstring only; the
  filter-off bit-identity test reruns with the next simulator session.
  `tests/test_nav_env_sim.py::test_the_contact_target_set_is_the_hall_and_its_unmapped_obstacles`:
  the collidable shapes outside the robot are exactly the episode's layout shapes, unmapped included,
  no floor tile. It fails on today's scene because of `/Wall/Cuboid`, as it should.
- **The floor test's positive control failed again, and was the control's fault:** the 0.02 m probe
  sat wholly inside a 0.08 m collidable target and nothing registered on any tile. Read as the check
  comparing surfaces, so an enclosed shape is not a contact; the target is now the probe's size,
  overlapping it by half its width. (The tile flags themselves were read back as uniform:
  collidable, measurable and detectable all False on all nine.)
- **Item 2:** `scripts/scene_extend_floor.py` removes the whole `/Wall` tree (dummy, script, cuboid;
  their properties kept in `wall_removed.json`), refuses anything else standing on the new floor, and
  on read-back requires the inventory to show exactly the 80 tile objects added and exactly the
  `/Wall` tree removed. `scripts/regress_floor.py` records every scan packet with the body's world
  pose. From phase A alone it predicts the bins whose return lies on the old wall (within 10 mm in
  plan; a policy sector counts when any point of its arc does), and it requires A-vs-C differences to
  equal those bins, packet by packet in both streams, and A-vs-B to differ nowhere
  (`pfbnav.eval.regression.wall_bin_check`, known-answer tests in
  `tests/test_regression_scan_bins.py`, including a wrong extra bin and a missing one). It reports how
  many packets viewed the wall, so a vacuous pass is visible. The traversability traverses end 4.5 m
  short of the wall, inside the 5 m range, so some are expected to view it.
- **Item 3, the tool:** `pfbnav/world/route_check.py` checks the ideal tracker on a route exactly
  against the emulator's field polytope (separating axes over the field's own normal grid and the
  obstacle's edges). Tests: the threshold at the lateral reach and at the depth ahead to 1e-9 m, and
  agreement with an independent scan-style reading on 200 random cases.
  `scripts/quantify_margins.py` runs the (lethal, soft) grid over tune, tune-unmapped, pilot and
  training. Each stratum's traversal band is derived as the probe derives it: **0.40 m/s for all
  three strata, the largest field.** Plannability covers every built scenario plus a minimum-clearance
  aisle per stratum at 10 sub-cell positions on each axis. The 50 mm inside-offset figure is reported
  as information.
- **Smoke run, six tune-narrow routes, soft margin 0.03 (NOT a result; the grid has not run).** At
  today's 0.03 m lethal margin, 5 of 6 routes breach, by up to 152 mm. At 0.10 m, 5 of 6 still
  breach, by up to 104 mm, all at bends or on the goal approach. **A route whose goal lies within
  the field's depth of a shelf breaches at traversal speed whatever the margins**, because margins
  do not move goals. So the script reports breaches by location (start, goal approach, bend,
  straight) beside the ruled all-route verdict, so the supervisor can see whether the criterion as
  worded is reachable.


### 2026-10-14: the first floor edit refused, on three per-object identifiers; nothing was saved

`runs/diag/FLOOR_EDIT_20261013T230555Z_seed0`: the wall was removed in memory, the forty tiles were
copied, and the in-memory check refused every copy on `meshes`, `modelHash` and
`vortex.materialUniqueId`. The scene file was not written (`git status` clean). The dry run's dump
of the nine ORIGINAL tiles shows these are identifiers, not physical properties. The nine tiles share
one mesh-content hash, yet `meshes` are sequential mesh handles (10003176, 10003179, ...),
`vortex.materialUniqueId` sequential ids (859, 863, ...), and `modelHash` differs on every tile. So
they join the identity list in `scripts/scene_extend_floor.py`. The geometry stays compared, through
the mesh-content hash, and every physical property stays compared; nothing else is relaxed.


### 2026-10-14: the scene committed; sim read-back passes; the regression's control FAILED -- a scene reload is not deterministic

**Sim tests on 537f1bbb** (`1da1ca9`, clean): 24 of 25 pass. They show:
- 49 tiles cover +-17.5 m, every one non-collidable, tops at z = 0;
- nothing else stands on the floor; the spawn is 0.12 mm from the origin;
- the contact target set is exactly the episode's 8 layout shapes (1 unmapped), no floor tile;
- the filter-off bit-identity test passes, completing the re-freeze at c63a1083.

The one failure is `test_following_the_plan_advances_along_it`, the shelf-corner stop that awaits the
margin ruling.

**Regression `runs/gates/REGRESS_FLOOR_20261013T232522Z`: CONTROL FAILED.** Phase B, the SAME old
scene loaded again in the same process, differs from phase A from row 0 in every check:
- the settled start pose by ~6 um (x -0.000112 vs -0.000106 m);
- trajectories by up to 0.6 mm (traversability), 0.15 mm (G0) and 1.8 mm along-track (G1b);
- scan ranges by a median 0.4 mm, up to 2.25 m where a sub-millimetre shift moves a grazing ray past
  an aisle wall's end.

Inside one loaded scene the start IS repeatable: within A, and within B, every traverse begins from
bit-identical settled poses. So **loading a scene is itself a perturbation**. The check as ruled
("one process, old scene then new") cannot be bit-identical whatever the edit did, and A vs C cannot
be read. Per the ruling: stopped and reported; G0, G1b and traversability are NOT carried over.

**The wall-bin machinery engaged as designed:** 168 of 1,058 packets viewed the old wall, and all 892
predicted filter bins changed, by at least 5 mm (median 264 mm). None was unchanged.

**Proposed for the supervisor:** compare without a reload between the compared runs, as the
2026-09-18 collidable-flag regression did. Load the old scene once; run A and B (the determinism
control, no reload); apply the edit IN MEMORY with the edit script's own code (remove the /Wall
tree, copy the forty tiles); run C. The saved file is joined to that in-memory edit by the edit's
read-back, which already showed the reloaded file's inventory and every tile property equal to the
edit. The alternative, a tolerance against the A-vs-B reload spread, is not bit-identity and would
need a ruling of its own.


### Ruled 2026-10-14 (supervisor): the regression in one scene load; determinism scoped to one load

Recorded before any measurement under it.
- **In-memory regression approved.** Load the old scene once and run A, then B with no reload; apply
  the edit in memory with the edit script's own code; run C. The criteria are unchanged:
  trajectories and contacts bit-identical across A, B and C, and scan packets differing exactly in
  the predicted wall bins.
- **A secondary saved-file leg.** Load the old scene three times in one process, run the same
  profile after each, and record the largest pairwise deviation as the reload envelope. Then load
  the new saved scene (537f1bbb), run D, and require D's deviation from C to lie within that
  envelope. This tolerance is ruled for this purpose only.
- **Determinism is scoped to one scene load, not one process.** `docs/reproducibility.md` records
  the reload spread (~6 um settled start, up to 1.8 mm trajectories), small against every gate
  threshold. The WP5 requirement now reads: all controllers for a scenario run within one scene
  load, with a test that the evaluation runner never reloads within a scenario's controller set. The
  reload's cause is not to be investigated.
- If the regression passes, G0, G1b and traversability carry over; then the offline margin
  quantification.

**Implemented.**
- **Scripts.** `scripts/regress_floor.py` is rewritten to the two legs. The edit script's wall
  removal and tile copy are now the functions `remove_wall` and `add_tiles`, used by both scripts;
  their behaviour is unchanged, and the scene was already edited with that same code.
  Wall-clock columns (`*_seconds`, `*_ms`; G0's step-loop log has two) are set aside from
  bit-identity: they are timings, not trajectory, and differ on every run.
- **Envelope and inventory.** The envelope is per CSV and column, with row counts and non-numeric
  columns (contact names) included (`pfbnav.eval.regression.reload_envelope`, `within_envelope`;
  known-answer tests). Leg 2 also requires the saved scene's inventory to equal the in-memory
  edit's, object for object.
- **WP5.** `pfbnav.eval.runner.run_controllers` runs a scenario's controller set back to back on one
  environment. Tests: it never loads a scene, and neither `pfbnav/envs/` nor the runner contains a
  scene-loading call. `CLAUDE.md`'s determinism convention, `docs/reproducibility.md` and
  `docs/requirements.md` are updated.

### Reviewed 2026-10-14: the margin quantification, PARTIAL (the run stopped at 1,938 of 2,271)

`runs/analysis/MARGINS_20261013T233627Z` (a6af115, clean; this laptop, 6 workers). `routes.csv` was
last written at 05:17. The final scenario's rows are cut short and there is no `summary.json`, so
the process ended from outside the script (cause unknown to me). Complete: every tune, tune-unmapped
and pilot route, all of train_narrow and train_medium, and 334 of 667 train_wide (1,938 routes x 25
pairs). The missing train_wide routes are to be completed with `--sets`.

On the complete routes:
- **Plannability holds at every pair:** 1,938/1,938 plannable, and the minimum-clearance aisle plans
  at all 20 sub-cell positions in every stratum, up to lethal margin 0.10.
- **No pair meets the zero-breach criterion.** The fewest breaching routes is 398 (20.5 %), at
  (0.09, 0.13). At today's (0.03, 0.03) it is 1,246.
- **By location:**
  - straight-segment breaches fall from 1,056 routes to 0 from lethal 0.07 at any soft margin;
  - the residual is bends (~350-550 routes), the goal approach (55-80) and the start (13-20);
  - the soft margin barely moves it, and the worst penetration stays at -180 to -210 mm.
- **By stratum, at (0.07, 0.08):** narrow 377/857 routes, medium 45/707, wide 0/374. Bends have a
  median of -59 mm (worst -179); the goal approach a median of -68 mm (worst -198).
- **Information, 50 mm toward the inside of bends:** nearly every route at lethal 0.03-0.05; 1,490-
  1,847 at 0.07; 339-786 at 0.09-0.10.
- **A diagnostic on the 120 tune routes at (0.07, 0.08), outside the criterion:** with the
  band-0.10 field within 0.3 m of a bend, 1 m of the goal and 0.5 m of the start, breaching routes
  fall from 19 (16 narrow, 3 medium) to 4 (all narrow). The 0.30 band there leaves 18. So the
  residual is mostly the ideal tracker holding 0.40 m/s through corners and up to the goal, which
  margins cannot remove.


### 2026-10-14: the redesigned floor regression -- leg 1 PASSES exactly; leg 2 STOPs on velocity estimates

`runs/gates/REGRESS_FLOOR_20261014T214648Z` (8c13385, clean; scene 537f1bbb).

**Leg 1, one load (decisive): exact.**
- The A vs B CSVs are identical, and so are the scan packets (0 bins differ).
- The A vs C CSVs are identical: every trajectory and contact column of G1b, G0 in both contexts,
  and all six traverses.
- Scan packets: 168 of 1,058 viewed the old wall, 1,011 bins were predicted to differ, and 1,011
  differ -- exactly the predicted ones, packet by packet, in both streams.

The edit, applied in memory, changes no trajectory and no contact. The scans change only where they
saw the wall.

**Leg 2, the saved file (secondary, ruled tolerance): STOP.**
- The saved scene's inventory equals the in-memory edit's object for object (0 added, removed, moved
  or resized).
- D deviates from C beyond the 3-load envelope in 15 file-columns, every one of them a velocity
  estimate or G1b's reference yaw:
  - `vx` and `speed_est` in the six traverses: 7.05e-5 m/s against an envelope of 6.23e-5 (x1.13);
  - `vx` in G0: 6.86e-5 against 6.19e-5 (x1.11), and 4.63e-5 against 2.09e-5 in the step-loop
    context (x2.22);
  - G1b's `yaw_ref`, i.e. its settled start yaw: 2.10e-5 rad against 9.3e-7 (x22.7).
- No position, cross-track, distance, contact, contact-name or row-count column exceeds it.
  Positions sit well inside: traverse x 7.3e-5 m against 4.33e-4; G1b travelled 7.1e-4 against
  2.45e-3.
- **The envelope is thinner than three loads suggest: L1 and L2 are bit-identical** in every column,
  so three loads gave two distinct outcomes and one distinct pair.
- **Information:** leg 1's own load is a fourth load of the old scene (and C = A bit for bit, so D vs C
  is D vs A). With it, the envelope leaves three excesses: G0 `vx` x1.11 and x1.99, `yaw_ref` x2.1.
  The saved file's settled yaw differs from the old scene's loads by ~21 urad. That is about 4,000
  times below G1b's 5 deg heading threshold, and the velocity excesses are under 0.1 mm/s.

Per the ruling: stopped and reported; G0, G1b and traversability are NOT carried over pending the
supervisor. No cause was investigated (ruled).


### Ruled 2026-10-14 (supervisor): carry over on leg 1; leg 2 INCONCLUSIVE (flawed criterion); rule 19

- **(a): G0, G1b and traversability CARRY OVER to scene 537f1bbb** on leg 1 of
  `REGRESS_FLOOR_20261014T214648Z`, which passed exactly (A = B = C bit-identical in trajectories
  and contacts; scan packets differing in exactly the 1,011 predicted wall bins).
- **Leg 2 is recorded as INCONCLUSIVE, due to a flawed criterion -- not as passed.**
  - The envelope rested on two distinct outcomes (L1 = L2 bit-identical).
  - A maximum-based envelope checked across many columns has a high false-alarm rate even for an
    identical file, especially on velocity-estimate columns, which amplify small pose differences.
  - The criterion was the supervisor's and was never null-tested.
  - Its measured deviations are listed in `docs/reproducibility.md` as a property of the saved file.
- **Supporting evidence the supervisor cites:** the saved file's settled-yaw deviation (~21 urad,
  0.0012 deg) is below G1b's cross-invocation heading spread (0.0044 deg), which spans separate scene
  loads the gate verdict already absorbed. **Context, for the record:** the 0.0044 deg is the spread
  of FINAL heading across the three invocations of the 0.50 m/s cell (the failing cell; the
  G1b re-run's entry above). The 0.40 m/s cell, the profile this regression ran, spread by 0.0007
  deg (2.3585-2.3592), which is below 0.0012 deg; and the 0.0012 deg is a SETTLED START yaw, not a
  final heading. Carry-over rests on leg 1 either way; this is recorded so the supporting figure is
  not quoted out of its context.
- **Standing rule, added to CLAUDE.md (hard rule 19):** any tolerance check applied across multiple
  columns states its expected false-alarm rate under the null before running, and is null-tested
  (e.g. on an extra load of the unchanged file) before it can gate anything.
- **Next:** the offline margin quantification and the route-level traversability report.


### Reviewed 2026-10-15: the margin quantification, COMPLETE -- no pair meets the criterion

`runs/analysis/MARGINS_REPORT_20261015T150304Z`, merging `MARGINS_20261013T233627Z` (a6af115) and
`MARGINS_20261014T222612Z` (708be4d, train_wide only). Both are clean. Between the two commits only
the reporting code moved: planning, world, safety and configs are identical. **The 334 train_wide
routes complete in both runs agreed bit for bit** (the merge requires it), an independent
reproduction across commits and invocations. Coverage: every built route, 2,271 (tune 120,
pilot 150, train 2,001) x 25 pairs.

- **Plannability holds at every pair:** 2,271/2,271, and 0 failures of the minimum-clearance
  aisle at 20 sub-cell positions per stratum, up to lethal margin 0.10.
- **No pair meets zero breaches.**
  - Today's (0.03, 0.03): 1,414 routes (62.3 %).
  - The fewest: (0.09, 0.13), 399 (17.6 %).
  - The smallest pair clearing every straight segment with plannability kept: (0.07, 0.03).
- **The defect proper is cured from lethal 0.07:** straight-segment breaches fall from 1,223 routes
  (0.03) through 30-80 (0.05) to 0 at 0.07 and above, at every soft margin. The lethal radius is
  then 0.2786 m, 20 mm outside the at-rest field.
- **The residual does not respond to margins.**
  - Above lethal 0.07: bends 349-548 routes, the goal approach 54-82, the start 13-20.
  - The worst clearance stays at -180 to -210 mm at every pair.
  - The soft margin moves the count non-monotonically (re-routing), not toward zero.
- **By stratum, at (0.07, 0.08):** narrow 377/857 (44 %), medium 45/707 (6 %), wide 1/707.
- **Information, 50 mm toward the inside of bends:** 100 % of routes at lethal 0.03, ~95-76 % at
  0.07, 15-38 % at 0.09-0.10.
- **Why** (the tune diagnostic of 2026-10-14, outside the criterion): an ideal tracker held at 0.40
  m/s through a polyline vertex swings its whole moving field round the corner at speed, and keeps
  its full stopping depth up to a goal that may sit in front of a shelf. With the band-0.10 field
  within 0.3 m of bends, 1 m of the goal and 0.5 m of the start, the tune routes' breaches fall from
  19 to 4 (all narrow). The residual is a property of the criterion's speed assumption; margins
  cannot remove it.


### Ruled 2026-10-15 (supervisor): lethal margin 0.07; the criterion amended to an oracle tracker

Recorded before any measurement under it.
- **Lethal margin 0.07 ruled now (lethal radius 0.2786 m):** the smallest value clearing every
  straight, plannability intact through 0.10, nothing above it clearing more. **It cures the D1
  defect** (the supervisor's own ruling put the lethal radius, 0.2386 m, inside the at-rest field,
  0.2586 m). The config value changes with the amendment and rebuild: the built sets were made at
  0.03 and must be rebuilt together with it.
- **The criterion is amended**, replacing "an ideal tracker at the stratum's traversal speed"
  (wrong: no controller holds top speed through a corner or into a goal). **An oracle tracker**
  follows the route exactly, travelling at each point in the fastest band whose field, oriented
  along the path tangent, is clear, subject to the deceleration limit. A route is traversable
  without a stop iff the band-0 field is clear everywhere along it, start and goal included.
  **Criterion: zero routes with a band-0 breach.** Report the oracle's mission time against
  constant top speed, and use the oracle for the per-route unavoidable-stop floor.
- **Stops caused by not slowing at bends are controller-attributable and count in H1.**
- **Generator validity rules:**
  - start validity: the band-0 field along the route's initial direction is clear at the start;
  - goal validity: the band-0 field along the final approach direction is clear on arrival;
  - on failure, re-sample the goal, never the layout; log rejections per stratum.
- **Then:** rerun the quantification at lethal 0.07 across soft margins under the oracle criterion,
  and choose the soft margin minimising bend breaches.
- **Residual rule, declared now:** residual <= 2 % of routes in EVERY stratum -> reject and
  re-sample those routes, and report the rate; any stratum above 2 % -> stop and report.
- Report the 50 mm inside-bend breach rate again, as information.


**Implemented 2026-10-15, before any measurement:**
- `pfbnav/world/route_check.py`: `band_caps` (the fastest clear band), `oracle_speeds` (the backward
  pass under `actuation.a_decel_max`, 1.0 m/s^2, ending at rest on the goal; no acceleration limit,
  as ruled, so mission time is a best case), `stop_runs` (separate breached stretches: the
  per-route unavoidable-stop floor) and `traverse_time`. All have known-answer tests.
- `pfbnav/eval/oracle_report.py` classes each route: unplannable, start-invalid, goal-invalid,
  residual or clear. Start validity is the band-0 field at the start along the initial direction;
  goal validity is the same on arrival along the final approach. The residual rule (<= 2 % in
  every stratum) and the soft-margin choice (fewest band-0 breaches at bends, among margins keeping
  every route plannable; ties to the smaller) are coded there before the run and tested.
- `scripts/quantify_oracle.py` with `configs/analysis/planner_oracle.yaml`: lethal 0.07, soft
  margins 0.03, 0.05, 0.08, 0.10, 0.13, 0.20 and 0.30, every built tune, pilot and training route.
  Plannability uses the same aisle-quantisation check as before. `--sets` completes an
  interrupted run and `--report` merges runs.
- The generator's validity rules (goal re-sampling, rejections logged per stratum) belong to the
  amendment and rebuild, after the values are ruled. This run reports how many built routes the
  rules would reject.


### Ruled 2026-10-15 (supervisor): the comparator set -- B4 and B5 retired, B6 and B7 added (A16)

Asked before WP3: two gaps in the comparators (no classical controller aware of the certified fields;
the attribution analysis built on plain Pure Pursuit). Ruled:
- **B4 ("PP + field switching only", the industrial status quo) retired.** With the certified layer
  beneath every controller it is identical to B1, a design error by the supervisor. B1 carries the
  "industrial status quo" label.
- **B5 (PP + PFB) retired**, superseded by B7.
- **Method IDs are never reused** (standing rule; CLAUDE.md hard rule 20; registry
  `configs/methods.yaml`).
- **B6 = holonomic MPPI with a field-aware cost**, an H1 comparator in B4's place:
  - the cost is on the certified layer's actual discrete band fields, NOT the PFB field (otherwise
    MPPI becomes a soft PFB);
  - declared now: route tracking, goal progress, field-breach penalty, hard rejection of rollouts
    intersecting scan points, control smoothness; horizon, samples, temperature;
  - tuned under the equal-tuning-budget rule on the tune set only;
  - the same route, certified layer, 120-bin scan stream and classical recovery as every baseline;
  - implemented in our stack with vectorised rollouts, not Nav2.
- **B7 = RPP + PFB**, replacing B5 in H5 (exploratory) and in H3's granularity study. H5 now reports
  protective stops as well as mission time.
- **Power:** B6 enters fully. Seven comparators remain, so the shape of the analysis is unchanged.
  Step 0 measures inter-comparator gaps including B6, so B6 is implemented and tuned before that
  measurement. Making B6 secondary to protect power was rejected: it leaves the objection
  unanswered, the move branch 5 forbids.
- **Amend now**, before WP3 and before any test evaluation. G4 is not affected (its positive control
  is B1).

**Prepared:**
- the amendment text `docs/pending/A16_comparators.md` and the replacement
  `docs/pending/endpoints_A16.yaml`. Only H1 and H1N comparators, `narrow_rule.methods` and H5
  change; checked to parse and to name only active methods;
- B6's declared design in `configs/base.yaml` `baselines.mppi`, with initial values marked VERIFY
  until tuned;
- `configs/methods.yaml` with every ID ever used and its status, and
  `tests/test_methods_registry.py`. That test fails by design until the researcher applies A16
  and relocks;
- the power configs and `tests/test_power_grid.py` now name B6 (`power_corrected.py` refuses a
  config that disagrees with the frozen endpoints, so they land together).

**Two IDs unconfirmed:** which of B2 and B3 is Regulated Pure Pursuit and which is 3-D DWA is not
restated in the repository; the registry marks both VERIFY.


### 2026-10-15: B2 and B3 confirmed; the classical set and what it shares

From the research plan's experimental matrix, confirmed by the researcher (the VERIFY marks of the
A16 preparation are removed):
- **B2 = Regulated Pure Pursuit**, Nav2-style: slows on curvature and near obstacles.
- **B3 = holonomic DWA**, a three-dimensional dynamic window over (vx, vy, omega).

**The classical set is now:**
- B1, holonomic Pure Pursuit (the industrial status quo);
- B2, RPP;
- B3, DWA;
- B6, field-aware MPPI;
- B7, RPP + PFB.

B4 and B5 are retired, their IDs never reused.

**All of them share:**
- the same route;
- the certified layer beneath;
- the 120-bin scan stream;
- the non-square actuation box with the wheel-budget projection;
- the classical recovery behaviour;
- an equal tuning budget, on the tune set only.

Recorded in `configs/methods.yaml` (each ID with its source) and in the A16 draft before it is applied.


### Reviewed 2026-10-15: the soft margin under the oracle -- chosen 0.10; residual rule STOP (narrow 16.2 %)

`runs/analysis/ORACLE_20261015T151712Z` (a944fd7, clean; scene 537f1bbb). Complete: 2,271 routes x 7
soft margins = 15,897 rows. Lethal margin 0.07.

- **Plannability holds at every soft margin:** 2,271/2,271, no quantisation failures.
- **Chosen soft margin, by the declared rule: 0.10.** Fewest routes breaching band 0 at a bend: 141,
  against 187 at 0.03, 167 at 0.05, 144 at 0.08, 160 at 0.13, 155 at 0.20 and 161 at 0.30.
- **Residual rule: STOP.** At 0.10 the residual is narrow 139/857 (16.2 %), medium 4/707 (0.6 %),
  wide 0/707. Start-invalid: 17 routes, all narrow. Goal-invalid: 14 (13 narrow, 1 medium).
- **What the narrow residual is:**
  - where: bends in 126 routes and the goal approach in 16; never a straight or the start;
  - almost always one stretch per route (135 with one, 4 with two);
  - depth: a median of 21 mm into the band-0 field (10th-90th percentile 63-3.5 mm; worst 88 mm);
  - by clearance: 23.5 % at 0.13-0.15 m per side, 16.7 % at 0.15-0.17 and 11.0 % at 0.17-0.20.
- **Oracle mission time against constant top speed: median x1.008** in every stratum. Slowing where
  the fields demand costs under 1 %.
- **Unavoidable stops per 100 m (the oracle floor), at 0.10:** narrow 1.957, medium 0.065, wide 0.
- **Information, 50 mm toward the inside of bends:** 1,885/2,271 routes breach band 0 at 0.10, and
  1,684-2,179 across the soft margins. A lethal radius of 0.2786 m leaves 20 mm to the band-0 field
  abeam (0.2586 m), less than G1b's 50 mm tracking bound.

**A diagnostic, outside the criterion: the residual is the oracle's body-orientation assumption.**
- The ruling orients the FIELD along the path tangent. My implementation also turned the BODY to
  face the tangent, an assumption the ruling does not make. A mecanum base can travel at any
  heading to its body, and with a 0.285 x 0.209 m footprint the orientation changes which corners
  sweep the shelves.
- On the 143 residual routes at soft 0.10 (139 narrow, 4 medium), I allowed a body offset of -90 to
  +90 deg in 15 deg steps, with the travel direction still along the tangent: `FieldShape.moving(...,
  heading)`, tested against the emulator's field at that heading.
- **Per breached point: all 354 points on all 143 routes clear.**
- **With ONE offset held through each route's breached stretch** (one a robot could take before the
  corner and keep): **142/143 clear**; 74 of them need only 15 deg. On that model the narrow
  residual would be 1/857 (0.1 %), under the 2 % limit.
- **Not checked:** whether rotating into and out of the offset breaches elsewhere along the route,
  and yaw-rate limits. So this is a diagnostic, not a verdict.
- The start and goal validity failures were not re-examined with a free orientation.


### Ruled 2026-10-15 (supervisor, second ruling): the free-orientation oracle; H4; the margin asymmetry

Recorded before any measurement under it.
- **(a): formalise the oracle with free body orientation**, constrained by:
  - yaw rate <= w_max (1.5 rad/s) and yaw acceleration <= alpha_max (3.0 rad/s^2, VERIFY);
  - the wheel budget at the nominal share 0.80: body rotation consumes budget, and when it binds
    the oracle drops to a slower band;
  - yaw continuity along the route;
  - the field oriented along the velocity direction, with the footprint (`FootprintTable`)
    rotating with the body.

  Rerun; the residual rule decides unchanged (<= 2 % in every stratum passes, otherwise stop and
  report).
- **The unavoidable-stop floor is platform-specific** (holonomic, full route knowledge); say so
  wherever it is reported.
- **Preregister now, as an exploratory directional expectation for H4,** that A1 shows a materially
  higher narrow-stratum stop rate than L5. Compute A1's own floor with the tangent-locked oracle
  and report it alongside.
- **The lethal margin stays 0.07** with the exact-tracking criterion. The asymmetry goes in the
  methods notes: straights are guaranteed at G1b's 50 mm bound, corners only for exact tracking;
  inside-bend breaches are controller-attributable.
- **Information only:** the 50 mm inside-bend breach rate under the free-orientation oracle at
  lethal 0.07 and 0.10, with narrow plannability at each.
- **Then** the generator's validity re-sampling for the start-invalid and goal-invalid routes,
  amendment, rebuild, relock.

**Implemented, before any measurement:** `pfbnav/world/oracle.py` (tested in
`tests/test_oracle_free.py`).
- **The yaw grid** has 15 deg steps, anchored at each scenario's start yaw, so the real start
  orientation is exact.
- **Feasibility is reachability (`reach`).** From the start yaw the robot may rotate in place
  through orientations whose band-0 field is clear, then moves to the next sample keeping its yaw.
  It never jumps across a blocked orientation. Every limit scales down with speed and band 0 admits
  any speed up to its top, so the limits shape the SPEED, not whether a route can be driven: stated
  in the module, not assumed silently.
- **The free floor** counts separate stretches where no continuous path survives, and where each
  begins (`deaths`).
- **Mission time (`fastest_path_time`):** dynamic programming over (sample, yaw).
  - **Moves:** a move keeps the yaw or turns one grid step, bounded by `turning_speed_limit` (the
    box; the budget, which rotation consumes; the yaw rate; the yaw acceleration to start the turn).
  - **In-place rotations:** from rest, rate-limited by w_max and by the budget (1.28 rad/s,
    below w_max), trapezoidal under alpha_max.
  - Then a deceleration-limited backward pass; no acceleration limit, as before.
- **Speed.** `clearance_to_field` groups its separating-axis work by yaw and takes an exact
  coarse-first option. A subset of the field's normals proves a clear state, and only unseparated
  pairs get the full set. Breach depths stay exact (tested against the full computation). 5-7x
  faster.
- **Run:** `scripts/quantify_oracle_free.py` with `configs/analysis/planner_oracle_free.yaml`, in
  two phases.
  - `decide`: every soft margin. The declared choice (fewest routes stopping at a bend, among
    margins keeping every route plannable) and the residual rule
    (`pfbnav/eval/oracle_report.summarise_free`, tested). It carries the tangent-locked floor, A1's,
    on every route.
  - `info`, at the chosen soft margin: mission time against constant top speed, and the 50 mm figure
    at lethal 0.07 and 0.10 with narrow plannability.
- **Smoke (three tune-narrow routes; not a result):** all traversable with free orientation; the
  tangent-locked oracle stops once on one of them.
- **Methods wording** in `docs/writeup_notes.md`. **H4's expectation** drafted as A17
  (`docs/pending/A17_H4.md`, `docs/pending/endpoints_A17.yaml`), for the researcher to apply and
  relock now.


### Reviewed 2026-10-15: the first free-orientation run is VOID -- an orientation-grid artefact; fixed

`runs/analysis/ORACLE_FREE_decide_20261015T192700Z` (a1dd28a, clean; 15,897 rows, complete). It
reported soft 0.05 chosen and the residual rule passing (narrow 0.5 %, medium 0.1 %, wide 0.6 %). But
the wide stratum left 4 residual routes where the tangent-locked oracle (A1's floor) stopped on none,
and an oracle free to choose its orientation cannot do worse than one locked to the tangent.
- **All 50 residual routes across the seven soft margins had `tangent_stops` = 0, and every stop was
  on a straight.**
- **Cause: my implementation.** The yaw grid was anchored at each scenario's start yaw in 15 deg
  steps, so the orientation exactly along the route was usually not on it; the nearest could be
  7.5 deg off. In a tight aisle a body turned even a few degrees swings its corners into the field.
  The oracle is free to face along the route; the grid did not let it.
- **Fix:** `yaw_set`, the start-anchored grid PLUS every segment direction of the route. The oracle
  now handles uneven spacing: rotation and turning angles come from the real gaps between
  neighbouring orientations, and the start yaw's index is passed explicitly.
- **Tests:** the set's contents; uneven angles in the mission time; and a geometric reproduction of
  the artefact -- a corridor 2 mm wider than the band-0 lateral reach, entered with the body at
  11.5 deg, fails with the grid alone and passes with the route's direction in the set (a positive
  control for the fix).
- **The 9 residual routes at soft 0.05, re-evaluated with the fix: all traversable, no stop.**
- **The run's figures are void:** the residual counts, the free floor, the bend-stop counts and so
  the soft choice. Start validity was not affected (it uses the start yaw itself), but it is re-read
  from the rerun. The rerun is requested.


### Reviewed 2026-10-15: the free-orientation oracle (corrected) -- soft 0.03 chosen; residual rule RE-SAMPLE

`runs/analysis/ORACLE_FREE_decide_20261015T210142Z` (855d846, clean; scene 537f1bbb; 15,897 rows,
complete). Lethal 0.07, seven soft margins, the orientation set including the route's directions.

- **Plannability holds at every soft margin** (2,271/2,271, no quantisation failures).
- **Residual: 0 routes at soft 0.03-0.13**, 1 at 0.20 and 1 at 0.30. Bend-stop routes: 0 at every
  soft margin.
- **Chosen soft margin, by the declared rule: 0.03.** Every margin ties at zero bend stops and the
  tie goes to the smaller: today's value, unchanged.
- **Residual rule at 0.03: RE-SAMPLE** (medium 0 %, narrow 0 %, wide 0 %), so nothing to re-sample
  as residual.
- **Start-invalid: 20 routes, all narrow** (16 train_narrow, 4 pilot_narrow): the robot's actual start
  yaw has a band-0 breach at the start. **Goal-invalid: 0.** These 20 are what the generator's
  validity rules will re-sample (goal re-sampled, never the layout; rejections logged per stratum).
  They replace the 17 + 14 the tangent-locked oracle reported, which assumed the body faced the route
  at the start.
- **The floor at 0.03 (platform-specific: holonomic, full route knowledge):**
  - free floor: narrow 0.222 /100 m, medium 0, wide 0. It is entirely the 20 start-invalid routes,
    one stop each; no clear route has a stop. After re-sampling it is expected to be zero.
  - **A1's (tangent-locked) floor: narrow 2.649 /100 m, medium 0.053, wide 0.** In narrow aisles 211
    of 857 routes stop under the tangent-locked oracle, against 20 with free orientation. This is
    consistent with H4's preregistered direction (A17), as context, not as H4's evidence.
- **The check promised before applying the rule** (no residual route the tangent-locked oracle drives
  clear): **2 such routes**, at soft 0.20 and 0.30 only. Both examined; **neither is an artefact.**
  - In both the robot starts facing about 0 deg, but the route re-planned at that soft margin leaves
    in the opposite direction (about 178 deg).
  - The oracle drives backwards, which is legitimate for a holonomic base, into an aisle where only
    the forward-facing orientation is clear, and the space never allows a rotation through 90 deg to
    turn round.
  - The tangent-locked oracle never sees this, because it places the body along the route from the
    start: a turn no robot can make in place there.
  - Start headings were set along routes planned at the old margins. The rebuild re-orients them.

**Next (ruled order):** the information phase at soft 0.03 (mission time; the 50 mm inside-bend rate
at lethal 0.07 and 0.10 with narrow plannability); then the generator's start/goal validity rules,
amendment, rebuild, relock.


### 2026-10-16: the generator's validity rules implemented; lethal 0.07 in the config; A18 drafted

Implements the 2026-10-15 rulings (start validity, goal validity, residual routes re-sampled; the goal
re-drawn, never the layout; rejections logged per stratum), with the free orientation of the second
ruling. Nothing has been rebuilt yet.

- **`pfbnav/world/oracle.py`: `route_validity`** classes a planned route as `ok`, `start_invalid`,
  `goal_invalid` or `residual`, using the same free-orientation oracle (yaw continuity, orientation set
  including the route's own directions) as the decision run, at the same resolution
  (`world.layout.validity` in `configs/base.yaml`). Known-answer tests for all four classes.
- **`pfbnav/world/scenarios.py`:** `ValidityCheck` builds the band-0 field geometry once per set.
  `make_scenario` runs it after the aisle-threading check. On a failure it re-draws the goal (another
  lane, the far quarter of the hall), keeping the layout and the start, up to `goal_resamples` = 8
  times; a re-drawn goal that does not plan or does not thread is drawn again without being counted
  as a rejection. Then it moves to the next endpoint pair. Each scenario records
  `validity_rejections` (by cause) and `goal_resamples`. A seed with no valid route fails as
  `no_route_passes_the_validity_rules`. The manifest gains `validity_rejections_by_stratum`, which
  always lists every cause.
- **`goal_resamples` = 8 is my choice, not a ruling** (the ruling says "re-sample the goal" without a
  count). It only moves which goal a rejected start ends with; any value gives valid routes. Flagged for
  the supervisor in A18.
- **Parallel set building** (`build_set(workers=...)`, `scripts/build_scenario_sets.py --workers`):
  each scenario draws only from its own seed's stream, so the set is identical for any worker count
  (tested). At lethal 0.07 a scenario costs about 3-7 s with the rules, up to about 20 s with
  unmapped placement. A 2,000-scenario set is therefore hours serial and well under an hour on 8
  workers.
- **`scenarios_sha256` now hashes the import closure** of `pfbnav.world.scenarios`, not
  `scenarios.py` alone. Found while adding the oracle: since A13 a change to `placement.py` or
  `bypass.py` would have moved routes without moving the locked identity. It changes the hash's
  definition, so the relock carries it (A18).
- **`planner.lethal_margin` 0.03 -> 0.07 in `configs/base.yaml`** (ruled 2026-10-15; "the config value
  changes with the amendment and rebuild"). It is in the working tree now, so that the validity rules
  and the margin they were calibrated under land in the same commit as A18.
  - At 0.03 the rules reject heavily: seed 1,000,020 logged 35 residual rejections.
  - At 0.07 the ten seeds 1,000,020-029 logged none, as the decision run predicts.
  - `tests/test_footprint_config.py` now pins 0.2786 and asserts the lethal radius exceeds the
    at-rest field abeam.
- **Tests** (`tests/test_scenarios.py`):
  - a built set re-planned from what it records passes the rules afresh;
  - a scripted rejection re-draws only the goal, keeps the start and layout, and is logged by cause;
  - a seed that never passes is the named failure with its rejections;
  - per-stratum tallies are complete and add up;
  - a parallel build equals the serial one;
  - the scenarios hash covers `oracle.py`, `route_check.py`, `placement.py`, `bypass.py` and
    `astar.py`.
  - Builds are cached and shared across the existing tests.
- **Consequences, expected:**
  - `tests/test_preregistration.py` fails until the A18 relock (`scenarios_sha256` moved).
  - **RE-FROZEN, c63a1083... -> 645358f2...:** `scenarios.py` is in the environment's import
    closure and now brings `oracle.py` into it. Reset rebuilds a recorded world through
    `world_of`, which is unchanged. No L1 run exists. **Completed 2026-10-16:** the filter-off
    bit-identity test passed on the simulator at 508e2a3 (researcher's run).
  - The planner's lethal margin is config, not code, so it does not move the freeze. It does
    change the routes the environment tracks, which is why it lands with the rebuild.
  - The built sets fail verification against the new `scenarios_sha256` until they are rebuilt.

**Next:** the researcher commits, applies A18 and relocks; re-freezes; rebuilds tune,
tune_unmapped, pilot and train with `--workers`; then the oracle check on the rebuilt sets (expected:
zero band-0 routes, floor zero).


### Reviewed 2026-10-16: the free-oracle information phase (lethal 0.07 and 0.10, soft 0.03)

`runs/analysis/ORACLE_FREE_info_20261015T223837Z_seed0` (cca16fe, clean, seed 0, scene 537f1bbb,
laptop; 4,542 rows = 2,271 routes x 2, every one planned). Edits to `oracle.py` made while it ran
were additions only (`route_validity`), so nothing the run called changed. Information only, as ruled;
no criterion attaches to these numbers.

- **Plannability: 100 % at both margins** in every stratum, no quantisation failures (narrow floor
  0.13 m at 20 positions).
- **Oracle traversability is unchanged from the decide run:** the same 20 narrow start-invalid
  routes at both margins; every other route is clear.
- **50 mm inside offset, routes with a band-0 breach.** The script's share counts the 20
  start-invalid routes as breaches; the figures below exclude them.

  | lethal | narrow (837) | medium (707) | wide (707) |
  |---|---|---|---|
  | 0.07 | 788 (94.1 %) | 666 (94.2 %) | 658 (93.1 %) |
  | 0.10 | 35 (4.2 %) | 13 (1.8 %) | 6 (0.8 %) |

  The step between the two margins is the arithmetic of the abeam field, and the quantitative form of
  the asymmetry already recorded in `docs/writeup_notes.md`. A route at the lethal boundary of a
  corner, displaced 50 mm inside, lies 0.2786 - 0.05 = 0.2286 m from it at lethal 0.07, under the
  band-0 field abeam (half_width + lateral_margin = 0.2586 m). At lethal 0.10 it lies at exactly
  0.2586 m, zero slack, so what remains is where the field ahead or the grid's corner-cutting matters.
  This explains the step; it was not checked route by route (the CSV does not say where a breach
  is).
- **Mission time against constant top speed:** median 1.090 at 0.07 and 1.075 at 0.10, about the
  same in every stratum. The spread is 1.03-1.20; the cost is about 2 s per route, growing slightly
  with path length. This is a best case: no acceleration limit, as ruled.

**What it means, for the supervisor (not a ruling):**
- At lethal 0.07 a controller tracking within G1b's 50 mm bound but on the inside of bends will
  breach band 0 on almost every route. Those stops are controller-attributable and count in H1
  (ruled 2026-10-15). Pure Pursuit cuts bends inward by construction, so B1 should be expected to
  do this.
- Lethal 0.10 removes most of it at no measured cost here: still 100 % plannable, and the oracle is
  faster.
- A18 is not yet applied, and nothing is rebuilt. If the margin is to be reconsidered, this is the
  cheapest point; after the rebuild a change means another amendment and rebuild.


### Ruled 2026-10-16 (supervisor): lethal margin 0.10, before A18

Recorded before any measurement under it.
- **Lethal margin 0.10** (lethal radius 0.3086 m), replacing 0.07 before A18 is applied.
- **Grounds:**
  - it meets the reconsideration condition: the 50 mm inside-bend breach rate falls from about 94 %
    to 0.8-4.2 %, with plannability at 100 % (information phase, 2026-10-16);
  - it is the ceiling the narrow floor allows. The centreline band is about 0.06 m, about one grid
    cell: 2 x (0.2086 + 0.13 - 0.3086).
- **Methods notes:** at 0.10 a tracker 50 mm inside a bend sits exactly on the band-0 side field
  (zero slack, against 0.03 m on straights). The residual 0.8-4.2 % inside-bend breaches remain
  controller-attributable.
- **The sub-cell narrow-aisle plannability check becomes a permanent regression test**, since the
  margin now sits at the edge of plannability.
- **Before A18, rerun the oracle procedure at lethal 0.10:**
  - soft-margin sweep under the free-orientation oracle, with the declared choice rule (fewest bend
    breaches; ties to the smaller);
  - residual rule unchanged: <= 2 % per stratum passes and the routes are re-sampled, otherwise stop
    and report;
  - recount start- and goal-invalid routes.
- **A18 then carries** lethal 0.10, the chosen soft margin, the validity rules, the rebuild and the
  relock.

**Implemented 2026-10-16, before any measurement at 0.10:**
- **`planner.lethal_margin` 0.10** in `configs/base.yaml`. `tests/test_footprint_config.py` pins
  the lethal radius at 0.3086 and asserts it exceeds the at-rest field abeam (0.2586).
- **The permanent regression:** `pfbnav/planning/plannability.py` (`aisle_plannability`), moved
  from `scripts/quantify_margins.py` unchanged in what it computes. It reproduces the information
  run's `quantisation.json` exactly from that run's recorded config.
  - `tests/test_plannability.py`: under the configured margins every stratum's minimum-clearance
    aisle plans at all 40 sub-cell positions (2.5 mm steps, both axes), and the narrow centreline
    band exceeds one cell.
  - The positive control is a known answer: past the edge the share of failing positions is
    1 - band / cell, with band = 2 x (0.13 - lethal margin). Measured before the test was written:
    0/40 at 0.105, 8/40 at 0.11, 16/40 at 0.115, 24/40 at 0.12 and 40/40 at 0.13, exactly the
    formula. **0.10 is 0.01 m (a fifth of a cell) from the first failure.**
- **`configs/analysis/planner_oracle_free_l010.yaml`:** the same procedure at lethal 0.10. The 0.07
  config is kept unchanged for its runs' record.
  - `scripts/quantify_oracle_free.py` now labels its decide report with the configured lethal
    margin (it said 0.07 literally).
  - The decide phase refuses a config listing more than one lethal margin, since its choice rule
    groups by soft margin alone.
- **Methods notes** (`docs/writeup_notes.md`) and the A18 draft carry 0.10. A18's soft margin and its
  0.10 evidence are left as marked blanks until the run.
- **Expected in the run, stated before it:** the built sets' start headings were set along routes
  planned at lethal 0.03. At 0.10 some routes may leave in another direction, which can appear as
  start-invalid routes or as continuity residuals like the two at soft 0.20/0.30 in the 0.07 run.
  The rebuild re-orients every start, and the validity rules re-draw whatever remains.


### Reviewed 2026-10-16: the free-orientation oracle at lethal 0.10 -- soft 0.03 chosen; residual rule RE-SAMPLE

`runs/analysis/ORACLE_FREE_decide_20261016T003847Z_seed0` (8906820, clean, seed 0, scene 537f1bbb,
laptop). Config lethal 0.10 with the seven soft margins; 15,897 rows (2,271 x 7), every route planned.

- **Plannability holds at every soft margin** (2,271/2,271, no quantisation failures).
- **Residual: 0 routes at every soft margin, in every stratum.** The two continuity residuals at soft
  0.20/0.30 under lethal 0.07 do not recur.
- **Chosen soft margin, by the declared rule: 0.03.** Every margin ties at zero bend-stop routes, and
  the tie goes to the smaller. The config value is unchanged; its VERIFY mark is resolved.
- **Residual rule at 0.03: RE-SAMPLE** (0 % in each stratum): nothing to re-sample as residual.
- **Start-invalid: 20 routes, all narrow, the same 20 (set, seed) as at lethal 0.07. Goal-invalid:
  0.** The generator's rules re-draw them in the rebuild.
- **Free floor at 0.03:** narrow 0.217 per 100 m, medium and wide 0. It is exactly 20 stops, one at
  the start of each start-invalid route; none at a bend, on a straight or at a goal. After the
  rebuild it is expected to be zero.
- **A1's (tangent-locked) floor rose with the lethal margin.** It is not a criterion; it is context
  for H4 and is reported beside L5's.

  | stratum | at lethal 0.07 | at lethal 0.10 |
  |---|---|---|
  | narrow | 2.649 / 100 m (211/857 routes) | 4.688 / 100 m (374/857) |
  | medium | 0.053 (4) | 0.131 (10) |
  | wide | 0 | 0.040 (3) |

  - Route by route at soft 0.03: 238 routes have more tangent-locked stops at 0.10, 45 have fewer.
  - **Not yet explained.** The CSV does not record where a tangent-locked stop falls. A plausible
    cause, unverified: with the narrow centreline band at about one cell, the planner can no longer
    ease into an aisle, so a body forced to face the route meets sharper turns at the mouths. A
    location diagnostic would settle it.
- **A1's floor also depends on the soft margin.** Pooled over all strata: 1.833 per 100 m at soft
  0.03, 0.973 at 0.08, 0.828 at 0.10, 0.866 at 0.13, 0.959 at 0.30. The declared choice rule uses
  the free oracle only, so this did not enter the choice; noted for the supervisor.

A18's blanks are filled from this run.

**Next:**
- the researcher commits;
- applies A18 and relocks;
- rebuilds tune, tune_unmapped, pilot and train;
- then the oracle check on the rebuilt sets (expected: no band-0 route, free floor zero).


### Reviewed 2026-10-16: the sets rebuilt under A18

`runs/diag/SCENARIO_SETS_20261016T091457Z_seed0` (c1a6e8b, the A18 relock; clean; seed 0; scene
537f1bbb; the scenario module matches the lock). Ten sets: tune x 3, tune_unmapped x 3, pilot_narrow,
train x 3. Test sets were not built (A14).

- **Every set built in full** (2,271/2,271, failed seeds 0) and **verifies** against the lock.
- **Unmapped rates are unchanged** from the committed sets; tune_medium's 0.35 (7/20) was already
  0.35.
- **Validity rejections: 29, all start-invalid, all narrow** (pilot 2, train_narrow 27).
  - Goal-invalid 0, residual 0 in every set.
  - 12 scenarios ended with a re-drawn goal (pilot 2, train_narrow 10).
  - The decide run had found 20 start-invalid routes on the old sets (16 train, 4 pilot). The
    numbers differ because the rebuild counts every candidate route it rejects, and re-plans at
    0.10.
- **Placement and engagement** are as before:
  - engaged 100 %, every unmapped_static set valid;
  - narrow obstacles 97-100 % on open floor, never in an aisle;
  - no bypass-control losses.
- **What moved, as expected:**
  - Endpoints: starts identical for 2,148 of 2,271 scenarios. The rest moved because a first
    endpoint pair no longer threads at 0.10, or failed validity (train_narrow 592/667 kept, pilot
    120/150).
  - Start yaws re-oriented along the re-planned routes: median change 0.021 rad among unchanged
    starts in train_narrow. A few reversed (up to 3.13 rad); these are the departures the 0.10
    review anticipated.
- **The lethal radius is a grid guarantee, not a bound on the continuous path.**
  - 12 of 2,271 routes pass inside 0.3086 m between vertices: 11 by 2 mm or less, and one by
    15.1 mm (train_narrow seed 1,000,472, mid-route, an interior segment).
  - All 12 passed the validity oracle, which checks the band-0 field along the actual path, so none
    is a field breach.
  - The old sets did the same: tightest route 0.2291 against 0.2386, 9.5 mm.
  - Noted for the methods; it changes nothing in the criterion.

**Next:** commit the ten sets, then the oracle check on them
(`configs/analysis/planner_oracle_free_rebuilt.yaml`). Expected: no invalid or residual route, and a
free floor of zero.


### Reviewed 2026-10-16: the oracle check on the rebuilt sets -- PASS

`runs/analysis/ORACLE_FREE_decide_20261016T094301Z_seed0` (31a519f, clean, seed 0, scene 537f1bbb;
`planner_oracle_free_rebuilt.yaml`, lethal 0.10, soft 0.03). 2,271 rows, every route planned.

- **Every expectation stated in the config before the run holds:**
  - plannability 2,271/2,271 and no quantisation failures;
  - start-invalid 0, goal-invalid 0, residual 0 and bend-stop routes 0, in every stratum;
  - **free floor 0.000 per 100 m in every stratum.**
  - **A18 is confirmed end to end.** Limitation: this check runs the oracle the generator ran.
- **Consistency check:** the 740 narrow scenarios whose endpoints did not change in the rebuild give
  identical path lengths and tangent-locked stops in this run and in the 0.10 run on the old sets.
- **A1's (tangent-locked) floor on the final sets:** narrow 5.486 per 100 m (4.688 on the old sets
  at 0.10), medium 0.131, wide 0.040.
  - The whole rise is in the 117 narrow scenarios whose endpoints changed: 42 stops before, 100
    after.
  - **It is not the validity rules.** In the 12 scenarios they re-drew, A1's stops FELL from 18 to
    12.
  - **It is the aisle-threading rule at 0.10.** In the 105 scenarios whose first endpoint pair no
    longer threads an aisle at 0.10, the generator took the next pair, and stops rose from 24 to 88.
  - So the holonomic validity rule does not tilt the sets against A1. The margin itself does, by
    changing which endpoint pairs qualify. That is H4 context, reported beside L5's floor (A17).

**Next (ruled order, 2026-10-13/15):**
1. the declared recovery behaviour for the classical baselines (now B1, B2, B3, B6, B7);
2. the G4 controls rerun;
3. the L1 pilot;
4. B6 implemented and tuned before Step 0.


### Found 2026-10-16, drafting the recovery specification: one kind of deadlock is labelled `timeout`

Not fixed; awaiting the ruling requested in `docs/pending/recovery_spec.md` (section 0).

- **The intended rule:** ruling 4 makes a step-limit episode with the stop latched and the restart
  blocked an `unrecoverable_stop`. The 2026-09-28 entry says an intrusion inside the minimum
  margin "ends as `unrecoverable_stop`".
- **The code:** `CertifiedLayer.step` sets `restart_blocked` only while the at-rest field is
  clear. With an intrusion inside it, the latch holds forever, `restart_blocked` is False, and
  `episode_outcome` returns `timeout`.
- **It occurred:** read from the 2026-10-13 traces, both of their two "timeouts" are this case.
  `TRACE_20261013T215044Z` trace 2 and `TRACE_20261013T221601Z` trace 13 end at rest with the stop
  latched, `breached` True and `restart_blocked` False. Every other traced episode ends with
  `breached` False and `restart_blocked` True.
- **Why it matters:** A12's narrow-stratum rule counts `unrecoverable_stop`. Only the void B1
  control's traces are affected; no reported result is.
- **Proposed fix**, for every method alike:
  `stuck = stop_active and (restart_blocked or (breached and at_rest))`.
  It touches the frozen environment (`pfbnav/envs/reward.py`), so it needs a re-freeze.

The recovery specification itself (trigger, manoeuvre, limits, reporting, interface, methods
wording, tests, six questions) is drafted in `docs/pending/recovery_spec.md` for the supervisor.


### Ruled 2026-10-16 (supervisor): the classical recovery specification

Recorded before any implementation or measurement under it (`docs/pending/recovery_spec.md`).

1. **Labelling fix approved:** `stuck = stop_active and (restart_blocked or (breached and at_rest))`.
   - Known-answer tests for both stuck forms, and for a genuine timeout (the robot still moving at
     the step limit).
   - Re-freeze via the filter-off bit-identity test.
   - Additionally show that relabelling the 16 traced 2026-10-13 episodes changes exactly the two
     labels found.
2. **Numbers approved:** wait 1.0 s, speed 0.08 m/s, distance 0.30 m, 5 attempts per episode,
   72 directions with the route-direction tie-break.
   - Verify 0.08 m/s against the measured overshoot of the velocity estimate; lower it if overshoot
     can cross band 0's top speed (0.10 m/s).
   - **Translation only, no rotation.** Smart corner handling is B6's role, and stop-recover-stop
     cycles are the naive controllers' honest cost.
3. **Interface approved, enforced.** Classical controllers get a typed input object (stop flag,
   velocity estimate, the 120-bin stream, the route), never the raw `info` dict. A test shows that
   controller code cannot reach the pose, the emulator's timers or any other privileged key.
   Sharing the certified restart test as a pure function is approved; `pfb.py` and `certified.py`
   still never import each other.
4. **When recovery gives up, the episode runs to the step limit**, for every method.
5. **The primary denominator stays the distance actually driven.**
   - A14 declares two secondary versions: excluding recovery distance, and stops per mission
     normalised by planned route length.
   - If the primary and the per-mission version disagree in direction on any H1 comparison, report
     it prominently.
6. **Recovery covers B1, B2, B3, B6 and B7 identically** (A16 supersedes "B1-B5"). For B7,
   recovery commands pass through the PFB.

**Controls:**
- positive: the 2026-10-13 corner stop recovers;
- negative: zero-action never triggers recovery;
- second negative: a stop whose base-controller direction becomes permitted on its own must not
  trigger recovery.

**Then the G4 controls.**


### Implemented 2026-10-16: the classical recovery, the labelling fix, the typed controller input

Under the rulings above; before any simulator run of it.

**1. Labelling fix** (`pfbnav/envs/reward.py`, `nav_env.py` passes `breached` and `at_rest`):
- `stuck = stop_active and (restart_blocked or (breached and at_rest))`.
- `tests/test_reward.py` has known answers for both stuck forms and for three genuine timeouts:
  - a stop just fired with the robot still moving;
  - a breach with no stop latched;
  - a latched stop about to release.
- **The 16 traces relabelled** (`pfbnav/eval/relabel.py`, `scripts/relabel_traces.py`):
  - the old rule, restated independently, reproduces all 16 recorded labels;
  - the new rule changes exactly two, `TRACE_20261013T215044Z` trace 2 and `TRACE_20261013T221601Z`
    trace 13, both `timeout` -> `unrecoverable_stop`.
  - The recorded run is the researcher's (seconds). The diagnostic has its known-answer test,
    including a record the old rule cannot reproduce, which is flagged.

**2. Recovery speed verified, 0.08 m/s stands** (`pfbnav/eval/overshoot.py`,
`scripts/check_recovery_speed.py`):
- The environment's velocity estimator was replayed over the ground-truth poses of the 198 G1b runs
  (`G1b_20260926T1805*`, d765d97c, clean; carried over to 537f1bbb).
- The estimate never exceeded the largest command issued so far: max ratio 0.985 (forward,
  0.10 m/s).
- At 0.08 m/s the estimate therefore stays <= 0.079 m/s, 0.021 below band 0's top. There is no
  overshoot, because slip makes the platform undershoot (8-28 %).
- **Positive control:** the replayed steady-state speed reproduces every run's recorded realised
  speed to within 0.001 m/s.
- **Caveat:** WP2's declared noise sensitivity (`noise_std` non-zero) is not covered by this check.

**3. Interface, enforced:**
- `pfbnav/envs/controller_input.py` (a pure data module that imports nothing from `pfbnav`, so
  the environment's closure still excludes `pfbnav/control/`, as `tests/test_freeze.py`
  requires): `ClassicalInput` holds exactly stop flag, velocity estimate, the
  120-bin stream, the route ahead in the body frame, and dt. It is frozen and slotted; every array
  is a read-only copy.
- `NavEnv` builds it beside the observation (`info["controller_input"]`).
- The runner hands a classical controller ONLY that object, and a learned policy only the
  observation.
- `tests/test_controller_inputs.py` checks:
  - the field names, against a privileged-name list;
  - that every leaf is a float, bool or read-only array, and nothing can be attached;
  - with a runner probe, that the env's `info` carries the pose and emulator timers beside the
    input, yet the probe only ever sees `ClassicalInput`;
  - that classical controller modules import only an allowlist (no simulator, environment,
    planner pose or evaluation code).
- A sim test checks that the input agrees with the observation's lookahead, goal and stop channels.
- **The restart test is shared, not copied:** recovery calls the emulator's own
  `CertifiedLayer._restart_allowed` on a layer built exactly as the step loop builds it, reading
  only its geometry and that test, never its latch or timers.
  - A first version moved the test out into a module-level function, which moved the scenario
    identity: since A18 that hash covers the generator's import closure, which reaches
    `certified.py` through the oracle. It would have forced an amendment, a relock and a rebuild for
    no change in generation.
  - So `certified.py` is left byte-identical, and so is `planning/lookahead.py` (also in that
    closure): the body-frame route helpers live in `envs/route_view.py`.
  - The scenario identity still matches the A18 lock. The `pfb.py` and `certified.py` separation
    holds (`tests/test_architecture.py`).
- **B1 now reads the typed input** (the same lookahead bearing and goal distance it read from the
  observation). The zero-action control takes it too, so that recovery can sit over it.

**4. Recovery** (`pfbnav/control/recovery.py`, `baselines.recovery` in `configs/base.yaml`, the ruled
numbers):
- One wrapper for every classical controller; `scripts/evaluate.py` and the trace diagnostic wrap
  B1 and the zero-action control.
- `tests/test_recovery.py`, on synthetic ray-cast scans; each scene asserts its own preconditions:
  - parity with the emulator's restart test for 72 directions on five scenes, and field-for-field
    geometry;
  - the direction choice against an independent brute force, on a wall ahead and on the 2026-10-13
    corner geometry: heading 35 deg into a corner at 45 deg, it picks -15 deg, ahead and away;
  - the route-direction tie-break;
  - the trigger at exactly 60 steps; never overriding a permitted command;
  - **the second negative control** (a direction that becomes permitted on its own: no attempt);
  - **the zero-action negative control**;
  - the inside-margin and nowhere-to-go deadlocks;
  - every ending: cleared, distance at exactly 75 steps (0.30 m), new stop, not released;
  - the cap; the report; determinism.
- **Two refinements of the ruled specification, flagged for acknowledgement:**
  1. **The wait counts from when the robot's own field comes clear**, not from when the stop
     fired. The emulator times `restart_delay` from that moment, so counting from a stop that is
     still breached could request a direction the emulator would not yet release. The clear test is
     conservative: the at-rest field AND the field of the speed's band must both be clear. Recovery
     can therefore only start after the emulator would release, never before. This matches the
     2026-10-13 wording, "restart blocked beyond a declared wait after the restart delay".
  2. **`not_released`, a fifth ending**: the stop was not released on the first commanded step.
     With shared geometry and the refined trigger it should never happen. It is counted, never
     silent (hard rule 9), and the simulator positive control asserts it stays zero.

**5. A14's secondary rates** are computed in `outcome_summary`:
- `stops_per_100m_excluding_recovery`;
- `stops_per_100m_planned` (per 100 m of planned route);
- with the recovery distance and the planned length, all known-answer tested.
- The prominence rule (primary and per-mission versions disagreeing in direction on an H1
  comparison) applies at analysis time and goes into A14's text.

**6. Simulator controls** (`tests/test_recovery_sim.py`, the researcher runs them):
- **Positive:** the 2026-10-13 corner stop itself. Scenario 2000057 as built then
  (`tests/data/corner_stop_20261013.json`) is planned at that day's lethal 0.03; the route passes
  the recorded stop pose by 5.7 mm (fast test). In one scene load:
  - bare B1 must reproduce the deadlock: a stop near s = 1.45 m, still latched and blocked, route
    not advanced;
  - B1 with recovery must clear and get at least 0.5 m further along the route, with no
    `not_released`.
- **Negative:** zero-action over a full run makes no attempt and ends `timeout`.

**Re-frozen** 645358f2 -> 564802d8 (`reward.py`, `nav_env.py`, and `envs/controller_input.py` and
`envs/route_view.py` entering the closure). The filter-off bit-identity test completes it. No L1 run
exists.


### Reviewed 2026-10-16: the recovery checks -- four pass; the corner control recovers, then a new deadlock

Run by the researcher at the recovery commit (pytest under Python 3.13).

**Passed:**
- **Relabelling** (`runs/diag/RELABEL_20261016T133717Z_seed0`): the old rule reproduces 16/16
  recorded labels, and exactly the two predicted change, `timeout` -> `unrecoverable_stop`.
- **Recovery speed** (`runs/diag/RECOVERY_SPEED_20261016T133804Z_seed0`): ratio 0.985 at most,
  over 198 runs; 0.0788 m/s at 0.08, 0.021 below band 0. The positive control agrees within
  0.0011 m/s. **0.08 stands.**
- **The filter-off bit-identity test:** the re-freeze at 564802d8 is complete.
- **Zero-action negative control:** a full episode, no attempt, `timeout`.

**`test_the_classical_input_agrees_with_the_observation` failed on a fault in the TEST:** the
observation's lookahead distance channel is clipped at one lookahead (0.8 m) and float32, while
the typed input carries the true 0.8024 m of a robot slightly off its path. The test now compares
the clipped value and the bearing. It is to be rerun.

**The corner positive control** (`tests/test_recovery_sim.py`):
- **The control half held.** Bare B1 reproduced the 2026-10-13 deadlock: stop at s = 1.447 m,
  latched and blocked, no progress.
- **The ruled control held.** With recovery, the same stop was CLEARED: one attempt, 0.065 m of
  recovery motion, no `not_released`.
- **The test's own further expectation failed:** at least 0.5 m more of route, declared by me before
  the run. The wrapped robot got 0.15 m further (s = 1.600). It then stopped AT REST WITH ITS OWN
  FIELD BREACHED, which no direction can release; recovery recorded `inside_min_margin` and stopped
  intervening.
- **Not explained yet, and the expectation is NOT relaxed.** A stop ending inside the at-rest
  field should be rare, because a moving field abeam equals the at-rest field abeam. One unverified
  candidate: Pure Pursuit's yaw command turns the body toward the shelf, and the rotated footprint
  widens the at-rest field at that bearing; recovery, translation only as ruled, cannot undo a
  rotation.
- `scripts/diag_recovery_corner.py` traces both arms in one load so the cause can be read: what
  breached the at-rest field, at what bearing and range, and the body's yaw against the route.


### Reviewed 2026-10-16: the corner diagnostic -- the second stop is the 0.03 route's D1 geometry, not recovery

`runs/diag/RECOVERY_CORNER_20261016T135000Z_seed0` (e9e3f81a, clean, scene 537f1bbb). It reproduces the
test run exactly: bare B1 stops at step 126 (s = 1.447) and stays there; with recovery, stops at
126 and 218, 0.0654 m of recovery, `inside_min_margin`. The input/observation sim test now passes.

**Stop 1 (step 126):**
- The band-1 field was breached by the shelf end at bearing 83.4 deg, 0.2642 m (field 0.2664) --
  the 2026-10-13 stop.
- At rest the restart is blocked: B1 asks for about 4.6 deg left of straight, toward the shelf.
- Recovery waited out the 60 steps. Every permitted direction tied on clearance, and the
  route-direction tie-break chose **straight ahead, 0 deg**.
- It crept forward at 0.08 m/s for 0.065 m, until B1's own direction was permitted (`cleared`, step
  217): recovery working as specified.

**Stop 2 (step 218):**
- B1 resumed, accelerating toward 0.25 m/s while steering left to its lookahead point (requested
  vy 0.035 m/s, wz 0.21 rad/s). Cross-track grew from 9.7 to 16.5 mm.
- Above 0.10 m/s the band-1 field selected; the shelf corner, now behind-left at bearings 98-108
  deg, breached it by up to 5 mm, and a stop fired.
- During the response time and braking the robot coasted about 3 cm. It came to rest with the
  corner at bearings 111-114 deg, 0.5 and 2.6 mm INSIDE the at-rest field (0.2782 m against
  0.2787; 0.2817 against 0.2843).
- No direction can release that latch, and recovery correctly records `inside_min_margin`.
- **Not rotation:** the body's yaw against the route moved 0.7 deg, and a left turn swings the rear
  corners away from a shelf on the left.

**Why:**
- The fixture is planned at the old lethal margin 0.03 by design, to replay the 2026-10-13 route.
  That route passes the shelf end about 0.262 m from the robot's centre measured sideways, about
  3 mm beyond the at-rest field's 0.2595 m abeam (half_width + lateral_margin). This is the D1
  defect itself: the 0.03 lethal radius, 0.2386, lies inside the at-rest field.
- A few millimetres of tracking drift toward the shelf put the robot inside its own minimum
  margin.
- Under today's lethal 0.10, every route keeps 0.3086 m, so 49 mm before this deadlock; the 12
  routes that dip up to 15.1 mm inside the lethal radius between vertices keep 34 mm. This
  scenario's route at 0.10 passes 7.7 cm further from the shelf.

**Status of the positive control, for the supervisor:**
- Its ruled content, "the 2026-10-13 corner stop recovers", held: control reproduced, the stop
  cleared, no `not_released`.
- My own added expectation, at least 0.5 m further along the route, failed for the reason above,
  which recovery cannot reach by ruling (ruling 4: inside the minimum margin, recovery is
  supervised).
- I have not changed the test. Three options are put to the supervisor:
  (a) accept the control as passed, with the expectation replaced by "the stop clears and the
      robot moves on until any further stop";
  (b) keep the 0.5 m expectation and replay the 2026-10-13 stop pose under the 0.10 route;
  (c) let the G4 controls on the rebuilt sets (lethal 0.10) carry the evidence that B1 with
      recovery completes routes.


### Ruled 2026-10-16 (supervisor): the corner positive control; a synthetic control; then G4

Recorded before any further run.
- **The corner positive control is PASSED on its defining criterion:** recovery released a blocked
  restart in one attempt.
- **The additional 0.5 m progress criterion is FAILED**, with its cause:
  - the replayed route used the old 0.03 margin and passes about 3 mm outside the at-rest field;
  - B1 drifted 7 mm inward and stopped with the shelf inside its at-rest field;
  - that is correct latch behaviour (2026-09-27 ruling): an `unrecoverable_stop` attributable to B1
    on that route, not a recovery defect.
- **A synthetic recovery control on CURRENT geometry**, constructed at run time like G0's cases:
  - a shelf corner placed so that B1, tracking a 0.10-margin route, stops with its restart blocked
    and its at-rest field clear;
  - **required:** recovery releases the stop, and B1 then makes 0.5 m of route progress;
  - the geometry and criteria are pre-declared, with a known-answer check that the constructed
    scene produces the intended stop state.
- **The G4 positive control on the rebuilt sets carries route completion**, criterion 3 unchanged.
- **Recovery statistics are recorded per episode:** attempts, the outcome of each, deadlocks,
  recovery distance and time.
- **Then run the G4 controls.**


### 2026-10-16: the synthetic recovery control, declared and predicted BEFORE its simulator run

**Declared** (`configs/controls/recovery_synthetic.yaml`; the code is `pfbnav/eval/recovery_control.py`
and `scripts/control_recovery_synthetic.py`):
- **Scene**, built at run time like G0's cases, in the frame of the robot's start pose, with
  `NavEnv`'s own `transform_placing` and `LayoutShapes`, and read back with `verify`:
  - one shelf 2.0 x 1.0 m, near corner at (1.5, 0.55) m ahead-left, running back along the
    robot's left;
  - the goal 0.8 m ahead of and 0.6 m left of the corner;
  - the route is the planner's own at today's margins, clearance 0.340 m >= 0.3086 (a genuine
    lethal-0.10 route).
- **Two arms in one scene load:** bare B1 (the control arm), then B1 with the declared recovery.
- **Criteria:**
  - the route is a genuine 0.10 route;
  - the first stop comes within 200 steps;
  - at rest after it, the at-rest field is clear by >= 10 mm;
  - B1's restart is blocked (not permitted; band-0 leading field breached by >= 2 mm);
  - recovery releases the stop (at least one attempt, no `not_released`);
  - **B1 then makes 0.5 m of route progress (ruled);**
  - bare B1 advances <= 0.10 m after its stop;
  - no contact.

**The known-answer check** (`tests/test_recovery_control.py`, passing):
- The step loop's own chain is emulated: B1 or B1 + recovery, `CertifiedLayer`, the base's clamp,
  wheel budget and acceleration limiter, the pose-differencing estimate, and an exact ray-cast scan
  (`pfbnav/eval/raycast.py`).
- At slips 0.85, 0.92 and 1.0 the scene produces **the intended stop state**:
  - first stop at steps 115, 107 and 99;
  - at-rest margin 25.8, 21.9 and 18.9 mm;
  - restart blocked, depth 2.9, 7.7 and 11.4 mm;
  - **recovery releases it** in each.
- **The emulator's own positive control:** replaying the 2026-10-13 fixture reproduces the
  simulator's bare-B1 stop (emulated step 125 at slip 0.95, s = 1.432 against 1.447; rest
  s = 1.479 against 1.483).
- After a recovery it is only indicative. On that fixture it gave stop-recover cycles where the
  simulator gave one `cleared` and a deadlock, though progress was in the same 0.1-0.2 m range.

**Predicted before the run: the ruled 0.5 m criterion FAILS.**
- In emulation, progress after release is 0.153, 0.172 and 0.189 m.
- Each time B1 resumes cutting the bend while turning into it (yaw rate about 0.26 rad/s). The next
  band-1 stop coasts the corner into the at-rest field's FRONT-LEFT DIAGONAL: the footprint's
  corner plus the lateral margin reaches about 0.37 m there, against the moving field's smaller
  diagonal reach. That ends in `inside_min_margin`, B1's own, by ruling 4.
- The same held in all 16 corner geometries explored (progress 0.09-0.34 m). Gentler bends do not
  stop B1 at all.
- **Two contributing properties**, recorded for the supervisor and not changed:
  1. The ruled maximum-clearance choice ties across permitted directions, because the minimum is
     set by the nearest point, which lies abeam or behind every candidate. The route-direction
     tie-break then moves recovery ALONG the route, gaining no lateral room.
  2. A stop's response time and braking, and B1's own rotation, which the field sizing does not
     cover, carry an obstacle at a diagonal into the at-rest field.
- **Implication for G4:** B1 tracking 0.10 routes through aisle entries may meet this deadlock
  often, and the G4 positive control requires success >= 0.30. The simulator run of this control,
  and the G4 controls, will show whether the emulation's pessimism holds.

**Recovery statistics per episode (ruled):** `Recovery.episode_report()` now carries
`recovery_attempt_log` (each attempt's start step, heading, ending, distance and steps),
`recovery_deadlock`, `recovery_time_s`, distance and attempts. `scripts/gate_g4.py --controls-only`
reads the two control evaluations under G4's checks and criteria, adding these statistics
(`pfbnav/eval/g4.recovery_statistics`, known-answer tested).


### Ruled 2026-10-16 (supervisor): a rotation-aware protective field; recovery's clearance made continuous

Recorded before any implementation or run.
1. **Field sizing that ignores rotation is an environment defect.** The rotation-blind field was the
   supervisor's construction.
   - **The field must contain the footprint's full stopping sweep:** translation braking along the
     velocity direction AND yaw braking at the current yaw rate, each over response time plus
     deceleration, then offset by the margins.
   - **Enforce a symmetric yaw deceleration limit in the acceleration limiter**, so the assumption
     holds by construction. `alpha_max` remains VERIFY.
   - **Tests:** extend the monotonicity test to non-decreasing in |omega| as well as speed, at every
     bearing. Assert the field at omega = 0 is bit-identical to the current one, so D2, the strata
     and straight traversability carry over.
   - Record that WP2's PFB adopts the same construction.
   - Re-freeze via the filter-off bit-identity test.
2. **Recovery direction: "maximise clearance" implemented continuously.** The clearance is the
   minimum footprint-to-scan-point distance after the full 0.30 m manoeuvre, with the route
   tie-break only within 1 mm. This corrects the implementation to the declared rule; it is not a
   redesign.
3. **Rerun the oracle residual check on the rebuilt sets with the rotation-aware field**, residual
   rule unchanged. Amend and rebuild only if new residual routes appear.
4. **The synthetic control keeps its 0.5 m progress criterion.**
   - Re-run the offline emulation after fixes 1 and 2 and report its prediction.
   - Then run the simulator control against the unchanged criteria.
   - If it fails, report; do not reinterpret.

**Order:** 1, 2, 3, 4, then the G4 controls.


### Implemented 2026-10-16: the rotation-aware field (ruling 1) and recovery's continuous clearance (ruling 2)

**1. The rotation-aware field** (`pfbnav/safety/certified.py`):
- `yaw_sweep(w) = sign(w) (|w| t_response + w^2 / (2 alpha_brake))`.
- `FieldGeometry.swept_support(n, sweep)` is the support of the footprint rotated through every
  angle up to the sweep: max over phi of h_fp(n - phi).
  - It is taken on the support grid's step, one step past the sweep, and padded by
    R x step / 2 (R the footprint's largest radius, about 0.74 mm), because a support function
    turns no faster than R.
  - Without the pad the grid missed maxima between its angles by up to 0.2 mm. A brute-force
    union of rotated footprints caught it, so the field now contains the exact union and errs
    larger by at most a grid step and the pad.
- The translation sweep and the margins are added exactly as before. `extent`, `select`,
  `lateral_reach` and the warning field take `yaw_rate`.
- `CertifiedLayer.step` reads the estimated yaw rate. The restart test is unchanged: at rest, zero
  yaw rate.
- `CertifiedParams.alpha_brake` = `actuation.alpha_max`, passed by `certified_params_from_config`.
  One value; the limiter already applies it symmetrically to the yaw rate. No further change to the
  limiter was needed, and a test pins both directions.

**Tests:**
- **Bit-identity at zero yaw rate** against fields stored from the committed code before the edit
  (`tests/data/field_golden_20261016.npz`): 66 arrays, every band, five headings, protective and
  warning, 120 and 720 bearings.
- **Monotonicity** extended to non-decreasing in |omega|, and in speed at every omega, at every
  bearing, on the real footprint, both senses, up to w_max (`tests/test_certified_layer.py`).
- **Known answers** (`tests/test_certified_rotation.py`):
  - the sweep formula;
  - the sweep against the brute-force union, both bounds;
  - containment of every rotated footprint point;
  - the limiter's symmetric alpha_max;
  - a turning robot stops where a still one does not.
- D2 and the strata tests pass unchanged.

**Re-frozen** 564802d8 -> ff14176d. The filter-off bit-identity test completes it.

**The scenario identity moved** (`certified.py` and `strata.py` are inside its closure, A18), so
`tests/test_preregistration.py` fails until a relock, whatever the oracle check finds. A19 is
drafted (`docs/pending/A19_rotation_aware_field.md`): a relock, and a rebuild only to re-stamp the
sets, checked scenario by scenario to reproduce the committed build exactly
(`scripts/compare_rebuilt_sets.py`, `pfbnav/eval/set_compare.py`, known-answer tested). This is
flagged for the supervisor, since the ruling said "amend and rebuild only if new residual routes
appear".

**2. Recovery's clearance, continuous as ruled** (`pfbnav/control/recovery.py`):
- A direction's clearance is the minimum signed distance from the footprint, translated the full
  0.30 m along it, to every scan point. Unobserved bins are points at `range_min`.
- A wall ahead now gives straight back (the unique answer by symmetry); the 2026-10-13 corner gives
  -105 deg, back and away (it was -15 deg, creeping along the route).
- The test's brute force uses 5,760 boundary POINTS against the implementation's 720 segments.

**4. The emulation's prediction after fixes 1 and 2 (declared scene, unchanged): the control
FAILS.**

| slip | first stop | at-rest margin | restart blocked | released | progress after release | bare progress |
|---|---|---|---|---|---|---|
| 0.85 | step 114 | 31.7 mm | **no** (permitted) | yes | **0.097 m** | 0.074 m |
| 0.92 | step 106 | 28.4 mm | **no** (permitted) | yes | **0.104 m** | 0.080 m |
| 1.00 | step 98 | 26.0 mm | yes (2.8 mm) | yes | **0.068 m** | 0.048 m |

- **The stop state.** The rotation-aware field now stops B1 earlier, its own turning enlarging its
  field, and farther from the corner. At the lower slips its restart is therefore no longer
  blocked, and the scene's premise (the intended stop state) holds only at slip 1.0.
- **The progress fails at every slip.** Recovery backs away at -105 deg and releases the stop; B1
  then deadlocks inside its minimum margin within 0.07-0.10 m.
- **The cause (traced at slip 1.0) is NOT rotation: it is the velocity estimate's lag in band
  selection.**
  - Accelerating from the release, B1's actual speed was 0.131 and 0.179 m/s at steps 173-174
    (band 1), while the low-pass estimate (tau 0.10 s) read 0.039 and 0.068 m/s, so the band-0
    field was in force.
  - When the estimate crossed 0.10 (step 175), the corner was already inside the band-1 field. The
    stop fired, and the response time and braking carried the robot 3.4 cm on.
  - It came to rest with the corner 1.5-10 mm inside the at-rest field's front-left diagonal
    (bearings 37-50 deg).
- An estimate that lags actual speed under-sizes the field whenever the robot accelerates.
  - The 2026-09 ruling chose the estimate over the command because the command over-states speed
    while decelerating; this is the mirror case.
  - Not changed: for the supervisor.
- **Step 3's oracle rerun:** see A19's expectation (zero new residuals by construction).

**Test status after rulings 1-2 (fast suite: 1,179 pass, 3 fail, all three known):**
- `test_preregistration`: the lock, until A19's relock.
- `test_the_scene_produces_the_intended_stop_state[0.85]` and `[0.92]`: the declared scene's
  premise, failing as the emulation predicts under the rotation-aware field. Left failing, not
  adjusted after the result; for the supervisor.
- **The emulator's positive control is re-anchored to the field the simulator ran under.** The
  2026-10-13 fixture's simulator run used the rotation-blind field, so the emulator is now checked
  with `rotation_blind=True` (zero yaw rate handed to the layer). It passes again.


### Ruled 2026-10-16 (supervisor): field sized never below true motion; the synthetic premise re-tuned; A19

Recorded before any implementation or run.
- **(b) Approved: band selection uses max(estimated speed, applied post-limiter command speed), and
  the rotation sweep uses max(|omega_est|, |omega_applied|).**
  - Principle: the field is never sized below true motion. The command understates in
    deceleration, the lagged estimate in acceleration; the maximum is conservative in both.
  - **Invariant test** on acceleration and deceleration profiles, using simulator ground-truth
    velocity inside the test only: the selected band and rotation sweep are never below those
    implied by true speed and yaw rate, at any step.
  - Record that WP2's PFB uses the same composition. Record in the write-up that real AMRs use an
    unlagged safety-rated speed signal, while ours is privileged and lagged.
- **(c) Re-tune the synthetic scene so its PREMISE (restart blocked, at-rest field clear) holds at
  every declared slip in emulation.**
  - **Never tune toward the outcome:** release, 0.5 m of progress and no contact stay exactly as
    declared.
  - Declare the new geometry before the simulator run, re-emulate with (b) in place, report the
    prediction.
- **(a) A19 approved, folding in (b)**, so that `certified.py` changes and the sets are re-stamped
  once.
  - **The oracle residual check must actually run.** "Zero by construction" does not hold: tangent
    tracking at a bend implies omega = v kappa, non-zero even at band 0.
  - Report the count. If residuals appear, the residual rule decides, and the affected sets are
    rebuilt rather than re-stamped.

**Order:**
1. (b) with its invariant test;
2. the re-tuned scene and its emulation prediction;
3. A19 with the residual check;
4. the simulator bit-identity test for the re-freeze;
5. the synthetic control in the simulator;
6. the G4 controls.

Both simulator runs are held until then.


### Implemented 2026-10-16: (b), the field never sized below true motion

- `CertifiedLayer.step(..., applied=...)`:
  - band from max(estimated speed, applied speed);
  - rotation sweep from the yaw rate of larger magnitude;
  - the field faces whichever signal dominates;
  - it reports `field_speed`, `field_yaw_rate` and `yaw_sweep` beside the estimate.
- `StepLoop` passes `base.applied_command` (the command the base is executing, after the
  limiter). Without `applied` the layer behaves exactly as before.
- **Tests:**
  - `tests/test_field_never_below_motion.py`: a lagged, slipping base through acceleration and
    deceleration; band and sweep are never below true motion. The control shows the estimate alone
    falls short while accelerating, and the command alone while decelerating.
  - `tests/test_step_loop_sim.py::test_the_field_is_never_sized_below_true_motion` (simulator):
    ground-truth velocity read in the test only, forward-turning and diagonal-turning profiles, at
    every decision. Its positive control requires the truth to show band-1 speed and a real yaw
    rate.
- WP2's PFB uses the same composition. The write-up note is in `docs/writeup_notes.md`.

### Declared 2026-10-16, before the search: how the synthetic scene is re-tuned (ruling (c))

- **Only the PREMISE is evaluated in the search:**
  - the first stop within 200 steps;
  - at rest after it, the at-rest field clear by >= 10 mm;
  - B1's restart not permitted, with the band-0 leading field breached by >= 2 mm;
  - at every declared slip (0.85, 0.92, 1.0), in emulation with (b) in place.
- **Bare B1 only.** Recovery does not act before the rest state, so the premise is the same with
  it.
- **Not computed in the search:** release, progress and contact.
- **Candidates:** corner (1.5, cy), cy in {0.40, 0.45, 0.50, 0.55, 0.60}; goal from the corner
  (gx, gy), gx in {0.6, 0.8, 1.0, 1.2}, gy in {0.4, 0.6, 0.8, 1.2}; shelf 2.0 x 1.0 m as declared;
  the route a genuine lethal-0.10 route.
- **Selection rule:** among the candidates meeting the premise at every slip, take the largest
  worst-case block depth; then the largest worst-case at-rest margin; then the smallest cy, gx,
  gy.


### Implemented 2026-10-16: the residual check's rotation sweep (for A19)

- **Under the oracle's own semantics zero was not guaranteed, as the supervisor ruled.** Holding
  band 0's top speed, tangent tracking at a bend turns the body at omega = v kappa.
- `pfbnav/world/oracle.py`:
  - `tracking_yaw_rates`: omega = v x (the tangent's turn over a window centred on each sample) /
    window. The window is `planner.lookahead` (0.8 m), the distance a tracker takes to make a turn;
    a polyline's curvature otherwise sits at its vertices.
  - `rate_levels` rounds UP to `world.layout.validity.yaw_rate_step` (0.05 rad/s), so shapes are
    shared and the field only grows.
  - `state_tables(..., yaw_rates=...)` groups states by (tangent, rate level) and asks
    `shape_for(heading, yaw_rate)` for the swept shape.
  - `route_validity(..., tracking_window, tracking_speed, rate_step)`.
- `FieldShape.moving(..., yaw_rate)` gives the swept shape; at zero it is exactly the old one.
- **The generator's `ValidityCheck` uses the same construction** (band 0's top speed, the
  lookahead window), so a rebuild cannot reproduce a route the check rejects.
- **`scripts/quantify_oracle_free.py`** applies it to the band-0 reach and to A1's tangent-locked
  floor. The info phase's speed bands are unchanged; they are not part of the residual check.
- **Tests** (`tests/test_oracle_free.py`):
  - the rate known answers (a quarter turn spread over the window, zero on straights);
  - rounding up;
  - with the sweep a state's clearance never grows; at a bend it shrinks (the positive control).
- **Re-frozen** ff14176d -> 6366102b (`certified.py`, `step_loop.py`, `route_check.py`,
  `oracle.py`, `scenarios.py`).
- A19 is updated (`docs/pending/A19_rotation_aware_field.md`). It is no longer a pure re-stamp: the
  validity rule itself carries the sweep, so the residual check decides between re-stamping and
  rebuilding.


### Reviewed 2026-10-16: three runs on the pre-ruling code; a discontinuity found and fixed

**The researcher ran the previous Run Request** (bit-identity, oracle, synthetic control) at
41698be. The synthetic control's provenance says **dirty True**: it ran on a tree carrying
in-progress edits of (b). None of the three is a ruled result:
- **The bit-identity pass** was for environment ff14176d, since superseded.
- **The oracle check** (`ORACLE_FREE_decide_20261016T202621Z`; zero residuals, A1 narrow
  5.486/100 m) used the rotation-blind oracle, now superseded by the tangent-tracking sweep.
- **The synthetic control** (`CONTROL_RECOVERY_SYNTHETIC_20261016T204356Z`, dirty, old scene):
  - restart blocked FAIL, recovery released FAIL, progress 0.105 m FAIL;
  - two attempts `cleared` and three `not_released`.

**What the three `not_released` attempts revealed** (steps 334-336 of the recovery trace):
- From step 265 the emulator reported the robot at rest with its own field breached. Recovery,
  judging its own field clear, counted 60 steps and attempted; the emulator could never release.
- **Cause: the sampled rotation sweep added a fixed 0.74 mm pad for ANY non-zero sweep.** At rest
  the yaw-rate estimate was noise (about 1e-5 rad/s), so the emulator's at-rest field was 0.74 mm
  larger than at exactly zero. The shelf sat inside that sliver. Recovery, evaluating at zero,
  disagreed.
- **A field that jumps for a 1e-5 rad/s yaw rate is wrong in principle.**

**Fix: the sweep is now exact.**
- For a convex polygon, the support over an interval of directions is largest at the interval's
  ends or at a vertex direction inside it; the support's corners are never maxima.
- `FieldGeometry.swept_support` = max(the hull support at both ends, the largest vertex radius with
  its angle inside), through the footprint's convex hull (1,334 vertices). The ends use each
  angle's supporting vertex; the inside uses a sparse-table range maximum.
- Against a brute force over 400 angles: never below it, above it by at most 1e-7 m (the brute
  force's sampling). About 1 ms per call on the full grid; the first exact version took 181 ms.
- It is continuous in the yaw rate (`tests/test_certified_rotation.py::
  test_the_field_is_continuous_in_the_yaw_rate`, the regression for this bug). Bit-identity at zero,
  monotonicity, containment and the brute-force tests all pass.
- Recovery's own-field and deadlock checks now pass the estimated yaw rate too, so at rest they
  compute exactly the emulator's field.
- **Re-frozen** 6366102b -> the value now pinned in `configs/base.yaml`.


### Reviewed 2026-10-16: the premise-only scene search finds NO scene; the residual check crashed (fixed)

**Scene search** (`runs/diag/SEARCH_RECOVERY_SCENE_20261016T215444Z_seed0`; 2928d70e, clean; the
80 declared candidates x 3 slips, bare B1, rest state only):
- **No candidate meets the premise at every slip.** Failures per (candidate, slip):
  - restart blocked by less than 2 mm: 100;
  - restart permitted (not blocked): 66;
  - at-rest margin under 10 mm: 27;
  - no stop: 3.
- The best meet it at two slips of three. For example cy 0.50, goal (1.2, 0.8): blocked at all
  three, depths 0.5 / 4.1 / 6.7 mm; cy 0.45, goal (1.0, 0.4): depths 4.6 / 4.3 / 8.7 mm, at-rest
  margin 10.0 mm at slip 1.0.
- **Why, structurally:**
  - The corrected field (rotation sweep plus max(estimate, applied)) is larger while B1 accelerates
    and turns, so B1 stops earlier and farther from the corner.
  - At rest the corner then sits at the very edge of the band-0 LEADING field along B1's direction:
    blocked by millimetres, or not blocked, in which case B1 restarts by itself.
  - The corrected layers largely remove the restart-blocked corner deadlock the recovery was built
    for; what remains at a corner is a stop-restart cycle.
- **Not resolved here:** the declared rule produced no scene. Extending the grid, or reconsidering
  the control, is for the supervisor.

**Residual check:** the decide phase crashed in its workers. In A1's tangent-locked floor
(added with the rotation sweep), a loop variable `rows` shadowed the list of result rows. My
fault; the fast suite tested the oracle's pieces but not the script joining them. Fixed (`idx`),
with `tests/test_quantify_oracle_free.py` running the decide phase end to end on one tune scenario
(it fails on the old code). Nothing was written; the check is to be rerun.


### Ruled 2026-10-16 (supervisor): the synthetic control closed as a recorded negative result; recovery reporting

- **(b) chosen.** The synthetic recovery control is CLOSED as a recorded negative result.
  - Under the corrected fields (rotation sweep, max(estimate, applied)), 0 of 80 declared
    candidates robustly reproduce a restart-blocked corner stop for B1 across three slips.
  - The failure breakdown, per (candidate, slip): restart blocked by less than 2 mm, 100;
    restart permitted, 66; at-rest margin under 10 mm, 27; no stop, 3.
  - **The extended search is not run:** candidates blocked by millimetres would make an
    uninformative, slip-dependent control.
- **Write-up note:** the corrected field converts corner deadlocks into stop-restart cycles, and
  those cycles still count as protective stops in H1.
- **Recovery evidence:** the known-answer tests, the emulation, and the 2026-10-13 release.
- **The G4 positive control reports recovery triggers and each attempt's outcome.** If any attempt
  ends `not_released` while a permitted direction existed, stop and report it as a suspected
  recovery defect. The same reporting continues through WP3 and evaluation.
- **Proceed:** the residual check rerun; A19; the simulator bit-identity and ground-truth invariant
  tests; then the G4 controls.

**Implemented:**
- `pfbnav/eval/g4.recovery_defects` lists every `not_released` attempt. An attempt begins only once
  recovery has chosen a direction it judged permitted (no permitted direction is its own ending),
  so each one is a disagreement with the emulator.
- `recovery_statistics` carries it. `scripts/gate_g4.py --controls-only` returns
  "STOP: SUSPECTED RECOVERY DEFECT" if any appears, reading no verdict.
- `scripts/evaluate.py` prints each episode's attempt endings, writes `summary.json["recovery"]`,
  and exits 2 with the defect listed. Known-answer tested.
- The synthetic config is marked CLOSED and kept unchanged for the record. Its premise test is
  withdrawn rather than left failing; the route check and the emulator's positive control remain.

### Reviewed 2026-10-16: the residual check with the tangent-tracking rotation sweep -- zero

`runs/analysis/ORACLE_FREE_decide_20261016T215945Z_seed0` (e7f17fe, clean, scene 537f1bbb; all 2,271
routes, every one planned).
- **Residual 0, start-invalid 0, goal-invalid 0, bend-stop 0, free floor 0**, in every stratum.
- The residual rule says re-sample, with nothing to re-sample; **A19 re-stamps.**
- A1's floor with the sweep: narrow 5.564 per 100 m (5.486 rotation-blind), medium 0.131, wide
  0.040.
- The generator's validity rule now carries the same sweep, and the sweep only ever adds
  constraints. Every built route passes it, and every rejected candidate is still rejected, so a
  rebuild should reproduce the sets exactly. `scripts/compare_rebuilt_sets.py` checks that record
  by record.


### Reviewed 2026-10-17: A19's re-stamp (identical), the re-freeze, and the invariant test's first run

- **Rebuild** (`runs/diag/SCENARIO_SETS_20261017T132058Z_seed0`):
  - all ten sets built in full;
  - rejection counts identical to the A18 build (29 start-invalid, all narrow; 0 goal-invalid;
    0 residual);
  - placement and engagement unchanged.
- **The comparison against the A18 build 31a519f says ALL IDENTICAL**
  (`runs/diag/COMPARE_SETS_20261017T152235Z_seed0`): all 2,271 scenarios reproduced exactly,
  ignoring only `scenarios_sha256`. A19 re-stamped, as its residual check required.
  - (The researcher's build command reads `tune_widetune_unmapped_narrow`. Both sets were built,
    so the missing space is a transcription artefact.)
- **The filter-off bit-identity test passed:** the re-freeze at 968d88af is complete.
- **`test_the_field_is_never_sized_below_true_motion` FAILED at its first check.**
  - Step 0 of the forward-turning profile, just after the reset: the simulator's true yaw rate was
    1.1e-5 rad/s, while the layer's composed yaw rate was 0 (the estimator restarts at zero and the
    applied command is still zero).
  - That is a sweep of 1.1e-6 rad against 0, which moves the field by well under a micrometre.
  - A strict violation of the ruled invariant, at the noise floor of a robot settling after reset.
  - The test stopped at that first case, so it said nothing about the acceleration or braking that
    followed. It now collects every violation, each with the field it costs in metres (the largest
    radial shortfall of the field used against the one true motion implies), and asserts strictly
    as before.
  - To be rerun before the G4 controls. Not relaxed: any tolerance is the supervisor's to rule.


### Reviewed 2026-10-17: the invariant test's full report -- violations only at standstill, at most 1.06 um

`tests/test_step_loop_sim.py::test_the_field_is_never_sized_below_true_motion`, rerun with every
violation collected. **19 violations, all at standstill, none while the robot moves:**
- **Step 0 of both profiles**, the robot settling after the reset: true 6.9e-6 m/s and 1.1e-5
  rad/s against a composed 0 (the estimator restarts at zero; nothing applied yet). Field shortfall
  0.34 um.
- **The rest steps after braking:** steps 74-79 (forward-turning) and 66-79 (diagonal-turning).
  True speed at most 4.9e-5 m/s, true yaw rate at most 5.3e-5 rad/s; the composed yaw rate is
  the estimate's filtered value, and the applied command is zero.
- **Worst field shortfall 1.06 um**, about a thousandth of the scan's 1 mm resolution.
- **No band shortfall anywhere. No violation during acceleration, turning or braking** (steps 1 to
  about 65), where the positive control shows band-1 speed and more than 0.5 rad/s really reached.
- **Reading:** (b) does what it was ruled to do. What remains is the simulator's ground-truth jitter
  at standstill (tens of micro-radians per second), which a low-pass estimate smooths and a zero
  command cannot carry.
- **The test still fails as written.** A tolerance, or restricting the invariant to motion above
  the standstill threshold, is the supervisor's to rule. Under hard rule 19 a tolerance must state
  its expected false-alarm rate under the null and be null-tested; this run is a null test of the
  correct composition at standstill (max 1.06 um). The G4 controls are held for the ruling.


### Ruled 2026-10-17: the invariant's tolerance -- (ii), 10 um, with an independent null test

Supervisor's ruling on the invariant test, recorded before the test is changed.
- **(ii) adopted: a field tolerance of 10 um, applied at every step, in motion and at rest.**
- **(i) rejected:** excluding true speed below 0.01 m/s would leave slow creep unchecked. Near
  standstill dD/ds is about 0.1 s, so a missed 0.01 m/s would under-size the field by about 1 mm.
- **(iii) rejected:** the frozen safety layer is not changed to accommodate solver jitter in the
  test's ground truth.
- **Null test, in the same simulator session:** hold the robot at rest after several resets with no
  commands. Any shortfall there is a false alarm. Require zero exceedances of 10 um, and report the
  maximum.
  - Expected false-alarm rate under the null, declared before it runs (hard rule 19): zero. The
    tolerance is about ten times the largest standstill shortfall seen so far (1.06 um, the run
    reviewed above).
- **Then the invariant test on both profiles is the verdict.** Also report the smallest margin by
  which the composed field exceeds what true motion requires, in motion.
- **If the null test shows any exceedance, stop and report. Do not raise the tolerance.**
- Then the G4 controls.

How it is implemented (`tests/test_step_loop_sim.py`), one quantity for both tests:
- **The field true motion requires:** the field the layer would select from the TRUE planar speed
  and yaw rate, with rest decided by a second standstill detector fed the true speed. The detector
  has the same hysteresis as the layer's, so rest means the same thing on both sides. Both fields
  are taken at one heading, so only their size is compared.
- **Shortfall at a step:** the largest amount, over 360 bearings, by which the required field
  reaches beyond the field the layer used. A step fails above 10 um.
- **The margin in motion:** the smallest amount by which the used field exceeds the required one,
  over the steps where the true-speed detector says the robot is moving.
- The verdict test refuses to read a verdict unless the null test ran in the same session and
  passed.
- **Positive control:** each test shows its check flags the recorded fields once they are
  under-stated by twice the tolerance.
- **On the margin in motion:** at the field level it is at most 0 by construction. Behind the robot,
  and within one band at one yaw rate, the used and required fields coincide, so the smallest field
  margin reads 0 whenever composition is correct. A known-answer check on synthetic reports gives:
  exact match 0; composed above truth 0; yaw rate under-stated 24.5 mm; band under-stated 63 mm;
  standstill jitter 1.6 um; slow creep at 0.03 m/s under an at-rest field 70 mm, flagged. The test
  therefore also reports the margins of the two signals that size the field: composed minus true
  speed (m/s) and |composed| minus |true| yaw rate (rad/s), smallest over the steps in motion.


### Reviewed 2026-10-17: the null test passes, and the invariant passes at 10 um

`pytest -m sim tests/test_step_loop_sim.py -k "null_the_field_tolerance or never_sized_below_true_motion" -s`
at 128e654 (clean, scene 537f1bbb), one simulator session, the null test first:
- **Null test PASS:** 400 steps at rest over 5 resets.
  - Zero exceedances of 10 um, as declared.
  - Largest shortfall 1.70 um (step 1 after a reset; true 2.2e-5 m/s, -5.7e-5 rad/s).
  - The true-speed detector never left rest.
  - The tolerance sits about six times above the null maximum.
- **Invariant verdict PASS:** 160 steps on both profiles, with zero exceedances and zero band
  shortfalls.
  - Largest shortfall 1.45 um, at rest (diagonal-turning profile, step 58).
  - 89 steps in motion. The smallest field margin there is 0, as expected: the fields coincide
    behind the robot and within a band, and nowhere does the used field fall short.
  - Smallest signal margins in motion, both at step 1 of the diagonal-turning profile:
    speed +0.0070 m/s, yaw rate +0.0223 rad/s. Step 1 is the acceleration onset, where the
    applied command leads the true motion. Composed values never fell below the true ones while
    moving.
  - Positive controls held: band-1 speed and a yaw rate above 0.5 rad/s were reached, and the check
    flags fields under-stated by 20 um.
- **A second, unintended run in the same instance** (Claude's shell quoting executed the command
  while appending this entry; about 100 s; no run directory): both PASS.
  - Null maximum 1.49 um (step 16).
  - Verdict maximum 1.43 um, at rest.
  - Same margins in motion, zero exceedances and zero band shortfalls.
  - Recorded for completeness. It was not requested, and it changes nothing.
- **The ruling (b) invariant is verified.** The G4 controls proceed.


### Reviewed 2026-10-18: G4 controls -- zero-action fails as required; B1 fails (crashed at 51/60, verdict already fixed)

Both runs at 569317d (clean), scene 537f1bbb, environment 968d88af, display on, seed 0, the three
tune sets (60 scenarios).

- **Zero-action** (`runs/eval/zero_action_20261017T155909Z_seed0`, complete):
  - 60/60 timeouts: success 0.00, contact 0.00, no protective stops, no recovery attempt.
  - 0.11 m moved in total (settling).
  - **The control fails, as required.**
- **B1 Pure Pursuit with the classical recovery** (`runs/eval/B1_pure_pursuit_20261017T201329Z_seed0`):
  - **The process ended after episode 51 of 60** (tune_wide index 10). The researcher reports a
    crash, memory or otherwise; no traceback is in hand. The host has 7.7 GB of RAM, and the
    display was on.
  - `episodes.jsonl` holds 51 complete records. There is no `summary.json`, so `gate_g4.py` cannot
    be run on it.
  - **51/51 `unrecoverable_stop`.** Success 0, contact 0 (no episode touched anything).
  - 152 protective stops over 173.3 m: 87.7 per 100 m.
  - 300 over-budget steps (display on).
  - **The verdict does not depend on the missing nine.** Even nine successes give 9/60 = 0.15,
    below the ruled 0.30. **B1's control FAILS, so the G4 controls read GATE BROKEN.** This is
    recorded from the partial log, by arithmetic, not by the gate script.
- **Recovery.** No suspected defect: 87 attempts, every one ended `cleared`, and none was
  `not_released`. Endings:
  - 45 episodes ended deadlocked `inside_min_margin`: narrow 17/20, medium 19/20, wide 9/11.
  - 6 gave up at the 5-attempt cap.
- **The pattern** (from the records only; nothing below is yet traced):
  - **The deadlock episodes.** A stop; recovery moves a few millimetres and clears. Median of the
    last attempt 11 mm; 18 of 45 at 5 mm or less, some at 1 mm in one step. Pure Pursuit resumes.
    The next stop leaves the robot at rest with something inside the at-rest field, a deadlock
    by the ruled definition.
  - **The cap episodes.** Recovery backs out at about 180 deg, 8-46 mm each time, and Pure
    Pursuit drives back into the same stop, five times.
  - **The wide stratum fails like the narrow one.** So aisle width alone does not explain it.
  - The episodes run on to 1,500 steps after the deadlock (outcome assigned at the end).
- **Not known, and needed before any ruling:** what lies inside the minimum margin at the
  deadlock, and how the robot got there from a release only millimetres outside it. The candidates
  are untested and named only so the trace can test them:
  - (1) "cleared" ends the manoeuvre the moment the restart is permitted, which leaves the robot
    at the edge of the minimum margin;
  - (2) on release, Pure Pursuit turns, and the yaw rate grows during the response time beyond the
    rate the field was sized with;
  - (3) something else.
- **Diagnostic prepared (not run):** `scripts/diag_episode_trace.py` now records:
  - every step: the scan point nearest the footprint (signed distance, bearing);
  - at every stop trigger, release and final step (`events_<index>.json`): the bins inside the
    at-rest field and inside the field in force.
  - The helpers are pure (`pfbnav/eval/trace.py`: `scan_points`, `nearest_to_footprint`,
    `bins_inside`), with known-answer tests in `tests/test_trace.py`. Nothing in the frozen
    environment or the scenario identity changed (freeze check clean, preregistration tests
    pass).
- **The G4 controls stay BROKEN until a ruling.** No L1 pilot.


### Reviewed 2026-10-18: the deadlock traces -- the field does not contain the motion after a stop triggers

`runs/diag/TRACE_20261018T173222Z_seed0` (tune_narrow 3, 6, 10, 12; display off) and
`runs/diag/TRACE_20261018T175027Z_seed0` (tune_wide 1, 4; display on), both at ba25887 (clean), scene
537f1bbb. All six end `unrecoverable_stop`, as in the evaluation: five deadlocked `inside_min_margin`,
narrow 12 at the attempt cap. No contact.

**The deadlock is a stop that ends inside the minimum margin.** Recovery is not the cause:
- every release left the nearest point 58-81 mm from the footprint, outside the 50 mm minimum
  margin (`lateral_margin`);
- Pure Pursuit resumed, the field triggered, and the robot came to rest 37-49 mm from the obstacle.

So candidate (1) of the previous entry ("cleared" ends too early) is not the mechanism.

**Mechanism 1 (narrow 3, 6, 10; and translation in narrow 12): the robot keeps accelerating
through the response time, and the field assumes it does not.**
- After a trigger the command still passes for `response_time` (0.10 s, two steps) before the
  latch zeroes it, and Pure Pursuit is accelerating.
- The field is sized from the composed speed and yaw rate AT the decision, held constant over
  the response time, then braked. The composed applied term is also the previous step's.
- Measured, trigger to rest (rotation from the pose; sized sweep from `yaw_sweep`):

  | trace, trigger | yaw rate sized | peak applied | sweep sized | rotated | clearance |
  |---|---|---|---|---|---|
  | narrow 3, step 157 | 0.30 rad/s | 0.60 | 0.045 rad | **0.089 rad** | 74.1 -> 37.0 mm |
  | narrow 6, step 107 | 0.30 | 0.40 | 0.045 | **0.052** | 70.2 -> 38.8 mm |
  | narrow 10, step 78 | 0.30 | 0.60 | 0.045 | **0.089** | 75.2 -> 39.3 mm |
  | narrow 10, step 4 | 0.45 | 0.70 | 0.079 | **0.121** | 122.8 -> 76.2 mm |

  - The yaw rate doubles inside the response time: at alpha_max 3 rad/s^2, plus 0.15 rad/s a
    step. The robot rotates up to twice the sweep. There is no band slack in yaw.
- **Translation shows the same in band 0** (narrow 12, all five stops):
  - sized at 0.07 m/s, so band 0 with top 0.10 and depth 15 mm;
  - the applied speed rose to 0.17 m/s during the response time, and the robot travelled
    **23.6 mm**.
  - In band 1 the band's top speed (0.30) absorbed it: travel 40-60 mm against the 75 mm depth.
- **The 2026-10-17 invariant test passed and is not contradicted.** It checks the field against
  the true motion at each decision; it does not check the motion the robot can reach before
  braking takes effect.

**Mechanism 2 (wide 1 and 4): small, not yet separated.**
- Rotation (0.022-0.030 rad) and travel (42-48 mm) stayed inside what was sized.
- Yet the robot came to rest 46.8 and 49.2 mm from the obstacle: 0.8-3.2 mm inside the minimum
  margin.
- Candidates:
  - detection is discrete: an obstacle can enter the field up to one step (50 ms) before the
    decision that sees it;
  - bins are 3 deg apart;
  - the braking profile.

**Narrow 12 (the attempt cap) is a livelock, not a deadlock.**
- Pure Pursuit cuts the same shelf corner (about 33-40 deg, band 0) after each release.
- Recovery clears it five times. Each stop ends 60-88 mm away, outside the margin.

**Consequences.**
- The certified layer's field does not contain the robot's stopping motion whenever the command
  is accelerating at the trigger, in yaw at any band and in translation within band 0.
  - A stop then ends inside the minimum margin, which by the ruled definition is a deadlock.
  - With a smaller margin it would be a contact.
- That accounts for 45 of 51 B1 failures. The G4 B1 control failed on the safety layer's
  equation, not on B1 or the recovery.
- **No code changed.** A change to the field equation needs a ruling (hard rule 7). It moves
  `scenarios_sha256` (certified.py is in A18's closure) and the frozen environment.
- Ruling requested.


### Ruled 2026-10-18: the field is sized for the motion reachable before braking acts (option a)

Supervisor's ruling on the deadlock traces, recorded before any change.

**The invariant of 2026-10-17 tested the wrong quantity.** This was the supervisor's design. It
compared the field with the true motion at each decision. The safety property is
**containment**: everything swept from the stop trigger to rest lies inside the field in force at
the trigger.

**(a) adopted, subject to the impact assessment below.**
- The band is chosen from v + a_max T.
- The rotation sweep is sized from |omega| + alpha_max T.
- Braking runs from those reached rates.
- T = response time + one control step.
- Reachability is capped by the non-square velocity box, the wheel budget and w_max.
- **This applies at rest too.**

**(b) rejected.** A zero-latency clamp inside the response window does not match the hardware,
and it would make the certified layer a speed governor for every controller.

**Before adoption, an offline impact assessment:**
1. D2 and traversability for the narrow stratum, at rest and moving, at the 50 mm offset.
2. The oracle residual check on the rebuilt sets.
3. Re-emulation of the six traced B1 episodes (2026-10-18), for containment and minimum-margin
   outcomes.

**Declared rules:**
- **Narrow admissibility holds and the residual rule passes:** adopt fully. Then the amendment,
  relock, re-stamp or rebuild, and re-freeze with the bit-identity test.
- **Narrow admissibility breaks:** stop and report. The floor may be raised; the field is never
  shrunk. Stratum boundaries are the supervisor's ruling.
- **Also report narrow admissibility at alpha_max in {3.0, 2.0, 1.5} and a_max in {1.0, 0.5}.**
  The supervisor rules on the limits from a platform rationale, and any change re-runs G1b.

**The invariant test is replaced:**
- containment for every stop;
- no stop ending inside the minimum margin;
- any tolerance null-tested under rule 19.

**The evaluation crash after 51 episodes** (2026-10-17, B1): diagnose it, fix it, and add a test
(hard rule 9).

**Pre-declared:**
- If B1 with recovery still fails G4 after the fix, with no environment defect identified, the
  positive control becomes a privileged oracle tracker run in the simulator.
- B1's result is then recorded as a baseline finding.


### 2026-10-18: the reachable-motion field, assessed -- narrow admissibility BREAKS at the present limits (stop and report)

**Implementation, not adopted.**
- `pfbnav/safety/reach.py` is pure and imports only `certified`'s geometry. It lies outside the
  generator's and the environment's closures, so `scenarios_sha256` and the freeze do not move
  with it.
- **Reach time** T = response_time + one control step = 0.15 s.
- **Translation:**
  - the band is chosen from min(v + a_max T, cap);
  - the band depth is V T + V^2 / (2 a_b);
  - cap = the largest speed in the non-square box and the wheel budget: 0.507 m/s at (0.086, 0.5).
- **Rotation:** the exact interval of headings reachable by accelerating at alpha_max for T from
  the present rate, then braking.
  - Both senses: a slow rate can be reversed inside T.
  - Capped by min(w_max, budget / lever) = 1.5 rad/s.
- **At rest:**
  - the isotropic reach a T^2 / 2 + (a T)^2 / (2 a_b) (22.5 mm at a = 1);
  - the two-sided sweep +-alpha T^2 (0.0675 rad at alpha = 3).
- **The infimum is kept:** every moving field is the pointwise maximum of its own support and the
  at-rest support.
- **Two translation variants are reported, since the ruling's words cover only the first:**
  - "ruled": reach along the travel direction only;
  - "full": also the swerve an isotropic limiter allows. The velocity may turn by a_max T inside
    T and then brakes along its new direction, which any holonomic controller can command.
  - Containment for every controller needs "full". `tests/test_reach.py` shows by brute force
    that "ruled" does not contain a sideways command at a_max while moving.
- **`tests/test_reach.py`, 26 tests:**
  - known answers;
  - the yaw excursion against a brute-force bang-bang search;
  - the swerve bound against sampled velocity changes;
  - containment of simulated limiter trajectories (the traced turns, band-0 speed-up, swerves,
    from rest);
  - the infimum;
  - the negative control: the present field does not contain the traced turn (more than 5 mm out).
- **A wrong bound, caught by its own test before any use.** The first swerve bound assumed the
  largest current speed behind the robot; the slowest robot is carried furthest back. Fixed: a
  maximum over the band's speeds.

**Assessment (1), narrow admissibility** (`scripts/assess_reach_admissibility.py`).
- The criterion was declared in the script before computing. At clearance 0.13 and the 50 mm
  offset, every near-side gap must be > 0, geometric and binned (120-bin sector-min):
  - A1, at rest centred;
  - A2, at rest at the offset;
  - A3, moving at the offset for every band a covered speed can select.
- Preview by Claude (`--no-record`, no run directory); the recorded run is the researcher's.
- The present field: A1 79.1, A2 29.1, A3 29.1 mm.

| alpha_max | a_max | ruled: worst gap | full: worst gap |
|---|---|---|---|
| 3.0 (present) | 1.0 (present) | **-17.0 mm, BREAKS** (A2 -8.2 / -17.0) | **-53.4 mm, BREAKS** |
| 3.0 | 0.5 | +0.3 mm, admissible | -25.7, breaks |
| 2.0 | 1.0 | -11.2, breaks | -46.8, breaks |
| 2.0 | 0.5 | +6.6, admissible | -17.7, breaks |
| 1.5 | 1.0 | -8.0, breaks | -43.1, breaks |
| 1.5 | 0.5 | +9.7, admissible | -13.6, breaks |

- Gaps are binned, the binding measure. A2 (at rest at the offset) is the binding requirement for
  "ruled". Binning costs 8-9 mm more than the geometry here: the swept corners bulge between bins.
- **Medium and wide stay admissible in every case.**
- **At the present limits narrow admissibility breaks: stop and report.**
  - The narrow floor that would restore it: about 0.147 m ("ruled") or 0.184 m ("full").
  - The +0.3 mm at a_max 0.5, alpha 3.0 is within the real scan's ~1 mm cost: likely not
    admissible in the real scan.
- **The band structure under reach (also reported):**
  - At a_max 1.0, a T = 0.15 > band 0's top of 0.10, so band 0 is never selected while moving.
  - At a_max 1.0, composed speeds above 0.25 m/s select no band (`band_index` raises). At
    a_max 0.5: band 0 for v <= 0.025, and none above 0.325.
  - **Pre-existing, independent of reach:** vy_max 0.5 and the box-and-budget cap of 0.507 m/s
    exceed the top band's 0.40. A lateral command above 0.40 has no field today, and
    `band_index` raises. No run has commanded one; a learned policy could.
- **Not run, by the declared rule:** assessment (2) (the oracle residual check on rebuilt sets)
  and (3) (re-emulating the six traced episodes).

### 2026-10-18: the evaluation crash after 51 episodes -- diagnosed, fixed, tested; re-freeze 5

- **Evidence**, from the Windows event log, local time; the run's last write was 22:59:
  - 23:01:42, Resource-Exhaustion-Detector 2004, "low virtual memory". Top consumers: python.exe
    (PID 5136) 811 MB, msedge 756 MB, claude 531 MB.
  - 23:01:50 and 23:13:34, dwm.exe crashed with 0xC00001AD, fatal memory exhaustion.
  - 23:09:22, the same warning, python.exe (5136) still alive at 810 MB.
- **Diagnosis:** the host ran out of commit memory; the display was on, with Edge, VS Code and
  Claude open, on 7.7 GB. The evaluation did not die with a traceback: its process stayed alive
  and printed nothing more.
  - The ZeroMQ client waits on `socket.recv()` with **no timeout**. An instance that stops
    answering hangs the caller forever, silently, against hard rule 9.
  - The process's own size was mostly fixed cost. Importing the environment and torch commits
    709 MB, and torch alone about 210 MB, although a classical evaluation never uses it. That
    leaves about 100 MB of growth over 51 episodes, not a gross leak.
  - Whether CoppeliaSim itself hung or crashed is not in the log.
- **Fixes:**
  - **(i) `pfbnav/sim/client.py`:** every remote call raises `SimError`, naming the call, the
    instance and the timeout, when no reply comes within `sim.reply_timeout_s`. That is 120 s in
    `configs/base.yaml`, far above any single call. No behaviour change while the instance
    answers.
  - **(ii) `scripts/evaluate.py`:** imports SAC (torch) only for a SAC evaluation. Committed memory
    at start falls from 709 to 496 MB.
- **Test:** `tests/test_sim_client_timeout.py`, no simulator:
  - an unanswered instance raises within 5 s;
  - a late reply raises;
  - a timely reply gets through (positive control: the fake's own error arrives, so the timeout
    did not fire);
  - the timeout comes from the configuration, and 0 is refused.
- **Re-freeze 5:** client.py is in the environment's closure, so `env.frozen_code_sha256` moves
  968d88af -> cc233459. The filter-off bit-identity test must rerun on the simulator. The
  scenario identity is unchanged (preregistration tests pass).
- **Advice for long runs on this laptop:** display off; browser and editor closed.


### Reviewed 2026-10-18: the admissibility assessment recorded; re-freeze 5 complete

- **`runs/analysis/REACH_ADMISSIBILITY_20261018T190434Z_seed0`** (the researcher's recorded run):
  identical to Claude's preview, line for line. At the present limits narrow breaks:
  ruled -17.0 mm, full -53.4 mm.
- **The filter-off bit-identity test PASSED** on cc233459. Re-freeze 5 (the reply timeout) is
  complete.

### Ruled 2026-10-18: amendment A20 -- the reachable-motion field, its limits, the strata and the bands

Supervisor's rulings, recorded before implementation. One amendment, A20.

1. **Construction: FULL.** The field contains the swerve within T. The along-heading variant fails
   containment against holonomic controllers. It came from the supervisor's under-specification.
2. **Limits:**
   - **a_max = 0.5 m/s^2.** Band coherence: a_max T must be below the narrowest band width
     (0.10 m/s), so a_max < 0.67.
   - **alpha_max = 1.4 rad/s^2.** That is a_max / 0.351 m (the outermost radius), rounded down,
     so no body point accelerates faster than a_max.
   - Measure in the simulator that 0.5 m/s^2 is achievable: the limiter binds, not the torque cap.
   - Rerun G1b in full.
   - Report the oracle mission time, old against new.
   - **Methods text:** the limits are a safety-design parameter co-specified with the fields, not
     measured youBot properties.
3. **The narrow floor F** = the smallest per-side clearance with all three gaps >= 0 (binned)
   under the full construction at the new limits, plus the recorded 0.03 m buffer.
   - Strata: narrow [F, F + 0.07], medium [F + 0.07, 0.40], wide [0.40, 0.80].
   - Stop and report if F + 0.07 > 0.30; otherwise proceed and report the values.
4. **Bands:**
   - Indexed by reachable speed (v + a_max T, capped by the box and the budget):
     **[0.10, 0.30, 0.51]**.
   - The top band covers the largest reachable composed speed. This fixes the uncovered lateral
     range above 0.40, a defect of the supervisor's G1b ruling.
   - Band 0 is the creep/departure band.
   - The recovery speed is re-derived so its reachable speed stays in band 0:
     **v_rec = 0.02 m/s**. The distance and attempt caps are unchanged.
   - Report if recovery occupies a material share of G4 episode time.
5. **Crash:** the fixes are accepted.
   - Also measure Python's and CoppeliaSim's memory across >= 200 resets.
   - Declare a bound on growth per 100 episodes before measuring.
   - Any monotonic growth beyond it must be fixed before training.

**Order:**
1. Implement, and compute F.
2. Simulator session: the acceleration check, full G1b, the memory check, bit-identity.
3. The oracle residual check and the rebuild.
4. Re-emulate the six B1 traces, then the containment test with a null-tested tolerance.
5. The G4 controls, with the pre-declared oracle fallback.


### 2026-10-18: A20's narrow floor F = 0.1735 m (proceed); two corrections to the at-rest reach

**F**, computed by `scripts/compute_narrow_floor.py`: the full construction, a_max 0.5,
alpha_max 1.4, bands [0.10, 0.30, 0.51] by reachable speed, T 0.15 s, speed cap 0.5074 m/s.
Preview by Claude; the recorded run is the researcher's.
- **The zero-gap clearance is 0.1435 m.**
  - The binding requirement is A3 in the top band (0.51), moving at the 50 mm offset: binned
    +0.00 mm, geometric +10.99 mm. Binning costs 11 mm against this field.
  - A1 at rest centred: +72.1 mm. A2 at rest at the offset: +21.0 mm. A3 at 0.10: +21.0 mm.
    A3 at 0.30: +15.4 mm.
- **F = 0.1435 + 0.03 = 0.1735 m.** At F the worst binned gap is 40.1 mm.
- **Strata:** narrow [0.1735, 0.2435], medium [0.2435, 0.40], wide [0.40, 0.80].
  F + 0.07 = 0.2435 <= 0.30: **proceed**, as ruled.
- **The band structure under reach:** band 0 for composed speeds <= 0.025 m/s, band 1 for
  0.025-0.225, band 2 for 0.225-0.435. Above 0.435 the capped reach (<= 0.5074) still falls in
  band 2, so every reachable speed is covered.
- **The outermost radius in the ruling (0.351 m)** matches neither the footprint table (0.3374 m)
  nor the rectangle's corner (0.3535 m). a_max / r rounded down is 1.4 rad/s^2 for all three, so
  the ruled limit stands.

**Two corrections to `pfbnav/safety/reach.py`**, made before adoption and found while checking
the recovery speed against the standstill thresholds:
- **(i) The at-rest reach assumed zero speed.** The layer calls the robot at rest up to the
  standstill EXIT speed (0.02 m/s, hysteresis), and the recovery now creeps at exactly that speed.
  The at-rest reach is now isotropic from s <= 0.02: s T + a T^2 / 2 + (s + a T)^2 / (2 a_b).
- **(ii) The at-rest field ignored the yaw rate.** A robot turning on the spot is at rest by speed.
  It now sweeps the union of the headings reachable from zero and from the composed yaw rate, as
  the present layer already sweeps its yaw rate at rest.
- **New tests:** containment from five at-rest states (creep in any direction, turning on the
  spot); a negative control showing that a zero-speed at-rest field does not contain creep that
  accelerates; the swerve control moved to 0.25 m/s.
- F is unchanged by both, since the binding requirement is the moving top band.


### 2026-10-18: A20 implemented (step 1 of the ruled order)

**The certified layer** (`pfbnav/safety/certified.py`) now carries the FULL reachable-motion field.
The pure reach functions moved into it, so the layer, the generator and the assessment share one
implementation: `reach_speed`, `band_speed_limit`, `rest_reach`, `band_depth`, `yaw_excursion`,
`yaw_interval`, `swerve`, `box_budget_speed_cap`.
- **`CertifiedParams` gains** `reach_time`, `a_max`, `alpha_max`, `speed_cap` and `yaw_cap`, all
  required. `rest_speed` is the standstill exit.
- **The parameters are validated against both A20 rules:**
  - a_max T below the narrowest band width;
  - the top band covering the speed cap.
- **`FieldGeometry`:**
  - `swept_interval` (exact, from the hull);
  - `rest_support`;
  - `support` = max(swept + travel/swerve + margins, the at-rest support at the same yaw rate);
  - `band_for` (the band from the reachable speed);
  - `select` uses it.
- **The layer reports** `reach_speed` and `yaw_interval` in place of `yaw_sweep`. `stopping_depth`
  and `yaw_sweep` are gone.
- **One construction choice, against the ruling's words:** the yaw interval is symmetric,
  +-e(|omega|).
  - The ruling sizes the sweep "from |omega| + alpha_max T". A signed interval (the body turning
    left can barely swing right) made the field shrink on one side as |omega| rose, against the
    2026-10-16 requirement that it never shrink with the yaw rate.
  - The symmetric interval is conservative on the side the body is not turning toward, and is
    monotone in |omega|. F is unchanged by it (0.1735), since the binding case is at omega = 0.
- **`strata.certified_params_from_config`** derives the reach parameters:
  - T = response_time + sim.dt;
  - the speed cap from the box and the base's final budget ceiling (the filter share);
  - yaw cap = min(w_max, budget / lever).
- `pfbnav/safety/reach.py` keeps only the assessment's extras: overridden limits and the rejected
  "ruled" variant. `tests/test_reach.py::test_the_adopted_field_is_the_assessed_full_variant`
  pins the two together.

**Configuration** (`configs/base.yaml`):
- a_max 0.5 and alpha_max 1.4, with the methods note: a safety-design parameter, not a youBot
  property;
- band_top_speeds [0.10, 0.30, 0.51];
- strata narrow [0.1735, 0.2435], medium [0.2435, 0.40], wide [0.40, 0.80];
- baselines.recovery.speed 0.02 (`Recovery` now checks that its reachable speed stays in band 0
  and that it is no faster than the standstill exit).

**The generator and the oracle drive at each band's speed limit, not its top** (top - a_max T:
0.025 / 0.225 m/s; the top band the cap, 0.5074):
- `ValidityCheck.tracking_speed`;
- `quantify_oracle_free` (the rotation-sweep yaw rates, the oracle's speed caps, and the
  constant-speed time, capped at v_max).
- The field shapes stay each band's own.

**Tests re-derived** (all numbers from the A20 formulas, each stated in its test):
- `test_certified_field_cache` (the reference arithmetic written out for A20);
- `test_certified_layer` (band selection 0.022 / 0.15 / 0.25 -> 0 / 1 / 2; 0.50 m/s now covered,
  0.52 raises; warning distances 0.400 / 0.545, 0.517 / 0.720, rest 0.273 / 0.293);
- `test_certified_rotation`: golden fields stored for A20 in
  `tests/data/field_golden_A20_20261018.npz`. The 2026-10-16 file is kept, and its bit-identity
  test ended with A20, which changes the field by design;
- `test_field_never_below_motion`;
- `test_d2_aisle_strata`:
  - rest floor 0.0709;
  - traversal 0.51 everywhere;
  - narrow gap at the offset 41.0 mm (warning extension 20 mm inside);
  - binned margins 102.3 / 90.6 / 40.1 mm;
  - stop offsets 0.1026 / 0.1726 / 0.3291;
  - the sideways reach now grows with the band (the swerve);
  - the at-rest field still touches band 0.
- **Three tests pinned to the real-scan MEASUREMENTS of the old field are skipped, with the reason,
  until D2 is re-measured in the simulator** (hard rule 15: a model value never replaces a
  measurement).

**Simulator-session tools:**
- **`scripts/check_acceleration.py`:**
  - five profiles through the base's own path;
  - C1 (the limiter binds, exactly) and C2 (zero wheel-steps at the torque cap during
    acceleration, an exact count, so no tolerance to null-test);
  - a positive control: a velocity step with the limiter bypassed must hit the cap, or the
    verdict is INCONCLUSIVE;
  - achieved accelerations reported, not gating.
- **`scripts/check_memory.py`:**
  - 220 episodes (20 warm-up), with scenarios generated from a dedicated seed range;
  - **the bound declared in the script before measuring: <= 10 MB growth per 100 episodes per
    process**, as the least-squares slope of private memory;
  - INCONCLUSIVE if the slope's standard error exceeds a third of the bound.
- Known-answer tests for both (`tests/test_check_scripts_a20.py`). They caught a precedence bug
  in the acceleration check's C1 before it ever ran.

**Drafts for the researcher** (the frozen files are not touched):
- `docs/pending/A20_reachable_motion.md` (the amendment);
- `docs/pending/endpoints_A20.yaml` (H2's narrow population, 0.1735-0.2435 m);
- `docs/pending/scenario_sets_A20.yaml` (the bypass clearance's stated value).

`scenarios_sha256` and the environment's identity both move. The sets are REBUILT, not
re-stamped, because the strata change the layouts.


### 2026-10-18: A20 re-freeze 6 (cb9c839c) and the remaining test re-derivations

**Fast suite after the implementation: 21 failures in four groups.**
- **Re-freeze, expected:** `test_freeze`, `test_display_flag`, three `test_runner` and one
  `test_training` test refused the moved environment.
  - **Re-freeze 6: `env.frozen_code_sha256` cc233459 -> cb9c839c.**
  - This is a filter-off BEHAVIOUR change by design: the fields, the limits and the bands all
    moved.
  - No L1 run exists. The filter-off bit-identity test reruns in the simulator session.
- **The lock, expected until the amendment:** `test_preregistration` (the generator's closure
  moved). The researcher applies A20 and relocks.
- **Rebuild, expected until step 3:** `test_extent` on seven narrow and medium sets. `world_of`
  no longer reproduces their layouts, because the strata moved, and it refuses as designed. The
  wide sets still reproduce, since A20 leaves wide [0.40, 0.80] unchanged.
- **Re-derived:**
  - `test_layouts::test_stratum_of_known_answers`: the new strata; 0.15 m now lies below the narrow
    floor and raises.
  - `test_plannability`: the positive control's lethal margins are now set past the edge relative
    to F (F - cell/2 + 2.5 to 10 mm), not past 0.13. Beyond 10 mm the lethal radius would pass the
    soft one, which the planner refuses.
  - `test_oracle_free::test_state_tables_at_the_tangent_yaw_match_the_tangent_locked_check`: the
    known clearance difference between two orientations is now the SWEPT footprint's support
    difference (the yaw interval from rest).
  - `test_recovery_control::test_the_emulator_reproduces_the_simulators_2026_10_13_stop` is
    **skipped with its reason.** Its fixture was recorded under the pre-A20 field and limits,
    which no longer exist in code, and `rotation_blind` no longer recreates any historical field
    (its docstring is corrected).
    - The emulator's positive control is re-established on a fresh A20 simulator trace of the same
      scenario. It is tune_wide 2000057, whose world still reproduces.
    - The run is `scripts/diag_recovery_corner.py`, in the simulator session, and is needed before
      the six traces are re-emulated (step 4).


### Reviewed 2026-10-19: the A20 simulator session (step 2)

All runs at f839db3 (the A20 relock), clean, scene 537f1bbb, display off. F's canonical run is
`runs/analysis/NARROW_FLOOR_A20_20261018T202400Z_seed0` (077e63e, clean): F = 0.1735, strata as
ruled.
- **Record defect:** the frozen `AMENDMENTS.md` A20 text still reads `NARROW_FLOOR_A20_<stamp>`.
  The next amendment should carry a one-line erratum naming this run.
- **Not yet in hand:** the filter-off bit-identity test and the D2 real-scan test print to the
  console only (pytest; no run directory). Their output was requested.

**Acceleration** (`runs/diag/ACCELERATION_20261018T202610Z_seed0`): **PASS**.
- C1 held on every ramp step of all five profiles: the limiter binds, exactly.
- C2: zero wheel-steps at the torque cap during any acceleration ramp, and none at all in the five
  profiles.
- The positive control (a velocity step with the limiter bypassed) put wheels at the cap 16
  times, so the counter sees.
- Reported, not gating: the achieved acceleration from the simulator's velocity is
  0.78-0.86 x a_max and 0.88-0.93 x alpha_max. The body accelerates no faster than the field
  assumes, which is the conservative direction, and the shortfall is consistent with the 8-28 %
  command-to-realised slip measured in WP0. Achieved braking is 0.87-0.96 x a_decel_max.

**The corner trace under A20** (`runs/diag/RECOVERY_CORNER_20261018T202835Z_seed0`; tune_wide
2000057 at the fixture's lethal 0.03):
- **Bare B1 no longer deadlocks.** Six stop-restart cycles carry it from s = 0.957 to 1.154 m in
  500 steps.
- **With recovery:** eight stops, two attempts (both cleared), no deadlock, s = 1.211 m.
- **No stop, in either arm, ended with anything inside the at-rest field.** Under the old field
  this corner ended inside it.
- This trace is the new reference for the emulator's positive control, to be re-pinned in step 4.

**Memory** (`runs/diag/MEMORY_20261018T203246Z_seed0`; 20 warm-up + 200 measured episodes; the
bound of 10 MB / 100 episodes declared before measuring): **PASS**.
- Python: -0.01 MB / 100 (standard error 0.24), 705 -> 709 MB private.
- CoppeliaSim: +1.71 MB / 100 (standard error 0.11), 187 -> 192 MB.
  - A small, real upward trend, inside the bound: about 50 MB over a 3,000-episode training run.
  - Recorded, so a training-host run can be compared with it.
- Resident memory fell in both processes: working-set trimming by Windows. Private memory is the
  declared measure.

**G1b in full** (three invocations: `G1b_20261018T205036Z_seed0`, `G1b_20261019T073425Z_seed1`,
`G1b_20261019T145746Z_seed2`; aggregate `runs/gates/G1b-SPREAD_20261019T153126Z_seed0`):
**PASS.**
- Within-invocation repetitions are bit-identical. Every cell passes at every grid speed in every
  invocation.
- **By the gate's outcome rule, v_max = 0.50 and vy_max = 0.50** (it was 0.40 / 0.50).
- **The thin cell is translate_while_rotating at 0.50 m/s:** final heading error 4.947-4.962 deg
  against 5.0, a worst margin of 0.038 deg.
  - On 2026-09-20 the same cell FAILED by 0.03-0.05 deg (5.033-5.052), which is what set v_max to
    0.40.
  - The lower accelerations of A20 moved it across the line.
  - Every other cell has wide margins (cross-track at most 13.7 mm against 50).
- **Not adopted.** D8 wrote G1b's outcome into the box, but under A20 the limits are the
  supervisor's to rule, and a 0.038 deg margin is a ruling, not a measurement to copy.
- A box of 0.50 / 0.50 leaves the speed cap at 0.5074 m/s (the budget binds), so the bands and F
  are unaffected either way. Only the oracle's mission time and the robot's forward speed would
  change.


### Ruled 2026-10-19: v_max = 0.50 per G1b's outcome rule; the box square again

Supervisor's ruling, recorded before the change.
- **Adopt v_max = 0.50** per G1b's declared outcome rule.
  - Three invocations pass, with a spread (0.015 deg) below the margin (0.038 deg): a consistent
    pass, not a straddle.
  - September's FAIL at a similar margin was accepted, so the rule applies symmetrically.
  - v_max is a tracking ceiling, not a safety parameter: the fields cover reachable speeds to
    0.51 m/s regardless.
- **Record:**
  - in the write-up, the 0.038 deg margin at 0.50 m/s, noting that the result is specific to the
    A20 limits;
  - the box is square again (v_max = vy_max = 0.50), and the asymmetry material is labelled
    pre-A20, not deleted;
  - achieved acceleration of 0.78-0.86 x a_max makes the field's reach assumption conservative;
  - memory growth (+1.7 MB per 100 episodes) stays in every run record.
- **Next:**
  - rebuild the sets and run the oracle residual check;
  - report the oracle mission time, old against new, at v_max 0.50;
  - re-emulate the six B1 traces;
  - the containment test;
  - the G4 controls.
- Checked before the change: the generator reads v_max only through the speed cap, which stays
  0.5074 m/s (the wheel budget binds), and the oracle's limits enter only the mission-time phase.
  So the rebuild started 2026-10-19 at v_max 0.40 (`runs/diag/SCENARIO_SETS_20261019T153556Z_seed0`)
  is unaffected.


### 2026-10-19: v_max 0.50 applied; memory in every run record; containment tooling; the emulator re-pinned

**The ruling applied** (configuration only, so the environment's code identity does not move):
- `actuation.v_max` 0.4 -> 0.5, with the G1b record (0.038 deg at 0.50, specific to the A20
  limits). The pre-A20 reasoning is kept and labelled PRE-A20, not deleted, in `configs/base.yaml`
  and `docs/writeup_notes.md`.
- Tests re-derived for the square box:
  - `test_footprint_config`;
  - `test_wheel_budget`: the corner is now 1.00 m/s at the rim, 45 deg off the nose;
  - `test_robot_kinematics`;
  - `test_g1b_verdict`: the overlay stays as the declared measurement ceiling. Base now equals the
    grid's top; A6 re-read; the test now asserts the overlay is never below base.
- **The write-up** (`docs/writeup_notes.md`, a new A20 section):
  - the reachable-motion field;
  - the limits as a safety-design parameter;
  - achieved acceleration 0.78-0.86 x a_max, which makes the reach assumption conservative;
  - the 0.038 deg margin;
  - the square box;
  - memory growth.

**Memory in every run record** (`pfbnav/utils/memory.py`, outside the frozen closure):
- `scripts/evaluate.py` and `scripts/train.py` write `summary["memory"]`: Python's and
  CoppeliaSim's committed memory at the start and end, and growth per 100 episodes, against the A20
  reference (+1.7 MB / 100).
- A value that cannot be measured (psutil absent, the simulator process not identifiable, the
  process gone) is recorded as a status, never a gap (hard rule 9).
- `tests/test_memory_record.py`.

**Containment** (`pfbnav/eval/containment.py`, known answers in `tests/test_containment.py`):
- the footprint swept from the trigger's decision pose to rest, expressed in that frame, against
  the field in force at the trigger;
- the at-rest margin once still.

**The simulator test that replaces the invariant** (`tests/test_containment_sim.py`; the invariant
test and its null test are removed from `tests/test_step_loop_sim.py`, with a pointer):
- The tolerance, 1 mm on both checks at every stop, is declared in the file.
- Expected false alarms under the null: zero.
- **Null test** in the same session first: stops at constant velocity, where the reach terms are
  unused.
- **Positive control:** on the accelerating runs' recorded poses, a field without reach must exceed
  the tolerance, or the verdict is inconclusive.
- **Runs:** forward from rest, turning while accelerating, a swerve toward a side wall, spinning
  beside a wall. Walls are placed in memory only.

**The emulator's positive control under A20** (`tests/test_recovery_control.py`):
- The new reference is `tests/data/corner_stop_A20_20261019.json`, taken from
  `runs/diag/RECOVERY_CORNER_20261018T202835Z_seed0`: bare B1, six stops, ending stopped and
  restart-blocked at s = 1.154 m.
- At the control's own slip, 0.95 (declared before the A20 run; not re-fitted), the emulator
  reproduces the first stop (step 88 vs 87, s 0.963 vs 0.957), the stop count (5 vs 6, within +-1),
  the final position (1.160 vs 1.154) and the final latch.
- The recovery arm drifts more (final s 1.28 vs 1.21), so the control stays on the bare arm, as
  before.
- `recovery_control.emulate_layout` now records each decision's pose and field heading, for
  containment.

**Re-emulation of the six B1 traces** (`scripts/reemulate_b1_traces.py`):
- the same scenarios, read from git at f839db3;
- narrow worlds rebuilt with the pre-A20 strata, because they lie below the A20 floor;
- B1 under the recovery, 1,500 steps, slip 0.95;
- per stop, containment and the rest margin.
- About 2 minutes per episode, so it is a researcher run. Claude's own preview was stopped at
  10 minutes, under hard rule 11.
- A 400-step smoke test on tune_wide 1: first stop contained (excess -63 mm), at rest 56 mm outside
  the minimum margin.

**Mission time** (`pfbnav/eval/mission_time.py`, `scripts/compare_mission_time.py`, known answers in
`tests/test_mission_time.py`):
- Per stratum: traversable share, median time, median time per planned metre.
- Matched by (set, seed): the median new / old ratio.
- Only the wide pairs are the same routes; A20 left wide unchanged.
- **OLD** is an info run of the last pre-A20 code (28afe21) on the pre-A20 sets, in a separate
  worktree. The 2026-10-15 info run predates A18/A19, so it is no baseline.

- **Hard rule 7: v_max is also a normalisation constant.** Through the per-axis ceilings
  (`NavEnv.ceilings`), v_max scales three things:
  - the action: command = a x ceilings;
  - the observation's velocity channels (`pfbnav/envs/observation.py`);
  - the reward's intervention term (`reward.intervention_magnitude`).
  The equations are unchanged, but the forward constant moved 0.40 -> 0.50, so a learned method's
  MDP changes with the ruling.
  - No L1 run exists, so nothing is invalidated. Recorded here, in the same change, as rule 7
    requires.
  - `tests/test_reward.py` re-derived: the configured ceilings; the per-axis property checked on
    an explicit asymmetric box.
  - `observation.py`'s docstring still reads "(0.40 / 0.50)". It is inside the frozen closure, so
    it is corrected at that file's next ruled change rather than with a re-freeze for a comment.
  - Flagged to the supervisor: the ruling treated v_max as a tracking ceiling only.


### Reviewed 2026-10-19: the sets rebuilt under A20

`runs/diag/SCENARIO_SETS_20261019T153556Z_seed0` (18b9c8a, clean; seed 0; the scenario module matches
the A20 lock; started before v_max moved, which the generator does not read except through the
unchanged speed cap).
- **Every set built in full** (2,271 / 2,271, failed seeds 0).
- **Validity rejections:** 10 start-invalid, all narrow (pilot 2, train_narrow 8; 29 under A18);
  goal-invalid 0 and residual 0 in every set; 13 goals re-drawn (pilot 3, train_narrow 10).
- **Unmapped rates** are as declared (tune_medium 0.35, as before).
- **Narrow and medium obstacles** land on open floor or at aisle mouths, never in an aisle.
- **The wide sets differ in their unmapped obstacles.** Placement uses the narrow floor (0.1735 now)
  as the bypass clearance. Their mapped layouts and routes are unchanged, which is what the oracle
  and the mission-time pairs read.
- The fast suite on the rebuilt sets: `test_extent` and `test_preregistration` pass.
- Next: the oracle residual check (decide) on these sets.


### Reviewed 2026-10-19: the oracle residual check on the A20 sets -- PASS; the new mission-time run

Both at 8047b50 (the rebuilt sets committed), clean.
- **`runs/analysis/ORACLE_FREE_decide_20261019T162419Z_seed0`** (lethal 0.10, soft 0.03): every
  declared expectation holds.
  - 2,271 / 2,271 plannable; 0 quantisation failures.
  - Start-invalid, goal-invalid, residual and bend-stop routes all 0; the free floor 0.000 / 100 m
    in every stratum.
  - The residual rule's label reads "re-sample" with nothing to re-sample, as under A19.
  - **A1's tangent-locked floor:** narrow 1.252 / 100 m (5.564 under A19; the A20 narrow aisles are
    wider), medium 0.039, wide 0.054. This is H4's context.
- **`runs/analysis/ORACLE_FREE_info_20261019T164220Z_seed0`** (A20, v_max 0.50): every route
  plannable.
  - The median mission time is 1.22 x the constant-speed time in every stratum. The constant-speed
    time is now arc / min(speed cap, v_max) = arc / 0.50.
  - The 50-mm-inside share breaching band 0 (information only, as ruled) is 64-74 %: the larger
    fields and a route moved 50 mm toward the inside of bends.
- The old baseline (28afe21 in a worktree, the researcher's) and the comparison follow.


### Reviewed 2026-10-19: oracle mission time, old against new (at v_max 0.50)

- **OLD:** `../pfb-nav-pre-a20/runs/analysis/ORACLE_FREE_info_20261019T194420Z_seed0`. A git worktree
  at 28afe21 (the last pre-A20 code; pfbnav confirmed loaded from the worktree), clean, on the
  pre-A20 sets. Every route plannable; median mission time / constant 1.075-1.078; the
  50-mm-inside breach share 1.7-2.7 %.
- **NEW:** `runs/analysis/ORACLE_FREE_info_20261019T164220Z_seed0` (8047b50).
- Preview by Claude, read straight from the two `routes.csv` files with `pfbnav/eval/mission_time.py`;
  the recorded `scripts/compare_mission_time.py` run is the researcher's.
- **Medians at lethal 0.10, soft 0.03; all 2,271 routes traversable in both:**

  | stratum | old | new | new / old per metre | matched pairs: median ratio [min, max] |
  |---|---|---|---|---|
  | narrow | 26.8 s, 2.695 s/m | 25.3 s, 2.438 s/m | 0.905 | 857: 0.907 [0.54, 1.98] |
  | medium | 28.0 s, 2.686 s/m | 25.5 s, 2.440 s/m | 0.908 | 707: 0.905 [0.59, 1.67] |
  | wide | 27.4 s, 2.689 s/m | 24.7 s, 2.436 s/m | 0.906 | 707, the same routes: 0.906 [0.86, 1.04] |

- **Reading:**
  - The oracle's missions are about 9 % FASTER under A20. The cause is v_max 0.40 -> 0.50.
  - **The oracle applies no acceleration limit** (its profile is a best case, as ruled), so a_max
    1.0 -> 0.5 does not enter this comparison at all.
  - What A20 slows that the oracle does model -- alpha_max 1.4 in its turn-speed and in-place
    rotation limits, and band 0's 0.025 m/s -- shows only at the margin: wide routes, which are
    identical between the runs, range up to 1.036.
  - A real tracker at a_max 0.5 gives some of the 9 % back. An acceleration-limited oracle variant
    would measure how much; not run, and the supervisor's to rule.
  - The narrow and medium pairs are different worlds (rebuilt under the new strata), so only their
    per-metre figures compare like with like.


### 2026-10-19: the containment test's first run -- null PASS; verdict not read (a run that tested nothing)

`pytest -m sim tests/test_containment_sim.py -s`, at 8047b50 + the test as committed:
- **Null test PASS:** 4 stops at constant velocity, 0 exceedances of 1 mm. The largest containment
  excess is -63.14 mm: every stop contained, the closest point 63 mm inside the field (the margins
  and the reach). The smallest rest margin is +74.42 mm.
- **The verdict failed before it was read:** "spinning beside a wall" produced no stop.
  - That is a construction defect of the test, not a containment result. The wall's face was 0.41 m
    from the body origin; spinning, the field reaches 0.337 (outer corner) + 0.013 (reach) + 0.05
    (margin) = 0.400 m, so nothing could trip it.
  - The test also stopped at the first silent run, discarding the other three runs' rows.
- **Fixed:**
  - The wall's face is now at 0.37 m: 90 mm outside the still at-rest field (0.280 m sideways) and
    inside the spinning one. It trips once the composed yaw rate passes about 0.7 rad/s.
  - The test prints each run's stops as it goes, and reports silent runs at the end without
    discarding the rest.
- The tolerance, the null runs and the criteria are unchanged. The module is rerun whole: the
  verdict needs the null test in the same session.


### Reviewed 2026-10-19: the containment test -- containment held; a stop ended inside the minimum margin (FAIL); positive control did not fire (INCONCLUSIVE)

`pytest -m sim tests/test_containment_sim.py -s`, the test with the spinning run's wall inside reach.
- **Null test PASS:** 4 stops at constant velocity, 0 exceedances. Largest excess -63.14 mm;
  smallest rest margin +74.28 mm.
- **Accelerating runs (9 stops):**
  - forward from rest: 1 stop at band 2 (field speed 0.50), excess -63.16 mm, rest margin
    +53.47 mm.
  - turning and accelerating: 1 stop at step 2 (field 0.025 m/s, 0.07 rad/s), excess -63.16 mm,
    rest margin +11.38 mm. It tripped before turning at speed, so it did not test what it is
    named for.
  - swerve toward a side wall: 5 stops (one at 0.346 m/s, then four creeping), excess -63.17 mm,
    rest margins +74.6 to +108.6 mm.
  - spinning beside a wall: 2 stops. Step 11 (0.70 rad/s): excess -63.13 mm, rest margin
    **+1.23 mm**. Step 72 (0.07 rad/s): excess -63.13 mm, rest margin **-3.54 mm**.
- **Containment held at every stop.** The excess is -63 mm everywhere: the lateral margin and the
  at-rest reach at the unmoved footprint. The motion after a trigger never came nearer the boundary
  than where the robot started.
- **FAIL: the second spinning stop ended 3.54 mm inside the minimum margin.** A robot left so cannot
  restart (the latch requires its own field clear): the B1 deadlock pattern, reached by rotation.
  - **How the spin resumed after the first stop is not established:** the test does not run the
    restart condition, which is evaluated only once a breach has cleared.
  - The likely mechanism has two parts, by geometry; the run recorded no trace to confirm it.
  - **(i) The at-rest field carries its own rotational reach** (+-e(0) = +-0.0315 rad, about the
    heading the robot comes to rest at). A stop that turned the body by up to e(|omega|) toward
    the wall puts that sweep up to e(0) beyond the trigger field's yaw interval: about 9 mm of
    support here (d h / d phi ~ 0.29 m / rad x 0.0315 rad).
  - So containment of the FOOTPRINT does not imply that the at-rest field the robot stops in lies
    inside the trigger field. Translation does not have this hole: the moving field carries the
    at-rest reach and margin abeam and behind.
  - **(ii) The restart condition ignores rotation.** A pure spin command counts as "asking for
    nothing" (`_restart_allowed` with zero translation) and is never blocked, even toward an
    obstacle at the margin.
- **The positive control did not fire, so the containment verdict is INCONCLUSIVE, as declared.**
  - A field without reach was not exceeded either: largest excess -43.33 mm.
  - Under A20's limits the motion added within T is small. These runs never approach the margins
    (the excess is the static footprint's everywhere), so they cannot show the instrument catching
    an under-sized field. The 2026-10-18 failures needed alpha_max 3.0.
- **No code changed.** The fixes alter the field equation or the restart rule (hard rule 7):
  ruling requested. The G4 controls stay held.

- **Mechanism (i) confirmed numerically** (`FieldGeometry` support toward the wall, the trigger
  frame).
  - The field in force at the second trigger (at rest, 0.07 rad/s; yaw interval +-0.0543 rad)
    reaches 284.5 mm.
  - The at-rest field after the body turns theta toward the wall reaches 279.5 / 285.5 / 291.3 mm
    at theta = 0 / 0.027 / 0.054 rad.
  - So beyond about 0.02 rad of the reachable turn, the field the robot stops in lies outside the
    one it stopped under: by up to 6.8 mm here, against the observed -3.54 mm.
- **For the ruling:**
  - (a) every field's yaw interval extended by e(0) on each side, so the field in force contains
    the at-rest field at every reachable final heading (the rotational counterpart of the floor the
    moving fields already carry in translation). About +7-9 mm of sideways support; F to be
    re-checked against the 40.1 mm binned slack it had.
  - (b) a rotation-aware restart condition: a commanded spin blocked while its sense leads into an
    intrusion.
  - (c) a positive control that can fire under the A20 limits: the field with its margins removed,
    with reach (must contain) and without reach (must be exceeded). Also the "turning and
    accelerating" wall moved so that it trips at speed.


### Ruled 2026-10-19: nesting (a), a rigid-body restart (b), the containment control (c)

Supervisor's rulings, recorded before implementation.
- **(a)** Adopt the extended yaw interval as the construction enforcing a general NESTING invariant:
  for every state, the field in force contains the at-rest field at every reachable rest pose
  (position and heading), so a stop can never create its own deadlock in a field that was clear at
  the trigger.
  - Add an exhaustive geometric unit test of nesting over speeds, yaw rates, headings and bands,
    beside the monotonicity tests.
  - Recompute F by its rule under the extended field: the smallest clearance with all three gaps
    >= 0, binned, + 0.03 m. If F changes, the strata move by rule and the sets are rebuilt. Report F.
- **(b)** The restart condition in rigid-body form:
  - for every breaching point p, the velocity of the closest footprint point under the commanded
    twist (translation plus omega x r) has no component toward p;
  - and the leading field for the commanded twist, including its rotation sweep, is clear.
  - Known-answer tests: a spin leading a corner into an intrusion is blocked; a spin swinging a
    corner away is permitted; pure translation reproduces the current rule exactly.
- **(c)** The containment control, declared before running:
  - against the field with margins removed, the with-reach field must contain each stop, and the
    without-reach field must be exceeded;
  - move the turning-and-accelerating wall so that it trips at speed;
  - the verdict counts only if the without-reach comparison fires.
- **Also:**
  - run the acceleration-limited oracle mission time as information for the methods (non-blocking);
  - record A1's narrow floor (1.25 per 100 m under A20).
- **Order:**
  1. (a) and (b), with tests;
  2. F by rule, the rebuild if needed, the oracle residual check;
  3. the containment test with its control;
  4. the G4 controls.


### 2026-10-19: (b) and (c) implemented; (a) measured -- as worded it cannot achieve nesting; ruling requested

**(a) Nesting, measured before building** (`scripts/probe_nesting.py`, preview by Claude).
- The probe takes bang-bang limiter trajectories from each state to rest. The worst amount by which
  the target field, placed at a reachable rest pose, reaches outside the field in force at the
  state (+ = nesting broken):

  | state | A20 now, target = at-rest field | (a) literal: every interval + e(0) | moving fields + e(0) yaw + 13.1 mm | A20 now, target = STANDING field |
  |---|---|---|---|---|
  | rest, still | +19.3 | +18.7 | +19.3 | -4.7 |
  | rest, spin 0.07 | +18.8 | +18.3 | +18.8 | -4.7 |
  | rest, spin 0.7 | +14.7 | +14.2 | +14.7 | -4.7 |
  | rest, creep 0.02 | +19.6 | +19.6 | +19.6 | +0.0 (touches) |
  | moving 0.10 | +17.7 | +17.1 | -5.5 | -6.4 |
  | moving 0.20, turning 0.3 | +23.9 | +23.2 | -0.6 | -0.6 |
  | moving 0.40 | +28.8 | +27.9 | -2.0 | -2.1 |

  (mm; STANDING = footprint + lateral margin, no reach.)

- **(i) The literal extension does not nest.** The target at-rest field is extended too, so it grows
  with the field meant to contain it.
- **(ii) Moving states nest only if the extension also carries the position part**: the at-rest
  field's own 13.1 mm reach on top of the travel, as well as e(0) in yaw. F then becomes 0.1926
  (zero-gap 0.1626 + 0.03; F + 0.07 = 0.263 <= 0.30), and the sets are rebuilt.
- **(iii) No construction nests from an AT-REST state** while the at-rest field carries the reach
  from rest (ruled "applies at rest too"). It would have to contain copies of itself shifted and
  turned by that reach.
- **(iv) Against the STANDING field, the A20 field as it is nests in every state.** F is unchanged
  (0.1735).
- **The design choice, for the supervisor:**
  - **S1:** the standing field (footprint + lateral margin: the "minimum margin" as it meant before
    A20) becomes the latch's own-field criterion and the definition of a stop "inside the minimum
    margin". The reach-inclusive field keeps triggering stops. Nesting holds everywhere, and F
    stays 0.1735.
    - Open risk: something between the standing field and the reach-inclusive field, not in the
      leading region, could re-trigger a stop right after a release (the 2026-09-27 loop).
  - **S2:** extend the moving fields (yaw + position): F = 0.1926 and a rebuild. At-rest triggers
    (spins, creep) still cannot nest; (b) only stops a latched robot from restarting into them.

**(b) Implemented: the rigid-body restart condition** (`CertifiedLayer._restart_allowed`, twist
(vx, vy, wz); the step loop and the emulator pass the full twist; the recovery still passes a
translation).
- **A conflict between the ruling's two requirements, found by its own tests:**
  - "the velocity of the closest footprint point has no component toward p", measured from q;
  - "pure translation reproduces the current rule exactly".
  - Measured from q alone, the rule PERMITS driving past an intrusion beside the front half (the old
    rule blocks it) and BLOCKS a near-straight departure that edges sideways toward a point beside
    the rear half (the old rule permits it).
  - Measured from the origin alone, it misses a reversing turn that swings a side into an intrusion
    (shown by a test).
- **Adopted pending the ruling: both tests required.** The old translation rule
  (v . p <= 0 from the origin) AND the rigid-body rule from q ((v + w x q) . (p - q) <= 0).
  - Never more permissive than the old rule.
  - Stricter in 76 of 3,000 random translation cases (2.5 %), each a real sideways approach.
  - An unobserved bin's unknown range: the rule from q is tested from just outside the footprint
    out to the field boundary.
- **`tests/test_restart_rigid.py`, 7 tests:**
  - never more permissive than a verbatim copy of the 2026-09-28 rule (the 76 pinned);
  - the corner-leading spin blocked, the corner-swinging spin permitted;
  - nothing asked, nothing blocked;
  - an unobserved bin blocks a spin toward it;
  - a reversing turn into an intrusion blocked, the opposite turn permitted.
- The layer, recovery and emulator tests pass (63, the A20 emulator control included).
- `certified.py` and `step_loop.py` changed, so the environment identity and `scenarios_sha256`
  move. The re-freeze and A21 wait for (a)'s ruling, so they move once.

**(c) Implemented in `tests/test_containment_sim.py`** (not yet run):
- the margins-removed control: with reach must contain, without reach must be exceeded, or no
  verdict;
- the null test applies the margins-removed check too;
- the turning wall moved (0.72 m across the way, a 0.35 m/s / 0.9 rad/s command), with an assertion
  that it trips at speed (>= 0.2 m/s, >= 0.3 rad/s).

**Recorded as ruled:** A1's narrow floor under A20 is 1.25 per 100 m (it was 5.56 under A19). The
acceleration-limited oracle (information, non-blocking) follows after the ruling.


### Ruled 2026-10-19: nesting restated (S2); the restart rule's two tests; F = 0.1926

Supervisor's rulings, recorded before implementation.
- **Recorded first:** the nesting invariant as the supervisor worded it is impossible. It required
  the at-rest field, which carries the reach from rest, to nest inside itself shifted by that
  reach. The error is the supervisor's.
- **(a) S2.**
  - **The restated invariant:** every stop beginning from a MOVING state (translating or spinning)
    nests. The trigger field contains the at-rest field at every reachable rest pose.
    - A nesting test over moving states.
  - **At-rest states are excluded.** In a static world a stationary robot with a clear at-rest
    field cannot be triggered by static obstacles. An at-rest trigger needs something entering the
    field (dynamic, out of scope), and there holding the latch is correct.
  - **A counter of at-rest triggers in every static-world run** (G4 controls, pilots, evaluation).
    Expected zero; any occurrence reported as a construction gap.
  - **At-rest classification requires both the estimate and the applied command below the
    standstill threshold.**
  - **S1 rejected:** releasing on the standing field leaves the obstacle in the reach shell after
    most stops near an obstacle. At departure the field is isotropic, so any first move
    re-triggers: a systematic release-retrigger loop created by the layer's geometry.
  - **F = 0.1926 by rule.** Narrow [0.1926, 0.2626], medium [0.2626, 0.40], wide unchanged.
    Rebuild the sets; the oracle residual check.
- **(b)** A restart is permitted only if BOTH the centre rule (v . p <= 0) and the closest-point
  rigid-body rule permit it.
  - A randomised property test: never more permissive than the centre rule.
  - Report the stricter fraction (about 2.5 % of translation cases, genuine sideways approaches).
    This supersedes "reproduces the current rule exactly".
  - Known-answer tests: a spin into an intrusion blocked; a spin swinging a corner away permitted;
    a reversing turn into an obstacle blocked.
- **Then:** the containment test with control (c), declared first; then the G4 controls.


### 2026-10-19: the nesting test found a coupling gap -- turning while translating; a provable term added (rule 7)

- **The exhaustive nesting test** (`tests/test_nesting.py`) integrates the body-frame velocity as it
  turns with the body. It found S2 moving states failing: up to about 50 mm, but at speed-yaw
  combinations the wheel budget forbids. With the budget applied
  (`|vx| + |vy| + L|w| <= 0.586`) and the correct kinematics, the worst excess is +1.0 mm (0.43 m/s
  straight) and +0.2 mm (0.20 m/s, 0.3 rad/s). Containment of the footprint held in every state, by
  about 72 mm.
- **The cause: no term bounds the sideways drift of a body that turns while it moves.**
  - The construction bounds the translation as if the body-frame velocity stayed fixed in the
    trigger frame, and the rotation separately about the body origin.
  - A holonomic base's commanded velocity turns with the body, so the path curves.
  - `tests/test_reach.py`'s brute-force trajectories made the same simplification (world-fixed
    velocity), which is why it did not see this. They are corrected in the same change.
- **The provable bound:**
  - the displacement d = integral of R(theta(t)) u(t) dt departs from the straight
    integral of u(t) dt by at most integral |u| |R(theta) - I| dt <= D(V) * 2 sin(Theta / 2);
  - the path length is at most the band depth D(V) = V T + V^2 / (2 a_b);
  - Theta, the heading change, is at most the reachable excursion e(|omega|).
- **Added to every moving field as an isotropic term** (`certified.coupling`). It is a field-equation
  change, recorded here in the same change. It is conservative and it never shrinks the field.
  - At the top band and zero yaw rate: about 6.5 mm.
  - F moves by its rule (recomputed below). The supervisor is told; the ruled invariant needs it.


### 2026-10-19: A21 implemented -- S2 nesting with the coupling term, rotation in the standstill rule, the at-rest trigger counter, the restart's two tests; F = 0.1987; re-freeze 7; one finding for a ruling

**The field (`pfbnav/safety/certified.py`).** Every MOVING field carries three nesting terms:
- the yaw interval +-(e(|omega|) + e(0));
- a margin term of at least the at-rest reach (13.1 mm) in every direction (ahead the forward margin
  already exceeds it);
- the coupling term `coupling(V, omega) = D(V) * 2 sin(e(|omega|) / 2)`. It is the drift of a body
  that turns while it translates, which the nesting test showed necessary (entry above). It is
  isotropic, at most a few millimetres in feasible states (6.5 mm at the top band and zero yaw
  rate).

The at-rest field is unchanged. `pfbnav/safety/reach.py`'s "full" variant mirrors it, and its
equality test pins the two together.

**At rest needs rotation below threshold too.**
- `standstill_yaw_enter` 0.0296 and `standstill_yaw_exit` 0.0592 rad/s are the translational
  thresholds over the footprint's outermost radius, 0.3374 m, rounded down, so at rest no body point
  moves faster than the translational threshold. They are derived, not chosen, and a test asserts
  the relation.
- The detector leaves rest on either signal and enters it only when both are below.
- A spin leaves rest within one control step (the limiter adds 0.07 rad/s per step).

**At-rest triggers are counted:**
- by the layer (`triggered_at_rest`, `at_rest_triggers`);
- per episode by the step loop's report;
- in `outcome_summary`;
- printed by `scripts/evaluate.py`, with a CONSTRUCTION GAP line when non-zero;
- in `scripts/gate_g4.py --controls-only`;
- in training logs and summaries.

Expected zero in every static-world run.

**The restart condition (b), as ruled.** Both the centre rule and the closest-point rule. It is
never more permissive than the centre rule (a randomised property test). The stricter fraction is
**85 / 3,000 = 2.8 %** under the A21 field (2.5 % under A20's smaller field). The known answers hold:
a spin into an intrusion blocked; a spin swinging the corner away permitted; a reversing turn into
an obstacle blocked.

**The nesting test** (`tests/test_nesting.py`, 12 tests):
- moving states at five speeds across all three bands, four travel directions and four yaw rates,
  plus five spins on the spot;
- reachable rest poses from bang-bang limiter trajectories with the body-frame velocity turning
  with the body and every twist projected into the wheel budget;
- worst excess within 0.3 mm of grid tolerance.
- **Positive control:** at-rest states fail by more than 10 mm, which is why the ruling excludes
  them.
- `tests/test_reach.py`'s trajectories were corrected to turn with the body too.

**F = 0.1687 + 0.03 = 0.1987 m by its rule.** The ruling named 0.1926, which was computed before the
coupling term. Strata: narrow [0.1987, 0.2687], medium [0.2687, 0.40], wide unchanged. F + 0.07 =
0.2687 <= 0.30.

**Tests re-derived from the formulas** (each stated in its test):
- the field cache (S2 written out);
- the layer: the at-rest field is now a strict lower bound of the moving fields, by the nesting
  terms, no longer touching;
- the golden fields, stored for A21 in `tests/data/field_golden_A21_20261019.npz` (the A20 file is
  kept);
- D2: ahead / behind / turn known answers, warning gap 39.6 mm, binned margins 127.5 / 89.1 /
  38.5 mm, the moving requirement 0.1591 m, stop offsets 0.1278 / 0.1978 / 0.3291, the traversal
  bisection 0.244 / 0.433 / 0.5;
- layouts;
- the plannability control: its soft margin is raised with the lethal one, which past the new edge
  would otherwise exceed the soft radius;
- the oracle's +-2 e(0).

**Re-freeze 7: cb9c839c -> de3a8e90.** The filter-off bit-identity test reruns in the simulator.

**A21 drafts:**
- `docs/pending/A21_nesting_restart.md`, with A20's erratum (the F run name);
- `docs/pending/endpoints_A21.yaml` (H2: narrow 0.1987-0.2687 m);
- `docs/pending/scenario_sets_A21.yaml` (the bypass clearance 0.1987 m).

The sets are REBUILT after the relock.

**FINDING, for a ruling: S2 also produces the release-retrigger loop, in a thinner shell.**
- `tests/test_certified_layer.py::test_reversing_away_from_an_intrusion_is_allowed_and_stays_allowed`
  fails under A21; it is marked xfail(strict) with the reason. A robot released to back away from an
  intrusion 0.28 m ahead (0.21 m disc) is re-stopped by its own moving field.
- **Nesting requires every moving field to contain the at-rest field displaced by everything
  reachable**, a brief reversal included. So moving fields exceed the at-rest field in every
  direction, BEHIND the motion too: by about 9 mm at band 0 and 11 mm at band 1 (swerve, drift,
  extra turn).
- **After a nested stop the obstacle typically ends just outside the at-rest field, in that shell.**
  Stops bring the robot as close as allowed, and the nesting margins are tight (-0.6 to -6 mm). Any
  departure at normal speed then re-triggers.
- The supervisor rejected S1 for this loop. It is intrinsic to any construction that both contains
  the reach and nests.
- **Proposed:** the restart test clears the WHOLE field the commanded twist will select (both
  approach rules kept), so a departure that would re-trigger is blocked rather than released into a
  loop.
  - The escape stays open: the classical recovery creeps at 0.02 m/s, which is at rest by the
    standstill rule, so the at-rest field is in force, and nesting keeps that field clear. It
    creeps out of the shell, then the normal departure is permitted.
  - A learned policy can learn the same creep.
- Not implemented: a restart-semantics change is the supervisor's.


### Ruled 2026-10-19: restarts clear the whole selected field; the at-rest field's sizing; the creep regime

Supervisor's rulings, recorded before implementation.
- **Approved:** a restart must clear the WHOLE field the commanded twist will select, keeping both
  approach rules. Departures that would re-trigger are blocked instead of looping.
  - **The rationale, recorded:** containment plus nesting implies a departure shell in every
    direction. It is resolved by a low-speed creep regime, mirroring industrial manoeuvring field
    sets.
- **Pause the A21 rebuild until the following is settled,** so that the sets are rebuilt once.
- **Condition:** the at-rest field must be sized for reachable motion from every state classified as
  at rest -- from the standstill EXIT threshold, not zero velocity -- so that 0.02 m/s creep is
  contained.
  - Verify it; if it is sized from zero, correct it and recompute F by rule.
  - Record the entry and exit thresholds, translational and rotational.
  - Confirm that 0.02 m/s sits below the exit threshold, hysteresis included.
- **Declared now:** stops beginning from creep are at-rest stops, not covered by nesting. Do not
  engineer for them. If any at-rest trigger in a static-world run ends as an unrecoverable_stop,
  stop and report.
- **Methods:** creep departure is a learnable skill for learned arms (stop_active stays set when a
  restart is blocked; 0.02 m/s is within the action range) and scripted for the classical baselines
  via the recovery. State the asymmetry.
- **The drift term D(V) 2 sin(e(|omega|) / 2) is accepted.** Record that the earlier containment test
  missed it.
- **Then:**
  1. the restart rule with known-answer tests (a re-triggering departure blocked; a creep departure
     permitted; the loop unit test now passes);
  2. one rebuild and the oracle residual check;
  3. the containment test with control (c);
  4. the G4 controls.


### 2026-10-19: the whole-field restart implemented; the at-rest sizing verified; three findings, proposed as one package

**Restart (as approved).** `CertifiedLayer._restart_allowed` clears the WHOLE field the commanded
twist selects: the at-rest field at or below both exit thresholds (the creep regime), otherwise the
band of the reachable speed with its yaw interval. The two approach rules follow; with the whole field
clear they cannot bind. The recovery judges each creep direction as its creep twist and hands back
when the base controller's full twist is permitted.
- **Known answers** (`tests/test_restart_rigid.py`, `tests/test_certified_layer.py`):
  - a re-triggering departure is blocked in every direction;
  - a creep departure is permitted in every direction;
  - reversing at speed stays blocked with no new stop, and creeping away is released with no
    re-trigger: **the loop test passes**;
  - the creep regime ends exactly at the exit speed;
  - a spin blocked at 1 rad/s, permitted below the rotational exit;
  - a reversing turn into an obstacle blocked;
  - unobserved bins block every departure.
- Never more permissive than the centre rule; stricter in 968 / 3000 random cases (each a
  departure whose own field holds an intrusion).
- The recovery: in the walled-in scene a creep is now permitted toward the most clearance (it ended
  `no_permitted_direction` before). That ending occurs only with the at-rest field breached, where
  the deadlock check comes first.

**At-rest sizing: verified, no change, F stays 0.1987.**
- The at-rest reach is from the 0.02 m/s EXIT threshold (`rest_reach` uses `standstill_exit`).
- The at-rest yaw sweep is from the composed yaw rate, at most the 0.0592 rad/s exit at rest.
- Thresholds: translational enter 0.01 / exit 0.02 m/s; rotational enter 0.0296 / exit
  0.0592 rad/s.

**Three findings, proposed together so that one confirmation settles them:**
1. **0.02 m/s does NOT sit below the exit threshold; it sits on it.**
   - The detector leaves rest only above 0.02, but a 0.02 m/s command at 45 deg has magnitude
     0.020000000000000004 (1 of 72 recovery headings), and estimator noise can cross it too.
   - **Proposed:** the recovery creeps at 0.018 m/s (10 % margin; reachable 0.093, band 0).
     Configured now, marked PENDING. It is a baseline parameter: outside the environment's
     identity, and it does not touch the rebuild.
2. **The at-rest trigger counter would flag every recovery creep stop as a construction gap.** A
   creeping robot is at rest by classification.
   - **Proposed and implemented:** split it. STATIONARY triggers (composed speed and yaw rate at or
     below the enter thresholds) are the construction gap. Creep stops are reported apart,
     with the episodes where one ends unrecoverable_stop, which stop the run as ruled.
   - In the layer, the step loop, `outcome_summary`, evaluation, G4 and training.
3. **(b)'s "a spin swinging a corner away is permitted" is superseded at normal spin rates.** The
   spin's field is sense-symmetric, so an intrusion inside it blocks a spin either way: it would
   re-trigger. It still holds at a creep yaw rate.

**Re-freeze 7 updated before any run of it:** de3a8e90 -> 42816b7c. The emulator's A20 positive
control is skipped (its reference predates the restart change) and is re-pinned on a fresh A21
corner trace before the six traces are re-emulated.

**2026-10-19 - Fast suite on the whole-field restart, and A22 drafted.**
- `pytest -m "not sim"`: 1244 passed, 7 skipped, 8 failed. The skips are the 5 D2 real-scan constants
  and the 2 emulator fixtures, pending the simulator session. The failures are 7 x `test_extent`
  (stale sets, pending the rebuild) and the lock, which moved because `certified.py` changed. Ruff is clean.
- A21 was already applied and relocked (234f5a7, 5bfebb8) with the restart as two approach rules. The
  whole-field restart, the creep regime, the recovery at 0.018 m/s and the at-rest counter split
  therefore go in a NEW amendment, A22 (`docs/pending/A22_whole_field_restart.md`). The applied A21
  draft is restored to its applied text.
- 0.018 m/s is still pending the supervisor's confirmation. It is a config value outside the scenario
  closure, so it does not move the lock; it does gate the corner trace and the G4 controls.

**2026-10-19 - RULED (supervisor): the three A22 items confirmed. Recorded before the change.**
1. **Recovery creep 0.018 m/s, confirmed** (about 10 % below the 0.02 exit; a diagonal command's
   magnitude can round above a creep sitting on the threshold).
   - The rotational equivalent is recorded. The supervisor wrote 0.018 / 0.351 ~ 0.051 rad/s, but 0.351
     is the outermost radius WITH the pods (0.3519). The yaw thresholds are derived over the current
     table's outermost radius 0.3374 m, so the same derivation gives **0.018 / 0.3374 = 0.0533 rad/s**.
     It is recorded only: the recovery is translation only.
   - Attempt duration: 0.30 m at 0.018 m/s is 16.7 s, so five attempts no longer fit in a 75 s episode
     and the cap is effectively lower. Accepted (recovery is rare under the corrected fields).
     **Implemented:** every attempt's time is logged, live, so an attempt cut short by the episode's
     end still reports it (`recovery.py`). `recovery_statistics` adds `attempt_times_s` and
     `attempts_cut_short`, and `gate_g4.py` prints the maximum and mean. Known-answer tests in
     `tests/test_g4.py` and `tests/test_recovery.py`. Neither file is in the environment's freeze closure.
2. **Two classes of at-rest trigger, confirmed.**
   - Stationary triggers: the construction gap, expected zero, any occurrence reported.
   - Creep triggers: counted per controller in every run; one ending `unrecoverable_stop` means stop
     and report.
   - **Both are protective stops and count in H1.**
3. **SUPERSEDED: the 2026-09-28 ruling that spinning away from an intrusion is permitted.**
   - Reason: a normal-rate spin selects a moving field that contains the departure shell, so an
     intrusion inside it would re-trigger at once. At creep rate the at-rest field governs, and
     nesting keeps it clear.
   - The original ruling's intent (never drive toward an intrusion, always leave a way out) is kept by
     the creep regime.
- The at-rest sizing from the exit thresholds, verified with a negative control, closes the condition.
  F stays 0.1987.
- Next, as ruled: one rebuild and the oracle residual check, then the containment test with control
  (c), then the G4 controls. The A22 draft carries all three items.

**2026-10-20 - REVIEWED: the A22 rebuild and its oracle residual check. PASS.**
- **Rebuild** (`runs/diag/SCENARIO_SETS_20261020T073343Z_seed0`, clean tree at c748dc2, generator
  matching the lock): 2,271/2,271 built in full, every set verified, every validity check passed,
  engagement 1.0. Committed at 6207c7d.
  - A first attempt died after nine sets (the laptop slept). Its nine files, stashed, were
    byte-identical to the complete rebuild's: the build is deterministic.
- **Oracle check** (`runs/analysis/ORACLE_FREE_decide_20261020T133802Z_seed0`, 6207c7d, clean;
  `planner_oracle_free_rebuilt.yaml`, lethal 0.10, soft 0.03). Against the criterion stated before the
  run: 2271/2271 plannable; 0 quantisation fails, 0 start-invalid, 0 goal-invalid, 0 residual; 0
  bend-stop routes; free floor 0.000 in every stratum. **PASS.**
- **A1's tangent-locked floor (information, H4 context):** 0.268 /100 m overall (narrow 0.542, medium
  0.092, wide 0.108); after A20 it was 0.503 (1.252 / 0.039 / 0.054).
  - Narrow fell because its floor rose to 0.1987.
  - Medium and wide rose because A21's moving fields are larger: + e(0) in yaw, the rest reach, the
    coupling.
  - Every set's scenarios differ from A20's, wide included: the generator's start and goal validity
    uses the certified fields.
- Next, as ruled: the simulator session (bit-identity for re-freeze 7, the D2 real scan, the
  containment test with control (c), the recovery corner trace, the recovery simulator tests), then
  re-pinning and the re-emulation of the six B1 traces, then the G4 controls.

**2026-10-20 - REVIEWED: the simulator session (re-freeze 7, D2, containment, recovery, corner trace).**
All runs at 6207c7d. The tree was dirty only through the uncommitted decision-log entry; no code differed.
1. **Bit-identity (filter off): PASS.** Re-freeze 7 (42816b7c) is complete.
2. **D2 real scan: 5/5 PASS.** Measured at the 0.1987 m floor, wall faces at the clearance: 128.3 mm at
   rest, 90.1 mm centred at the top band, 40.1 mm at the 50 mm tracking offset.
   - The model gives 127.5 / 89.1 / 38.5 mm: conservative in every case, and within 2 mm.
   - Re-pinned in `tests/test_d2_aisle_strata.py` (the three tests un-skipped). Two WP1 properties are
     restated for A20/A21:
     - the at-rest and moving margins no longer coincide abeam: the at-rest field is strictly inside,
       and the measured gap (38.2 mm) matches the lateral reaches' gap to 2 mm;
     - the 50 mm tracking bound is now taken from the MOVING margin (90.1 -> 40.1 mm, exactly 50.0).
3. **Containment with control (c): PASS.**
   - Every stop was contained, the largest excess -77.8 mm.
   - The null run is clean: 2 stops, exceedance 0.
   - The positive control fired: +12.75 mm with the margins and the reach removed.
   - The spinning stop now ends +27.2 mm outside the minimum margin (it was -3.54 mm before A21).
     Nesting is confirmed in the simulator.
4. **Corner trace** (`runs/diag/RECOVERY_CORNER_20261020T152314Z_seed0`): bare B1 stops once (step 86,
   s = 0.946 m, band 2, a moving trigger) and is then HELD (restart blocked to the end, s = 0.983 m).
   - The A20 run's six stop-restart cycles are gone; that is the whole-field restart working as ruled.
   - Recovery: 3 attempts, all `cleared`, 0 `not_released`. B1 re-stopped after each, reaching
     s = 1.174 m in 25 s.
   - At-rest triggers: 0. Stationary triggers: 0.
   - Reference fixture: `tests/data/corner_stop_A22_20261020.json`.
5. **The emulator's positive control, re-pinned on that trace: FAIL on one criterion.**
   - It passes on the first stop (step 87 vs 86; s 0.951 vs 0.946), the stop count (1) and the final
     latch.
   - The final s is 0.9985 against the simulator's 0.9825, 15.8 mm, beyond the 10 mm tolerance.
   - Cause: the emulator triggers ONE STEP later. Its decision pose is 17 mm further on: the
     simulator's breach at the trigger was 9.1 mm deep, and one step moves the boundary about 10 mm.
     Measured FROM THE TRIGGER DECISION POSE, the braking travel agrees: simulator 48.6 mm, emulator
     47.5 mm.
   - At 0.24 m/s one step is 11.9 mm, above the tolerance. The A20 reference ended after slow
     re-starts, where a step is small. Not relaxed here; proposed to the supervisor.
6. **`tests/test_recovery_sim.py::test_the_2026_10_13_corner_stop_recovers`: FAIL.**
   - The control's pinned location (s = 1.45 m, 2026-10-13) predates A20; the stop is now at 0.946 m.
   - Recovery's "0.5 m past the corner" is not reached (0.23 m).

**Diagnosis (emulator; one tune scenario per stratum, the current sets, B1 as configured).** B1
DEADLOCKS on 3/3 routes at its first protective stop: tune_narrow at 5.7/10.1 m, tune_medium at
6.5/9.7 m, tune_wide at 1.7/11.3 m, each in band 2 and held to the end.
- **Why it is held.** The whole-field restart judges the field of the COMMANDED twist. B1 always commands
  0.25 m/s (reachable 0.325, so band 2) and up to 1.4 rad/s at bends, and the field it would need
  never clears.
- **Withdrawn.** The emulator refuted my first explanation (band 2 alone): B1 at 0.225 or 0.20 m/s,
  its own shorter lookahead (0.3 / 0.5 m), body yaw toward the tangent, and two scan-reactive
  governors (fastest clear band; plus yaw scaling and a creep fallback) all deadlock at the same
  spots.
- Every tracker cut 20-30 mm inside the planned route at those spots (minimum centre clearance
  0.28-0.30 m against the planner's 0.3086).
- **Both oracles pass these three routes with ZERO stops** (free-orientation and tangent-locked,
  `ORACLE_FREE_decide_20261020T133802Z`): they slow into band 0 ahead of the spot, with full knowledge
  of the route, and track it exactly.
- So the spots are traversable, but only with an anticipating speed and yaw schedule and exact
  tracking. **No environment defect is identified.** This is the case pre-declared on 2026-10-17:
  "If B1 with recovery still fails G4 ... the positive control becomes a privileged oracle tracker run
  in the simulator; B1's result is recorded as a baseline finding."
- Prototypes stay in the session scratchpad: nothing is committed, and B1 is unchanged.

### RULED 2026-10-20 (supervisor): the G4 path, the two control criteria, the classical route cap, the freeze. Recorded before any change.
1. **Approved: the privileged oracle tracker, then ONE G4 controls session.**
   - zero-action must FAIL;
   - the oracle tracker is the positive control and must pass criterion 3;
   - B1 + recovery is recorded as a baseline finding, not judged.
2. **Approved: the emulator check becomes the braking travel from the trigger decision pose, +-5 mm.**
3. **Approved: the recovery control.**
   - It is re-pinned to the measured 0.946 m and judged on recovery's job only: every attempt cleared,
     none refused.
   - The 0.5 m progress criterion is dropped (B1 re-stops under its own command).
4. **Every classical baseline (B1, B2, B3, B6, B7) gets the same declared, map-based speed and
   yaw-rate cap along the route.**
   - The cap is the fastest band whose field is clear of MAPPED obstacles at each point, plus a
     yaw-rate limit at bends, computed offline by reusing the oracle restricted to the map.
   - It is non-privileged: unmapped obstacles are excluded.
   - Controllers may go slower, never faster.
   - Learned arms do not get it; the asymmetry is stated in the methods.
   - Implement before B2, B3 and B7 are built.
5. **Freeze, effective when this G4 session passes.**
   - The environment and the certified layer are frozen.
   - Changes are made only for blocking defects (a gate or control made unjudgeable, or the primary
     metric invalidated); everything else goes to `docs/requirements.md`.
   - Then WP2 starts immediately: the PFB is implemented with the settled field construction (reach,
     rotation sweep, nesting, lag-safe composition), and L1's pilot runs in parallel against G4.
6. **Start `docs/realism_audit.md` now:** each emulator feature, its industrial counterpart, and whether
   it is more conservative than hardware.

**2026-10-20 - Rulings 1-6 implemented; the G4 session HELD on one finding (the episode horizon).**
- **Ruling 2:** the emulator control now checks the braking travel from the trigger decision pose,
  +-5 mm (simulator 48.8 mm, emulator 47.5 mm), and passes. The fixture records the simulator's
  decision pose (its `s_m` is taken after each step's motion).
- **Ruling 3:** `tests/test_recovery_sim.py` is re-pinned to 0.946 m and judged on recovery's job only.
- **Ruling 4: the route cap** (`pfbnav/planning/route_profile.py`, `pfbnav/control/route_cap.py`,
  `pfbnav/eval/route_profiles.py`; `baselines.route_cap`). New module; `oracle.py` is untouched,
  since it sits in the scenario closure.
  - Tangent-locked, from the MAPPED world. Per sample: the fastest band clear with its
    tangent-tracking sweep, on the route and at the 50 mm inside offset; where no band survives the
    offset, the band clear on the route.
  - Then anticipated over 0.10 m, and deceleration-limited backward at 0.5 m/s^2.
  - The offset alone left 1.5-2.6 m creep stretches on three tune routes, while on the route itself
    band 0 or better was clear everywhere.
- **Ruling 1: the privileged oracle tracker.** Its schedule is the free-orientation oracle over ALL
  obstacles, bands 1 and 2 added. `fastest_schedule` repeats `fastest_path_time`'s programme and is
  pinned to it by test.
  - **Every state's field carries the sweep of ONE fixed yaw rate, 0.2 rad/s, and the schedule turns
    no faster.** Sweeping every state at its tangent-tracking rate (about 1 rad/s at a 90 deg bend
    in the top band) left 0.5-1.6 m of each route to band 0. At 0.5 rad/s the swept band 0 is not
    traversable on 9/9 routes. A holonomic base need not turn at a bend.
  - When stopped, the tracker asks for the creep regime. A stop mid-rotation followed by a
    0.2 rad/s spin request was held forever (tune_narrow 1).
- **The G4 scripts.** `evaluate.py` gains the `oracle_tracker` controller, and `pure_pursuit` is now
  Recovery(RouteCapped(B1)). Profiles are computed offline before the simulator and written to the
  run.
  - `gate_g4.py`: `--positive-run` is judged; `--b1-run` is recorded, never judged (B1's
    not-released attempts still stop the gate).
  - The runner hands `begin_episode` the episode's identity only.
  - `docs/realism_audit.md` is started.
  - A23 is drafted: `docs/pending/gates_A23.yaml`.
- **Emulated, B1 under the cap (3 tune routes): still fails on 3/3.** It stops where its 0.8 m
  lookahead points its field across a bend, then uses up its 5 recovery attempts. Recorded as the
  baseline finding, as ruled.
- **Emulated, the oracle tracker (9 tune routes, 150 s allowed): 8/9 reach the goal**, at 64, 73,
  75, 79, 83, 88, 90 and 123 s. 0 contacts and 4 stops; the ninth reached 11.5/12.4 m after the
  creep fix.
- **FINDING -- the 75 s horizon cannot be met.** `reward.max_steps` 1500 is a dependent of v_max
  under A5 and was never re-derived for A20.
  - Over the 60 tune routes the privileged schedule needs a median 88 s (8.4 s/m); the emulator takes
    about 1.25 x the schedule.
  - All 17 routes it cannot traverse carry an unmapped obstacle across the route (the tracker
    follows the mapped plan); all 43 without one are traversable.
  - Predicted positive-control successes: 10/60 at 75 s (17 %, below G4's 30 %: GATE BROKEN),
    24/60 at 100 s, 41/60 at 150 s, 43/60 at 200 s.
  - 90 training routes give the same picture (9/90 at 75 s, 62/90 at 200 s).
- **WITHDRAWN: the oracle information run's "mission time 1.22 x constant"**
  (`ORACLE_FREE_info_20261019T164220Z`, A20). It gave bands 1 and 2 no rotation sweep and rotated in
  place at up to 1.5 rad/s with no field check. Under the fields as built, the privileged oracle
  averages about 0.12 m/s.
- **Not run:** no simulator session until the horizon is ruled.

### RULED 2026-10-20 (supervisor): the oracle-tracker finding (A23). Recorded before any change.
1. **Horizon `reward.max_steps` 4000 (200 s), approved as a CORRECTION.** It is a v_max dependent under
   A5 and was never re-derived after A20.
   - **Derivation rule:** the horizon covers the oracle schedule of every traversable built route,
     with margin.
   - A test runs over all built sets, and over the test sets once they are built.
   - `time_cost` per step is unchanged.
   - **Knock-on effects, recorded:**
     - fewer episodes per seed at 300k steps. If L1's pilot fails G4 on learning, `total_timesteps`
       is the first lever, not the reward;
     - a larger evaluation step budget, re-costed in G2 and in the A14 size decision.
2. **The tracker stays on the mapped route** (43/60 clears 30 %).
   - **Before G4:** verify the clearance the last rebuild's A13 bypass rule used. It must reference the
     current narrow floor (0.1987 m) and the current fields, not a hard-coded 0.13.
   - Report, for the 17 non-traversable tune routes, whether a bypass exists at the current floor.
   - A stale clearance is a BLOCKING defect: fix it by reference, rebuild, relock, then G4.
3. **The yaw-rate allowance:** record 0.2 rad/s as the largest yaw rate keeping band 0 admissible
   under the current fields, with its computation, rounded down.
4. **Record, do not fix (freeze):** an ideal tracker averages about 0.12 m/s under A20-A22.
   - It goes into `docs/realism_audit.md` (the fields are more conservative than typical industrial
     configurations, especially in rotation) and into the limitations notes.
   - The oracle mission time is reported as a reference line in the results.
5. **Declared now: L5-nowp**, an exploratory planner-contribution analysis.
   - L5 without the planner: the lookahead channels removed, a straight-line progress reward.
   - 5 seeds, the same seed permutations as L5, outside the Holm family, reported whatever it shows.
   - Trained last, after every confirmatory arm, and the first arm cut if throughput falls short.
   - v1 is recorded as the pilot study motivating it.
6. Then the G4 controls session.

**2026-10-20 - The A23 rulings implemented; two checks reported.**
- **Ruling 2 (bypass clearance): VERIFIED, not stale.** `BypassRule.from_config` reads half_width +
  the narrow stratum's min_clearance. The last rebuild (`SCENARIO_SETS_20261020T073343Z`, c748dc2)
  ran with 0.2086 + 0.1987 = 0.4073 m.
  - **All 17 of the 17 non-traversable tune routes have a bypass at that floor** (W = 1.0 m), so
    nothing is rebuilt.
  - The 17 seeds: 2000008, 2000012, 2000014, 2000018, 2000019, 2000023, 2000024, 2000025, 2000028,
    2000029, 2000033, 2000037, 2000043, 2000050, 2000051, 2000053, 2000055.
  - Only `bypass.py`'s docstring still says 0.13 m. It sits in the scenario closure, so editing it
    would move the lock; it is left under the freeze.
- **Ruling 3 (the yaw-rate allowance), its computation, and a refinement to report.**
  - In the narrowest aisle alone (band 0 at the 50 mm offset) band 0 admits about 0.5 rad/s: 5.2 mm
    at 0.5, -59 mm at 1.0. The binding places are therefore shelf ends along routes.
  - Route-level, band 0 swept at r, the mapped world, the generator's own oracle test:
    - tune: 60/60 traversable at 0.20 and 54/60 at 0.25 (2000018, 2000023, 2000039, 2000042,
      2000051, 2000059 fail), so 0.20 for every tune route;
    - a 90-route training sample: 7 of its 69 routes without an unmapped obstacle are NOT
      traversable at 0.20 (6 at 0.15, 1 needs 0.10).
  - So "0.2 rad/s as the largest yaw rate keeping band 0 admissible" holds on the tune set (G4's),
    not on every built route.
  - **Implemented as the ruling's own rule applied per route:** the largest rate of the ladder
    [0.20, 0.15, 0.10, 0.05] at which band 0 stays traversable, rounded down
    (`oracle_schedule(yaw_rates=...)`, `baselines.oracle_tracker.yaw_rate_ladder`). G4 is unaffected
    (0.20 everywhere on tune). The full set's allowances come from `scripts/route_schedules.py`.
- **Ruling 1 (the horizon):** `reward.max_steps` is 4000 and `reward.horizon_margin` 1.3.
  - The rule is `pfbnav/eval/horizon.py`, checked by `tests/test_horizon.py` over every built set.
    It fails until `scripts/route_schedules.py` is run and `tests/data/route_schedules.json`
    committed.
  - The emulated/scheduled ratios behind the margin, on eight tune routes: 1.12-1.25.
- **Records:** `docs/realism_audit.md` (the consequence section), `docs/writeup_notes.md`
  (limitations), `docs/requirements.md` (the knock-on effects, the oracle reference line, the route
  cap, L5-nowp) and `configs/methods.yaml` (L5-nowp).
- **A23 drafted:** `docs/pending/A23_g4_controls_horizon.md` and `docs/pending/gates_A23.yaml`.

**2026-10-21 - REVIEWED: the G4 positive control (`runs/eval/oracle_tracker_privileged_20261020T221714Z_seed0`,
eef03ef, clean; the horizon check had held over every built set). STOP AND REPORT, as ruled.**
- **The criterion is met.** Success 0.72 (43/60) against >= 0.30; contact 0.00 against <= 0.10.
  Stationary triggers: 0.
- **Every success is a traversable route and every failure a blocked one.** The 43 successes are
  exactly the 43 routes the privileged schedule traverses. The 17 failures (16 `unrecoverable_stop`,
  1 timeout) are exactly the 17 routes reported 2026-10-20, each with an unmapped obstacle across
  the mapped route.
- **Successes:** 17 stops over 447.6 m (3.8 /100 m), 10 of the 43 episodes with any stop. Mission
  times 58-178 s, median about 93 s: the 75 s horizon would have allowed 11.
- **STOP AND REPORT: 14 episodes in which a creep stop ended `unrecoverable_stop`**, all among the 17
  blocked routes. The mechanism, emulated on seed 2000023 (obstacle 0.01 m from the route):
  1. The schedule cannot traverse the route, so it falls back to creep in front of the obstacle.
  2. The tracker, which stays on the mapped route as ruled, creeps along it toward the obstacle at
     0.018 m/s.
  3. It stops once just above standstill (s 1.906), then again as an at-rest trigger (s 1.970).
  4. It ends at rest with its AT-REST field BREACHED.
  - The creep's own reach is contained in the at-rest field (A22), so the robot keeps closing on the
    obstacle by its braking travel after the trigger. The obstacle ends a few mm INSIDE the at-rest
    field.
  - The whole-field restart then blocks every twist, moving away included, because the at-rest field
    is the infimum of every field. The deadlock is permanent.
- **A22 anticipated this** ("stops beginning from creep are at-rest stops, not covered by nesting").
  It is not a construction gap (no stationary trigger), but any controller creeping into an obstacle
  is held forever, learned arms included. Ruling requested before any further G4 run, because a fix
  would change `certified.py` and so invalidate the session.

### RULED 2026-10-21 (supervisor): the receding release (A24), a blocking defect under the freeze. Recorded before any change.
- **Approved: the receding release.** It is the pre-declared creep construction gap. Creep stops are
  at-rest stops and cannot nest, so a release rule is unavoidable. Accepting the deadlock would
  inflate `unrecoverable_stop` and make creep departure unlearnable, contradicting A22.
- **Conditions:**
  1. **Release.** A creep twist releases only if, for every point inside the at-rest field, the
     distance from the closest footprint point strictly increases under the commanded twist (both
     approach rules, strict). A rotation swinging any footprint part toward the obstacle is blocked.
     Unobserved bins still block.
  2. **The exemption.** It covers only the points inside the at-rest field when the stop latched.
     - It holds only while the command stays receding and at creep speed.
     - It re-arms immediately on any non-receding command, or when creep speed is exceeded.
     - It ends when those points leave the at-rest field.
     - Any other breach triggers normally.
  3. **Known-answer tests:**
     - creep in, then the receding release;
     - an approach blocked;
     - a corner-swinging rotation blocked;
     - receding into a second obstacle triggers;
     - receding then approaching re-triggers the next step;
     - with no creep stop, bit-identical to the current rule.
  4. **`docs/realism_audit.md`:** the industrial counterpart is a reset plus a reduced-speed
     manoeuvring mode. Ours releases automatically: less conservative than an operator reset, and
     consistent with the modelled restart delay.
- **The G4 session of 2026-10-20 is recorded as DIAGNOSTIC.** The verdict comes from the repeat after
  A24.
- **Still outstanding from 2026-10-20:** for the 17 blocked tune routes, report whether a bypass exists
  at the current narrow floor (0.1987 m) with the current fields, and confirm the generator's bypass
  rule references the current floor, not a stale 0.13. If it is stale, fix it in the same rebuild.
- **Then:** re-freeze 8 with bit-identity, A24, relock, rebuild, the schedules check, and the G4
  session repeated.

**2026-10-21 - A24 implemented: the receding release; re-freeze 8 (9ed9cfca); A24 drafted.**
- **`certified.py`.** A robot held at rest with its at-rest field breached is released, after
  `restart_delay`, for a creep twist under which every point inside the field strictly recedes:
  - the strict centre rule;
  - the strict rigid-body rule for every footprint boundary point within the at-rest field's
    thickness of the point (so a corner-swinging rotation is blocked);
  - unobserved bins block.
- **The exemption** covers only the released points, tracked +-1 bin per step, while the command
  keeps receding at creep. Anything else ends it and triggers normally; it ends by itself when the
  points have left the field.
- **New report keys:** `receding` and `receding_releases`. `restart_blocked` is now reported while
  held inside the field (it was False there). The stuck classification is unchanged:
  `breached & at_rest` was already stuck.
- **12 known-answer tests** in `tests/test_receding_release.py`, including random scenarios
  bit-identical to A22 (the release disabled) with no creep stop.
- **Outstanding item, answered again (first reported 2026-10-20).** `BypassRule.from_config` reads
  half_width + the narrow stratum's min_clearance, and the last rebuild used 0.2086 + 0.1987 =
  0.4073 m.
  - All 17 blocked tune routes have a bypass at that floor (W 1.0 m).
  - The "current fields" enter through the floor itself, F = the D2 rule under the A21 field.
  - Not stale. The docstring's "0.13 m" is corrected now, since the rebuild happens anyway.
- **Also:** `reward.py`'s docstring ("no recovery mode is added to the emulator") is corrected for
  A24.
- **Open, for the supervisor (not implemented):** the classical recovery still judges directions by
  the whole-field rule, so it gives up when the robot is held inside its at-rest field
  (`no_permitted_direction` or `inside_min_margin`). Letting the declared recovery choose a strictly
  receding creep there would need a ruling (the recovery is declared, 2026-10-16).

**2026-10-23 - REVIEWED: the G4 controls session after A24 (all at 0eba699, clean; re-freeze 8, relock
A24, rebuild 07e468d, the horizon held over every built set). CONTROLS SOUND by the gate's arithmetic,
with two ruled reports.**
- **Oracle tracker** (`oracle_tracker_privileged_20261021T235723Z`): success 0.717 (43/60), contact
  0.00, PASS against >= 0.30 and <= 0.10. Stationary triggers 0.
  - The 17 non-successes are again exactly the 17 blocked routes: 16 `unrecoverable_stop`, 1 timeout.
  - Successes: 4.25 stops /100 m, mission time median 89 s, max 178 s.
- **Zero-action** (`zero_action_20261022T195850Z`): 60/60 timeouts, 0 successes. It behaves as
  required.
  - Two earlier attempts, `zero_action_20261022T074653Z` (19 episodes) and
    `zero_action_20261022T132922Z` (5), were interrupted; they have no summary, and the gate does
    not read them.
- **B1 + route cap + recovery** (`B1_pure_pursuit_20261023T001602Z`), RECORDED, not judged: 0/60,
  all `unrecoverable_stop`.
  - 300 recovery attempts, all `cleared`, none refused: no suspected recovery defect. B1 re-stops
    after each cleared attempt until the cap of 5, then is held.
  - 195 stops /100 m.
- **Report 1 (ruled: a stationary trigger is a construction gap, reported).** Zero-action, seed
  2000018, index 18: one stationary trigger, held 123 steps (6.2 s), then released; the episode ended
  as a timeout.
  - Nothing lies within reach of the start pose's at-rest field. Over all 717 built scenarios with
    unmapped obstacles, no start has an obstacle within the field's reach.
  - The same scenario at the same commit had 0 stops in the interrupted first run.
  - The trigger coincides with a burst of over-budget steps, host load: 151, 199, 259 and 120 over
    the 160 ms budget on indices 15-18.
  - A geometric cause is excluded. A sensing or timing artefact under load is suspected and NOT
    established: the evaluation keeps no per-step trace. A traced rerun of that scenario is
    proposed.
- **Report 2 (ruled: a creep stop ending `unrecoverable_stop` means stop and report).** The tracker
  has 14 such episodes, the same 14 blocked routes as the diagnostic session.
  - A24 behaves as designed: the release needs a strictly receding creep.
  - The tracker never asks for one. It stays on the mapped route (ruling 2 of 2026-10-20) and keeps
    commanding creep TOWARD the obstacle, which the layer correctly holds.
  - So this is the positive control's behaviour on routes it cannot traverse, not a construction
    gap.

### RULED 2026-10-23 (supervisor): G4 controls accepted; the freeze takes effect; WP2 starts. Recorded before any change.
- **The verdict is accepted: G4 controls sound.** The freeze takes effect now, and WP2 starts. L1's
  pilot waits for the traced rerun below.
1. **The stationary trigger: a traced rerun, approved and declared before running.**
   - Scenario 2000018, zero-action, per-step trace, at the current commit: 20 runs under normal load
     and 20 under induced heavy load.
   - **The outcomes, declared:**
     - reproduces only under load: a BLOCKING defect (wall-clock leaking into the step); fix and
       re-freeze before L1's pilot;
     - reproduces under both: a construction gap at that start pose; report it with the trace;
     - never reproduces in 40: record it as an unexplained single event, keep the counter, escalate
       on recurrence.
   - **Also:** add a per-step ring buffer to evaluations, dumped automatically on stationary
     triggers, unrecoverable stops and contacts. The changed evaluation code must pass the filter-off
     bit-identity test.
2. **The creep deadlocks on blocked routes are accepted as controller-attributable** (the tracker
   never commands the receding creep A24 allows).
   - In the same session, add one scripted in-loop check of A24: creep into an obstacle until
     stopped, then command a receding creep.
   - Required: the release occurs, and the robot leaves the at-rest field without a new trigger.
- **Confirm in the record:**
  - (a) the bypass check for the 17 blocked tune routes at the current narrow floor, and that the
    generator's bypass rule references the current floor -- cite it if done, otherwise run it before
    L1's pilot;
  - (b) whether the map-based speed and yaw cap was in force for B1's 0/60. If not, label the result
    pre-cap; if so, record it as a possible strawman warning to revisit at WP3.
- **Then WP2, and L1's pilot once the rerun and both checks are reported.**

**2026-10-23 - The 2026-10-23 rulings implemented; both record checks confirmed.**
- **(a) Bypass, CONFIRMED (done; cited).**
  - The generator's rule reads the floor by reference: `BypassRule.from_config` takes half_width +
    the narrow stratum's min_clearance (`pfbnav/world/bypass.py`; `scenarios.py` calls it).
  - The rebuild of 2026-10-20 (`SCENARIO_SETS_20261020T073343Z`, c748dc2) ran at 0.2086 + 0.1987 =
    0.4073 m. The A24 rebuild (07e468d) reads the same config.
  - All 17 blocked tune routes have a bypass at that floor (W 1.0 m). Reported 2026-10-20 and
    2026-10-21, with the seeds in the 2026-10-21 entry.
  - The only stale "0.13" (a docstring) was corrected with A24.
- **(b) The route cap WAS in force for B1's 0/60** (`B1_pure_pursuit_20261023T001602Z`). The run
  holds 60 `cap` profiles in `route_profiles.json`; every episode reports the cap, which bound on
  213,593 of its 240,000 steps (89 %).
  - **Recorded as a POSSIBLE STRAWMAN WARNING, to revisit at WP3.** B1 under the cap and the declared
    recovery reaches no goal on the tune set: 300 recovery attempts, all `cleared`, each followed by
    a re-stop until the cap of 5. Before B1 serves as an H1 comparator, WP3 must show a classical
    baseline that is not trivially beaten (its tuning, or the rationale for its form).
- **The ring buffer:** `pfbnav/eval/ring.py`, the runner's `ring=` and `evaluation.ring_steps` 200,
  wired into `scripts/evaluate.py`, with dumps under `ring/`.
  - The last 200 steps (full scan to the mm, certified report, commands, step time and over-budget
    flag) are dumped on a stationary trigger, at the first contact, and for an unrecoverable stop
    (the window before the last stop trigger plus the episode's last steps).
  - It is passive: `tests/test_ring.py` shows the runner returns identical records with it.
  - It changes no environment code, so the environment's identity is unchanged; the filter-off
    bit-identity test is rerun as ruled.
- **The traced rerun, DECLARED before running:** `scripts/diag_stationary_repro.py`.
  - Scenario 2000018, zero-action wrapped as G4 ran it, one scene load: 20 runs normal, then 20 under
    induced load (one busy process per logical core). Every step is traced, with ring dumps.
  - The supervisor's three outcomes, plus "only without load" (stop and report).
  - **The load's positive control:** a median of >= 100 over-budget steps per loaded run and < 100 per
    normal run, else INCONCLUSIVE. The verdict logic is tested in `tests/test_diag_stationary_repro.py`.
- **The A24 in-loop check:** `tests/test_receding_release_sim.py` (sim-marked).
  - It creeps into a wall placed in memory until the at-rest field trips, then commands a receding
    creep.
  - Required: held at rest WITH the at-rest field breached (the positive control: a genuine A24
    case), the release occurs, and the robot leaves the at-rest field with no new trigger.

### RULED 2026-10-23 (supervisor): the scan-fault fix. Recorded before any change.
- **Approved as proposed.**
  - A plausibility check against the last accepted scan: +-2 bins, +-6 cm; >= 6 bins outside means
    corrupt.
  - Up to 2 re-renders of the same tick, then all bins unobserved (fail-safe), counted as
    `scan_faults`.
  - Re-freeze 9 with bit-identity. The 20 loaded runs are repeated, and zero stationary triggers are
    required. No rebuild or relock.
- **Why it is blocking:** the fault is load-dependent, and the PFB adds per-step compute. Left
  unfixed, it would bias H1 against L5.
- **H1:** stops caused by scan faults are EXCLUDED from the primary stop count and reported alongside
  it, per method.
  - If `scan_faults` differ materially between methods, that is reported prominently.
  - Any run with more than 0.1 % of its steps scan-faulted is flagged, and that host configuration
    runs fewer instances.
- **G2 gains a criterion:** the selected instance count must produce essentially no over-budget
  steps. The over-budget count is recorded per run.
- **The Run Request for the 20 loaded runs; the researcher runs them.** WP2 proceeds in parallel. L1's
  pilot follows once the loaded runs pass and the two checks of the previous ruling are reported
  (the bypass at the current floor, B1's cap status).

### RULED 2026-10-23 (supervisor, amending the scan-fault ruling): the 20 loaded runs WITHDRAWN. Recorded before any change.
- **Why:** with a base rate near 1/20, zero events in 20 runs would occur about 36 % of the time even
  without a fix, so the test cannot verify it.
- **Replaced by:**
  1. **A deterministic fault-injection test** replaying the two recorded corrupted packets through the
     scanner read path: both flagged, re-rendered, then marked unobserved, and no stationary trigger.
  2. **The false-alarm checks kept as regression tests:** the 198 recorded good steps and the 3,196
     emulated moving steps.
  3. **The re-freeze 9 bit-identity test.**
- **Verification in the loop comes from L1's pilot:** report its `scan_faults` and any ring-buffer
  dumps.
- **Everything else in the previous ruling stands.** Proceed to WP2 and, once the two outstanding
  checks are reported, L1's pilot.
- **Implementation note:** a stop decided on a fail-safed packet (every bin unobserved) fires with the
  robot standing, so the layer's own report calls it stationary. The loop therefore counts it as a
  scan-fault stop. It is kept OUT of the at-rest and stationary counters (the construction-gap
  counters) and out of the primary stop count, and reported apart.

**2026-10-23 - The scan-fault fix implemented; re-freeze 9 (94ac13c7); A25 drafted.**
- **The fix:**
  - the guard, `pfbnav/sim/scan_guard.py` (`scanners.plausibility`);
  - the same-tick `rescan` in `lua/scanner_v2.lua` (the stamp is kept) and `Scanner.rescan`;
  - `StepLoop._sense` (read, re-render up to 2 times, fail-safe);
  - the counters `scan_faults`, `scan_rerenders`, `scan_fault_stops` (out of the at-rest and
    stationary counters, and out of `stops_per_100m`);
  - `info["scan_fault"]` and `info["decided_on_fault"]`, a ring dump on every scan fault;
  - `outcome_summary`: the primary count without scan-fault stops, the scan-fault share and its 0.1 %
    flag, and over-budget steps.
- **Tests** (`tests/test_scan_guard.py`, 12):
  - the fault injection: the recorded packets 1336 and 1337 through the step loop's read path with a
    stand-in scanner whose re-renders are corrupted too. Both are flagged and re-rendered twice, the
    fail-safe applies, and the stop it causes counts as one scan-fault stop with 0 stationary and 0
    at-rest triggers;
  - a corrupted read that re-renders clean is used and is not a fault;
  - without corruption, the read path is the plain read;
  - the two false-alarm regressions: 0 of the 198 recorded good steps flagged; at most 5 of 120 bins
    out of bounds over 3,196 emulated moving steps (the prototype's worst was 1).
- **Added (sim-marked, `tests/test_scan_rescan_sim.py`):** the real re-render must reproduce the
  packet exactly. The fault injection uses a stand-in scanner, so only this shows the Lua re-render
  path works.
- **A25 drafted:** H1's exclusion and reporting (`endpoints_A25.yaml`) and G2's over-budget criterion
  (`gates_A25.yaml`). These change preregistered text, so A25 needs a relock; no rebuild.

**2026-10-23 - WP2 started: design note and decision request** (`docs/pending/WP2_pfb_design.md`).
- No filter code is written; the equations wait for a ruling (hard rule 7).
- **Proposed:**
  - per-bin barrier rows h_i = r_i - rho(psi_i; V(u), theta(u), w(u)), with rho built as the settled
    construction (reach, rotation sweep, nesting, lag-safe composition) but at a continuous speed;
  - a discrete-time CBF condition, linearised about the nominal command with a true-constraint check;
  - a QP with the wheel polytope (0.95 v_rim), the box and the limiter envelope;
  - a maximum-deceleration fallback, flagged and counted;
  - independence from `certified.py`, cross-checked in tests only.
- **The central question (Q1):** a PFB that keeps only a continuous field clear cannot prevent
  band-discretisation stops. The recommendation is the band field, solved per band (three small QPs),
  so that the certified layer never triggers on observed static geometry.
- **Seven questions (Q1-Q7)** are put to the supervisor.

### RULED 2026-10-23 (supervisor): WP2, the PFB. Recorded before any filter code (hard rule 7).
- **Q1: (b), the band field, one QP per band, best feasible.**
  - **C1 restated:** a CBF whose safe set is the complement of the certified safety function's own
    trigger set; the direction, footprint and rotation terms are the technical content.
    `writeup_notes` updated accordingly.
  - **Recorded:** success rate and mission time are reported beside H1 for every comparison, in the
    same table, and the discussion leads with the stop-throughput trade-off. B6 is the decisive
    comparator.
  - **H3 restated before training** as an exploratory ablation: PFB-(b) banded vs PFB-(a)
    continuous.
- **Q2:** the field geometry uses the certified parameters, read from the same config keys (rule 16).
  The PFB's own latency sets only the prediction horizon: one control step plus a declared lag
  allowance, covered by the noise and latency sensitivity check.
- **Q3:**
  - linearise about u_nom, check the true constraint, one re-linearisation, then fall back;
  - analytic gradients verified by finite differences;
  - report `pfb_infeasible` per method and per stratum;
  - test that the max-braking fallback never produces contact from any fallback state.
- **Q4:** one alpha for L5, B7 and L3, tuned with B7 on the tune layouts under the equal-budget rule,
  before L5's pilot.
  - The grid: {1, 2, 5, 10, 20} 1/s.
  - The selection: among settings with success within 5 pp of the best, the fewest protective stops
    per 100 m.
  - The grid and the rule are declared before tuning.
- **Q5:** W = diag(1, 1, rho_out^2), with rho_out = 0.351 m read from the footprint table.
- **Q6:** carry the binning penalty, set from the measured value at its upper bound (hard rule 15), not
  the derived term.
- **Q7:** under the latch, pass only commands the restart condition could release (creep, receding
  twists per A22/A24), else zero. Test that B7's recovery commands pass unaltered when releasable.
- **A PFB acceptance step before alpha tuning:** an ideal tracker plus the PFB on the tune routes, with
  the certified layer beneath. Every certified stop on static geometry is attributed to prediction
  error, an unobserved bin, infeasibility or creep. Unattributed stops must be zero; otherwise stop and
  report.
- **Order:**
  1. the rho construction, with cross-check tests;
  2. the QP, with known answers;
  3. the emulator closed loop and the acceptance step;
  4. the simulator acceptance tests;
  5. alpha tuning;
  6. L5's pilot against G4.

**2026-10-23 - Two values in the WP2 rulings, as found in the repository.**
- **Q5's rho_out.** The footprint table's outermost radius is **0.3374 m** (0.33742; `FootprintTable`
  `circumscribed`). 0.351 m was the radius WITH the pods (0.3519), before the S300 layout, the same
  slip as in the recovery ruling of 2026-10-19. The ruling says "read from the footprint table", so W
  uses the table's value, read at run time, never restated.
- **Q6's measured penalty.**
  - The measurement is the 2026-09-27 positive control (`TRAVERSABILITY_20260927T222532Z`): against
    the wall's near face the scan residual is **-0.27 to +0.50 mm** (mean +0.08).
  - At its upper bound (the CLAUDE.md rule: measured values at the bound that overstates the robot),
    delta = **0.50 mm**.
  - The sector-minimum binning is already IN the real scan, which both layers read and compare bin by
    bin in the same way, so the derived sector-min term is not added (as ruled).
  - For reference, the D2 real-scan margins of 2026-10-20 sit 0.49-0.54 mm ABOVE the unbinned
    geometry, so no positive scan cost is measured there.
- **H3 restated:** it is preregistered (`endpoints.yaml`, "mission time decreases with field
  granularity (1, 3, 5 field sets, continuous PFB)"), so the restatement is an amendment (A26,
  drafted with the H1 reporting rule).

**2026-10-23 - WP2 step 1 done: the PFB field (`pfbnav/safety/pfb_field.py`), cross-checked.**
- **The module:** pure, and independent of `certified.py`. It imports neither the certified layer
  nor `world.strata` (which does); `tests/test_architecture.py` is extended to cover it.
- **It reads the SAME config keys (rule 16, Q2):** T = certified response time + dt; a_brake, a_max,
  alpha_max (also the yaw braking); the planar speed cap from the box and the filter's budget, its
  own derivation; yaw_cap; the margins; the standstill exit; the band tops.
- **The construction:** the reach, the rotation sweep +-(E(|w|) + E(0)), travel and swerve, the
  turning drift, the margins, the at-rest infimum.
  - The rotation sweep is DERIVED DIFFERENTLY: the maximum over hull vertices of
    r_v max over phi of cos(n - phi - a_v), in closed form.
  - V may be any speed (PFB-(a), H3's ablation) or a band top (PFB-(b), adopted).
- **42 tests** (`tests/test_pfb_field.py`):
  - the same constants from the same keys;
  - each scalar term against the certified one (to 1e-15);
  - the band choice;
  - the closed-form sweep against brute-force sampling, and against the certified derivation
    (1e-12);
  - **the PFB-(b) band field EQUALS the certified field at every band top, heading and yaw rate
    (1e-12)**, so its safe set is the complement of the certified trigger set (C1); the at-rest field
    is equal too;
  - PFB-(a) never exceeds the band field;
  - monotone in speed and yaw rate;
  - exact known sums ahead and behind.
  - **A wrong premise of mine was corrected:** the field does not reach furthest straight ahead. With
    a rectangular footprint its farthest point is about 16 deg off-axis (the front corners plus the
    depth), the certified field's shape too.
- **Found: too slow for the loop.** One band-field evaluation costs 297 ms, because the hull has
  1,334 vertices and the closed form visits every vertex for each of the 1,440 normals.
  - The first task of step 2: the supporting-vertex search plus a range-maximum table (written
    independently of the certified layer's), proven equal by the same cross-checks, then timed
    against the step budget.

**2026-10-23 - WP2 step 2 done: the QP (`pfbnav/safety/pfb.py`), with known answers; the field
sped up.**
- **The speed fix** (`SweptFootprint`): S(n; e) = max(h(n - e), h(n + e), the largest radius among
  the vertices with angle in [n - e, n + e]).
  - h is found by binary search on the hull's edge normals, the range maximum by a sparse table.
  - It is written independently of the certified layer's.
  - 297 ms -> 2.0 ms per band-field evaluation. It equals the all-vertices closed form to 1e-15
    (test), and the certified cross-checks still hold.
- **Analytic partials of rho** (bearing, heading, |w|) through each bearing's active normal, every
  max differentiated on its active branch. They match finite differences on 1,500 bearings each.
  |w| has a genuine kink at w = 0 (one-sided slope 0.3).
- **The QP, as ruled:**
  - one per regime (the at-rest creep regime from rest, then every band from the one the ESTIMATE
    selects), the best feasible kept (Q1);
  - rows g_i(u) = r_i(u) - rho(psi_i(u); theta(u), max(|w_hat|, |w|)) - delta, predicted over
    tau = dt + `lag_allowance_s` (Q2);
  - the CBF condition >= (1 - alpha dt) h_i(now);
  - linearised about u_nom, the TRUE rows checked, one re-linearisation, then the fallback (Q3);
  - W = diag(1, 1, rho_out^2) with rho_out = 0.3374 from the table (Q5); delta = 0.50 mm (Q6);
  - the wheel polytope (8 exact faces), the box, the limiter's one-step envelope (an inscribed 16-gon
    of radius a_max dt; the fallback brakes at a_decel), the regime's speed range;
  - max-braking fallback, flagged, with the reason `unobserved` or `infeasible`;
  - under the latch, only releasable commands (a creep with the at-rest field clear or strictly
    receding, or a twist whose band field is clear), else zero (Q7).
- **The `filter:` config:** the field reads the certified keys (Q2). `t_react`, `a_brake` and `margin`
  are removed as restated constants. `test_infrastructure` and `diag_braking` now read
  `certified.a_brake`, the same value. Added: `lag_allowance_s` 0.05 (VERIFY), `binning_penalty_m`
  0.0005, `creep_fraction` 0.9, `row_window_m` 0.5, `linearisations` 2. alpha stays 2.0, PROVISIONAL
  until tuning (Q4).
- **13 tests** (`tests/test_pfb.py`):
  - the row gradients equal finite differences in every regime;
  - a clear nominal passes unchanged with no QP;
  - a wall ahead slows the robot and every true row holds;
  - sliding past is preferred to stopping;
  - an unobserved bin, or being boxed in, falls back to maximum braking along the previous command;
  - random scans either hold every true row or say infeasible, and the limits always hold;
  - the weights and delta are the ruled ones;
  - under the latch only releasable commands pass;
  - B7's recovery creep passes unaltered (Q7).
- **Open, for the performance sub-step after the closed loop:** about 25-40 ms per filtered step on
  this laptop. Each QP is set up fresh with tolerances at 1e-9, and three row evaluations are made per
  regime. Planned: warm-started or reused solver set-up, looser tolerances verified by the
  true-constraint check, rows evaluated only where they can bind. G2 sets the budget.

- **Retired with step 2: `tests/test_pfb_primitives.py`** (WP0). It tested the v1 stub's rectangular
  `footprint_extent` and v1 `stop_distance` (vT + v^2/2a + margin), both removed from `pfb.py`.
  - The footprint is the measured `FootprintTable` (its own tests).
  - The depth is the reach construction's D(V) = V T + V^2/(2 a_b), with margins applied by direction.
    It is known-answer- and cross-checked in `tests/test_pfb_field.py`.

- **Reverted the same day:** a docstring edit in `pfbnav/sim/robot.py` (`filter.a_brake` ->
  `certified.a_brake`).
  - The file is inside the frozen environment's identity, so the edit moved its hash. The fast suite
    showed it: the freeze check failed, and with it five evaluation and training tests that refuse a
    non-frozen environment.
  - Under the freeze only blocking defects change environment code, so the edit waits in
    `docs/requirements.md` for the next legitimate re-freeze.
  - `pfb.py` and `pfb_field.py` are NOT in the environment's closure (checked), so WP2's filter code
    does not move the freeze until it is wired into the step loop.

**2026-10-23 - Rule 7 entry: the PFB's CBF condition clipped at zero; found by the acceptance machinery.**
- **The condition is now g_i(u) >= (1 - alpha dt) max(h_i(now), 0)** (was (1 - alpha dt) h_i(now)).
- **Found:** on a blocked tune route (tune_medium 3, seed 2000023) the attribution reported two
  UNATTRIBUTED certified stops.
  - The robot was at rest beside the unmapped obstacle. The PFB chose a command just above the creep
    limits (0.023 m/s, 0.088 rad/s), entering band 0, whose field already held the obstacle
    (h_now < 0).
  - The decay form only asks a negative h to recover at rate alpha, so it allowed the switch, and the
    certified layer stopped the robot the next step. Regime, heading, yaw rate and motion all matched
    the PFB's assumptions, so nothing explained the stop.
- **Why the clip is right:** within one field the decay condition keeps h >= 0 once it is >= 0. ACROSS
  regimes h differs, and a field the robot is not yet in may already be breached. C1 is to keep the
  field the robot WILL be in clear, so a regime whose field is breached now must be clear after one
  step, which in practice rules it out. For the regime in force, h_now < 0 means the certified layer
  has already triggered.
- This is a change to the filter equation (hard rule 7), reported to the supervisor with the
  acceptance results.

**2026-10-23 - WP2 step 3: the emulator closed loop and the acceptance machinery; QP corrections.**
- **What was built:**
  - **Emulator PFB slot.** `emulate_layout(..., pfb=, goal=, goal_tolerance=)` in
    `pfbnav/eval/recovery_control.py` filters the controller's command through `PFB.filter` (Q7 under
    the latch) before the limiter and the certified layer, records the PFB's decision on every step,
    and ends at the goal.
  - **Ideal tracker.** `pfbnav/control/ideal_tracker.py`, `baselines.ideal_tracker`: tangent tracking
    at 0.5 m/s, blind to the scan, so every slow-down is the PFB's. It is not a baseline.
  - **Attribution.** `pfbnav/eval/pfb_acceptance.py` gives each certified stop exactly one of the ruled
    causes (unobserved, infeasible, creep, prediction: regime / heading / yaw / motion), or
    unattributed. It also rolls the Q3 braking fallback forward from every fallback state and checks
    footprint contact on every step. Known-answer tests are in `tests/test_pfb_acceptance.py`.
  - **Runner.** `scripts/pfb_acceptance.py` (researcher-run, 60 tune routes).
- **QP corrections, each found by a closed-loop failure** (filter internals, not the field or the
  CBF form; rule 7 recorded here):
  1. **Linearisation margin 0.5 mm.** The true rows are checked with no tolerance; the linearised rows
     are tightened by 0.5 mm. With 1e-7, 28 % of steps fell back.
  2. **Chord rows over a yaw trust interval of 0.05 rad/s.** g is concave in w (convex field growth
     and the |w_hat| kink), so tangents overshot by 3-15 mm.
  3. **Linearisation at a reachable point**, not at u_nom.
  4. **The limiter's braking case as a second convex envelope** (a_decel, |u| <= |u_prev|), tried
     after the a_max case; the best feasible cost wins.
  5. **Band speed margin 0.002 m/s** below each band's limit.
  6. **The certified standstill hysteresis mirrored** from the same keys (`rest_enter`,
     `rest_yaw_enter` added to `FieldParams`, cross-checked).
  7. **The CBF right-hand side clipped at zero** (entry above).
  8. **Re-linearisation after an infeasible first QP, about the braking command.** Q3's one
     re-linearisation was spent only after a solution. Rows linearised toward a nominal driving into an
     obstacle excluded backing away.
  9. **The braking command is checked against the TRUE rows before a step is declared infeasible.**
     It is the same command as the fallback, and is labelled `brake` when it satisfies the CBF. The
     0.5 mm tightening exceeds the allowed decay alpha dt h_now once h_now < 5 mm, so the QP could
     miss that holding still is safe. `pfb_infeasible` now counts only steps where no checked command
     satisfied the CBF.
  - Effect on the blocked tune route (tune_medium 3, 600 steps): 'infeasible' steps fell from 62 %
    (fix 7 alone) to 148 `brake` + 1 `fallback` with fix 9, then to 11 `brake` + 8 `fallback` with
    fix 8.
  - Effect: fallbacks fell from 85/300 steps to 1/300 on the dead-end probe.
- **Before any tuning:** with the PFB, the ideal tracker reached the goal at about 0.4 m/s with zero
  certified stops on tune_wide 1 (19 s, 7.9 m) and tune_narrow 0 (26 s, 10.1 m). The recorded
  "ideal tracker ~0.12 m/s" was the oracle schedule's 0.2 rad/s yaw cap, not a property of the field.
  To be reported to the supervisor with the acceptance results.
- `VERIFY` on `yaw_trust_rad_s` and `band_speed_margin_m_s`: chosen to cure the failures seen, not
  measured.
- **Declared stall cut-off for the acceptance run:** a route ends when its progress over the last 30 s
  is below 0.05 m.
  - Why: the ideal tracker is blind and never replans, and 19 of the 60 tune routes have an unmapped
    obstacle near the route. Without the cut-off they stall to 4000 steps at about 1 s of compute per
    step.
  - Stalls are counted, not judged.
  - Checked on three such routes: two stalled, at 754 and 856 steps; one reached the goal. Every stop
    was attributed (`infeasible`), with no contact and no rollout contact.

**2026-10-24 - REVIEWED: the PFB acceptance run (PFB_ACCEPTANCE_20261024T051004Z_seed0): STOP AND
REPORT, one unattributed stop, traced to the emulator's scan; the scan corrected.**
- **Provenance:** commit 762531b; seed 0; scene 537f1bbb. The `dirty` flag is set only by the
  untracked `paper/` folder (the record counts untracked files). The A26 relock was not yet committed;
  the lock test still fails on AMENDMENTS.md.
- **Result:**
  - 60 routes: 48 reached the goal and 12 stalled under the declared cut-off.
  - Contact steps 0, braking-rollout contacts 0.
  - 29 stops: 15 infeasible, 11 prediction (heading 11, of which motion 1), 2 creep,
    **1 unattributed** (tune_narrow 2000007, step 523).
  - Fallback steps 264, all `infeasible`.
  - The verdict is STOP AND REPORT, as declared.
- **The unattributed stop, traced** (reproduced exactly in the emulator):
  - The robot was turning at 0.51 rad/s past a shelf corner behind-right. At step 522 the PFB
    predicted a minimum g of +4.4 mm for its command; the layer then found -3.1 mm at bearing
    -1.963, range 0.376.
  - Regime, field heading and yaw rate all matched what the PFB assumed. The realised rotation equalled
    the command over the step.
  - **The cause is the emulator's scan.** It casts ONE ray per bin centre, while the simulator's
    filter stream sends each 3 deg sector's NEAREST point at the bearing it was measured at
    (`pfbnav/sim/scan_packet.py`: "the PFB evaluates its barrier at the bearing each range was
    actually measured at"). With a single ray, the surface between centres is unobserved. As the
    robot turned, the ray at -1.963 fell on a nearer part of the corner that no earlier reading
    contained, and no prediction made from the earlier readings could include it.
  - Predicting over dt instead of dt + lag still gave +1.7 mm, so the lag horizon does not explain it.
- **Evidence:** the same four routes were rerun with a sector-minimum, exact-bearing scan (16 rays per
  sector, prototype).
  - The unattributed stop is gone (tune_narrow 2000007: one `infeasible` stop, at step 439).
  - The heading-prediction stops on tune_narrow 2000001 and 2000006 and tune_medium 2000022 are gone.
  - All four routes still reach the goal.
- **The correction:** `raycast.sector_scan` reproduces the filter stream. Its ray density is set
  from the MEASURED pixel density (`scanners.rays_per_degree` 4.27 x 3 deg = 13 rays; rule 15).
  - Tests: a known answer, a cross-check of the vectorised caster against the exact one, and a
    positive control (a corner between two bin centres is missed by one ray per centre and seen by
    the sector scan; `tests/test_raycast.py`).
  - `emulate_layout(..., sector_rays=)` uses it. The default is unchanged, so earlier emulator
    analyses reproduce. The acceptance runner declares it.
  - No filter equation changed.
- **For the supervisor:** whether the rerun with the faithful scan stands as the acceptance step. The
  first run's verdict is not reinterpreted: it stays STOP AND REPORT for the scan it used.

**2026-10-24 - REVIEWED: the PFB acceptance rerun (PFB_ACCEPTANCE_20261024T174707Z_seed0): PASS,
subject to the supervisor accepting the scan correction.**
- **Provenance:** commit 2ff2f74; seed 0; scene 537f1bbb. The `dirty` flag comes only from the
  untracked `paper/`. The A26 relock is still not committed: LOCK.json is relocked_for A25.
- **Declared criteria:**
  - Unattributed stops 0.
  - Braking-rollout contacts 0 from 243 fallback states.
  - Contact steps 0 over 38,875 steps.
  - **PASS.**
- **Counts** (not criteria):
  - 48 routes reached the goal; 12 stalled, all on routes with an unmapped obstacle near the
    route, which the blind tracker cannot bypass.
  - 30 stops: 17 prediction (all `heading`; 10 of them in wide) and 13 infeasible.
  - Commands: filtered 35,547, nominal 743, brake 1,168, latched 1,174, fallback 243 (0.63 % of
    steps).
- **Against the first run:** unattributed 1 -> 0, creep 2 -> 0, prediction 11 -> 17,
  infeasible 15 -> 13.
- **The remaining stop sources are heading prediction and infeasibility.** Heading: the field heading
  the layer used differed from the one the PFB assumed by more than 0.02 rad. These are the PFB's
  share of H1 stops, and alpha tuning (step 5) is ruled to select on stops/100 m.

**2026-10-24 - WP2: the PFB brought inside the per-step budget, by two exact changes, before it
enters the frozen environment.**
- **Why now:** once wired into the step loop, `pfb.py` and `pfb_field.py` join the environment's
  import closure, so any later speed-up would need another re-freeze. The budget is 160 ms per step
  per instance (25 steps/s over 4 instances), and the simulator alone takes about 60 ms. The PFB took
  59 ms mean (p95 102) on an open route and about 1 s on a stalled one.
- **1. The field's extent through its boundary polygon** (`PFBField._active_normal`).
  - The field is the intersection of the half-planes n . x <= h(n), and h is built from sums and
    maxima of support functions, so every line touches the set. The boundary is then the polygon of
    consecutive line intersections, and the ray at psi leaves through the edge whose vertices bracket
    it: one sorted search instead of a 120 x 1440 minimum.
  - Where the vertices are not a valid convex polygon, the brute-force minimum answers and the
    evaluation is counted.
  - Tests: rho equals the brute-force minimum to 1e-12 over 120 random fields (every regime, heading,
    yaw rate), with no fallback needed; a positive control (a support with a line that misses)
    triggers and counts the fallback. The certified cross-checks still pass.
- **2. The QP solved by Clarabel** (interior point; `filter.qp_solver: clarabel`, OSQP kept as the
  check).
  - About 1 ms per QP against OSQP's 12 ms, almost all of it OSQP's setup; the same answers.
  - Test: same modes, and commands within 1e-5 on 25 scans that exercise the QP. Every answer is
    still checked against the TRUE rows.
  - `clarabel` added to the dependencies.
- **Effect:**
  - open route (tune_narrow 0): 59 -> 14 ms mean, p95 102 -> 22 ms;
  - blocked route while stalled (tune_medium 3): 74 ms mean, p95 134 ms. Every band is tried there
    (8 solves).
- The acceptance run used the previous code. Both changes are equivalence-tested, but the acceptance
  step is rerun on the final code with step 4's Run Request.

**2026-10-24 - WP2 step 4: the PFB in the step loop's slot; re-freeze 10 (83cc02d8); the simulator
acceptance step prepared.**
- **The slot** (`pfbnav/envs/step_loop.py`). With `filter.enabled`, the loop builds the PFB from the
  same footprint table and config keys as the certified layer.
  - Each step it filters the controller's command on what the layer is about to decide on: the
    scan in hand (the filter stream, with exact bearings and validity), the velocity ESTIMATE, the
    command the base is executing, and the latch the layer left after the previous step (Q7). The
    layer then judges the PFB's output.
  - Recorded: the PFB's decision on every `StepState` and in `info["pfb"]`; `pfb_infeasible`
    (reported per method and stratum, Q3) and `pfb_modified` in the loop's report; the PFB's time
    under `timings["pfb"]`.
  - **Off:** nothing is built, the slot hands on the command object itself, and the off path reads
    no PFB state. This is what the filter-off bit-identity test guards.
- **Re-freeze 10:** `pfb.py` and `pfb_field.py` enter the import closure, as `freeze.py` anticipated.
  The filter-off bit-identity test reruns on the simulator before the re-freeze stands.
- **Tests (fast):**
  - the loop builds the PFB only with the filter on, with no simulator I/O;
  - offline, the filter is handed exactly the layer's inputs (exact non-centre bearings, the
    estimate, the applied command, the latch);
  - its output is what the layer judges and the base receives; recording and counting are checked;
  - **ruling 3's acceptance test:** fed an estimate that differs from the command, the field follows
    the estimate (band and heading).
- **Refusals updated to the true reason.** `train.py` refuses `filter.enabled` until alpha is ruled
  (step 5); `evaluate.py` refuses it until step 5 declares the alpha grid.
- **The simulator acceptance step** (`scripts/pfb_sim_acceptance.py`,
  `configs/analysis/pfb_acceptance_sim.yaml`): the emulator step's declared criteria, unchanged,
  through `NavEnv` on the 60 tune routes.
  - Contact is the simulator's contact check (hard rule 4).
  - Stops decided on a scan-faulted packet are counted apart (A25).
  - The fallback rollout uses slip 1.0, the conservative travel.
  - The PFB's own time per step is reported.
- **Not changed:** the reward's intervention term still compares the policy's request with the
  applied command, so with the filter on it includes the PFB's changes. That is the existing
  definition (`intervention_magnitude`), recorded here because L5 will train on it.

### RULED 2026-10-24 (supervisor): the WP2 progress report. Recorded before implementation.
1. **Acceptance.**
   - The emulator rerun (PFB_ACCEPTANCE_20261024T174707Z) stands as evidence, and the first run's
     STOP AND REPORT stays recorded.
   - **The decisive acceptance is the simulator step on the final code.**
   - **New rule:** every new emulator analysis uses the sector scan (`sector_rays`); the one-ray
     default remains only for reproducing earlier analyses. Added to CLAUDE.md's conventions.
2. **Approved: g >= (1 - alpha dt) max(h_now, 0).**
   - Recorded as a **deliberate departure from the standard discrete-time CBF form**: entering a
     breached band's field is itself the event to avoid, since the certified layer acts at once.
   - `yaw_trust_rad_s` and `band_speed_margin_m_s` keep VERIFY and are recorded as design parameters
     selected on tune routes.
   - **After alpha is tuned:** report the sensitivity of stops and infeasibility to each at +-50 %.
3. **The intervention term must equal ||u_request - u_PFB||^2.**
   - It is zero for methods without the filter, and excludes certified overrides, the budget
     projection and the limiter, so it never acts as a hidden stop penalty. Such a penalty would break
     L2's constraint-only design and double-count stops for L1.
   - If the current definition matches, record it and change nothing. If not, fix it before any
     learned run, as a rule-7 change. **If any L1 run already exists, stop and report first.**
4. **B2 (RPP)** is implemented and tuned under the equal-budget rule, with the route cap and the
   classical recovery.
   - **B7 = B2's tuned parameters plus the PFB, nothing else.**
   - **Declared in the preregistration before tuning:**
     - the alpha grid {1, 2, 5, 10, 20} 1/s;
     - selection: among settings within 5 pp of the best success, the fewest protective stops per
       100 m, ties broken by mission time;
     - the tune set at the training unmapped rate.
   - Alpha is tuned with B7, then frozen for L3, L4, L5 and B7.
- **Recorded:** the "ideal tracker ~0.12 m/s" limitation is WITHDRAWN (an artefact of the oracle
  schedule's yaw cap). The withdrawal is beside the original line in `docs/realism_audit.md` and
  `docs/writeup_notes.md`, with the acceptance run's figures: median 0.37 m/s on the 48 routes
  reached, 2.7 s/m against 8.4.
- **Housekeeping:** the A26 relock is committed (4996482), and `paper/` is committed on its own
  (6965c96), so it no longer sets the dirty flag.
- **Order:**
  1. filter-off bit identity on the simulator;
  2. the simulator acceptance step on the final code;
  3. the intervention-term check;
  4. B2;
  5. B7;
  6. alpha declaration and tuning;
  7. L5's pilot against G4.

**2026-10-24 - Ruling 3: the current intervention term, reported. It does NOT match.**
- **Current** (`pfbnav/envs/reward.py::intervention_magnitude`, called from `NavEnv.step`): the
  reward term is `-k_intervention ||(u_request - u_applied) / ceilings||`, with k_intervention 0.1.
- It differs from the ruling in three ways:
  - **(a)** it compares the request with the APPLIED command, so it includes certified overrides
    (a stop zeroes the command: a hidden stop penalty), the wheel-budget projection and the limiter;
  - **(b)** it is the norm, not the squared norm;
  - **(c)** it is non-zero for methods without the filter (L1, L2, L4's unfiltered parts).
- Each axis is normalised by its ceiling, which makes m/s and rad/s commensurable. The ruling writes
  ||u_request - u_PFB||^2 without saying how the axes are weighted.
- **Fix to implement (rule 7), before any learned run:**
  intervention = ||(u_request - u_PFB) / ceilings||^2 with the filter on, and exactly 0 with it off.
  u_PFB is the PFB's output, before the certified layer, the projection and the limiter.
- It touches `reward.py` and `nav_env.py`, both in the environment's closure, so it belongs in the
  same re-freeze as step 4 (re-freeze 10, whose bit-identity test has not run yet).
- **Waiting on:** whether any L1 run exists (ruling 3's stop condition). None exists in this
  checkout's `runs/`; the training host is to be confirmed by the researcher.

**2026-10-24 - Rule 7 entry: the intervention term made the PFB's alone (ruling 3); in re-freeze 10.**
- **No L1 run exists** (confirmed by the researcher, 2026-10-24), so ruling 3's stop condition does
  not apply.
- **Now:** `reward.intervention_cost(u_request, u_PFB, ceilings)` = ||(u_request - u_PFB) /
  ceilings||^2, with u_PFB the PFB's output before the certified layer, the projection and the
  limiter. It is exactly 0 when the filter is off (`NavEnv` passes None). `intervention_magnitude`
  is removed.
- `k_intervention` is unchanged at 0.1. The term is now squared, so a full-ceiling change on one
  axis still costs 0.1 and smaller changes cost less.
- **Tests** (`tests/test_reward.py`):
  - exactly 0 without the filter;
  - 0 when the PFB passes the command;
  - 1 for a full ceiling on one axis;
  - the squared norm on each axis's own ceiling (0.0625 / 0.04 / 0.1025 known answers).
- **Re-freeze 10, re-pinned:** be0a5e16, which includes this change. For filter-off methods this
  changes the REWARD (0 instead of request vs applied), but not the step loop, which the bit-identity
  test guards. With no L1 run in existence, nothing is invalidated.
- **A consequence, for the supervisor:** under the certified latch the PFB passes only releasable
  commands and zeroes the rest (Q7). That zeroing is part of u_PFB, so an L5 policy that keeps asking
  for motion during a stop is charged intervention on every latched step.
  - It is the ruled definition taken literally. It is not the certified override, but it is driven
    by the stop.
  - If it should not count, the term would exclude latched steps (u_PFB := u_request while the latch
    holds).

**2026-10-25 - Re-freeze 10 re-pinned (bd3f8878) for documentation only, before its bit-identity run.**
- **Found while drafting the manuscript.** Several docstrings and comments no longer described the
  code, and four of them sit in the environment's import closure.
- **Corrected** (text only; no code changed, and the scenario identity is unchanged):
  - `nav_env.py`: "the policy is charged on what it got" now describes the ruled intervention term
    (the PFB's change only, nothing without the filter; ruling 3 of 2026-10-24).
  - `observation.py`: the "0.40 / 0.50" asymmetric ceilings; the box has been square since A20.
  - `robot.py::limit_acceleration`: `filter.a_brake`, a removed key, now `certified.a_brake`. The
    same correction is made in two `configs/base.yaml` comments.
  - `pfb.py`'s header: the QP paragraph now describes the implementation (reachable-point
    linearisation, chord rows, the two envelope cases, the re-linearisation, the `brake` label).
- **Not corrected, reported:**
  - **`certified.py`'s `CertifiedLayer` docstring** still describes field 0 at rest as "the
    footprint grown by `margin` all round". `certified.py` is in the scenario identity's closure
    (A18), so editing it moves the preregistration lock. It waits for the next amendment that
    touches the file.
  - **`configs/methods.yaml`** still says "the non-square actuation box".
  - **The nominal wheel-budget share** (`nominal_fraction` 0.80) is described in the config and in
    `robot.py` as applied to nominal commands before the filter. Nothing in the environment or the
    controllers applies it; only the 0.95 share is enforced, by the base (and by the PFB's QP). This
    is a question for the supervisor: implement it, or correct the descriptions.

**2026-10-26 - REVIEWED: the filter-off bit identity (re-freeze 10) and the PFB simulator acceptance
step (PFB_SIM_ACCEPTANCE_20261025T225341Z_seed0).**
- **Bit identity: PASS** (`test_filter_off_is_bit_identical_with_and_without_the_pfb_slot`). Re-freeze
  10 (bd3f8878) stands.
- **Simulator acceptance: STOP AND REPORT.** Commit a7df60d; seed 0; scene 537f1bbb; display off.
  `dirty` is set only by an untracked `This PC - Shortcut.lnk` in the repository root.
  - **Unattributed stops: 3.** The criterion is 0.
  - Contact steps 0. Braking-rollout contacts 0 from 203 fallback states. Scan-fault stops 0.
  - 47 routes reached the goal; 13 stalled on unmapped obstacles.
  - 23 stops: 9 prediction (all heading, 8 also motion), 11 infeasible, 3 unattributed.
- **The unattributed stops:**
  - tune_narrow 2000007, step 506, s 7.46 m. This is the same route and arc position as the
    emulator's first-run unattributed stop (a shelf corner while turning). Here the scan is the
    simulator's own filter stream, so that case is not fully explained by the emulator's one-ray
    scan.
  - tune_wide 2000043, steps 692 and 738, both at s 8.97 m, on a stalled route beside an unmapped
    obstacle. Neither was triggered at rest, so neither is a creep stop.
- **Over-budget steps: 7,811 of 40,982.** 7,733 of them are on the 13 stalled routes, where the PFB
  averages 66-115 ms per step (every regime tried). Routes that reached the goal had 78.
  A concern for G2; recorded here, to be addressed with the PFB's stall-time cost.
- **The run kept per-route summaries only, so the stops cannot be diagnosed from it.**
  `scripts/pfb_sim_acceptance.py` now writes a trace around every stop (20 steps before, 5 after):
  the full records, including the scan the PFB filtered on, its velocity estimate, the applied
  command and the PFB's own state before each call, so each decision can be re-solved offline.
  - `--seeds` selects routes for a TRACE run (a diagnosis, not the acceptance step).
  - Tests: `tests/test_pfb_acceptance.py` (the record's new fields; the trace window).
  - The declared criteria are unchanged.
- **Next:** a traced rerun of 2000007 and 2000043 (one scene load, about 10 min). A reload is not
  bit-identical, so a stop may move or vanish; any stop it records will be traced.

**2026-10-26 - The unattributed simulator stop traced; a prediction-horizon gap; proposal to the
supervisor (no change made).**
- **The traced rerun** (PFB_SIM_ACCEPTANCE_TRACE_20261026T124853Z_seed0, commit 51b1f49):
  - tune_narrow 2000007 stopped unattributed again, at step 509, s 7.46 m;
  - tune_wide 2000043 did not stop (a reload is not bit-identical).
  - The trace re-solves the PFB's decision at step 508 exactly from its recorded inputs.
- **Cause.** The PFB predicts each return over tau = dt + lag (0.10 s) at the commanded rate:
  2.3 mm forward and -0.049 rad. The robot realised 1.0 mm and -0.024 rad, which is 93 % of the
  commanded translation and 97 % of the turn over dt.
  - For a return behind-right (-1.99 rad, 0.38 m) that the turn sweeps, the over-predicted motion is
    non-conservative: predicted g +6.8 mm; the layer measured r - rho = -0.0 mm.
  - The previous scan moved by the realised motion gives +0.07 mm.
  - The lag allowance added motion where Q2 meant it to add margin.
- **Options, on the traced step** (margin under the realised motion, in the layer's field):
  - **Current:** +0.07 mm.
  - **A. Rows at both ends of tau in {0, dt + lag}:** +16 mm (it brakes and eases the turn).
  - **B. Horizon dt only:** +3.9 mm. B assumes the full command is realised; G1b measured 72-97 %
    under slip.
- **Emulator cost of A on 4 tune routes:** mission time +0-5 %, PFB compute +35-40 %, a few more
  fallbacks, stops unchanged. The emulator realises 95 % with no lag, so A's no-motion end rarely
  binds there.
- **Recommended: A.** It changes Q2's horizon and is a rule 7 change, so it waits for the
  supervisor's ruling.

**2026-10-26 - B2 (holonomic Regulated Pure Pursuit) implemented; its tuning waits for the
equal-tuning-budget rule to be defined.**
- **The controller:** `pfbnav/control/regulated_pure_pursuit.py`. Nav2's Regulated Pure Pursuit for a
  mecanum base, on the typed classical input only.
  - **Adaptive lookahead:** clip(\|v_hat\| t_L, L_min, L_max) along the route ahead.
  - **Regulation:** by curvature (the route's turn over the lookahead arc, below min_radius), by
    proximity (the smallest gap between a return and the footprint along its bearing, below
    proximity_distance; an invalid bin is clearance 0, hard rule 5) and by the goal approach. The
    least factor applies, floored at min_speed; the approach is floored at min_approach_speed.
  - **Steering:** it translates toward the lookahead point and yaws to face it, the law B1 uses.
- **Parameters:** `baselines.regulated_pure_pursuit`. These are INITIAL values (Nav2's defaults
  where Nav2 has one), all VERIFY until tuned.
- **Evaluation:** `scripts/evaluate.py --controller regulated_pure_pursuit`, under the declared route
  cap and the classical recovery exactly as B1.
- **Tests:**
  - `tests/test_regulated_pure_pursuit.py`: known answers for the lookahead, curvature, proximity,
    unobserved-bin, approach and floor cases, parameter refusal, and the protocol under the cap and
    the recovery;
  - `tests/test_runner.py`: construction through the evaluation script.
- **Open, for the supervisor: the equal-tuning-budget rule is named but not defined** (A16,
  `configs/methods.yaml`). Nothing fixes its size (configurations or episodes per method), the search
  (grid or random, over which ranges) or the selection rule for the classical baselines. B1 is
  untuned too. The rule must be declared before B2 (or any baseline) is tuned.
  - **Proposal:** the same budget for B1, B2, B3 and B6, namely N configurations each, drawn by a
    seeded random search over declared ranges. Each is evaluated once on the 60 tune routes at the
    training rate, in the simulator. Selection by the rule ruled for alpha: among settings within
    5 pp of the best success, the fewest protective stops per 100 m, ties by mission time.
  - N sets the cost: about 45 min of simulator time per configuration on this laptop, so N = 16 is
    about 12 h per baseline.

### RULED 2026-10-26 (supervisor): the simulator acceptance STOP AND REPORT. Recorded before implementation.
- **PFB fix (rule 7; amends Q2's horizon):** each barrier row must hold over the realised fraction
  lambda in [0, 1] of the predicted motion over dt + lag, covering any slip.
  - Endpoints alone are insufficient unless proven. Either prove that the per-row minimum over lambda
    lies at an endpoint, with a randomised test against a dense grid, or use the declared grid
    lambda in {0, 0.25, 0.5, 0.75, 1} with a test bounding the gap to the dense minimum.
  - The simulator acceptance step is rerun on the final code, criteria unchanged.
  - The two untraced stops on wide 2000043 must be explained (from ring-buffer dumps if they were in
    force); the rerun must have the ring buffer.
  - Any unattributed stop means stop and report.
- **1. Approved:** reduce the PFB's stalled-route cost before G2, with exact, equivalence-tested
  changes only, implemented together with the interval check. Report the per-step mean and p95 for
  open and stalled routes.
- **2. Nominal wheel-budget share:** correct the descriptions, no code change. The QP optimises
  directly in the 0.95 polytope, and one share for all methods is fairer. Confirm and record which
  share G1b, the oracle and the acceptance runs actually used.
- **3. Exclude latched steps from the intervention term.** The latch is the certified layer's, and
  penalising there would give PFB methods a hidden stop signal that L1 and L2 lack.
- **4. A14a now, before any tuning:**
  - the statistical analysis: the rate-ratio estimand, the two-level bootstrap with K/(K-1) and t
    correction, Welch-Satterthwaite or flat K-1 as ruled, intersection-union logic, Holm at alpha/3,
    the decision tree;
  - the alpha grid {1, 2, 5, 10, 20} 1/s with its selection rule.
  - **A14b later:** the seed count and test size, from the tree after Step 0 and G2.
- **5. Do not drop the rotation row by association.** Report the yaw rates the PFB plus ideal tracker
  used at bends, and in which bands, and rewrite the realism-audit row on that evidence.
- **L1's pilot may start now on the training host,** provided the intervention term is first
  confirmed identically zero with the filter off, latched steps included. If it is not, fix it
  first. The filter-off bit-identity test on the next re-freeze must then show L1 unaffected.

**2026-10-26 - The 2026-10-26 rulings implemented (rule 7 entries); a defect in the PFB's field found
and corrected.**
- **The rows over the realised fraction of the motion** (ruling; `pfb.py::PFB._rows`):
  - Every barrier row is the minimum over lambda in `filter.motion_fractions` {0, 0.25, 0.5, 0.75, 1}
    of the row at p'(lambda) = R(-lambda w tau)(p - lambda tau v). The gradient is the minimising
    fraction's. The field's support is evaluated once for all the fractions (it depends on u, not
    lambda).
  - **The grid's gap to the dense minimum, measured:** at most 0.454 mm over 24,390 rows of random
    scans with commands up to the top band and 1.5 rad/s, against 201 fractions.
  - **`filter.motion_grid_margin_m` 0.0005** (rounded up) is subtracted from every row, and from
    h_now so that holding still keeps its row. The grid's minimum is then a lower bound of the
    minimum over all lambda.
  - **Tests** (`tests/test_pfb.py`):
    - the bound holds on random scans, with a positive control (without the margin the grid
      overstates rows);
    - a known case from the traced stop's geometry: full horizon +5.8 mm (clear), over every
      fraction -1.6 mm (violated);
    - the vectorised rows equal a plain per-fraction reference to 1e-12;
    - the finite-difference gradients still hold.
  - **On the traced step** (2026-10-26, tune_narrow 2000007, step 508) the re-solved PFB now brakes
    and eases the turn (-0.37 rad/s) instead of driving into the breach.
- **A defect, found by the reference test and corrected:** `PFBField.support_and_partials`, the path
  the QP's rows use, took the turning drift K from the sweep's E(|w|) + E(0). `turning_drift` and the
  certified layer's `coupling` take E(|w|).
  - The PFB's own field was up to 6 mm LARGER than the certified field on moving fields: conservative,
    but not the field Q2 ruled.
  - Only `support` had been cross-checked. `tests/test_pfb_field.py` now cross-checks the partials
    path against the certified field (1e-12) at every band, heading and yaw rate, and its partials
    against finite differences.
  - The acceptance runs so far used the larger field.
- **The PFB's cost (ruling 1), exact changes only:**
  - the rows vectorised;
  - `PFBField.support_and_partials` cached by its exact arguments (64 fields, read-only arrays);
  - the rows memoised within one `filter` call.
  - Equivalence tests: cached against fresh fields (identical arrays); the filter with and without
    the memo (identical command, mode, regime and solves on 25 scans that exercise the QP).
  - The per-step mean and p95 for open and stalled routes are measured by
    `scratchpad bench_exact.py` on identical inputs (entry below).
- **Ruling 3: latched steps excluded from the intervention term.**
  - `reward.pfb_intervention(request, pfb_report, ceilings)` is 0 with no report (filter off,
    latched or not) and 0 in mode "latched"; otherwise `intervention_cost`. `NavEnv` uses it.
  - Tests in `tests/test_reward.py`.
  - **L1's condition confirmed:** with the filter off the step loop records no PFB report, so the
    term is identically 0, latched steps included. This held before this change too.
- **Ruling 2: the nominal share, descriptions corrected** (`configs/base.yaml`,
  `robot.py::WheelBudget`, `YouBotBase.command_body_velocity`; also `step_loop.py`'s intervention
  sentence). No code change.
  - **Recorded:** every executed command uses the 0.95 share. That covers G1b (through the base),
    every simulator run, the emulator (`recovery_control.py`), both acceptance steps, the PFB's QP,
    and the certified fields' speed cap, reach and strata.
  - The 0.80 share is used only OFFLINE, by the free-orientation oracle's schedule
    (`scripts/quantify_oracle_free.py`) and the classical route cap (`pfbnav/eval/route_profiles.py`),
    to plan their speeds. **For the supervisor:** the route cap therefore plans the classical
    baselines' speeds on the 0.80 share, while every method executes in 0.95.
- **The ring buffer in the simulator acceptance step** (`scripts/pfb_sim_acceptance.py`; required for
  the rerun). The ring's row now carries `info["pfb"]` (`pfbnav/eval/ring.py`).
- **Ruling 5's evidence:** `pfb_acceptance.py::yaw_profile` / `yaw_summary`. Both acceptance scripts
  report, pooled, the applied \|yaw rate\| by certified field in force (rest, 0, 1, 2) and by bend
  (the route turning >= 0.5 rad within the next 0.8 m) or straight. Known-answer tests are in
  `tests/test_pfb_acceptance.py`. The realism-audit row is rewritten from the rerun's figures.
- **Ruling 4: A14a drafted** (`docs/pending/A14a_statistics_alpha.md`,
  `docs/pending/endpoints_A14a.yaml`); the researcher applies it and relocks.
  - **Flagged in it:** the preregistered interval is the t interval on the closed-form two-level
    bootstrap variance of the linearised log rate ratio, as implemented and calibrated, not a
    percentile resampling bootstrap.
  - "Holm at alpha/3" is written as Holm's step-down at familywise 0.05, with power planned at the
    most stringent step, alpha/3.

**2026-10-26 - The PFB's per-step cost after the exact changes (ruling 1), measured on identical
inputs.**
- **Method** (scratchpad `bench_exact.py`):
  - every filter call on a route is recorded once (emulator, sector scan, ideal tracker), then
    replayed through three variants of the same filter on the same inputs, in one process;
  - variants: no caches; the field and sweep caches; the caches plus the row memo;
  - **decisions are identical across the variants** (command, mode, regime) on both routes.
- **Exact changes:**
  - the rows vectorised;
  - the field's support cached by its exact arguments;
  - the footprint's rotation sweep cached by its exact half-angle. It depends on the yaw rate alone,
    so it repeats across headings, bands and envelope cases where the whole field does not. This
    is the change that matters.
  - the rows memoised within a `filter` call;
  - the QP's cost matrix and settings built once.

| route | none: mean / p95 | all exact changes: mean / p95 |
|---|---|---|
| open (tune_narrow 2000000, 400 steps) | 21.5 / 37.7 ms | 18.7 / 35.9 ms |
| stalled (tune_medium 2000023, 500 steps) | 113.4 / 196.1 ms | 57.0 / 101.6 ms |

- **Against the code before the motion-fraction rows:** 14 ms open and 74 ms stalled (2026-10-24,
  different inputs).
- The rows over five fractions cost more per row. The sweep cache more than recovers it where it
  mattered: stalled steps fell from 74 to 57 ms mean.
- These are laptop figures. G2 measures the training host.

**2026-10-26 - The two untraced stops on tune_wide 2000043: what can and cannot be said.**
- The ring buffer was NOT in force in PFB_SIM_ACCEPTANCE_20261025T225341Z. The acceptance script did
  not use the runner, and the run kept per-route summaries only. The traced rerun did not reproduce
  them (a reload is not bit-identical).
- **They cannot be explained from data.** Two things are recorded:
  - both were triggered while moving, not at rest, on a stalled route, beside the unmapped
    obstacle, at the same arc position (s 8.97 m) 46 steps apart;
  - the one stop that was traced, on 2000007, was the prediction-horizon gap now closed.
  - That is a plausible common mechanism (slow motion with a turn beside an obstacle), and is
    recorded as a hypothesis, not as an explanation.
- **The rerun of the step has both:** the ring buffer, whose row now carries the PFB's decision, and
  a re-solvable trace around every protective stop. Any unattributed stop means stop and report.

**2026-10-26 - B2's tuning runner (prepared; budget awaits the equal-tuning-budget ruling).**
- `scripts/tune_classical.py` runs a seeded random search; `pfbnav/eval/tuning.py` holds the pure
  routines; the space is `configs/tuning/b2_regulated_pure_pursuit.yaml`.
  - Candidate 0 is the configured initial values.
  - The other candidates are drawn uniformly or log-uniformly from ranges that bracket them.
  - A combination the controller refuses is re-drawn, and the refusals are counted.
- **Evaluation:** each candidate is evaluated exactly as `scripts/evaluate.py` evaluates B2, using
  `make_controller`, with the route cap and the classical recovery.
  - It covers every scenario of the three tune sets, each once.
  - All candidates run in ONE scene load, because determinism is claimed per load.
  - It keeps a ring buffer for each candidate, and the episodes are written as they finish.
- **Refuses:** test sets (hard rule 2), an environment that is not the frozen one, `filter.enabled`,
  and an unknown controller.
- **Selection (PROPOSED):** the rule ruled for alpha on 2026-10-24.
  - Among candidates within 5 pp of the best success rate, take the fewest stops per 100 m.
  - Ties go by mission time, then by index.
  - This needs the supervisor's confirmation for the baselines.
- **Open:**
  - `--budget` is required and never defaulted. Its size is the equal-tuning-budget rule, which is
    not ruled.
  - The ranges are VERIFY.
- **Nothing is written back automatically.** The selected parameters go into
  `baselines.regulated_pure_pursuit` by hand after review, with an entry here.
- **Tests:** `tests/test_tuning.py`, 16 known-answer tests, covering the selection window and
  tie-breaks, log-uniform draws, redraws, scoring, the B2 space bracketing the configured values,
  and the runner's refusals.

**2026-10-26 - Ruled (supervisor, on the 2026-10-26 report): acceptance, ring buffer, re-freeze 11,
tuning budget, A14a interval, G2, order.** Implemented the same day.
- **Acceptance.** Both runs on 81c1d70 PASS:
  - simulator: PFB_SIM_ACCEPTANCE_20261026T171339Z, 5 stops, 0 unattributed, 0 contacts;
  - emulator: PFB_ACCEPTANCE_20261026T190701Z, 29 stops, 0 unattributed, 0 contacts.
  - The emulator's fallback rise from 243 to 317 steps is reported in the paper as part of the
    filter's cost.
  - Route 2000043 stays a recorded hypothesis.
- **LAPSE recorded.** The 2026-10-25 simulator acceptance (PFB_SIM_ACCEPTANCE_20261025T225341Z) ran
  without the ring buffer ruled on 2026-10-23, which is why its 2000043 stops cannot be explained.
  - **Fix (ruled): the ring is on by default in every evaluation and acceptance run.**
    `run_episodes` takes `ring` as a REQUIRED keyword and refuses anything but a StepRing before the
    first reset; `run_controllers` takes one ring per controller; the simulator acceptance's
    `run_route` requires one.
  - A ring writes `ring/ring.json` when created, so a run directory shows the ring was in force even
    without a dump (the 10-26 run's empty ring/ could not show that).
  - Tests (`tests/test_ring.py`): a run without a ring is refused; the marker is written; every
    script call passes a ring (an AST scan of scripts/, with a positive control).
  - The emulator acceptance (`scripts/pfb_acceptance.py`) has no simulator step loop to ring: every
    emulator route is deterministic and re-solvable, and its traces carry the inputs.
- **Re-freeze 11: the filter-off bit-identity test PASSED, run explicitly**
  (`tests/test_step_loop_sim.py::test_filter_off_is_bit_identical_with_and_without_the_pfb_slot`,
  1 passed in 14.61 s, researcher, 2026-10-26).
  - **The G4 controls carry over, with no rerun.** They run filter off, and every change since they
    ran is filter-on only (the PFB rows, the field partials, the intervention term, which is zero
    filter off). The bit-identity test shows the filter-off path unchanged.
- **Tuning (ruled).**
  - **Budget:** N = 24 candidates per classical baseline, candidate 0 (the defaults) included; the
    same N for B1, B2, B3 and B6; B7 reuses B2's tuned parameters.
  - **Selection:** the alpha rule (within 5 pp of the best success rate, fewest stops per 100 m,
    ties by mission time).
  - **Ranges:** approved. RPP's proximity speed-scaling gain was absent, so Nav2's
    `cost_scaling_gain` is added as `proximity_gain` (`baselines.regulated_pure_pursuit`, initial
    1.0, range 0.25-1.0, <= 1 as in Nav2; 1.0 reproduces the previous behaviour exactly). Curvature
    scaling (min_radius, min_speed) matches Nav2's already. VERIFY is removed from the space.
  - **Boundaries:** a winner on a range boundary (within 1 % of the range width, in log for a log
    range) is reported in summary.json, never extended.
  - `configs/tuning/budget.yaml` holds N and the window. The runner refuses a space or budget file
    that is uncommitted, modified or still says VERIFY, and an instance count G2 has not approved
    on this host (`--g2 summary.json --instances k`).
  - The methods (`configs/methods.yaml`, the paper) record that the learned arms share SAC's
    hyperparameters and that the equal-budget rule governs the classical baselines.
- **A14a interval (ruled):** the closed-form t interval on the log rate ratio, scenario and seed
  variance combined with K/(K-1), the ruled df; the bootstrap was verification only.
  - `rate_ratio.confirmatory_interval` is the one implementation: flat K-1 against a classical arm,
    Welch-Satterthwaite between learned arms.
  - `tests/test_a14a_interval.py` shows it is bit-equal to `rate_ratio.py` at the calibration commit
    0329941 (loaded from git; CORRECTED_20261003T181634Z, classical_df flat), with a positive
    control (WS would differ) and a known answer written from endpoints.yaml's text.
- **G2 (ruled):** run on the training host at the start of L1's pilot (seed 9000000); tuning runs
  only at an instance count G2 approves.
  - `scripts/gate_g2_throughput.py` is implemented (it was a stub) with `pfbnav/eval/g2.py`.
  - It runs N workers for N in 1..4: random actions, then SAC with an update every step, 300 s
    each.
  - `learning_starts` is lowered to `batch_size` for the measurement only, because at the pilot's
    2,000 the window would contain no updates; the update count is recorded as the positive control.
  - The per-step budget is set for each N (N / 25 s).
  - Throughput is split by rendered and non-rendered episodes.
  - The verdict per N: aggregate >= 25 steps/s, rendered fraction <= 0.10, over-budget share <= the
    ruled maximum, scan-fault share <= 0.1 %. The approved count is the largest N that passes, and
    the summary gives the training-matrix projection.
  - **OPEN, blocks G2:** the numeric reading of "essentially no over-budget steps". Proposed 0.1 %;
    `configs/gates/g2.yaml` holds `max_over_budget_share: null`, and G2 refuses to run until it is
    ruled.
- **Order (ruled):**
  1. the bit-identity test (done);
  2. G2 with L1's pilot;
  3. commit B2's ranges and tune B2;
  4. B1, B3 and B6 with the same N;
  5. B7;
  6. alpha.

**2026-10-26 - Ruled (supervisor): the path to primary results.**
- **Principle.** The project must produce a defensible primary result whether or not RL succeeds.
  - Evidence comes first from the filter on a classical controller (B7), which carries no learning
    risk, while learning starts in parallel.
  - Only L5's pilot waits, because it needs alpha.
- **Stage 1 (this week):**
  1. the explicit filter-off bit-identity test for re-freeze 11 (DONE, recorded above);
  2. L1's pilot (seed 9000000) on the training host, with G2 at its start; all tuning runs only at
     an instance count G2 approves;
  3. commit B2's ranges, tune B2 (N = 24, the ruled rule), then tune alpha with B7 = B2's tuned
     parameters plus the PFB;
  4. once alpha is selected, the B2-versus-B7 comparison on the tune set, per stratum: stops per
     100 m, success rate, mission time, pfb_infeasible and fallback counts.
     - It is **labelled tune-set evidence for monitoring and framing only**, never cited as
       confirmatory, and alters no preregistered analysis.
  5. In parallel, code only with no compute: B3 (holonomic DWA), B6 (field-aware MPPI) and L2
     (SAC-Lagrangian), with tests and L2's toy positive control. All are needed before Stage 3.
- **Stage 2 (target end of week 8):** L5's pilot against G4 once alpha is frozen.
  - **Declared:** if L1's or L5's pilot fails G4 twice, stop and report; the study pivots to the
    filter-only path rather than debugging RL under deadline pressure.
- **Stage 3, confirmatory training, in priority order:**
  1. L5, L1, L4 (L3 reuses L1's checkpoints);
  2. L2;
  3. B1, B3 and B6 tuned (N = 24) in parallel with training; they are not on L5's path;
  4. exploratory arms last.
  - If compute runs short, cut in this order: L5-nowp, the PFB-(a) ablation, the stop_active
    ablation, A1's seeds. L1, L4, L5 and B6 are never cut.
- **Stage 4:** one test evaluation. Every controller runs once on the frozen test set, with every
  controller for a scenario within one scene load. No arm is evaluated on the test set early.
- **Contingency hypothesis H1prime**, preregistered before B2's tuning:
  - B7 triggers fewer protective stops per 100 m than B1, B2 and B6, under the A14a analysis, with
    success rate and mission time reported beside it.
  - It is the confirmatory family only if the Stage 2 pivot triggers. Otherwise it is exploratory
    and the original family stands.
  - Amend and relock.
- **Milestones** (targets, not commitments), reported against in every weekly report:
  - week 7: G2 measured, L1 pilot judged, B2, B7 and alpha tuned;
  - week 8: L5 pilot judged; B3, B6 and L2 implemented;
  - weeks 9-14: confirmatory training, with classical tuning in parallel;
  - weeks 15-17: test evaluation.
- **Freeze discipline:** from here, environment and certified-layer changes are permitted only for
  blocking defects. Everything else goes to `docs/requirements.md`.

**Implemented the same day:**
- **A27 drafted** (`docs/pending/A27_contingency_h1prime.md`, `docs/pending/endpoints_A27.yaml`):
  - H1prime: intersection-union over B1, B2 and B6, population and scan-fault rule as H1.
  - The pivot is declared in `confirmatory_family.pivot`: under it the family is H1prime alone at
    alpha 0.05.
  - H1prime is listed as exploratory otherwise.
  - **Flagged in A27:** every H1prime arm runs once, so its interval has the scenario part only (t on
    n - 1 df). This classical-pair case was NOT in the 2026-10-03 calibration grid; it must pass the
    same calibration gate before H1prime is tested. `rate_ratio.confirmatory_interval` still refuses
    two classical arms until then.
  - H1prime's test size goes with A14b.
- **`scripts/tune_classical.py` refuses to start until A27 is in the preregistration**
  (`contingency_problem`).
- **`scripts/tune_alpha.py`:**
  - B7 = B2's tuned parameters (the config must equal the B2 run's selection) plus the PFB.
  - The grid is read from `endpoints.yaml`; all alphas run in one scene load, the PFB rebuilt per
    alpha and the swap checked.
  - The A14a selection rule applies, with a grid-edge selection reported.
  - It then writes `b2_vs_b7_tune_monitoring.json`, per stratum and pooled, labelled as above. B2's
    episodes come from its own tuning run, another scene load, and the label says so.
- **The runner** now counts each episode's PFB decision modes (`pfb_modes`) in eval code, giving the
  fallback count without touching the frozen environment.

## 2026-10-26 - L2 (SAC-Lagrangian) implemented, code only; toy positive control passes
- **Design** (`pfbnav/control/sac_lagrangian.py`, as ruled 2026-10-03): SB3 SAC subclass, every
  `training:` hyperparameter shared. Cost per step = `info["certified"]["stop_triggered"]` (0/1, the
  existing signal; raises if absent) -- no environment change, the freeze does not move. A separate
  cost critic Q_c (one ContinuousCritic, same architecture/optimiser/lr/tau/gamma), TD target
  c + gamma (1 - done) Q_c'(s', a') with the reward target's a', no entropy term. Actor loss
  alpha log pi - min Q + lambda Q_c (unnormalised). Multiplier by projected dual ascent once per
  finished episode on its measured stop count: lambda <- max(0, lambda + eta (C_ep - d)); episodes
  ending before `learning_starts` are not counted. With lambda held at 0 it is plain SAC bit for bit
  (tested; the cost critic is built under a forked RNG).
- **Configs:** `configs/experiments/L2_sac_lagrangian.yaml` (cost_limit OPEN; reward k_stop 0 and
  k_proximity 0 as ruled) and one child per d in {0.05, 0.1, 0.2}. `multiplier_lr` 0.05 and
  `multiplier_init` 0.0 are NOT ruled (VERIFY).
- `scripts/train.py` builds SAC-Lagrangian when a `lagrangian:` block is present and refuses an
  unruled one; summary.json records the dual history. `SACController` loads L2 checkpoints.
- **Toy control** (`tests/test_sac_lagrangian.py`, 300 steps, d = 1 stop per 4-step episode): seed 0
  in the suite; seeds 0-3 measured: unconstrained 3.05-3.75 stops/episode, SAC-Lagrangian 1.0-1.1.
- **Open for the supervisor:** eta and lambda_0; G2 measures plain SAC, and L2's update carries one
  more critic; methods.yaml says learned arms get no per-arm tuning budget, while d is chosen on
  the tune set from three candidates (ruled 2026-10-03).

**2026-10-26 - B3, holonomic DWA, implemented (code only; the ruling of 2026-10-26: B3, B6 and L2 are
built in parallel with Stage 1, before confirmatory training).** Nothing was run in the simulator, and
no evaluation or tuning was run.
- **Files.**
  - `pfbnav/control/dwa.py`: `DWAParams` and `HolonomicDWA`, name `B3_holonomic_dwa`. It takes only
    `ClassicalInput`.
  - `configs/base.yaml`: the new block `baselines.dwa`. Every value is VERIFY.
  - `configs/tuning/b3_dwa.yaml`: the proposed tuning space. It is still VERIFY, so the runner refuses
    it until the supervisor approves the ranges.
  - `scripts/evaluate.py`: the controller `dwa`. Its profile is `cap`, so B3 runs under RouteCapped,
    under the recovery, exactly as B1 and B2 do.
  - `scripts/tune_classical.py`: a `TUNABLE` entry for `dwa`. Its validity hook reads candidates the
    way the controller reads its block, so whole sample counts survive being carried as floats.
  - `pfbnav/control/route_cap.py`: `RouteCapped.episode_report` now passes on the base controller's own
    report when the base has one. This is how B3's blocked-step count reaches the episode record. B1
    and B2 have no report of their own, so their records are unchanged. The control modules are
    outside the frozen environment's import closure, so the freeze does not move.
- **Design** (Fox, Burgard and Thrun 1997, in Nav2 DWB's holonomic form):
  1. **The window** is centred on the velocity ESTIMATE.
     - Translation: a disc of radius a_max x window_time. It is a disc because the limiter bounds the
       magnitude of the change.
     - Yaw: alpha_max x window_time either side of the estimated yaw rate.
     - The window is intersected with the actuation box, `max_speed` and the enforced wheel budget
       (filter_fraction x v_rim, the one share every method gets).
     - Deceleration beyond a_max is not sampled. When no sample is admissible, B3 commands zero, which
       the limiter brakes at a_decel_max. The step is counted as blocked (`dwa_blocked_steps`; hard
       rule 9).
  2. **Rollout:** a constant twist from the current pose, in closed form. States are spaced at DWB's
     granularities (0.025 m, 0.1 rad), so a slow window rolls out fewer states.
  3. **Rejection** (DWB): a sample is rejected if any state within the horizon brings a scan point
     nearer the footprint than its group's threshold, min(safety_margin, the group's least clearance
     now).
     - The two groups are the valid returns and the unobserved points.
     - "Least clearance now" means that a robot already inside the margin is not frozen: it may move
       away, but may not come nearer.
     - Fox's admissibility (able to brake before the obstacle) follows from requiring sim_time >=
       max(max_speed / 2 a_decel_max, w_max / 2 alpha_max) = 0.536 s, which the controller enforces.
     - **An invalid bin is an occupied ray** from the footprint's edge outward, sampled every
       sim_granularity (hard rule 5). This deliberately differs from B6's declared convention, a point
       at range_min (0.05 m). That point lies inside the footprint and would forbid every motion
       whenever any bin is invalid.
  4. **Scoring:** an unnormalised weighted sum, as in DWB, of four terms:
     - the end state's distance to the route;
     - minus the route arc it reaches;
     - the end yaw's error to the route tangent there (face where it goes, as B1 and B2 turn);
     - the trajectory's clearance shortfall below clearance_cap.

     The route is cropped to 2 x max_speed x sim_time of arc, so a returning leg cannot pull the
     projection forward.
  5. **The footprint in the rollout** is the measured table's e(psi), taken as its maximum over 0.05
     deg cells.
     - This over-states the robot by at most one cell's variation, about 0.1 mm: the safe direction.
     - It replaces the spline because of compute (next bullet).
- **Compute, measured on this laptop only (not a property of the training host):**
  - 2 to 34 ms per step on synthetic scans. The worst case is an aisle with both walls inside 0.5 m,
    at 0.5 m/s.
  - The first draft took 180 to 330 ms per step in the aisle: 7 x 7 x 9 samples, fixed 0.05 s states,
    and the spline footprint. 24 tuning candidates would then have cost days. Hence three changes:
    - DWB's granularities instead of fixed states;
    - 5 x 5 x 7 samples (0.0375 m/s and 0.07 rad/s apart in a 0.075 m/s, 0.21 rad/s window);
    - the lookup instead of the spline.
  - Points that no state can reach are pruned. A test shows the pruning changes no decision.
- **Initial values** (`baselines.dwa`, all VERIFY):
  - max_speed 0.5 m/s;
  - window_time 0.15 s. It is above the 0.05 s period so that the estimate's lag (filter_tau 0.10 +
    one step) does not hold acceleration below a_max; it equals the certified reach time;
  - sim_time 1.5 s (DWB 1.7);
  - sim_granularity 0.025 m and angular_sim_granularity 0.1 rad (DWB);
  - samples 5 / 5 / 7;
  - safety_margin 0.05 m;
  - clearance_cap 0.3 m;
  - weights: w_path 1.0, w_progress 1.0, w_heading 0.3, w_clearance 1.0.
- **Proposed tuning space** (`configs/tuning/b3_dwa.yaml`, awaiting approval). Eight parameters:
  - max_speed 0.20 to 0.50;
  - window_time 0.10 to 0.40 (log);
  - sim_time 0.8 to 3.0 (log);
  - safety_margin 0 to 0.10;
  - clearance_cap 0.10 to 0.60;
  - w_path 0.2 to 5 (log);
  - w_heading 0.05 to 2 (log);
  - w_clearance 0.2 to 5 (log).

  Not tuned:
  - w_progress, the unit of the weights, because scaling every weight changes no argmin;
  - the granularities and the sample counts, which are resolution.
- **Tests** (hard rule 6):
  - `tests/test_dwa.py`, 18 tests:
    - the window: the disc, box, max_speed and wheel budget;
    - the state counts;
    - the closed-form rollout against RK4 integration;
    - route projection and crop known answers;
    - the cell table never under-states the footprint;
    - each scoring term's known answer;
    - straight-route choices from rest and at speed;
    - a wall that admits exactly vx = 0.175;
    - the blocked case and its counts;
    - the invalid-bin ray: nothing with a forward component survives;
    - inside the margin: away is admitted, toward is rejected;
    - pruning is exact;
    - refusals;
    - the configured B3 under the cap and the recovery, with its report reaching the record.
  - `tests/test_runner.py`: `dwa` is constructed as Recovery(RouteCapped(HolonomicDWA)).
  - `tests/test_tuning.py`: the space brackets the initial values; 24 sampled candidates all build the
    controller; the runner refuses the file while it says VERIFY.
- **Open, for the supervisor:**
  - (a) Approve the B3 ranges, or amend them.
  - (b) Choose the invalid-bin convention: keep B3's occupied ray, or align B6 with it.
  - (c) A known limitation. Inside the margin, motion ALONG a wall is admitted only where the scan's
    3 deg sampling allows: the wall point passed over may be a fraction of a millimetre nearer than
    the nearest sampled one now. Escaping diagonally is always admitted.
  - (d) Whether `dwa_blocked_steps` should appear in the evaluation summary, beyond the episode
    records.


### 2026-10-27: B6 (field-aware MPPI) implemented, code only; FOUR findings and the specification gaps for the supervisor

Implemented as code-only work in parallel with Stage 1 (ruling of 2026-10-26). Nothing was run in the
simulator; no evaluation and no tuning were run.

**Design** (`pfbnav/control/mppi.py`, `FieldAwareMPPI`, name `B6_field_aware_mppi`), the declared
structure of `baselines.mppi` (A16):
- **Input:** only the `ClassicalInput` (route ahead, 120-bin filter stream, control period). Field
  geometry from `pfbnav/safety/certified.py` (`FieldGeometry.support`), read-only. It never imports
  `pfb.py` (hard rule 3).
- **Model:** kinematic holonomic, as declared. Each command is held for one `sim.dt` at the mid-step
  heading, clamped to the box and projected into the filter-share wheel budget exactly as the base
  (tested equal to `clamp_velocity_with_status` + `project_to_wheel_budget`).
- **Sampling:** the nominal plus Gaussian noise; sample 0 is the nominal. The update averages the
  feasible sequences, so the nominal stays feasible. Warm start by a one-step shift.
- **Cost:**
  - route tracking (squared distance to the route, summed over the states);
  - goal progress (minus the arc of the last state's projection; the route is clipped at twice the
    horizon's travel, so a route winding back cannot fake progress);
  - field breach (the number of (state, point) pairs inside the certified field of each state, sized
    from that state's command: band from the reachable speed, A20; travel heading; yaw sweep; the
    at-rest field below both standstill ENTER thresholds);
  - smoothness (squared command change, within the rollout, the first against the last issued
    command);
  - hard rejection: any scan point inside the footprint at any state -> weight 0.
- **Weights:** exp(-(S - min S) / temperature), with S the rollout's cost divided by the horizon
  ("on per-step costs").
- **Invalid bins:** an invalid bin is a point at `scanners.range_min` on its bearing (hard rule 5). It
  lies inside the footprint, so every rollout is rejected. When every rollout is rejected the command
  is zero, the nominal is cleared and the step is counted (`mppi_all_rejected_steps`).
- **Seeding:** from the EPISODE: `begin_episode` reseeds from `[0xB6, scenario_seed]`, so a
  scenario's noise does not depend on run order (tested bit-identical).
- **The field table.** An exact field at every (rollout, state, point) is out of reach, so the band
  fields are tabulated once per process:
  - bearing cells uniform in the diamond angle (no arctangent), each the maximum of 3 samples;
  - bilinear interpolation over the travel heading (5 deg grid) and |yaw rate| (0.1 rad/s knots);
  - rest rows per yaw knot;
  - 360 support normals (at most +0.3 mm against 1440).
  - **Measured against `FieldGeometry.select`** (pinned in `tests/test_mppi.py`): moving -1.8 to
    +8.8 mm, abeam within 5.1 mm; at rest -0.2 to +4.3 mm.
  - Two variants were measured and rejected:
    - the larger of the two bracketing headings: up to +3 cm, +2 cm abeam, where the narrowest
      aisle leaves about 21 mm between the top band's field and the wall;
    - the yaw rate rounded up: up to +7.7 cm.
  - The extent is NOT convex in the yaw rate, so no interpolation in it is one-sided.

**Wired in:**
- `scripts/evaluate.py`: controller `field_aware_mppi`, under the declared route cap and the classical
  recovery (`PROFILE_KIND` "cap").
- `pfbnav/control/route_cap.py`: `RouteCapped` now passes `begin_episode` and the base's
  `episode_report` through, so the scenario seed reaches B6. No change for B1 and B2, which have
  neither hook (`tests/test_route_profile.py`).
- `scripts/tune_classical.py`: `TUNABLE["field_aware_mppi"]`. The block is nested and holds
  integers and declared strings, so the runner now addresses numeric leaves by dotted name
  (`pfbnav/eval/tuning.py`, `flatten_numeric`, `assign_flat`). Integer leaves stay integers, and
  integer ranges draw and round (`type: int`). For B2's flat block the behaviour is unchanged.
- `configs/tuning/b6_mppi.yaml`: PROPOSED ranges, VERIFY (the runner refuses the file until they are
  approved). `iterations` is left at 1 (not named as tuned in A16; each pass costs a full step).

**Tests:**
- `tests/test_mppi.py`, 17 tests:
  - the box and budget against the base's functions;
  - the diamond angle;
  - the extent formula against the layer's;
  - table accuracy against the layer;
  - band and rest rows;
  - parameter refusals;
  - rollout and cost-term known answers;
  - a breaching rollout penalised by the layer's own count (8 pairs; precondition: every point
    > 10 mm from the boundary, which caught a wall-based version with a point 2.8 mm away);
  - footprint rejection;
  - an invalid bin rejects everything;
  - determinism and order independence;
  - a straight open route goes straight;
  - the finding below pinned;
  - the interface;
  - a per-step timing print.
- `tests/test_tuning.py`: +6 tests (flatten/assign, nested `with_block`, integer draws, the B6 space
  brackets the configured values and samples valid candidates, the space is refused while VERIFY).
- `tests/test_runner.py`: B6's construction.
- `tests/test_route_profile.py`: +1 test (the pass-through).

**FINDINGS (for the supervisor; nothing in `configs/base.yaml` was changed):**
1. **The declared initial values do not drive.**
   - The per-step cost spread across samples is about 1e-3 to 1e-2. Against temperature 1.0 the
     weights are near uniform (effective sample size about 1024 of 1024).
   - Smoothness on i.i.d. noise (sigma_wz 0.5) outweighs goal progress about 25 to 1 in the spread.
   - On an open, straight route the command stays near zero.
   - In a 0.9 m aisle every noisy rollout's yaw sweeps its field into the walls (about 280 breach
     pairs per rollout), so only the zero nominal is clean, and the robot never moves.
   - Candidate 0 of the tuning (the configured values) would therefore spend 1/24 of the budget on a
     controller that does not move.
   - Values that drive straight at about 0.37 m/s in the open: noise (0.1, 0.1, 0.1),
     smoothness 0.01, temperature 0.01, goal_progress 10 (used by the tests). In an aisle the
     command is still erratic at these values: a tuning matter.
   - **Asked:** re-rule the initial values, or keep them as candidate 0.
   - Pinned by `test_the_configured_initial_values_do_not_drive`, which must change when they do.
2. **Per-step time is far over the budget** (N / 25 s on the training host, 40 ms at N = 1).
   - This laptop, 1024 x 30: 15-45 ms in the open, 280-445 ms in a 0.7-0.9 m aisle (about 100 scan
     points within the 1.6 m reach).
   - The cost is about samples x horizon x points; the table lookups dominate.
   - The table costs about 15 s once per process.
   - The simulator is stepped synchronously, so this changes no outcome, but tuning time scales with
     it.
   - **Asked:** a sample count or horizon chosen with the step budget in mind (the proposed space
     spans 128-1024 samples and 15-40 steps), or a time limit in the tuning rule. No figure here is a
     property of the training host.
3. **Field-table accuracy** is a few mm either way (above), not one-sided. B6 may occasionally predict
   no breach where the layer stops it (at most 1.8 mm inside), or penalise up to 8.8 mm early.
4. **An invalid bin stops B6** (every rollout rejected, the command zero) for as long as it lasts.
   This is consistent with the layer, for which an invalid bin breaches every field. B6 commanding
   zero never asks the recovery for anything.

**Specification gaps (reported; no ruling invented):**
- the acceleration limiter (a_max, alpha_max, a_decel_max) is not in the declared kinematic model;
  B6's rollouts can reach any speed in one step, while the robot cannot;
- no heading term is declared; yaw is driven only by the field and smoothness costs;
- "smoothness: squared change of command between consecutive steps" could mean within the rollout
  (implemented) or between control steps;
- "temperature ... on per-step costs" is read as the rollout cost divided by the horizon;
- the fallback when every rollout is rejected is not declared (implemented: zero, counted);
- the field is sized from the rollout's own command (kinematic: velocity = command); the layer takes
  the larger of the estimate and the applied command, which the rollout cannot see beyond step 0;
- B6's noise is seeded from the scenario alone, not from the run's `--seed`;
- `iterations` is not named among the tuned quantities; it is left out of the space.

## 2026-10-27 - Rulings on the questions of 2026-10-27: implemented (A28 drafted)

**The rulings** (supervisor, 2026-10-27, verbatim in substance):
1. G2: `max_over_budget_share` = 0.001; scan faults reported beside it per instance count.
2. B6 candidate 0 = noise 0.1/0.1/0.1, smoothness 0.01, temperature 0.01, progress 10, disclosed as
   coming from a manual check (record the routes used), not counted in N. Ranges re-centred on it;
   temperature on a log scale that includes 0.01.
3. The real-time criterion for every controller: p95 compute per step <= 50 ms (the control period)
   on the training host. Samples, horizon and iterations are removed from B6's tuning space and fixed
   by a declared measurement on the training host: the largest pair from samples {256, 512, 1024} x
   horizon {15, 20, 30} with p95 <= 50 ms in 0.7-0.9 m aisles. The PFB is measured under the same
   criterion; a stalled p95 above 50 ms on the training host is a finding and a limitation. Per-step
   compute of every controller is reported in the results.
4. B6:
   - rollouts respect the limiter (a_max, alpha_max, a_decel_max) and the wheel budget;
   - no heading term (recorded);
   - smoothness within the rollout and temperature on cost / horizon: confirmed;
   - all rollouts rejected -> zero command, reported as mppi_all_rejected;
   - rollout fields use the certified composition, max(estimate, rollout command), with reach;
   - scenario-seeded noise confirmed, its seed dependence recorded;
   - the lookup table is made conservative (never smaller than the certified field), with a test.
5. B3 ranges approved; B3 candidate 0 = the standard DWA defaults with their source cited. B6 ranges
   approved after items 2-4. Both committed before the first candidate; a boundary winner is
   reported, never extended.
6. One shared invalid-bin convention: an invalid bin is an obstacle at the footprint boundary along
   its bearing, one shared function for B1, B2, B3, B6 and the PFB; invalid-bin events counted per
   method.
7. B3's wall-following limitation accepted and recorded. `dwa_blocked_steps` goes in the evaluation
   summary per method and stratum, beside pfb_infeasible and mppi_all_rejected.
8. L2:
   - lambda_0 = 0, eta = 3e-4, lambda updated per gradient step from the cost critic's batch
     estimate, projected to >= 0 and capped at lambda_max = 100;
   - declared, not tuned; validated on the toy positive control;
   - throughput measured in its pilot; more than 20 % below plain SAC -> report and re-plan;
   - cost limit d = 0.1 stops per episode, fixed a priori, superseding the 2026-10-03 tuning ruling;
   - L2 at d in {0.05, 0.2} is an exploratory sensitivity, first in the cut order;
   - the methods record that d was set a priori.

Then the declared path: G2 with L1's pilot, B2 tuning, B7 and alpha.

**Implemented** (the changes, the reasons, the tests):

- **G2** (`configs/gates/g2.yaml`):
  - `max_over_budget_share: 0.001`. G2's summary already reports the scan-fault share beside the
    over-budget share for each N (`pfbnav/eval/g2.py`).
  - A28 writes the number into `gates.yaml`. From then on, `scripts/gate_g2_throughput.py` refuses
    if the overlay and the preregistration disagree (hard rule 16; `tests/test_g2.py`).
- **The real-time criterion:**
  - **Per-step compute in every evaluation** (`pfbnav/eval/compute.py`, `pfbnav/eval/runner.py`).
    The runner times each `controller.act`. It also records the PFB's own time per step, from the
    step loop's timing of its slot (new info key `pfb_seconds`), and their sum, the "decision".
  - These are kept per episode as sparse 0.1 ms histograms (`compute_hist`), so a run's pooled p95
    is exact to one bin. A quantile is read at its bin's upper edge, so it is never below the
    measured value.
  - `outcome_summary` reports p50, p95 and max for each part, per method and stratum. Known-answer
    tests are in `tests/test_measure_realtime.py` and `tests/test_runner.py`.
  - **The declared measurement** (`scripts/measure_realtime.py`, run by the researcher on the
    training host) drives the robot kinematically, with no simulator, through synthetic straight
    aisles 0.7, 0.8 and 0.9 m wide. Each step applies the box, the budget and the limiter as the
    base does, with the estimator's filter and a ray-cast 120-bin scan. It has three parts:
    - **mppi**: B6 at candidate 0 for all 9 pairs, 200 timed steps per aisle. The rule selects a
      pair; ties in samples x horizon go to the longer horizon (my reading, now declared in A28).
    - **pfb**: a constant 0.4 m/s request toward a box across the aisle 1 m ahead. A step is
      "stalled" when the applied speed and yaw rate are at or below the layer's standstill ENTER
      thresholds. The PFB creeps toward the box and settles about 0.36 m short; 400 steps per aisle.
    - **classical**: B1, B2 and B3 at their configured values.
  - The measurement refuses to run until `gates.yaml` has `realtime_compute` (A28).
  - Tests cover the selection rule, the ray cast, the kinematic aisle and a shortened PFB part that
    moves and then stalls.
  - **B6 is not tunable until measured.** `baselines.mppi.compute_measurement` (null) must name the
    measurement's run directory; `scripts/tune_classical.py` refuses B6 until it does
    (`measurement_problem`). Every tuning run also waits for A28 (`realtime_problem`).
- **B6** (`pfbnav/control/mppi.py`, `configs/base.yaml`, `configs/tuning/b6_mppi.yaml`):
  - **Candidate 0** as ruled. The manual check is DISCLOSED in the config and the methods. It used
    two synthetic routes only, from `tests/test_mppi.py` and no scenario of any set:
    - a straight open route along +x with nothing in the scan;
    - a straight 0.9 m aisle (walls at y = +-0.45).
  - The space drops samples, horizon_steps and iterations. Every range is re-centred on candidate 0
    (geometric centres within 6 %): temperature 0.001-0.1 on a log scale. VERIFY is removed.
  - **The limiter in the rollouts.** Box, budget, then `limit_acceleration`'s rule (a_max speeding
    up, a_decel_max slowing down, alpha_max on yaw), vectorised and tested equal to the base's.
  - **Where the rollouts start.** The rollouts start from B6's model of the command the base is
    executing: the limiter's step from the command before toward the command last issued. After a
    step with `stop_active`, the layer replaced the command by zero, so the start is the step toward
    zero instead. The route cap and the recovery may also change what reaches the base; B6 cannot
    see that, and the base's own limiter governs.
  - **Found and fixed while implementing.** The first version averaged the LIMITED sequences. Since
    a_decel_max dt = 0.05 exceeds a_max dt = 0.025, the limiter turns zero-mean noise into a downward
    drift, and averaging fed that drift back into the plan every step. Candidate 0 then settled near
    0.08 m/s on an open route; smaller noise made it slower still (0.03 m/s at sigma 0.05).
    - The update now averages the COMMAND sequences, MPPI's control-sequence update (Williams et
      al. 2017), with the limiter in the dynamics. Box and budget are convex, so the average stays
      feasible.
    - Candidate 0 now reaches about 0.37 m/s within 2 s (40 steps), in the open and in a 0.9 m aisle
      (`test_the_configured_candidate_0_drives`, `test_a_straight_open_route_goes_straight`).
  - **The certified composition.** Each rollout state's field is sized from max(|estimate|, |command
    executing|), with the yaw rate the larger of the two and the heading from whichever dominates.
    The estimate along the rollout is predicted by the estimator's first-order filter from the input
    estimate. Reach (A20) is in the band choice, as before.
  - **The conservative table.** Every entry carries `TABLE_PAD_M` = 3 mm, 1.5 x the worst undershoot
    of the unpadded interpolant against `FieldGeometry.select`. That undershoot was -2.0 mm over
    5,400 random states x 2,880 bearings, moving and at rest, the same on two seeds.
    - The test sweeps 460 states x 2,880 bearings and asserts the table is never below the exact
      field.
    - The over-statement is at most 13.5 mm, and 9 mm abeam, where the narrowest aisle leaves about
      21 mm.
  - `mppi_all_rejected` (renamed from `mppi_all_rejected_steps`) and `mppi_held_steps` are now in
    the episode report. The seed dependence is recorded in the module, the config and the methods.
  - **Measured on this laptop (not the training host).** 1024 x 30 takes 250 ms per step in a 0.9 m
    aisle and 31 ms in the open. B6 will need a much smaller pair to meet 50 ms.
- **B3** (`configs/base.yaml`, `configs/tuning/b3_dwa.yaml`):
  - Ranges approved; VERIFY removed.
  - **Candidate 0** = Nav2 DWB's defaults: `nav2_bringup/params/nav2_params.yaml`, ROS 2 Humble,
    read 2026-10-27. Each value is converted on its own line:
    - sim_time 1.7;
    - safety_margin 0 (BaseObstacle rejects only a footprint in collision);
    - clearance_cap 0.33 (inflation_radius 0.55 - robot_radius 0.22);
    - w_path 1.333 (PathDist 32 / GoalDist 24);
    - w_heading 0.133 (PathAlign 32 per cell of a point 0.1 m ahead = 64 per rad, / 480 per m);
    - max_speed = the box ceiling (DWB's 0.26 is the TurtleBot's own limit).
  - Sample counts and granularities are resolution, not tuned. DWB's 20/5/20 samples would not meet
    the real-time criterion.
  - **Two defaults fall below the approved ranges.** They sit at the floors until ruled:
    - window_time: DWB uses the 0.05 s control period;
    - w_clearance: BaseObstacle's 0.02 x the inflation cost's secant slope, about 0.020.
- **One invalid-bin convention** (`pfbnav/safety/unobserved.py`, `resolve_unobserved`). Each consumer
  passes its own footprint representation, so the point lies exactly on the boundary it tests
  contact against:
  - **B1 / recovery:** the recovery's clearance (it was range_min).
  - **B2:** the clearance is 0 with any invalid bin, as before.
  - **B3:** one point at the rollout footprint's edge instead of the whole ray. Moving away is
    admitted, moving toward is not.
  - **B6:** the point is on B6's cell-max footprint (the larger of the cell and its two neighbours).
    Standing still is no longer rejected, so B6 can back away; before, it froze.
  - **The PFB:** the point enters the rows. It lies inside every regime's field and cannot leave it
    in one step, so the step is infeasible and falls back to maximum braking. This is the same
    command as before, and its reason stays "unobserved". `PFBResult.unobserved` counts the bins.
    `releasable` (the latch test) still blocks on an unobserved bin: it predicts the certified
    layer's restart condition, and the layer keeps its own rule (an invalid bin breaches every
    field; `certified.py` is unchanged).
  - Invalid-bin events: the environment already counts `invalid_bin_steps` per episode for every
    method; it is now in the summary per method and stratum.
- **Statuses in the summary** (`pfbnav/eval/runner.py`, `STATUS_COUNTS`): pfb_infeasible,
  dwa_blocked_steps, mppi_all_rejected and invalid_bin_steps, per method and stratum (0 where a
  method has no such status). B3's wall-following limitation is accepted (recorded in `dwa.py`).
- **L2** (`pfbnav/control/sac_lagrangian.py`, `configs/experiments/L2_*.yaml`, `scripts/train.py`):
  - The dual step runs per gradient step:
    lambda <- min(100, max(0, lambda + 3e-4 (mean_batch Q_c(s, a~pi) - d_Q))).
    The actor's loss on that step uses lambda from before the update.
  - **d_Q, the limit in the cost critic's units, is MY reading, reported for a ruling.** It is
    d (1 - gamma^T) / ((1 - gamma) T) with T = `reward.max_steps` = 4,000: the rule of the Safety
    Starter Agents' SAC-Lagrangian (Ray, Achiam and Amodei 2019). That gives 0.0025 at d = 0.1. It
    is stricter than d per episode when episodes end before the cap. The measured cost of each
    finished episode is still recorded beside lambda (`dual_history`), so compliance with d itself
    is read directly.
  - **The configs.**
    - `L2_sac_lagrangian.yaml` IS L2 (d = 0.1) and declares the multiplier settings.
    - `_d005` and `_d020` are L2-d005 and L2-d020, new IDs in `configs/methods.yaml`, exploratory.
    - `_d010` is removed; no run used it.
    - `LagrangianSettings` refuses any other d or multiplier setting.
  - **The toy positive control at the ruled settings** (`pfbnav/training/l2_toy.py`, the toy moved
    out of the test so the researcher's script can use it). The toy's reward is scaled by 0.05 so a
    multiplier moving at eta 3e-4 can price a stop against it.
    - At 1,000 steps (the fast suite's limit) lambda is still rising: seeds 0-3, last 20 episodes:
      L2 1.65-2.2 stops per episode against plain SAC's 2.55-2.65, with d = 1.
    - So the suite asserts the DIRECTION: lambda rises, and L2 stops at most 0.9 x as often as plain
      SAC.
    - The limit itself is judged by `scripts/validate_l2_toy.py` (5,000 steps per seed, criteria
      declared in the script). The researcher runs it, since it is RL training beyond 1,000 steps.
  - **Throughput** in L2's pilot: compare `summary.json` steps per second against L1's pilot; more
    than 20 % slower -> report and re-plan.
- **Re-freeze 12** (`env.frozen_code_sha256` d7cd5e3a...). `unobserved.py` enters the closure; pfb.py,
  step_loop.py and nav_env.py change (the PFB's unobserved bins as rows; `pfb_seconds` and
  `pfb["unobserved"]` in the info). Filter-off behaviour is unchanged (the PFB is not built). The
  filter-off bit-identity test reruns on the simulator.
- **A28 drafted** (`docs/pending/A28_rulings_20261027.md`, `endpoints_A28.yaml` built on A27's draft,
  `gates_A28.yaml`):
  - the G2 share;
  - `realtime_compute`;
  - the `declared` block (L2's d and multiplier, the invalid-bin convention, B3's and B6's candidate
    0, B6's compute and seed);
  - `per_method_reporting`;
  - the L2 sensitivity under `exploratory`;
  - the cut order [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1].

**Open, for the supervisor:**
- (a) L2's d_Q conversion (above): the episode cap T = 4,000, or the mean episode length.
- (b) B3's candidate 0 has two values below their approved ranges (window_time DWB 0.05 s vs floor
  0.10; w_clearance DWB-mapped 0.020 vs floor 0.2). They are at the floors for now. Either keep them
  there or extend those two ranges down.
- (c) The cut order. The frozen `reduction_order` [A1, L2] listed L2 itself as cuttable; A28 drops
  it, as the plan of record trains L2 in Stage 3.
- (d) B6's update now averages commands rather than the limited sequences (the fix above): confirm.
- (e) The tie rule in B6's compute selection (the longer horizon).

## 2026-10-27 - Rulings on A28's five points: implemented (A28a drafted; A28 already applied)

**The rulings** (supervisor, 2026-10-27):
1. **L2:** update lambda from the empirical mean protective stops per episode over the last 20
   completed episodes, compared directly with d = 0.1; the cost critic still drives the policy
   gradient. eta = 3e-4 per gradient step, lambda_0 = 0, lambda >= 0, lambda_max = 100. This
   supersedes "from the cost critic's batch estimate". **Reason:** units consistency with the
   declared d. No (1 - gamma^T) conversion.
2. **B3:** extend the two floors to include candidate 0: window time 0.05-0.40 s, clearance weight
   0.02-5 (log). This corrects the declaration before any run; the never-extend rule applies only
   after tuning.
3. **Cut order** confirmed: [L2-d005, L2-d020, L5-nowp, PFB-(a), stop_active, A1]. Never cut: L1, L2,
   L4, L5 and every classical comparator, B1, B2, B3, B6, B7.
4. **B6:** standard MPPI confirmed (weight-average the sampled commands; the limiter inside the
   rollouts). Record the change and why the earlier form crawled.
5. **Compute ties:** prefer the longer horizon, confirmed. Declared outcome: if even 256 x 15 exceeds
   50 ms p95 on the training host, stop and report; the criterion is not relaxed.

Then finalise A28, relock, and proceed with G2 and L1's pilot. A28 had already been applied and
relocked (`4fa4798`, `1a74df7`), so the finalisation is a dated revision, **A28a**
(`docs/pending/A28a_rulings_on_A28.md`, `endpoints_A28a.yaml`, `gates_A28a.yaml`), relocked the
way A14a was.

**Implemented:**
- **L2** (`pfbnav/control/sac_lagrangian.py`, `configs/experiments/L2_sac_lagrangian.yaml`):
  - Each gradient step: lambda <- min(100, max(0, lambda + 3e-4 (mean of the last 20 completed
    episodes' measured stops - d))).
  - The actor's loss on that step uses lambda from before the update; lambda Q_c stays in it.
  - lambda is held until 20 episodes have completed, since a mean over fewer is not the declared
    statistic. Those 20 may include episodes from before `learning_starts`, as "completed episodes"
    says.
  - `cost_window: 20` is declared in the config; `LagrangianSettings` refuses any other value.
    `discounted_budget` and the `budget` field are gone.
  - The window (`recent_costs`) is saved with the checkpoint.
  - **Toy positive control, re-measured** (1,000 steps, seeds 0-3, last 20 episodes): L2 meets the
    limit, 0.45-1.05 stops per episode against d = 1, while plain SAC is at 2.55-2.65. lambda ends at
    0.18-0.20; the deterministic action is -0.55 to -0.43 (plain SAC +0.36 to +0.41).
  - The fast suite therefore asserts the limit itself again (at most 1.25 d and at most half of plain
    SAC on seed 0), plus known answers for the window: held below 20; exactly the last 20 kept.
  - `scripts/validate_l2_toy.py` (5,000 steps per seed, the researcher's run) now checks that the
    limit still holds over a longer run.
- **B3** (`configs/tuning/b3_dwa.yaml`, `configs/base.yaml`): window_time is 0.05-0.40 s (log) and
  w_clearance 0.02-5 (log). Candidate 0 is now DWB's own values, window_time 0.05 and w_clearance
  0.020, no longer the floors (`tests/test_tuning.py`).
- **Cut order:** `never_cut` is added in A28a's `gates.yaml` (`tests/test_methods_registry.py`
  checks that it and the cut order are disjoint).
- **B6:** the update recorded with its reason, in `endpoints.yaml` (`declared.B6_update`), the
  methods registry and `mppi.py`. The finding is in the entry before this one: averaging the limited
  sequences fed the limiter's asymmetry (0.05 m/s down, 0.025 m/s up per step) back into the plan,
  and B6 settled near 0.08 m/s.
- **The compute stop rule:** in A28a's `realtime_compute.B6_rule`. `scripts/measure_realtime.py`'s
  finding now reads "STOP and report (A28a) ... the criterion is not relaxed".

## 2026-10-27 - L2 toy control FAILED under A28a; RULED: PID-Lagrangian (A28b drafted)

**The run** (the researcher's, laptop, commit `171f366`, clean tree; `scripts/validate_l2_toy.py`,
5,000 steps per seed, `runs/validation/L2_TOY_20261027T16*/17*`): FAIL on 2 of 4 seeds against the
declared criteria (last 20 episodes <= 1.25 d at d = 1, <= half of plain SAC's).
- Last 20 episodes, L2: 0.00, **3.20**, 0.60, **1.35** stops per episode. Plain SAC: 3.95-4.00 (the
  constraint binds).
- Cause: a dual-ascent limit cycle, the same on every seed. Its period is ~300-400 episodes; lambda
  moves between 0.01 and 0.27; corr(lambda, stops over the next 100 episodes) is -0.84 to -0.88.
- The time average meets d (0.86-1.06 over episodes 250-1,250), but only 69-77 % of 20-episode
  windows meet 1.25 d. The last iterate passes or fails by phase.
- The fast suite's 1,000-step test had passed on the cycle's first trough (~episode 250). Its
  wording overstated what it shows and is corrected below.

**Ruling** (supervisor, 2026-10-27): option (b), the PID-Lagrangian (Stooke, Achiam and Abbeel, ICML
2020).
- (a), accepting the cycle, is rejected: L2's final checkpoint would depend on its phase, biasing H1.
- (c), a time-averaged criterion, is rejected: it rewrites a declared criterion after a failure, and
  the evaluated policy is the last iterate.

The update, once per completed episode, from the 20-episode moving-average cost c_bar:
lambda = max(0, K_P e + K_I sum(e) + K_D max(0, c_bar_k - c_bar_{k-1})), with e = c_bar - d and
lambda_max = 100. The per-gradient-step update is retired. Three (K_P, K_I, K_D) configurations are
declared before any toy run, in Stooke et al.'s form, scaled so that the integral term alone reaches
the oscillating run's multiplier level (~0.1-0.3) within ~100 episodes; the derivation is recorded.
The rerun is 5,000 steps on seeds 0-3, with the criteria unchanged, and also reports the fraction of
20-episode windows meeting d. The first declared configuration that passes is selected; if none
passes, stop and report. Then amend A28, relock, and add Stooke et al. to references.bib with a
VERIFY note. L1's pilot and G2 are unaffected.

**The derivation** (from the A28a runs' recorded costs and multipliers, seeds 0-3):
- **K_I:** the summed excess sum(c_bar - d) over the first 100 updates was 116, 102, 109 and 88
  (mean ~104). For the integral term alone to reach 0.2 -- the middle of 0.1-0.3, and the A28a
  lambda's 90th percentile of 0.18-0.20 -- after ~100 episodes: K_I = 0.2 / 104 ~ **2e-3**. This
  gives 0.18-0.23 per seed. (For comparison, A28a's eta 3e-4 per gradient step, at 4 steps per toy
  episode, was a pure integral gain of 1.2e-3 per episode.)
- **K_P:** the median |e| was 0.90-0.95. K_P = **0.1** moves lambda by ~0.1 at a typical excess,
  half the level; K_P = **0.2** moves it by the whole level.
- **K_D:** c_bar rises by at most 0.2 per episode (one 4-stop episode replacing a 0), at the 95th
  percentile by 0.1, and on average by 0.017-0.020. K_D = **1.0** adds ~0.1 on a 95th-percentile
  rise, and biases lambda by ~0.02 on average.
- **The candidates, in declared order:** 1 = (0.1, 2e-3, 0), PI; 2 = (0.1, 2e-3, 1.0), PID;
  3 = (0.2, 2e-3, 1.0), PID with the stronger proportional term. They are declared in
  `pfbnav/control/sac_lagrangian.py` (`PID_CANDIDATES`) and in A28b's `endpoints.yaml`
  (`declared.L2_pid_candidates`).
- **Units:** lambda is in reward units per stop, e in stops per episode.
- **Note:** the toy's episodes are 4 steps, and L2's are up to 4,000. The gains act per EPISODE, and a
  1M-step L2 run completes episodes in roughly the toy's count (hundreds to ~1,000), so the toy's
  episode-scale dynamics are the relevant ones. A28a's per-gradient-step eta had no such
  correspondence: per episode it was ~1,000 times faster in L2 than in the toy.

**Implemented:**
- `pfbnav/control/sac_lagrangian.py`:
  - `pid_step` is a pure function, Stooke et al.'s Algorithm 2, capped.
  - `SACLagrangian` takes `pid_gains` and keeps `pid_integral` and `previous_mean`, both saved with
    the checkpoint. It updates lambda in `_end_episode` once the window is full, and the actor uses
    the lambda in force for the whole episode.
  - `dual_step` and `multiplier_lr` are removed. A config that still carries `multiplier_lr` is
    refused as retired.
  - `dual_history` records the multiplier in force during each episode, plus the window mean and
    the integral.
- **The integral is projected onto >= 0, as Stooke et al.'s Algorithm 2 does** (I <- (I + e)+). The
  ruling writes K_I sum(e); without the projection the integral would wind below zero under the
  limit and hold lambda at 0 until it unwound. This is flagged in A28b for the supervisor.
- **The first update has no derivative term** (there is no previous mean). The ruling does not cover
  this case. Stooke et al. initialise the previous cost at 0, which would put a one-off kick of
  K_D c_bar into the first update.
- Configs and selection:
  - The L2 configs carry `pid_candidate: OPEN`. `LagrangianSettings.from_config` refuses OPEN, so
    `scripts/train.py` refuses L2 until the toy has selected.
  - `pfbnav/training/l2_toy.py` adds `toy_cfg(candidate)`.
  - `scripts/validate_l2_toy.py` takes `--candidate` and reports `window_fractions`: the fraction of
    20-episode windows at or below d and 1.25 d, over all windows and over those ending at episode
    250 or later.
  - `scripts/train.py`'s summary records the gains, the candidate and the final integral.
- Tests:
  - `tests/test_sac_lagrangian.py`:
    - `pid_step` known answers (each term alone, both projections, the cap, the first update);
    - the declared candidates;
    - the window and hold on a scripted history;
    - an exact replay of the recorded lambda from the recorded costs;
    - checkpoint round trip of the PID state;
    - OPEN refused.
  - The 1,000-step toy test is renamed and documented as a regression of the measured state
    (candidate 1, seed 0: L2 0.80 stops per episode, plain 2.55, lambda 0.21, action -0.52), not as
    the control.
  - `tests/test_validate_l2_toy.py` (new): the verdict and the window fractions on hand-counted
    synthetic costs.
- Drafts: `docs/pending/A28b_pid_lagrangian.md` and `endpoints_A28b.yaml`, which differs from the
  frozen file only in `declared.L2_multiplier` and the new `declared.L2_pid_candidates`.
  `paper/MDPI_template/references.bib`: `stooke2020pid`, VERIFY.

## 2026-10-27 - L2 toy control under A28b: candidate 1 PASSES on seeds 0-3, SELECTED; the cycle persists (flagged)

**The runs** (the researcher's, laptop, commit `b9c1b42`, the A28b relock, clean tree; 5,000 steps per
seed, `runs/validation/L2_TOY_PID1_20261027T18*`): candidate 1 = (0.1, 2e-3, 0) PASSES on all four
seeds against the unchanged criteria.
- Last 20 episodes, L2: 1.00, 0.05, 0.55, 0.25 stops per episode (<= 1.25 at d = 1). Plain SAC:
  3.95-4.00. Final lambda 0.03-0.07.
- Under the declared rule (the first candidate in declared order that passes) **candidate 1 is
  selected**. Candidates 2 and 3 were not run. `pid_candidate: 1` is set in
  `configs/experiments/L2_sac_lagrangian.yaml`; L2-d005 and L2-d020 inherit it.

**Anomaly, reported and not acted on** (read from the runs' `summary.json`; per-episode costs and
multipliers, compared with the A28a runs `L2_TOY_20261027T16*/17*` by the same code):
- **The oscillation is not removed.** On every seed the 20-episode mean still spans 0 to 4 after
  episode 250, and windows above 2 stops per episode (plain SAC's level) still occur:

  | episodes (window end) | 250-500 | 500-750 | 750-1,000 | 1,000-1,231 |
  |---|---|---|---|---|
  | A28a, share of windows > 2 (seeds 0-3) | 0.00-0.10 | 0.24-0.40 | 0.26-0.29 | 0.01-0.33 |
  | PID 1, share of windows > 2 (seeds 0-3) | 0.00 | 0.13-0.38 | 0.00-0.48 | 0.00-0.26 |

- It is smaller, not gone: windows meeting 1.25 d from episode 250 are 0.78-0.82 (A28a 0.69-0.77);
  all windows 0.70-0.74 (A28a 0.60-0.67). corr(lambda, stops over the next 100 episodes) is -0.28 to
  -0.48 (A28a -0.84 to -0.88). lambda now swings wider, 0 to 0.65-0.98 (A28a 0.01-0.27), and reaches 0
  under the limit (the proportional term at e = -1 cancels an integral of ~50).
- Seed 1 relapsed in the final quarter (26 % of windows > 2) and passed on its phase. The last-20
  verdict therefore still depends partly on phase. At the late per-seed rates of ~0.8, four passes
  in four have a probability of ~0.4 even if each seed's pass were a coin with that bias.
- No damping is visible over the 5,000 steps.
- A possible reading, not tested: the toy's per-episode cost is 0-4 in whole stops, and meeting d = 1
  needs a policy that stops in a quarter of its episodes; a dual method may chatter about such a
  boundary whatever its gains. L2's real episodes have different cost statistics.

**For the supervisor** (no change made): the declared rule selects candidate 1 and is applied. The
ruling's reason for rejecting (a) -- a final checkpoint that depends on the phase -- is reduced, not
removed, on the toy. Running candidates 2 and 3 now to choose among passers would be a selection
outside the declared rule; any such step needs a ruling.

## 2026-10-28 - The repository moved; one test no longer reads git history

The project moved to a new folder and GitHub repository whose history starts at two `init` commits
(the full history stays in the original `pfb-nav` folder). `tests/test_a14a_interval.py` compared the
confirmatory interval with `pfbnav/eval/rate_ratio.py` as of commit `0329941` (the calibration run
that adopted flat K - 1), fetched with `git show`; on the new repository that commit does not exist
and the two tests that use it errored (first run on the training host: 1505 passed, 1 skipped,
2 errors).
- **Change:** the file is stored unchanged as `tests/data/rate_ratio_0329941.py.txt` (extracted with
  `git show 0329941:pfbnav/eval/rate_ratio.py` in the original repository, 13,670 bytes), the test
  reads it, and the test asserts its SHA-256 (`27435685...67268008`) before using it. The comparison,
  its tolerances and its positive control are unchanged. The `.txt` suffix keeps the old module out of
  pytest collection and ruff.
- **Not changed:** the analysis scripts that also read old commits (`scripts/reemulate_b1_traces.py`,
  `scripts/compare_rebuilt_sets.py --against`, `scripts/regress_floor.py --rev`) are not in the fast
  suite and need the original repository's history to reproduce those past analyses.
- **Correction, same day:** the first version hashed the raw bytes, and the training host (a Windows
  checkout, CRLF) got `6cc85484...` for the same text, so both tests errored there. The test now uses
  `pfbnav.utils.hashing.text_sha256` (line endings normalised, the project's rule for identity hashes);
  a CRLF copy of the file was checked to give the pinned `27435685...67268008`.
