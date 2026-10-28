# Reproducibility notes

What a result needs besides a commit hash to be reproduced, and what is known about the
platform's determinism. Kept current; the decision log has the history.

## The scene

The scene carries settings that live outside `configs/` -- which shapes count as
collidable among them -- so every run records the scene's SHA-256 in `provenance.json`,
and `sim.scene_sha256` in `configs/base.yaml` pins the one runs may use. A run refuses a
scene with another hash (`SceneMismatch`), and `tests/test_infrastructure.py` fails if the
committed scene and the pinned hash disagree.

| SHA-256 | commit | what it is |
|---|---|---|
| `2f7a8b8ed0521ab54185f8fd1f127fec30b6bc1c07073483db40b251b2a04705` | `a7c1a63`, restored in `bcbd57d` | **original** scene; wheel spheres' collidable flag off |
| `340dbc5d75972436833d016e025e5294937b48af7ef9ccf69500124a6805dcdd` | `61e4494` | the four `wheel_respondable_*` spheres collidable, nothing else changed |
| `0ae35a50290d097857bb3cd828858f70a736ae0fa2102f0fac1e69b452164790` | `3816e11` | S300-type layout: two 270 deg scanners on diagonal corners; the footprint table was built here |
| `20974276a354e7f12e72c52670a09a8f032e7c93ff3b890e752dbeeffdad2b81` | `d936360` | camera moved back; geometry identical to `0ae35a50` (`docs/scene/equivalence.json`) |
| `537f1bbbb7d674a395107398b46b7b6a765428b2942acefd529bf0c24dc35eb4` | the floor-edit commit, 2026-10-14 | **current**: `/Wall` removed and the floor 7 x 7 tiles (+-17.5 m); the robot untouched (robot-scoped equivalence) |

Two further hashes appear in the record and are **not** usable baselines: `c89392ff...`
(an unreviewed GUI save swept into `ba8199d`) and `d0f1d15c...` (a manual undo that left
`intermediateLink_fl` collidable; never committed). See the decision log, 2026-09-18.

**Standing rule.** The scene is never staged implicitly (no `git add -A` or `.`). An edit
is deliberate, verified by reading the flags back (`tests/test_collection_semantics.py`),
committed alone, and updates `sim.scene_sha256` in the same commit.

## Runs made before the scene hash was recorded

G1 (Bullet), G1-ODE, both G1b runs and the footprint probes up to
`FOOTPRINT_20260918T133012Z` predate scene hashing; all ran on the original scene
(`2f7a8b8e...`), the only one committed at the time.

## Physics

ODE, dynamics step 0.005 s, simulation step 0.05 s, applied by `SimClient.start()` on
every start, overriding the scene file (which still says Bullet). Frozen by D7 (A4).

## Determinism

- **Within one scene load: bit-identical.** Repetitions inside one invocation have matched to
  the last bit in every gate run (G1b: 66 runs; the flag regression: 378 rows x 14 columns).
- **Across processes: not bit-identical.** Two invocations on the same commit, config and
  scene differ from the first logged step (17 um at row 0, up to 4 mm along-track over
  5 m); G1-ODE's three invocations differed within the 0.64 deg solver band. Cause not
  investigated.
- **Across scene loads, even in one process: not bit-identical** (found 2026-10-14,
  `runs/gates/REGRESS_FLOOR_20261013T232522Z`). Loading the SAME scene file a second time in the
  same process moved the settled start by ~6 um (x -0.000112 vs -0.000106 m) and the
  trajectories by up to 1.8 mm along-track over G1b's 5 m (0.6 mm in the traversability probe,
  0.15 mm in G0). Within one load, every start is bit-identical. The spread is small against
  every gate threshold it touches: G1b's 50 mm peak / 20 mm RMS cross-track and 5 deg heading
  (the reload moved them by <= 0.4 mm and 0.05 deg); G0's one-step detection lag and its 10 mm
  negative-case gap (gap moved <= 0.2 mm); the traversability probe's 2 mm line-held and 5 mm
  wall-seen controls (<= 0.6 mm). Cause not investigated (ruled 2026-10-14).
- **The saved scene 537f1bbb against the old scene's loads** (measured 2026-10-14,
  `runs/gates/REGRESS_FLOOR_20261014T214648Z`, leg 2; INCONCLUSIVE as a test, ruled, and listed here
  as a measured property of the file). Run D on the saved file deviated from the in-memory edit
  (C, bit-identical to the old scene's run A) by more than the largest deviation among three
  reloads of the old scene in 15 file-columns, every one a velocity estimate or G1b's reference yaw:
  - traverses (all six), `vx` and `speed_est`: 7.05e-5 m/s, against 6.23e-5 among the reloads;
  - G0 `side_wall_lateral`, `vx`: 6.86e-5 m/s (6.19e-5), and in the step-loop context 4.63e-5 m/s
    (2.09e-5);
  - G1b `translate_while_rotating` 0.40, `yaw_ref` (the settled start yaw carried forward):
    2.10e-5 rad = 0.0012 deg (9.3e-7 rad).

  Positions, cross-track, distances, contacts and row counts all lay inside the reloads' spread
  (e.g. traverse x 7.3e-5 m against 4.33e-4; G1b travelled 7.1e-4 m against 2.45e-3). Two of the
  three reloads were bit-identical to each other, so the comparison rested on two distinct outcomes.
- **Consequence:** determinism claims are scoped to **one scene load**, not one process (ruled
  2026-10-14). Paired comparisons run their members within one scene load (WP5
  evaluation-runner requirement, `docs/requirements.md`). A regression across a scene edit
  applies the edit in memory, within the load, and joins the saved file to it separately.

## Interpreter

Scripts run under Python 3.10.10. The researcher's bare `pytest` resolves to Python
3.13.13, so sim tests are run as `python -m pytest`.
