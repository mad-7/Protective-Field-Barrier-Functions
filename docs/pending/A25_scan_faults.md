## A25 - 2026-10-23 - Scan faults: the plausibility guard; H1 excludes scan-fault stops; G2's over-budget criterion

**Decided by the supervisor on 2026-10-23**, in two rulings recorded in `docs/decision_log.md`
before any change. No test layout was built, read or used. No method has been trained or evaluated.

**Why.** The zero-action G4 control had one stationary trigger (2026-10-22, seed 2000018). The traced
rerun, declared before running, reproduced it only under induced heavy load: 0/20 normal, 1/20
loaded, and the load control passed.
- The trace shows, with the robot still, two consecutive vision-sensor renders corrupted under CPU
  starvation: 104-114 of 120 bins empty (the walls gone), and phantom returns at 0.30-0.47 m near
  the scanner corners.
- These were fresh packets of the current tick, so the staleness check passed them.
- A corrupted render can create a stop or hide an obstacle.
- **It is blocking** because the fault is load-dependent and the PFB adds per-step compute: unfixed,
  it would bias H1 against L5.

**What changed** (environment code; re-freeze 9 with the filter-off bit-identity test):
1. **The plausibility guard** (`pfbnav/sim/scan_guard.py`). Each bin must lie within the range
   spanned by the last accepted scan's bins within +-2 of it, +-6 cm, both widened per step since
   that scan. 6 or more bins outside means a corrupt packet.
2. **Up to 2 re-renders of the same tick** (`rescan` in `lua/scanner_v2.lua`; the stamp is kept).
3. **The fail-safe:** if still corrupt, every bin is unobserved for that step, counted as a
   `scan_fault`.
   - A stop decided on such a packet is a `scan_fault_stop`.
   - It is kept out of the at-rest and stationary (construction-gap) counters.
   - The ring buffer dumps on every scan fault.
4. **Verification** (the 20 loaded runs WITHDRAWN: at a base rate near 1/20, zero events in 20 would
   occur about 36 % of the time without a fix):
   - a deterministic fault injection replaying the two recorded corrupted packets through the read
     path (both flagged, re-rendered, fail-safed, no stationary trigger);
   - the false-alarm regressions (198 recorded good steps, 3,196 emulated moving steps);
   - the re-freeze 9 bit-identity test;
   - in the loop, L1's pilot reports `scan_faults` and any ring dumps.

**In the preregistration** (copies with only these edits, to apply verbatim):
- **`endpoints.yaml`, H1:** stops decided on a scan-faulted packet are excluded from the primary stop
  count and reported alongside it per method. `scan_faults` are reported prominently if they differ
  materially between methods. File: `docs/pending/endpoints_A25.yaml`.
- **`gates.yaml`, G2:** the selected instance count must produce essentially no over-budget steps,
  with the over-budget count recorded per run. A run with more than 0.1 % of its steps scan-faulted
  is flagged, and that host configuration runs fewer instances. File: `docs/pending/gates_A25.yaml`.
- No generator or scenario code changes: `scenarios_sha256` is unchanged and nothing is rebuilt.
  The preregistration text changes, so the lock is renewed (`--relock A25`).
