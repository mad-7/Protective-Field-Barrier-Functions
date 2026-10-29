## A29 - 2026-10-29 - The certified layer's restart check sized from the executable speed (blocking defect)

**A blocking-defect fix under the freeze discipline** (plan of record, 2026-10-26: environment and
certified-layer changes only for blocking defects). Recorded in `docs/decision_log.md`, 2026-10-29.
No test layout was built, read or used. No method has been trained, tuned or evaluated; no L1 run
exists.

**The defect.** G2's first run on the training host (`G2_20261029T132046Z_seed9000000`, commit
`b6c6128`) crashed at N = 1 in the random phase:
`ValueError: speed 0.5486 m/s exceeds the top band (0.51 m/s)`, raised in
`CertifiedLayer._restart_allowed` (`pfbnav/safety/certified.py`). While the robot is held with its
own field clear, the restart check selects the field of the commanded departure. It took the band
from the speed of the RAW request, which a learned policy may set anywhere in the action box: a
corner is |(0.5, 0.5)| = 0.71 m/s. The base clamps every request to the box and the wheel budget,
so it executes at most `speed_cap` = 0.5074 m/s, which the top band (0.51) covers. The classical
controllers never request beyond the box along their routes, which is why no run before G2 hit it;
every learned run would have.

**The change.** The band of the departure field is selected from `min(|request|, speed_cap)`
instead of `|request|`. The direction still comes from the request (the restart ruling of
2026-09-27). Nothing else in the layer changes: the stop, the field the robot stands in, the latch,
the restart delay, the receding release and both approach rules are as they were.

**Why it is safe.** The base executes at most `speed_cap`, and never more than the request, so the
selected band is never below the band of the executed motion. `CertifiedParams` already validates
that the top band covers `speed_cap` (A20). For requests within the cap the result is identical, so
no prior run, gate or carried-over result changes. The behaviour that changes is the one that
raised.

**Verified** (`tests/test_restart_rigid.py`):
- A dense sweep of requests: every heading in 5 degree steps, speeds 0.40-0.75 m/s in 5 mm/s steps
  (through the cap, the 0.51 top band and the action box's corner), at four yaw rates, plus the
  action box's corners (20,000+ requests). No exception; open space still permits the departure;
  the selected band's top is never below the band of the speed the base would execute.
- Positive control: the same sweep fails on the code before the change (it raises at 0.51 m/s), and
  the pre-change sizing raises on the request that crashed G2.
- An above-cap request toward an intrusion in the departure shell is still blocked.

**Identity.** `certified.py` is in the environment's closure and in the scenario identity's import
closure (A18): the environment is re-frozen (`env.frozen_code_sha256` `d7cd5e3a...` ->
`3757aeca...`), the filter-off bit-identity test reruns on the host, and the lock is relocked under
this amendment. The scenario sets themselves do not change (the generator's behaviour does not
depend on the restart check).

**Not changed, and why.** The main field selection uses max(velocity estimate, applied command). The
applied command is capped; the estimate never exceeded the command in the 198 G1b runs replayed on
2026-10-16 (maximum ratio 0.985). An estimate above 0.51 m/s would still raise, loudly, by design.
G2's random phase, saturated random actions at every instance count, is the first wide test of it.
