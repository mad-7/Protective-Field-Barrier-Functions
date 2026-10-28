"""The classical recovery's controls on the simulator (ruled 2026-10-16). The researcher runs these.

**Positive control: the 2026-10-13 corner stop recovers.** Scenario 2000057 of tune_wide, as built
then (`tests/data/corner_stop_20261013.json`), planned at that day's lethal margin 0.03 so that the
route is the one B1 drove. That day B1 stopped at step 126 passing the end of a shelf at 0.263 m
abeam and never restarted. Both arms run in ONE scene load (determinism is per load, ruled
2026-10-14), each for `STEPS`:
  * **bare B1** must reproduce the deadlock -- the control that shows the scene still means what
    it meant: a stop near the recorded place, still latched and blocked at the end, and the
    route barely advanced past it;
  * **B1 with the declared recovery** is judged on RECOVERY'S JOB only (ruled 2026-10-20): at least
    one attempt, every attempt that ended ended `cleared`, none `not_released`. The earlier "route
    advanced 0.5 m past the corner" is dropped: under the whole-field restart (A22) B1 re-stops by
    its own command after each cleared attempt, which is B1's behaviour, not the recovery's.

The stop's location is re-pinned to the MEASURED 0.946 m (ruled 2026-10-20; run
RECOVERY_CORNER_20261020T152314Z under A21 + A22). The 2026-10-13 location, 1.45 m, predates A20's
larger fields.

**Negative control: zero-action never triggers recovery**, over a full run on the same scenario.
(The second negative control -- a stop whose base direction becomes permitted on its own -- needs
a scan that changes under a stopped robot; it is pinned on synthetic scans in
`tests/test_recovery.py`, where the change can be made exactly.)
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from pfbnav.control.pure_pursuit import PurePursuit, ZeroAction
from pfbnav.control.recovery import Recovery
from pfbnav.envs.nav_env import NavEnv

pytestmark = pytest.mark.sim

REPO = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((REPO / "tests" / "data" / "corner_stop_20261013.json")
                     .read_text(encoding="utf-8"))
SETTLE = 10
STEPS = 500                 # 25 s: the stop at ~86, the 3 s trigger, the recovery, and beyond
STOP_AT_M = 0.946           # m, measured 2026-10-20 (A21 + A22)


def _run(env, controller, steps):
    controller.reset()
    _obs, info = env.reset(options={"index": 0})
    first_stop_s, last = None, info
    for _ in range(steps):
        _obs, _r, terminated, truncated, info = env.step(controller.act(info["controller_input"]))
        if first_stop_s is None and info["certified"]["stop_triggered"]:
            first_stop_s = float(info["s_m"])
        last = info
        if terminated or truncated:
            break
    return first_stop_s, last


@pytest.fixture
def env(cfg, sim_client):
    c = copy.deepcopy(cfg)
    c["planner"]["lethal_margin"] = float(FIXTURE["planner_lethal_margin"])
    environment = NavEnv(sim_client, c, [FIXTURE["scenario"]], settle_steps=SETTLE, seed=0)
    try:
        yield environment, c
    finally:
        environment.close()
        sim_client.stop()
        sim_client.start(settle_steps=SETTLE)


def test_the_2026_10_13_corner_stop_recovers(env):
    environment, cfg = env
    bare_stop_s, bare = _run(environment, PurePursuit.from_config(cfg), STEPS)
    rec = Recovery.from_config(PurePursuit.from_config(cfg), cfg)
    rec_stop_s, _wrapped = _run(environment, rec, STEPS)

    # The control: bare B1 reproduces the deadlock near where it happened (s = 0.946 m).
    assert bare_stop_s is not None and abs(bare_stop_s - STOP_AT_M) < 0.25, bare_stop_s
    assert bare["certified"]["stop_active"], "bare B1 did not stay stopped"
    assert bare["certified"]["restart_blocked"] or bare["certified"]["breached"]
    assert float(bare["s_m"]) < bare_stop_s + 0.1

    # Recovery, on its own job: the same stop, then every attempt that ended was cleared and none
    # was refused. (An attempt still in progress when the episode ends has no ending.)
    report = rec.episode_report()
    assert rec_stop_s is not None and abs(rec_stop_s - bare_stop_s) < 0.05
    assert report["recovery_attempts"] >= 1
    ended = [a["ending"] for a in report["recovery_attempt_log"] if a["ending"] is not None]
    assert ended and all(e == "cleared" for e in ended), report
    assert report["recovery_endings"]["not_released"] == 0, report


def test_zero_action_never_triggers_recovery(env):
    environment, cfg = env
    rec = Recovery.from_config(ZeroAction.from_config(cfg), cfg)
    _stop, last = _run(environment, rec, int(cfg["reward"]["max_steps"]))
    assert rec.episode_report()["recovery_attempts"] == 0
    assert rec.gave_up is None
    assert last["outcome"] == "timeout"
