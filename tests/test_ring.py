"""The evaluation ring buffer (ruled 2026-10-23): known answers on a scripted environment."""
import json
from pathlib import Path

import numpy as np
import pytest

from pfbnav.envs.controller_input import ClassicalInput, RouteView, ScanStream
from pfbnav.eval.ring import StepRing, step_row
from pfbnav.eval.runner import run_episodes

BEARINGS = np.linspace(-np.pi, np.pi, 8, endpoint=False)


def _info(t, *, trigger=False, stationary=False, contact=False):
    ranges = np.full(8, 3.0)
    ranges[2] = 0.3 if stationary else 3.0
    valid = np.ones(8, bool)
    valid[5] = False
    pts = np.array([[0.0, 0.0], [1.0, 0.0]])
    inp = ClassicalInput.of(stop_active=False, velocity=np.zeros(3),
                            scan=ScanStream.of(ranges, BEARINGS, valid),
                            route=RouteView.of(pts, pts[1], pts[1], 1.0), dt=0.05)
    return {"certified": {"stop_triggered": trigger or stationary,
                          "triggered_stationary": stationary},
            "in_contact": contact, "step_seconds": 0.01 * t, "over_budget": False,
            "controller_input": inp}


class EventEnv:
    """Episode 0: a stationary trigger at step 5, ends at 8 as a timeout. Episode 1: a stop at 3,
    contact at 6, ends at 9 as an unrecoverable stop."""

    def reset(self, *, options=None):
        self.i, self.t = options["index"], 0
        return np.zeros(3), {}

    def step(self, action):
        self.t += 1
        if self.i == 0:
            info = _info(self.t, stationary=self.t == 5)
            done, outcome = self.t >= 8, "timeout"
        else:
            info = _info(self.t, trigger=self.t == 3, contact=self.t >= 6)
            done, outcome = self.t >= 9, "unrecoverable_stop"
        if done:
            info["episode"] = {"outcome": outcome}
        return np.zeros(3), 0.0, False, done, info


class Zero:
    name = "zero"

    def reset(self):
        pass

    def act(self, obs):
        return np.zeros(3)


def test_the_row_carries_the_scan_and_the_nearest_return():
    row = step_row(4, _info(4, stationary=True))
    assert row["ranges_mm"][2] == 300 and row["invalid_bins"] == [5]
    assert row["nearest"]["bin"] == 2 and row["nearest"]["range_m"] == pytest.approx(0.3)


def test_dumps_on_a_stationary_trigger_contact_and_an_unrecoverable_stop(tmp_path):
    ring = StepRing(4, tmp_path)
    records = run_episodes(EventEnv(), Zero(), [0, 1], ring=ring)
    assert records[0]["ring_dumps"] == ["ring_0_stationary_step5.jsonl"]
    rows = [json.loads(x) for x in (tmp_path / "ring_0_stationary_step5.jsonl").read_text().splitlines()]
    assert [r["step"] for r in rows] == [2, 3, 4, 5]           # the last 4 steps, up to it
    assert records[1]["ring_dumps"] == ["ring_1_contact_step6.jsonl",
                                        "ring_1_unrecoverable_before_last_stop.jsonl",
                                        "ring_1_unrecoverable_end.jsonl"]
    before = [json.loads(x) for x in
              (tmp_path / "ring_1_unrecoverable_before_last_stop.jsonl").read_text().splitlines()]
    end = [json.loads(x) for x in
           (tmp_path / "ring_1_unrecoverable_end.jsonl").read_text().splitlines()]
    assert [r["step"] for r in before] == [1, 2, 3] and [r["step"] for r in end] == [6, 7, 8, 9]


def test_the_ring_changes_nothing_a_run_returns(tmp_path):
    """Passive: rings of different capacity give the same records, the dumps aside."""
    short = run_episodes(EventEnv(), Zero(), [0, 1], ring=StepRing(1, tmp_path / "short"))
    ringed = run_episodes(EventEnv(), Zero(), [0, 1], ring=StepRing(4, tmp_path / "long"))
    for a, b in zip(short, ringed, strict=True):
        a, b = dict(a), dict(b)
        a.pop("ring_dumps")
        b.pop("ring_dumps")
        assert a == b


def test_a_run_without_a_ring_is_refused_before_it_starts(tmp_path):
    """Ruled 2026-10-26: the ring buffer is on in every evaluation and acceptance run."""
    env = EventEnv()
    with pytest.raises(TypeError, match="StepRing"):
        run_episodes(env, Zero(), [0], ring=None)
    with pytest.raises(TypeError):
        run_episodes(env, Zero(), [0])                      # no default
    with pytest.raises(ValueError, match="directory"):
        StepRing(4, None)


def test_a_ring_leaves_its_marker_even_without_a_dump(tmp_path):
    StepRing(7, tmp_path / "ring")
    assert json.loads((tmp_path / "ring" / StepRing.MARKER).read_text()) == {"capacity": 7}


def _ringless_calls(source: str, sim_acceptance: bool) -> list[int]:
    """Lines of `run_episodes` / `run_controllers` (and, in the simulator acceptance, `run_route`)
    calls that pass no `ring=` / `rings=`."""
    import ast
    names = {"run_episodes": "ring", "run_controllers": "rings"}
    out = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        name = f.id if isinstance(f, ast.Name) else getattr(f, "attr", None)
        need = names.get(name) or ("ring" if sim_acceptance and name == "run_route" else None)
        if need and need not in {k.arg for k in node.keywords}:
            out.append(node.lineno)
    return out


def test_the_script_scan_catches_a_ringless_call():
    """Positive control (hard rule 15): the scan below must see a call without a ring."""
    bad = "run_episodes(env, c, [0])\nrunner.run_controllers(env, cs, [0])\nrun_route(e, 0, s)\n"
    assert _ringless_calls(bad, sim_acceptance=True) == [1, 2, 3]
    assert _ringless_calls(bad, sim_acceptance=False) == [1, 2]
    assert _ringless_calls("run_episodes(env, c, [0], ring=r)\n", sim_acceptance=False) == []


def test_every_script_that_runs_episodes_passes_a_ring():
    """The runner refuses a run without a ring; this catches a script that would only find out on
    the training host."""
    root = Path(__file__).resolve().parents[1] / "scripts"
    missing = [f"{path.name}:{line}" for path in sorted(root.glob("*.py"))
               for line in _ringless_calls(path.read_text(encoding="utf-8"),
                                           path.name == "pfb_sim_acceptance.py")]
    assert not missing, missing
