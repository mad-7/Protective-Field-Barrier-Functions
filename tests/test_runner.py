"""Known answers for the evaluation runner (hard rule 6). Synthetic environment and controller."""
import numpy as np
import pytest

from pfbnav.eval.ring import StepRing
from pfbnav.eval.runner import outcome_summary, run_episodes


class ScriptedEnv:
    """Scenario i lasts `lengths[i]` steps and ends with `endings[i]`; reward 1 per step."""

    def __init__(self, lengths, endings, distances, stops, contacts):
        self.lengths, self.endings = lengths, endings
        self.distances, self.stops, self.contacts = distances, stops, contacts
        self.i = self.t = 0
        self.seen = []

    def reset(self, *, options=None):
        self.i, self.t = options["index"], 0
        self.seen.append(self.i)
        return np.zeros(3), {}

    def step(self, action):
        self.t += 1
        done = self.t >= self.lengths[self.i]
        info = {}
        if done:
            info["episode"] = {"outcome": self.endings[self.i], "distance_m": self.distances[self.i],
                               "protective_stops": self.stops[self.i],
                               "contact_steps": self.contacts[self.i]}
        terminated = done and self.endings[self.i] in ("success", "collision")
        return np.zeros(3), 1.0, terminated, done and not terminated, info


class Counter:
    name = "scripted"

    def __init__(self):
        self.resets = 0

    def reset(self):
        self.resets += 1

    def act(self, obs):
        return np.zeros(3)


ENV = {"lengths": [3, 5, 2, 4], "endings": ["success", "timeout", "collision", "success"],
       "distances": [10.0, 2.0, 1.0, 7.0], "stops": [1, 0, 0, 2], "contacts": [0, 0, 1, 1]}


def test_each_index_runs_once_in_order_with_its_own_record(tmp_path):
    env, ctrl = ScriptedEnv(**ENV), Counter()
    got = []
    records = run_episodes(env, ctrl, [2, 0, 3], on_episode=got.append,
                           ring=StepRing(4, tmp_path))
    assert env.seen == [2, 0, 3] and ctrl.resets == 3 and got == records
    assert [r["index"] for r in records] == [2, 0, 3]
    assert [r["episode_steps"] for r in records] == [2, 3, 4]
    assert [r["episode_return"] for r in records] == [2.0, 3.0, 4.0]
    assert records[0]["outcome"] == "collision" and records[0]["controller"] == "scripted"


def test_the_summary_known_answer(tmp_path):
    """Four episodes: 2 success, 1 timeout, 1 collision. Contact in 2 (the collision, and a success
    that touched something on the way). Stops pooled: 3 over 20 m = 15 per 100 m -- the average of
    per-episode rates would be (10 + 0 + 0 + 28.57) / 4 = 9.64, which is why it is pooled."""
    records = run_episodes(ScriptedEnv(**ENV), Counter(), range(4), ring=StepRing(4, tmp_path))
    s = outcome_summary(records)
    assert s["counts"] == {"success": 2, "collision": 1, "timeout": 1, "unrecoverable_stop": 0}
    assert s["success_rate"] == 0.5 and s["contact_rate"] == 0.5
    assert s["largest_outcome_share"] == 0.5
    assert s["stops_per_100m"] == pytest.approx(15.0)
    assert s["mean_return"] == pytest.approx(3.5)


def test_an_unknown_outcome_or_a_missing_one_fails_loudly(tmp_path):
    with pytest.raises(ValueError, match="vocabulary"):
        outcome_summary([{"outcome": "teleported", "episode_return": 0.0}])
    env = ScriptedEnv(**ENV)
    env.endings = ["success", "timeout", "collision", "success"]
    original = env.step

    def no_outcome(action):
        obs, r, term, trunc, info = original(action)
        info.pop("episode", None)
        return obs, r, term, trunc, info

    env.step = no_outcome
    with pytest.raises(RuntimeError, match="without an outcome"):
        run_episodes(env, Counter(), [0], ring=StepRing(4, tmp_path))


def test_an_empty_set_has_no_rates_rather_than_zero_ones():
    s = outcome_summary([])
    assert s["episodes"] == 0 and s["success_rate"] is None and s["stops_per_100m"] is None


# ---------------------------------------------------------------- scripts/evaluate.py refusals
def _evaluate():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "evaluate_script", Path(__file__).resolve().parents[1] / "scripts" / "evaluate.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _cfg(name):
    from pathlib import Path

    from pfbnav.config import load_config
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "experiments"
                       / f"{name}.yaml")


TUNE = ["scenarios/tune/tune_narrow.json"]


def test_evaluation_never_touches_test_layouts(tmp_path):
    ckpt = tmp_path / "ckpt.zip"
    ckpt.write_bytes(b"x")
    with pytest.raises(SystemExit, match="hard rule 2"):
        _evaluate().preflight(_cfg("L1_sac_shaped"), ["scenarios/test/anything.json"], str(ckpt))


def test_evaluation_refuses_the_filter_a_missing_checkpoint_and_a_missing_set(tmp_path):
    script = _evaluate()
    ckpt = tmp_path / "ckpt.zip"
    ckpt.write_bytes(b"x")
    with pytest.raises(SystemExit, match="WP2"):
        script.preflight(_cfg("L5_sac_pfb_unshaped"), TUNE, str(ckpt))
    with pytest.raises(SystemExit, match="no checkpoint"):
        script.preflight(_cfg("L1_sac_shaped"), TUNE, str(tmp_path / "missing.zip"))
    with pytest.raises(SystemExit, match="not found"):
        script.preflight(_cfg("L1_sac_shaped"), ["scenarios/tune/none.json"], str(ckpt))
    assert [p.name for p in script.preflight(_cfg("L1_sac_shaped"), TUNE, str(ckpt))] == [
        "tune_narrow.json"]


def test_the_controls_run_on_the_base_config_without_a_checkpoint(tmp_path):
    from pathlib import Path

    from pfbnav.config import load_config
    script = _evaluate()
    base = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    for name in ("pure_pursuit", "regulated_pure_pursuit", "dwa", "zero"):
        assert script.preflight(base, TUNE, None, name)[0].name == "tune_narrow.json"
    with pytest.raises(SystemExit, match="for the sac controller"):
        script.preflight(base, TUNE, str(tmp_path), "zero")
    with pytest.raises(SystemExit, match="experiment"):
        script.preflight(base, TUNE, None, "sac")
    with pytest.raises(SystemExit, match="unknown controller"):
        script.preflight(base, TUNE, None, "no_such_controller")
    assert script.make_controller("pure_pursuit", base, None).name == "B1_pure_pursuit"
    assert script.make_controller("regulated_pure_pursuit", base, None).name == \
        "B2_regulated_pure_pursuit"
    assert script.PROFILE_KIND["regulated_pure_pursuit"] == "cap"     # under the route cap, as B1
    # B3 (ruled 2026-10-26): the route cap under the recovery, as B1 and B2.
    from pfbnav.control.dwa import HolonomicDWA
    from pfbnav.control.recovery import Recovery
    from pfbnav.control.route_cap import RouteCapped
    b3 = script.make_controller("dwa", base, None)
    assert b3.name == "B3_holonomic_dwa" and isinstance(b3, Recovery)
    assert isinstance(b3.base, RouteCapped) and isinstance(b3.base.base, HolonomicDWA)
    assert script.PROFILE_KIND["dwa"] == "cap"
    assert script.preflight(base, TUNE, None, "field_aware_mppi")[0].name == "tune_narrow.json"
    b6 = script.make_controller("field_aware_mppi", base, None)
    assert b6.name == "B6_field_aware_mppi" and script.PROFILE_KIND["field_aware_mppi"] == "cap"
    assert type(b6).__name__ == "Recovery" and type(b6.base).__name__ == "RouteCapped"
    assert script.make_controller("zero", base, None).name == "zero_action"



def test_a_failed_step0_check_stops_the_run_rather_than_skipping_the_episode(tmp_path):
    """Ruled 2026-10-10 (supervisor, 2026-10-13): the reset's step-0 pose check must raise and stop
    the run, never warn and continue. The environment raises; nothing between it and the script
    may catch it, so the episode after it never runs and no record is written for it."""
    class Misplaced(ScriptedEnv):
        def reset(self, *, options=None):
            if options["index"] == 1:
                raise RuntimeError("at step 0 the robot is 5.764 m from the scenario's start")
            return super().reset(options=options)

    env, got = Misplaced(**ENV), []
    with pytest.raises(RuntimeError, match="step 0"):
        run_episodes(env, Counter(), [0, 1, 2], on_episode=got.append,
                     ring=StepRing(4, tmp_path))
    assert env.seen == [0] and [r["index"] for r in got] == [0]


class LoadCounting(ScriptedEnv):
    """A scripted environment whose client counts scene loads."""

    class Client:
        def __init__(self):
            self.loads = 0

        def load_scene(self, path):
            self.loads += 1

    def __init__(self, **kw):
        super().__init__(**kw)
        self.client = self.Client()


def test_a_scenarios_controller_set_runs_back_to_back_without_a_reload(tmp_path):
    """WP5 (ruled 2026-10-14): all controllers for a scenario run within one scene load."""
    from pfbnav.eval.runner import run_controllers

    class Named(Counter):
        def __init__(self, name):
            super().__init__()
            self.name = name

    env = LoadCounting(**ENV)
    rings = {n: StepRing(4, tmp_path / n) for n in ("a", "b")}
    records = run_controllers(env, [Named("a"), Named("b")], [2, 0], rings=rings)
    assert [(r["index"], r["controller"]) for r in records] == [(2, "a"), (2, "b"), (0, "a"),
                                                                (0, "b")]
    assert env.client.loads == 0


def test_neither_the_runner_nor_the_environment_can_load_a_scene():
    """The structural half: no call that loads a scene anywhere in the environment's package or
    the runner, so no reset and no evaluation loop can put a reload between paired members."""
    import ast
    from pathlib import Path

    root = Path(__file__).resolve().parents[1] / "pfbnav"
    files = [*sorted((root / "envs").glob("*.py")), root / "eval" / "runner.py"]
    calls = []
    for path in files:
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr in ("load_scene", "loadScene", "closeScene")):
                calls.append(f"{path.name}:{node.lineno}")
    assert len(files) > 3 and calls == [], f"scene loads in {calls}"


def test_the_two_secondary_stop_rates_declared_for_a14():
    """Ruled 2026-10-16. Primary: stops per 100 m DRIVEN, recovery motion included. Secondary:
    excluding recovery distance, and per 100 m of PLANNED route."""
    records = [
        {"outcome": "success", "episode_return": 0.0, "protective_stops": 2, "distance_m": 12.0,
         "recovery_distance_m": 2.0, "plan_length_m": 10.0},
        {"outcome": "timeout", "episode_return": 0.0, "protective_stops": 1, "distance_m": 8.0,
         "plan_length_m": 10.0},                      # no recovery: a learned method's record
    ]
    s = outcome_summary(records)
    assert s["stops_per_100m"] == pytest.approx(100.0 * 3 / 20.0)
    assert s["recovery_distance_m"] == pytest.approx(2.0)
    assert s["stops_per_100m_excluding_recovery"] == pytest.approx(100.0 * 3 / 18.0)
    assert s["planned_length_m"] == pytest.approx(20.0)
    assert s["stops_per_100m_planned"] == pytest.approx(100.0 * 3 / 20.0)
    with pytest.raises(ValueError, match="recovery distance"):
        outcome_summary([{**records[0], "recovery_distance_m": 13.0}])


def test_the_record_counts_the_pfbs_decision_modes(tmp_path):
    """With the filter on, each episode's record carries how many steps were in each PFB mode (the
    fallback count of the B2-B7 tune-set comparison, ruled 2026-10-26); with it off, nothing."""
    class Filtered(ScriptedEnv):
        def step(self, action):
            obs, r, term, trunc, info = super().step(action)
            self.k = getattr(self, "k", 0) + 1
            info["pfb"] = {"mode": "fallback" if self.k % 2 else "filtered"}
            return obs, r, term, trunc, info

    rec = run_episodes(Filtered(**ENV), Counter(), [1], ring=StepRing(4, tmp_path / "a"))[0]
    assert rec["pfb_modes"] == {"fallback": 3, "filtered": 2} and rec["episode_steps"] == 5
    plain = run_episodes(ScriptedEnv(**ENV), Counter(), [1], ring=StepRing(4, tmp_path / "b"))[0]
    assert "pfb_modes" not in plain


def test_every_episode_records_its_per_step_compute(tmp_path):
    """Ruled 2026-10-27: per-step compute for every controller. The controller's act is timed each
    step; where the PFB runs, its own time (`info["pfb_seconds"]`) is recorded and added to the
    decision's. A controller that sleeps 3 ms is recorded at no less than 2 ms (a margin for the
    sleep's clock against perf_counter's under load); a PFB reporting 3 ms lands in the 3.0-3.1 ms
    bin."""
    import time

    from pfbnav.eval.compute import ComputeHistogram

    class Slow(Counter):
        def act(self, obs):
            time.sleep(0.003)
            return np.zeros(3)

    class Filtered(ScriptedEnv):
        def step(self, action):
            obs, r, term, trunc, info = super().step(action)
            return obs, r, term, trunc, {**info, "pfb_seconds": 0.003}

    rec = run_episodes(Filtered(**ENV), Slow(), [1], ring=StepRing(4, tmp_path / "a"))[0]
    hist = {k: ComputeHistogram.from_record(v) for k, v in rec["compute_hist"].items()}
    assert {k: h.steps for k, h in hist.items()} == {"controller": 5, "pfb": 5, "decision": 5}
    assert min(hist["controller"].counts) >= 20                 # bins of 0.1 ms: >= 2.0 ms
    assert hist["pfb"].counts == {30: 5}
    assert min(hist["decision"].counts) >= 50                   # 2 + 3 ms
    plain = run_episodes(ScriptedEnv(**ENV), Counter(), [1], ring=StepRing(4, tmp_path / "b"))[0]
    assert set(plain["compute_hist"]) == {"controller", "decision"}
    s = outcome_summary([rec])["compute"]
    assert s["pfb"]["p95_ms"] == pytest.approx(3.1) and s["controller"]["p50_ms"] >= 2.0


def test_the_summary_reports_each_controllers_counted_statuses():
    """Ruled 2026-10-27: dwa_blocked_steps, mppi_all_rejected and pfb_infeasible per method and
    stratum, with the environment's invalid_bin_steps; 0 for a method without the status."""
    recs = [{"outcome": "success", "episode_return": 0.0, "distance_m": 5.0,
             "dwa_blocked_steps": 3, "invalid_bin_steps": 1},
            {"outcome": "timeout", "episode_return": 0.0, "distance_m": 1.0,
             "dwa_blocked_steps": 4, "mppi_all_rejected": 2, "pfb_infeasible": 5}]
    s = outcome_summary(recs)
    assert (s["dwa_blocked_steps"], s["mppi_all_rejected"], s["pfb_infeasible"],
            s["invalid_bin_steps"]) == (7, 2, 5, 1)
    assert outcome_summary(recs[:1])["mppi_all_rejected"] == 0
