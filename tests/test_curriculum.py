"""The training curriculum, the memory alarm and the unicycle diagnostic (A30, ruled 2026-10-30)."""
import json
from pathlib import Path

import gymnasium as gym
import numpy as np
import pytest
import yaml
from gymnasium import spaces

from pfbnav.config import load_config
from pfbnav.envs.freeze import import_closure
from pfbnav.training.curriculum import FULL, Curriculum, stage_at, stages_from_config
from pfbnav.training.diagnostic import UnicycleAction, diagnostic_problem

REPO_ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(REPO_ROOT / "configs" / "base.yaml")
GATES = yaml.safe_load((REPO_ROOT / "preregistration" / "gates.yaml").read_text(encoding="utf-8"))
STAGES = stages_from_config(CFG)


def test_the_configured_curriculum_is_the_ruled_one():
    assert [(s.name, s.from_fraction, set(s.strata), s.unmapped) for s in STAGES] == [
        ("wide_medium_mapped", 0.0, {"wide", "medium"}, False),
        ("all_strata_mapped", 0.2, {"narrow", "medium", "wide"}, False),
        (FULL, 0.4, {"narrow", "medium", "wide"}, True)]
    assert CFG["training"]["total_timesteps"] == 2_000_000


@pytest.mark.parametrize("bad, match", [
    ([{"name": "a", "from_fraction": 0.1, "strata": ["wide"], "unmapped": True}], "starts at"),
    ([{"name": "a", "from_fraction": 0.0, "strata": ["wide"], "unmapped": False},
      {"name": FULL, "from_fraction": 0.0, "strata": ["wide"], "unmapped": True}], "increase"),
    ([{"name": "a", "from_fraction": 0.0, "strata": ["wide"], "unmapped": True}], "last"),
    ([{"name": FULL, "from_fraction": 0.0, "strata": ["wide"], "unmapped": False}], "last"),
])
def test_a_malformed_curriculum_is_refused(bad, match):
    with pytest.raises(ValueError, match=match):
        stages_from_config({"training": {"curriculum": bad}})


def test_stage_boundaries_dense_sweep():
    """Every step of a 1,000-step run: stage 0 below 200, stage 1 in [200, 400), full from 400."""
    names = [stage_at(STAGES, t, 1000).name for t in range(1001)]
    assert names[:200] == ["wide_medium_mapped"] * 200
    assert names[200:400] == ["all_strata_mapped"] * 200
    assert names[400:] == [FULL] * 601


def test_the_training_set_s_pools():
    scenarios = [s for p in CFG["training"]["scenarios"]
                 for s in json.loads((REPO_ROOT / p).read_text(encoding="utf-8"))["scenarios"]]
    pools = {s.name: [sc for sc in scenarios if s.admits(sc)] for s in STAGES}
    assert len(pools[FULL]) == 2001
    assert {sc["stratum"] for sc in pools["wide_medium_mapped"]} == {"wide", "medium"}
    for name in ("wide_medium_mapped", "all_strata_mapped"):
        assert all(sc["unmapped_count"] == 0 for sc in pools[name])
    n0 = sum(sc["unmapped_count"] == 0 for sc in scenarios)
    assert len(pools["all_strata_mapped"]) == n0 and 1400 < n0 < 1600      # rate ~0.245 built
    assert 900 < len(pools["wide_medium_mapped"]) < 1100


class Picker(gym.Env):
    """Records the scenario index each reset was given; each episode lasts `length` steps."""
    observation_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
    action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def __init__(self, length=10):
        self.length, self.indices, self.actions = length, [], []

    def reset(self, *, seed=None, options=None):
        self.indices.append(options["index"])
        self.t = 0
        return np.zeros(1, dtype=np.float32), {}

    def step(self, action):
        self.actions.append(np.array(action))
        self.t += 1
        done = self.t >= self.length
        return (np.zeros(1, dtype=np.float32), 0.0, False, done,
                {"episode": {"outcome": "timeout"}} if done else {})


SCEN = [{"stratum": st, "unmapped_count": u} for st, u in
        [("narrow", 0), ("wide", 1), ("medium", 0), ("wide", 0), ("narrow", 2), ("medium", 0)]]


def run(env, steps):
    env.reset()
    for _ in range(steps):
        *_, term, trunc, info = env.step(np.zeros(3))
        if term or trunc:
            yield info
            env.reset()


def test_each_stage_walks_the_seed_order_restricted_and_cycles():
    """Episodes of 10 steps in a 100-step run: stage 0 for steps 0-19 (2 episodes), stage 1 for
    20-39, full from 40. Stage 0 admits indices 2, 3, 5 (in the seed's order); stage 1 adds 0;
    full is every index."""
    base = Picker()
    env = Curriculum(base, STAGES, SCEN, total_timesteps=100)
    infos = list(run(env, 100))
    assert base.indices[:2] == [2, 3]                 # stage 0: walks its own pool
    assert base.indices[2:4] == [0, 2]                # stage 1: from ITS start
    assert base.indices[4:11] == [0, 1, 2, 3, 4, 5, 0]   # full: everything, cycling
    assert [i["episode"]["curriculum_stage"] for i in infos][:5] == [
        "wide_medium_mapped", "wide_medium_mapped", "all_strata_mapped", "all_strata_mapped", FULL]
    assert env.report()["stages"][0] | {} == {"name": "wide_medium_mapped", "from_fraction": 0.0,
                                              "strata": ["wide", "medium"], "unmapped": False,
                                              "pool": 3, "episodes": 2}


def test_a_stage_never_switches_within_an_episode():
    """A 30-step episode straddling the 20 % boundary stays stage 0; the next one is stage 1."""
    base = Picker(length=30)
    infos = list(run(Curriculum(base, STAGES, SCEN, total_timesteps=100), 60))
    assert [i["episode"]["curriculum_stage"] for i in infos] == ["wide_medium_mapped",
                                                                 "all_strata_mapped"]


def test_a_stage_with_no_scenario_is_refused():
    with pytest.raises(ValueError, match="admit no"):
        Curriculum(Picker(), STAGES, [{"stratum": "narrow", "unmapped_count": 1}], 100)


def test_training_only_and_outside_the_frozen_environment():
    users = sorted(p.relative_to(REPO_ROOT).as_posix()
                   for p in list((REPO_ROOT / "scripts").glob("*.py"))
                   + list((REPO_ROOT / "pfbnav").rglob("*.py"))
                   if "pfbnav.training.curriculum" in p.read_text(encoding="utf-8")
                   or "pfbnav.training.diagnostic" in p.read_text(encoding="utf-8"))
    assert users == ["scripts/train.py"]
    closure = import_closure()
    for name in ("curriculum.py", "diagnostic.py", "stall.py"):
        assert REPO_ROOT / "pfbnav" / "training" / name not in closure


# ------------------------------------------------------------------ the unicycle diagnostic
def test_unicycle_zeroes_only_the_lateral_axis():
    base = Picker()
    env = UnicycleAction(base)
    env.reset(options={"index": 0})
    env.step(np.array([0.7, -0.9, 0.3]))
    assert np.allclose(base.actions[-1], [0.7, 0.0, 0.3])


def test_the_unicycle_config_and_its_seed():
    cfg = load_config(REPO_ROOT / "configs" / "experiments" / "diag" / "L1_unicycle.yaml")
    l1 = load_config(REPO_ROOT / "configs" / "experiments" / "L1_sac_shaped.yaml")
    assert cfg["diagnostic"]["unicycle"] is True and cfg["experiment"] == "DIAG_L1_unicycle"
    assert {k: v for k, v in cfg.items() if k not in ("experiment", "diagnostic")} == \
        {k: v for k, v in l1.items() if k != "experiment"}
    assert diagnostic_problem(cfg, 9_000_100, GATES) is None
    assert "pilot training seed" in diagnostic_problem(cfg, 9_000_001, GATES)
    assert "pilot range" in diagnostic_problem(cfg, 1_000_000, GATES)
    assert "DIAG_" in diagnostic_problem({**cfg, "experiment": "L1x"}, 9_000_100, GATES)
    assert diagnostic_problem(l1, 9_000_001, GATES) is None
    methods = (REPO_ROOT / "configs" / "methods.yaml").read_text(encoding="utf-8")
    assert "DIAG_" not in methods          # a diagnostic is never a method (no ID)


# ------------------------------------------------------------------ the memory alarm
class FakeMemory:
    def __init__(self, values):
        self.values = iter(values)

    def snapshot(self):
        return {"python": next(self.values), "sim": None}


def test_the_memory_alarm_fires_once_past_1_gb_beyond_the_settled_baseline(tmp_path, capsys):
    from pfbnav.training.record import MemoryTrend
    # Samples at steps 1, 1000, 2000, ...: 1,000 MB of start-up by step 2,000 (not counted), the
    # baseline at 3,000 (3,000 MB), then +200 MB per sample: past 1,024 MB at step 9,000.
    values = [1000, 2500, 2950, 3000] + [3000 + 200 * k for k in range(1, 10)]
    trend = MemoryTrend(tmp_path / "m.jsonl", FakeMemory(values), 1000, alarm_mb=1024.0,
                        settled_from=3000)
    for t in [1] + list(range(1000, 13_000, 1000)):
        trend.num_timesteps = t
        trend._on_step()
    assert trend.baseline == {"timestep": 3000, "python_mb": 3000}
    assert trend.alarm["timestep"] == 9000 and trend.alarm["grown_mb"] == 1200
    assert capsys.readouterr().out.count("MEMORY ALARM") == 1
    rows = [json.loads(line) for line in (tmp_path / "m.jsonl").read_text().splitlines()]
    assert len(rows) == 13 and {"gpu_allocated", "gpu_reserved"} <= set(rows[0])
    assert CFG["training"]["memory_alarm_mb"] == 1024.0
