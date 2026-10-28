"""Gate G2's measurement and verdict (`pfbnav/eval/g2.py`, `scripts/gate_g2_throughput.py`): known
answers on synthetic data (hard rule 6), and the script's refusal while its criterion is unruled."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import gymnasium as gym
import numpy as np
import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.eval.g2 import Meter, projection, select_instances, summarise_phase, verdict_n

REPO = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeEnv(gym.Env):
    """Reset costs 2 s, each step 0.1 s; episodes of 3 steps; step 2 of each episode over budget."""
    observation_space = gym.spaces.Box(-1, 1, (1,))
    action_space = gym.spaces.Box(-1, 1, (1,))

    def __init__(self, clock):
        self.clock, self.k = clock, 0

    def reset(self, *, seed=None, options=None):
        self.clock.t += 2.0
        self.k = 0
        return np.zeros(1), {}

    def step(self, action):
        self.clock.t += 0.1
        self.k += 1
        info = {"over_budget": self.k == 2, "scan_fault": False}
        return np.zeros(1), 0.0, self.k == 3, False, info


def test_meter_counts_steps_resets_and_episodes_in_its_window():
    clock = Clock()
    rendered = iter([True, False])               # read only for episodes inside the window
    meter = Meter(FakeEnv(clock), lambda: next(rendered), clock=clock)
    meter.reset()
    meter.step(0)                               # outside the window: not counted
    meter.start_window()
    for _ in range(2):                          # two whole episodes: reset + 3 steps each
        meter.reset()
        for _ in range(3):
            meter.step(0)
    out = meter.stop_window()
    assert out["steps"] == 6 and out["over_budget_steps"] == 2
    assert out["seconds"] == pytest.approx(2 * (2.0 + 0.3))
    assert [e["steps"] for e in out["episodes"]] == [3, 3]
    assert [e["rendered"] for e in out["episodes"]] == [True, False]
    assert all(e["seconds"] == pytest.approx(2.3) and e["reset_s"] == pytest.approx(2.0)
               for e in out["episodes"])
    meter.step(0)                               # closed again: not counted
    assert meter.result()["steps"] == 6
    with pytest.raises(RuntimeError):
        meter.stop_window()


def _worker(steps, seconds, episodes, over=0, faults=0):
    return {"steps": steps, "seconds": seconds, "over_budget_steps": over,
            "scan_fault_steps": faults, "episodes": episodes}


def test_summarise_phase_known_answer():
    ep = lambda steps, seconds, rendered: {"steps": steps, "seconds": seconds,
                                           "rendered": rendered, "reset_s": 2.0}
    w1 = _worker(3000, 300.0, [ep(100, 20.0, False), ep(100, 25.0, True)], over=3)
    w2 = _worker(1500, 300.0, [ep(50, 10.0, False), ep(50, 10.0, False)], faults=1)
    s = summarise_phase([w1, w2])
    assert s["aggregate_steps_per_s"] == pytest.approx(10.0 + 5.0)
    assert s["per_instance_steps_per_s"] == pytest.approx([10.0, 5.0])
    assert s["rendered_episode_fraction"] == pytest.approx(0.25)
    assert s["rendered"]["per_instance_steps_per_s"] == pytest.approx(4.0)
    assert s["not_rendered"]["per_instance_steps_per_s"] == pytest.approx(200 / 40.0)
    assert s["over_budget_share"] == pytest.approx(3 / 4500)
    assert s["scan_fault_share"] == pytest.approx(1 / 4500)
    assert s["reset_s"]["median"] == pytest.approx(2.0)


CRIT = {"min_steps_per_second": 25.0, "max_rendered_episode_fraction": 0.10,
        "max_over_budget_share": 0.001, "max_scan_fault_share": 0.001}


def test_verdict_each_criterion_and_its_boundary():
    good = {"aggregate_steps_per_s": 25.0, "rendered_episode_fraction": 0.10,
            "over_budget_share": 0.001, "scan_fault_share": 0.001}
    assert verdict_n(good, **CRIT)["pass"]                       # every bound is inclusive
    for key, bad in (("aggregate_steps_per_s", 24.99), ("rendered_episode_fraction", 0.11),
                     ("over_budget_share", 0.0011), ("scan_fault_share", 0.002),
                     ("aggregate_steps_per_s", None)):
        v = verdict_n({**good, key: bad}, **CRIT)
        assert not v["pass"] and len(v["failed"]) == 1


def test_the_approved_count_is_the_largest_passing_n():
    v = {1: {"pass": False}, 2: {"pass": True}, 3: {"pass": True}, 4: {"pass": False}}
    assert select_instances(v) == 3
    assert select_instances({1: {"pass": False}}) is None


def test_projection_known_answer():
    p = projection(25.0, 25, 300_000, 6)
    assert p["seconds"] == pytest.approx(300_000.0) and p["fits"]
    assert p["days"] == pytest.approx(300_000 / 86400)
    assert not projection(1.0, 25, 300_000, 6)["fits"]          # 86.8 days > 42
    assert not projection(0.0, 25, 300_000, 6)["fits"]


def _script():
    spec = importlib.util.spec_from_file_location("gate_g2", REPO / "scripts" /
                                                  "gate_g2_throughput.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_g2_refuses_while_its_over_budget_reading_is_unruled(tmp_path, monkeypatch):
    mod = _script()
    gate = tmp_path / "g2.yaml"
    gate.write_text("max_over_budget_share: null\n", encoding="utf-8")
    monkeypatch.setattr(mod, "GATE_CONFIG", gate)
    with pytest.raises(SystemExit, match="not ruled"):
        mod.criteria()
    gate.write_text("max_over_budget_share: 0.001\n", encoding="utf-8")
    crit = mod.criteria()
    assert crit["min_steps_per_second"] == 25 and crit["parallel_instances"] == [1, 2, 3, 4]
    assert crit["max_scan_fault_share"] == 0.001


def test_g2_uses_the_ruled_over_budget_share_and_refuses_a_disagreeing_overlay(tmp_path,
                                                                               monkeypatch):
    """Ruled 2026-10-27: 0.001. The overlay carries it now; A28 writes it into gates.yaml, and from
    then the two must agree (hard rule 16: the overlay never quietly states another number)."""
    mod = _script()
    assert mod.criteria()["max_over_budget_share"] == 0.001          # the committed overlay
    gates = yaml.safe_load((REPO / "docs" / "pending" / "gates_A28.yaml").read_text(
        encoding="utf-8"))
    assert gates["G2_compute_budget"]["max_over_budget_share"] == 0.001
    gate = tmp_path / "g2.yaml"
    gate.write_text("max_over_budget_share: 0.002", encoding="utf-8")
    monkeypatch.setattr(mod, "GATE_CONFIG", gate)
    monkeypatch.setattr(mod, "REPO_ROOT", tmp_path)
    (tmp_path / "preregistration").mkdir()
    (tmp_path / "preregistration" / "gates.yaml").write_text(yaml.safe_dump(gates),
                                                             encoding="utf-8")
    with pytest.raises(SystemExit, match="preregistration/gates.yaml 0.001"):
        mod.criteria()


def test_worker_config_sets_port_budget_and_learning_starts_only():
    mod = _script()
    cfg = load_config(REPO / "configs" / "experiments" / "L1_sac_shaped.yaml")
    w = mod.worker_config(cfg, n=3, index=2)
    assert w["sim"]["port"] == cfg["sim"]["port"] + 2 * cfg["sim"]["port_stride"]
    assert w["env"]["step_budget"]["instances"] == 3
    assert w["training"]["learning_starts"] == cfg["training"]["batch_size"]
    for key in ("sim", "env", "training"):
        changed = {k for k in cfg[key] if cfg[key][k] != w[key][k]}
        assert changed <= {"port", "step_budget", "learning_starts"}
    assert cfg["env"]["step_budget"]["instances"] == 4          # the original is untouched
