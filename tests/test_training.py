"""SAC construction, the episode recorder and the episode log, on a synthetic environment.

No simulator: `ToyEnv` has NavEnv's spaces (82-D observation, 3-D action in [-1, 1]) and puts
its own record under `info["episode"]` on the last step, as NavEnv does.
"""
import json

import gymnasium as gym
import numpy as np
import pytest
from gymnasium import spaces

from pfbnav.control.base import Controller
from pfbnav.control.sac import SACController, make_sac, sac_kwargs
from pfbnav.training.record import RECORD_KEY, EpisodeLog, EpisodeRecorder

TRAINING = {"algo": "sac", "learning_rate": 3e-4, "buffer_size": 5000, "learning_starts": 50,
            "batch_size": 32, "tau": 0.005, "gamma": 0.99, "train_freq": 1, "gradient_steps": 1,
            "ent_coef": "auto", "net_arch": [32, 32], "device": "cpu"}


class ToyEnv(gym.Env):
    """Deterministic: the observation is a fixed function of the step and the last action."""

    def __init__(self, length=10):
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(82,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.length = length
        self.t = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.t = 0
        return np.zeros(82, dtype=np.float32), {}

    def step(self, action):
        self.t += 1
        obs = np.full(82, np.tanh(0.1 * self.t + float(np.sum(action))), dtype=np.float32)
        reward = float(-np.sum(np.square(action)))
        done = self.t >= self.length
        info = {}
        if done:
            info["episode"] = {"outcome": "timeout", "protective_stops": 2,
                               "stops_per_100m": np.float64(1.5), "scenario_seed": 1000007}
        return obs, reward, done, False, info


def test_every_hyperparameter_is_required_and_mapped():
    kw = sac_kwargs(TRAINING)
    assert kw["batch_size"] == 32 and kw["policy_kwargs"] == {"net_arch": [32, 32]}
    assert kw["ent_coef"] == "auto" and kw["device"] == "cpu"
    for key in ("net_arch", "gamma", "train_freq"):
        broken = {k: v for k, v in TRAINING.items() if k != key}
        with pytest.raises(ValueError, match=key):
            sac_kwargs(broken)
    with pytest.raises(ValueError, match="algo"):
        sac_kwargs({**TRAINING, "algo": "ppo"})


def test_the_recorder_moves_the_environments_record_and_gives_sb3_what_it_expects():
    env = EpisodeRecorder(ToyEnv(length=3))
    env.reset()
    infos = [env.step(np.array([0.1, 0.0, 0.0]))[4] for _ in range(3)]
    assert RECORD_KEY not in infos[0] and "episode" not in infos[0]
    last = infos[-1]
    assert last["episode"]["l"] == 3
    assert last["episode"]["r"] == pytest.approx(-3 * 0.01)
    assert last[RECORD_KEY]["outcome"] == "timeout"
    assert last[RECORD_KEY]["episode_length"] == 3 and last[RECORD_KEY]["truncated"] is False
    env.reset()                                                  # return and length restart
    env.step(np.zeros(3))
    assert env._return == 0.0 and env._length == 1


def test_sac_learns_on_the_wrapped_env_and_logs_each_episode(tmp_path):
    env = EpisodeRecorder(ToyEnv(length=10))
    model = make_sac(env, {"training": TRAINING}, seed=3)
    log = EpisodeLog(tmp_path / "episodes.jsonl")
    model.learn(total_timesteps=120, callback=log)
    lines = [json.loads(s) for s in (tmp_path / "episodes.jsonl").read_text().splitlines()]
    assert len(lines) == 12 and log.episodes == 12 and log.outcomes == {"timeout": 12}
    assert lines[0]["episode"] == 1 and lines[0]["timestep"] == 10
    assert lines[-1]["timestep"] == 120 and lines[0]["stops_per_100m"] == 1.5
    assert lines[0]["scenario_seed"] == 1000007


def test_a_log_never_appends_to_an_existing_file(tmp_path):
    (tmp_path / "episodes.jsonl").write_text("{}\n")
    with pytest.raises(FileExistsError):
        EpisodeLog(tmp_path / "episodes.jsonl")


def test_the_same_seed_gives_bit_identical_learning_within_one_process():
    params = []
    for _ in range(2):
        model = make_sac(EpisodeRecorder(ToyEnv(length=10)), {"training": TRAINING}, seed=11)
        model.learn(total_timesteps=150)
        params.append(np.concatenate([p.detach().numpy().ravel()
                                      for p in model.policy.actor.parameters()]))
    assert np.array_equal(params[0], params[1])
    other = make_sac(EpisodeRecorder(ToyEnv(length=10)), {"training": TRAINING}, seed=12)
    other.learn(total_timesteps=150)
    theirs = np.concatenate([p.detach().numpy().ravel() for p in other.policy.actor.parameters()])
    assert not np.array_equal(params[0], theirs)


def test_a_checkpoint_becomes_a_deterministic_controller(tmp_path):
    model = make_sac(EpisodeRecorder(ToyEnv()), {"training": TRAINING}, seed=5)
    model.learn(total_timesteps=60)
    model.save(tmp_path / "ckpt.zip")
    ctrl = SACController(tmp_path / "ckpt.zip", name="L1")
    assert isinstance(ctrl, Controller)
    obs = np.linspace(-1, 1, 82).astype(np.float32)
    a, b = ctrl.act(obs), ctrl.act(obs)
    assert a.shape == (3,) and np.array_equal(a, b) and np.all(np.abs(a) <= 1.0)
    with pytest.raises(FileNotFoundError):
        SACController(tmp_path / "missing.zip", name="L1")


# ---------------------------------------------------------------- scripts/train.py refusals
def _train_script():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "scripts" / "train.py"
    spec = importlib.util.spec_from_file_location("train_script", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _cfg(experiment):
    from pathlib import Path

    from pfbnav.config import load_config
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "experiments"
                       / f"{experiment}.yaml")


def test_training_refuses_a_filtered_method_until_alpha_is_ruled():
    with pytest.raises(SystemExit, match="alpha is not ruled"):
        _train_script().preflight(_cfg("L5_sac_pfb_unshaped"))


def test_training_refuses_an_open_or_unbuilt_training_set():
    script = _train_script()
    cfg = _cfg("L1_sac_shaped")
    assert [p.name for p in script.preflight(cfg)] == [   # built 2026-10-03, committed b1cf775
        "train_narrow.json", "train_medium.json", "train_wide.json"]
    cfg["training"]["scenarios"] = ["scenarios/train/not_built.json"]
    with pytest.raises(SystemExit, match="has not been built"):
        script.preflight(cfg)
    cfg["training"]["scenarios"] = "OPEN"
    with pytest.raises(SystemExit, match="await a ruling"):
        script.preflight(cfg)
    with pytest.raises(SystemExit, match="experiment"):
        script.preflight({**cfg, "experiment": None})


def test_training_refuses_an_environment_that_is_not_frozen_or_not_the_frozen_one():
    script = _train_script()
    cfg = _cfg("L1_sac_shaped")
    cfg["env"]["frozen_code_sha256"] = None
    with pytest.raises(SystemExit, match="not frozen"):
        script.preflight(cfg)
    cfg["env"]["frozen_code_sha256"] = "0" * 64
    with pytest.raises(SystemExit, match="not the frozen one"):
        script.preflight(cfg)


def test_the_base_config_carries_every_sac_hyperparameter():
    from pathlib import Path

    from pfbnav.config import load_config
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    kw = sac_kwargs(cfg["training"])
    assert kw["train_freq"] == 1 and kw["gradient_steps"] == 1   # one update per step (G2)


# ---------------------------------------------------------------- the per-seed order (ruled)
def test_each_seed_has_its_own_permutation_and_it_depends_on_the_seed_alone():
    from pfbnav.training.order import order_sha256, scenario_order
    a, b = scenario_order(2001, 0), scenario_order(2001, 0)
    assert np.array_equal(a, b) and sorted(a.tolist()) == list(range(2001))
    others = [scenario_order(2001, k) for k in range(1, 5)]
    assert all(not np.array_equal(a, o) for o in others)
    assert len({order_sha256(o) for o in [a, *others]}) == 5
    # Not the identity, and not a stream shared with plain default_rng(seed).
    assert not np.array_equal(a, np.arange(2001))
    assert not np.array_equal(a, np.random.default_rng(0).permutation(2001))
    with pytest.raises(ValueError):
        scenario_order(0, 0)
