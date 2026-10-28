"""L2, SAC-Lagrangian: the toy positive control ruled 2026-10-03, and its parts on known answers.

No simulator. **The toy** (`StopToy`) is a constrained problem whose answer is known:

    observation  t / T                   (the step within a T-step episode)
    action       a in [-1, 1]
    reward       0.05 a                  (more is better: the unconstrained optimum is a -> 1)
    cost         1 if a > 0 else 0       (a "protective stop", reported exactly as NavEnv reports
                                          one: info["certified"]["stop_triggered"])
    limit        d = 1 stop per episode  (a quarter of the steps)

The constraint BINDS: the unconstrained policy pays a stop on nearly every step (~T per episode),
so it violates d; the constrained optimum keeps a <= 0 on all but d of the T steps, so its mean
action lies BELOW the threshold 0. The test shows unconstrained SAC violating d and SAC-Lagrangian,
with the same `training:` block, meeting it -- and the multiplier rising while it is violated.

The toy's `training:` block is its own (small nets, a higher learning rate, gamma 0.9). Its
MULTIPLIER settings are the ruled ones (A28b, 2026-10-27: a PID-Lagrangian, lambda set once per
completed episode from the measured mean stops per episode over the last 20 against d, lambda_max
100), here with declared PID candidate 1; the reward is scaled by 0.05 so that a multiplier at the
declared gains' level (~0.1-0.3) prices a stop against it. **What 1,000 steps can show is limited:**
under A28a's dual ascent the limit was met here at 1,000 steps on every seed, and still failed on
seeds 1 and 3 at 5,000 -- it was the first trough of a limit cycle. So the test below is a
REGRESSION of the measured 1,000-step state (candidate 1, seed 0, 2026-10-27: plain SAC 2.55 stops
per episode, SAC-Lagrangian 0.80, lambda 0.21, deterministic action -0.52 against +0.41), and the
control itself -- the last iterate meeting the limit, on seeds 0-3 -- is `scripts/validate_l2_toy.py`
(the researcher's, 5,000 steps per seed and candidate), which also selects L2's gains.
"""
import json
import zipfile

import numpy as np
import pytest

from pfbnav.control.sac import SACController, learner_class, make_sac
from pfbnav.control.sac_lagrangian import (
    PID_CANDIDATES,
    RULED,
    LagrangianSettings,
    SACLagrangian,
    make_sac_lagrangian,
    pid_step,
    step_cost,
)
from pfbnav.training.l2_toy import TOY_CFG, D, StopToy, T, mean_action, toy_cfg

STEPS = 1000                     # environment steps per toy run (fast suite: <= 1,000)
LAST = 20                        # episodes over which the end state is judged
_mean_action = mean_action


@pytest.fixture(scope="module")
def runs():
    """Both learners on the toy, same seed, same `training:` block."""
    plain_env, lag_env = StopToy(), StopToy()
    plain = make_sac(plain_env, TOY_CFG, seed=0)
    plain.learn(STEPS)
    lag = make_sac_lagrangian(lag_env, TOY_CFG, seed=0, ruled=False)
    lag.learn(STEPS)
    return plain, plain_env, lag, lag_env


# ---------------------------------------------------------------- the positive control
def test_unconstrained_sac_violates_the_limit_on_the_toy(runs):
    """The constraint binds: without it, the toy's optimum is a stop on nearly every step."""
    plain, env, _, _ = runs
    assert np.mean(env.episode_costs[-LAST:]) >= 2.0 * D
    assert _mean_action(plain) > 0.0                 # on the stop side of the threshold


def test_sac_lagrangian_is_under_the_limit_at_1000_steps_on_the_toy(runs):
    """A regression of the measured 1,000-step state (module docstring; not the control itself):
    the last episodes within 25 % of d (the control's tolerance, stated 2026-10-26) and at most half
    of plain SAC's, the deterministic action below the threshold 0, not pinned at -1."""
    plain, plain_env, lag, env = runs
    assert len(env.episode_costs) == STEPS // T
    assert np.mean(env.episode_costs[-LAST:]) <= 1.25 * D
    assert np.mean(env.episode_costs[-LAST:]) <= 0.5 * np.mean(plain_env.episode_costs[-LAST:])
    assert -0.9 < _mean_action(lag) < 0.0 < _mean_action(plain)
    # The learner's own episode costs are the environment's, episode for episode.
    assert [e["cost"] for e in lag.dual_history] == env.episode_costs


def test_the_multiplier_follows_the_pid_rule_once_per_episode(runs):
    """A28b: lambda is 0 for the first 20 episodes; from then on, the lambda in force during episode
    k + 1 is `pid_step` on the mean of episodes k-19..k, the previous such mean and the integral --
    replayed here from the recorded costs alone, exactly -- never leaving [0, lambda_max]. While
    the limit is violated at first, lambda becomes positive."""
    _, _, lag, _ = runs
    history = lag.dual_history
    window = RULED["cost_window"]
    gains = PID_CANDIDATES[TOY_CFG["lagrangian"]["pid_candidate"]]
    assert all(e["multiplier"] == 0.0 for e in history[:window])
    assert all("window_mean" not in e for e in history[:window - 1])
    costs = [e["cost"] for e in history]
    integral, previous, lam = 0.0, None, 0.0
    for k in range(len(history)):
        assert history[k]["multiplier"] == lam
        if k >= window - 1:
            mean = float(np.mean(costs[k - window + 1:k + 1]))
            lam, integral = pid_step(mean, previous, integral, D, gains, RULED["multiplier_max"])
            previous = mean
            assert history[k]["window_mean"] == mean and history[k]["integral"] == integral
    assert lag.lagrange_multiplier == lam and lag.pid_integral == integral
    assert all(0.0 <= e["multiplier"] <= RULED["multiplier_max"] for e in history)
    assert np.mean(costs[:window]) > D and history[window]["multiplier"] > 0.0
    assert lag.recent_costs == costs[-window:]


# ---------------------------------------------------------------- lambda = 0 is plain SAC
def test_with_the_multiplier_held_at_zero_it_is_plain_sac_bit_for_bit():
    """The copied training loop is SB3's: with lambda never leaving 0 (a limit no episode can
    exceed), every actor and critic parameter equals plain SAC's after the same steps."""
    cfg = {**TOY_CFG, "lagrangian": {**TOY_CFG["lagrangian"], "cost_limit": 1e9}}
    plain = make_sac(StopToy(), cfg, seed=7)
    plain.learn(120)
    lag = make_sac_lagrangian(StopToy(), cfg, seed=7, ruled=False)
    lag.learn(120)
    assert lag.lagrange_multiplier == 0.0 and lag._n_updates == plain._n_updates > 0
    for mine, theirs in ((lag.actor, plain.actor), (lag.critic, plain.critic),
                         (lag.critic_target, plain.critic_target)):
        for a, b in zip(mine.parameters(), theirs.parameters(), strict=True):
            assert np.array_equal(a.detach().numpy(), b.detach().numpy())
    # ...and the cost critic did learn: its weights moved from initialisation.
    fresh = make_sac_lagrangian(StopToy(), cfg, seed=7, ruled=False)
    moved = [not np.array_equal(a.detach().numpy(), b.detach().numpy())
             for a, b in zip(lag.cost_critic.parameters(), fresh.cost_critic.parameters(),
                             strict=True)]
    assert any(moved)


# ---------------------------------------------------------------- checkpoints
def test_an_l2_checkpoint_restores_and_becomes_a_controller(runs, tmp_path):
    _, _, lag, _ = runs
    lag.save(tmp_path / "l2.zip")
    assert "cost_critic.pth" in zipfile.ZipFile(tmp_path / "l2.zip").namelist()
    assert learner_class(tmp_path / "l2.zip") is SACLagrangian
    loaded = SACLagrangian.load(tmp_path / "l2.zip", device="cpu")
    assert loaded.lagrange_multiplier == lag.lagrange_multiplier
    assert loaded.cost_limit == D and loaded.dual_history == lag.dual_history
    assert loaded.cost_window == lag.cost_window == RULED["cost_window"]
    assert loaded.recent_costs == lag.recent_costs
    assert loaded.pid_gains == lag.pid_gains and loaded.pid_integral == lag.pid_integral
    assert loaded.previous_mean == lag.previous_mean
    assert loaded.multiplier_max == RULED["multiplier_max"]
    for a, b in zip(loaded.cost_critic.parameters(), lag.cost_critic.parameters(), strict=True):
        assert np.array_equal(a.detach().numpy(), b.detach().numpy())
    ctrl = SACController(tmp_path / "l2.zip", name="L2")
    obs = np.array([0.5], dtype=np.float32)
    expected = lag.predict(obs, deterministic=True)[0]
    assert np.array_equal(ctrl.act(obs), np.clip(np.asarray(expected, dtype=float).ravel(), -1, 1))


def test_a_plain_checkpoint_still_loads_as_sac(runs, tmp_path):
    plain, _, _, _ = runs
    plain.save(tmp_path / "l1.zip")
    assert learner_class(tmp_path / "l1.zip").__name__ == "SAC"
    SACController(tmp_path / "l1.zip", name="L1")


# ---------------------------------------------------------------- the parts, known answers
def test_the_cost_is_the_stop_trigger_and_is_never_defaulted():
    assert step_cost({"certified": {"stop_triggered": True}}) == 1.0
    assert step_cost({"certified": {"stop_triggered": np.bool_(False)}}) == 0.0
    with pytest.raises(KeyError):
        step_cost({"certified": {}})
    with pytest.raises(KeyError):
        step_cost({})
    with pytest.raises(TypeError):
        step_cost({"certified": {"stop_triggered": 1}})


def test_pid_step_known_answers():
    """Stooke et al.'s Algorithm 2, capped: lambda = max(0, K_P e + K_I I + K_D max(0, rise)),
    I <- max(0, I + e). Each term alone, the projections, the cap, and the first update's zero
    derivative, on hand-computed values."""
    # P alone: e = 3 - 1 = 2, K_P 0.5 -> 1.0; I = 2.
    assert pid_step(3.0, None, 0.0, 1.0, (0.5, 0.0, 0.0), 100.0) == pytest.approx((1.0, 2.0))
    # I alone: I = 4 + (1.5 - 1) = 4.5, K_I 0.2 -> 0.9.
    assert pid_step(1.5, 1.5, 4.0, 1.0, (0.0, 0.2, 0.0), 100.0) == pytest.approx((0.9, 4.5))
    # D alone: rise 1.5 - 1.2 = 0.3, K_D 2 -> 0.6; a FALL gives no derivative term.
    assert pid_step(1.5, 1.2, 0.0, 1.0, (0.0, 0.0, 2.0), 100.0)[0] == pytest.approx(0.6)
    assert pid_step(1.2, 1.5, 0.0, 1.0, (0.0, 0.0, 2.0), 100.0)[0] == 0.0
    # The first update (no previous mean) has no derivative term.
    assert pid_step(1.5, None, 0.0, 1.0, (0.0, 0.0, 2.0), 100.0)[0] == 0.0
    # All three: e = 0.5, I = 1 + 0.5 = 1.5, rise 0.2 -> 0.1*0.5 + 0.2*1.5 + 1.0*0.2 = 0.55.
    assert pid_step(1.5, 1.3, 1.0, 1.0, (0.1, 0.2, 1.0), 100.0) == pytest.approx((0.55, 1.5))
    # The integral is projected onto >= 0 (no wind-down below zero) ...
    assert pid_step(0.0, 0.0, 0.3, 1.0, (0.0, 1.0, 0.0), 100.0) == (0.0, 0.0)
    # ... and lambda onto [0, cap]: a negative P term with no integral gives 0; a huge one, the cap.
    assert pid_step(0.0, 0.0, 0.0, 1.0, (5.0, 0.0, 0.0), 100.0)[0] == 0.0
    assert pid_step(50.0, None, 0.0, 0.0, (5.0, 0.0, 0.0), 100.0)[0] == 100.0
    # On the limit with an integral of 2: lambda stays K_I x 2.
    assert pid_step(1.0, 1.0, 2.0, 1.0, (0.3, 0.1, 1.0), 100.0) == pytest.approx((0.2, 2.0))
    for bad in [(-0.1, 0.0, 0.0), (0.0, -1e-3, 0.0), (0.0, 0.0, -1.0)]:
        with pytest.raises(ValueError):
            pid_step(1.0, None, 0.0, 0.1, bad, 100.0)
    with pytest.raises(ValueError):
        pid_step(-1.0, None, 0.0, 0.1, (0.1, 0.0, 0.0), 100.0)
    with pytest.raises(ValueError):
        pid_step(1.0, None, -1.0, 0.1, (0.1, 0.0, 0.0), 100.0)


def test_the_declared_pid_candidates():
    """A28b: three (K_P, K_I, K_D) in declared order, K_I so that the integral alone reaches ~0.2
    after the A28a run's ~104 summed stops per episode of excess over its first 100 updates."""
    assert PID_CANDIDATES == {1: (0.1, 2e-3, 0.0), 2: (0.1, 2e-3, 1.0), 3: (0.2, 2e-3, 1.0)}
    assert all(0.1 <= k_i * 104 <= 0.3 for _, k_i, _ in PID_CANDIDATES.values())


def test_the_multiplier_is_held_until_the_window_fills_then_set_once_per_episode():
    """Known answers for the A28b update on a scripted history (no learning): lambda and the
    integral stay 0 for the first 19 episodes; the 20th sets lambda from the window mean (no
    derivative on the first update); the 21st also uses the rise of the mean."""
    cfg = {**toy_cfg(2), "lagrangian": {**toy_cfg(2)["lagrangian"], "cost_limit": 0.1}}
    lag = make_sac_lagrangian(StopToy(), cfg, seed=0)
    from stable_baselines3.common.logger import configure
    lag.set_logger(configure(None, []))                     # no learn() here: records go nowhere
    k_p, k_i, k_d = PID_CANDIDATES[2]
    for c in [1.0] * 19:
        lag._end_episode(c)
    assert len(lag.recent_costs) == 19 and lag.lagrange_multiplier == 0.0
    assert lag.pid_integral == 0.0 and lag.previous_mean is None
    lag._end_episode(0.0)                                   # mean 19/20 = 0.95, e = 0.85
    assert lag.pid_integral == pytest.approx(0.85)
    assert lag.lagrange_multiplier == pytest.approx(k_p * 0.85 + k_i * 0.85)
    lag._end_episode(3.0)                                   # drops a 1, adds a 3: mean 1.05
    assert lag.recent_costs == [1.0] * 18 + [0.0, 3.0]
    assert lag.pid_integral == pytest.approx(0.85 + 0.95)
    assert lag.lagrange_multiplier == pytest.approx(k_p * 0.95 + k_i * 1.8 + k_d * 0.1)
    assert [e["multiplier"] for e in lag.dual_history][-2:] == [
        0.0, pytest.approx(k_p * 0.85 + k_i * 0.85)]       # the lambda in force DURING each


def test_the_learner_refuses_missing_settings_and_several_environments():
    with pytest.raises(ValueError, match="cost_limit"):
        SACLagrangian("MlpPolicy", StopToy(), pid_gains=(0.1, 2e-3, 0.0))
    with pytest.raises(ValueError, match="pid_gains"):
        SACLagrangian("MlpPolicy", StopToy(), cost_limit=1.0, multiplier_max=100.0,
                      cost_window=20)
    with pytest.raises(ValueError, match="gains >= 0"):
        SACLagrangian("MlpPolicy", StopToy(), cost_limit=1.0, pid_gains=(0.1, -1.0, 0.0),
                      multiplier_max=100.0, cost_window=20)
    from stable_baselines3.common.env_util import make_vec_env
    with pytest.raises(ValueError, match="one environment"):
        SACLagrangian("MlpPolicy", make_vec_env(StopToy, n_envs=2), cost_limit=1.0,
                      pid_gains=(0.1, 2e-3, 0.0), multiplier_max=100.0, cost_window=20)


# ---------------------------------------------------------------- the L2 configs
def _cfg(name):
    from pathlib import Path

    from pfbnav.config import load_config
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "experiments"
                       / f"{name}.yaml")


# Ruled 2026-10-27: L2 is d = 0.1 a priori; 0.05 and 0.2 are the exploratory sensitivity.
CONFIGS = {"L2_sac_lagrangian": ("L2_sac_lagrangian", 0.1),
           "L2_sac_lagrangian_d005": ("L2-d005", 0.05), "L2_sac_lagrangian_d020": ("L2-d020", 0.2)}


def _selected(cfg, candidate=1):
    """An L2 config with the given PID candidate in place of the committed one."""
    from pfbnav.config import _deep_merge
    return _deep_merge(cfg, {"lagrangian": {"pid_candidate": candidate}})


def test_the_l2_configs_carry_the_candidate_the_toy_control_selected():
    """A28b: every committed L2 config carries candidate 1, the first in declared order to pass the
    toy control on seeds 0-3 (2026-10-27, runs/validation/L2_TOY_PID1_20261027T18*); an OPEN or
    undeclared candidate is refused."""
    from pfbnav.config import _deep_merge
    for name in CONFIGS:
        cfg = _cfg(name)
        assert cfg["lagrangian"]["pid_candidate"] == 1
        assert LagrangianSettings.from_config(cfg).pid_candidate == 1
        with pytest.raises(ValueError, match="not selected"):
            LagrangianSettings.from_config(_deep_merge(cfg, {"lagrangian": {"pid_candidate": "OPEN"}}))
        with pytest.raises(ValueError, match="not declared"):
            LagrangianSettings.from_config(_deep_merge(cfg, {"lagrangian": {"pid_candidate": 4}}))


def test_each_l2_config_is_one_ruled_limit_and_shares_every_sac_hyperparameter():
    l1 = _cfg("L1_sac_shaped")
    for name, (experiment, d) in CONFIGS.items():
        cfg = _cfg(name)
        assert cfg["experiment"] == experiment
        settings = LagrangianSettings.from_config(_selected(cfg, 2))
        assert settings.cost_limit == d and settings.pid_candidate == 2
        assert (settings.multiplier_init, settings.multiplier_max,
                settings.cost_window) == (0.0, 100.0, 20)
        assert cfg["training"] == l1["training"]        # learned arms share SAC (ruled 10-26)
        assert cfg["reward"]["k_stop"] == 0.0 and cfg["reward"]["k_proximity"] == 0.0
        assert {k: v for k, v in cfg["reward"].items() if k not in ("k_stop", "k_proximity")} \
            == {k: v for k, v in l1["reward"].items() if k not in ("k_stop", "k_proximity")}
        assert cfg["filter"]["enabled"] is False


@pytest.mark.parametrize("change, match", [
    ({"lagrangian": {"cost_limit": 0.15}}, "not ruled"),
    ({"lagrangian": {"cost_limit": "OPEN"}}, "not set"),
    ({"lagrangian": {"multiplier_lr": 3e-4}}, "retired"),
    ({"lagrangian": {"multiplier_max": 10.0}}, "declared"),
    ({"lagrangian": {"multiplier_init": 1.0}}, "declared"),
    ({"lagrangian": {"cost_window": 10}}, "declared"),
    ({"reward": {"k_stop": 5.0}}, "unshaped"),
    ({"reward": {"k_proximity": 2.0}}, "unshaped"),
    ({"filter": {"enabled": True}}, "without the filter"),
])
def test_an_unruled_l2_config_is_refused(change, match):
    from pfbnav.config import _deep_merge
    cfg = _deep_merge(_selected(_cfg("L2_sac_lagrangian")), change)
    with pytest.raises(ValueError, match=match):
        LagrangianSettings.from_config(cfg)


def test_training_preflight_accepts_l2_only_with_a_ruled_limit():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "scripts" / "train.py"
    spec = importlib.util.spec_from_file_location("train_script", path)
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    from pfbnav.config import _deep_merge
    assert len(script.preflight(_selected(_cfg("L2_sac_lagrangian")))) == 3
    with pytest.raises(SystemExit, match="cost_limit is not set"):
        script.preflight(_deep_merge(_selected(_cfg("L2_sac_lagrangian")),
                                     {"lagrangian": {"cost_limit": "OPEN"}}))
    assert len(script.preflight(_cfg("L2_sac_lagrangian"))) == 3      # as committed: candidate 1
    with pytest.raises(SystemExit, match="pid_candidate is not selected"):
        script.preflight(_selected(_cfg("L2_sac_lagrangian"), "OPEN"))
    # The learner the script builds follows the config (a ruled limit: the script builds no toy).
    ruled = {**TOY_CFG, "lagrangian": {**TOY_CFG["lagrangian"], "cost_limit": 0.1}}
    assert isinstance(script.make_learner(StopToy(), ruled, 0), SACLagrangian)

    plain = {k: v for k, v in TOY_CFG.items() if k != "lagrangian"}
    assert type(script.make_learner(StopToy(), plain, 0)).__name__ == "SAC"


def test_the_dual_history_is_json_serialisable(runs):
    _, _, lag, _ = runs
    json.dumps(lag.dual_history)
