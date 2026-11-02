"""The derived proximity-shaping weight (ruled 2026-11-01): known answers, the configs, the refusal."""
import importlib.util
from pathlib import Path

import pytest

from pfbnav.config import _deep_merge, load_config
from pfbnav.envs.reward import RewardWeights, proximity_cost, step_reward
from pfbnav.training.shaping import derive_k_proximity, k_proximity_from_config, shaping_problem

REPO = Path(__file__).resolve().parents[1]
BASE = load_config(REPO / "configs" / "base.yaml")
ARGS = {"k_progress": 5.0, "speed": 0.3, "dt": 0.05, "share": 0.25, "half_width": 0.2086,
        "clearance": 0.1987, "threshold": 0.5}


def test_known_answer_by_hand():
    """progress/step 5 x 0.3 x 0.05 = 0.075; 25 % = 0.01875; wall abeam at 0.4073 m, cost
    (0.5 - 0.4073) / 0.5 = 0.1854; k = 0.01875 / 0.1854."""
    assert derive_k_proximity(**ARGS) == pytest.approx(0.01875 / 0.1854, rel=1e-12)
    assert derive_k_proximity(**ARGS) == pytest.approx(0.101132686, rel=1e-8)


def test_the_ruled_ratio_holds_through_the_reward_itself():
    """Through `step_reward` and `proximity_cost` (the environment's own functions): one step at
    0.3 m/s on the narrow floor's centreline, the proximity term is -25 % of the progress term."""
    k = derive_k_proximity(**ARGS)
    w = RewardWeights.from_config(_deep_merge(BASE, {"reward": {"k_proximity": k}}))
    cost = proximity_cost(0.2086 + 0.1987, 0.5)
    _, terms = step_reward(w, progress_m=0.3 * 0.05, stop_triggered=False, intervention=0.0,
                           proximity=cost)
    assert terms["proximity"] == pytest.approx(-0.25 * terms["progress"], rel=1e-12)


def test_scaling_and_refusals():
    assert derive_k_proximity(**{**ARGS, "share": 0.5}) == pytest.approx(
        2 * derive_k_proximity(**ARGS))
    with pytest.raises(ValueError, match="beyond"):
        derive_k_proximity(**{**ARGS, "clearance": 0.4})          # wall outside the threshold
    with pytest.raises(ValueError, match="positive"):
        derive_k_proximity(**{**ARGS, "speed": 0.0})


def test_the_config_reads_the_ruled_inputs():
    sh = BASE["reward"]["shaping"]
    assert (sh["proximity_share"], sh["reference_speed"], sh["reference_stratum"]) == (
        0.25, 0.3, "narrow")
    assert k_proximity_from_config(BASE) == pytest.approx(derive_k_proximity(**ARGS), rel=1e-12)


def test_every_shaped_experiment_carries_the_derived_value_and_unshaped_ones_zero():
    configs = sorted((REPO / "configs" / "experiments").rglob("*.yaml"))
    shaped = []
    for path in configs:
        cfg = load_config(path)
        assert shaping_problem(cfg) is None, path.name
        if float(cfg["reward"]["k_proximity"]) > 0.0:
            shaped.append(cfg["experiment"])
    assert "L1_sac_shaped" in shaped and "DIAG_L1_unicycle" in shaped
    for name in ("L2_sac_lagrangian", "L5_sac_pfb_unshaped"):
        assert float(load_config(REPO / "configs" / "experiments" / f"{name}.yaml")
                     ["reward"]["k_proximity"]) == 0.0


def test_training_refuses_the_old_underived_value():
    spec = importlib.util.spec_from_file_location("train_script", REPO / "scripts" / "train.py")
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    l1 = load_config(REPO / "configs" / "experiments" / "L1_sac_shaped.yaml")
    with pytest.raises(SystemExit, match="not the derived value"):
        script.preflight(_deep_merge(l1, {"reward": {"k_proximity": 2.0}}))
    assert len(script.preflight(l1)) == 3
