"""Config overrides: known answers, the three guards, and the flag on the two scripts (hard rule 6)."""
import subprocess
import sys
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.utils.overrides import apply_overrides, parse_item

ROOT = Path(__file__).resolve().parents[1]


def test_values_are_read_as_numbers_bools_strings_and_lists():
    assert parse_item("a.b=3") == (["a", "b"], 3)
    assert parse_item("lr=3e-4") == (["lr"], 3e-4)            # YAML would read 3e-4 as a string
    assert parse_item("x=0.5") == (["x"], 0.5)
    assert parse_item("f=true") == (["f"], True)
    assert parse_item("name=L1_smoke") == (["name"], "L1_smoke")
    assert parse_item("net=[64, 64]") == (["net"], [64, 64])
    with pytest.raises(ValueError):
        parse_item("no_equals_sign")
    with pytest.raises(ValueError):
        parse_item("a..b=1")


def test_overrides_apply_in_order_without_touching_the_input_and_are_recorded():
    cfg = {"training": {"total_timesteps": 2000000, "device": "cuda"}, "experiment": "L1"}
    out = apply_overrides(cfg, ["training.total_timesteps=100000", "experiment=DIAG_L1_smoke"])
    assert out["training"]["total_timesteps"] == 100000 and out["experiment"] == "DIAG_L1_smoke"
    assert cfg["training"]["total_timesteps"] == 2000000 and "cli_overrides" not in cfg
    assert out["cli_overrides"] == ["training.total_timesteps=100000", "experiment=DIAG_L1_smoke"]
    assert apply_overrides(cfg, []) is cfg                    # no overrides: the same object


def test_a_missing_key_and_a_value_of_another_kind_are_errors_not_new_keys():
    cfg = {"training": {"total_timesteps": 5, "device": "cuda", "net_arch": [256, 256]}, "reward": {"k": 1.0}}
    with pytest.raises(ValueError, match="no key"):
        apply_overrides(cfg, ["training.total_timestep=1"])    # a typo
    with pytest.raises(ValueError, match="no key"):
        apply_overrides(cfg, ["trainin.total_timesteps=1"])
    with pytest.raises(ValueError, match="number"):
        apply_overrides(cfg, ["training.total_timesteps=many"])
    with pytest.raises(ValueError, match="list"):
        apply_overrides(cfg, ["training.net_arch=64"])
    assert apply_overrides(cfg, ["reward.k=2"], report=lambda m: None)["reward"]["k"] == 2   # int for a float is a number


def test_a_ruled_section_warns_and_an_unruled_one_does_not():
    cfg = {"reward": {"k_progress": 5.0}, "training": {"batch_size": 256}}
    seen: list[str] = []
    apply_overrides(cfg, ["reward.k_progress=4.0"], report=seen.append)
    assert len(seen) == 1 and "reward.k_progress" in seen[0] and "decision_log" in seen[0]
    seen.clear()
    apply_overrides(cfg, ["training.batch_size=128"], report=seen.append)
    assert seen == []


def test_the_real_base_config_accepts_the_smoke_overrides():
    cfg = load_config(ROOT / "configs" / "experiments" / "L1_sac_shaped.yaml")
    out = apply_overrides(cfg, ["training.total_timesteps=100000", "experiment=DIAG_L1_smoke"])
    assert out["training"]["total_timesteps"] == 100000 and out["training"]["batch_size"] == 256


@pytest.mark.parametrize("script", ["train.py", "evaluate.py"])
def test_the_scripts_offer_the_set_flag(script):
    out = subprocess.run([sys.executable, str(ROOT / "scripts" / script), "--help"],
                         capture_output=True, text=True, check=True, cwd=ROOT).stdout
    assert "--set KEY=VALUE" in out
