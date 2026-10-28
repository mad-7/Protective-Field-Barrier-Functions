"""The alpha runner's refusals and config handling (`scripts/tune_alpha.py`); its selection and the
B2-B7 comparison are `pfbnav/eval/tuning.py`'s, tested in tests/test_tuning.py."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from pfbnav.config import load_config

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("tune_alpha", REPO / "scripts" / "tune_alpha.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_b7_must_carry_b2s_tuned_parameters_exactly(mod):
    cfg = load_config(REPO / "configs" / "base.yaml")
    tuned = {k: float(v) for k, v in cfg["baselines"]["regulated_pure_pursuit"].items()}
    summary = {"controller": "regulated_pure_pursuit", "selected_parameters": tuned}
    assert mod.b2_problem(cfg, summary) is None
    off = {**tuned, "desired_speed": tuned["desired_speed"] * (1 + 1e-6)}
    assert "not the tuned ones" in mod.b2_problem(cfg, {**summary, "selected_parameters": off})
    fewer = {k: v for k, v in tuned.items() if k != "k_omega"}
    assert "differ" in mod.b2_problem(cfg, {**summary, "selected_parameters": fewer})
    assert "is for" in mod.b2_problem(cfg, {**summary, "controller": "pure_pursuit"})


def test_with_alpha_enables_the_filter_and_sets_only_alpha(mod):
    cfg = load_config(REPO / "configs" / "base.yaml")
    on = mod.with_alpha(cfg, 5.0)
    assert on["filter"]["enabled"] is True and on["filter"]["alpha"] == 5.0
    assert {k for k in cfg["filter"] if cfg["filter"][k] != on["filter"][k]} == {"enabled", "alpha"}
    assert cfg["filter"]["enabled"] is False


def test_the_grid_is_read_from_the_preregistration():
    import yaml
    endpoints = yaml.safe_load((REPO / "preregistration" / "endpoints.yaml").read_text(
        encoding="utf-8"))
    assert endpoints["tuning"]["pfb_alpha"]["grid_per_s"] == [1, 2, 5, 10, 20]
