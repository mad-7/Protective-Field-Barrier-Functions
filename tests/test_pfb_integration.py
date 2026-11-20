"""Every classical baseline has a PFB-wrapped form that differs from it ONLY in the filter switch, and the
per-baseline parameter files mirror configs/base.yaml exactly (restructure, 2026-11-20)."""
from pathlib import Path

import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.utils.filter_ruling import filter_problem

ROOT = Path(__file__).resolve().parents[1]

MIRRORS = {"B1_pure_pursuit": "pure_pursuit", "B2_regulated_pure_pursuit": "regulated_pure_pursuit",
           "B3_dwa": "dwa", "B6_field_aware_mppi": "mppi"}


def _diff(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += _diff(a.get(k, "<missing>"), b.get(k, "<missing>"), f"{path}.{k}" if path else k)
        return out
    return [] if a == b else [path]


@pytest.mark.parametrize("overlay", ["B7_rpp_pfb", "B10_pp_pfb"])
def test_overlay_differs_from_base_only_in_its_name_and_the_filter_switch(overlay):
    cfg = load_config(ROOT / "configs" / "experiments" / f"{overlay}.yaml")
    base = load_config(ROOT / "configs" / "base.yaml")
    assert _diff(cfg, base) == ["experiment", "filter.enabled"]
    assert cfg["filter"]["enabled"] is True and base["filter"]["enabled"] is False
    assert cfg["baselines"] == base["baselines"]
    assert filter_problem(cfg) is None


def test_b10_is_registered_and_never_confirmatory():
    methods = yaml.safe_load((ROOT / "configs" / "methods.yaml").read_text(encoding="utf-8"))["methods"]
    assert methods["B10"]["status"] == "active" and "HF-exploratory" in methods["B10"]["role"]


@pytest.mark.parametrize("name", sorted(MIRRORS))
def test_baseline_param_file_mirrors_base_exactly(name):
    base = yaml.safe_load((ROOT / "configs" / "base.yaml").read_text(encoding="utf-8"))["baselines"]
    mirror = yaml.safe_load((ROOT / "configs" / "baselines" / f"{name}.yaml").read_text(encoding="utf-8"))
    assert mirror["base_key"] == MIRRORS[name]
    assert mirror["params"] == base[MIRRORS[name]]


def test_shared_recovery_and_route_cap_mirror_base():
    base = yaml.safe_load((ROOT / "configs" / "base.yaml").read_text(encoding="utf-8"))["baselines"]
    shared = yaml.safe_load((ROOT / "configs" / "baselines" / "shared.yaml").read_text(encoding="utf-8"))
    assert shared["recovery"] == base["recovery"] and shared["route_cap"] == base["route_cap"]


def test_mirror_test_detects_drift():
    """Positive control: a perturbed copy must not equal base."""
    base = yaml.safe_load((ROOT / "configs" / "base.yaml").read_text(encoding="utf-8"))["baselines"]
    mirror = yaml.safe_load((ROOT / "configs" / "baselines" / "B1_pure_pursuit.yaml").read_text(encoding="utf-8"))
    mirror["params"]["cruise_speed"] += 1e-6
    assert mirror["params"] != base["pure_pursuit"]


def test_with_pfb_reproduces_every_overlay_file():
    """The function and the overlay files are the same switch."""
    from pfbnav.integration import with_pfb
    base = load_config(ROOT / "configs" / "base.yaml")
    wrapped = with_pfb(base)
    for name in ("B7_rpp_pfb", "B8_mppi_pfb", "B9_dwa_pfb", "B10_pp_pfb"):
        overlay = load_config(ROOT / "configs" / "experiments" / f"{name}.yaml")
        assert _diff(wrapped, overlay) == ["experiment"], name
    assert _diff(wrapped, base) == ["filter.enabled"]


def test_with_pfb_refuses_another_alpha_and_pair_differs_in_the_switch_only():
    from pfbnav.integration import pfb_pair, with_pfb
    base = load_config(ROOT / "configs" / "base.yaml")
    bad = {**base, "filter": {**base["filter"], "alpha": 2.0}}
    with pytest.raises(ValueError, match="frozen alpha"):
        with_pfb(bad)
    alone, wrapped = pfb_pair(base)
    assert _diff(alone, wrapped) == ["filter.enabled"]


def test_the_pfb_flag_applies_and_leaves_the_input_alone():
    from pfbnav.utils.cli import apply_pfb
    base = load_config(ROOT / "configs" / "base.yaml")
    assert apply_pfb(base, "config") is base
    assert apply_pfb(base, "on")["filter"]["enabled"] is True
    assert base["filter"]["enabled"] is False
    with pytest.raises(ValueError):
        apply_pfb(base, "maybe")
