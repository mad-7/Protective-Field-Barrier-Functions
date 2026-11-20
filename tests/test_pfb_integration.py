"""Every classical baseline has a PFB-wrapped form that differs from it ONLY in the filter switch, and
each baseline's parameters live only in configs/baselines/ (restructure, 2026-11-20)."""
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
def test_the_loaded_config_takes_each_baseline_from_its_file(name):
    """configs/baselines/ is the only home of a baseline's parameters: base.yaml names the files, the
    loader puts each file's params under cfg['baselines'][key]."""
    raw = yaml.safe_load((ROOT / "configs" / "base.yaml").read_text(encoding="utf-8"))
    key = MIRRORS[name]
    assert key not in raw["baselines"], f"{key} is also written in base.yaml"
    file = yaml.safe_load((ROOT / "configs" / "baselines" / f"{name}.yaml").read_text(encoding="utf-8"))
    assert file["base_key"] == key and f"baselines/{name}.yaml" in raw["baseline_files"]
    assert load_config(ROOT / "configs" / "base.yaml")["baselines"][key] == file["params"]


def test_the_shared_recovery_and_route_cap_come_from_their_file():
    raw = yaml.safe_load((ROOT / "configs" / "base.yaml").read_text(encoding="utf-8"))
    assert "recovery" not in raw["baselines"] and "route_cap" not in raw["baselines"]
    shared = yaml.safe_load((ROOT / "configs" / "baselines" / "shared.yaml").read_text(encoding="utf-8"))
    cfg = load_config(ROOT / "configs" / "base.yaml")
    assert cfg["baselines"]["recovery"] == shared["recovery"]
    assert cfg["baselines"]["route_cap"] == shared["route_cap"]
    assert "baseline_files" not in cfg


def test_a_parameter_written_in_two_places_is_refused(tmp_path):
    """Positive control: a second home for a baseline raises instead of silently picking one."""
    (tmp_path / "baselines").mkdir()
    (tmp_path / "baselines" / "x.yaml").write_text(
        "controller: c\nbase_key: pure_pursuit\nparams:\n  a: 1\n", encoding="utf-8")
    (tmp_path / "dup.yaml").write_text(
        "baseline_files: [baselines/x.yaml]\nbaselines:\n  pure_pursuit: {a: 2}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="defined twice"):
        load_config(tmp_path / "dup.yaml")


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
