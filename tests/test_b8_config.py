"""B8 = B6 + the PFB at the frozen alpha, and nothing else (2026-11-09; configs/methods.yaml)."""
from pathlib import Path

import yaml

from pfbnav.config import load_config
from pfbnav.utils.filter_ruling import filter_problem

ROOT = Path(__file__).resolve().parents[1]


def _diff(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += _diff(a.get(k, "<missing>"), b.get(k, "<missing>"), f"{path}.{k}" if path else k)
        return out
    return [] if a == b else [path]


def test_b8_differs_from_base_only_in_its_name_and_the_filter_switch():
    b8 = load_config(ROOT / "configs" / "experiments" / "B8_mppi_pfb.yaml")
    base = load_config(ROOT / "configs" / "base.yaml")
    assert _diff(b8, base) == ["experiment", "filter.enabled"]
    assert b8["filter"]["enabled"] is True and base["filter"]["enabled"] is False
    assert b8["baselines"]["mppi"] == base["baselines"]["mppi"]      # B6's tuned parameters, untouched
    assert filter_problem(b8) is None                                 # on, at the frozen alpha


def test_b8_is_registered_as_a_new_exploratory_method():
    methods = yaml.safe_load((ROOT / "configs" / "methods.yaml").read_text(encoding="utf-8"))
    found = methods.get("methods", methods)["B8"]
    assert found["status"] == "active" and "HF-exploratory" in found["role"]


def test_b9_differs_from_base_only_in_its_name_and_the_filter_switch():
    b9 = load_config(ROOT / "configs" / "experiments" / "B9_dwa_pfb.yaml")
    base = load_config(ROOT / "configs" / "base.yaml")
    assert _diff(b9, base) == ["experiment", "filter.enabled"]
    assert b9["baselines"]["dwa"] == base["baselines"]["dwa"]         # B3's tuned parameters, untouched
    assert filter_problem(b9) is None
    methods = yaml.safe_load((ROOT / "configs" / "methods.yaml").read_text(encoding="utf-8"))["methods"]
    assert methods["B9"]["status"] == "active" and "HF-exploratory" in methods["B9"]["role"]
