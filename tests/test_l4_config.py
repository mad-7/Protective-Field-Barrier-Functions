"""L4 = L5 + the derived proximity shaping, and nothing else (ruled 2026-11-07, item 4)."""
from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.training.shaping import k_proximity_from_config

EXP = Path(__file__).resolve().parents[1] / "configs" / "experiments"


def _diff(a, b, path=""):
    """Every dotted key whose resolved value differs between two configs (missing keys included)."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += _diff(a.get(k, "<missing>"), b.get(k, "<missing>"), f"{path}.{k}" if path else k)
        return out
    return [] if a == b else [path]


def test_l4_differs_from_l5_only_in_its_name_and_the_derived_shaping():
    l4 = load_config(EXP / "L4_sac_pfb_shaped.yaml")
    l5 = load_config(EXP / "L5_sac_pfb_unshaped.yaml")
    assert _diff(l4, l5) == ["experiment", "reward.k_proximity"]
    assert l5["reward"]["k_proximity"] == 0.0
    assert l4["reward"]["k_proximity"] == pytest.approx(k_proximity_from_config(l4), rel=1e-12)
    assert l4["reward"]["k_proximity"] == load_config(EXP / "L1_sac_shaped.yaml")["reward"]["k_proximity"]
    assert l4["filter"]["enabled"] and l4["filter"]["alpha"] == l5["filter"]["alpha"] == 1.0


def test_the_diff_helper_sees_nested_and_missing_keys():
    assert _diff({"a": {"b": 1, "c": 2}}, {"a": {"b": 1, "c": 3}, "d": 0}) == ["a.c", "d"]
    assert _diff({"x": [1, 2]}, {"x": [1, 2]}) == []
