"""Method IDs are never reused, and every confirmatory list names only active methods.

Standing rule (supervisor, 2026-10-15; hard rule 20). B4 and B5 were retired by A16 and never
implemented, but frozen documents name them, so they stay in `configs/methods.yaml` as retired and
no list that decides anything may use them. Until A16 is applied to `preregistration/endpoints.yaml`
and relocked, the endpoint test below fails by design: the frozen file still names B4.
"""
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
REGISTRY = yaml.safe_load((REPO / "configs" / "methods.yaml").read_text(encoding="utf-8"))["methods"]
ACTIVE = {k for k, v in REGISTRY.items() if v["status"] == "active"}
RETIRED = {k for k, v in REGISTRY.items() if v["status"] == "retired"}


def test_every_id_has_one_status_and_retired_ones_say_when():
    assert ACTIVE.isdisjoint(RETIRED) and ACTIVE | RETIRED == set(REGISTRY)
    assert all("retired" in REGISTRY[k] for k in RETIRED)
    assert {"B4", "B5"} <= RETIRED and {"B6", "B7"} <= ACTIVE


def test_the_power_analysis_configs_name_only_active_comparators():
    for name in ("h1_power.yaml", "h1_rate_ratio.yaml", "power_corrected.yaml"):
        text = yaml.safe_load((REPO / "configs" / "analysis" / name).read_text(encoding="utf-8"))
        found = _lists_named(text, "classical") + _lists_named(text, "learned")
        assert found, f"{name}: no comparator list found"
        for ids in found:
            assert set(ids) <= ACTIVE, f"{name} names {sorted(set(ids) - ACTIVE)}"


def test_the_preregistered_endpoints_name_only_active_methods():
    endpoints = yaml.safe_load((REPO / "preregistration" / "endpoints.yaml")
                               .read_text(encoding="utf-8"))
    lists = {"H1": endpoints["H1"]["comparators"], "H1N": endpoints["H1N"]["comparators"],
             "narrow_rule": endpoints["narrow_rule"]["methods"]}
    if "H1prime" in endpoints:                  # A27, the contingency hypothesis
        lists["H1prime"] = endpoints["H1prime"]["comparators"]
    for where, ids in lists.items():
        assert set(ids) <= ACTIVE, f"{where} names retired or unknown {sorted(set(ids) - ACTIVE)}"


def test_the_a28_cut_order_names_only_active_methods_or_declared_analyses():
    """A28 (ruled 2026-10-27): the cut order puts the L2 sensitivity first; every entry is an active
    method ID or one of the two declared analyses (the PFB-(a) ablation of H3, the stop_active
    ablation), and no confirmatory arm is cut."""
    for path in (REPO / "docs" / "pending" / "gates_A28.yaml",
                 REPO / "docs" / "pending" / "gates_A28a.yaml",
                 REPO / "preregistration" / "gates.yaml"):
        g2 = yaml.safe_load(path.read_text(encoding="utf-8"))["G2_compute_budget"]
        order = g2["reduction_order"]
        if path.parent.name == "preregistration" and "L2-d005" not in order:
            continue                            # A28 not applied yet: the frozen file is older
        assert order[:2] == ["L2-d005", "L2-d020"]
        assert set(order) - {"PFB-(a)", "stop_active"} <= ACTIVE
        assert not {"L1", "L2", "L4", "L5"} & set(order)
        assert {"L2-d005", "L2-d020"} <= ACTIVE
        assert REGISTRY["L2-d005"]["role"] == REGISTRY["L2-d020"]["role"] == ["exploratory"]
        if "never_cut" in g2:                   # A28a: no confirmatory arm or comparator is cut
            assert set(g2["never_cut"]) == {"L1", "L2", "L4", "L5", "B1", "B2", "B3", "B6", "B7"}
            assert not set(g2["never_cut"]) & set(order)


def _lists_named(node, key):
    """Every list stored under `key` anywhere in a nested mapping."""
    out = []
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key and isinstance(v, list):
                out.append(v)
            else:
                out += _lists_named(v, key)
    elif isinstance(node, list):
        for v in node:
            out += _lists_named(v, key)
    return out
