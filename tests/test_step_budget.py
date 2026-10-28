"""Known-answer tests for the per-step budget arithmetic (hard rule 6)."""
from pathlib import Path

import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.eval.step_budget import (
    aggregate_from_components,
    compare_components,
    headroom,
    per_step_budget_s,
)

REPO = Path(__file__).resolve().parents[1]
CFG = load_config(REPO / "configs" / "base.yaml")


def test_the_budget_is_the_instance_count_over_the_aggregate_target():
    assert per_step_budget_s(25.0, 4) == pytest.approx(0.160)
    assert per_step_budget_s(25.0, 1) == pytest.approx(0.040)
    assert per_step_budget_s(25.0, 8) == pytest.approx(0.320)


def test_the_withdrawn_target_is_what_gave_the_old_40_ms():
    # Recorded so the stale figure cannot come back unexplained: 40 ms per step at four
    # instances is the >= 100 steps/s target, which was withdrawn on 2026-09-20.
    assert per_step_budget_s(100.0, 4) == pytest.approx(0.040)


def test_the_budget_refuses_nonsense():
    with pytest.raises(ValueError):
        per_step_budget_s(0.0, 4)
    with pytest.raises(ValueError):
        per_step_budget_s(25.0, 0)


def test_headroom_reports_the_fraction_used_and_the_verdict():
    h = headroom(0.102, 0.160)
    assert h["fraction_used"] == pytest.approx(0.6375)
    assert h["margin_s"] == pytest.approx(0.058)
    assert h["within"]
    assert not headroom(0.200, 0.160)["within"]


def test_headroom_at_the_boundary_is_within():
    assert headroom(0.160, 0.160)["within"]
    assert not headroom(0.1600001, 0.160)["within"]


def test_aggregate_from_components_is_instances_over_the_step():
    # A 100 ms step on four instances stepping together: 40 steps/s.
    assert aggregate_from_components({"step": 0.08, "scan": 0.02}, 4) == pytest.approx(40.0)
    assert aggregate_from_components({"step": 0.08, "scan": 0.02}, 4,
                                     parallel=False) == pytest.approx(10.0)


def test_compare_components_sorts_by_what_moved_and_counts_an_added_component():
    rows = compare_components({"step": 0.076, "contact": 0.0055},
                              {"step": 0.070, "contact": 0.0055, "scan": 0.017})
    assert rows[0]["component"] == "scan"
    assert rows[0]["before_ms"] == 0.0
    assert rows[0]["change_ms"] == pytest.approx(17.0)
    assert rows[1]["component"] == "step"
    assert rows[1]["change_ms"] == pytest.approx(-6.0)
    contact = next(r for r in rows if r["component"] == "contact")
    assert contact["change_ms"] == pytest.approx(0.0)
    # Shares are of the whole step on each side, so an added component moves the others.
    assert contact["before_share"] > contact["after_share"]


def test_the_budget_is_derived_from_the_configured_target_not_restated():
    """The environment's budget comes from the configured aggregate target and instance
    count; no millisecond figure is stored anywhere."""
    budget = CFG["env"]["step_budget"]
    target = float(budget["aggregate_steps_per_s"])
    instances = int(budget["instances"])
    assert per_step_budget_s(target, instances) == pytest.approx(instances / target)
    text = (REPO / "configs" / "base.yaml").read_text()
    assert "step_budget_ms" not in text and "per_step_ms" not in text


def test_g2s_threshold_is_not_yet_committed_to_preregistration():
    """G2 cannot run until its threshold is preregistered (CLAUDE.md: thresholds for gates
    live in preregistration/gates.yaml). The target is derived in docs/requirements.md and
    carried in the config meanwhile; this test is the reminder, and flips when it lands."""
    gates = yaml.safe_load((REPO / "preregistration" / "gates.yaml").read_text())
    g2 = gates["G2_compute_budget"]
    if "min_steps_per_second" in g2:
        assert float(g2["min_steps_per_second"]) == float(
            CFG["env"]["step_budget"]["aggregate_steps_per_s"]), (
            "the committed threshold and the configured target disagree")
    else:
        pytest.skip("G2's min_steps_per_second is not preregistered yet (WP1 open item)")
