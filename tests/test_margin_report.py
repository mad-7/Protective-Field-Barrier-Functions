"""Known answers for merging and summarising the margin quantification (ruled 2026-10-13)."""
import pytest

from pfbnav.eval.margin_report import (
    complete_scenarios,
    merge_runs,
    parse_row,
    smallest_meeting,
    summarise,
)

PAIRS = {(0.03, 0.03), (0.07, 0.03)}
CFG = {"robot": {"half_width": 0.2086},
       "planner_margins": {"lethal_margins": [0.03, 0.07], "soft_margins": [0.03]}}


def row(seed, lethal, breached=0, where="bend", status="ok", stratum="narrow", s="tune"):
    r = {"set": s, "seed": str(seed), "stratum": stratum, "lethal_margin": lethal,
         "soft_margin": 0.03, "status": status}
    if status == "ok":
        r.update(samples=100, breached_samples=breached, offset_breached_samples=breached,
                 min_clearance_m=-0.01 if breached else 0.05,
                 **{f"breached_{c}": (breached if c == where else 0)
                    for c in ("start", "goal", "bend", "straight")})
    return r


def test_parse_row_types_numbers_and_keeps_empty_cells_none():
    r = parse_row({"set": "t", "seed": "1", "lethal_margin": "0.07", "samples": "12",
                   "min_clearance_m": ""})
    assert r["lethal_margin"] == 0.07 and r["samples"] == 12 and r["min_clearance_m"] is None


def test_only_scenarios_with_every_pair_are_complete():
    rows = [row(1, 0.03), row(1, 0.07), row(2, 0.03)]
    complete, incomplete = complete_scenarios(rows, PAIRS)
    assert list(complete) == [("tune", "1")] and incomplete == [("tune", "2")]


def test_merging_an_interrupted_run_with_its_completion():
    first = [row(1, 0.03), row(1, 0.07), row(2, 0.03)]            # stopped inside scenario 2
    second = [row(2, 0.03), row(2, 0.07, breached=3)]
    merged = merge_runs([first, second], PAIRS)
    assert merged["scenarios"] == [("tune", "1"), ("tune", "2")]
    assert merged["incomplete"] == [] and merged["duplicates"] == 0
    assert len(merged["rows"]) == 4


def test_a_scenario_complete_in_two_runs_must_agree():
    a = [row(1, 0.03), row(1, 0.07)]
    assert merge_runs([a, [dict(r) for r in a]], PAIRS)["duplicates"] == 1
    b = [row(1, 0.03), row(1, 0.07, breached=5)]
    with pytest.raises(ValueError, match="differs"):
        merge_runs([a, b], PAIRS)


def test_summary_counts_breaches_by_location_and_the_criterion():
    rows = [row(1, 0.03, breached=4, where="straight"), row(2, 0.03, breached=2, where="goal"),
            row(1, 0.07), row(2, 0.07)]
    quant = [{"lethal_margin": 0.03, "soft_margin": 0.03, "failures": []},
             {"lethal_margin": 0.07, "soft_margin": 0.03, "failures": []}]
    table = {(e["lethal_margin"], e["soft_margin"]): e for e in summarise(rows, quant, CFG)}
    today = table[(0.03, 0.03)]
    assert today["breaching_routes"] == 2 and not today["criterion_holds"]
    assert today["breaching_routes_by_location"] == {"start": 0, "goal": 1, "bend": 0,
                                                     "straight": 1}
    assert today["breaching_routes_outside_goal_approach"] == 1
    assert table[(0.07, 0.03)]["criterion_holds"]
    assert smallest_meeting(list(table.values())) == (0.07, 0.03)


def test_an_unplannable_route_or_a_quantisation_failure_breaks_plannability():
    rows = [row(1, 0.03), row(1, 0.07, status="no_path")]
    quant = [{"lethal_margin": 0.03, "soft_margin": 0.03, "failures": [{"axis": "x"}]},
             {"lethal_margin": 0.07, "soft_margin": 0.03, "failures": []}]
    table = {(e["lethal_margin"], e["soft_margin"]): e for e in summarise(rows, quant, CFG)}
    assert not table[(0.03, 0.03)]["plannability_holds"]          # the quantisation aisle
    assert not table[(0.07, 0.03)]["plannability_holds"]          # the unplannable route
    assert smallest_meeting(list(table.values())) is None
