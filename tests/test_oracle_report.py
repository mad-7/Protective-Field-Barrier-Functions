"""Known answers for the oracle criterion's summary and declared decisions (ruled 2026-10-15)."""
import pytest

from pfbnav.eval.oracle_report import classify, summarise, summarise_soft


def route(seed, soft=0.08, stratum="narrow", *, status="ok", start=1, goal=1, band0=0, bend=0,
          stops=0, length=10.0, oracle=30.0, offset=0):
    return {"set": "s", "seed": str(seed), "stratum": stratum, "lethal_margin": 0.07,
            "soft_margin": soft, "status": status, "path_length_m": length, "start_valid": start,
            "goal_valid": goal, "band0_breached_samples": band0, "band0_breach_bend": bend,
            "unavoidable_stops": stops, "oracle_time_s": oracle if band0 == 0 else None,
            "constant_time_s": length / 0.40, "offset_band0_breached_samples": offset}


def test_routes_are_classed_start_first_then_goal_then_residual():
    assert classify(route(1, status="no_path")) == "unplannable"
    assert classify(route(1, start=0, goal=0, band0=3)) == "start_invalid"
    assert classify(route(1, goal=0, band0=3)) == "goal_invalid"
    assert classify(route(1, band0=2, bend=2, stops=1)) == "residual"
    assert classify(route(1)) == "clear"


def test_the_residual_rule_is_per_stratum_at_two_percent():
    ok = [route(i) for i in range(98)] + [route(98, band0=1, stops=1), route(99, start=0, band0=1)]
    assert summarise_soft(ok)["residual_rule"] == "re-sample"          # 1 % residual
    bad = [route(i) for i in range(97)] + [route(97 + k, band0=1, stops=1) for k in range(3)]
    wide = [route(200 + i, stratum="wide") for i in range(100)]
    out = summarise_soft(bad + wide)
    assert out["per_stratum"]["narrow"]["residual_share"] == pytest.approx(0.03)
    assert out["residual_rule"] == "stop"                              # one stratum over


def test_floor_time_ratio_and_offset_information():
    rows = [route(1, length=10.0, oracle=30.0), route(2, length=10.0, band0=4, bend=4, stops=2),
            route(3, length=20.0, oracle=55.0, offset=5)]
    out = summarise_soft(rows)["all"]
    assert out["unavoidable_stops_per_100m"] == pytest.approx(100 * 2 / 40)
    assert out["mission_time_ratio_median"] == pytest.approx((30 / 25 + 55 / 50) / 2)
    assert out["bend_breach_routes"] == 1 and out["offset_band0_breach_routes"] == 1


def test_the_soft_margin_with_fewest_bend_breaches_is_chosen_among_plannable_ones():
    rows = ([route(1, 0.03, band0=1, bend=1), route(2, 0.03, band0=1, bend=1)]
            + [route(1, 0.08, band0=1, bend=1), route(2, 0.08)]
            + [route(1, 0.13), route(2, 0.13, status="no_path")]      # fewer, but loses a route
            + [route(1, 0.20, band0=1, bend=1), route(2, 0.20)])
    out = summarise(rows, [0.03, 0.08, 0.13, 0.20])
    assert out["chosen_soft"] == 0.08                                  # tie with 0.20: smaller
    assert out["by_soft"][0.13]["plannable_everywhere"] is False


# --- the free-orientation oracle -----------------------------------------------------------------

def free(seed, soft=0.08, stratum="narrow", *, status="ok", start=1, goal=1, trav=1, stops=0,
         bend=0, tangent=0, length=10.0, offset_trav=1, oracle=None):
    return {"set": "s", "seed": str(seed), "stratum": stratum, "lethal_margin": 0.07,
            "soft_margin": soft, "status": status, "path_length_m": length, "start_valid": start,
            "goal_valid": goal, "traversable": trav, "unavoidable_stops": stops, "stops_bend": bend,
            "tangent_stops": tangent, "offset_traversable": offset_trav, "oracle_time_s": oracle,
            "constant_time_s": length / 0.40}


def test_free_routes_are_classed_like_the_tangent_ones():
    from pfbnav.eval.oracle_report import classify_free
    assert classify_free(free(1, status="no_path")) == "unplannable"
    assert classify_free(free(1, start=0, goal=0, trav=0)) == "start_invalid"
    assert classify_free(free(1, goal=0, trav=0)) == "goal_invalid"
    assert classify_free(free(1, trav=0, stops=1, bend=1)) == "residual"
    assert classify_free(free(1)) == "clear"


def test_free_summary_reports_both_floors_and_applies_the_same_rule():
    from pfbnav.eval.oracle_report import summarise_free_soft
    rows = ([free(i, tangent=1) for i in range(98)]                     # A1 stops, L5-type does not
            + [free(98, trav=0, stops=1, bend=1, tangent=2), free(99, start=0, trav=0)])
    out = summarise_free_soft(rows)
    assert out["residual_rule"] == "re-sample"                          # 1 % residual
    assert out["all"]["unavoidable_stops_per_100m"] == pytest.approx(100 * 1 / 1000)
    assert out["all"]["a1_tangent_stops_per_100m"] == pytest.approx(100 * 100 / 1000)
    assert out["all"]["bend_stop_routes"] == 1


def test_free_choice_fewest_bend_stops_among_plannable_and_the_floor_note():
    from pfbnav.eval.oracle_report import FLOOR_NOTE, summarise_free
    rows = ([free(1, 0.03, trav=0, stops=1, bend=1), free(2, 0.03, trav=0, stops=1, bend=1)]
            + [free(1, 0.10, trav=0, stops=1, bend=1), free(2, 0.10)]
            + [free(1, 0.13), free(2, 0.13, status="no_path")])
    out = summarise_free(rows, [0.03, 0.10, 0.13])
    assert out["chosen_soft"] == 0.10 and out["floor_note"] == FLOOR_NOTE
    assert "platform-specific" in FLOOR_NOTE


def test_free_information_per_lethal_margin_and_stratum():
    from pfbnav.eval.oracle_report import summarise_free_info
    rows = [free(1, oracle=30.0), free(2, offset_trav=0, oracle=27.5),
            dict(free(3, oracle=None, offset_trav=0), lethal_margin=0.10)]
    info = summarise_free_info(rows)
    narrow = info["0.07"]["narrow"]
    assert narrow["offset_breach_routes"] == 1 and narrow["offset_breach_share"] == 0.5
    assert narrow["mission_time_ratio_median"] == pytest.approx((30 / 25 + 27.5 / 25) / 2)
    assert info["0.1"]["narrow"]["offset_breach_routes"] == 1
