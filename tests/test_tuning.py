"""Known-answer tests for the classical tuning routines (`pfbnav/eval/tuning.py`, hard rule 6) and
the runner's refusals (`scripts/tune_classical.py`)."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from pfbnav.config import load_config
from pfbnav.control.regulated_pure_pursuit import RPPParams
from pfbnav.eval.tuning import ParamRange, candidate_score, parse_space, sample_candidates, select

REPO = Path(__file__).resolve().parents[1]
SPACE = REPO / "configs" / "tuning" / "b2_regulated_pure_pursuit.yaml"


def _script():
    spec = importlib.util.spec_from_file_location("tune_classical",
                                                  REPO / "scripts" / "tune_classical.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _score(success, rate, time=10.0):
    return {"success_rate": success, "stops_per_100m": rate, "mission_time_s": time}


# --- select ------------------------------------------------------------------------------------

def test_select_takes_fewest_stops_within_the_success_window():
    # best success 0.90; window 0.85..0.90 -> candidates 0, 1, 3; fewest stops is 3 (1.0)
    scores = [_score(0.90, 3.0), _score(0.86, 2.0), _score(0.84, 0.0), _score(0.85, 1.0)]
    assert select(scores) == 3


def test_select_window_edge_is_inclusive():
    assert select([_score(0.90, 3.0), _score(0.85, 1.0)]) == 1
    assert select([_score(0.90, 3.0), _score(0.849, 1.0)]) == 0


def test_select_ties_by_mission_time_then_index():
    scores = [_score(0.9, 1.0, 30.0), _score(0.9, 1.0, 20.0), _score(0.9, 1.0, 20.0)]
    assert select(scores) == 1


def test_select_ranks_missing_rate_and_time_last():
    assert select([_score(0.0, None, None), _score(0.0, 5.0, None)]) == 1
    assert select([_score(0.5, 1.0, None), _score(0.5, 1.0, 40.0)]) == 1


def test_select_refuses_empty():
    with pytest.raises(ValueError):
        select([])


# --- sampling ----------------------------------------------------------------------------------

def test_param_range_refuses_bad_ranges():
    with pytest.raises(ValueError):
        ParamRange(1.0, 1.0)
    with pytest.raises(ValueError):
        ParamRange(0.0, 1.0, log=True)


def test_log_uniform_draws_are_uniform_in_log():
    r = ParamRange(0.1, 10.0, log=True)
    rng = np.random.default_rng(0)
    x = np.array([r.draw(rng) for _ in range(20000)])
    assert x.min() >= 0.1 and x.max() <= 10.0
    # log10 uniform on [-1, 1]: median 1.0, a quarter below 10**-0.5
    assert np.median(x) == pytest.approx(1.0, rel=0.05)
    assert np.mean(x < 10 ** -0.5) == pytest.approx(0.25, abs=0.015)


def test_parse_space_reads_scale_and_refuses_unknown():
    space = parse_space({"a": {"low": 1, "high": 2}, "b": {"low": 1, "high": 4, "scale": "log"}})
    assert space == {"a": ParamRange(1.0, 2.0), "b": ParamRange(1.0, 4.0, True)}
    with pytest.raises(ValueError):
        parse_space({"a": {"low": 1, "high": 2, "scale": "sqrt"}})


def test_sample_candidates_defaults_first_reproducible_and_within_ranges():
    space = {"a": ParamRange(0.0, 1.0), "b": ParamRange(1.0, 100.0, log=True)}
    defaults = {"a": 0.5, "b": 10.0, "c": 7.0}
    c1, r1 = sample_candidates(space, defaults, 50, np.random.default_rng(3), lambda p: True)
    c2, r2 = sample_candidates(space, defaults, 50, np.random.default_rng(3), lambda p: True)
    assert c1 == c2 and r1 == r2 == 0
    assert c1[0] == defaults and len(c1) == 50
    assert all(0.0 <= c["a"] <= 1.0 and 1.0 <= c["b"] <= 100.0 and c["c"] == 7.0 for c in c1)


def test_sample_candidates_redraws_and_counts_refusals():
    cands, refused = sample_candidates({"a": ParamRange(0.0, 1.0)}, {"a": 0.9}, 200,
                                       np.random.default_rng(1), lambda p: p["a"] > 0.5)
    assert all(c["a"] > 0.5 for c in cands)
    # about half the draws are refused: 199 accepted -> about 199 refused
    assert 140 < refused < 270


def test_sample_candidates_refuses_unusable_spaces():
    space = {"a": ParamRange(0.0, 1.0)}
    with pytest.raises(ValueError):            # nothing valid
        sample_candidates(space, {"a": 2.0}, 2, np.random.default_rng(0), lambda p: p["a"] > 1.5)
    with pytest.raises(ValueError):            # defaults invalid
        sample_candidates(space, {"a": 0.0}, 1, np.random.default_rng(0), lambda p: False)
    with pytest.raises(ValueError):            # unknown parameter
        sample_candidates({"z": ParamRange(0, 1)}, {"a": 0.0}, 1, np.random.default_rng(0),
                          lambda p: True)


# --- scoring -----------------------------------------------------------------------------------

def test_candidate_score_known_answer():
    records = [
        {"outcome": "success", "episode_return": 0.0, "episode_steps": 200, "protective_stops": 1, "distance_m": 10.0},
        {"outcome": "success", "episode_return": 0.0, "episode_steps": 400, "protective_stops": 0, "distance_m": 20.0},
        {"outcome": "timeout", "episode_return": 0.0, "episode_steps": 1000, "protective_stops": 2, "distance_m": 20.0},
    ]
    s = candidate_score(records, dt=0.05)
    assert s["episodes"] == 3
    assert s["success_rate"] == pytest.approx(2 / 3)
    assert s["stops_per_100m"] == pytest.approx(3 / 50 * 100)
    assert s["mission_time_s"] == pytest.approx((10.0 + 20.0) / 2)


def test_candidate_score_without_success_has_no_mission_time():
    s = candidate_score([{"outcome": "timeout", "episode_return": 0.0, "episode_steps": 10, "protective_stops": 0,
                          "distance_m": 1.0}], dt=0.05)
    assert s["mission_time_s"] is None and s["success_rate"] == 0.0


# --- the B2 space and the runner ---------------------------------------------------------------

def test_b2_space_brackets_the_configured_values_and_samples():
    cfg = load_config(REPO / "configs" / "base.yaml")
    doc = yaml.safe_load(SPACE.read_text(encoding="utf-8"))
    defaults = {k: float(v) for k, v in cfg["baselines"]["regulated_pure_pursuit"].items()}
    space = parse_space(doc["space"])
    for name, r in space.items():
        assert r.low <= defaults[name] <= r.high, name

    def valid(p):
        try:
            RPPParams(**p)
        except ValueError:
            return False
        return True

    cands, _ = sample_candidates(space, defaults, 30, np.random.default_rng(0), valid)
    assert len(cands) == 30 and all(valid(c) for c in cands)


def test_b3_space_brackets_the_configured_values_and_every_candidate_builds_the_controller():
    """B3's space (ranges APPROVED 2026-10-27, two floors extended by A28a): each range brackets
    candidate 0 (the standard DWA defaults); every sampled candidate passes the runner's own
    validity hook, and the whole config with it substituted builds the controller (whole sample
    counts survive the floats)."""
    from pfbnav.control.dwa import HolonomicDWA
    mod = _script()
    cfg = load_config(REPO / "configs" / "base.yaml")
    path = REPO / "configs" / "tuning" / "b3_dwa.yaml"
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    block, label, params_cls = mod.TUNABLE["dwa"]
    assert doc["controller"] == "dwa" and doc["config_block"] == block == "baselines.dwa"
    assert label == HolonomicDWA.name
    defaults = {k: float(v) for k, v in cfg["baselines"]["dwa"].items()}
    space = parse_space(doc["space"])
    for name, r in space.items():
        assert r.low <= defaults[name] <= r.high, name
    assert "w_progress" not in space          # the unit of the weights
    cands, refused = sample_candidates(space, defaults, 24, np.random.default_rng(0),
                                       lambda p: mod._valid(params_cls, p))
    assert len(cands) == 24 and refused == 0
    for c in cands:
        HolonomicDWA.from_config(mod.with_block(cfg, block, c))
    assert len(mod.preflight(cfg, doc, list(mod.SETS), 24)) == 3
    assert "VERIFY" not in path.read_text(encoding="utf-8")         # approved 2026-10-27
    # Candidate 0, the Nav2 DWB defaults as converted in configs/base.yaml (ruled 2026-10-27).
    d = cfg["baselines"]["dwa"]
    assert (d["sim_time"], d["safety_margin"], d["clearance_cap"]) == (1.7, 0.0, 0.33)
    assert (d["w_path"], d["w_progress"], d["w_heading"]) == (1.333, 1.0, 0.133)
    # A28a: the two floors extended to DWB's own values, which are now the floors.
    assert (d["window_time"], d["w_clearance"]) == (0.05, 0.020)
    assert (space["window_time"].low, space["w_clearance"].low) == (0.05, 0.02)


def test_with_block_replaces_only_named_keys():
    mod = _script()
    cfg = {"baselines": {"x": {"a": 1.0, "b": 2.0}}, "other": 3}
    out = mod.with_block(cfg, "baselines.x", {"a": 5.0})
    assert out["baselines"]["x"] == {"a": 5.0, "b": 2.0} and cfg["baselines"]["x"]["a"] == 1.0
    with pytest.raises(ValueError):
        mod.with_block(cfg, "baselines.x", {"zz": 1.0})


def test_runner_refuses_test_sets_filter_and_unknown_controller():
    mod = _script()
    cfg = load_config(REPO / "configs" / "base.yaml")
    doc = yaml.safe_load(SPACE.read_text(encoding="utf-8"))
    with pytest.raises(SystemExit, match="hard rule 2"):
        mod.preflight(cfg, doc, ["scenarios/test/anything.json"], 5)
    with pytest.raises(SystemExit, match="cannot tune"):
        mod.preflight(cfg, doc | {"controller": "sac"}, list(mod.SETS), 5)
    with pytest.raises(SystemExit, match="budget"):
        mod.preflight(cfg, doc, list(mod.SETS), 0)
    on = mod.with_block(cfg, "filter", {})
    on["filter"]["enabled"] = True
    with pytest.raises(SystemExit, match="filter.enabled"):
        mod.preflight(on, doc, list(mod.SETS), 5)
    assert len(mod.preflight(cfg, doc, list(mod.SETS), 5)) == 3


# --- rulings of 2026-10-26: budget, committed ranges, G2 approval, boundaries ------------------

def test_the_budget_is_24_candidates_and_the_ruled_window():
    doc = yaml.safe_load((REPO / "configs" / "tuning" / "budget.yaml").read_text(encoding="utf-8"))
    assert doc["candidates"] == 24 and doc["selection"]["success_window"] == 0.05


def test_the_b2_space_includes_rpp_curvature_and_proximity_scaling_and_no_verify():
    doc = yaml.safe_load(SPACE.read_text(encoding="utf-8"))
    assert {"min_radius", "min_speed", "proximity_distance", "proximity_gain"} <= set(doc["space"])
    assert "VERIFY" not in SPACE.read_text(encoding="utf-8")


def test_on_boundary_known_answer():
    from pfbnav.eval.tuning import on_boundary
    space = {"a": ParamRange(0.0, 1.0), "b": ParamRange(1.0, 100.0, log=True)}
    assert on_boundary({"a": 0.5, "b": 10.0}, space) == {}
    assert on_boundary({"a": 0.005, "b": 10.0}, space) == {"a": "low"}
    assert on_boundary({"a": 0.995, "b": 10.0}, space) == {"a": "high"}
    # log range: 1 % of the log width is a factor 100 ** 0.01 = 1.047 from either end
    assert on_boundary({"a": 0.5, "b": 1.04}, space) == {"b": "low"}
    assert on_boundary({"a": 0.5, "b": 1.06}, space) == {}
    assert on_boundary({"a": 0.5, "b": 96.0}, space) == {"b": "high"}


def test_g2_approval_is_per_host_and_bounded_by_the_approved_count(tmp_path):
    import json
    mod = _script()
    path = tmp_path / "summary.json"
    assert "no G2 summary" in mod.g2_problem(path, 1, "host")
    path.write_text(json.dumps({"verdict": "PASS", "approved_instances": 3,
                                "host": {"hostname": "trainer"}}), encoding="utf-8")
    assert mod.g2_problem(path, 3, "trainer") is None
    assert "approved 3" in mod.g2_problem(path, 4, "trainer")
    assert "per host" in mod.g2_problem(path, 1, "laptop")
    path.write_text(json.dumps({"verdict": "FAIL", "approved_instances": None,
                                "host": {"hostname": "trainer"}}), encoding="utf-8")
    assert "did not pass" in mod.g2_problem(path, 1, "trainer")


def test_a_space_that_is_uncommitted_or_says_verify_is_refused(tmp_path):
    mod = _script()
    scratch = REPO / "configs" / "tuning" / "_scratch_test_space.yaml"
    try:
        scratch.write_text("space: {}\n", encoding="utf-8")
        assert "not committed" in mod.committed_problem(scratch)
        scratch.write_text("space: {}  # VERIFY\n", encoding="utf-8")
        assert "VERIFY" in mod.committed_problem(scratch)
    finally:
        scratch.unlink()


def _ep(index, stratum, outcome="success", steps=100, stops=0, dist=10.0, infeasible=0,
        fallback=0, pfb=True):
    r = {"set": "tune_" + stratum, "index": index, "stratum": stratum, "outcome": outcome,
         "episode_return": 0.0, "episode_steps": steps, "protective_stops": stops,
         "distance_m": dist}
    if pfb:
        r["pfb_infeasible"] = infeasible
        r["pfb_modes"] = {"fallback": fallback, "filtered": steps - fallback}
    return r


def test_stratum_comparison_known_answer():
    from pfbnav.eval.tuning import stratum_comparison
    b2 = [_ep(0, "narrow", stops=2, pfb=False), _ep(1, "wide", outcome="timeout", stops=1,
                                                       pfb=False)]
    b7 = [_ep(0, "narrow", steps=120, infeasible=3, fallback=4), _ep(1, "wide", infeasible=1)]
    out = stratum_comparison({"B2": b2, "B7": b7}, dt=0.05)
    assert out["narrow"]["B2"]["stops_per_100m"] == pytest.approx(20.0)
    assert out["narrow"]["B7"]["stops_per_100m"] == pytest.approx(0.0)
    assert out["narrow"]["B7"]["mission_time_s"] == pytest.approx(6.0)
    assert out["narrow"]["B7"]["pfb_infeasible"] == 3 and out["narrow"]["B7"]["pfb_fallback_steps"] == 4
    assert out["narrow"]["B2"]["pfb_infeasible"] == 0 and out["narrow"]["B2"]["pfb_fallback_steps"] == 0
    assert out["wide"]["B2"]["success_rate"] == 0.0 and out["all"]["B2"]["success_rate"] == 0.5
    assert out["all"]["B7"]["pfb_infeasible"] == 4


def test_stratum_comparison_refuses_arms_on_different_scenarios():
    from pfbnav.eval.tuning import stratum_comparison
    with pytest.raises(ValueError, match="same scenarios"):
        stratum_comparison({"B2": [_ep(0, "narrow")], "B7": [_ep(1, "narrow")]}, dt=0.05)


def test_tuning_waits_for_the_contingency_hypothesis():
    mod = _script()
    assert "A27" in mod.contingency_problem({"H1": {}, "confirmatory_family": {}})
    assert "A27" in mod.contingency_problem({"H1prime": {"comparators": ["B1"]},
                                             "confirmatory_family": {}})
    pending = yaml.safe_load((REPO / "docs" / "pending" / "endpoints_A27.yaml").read_text(
        encoding="utf-8"))
    assert mod.contingency_problem(pending) is None


# --- B6 (A16): a nested block with integers; the space of 2026-10-27 ---------------------------

B6_SPACE = REPO / "configs" / "tuning" / "b6_mppi.yaml"


def test_flatten_numeric_known_answer():
    from pfbnav.eval.tuning import flatten_numeric
    block = {"model": "holonomic", "horizon_steps": 30, "temperature": 1.0,
             "noise_std": [0.1, 0.2, 0.5], "weights": {"a": 1.0, "b": 10.0}, "flag": True}
    assert flatten_numeric(block) == {"horizon_steps": 30.0, "temperature": 1.0,
                                      "noise_std.0": 0.1, "noise_std.1": 0.2, "noise_std.2": 0.5,
                                      "weights.a": 1.0, "weights.b": 10.0}
    flat = {"a": 1.0, "b": 2}
    assert flatten_numeric(flat) == {"a": 1.0, "b": 2.0}          # a flat block maps to itself


def test_assign_flat_keeps_integers_and_refuses_what_is_not_there():
    from pfbnav.eval.tuning import assign_flat
    block = {"samples": 1024, "noise_std": [0.1, 0.2, 0.5], "weights": {"a": 1.0}, "s": "x"}
    assign_flat(block, {"samples": 256.0, "noise_std.2": 0.3, "weights.a": 5.0})
    assert block == {"samples": 256, "noise_std": [0.1, 0.2, 0.3], "weights": {"a": 5.0},
                     "s": "x"}
    assert isinstance(block["samples"], int)
    with pytest.raises(ValueError, match="integer"):
        assign_flat(block, {"samples": 30.5})
    for bad in ("noise_std.3", "weights.zz", "zz", "zz.a"):
        with pytest.raises(ValueError, match="no parameter"):
            assign_flat(block, {bad: 1.0})
    with pytest.raises(TypeError, match="not a numeric"):
        assign_flat(block, {"s": 1.0})


def test_with_block_writes_nested_leaves_by_dotted_name():
    mod = _script()
    cfg = {"baselines": {"m": {"n": 30, "w": {"a": 1.0}, "s": [0.1, 0.2], "model": "h"}}}
    out = mod.with_block(cfg, "baselines.m", {"n": 20.0, "w.a": 3.0, "s.1": 0.4})
    assert out["baselines"]["m"] == {"n": 20, "w": {"a": 3.0}, "s": [0.1, 0.4], "model": "h"}
    assert cfg["baselines"]["m"]["n"] == 30                         # the input is untouched
    with pytest.raises(ValueError, match="has no"):
        mod.with_block(cfg, "baselines.m", {"model": 1.0})


def test_integer_ranges_draw_integers_within_the_ends():
    r = ParamRange(128.0, 1024.0, log=True, integer=True)
    draws = [r.draw(np.random.default_rng(i)) for i in range(300)]
    assert all(float(d).is_integer() and 128 <= d <= 1024 for d in draws)
    assert len(set(draws)) > 100
    with pytest.raises(ValueError, match="integer ends"):
        ParamRange(0.5, 3.0, integer=True)
    assert parse_space({"n": {"low": 15, "high": 40, "type": "int"}}) == {
        "n": ParamRange(15.0, 40.0, False, True)}
    with pytest.raises(ValueError, match="type"):
        parse_space({"n": {"low": 15, "high": 40, "type": "bool"}})


def test_b6_space_brackets_the_configured_values_and_samples_valid_candidates():
    from pfbnav.control.mppi import MPPIParams
    from pfbnav.eval.tuning import flatten_numeric
    mod = _script()
    cfg = load_config(REPO / "configs" / "base.yaml")
    doc = yaml.safe_load(B6_SPACE.read_text(encoding="utf-8"))
    assert doc["controller"] == "field_aware_mppi"
    assert doc["config_block"] == mod.TUNABLE["field_aware_mppi"][0] == "baselines.mppi"
    assert mod.TUNABLE["field_aware_mppi"][1] == "B6_field_aware_mppi"
    defaults = flatten_numeric(cfg["baselines"]["mppi"])
    space = parse_space(doc["space"])
    for name, r in space.items():
        assert r.low <= defaults[name] <= r.high, name
    # Ruled 2026-10-27: samples, horizon and iterations are fixed by measurement, not tuned; every
    # range is re-centred on candidate 0 (geometric centres within 6 %).
    assert not {"samples", "horizon_steps", "iterations"} & set(space)
    for name, r in space.items():
        assert math.sqrt(r.low * r.high) == pytest.approx(defaults[name], rel=0.06), name
    cands, _ = sample_candidates(space, defaults, 30, np.random.default_rng(0),
                                 lambda p: mod._valid(MPPIParams.from_flat, p))
    assert len(cands) == 30
    for c in cands:                                   # each builds the controller's own config
        block = mod.with_block(cfg, "baselines.mppi", c)["baselines"]["mppi"]
        assert MPPIParams.from_block(block) == MPPIParams.from_flat(**c)
    cfg["baselines"]["mppi"]["compute_measurement"] = None           # as if not measured yet
    with pytest.raises(SystemExit, match="compute_measurement"):
        mod.preflight(cfg, doc, list(mod.SETS), 5)


def test_the_b6_space_is_approved_and_b6_waits_for_its_compute_measurement(tmp_path):
    """Ranges APPROVED 2026-10-27 (no VERIFY). B6 is refused until
    `baselines.mppi.compute_measurement` names the declared real-time measurement that fixed its
    samples and horizon; every other controller is unaffected. The config names the run of
    2026-10-29; the refusal is checked with it cleared, since runs/ exists only on the host."""
    mod = _script()
    cfg = load_config(REPO / "configs" / "base.yaml")
    assert "VERIFY" not in B6_SPACE.read_text(encoding="utf-8")
    assert (cfg["baselines"]["mppi"]["compute_measurement"]
            == "runs/realtime/REALTIME_20261029T095844Z_seed0")
    cfg["baselines"]["mppi"]["compute_measurement"] = None
    assert "compute_measurement" in mod.measurement_problem("field_aware_mppi", cfg)
    assert mod.measurement_problem("regulated_pure_pursuit", cfg) is None
    doc = yaml.safe_load(B6_SPACE.read_text(encoding="utf-8"))
    with pytest.raises(SystemExit, match="compute_measurement"):
        mod.preflight(cfg, doc, list(mod.SETS), 24)
    run = tmp_path / "run"
    run.mkdir()
    cfg["baselines"]["mppi"]["compute_measurement"] = str(run)
    assert "no summary.json" in mod.measurement_problem("field_aware_mppi", cfg)
    (run / "summary.json").write_text("{}", encoding="utf-8")
    assert mod.measurement_problem("field_aware_mppi", cfg) is None


def test_tuning_waits_for_the_realtime_criterion_in_the_preregistration():
    """Ruled 2026-10-27: the rulings are preregistered (A28) and relocked before any tuning."""
    mod = _script()
    assert "A28" in mod.realtime_problem({"G2_compute_budget": {}})
    assert mod.realtime_problem({"realtime_compute": {"p95_max_s": 0.05}}) is None
