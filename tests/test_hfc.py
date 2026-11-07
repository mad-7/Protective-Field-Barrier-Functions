"""HF-c (A36): the classical-vs-classical interval, the pairing table, the sealed runner's helpers and the
analysis's integrity checks -- known answers on synthetic data (hard rule 6)."""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy import stats

from pfbnav.eval.hfc import ARMS, classical_comparison, paired_table
from pfbnav.eval.rate_ratio import arm_terms, components

ROOT = Path(__file__).resolve().parents[1]


def _script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_estimate_and_interval_known_answer():
    """Estimate = log((S_a + 0.5) / (S_b + 0.5)) on equal planned lengths; half-width = t(n-1) x
    sqrt(the scenario part); the interval lies below 0 for a clear 10x reduction."""
    rng = np.random.default_rng(3)
    d = rng.uniform(5, 15, 150)
    sa, sb = rng.poisson(0.05 * d / 10), rng.poisson(5.0 * d / 10)
    r = classical_comparison(sa, sb, d, 0.05)
    assert r["log_rate_ratio"] == pytest.approx(math.log((sa.sum() + 0.5) / (sb.sum() + 0.5)))
    c = components(arm_terms(sa[None, None, :].astype(float), d[None, :]),
                   arm_terms(sb[None, None, :].astype(float), d[None, :]))
    half = stats.t.isf(0.025, 149) * math.sqrt(float(c.scenario[0]))
    assert r["log_upper"] - r["log_rate_ratio"] == pytest.approx(half)
    assert r["df"] == 149 and r["favours_a"] and not r["favours_b"] and r["verdict"] == "a fewer"
    assert r["ci"][0] < r["rate_ratio"] < r["ci"][1] < 1.0
    assert r["rate_a_per_100m"] == pytest.approx(100 * sa.sum() / d.sum())


def test_equal_arms_are_inconclusive_and_inputs_are_checked():
    d = np.full(40, 10.0)
    s = np.arange(40) % 3
    r = classical_comparison(s, s, d, 0.05)
    assert r["log_rate_ratio"] == 0.0 and r["verdict"] == "inconclusive"
    with pytest.raises(ValueError):
        classical_comparison([1, 2], [1], [10.0, 10.0], 0.05)
    with pytest.raises(ValueError):
        classical_comparison([1, 2], [1, 2], [10.0, 0.0], 0.05)


def _rec(sha, stops, fault=0, plan=10.0, stratum="narrow"):
    return {"scenario_sha256": sha, "protective_stops": stops, "scan_fault_stops": fault,
            "plan_length_m": plan, "stratum": stratum, "outcome": "success", "episode_return": 0.0}


def test_paired_table_aligns_and_excludes_scan_fault_stops():
    by = {a: [_rec("b", 2, 1, 12.0), _rec("a", 1)] for a in ARMS}
    t = paired_table(by)
    assert t["scenarios"] == ["a", "b"] and t["planned_m"] == [10.0, 12.0]
    assert t["stops"]["B7"] == [1, 1]          # 2 stops on "b", one decided on a faulted scan
    bad = dict(by, B6=[_rec("a", 1)])
    with pytest.raises(ValueError):
        paired_table(bad)                       # not the same scenarios
    with pytest.raises(ValueError):
        paired_table(dict(by, B6=[_rec("a", 1), _rec("a", 1)]))     # two episodes on one
    with pytest.raises(ValueError):
        paired_table(dict(by, B6=[_rec("b", 2, 1, 12.5), _rec("a", 1)]))   # planned lengths differ


def test_shards_partition_and_the_filter_switch():
    m = _script("evaluate_test")
    parts = [m.shard_indices(150, i, 4) for i in range(4)]
    assert sorted(x for p in parts for x in p) == list(range(150))
    assert [len(p) for p in parts] == [38, 38, 37, 37]
    with pytest.raises(ValueError):
        m.shard_indices(10, 4, 4)

    class Loop:
        pfb, filter_enabled = "P", True
    loop = Loop()
    m.set_filter(loop, None)
    assert loop.pfb is None and loop.filter_enabled is False
    m.set_filter(loop, "P")
    assert loop.pfb == "P" and loop.filter_enabled is True


def _shard(root: Path, i: int, n: int, positions, per_arm, *, dirty=False, commit="c0"):
    d = root / f"CLASSICAL_shard{i}of{n}_x_seed0"
    d.mkdir()
    (d / "provenance.json").write_text(json.dumps(
        {"dirty": dirty, "commit": commit, "host": {"hostname": "H"}}), encoding="utf-8")
    (d / "evaluated.json").write_text(json.dumps(
        {"shard": i, "shards": n, "positions": positions, "sets": {"s": "h"}, "environment": "e",
         "alpha": 1.0, "b2_parameters": {"k": 1}, "arms": {"B1": 1}, "rerun_after_crash": None}),
        encoding="utf-8")
    for a in ARMS:
        (d / a).mkdir()
        (d / a / "episodes.jsonl").write_text(
            "".join(json.dumps(_rec(f"s{p}", 0)) + "\n" for p in positions[:per_arm]),
            encoding="utf-8")
    return d


def test_the_analysis_refuses_anything_but_complete_clean_shards(tmp_path):
    m = _script("analyse_hfc")
    ok = [_shard(tmp_path, 0, 2, [0, 2], 2), _shard(tmp_path, 1, 2, [1, 3], 2)]
    records, checked = m.read_runs(ok)
    assert checked["scenarios"] == 4 and all(len(r) == 4 for r in records.values())
    with pytest.raises(SystemExit):
        m.read_runs(ok[:1])                                        # a shard missing
    (tmp_path / "t").mkdir()
    trunc = [_shard(tmp_path / "t", 0, 2, [0, 2], 1), _shard(tmp_path / "t", 1, 2, [1, 3], 2)]
    with pytest.raises(SystemExit):
        m.read_runs(trunc)                                         # a truncated shard
    (tmp_path / "d").mkdir()
    dirty = [_shard(tmp_path / "d", 0, 2, [0, 2], 2, dirty=True), _shard(tmp_path / "d", 1, 2, [1, 3], 2)]
    with pytest.raises(SystemExit):
        m.read_runs(dirty)
    (tmp_path / "c").mkdir()
    mixed = [_shard(tmp_path / "c", 0, 2, [0, 2], 2), _shard(tmp_path / "c", 1, 2, [1, 3], 2, commit="c1")]
    with pytest.raises(SystemExit):
        m.read_runs(mixed)


def test_the_calibration_gate_value():
    m = _script("calibrate_hfc")
    assert m.gate(10000, 0.025, 3.0) == pytest.approx(0.025 + 3 * math.sqrt(0.025 * 0.975 / 1e4))


def test_the_paper_outputs_follow_the_wording_rules():
    """An inconclusive comparison reads 'indistinguishable at x vs y events'; the table has a row per
    arm; the numbers carry the held-out label."""
    m = _script("analyse_hfc")
    d = np.full(150, 10.0)
    fewer = classical_comparison(np.r_[np.ones(3), np.zeros(147)], np.full(150, 5.0), d, 0.05)
    tie = classical_comparison(np.r_[np.ones(3), np.zeros(147)], np.r_[np.zeros(147), np.ones(3)], d, 0.05)
    assert m.wording(fewer, "B7", "B2").startswith("B7 triggers fewer protective stops than B2")
    assert m.wording(tie, "B7", "B6").startswith("B7 and B6 are indistinguishable at 3 vs 3 events")
    recs = [_rec(f"s{i}", 0) for i in range(4)]
    desc = {a: {"pooled": m.describe(recs, 0.05), "per_stratum": {}} for a in ARMS}
    report = {"checked": {"commit": "c0"}, "HFc": {**fewer, "result": "B7 fewer protective stops than B2"},
              "H1prime_exploratory": {"comparisons": {"B6": tie}}, "descriptive": desc}
    table = m.paper_table(report)
    assert all(name in table for name in m.NAMES.values()) and r"\bottomrule" in table
    nums = m.paper_numbers(report)
    assert nums["label"].startswith("held-out test set") and "preregistered but exploratory" not in json.dumps(nums)
    assert "indistinguishable" in nums["H1prime_sentences"]["B6"]
