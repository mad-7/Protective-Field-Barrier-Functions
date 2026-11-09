"""HF_hosts (A37): the arm set, the declared low-count and consistency checks, the apply script on a copy (hard rule 6)."""
import importlib.util
import json
import shutil
from pathlib import Path

import pytest
import yaml

from pfbnav.eval.hfc import consistency, low_count

ROOT = Path(__file__).resolve().parents[1]


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _script(name):
    return _load(ROOT / "scripts" / f"{name}.py", name)


def test_the_host_arms_pair_each_planner_with_its_filtered_self(tmp_path):
    ev = _script("evaluate_test")
    hosts = ev.ARM_SETS["hosts"]
    assert list(hosts) == ["B3", "B9", "B6", "B8"]
    assert hosts["B3"] == ("dwa", False) and hosts["B9"] == ("dwa", True)
    assert hosts["B6"] == ("field_aware_mppi", False) and hosts["B8"] == ("field_aware_mppi", True)
    assert ev.ARM_SETS["classical"] is ev.ARMS and list(ev.ARMS) == ["B1", "B2", "B3", "B6", "B7"]
    assert ev.run_prefix("mapped_only", "hosts") == "HOSTS_mapped_only_"
    assert ev.run_prefix("unmapped_static") == "CLASSICAL_"
    (tmp_path / "CLASSICAL_shard0of4_x_seed0").mkdir()              # a sealed shard never blocks a host shard
    assert ev.prior_attempts(0, 4, "unmapped_static", tmp_path, arm_set="hosts") == []
    (tmp_path / "HOSTS_unmapped_static_shard0of4_y_seed0").mkdir()
    assert len(ev.prior_attempts(0, 4, "unmapped_static", tmp_path, arm_set="hosts")) == 1
    assert ev.prior_attempts(0, 4, "mapped_only", tmp_path, arm_set="hosts") == []
    assert len(ev.prior_attempts(0, 4, "unmapped_static", tmp_path)) == 1


def test_low_count_and_consistency_known_answers():
    assert low_count(0, 10) and low_count(3, 3) and low_count(7, 29)
    assert not low_count(0, 30) and not low_count(1, 815) and not low_count(8, 9)
    ok = consistency({"success_rate": 0.60, "protective_stops": 10}, {"success_rate": 0.553, "protective_stops": 10})
    assert not ok["flagged"] and ok["success_difference"] == pytest.approx(0.047)
    assert consistency({"success_rate": 0.40, "protective_stops": 10},
                       {"success_rate": 0.553, "protective_stops": 10})["flagged"]


def _rec(sha, stops, plan=10.0):
    return {"scenario_sha256": sha, "protective_stops": stops, "scan_fault_stops": 0, "plan_length_m": plan,
            "stratum": "narrow", "outcome": "success", "episode_return": 0.0}


def test_the_comparisons_flag_low_counts_and_keep_clear_verdicts():
    m = _script("analyse_hosts")
    n = 150
    records = {"B3": [_rec(f"s{i}", 5) for i in range(n)], "B9": [_rec(f"s{i}", int(i < 2)) for i in range(n)],
               "B6": [_rec(f"s{i}", int(i < 10)) for i in range(n)], "B8": [_rec(f"s{i}", 0) for i in range(n)]}
    c = m.comparisons(records)
    assert not c["B9_vs_B3"]["low_count"] and c["B9_vs_B3"]["reading"] == "a fewer"
    assert c["B8_vs_B6"]["low_count"] and c["B8_vs_B6"]["reading"] == "counts only: 0 vs 10 events"


def test_read_runs_reads_only_the_named_arms(tmp_path):
    hfc = _script("analyse_hfc")
    for i, positions in enumerate(([0, 2], [1, 3])):
        d = tmp_path / f"s{i}"
        d.mkdir()
        (d / "provenance.json").write_text(json.dumps({"dirty": False, "commit": "c", "host": {"hostname": "H"}}),
                                           encoding="utf-8")
        (d / "evaluated.json").write_text(json.dumps(
            {"shard": i, "shards": 2, "positions": positions, "sets": {"s": "h"}, "environment": "e", "alpha": 1.0,
             "b2_parameters": {}, "arms": {}, "rerun_after_crash": None}), encoding="utf-8")
        for a in ("B3", "B9"):
            (d / a).mkdir()
            (d / a / "episodes.jsonl").write_text("".join(json.dumps(_rec(f"p{p}", 0)) + "\n" for p in positions),
                                                  encoding="utf-8")
    records, _ = hfc.read_runs([tmp_path / "s0", tmp_path / "s1"], ("B3", "B9"))
    assert set(records) == {"B3", "B9"} and len(records["B9"]) == 4
    with pytest.raises(SystemExit):
        hfc.read_runs([tmp_path / "s0", tmp_path / "s1"], ("B3", "B8"))       # an arm that did not run


def test_apply_a37_on_a_copy(tmp_path, monkeypatch):
    if "## A37 - " in (ROOT / "preregistration" / "AMENDMENTS.md").read_text(encoding="utf-8"):
        pytest.skip("A37 is already applied")
    prereg = tmp_path / "preregistration"
    shutil.copytree(ROOT / "preregistration", prereg)
    m = _load(ROOT / "docs" / "pending" / "apply_A37.py", "apply_A37")
    monkeypatch.setattr(m, "PREREG", prereg)
    assert m.main() == 0
    e = yaml.safe_load((prereg / "endpoints.yaml").read_text(encoding="utf-8"))
    assert "B9" in e["exploratory"]["HF_hosts"] and "B8" in e["exploratory"]["HF_hosts"]
    assert "L2_sensitivity" in e["exploratory"]
    before = yaml.safe_load((ROOT / "preregistration" / "endpoints.yaml").read_text(encoding="utf-8"))
    assert e["HFc"] == before["HFc"] and e["H1"] == before["H1"]              # nothing else moved
    assert (prereg / "AMENDMENTS.md").read_text(encoding="utf-8").count("## A37 - ") == 1
    with pytest.raises(SystemExit):
        m.main()
