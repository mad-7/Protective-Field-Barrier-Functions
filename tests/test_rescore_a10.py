"""Known-answer tests for scripts/rescore_a10_drift.py on a synthetic probe run (hard rule 6)."""
import csv
import importlib.util
import json
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "rescore_a10_drift.py"


def _load():
    spec = importlib.util.spec_from_file_location("rescore_a10_drift", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_lateral(path: Path, drift_deg: float, steps: int = 20) -> None:
    path.parent.mkdir(parents=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "yaw", "cmd_wz"])
        for k in range(steps + 1):
            w.writerow([k, math.pi / 2 + math.radians(drift_deg) * k / steps, 0.0])


def _fake_run(tmp_path: Path, drifts: dict[str, float]) -> Path:
    run = tmp_path / "DSTEP_fake_seed0"
    candidates = {}
    for key, d in drifts.items():
        _write_lateral(run / f"step_{key}" / "lateral.csv", d)
        magnitude = abs(d)
        candidates[key] = {
            "physics": {"simulation_time_step": 0.05},
            "g1b": {"result": "PASS"}, "g1b_ceilings_unchanged": True, "g0_zero_lag": True,
            "lateral_drift_deg": magnitude,
            # What the defective magnitude comparison recorded:
            "drift_within_band": abs(magnitude - abs(drifts["0.005"])) <= 0.64,
        }
        candidates[key]["passes_a10"] = candidates[key]["drift_within_band"]
    (run / "summary.json").write_text(json.dumps({
        "drift_band_deg": 0.64, "frozen_step_s": 0.005,
        "largest_passing_step_s": 0.05, "candidates": candidates}))
    (run / "provenance.json").write_text(json.dumps(
        {"commit": "0" * 40, "dirty": False, "scene_sha256": "f" * 64}))
    return run


def test_reversed_sign_candidates_fail_and_same_sign_ones_pass(tmp_path):
    run = _fake_run(tmp_path, {"0.005": 0.8, "0.010": 0.5, "0.025": -0.8, "0.050": -0.2})
    r = _load().rescore(run)
    c = r["candidates"]
    assert c["0.010"]["drift_within_band"]                   # +0.5 vs +0.8: gap 0.3
    assert c["0.025"]["recorded_drift_within_band"]          # magnitudes equal
    assert not c["0.025"]["drift_within_band"]               # signed gap 1.6
    assert c["0.025"]["signed_gap_deg"] == pytest.approx(-1.6)
    assert not c["0.050"]["drift_within_band"]               # signed gap 1.0
    assert r["corrected_largest_passing_step_s"] == 0.010


def test_no_candidate_passing_leaves_the_frozen_step(tmp_path):
    run = _fake_run(tmp_path, {"0.005": 0.831, "0.010": -0.177, "0.025": -0.802,
                               "0.050": -1.323})
    r = _load().rescore(run)
    assert r["corrected_largest_passing_step_s"] is None
    assert r["recorded_largest_passing_step_s"] == 0.05


def test_refuses_when_the_csv_does_not_reproduce_the_recorded_magnitude(tmp_path):
    run = _fake_run(tmp_path, {"0.005": 0.8, "0.010": 0.5})
    summary = json.loads((run / "summary.json").read_text())
    summary["candidates"]["0.010"]["lateral_drift_deg"] = 0.6
    (run / "summary.json").write_text(json.dumps(summary))
    with pytest.raises(SystemExit):
        _load().rescore(run)


def test_rejects_a_drift_run_that_commands_rotation(tmp_path):
    path = tmp_path / "x" / "lateral.csv"
    path.parent.mkdir()
    path.write_text("step,yaw,cmd_wz\n0,0.0,0.1\n1,0.01,0.1\n")
    with pytest.raises(ValueError):
        _load().drift_from_csv(path, 0.05)
