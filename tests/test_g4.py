"""Known answers for gate G4 (amendment A15; hard rule 6). Synthetic logs and summaries."""
import math

from pfbnav.eval.g4 import (
    checkpoint_criterion,
    controls_verdict,
    g4_verdict,
    integrity_criterion,
    learning_criterion,
)

LEARN = {"window_fraction": 0.10, "min_last_success": 0.20, "min_improvement": 0.15}
CKPT = {"min_success": 0.30, "max_contact_rate": 0.10}


def log(outcomes):
    return [{"episode": i + 1, "timestep": 100 * (i + 1), "outcome": o}
            for i, o in enumerate(outcomes)]


def test_learning_known_answers():
    """100 episodes, windows of 10. First 10: 1 success (0.10). Last 10: 3 (0.30): +0.20 -> pass.
    Last 10 with 2 successes: 0.20, but +0.10 < 0.15 -> fail. First 0, last 2: 0.20 and +0.20 ->
    pass exactly at both thresholds."""
    base = ["timeout"] * 100
    a = list(base)
    a[0] = "success"
    for i in (91, 95, 99):
        a[i] = "success"
    got = learning_criterion(log(a), **LEARN)
    assert got["window"] == 10 and got["first"] == 0.1 and got["last"] == 0.3 and got["passed"]
    b = list(a)
    b[99] = "timeout"
    assert learning_criterion(log(b), **LEARN)["passed"] is False
    c = list(base)
    c[95] = c[99] = "success"
    got = learning_criterion(log(c), **LEARN)
    assert math.isclose(got["last"], 0.2) and math.isclose(got["improvement"], 0.2) and got["passed"]


def test_learning_needs_two_disjoint_windows_and_reads_episode_order():
    assert learning_criterion(log(["success"]), **LEARN)["passed"] is False
    shuffled = list(reversed(log(["timeout"] * 90 + ["success"] * 10)))
    assert learning_criterion(shuffled, **LEARN)["last"] == 1.0


def summary(success, timeout, collision=0, contact_extra=0):
    n = success + timeout + collision
    return {"episodes": n, "counts": {"success": success, "timeout": timeout,
                                      "collision": collision, "unrecoverable_stop": 0},
            "success_rate": success / n, "contact_rate": (collision + contact_extra) / n}


def test_checkpoint_known_answers():
    assert checkpoint_criterion(summary(18, 42), **CKPT)["passed"]               # 0.30, 0.00
    assert not checkpoint_criterion(summary(17, 43), **CKPT)["passed"]           # 0.283
    assert checkpoint_criterion(summary(30, 24, collision=6), **CKPT)["passed"]  # contact 0.10
    assert not checkpoint_criterion(summary(30, 23, collision=7), **CKPT)["passed"]


def test_controls_zero_must_fail_with_all_timeouts_and_the_positive_control_must_pass():
    sound = controls_verdict(summary(0, 60), summary(40, 20), **CKPT)
    assert sound["gate_sound"] and sound["zero_action"]["behaved"]
    # Zero action that SUCCEEDS somewhere means goals are reachable by doing nothing: broken.
    assert not controls_verdict(summary(1, 59), summary(40, 20), **CKPT)["gate_sound"]
    # Zero action that collides (something drives into the robot?) is not "all timeouts".
    assert not controls_verdict(summary(0, 59, collision=1), summary(40, 20), **CKPT)["gate_sound"]
    # Pure Pursuit failing the floor means the floor or the task is wrong: broken.
    assert not controls_verdict(summary(0, 60), summary(10, 50), **CKPT)["gate_sound"]


def integrity(**over):
    eps = log(["timeout", "success", "success"])
    base = {"completed": True, "timesteps": 300000, "total_timesteps": 300000,
            "final_model": True, "environment_sha256": "abc", "frozen_sha256": "abc",
            "losses": [0.5, 0.2, -1.3], "episodes": eps, "error": None}
    return integrity_criterion(**{**base, **over})


def test_integrity_each_check_fails_on_its_own():
    assert integrity()["passed"]
    assert not integrity(timesteps=299999)["checks"]["reached_total_timesteps"]
    assert not integrity(final_model=False)["checks"]["reached_total_timesteps"]
    assert not integrity(environment_sha256="abd")["checks"]["frozen_environment"]
    assert not integrity(losses=[0.1, float("nan")])["checks"]["finite_losses"]
    assert not integrity(losses=[float("inf")])["checks"]["finite_losses"]
    gap = log(["timeout", "success", "success"])
    gap[2]["episode"] = 4
    assert not integrity(episodes=gap)["checks"]["episode_log_complete"]
    flat = log(["timeout", "success"])
    flat[1]["timestep"] = flat[0]["timestep"]
    assert not integrity(episodes=flat)["checks"]["episode_log_complete"]
    assert not integrity(error="ScenarioUnusable('x')")["checks"]["no_environment_exception"]


def test_the_verdict_reads_the_controls_first():
    good_controls = controls_verdict(summary(0, 60), summary(40, 20), **CKPT)
    bad_controls = controls_verdict(summary(2, 58), summary(40, 20), **CKPT)
    ok = {"passed": True}
    no = {"passed": False}
    assert g4_verdict(bad_controls, ok, ok, ok) == "GATE BROKEN"
    assert g4_verdict(good_controls, no, ok, ok) == "INTEGRITY FAIL"
    assert g4_verdict(good_controls, ok, ok, ok) == "PASS"
    assert g4_verdict(good_controls, ok, no, ok) == "FAIL"
    assert g4_verdict(good_controls, ok, ok, no) == "FAIL"


# ---------------------------------------------------------------- scripts/gate_g4.py, end to end
G4 = {"pilot_training_seeds": [9000000, 9000001],
      "learning": {"window_fraction": 0.10, "min_last_success": 0.20, "min_improvement": 0.15},
      "checkpoint": {"sets": ["scenarios/tune/a.json", "scenarios/tune/b.json"],
                     "min_success": 0.30, "max_contact_rate": 0.10}}


def _gate():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "gate_g4", Path(__file__).resolve().parents[1] / "scripts" / "gate_g4.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path, obj):
    import json
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj), encoding="utf-8")


def _eval_dir(root, name, success, timeout, collision=0, sets=None, env="abc"):
    d = root / name
    _write(d / "evaluated.json", {"sets": {s: "x" for s in (sets or G4["checkpoint"]["sets"])},
                                  "environment": env})
    _write(d / "summary.json", {"overall": {**summary(success, timeout, collision),
                                            "stops_per_100m": 4.0,
                                            "shares": {"success": success / 60}},
                                "per_stratum": {"narrow": {"success_rate": 0.4}}})
    return d


def _train_dir(root, seed=9000000, nan=False):
    import json
    d = root / "train"
    outcomes = ["timeout"] * 90 + ["success"] * 10
    (d / "sb3").mkdir(parents=True)
    (d / "episodes.jsonl").write_text(
        "".join(json.dumps(e) + "\n" for e in log(outcomes)), encoding="utf-8")
    (d / "final_model.zip").write_bytes(b"model")
    _write(d / "provenance.json", {"seed": seed})
    _write(d / "environment.json", {"environment": {"sha256": "abc"}})
    _write(d / "summary.json", {"completed": True, "timesteps": 10000, "total_timesteps": 10000,
                                "error": None, "steps_per_second": 12.5})
    loss = "nan" if nan else "0.5"
    (d / "sb3" / "progress.csv").write_text(
        f"train/actor_loss,train/critic_loss,time/fps\n{loss},0.2,12\n,,13\n", encoding="utf-8")
    return d


def test_gate_end_to_end_pass_broken_and_refusals(tmp_path):
    import pytest as _pytest
    gate = _gate()
    good = (_train_dir(tmp_path), _eval_dir(tmp_path, "pilot", 30, 30),
            _eval_dir(tmp_path, "zero", 0, 60), _eval_dir(tmp_path, "pp", 40, 20))
    report = gate.assess(G4, "abc", *good)
    assert report["verdict"] == "PASS" and report["learning"]["last"] == 1.0
    assert report["recorded_not_gated"]["per_stratum_success"] == {"narrow": 0.4}
    # A Pure Pursuit control that fails the floor breaks the gate whatever the pilot did.
    broken = (good[0], good[1], good[2], _eval_dir(tmp_path, "pp_bad", 5, 55))
    assert gate.assess(G4, "abc", *broken)["verdict"] == "GATE BROKEN"
    with _pytest.raises(ValueError, match="G4's sets"):
        gate.assess(G4, "abc", good[0], _eval_dir(tmp_path, "wrong", 30, 30,
                                                   sets=["scenarios/tune/a.json"]), *good[2:])
    with _pytest.raises(ValueError, match="frozen environment"):
        gate.assess(G4, "abc", good[0], _eval_dir(tmp_path, "oldenv", 30, 30, env="old"),
                    *good[2:])


def test_gate_refuses_a_confirmatory_seed_and_catches_a_nan_loss(tmp_path):
    import pytest as _pytest
    gate = _gate()
    evals = (_eval_dir(tmp_path, "pilot", 30, 30), _eval_dir(tmp_path, "zero", 0, 60),
             _eval_dir(tmp_path, "pp", 40, 20))
    with _pytest.raises(ValueError, match="pilot range"):
        gate.assess(G4, "abc", _train_dir(tmp_path / "a", seed=0), *evals)
    report = gate.assess(G4, "abc", _train_dir(tmp_path / "b", nan=True), *evals)
    assert report["verdict"] == "INTEGRITY FAIL"
    assert report["integrity"]["checks"]["finite_losses"] is False


def test_recovery_statistics_known_answers():
    from pfbnav.eval.g4 import recovery_statistics
    eps = [
        {"scenario_seed": 1, "stratum": "narrow", "outcome": "success", "recovery_attempts": 2,
         "recovery_attempt_log": [{"ending": "cleared", "time_s": 3.0},
                                  {"ending": "new_stop", "time_s": 2.0}],
         "recovery_deadlock": None, "recovery_distance_m": 0.4, "recovery_time_s": 5.0},
        {"scenario_seed": 2, "stratum": "wide", "outcome": "unrecoverable_stop",
         "recovery_attempts": 1, "recovery_attempt_log": [{"ending": "no_permitted_direction"}],
         "recovery_deadlock": "no_permitted_direction", "recovery_distance_m": 0.0,
         "recovery_time_s": 0.0},
        {"scenario_seed": 3, "stratum": "wide", "outcome": "timeout"},     # a learned policy
        {"scenario_seed": 4, "stratum": "narrow", "outcome": "timeout", "recovery_attempts": 1,
         "recovery_attempt_log": [{"ending": None, "time_s": 16.0}],      # cut short at the end
         "recovery_deadlock": None, "recovery_distance_m": 0.29, "recovery_time_s": 16.0},
    ]
    s = recovery_statistics(eps)
    assert s["episodes"] == 4 and s["episodes_with_recovery"] == 3 and s["attempts"] == 4
    assert s["endings"] == {"cleared": 1, "new_stop": 1, "no_permitted_direction": 1}
    assert s["deadlocks"] == {"no_permitted_direction": 1}
    assert math.isclose(s["distance_m"], 0.69) and math.isclose(s["time_s"], 21.0)
    assert s["attempt_times_s"] == [3.0, 2.0, 0.0, 16.0] and s["attempts_cut_short"] == 1
    assert s["per_episode"][0]["endings"] == ["cleared", "new_stop"]


def test_controls_only_reads_the_two_controls_with_the_same_checks(tmp_path):
    import json

    import pytest as _pytest
    gate = _gate()
    zero, pp = _eval_dir(tmp_path, "zero", 0, 60), _eval_dir(tmp_path, "pp", 40, 20)
    for d in (zero, pp):
        (d / "episodes.jsonl").write_text(json.dumps({"outcome": "timeout"}) + "\n",
                                          encoding="utf-8")
    report = gate.assess_controls(G4, "abc", zero, pp)
    assert report["verdict"] == "CONTROLS SOUND"
    assert report["recovery"]["zero"]["episodes"] == 1
    bad = _eval_dir(tmp_path, "pp_bad", 5, 55)
    (bad / "episodes.jsonl").write_text("", encoding="utf-8")
    assert gate.assess_controls(G4, "abc", zero, bad)["verdict"] == "GATE BROKEN"
    with _pytest.raises(ValueError, match="frozen environment"):
        gate.assess_controls(G4, "abc", zero, _eval_dir(tmp_path, "old", 40, 20, env="old"))
    # B1 is RECORDED, never judged (ruled 2026-10-20): a B1 run that would fail the checkpoint
    # leaves the verdict alone, and its recovery statistics are reported.
    b1 = _eval_dir(tmp_path, "b1", 0, 60)
    (b1 / "episodes.jsonl").write_text(json.dumps({"outcome": "timeout"}) + "\n",
                                       encoding="utf-8")
    with_b1 = gate.assess_controls(G4, "abc", zero, pp, b1)
    assert with_b1["verdict"] == "CONTROLS SOUND" and with_b1["recovery"]["b1"]["episodes"] == 1
    assert "b1" in with_b1["recorded_not_gated"] and "b1" not in with_b1["controls"]


def test_a_not_released_attempt_is_a_suspected_defect_and_stops_the_gate(tmp_path):
    import json

    from pfbnav.eval.g4 import recovery_defects
    eps = [{"scenario_seed": 7, "index": 3, "recovery_attempt_log": [
        {"ending": "cleared", "start_step": 60}, {"ending": "not_released", "start_step": 140,
                                                  "heading_deg": -105.0}]},
           {"scenario_seed": 8, "index": 4, "recovery_attempt_log": [
               {"ending": "no_permitted_direction", "start_step": 61}]}]
    found = recovery_defects(eps)
    assert found == [{"scenario_seed": 7, "index": 3, "set": None, "start_step": 140,
                      "heading_deg": -105.0}]
    gate = _gate()
    zero, pp = _eval_dir(tmp_path, "zero", 0, 60), _eval_dir(tmp_path, "pp", 40, 20)
    (zero / "episodes.jsonl").write_text(json.dumps({"outcome": "timeout"}) + "\n",
                                         encoding="utf-8")
    (pp / "episodes.jsonl").write_text(json.dumps({"outcome": "timeout"}) + "\n",
                                       encoding="utf-8")
    # B1 carries the recovery; its suspected defect stops the gate though B1 is not judged.
    b1 = _eval_dir(tmp_path, "b1", 10, 50)
    (b1 / "episodes.jsonl").write_text("".join(json.dumps(e) + "\n" for e in eps),
                                       encoding="utf-8")
    report = gate.assess_controls(G4, "abc", zero, pp, b1)
    assert report["verdict"] == "STOP: SUSPECTED RECOVERY DEFECT"
    assert report["suspected_recovery_defects"][0]["scenario_seed"] == 7



# ------------------------------------------- A30 (ruled 2026-10-30): last window, full-distribution only
LEARN_A30 = {"window_fraction": 0.10, "min_last_success": 0.20, "stage": "full"}


def staged(outcomes, full_from):
    return [{**e, "curriculum_stage": "full" if i >= full_from else "wide_medium_mapped"}
            for i, e in enumerate(log(outcomes))]


def test_a30_learning_reads_only_the_last_window_without_improvement():
    """100 episodes, window 10. Two successes in the last 10: 0.20 passes with no improvement
    clause, even with 10 successes early on; one success fails."""
    a = ["success"] * 10 + ["timeout"] * 90
    a[95] = a[99] = "success"
    got = learning_criterion(staged(a, 40), **LEARN_A30)
    assert got["window"] == 10 and math.isclose(got["last"], 0.2) and got["passed"]
    assert "improvement" not in got and "first" not in got
    a[99] = "timeout"
    assert learning_criterion(staged(a, 40), **LEARN_A30)["passed"] is False


def test_a30_learning_refuses_a_window_with_curriculum_episodes_or_untagged_ones():
    outcomes = ["success"] * 100
    got = learning_criterion(staged(outcomes, 95), **LEARN_A30)
    assert not got["passed"] and "5 of the last 10" in got["reason"]
    assert not learning_criterion(log(outcomes), **LEARN_A30)["passed"]   # no stage recorded
    assert learning_criterion([], **LEARN_A30)["passed"] is False


def test_the_a30_draft_states_the_amended_criterion():
    from pathlib import Path

    import yaml
    draft = yaml.safe_load((Path(__file__).resolve().parents[1] / "docs" / "pending"
                            / "gates_A30.yaml").read_text(encoding="utf-8"))["G4_learned_pilot"]
    assert draft["learning"] == LEARN_A30
    assert draft["pilot_training_seeds"] == [9000000, 9000001, 9000002]
