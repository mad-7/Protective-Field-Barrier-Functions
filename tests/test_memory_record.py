"""Memory in every run record (ruled 2026-10-19): `pfbnav/utils/memory.py`, known answers."""
import numpy as np
import pytest

from pfbnav.utils import memory


def test_growth_per_100_episodes_known_answers():
    assert memory.growth_per_100(187.0, 191.0, 200) == pytest.approx(2.0)
    assert memory.growth_per_100(700.0, 690.0, 50) == pytest.approx(-20.0)
    assert memory.growth_per_100(1.0, 2.0, 0) is None


def test_the_record_measures_this_process_and_says_why_the_simulator_is_missing():
    rec = memory.MemoryRecord(port=None)
    assert rec.start["python"] is not None and rec.start["python"] > 0.0
    assert rec.start["sim"] is None
    out = rec.report(10)
    assert out["status"]["sim"] == "no simulator port given"
    assert out["python"]["private_start_mb"] > 0.0
    assert out["python"]["growth_mb_per_100_episodes"] is not None
    assert out["sim"]["growth_mb_per_100_episodes"] is None
    assert "MEMORY_20261018T203246Z" in out["reference"]


def test_without_psutil_every_value_is_missing_with_a_status(monkeypatch):
    monkeypatch.setattr(memory, "_psutil", lambda: None)
    out = memory.MemoryRecord(port=23000).report(5)
    assert out["status"] == {"python": "psutil not installed", "sim": "psutil not installed"}
    assert out["python"]["private_start_mb"] is None and out["sim"]["private_end_mb"] is None


def test_an_unidentifiable_simulator_is_a_status_not_a_crash(monkeypatch):
    def nothing(port):
        raise LookupError("cannot identify the CoppeliaSim process (0 candidates)")
    monkeypatch.setattr(memory, "coppelia_process", nothing)
    out = memory.MemoryRecord(port=23000).report(5)
    assert out["status"]["sim"].startswith("cannot identify")
    assert out["python"]["private_end_mb"] is not None


# ------------------------------------- the training memory criterion (ruled 2026-10-30), known answers
CHECK = {"total_timesteps": 100_000, "settled_from": 3000, "mark_fraction": 0.10,
         "max_growth_mb": 100.0}


def _trend(fn, every=1000, total=100_000):
    return [{"timestep": t, "python": fn(t)} for t in [1, *range(every, total + 1, every)]]


def test_a_start_up_jump_then_flat_passes():
    # CUDA (+528) at the start and the first update's workspaces (+400) by step 2,500, then flat
    # with +-20 MB allocator noise.
    noise = np.random.default_rng(0).uniform(-20, 20, 200)
    out = memory.levels_off(_trend(lambda t: 1900 + (528 if t > 1 else 0) + (400 if t >= 2500 else 0)
                                   + noise[t // 1000]), **CHECK)
    assert out["passed"] and out["reason"] == "levelled off"
    assert abs(out["slope_after_mark_mb_per_100k_steps"]) < 30


def test_a_linear_leak_at_the_pilot_s_unexplained_rate_fails():
    # ~600 MB per 300k steps beyond the one-off costs: 180 MB over the 90k steps after the mark.
    out = memory.levels_off(_trend(lambda t: 2800 + 0.002 * t), **CHECK)
    assert not out["passed"] and out["growth_after_mark_mb"] == pytest.approx(180.0)
    assert out["slope_after_mark_mb_per_100k_steps"] == pytest.approx(200.0)


def test_growth_that_settles_before_the_mark_passes_and_after_it_fails():
    early = memory.levels_off(_trend(lambda t: 2000 + min(t, 9000) * 0.05), **CHECK)
    late = memory.levels_off(_trend(lambda t: 2000 + min(t, 40_000) * 0.05), **CHECK)
    assert early["passed"] and early["growth_baseline_to_mark_mb"] == pytest.approx(300.0)
    assert not late["passed"] and late["growth_after_mark_mb"] == pytest.approx(1500.0)


def test_a_check_that_stopped_early_or_has_no_samples_cannot_pass():
    short = memory.levels_off(_trend(lambda t: 2000.0, total=60_000), **CHECK)
    assert not short["passed"] and "before the run's end" in short["reason"]
    none = memory.levels_off([{"timestep": t, "python": None} for t in range(0, 100_001, 1000)],
                             **CHECK)
    assert not none["passed"]


def test_the_criterion_is_the_declared_one_in_the_config():
    from pathlib import Path

    from pfbnav.config import load_config
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    assert cfg["training"]["memory_check"] == CHECK
    assert cfg["training"]["memory_sample_every"] == 1000


def test_the_memory_check_never_uses_a_pilot_seed():
    import importlib.util
    from pathlib import Path

    import yaml
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location("ctm", root / "scripts" / "check_training_memory.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    gates = yaml.safe_load((root / "preregistration" / "gates.yaml").read_text(encoding="utf-8"))
    assert mod.seed_problem(9_200_000, gates) is None
    assert "pilot training seed" in mod.seed_problem(9_000_001, gates)
    assert "outside the pilot range" in mod.seed_problem(1_000_000, gates)
    assert mod.positive_control()["passed"]
