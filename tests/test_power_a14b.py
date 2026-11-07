"""A14b's wall-clock routine (ruled 2026-11-07): a known answer (hard rule 6)."""
import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def _mod():
    spec = importlib.util.spec_from_file_location("power_a14b", REPO / "scripts" / "power_a14b.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_wall_clock_counts_classical_once_and_learned_k_times():
    """2 classical arms at 100 s and 50 s, 1 learned arm at 30 s x K = 10, on 2 instances: per
    scenario 100 + 50 + 300 = 450 s; 80 scenarios = 36,000 s / 2 = 5 h."""
    p = {"episode_wall_s": {"A": 100, "B": 50, "C": 30}, "seeds_per_learned_arm": 10,
         "classical_arms_evaluated": ["A", "B"], "learned_arms_evaluated": ["C"], "instances": 2}
    assert _mod().wall_clock_h(80, p) == pytest.approx(5.0)
