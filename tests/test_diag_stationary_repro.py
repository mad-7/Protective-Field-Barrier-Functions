"""The traced rerun's declared outcomes (ruled 2026-10-23): known answers, before it runs."""
import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("diag_stationary_repro",
                                              REPO / "scripts" / "diag_stationary_repro.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
LOW, HIGH = [0] * 20, [150] * 20


def test_the_declared_outcomes():
    none = [0] * 20
    one = [0] * 19 + [1]
    assert mod.verdict(none, one, LOW, HIGH).startswith("REPRODUCES ONLY UNDER LOAD")
    assert mod.verdict(one, one, LOW, HIGH).startswith("REPRODUCES UNDER BOTH")
    assert mod.verdict(none, none, LOW, HIGH).startswith("NEVER REPRODUCES")
    assert mod.verdict(one, none, LOW, HIGH).startswith("REPRODUCES ONLY WITHOUT LOAD")


def test_without_the_load_actually_induced_the_verdict_is_inconclusive():
    one = [0] * 19 + [1]
    assert mod.verdict([0] * 20, one, LOW, [50] * 20).startswith("INCONCLUSIVE")
    assert mod.verdict([0] * 20, one, HIGH, HIGH).startswith("INCONCLUSIVE")
