"""Memory in every run record (ruled 2026-10-19): `pfbnav/utils/memory.py`, known answers."""
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
