"""Known answers for the A20 simulator-session diagnostics' arithmetic (hard rule 6), on synthetic
data: `scripts/check_acceleration.py` (the limiter binds, not the torque cap) and
`scripts/check_memory.py` (growth per 100 episodes)."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]


def _script(name):
    spec = importlib.util.spec_from_file_location(name, REPO / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ACC = _script("check_acceleration")
MEM = _script("check_memory")
LIM = {"a_max": 0.5, "alpha_max": 1.4, "a_decel_max": 1.0}
DT = 0.05


def _rows(target, *, a=0.5, capped_at=None, lag_steps=2, hold=20, brake=30):
    """A synthetic forward profile: the applied command ramps at `a`, the true speed follows it
    `lag_steps` behind, then holds and brakes at 1.0 m/s^2."""
    rows, v, t = [], 0.0, 0.0
    applied = []
    while v < target - 1e-12:
        v = min(v + a * DT, target)
        applied.append(v)
    ramp_n = len(applied)
    applied += [target] * hold
    w = target
    while w > 0.0:
        w = max(w - 1.0 * DT, 0.0)
        applied.append(w)
    applied += [0.0] * (brake - 0)
    for k, cmd in enumerate(applied):
        true = applied[max(k - lag_steps, 0)] if k >= lag_steps else 0.0
        phase = "ramp" if k < ramp_n else ("hold" if k < ramp_n + hold else "brake")
        rows.append({"t": k * DT, "applied": [cmd, 0.0, 0.0], "true": [true, 0.0, 0.0],
                     "capped": int(capped_at is not None and k == capped_at), "phase": phase})
        t += DT
    return {"rows": rows, "dt": DT, "limits": LIM}


def test_a_clean_ramp_passes_and_measures_the_limit():
    out = ACC.assess(_rows(0.40))
    assert out["C1_limiter_binds"] and out["C2_torque_cap_wheel_steps_in_ramp"] == 0
    assert out["passes"]
    assert out["achieved_over_a_max"] == pytest.approx(1.0, abs=1e-6)   # lagged, same slope


def test_a_wheel_at_the_torque_cap_during_the_ramp_fails():
    out = ACC.assess(_rows(0.40, capped_at=3))
    assert out["C2_torque_cap_wheel_steps_in_ramp"] == 1 and not out["passes"]
    # at the cap AFTER the ramp (holding, braking) is reported but does not fail C2
    out = ACC.assess(_rows(0.40, capped_at=40))
    assert out["C2_torque_cap_wheel_steps_in_ramp"] == 0 and out["passes"]
    assert out["torque_cap_wheel_steps_total"] == 1


def test_a_ramp_faster_than_the_limit_breaks_c1():
    assert not ACC.assess(_rows(0.40, a=0.7))["C1_limiter_binds"]


def test_the_ramp_slope_known_answer():
    t = np.arange(0.0, 1.0, 0.05)
    assert ACC.ramp_slope(t, 0.5 * t, 0.4) == pytest.approx(0.5)
    assert ACC.ramp_slope(t, 0.0 * t, 0.4) is None      # never rises: no slope


def test_the_memory_slope_per_100_episodes_known_answer():
    rng = np.random.default_rng(1)
    flat = 500.0 + rng.normal(0.0, 1.0, 200)
    growing = 500.0 + 0.2 * np.arange(200) + rng.normal(0.0, 1.0, 200)   # 20 MB per 100
    s0, se0 = MEM.slope_per_100(list(flat))
    s1, _se1 = MEM.slope_per_100(list(growing))
    assert abs(s0) < 3 * se0 + 0.5 and s0 <= MEM.BOUND_MB_PER_100
    assert s1 == pytest.approx(20.0, abs=1.0) and s1 > MEM.BOUND_MB_PER_100
    assert se0 < MEM.BOUND_MB_PER_100 / 3.0                          # 1 MB noise: conclusive
