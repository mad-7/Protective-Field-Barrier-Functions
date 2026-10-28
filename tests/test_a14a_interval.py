"""A14a (ruled 2026-10-26): the confirmatory interval is the closed-form t interval on the log rate
ratio, scenario and seed variance combined with K/(K-1), the ruled df -- and the implementation equals
the form that passed calibration (commit 0329941, runs/power/CORRECTED_20261003T181634Z_seed0,
classical_df flat_k_minus_1).

That code is kept as tests/data/rate_ratio_0329941.py.txt, extracted unchanged from commit 0329941 of
the original repository (the repository moved on 2026-10-28 and its history starts at a later
commit, so `git show` of that commit no longer works). Its SHA-256 is pinned below."""
from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy import stats

from pfbnav.eval import rate_ratio as rr

REPO = Path(__file__).resolve().parents[1]
CALIBRATED_AT = "0329941"            # the commit of the calibration run that adopted flat K - 1
CALIBRATED_COPY = REPO / "tests" / "data" / "rate_ratio_0329941.py.txt"
CALIBRATED_SHA256 = "27435685271543ce4a853fcfbd462dad8b6622d91f16a19e10eb7efa67268008"
LEVEL = 0.05 / 3                     # the power level alpha/3 (A14a); any level would do


@pytest.fixture(scope="module")
def old(tmp_path_factory):
    """pfbnav/eval/rate_ratio.py exactly as it was when calibration passed, imported from a copy."""
    raw = CALIBRATED_COPY.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == CALIBRATED_SHA256,         f"tests/data/rate_ratio_0329941.py.txt is not the code of commit {CALIBRATED_AT}"
    source = raw.decode("utf-8")
    path = tmp_path_factory.mktemp("calibrated") / "rate_ratio_calibrated.py"
    path.write_text(source, encoding="utf-8")
    spec = importlib.util.spec_from_file_location("rate_ratio_calibrated", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module            # dataclasses look their module up here
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.pop(spec.name, None)


def _arms(rng, reps=40, n=30):
    d = rng.uniform(5.0, 25.0, size=(reps, n))
    l5 = rng.poisson(0.03 * d[:, None, :] * rng.gamma(4, 0.25, size=(reps, 10, 1)))
    l4 = rng.poisson(0.05 * d[:, None, :] * rng.gamma(4, 0.25, size=(reps, 10, 1)))
    b1 = rng.poisson(0.06 * d[:, None, :], size=(reps, 1, n))
    return d, l5, l4, b1


def test_the_confirmatory_interval_equals_the_code_that_passed_calibration(old):
    d, l5, l4, b1 = _arms(np.random.default_rng(7))
    for s_b, rule in ((b1, old.FLAT), (l4, old.WS)):
        new = rr.confirmatory_interval(rr.arm_terms(l5, d), rr.arm_terms(s_b, d), LEVEL)
        a, b = old.arm_terms(l5, d), old.arm_terms(s_b, d)
        est = old.log_rate_ratio_terms(a, b)
        var, half = old.corrected_interval(old.components(a, b), LEVEL, rule)
        assert new["df_rule"] == rule
        assert np.array_equal(new["estimate"], est) and np.array_equal(new["variance"], var)
        assert np.array_equal(new["lower"], est - half) and np.array_equal(new["upper"], est + half)
        lower_rej, higher_rej = old.corrected_verdict(a, b, LEVEL, classical_df=rule)
        assert np.array_equal(new["upper"] < 0.0, lower_rej)
        assert np.array_equal(new["lower"] > 0.0, higher_rej)


def test_positive_control_a_different_df_rule_would_be_caught(old):
    """The equality above can fail: against a classical arm, Welch-Satterthwaite gives a different
    interval from the calibrated flat K - 1 on the same data."""
    d, l5, _, b1 = _arms(np.random.default_rng(7))
    a, b = old.arm_terms(l5, d), old.arm_terms(b1, d)
    _, flat = old.corrected_interval(old.components(a, b), LEVEL, old.FLAT)
    _, ws = old.corrected_interval(old.components(a, b), LEVEL, old.WS)
    assert not np.allclose(flat, ws)


def test_the_closed_form_known_answer_from_the_a14a_text():
    """Written from endpoints.yaml's text, not from the implementation: variance = scenario part +
    K/(K-1) x each learned arm's seed part; half-width t(level/2, df) sqrt(variance); df K - 1 of the
    learned arm against a classical arm, Welch-Satterthwaite between learned arms."""
    d, l5, l4, b1 = _arms(np.random.default_rng(11))
    a, c, b = rr.arm_terms(l5, d), rr.arm_terms(l4, d), rr.arm_terms(b1, d)
    comp = rr.components(a, b)
    var = comp.scenario + 10 / 9 * comp.seed_a
    half = stats.t.isf(LEVEL / 2, 9) * np.sqrt(var)
    got = rr.confirmatory_interval(a, b, LEVEL)
    assert np.allclose(got["upper"] - got["estimate"], half, rtol=1e-12)
    comp = rr.components(a, c)
    parts = [(comp.scenario, 29), (10 / 9 * comp.seed_a, 9), (10 / 9 * comp.seed_b, 9)]
    var = sum(v for v, _ in parts)
    df = var**2 / sum(v**2 / k for v, k in parts)
    got = rr.confirmatory_interval(a, c, LEVEL)
    assert np.allclose(got["estimate"] - got["lower"], stats.t.isf(LEVEL / 2, df) * np.sqrt(var),
                       rtol=1e-12)
    # not the uncorrected normal interval
    normal = stats.norm.isf(LEVEL / 2) * np.sqrt(rr.variance_terms(a, b, rr.TWO_LEVEL))
    assert np.all(rr.confirmatory_interval(a, b, LEVEL)["upper"] - got["estimate"] != normal)


def test_two_classical_arms_are_refused():
    d, _, _, b1 = _arms(np.random.default_rng(1))
    with pytest.raises(ValueError, match="no learned arm"):
        rr.confirmatory_interval(rr.arm_terms(b1, d), rr.arm_terms(b1, d), LEVEL)
