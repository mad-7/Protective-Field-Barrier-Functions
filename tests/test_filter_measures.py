"""The PFB's correction measures (ruled 2026-11-07, item 5): known answers (hard rule 6)."""
import pytest

from pfbnav.eval.filter_measures import BIN, ValueHistogram, step_values, summarise
from pfbnav.eval.runner import outcome_summary


def test_one_step_values():
    """Asked (0.6, 0.0, 1.0), passed (0.3, 0.4, 0.2), applied (0.3, 0.4): the translational
    correction is |(-0.3, 0.4)| = 0.5 m/s, the yaw 0.8 rad/s; asked 0.6 m/s, executed 0.5 m/s."""
    v = step_values({"u": [0.3, 0.4, 0.2], "nominal": [0.6, 0.0, 1.0]}, [0.3, 0.4, 0.2])
    assert v == pytest.approx({"correction_mps": 0.5, "correction_yaw_rps": 0.8,
                               "requested_mps": 0.6, "executed_mps": 0.5})


def test_histogram_quantiles_and_shares():
    """Values 0.00 (x6), 0.06 (x2), 0.12, 0.30: median 0.005 (the upper edge of bin 0), p95 0.305,
    shares above 0.05 / 0.10 / 0.20: 0.4, 0.2, 0.1. A value on a bin edge goes to the upper bin."""
    h = ValueHistogram()
    for x in [0.0] * 6 + [0.06, 0.06, 0.12, 0.30]:
        h.add(x)
    assert h.n == 10 and h.quantile(0.5) == pytest.approx(BIN)
    assert h.quantile(0.95) == pytest.approx(0.305)
    assert [h.share_above(t) for t in (0.05, 0.10, 0.20)] == pytest.approx([0.4, 0.2, 0.1])
    e = ValueHistogram()
    e.add(0.05)
    assert e.share_above(0.05) == 1.0 and e.share_above(0.055) == 0.0
    with pytest.raises(ValueError):
        h.add(-0.1)
    assert ValueHistogram().quantile(0.5) is None


def test_pooled_over_episodes_and_absent_without_the_filter():
    """Two episodes pooled bin by bin; an arm without `pfb_measures` reports None."""
    def ep(values):
        hs = {k: ValueHistogram() for k in ("correction_mps", "correction_yaw_rps",
                                            "requested_mps", "executed_mps")}
        for x in values:
            for h in hs.values():
                h.add(x)
        return {"outcome": "success", "episode_return": 0.0,
                "pfb_measures": {k: h.to_record() for k, h in hs.items()}}
    s = summarise([ep([0.0, 0.0, 0.3]), ep([0.3, 0.3])])
    assert s["steps"] == 5 and s["correction_mps"]["median"] == pytest.approx(0.305)
    assert s["correction_share_above_mps"]["0.2"] == pytest.approx(0.6)
    assert summarise([{"outcome": "success"}]) is None
    assert outcome_summary([{"outcome": "success", "episode_return": 0.0}])["pfb_measures"] is None
