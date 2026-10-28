"""Known-answer tests for the relabelling diagnostic (hard rule 6), on synthetic final records."""
from pathlib import Path

from pfbnav.config import load_config
from pfbnav.envs.reward import RewardWeights
from pfbnav.eval.relabel import relabel

REPO = Path(__file__).resolve().parents[1]
W = RewardWeights.from_config(load_config(REPO / "configs" / "base.yaml"))


def _final(name, outcome, *, stop, blocked, breached, at_rest, step=None, contact=False):
    return {"name": name, "outcome": outcome, "step": W.max_steps if step is None else step,
            "in_contact": contact,
            "certified": {"stop_active": stop, "restart_blocked": blocked,
                          "breached": breached, "at_rest": at_rest}}


def test_only_the_breached_at_rest_timeout_changes():
    finals = [
        _final("blocked", "unrecoverable_stop", stop=True, blocked=True, breached=False,
               at_rest=True),
        _final("inside_margin", "timeout", stop=True, blocked=False, breached=True, at_rest=True),
        _final("moving", "timeout", stop=True, blocked=False, breached=True, at_rest=False),
        _final("free", "timeout", stop=False, blocked=False, breached=False, at_rest=False),
        _final("hit", "collision", stop=True, blocked=False, breached=True, at_rest=True,
               step=40, contact=True),
    ]
    rows = {r["name"]: r for r in relabel(finals, W)}
    assert all(r["reproduced"] for r in rows.values())
    assert [n for n, r in rows.items() if r["changed"]] == ["inside_margin"]
    assert rows["inside_margin"]["new_rule"] == "unrecoverable_stop"
    assert rows["moving"]["new_rule"] == "timeout"


def test_a_record_the_old_rule_does_not_reproduce_is_reported():
    """The positive control: a label the old rule could not have produced is flagged, so a
    corrupted or hand-edited record is not silently relabelled."""
    rows = relabel([_final("odd", "unrecoverable_stop", stop=False, blocked=False,
                           breached=False, at_rest=True)], W)
    assert rows[0]["reproduced"] is False
