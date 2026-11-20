"""Putting the PFB beneath a baseline: the one switch, and nothing else.

The PFB is not part of any controller. It sits in the environment's step loop between the
controller's command and the certified layer (`pfbnav/envs/step_loop.py`), so wrapping a baseline
means turning `filter.enabled` on and changing nothing else: the baseline's parameters, its route,
its recovery, the certified layer and the actuation limits stay as they are. `with_pfb` is that
switch as a function; `configs/experiments/B7..B10_*_pfb.yaml` are the same switch as files, and
`tests/test_pfb_integration.py` checks that the two agree.

The PFB runs only at the alpha frozen after its tuning (`pfbnav/utils/filter_ruling.py`); a config
that carries another alpha is refused here, not silently run.
"""
from __future__ import annotations

import copy
from typing import Any

from pfbnav.utils.filter_ruling import filter_problem


def with_pfb(cfg: dict[str, Any]) -> dict[str, Any]:
    """A copy of `cfg` with the PFB on and every other key unchanged. Raises ValueError if the
    result may not run (the filter's alpha is not the frozen one)."""
    out = copy.deepcopy(cfg)
    out["filter"]["enabled"] = True
    problem = filter_problem(out)
    if problem:
        raise ValueError(problem)
    return out


def without_pfb(cfg: dict[str, Any]) -> dict[str, Any]:
    """A copy of `cfg` with the PFB off and every other key unchanged."""
    out = copy.deepcopy(cfg)
    out["filter"]["enabled"] = False
    return out


def pfb_pair(cfg: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """(alone, wrapped): the two members of a paired comparison, differing in the switch only."""
    return without_pfb(cfg), with_pfb(cfg)
