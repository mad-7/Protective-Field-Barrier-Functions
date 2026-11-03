"""Whether a config may run with the PFB on: only at the alpha frozen after its tuning with B7.

Alpha was tuned with B7 on the tune layouts and frozen by hand for L3, L4, L5 and B7 on 2026-11-03
(`filter.alpha_frozen` names the ruling; `filter.alpha` in `configs/base.yaml` is the value). A
config that turns the filter on before that, or carries another alpha than the base config's (an
experiment overlay quietly retuning it), is refused. Outside the environment's import closure: this
gates scripts, it changes no step.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from pfbnav.config import load_config

BASE = Path(__file__).resolve().parents[2] / "configs" / "base.yaml"


def filter_problem(cfg: dict[str, Any], base: dict[str, Any] | None = None) -> str | None:
    """None when the filter is off, or on at the frozen alpha; otherwise why not."""
    if not cfg["filter"]["enabled"]:
        return None
    base = load_config(BASE) if base is None else base
    if not base["filter"].get("alpha_frozen"):
        return ("filter.enabled is true, but the PFB's alpha is not frozen yet (tuned with B7, Q4; "
                "`filter.alpha_frozen` in configs/base.yaml)")
    if float(cfg["filter"]["alpha"]) != float(base["filter"]["alpha"]):
        return (f"filter.alpha {cfg['filter']['alpha']} is not the frozen alpha "
                f"{base['filter']['alpha']} (configs/base.yaml, {base['filter']['alpha_frozen']})")
    return None
