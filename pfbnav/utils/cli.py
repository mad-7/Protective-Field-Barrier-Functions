"""Command-line helpers shared by the scripts. Pure; no simulator.

`--display` overrides `sim.display_enabled` for one run. Rendering is off by default
because it costs ~7 ms of a ~27 ms step and changes no physics (trajectories checked
bit-identical, `scripts/regress_display_off.py`), which is also why an override is safe to
offer: watching a run cannot change its result, only its speed. Every run records what was
actually in force in `provenance.json`, so a watched run is labelled as one.
"""
from __future__ import annotations

import copy
from typing import Any

CHOICES = ("config", "on", "off")


def add_display_flag(parser, default: str = "config") -> None:
    """Add `--display {config,on,off}` to a script's parser. `default` is the script's own default:
    `train.py` and `evaluate.py` use "on" (the researcher's choice, 2026-11-07), every other script
    "config"."""
    if default not in CHOICES:
        raise ValueError(f"display default must be one of {CHOICES}, got {default!r}")
    parser.add_argument(
        "--display", choices=CHOICES, default=default,
        help=f"rendering for this run (default: {default}): 'config' uses sim.display_enabled "
             "(off), 'on' shows the scene so you can watch, 'off' forces it off. Watching costs "
             "wall clock and changes no physics; the run records what was in force.")


def add_set_flag(parser) -> None:
    """Add `--set KEY=VALUE` (repeatable): override one config key for this run (`pfbnav.utils.overrides`)."""
    parser.add_argument(
        "--set", dest="overrides", action="append", default=[], metavar="KEY=VALUE",
        help="override a config key for this run, e.g. --set training.total_timesteps=100000 "
             "(the key must exist; the override is recorded in the run's config.yaml)")


def apply_set(cfg: dict[str, Any], items: list[str]) -> dict[str, Any]:
    """A copy of `cfg` with the `--set` overrides applied; `cfg` itself when there are none."""
    from pfbnav.utils.overrides import apply_overrides
    try:
        return apply_overrides(cfg, items)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


PFB_CHOICES = ("config", "on", "off")


def add_pfb_flag(parser) -> None:
    """Add `--pfb {config,on,off}`: put the PFB beneath the chosen classical baseline for this run."""
    parser.add_argument(
        "--pfb", choices=PFB_CHOICES, default="config",
        help="'on' puts the PFB between the controller and the certified layer (at the frozen "
             "alpha, nothing else changes); 'off' forces it off; 'config' follows filter.enabled.")


def apply_pfb(cfg: dict[str, Any], choice: str) -> dict[str, Any]:
    """A copy of `cfg` with the PFB switch set by the flag (`pfbnav.integration`)."""
    from pfbnav.integration import with_pfb, without_pfb
    if choice not in PFB_CHOICES:
        raise ValueError(f"pfb must be one of {PFB_CHOICES}, got {choice!r}")
    if choice == "config":
        return cfg
    return with_pfb(cfg) if choice == "on" else without_pfb(cfg)


def apply_display(cfg: dict[str, Any], choice: str) -> dict[str, Any]:
    """A copy of `cfg` with `sim.display_enabled` set by the flag. `config` changes nothing."""
    if choice not in CHOICES:
        raise ValueError(f"display must be one of {CHOICES}, got {choice!r}")
    if choice == "config":
        return cfg
    out = copy.deepcopy(cfg)
    out["sim"]["display_enabled"] = choice == "on"
    return out
