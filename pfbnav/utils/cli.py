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


def add_display_flag(parser) -> None:
    """Add `--display {config,on,off}` to a script's parser."""
    parser.add_argument(
        "--display", choices=CHOICES, default="config",
        help="rendering for this run: 'config' uses sim.display_enabled (off by default), "
             "'on' shows the scene so you can watch, 'off' forces it off. Watching costs "
             "wall clock and changes no physics; the run records what was in force.")


def apply_display(cfg: dict[str, Any], choice: str) -> dict[str, Any]:
    """A copy of `cfg` with `sim.display_enabled` set by the flag. `config` changes nothing."""
    if choice not in CHOICES:
        raise ValueError(f"display must be one of {CHOICES}, got {choice!r}")
    if choice == "config":
        return cfg
    out = copy.deepcopy(cfg)
    out["sim"]["display_enabled"] = choice == "on"
    return out
