"""Command-line config overrides: `--set section.key=value`. Pure; no simulator.

Every setting lives in a YAML config (`configs/base.yaml` and one overlay per experiment). A run that needs a
different value for one key (a short smoke run, a sensitivity point) says so on the command line instead of
copying a config:

    python scripts/train.py --config configs/experiments/L1_sac_shaped.yaml --seed 9000010 \
        --set training.total_timesteps=100000 --set experiment=DIAG_L1_smoke

The overrides are applied to the resolved config, so they are recorded in the run's `config.yaml` (and listed
under `cli_overrides`). Three guards keep a command line from quietly rewriting the study:

  * the key must already exist (a typo is an error, not a new key);
  * the new value must have the old value's kind (a number for a number, a list for a list, ...);
  * a key under a RULED section (reward, observation, filter, certified layer, actuation, scanners) prints a
    warning: changing it needs a decision-log entry (hard rule 7) and, for a learned arm, applies identically
    to every arm (hard rule 17).
"""
from __future__ import annotations

import copy
import sys
from collections.abc import Callable, Iterable
from typing import Any

import yaml

# Sections whose values are ruled; an override there warns.
RULED_PREFIXES = ("reward.", "env.observation.", "filter.", "certified.", "actuation.", "scanners.",
                  "planner.", "robot.")


def parse_item(item: str) -> tuple[list[str], Any]:
    """`'a.b=3'` -> (['a', 'b'], 3). The value is read as YAML, except that `1e-4` (which YAML reads as a
    string) is read as a number."""
    if "=" not in item:
        raise ValueError(f"override {item!r} is not KEY=VALUE")
    key, raw = item.split("=", 1)
    key, raw = key.strip(), raw.strip()
    if not key or any(not part for part in key.split(".")):
        raise ValueError(f"override {item!r} has an empty key part")
    try:
        value: Any = int(raw)
    except ValueError:
        try:
            value = float(raw)
        except ValueError:
            value = yaml.safe_load(raw)
    return key.split("."), value


def _kind(value: Any) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, (int, float)):
        return "number"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list):
        return "list"
    if isinstance(value, dict):
        return "mapping"
    return "null"


def apply_overrides(cfg: dict[str, Any], items: Iterable[str],
                    report: Callable[[str], None] | None = None) -> dict[str, Any]:
    """A copy of `cfg` with each `KEY=VALUE` applied in order. Raises ValueError on a missing key, a value of
    another kind, or a malformed item. `report` (default: stderr) receives the warnings."""
    items = list(items)
    if not items:
        return cfg
    say = report or (lambda m: print(m, file=sys.stderr))
    out = copy.deepcopy(cfg)
    for item in items:
        path, value = parse_item(item)
        node = out
        for part in path[:-1]:
            if not isinstance(node, dict) or part not in node:
                raise ValueError(f"override {item!r}: no key {'.'.join(path[:path.index(part) + 1])!r} in the config")
            node = node[part]
        leaf = path[-1]
        if not isinstance(node, dict) or leaf not in node:
            raise ValueError(f"override {item!r}: no key {'.'.join(path)!r} in the config")
        old = node[leaf]
        if old is not None and _kind(value) != _kind(old):
            raise ValueError(f"override {item!r}: {'.'.join(path)} is a {_kind(old)} ({old!r}), "
                             f"the new value is a {_kind(value)} ({value!r})")
        node[leaf] = value
        dotted = ".".join(path)
        if any((dotted + ".").startswith(p) for p in RULED_PREFIXES):
            say(f"WARNING: --set {dotted} changes a RULED setting: record it in docs/decision_log.md "
                "(hard rule 7), and for a learned arm apply it to every arm (hard rule 17)")
    out["cli_overrides"] = items
    return out
