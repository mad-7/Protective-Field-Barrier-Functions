"""YAML configuration with single-parent inheritance via a `defaults:` key."""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


def _deep_merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(path: str | Path) -> dict[str, Any]:
    """Load a config, resolving `defaults:` relative to the file's directory."""
    path = Path(path).resolve()
    with path.open() as f:
        cfg = yaml.safe_load(f) or {}
    parent = cfg.pop("defaults", None)
    if parent is None:
        return cfg
    return _deep_merge(load_config(path.parent / parent), cfg)
