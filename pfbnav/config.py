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
        return _with_baseline_files(cfg, path.parent)
    return _deep_merge(load_config(path.parent / parent), cfg)


def _with_baseline_files(cfg: dict[str, Any], root: Path) -> dict[str, Any]:
    """Merge the files named by `baseline_files` into cfg['baselines'] and drop the key.

    A baseline file holds {controller, base_key, params}: `params` becomes
    cfg['baselines'][base_key]. A file with neither key (the shared one) holds blocks that become
    cfg['baselines'][name] each. A block already in cfg['baselines'] is never overwritten here:
    two homes for one parameter is the ambiguity this mechanism removes, so it is an error.
    """
    files = cfg.pop("baseline_files", None)
    if not files:
        return cfg
    baselines = cfg.setdefault("baselines", {})
    for rel in files:
        with (root / rel).open() as f:
            doc = yaml.safe_load(f) or {}
        blocks = {doc["base_key"]: doc["params"]} if "base_key" in doc else {
            k: v for k, v in doc.items()}
        for name, block in blocks.items():
            if name in baselines:
                raise ValueError(f"baselines.{name} is defined twice (base.yaml and {rel})")
            baselines[name] = block
    return cfg
