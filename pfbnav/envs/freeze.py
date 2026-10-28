"""The environment's code identity, for the freeze L1 trains under (ruled 2026-10-03).

L1 may train while WP2 builds the PFB only on a FROZEN environment: its code hash is recorded
before L1's first seed, every training run checks it, and any filter-off behaviour change
invalidates prior L1 runs. "The environment" is defined mechanically, so nobody has to remember
to add a file to a list:

  * every `pfbnav` module in the **import closure of `pfbnav.envs.nav_env`** -- every import,
    including those inside functions -- and
  * the **data files it reads**: the scanner's Lua script and the footprint table the config
    names.

`pfbnav/safety/pfb.py` is outside the closure today because nothing on the filter-off path imports
it. When WP2 wires it into the step loop it enters the closure, the hash changes, and the
re-freeze has to go through the filter-off bit-identity test
(`tests/test_step_loop_sim.py::test_filter_off_is_bit_identical_with_and_without_the_pfb_slot`).
That is the point: the freeze cannot move silently.

File hashes are of text with line endings normalised (`pfbnav/utils/hashing.py`), so the identity
agrees between this laptop and the training host.
"""
from __future__ import annotations

import ast
import hashlib
from pathlib import Path
from typing import Any

from pfbnav.utils.hashing import text_sha256

REPO_ROOT = Path(__file__).resolve().parents[2]
ENTRY = "pfbnav.envs.nav_env"
LUA_SCRIPTS = ("lua/scanner_v2.lua",)


def _module_path(module: str, root: Path) -> Path | None:
    base = root / module.replace(".", "/")
    for candidate in (base.with_suffix(".py"), base / "__init__.py"):
        if candidate.is_file():
            return candidate
    return None


def import_closure(entry: str = ENTRY, root: Path = REPO_ROOT) -> list[Path]:
    """Every `pfbnav` source file reachable by import from `entry`, sorted."""
    seen: set[str] = set()
    files: set[Path] = set()
    todo = [entry]
    while todo:
        module = todo.pop()
        if module in seen:
            continue
        seen.add(module)
        path = _module_path(module, root)
        if path is None:
            continue
        files.add(path)
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
                names = [node.module] + [f"{node.module}.{a.name}" for a in node.names]
            elif isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            else:
                continue
            todo += [n for n in names if n == "pfbnav" or n.startswith("pfbnav.")]
    return sorted(files)


def environment_identity(cfg: dict[str, Any], root: Path = REPO_ROOT) -> dict[str, Any]:
    """{"sha256": the environment's code identity, "files": {relative path: text sha256}}."""
    paths = import_closure(root=root)
    paths += [root / p for p in LUA_SCRIPTS]
    paths.append(root / cfg["robot"]["footprint_table"])
    files = {p.relative_to(root).as_posix(): text_sha256(p) for p in sorted(set(paths))}
    digest = hashlib.sha256("".join(f"{k}\0{v}\n" for k, v in sorted(files.items()))
                            .encode("utf-8")).hexdigest()
    return {"sha256": digest, "files": files}


def freeze_problem(cfg: dict[str, Any], root: Path = REPO_ROOT) -> str | None:
    """Why this environment may not be used for a frozen-environment run, or None if it may.
    Every script that trains or evaluates under the freeze refuses on a non-None answer."""
    pinned = cfg["env"].get("frozen_code_sha256")
    if not pinned:
        return "the environment is not frozen (env.frozen_code_sha256 is not set)"
    found = environment_identity(cfg, root)["sha256"]
    if found != pinned:
        return (f"the environment is not the frozen one ({found[:12]} != {pinned[:12]}). "
                "Re-freezing needs the filter-off bit-identity test on the simulator and a "
                "decision-log entry; a filter-off behaviour change invalidates prior L1 runs.")
    return None
