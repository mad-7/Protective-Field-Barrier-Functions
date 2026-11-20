"""The environment's code identity, recorded in a run's provenance.

Nothing refuses on it: it only lets a result be traced to the code that produced it. "The
environment" is defined mechanically, so nobody has to remember to add a file to a list:

  * every `pfbnav` module in the **import closure of `pfbnav.envs.nav_env`** -- every import,
    including those inside functions -- and
  * the **data files it reads**: the scanner's Lua script and the footprint table the config
    names.

File hashes are of text with line endings normalised (`pfbnav/utils/hashing.py`), so the identity
agrees between machines.
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
