"""The environment freeze L1 trains under (ruled 2026-10-03). Known answers on a synthetic tree,
and the pin: the environment in the repository must be the one `configs/base.yaml` froze."""
from pathlib import Path

import pytest

from pfbnav.envs.freeze import environment_identity, import_closure

REPO = Path(__file__).resolve().parents[1]
CFG = {"robot": {"footprint_table": "configs/footprint/fp.csv"}}


def _tree(root: Path) -> None:
    (root / "pfbnav" / "envs").mkdir(parents=True)
    (root / "pfbnav" / "__init__.py").write_text("")
    (root / "pfbnav" / "envs" / "__init__.py").write_text("")
    (root / "pfbnav" / "envs" / "nav_env.py").write_text(
        "from pfbnav.a import thing\n\ndef f():\n    from pfbnav import c\n")
    (root / "pfbnav" / "a.py").write_text("thing = 1\n")
    (root / "pfbnav" / "b.py").write_text("other = 2\n")
    (root / "pfbnav" / "c.py").write_text("late = 3\n")
    (root / "lua").mkdir()
    (root / "lua" / "scanner_v2.lua").write_text("-- scanner\n")
    (root / "configs" / "footprint").mkdir(parents=True)
    (root / "configs" / "footprint" / "fp.csv").write_text("psi,e\n0,0.3\n")


def test_the_closure_follows_every_import_including_those_inside_functions(tmp_path):
    _tree(tmp_path)
    names = {p.relative_to(tmp_path).as_posix() for p in import_closure(root=tmp_path)}
    assert {"pfbnav/envs/nav_env.py", "pfbnav/a.py", "pfbnav/c.py"} <= names
    assert "pfbnav/b.py" not in names


def test_the_identity_moves_with_the_environment_and_only_with_it(tmp_path):
    _tree(tmp_path)
    first = environment_identity(CFG, root=tmp_path)["sha256"]
    (tmp_path / "pfbnav" / "b.py").write_text("other = 99\n")          # outside the closure
    assert environment_identity(CFG, root=tmp_path)["sha256"] == first
    (tmp_path / "pfbnav" / "a.py").write_text("thing = 2\n")           # inside it
    changed = environment_identity(CFG, root=tmp_path)["sha256"]
    assert changed != first
    (tmp_path / "pfbnav" / "a.py").write_text("thing = 2\nimport pfbnav.b\n")   # b joins
    assert environment_identity(CFG, root=tmp_path)["sha256"] != changed
    (tmp_path / "configs" / "footprint" / "fp.csv").write_text("psi,e\n0,0.31\n")  # data too
    assert environment_identity(CFG, root=tmp_path)["sha256"] != changed


def test_line_endings_do_not_move_the_identity(tmp_path):
    _tree(tmp_path)
    path = tmp_path / "pfbnav" / "a.py"
    path.write_bytes(b"thing = 1\nmore = 2\n")          # explicit LF: write_text is CRLF on Windows
    first = environment_identity(CFG, root=tmp_path)["sha256"]
    path.write_bytes(b"thing = 1\r\nmore = 2\r\n")
    assert environment_identity(CFG, root=tmp_path)["sha256"] == first


def test_the_real_environment_includes_the_pfb_and_excludes_the_learner():
    """Since WP2 step 4 (re-freeze 10, 2026-10-24) the PFB is in the step loop's slot, so it is
    part of the environment's identity, as `freeze.py` anticipated; the learner and the
    controllers never are."""
    names = {p.relative_to(REPO).as_posix() for p in import_closure()}
    assert {"pfbnav/envs/step_loop.py", "pfbnav/safety/certified.py",
            "pfbnav/world/scenarios.py", "pfbnav/safety/pfb.py",
            "pfbnav/safety/pfb_field.py"} <= names
    assert not any(n.startswith(("pfbnav/training/", "pfbnav/control/")) for n in names)


def test_the_environment_is_the_frozen_one():
    """L1's runs are valid only on the pinned environment. If this fails, the environment changed:
    re-freezing needs the filter-off bit-identity test to pass on the simulator and a decision-log
    entry, and a filter-off behaviour change invalidates prior L1 runs (ruled 2026-10-03)."""
    from pfbnav.config import load_config
    cfg = load_config(REPO / "configs" / "base.yaml")
    pinned = cfg["env"]["frozen_code_sha256"]
    if pinned is None:
        pytest.fail("the environment is not frozen (env.frozen_code_sha256 is null)")
    assert environment_identity(cfg)["sha256"] == pinned, "the environment changed since its freeze"
