"""Structural rules: the PFB filter and the certified emulator stay independent."""
import ast
from pathlib import Path

SAFETY = Path(__file__).resolve().parents[1] / "pfbnav" / "safety"


def _imports(path: Path) -> set[str]:
    names = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            names |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
            names |= {f"{node.module}.{a.name}" for a in node.names}
    return names


def test_certified_does_not_import_pfb():
    assert not any("pfb" in n for n in _imports(SAFETY / "certified.py"))


def test_pfb_does_not_import_certified_or_sim():
    bad = ("certified", "coppeliasim", "pfbnav.sim", "pfbnav.envs")
    assert not any(b in n for n in _imports(SAFETY / "pfb.py") for b in bad)


def test_the_pfb_field_is_independent_of_the_certified_layer():
    """WP2 (2026-10-23): the PFB's field module imports neither the certified layer nor anything
    that does (`pfbnav.world.strata` imports it), nor the simulator or the environment."""
    bad = ("certified", "pfbnav.world.strata", "coppeliasim", "pfbnav.sim", "pfbnav.envs")
    assert not any(b in n for n in _imports(SAFETY / "pfb_field.py") for b in bad)
