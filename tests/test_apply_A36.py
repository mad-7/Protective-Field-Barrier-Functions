"""docs/pending/apply_A36.py on a COPY of preregistration/ (never the real one; hard rule 1)."""
import importlib.util
import shutil
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def _module():
    spec = importlib.util.spec_from_file_location("apply_A36", ROOT / "docs" / "pending" / "apply_A36.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_apply_on_a_copy(tmp_path, monkeypatch):
    if "## A36 - " in (ROOT / "preregistration" / "AMENDMENTS.md").read_text(encoding="utf-8"):
        pytest.skip("A36 is already applied")
    prereg = tmp_path / "preregistration"
    shutil.copytree(ROOT / "preregistration", prereg)
    m = _module()
    monkeypatch.setattr(m, "PREREG", prereg)
    assert m.main() == 0
    e = yaml.safe_load((prereg / "endpoints.yaml").read_text(encoding="utf-8"))
    s = yaml.safe_load((prereg / "scenario_sets.yaml").read_text(encoding="utf-8"))
    assert e["filter_efficacy_family"] == {"members": ["HFc"], "alpha": 0.05}
    assert e["HFc"]["comparison"] == "B7 vs B2"
    assert e["HFc"]["endpoint"] == "protective_stops_per_100m_planned"
    assert s["scenario_sets"]["test"] == {"per_cell": 50}
    assert e["H1"] == yaml.safe_load((ROOT / "preregistration" / "endpoints.yaml")
                                     .read_text(encoding="utf-8"))["H1"]      # H1 untouched
    text = (prereg / "AMENDMENTS.md").read_text(encoding="utf-8")
    assert text.count("## A14b - ") == 1 and text.count("## A36 - ") == 1
    with pytest.raises(SystemExit):        # a second application writes nothing
        m.main()
