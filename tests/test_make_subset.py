"""scripts/make_subset.py: known answers and its refusals."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("make_subset", ROOT / "scripts" / "make_subset.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_subset_keeps_the_chosen_scenarios_in_order():
    manifest = {"scenarios": [{"seed": s} for s in (10, 11, 12, 13)], "built": 4, "requested": 4,
                "failures": [{"x": 1}], "locked": True}
    out = mod.subset(manifest, [3, 1])
    assert [s["seed"] for s in out["scenarios"]] == [13, 11]
    assert out["built"] == out["requested"] == 2 and out["failures"] == [] and out["locked"] is False
    assert len(manifest["scenarios"]) == 4                      # the source is untouched


def test_a_bad_index_is_refused():
    with pytest.raises(ValueError, match="outside"):
        mod.subset({"scenarios": [{"seed": 1}], "built": 1, "requested": 1}, [1])


def test_mapped_only_keeps_the_scenarios_without_unmapped_obstacles():
    manifest = {"scenarios": [{"unmapped_count": 0}, {"unmapped_count": 2}, {"unmapped_count": 0}],
                "built": 3, "requested": 3}
    assert mod.mapped_only_indices(manifest) == [0, 2]
