"""Known-answer tests for the layout read-back's geometry (hard rule 6)."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "probe_scanner_layout.py"


def _load():
    spec = importlib.util.spec_from_file_location("probe_scanner_layout", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


M = _load()


def test_identity_quaternion_leaves_vectors_alone():
    v = np.array([[1.0, 2.0, 3.0]])
    assert M.quat_rotate([0, 0, 0, 1], v) == pytest.approx(v)


def test_a_quarter_turn_about_z():
    q = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    assert M.quat_rotate(q, np.array([[1.0, 0.0, 0.0]]))[0] == pytest.approx([0.0, 1.0, 0.0])


def test_box_corners_span_the_box():
    c = M.box_corners([0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0, 0, 0, 1])
    assert c.shape == (8, 3)
    assert c.min(axis=0) == pytest.approx([0.9, -0.2, -0.3])
    assert c.max(axis=0) == pytest.approx([1.1, 0.2, 0.3])


def test_ref_to_body_undoes_the_quarter_turn():
    # youBot_ref's +y is chassis-forward: a point there is at body +x.
    got = M.ref_to_body(np.array([[0.0, 1.0, 0.5]]), math.pi / 2)[0]
    assert got == pytest.approx([1.0, 0.0, 0.5])


def test_protrusion_is_zero_within_the_chassis_and_positive_beyond():
    assert M.protrusion((0.28, 0.17), (0.2854, 0.1789)) == {"beyond_x_m": 0.0,
                                                            "beyond_y_m": 0.0}
    p = M.protrusion((0.31, 0.19), (0.2854, 0.1789))
    assert p["beyond_x_m"] == pytest.approx(0.0246)
    assert p["beyond_y_m"] == pytest.approx(0.0111)
