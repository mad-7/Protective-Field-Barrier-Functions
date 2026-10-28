"""Classical controllers see the typed input and nothing else (supervisor, 2026-10-16: enforced).

Four ways the boundary could leak, each closed by a test:
  1. the input carries a field it should not -- its fields are exactly the ruled four (+ dt);
  2. something reached THROUGH the input leads back to the environment -- every leaf is a plain
     float, bool or read-only array, and nothing can be attached to it;
  3. the runner hands a classical controller the `info` dict -- a probe records what it is given;
  4. controller code imports its way to the simulator, the environment or the pose -- the
     classical modules' imports are checked against an allowlist.
"""
import ast
from pathlib import Path

import numpy as np
import pytest
from synthetic_inputs import make_input

from pfbnav.envs.controller_input import (
    ALLOWED_FIELDS,
    ClassicalInput,
    RouteView,
    ScanStream,
    field_names,
)
from pfbnav.eval.ring import StepRing
from pfbnav.eval.runner import run_episodes

REPO = Path(__file__).resolve().parents[1]
PRIVILEGED = ("pose", "world", "time", "timer", "clock", "triggered_at", "clear_since",
              "contact", "certified", "layout", "scenario", "transform", "info", "state", "env")
# What a classical controller module may import from pfbnav: its own interfaces, and the
# certified layer's geometry and restart test (shared, not copied; ruled 2026-10-16), the typed
# input (a pure data module, checked below), and the config helpers.
ALLOWED_IMPORTS = {"pfbnav.envs.controller_input", "pfbnav.control.base",
                   "pfbnav.control.pure_pursuit",
                   "pfbnav.safety.certified", "pfbnav.safety.footprint_table",
                   "pfbnav.world.strata",
                   # The route profile (ruled 2026-10-20): pure maths on an offline-computed array,
                   # handed in per episode; the controller reads only its own arc length from the
                   # typed input.
                   "pfbnav.planning.route_profile",
                   # The shared invalid-bin convention (ruled 2026-10-27): one pure function on the
                   # scan stream and the controller's own footprint; it imports nothing from pfbnav.
                   "pfbnav.safety.unobserved"}
CLASSICAL_MODULES = ("pure_pursuit.py", "recovery.py", "base.py", "route_cap.py")


def test_the_input_carries_exactly_the_ruled_fields():
    assert field_names(ClassicalInput) == ("stop_active", "velocity", "scan", "route", "dt")
    for kind, names in ALLOWED_FIELDS.items():
        assert field_names(kind) == names
        assert not any(p in n for n in names for p in PRIVILEGED), names


def _leaves(obj):
    if isinstance(obj, (ClassicalInput, ScanStream, RouteView)):
        for name in field_names(type(obj)):
            yield from _leaves(getattr(obj, name))
    else:
        yield obj


def test_nothing_reached_through_the_input_leads_anywhere_else():
    inp = make_input(stop=True, velocity=(0.1, 0.0, 0.2))
    for leaf in _leaves(inp):
        assert isinstance(leaf, (bool, float, np.ndarray)), type(leaf)
        if isinstance(leaf, np.ndarray):
            assert not leaf.flags.writeable
            assert leaf.dtype.kind in "fb"
    # Frozen and slotted: nothing can be attached. (Python 3.10 raises TypeError from the slotted
    # frozen __setattr__, 3.11+ FrozenInstanceError, an AttributeError; either way it is refused.)
    with pytest.raises((AttributeError, TypeError)):
        inp.pose = (0.0, 0.0, 0.0)
    assert not hasattr(inp, "pose") and not hasattr(inp, "__dict__")
    with pytest.raises(ValueError):
        inp.velocity[0] = 9.0                          # read-only


def test_the_input_copies_rather_than_shares_the_environments_arrays():
    ranges = np.full(120, 2.0)
    scan = ScanStream.of(ranges, np.linspace(-3.1, 3.1, 120), np.ones(120, bool))
    ranges[0] = 0.0
    assert scan.ranges[0] == 2.0


class _Env:
    """An environment whose info carries privileged keys beside the controller input."""

    def __init__(self, steps=3):
        self.steps, self.t = steps, 0

    def _info(self):
        return {"controller_input": make_input(), "pose": (1.0, 2.0, 0.3),
                "certified": {"stop_active": False, "_clear_since": 4.0}}

    def reset(self, *, options=None):
        self.t = 0
        return np.zeros(82), self._info()

    def step(self, action):
        self.t += 1
        done = self.t >= self.steps
        info = self._info()
        if done:
            info["episode"] = {"outcome": "timeout"}
        return np.zeros(82), 0.0, False, done, info


class _Probe:
    def __init__(self, kind):
        self.name, self.input_kind, self.seen = "probe", kind, []

    def reset(self):
        pass

    def act(self, given):
        self.seen.append(type(given))
        return np.zeros(3)


def test_the_runner_hands_a_classical_controller_only_the_typed_input(tmp_path):
    probe = _Probe("classical")
    run_episodes(_Env(), probe, [0], ring=StepRing(4, tmp_path))
    assert probe.seen and set(probe.seen) == {ClassicalInput}


def test_the_runner_hands_a_learned_policy_only_the_observation(tmp_path):
    probe = _Probe("observation")
    run_episodes(_Env(), probe, [0], ring=StepRing(4, tmp_path))
    assert set(probe.seen) == {np.ndarray}


def test_a_classical_controller_without_its_input_is_refused(tmp_path):
    class Bare(_Env):
        def _info(self):
            return {"pose": (0.0, 0.0, 0.0)}

    with pytest.raises(TypeError, match="controller_input"):
        run_episodes(Bare(), _Probe("classical"), [0], ring=StepRing(4, tmp_path))


def test_classical_controller_code_imports_nothing_privileged():
    """No path from a classical controller to the simulator, the environment, the planner's world
    pose or the evaluation code: its pfbnav imports are on the allowlist."""
    for name in CLASSICAL_MODULES:
        tree = ast.parse((REPO / "pfbnav" / "control" / name).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module and \
                    node.module.startswith("pfbnav"):
                assert node.module in ALLOWED_IMPORTS, (name, node.module)
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name.startswith("pfbnav") or alias.name in ALLOWED_IMPORTS, \
                        (name, alias.name)


def test_the_input_module_is_pure_data_and_imports_nothing_from_pfbnav():
    """The one environment module a classical controller may import: it must lead nowhere."""
    tree = ast.parse((REPO / "pfbnav" / "envs" / "controller_input.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            assert not (node.module or "").startswith("pfbnav"), node.module
        elif isinstance(node, ast.Import):
            assert not any(a.name.startswith("pfbnav") for a in node.names)
