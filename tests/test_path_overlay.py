"""The route overlay, against a recording stand-in for the simulator (hard rule 6)."""
from types import SimpleNamespace

import numpy as np

from pfbnav.sim.path_overlay import MIN_STEP_M, PathOverlay, segments


class FakeSim:
    drawing_lines, drawing_spherepoints = 1, 2

    def __init__(self):
        self.created, self.items, self.removed = [], [], []
        self.signals, self.unknown = {}, set()

    def setStringSignal(self, name, value):
        self.signals[name] = value

    def getStringSignal(self, name):
        return self.signals.get(name)

    def addDrawingObject(self, kind, size, tol, parent, count, colour):
        self.created.append((kind, colour))
        return len(self.created)

    def addDrawingObjectItem(self, handle, item):
        self.items.append((handle, list(item)))

    def removeDrawingObject(self, handle):
        if handle in self.unknown:
            raise RuntimeError("unknown handle")
        self.removed.append(handle)


class FakeEnv:
    def __init__(self, poses):
        self.plan = SimpleNamespace(path_world=np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 2.0]]))
        self.transform = SimpleNamespace(apply=lambda p: np.asarray(p) + np.array([10.0, 5.0]))
        self._poses = iter(poses)

    def _robot_world_pose(self):
        return next(self._poses)


def test_segments_known_answer():
    s = segments(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 2.0]]), z=0.1)
    assert s == [[0.0, 0.0, 0.1, 1.0, 0.0, 0.1], [1.0, 0.0, 0.1, 1.0, 2.0, 0.1]]
    assert segments(np.array([[0.0, 0.0]])) == []


def test_begin_draws_the_route_in_world_coordinates_with_start_and_goal():
    sim = FakeSim()
    PathOverlay(sim).begin(FakeEnv([]), {})
    plan_handle = 1
    route = [item for h, item in sim.items if h == plan_handle]
    assert route[0][:2] == [10.0, 5.0] and route[-1][3:5] == [11.0, 7.0]    # shifted by the transform
    dots = [item for h, item in sim.items if h in (2, 3)]
    assert dots[0][:2] == [10.0, 5.0] and dots[1][:2] == [11.0, 7.0]
    assert len(sim.created) == 4                                           # route, start, goal, track


def test_the_track_grows_only_when_the_robot_moves():
    sim = FakeSim()
    overlay = PathOverlay(sim)
    env = FakeEnv([(10.0, 5.0, 0.0), (10.0 + MIN_STEP_M / 2, 5.0, 0.0), (10.5, 5.0, 0.0)])
    overlay.begin(env, {})
    n = len(sim.items)
    for _ in range(3):
        overlay.step(env, {})
    track = [item for h, item in sim.items[n:] if h == 4]
    assert len(track) == 1 and track[0][3:5] == [10.5, 5.0]               # the two small moves added nothing


def test_end_removes_every_object_and_a_disabled_overlay_does_nothing():
    sim = FakeSim()
    overlay = PathOverlay(sim)
    overlay.begin(FakeEnv([]), {})
    overlay.end()
    assert sorted(sim.removed) == [1, 2, 3, 4]
    off, quiet = PathOverlay(sim, enabled=False), FakeSim()
    off.sim = quiet
    off.begin(FakeEnv([]), {})
    off.step(FakeEnv([(0.0, 0.0, 0.0)]), {})
    assert quiet.created == [] and quiet.items == []


def test_a_later_run_removes_what_an_interrupted_one_left_and_counts_what_it_cannot():
    sim = FakeSim()
    first = PathOverlay(sim)
    first.begin(FakeEnv([]), {})                  # ... and the process dies without end()
    leftovers = sorted(h for h in range(1, 5))
    second_sim_state = sim.signals.copy()
    second = PathOverlay(sim)
    sim.unknown = {2}                             # one of them is already gone
    sim.removed.clear()
    sim.signals = second_sim_state
    second.begin(FakeEnv([]), {})
    assert sorted(h for h in sim.removed if h in leftovers) == [1, 3, 4]
    assert second.stale_removed == 3 and second.stale_failed == 1


def test_the_signal_is_cleared_when_an_episode_ends():
    sim = FakeSim()
    overlay = PathOverlay(sim)
    overlay.begin(FakeEnv([]), {})
    assert sim.signals["pfb_path_overlay"] != "[]"
    overlay.end()
    assert sim.signals["pfb_path_overlay"] == "[]"
