"""The training overlay wrapper: call order, pass-through, and the flag on train.py (hard rule 6)."""
import subprocess
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np

from pfbnav.training.overlay import PathOverlayWrapper

ROOT = Path(__file__).resolve().parents[1]


class Tiny(gym.Env):
    """Three-step episodes; the third step ends them (terminated), or truncates when asked."""

    observation_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
    action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def __init__(self, truncate=False):
        self.n, self.truncate, self.closed = 0, truncate, False

    def reset(self, *, seed=None, options=None):
        self.n = 0
        return np.zeros(2, np.float32), {"reset": True}

    def step(self, action):
        self.n += 1
        done = self.n >= 3
        return np.full(2, self.n, np.float32), 0.5, done and not self.truncate, done and self.truncate, {"n": self.n}

    def close(self):
        self.closed = True


class Recorder:
    def __init__(self):
        self.calls = []

    def begin(self, base, info):
        self.calls.append(("begin", base, dict(info)))

    def step(self, base, info):
        self.calls.append(("step", base, dict(info)))

    def end(self):
        self.calls.append(("end",))


def test_the_overlay_is_told_of_the_reset_each_step_and_the_end_and_the_close():
    base, rec = object(), Recorder()
    env = PathOverlayWrapper(Tiny(), base, rec)
    env.reset()
    for _ in range(3):
        env.step(np.zeros(1, np.float32))
    env.close()
    names = [c[0] for c in rec.calls]
    assert names == ["begin", "step", "step", "step", "end", "end"]       # end on done, and again on close
    assert all(c[1] is base for c in rec.calls if c[0] in ("begin", "step"))


def test_a_truncated_episode_also_ends_the_overlay():
    rec = Recorder()
    env = PathOverlayWrapper(Tiny(truncate=True), object(), rec)
    env.reset()
    for _ in range(3):
        env.step(np.zeros(1, np.float32))
    assert [c[0] for c in rec.calls] == ["begin", "step", "step", "step", "end"]


def test_what_the_learner_sees_is_untouched():
    plain, wrapped = Tiny(), PathOverlayWrapper(Tiny(), object(), Recorder())
    o1, i1 = plain.reset()
    o2, i2 = wrapped.reset()
    assert np.array_equal(o1, o2) and i1 == i2
    for _ in range(3):
        a = plain.step(np.zeros(1, np.float32))
        b = wrapped.step(np.zeros(1, np.float32))
        assert np.array_equal(a[0], b[0]) and a[1:] == b[1:]


def test_train_offers_the_draw_path_flag_and_refuses_it_without_rendering():
    out = subprocess.run([sys.executable, str(ROOT / "scripts" / "train.py"), "--help"],
                         capture_output=True, text=True, check=True, cwd=ROOT).stdout
    assert "--draw-path" in out
    refused = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "train.py"), "--config", "configs/experiments/L1_sac_shaped.yaml",
         "--seed", "9000099", "--draw-path", "--display", "off"],
        capture_output=True, text=True, cwd=ROOT, check=False)
    assert refused.returncode != 0 and "--draw-path needs rendering" in (refused.stderr + refused.stdout)
