"""What a training run keeps about its episodes, and how.

**The `info["episode"]` clash.** `NavEnv` puts its own account of a finished episode under
`info["episode"]` (scenario, outcome, protective stops, distance, timings). Stable-Baselines3
reads the same key and expects `{"r", "l", "t"}`: SAC's logger would fail on the environment's
record, and SB3's `Monitor` wrapper would silently overwrite it. `EpisodeRecorder` resolves this
in one place: the environment's record moves to `info["episode_record"]`, with the episode's
return and length added, and `info["episode"]` becomes what SB3 expects.

**`EpisodeLog`** appends one JSON line per finished episode to the run directory, as it
happens, so a run that dies keeps every episode it finished.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

RECORD_KEY = "episode_record"


class EpisodeRecorder(gym.Wrapper):
    """Moves the environment's episode record out of SB3's way and adds return and length."""

    def __init__(self, env: gym.Env):
        super().__init__(env)
        self._return = 0.0
        self._length = 0
        self._started = time.perf_counter()

    def reset(self, **kwargs):
        self._return, self._length = 0.0, 0
        self._started = time.perf_counter()
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._return += float(reward)
        self._length += 1
        if terminated or truncated:
            info = dict(info)
            record = dict(info.pop("episode", {}) or {})
            record.update(episode_return=self._return, episode_length=self._length,
                          terminated=bool(terminated), truncated=bool(truncated))
            info[RECORD_KEY] = record
            info["episode"] = {"r": self._return, "l": self._length,
                               "t": round(time.perf_counter() - self._started, 6)}
        return obs, reward, terminated, truncated, info


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


class EpisodeLog(BaseCallback):
    """Appends each finished episode's record, with the step at which it ended, to a JSONL file."""

    def __init__(self, path: str | Path):
        super().__init__(verbose=0)
        self.path = Path(path)
        if self.path.exists():
            raise FileExistsError(f"{self.path} exists; a run never appends to another's log")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.episodes = 0
        self.outcomes: dict[str, int] = {}
        # Stops triggered at rest (ruled 2026-10-19): expected zero in a static world.
        self.at_rest_triggers = 0
        self.stationary_triggers = 0            # of which stationary: the construction gap

    def _on_step(self) -> bool:
        for info in self.locals.get("infos", []):
            record = info.get(RECORD_KEY)
            if record is None:
                continue
            self.episodes += 1
            outcome = str(record.get("outcome", "unknown"))
            self.outcomes[outcome] = self.outcomes.get(outcome, 0) + 1
            self.at_rest_triggers += int(record.get("at_rest_triggers", 0))
            self.stationary_triggers += int(record.get("stationary_triggers", 0))
            line = {"episode": self.episodes, "timestep": int(self.num_timesteps),
                    **_jsonable(record)}
            with self.path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(line) + "\n")
        return True


class MemoryTrend(BaseCallback):
    """Samples the training process's and the simulator's private memory every `every` steps into
    a JSONL file (ruled 2026-10-30: the in-run trend, read by `pfbnav.utils.memory.levels_off`).
    `memory` is the run's `MemoryRecord`; the first sample is taken at the first step."""

    def __init__(self, path: str | Path, memory: Any, every: int):
        super().__init__(verbose=0)
        if every < 1:
            raise ValueError("the memory sampling interval must be at least one step")
        self.path = Path(path)
        if self.path.exists():
            raise FileExistsError(f"{self.path} exists; a run never appends to another's log")
        self.memory = memory
        self.every = int(every)
        self.samples: list[dict[str, Any]] = []

    def sample(self) -> dict[str, Any]:
        snap = self.memory.snapshot()
        row = {"timestep": int(self.num_timesteps), "python": snap.get("python"),
               "sim": snap.get("sim"), "wall_s": time.perf_counter()}
        self.samples.append(row)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")
        return row

    def _on_step(self) -> bool:
        if self.num_timesteps == 1 or self.num_timesteps % self.every == 0:
            self.sample()
        return True

    def _on_training_end(self) -> None:
        if not self.samples or self.samples[-1]["timestep"] != self.num_timesteps:
            self.sample()


#: A file of this name in the run directory ends training cleanly at the next check.
STOP_FILE = "STOP"


class StopFile(BaseCallback):
    """Ends training when `<run_dir>/STOP` exists, checked every `every` steps: `learn()` returns
    normally, so the caller saves the model (`stopped_model.zip`, never `final_model.zip`: a run
    stopped early has not reached `total_timesteps` and fails G4's integrity check by design)."""

    def __init__(self, run_dir: str | Path, every: int = 100):
        super().__init__(verbose=0)
        if every < 1:
            raise ValueError("the stop-file interval must be at least one step")
        self.path = Path(run_dir) / STOP_FILE
        if self.path.exists():
            raise FileExistsError(f"{self.path} exists before training started; remove it")
        self.every = int(every)
        self.stopped_at: int | None = None

    def _on_step(self) -> bool:
        if self.num_timesteps % self.every == 0 and self.path.exists():
            self.stopped_at = int(self.num_timesteps)
            return False
        return True
