"""The training curriculum (ruled 2026-10-30, A30). Training only; identical for every learned arm.

    0-20 % of training     wide and medium strata only, unmapped rate 0
    20-40 %                all strata, unmapped rate 0
    40-100 %               the full training distribution (all strata, unmapped rate 0.25)

Each seed's permutation applies within each stage: a stage walks the seed's order of the training
set (`pfbnav/training/order.py`), restricted to the scenarios it admits, from its start, cycling if
it runs out. The test distribution and every evaluation are unchanged.

**"Unmapped rate 0"** is implemented by admitting only the scenarios built with no unmapped obstacle
(`unmapped_count == 0`). The training sets were built at rate 0.25 with the obstacle drawn
independently of the layout, so the admitted scenarios are the same layout distribution, and no
scenario is altered: each still passes `verify()` and keeps its identity. "Rate 0.25" is the set as
built (achieved 0.244-0.247 per stratum).

**Stages switch between episodes, never within one.** At each reset the stage is the one whose
`from_fraction` x `total_timesteps` the run has reached, counted in environment steps (one
environment: steps = SB3 timesteps). The chosen scenario is passed to `NavEnv.reset` as
`options={"index": i}`, and the episode's record carries `curriculum_stage`, so G4 can read its
learning criterion on full-distribution episodes only (A30).

The wrapper sits outside the frozen environment's closure (`env.frozen_code_sha256` unchanged).
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Any

import gymnasium as gym

#: The stage whose episodes are the full training distribution: G4's learning criterion reads these.
FULL = "full"


@dataclass(frozen=True)
class Stage:
    name: str
    from_fraction: float
    strata: tuple[str, ...]
    unmapped: bool

    def admits(self, scenario: dict[str, Any]) -> bool:
        if scenario["stratum"] not in self.strata:
            return False
        return self.unmapped or int(scenario["unmapped_count"]) == 0


def stages_from_config(cfg: dict[str, Any]) -> tuple[Stage, ...]:
    """`training.curriculum`, validated: the first stage starts at 0, fractions strictly increase
    within [0, 1), names are unique, and the last stage is the full distribution."""
    stages = tuple(Stage(name=str(s["name"]), from_fraction=float(s["from_fraction"]),
                         strata=tuple(str(x) for x in s["strata"]), unmapped=bool(s["unmapped"]))
                   for s in cfg["training"]["curriculum"])
    if not stages or stages[0].from_fraction != 0.0:
        raise ValueError("the curriculum's first stage starts at fraction 0")
    fractions = [s.from_fraction for s in stages]
    if any(b <= a for a, b in itertools.pairwise(fractions)) or fractions[-1] >= 1.0:
        raise ValueError("curriculum fractions must strictly increase and stay below 1")
    if len({s.name for s in stages}) != len(stages):
        raise ValueError("curriculum stage names must be unique")
    if stages[-1].name != FULL or not stages[-1].unmapped:
        raise ValueError(f"the last curriculum stage is `{FULL}`, the full distribution")
    return stages


def stage_at(stages: tuple[Stage, ...], steps: int, total_timesteps: int) -> Stage:
    """The stage in force after `steps` environment steps of a `total_timesteps` run."""
    current = stages[0]
    for stage in stages:
        if steps >= stage.from_fraction * total_timesteps:
            current = stage
    return current


class Curriculum(gym.Wrapper):
    """Chooses each training episode's scenario by stage. `scenarios` is the training set in the
    seed's order -- the same list, in the same order, `NavEnv` was built with -- so an index into it
    is an index `NavEnv.reset(options={"index": i})` understands."""

    def __init__(self, env: gym.Env, stages: tuple[Stage, ...], scenarios: list[dict[str, Any]],
                 total_timesteps: int):
        super().__init__(env)
        if total_timesteps < 1:
            raise ValueError("total_timesteps must be positive")
        self.stages = stages
        self.total_timesteps = int(total_timesteps)
        self.pools = {s.name: [i for i, sc in enumerate(scenarios) if s.admits(sc)] for s in stages}
        empty = [name for name, pool in self.pools.items() if not pool]
        if empty:
            raise ValueError(f"curriculum stage(s) {empty} admit no training scenario")
        self.cursor = {s.name: 0 for s in stages}
        self.steps = 0
        self.stage = stages[0]
        self.episodes_per_stage = {s.name: 0 for s in stages}

    def reset(self, **kwargs):
        self.stage = stage_at(self.stages, self.steps, self.total_timesteps)
        pool = self.pools[self.stage.name]
        index = pool[self.cursor[self.stage.name] % len(pool)]
        self.cursor[self.stage.name] += 1
        self.episodes_per_stage[self.stage.name] += 1
        options = dict(kwargs.pop("options", None) or {})
        options["index"] = index
        return self.env.reset(options=options, **kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.steps += 1
        if (terminated or truncated) and "episode" in info:
            info = dict(info)
            info["episode"] = {**info["episode"], "curriculum_stage": self.stage.name}
        return obs, reward, terminated, truncated, info

    def report(self) -> dict[str, Any]:
        return {"stages": [{"name": s.name, "from_fraction": s.from_fraction,
                            "strata": list(s.strata), "unmapped": s.unmapped,
                            "pool": len(self.pools[s.name]),
                            "episodes": self.episodes_per_stage[s.name]} for s in self.stages]}
