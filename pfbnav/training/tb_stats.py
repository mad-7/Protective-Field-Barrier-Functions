"""Episode statistics for TensorBoard: what the learning curves of this study actually are.

SB3 logs losses and the mean episode return, but the quantities the study cares about (success, how episodes
end, protective stops, how often the PFB changed the command, the curriculum stage) live only in
`episodes.jsonl`. `EpisodeStats` keeps a rolling window of the last `window` finished episodes and writes their
statistics to the same logger, so they appear in TensorBoard next to the losses (x-axis: environment steps).

It only READS the episode records the recorder already collects (`pfbnav.training.record.RECORD_KEY`); it
changes nothing the policy sees, learns or is rewarded for.

Scalars (all over the window):
  episodes/success_rate                 share of episodes whose outcome is `success`
  episodes/outcome_share/<outcome>      share of each outcome seen (success, timeout, stall_truncated, ...)
  episodes/stops_per_100m_driven        protective stops per 100 m of distance driven
  episodes/stops_per_100m_planned       protective stops per 100 m of planned route (the study's primary form)
  episodes/mean_return, episodes/mean_length
  episodes/pfb_modified_share           share of steps on which the PFB changed the command (filtered arms)
  episodes/pfb_infeasible_share         share of steps on which the PFB found no feasible command
  episodes/window                       episodes in the window
  curriculum/stage                      index of the stage of the latest episode (see the config's curriculum)
"""
from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from typing import Any

from stable_baselines3.common.callbacks import BaseCallback

from pfbnav.training.record import RECORD_KEY


def window_stats(records: Sequence[dict[str, Any]], stages: Sequence[str] = ()) -> dict[str, float]:
    """The scalars above for a window of episode records. Empty window: {}. A quantity with no denominator
    (no distance driven, no steps) is left out, never reported as zero."""
    n = len(records)
    if n == 0:
        return {}
    out: dict[str, float] = {"episodes/window": float(n)}
    outcomes: dict[str, int] = {}
    for r in records:
        name = str(r.get("outcome", "unknown"))
        outcomes[name] = outcomes.get(name, 0) + 1
    out["episodes/success_rate"] = outcomes.get("success", 0) / n
    for name, count in sorted(outcomes.items()):
        out[f"episodes/outcome_share/{name}"] = count / n
    stops = sum(float(r.get("protective_stops", 0)) for r in records)
    driven = sum(float(r.get("distance_m", 0.0)) for r in records)
    planned = sum(float(r.get("plan_length_m", 0.0)) for r in records)
    if driven > 0.0:
        out["episodes/stops_per_100m_driven"] = 100.0 * stops / driven
    if planned > 0.0:
        out["episodes/stops_per_100m_planned"] = 100.0 * stops / planned
    out["episodes/mean_return"] = sum(float(r.get("episode_return", 0.0)) for r in records) / n
    steps = sum(float(r.get("episode_length", 0.0)) for r in records)
    out["episodes/mean_length"] = steps / n
    if steps > 0.0 and any("pfb_modified" in r for r in records):
        out["episodes/pfb_modified_share"] = sum(float(r.get("pfb_modified", 0)) for r in records) / steps
        out["episodes/pfb_infeasible_share"] = sum(float(r.get("pfb_infeasible", 0)) for r in records) / steps
    latest = records[-1].get("curriculum_stage")
    if isinstance(latest, str) and latest in stages:
        out["curriculum/stage"] = float(list(stages).index(latest))
    return out


class EpisodeStats(BaseCallback):
    """Writes `window_stats` of the last `window` finished episodes to the learner's logger each time an
    episode finishes; SB3 dumps them with its own scalars."""

    def __init__(self, window: int, stages: Sequence[str] = ()):
        super().__init__(verbose=0)
        if window < 1:
            raise ValueError("the statistics window must hold at least one episode")
        self.window = int(window)
        self.stages = list(stages)
        self.records: deque[dict[str, Any]] = deque(maxlen=self.window)

    def _on_step(self) -> bool:
        finished = False
        for info in self.locals.get("infos", []):
            record = info.get(RECORD_KEY)
            if record is not None:
                self.records.append(record)
                finished = True
        if finished:
            for name, value in window_stats(list(self.records), self.stages).items():
                self.logger.record(name, value)
        return True
