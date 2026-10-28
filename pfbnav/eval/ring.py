"""A per-step ring buffer for evaluations, dumped automatically on the events that need a trace
(ruled 2026-10-23). Passive: it reads each step's `info`, never the environment, and changes
nothing a controller or the environment does.

Why: a stationary trigger in the zero-action control (2026-10-22, seed 2000018) could not be
attributed, because evaluations kept only per-episode summaries. Now every evaluation keeps the last
`capacity` steps, and writes them out when

  * a **stationary trigger** fires (the construction gap, ruled 2026-10-19): the steps up to it;
  * a **scan fault** replaces a packet with the fail-safe (ruled 2026-10-23): the steps up to it;
  * the robot is **in contact**, at the first contacted step: the steps up to it;
  * the episode ends as an **unrecoverable stop**: the steps before the LAST stop trigger (kept
    when it fired) and the last steps of the episode.

Each row: the step, the wall-clock step time and the over-budget flag (to see load), the certified
layer's full report, the requested and applied commands, route progress, contact, and the scan --
every range to the millimetre, its validity, and the nearest valid return with its bearing.

**On in every evaluation and acceptance run** (ruled 2026-10-26, after the 2026-10-25 simulator
acceptance ran without it -- a lapse of the 2026-10-23 ruling). `run_episodes` refuses to start
without a ring, and a ring writes `ring.json` (its capacity) into its directory when it is created, so
a run directory shows that the ring was in force even when nothing triggered a dump.
"""
from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np


def step_row(step: int, info: dict[str, Any]) -> dict[str, Any]:
    """One step's record from the environment's `info` (no privileged access beyond it)."""
    row: dict[str, Any] = {
        "step": int(step),
        "step_seconds": info.get("step_seconds"), "over_budget": info.get("over_budget"),
        "certified": dict(info.get("certified") or {}),
        "requested": info.get("requested"), "applied": info.get("applied"),
        "s_m": info.get("s_m"), "cross_track_m": info.get("cross_track_m"),
        "goal_distance_m": info.get("goal_distance_m"),
        "in_contact": info.get("in_contact"), "contact_names": info.get("contact_names"),
        "scan_fault": info.get("scan_fault"), "decided_on_fault": info.get("decided_on_fault"),
        # The PFB's decision this step (WP2; None with the filter off): mode, regime, command,
        # field heading and swept yaw rate, nominal.
        "pfb": info.get("pfb"),
    }
    inp = info.get("controller_input")
    scan = getattr(inp, "scan", None)
    if scan is not None:
        r = np.asarray(scan.ranges, dtype=float)
        ok = np.asarray(scan.valid, dtype=bool)
        b = np.asarray(scan.bearings, dtype=float)
        row["ranges_mm"] = [round(1000.0 * float(x)) if math.isfinite(x) else None for x in r]
        row["invalid_bins"] = [int(i) for i in np.flatnonzero(~ok)]
        if ok.any():
            i = int(np.flatnonzero(ok)[np.argmin(r[ok])])
            row["nearest"] = {"range_m": float(r[i]), "bearing_rad": float(b[i]), "bin": i}
    return row


class StepRing:
    """The last `capacity` steps of the current episode, and the dumps it has written."""

    MARKER = "ring.json"

    def __init__(self, capacity: int, out_dir: Path):
        if capacity < 1:
            raise ValueError("capacity must be at least 1")
        if out_dir is None:
            raise ValueError("a ring needs a directory: its dumps and its marker go there")
        self.capacity = int(capacity)
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        (self.out_dir / self.MARKER).write_text(json.dumps({"capacity": self.capacity}),
                                                encoding="utf-8")
        self._rows: deque[dict[str, Any]] = deque(maxlen=self.capacity)
        self._before_last_stop: list[dict[str, Any]] = []
        self._contact_dumped = False
        self.dumps: list[str] = []
        self._episode = -1

    def begin(self, episode_index: int) -> None:
        self._rows.clear()
        self._before_last_stop = []
        self._contact_dumped = False
        self.dumps = []
        self._episode = int(episode_index)

    def record(self, step: int, info: dict[str, Any]) -> None:
        row = step_row(step, info)
        self._rows.append(row)
        cert = row["certified"]
        if cert.get("stop_triggered"):
            self._before_last_stop = list(self._rows)
        if cert.get("triggered_stationary") and not row.get("decided_on_fault"):
            self._dump(f"stationary_step{step}", list(self._rows))
        if row.get("scan_fault"):
            self._dump(f"scan_fault_step{step}", list(self._rows))
        if row.get("in_contact") and not self._contact_dumped:
            self._contact_dumped = True
            self._dump(f"contact_step{step}", list(self._rows))

    def end(self, outcome: str) -> list[str]:
        """Called once the episode has ended; returns every dump written for it."""
        if outcome == "unrecoverable_stop":
            self._dump("unrecoverable_before_last_stop", self._before_last_stop)
            self._dump("unrecoverable_end", list(self._rows))
        return list(self.dumps)

    def _dump(self, reason: str, rows: list[dict[str, Any]]) -> None:
        name = f"ring_{self._episode}_{reason}.jsonl"
        if self.out_dir is not None:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            with (self.out_dir / name).open("w", encoding="utf-8") as f:
                for r in rows:
                    f.write(json.dumps(r, default=float) + "\n")
        self.dumps.append(name)
