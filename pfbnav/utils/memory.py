"""Process memory in run records (ruled 2026-10-19: memory growth stays in every run record).

The A20 memory check (`scripts/check_memory.py`) measured CoppeliaSim growing about +1.7 MB per 100
episodes and Python flat. Every evaluation and training run records both processes' committed
(private) memory at its start and end, and the growth per 100 episodes, so a long run on any host can
be read against that figure. When a value cannot be measured -- psutil absent, the simulator process
not identifiable -- the record says so with a status rather than leaving a gap (hard rule 9).
"""
from __future__ import annotations

import importlib.util
import os
from typing import Any

MB = 2 ** 20


def _psutil():
    if importlib.util.find_spec("psutil") is None:
        return None
    import psutil
    return psutil


def coppelia_process(port: int):
    """The CoppeliaSim process: the one listening on `port` if the OS says, else the single process
    whose name contains 'coppelia'. Raises LookupError when it cannot be identified."""
    psutil = _psutil()
    if psutil is None:
        raise LookupError("psutil is not installed")
    try:
        for conn in psutil.net_connections(kind="tcp"):
            if conn.laddr and conn.laddr.port == port and conn.status == psutil.CONN_LISTEN \
                    and conn.pid:
                return psutil.Process(conn.pid)
    except psutil.AccessDenied:
        pass
    found = [p for p in psutil.process_iter(["name"])
             if "coppelia" in (p.info["name"] or "").lower()]
    if len(found) != 1:
        raise LookupError(f"cannot identify the CoppeliaSim process ({len(found)} candidates)")
    return found[0]


def private_mb(process) -> float:
    """Committed (private) memory of a psutil process, MB; the unique set size off Windows."""
    info = process.memory_info()
    private = getattr(info, "private", None)
    if private is None:
        private = process.memory_full_info().uss
    return private / MB


def growth_per_100(start_mb: float, end_mb: float, episodes: int) -> float | None:
    """(end - start) per 100 episodes, MB; None without an episode to divide by."""
    if episodes <= 0:
        return None
    return (end_mb - start_mb) * 100.0 / episodes


def levels_off(samples: list[dict[str, Any]], *, total_timesteps: int, settled_from: int,
               mark_fraction: float, max_growth_mb: float) -> dict[str, Any]:
    """The training memory criterion (ruled 2026-10-30; operationalised in the decision log).

    "Growth beyond the CUDA start-up cost levels off within the first 10 % of the run": `samples`
    are {"timestep", "python"} (MB), in time order. The start-up costs (CUDA context, the first
    update's cuBLAS/cuDNN workspaces, the replay buffer) are over by `settled_from` (the first sample
    at or after it is the baseline). The mark is `mark_fraction` x `total_timesteps`; the criterion
    passes when the training process grew at most `max_growth_mb` from the first sample at or after
    the mark to the last sample, and the samples reach the run's end (within one sampling interval
    of `total_timesteps`; a check that stopped early cannot pass). Also reports the growth from the
    baseline to the mark and the least-squares slope after the mark, MB per 100k steps."""
    rows = [s for s in samples if s.get("python") is not None]
    out: dict[str, Any] = {"total_timesteps": int(total_timesteps), "settled_from": int(settled_from),
                           "mark_timestep": round(mark_fraction * total_timesteps),
                           "max_growth_mb": float(max_growth_mb), "samples": len(rows)}
    base = next((s for s in rows if s["timestep"] >= settled_from), None)
    mark = next((s for s in rows if s["timestep"] >= out["mark_timestep"]), None)
    if base is None or mark is None or len(rows) < 2:
        return {**out, "passed": False, "reason": "too few samples after the mark"}
    last = rows[-1]
    interval = rows[-1]["timestep"] - rows[-2]["timestep"]
    reached_end = last["timestep"] >= total_timesteps - max(interval, 1)
    after = [s for s in rows if s["timestep"] >= mark["timestep"]]
    t = [float(s["timestep"]) for s in after]
    m = [float(s["python"]) for s in after]
    slope = None
    if len(after) >= 2 and max(t) > min(t):
        mt, mm = sum(t) / len(t), sum(m) / len(m)
        slope = (sum((a - mt) * (b - mm) for a, b in zip(t, m))
                 / sum((a - mt) ** 2 for a in t)) * 1e5
    growth_after = float(last["python"]) - float(mark["python"])
    passed = bool(reached_end and growth_after <= max_growth_mb)
    reason = ("levelled off" if passed else
              "the samples stop before the run's end" if not reached_end else
              f"grew {growth_after:.1f} MB after the mark (> {max_growth_mb:g} MB)")
    return {**out, "passed": passed, "reason": reason,
            "baseline": {"timestep": base["timestep"], "python_mb": base["python"]},
            "at_mark": {"timestep": mark["timestep"], "python_mb": mark["python"]},
            "last": {"timestep": last["timestep"], "python_mb": last["python"]},
            "growth_baseline_to_mark_mb": float(mark["python"]) - float(base["python"]),
            "growth_after_mark_mb": growth_after,
            "slope_after_mark_mb_per_100k_steps": slope}


class MemoryRecord:
    """Start-and-end memory of this Python process and of the simulator on `port`."""

    def __init__(self, port: int | None):
        self.status: dict[str, str] = {}
        self._procs: dict[str, Any] = {}
        psutil = _psutil()
        if psutil is None:
            self.status = {"python": "psutil not installed", "sim": "psutil not installed"}
        else:
            self._procs["python"] = psutil.Process(os.getpid())
            if port is None:
                self.status["sim"] = "no simulator port given"
            else:
                try:
                    self._procs["sim"] = coppelia_process(int(port))
                except LookupError as exc:
                    self.status["sim"] = str(exc)
        self.start = self.snapshot()

    def snapshot(self) -> dict[str, float | None]:
        out: dict[str, float | None] = {}
        for name in ("python", "sim"):
            proc = self._procs.get(name)
            if proc is None:
                out[name] = None
                continue
            try:
                out[name] = private_mb(proc)
            except _psutil().Error as exc:               # the process gone, or access denied
                out[name] = None
                self.status[name] = f"{type(exc).__name__}: {exc}"
        return out

    def report(self, episodes: int) -> dict[str, Any]:
        """The record for a run's summary: start, end, growth per 100 episodes, statuses."""
        end = self.snapshot()
        rec: dict[str, Any] = {"episodes": int(episodes), "status": dict(self.status),
                               "reference": "A20 check: CoppeliaSim +1.7 MB/100 episodes, Python "
                                            "flat (runs/diag/MEMORY_20261018T203246Z_seed0)"}
        for name in ("python", "sim"):
            s, e = self.start.get(name), end.get(name)
            rec[name] = {"private_start_mb": s, "private_end_mb": e,
                         "growth_mb_per_100_episodes": (growth_per_100(s, e, episodes)
                                                        if s is not None and e is not None
                                                        else None)}
        return rec
