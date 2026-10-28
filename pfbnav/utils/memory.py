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
