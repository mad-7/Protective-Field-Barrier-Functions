"""The simulator client never waits forever for a reply (hard rule 9; 2026-10-18).

The B1 evaluation of 2026-10-17 stalled after episode 51 while the host ran out of commit memory:
the ZeroMQ client waits on `recv()` with no timeout, so an instance that stops answering hung the
run silently, with no error and no summary. No simulator is needed here: a fake instance on a
local socket either answers late (the call must raise `SimError`) or answers in time with an
error reply (positive control: the reply gets through, and the timeout is not what fails).
"""
import socket
import threading
import time

import coppeliasim_zmqremoteapi_client as zmq_client
import pytest
import zmq

from pfbnav.sim.client import SimClient, SimConnection, SimError


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _fake_instance(port: int, delay: float, stop: threading.Event) -> threading.Thread:
    """A REP socket that answers every request with an error reply after `delay` s."""
    ctx = zmq.Context.instance()
    rep = ctx.socket(zmq.REP)
    rep.setsockopt(zmq.LINGER, 0)
    rep.bind(f"tcp://127.0.0.1:{port}")

    def serve():
        poller = zmq.Poller()
        poller.register(rep, zmq.POLLIN)
        while not stop.is_set():
            if poller.poll(50):
                rep.recv()
                time.sleep(delay)
                rep.send(zmq_client.cbor.dumps({"err": "probe reply"}))   # the client's own codec
        rep.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    return t


def _client(port: int, timeout: float) -> SimClient:
    return SimClient(SimConnection("127.0.0.1", port, 0.05, 1, 0.005, reply_timeout=timeout))


def test_an_instance_that_never_answers_raises_instead_of_hanging():
    port = _free_port()                         # nothing listens: the request is never answered
    started = time.monotonic()
    with pytest.raises(SimError, match="did not reply"):
        _client(port, 0.3).connect()
    assert time.monotonic() - started < 5.0


def test_a_late_reply_raises_and_a_timely_one_gets_through():
    for delay, timeout, expect in ((1.0, 0.3, SimError), (0.05, 2.0, Exception)):
        port = _free_port()
        stop = threading.Event()
        server = _fake_instance(port, delay, stop)
        try:
            with pytest.raises(Exception) as info:
                _client(port, timeout).connect()
            if expect is SimError:
                assert isinstance(info.value, SimError)
            else:
                # positive control: the fake's own error reply arrived, so the timeout did not fire
                assert not isinstance(info.value, SimError)
                assert "probe reply" in str(info.value)
        finally:
            stop.set()
            server.join(timeout=3.0)


def test_the_timeout_comes_from_the_configuration():
    from pathlib import Path

    from pfbnav.config import load_config
    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "base.yaml")
    conn = SimConnection.from_config(cfg)
    assert conn.reply_timeout == pytest.approx(float(cfg["sim"]["reply_timeout_s"]))
    assert conn.reply_timeout > 10.0          # far above any one call (a step ~50 ms)
    with pytest.raises(ValueError):
        SimClient(SimConnection("127.0.0.1", _free_port(), 0.05, 1, 0.005,
                                reply_timeout=0.0)).connect()
