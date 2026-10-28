"""Shared fixtures for the simulator-backed tests.

The connection and the scene load are session-scoped so that a `pytest -m sim`
run loads `sim.scene` once, rather than restarting the researcher's instance for
every test module.

This fixture does the minimum needed to hand back a working client, and nothing
else. Anything optional here -- a diagnostic print, a convenience readout -- is a
single point of failure for every sim-marked test in the run: on 2026-09-17 a
version print using a constant that does not exist in CoppeliaSim 4.10 errored all
nine of them at setup. Optional readouts belong in a test, where a failure costs
one test instead of the whole run.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection

REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def cfg() -> dict:
    return load_config(REPO_ROOT / "configs" / "base.yaml")


@pytest.fixture(scope="session")
def sim_client(cfg):
    """A connected, running simulator with the configured scene loaded.

    Loads `sim.scene` rather than measuring whatever scene happens to be open, so
    that anything recorded in the config from a sim run is traceable to a named
    scene file.
    """
    scene = REPO_ROOT / cfg["sim"]["scene"]
    if not scene.is_file():
        pytest.fail(
            f"scene {scene} does not exist; the sim-marked tests load the configured "
            "scene so their measurements are traceable to a named scene file."
        )
    client = SimClient(SimConnection.from_config(cfg))
    client.connect()
    # A simulation left running by an earlier (perhaps interrupted) run makes loadScene
    # refuse: "simulation is not stopped". Stopping first costs nothing when it is already
    # stopped, and means a leftover does not fail every sim-marked test (2026-09-23).
    client.stop()
    client.load_scene(str(scene))
    client.start(settle_steps=20)
    try:
        yield client
    finally:
        client.stop()
