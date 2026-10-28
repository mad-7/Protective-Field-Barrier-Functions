"""D7 froze the physics engine and substep in configs/, and every start() must apply them.

The scene file still carries Bullet, so a client that trusted the scene would run on the
wrong engine without complaint. A fake `sim` object stands in for CoppeliaSim here.
"""
import pytest

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection, SimError


class FakeSim:
    intparam_dynamic_engine = "engine"
    floatparam_dynamic_step_size = "dyn_step"
    floatparam_simulation_time_step = "sim_step"
    simulation_stopped = 0

    def __init__(self, available_engines=(0, 1, 3, 4)):
        self.available = set(available_engines)
        self.params = {"engine": 0, "dyn_step": 0.005, "sim_step": 0.05}
        self.log = []

    def setInt32Param(self, key, value):
        if key == "engine" and value not in self.available:
            return  # silently ignored, as an absent engine would be
        self.params[key] = value

    def getInt32Param(self, key):
        return self.params[key]

    def setFloatParam(self, key, value):
        self.params[key] = value

    def getFloatParam(self, key):
        return self.params[key]

    def getSimulationTimeStep(self):
        return self.params["sim_step"]

    def stopSimulation(self):
        self.log.append("stop")

    def getSimulationState(self):
        return self.simulation_stopped

    def setStepping(self, flag):
        pass

    def startSimulation(self):
        self.log.append(("start", self.params["engine"], self.params["dyn_step"]))

    def step(self):
        pass


def _client(engine=1, dynamic_step=0.002, sim=None):
    client = SimClient(SimConnection("127.0.0.1", 23000, 0.05, engine, dynamic_step))
    client.sim = sim or FakeSim()
    return client


def test_base_config_carries_the_frozen_physics():
    connection = SimConnection.from_config(load_config("configs/base.yaml"))
    assert connection.engine == 1           # ODE, D7 2026-09-18
    assert connection.dynamic_step == pytest.approx(0.005)


def test_start_applies_configured_physics_before_starting():
    client = _client(engine=1, dynamic_step=0.002)
    client.start()
    assert client.sim.log[-1] == ("start", 1, 0.002)


def test_start_reapplies_physics_on_every_start():
    client = _client(engine=1, dynamic_step=0.005)
    client.start()
    client.sim.params["engine"] = 0          # e.g. a scene reload between runs
    client.stop()
    client.start()
    assert client.sim.log[-1] == ("start", 1, 0.005)


def test_unavailable_engine_fails_instead_of_running_the_previous_one():
    client = _client(engine=2, sim=FakeSim(available_engines=(0, 1)))
    with pytest.raises(SimError, match="engine 2"):
        client.start()
    assert not any(isinstance(e, tuple) for e in client.sim.log)


def test_physics_cannot_change_while_running():
    client = _client()
    client.start()
    with pytest.raises(SimError, match="stop the simulation"):
        client.set_physics(engine=0)
