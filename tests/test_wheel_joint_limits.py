"""What caps the wheels near 12.34 rad/s? Read the wheel joints' settings. Read-only.

Requires a running CoppeliaSim (`pytest -m sim`).

G1-ODE's diagonal profile commanded 16.0 rad/s and the wheels reached 12.33-12.34 rad/s,
flat, while every profile commanding up to 12.32 rad/s got it (decision log, 2026-09-18).
Ruling 2 on the WP0 results report: find out whether that is a joint velocity limit or a
torque limit, and do not raise it. This reads, for each rolling joint, whatever of the
relevant settings the build exposes; it changes nothing and commands nothing.
"""
from __future__ import annotations

import pytest

from pfbnav.sim.robot import SceneObjects

pytestmark = pytest.mark.sim

# Candidate parameter names across CoppeliaSim 4.x; each is read only if the build has it.
FLOAT_PARAMS = ("jointfloatparam_maxvel", "jointfloatparam_maxaccel",
                "jointfloatparam_upper_limit", "jointfloatparam_velocity",
                "jointfloatparam_kc_k", "jointfloatparam_kc_c")
INT_PARAMS = ("jointintparam_dynctrlmode", "jointintparam_dynvelctrltype",
              "jointintparam_motor_enabled", "jointintparam_ctrl_enabled")


def test_report_wheel_joint_limits(cfg, sim_client):
    sim = sim_client.sim
    objects = SceneObjects.from_config(cfg)
    lines = [""]
    for path in objects.wheels:
        h = sim_client.object(path)
        lines.append(path)
        lines.append(f"   target force/torque: {sim.getJointTargetForce(h)}")
        lines.append(f"   target velocity:     {sim.getJointTargetVelocity(h)}")
        if hasattr(sim, "getJointMode"):
            lines.append(f"   joint mode:          {sim.getJointMode(h)}")
        for name in FLOAT_PARAMS:
            if hasattr(sim, name):
                lines.append(f"   {name:<32} {sim.getObjectFloatParam(h, getattr(sim, name))}")
        for name in INT_PARAMS:
            if hasattr(sim, name):
                lines.append(f"   {name:<32} {sim.getObjectInt32Param(h, getattr(sim, name))}")
    print("\n".join(lines))
