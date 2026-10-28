"""D2 against the REAL scan packet: a stationary robot in the narrowest aisle. (WP1 item 1)

Requires a running CoppeliaSim (`pytest -m sim`). `tests/test_d2_aisle_strata.py` settles
the geometry on synthetic scans -- exact ranges from the body origin, one per bearing. This
settles the part only the simulator can answer: whether the scan the certified layer will
actually receive keeps the same promise.

Three things differ between the synthetic scan and the real one, and each could break the
D2 requirement without touching the geometry:

  * **Binning takes the nearest point in each sector**, not the range at its centre. The
    closest point of a wall inside a 3 deg bin is nearer than the bin-centre range, so real
    ranges are shorter than synthetic ones -- the direction that trips a stop.
  * **The sensors sit on the corners**, not at the body origin, so what is visible in a bin
    is decided from there (`pfbnav/sim/coverage.py`), and a bin flagged unobserved breaches
    the field whatever it reads (hard rule 5).
  * **Ranges carry measurement error** (pixel sampling, ~1 % at these distances).

The aisle is built from two thin walls placed about `base_ref`, so it is centred on the
robot wherever the robot stands. They exist in memory only and are removed afterwards; the
scene is never saved.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from pfbnav.safety.certified import FieldGeometry, field_breached
from pfbnav.safety.footprint_table import FootprintTable
from pfbnav.sim.robot import body_point_in_ref
from pfbnav.sim.scanner import Scanner
from pfbnav.world.strata import certified_params_from_config, strata_from_config

pytestmark = pytest.mark.sim

REPO = Path(__file__).resolve().parents[1]

WALL_LENGTH_M = 4.0        # along the aisle, well past the scanners' reach at these angles
WALL_THICKNESS_M = 0.02
# The scanner measures to the wall's near FACE (residual -0.27 to +0.50 mm over 0.28-0.60 m,
# measured 2026-09-27). So each wall is placed with its face where the stratum puts the aisle's
# surface and its centre HALF_THICKNESS_M behind that (2026-09-28).
HALF_THICKNESS_M = WALL_THICKNESS_M / 2
WALL_HEIGHT_M = 0.60       # from the floor, so it crosses the scan plane (0.221 m)
SETTLE_STEPS = 10
# Tolerance on a measured range against the exact distance to the wall's near face. Measured
# 2026-09-27 over six traverses at three clearances: the residual is within +-0.5 mm and does NOT
# scale with distance, so it is not the ~1 % pixel sampling this once assumed. 2 mm is generous.
# It was 0.01 m, which is exactly HALF_THICKNESS_M -- so the tolerance absorbed the systematic
# offset it should have caught, and the test passed while reading 10 mm short of its assertion.
RANGE_TOL_M = 0.002


@pytest.fixture(scope="module")
def aisle(cfg, sim_client):
    """The scanner installed and an aisle of the narrowest per-side clearance around it.

    Yields a function that (re)places the walls at a given clearance and offset and returns
    the scan packet, so one installation serves every case.
    """
    table = FootprintTable.from_csv(REPO / cfg["robot"]["footprint_table"])
    scanner = Scanner(sim_client, cfg)
    sim = sim_client.sim
    half_width = float(cfg["robot"]["half_width"])
    yaw = float(cfg["robot"]["yaw_offset"])
    walls: list[int] = []

    sim_client.stop()                     # the script must exist before sysCall_init runs
    scanner.install()
    sim_client.start(settle_steps=SETTLE_STEPS)
    base_ref = sim_client.object(cfg["sim"]["objects"]["base_ref"])
    ref_z = float(sim.getObjectPosition(base_ref, sim.handle_world)[2])
    for _ in range(2):
        walls.append(int(sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            [WALL_LENGTH_M, WALL_THICKNESS_M, WALL_HEIGHT_M], 0)))

    def place(clearance: float, offset: float = 0.0):
        """Wall FACES at y = +(w - offset) and y = -(w + offset), w = half_width + clearance.

        Each wall is also turned to lie ALONG the aisle: a cuboid is created with its long
        axis on ref +x, while the aisle runs along body +x, a `yaw_offset` away. Placed
        without that turn the walls cross the robot's path instead of flanking it, and the
        scan reads 0.215 m abeam rather than the wall's 0.329 m (found 2026-09-23).
        """
        # Each wall's FACE at the clearance, its centre half a thickness behind it
        # (2026-09-28). A clearance is measured to the aisle's surface; placing the centre there
        # built every aisle 10 mm narrower than the stratum it claimed to test.
        w = half_width + clearance
        centres = (w - offset + HALF_THICKNESS_M, -(w + offset + HALF_THICKNESS_M))
        for wall, y in zip(walls, centres, strict=True):
            x_ref, y_ref, z_ref = body_point_in_ref(
                [0.0, y, 0.5 * WALL_HEIGHT_M - ref_z], yaw)
            sim.setObjectOrientation(wall, [0.0, 0.0, yaw], base_ref)
            sim.setObjectPosition(wall, [x_ref, y_ref, z_ref], base_ref)
        sim_client.step()
        return scanner.read(sim_time=sim_client.time())

    try:
        yield place, table, half_width
    finally:
        for wall in walls:
            if bool(sim.isHandle(wall)):
                sim.removeObjects([wall])
        scanner.remove()
        sim_client.stop()
        sim_client.start(settle_steps=20)   # leave the session fixture as it was found


@pytest.fixture(scope="module")
def narrowest(cfg):
    return min(strata_from_config(cfg), key=lambda s: s.min_clearance)


def test_the_aisle_is_measured_where_it_was_built(aisle, narrowest):
    """The chain end to end: walls at a known distance come back at that distance.

    Against the wall's near FACE, which is now placed where the stratum puts the aisle's surface
    (2026-09-28). History: this test once compared against the wall centre with a 10 mm
    tolerance -- exactly the half-thickness -- and passed while every reading sat a full tolerance
    below its assertion (found 2026-09-27); then the expectation was moved to the face while the
    construction still put the centre at the clearance, so the aisle was 10 mm too narrow (found
    2026-09-28). Now construction and expectation agree on where the surface is.
    """
    place, _table, half_width = aisle
    packet = place(narrowest.min_clearance)
    face = half_width + narrowest.min_clearance
    b = np.asarray(packet.filter_bearings)
    r = np.asarray(packet.filter_ranges)
    for side in (math.pi / 2, -math.pi / 2):
        i = int(np.argmin(np.abs(b - side)))
        assert r[i] == pytest.approx(face, abs=RANGE_TOL_M), (
            f"wall face at {face:.4f} m read {r[i]:.4f} m at {math.degrees(b[i]):.1f} deg")


def test_every_bin_is_observed_in_the_narrowest_aisle(aisle, narrowest):
    """Hard rule 5: an unobserved bin breaches the field whatever it reads, so D2 would
    fail on the inherited layout for a reason that has nothing to do with the aisle."""
    place, _table, _half_width = aisle
    packet = place(narrowest.min_clearance)
    assert np.asarray(packet.filter_valid).all()
    assert np.asarray(packet.policy_valid).all()


def test_a_stationary_robot_centred_in_the_narrowest_aisle_triggers_no_stop(
        cfg, aisle, narrowest):
    """D2's requirement, on the scan the certified layer will really receive."""
    place, table, _half_width = aisle
    packet = place(narrowest.min_clearance)
    fields = FieldGeometry(table.extent, certified_params_from_config(cfg))
    field = fields.select(np.asarray(packet.filter_bearings), 0.0, 0.0, at_rest=True)
    breach = field_breached(packet.filter_ranges, packet.filter_valid, field)
    margin = float(np.min(np.asarray(packet.filter_ranges) - field))
    print(f"\nat rest in the narrowest aisle: breach={breach}, "
          f"tightest bin clears the field by {margin * 1000:.1f} mm")
    assert not breach


def test_the_top_band_still_fits_while_tracking_off_centre(cfg, aisle, narrowest):
    """The moving field at the top band, with the robot displaced by G1b's committed peak
    cross-track -- the offset the narrow floor was derived to absorb (ruling 4)."""
    place, table, _half_width = aisle
    params = certified_params_from_config(cfg)
    fields = FieldGeometry(table.extent, params)
    prereg = yaml.safe_load((REPO / "preregistration" / "gates.yaml").read_text())
    offset = float(prereg["G1b_closed_loop_tracking"]["max_cross_track"])
    for shift in (0.0, offset):
        packet = place(narrowest.min_clearance, shift)
        field = fields.select(np.asarray(packet.filter_bearings),
                              params.band_top_speeds[-1], 0.0, at_rest=False)
        margin = float(np.min(np.asarray(packet.filter_ranges) - field))
        print(f"offset {shift * 1000:.0f} mm: tightest bin clears the moving field by "
              f"{margin * 1000:.1f} mm")
        assert not field_breached(packet.filter_ranges, packet.filter_valid, field)


def test_an_aisle_below_the_rest_floor_does_trip_a_stop(cfg, aisle):
    """The other side of the knife edge: the test can fail when it should, and the real
    scan agrees with the geometry about where the floor is."""
    place, table, half_width = aisle
    fields = FieldGeometry(table.extent, certified_params_from_config(cfg))
    floor = fields.lateral_reach(0.0, 0.0, at_rest=True) - half_width
    packet = place(floor - 0.01)
    field = fields.select(np.asarray(packet.filter_bearings), 0.0, 0.0, at_rest=True)
    assert field_breached(packet.filter_ranges, packet.filter_valid, field)
