"""The scanner's client side against a fake simulator (hard rule 6).

No CoppeliaSim: a fake `sim` records what the client asks of it and plays back a packet
built with `encode_scan`, so installation, configuration, validity masks, the tick check and
the decode are all verified before an instance is involved. What the fake cannot check --
that `lua/scanner_v2.lua` produces this layout from real geometry -- is what the simulator
checks (scripts/diag_scanner_check.py, scripts/diag_coverage_map.py) are for.

The fake robot carries the inherited layout: two 240 deg Hokuyo pods on the centreline at
body x = +-0.325 m, each two 120 deg sensors, in youBot_ref's frame a quarter turn from the
body frame (yaw_offset pi/2) -- the geometry the scanner itself reported on 2026-09-21.
"""
import copy
import math
import re
import struct
from typing import ClassVar

import numpy as np
import pytest

from pfbnav.config import load_config
from pfbnav.sim.client import SimClient, SimConnection, SimError
from pfbnav.sim.depth_geometry import matrix_from_pose
from pfbnav.sim.scan_packet import FORMAT_VERSION, bin_bearings, encode_scan
from pfbnav.sim.scanner import (
    CONFIG_SIGNAL,
    LUA_PATH,
    SCAN_SIGNAL,
    STATUS_SIGNAL,
    Scanner,
    rays_for,
    sensor_view_from_matrix,
)

CFG = load_config("configs/base.yaml")
POLICY = int(CFG["scanners"]["policy_bins"])
FILTER = int(CFG["scanners"]["filter_bins"])
R_MIN = float(CFG["scanners"]["range_min"])
R_MAX = float(CFG["scanners"]["range_max"])
DT = float(CFG["sim"]["dt"])
YAW = float(CFG["robot"]["yaw_offset"])
ROOT, REF = 10, 11


def ref_matrix(body_x, body_y, body_heading_deg, z=0.163):
    """A sensor's matrix relative to youBot_ref, given its pose in the body frame."""
    c, s = math.cos(YAW), math.sin(YAW)            # body -> ref is +yaw_offset
    x, y = c * body_x - s * body_y, s * body_x + c * body_y
    return matrix_from_pose(math.radians(body_heading_deg) + YAW, [x, y, z])


class FakeSim:
    handle_scene = -2
    handle_all = -3
    sceneobject_visionsensor = "visionsensor"
    sceneobject_script = "script"
    scripttype_simulation = 1
    visionfloatparam_perspective_angle = "fov"

    # Pods 200 (front) and 201 (rear) under the robot root, two sensors each, two scripts
    # each -- the inherited Hokuyo layout.
    PODS: ClassVar[dict[int, list[int]]] = {200: [210, 211], 201: [212, 213]}
    SENSOR_POD: ClassVar[dict[int, int]] = {90: 200, 91: 200, 92: 201, 93: 201}
    # The S300 layout as read back from the pinned scene (LAYOUT_20260922T170201Z): two
    # models on diagonal corners, each with two 135 deg sensors, so 270 deg per scanner.
    SENSOR_POSE: ClassVar[dict[int, tuple[float, float, float]]] = {
        90: (0.226, -0.147, -112.56), 91: (0.226, -0.147, 22.44),
        92: (-0.226, 0.145, 67.44), 93: (-0.226, 0.145, -157.56)}

    def __init__(self, *, fov_deg=135.0, legacy_disabled=(), refuse_disable=False,
                 sensors=None, poses=None):
        self.props: dict[str, bytes] = {}
        self.poses = dict(self.SENSOR_POSE if poses is None else poses)
        self.sensor_handles = list(self.poses) if sensors is None else sensors
        self.fovs = {h: math.radians(fov_deg) for h in self.sensor_handles}
        self.created: list[str] = []
        self.removed: list[int] = []
        self.next_handle = 500
        self.script_disabled = {h: (h in legacy_disabled)
                                for hs in self.PODS.values() for h in hs}
        self.refuse_disable = refuse_disable

    # -- objects ----------------------------------------------------
    def getObject(self, path):
        return {"/youBot": ROOT, "/youBot/youBot_ref": REF}.get(path, 12)

    def getObjectsInTree(self, root, kind=None, options=0):
        if kind == self.sceneobject_visionsensor:
            return list(self.sensor_handles)
        if kind == self.sceneobject_script:
            return list(self.PODS.get(root, []))
        return []

    def getObjectParent(self, handle):
        if handle in self.SENSOR_POD:
            return self.SENSOR_POD[handle]
        if handle in self.PODS:
            return ROOT
        return -1

    def getObjectMatrix(self, handle, relative_to):
        assert relative_to == REF
        return ref_matrix(*self.poses[handle])

    def getBoolProperty(self, handle, name, options=None):
        assert name == "scriptDisabled"
        return self.script_disabled[handle]

    def setBoolProperty(self, handle, name, value, options=None):
        assert name == "scriptDisabled"
        if self.refuse_disable and value:
            return                              # a write that silently does nothing
        self.script_disabled[handle] = bool(value)

    def getObjectFloatParam(self, handle, param):
        return self.fovs[handle]

    # -- properties (signals) ---------------------------------------
    def setBufferProperty(self, target, name, value):
        self.props[name] = value

    def getBufferProperty(self, target, name, options=None):
        if name not in self.props:
            if options and options.get("noError"):
                return None
            raise RuntimeError(f"no property {name}")
        return self.props[name]

    # -- packing (CoppeliaSim's own helpers, stood in for here) ------
    def packTable(self, table):
        return repr(table).encode()

    def unpackTable(self, raw):
        import ast
        return ast.literal_eval(raw.decode())

    def packFloatTable(self, values):
        return struct.pack(f"<{len(values)}f", *values)

    def unpackFloatTable(self, raw):
        return list(struct.unpack(f"<{len(raw) // 4}f", raw))

    # -- scripts ----------------------------------------------------
    def createScript(self, script_type, text, options=0, lang="lua"):
        self.created.append(text)
        self.next_handle += 1
        return self.next_handle

    def removeObjects(self, handles):
        self.removed.extend(handles)


def _client(sim):
    client = SimClient(SimConnection("127.0.0.1", 23000, DT, 1, 0.005))
    client.sim = sim
    return client


def _scanner(sim=None, cfg=CFG):
    sim = sim or FakeSim()
    return Scanner(_client(sim), cfg), sim


def _config_sent(sim):
    return sim.unpackTable(sim.props[CONFIG_SIGNAL])


def _publish(sim, *, sim_time=1.0, filter_ranges=None, filter_valid=None):
    fb = bin_bearings(FILTER)
    values = encode_scan(
        sim_time,
        np.full(POLICY, R_MAX), np.ones(POLICY, bool), fb,
        np.full(FILTER, R_MAX) if filter_ranges is None else filter_ranges,
        np.ones(FILTER, bool) if filter_valid is None else filter_valid,
        range_min=R_MIN, range_max=R_MAX)
    sim.props[SCAN_SIGNAL] = sim.packFloatTable(values)


# ----------------------------------------------------------------------
# Sensor geometry
# ----------------------------------------------------------------------
def test_a_sensor_pose_comes_back_in_the_body_frame():
    # Built in the body frame, carried through youBot_ref's quarter turn, read back.
    v = sensor_view_from_matrix(ref_matrix(0.325, 0.0, 60.0), math.radians(120.0), YAW, R_MAX)
    assert (v.x, v.y) == pytest.approx((0.325, 0.0), abs=1e-9)
    assert math.degrees(v.heading) == pytest.approx(60.0)


def test_ray_count_is_field_of_view_times_density():
    assert rays_for(120.0, 4.2666667) == 512
    assert rays_for(135.0, 4.2666667) == 576                # the S300's sensors
    assert rays_for(120.00000333930423, 4.2666667) == 512   # single-precision noise


def test_the_configured_density_is_the_one_derived_from_the_resolution_experiment():
    assert rays_for(120.0, CFG["scanners"]["rays_per_degree"]) == 512


def test_a_density_below_what_the_bins_need_is_refused():
    cfg = copy.deepcopy(CFG)
    cfg["scanners"]["rays_per_degree"] = 0.5                # 60 rays on 120 deg; bins need 80
    scanner, _ = _scanner(cfg=cfg)
    with pytest.raises(ValueError, match="filter bins need"):
        scanner.install()


def test_a_robot_with_no_scanners_is_refused():
    scanner, _ = _scanner(FakeSim(sensors=[]))
    with pytest.raises(SimError, match="no vision sensors"):
        scanner.install()


# ----------------------------------------------------------------------
# Position-aware validity (the fix of 2026-09-21)
# ----------------------------------------------------------------------
def test_the_s300_layout_leaves_no_bin_unobserved():
    # Two 270 deg scanners on diagonal corners see the whole ring: what the layout change
    # was for. The inherited pods' blind zone is kept as pure geometry in test_coverage.py.
    scanner, sim = _scanner()
    scanner.install()
    cfg = _config_sent(sim)
    mask = np.array(cfg["filter_valid_mask"], dtype=bool)
    b = np.degrees(bin_bearings(FILTER))
    assert mask.all(), f"unobserved bins at {b[~mask]} deg"
    assert len(cfg["policy_valid_mask"]) == POLICY
    assert all(cfg["policy_valid_mask"])


def test_the_coverage_report_finds_nothing_unobserved_on_the_s300_layout():
    scanner, _ = _scanner()
    scanner.install()
    r = scanner.coverage_report()
    assert r["unobserved_area_m2"] == 0.0
    assert r["filter_valid"] == FILTER and r["policy_valid"] == POLICY
    assert len(r["sensors"]) == 4


def test_the_coverage_report_names_a_blind_zone_when_there_is_one():
    # The inherited centreline pods, as a fixture: the report must still be able to say
    # "unobserved", or the S300 result above would be untestable.
    sim = FakeSim(fov_deg=120.0,
                  poses={90: (0.325, 0.0, 60.0), 91: (0.325, 0.0, -60.0),
                         92: (-0.325, 0.0, 120.0), 93: (-0.325, 0.0, -120.0)})
    scanner, _ = _scanner(sim)
    scanner.install()
    r = scanner.coverage_report()
    assert r["unobserved_area_m2"] > 0.05
    assert r["filter_valid"] < FILTER


def test_the_footprint_travels_to_the_lua_for_the_self_filter():
    scanner, sim = _scanner()
    scanner.install()
    cfg = _config_sent(sim)
    assert len(cfg["footprint_extent"]) == 360 and cfg["footprint_step_deg"] == 1
    assert cfg["self_filter_margin"] == CFG["scanners"]["self_filter_margin"]
    # The table's forward extent is the chassis, 0.2854 m (robot.half_length) plus the
    # spline margin: with the S300s on the corners nothing protrudes fore or aft.
    assert cfg["footprint_extent"][180] == pytest.approx(CFG["robot"]["half_length"],
                                                         abs=0.002)


# ----------------------------------------------------------------------
# Scanner-model scripts: dealt with at run time, never in the scene
# ----------------------------------------------------------------------
def test_scanner_models_are_found_by_ancestry_not_by_name():
    scanner, _ = _scanner()
    scanner.discover()
    assert scanner.scanner_models() == [200, 201]


def test_live_scanner_model_scripts_are_disabled_and_read_back():
    scanner, sim = _scanner()
    scanner.install()
    assert all(sim.script_disabled.values())
    assert scanner.legacy_report() == {"legacy_scripts_found": 4,
                                       "found_live_and_disabled": 4, "already_disabled": 0}


def test_remove_re_enables_only_what_the_scanner_disabled():
    sim = FakeSim(legacy_disabled={210, 212})
    scanner, _ = _scanner(sim)
    scanner.install()
    scanner.remove()
    assert sim.script_disabled == {210: True, 211: False, 212: True, 213: False}


def test_a_script_that_will_not_disable_is_an_error():
    scanner, _ = _scanner(FakeSim(refuse_disable=True))
    with pytest.raises(SimError, match="did not disable"):
        scanner.install()


# ----------------------------------------------------------------------
# Installation
# ----------------------------------------------------------------------
def test_install_publishes_the_configuration_before_creating_the_script():
    scanner, sim = _scanner()
    handle = scanner.install()
    cfg = _config_sent(sim)
    assert cfg["policy_bins"] == POLICY and cfg["filter_bins"] == FILTER
    assert cfg["range_min"] == R_MIN and cfg["range_max"] == R_MAX
    assert cfg["robot_ref"] == CFG["sim"]["objects"]["base_ref"]
    assert cfg["yaw_offset"] == pytest.approx(YAW)
    assert cfg["pixel_mirror"] is CFG["scanners"]["pixel_mirror"]
    assert cfg["rays_per_degree"] == CFG["scanners"]["rays_per_degree"]
    assert "resolution_x" not in cfg                         # per sensor, from density
    assert scanner.script == handle


def test_the_script_installed_is_the_file_in_lua():
    scanner, sim = _scanner()
    scanner.install()
    assert sim.created == [LUA_PATH.read_text(encoding="utf-8")]


def test_a_diagnostic_can_force_one_ray_count():
    scanner, sim = _scanner()
    scanner.install(resolution_x=256)
    assert _config_sent(sim)["resolution_x"] == 256


def test_an_override_of_zero_rays_is_refused():
    scanner, _ = _scanner()
    with pytest.raises(ValueError):
        scanner.install(resolution_x=0)


def test_installing_twice_is_refused():
    scanner, _ = _scanner()
    scanner.install()
    with pytest.raises(SimError, match="already installed"):
        scanner.install()


def test_remove_takes_the_script_out_and_is_safe_to_repeat():
    scanner, sim = _scanner()
    handle = scanner.install()
    scanner.remove()
    scanner.remove()
    assert sim.removed == [handle] and scanner.script is None


# ----------------------------------------------------------------------
# Reading
# ----------------------------------------------------------------------
def test_read_decodes_the_packet():
    scanner, sim = _scanner()
    fr = np.full(FILTER, R_MAX)
    fr[10] = 1.5
    _publish(sim, sim_time=2.0, filter_ranges=fr)
    packet = scanner.read()
    assert packet.sim_time == pytest.approx(2.0)
    assert packet.filter_ranges[10] == pytest.approx(1.5)


def test_a_missing_packet_is_an_error_not_an_empty_scan():
    scanner, _ = _scanner()
    with pytest.raises(SimError, match="not running"):
        scanner.read()


def test_a_packet_from_another_tick_is_refused():
    scanner, sim = _scanner()
    _publish(sim, sim_time=2.0)
    scanner.read(sim_time=2.0)
    with pytest.raises(ValueError, match="did not run this tick"):
        scanner.read(sim_time=2.0 + DT)


def test_an_unobserved_bin_survives_the_round_trip_as_unobserved():
    scanner, sim = _scanner()
    fv = np.ones(FILTER, bool)
    fv[:4] = False
    _publish(sim, filter_valid=fv)
    assert not scanner.read().filter_valid[:4].any()


def test_status_reports_what_the_script_says_about_itself():
    scanner, sim = _scanner()
    sim.props[STATUS_SIGNAL] = sim.packTable({"format_version": FORMAT_VERSION, "sensors": 4})
    assert scanner.read_status()["sensors"] == 4


def test_a_script_that_never_initialised_is_an_error():
    scanner, _ = _scanner()
    with pytest.raises(SimError, match="did not initialise"):
        scanner.read_status()


# ----------------------------------------------------------------------
# The two sides of the contract agree (checked here, not at run time)
# ----------------------------------------------------------------------
LUA = LUA_PATH.read_text(encoding="utf-8")


def test_the_lua_declares_the_same_format_version_as_the_decoder():
    declared = re.search(r"local FORMAT_VERSION = (\d+)", LUA)
    assert declared is not None and int(declared.group(1)) == FORMAT_VERSION


def test_the_lua_uses_the_signal_names_the_client_reads():
    for name in (CONFIG_SIGNAL, SCAN_SIGNAL, STATUS_SIGNAL):
        assert f"'{name}'" in LUA


def test_the_lua_stamps_the_state_time_not_the_pass_time():
    assert "sim.getSimulationTime() + sim.getSimulationTimeStep()" in LUA


def test_the_lua_takes_validity_from_the_mask_not_from_bearings():
    assert "newStream(cfg.filter_bins, cfg.filter_valid_mask)" in LUA
    assert "markCoverage" not in LUA


def test_the_lua_renders_the_robot_and_filters_its_returns_by_the_footprint():
    assert "range <= footprintExtent(bearing) + cfg.self_filter_margin" in LUA


def test_the_lua_excludes_each_sensors_own_model_from_its_render_set():
    # Rendering everything (-1) blinded every sensor on its own casing
    # (SELFOCC_20260922T163041Z); it must not come back.
    assert "entity_to_render, -1)" not in LUA
    assert "local model = ownModel(h, tree)" in LUA
    assert "local renderSet = everythingExcept(model)" in LUA
    assert "sim.addItemToCollection(col, sim.handle_all, -1, 0)" in LUA
    assert "sim.addItemToCollection(col, sim.handle_tree, root, 1)" in LUA
    assert "sim.setObjectInt32Param(h, sim.visionintparam_entity_to_render, renderSet)" in LUA
    # ... confirmed by read-back (hard rule 13).
    assert "if rendered ~= renderSet then" in LUA


def test_the_lua_rotates_every_point_into_the_body_frame():
    assert "cosYaw, sinYaw = math.cos(-cfg.yaw_offset), math.sin(-cfg.yaw_offset)" in LUA
    assert "local x, y = toBody(p[k], p[k + 1])" in LUA


def test_the_lua_transforms_a_sensors_returns_in_one_call():
    assert "local p = sim.multiplyVector(m, flat)" in LUA


def test_the_lua_refuses_returns_beyond_range_max():
    assert "range <= cfg.range_max" in LUA


def test_the_lua_computes_rays_exactly_as_the_client_does():
    # rays_for: ceil(round(fov * density, 4)); the Lua mirrors the rounding.
    assert "math.floor(fovDeg * cfg.rays_per_degree * 10000 + 0.5) / 10000" in LUA
