"""Known-answer tests for the certified-layer emulator's state machine (hard rule 6).

The field geometry is tested in `tests/test_d2_aisle_strata.py`; this covers the sequence
the emulator runs: band selection from the estimated velocity, the response latency before
a stop bites, the latch, the restart delay, and what `stop_triggered` counts -- the primary
metric is protective stops per 100 m, so a miscount here is a miscount of the result.

Scans are synthetic: a ring at a fixed range, or one bin brought in close.
"""
import itertools
import math
from pathlib import Path

import numpy as np
import pytest

from pfbnav.safety.certified import CertifiedLayer, CertifiedParams, FieldGeometry

N_BINS = 120
BEARINGS = np.radians(np.arange(-180.0, 180.0, 360.0 / N_BINS))
PARAMS = CertifiedParams(
    response_time=0.10, a_brake=1.0, restart_delay=2.0, n_field_sets=3,
    margin=0.10, lateral_margin=0.05, standstill_enter=0.01, standstill_exit=0.02,
    band_top_speeds=(0.10, 0.30, 0.51), warning_scale=1.5, warning_extra=0.05,
    warning_extra_abeam=0.02, alpha_brake=1.4,
    # A20 (2026-10-18): the reachable-motion parameters, as configured
    reach_time=0.15, a_max=0.5, alpha_max=1.4, speed_cap=0.5074, yaw_cap=1.5,
    standstill_yaw_enter=0.0296, standstill_yaw_exit=0.0592)
DT = 0.05
HALF_LENGTH, HALF_WIDTH = 0.2854, 0.2086


def layer(radius=0.21):
    """An emulator whose robot is a disc, so every field is a ring of known radius."""
    return CertifiedLayer(PARAMS, HALF_LENGTH, HALF_WIDTH,
                          lambda psi: np.full_like(np.asarray(psi, dtype=float), radius))


def clear(range_m=4.0):
    return np.full(N_BINS, range_m), np.ones(N_BINS, dtype=bool)


def run(cl, steps, velocity, ranges, valid, t0=0.0):
    """Step the emulator `steps` times at DT, returning each step's state."""
    out = []
    for k in range(steps):
        out.append(cl.step(t0 + k * DT, velocity, ranges, BEARINGS, valid))
    return out


def _bin(bearing):
    """Index of the bin nearest a bearing."""
    return int(np.argmin(np.abs(BEARINGS - bearing)))


def _scan(ranges, valid):
    return ranges, BEARINGS, valid


def test_a_clear_field_never_stops():
    cl = layer()
    ranges, valid = clear()
    states = run(cl, 20, (0.3, 0.0, 0.0), ranges, valid)
    assert not any(s["stop_active"] or s["stop_triggered"] for s in states)
    assert cl.stops == 0


def test_the_band_follows_the_estimated_speed_not_the_command():
    """Since A20 (2026-10-18) the band is chosen from the REACHABLE speed, v + a_max T (0.075
    m/s here): 0.022 -> 0.097, band 0; 0.15 -> 0.225, band 1; 0.25 -> 0.325, band 2."""
    cl = layer()
    ranges, valid = clear()
    for speed, expected in ((0.022, 0), (0.15, 1), (0.25, 2)):
        cl.reset()
        # Two steps: the first leaves standstill, the second reports the moving band.
        states = run(cl, 2, (speed, 0.0, 0.0), ranges, valid)
        assert states[-1]["field_id"] == expected
        assert not states[-1]["at_rest"]


def test_at_rest_the_field_is_the_at_rest_one_whatever_the_bands_say():
    cl = layer()
    ranges, valid = clear()
    state = run(cl, 1, (0.0, 0.0, 0.0), ranges, valid)[0]
    assert state["at_rest"] and state["field_id"] == 0


def test_a_speed_above_the_top_band_is_an_error_not_the_top_band():
    """The top band (0.51) covers every reachable speed (cap 0.5074, A20), so 0.50 m/s -- above
    the old 0.40 top, reachable laterally -- has a field; a composed speed above 0.51 does not."""
    cl = layer()
    ranges, valid = clear()
    assert run(cl, 2, (0.50, 0.0, 0.0), ranges, valid)[-1]["field_id"] == 2
    cl.reset()
    with pytest.raises(ValueError, match="exceeds the top band"):
        run(cl, 2, (0.52, 0.0, 0.0), ranges, valid)


def test_a_breach_triggers_once_and_bites_after_the_response_time():
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.25            # ahead, inside the at-rest field (0.21 + 0.10)
    states = run(cl, 6, (0.0, 0.0, 0.0), ranges, valid)
    assert [s["stop_triggered"] for s in states] == [True, False, False, False, False, False]
    # response_time 0.10 s at DT 0.05: triggered at t=0, active from t=0.10, step 2.
    assert [s["stop_active"] for s in states] == [False, False, True, True, True, True]
    assert cl.stops == 1


def test_the_stop_is_latched_and_the_restart_delay_is_served_after_the_field_clears():
    cl = layer()
    blocked, valid = clear()
    blocked = blocked.copy()
    blocked[N_BINS // 2] = 0.25
    run(cl, 4, (0.0, 0.0, 0.0), blocked, valid)          # t = 0.00 .. 0.15, stop active
    assert cl.stop_active
    free, _ = clear()
    # Field clear from t = 0.20; restart_delay 2.0 s, so it holds until t = 2.20.
    states = run(cl, 44, (0.0, 0.0, 0.0), free, valid, t0=0.20)
    active = [s["stop_active"] for s in states]
    assert all(active[:40])                              # t = 0.20 .. 2.15
    assert not any(active[40:])                          # from t = 2.20
    assert cl.stops == 1                                 # one stop, not one per step


def test_a_breach_during_the_restart_delay_restarts_the_delay_without_a_new_stop():
    cl = layer()
    blocked, valid = clear()
    blocked = blocked.copy()
    blocked[N_BINS // 2] = 0.25
    free, _ = clear()
    run(cl, 4, (0.0, 0.0, 0.0), blocked, valid)
    run(cl, 10, (0.0, 0.0, 0.0), free, valid, t0=0.20)   # clearing, t = 0.20 .. 0.65
    run(cl, 2, (0.0, 0.0, 0.0), blocked, valid, t0=0.70) # breached again
    states = run(cl, 40, (0.0, 0.0, 0.0), free, valid, t0=0.80)
    assert cl.stops == 1                                 # still the same stop
    assert states[-1]["stop_active"]                     # t = 2.75 < 0.80 + 2.0 + ...
    later = run(cl, 10, (0.0, 0.0, 0.0), free, valid, t0=2.85)
    assert not later[-1]["stop_active"]


def test_an_unobserved_bin_stops_the_robot_even_with_nothing_in_range():
    cl = layer()
    ranges, valid = clear()
    valid = valid.copy()
    valid[0] = False
    states = run(cl, 4, (0.0, 0.0, 0.0), ranges, valid)
    assert states[0]["stop_triggered"] and states[0]["breached"]
    assert states[-1]["stop_active"]


def test_override_zeroes_the_command_only_while_the_stop_is_active():
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.25
    command = np.array([0.3, 0.1, 0.2])
    first = run(cl, 1, (0.0, 0.0, 0.0), ranges, valid)[0]
    assert not first["stop_active"]
    assert np.allclose(cl.override(command), command)     # the response window: unchanged
    run(cl, 3, (0.0, 0.0, 0.0), ranges, valid, t0=DT)
    assert np.allclose(cl.override(command), np.zeros(3))


def test_reset_clears_the_latch_and_the_count():
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.25
    run(cl, 4, (0.0, 0.0, 0.0), ranges, valid)
    assert cl.stop_active and cl.stops == 1
    cl.reset()
    assert not cl.stop_active and cl.stops == 0
    assert cl.standstill.at_rest


def test_travel_direction_comes_from_the_velocity_and_survives_a_slow_step():
    """A velocity estimate that dips below the standstill threshold must not swing the field's
    depth to a meaningless bearing: `atan2(0, 0)` is 0, i.e. straight ahead.

    Rewritten 2026-09-27. The old version drove along +x, so the heading it was guarding
    (0 rad) was the same as the one a zero velocity would have produced -- and it asserted a
    breach that the at-rest field delivered on its own, when the at-rest field was still the
    larger one. It passed without testing anything. Travelling along **+y** makes the two
    headings differ by 90 deg, and the depth is checked after the robot resumes.
    """
    cl = layer()
    ranges, valid = clear()
    cl.step(0.00, (0.0, 0.3, 0.0), *_scan(ranges, valid))     # heading +90 deg
    cl.step(0.05, (0.0, 0.0, 0.0), *_scan(ranges, valid))     # a dip below standstill
    assert cl._heading == pytest.approx(math.pi / 2), "the dip overwrote the heading"

    # Moving again at 0.3 m/s: the depth (0.075 + 0.21 + 0.10 = 0.385) lies along +y. An
    # obstacle 0.30 m to the LEFT is inside it; the same obstacle straight ahead would not be,
    # because abeam the field reaches only 0.21 + 0.05 = 0.26.
    left = ranges.copy()
    left[_bin(math.pi / 2)] = 0.30
    assert cl.step(0.10, (0.0, 0.3, 0.0), left, BEARINGS, valid)["breached"]

    ahead = ranges.copy()
    ahead[_bin(0.0)] = 0.30
    cl2 = layer()
    cl2.step(0.00, (0.0, 0.3, 0.0), *_scan(ranges, valid))
    cl2.step(0.05, (0.0, 0.0, 0.0), *_scan(ranges, valid))
    assert not cl2.step(0.10, (0.0, 0.3, 0.0), ahead, BEARINGS, valid)["breached"], (
        "the depth is pointing along +x, so the heading did not survive the slow step")


def test_a_warning_is_reported_between_the_protective_and_warning_fields():
    """Known answer on the band's OWN warning field (supervisor 2026-09-26).

    At 0.15 m/s (reachable 0.225: band 1, top 0.30) the protective depth is D = 0.30 T +
    0.30^2 / 2 = 0.09 m (T = 0.15, A20), so the protective field reaches 0.21 + 0.09 + 0.10 =
    0.40 m ahead. The warning field takes D_w = 1.5 * 0.09 + 0.05 = 0.185 and adds
    warning_extra: 0.21 + 0.185 + 0.10 + 0.05 = 0.545 m ahead. An obstacle at 0.47 m is between.
    """
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.47
    states = run(cl, 3, (0.15, 0.0, 0.0), ranges, valid)
    assert not states[-1]["breached"]
    assert states[-1]["warning"]
    assert not states[-1]["stop_active"]


def test_the_top_band_warns_too_now_that_the_warning_field_is_its_own():
    """This test asserted the OPPOSITE until 2026-09-26, and the old answer was the defect.

    Approximating the warning field by "the field of the band above" left the top band --
    the fastest, most dangerous one -- unable to warn at all, because there is no band above
    it. The configured warning field exists in every band, so the top band warns.
    """
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    # the top band (0.51, A20): protective 0.517 m ahead, warning 0.720 m
    ranges[N_BINS // 2] = 0.60     # outside the top band's protective field, inside its warning
    states = run(cl, 3, (0.40, 0.0, 0.0), ranges, valid)
    assert not states[-1]["breached"]
    assert states[-1]["warning"]


def test_a_stationary_robot_can_be_warned_now_that_the_at_rest_field_is_extended():
    """The other half of the old defect: at rest there is no depth to scale, so a warning
    field defined only by a depth factor could never warn a stationary robot. The at-rest
    warning field is the protective one extended by warning_extra all round."""
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    # At rest the protective field is e + rest_reach + lateral_margin = 0.21 + 0.0131 + 0.05 =
    # 0.2731 all round (A20: the reach from the standstill speed); the warning field adds
    # warning_extra_abeam, 0.2931. An obstacle at 0.283 m is between them.
    ranges[N_BINS // 2] = 0.283
    state = cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)
    assert not state["breached"] and state["warning"]


def test_warning_entries_count_rising_edges_not_steps():
    """The near-miss metric is entries, so a long stay inside the field counts once."""
    cl = layer()
    clear_ranges, valid = clear()
    inside = clear_ranges.copy()
    inside[N_BINS // 2] = 0.27          # at rest: between protective 0.26 and warning 0.28
    for _ in range(5):
        cl.step(0.0, (0.0, 0.0, 0.0), inside, BEARINGS, valid)
    assert cl.warning_entries == 1, "a sustained near miss was counted every step"
    cl.step(0.0, (0.0, 0.0, 0.0), clear_ranges, BEARINGS, valid)
    state = cl.step(0.0, (0.0, 0.0, 0.0), inside, BEARINGS, valid)
    assert cl.warning_entries == 2 and state["warning_entered"]
    assert cl.step(0.0, (0.0, 0.0, 0.0), inside, BEARINGS, valid)["warning_entered"] is False


def test_a_protective_breach_is_not_reported_as_a_warning():
    """The warning field contains the protective one, so a stop is always also inside the
    warning field; `warning` means "warned but not yet stopped" and must go quiet."""
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.20          # well inside the protective field at rest
    state = cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)
    assert state["breached"] and not state["warning"]
    # It still counts as a near-miss entry: the robot did enter the warning field.
    assert cl.warning_entries == 1


def test_reset_clears_the_near_miss_count():
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[N_BINS // 2] = 0.27
    cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)
    assert cl.warning_entries == 1
    cl.reset()
    assert cl.warning_entries == 0 and not cl.warning_active


def test_the_default_footprint_is_the_configured_rectangles_circumscribed_radius():
    cl = CertifiedLayer(PARAMS, HALF_LENGTH, HALF_WIDTH)
    ranges, valid = clear()
    ranges = ranges.copy()
    radius = math.hypot(HALF_LENGTH, HALF_WIDTH)
    ranges[N_BINS // 2] = radius + PARAMS.lateral_margin - 0.001
    assert cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)["breached"]


# ------------------------- The field is monotone non-decreasing in speed (supervisor 2026-09-27)
def test_the_field_never_shrinks_as_speed_rises_in_any_direction():
    """The requirement, asserted per bearing and per heading, over the whole band ladder.

    Until 2026-09-27 it did not hold: the at-rest field carried `margin` (0.10) all round
    while a moving field carried `lateral_margin` (0.05) abeam, so the field SHRANK with speed
    over 80 % of bearings at every band, by up to 62.6 mm on the real footprint. A robot was
    protected less the faster it went, and -- the way it surfaced -- a stop taken off-centre in
    a narrow aisle could not be cleared, because the field it had to escape was larger than the
    one it had driven in with.

    The fix makes the at-rest field the INFIMUM of the moving fields, `e(psi) + lateral_margin`.
    This test must exist before the strata lock (ruling 1).
    """
    from pfbnav.config import load_config
    from pfbnav.safety.footprint_table import FootprintTable
    from pfbnav.world.strata import certified_params_from_config
    repo = Path(__file__).resolve().parents[1]
    cfg = load_config(repo / "configs" / "base.yaml")
    params = certified_params_from_config(cfg)
    real = FieldGeometry(FootprintTable.from_csv(repo / cfg["robot"]["footprint_table"]).extent,
                         params)
    disc = FieldGeometry(lambda psi: np.full_like(np.asarray(psi, dtype=float), 0.21), PARAMS)

    psi = np.linspace(-math.pi, math.pi, 721)
    for fields, tops in ((real, params.band_top_speeds), (disc, PARAMS.band_top_speeds)):
        for heading in np.linspace(-math.pi, math.pi, 25):
            previous = fields.extent(psi, 0.0, heading, True)
            for top in tops:
                nxt = fields.extent(psi, top, heading, False)
                worst = float((nxt - previous).min())
                assert worst >= -1e-12, (
                    f"the field shrinks by {-worst * 1000:.3f} mm going to band top {top} "
                    f"at heading {math.degrees(heading):.0f} deg")
                previous = nxt


def test_the_field_never_shrinks_as_the_yaw_rate_rises_nor_as_speed_rises_at_any_yaw_rate():
    """Extended 2026-10-16 (supervisor): the field contains the footprint's ROTATION sweep too,
    so it must be non-decreasing in |omega| as well as in speed, at every bearing. Asserted on
    the real (non-circular) footprint, where rotation matters, for both senses of rotation, at
    rest and in every band, protective and warning, up to the actuation's w_max."""
    from pfbnav.config import load_config
    from pfbnav.safety.footprint_table import FootprintTable
    from pfbnav.world.strata import certified_params_from_config
    repo = Path(__file__).resolve().parents[1]
    cfg = load_config(repo / "configs" / "base.yaml")
    params = certified_params_from_config(cfg)
    real = FieldGeometry(FootprintTable.from_csv(repo / cfg["robot"]["footprint_table"]).extent,
                         params)
    psi = np.linspace(-math.pi, math.pi, 721)
    w_max = float(cfg["actuation"]["w_max"])
    rates = np.linspace(0.0, w_max, 9)
    for heading in np.linspace(-math.pi, math.pi, 13):
        for warning in (False, True):
            for sense in (1.0, -1.0):
                fields_by_rate = []
                for w in rates:
                    ladder = [real.extent(psi, 0.0, heading, True, warning=warning,
                                          yaw_rate=sense * w)]
                    ladder += [real.extent(psi, top, heading, False, warning=warning,
                                           yaw_rate=sense * w) for top in params.band_top_speeds]
                    # in speed, at this yaw rate
                    for lo, hi in itertools.pairwise(ladder):
                        assert float((hi - lo).min()) >= -1e-12, (heading, warning, w)
                    fields_by_rate.append(ladder)
                # in |omega|, at each speed
                for lo, hi in itertools.pairwise(fields_by_rate):
                    for a, b in zip(lo, hi, strict=True):
                        assert float((b - a).min()) >= -1e-12, (heading, warning, sense)


def test_the_warning_field_is_monotone_in_speed_too():
    """It is a field the robot is judged against, so the same requirement applies to it."""
    disc = FieldGeometry(lambda psi: np.full_like(np.asarray(psi, dtype=float), 0.21), PARAMS)
    psi = np.linspace(-math.pi, math.pi, 361)
    for heading in np.linspace(-math.pi, math.pi, 13):
        previous = disc.extent(psi, 0.0, heading, True, warning=True)
        for top in PARAMS.band_top_speeds:
            nxt = disc.extent(psi, top, heading, False, warning=True)
            assert float((nxt - previous).min()) >= -1e-12
            previous = nxt


def test_the_at_rest_field_is_exactly_the_infimum_of_the_moving_fields():
    """Not merely a lower bound: at some heading each moving field TOUCHES it, so the at-rest
    field is the largest set that keeps monotonicity. A smaller one would give away margin
    for nothing."""
    from pfbnav.safety.certified import rest_reach
    disc = FieldGeometry(lambda psi: np.full_like(np.asarray(psi, dtype=float), 0.21), PARAMS)
    psi = np.linspace(-math.pi, math.pi, 721)
    rest = disc.extent(psi, 0.0, 0.0, True)
    # A20: the footprint grown by the reach from the standstill speed, then the lateral margin
    assert np.allclose(rest, 0.21 + rest_reach(PARAMS) + PARAMS.lateral_margin, atol=2e-5)
    # A lower bound, no longer a touching one: since A21 (S2 nesting, 2026-10-19) every moving
    # field contains the at-rest field at every reachable rest pose, so it exceeds the at-rest
    # field everywhere -- by the nesting terms (the extra turn, the at-rest reach on top of the
    # travel, the turning-while-translating drift). For this disc the closest approach is band 0.
    gaps = [float((disc.extent(psi, top, 0.0, False) - rest).min())
            for top in PARAMS.band_top_speeds]
    assert min(gaps) > 0.0
    assert gaps == sorted(gaps)


# ------------------------------------ The restart condition (supervisor 2026-09-27, ruling 3;
# since 2026-10-19 the WHOLE field the commanded twist selects must be clear). For this 0.21 m disc
# the at-rest field reaches 0.2731 m and every moving field more, so an intrusion at 0.28 m sits in
# the departure shell: clear while standing, inside any normal-speed departure's field.
CREEP = 0.015                            # m/s, below the 0.02 standstill exit
def _stop_then_clear(cl, obstacle_range, bearing):
    """Drive into a breach, then remove it, leaving the latch waiting on restart_delay."""
    ranges, valid = clear()
    blocked = ranges.copy()
    blocked[_bin(bearing)] = obstacle_range
    cl.step(0.0, (0.2, 0.0, 0.0), blocked, BEARINGS, valid)
    cl.step(0.2, (0.0, 0.0, 0.0), blocked, BEARINGS, valid)
    return ranges, valid


def test_a_stopped_robot_may_be_approached_to_the_minimum_margin():
    """The counterpart of shrinking the at-rest field. Something standing 0.28 m away -- outside
    the at-rest field's 0.26 m -- does not stop a robot that is already still."""
    cl = layer()
    ranges, valid = clear()
    ranges = ranges.copy()
    ranges[_bin(0.0)] = 0.28
    state = cl.step(0.0, (0.0, 0.0, 0.0), ranges, BEARINGS, valid)
    assert not state["breached"] and not state["stop_active"]



def test_it_may_not_then_drive_toward_the_thing_standing_there():
    """Ruling 3, and since 2026-10-19 the whole selected field: standing near an intrusion does not
    buy the right to set off at it."""
    cl = layer()
    ranges, valid = _stop_then_clear(cl, 0.20, 0.0)
    assert cl.stop_active
    near = ranges.copy()
    near[_bin(0.0)] = 0.28              # clear of the at-rest field, inside every moving field
    t = 0.2
    for _ in range(80):                 # well past restart_delay = 2.0 s
        t += DT
        state = cl.step(t, (0.0, 0.0, 0.0), near, BEARINGS, valid,
                        intended_direction=(0.25, 0.0))
    assert state["restart_blocked"], "the restart condition did not engage"
    assert cl.stop_active, "the robot was released toward an intrusion"



def test_leaving_abeam_needs_the_creep_regime():
    """Since 2026-10-19 a departure whose own field holds the intrusion is blocked, abeam too (it
    would re-trigger at once); a creep keeps the at-rest field, which is clear, so it is released
    and stays released."""
    cl = layer()
    ranges, valid = _stop_then_clear(cl, 0.20, 0.0)
    near = ranges.copy()
    near[_bin(0.0)] = 0.28
    t = 0.2
    for _ in range(80):
        t += DT
        state = cl.step(t, (0.0, 0.0, 0.0), near, BEARINGS, valid,
                        intended_direction=(0.0, 0.25))
    assert state["restart_blocked"] and cl.stop_active
    for _ in range(80):
        t += DT
        state = cl.step(t, (0.0, 0.0, 0.0), near, BEARINGS, valid,
                        intended_direction=(0.0, CREEP))
    assert not cl.stop_active, "a creep abeam stayed latched"
    stops = cl.stops
    for _ in range(20):
        t += DT
        cl.step(t, (0.0, CREEP, 0.0), near, BEARINGS, valid, intended_direction=(0.0, CREEP))
    assert cl.stops == stops, "the creep re-triggered"



def test_reversing_away_never_loops_and_creeping_away_is_released():
    """The release-retrigger loop (found 2026-10-19 under S2 nesting) is gone. Reversing at a normal
    speed would select a moving field that holds the intrusion behind the motion, so it stays
    BLOCKED -- not released and re-stopped. Creeping away keeps the at-rest field: released, and
    no new stop while it creeps."""
    cl = layer()
    ranges, valid = _stop_then_clear(cl, 0.20, 0.0)
    near = ranges.copy()
    near[_bin(0.0)] = 0.28
    t = 0.2
    stops = cl.stops
    for _ in range(80):
        t += DT
        state = cl.step(t, (0.0, 0.0, 0.0), near, BEARINGS, valid,
                        intended_direction=(-0.25, 0.0))
    assert state["restart_blocked"] and cl.stop_active and cl.stops == stops, "no loop"
    for _ in range(80):
        t += DT
        cl.step(t, (0.0, 0.0, 0.0), near, BEARINGS, valid, intended_direction=(-CREEP, 0.0))
    assert not cl.stop_active, "creeping away was blocked"
    for _ in range(20):
        t += DT
        state = cl.step(t, (-CREEP, 0.0, 0.0), near, BEARINGS, valid,
                        intended_direction=(-CREEP, 0.0))
    assert cl.stops == stops and not state["breached"] and state["at_rest"]



def test_the_creep_regime_ends_exactly_at_the_standstill_exit():
    """Known answer for the field the twist selects: at the exit speed itself the robot stays at
    rest (the detector leaves rest only ABOVE it), so the at-rest field governs and a shell
    intrusion does not block; just above it, a band field holds it."""
    cl = layer()
    ranges, valid = clear()
    near = ranges.copy()
    near[_bin(0.0)] = 0.28
    exit_speed = PARAMS.standstill_exit
    assert cl._restart_allowed(near, BEARINGS, valid, (-exit_speed, 0.0))
    assert not cl._restart_allowed(near, BEARINGS, valid, (-1.01 * exit_speed, 0.0))



def test_an_unobserved_bin_blocks_every_restart():
    """Hard rule 5: unobserved space is never free, so a blind bin breaches every field -- the
    whole-field rule (2026-10-19) blocks driving away from it too, creep included."""
    cl = layer()
    ranges, valid = clear()
    blind = valid.copy()
    blind[_bin(0.0)] = False
    for twist in ((0.25, 0.0), (-0.25, 0.0), (-CREEP, 0.0)):
        assert not cl._restart_allowed(ranges, BEARINGS, blind, twist)


def test_an_intrusion_inside_the_minimum_margin_keeps_the_latch_even_driving_away():
    """Where the implementation is STRICTER than the ruling's words -- deliberately, and flagged.

    The ruling allows a restart when the leading field is clear and v . p <= 0; taken alone that
    would release a robot driving away from something 0.25 m behind it, inside the 0.26 m
    minimum margin. But the very next step it is still at rest, the at-rest field is breached,
    and a NEW stop fires -- so it would be released and re-stopped every `restart_delay`, each
    one counted as a protective stop. The latch therefore also keeps its existing requirement
    that the field the robot stands in is clear. Closer than the minimum margin is a real
    breach, and recovery from it is supervised (ruling 4): the episode ends as
    `unrecoverable_stop`.
    """
    cl = layer()
    ranges, valid = _stop_then_clear(cl, 0.20, 0.0)
    inside = ranges.copy()
    inside[_bin(math.pi)] = 0.25         # behind the robot, inside the 0.26 m at-rest field
    # Since 2026-10-19 the restart condition itself blocks it (the whole field holds the point);
    # the latch's own-field requirement stays as before.
    assert not cl._restart_allowed(inside, BEARINGS, valid, (0.25, 0.0))
    t = 0.2
    for _ in range(80):
        t += DT
        cl.step(t, (0.0, 0.0, 0.0), inside, BEARINGS, valid, intended_direction=(0.25, 0.0))
    assert cl.stop_active, "released into an immediate re-stop"


def test_asking_for_nothing_does_not_block_a_restart():
    """No intended direction, or a zero one, is not a request to move, so there is nothing to
    block; the plain at-rest check governs and the latch releases on time."""
    cl = layer()
    ranges, valid = _stop_then_clear(cl, 0.20, 0.0)
    t = 0.2
    for _ in range(80):
        t += DT
        state = cl.step(t, (0.0, 0.0, 0.0), ranges, BEARINGS, valid,
                        intended_direction=(0.0, 0.0))
    assert not state["restart_blocked"] and not cl.stop_active
