"""Known-answer tests for the traversability precondition and the unavoidable-stop floor.

Hard rule 6: every metric gets a test on synthetic data with a known answer before it is
applied to real results. No simulator; `scripts/probe_traversability.py` supplies the counts.
"""
import pytest

from pfbnav.world.traversability import (
    TraverseResult,
    floor_from_ideal,
    precondition_report,
    stops_above_floor,
)


def traverse(**kw):
    base = {"stratum": "narrow", "clearance_m": 0.13, "speed_m_s": 0.4, "offset_m": 0.0,
            "steps": 100, "steps_breached": 0, "stops": 0, "distance_m": 4.0}
    return TraverseResult(**{**base, **kw})


# ---------------------------------------------------------------- the fraction inside
def test_the_fraction_inside_is_breached_steps_over_steps():
    assert traverse(steps=100, steps_breached=25).fraction_inside == pytest.approx(0.25)
    assert traverse(steps=100, steps_breached=0).fraction_inside == 0.0


def test_a_traverse_that_did_not_run_reports_nothing_rather_than_zero():
    """0/0 is not "never inside the field", the same reason stops_per_100m is None before the
    robot moves. A probe that failed to start must not read as a clean stratum."""
    empty = traverse(steps=0, steps_breached=0, distance_m=0.0)
    assert empty.fraction_inside is None
    assert empty.stops_per_100m is None
    assert not empty.ran
    assert not empty.admissible, "a traverse that never ran is not evidence of admissibility"


def test_counts_that_cannot_be_true_are_refused():
    with pytest.raises(ValueError, match="breached steps"):
        traverse(steps=10, steps_breached=11)
    with pytest.raises(ValueError, match="negative"):
        traverse(steps=-1)
    with pytest.raises(ValueError, match="negative"):
        traverse(distance_m=-0.1)


# ---------------------------------------------------------------- the ruled criterion
def test_permanently_inside_is_inadmissible():
    """The ruling: a stratum where a correctly tracking robot is always inside the field is
    inadmissible -- raise the floor, never shrink the field."""
    always = traverse(steps=200, steps_breached=200, offset_m=0.05)
    assert always.permanently_inside and not always.admissible


def test_mostly_inside_is_still_admissible_but_the_fraction_is_reported():
    """The criterion is 'permanently', not 'mostly'. 0.9 passes and would be a terrible
    stratum, which is why the fraction is always carried beside the boolean rather than
    collapsed into it. A threshold below 1.0 would be a preregistered number, not a default."""
    nearly = traverse(steps=200, steps_breached=180, offset_m=0.05)
    assert nearly.admissible
    assert nearly.fraction_inside == pytest.approx(0.9)


def test_one_clear_step_is_enough_to_be_admissible_by_the_ruled_criterion():
    assert traverse(steps=200, steps_breached=199).admissible


# ---------------------------------------------------------------- the floor
def test_stops_per_100m_known_answer():
    assert traverse(stops=3, distance_m=50.0).stops_per_100m == pytest.approx(6.0)
    assert traverse(stops=1, distance_m=200.0).stops_per_100m == pytest.approx(0.5)
    assert traverse(stops=0, distance_m=12.0).stops_per_100m == 0.0


def test_the_floor_comes_only_from_the_ideal_traverse():
    """A displaced robot is not the best case, so its rate is not a floor. Letting one in
    would make the floor depend on the tracking tolerance rather than on the aisle."""
    results = [traverse(stratum="narrow", offset_m=0.0, stops=2, distance_m=4.0),
               traverse(stratum="narrow", offset_m=0.05, stops=9, distance_m=4.0),
               traverse(stratum="wide", clearance_m=0.4, offset_m=0.0, stops=0,
                        distance_m=4.0)]
    assert floor_from_ideal(results) == {"narrow": pytest.approx(50.0), "wide": 0.0}


def test_two_ideal_traverses_for_one_stratum_are_an_error():
    """Which one is the floor? Silently taking either would make the answer depend on order."""
    results = [traverse(offset_m=0.0), traverse(offset_m=0.0)]
    with pytest.raises(ValueError, match="two ideal traverses"):
        floor_from_ideal(results)


def test_stops_above_floor_known_answer():
    assert stops_above_floor(12.0, 5.0) == pytest.approx(7.0)
    assert stops_above_floor(5.0, 5.0) == pytest.approx(0.0)


def test_stops_above_floor_is_not_clamped_at_zero():
    """A controller CAN beat the ideal centreline tracker by taking a line the geometry treats
    more kindly. Clamping would report that as "no better than the floor" and hide a result."""
    assert stops_above_floor(3.0, 5.0) == pytest.approx(-2.0)


# ---------------------------------------------------------------- the report
def test_the_report_fails_on_an_inadmissible_stratum_and_names_it():
    results = [traverse(stratum="narrow", offset_m=0.0, stops=1, distance_m=4.0),
               traverse(stratum="narrow", offset_m=0.05, steps=200, steps_breached=200),
               traverse(stratum="wide", clearance_m=0.4, offset_m=0.0)]
    report = precondition_report(results)
    assert not report["passed"]
    assert report["inadmissible_strata"] == ["narrow"]
    assert report["unavoidable_stop_floor_per_100m"]["narrow"] == pytest.approx(25.0)


def test_the_report_passes_when_every_stratum_can_be_traversed():
    results = [traverse(stratum="narrow", offset_m=0.0),
               traverse(stratum="narrow", offset_m=0.05, steps=200, steps_breached=3)]
    report = precondition_report(results)
    assert report["passed"] and report["inadmissible_strata"] == []
    assert len(report["traverses"]) == 2


def test_an_empty_report_is_an_error_rather_than_a_pass():
    """Nothing measured is not "every stratum is fine"; it is a probe that did not run."""
    with pytest.raises(ValueError, match="no traverses"):
        precondition_report([])
