"""The traversability precondition, and the unavoidable-stop floor per stratum.

Pure functions; no simulator. `scripts/probe_traversability.py` drives the simulator and
feeds its counts in here.

**Why this exists** (supervisor, 2026-09-26 and 2026-09-27). A stratum is defined by geometry,
but whether a *correctly tracking* robot can actually traverse it is a measurement. The
precondition drives the aisle centreline at the stratum's derived speed, displaced by the
committed peak cross-track -- the worst position a controller inside its tolerance may occupy
-- and records the fraction of steps the certified field is breached. **A stratum in which such
a robot is permanently inside the field is inadmissible: the floor is raised, the field is never
shrunk.**

Two quantities come out of it and they answer different questions:

  * **`fraction_inside`** -- how much of the traverse is spent inside the field. Any value above
    zero means the geometry forces protective stops on a robot that is tracking correctly.
  * **`stops_per_100m` from an IDEAL centreline tracker** -- the **unavoidable-stop floor**.
    No controller can do better in that stratum, because a controller with zero cross-track
    error is the best case. H1 is then reported as **stops above this floor**, beside the raw
    count, so a method is not credited or blamed for what the aisle costs everybody.

The layer is **evaluated but not overriding** while this is measured, for the same reason G0's
`env_step_loop` context does it: a stop that zeroes the command ends the traverse, distance
goes to zero, and stops-per-100-m diverges. What is wanted is how many stops the geometry
forces over a given distance, which needs the distance to be travelled.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# A traverse of zero steps is not a traversable stratum; it is a probe that did not run.
MIN_STEPS = 1


@dataclass(frozen=True)
class TraverseResult:
    """One traverse of one stratum, at one lateral offset."""

    stratum: str
    clearance_m: float
    speed_m_s: float
    offset_m: float
    steps: int
    steps_breached: int
    stops: int                 # rising edges, i.e. protective stops that would have fired
    distance_m: float

    def __post_init__(self) -> None:
        if self.steps < 0 or self.steps_breached < 0 or self.stops < 0:
            raise ValueError("counts may not be negative")
        if self.steps_breached > self.steps:
            raise ValueError(
                f"{self.steps_breached} breached steps out of {self.steps}")
        if self.distance_m < 0.0:
            raise ValueError("distance may not be negative")

    @property
    def ran(self) -> bool:
        return self.steps >= MIN_STEPS

    @property
    def fraction_inside(self) -> float | None:
        """Fraction of steps with the field breached. None if the traverse did not run."""
        if not self.ran:
            return None
        return self.steps_breached / self.steps

    @property
    def permanently_inside(self) -> bool:
        """Every step inside the field: the robot could never legally move here."""
        return self.ran and self.steps_breached == self.steps

    @property
    def admissible(self) -> bool:
        """The ruled criterion: a stratum permanently inside the field is inadmissible.

        Note what this does NOT say. A stratum with `fraction_inside` of 0.9 is admissible by
        this criterion while being close to useless, so the fraction is always reported and
        never collapsed into the boolean. If a threshold below 1.0 is wanted it is a
        preregistered number, not a default chosen here.
        """
        return self.ran and not self.permanently_inside

    @property
    def stops_per_100m(self) -> float | None:
        """Protective stops per 100 m. None until the robot has moved: 0/0 is not zero."""
        if self.distance_m <= 0.0:
            return None
        return 100.0 * self.stops / self.distance_m

    def as_dict(self) -> dict[str, Any]:
        return {"stratum": self.stratum, "clearance_m": self.clearance_m,
                "speed_m_s": self.speed_m_s, "offset_m": self.offset_m,
                "steps": self.steps, "steps_breached": self.steps_breached,
                "stops": self.stops, "distance_m": self.distance_m,
                "fraction_inside": self.fraction_inside,
                "permanently_inside": self.permanently_inside,
                "admissible": self.admissible,
                "stops_per_100m": self.stops_per_100m}


def stops_above_floor(observed_per_100m: float, floor_per_100m: float) -> float:
    """Stops per 100 m above the stratum's unavoidable floor, for H1 (supervisor 2026-09-27).

    **Deliberately not clamped at zero.** A negative value means the controller incurred fewer
    stops than an ideal centreline tracker did -- which is possible, because the ideal tracker
    follows the centreline and a controller may take a line the geometry treats more kindly.
    Clamping would present that as "no better than the floor" and hide a real result. It is
    reported beside the raw count, never instead of it: the raw count is the primary endpoint.
    """
    return float(observed_per_100m) - float(floor_per_100m)


def floor_from_ideal(results: list[TraverseResult]) -> dict[str, float | None]:
    """The unavoidable-stop floor per stratum: the ideal (zero-offset) traverse's rate.

    Only zero-offset traverses count. A displaced robot is not the best case, so its rate is
    not a floor, and letting one in would make the floor depend on the tolerance rather than
    on the aisle.
    """
    floors: dict[str, float | None] = {}
    for r in results:
        if r.offset_m != 0.0:
            continue
        if r.stratum in floors:
            raise ValueError(f"two ideal traverses for stratum {r.stratum}")
        floors[r.stratum] = r.stops_per_100m
    return floors


def precondition_report(results: list[TraverseResult]) -> dict[str, Any]:
    """The precondition's verdict over every traverse, with the numbers it rests on."""
    if not results:
        raise ValueError("no traverses")
    floors = floor_from_ideal(results)
    inadmissible = sorted({r.stratum for r in results if not r.admissible})
    return {
        "traverses": [r.as_dict() for r in results],
        "unavoidable_stop_floor_per_100m": floors,
        "inadmissible_strata": inadmissible,
        "passed": not inadmissible,
        "criterion": ("a stratum in which a robot tracking within the committed cross-track "
                      "bound is permanently inside the field is inadmissible; raise the "
                      "floor, never shrink the field"),
    }
