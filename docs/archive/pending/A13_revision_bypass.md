

### A13, revised 2026-10-02 - Feasibility: a local bypass at the narrow floor, not reachability

**Decided by the supervisor on 2026-10-02**, after the first dry run of A13 and before any set is
locked or any policy trains. Measured on tune and pilot seeds only. This revision **replaces**
the feasibility rule stated above ("kept only if the goal remains reachable in the true world");
everything else in A13 stands.

**Why.** The stack plans once, on the mapped map, and never replans. "Reachable in the true world"
was met by detours through other aisles, which the robot cannot take. Planning in the true world
confined to a corridor around the route, at the lethal radius, 29 % of pilot scenarios (43/150)
had no way past their obstacles within +-1.5 m of the route, and 62 % within +-0.75 m; the same
test on the mapped world alone passed in 150/150. In such a scenario every controller stalls or
takes a stop it cannot avoid.

**The rule.** An unmapped obstacle is kept only if, together with every unmapped obstacle already
placed (judged **jointly**: two individually passable obstacles can close a gap together), the
robot can get from start to goal:

* keeping its centre at least `half_width + 0.13 m` from every surface -- the **narrow-stratum
  floor** beyond the footprint, the same admissibility test as any aisle. The lethal radius
  (0.03 m beside the robot) would put the obstacle inside the 0.05 m abeam protective field, a
  stop no controller could avoid. The abeam margin is constant across speed bands, so the floor
  admits the bypass at any band;
* never further than **W = 1.0 m** from the planned route: local avoidance within the corridor
  the route uses, not a detour through another aisle.

A refused obstacle is re-sampled, never the layout. **Fallback, declared before the measurement:**
W = 1.0 m stands unless a tune (20 per stratum) or pilot (150) set cannot be built in full within
the re-sampling cap; only then W = 1.5 m. **It was not needed** (below).

**How it is decided.** A grid of cell centres over the corridor at 0.01 m; the exact distance from
each to every polygon; a cell is admissible at `half_width + 0.13 - 0.005` m; a bypass exists if
start and goal share an 8-connected component (`pfbnav/world/bypass.py`). The half-cell allowance
lets a gap of exactly the floor through at any alignment of the grid -- without it the narrowest
mapped aisles fail the control. Its cost: a gap up to 5 mm per side below the floor may be
admitted, out of the floor's deliberate 0.03 m buffer. A dense sweep (0.115-0.145 m per side in
1 mm steps, five grid alignments) confirms both bounds. **Positive control:** before any obstacle
is placed, each route's empty corridor must pass the same test.

**Measured cost** (dry runs at commit `4384969`, seed 0; tune sets at rate 0.25, and 150
`unmapped_static` scenarios per stratum -- `pilot_narrow` and two probes, `probe_medium` and
`probe_wide`, from pilot seeds 9000150-9000449):

| W (m) | sets built in full | lost to placement | draws per obstacle, narrow / medium / wide | refused for no bypass, % of draws, narrow / medium / wide |
|---|---|---|---|---|
| 0.75 | all | 0 | 87 / 72 / 6 | 6.9 / 14.8 / 28.2 |
| **1.0** | **all** | **0** | **50 / 35 / 4** | **3.6 / 10.8 / 18.6** |
| 1.5 | all | 0 | 40 / 29 / 3 | 2.5 / 9.5 / 15.0 |

Most refusals at every W are `too_close_to_mapped` (the preregistered 0.15 m margin). The positive
control set aside no route in any set at any W. At W = 1.0, at least 12 of 150 narrow and 6 of 150
medium scenarios moved to another endpoint pair because their first route could not hold their
obstacles (a lower bound: scenarios whose draws exceeded 200 per obstacle); none in wide.

**Where obstacles land** (W = 1.0; inside an aisle between two shelf rows / at an aisle's mouth,
within 1.2 m of its end / on open floor):

| stratum | obstacles | aisle | mouth | open floor |
|---|---|---|---|---|
| narrow | 225 | 0 % | 3 % | 97 % |
| medium | 221 | 4 % | 17 % | 79 % |
| wide | 228 | 54 % | 7 % | 39 % |

In the narrow stratum no obstacle can sit inside an aisle: the smallest obstacle (0.25 m), its
0.15 m margin to the shelf behind it, and a bypass at the floor (2 x 0.3386 m) need 1.08 m, while
the widest narrow aisle is 0.82 m. **"An unmapped obstacle in the narrow stratum" therefore means
an obstacle on the open floor of a narrow-aisled layout**, met on the way into or out of the
aisles, never within them. Results state this.

**Limitation, recorded as ruled.** The test population contains no route blockage that requires
replanning. A pallet left across an aisle the robot was sent down is a real warehouse case; it is
outside this study's scope, and global replanning on detection is the future work that addresses
it.
