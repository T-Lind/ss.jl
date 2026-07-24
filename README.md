# SatelliteSim.jl (`ss.jl`)

Mid-fidelity, extensible satellite mission simulation in pure Julia
(standard library only — no package dependencies). Two reference missions:

1. **LEO reentry** — an unpropelled pod returning from low Earth orbit to a
   Pacific splashdown off the US west coast (the original v0.1 mission).
2. **Circumlunar free return** (v0.2) — the whole flight: a three-stage
   rocket launches the same pod from a Cape-like site, a tuned gravity-turn
   ascent inserts into a 200 km parking orbit, the kick stage performs a
   finite trans-lunar-injection burn, the pod coasts around the Moon on a
   free-return trajectory (2000 km perilune, no burns after TLI), and comes
   straight home — 6.5 days pad to Pacific splashdown — to a ballistic
   10.6 km/s entry.

![free return](output/plots/moonshot_3d_cislunar.png)

## The circumlunar mission at a glance

```
launch (57.8 t, 3 stages)  ──►  200 km parking orbit, i = 28.5°
        tuned pitch program        insertion by closed-loop cutoff
                                        │  coast <1 rev
                                        ▼
                       TLI: kick stage, Δv ≈ 3.15 km/s finite burn
                                        │  3.3 d outbound
                                        ▼
                          lunar flyby, perilune 2000 km (free return)
                                        │  3.2 d coast home (first-pass entry)
                                        ▼
              vacuum perigee ~35 km  ──►  EI at 10.6 km/s, Mach 28
                                        │  4-DOF ballistic entry, 19 g
                                        ▼
                          drogue + main chutes, Pacific splashdown
```

Everything is designed by the code, not hand-tuned: the ascent pitch program
is closed by a 2×2 Newton shooting method, the TLI ignition time and Δv are
closed by a patched-conic-seeded Newton iteration on (perilune altitude,
return perigee altitude) with an outer corrector for the multi-day lunar
tide drift, and the whole chain — design included — runs in about a second.

![rotating frame](output/plots/moonshot_rotating.png)

## What the circumlunar chain models

**Launch** (`src/propulsion.jl`, `src/launch.jl`): 3-DOF + mass point
dynamics over the rotating WGS-84 Earth. Stages burn at constant vacuum mass
flow with nozzle-pressure thrust correction; drag uses a Mach-indexed
slender-body CD table on the USSA76 atmosphere. Guidance is the classic
sequence — vertical rise, pitch-over kick toward the launch azimuth, gravity
turn (thrust along the relative wind) through stage 1, then a linear-tangent
pitch law for the upper stages with cutoff at the target orbit energy.
`tune_ascent` closes (pitch0, pitch rate) on (insertion altitude, γ = 0) by
damped-Newton shooting — a stand-in for PEG-class guidance. Staging drops
dry mass, the fairing jettisons at 120 km, and unspent kick-stage propellant
is the TLI budget. Burnouts are the one boundary a fixed step gets visibly
wrong, so the final step of a burn is trimmed to land exactly on depletion:
a stage burns its propellant load and not a kilogram more. The default
`Sable` launcher (S1 kerolox 950 kN, S2 kerolox 95 kN, S3 storable 15 kN)
puts ~1.4 t through TLI from a 57.8 t liftoff.

**Strap-on boosters** (`BoosterSet`) burn in parallel with stage 1: thrust
and mass flow sum, the attached set adds its own frontal area to the drag,
the core can be held at a reduced throttle while the sides carry the stack,
and each set lights, burns out and separates on its own schedule. The pitch
kick is where parallel burn bites — it is not a constraint (the 2×2 above
already pins the insertion state) but it decides how much of the climb is
spent fighting gravity, and the right value moves with thrust-to-weight. A
stack with strap-ons lifts off so hard that the reference 8° kick lofts it,
and it reaches the target energy having *wasted* the extra impulse: two
boosters on the reference vehicle deliver **less** mass to orbit than none.
`tune_ascent(...; optimize_kick = true)` scans the kick, solves the 2×2
inside each candidate and keeps whichever puts the most mass in orbit —
turning that 1.2 t into 3.7 t. It costs a few seconds, so it is opt-in.

**Trans-lunar injection & free return** (`src/moon.jl`, `src/translunar.jl`):
the Moon is a circular ephemeris in the achieved parking-orbit plane (the
coplanar-launch-window assumption real missions buy with launch timing), and
cislunar flight integrates Earth point-mass + differential lunar gravity
with a step size scheduled by the local dynamical timescale of whichever
body dominates. The TLI burn is finite (prograde thrust on the actual kick
stage; gravity losses are physical, not modeled). `design_free_return`
seeds from a patched conic — transfer apogee past the lunar distance, Kepler
time-of-flight to the crossing, Moon lead angle — scans the encounter window
(1 min of ignition time sweeps the b-plane by ~27,000 km, so the impact zone
and its escape-side and free-return-side flanks are all a few minutes wide),
then Newton-iterates (ignition time, Δv) onto (perilune altitude, return
vacuum perigee), with an outer corrector that absorbs the few-hundred-km
perigee drift the Moon's tides add over the coast home. The corrector
requires entry on the **first** post-flyby perigee pass — the propagator
counts perigee passes that stay above the entry handoff, and any such
"phasing loop" solution (a return that misses entry, swings back out past
the lunar distance, and enters a revolution later) is steered back onto the
direct-return family.

**Entry**: the pod hands off at 140 km to the original 4-DOF reentry
simulation — same vehicle, same aero/heating models — now at 10.6 km/s
instead of 7.6. Tauber–Sutton radiative heating switches itself on above
9 km/s, peak loads reach ~18 g on this ballistic corridor, and the drogue +
main sequence flies unchanged.

**6-DOF attitude & RCS** (`rigidbody.jl`, `rcs.jl`, `entry6.jl`): full
quaternion entry with total-angle-of-attack aerodynamics from the same Mach
tables, Euler rotational dynamics on a diagonal inertia tensor, and RCS
hardware (thruster couples with torque authority, PWM duty cycles, real
propellant drawdown). `simulate_entry6` cross-validates against the 4-DOF
sim to a fraction of a percent — and exposes what the 4-DOF assumes away:
an uncontrolled pod arrives at entry interface **broadside** (the velocity
vector rotates >100° during the coast while the body stays inertially
fixed), and the statically stable shape genuinely self-rights from
α = 122°. The RCS wind-hold mode holds trim through the coast for about a
gram of propellant; damping 3°/s of tipoff costs ~9 grams. Long-coast
attitude budgets (deadband limit cycling, slews, settling) use the standard
closed-form results — which immediately re-sized the kick stage's thrusters:
the first cut (25 N, 20 ms pulses) would have emptied its tank limit-cycling
across a multi-week cruise.

**Execution dispersions & mid-course correction** (`translunar.jl`): the TLI
burn accepts magnitude and pointing errors, and they matter enormously — a
0.3% overburn alone moves perilune by +10,000 km and drops the return
perigee 6,300 km below the Earth's surface. `design_tcm` recovers the
mission with the classical two-stage scheme: a return-to-reference Newton
(re-join the nominal trajectory's position at the nominal perilune epoch —
nearly linear, converges from arbitrarily large errors) followed by a
terminal polish on (perilune, proxy perigee). `moonshot(tli_mag_err=...,
tli_point_err=...)` flies the dispersed cruise with the correction at
T+24 h and reports the TCM Δv, its kick-propellant cost, and the cruise RCS
budget. `scripts/run_tcm_mc.jl` runs the Monte Carlo: at σ = 0.2% / 0.25°,
the TCM budget is ~36 m/s mean / ~100 m/s p99 — p95 propellant 13.7 kg
against the 22.8 kg post-TLI kick margin, with 99% of samples reaching the
entry corridor.

Nominal circumlunar numbers (v0.2): TLI Δv 3151 m/s of a 3330 m/s budget,
perilune 2000 ± 0.1 km, return vacuum perigee 50.2 km (γ_EI ≈ −6.3°), entry
peak **6.4 g** / 191 W/cm², stagnation heat load 187 MJ/m², splashdown
6.6 days after liftoff at 4.5 m/s under main (3.3 d out, 3.2 d home — the
symmetric free-return figure-8).

These are reproducible, which they previously were not. The free-return
corrector used to accept any design landing within **3 km** of the perigee
target — wide enough that two distinct solutions both qualified, so the
search settled on either depending on numerical noise. Changing nothing but
the coast step size moved the reported perigee by 2 km and peak entry load by
0.6 g. The band is now 250 m (`PERIGEE_TOL`), and over an 8× range of step
sizes the flown result varies by 0.27 km of perigee and 0.08 g — the
remaining spread is just where inside the band the search stops. Perilune was
always solid: 40 m across the same range.

Plots: `output/plots/moonshot_*` — 3D ascent, Earth-Moon trajectory,
rotating-frame figure-8, 3D entry — plus the full analysis suite from
`scripts/make_mission_plots.py` (see Running). `scripts/make_viewer.py`
builds an interactive HTML viewer (`output/moonshot_viewer.html`) with
mission-time playback and an inertial/rotating frame toggle.

**Maneuver planning** (`maneuvers.jl`): the two-body transfer toolbox — a
universal-variables Lambert solver (validated against analytic ellipse
states to 1e-5), Hohmann transfers, plane changes, impulsive propellant
costs, and Clohessy-Wiltshire relative motion with the two-impulse
rendezvous solution. `scripts/run_rendezvous.jl` closes a 10 km approach
for 3.2 m/s and verifies the linearized design against a full nonlinear
propagation (113 m arrival miss — the 1% linearization error a terminal
prox-ops phase absorbs). These are the building blocks for faster lunar
transfers, orbital refueling, and intercepts.

**Missions as data** (`config.jl`, `missions/`): a TOML file fully defines a
mission — pod, targets, all stages, dispersions — and
`run_mission("missions/moonshot.toml")` designs and flies it. New vehicles
and mission variants are files, not code.

**Propellants, engines and stage sizing** (`engines.jl`): the flight model
only needs a stage's mass, thrust, Isp and nozzle exit area, and that is all
`Stage` carries into the integrator. Above it sits a design layer with real
propellant properties (kerolox, hydrolox, methalox, hypergolic, hydrazine,
solid — storage densities and flight mixture ratios), a catalogue of
representative engines (Merlin 1D and its vacuum variant, Rutherford,
Raptor 2, RS-25, RL10B-2, Vinci, AJ10), and mass-estimating relations. An
engine's nozzle exit area is *derived* from its sea-level/vacuum Isp split,
so the pressure correction the sim flies reproduces both quoted figures
exactly — a tested invariant, not a coincidence.

This is what makes propellant choice physical rather than cosmetic. Bulk
density spans more than a factor of three across the catalogue (hydrolox
343 kg/m³ against kerolox 1023), and tanks are sized from the real fuel and
oxidiser volumes — so swapping the reference upper stage from its kerolox
engine to an RL10B-2 grows that barrel from 5.8 m to 14.1 m and the whole
vehicle from 32.4 m to 40.8 m, in the panel viewer, the launch view, and the
exported STL alike. Stages carry their own **diameter** too, and where two
neighbours differ the lower one is capped with a **transition cone** — a
real interstage adapter, held at a constant shallow wall angle (17°) so its
length follows the size of the step, necking down to a narrower upper stage
or flaring out to a wider one. It sits above the tank rather than inside it,
so it lengthens the stack without eating capacity, and it belongs to the
lower stage's section: it departs at separation, as the real article does.
Stepping the reference vehicle 2.6/1.8/1.2 m buys a 1.31 m cone and a 0.98 m
cone; a uniform stack emits none and is untouched. The capsule and fairing
follow whichever stage they ride on. Because a barrel's length comes from
its own cross-section, widening the reference booster to 2.6 m packs the
same 42 t into 11.2 m instead of 20.2 m. Run it the other way — hold the
length and widen — and the capacity is what moves: 21.2 m at 2.6 m across
holds 89 t of kerolox rather than 42 t. Clustering is equally concrete: `engines = 9` multiplies
thrust and exit area by nine and hangs nine bells under the stage, packed to
fit. `sized_stage` estimates dry mass from tank volume, engine mass, thrust
structure and cryogenic insulation, plus one lumped `systems` term that
scales as `mprop^0.78` because mass fraction improves with size; it lands
within a few percent of a Falcon-9-class first stage and the reference
upper stage, and about 15% light on small dense boosters. Pass a measured
`dry_kg` and the estimate is skipped entirely.

**Aerodynamics from geometry** (`mesh.jl`, `panelaero.jl`, `geometry/`):
load an STL (or build one procedurally — `lathe_mesh`, `box_mesh`, and
`rocket_mesh`, which sizes a stacked launcher so each stage's barrel holds
its propellant and details it with a five-bell first-stage engine cluster,
recessed interstage collars hiding nested vacuum bells, cable raceways,
RCS pods and a payload adapter on the kick stage, a crew capsule under
the fairing, and any strap-on booster sets clustered around the first stage
— reporting each section's axial extent *and* triangle range so viewers can
detach pieces individually). The capsule comes from
`pod_mesh`: Apollo proportions (spherical-section ablator, 32.5° afterbody,
docking tunnel), built as a genuine pressure *shell* by `_shell_mesh`,
which revolves a contour, offsets it along its own surface normal and cuts
watertight, rimmed apertures — so the three glazed windows and the side
hatch are real holes through a real wall, and the cabin behind them (deck,
crew couches, display console, equipment racks, all scaled to what the
diameter can actually seat) is modelled and returned separately from the
hull and the panes. Every piece is a closed, consistently wound solid, so
you can compute exact polyhedral mass properties (volume, CG, inertia —
Eberly's method, validated to machine precision on primitives), and
generate hypersonic aero
tables with a modified-Newtonian panel method: CA/CN/Cm over (α, Mach) plus the pitch
damping derivative Cm_q from a rotating-panel sweep. The sphere reproduces
the analytic Newtonian drag to 0.2%; the committed capsule mesh flies the
full 6-DOF entry on mesh-derived aero within ~10% of the handbook-table
result (7.11 g vs 7.20 g peak). The simplified Starship mesh
(`geometry/starship.stl`: 9 m × 50 m body + four flaps) comes out as a
proper lifting body — L/D 1.2 at 20°, a stable passive belly-first trim at
35° set by its flap geometry, negative Cm_q — ready for the flap-control
and propulsive-landing work that a real Starship entry needs. Newtonian
aero is hypersonic-only (tables clamp below Mach ~4; pair with drogues
before transonic, as the capsule missions do).

## What the reentry sim models

**4 degrees of freedom** — 3 translational + 1 rotational (the body pitch
angle):

* Translation is integrated in Earth-centered-inertial Cartesian coordinates
  over a rotating WGS-84 Earth, so aerodynamic forces use the
  atmosphere-relative velocity and splashdown targeting sees Earth rotation.
* The 4th DOF is the body pitch angle, carried as angle-of-attack dynamics:
  `α̇ = q − γ̇`, `Iyy·q̇ = q̄·Sref·Lref·Cm(M, α, q̂)` with Mach-dependent
  static stability `Cm_α(M)` and pitch damping `Cm_q(M)`. The capsule
  genuinely oscillates about trim and damps as dynamic pressure builds —
  see `output/plots/nominal_attitude.png`.

**Entry interface at 120 km** (per the mission spec):

* Above EI — pure orbital mechanics: point-mass gravity + J2 oblateness.
* Below EI — full aerodynamic environment: US Standard Atmosphere 1976
  (analytic layers to 86 km, USSA76 tables to 500 km), Mach-dependent
  blunt-capsule coefficient tables (CD, CL_α, Cm_α, Cm_q from subsonic to
  Mach 30), Sutton–Graves convective + Tauber–Sutton radiative stagnation
  heating, integrated heat load and radiative-equilibrium wall temperature,
  and a drogue + main parachute sequence with finite canopy fill times.

**Events**: EI crossing and splashdown are located by bisection to sub-meter
altitude tolerance; parachute deploys trigger on Mach/altitude gates.

**Deorbit targeting**: `target_deorbit` tunes the deorbit ellipse's RAAN and
argument of perigee by fixed-point iteration on full trajectory simulations
until splashdown hits the target (converges in ~5–6 runs to a few km).

**Monte Carlo**: threaded dispersion analysis over vehicle mass (the "mass
above or below target" case), drag coefficient, pitch stiffness, trim angle
(CG offset), atmospheric density, deorbit-burn state errors, initial
attitude, and parachute drag areas — with footprint statistics (CEP, R95,
1σ covariance ellipse).

## Nominal results (v0.1 reference mission)

350 kg, 1.5 m diameter pod (β ≈ 130 kg/m²), deorbit ellipse 400 × 25 km,
i = 51.6°, targeted at 32.5°N 121.5°W:

| Quantity | Value |
|---|---|
| Entry interface state | 7.60 km/s, γ = −1.46°, Mach 20 |
| Downrange EI → splash | ~2 930 km |
| Peak deceleration | 7.1 g |
| Peak stagnation heating | 45 W/cm² at 63 km (T_wall ≈ 1750 K) |
| Stagnation heat load | 77 MJ/m² |
| Drogue → main → splashdown | 9 km → 3 km → 4.5 m/s |
| Nominal miss | 2.5 km |
| MC footprint (300 samples) | 1σ ellipse 363 × 12 km along-track, CEP 228 km |

The long thin footprint is the correct physics of a *shallow unguided
ballistic* entry: downrange is very sensitive to density/CD/mass at
γ_EI ≈ −1.5°. Steepen the entry (lower `periapsis_alt` in
`DeorbitElements`) to trade footprint size against g-load and heating, or
add a lift-modulation guidance law (see extension points) to shrink it by
orders of magnitude.

Plots: `output/plots/` — flight profile, aerothermal environment, pitch-DOF
behavior, ground track, MC footprint and statistics.

## Running

Julia ≥ 1.9. The core has **zero external dependencies**.

```bash
# tests (329 assertions: atmosphere vs USSA76 tables, vis-viva, J2, heating,
# orbit propagation, Tsiolkovsky, ephemeris, ascent-to-orbit, the full
# circumlunar chain — including a first-pass-return regression check —
# propellant/engine consistency and stage sizing, scalar targeting including
# infeasible-design retreat, parallel-burn strap-on boosters, RK4 order and
# the Jacobi constant of the cislunar coast, step-size independence of the
# flown mission, meshes/panel aero including interstage transition cones,
# capsule watertightness and cabin clearances, maneuvers, end-to-end reentry)
julia --project -e 'push!(LOAD_PATH, "src"); using SatelliteSim; include("test/runtests.jl")'

# targeted nominal trajectory -> output/*.csv
julia --project scripts/run_nominal.jl

# full circumlunar mission (design + fly) -> output/moonshot_*.csv
julia --project scripts/run_moonshot.jl

# TCM Monte Carlo over TLI execution errors -> output/tcm_montecarlo.csv
julia --project -t auto scripts/run_tcm_mc.jl 100 0.2 0.25

# missions from TOML specs
julia --project scripts/run_mission.jl missions/moonshot.toml

# regenerate the demo geometry (capsule + simplified Starship + Sable launcher)
julia --project scripts/make_meshes.jl

# Monte Carlo (threaded) -> output/montecarlo.csv + summary
julia --project -t auto scripts/run_montecarlo.jl 300

# plots
python3 scripts/make_plots.py          # reentry plots (matplotlib)
python3 scripts/make_3d.py             # circumlunar 3D/mission-plane plots
python3 scripts/make_mission_plots.py  # full mission analysis suite: broken-
                                       # time-axis overview, orbital energy,
                                       # ascent/entry profiles, ground track,
                                       # TCM Monte Carlo -> output/plots/
python3 scripts/make_viewer.py         # interactive HTML mission viewer
julia --project scripts/make_plots.jl  # requires Plots.jl installed

# mission-control panel: configure, run, and explore in the browser
julia --project -t auto scripts/panel.jl   # then open http://localhost:8137
```

### Mission-control panel

`scripts/panel.jl` serves a local cockpit (pure stdlib — a raw-`Sockets`
HTTP server, no dependencies): edit the mission targets and all three
stages' propellant/dry mass/thrust/Isp, hit **Run** (or pick a one-click
**preset** — heavy pod, low/high flyby, steep/shallow entry, inclined,
dispersed TLI), and get the full design + flight back in about a second —
stat tiles, the interactive 3D scene, ascent & entry profile charts, the
event timeline, and a run history for side-by-side comparison.

The 3D scene is true ECI geometry: the full pad-to-splashdown track
(ascent, parking orbit, TLI burn, outbound, return, entry — each its own
color), a textured globe spinning about the real pole with **launch-site
and splashdown markers riding the rotating surface**, correct
depth-occlusion of trajectory lines behind the Earth, mission-time
playback, an inertial/rotating frame toggle, and Earth/full zoom shortcuts.
The **stage count** is a field (2–5) and the whole launcher form is generated
from it — nothing in the page or the server counts stages itself, so the
geometry viewer, the sweep list and the solver all pick up a new stage the
moment you add one. Each stage has a **mixture**, an **engine count**, and an
optional **engine** from the catalogue; pick an engine and its thrust, Isp,
exit area and propellant take over, with dry mass estimated if you leave it
to. Each stage also has its own **diameter** and a choice of what drives its
size: give it a propellant mass and the tank length follows, or give it a
tank length and the propellant load follows from the volume — which is how
widening a stage turns into extra propellant rather than a shorter barrel.
Whichever you are not driving is greyed out and reported back to you. The dropdowns are filled from the server's own catalogue, so the page can
never offer something the simulator doesn't have. Bear in mind the reference
vehicle carries about 23 kg of propellant margin, so inserting a stage adds
enough mass to break the mission — which is what the solver is for.

Every stage reports its own **verdict** as you type — propellant, tank
length, volume, ideal Δv, thrust-to-weight at its own ignition, burn time,
and the adapter cone it needs to meet the stage above — with a summary line
for the whole stack: mass on the pad, lift-off T/W (flagged red below 1.15,
because that vehicle does not leave the ground), and the total ideal Δv it
has to spend. That is all arithmetic on numbers the geometry endpoint
already returns, and it turns "run it and see" into "look at it and see".

**Strap-on boosters** are a field too. Pick 2, 3, 4 or 6 and a set appears,
configured exactly like a stage — propellant, engine, mixture, diameter,
size-by-mass-or-length — plus the three things that only make sense in
parallel: the **core throttle** held while they burn (the Falcon-Heavy trick
of running the first stage down so it still has propellant when the sides
go), an **ignition delay** for an air-lit set, and a **separation delay**
that carries them as dead weight after burnout. The numbers describe one
booster; the set contributes them `count` times over. They fly as a real
parallel burn — summed thrust and mass flow, their own frontal area in the
drag while attached, their own separation events — and they show up in the
geometry, the panel viewer and the launch view as bodies beside the core
with their own nose cones, bells and exhaust plumes, dropping away at their
separation event while the core keeps burning.

A **solve** card locks every field but one and searches it until a mission
metric hits a target: *vary perilune target until perilune = 3000 km*, or
*vary pod mass until the kick stage's propellant margin = 0*. Each step flies
the whole chain, so this is a bracketing search (Illinois-modified regula
falsi, `solve.jl`) on a hard iteration budget rather than a scan — typically
five or six missions. Every evaluation is plotted, because with second-long
evaluations the path is most of what you learn. A configuration that fails
outright counts as past the feasible edge: the search retreats from it and
reports the interval it could actually use, so "a 500 kg pod flies no valid
mission" reads differently from "your range is too narrow". Running it is
instructive about this vehicle — most targets turn out to be unreachable,
and the reason is always the same thin margin. A **vehicle geometry** card
renders the launcher those choices imply — built from the very same
`LaunchVehicle` the mission flies, so the drawing and the trajectory cannot
disagree — and updates as you edit the configuration; it
**follows the mission clock**: scrub the timeline and stage 1, the fairing,
stage 2, and finally the spent kick stage drop away at their actual event
times, with an engine flame while a stage burns and the view recentering
on whatever is still flying (down to the bare pod on the return leg).

### Mission experience

The **🚀 launch view** chip (or `http://localhost:8137/launch`) opens a
cinematic WebGL rendering of the whole mission, pad to splashdown, flown
directly from the simulator's logs for the currently configured vehicle —
same `/api/run` data, nothing canned.

The timeline is **built from the run, not hardcoded**. Phases are emitted
only if the trajectory contains them (no TLI in the log ⇒ no TLI phase, no
translunar coast, no flyby), coasts are paced to compress to a roughly
fixed wall-clock length whichever trajectory you configured, and every
event the sim reported becomes a callout and a seek-bar marker — including
ones whose names are derived from your own stage and parachute names, so a
four-stage vehicle gets four separations. Burns and entry run in real
time; a warp ladder (×1 · ×2 · ×4 · ×10 … ×10k, or auto) and a
phase-segmented seek bar let you jump around. Seven cameras (keys `1`-`7`)
cover exterior, onboard and in-cabin views:

* **Ascent** — from T−15 the umbilical arms swing back and the pad lights
  under the real ignition ramp; the camera director cuts through pad,
  tracker, chase, and onboard views as the actual trajectory unfolds:
  gravity-turn pitch from the guidance solution, max-q vapor at the logged
  transonic time, stage separation and fairing halves tumbling away
  ballistically, Mach diamonds giving way to a vacuum-expanded plume.
* **Orbit → TLI → translunar** — the scene switches to true ECI
  coordinates: the kick stage + pod coast over a day/night Earth, relight
  for the TLI burn against the limb, and cruise out with the Moon growing
  from a disc (sim ephemeris) to a cratered sphere filling the frame at
  the 2 000 km far-side flyby.
* **Entry** — the pod hits the interface at ~10.6 km/s with a shock layer
  standing on the heat shield and an incandescent wake streaming behind
  it, driven by the logged heating rate; then drogue, gored
  orange-and-white main, and splashdown in the Pacific, with the final
  stat card. Exhaust and wake are seeded in the *vehicle's* frame: what you
  see is the relative recession, a few hundred m/s, not the 8-11 km/s the
  vehicle is doing. Left in the world frame a trail falls kilometres behind
  between frames and never becomes visible — and if the velocity it is
  measured against is even slightly overstated, the plume overtakes the
  rocket, which is why the cruise state uses a 1 s velocity baseline rather
  than a 40 s one that would average across the TLI burn's acceleration.
  The plume also runs on its own near-real-time clock, so it keeps burning
  when you wind the time warp up instead of blinking out.
* **Inside the capsule** — `cabin` (key `5`) puts the eye on the couch;
  `window` (key `6`) puts it up against a viewport looking out. Because the
  hull is a real shell with real apertures, the renderer simply drops the
  window panes and you see straight out through the openings: display
  console overhead, equipment racks along the wall, the ocean or the stars
  through the glass. In space the view picks whichever of the three
  windows currently faces the Moon or Earth, and dragging turns your head
  rather than orbiting the capsule. Riding inside, the camera moves *with*
  the hull — only a few millimetres of vibration remain, because a crew
  member is shaken along with the vehicle rather than watching it shake.
  The director cuts inside when the fairing splits and daylight first
  reaches the cabin, and again through peak heating.

The vehicle wears a **livery**: a decal sheet painted procedurally in
(station, roll) space and wrapped cylindrically over the skin — roll-pattern
quadrants on the booster, an interstage trim band, stage numerals, an
insignia roundel and ensign, split-line markings down the fairing, and the
vehicle name. Baked vertex colours could never carry lettering at this
triangle count, so the markings live in a generated texture and the mesh
itself stays clean. The insignia is an original mark rather than any real
agency's; it is all drawn in one function (`buildLiveryTex`) if you want a
different scheme.

Earth and Moon are **exact ray-traced spheres** rendered in a single
fullscreen pass with camera-relative centers — the limb, horizon dip, and
atmosphere shell are geometrically correct from the pad, from orbit, and
from lunar distance (no flat terrain disc, no far-plane clipping). The
surface combines the panel's real land-mask texture with procedural
detail, cloud fields, polar ice, night-side shading, and Earth's actual
rotation over the 6.5-day cruise. HUD readouts sample the same logs the
analysis plots use; event callouts, procedural audio (distance-delayed
rumble + crackle, vacuum-silent, wind on entry), and keyboard/manual
cameras round it out. Everything is generated in-page — vehicle mesh from
`rocket_mesh` via the API, lattice tower, pad, terrain, clouds, plume,
parachutes, and sound are all procedural; no external assets, no
libraries.

A **parameter sweep** tab varies any knob across a range (threaded; ~50
missions/min) and plots the outcome curve — e.g. sweeping pod mass shows
the TLI propellant margin hitting zero just above 400 kg, which is the
actual payload limit of the default launcher. Runs that fly but miss the
requested perilune/perigee (e.g. a prop-starved TLI) are flagged **off
target** rather than silently plotted.

Programmatic use:

```julia
push!(LOAD_PATH, "src"); using SatelliteSim

scn, el, _ = west_coast_scenario()        # targeted reference mission
res = simulate(scn)                        # SimResult: log, events, metrics
print_summary(res, scn)

samples = run_montecarlo(scn, Dispersions(mass_frac = 0.05); n = 500)
mc_statistics(samples)

ms = moonshot()                            # design + fly the lunar mission
print_moonshot_summary(ms)
ms.ascent.elements                         # parking-orbit insertion
ms.cislunar.perilune_alt                   # 2000 km
ms.entry.peak_gload                        # ~18 g

# pieces are usable on their own:
lv = default_moon_rocket()
guid, asc = tune_ascent(lv, AscentGuidance(h_target = 250e3))
eph = coplanar_moon(asc.r, asc.v)
```

## Architecture

```
src/
  SatelliteSim.jl   module root & exports
  constants.jl      physical/planetary constants (Earth + Moon)
  vec3.jl           allocation-free 3-vector helpers
  atmosphere.jl     AbstractAtmosphere: USSA76, ScaledAtmosphere
  gravity.jl        AbstractGravity: PointMass, J2, ThirdBody, Composite
  frames.jl         ECI/ECEF/WGS-84 geodetic, Kepler elements <-> states, ENU
  aerodynamics.jl   Mach-interpolated capsule coefficient tables
  vehicle.jl        Vehicle + Parachute definitions
  heating.jl        Sutton-Graves, Tauber-Sutton, wall temperature
  dynamics.jl       4-DOF equations of motion + phase logic (EI at 120 km)
  integrator.jl     RK4 (phase-scheduled step sizes)
  rigidbody.jl      quaternions + Euler rotational dynamics
  rcs.jl            RCS thrusters, PWM control laws, analytic coast budgets
  propulsion.jl     Stage / LaunchVehicle, pressure-corrected thrust
  moon.jl           circular lunar ephemeris (mission-plane construction)
  launch.jl         3-DOF+mass ascent, staging, guidance, Newton tuning
  translunar.jl     cislunar propagation, TLI burn + execution errors,
                    free-return design, two-stage TCM
  simulation.jl     entry driver: events, bisection, logging
  entry6.jl         full 6-DOF entry (quaternion attitude, total-AoA aero,
                    RCS wind-hold / rate damping)
  scenarios.jl      deorbit design + splashdown targeting
  mission.jl        the full launch->Moon->splashdown chain (moonshot)
  maneuvers.jl      Lambert, Hohmann, plane change, Clohessy-Wiltshire
  mesh.jl           STL I/O, polyhedral mass properties, mesh builders
  panelaero.jl      modified-Newtonian panel aero (CA/CN/Cm + Cm_q) from meshes
  config.jl         missions/vehicles as TOML specs (load_mission/run_mission)
  montecarlo.jl     dispersions, threaded MC, footprint statistics
  output.jl         CSV writers, console summaries
missions/           TOML mission specs (moonshot.toml = the reference)
geometry/           demo STL meshes (capsule, simplified Starship)
```

### Extension points (built in on purpose)

| Goal | How |
|---|---|
| Different rockets | build a `LaunchVehicle` from `Stage`s; `tune_ascent` re-closes the pitch program for any stack that can reach orbit |
| Faster (Apollo-class) returns | add a third design parameter (e.g. burn direction or a mid-course trim) and target flight time alongside perilune/perigee in `design_free_return` |
| Real lunar ephemeris | any callable `t -> V3` plugs into `ThirdBodyGravity` and `fly_cislunar`; swap `CircularMoonEphemeris` for DE-series data to get eccentricity + plane evolution |
| Out-of-plane launch windows | replace `coplanar_moon` with a real ephemeris + a plane-targeting layer on `AscentGuidance.azimuth` and launch time |
| Sun perturbation | append another `ThirdBodyGravity` to the cislunar acceleration |
| Better atmosphere | subtype `AbstractAtmosphere` (NRLMSISE-00, GRAM dispersions, Mars…) |
| Richer aero | subtype `AbstractAeroDatabase` with full (Mach, α) CN/CA/Cm maps |
| Entry guidance | wrap `simulate` — `bank` and trim α (`cl_trim`) are the control channels a bank-angle guidance law would command; a lifting entry shrinks the 18 g lunar-return load dramatically |
| 6-DOF | the pitch channel is isolated in `dynamics.jl`; adding roll/yaw states extends the same pattern |
| Higher-order integration | swap `rk4_step!` behind the same signature |
| Mission Monte Carlo | disperse `Stage` performance, TLI execution errors, and ephemeris phase through `moonshot` the same way `run_montecarlo` disperses entry |

### The entry corridor, and why the pod is lifting

An early version of this flew a **ballistic** capsule down a 35 km-perigee
return and peaked at **18 g**, holding above 15 g for 26 seconds. That is not
a simulation artefact — it is the correct answer for that flight mode, and
close to what Zond 5 actually pulled on the first ballistic circumlunar
return. It is also why nobody flies crew that way.

Two levers fix it, and the sim shows both:

| return perigee | L/D | γ_EI | peak g | > 6 g | heat load |
|---|---|---|---|---|---|
| 35 km | 0 (ballistic) | −6.87° | 18.0 | 68 s | 104 MJ/m² |
| 35 km | 0.3 | −6.87° | 9.4 | 30 s | 161 MJ/m² |
| **50 km** | **0.3** | **−6.25°** | **6.4** | **12.5 s** | **187 MJ/m²** |
| 65 km | 0.3 | −5.56° | 4.5 | 0 s | 247 MJ/m² |
| 80 km | 0.3 | −4.77° | — | — | *skips out, never returns* |

The default is the third row: `hp_return = 50 km` with `cl_trim_hyp = 0.45`
(L/D ≈ 0.3, the Apollo figure) flown lift-up. That lands at 6.4 g and
γ_EI = −6.3°, essentially the Apollo entry point.

The corridor is genuinely narrow, and both walls are real: steepen it and the
loads climb fast, shallow it past ~65 km and the vehicle skips back out and
never comes home (the 80 km case terminates on timeout, not splashdown).
The trade for the low g is **integrated heating, which nearly doubles** —
a lifting entry soaks for longer even though its peak heat *rate* is lower,
and it is the integral that sizes the ablator.

**Bank modulation** (`gload_bank`) is available and is *not* a way to reduce
peak load — inside the corridor, full lift-up is already the minimum-g
solution and modulating costs 1–2 g. What a roll law buys is the shallow
wall:

| return perigee | fixed lift-up | modulated (g=6) |
|---|---|---|
| 50 km | 6.4 g | 8.4 g |
| 65 km | 4.5 g | 5.5 g |
| **80 km** | **skips out — lost** | **4.5 g, home** |

It converts a mission loss into a survivable entry, widening the usable
corridor by roughly 15 km. That is why the default stays fixed lift-up (the
nominal perigee is held to 250 m) and the law is there for when the corridor
is uncertain — a dispersed TLI, a missed correction, an off-nominal return.

**Roll control is simulated, not assumed.** The 4-DOF model holds the
commanded bank by construction; a real capsule only does that with RCS. The
6-DOF model flying `rcs_mode = :bank_hold` now reproduces the 4-DOF result to
0.01 g and prices it at **0.26 kg** of propellant. Without roll control the
same vehicle lets its lift vector tumble and pulls 13 g instead of 6.4 — so
the low number is genuinely a *guided* entry number, and now the guidance is
in the loop rather than in the assumptions.

Getting there exposed a modelling inconsistency worth recording. Trim lift
and trim angle of attack are the same physical fact — an offset centre of
gravity produces both — but the aero database let you set one without the
other. With `cl_trim_hyp = 0.45` and `alpha_trim = 0`, the 4-DOF looked fine
(it constructs the lift direction from `bank`), while the 6-DOF flew at
**0.0° AoA**: the pitching moment restored toward zero, the off-wind body
component that carries the lift collapsed, and its direction became numerical
noise. Roll control burned 2.7 kg chasing a vector that was not there. The
two are now tied in `default_reentry_pod`, and the models agree.

Reaction wheels are the wrong device for this and the numbers are not close.
At peak dynamic pressure the aero restoring torque at 5° off trim is
**182 N·m**; a large reaction wheel (0.5 N·m, 20 N·m·s) has 0.3% of that
authority and saturates in **0.11 s**. RCS at 26 N·m is also under the aero
torque — and does not need to match it, because the capsule is
aerodynamically stable in pitch and yaw and self-trims. Roll is the only axis
with no restoring moment, so roll is the only axis worth spending propellant
on. For the cruise, RCS spends 1.04 kg over 6.5 days against an 11 kg margin,
so wheels would save about a kilogram while costing more than that in mass,
plus RCS for desaturation anyway.

### Verification

The integrator and the invariants are checked, not assumed:

* **RK4 is 4th order in practice.** A closed two-body orbit must return to
  where it started; the closure error falls 2.0 m → 26 µm across five step
  halvings, observed order 4.15 → 4.03.
* **The Jacobi constant holds on the real trajectory.** Because the Moon is a
  circular coplanar ephemeris, the cislunar coast *is* the circular restricted
  three-body problem, so C_J is a true invariant and any drift is integration
  error on the production dynamics — not a toy problem. Over the full 6.5-day
  flight including the flyby it drifts **3.5×10⁻⁵ relative**.
* **The answer does not depend on the step size.** Halving the coast step
  moves perilune by under 200 m and peak entry load by under 0.15 g.

All three are regression tests, so they run in CI rather than living in a
notebook somewhere. The numerical knobs that make such a study possible —
`cis_eta` (coast step as a fraction of the local orbital period) and
`perigee_tol` — are parameters of `moonshot`, `fly_cislunar` and
`design_free_return`, because a simulator you cannot run a convergence study
on is a simulator you cannot check.

### Performance

The whole design-and-fly chain is about **0.45 s**, split roughly evenly
between the ascent shooting solve and the free-return design. Every inner
loop is allocation-free after warmup — `_cis_accel`, `_cis_step`,
`dynamics!`, `_ascent_deriv!`, `atmosphere_state` and `gravity_accel` all
measure 0 bytes per call — so the cost is arithmetic, not garbage.

What is expensive is the search layers on top, and those parallelise: the
optional pitch-kick scan runs its candidates across threads (2.94 s → 1.12 s
on 16 threads), and the panel's parameter sweep already did. Run with
`-t auto`.

The launch view holds **~1 ms per frame** at 1280×800 with 690 live
particles, so it is nowhere near the 16.7 ms budget. Two things were still
worth fixing: attribute locations were being re-queried from the driver every
frame in four draw paths (they are fixed at link time, and are now resolved
alongside the uniforms), and the particle packer allocated a transformed
position per particle plus two partition arrays per frame — ~600 objects a
frame of pure GC churn, now packed straight into the GL staging buffer.

### Fidelity notes & current limits

* The pitch DOF uses the planar (pitch-plane) formulation coupled to the 3D
  translational state via γ̇ — valid while heading changes slowly, which
  holds for entry; a full 6-DOF quaternion model is the upgrade path.
* Above ~86 km, "Mach" and continuum aero coefficients are bookkeeping
  conveniences (free-molecular regime); forces there are negligible anyway.
* Radiative heating is ~0 below 9 km/s (LEO return) by construction of the
  Tauber–Sutton correlation; it activates automatically for faster entries
  (and does, at 10.6 km/s on the lunar return).
* Earth rotation angle starts at 0 by convention; longitudes are consistent
  internally (targeting absorbs the choice).
* The LEO-reentry scenario starts on the post-deorbit-burn coast ellipse;
  the circumlunar mission models all burns explicitly.
* The lunar model is a circular, coplanar ephemeris: right for designing and
  flying the free-return class, but real launch windows, the Moon's 5.1°
  plane and ±21,000 km eccentricity, and solar perturbation belong to the
  ephemeris upgrade (see extension points).
* The designed free return is the near-minimum-energy family (3.3 d out,
  3.2 d home, symmetric about the flyby). Apollo flew a faster,
  higher-energy family — reachable here by adding flight time as a third
  design target.
* Splashdown of the lunar return lands wherever the geometry says; targeting
  a specific site couples TLI epoch to Earth rotation and is a
  straightforward outer loop that is not yet closed.
* Ascent steering is a *command*: the thrust direction follows the guidance
  law with no rate limit and no attitude lag, so the gravity turn holds
  α ≈ 0 by construction and max-q structural loads are trivially zero. Rate-
  limiting the command and logging q·α is the honest upgrade.
* Stages burn at fixed throttle. `Engine` already carries a `throttle_min`
  that nothing reads yet; a thrust-vs-time curve is what solid strap-ons
  want, so `BoosterSet` currently models solids only as constant-thrust.
* Booster drag sums each strap-on's full frontal area onto the core's, with
  no shielding between neighbours and no change to the CD table shape — it
  errs high, and only while they are attached.
