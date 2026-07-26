# SatelliteSim.jl (`ss.jl`)

Mid-fidelity, extensible satellite mission simulation in pure Julia
(standard library only — no package dependencies). Three reference missions:

1. **LEO reentry** — an unpropelled pod returning from low Earth orbit to a
   Pacific splashdown off the US west coast (the original v0.1 mission).
2. **Circumlunar free return** (v0.2) — the whole flight: a three-stage
   rocket launches the same pod from a Cape-like site, a tuned gravity-turn
   ascent inserts into a 200 km parking orbit, the kick stage performs a
   finite trans-lunar-injection burn, the pod coasts around the Moon on a
   free-return trajectory (2000 km perilune, no burns after TLI), and comes
   straight home — 6.5 days pad to Pacific splashdown — to a
   10.6 km/s lifting entry.
3. **Lunar landing** (v0.3) — the same launch and the same free return, flown
   to a 100 km perilune and then *stopped there*: insertion into lunar orbit,
   a descent-orbit burn, and a guided powered descent to touchdown on the
   surface, 3 days after lift-off. Arrival is on a free return for Apollo's
   reason — a failed insertion is a trip home rather than a lunar impact.

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
              vacuum perigee 50 km  ──►  EI at 10.6 km/s, Mach 28
                                        │  lifting entry (L/D 0.3), 6.5 g
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

`scripts/make_analysis_plots.py` asks the other question. The flight suite
shows what the vehicle did; this one shows why. **`ascent_budget.png`** is a
Δv ledger: ideal Δv against the speed actually gained, with the gravity loss
integrated as ∫g·sinγ dt and everything else — drag, and steering the thrust
off the velocity vector — falling out as the residual. On the reference
vehicle that is 8975 m/s of propellant buying 7367 m/s of orbital speed, with
720 m/s to gravity and 888 m/s to drag and steering; the residual's time
history tracks dynamic pressure through max-q and then flattens onto the
linear-tangent upper stage's steering loss, which is where most of it goes.
**`montecarlo_sensitivity.png`** replaces one-input-at-a-time scatter with
standardized regression coefficients across every dispersed input and every
flight outcome at once, so peak load reading −0.85 on density and −0.41 on
C_D is directly comparable, alongside the R² that says how much of each
outcome the dispersions explain at all — and a noise floor drawn at 2/√n,
because with a few dozen samples most of the small coefficients are nothing.

**Launch windows and the mission epoch** (`launch.jl`, `mission.jl`):
`tune_ascent` closes the pitch program on (insertion altitude, γ = 0), which
pins the orbit's size and shape but says nothing about where its *plane* sits
in inertial space. The plane is decided by when you launch — the site is
carried around by the Earth and the vehicle inherits wherever it happens to
be. Every mission here used to simply accept whatever RAAN it got, because
`moonshot` never passed a launch epoch and so every flight implicitly lifted
off at Greenwich hour angle 0. That is fine for one flight designed alone, and
untenable the moment a second vehicle has to reach the first.

`moonshot` now takes `theta_g0`, threaded through the ascent, the cislunar
coast and the entry scenario alike (every simulator already accepted it), so a
flight lands in a *definite* inertial plane and two flights can share one
clock. `launch_window(guid, inc, raan)` supplies the epoch. A direct ascent is
in the target plane from liftoff, so the site's inertial position must lie in
it — `u·ĥ = 0` — which reduces to

```
sin(raan − α) = −tan(φ)·cot(inc),    α = lon + θ_g0 + ω⊕·t
```

for the site's geocentric latitude φ: two opportunities per sidereal day, the
ascending and descending node passes, the second flying the supplementary
(southerly) azimuth. When `|tan φ · cot inc| > 1` there are none, which is the
familiar rule that a site cannot reach an inclination below its own latitude —
here it is the arcsine running out of domain rather than a check bolted on.

The two latitudes in that paragraph are deliberately different. The epoch uses
the **geocentric** latitude, because the condition is about where the site's
position vector actually points and geodetic would misplace it — by 0.161° at
the Cape, and by up to 0.192° near 45°, which is ~21 km of position. The
azimuth uses the **geodetic** latitude, because that is what `launch_azimuth`
and the guidance already fly, and one imperfect convention beats two that
disagree.

Which leaves the honest limit. The epoch fixes the RAAN; the *azimuth* fixes
the inclination, and `launch_azimuth` is the classic non-rotating formula, so
the inclination it delivers is not the one asked for. That is nearly free at
the reference mission's almost-due-east heading — 28.40° achieved for a 28.5°
target — and expensive away from it: commanding 44.98° for a 51.6° orbit gets
**46.8°**, because at that heading the site's own 408 m/s of eastward motion
lies across the flight path rather than along it. Correcting it would move
every committed number in this repo, so it is measured and left alone.

The window is therefore solved for the inclination the vehicle *will* achieve,
which costs one extra flight to measure (the achieved inclination depends on
the azimuth, not on the epoch) and is the same design-then-correct shape as the
free-return corrector. Done that way the suite solves a window, flies the real
ascent at that epoch, and closes the **achieved** RAAN to under 2° on both the
ascending and descending opportunities — the remainder being the eight minutes
of ascent during which the site keeps turning under a plane matched at liftoff.

**Maneuver planning** (`maneuvers.jl`): the two-body transfer toolbox — a
universal-variables Lambert solver (validated against analytic ellipse
states to 1e-5), Hohmann transfers, plane changes, impulsive propellant
costs, and Clohessy-Wiltshire relative motion with the two-impulse
rendezvous solution. `scripts/run_rendezvous.jl` closes a 10 km approach
for 3.2 m/s and verifies the linearized design against a full nonlinear
propagation (113 m arrival miss — the 1% linearization error a terminal
prox-ops phase absorbs). These are the building blocks for faster lunar
transfers, orbital refueling, and intercepts.

The run logs both trajectories to `output/rendezvous.csv` in the target's
*instantaneous* RIC frame — the frame the CW solution is written in, so the
gap between the design and the flown arc on those axes IS the linearization
error, where in ECI it would be buried inside the 7.7 km/s both spacecraft
share — plus `output/rendezvous_tof.csv`, the same geometry re-solved for
every time of flight. Plotted by `make_analysis_plots.py`, that sweep is the
more instructive half: cost falls steeply with transfer time, but the
two-impulse solution has a pole wherever n·t is a multiple of π, because the
transfer matrix loses rank there and the burn that has to cover the offset in
the time remaining goes to infinity with it. Half an orbit is a choice made
between two poles, not a natural constant — the cheapest transfer in that
first basin is 1.99 m/s at 60 minutes, against 3.20 m/s at the 46-minute
reference.

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

## The lunar landing

The flyby mission treats perilune as a place to pass through. The landing
mission stops there, and everything after it is Apollo's sequence for
Apollo's reasons (`src/landing.jl`):

```
free return, perilune 100 km  ──►  LOI: 925 m/s retrograde, circular lunar orbit
                                        │  n revolutions of coast (this is what
                                        ▼   moves the landing site)
                       DOI: 19 m/s retrograde, periapsis to 15 km
                                        │  half a revolution
                                        ▼
        powered descent ignition ──►  braking: full thrust, linear pitch program
                                        │  1573 m/s, 250 s, shot onto high gate
                                        ▼
              high gate: 2 km, 45 m/s down, 150 m/s forward
                                        │  hazard scan → aim point moves 608 m
                                        │  terminal: throttled, 326 m/s
                                        ▼
                    touchdown at 1.29 m/s on 0.1° ground, 1420 kg and
                    9 minutes of hover left
```

**Why insertion happens at perilune.** At closest approach the relative
velocity is exactly perpendicular to the relative position — that is what
closest approach *means* — so the cheapest circularisation is purely
retrograde and its size is just the speed excess over circular. Nothing is
targeted: the altitude of the resulting orbit is the perilune the trans-lunar
design already flew to, which is why the panel's "lunar orbit" field is the
free return's perilune target.

**The braking phase is a shooting problem, and the gate is a velocity.** Full
thrust, thrust elevation following `theta(t) = theta0 + theta_dot * t`, and a
2x2 damped Newton on `(theta0, theta_dot)` against altitude and sink rate at
high gate — the same machinery `tune_ascent` uses for the climb out of the
atmosphere, because it is the same problem upside down. The phase *ends* when
forward speed falls through 150 m/s rather than at a chosen altitude: braking
is nearly all horizontal, so altitude is the free variable the pitch program
controls, and fixing it instead would pin the one thing the guidance has
authority over while leaving the speed — the thing that has to be gone — as
whatever fell out. The residual surface has a fold in it (programs that point
too high never come down; programs that point too low reach the ground still
moving at hundreds of metres per second), so a coarse grid seeds the Newton
and a local refinement restarts it if the first pass lands on the wrong side.

**The terminal phase is closed-loop.** Below high gate the guidance holds a
commanded sink rate that tapers as `v = -(0.8 + 0.85*sqrt(h))` and nulls the
along-track drift on a 18-second time constant. Two details decide whether it
lands or craters:

* **Feed-forward on the profile.** The commanded sink rate is a function of
  altitude, so it moves as the vehicle descends; a pure proportional law lags
  it by `tau * dv/dt`, which near the ground — where the profile steepens as
  `1/sqrt(h)` — is metres per second of extra sink exactly where it hurts.
  Differentiating the profile along the trajectory and commanding that
  outright took touchdown from **7.9 m/s to 1.2 m/s**.
* **The vertical channel is served first.** Thrust is finite and the two
  channels are not equally important: arriving with a metre per second of
  drift is a bad landing, arriving with an unchecked sink rate is a crater.
  The lateral command gets whatever acceleration is left over after gravity
  and the sink-rate demand are paid.

Touchdown limits are the lander's, not the trajectory's: over 3 m/s of sink
or 1.5 m/s of drift is reported as a **crash**, which is the whole point of
flying the last kilometre rather than assuming it.

The default `Lander` is Apollo-LM-class — 12.5 t wet, 9 t of propellant, one
45 kN engine throttleable to 10% — and it needs about 2.8 km/s for insertion,
descent-orbit initiation and the descent itself. That is why the landing
mission ships with a Starship-class launcher: the reference Sable tops out
around 400 kg through TLI, and a lander is thirty times that. The throttle is
not a detail either — a lander arrives light, and at touchdown mass a
fixed-thrust engine is pushing five lunar g upward. A `throttle_min = 1.0`
lander does not land, and the sim says so.

**Frames.** Everything from perilune on is integrated in a Moon-centred frame
that falls freely with the Moon. That is not a convenience: the residual
Earth term in that frame is the *tidal* difference, `2*mu_E*r/d^3`, which at a
100 km lunar orbit is 2.5e-5 m/s² — four orders below the modelling error in
a point-mass Moon. The landing site is reported in the tidally-locked
Moon-fixed frame, longitude zero at the sub-Earth meridian, so near side and
far side are exact; the latitude is relative to the ephemeris plane, which
with a coplanar circular Moon is the mission plane rather than the true lunar
equator.

### How much Moon to land on

A descent onto a smooth sphere, under a point mass, flown by a vehicle that
reads the integrator's own state vector, lands every time — and tells you
about the guidance law rather than about the Moon. `moonlanding` takes four
switches, all off by default so every figure predating them still reproduces,
and [`apollo_landing`](src/landing.jl) turns on all four:

| switch | what it adds | what it costs |
|---|---|---|
| `terrain` | procedural ground, ±3 km relief, craters 40 km → 50 m (`src/terrain.jl`) | altitude means height above *ground*, and the ground is nowhere near the sphere |
| `field` | lunar J2 and five near-side mascons (`src/moon.jl`) | the parking orbit stops being the ellipse the burn put it in |
| `nav` | orbit-determination error, an onboard point-mass model, landing radar (`src/landingnav.jl`) | the vehicle no longer knows where it is |
| `hazard` | scan the reachable footprint at high gate and redesignate | the aim point is chosen, not inherited |

Switch them on one at a time and the same mission fails a different way each
time. This is the whole argument for having them:

| configuration | outcome |
|---|---|
| sphere, point mass, perfect nav | lands, 1.2 m/s |
| + terrain | **tips over** — 22° ground |
| + mascons | **tips over** — 15° ground |
| + nav, no radar, no site survey | **crashes at 25 m/s** into a 1.4 km plateau it thought was sea level |
| + landing radar | altitude right to 12 m — and still crashes: the braking phase was aimed at the sphere, so high gate arrived 600 m over ground instead of 2000 |
| + site survey | lands at 1.2 m/s — on a 15° slope |
| + hazard avoidance | **lands at 1.1 m/s on 0.2° ground**, having moved its aim point 600 m |

Three things had to be right for that last row, and each was wrong first:
the terminal phase must not restart its own clock (the radar never updated
and terrain was looked up half a kilometre from the vehicle); the guidance
must null velocity **relative to the ground**, which moves at 4.6 m/s, or
every landing arrives sliding sideways at four times the tipping limit and
every hover drifts 400 m off the chosen site; and the chosen site must be
expressed in the navigation frame, not in absolute coordinates, because a
sensor sees real terrain but has to be flown to on an estimate.

```julia
ls = apollo_landing(lander = default_lander(), orbiter = Orbiter(),
                    lv = starship_expendable(payload = 22_500.0),
                    kick_angle = deg2rad_(5.0))
print_landing_summary(ls)
ls.descent.v_vertical      # 1.29 m/s
ls.descent.slope           # 0.1 deg of ground to stand on
ls.descent.redesignated    # 608 m to get it
ls.descent.hover_s         # 543 s of hover left at touchdown mass
```

## The trip home

Leaving the Moon from the surface costs about 1.85 km/s to orbit and another
0.9 to leave lunar orbit for Earth. Carry all of it on the thing you land and
you land something enormous; leave most of it in orbit and rendezvous with it
afterwards and you land something small. That is the whole argument for
lunar-orbit rendezvous, and `src/lunarreturn.jl` is it, priced
(`moonlanding(orbiter = Orbiter())` puts something up there to come back to):

```
surface, 21.6 h stay  ──►  ascent: 1791 m/s, 243 s, shot on (pitch, pitch rate)
                                        │   to a 15 × 85 km orbit, level
                                        ▼
              rendezvous: 23 m/s over 55 min, Lambert-targeted
                                        │   lift-off time is a search variable
                                        ▼   too — you cannot leave whenever
        docking, 244 kg unused           you like
                                        │
                                        ▼
              TEI: 816 m/s prograde, swept for the ignition point and
              closed by secant on the perigee that actually arrives
                                        │   4.7 days home
                                        ▼
        entry interface, γ = −6.8°  ──►  splashdown at 8.84 days, 5.8 g,
                                         232 W/cm², 180 MJ/m²
```

Ten metres per second at the Moon is thousands of kilometres of perigee at the
Earth, which is why the cheap model — read the osculating elements once you
are 80,000 km clear — picks the ignition point and nothing else. The magnitude
is closed with the full coast in the loop.

```julia
rr = moonreturn(ls)                 # ls must have left an orbiter behind
print_return_summary(rr, ls)
rr.dv_tei                           # 816 m/s
rr.entry.peak_gload                 # 5.8 g
```

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
# tests (450 assertions: atmosphere vs USSA76 tables, vis-viva, J2, heating,
# orbit propagation, Tsiolkovsky, ephemeris, ascent-to-orbit, the full
# circumlunar chain — including a first-pass-return regression check —
# propellant/engine consistency and stage sizing, scalar targeting including
# infeasible-design retreat, parallel-burn strap-on boosters, the lunar
# landing chain (insertion and descent-orbit burns against closed-form
# two-body values, a powered descent that touches down inside the lander's
# limits, and a fixed-thrust lander that cannot), procedural terrain
# (continuity, statistics, hazard scoring, Moon-fixed round trips), the lunar
# gravity field (mascon pairs adding no net mass, anomalies the right size at
# orbital altitude, a low orbit that visibly wanders), landing radar driving
# navigation onto truth, the layer-by-layer descent story from smooth sphere
# to hazard avoidance, the return trip end to end, RK4 order and
# the Jacobi constant of the cislunar coast, step-size independence of the
# flown mission, meshes/panel aero including interstage transition cones,
# capsule watertightness and cabin clearances, maneuvers, end-to-end reentry)
julia --project -e 'push!(LOAD_PATH, "src"); using SatelliteSim; include("test/runtests.jl")'

# targeted nominal trajectory -> output/*.csv
julia --project scripts/run_nominal.jl

# full circumlunar mission (design + fly) -> output/moonshot_*.csv
julia --project scripts/run_moonshot.jl

# lunar landing and the trip home -> output/landing_*.csv
julia --project -t auto scripts/run_landing.jl

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
python3 scripts/make_analysis_plots.py # "why" rather than "what": the ascent
                                       # Δv ledger, Monte Carlo sensitivity,
                                       # and the rendezvous figures
python3 scripts/make_viewer.py         # interactive HTML mission viewer
julia --project scripts/make_plots.jl  # requires Plots.jl installed

# mission-control panel: configure, run, and explore in the browser
julia --project -t auto scripts/panel.jl   # then open http://localhost:8137
```

### Mission-control panel

`scripts/panel.jl` serves a local cockpit (pure stdlib — a raw-`Sockets`
HTTP server, no dependencies): edit the mission targets and all three
stages' propellant/dry mass/thrust/Isp, hit **Run** (or pick a one-click
**preset**), and get the full design + flight back in about a second —
stat tiles, the interactive 3D scene, ascent & entry profile charts, the
event timeline, and a run history for side-by-side comparison.

The panel logs one line per request — method, path, status, duration, bytes —
and answers `GET /api/health` with its uptime, Julia version and thread count.
Both exist because an intermittent browser-side "Failed to fetch" is otherwise
invisible from the server: it leaves no trace unless the server writes one.
It listens on IPv4 and IPv6 loopback, so `localhost` resolves either way
without a failed connection attempt first.

A switch at the top picks the **mission**: the free-return flyby, or the
lunar landing. They share every launch field — the launcher, the ascent, the
trans-lunar leg are the same mission underneath — so switching keeps the
vehicle and swaps the half that differs: pod and entry corridor for lander
and descent plan. The landing view adds its own tiles (insertion, descent-
orbit and descent Δv, touchdown sink and drift, propellant left, hover
margin, deepest throttle, landing site), a **powered-descent card** —
altitude against downrange with high gate called out, the two velocity
components, and the commanded throttle — and a **Moon-frame view** of the
parking orbit, the descent ellipse and the descent arc, with the sub-Earth
direction drawn so near side and far side are obvious. Every landing metric
is sweepable and solvable: *vary lander propellant until hover margin = 200 s*
is a search over whole missions like any other.

**Vehicle presets** sit beside the mission presets and configure the entire
launcher — stack height, engines, propellant loads, diameters, and the
pitch-over kick the stack needs: Sable (the reference), Sable with four
strap-ons, a Falcon-class Merlin stack, a hydrolox upper stage, and an
**expendable Starship** — 9 m across, 33 Raptors under 3400 t, a six-engine
ship that does insertion and TLI itself. The kick angle is a field now,
because it is the one guidance number that does not scale: 8° suits the
reference vehicle, a Starship-class stack wants about 5°, and anything with
strap-ons lofts on either. Get it wrong and the shooting method closes on the
target *energy* with the perigee underground — which the chain now catches and
reports instead of designing a trans-lunar injection off a garbage orbit.
Engine-driven stages can also **estimate their own dry mass** from the design
(the server always accepted this; the page now offers it), which is what makes
a preset like Starship weigh 4867 t instead of whatever was left in the boxes.

The 3D scene is true ECI geometry: the full pad-to-splashdown track
(ascent, parking orbit, TLI burn, outbound, return, entry — each its own
color), a textured globe spinning about the real pole with **launch-site
and splashdown markers riding the rotating surface**, correct
depth-occlusion of trajectory lines behind the Earth, mission-time
playback, an inertial/rotating frame toggle, and Earth/fit zoom shortcuts.
The camera **frames whatever the run turned out to be** rather than a fixed
number — a 500 km flyby and a 6000 km one are not the same picture — and an
adaptive scale bar says how big the frame is, because across one mission it
spans five orders of magnitude. On a landing run the lunar parking orbit and
the descent are drawn in the same scene, each sample offset by where the Moon
actually was at that moment, so zooming in on the Moon shows them in place.
The timeline, the frame toggle, the zooms and Run all have keys
(`R`, `space`, `←`/`→`, `F`, `E`, `Z`, `L`), listed on the page.
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

### Earth-orbit missions

The mission switch offers a third family: **Earth orbit**, whose goal is the
orbit itself. Presets cover **LEO, polar, Molniya, GEO,** and **LEO
re-entry** (up, a few revolutions, a deorbit burn, and home on parachutes).
The chain flies the ascent at the target inclination, executes finite
transfer burns on the kick stage — the raise at an equator crossing when the
plane must move, the combined circularise-and-plane-change at apogee — and
reports the achieved elements against the request. A vehicle that cannot
afford a target says so: GEO wants ~4.3 km/s of kick stage, and the
reference vehicle's honest answer is "propellant depleted". Failed missions
in any mode now return everything that **was** simulated, with the outcome
named, and the launch view flies the flight to wherever the simulation
actually ended.

### Suborbital missions

The switch also offers **suborbital**, for flights that never intend to reach
orbit. There are two, because there are two things people mean by the word:

- a **hop** — straight up and back down, closing on an **apogee**. No
  pitch-over at all; the vertical is held the whole way and the engines cut the
  moment the arc they have built reaches the altitude asked for. The reference
  vehicle asked for 100 km reaches 99.9 and splashes down 2.7 km from the pad.
- a **shot** — a lofted ballistic arc, closing on a **ground range**. The
  gravity turn hands over to the commanded loft attitude, and cutoff comes when
  the free-flight arc through the vehicle's own state reaches the range asked
  for. 400 km asked, 399 flown, apogee 91 km.

Neither can use the orbital cutoff, which closes on specific energy: energy is
the right quantity for an orbit and the wrong one here, since the same energy
describes an arc that lands 200 km downrange and one that lands 2000, and says
nothing at all about apogee. A vehicle has one degree of freedom at cutoff, so
it is given one target — the other number is a consequence and is reported, not
commanded. The commanded target is corrected for what the atmosphere takes: a
hop that cuts at 40 km still has 40 km of air to climb through, and one secant
step on the command is the difference between asking for 100 km and reaching
it. The capsule is handed to the same entry simulator the returning missions
use, so the arc gets real drag, heating and parachutes.

### Vehicle builder

The **🛠 vehicle builder** chip in the Launcher card (key `B`, or
`http://localhost:8137/build`) opens the vehicle in its own configuration
window: the 3D model in the centre — the very mesh `/api/geometry` builds
from the `LaunchVehicle` the mission flies — with the configuration on the
left and the derived statistics on the right. Stages can be added and
removed (2–5, roles relabel as the stack changes), strap-on boosters
attached 0–8 at a time, and any diameter edited with the model and every
number updating live. A named engine owns its propellant: picking one
locks the mixture select to it, and the server rejects a form that names
both an engine and a different mixture — a Raptor does not burn kerolox.
The builder opens on the panel's current form, and its **↩ mission
control** and **🚀 launch view** links carry the configuration back out.

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
* **The lunar descent** — a landing configuration replaces the return leg
  with the powered descent itself, flown over *the same procedural ground the
  simulation flew over*: the height field in the viewer is a line-by-line port
  of `src/terrain.jl`, sharing its 32-bit hash so that `Math.imul` reproduces
  the crater lattice exactly and the two surfaces agree to a nanometre. A
  crater the guidance steered around is a crater you can watch it steer
  around. Concentric rings of terrain, each four times wider and four times
  coarser than the one inside it, run from two thirds of a metre at the
  footpads out past the horizon. The director cuts from the vehicle firing
  retrograde eight kilometres up, through the pitch-over onto the approach and
  the view down the last kilometre, to a camera *standing on the surface* —
  placed by searching a ring of candidate spots for one with high ground and
  an unobstructed sight line, because ground chosen by a hazard scan for being
  flat very often has a crater next to it. Below forty metres the plume starts
  moving the surface, and what leaves travels almost flat and very fast: no
  air to slow it, so it does not billow, does not rise and does not settle.
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

Press `p` in the launch view to show the frame time and the background
render scale. The planet/sky and cloud-deck shaders are the entire frame
budget — measured at 84 ms and 52 ms of a 136.8 ms frame on an integrated
Intel UHD 630, against 0.1 ms for every mesh, particle and HUD draw combined
— so those two passes render into an offscreen target at a fraction of
native resolution while the vehicle stays sharp at full resolution. The
scale adapts to hold ~60 fps: on the machine this was developed against it
settles at 0.35 and the frame comes in at 17.5 ms, down from 136.8 ms. Lower
means a softer sky, never a softer vehicle.

The **sky** is a procedural star catalogue rather than a texture. Stars sit
at hashed positions inside a cube-face cell grid and are drawn as smooth
points whose angular radius is tied to the *pixel* solid angle, which is the
whole trick: a star stays about a pixel across at any zoom, so it antialiases
instead of stair-stepping and holds still when the camera turns. Brightness
follows a steep magnitude law, color comes off a blackbody-ish ramp that
keeps most stars near white and puts only the tails at amber and blue, and
the galactic plane both glows — noise-modulated, with dust lanes — and
carries roughly twice the local star count. Because the limiting magnitude
tracks the pixel size, the 8° tracker camera reaches deeper than the 83° pad
view instead of showing an empty sky at a hundredth the solid angle, and
brightness is measured *from* that limit so the visible population always
spans the full range. Inside the atmosphere the field is extinguished by the
air column, reddened toward the horizon, and scintillates on a wall clock
that ignores the time warp; above ~35 km it is clean. Its predecessor
quantized the view direction into cells and lit whole cells — the "stars"
were squares that grew with every zoom-in.

The 2D mission viewer (`make_viewer.py`) got the same treatment on a smaller
budget: its sky is now real directions on a celestial sphere, projected
gnomonically through the same yaw and pitch as the scene, so it swings when
you drag and holds still when you zoom, as an infinitely distant sky does.
It was 90 fixed screen-space pixels, which sat where they were painted
however the camera moved.

The vehicle wears a **livery**: a decal sheet painted procedurally in
(station, roll) space and wrapped cylindrically over the skin — roll-pattern
quadrants on the booster, an interstage trim band, stage numerals, an
insignia roundel and ensign, split-line markings down the fairing, and the
vehicle name. Baked vertex colours could never carry lettering at this
triangle count, so the markings live in a generated texture and the mesh
itself stays clean. The insignia is an original mark rather than any real
agency's; it is all drawn in one function (`buildLiveryTex`) if you want a
different scheme.

The **launch complex is dimensioned off the vehicle**, which matters the
moment you fly something that is not the reference rocket. The plume
aperture, the flame-trench width and the launch table already followed the
core diameter; the service tower, the hardstand extents, the table height and
the pad cameras now do too. Left fixed, a 9 m vehicle gets a tower standing
*inside* its own launch table, a table hanging off the edge of the concrete,
and a pad camera framing a tank barrel. The site outside the fence line grows
as the square root of the diameter — a pad five times as wide is not a site
five times as large — and the connections that have to meet the hardstand are
pre-divided by that scale so they land on the deck edge rather than short of
it. The reference vehicle's pad is unchanged, to the metre.

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
  landing.jl        lunar-orbit insertion, descent orbit, powered descent
                    to touchdown (moonlanding)
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
| Lunar ascent & rendezvous | `powered_descent` run backwards is the ascent; `maneuvers.jl` already has the Lambert and Clohessy-Wiltshire pieces for the rendezvous that follows |
| Landing site targeting | the site is wherever the ground track goes; targeting one couples the number of parking revolutions and the descent-orbit geometry, which is an outer loop on `moonlanding` |
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
* The lunar terrain is *statistically* a Moon and not the Moon: the right
  relief, crater size distribution and footpad-scale roughness, at no
  particular place. Mare Tranquillitatis is not at longitude 23° E here.
  Craters superpose rather than degrade, so every one of them is young, and
  there are no boulders, rilles or ejecta rays as distinct objects.
* The mascons are five point-mass anomalies at roughly the right selenographic
  positions, buried at the depth that matches the observed ratio between the
  surface anomaly and the anomaly at orbital altitude. That gets the field
  right where a spacecraft flies and not where a geologist works.
* The landing radar is an altimeter and a velocimeter, not a position fix, so
  the horizontal navigation error at touchdown is whatever orbit determination
  left — a few hundred metres, as it was on Apollo. Hazard avoidance registers
  its chosen site to the navigation frame once, at high gate; there is no
  continuous terrain-relative navigation, and so no drift correction after it.
* No aborts, no staging during descent, no plume–surface interaction, and no
  attitude dynamics anywhere in the lunar chain: thrust points where guidance
  asks, with no rate limit and no RCS budget to pay for it.
* The trip home assumes the landing site stays in the orbiter's plane, which
  it does here because the coplanar model puts the Moon's spin axis along the
  mission-plane normal. On the real Moon a site drifts out of plane at up to
  half a degree an hour, and the plane change to catch up is what limits a
  surface stay to a few days.
* Rendezvous is a single Lambert transfer plus a braking burn, searched over
  lift-off and transfer time. Real ones are a phasing sequence with a
  mid-course correction and a manual terminal phase, and they carry a much
  bigger dispersion budget than one impulse pair implies.
