# Propulsion: rocket stages and the stacked launch vehicle.
#
# A `Stage` burns at constant vacuum mass flow; delivered thrust is
# pressure-corrected with the nozzle exit area,
#     F(p_amb) = F_vac - Ae * p_amb,
# which reproduces the usual sea-level/vacuum Isp split of a fixed-throttle
# engine. Stages stack bottom-to-top in a `LaunchVehicle`; the payload on top
# is whatever the mission puts there (here: the reentry pod).

"""
    Stage(name, mdry, mprop, thrust_vac, isp_vac, ae, [prop, n_engines, diameter])

One rocket stage. `mdry`/`mprop` [kg], `thrust_vac` [N], `isp_vac` [s],
`ae` nozzle exit area [m^2] (pressure thrust correction; 0 for vacuum stages).

`prop`, `n_engines` and `diameter` carry no flight dynamics — the integrator
never looks at them — but they are what makes the stage a physical object
rather than four numbers: the propellant's bulk density and the stage's
cross-section together set how long the barrel has to be to hold `mprop`,
and the engine count sets how many bells hang under it. A `diameter` of 0
means "whatever the vehicle is", so a stack of uniform stages needs no
per-stage value. Omit all three and a stage is assumed to be single-engine
kerolox at the vehicle diameter, which is what every hand-specified stage in
this repo was implicitly using.

See [`sized_stage`](@ref) to build one from an engine and a propellant load.
"""
struct Stage
    name::Symbol
    mdry::Float64
    mprop::Float64
    thrust_vac::Float64
    isp_vac::Float64
    ae::Float64
    prop::Propellant
    n_engines::Int
    diameter::Float64
end

Stage(name::Symbol, mdry, mprop, thrust_vac, isp_vac, ae) =
    Stage(name, mdry, mprop, thrust_vac, isp_vac, ae, PROPELLANTS[:kerolox], 1, 0.0)
Stage(name::Symbol, mdry, mprop, thrust_vac, isp_vac, ae, prop, n_engines) =
    Stage(name, mdry, mprop, thrust_vac, isp_vac, ae, prop, n_engines, 0.0)

"Stage diameter [m], falling back to the vehicle's when unset."
stage_diameter(st::Stage, vehicle_d::Float64) =
    st.diameter > 0 ? st.diameter : vehicle_d

"Vacuum mass flow rate [kg/s] (constant while burning)."
@inline stage_mdot(st::Stage) = st.thrust_vac / (G0 * st.isp_vac)

"Delivered thrust [N] at ambient pressure `pamb` [Pa] (floored at 20% F_vac)."
@inline stage_thrust(st::Stage, pamb::Float64) =
    max(st.thrust_vac - st.ae * pamb, 0.2 * st.thrust_vac)

"Burn time to propellant depletion [s]."
@inline stage_burn_time(st::Stage) = st.mprop / stage_mdot(st)

"Ideal (Tsiolkovsky) vacuum delta-v [m/s] of a stage pushing mass `mpayload`."
stage_dv(st::Stage, mpayload::Float64) =
    G0 * st.isp_vac * log((st.mdry + st.mprop + mpayload) / (st.mdry + mpayload))

"""
    BoosterSet(; stage, count, ignition_delay, sep_delay, core_throttle)

`count` identical strap-on boosters clustered around the first stage and
burning in parallel with it. Each carries its own `stage`'s propellant load,
so the set contributes `count` times that stage's thrust and mass flow.

`ignition_delay` [s] lights the set after lift-off (0 = lit on the pad; a
positive value is the air-lit half of a Atlas-style solid set). Once a
booster is out of propellant it hangs on as dead weight for `sep_delay` [s]
before being jettisoned. `core_throttle` is the fraction of its rated thrust
the first stage holds while any booster in the set is burning — the
Falcon-Heavy trick of running the core down so it still has propellant left
when the sides go.

All the boosters in a set separate together. Two sets with different
`ignition_delay`s model a staggered light; two sets with different
`sep_delay`s, a staggered drop.
"""
Base.@kwdef struct BoosterSet
    stage::Stage
    count::Int = 2
    ignition_delay::Float64 = 0.0
    sep_delay::Float64 = 0.0
    core_throttle::Float64 = 1.0
end

"Total wet mass of a booster set [kg]."
booster_mass(b::BoosterSet) = b.count * (b.stage.mdry + b.stage.mprop)

"Thrust [N] of a whole booster set at ambient pressure `pamb` [Pa]."
booster_thrust(b::BoosterSet, pamb::Float64) = b.count * stage_thrust(b.stage, pamb)

"Mass flow [kg/s] of a whole booster set."
booster_mdot(b::BoosterSet) = b.count * stage_mdot(b.stage)

"""
    LaunchVehicle

Stacked stages (element 1 is the first to burn) plus fairing and payload.
`sref` is the vehicle aerodynamic reference area, `cd` a Mach-indexed drag
table used during atmospheric ascent. `boosters` are strap-on sets burning
in parallel with stage 1; the default is none, and a vehicle without them
behaves exactly as before.
"""
Base.@kwdef struct LaunchVehicle
    name::String = "launcher"
    stages::Vector{Stage}
    fairing_mass::Float64
    payload_mass::Float64
    sref::Float64
    cd::Table1D
    boosters::Vector{BoosterSet} = BoosterSet[]
end

"Core diameter [m] implied by the vehicle's reference area."
core_diameter(lv::LaunchVehicle) = 2 * sqrt(lv.sref / pi)

"""
    frontal_area(lv, attached) -> Float64

Reference area [m^2] with the booster sets in `attached` (a boolean per set)
still on: the core cross-section plus every strap-on's. Boosters sit beside
the core rather than behind it, so while they are attached the stack really
does present their area to the flow; once they go, drag drops back to the
core's. Shielding between neighbours is ignored, which errs high.
"""
function frontal_area(lv::LaunchVehicle, attached::AbstractVector{Bool})
    a = lv.sref
    dc = core_diameter(lv)
    for (i, b) in enumerate(lv.boosters)
        attached[i] || continue
        a += b.count * pi * (stage_diameter(b.stage, dc) / 2)^2
    end
    a
end

"Loaded propellant volume of a stage [m^3] — what sets its physical size."
stage_volume(st::Stage) = st.mprop / bulk_density(st.prop)

"""
    barrel_length(mprop, density, diameter) -> Float64

Structural length [m] a stage needs: its propellant's own volume in a barrel
of that diameter, plus 15% for ullage and tank domes, plus an engine bay of
0.9 diameters.

`rocket_mesh` sizes the drawn barrels with exactly this call. That sharing is
the point — the vehicle on screen and the vehicle whose inertia sets the RCS
budget have to be one vehicle, and a second copy of this rule is how they
would quietly stop being that.

Note it takes LOADED propellant: a tank does not get shorter as it drains, so
this is the right length for a stage at any point in its life.
"""
barrel_length(mprop::Real, density::Real, diameter::Real) =
    mprop / (density * pi * (diameter / 2)^2) * 1.15 + 0.9 * diameter

"Length [m] of stage `st`'s structure at core diameter `dc`."
stage_length(st::Stage, dc::Real) =
    barrel_length(st.mprop, bulk_density(st.prop), stage_diameter(st, dc))

"""
    cruise_inertia(lv, m_stack, payload_diameter; payload_mass) -> (I_roll, I_transverse)

Inertia [kg·m²] of the stack that actually coasts: the kick stage — dry
structure plus whatever propellant is still aboard — with the payload on top
of it.

This replaced a hardcoded 1000 kg·m². The constant was not merely imprecise,
it was *inert*: transverse inertia is the only route by which vehicle
geometry reaches the attitude-control budget, so with it pinned, building a
different spacecraft moved the RCS numbers not at all. It is also the term
that pulls hardest in both directions — limit-cycle propellant goes as 1/I
while slews go as √I — so an inertia an order of magnitude low does not err
in a single safe direction.
"""
function cruise_inertia(lv::LaunchVehicle, m_stack::Real, payload_diameter::Real;
                        payload_mass::Real = lv.payload_mass)
    kick = lv.stages[end]
    dc = core_diameter(lv)
    dk = stage_diameter(kick, dc)
    Lk = barrel_length(kick.mprop, bulk_density(kick.prop), dk)
    # what is left of the stack once the payload is subtracted; never less
    # than the stage's own dry mass, so a mass bookkeeping slip cannot produce
    # a weightless kick stage
    mk = max(Float64(m_stack) - payload_mass, kick.mdry)
    dp = max(Float64(payload_diameter), 0.1)
    # a capsule is roughly as tall as it is wide (Apollo CM: 3.9 m across,
    # 3.5 m high), and a bus in the same envelope is no worse an assumption
    Lp = dp
    stack_inertia(((mk, dk / 2, Lk, Lk / 2),
                   (Float64(payload_mass), dp / 2, Lp, Lk + Lp / 2)))
end

"Total lift-off mass [kg], strap-on boosters included."
liftoff_mass(lv::LaunchVehicle) =
    sum(s.mdry + s.mprop for s in lv.stages) + lv.fairing_mass + lv.payload_mass +
    sum(booster_mass, lv.boosters; init = 0.0)

"Thrust [N] leaving the pad: stage 1 plus every set lit at t = 0."
pad_thrust(lv::LaunchVehicle) =
    stage_thrust(lv.stages[1], P0_SEA) +
    sum(booster_thrust(b, P0_SEA) for b in lv.boosters if b.ignition_delay <= 0.0;
        init = 0.0)

"Stack mass above (and including) stage `k`, with fairing if not jettisoned."
function stack_mass_above(lv::LaunchVehicle, k::Int; fairing::Bool = true)
    m = lv.payload_mass + (fairing ? lv.fairing_mass : 0.0)
    for i in k:length(lv.stages)
        m += lv.stages[i].mdry + lv.stages[i].mprop
    end
    m
end

# Slender-launcher drag coefficient vs Mach (power-on, generic small LV).
const LV_CD_TABLE = Table1D(
    [0.0, 0.6, 0.9, 1.05, 1.2, 1.6, 2.5, 4.0, 6.0, 10.0, 25.0],
    [0.28, 0.30, 0.42, 0.58, 0.55, 0.46, 0.36, 0.30, 0.26, 0.24, 0.22],
)

"""
Drag INCREMENT for flying with the payload exposed instead of inside a shroud,
on the same Mach abscissa as `LV_CD_TABLE`.

A fairing is an ogive: a sharp nose that puts an oblique shock on the flow. A
crew capsule is the opposite of that on purpose — a spherical cap on a 32.5
degree afterbody, shaped to stand a detached bow shock off itself during entry.
Flown nose-first up through the atmosphere, that same bluntness is pure wave
drag, and it costs most exactly where wave drag peaks: through the transonic
and the first stretch of supersonic. Above about Mach 4 the two noses converge,
because by then the whole stack is a slender body and its base and skin
friction dominate whatever is on the front of it.

These are increments on the coefficient, not a coefficient: the payload is
usually narrower than the core it sits on, so it only owns the part of the
frontal area it actually presents. See [`bare_payload_cd`].
"""
const LV_CD_BARE_DELTA = Table1D(
    [0.0, 0.6, 0.9, 1.05, 1.2, 1.6, 2.5, 4.0, 6.0, 10.0, 25.0],
    [0.06, 0.09, 0.22, 0.34, 0.32, 0.24, 0.16, 0.11, 0.09, 0.08, 0.07],
)

"""
    bare_payload_cd(pod_diameter, sref; base=LV_CD_TABLE) -> Table1D

`base` with the blunt-nose increment of `LV_CD_BARE_DELTA` weighted by the
fraction of `sref` the exposed payload presents. A 1.5 m capsule on a 10 m core
is 2% of the frontal area and changes essentially nothing; an Apollo command
module on an S-IVB is a third of it and changes the ascent; a payload as wide
as the stack owns the whole increment.

The weight is an AREA ratio and not a diameter ratio because drag is a force on
an area, and using the diameter would trip a 1.5 m capsule on a 10 m core into
a 15% drag rise it has no way to cause.
"""
function bare_payload_cd(pod_diameter::Real, sref::Real; base::Table1D = LV_CD_TABLE)
    dref = 2 * sqrt(max(float(sref), 1e-9) / pi)
    frac = clamp((float(pod_diameter) / dref)^2, 0.0, 1.0)
    Table1D(base.x, base.y .+ frac .* LV_CD_BARE_DELTA.y)
end

"""
    stack_sref(stage_diameters, pod_diameter, fairing) -> Float64

Reference area [m^2] for a stack. With a fairing the shroud is sized around the
payload and tucks inside the stack's own envelope, so the widest stage sets it.
Without one the payload is out in the wind on its own, and if it is wider than
anything under it — a capsule on a narrow kick stage — then IT is the widest
cross-section and the one the flow sees.
"""
function stack_sref(stage_diameters, pod_diameter::Real, fairing::Bool)
    d = maximum(stage_diameters)
    fairing || (d = max(d, float(pod_diameter)))
    pi * (d / 2)^2
end

"""
    starship_expendable(; payload = 12500.0)

A Starship-class two-stage methalox vehicle, flown expendably: a 9 m core
with 33 sea-level Raptors under 3400 t of propellant, and a 6-engine upper
stage with 1200 t that does insertion *and* trans-lunar injection itself —
no kick stage, because at this scale the ship is the kick stage.

Dry masses come from `sized_stage`'s mass-estimating relations rather than
from a data sheet, so they are conceptual figures (about 190 t and 60 t)
rather than SpaceX's. The point of the vehicle here is the difference in
kind: the reference Sable tops out around 400 kg through TLI, while this
throws tens of tonnes, which is the entire reason a lander mission is
possible at all.

It wants a **shallower pitch-over kick** than the reference vehicle — around
5° against Sable's 8°. Nothing is wrong with 8°; it simply lofts a stack
with this much upper-stage thrust, and the shooter cannot recover the
trajectory from there. `optimize_kick = true` finds the angle on its own.
"""
function starship_expendable(; payload::Float64 = 12500.0)
    d = 9.0
    LaunchVehicle(
        name = "Starship (expendable)",
        stages = [
            sized_stage(:superheavy; engine = :raptor_2, n_engines = 33,
                        prop_mass = 3400.0e3, diameter = d),
            sized_stage(:ship; engine = :raptor_2, n_engines = 6,
                        prop_mass = 1200.0e3, diameter = d),
        ],
        # the payload rides inside the ship, so there is no separate fairing
        fairing_mass = 0.0,
        payload_mass = payload,
        sref = pi * (d / 2)^2,
        cd = LV_CD_TABLE,
    )
end

"""
    default_moon_rocket(; payload = 350.0)

Three-stage launch vehicle sized to send a ~350 kg reentry pod around the
Moon on a free-return trajectory:

  * Stage 1 "Sable-1": kerolox booster, 950 kN vac / ~870 kN SL.
  * Stage 2 "Sable-2": kerolox upper stage, 95 kN vac — to parking orbit.
  * Stage 3 "Sable-K": storable-propellant kick stage, 15 kN — parking-orbit
    trim + trans-lunar injection (restart capable).

Ideal vacuum delta-v of the stack is ~13.2 km/s: ~9.6 to LEO (before
gravity/drag losses, after Earth-rotation credit) and ~3.3 for TLI.
"""
function default_moon_rocket(; payload::Float64 = 350.0)
    d = 1.8
    LaunchVehicle(
        name = "Sable",
        # masses and performance are the hand-tuned reference figures; the
        # propellant and engine count are what give the stage a physical size
        # and shape (five 190 kN engines under the booster, one restartable
        # storable-propellant engine on the kick stage)
        stages = [
            Stage(:sable1, 3800.0, 42000.0, 950.0e3, 305.0, 0.80,
                  PROPELLANTS[:kerolox], 5),
            Stage(:sable2,  900.0,  9500.0,  95.0e3, 345.0, 0.0,
                  PROPELLANTS[:kerolox], 1),
            Stage(:sablek,  140.0,   950.0,  15.0e3, 315.0, 0.0,
                  PROPELLANTS[:hypergolic], 1),
        ],
        fairing_mass = 150.0,
        payload_mass = payload,
        sref = pi * (d / 2)^2,
        cd = LV_CD_TABLE,
    )
end
