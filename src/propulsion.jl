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
