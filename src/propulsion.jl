# Propulsion: rocket stages and the stacked launch vehicle.
#
# A `Stage` burns at constant vacuum mass flow; delivered thrust is
# pressure-corrected with the nozzle exit area,
#     F(p_amb) = F_vac - Ae * p_amb,
# which reproduces the usual sea-level/vacuum Isp split of a fixed-throttle
# engine. Stages stack bottom-to-top in a `LaunchVehicle`; the payload on top
# is whatever the mission puts there (here: the reentry pod).

"""
    Stage(name, mdry, mprop, thrust_vac, isp_vac, ae)

One rocket stage. `mdry`/`mprop` [kg], `thrust_vac` [N], `isp_vac` [s],
`ae` nozzle exit area [m^2] (pressure thrust correction; 0 for vacuum stages).
"""
struct Stage
    name::Symbol
    mdry::Float64
    mprop::Float64
    thrust_vac::Float64
    isp_vac::Float64
    ae::Float64
end

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
    LaunchVehicle

Stacked stages (element 1 is the first to burn) plus fairing and payload.
`sref` is the vehicle aerodynamic reference area, `cd` a Mach-indexed drag
table used during atmospheric ascent.
"""
Base.@kwdef struct LaunchVehicle
    name::String = "launcher"
    stages::Vector{Stage}
    fairing_mass::Float64
    payload_mass::Float64
    sref::Float64
    cd::Table1D
end

"Total lift-off mass [kg]."
liftoff_mass(lv::LaunchVehicle) =
    sum(s.mdry + s.mprop for s in lv.stages) + lv.fairing_mass + lv.payload_mass

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
        stages = [
            Stage(:sable1, 3800.0, 42000.0, 950.0e3, 305.0, 0.80),
            Stage(:sable2,  900.0,  9500.0,  95.0e3, 345.0, 0.0),
            Stage(:sablek,  140.0,   950.0,  15.0e3, 315.0, 0.0),
        ],
        fairing_mass = 150.0,
        payload_mass = payload,
        sref = pi * (d / 2)^2,
        cd = LV_CD_TABLE,
    )
end
