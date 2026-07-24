# Vehicle definition: mass properties, reference geometry, aero database,
# thermal parameters and the parachute sequence.

"""
    Parachute(name, cda, mach_max, alt_max, fill_time)

A drag device that deploys once Mach < `mach_max` AND geodetic altitude <
`alt_max`, then ramps its drag area from 0 to `cda` [m^2] over `fill_time`
seconds (quadratic fill — avoids force discontinuities in the integrator).
"""
struct Parachute
    name::Symbol
    cda::Float64        # steady drag area CD*A [m^2]
    mach_max::Float64   # deploy only below this Mach
    alt_max::Float64    # deploy only below this geodetic altitude [m]
    fill_time::Float64  # canopy fill time [s]
end

"Fraction of steady drag area at time `tau` after deployment."
@inline chute_fill(c::Parachute, tau::Float64) = clamp(tau / c.fill_time, 0.0, 1.0)^2

Base.@kwdef struct Vehicle{A<:AbstractAeroDatabase}
    name::String = "capsule"
    mass::Float64                 # [kg]
    sref::Float64                 # aerodynamic reference area [m^2]
    lref::Float64                 # reference length (diameter) [m]
    rn::Float64                   # nose (heatshield) radius for heating [m]
    iyy::Float64                  # pitch moment of inertia [kg m^2]
    emissivity::Float64 = 0.85    # TPS surface emissivity (radiative equilibrium wall temp)
    aero::A
    chutes::Vector{Parachute} = Parachute[]
end

"Ballistic coefficient at a given Mach and trim alpha [kg/m^2]."
ballistic_coefficient(v::Vehicle, M::Float64 = 25.0) =
    v.mass / (cd_coeff(v.aero, M, v.aero isa CapsuleAero ? v.aero.alpha_trim : 0.0) * v.sref)

"""
    default_reentry_pod(; mass = 350.0, cl_trim_hyp = 0.45)

A small crewed reentry pod: 1.5 m diameter blunt capsule, drogue + main
parachute sequence sized for ~5.5 m/s splashdown. Ballistic coefficient
~130 kg/m^2 — typical small-capsule class.

`cl_trim_hyp` is the hypersonic trim lift coefficient, produced in reality by
an offset centre of gravity that holds the capsule at an angle of attack. At
the default it gives L/D ~ 0.3, the Apollo figure, and `Scenario.bank` then
points that lift vector (0 = lift up).

`alpha_trim` is that angle of attack, and it defaults to a nonzero value
whenever there is trim lift **because the two are the same physical fact** —
the CG offset produces both. Setting lift without setting the angle leaves
the model incoherent: the 4-DOF hides it, since it builds the lift direction
from `bank` explicitly, but the 6-DOF derives that direction from the body's
off-wind axis component, which collapses to noise when the pitching moment
restores toward zero AoA. With them tied, the 6-DOF flying `:bank_hold` RCS
reproduces the 4-DOF result to 0.01 g.

**This is not cosmetic on a lunar return.** Flown ballistically
(`cl_trim_hyp = 0`) the same trajectory peaks at 18 g and stays above 15 g
for 26 seconds — survivable for cargo, not for people, and very close to what
Zond 5 actually pulled on the first ballistic circumlunar return. Lift-up at
L/D 0.3 stretches the deceleration over a longer, higher path and brings the
peak to about 6 g. The cost is integrated heating, which nearly doubles: a
lifting entry soaks for longer even though its peak heat *rate* is lower, and
that is what sizes the ablator.
"""
function default_reentry_pod(; mass::Float64 = 350.0,
                             cl_trim_hyp::Float64 = 0.45,
                             alpha_trim::Float64 = cl_trim_hyp > 0 ?
                                                   deg2rad_(25.0) : 0.0)
    d = 1.5
    Vehicle(
        name = "reentry-pod",
        mass = mass,
        sref = pi * (d / 2)^2,
        lref = d,
        rn = 1.2 * d,              # heatshield spherical-cap radius, Apollo-like Rn/D
        iyy = 0.35 * mass * (d / 2)^2 * 2.0,   # ~ solid-ish squat body about pitch axis
        aero = default_capsule_aero(cl_trim_hyp = cl_trim_hyp,
                                    alpha_trim = alpha_trim),
        chutes = [
            Parachute(:drogue, 12.0, 1.5, 9000.0, 2.0),
            Parachute(:main, 260.0, 0.5, 3000.0, 6.0),
        ],
    )
end
