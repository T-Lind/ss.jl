"""
    SatelliteSim

Mid-fidelity, extensible satellite mission simulation, currently focused on
atmospheric reentry from low Earth orbit.

Current capabilities
  * 4-DOF flight dynamics: 3 translational DOF in Earth-centered inertial
    Cartesian coordinates over a rotating WGS-84 Earth, plus body pitch
    (angle-of-attack) dynamics with Mach-dependent static stability and
    pitch damping — the "body angle" degree of freedom.
  * Entry interface at 120 km: pure orbital mechanics above (point mass + J2),
    full aero/heating environment below.
  * US Standard Atmosphere 1976 (analytic layers to 86 km, tables to 500 km).
  * Mach-dependent blunt-capsule aerodynamic database (CD, CL_alpha,
    Cm_alpha, Cm_q) with clamped-linear interpolation.
  * Stagnation heating: Sutton-Graves convective + Tauber-Sutton radiative,
    integrated heat load, radiative-equilibrium wall temperature.
  * Parachute sequence (drogue + main) with finite fill time.
  * Deorbit targeting to a splashdown point (US west coast by default).
  * Threaded Monte Carlo dispersion analysis (mass, aero, atmosphere,
    state, parachutes) with footprint statistics.

Extension points (deliberately structured for growth)
  * `AbstractGravity` / `CompositeGravity` / `ThirdBodyGravity` — add lunar &
    solar perturbations by supplying an ephemeris.
  * `AbstractAtmosphere` — swap in NRLMSISE-00, GRAM-style dispersions, or
    other planets.
  * `Scenario.extra_accel(r, v, t)` — thrust (launch / mid-course), SRP, drag
    makeup, etc.
  * `AbstractAeroDatabase` — richer CN/CA/Cm tables in (Mach, alpha).
  * The integrator is isolated behind `rk4_step!`; the event/phase driver in
    `simulate` accepts any state-vector-compatible dynamics.
"""
module SatelliteSim

using LinearAlgebra
using Printf
using Random
using Statistics

include("constants.jl")
include("vec3.jl")
include("atmosphere.jl")
include("gravity.jl")
include("frames.jl")
include("aerodynamics.jl")
include("vehicle.jl")
include("heating.jl")
include("dynamics.jl")
include("integrator.jl")
include("simulation.jl")
include("scenarios.jl")
include("montecarlo.jl")
include("output.jl")

export
    # constants / helpers
    MU_EARTH, RE_EQ, RE_MEAN, OMEGA_EARTH, G0, deg2rad_, rad2deg_,
    # atmosphere
    AbstractAtmosphere, USSA76, ScaledAtmosphere, atmosphere_state,
    # gravity
    AbstractGravity, PointMassGravity, J2Gravity, ThirdBodyGravity,
    CompositeGravity, gravity_accel,
    # frames
    geodetic_from_ecef, ecef_from_geodetic, state_from_elements, haversine,
    # aero / vehicle
    Table1D, interp1, CapsuleAero, default_capsule_aero, scaled_aero,
    cd_coeff, cl_coeff, cm_coeff,
    Parachute, Vehicle, default_reentry_pod, ballistic_coefficient,
    # heating
    heating_convective, heating_radiative, wall_temperature,
    # dynamics / sim
    Scenario, FlightContext, simulate, SimResult, FlightEvent, flight_data,
    # scenarios
    DeorbitElements, scenario_from_elements, target_deorbit, west_coast_scenario,
    WEST_COAST_TARGET_LAT, WEST_COAST_TARGET_LON,
    # monte carlo
    Dispersions, MCSample, run_montecarlo, mc_statistics,
    # output
    write_csv, write_trajectory_csv, write_events_csv, write_montecarlo_csv,
    print_summary

end # module
