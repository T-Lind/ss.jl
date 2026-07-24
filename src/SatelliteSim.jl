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
import TOML

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
include("rigidbody.jl")
include("rcs.jl")
include("engines.jl")
include("propulsion.jl")
include("moon.jl")
include("launch.jl")
include("translunar.jl")
include("simulation.jl")
include("entry6.jl")
include("scenarios.jl")
include("mission.jl")
include("maneuvers.jl")
include("solve.jl")
include("mesh.jl")
include("panelaero.jl")
include("config.jl")
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
    geodetic_from_ecef, ecef_from_geodetic, state_from_elements,
    elements_from_state, haversine,
    # aero / vehicle
    Table1D, interp1, CapsuleAero, default_capsule_aero, scaled_aero,
    cd_coeff, cl_coeff, cm_coeff,
    Parachute, Vehicle, default_reentry_pod, ballistic_coefficient,
    # heating
    heating_convective, heating_radiative, wall_temperature,
    # dynamics / sim
    Scenario, FlightContext, simulate, SimResult, FlightEvent, flight_data,
    bank_command, gload_bank,
    # scenarios
    DeorbitElements, scenario_from_elements, target_deorbit, west_coast_scenario,
    WEST_COAST_TARGET_LAT, WEST_COAST_TARGET_LON,
    # monte carlo
    Dispersions, MCSample, run_montecarlo, mc_statistics,
    # scalar targeting
    SolveResult, find_root, converged,
    # propellants, engines, conceptual sizing
    Propellant, PROPELLANTS, propellant, bulk_density, propellant_volumes,
    Engine, ENGINES, engine, lookup_engine, stage_mass, sized_stage,
    # propulsion / launch vehicle
    Stage, LaunchVehicle, BoosterSet, default_moon_rocket,
    stage_thrust, stage_mdot, booster_mass, booster_thrust, booster_mdot,
    frontal_area, core_diameter, pad_thrust,
    stage_burn_time, stage_dv, liftoff_mass, stack_mass_above,
    stage_volume, stage_diameter,
    # moon
    MU_MOON, R_MOON, A_MOON, N_MOON,
    CircularMoonEphemeris, coplanar_moon, moon_position, moon_velocity,
    moon_distance, moon_altitude,
    # launch
    AscentGuidance, AscentResult, simulate_ascent, tune_ascent, launch_azimuth,
    launch_window, next_launch_window, site_geocentric_lat,
    # rigid body / attitude
    Quat, qmul, qconj, qnormalize, qrotate, qrotate_inv, quat_axis_angle,
    quat_from_to, qdot, euler_wdot, rot_energy, ang_momentum,
    # rcs
    RCSThruster, RCSystem, thruster_torque, torque_authority, rate_damp_command,
    limit_cycle_prop, slew_prop, cruise_rcs_budget,
    default_pod_rcs, default_kick_rcs, rcs_mdot,
    # 6-DOF entry
    Entry6Result, Entry6Log, simulate_entry6,
    # translunar
    CislunarResult, fly_cislunar, tli_burn, design_free_return, seed_free_return,
    design_tcm, fly_cislunar_tcm,
    # mission
    MoonshotResult, CruiseReport, moonshot, print_moonshot_summary,
    # maneuvers
    lambert, hohmann, plane_change_dv, impulsive_prop, stumpff,
    cw_stm, cw_propagate, cw_two_impulse,
    # mesh & panel aero
    TriMesh, read_stl, write_stl, mesh_area, mesh_volume, mass_properties,
    lathe_mesh, box_mesh, merge_meshes, rocket_mesh, pod_mesh, interstage_length,
    PanelAero, panel_aero, cp_max_newtonian, trim_alpha,
    # config
    MissionSpec, load_mission, run_mission,
    # output
    write_csv, write_trajectory_csv, write_events_csv, write_montecarlo_csv,
    write_ascent_csv, write_cislunar_csv,
    print_summary

end # module
