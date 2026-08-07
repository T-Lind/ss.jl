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
include("terrain.jl")
include("launch.jl")
include("translunar.jl")
include("simulation.jl")
include("entry6.jl")
include("scenarios.jl")
include("mission.jl")
include("earthorbit.jl")
include("suborbital.jl")
include("landingnav.jl")
include("landing.jl")
include("lunarreturn.jl")
include("maneuvers.jl")
include("solve.jl")
include("mesh.jl")
include("panelaero.jl")
include("config.jl")
include("montecarlo.jl")
include("output.jl")

"""
    julia_main() -> Cint

Entry point for the frozen Windows application built by `build/build_app.jl`.

Opens the desktop window and returns when it is closed. Not used when running
from source — `scripts/desktop.jl` is the entry point there — and deliberately
thin, because everything it needs already exists.

The panel layer is `include`d at run time rather than compiled into the
package. That is on purpose: the expensive thing to compile is the mission
stack above, which IS frozen into the sysimage, while `panelapp.jl` is
straightforward code whose compile cost is small. Keeping it out of the
package keeps the simulator importable without dragging an HTTP server and a
browser launcher along with it.

Assets (the three HTML pages, `static/*.js`, and these two scripts) sit next
to the executable under `share/scripts`. `SSJL_SCRIPTS` overrides that, which
is how a frozen build can be pointed at a working tree.
"""
function julia_main()::Cint
    try
        here = dirname(abspath(PROGRAM_FILE))

        # Thread count is fixed by the runtime BEFORE this function runs, and a
        # compiled app starts with one thread — `-t auto` is a julia flag and
        # there is no julia command line here to put it on. One thread is not a
        # detail: `panel_mission` never yields, so a mission in flight blocks
        # the accept loop and every geometry request the builder makes while
        # you type. Measured on this exe: 1 thread out of 16 available.
        #
        # JULIA_NUM_THREADS *is* read at startup, so the fix is to set it and
        # start again. Ugly, and the alternative is an app that quietly uses a
        # sixteenth of the machine unless its user knows to set an environment
        # variable before double-clicking it.
        if Threads.nthreads() == 1 && !haskey(ENV, "SSJL_THREADED")
            exe = abspath(PROGRAM_FILE)
            if isfile(exe)
                env = copy(ENV)
                env["JULIA_NUM_THREADS"] = "auto"
                env["SSJL_THREADED"] = "1"       # the guard against looping
                p = run(setenv(Cmd([exe; ARGS]), env); wait = true)
                return Cint(p.exitcode)
            end
        end

        candidates = [
            get(ENV, "SSJL_SCRIPTS", ""),
            joinpath(here, "..", "share", "scripts"),   # installed layout
            joinpath(here, "share", "scripts"),
            joinpath(dirname(@__DIR__), "scripts"),     # running from source
        ]
        root = nothing
        for c in candidates
            isempty(c) && continue
            if isfile(joinpath(c, "desktop.jl"))
                root = abspath(c)
                break
            end
        end
        if root === nothing
            println(stderr, "cannot find the panel scripts; set SSJL_SCRIPTS " *
                            "to the directory holding desktop.jl")
            return Cint(1)
        end
        Base.include(Main, joinpath(root, "desktop.jl"))
        # Both the binding lookup AND the call have to be deferred. Reading
        # `Main.launch` directly is what Julia 1.12 warns about — the binding
        # did not exist in this function's world age, only the call was wrapped,
        # and the warning says plainly that it becomes an error in a future
        # version. Fetching the binding through invokelatest too is the fix.
        launch_fn = Base.invokelatest(getglobal, Main, :launch)
        Base.invokelatest(launch_fn)
        return Cint(0)
    catch err
        # a frozen app has no console to read a stack trace from unless it is
        # printed here, and an exit code alone is not a bug report
        showerror(stderr, err, catch_backtrace())
        println(stderr)
        return Cint(1)
    end
end

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
    Parachute, Vehicle, default_reentry_pod, apollo_capsule,
    ballistic_coefficient,
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
    Stage, LaunchVehicle, BoosterSet, default_moon_rocket, starship_expendable,
    stage_thrust, stage_mdot, booster_mass, booster_thrust, booster_mdot,
    frontal_area, core_diameter, pad_thrust,
    stage_burn_time, stage_dv, liftoff_mass, stack_mass_above,
    stage_volume, stage_diameter,
    LV_CD_TABLE, LV_CD_BARE_DELTA, bare_payload_cd, stack_sref,
    # moon
    MU_MOON, R_MOON, A_MOON, N_MOON,
    CircularMoonEphemeris, coplanar_moon, moon_position, moon_velocity,
    moon_distance, moon_altitude,
    # lunar terrain
    LunarTerrain, mare_terrain, highland_terrain, terrain_height, terrain_radius,
    terrain_normal, terrain_slope, site_hazard, safe_site, surface_offset,
    SurfaceModel, surface_radius, surface_altitude, ground_elevation,
    terrain_profile, moonfixed, moonfixed_inv, moonfixed_basis,
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
    # earth orbit
    OrbitTarget, ORBITS, EarthOrbitResult, earthorbit,
    # suborbital
    SuborbitalResult, suborbital,
    # translunar
    CislunarResult, fly_cislunar, tli_burn, design_free_return, seed_free_return,
    design_tcm, fly_cislunar_tcm,
    # mission
    MoonshotResult, CruiseReport, moonshot, print_moonshot_summary,
    translunar_design,
    # lunar landing
    Lander, default_lander, lander_mass, lander_dv, LandingResult, DescentResult,
    DescentLog, LunarOrbitLog, moonlanding, print_landing_summary,
    fly_to_perilune, loi_burn, doi_burn, powered_descent, tune_braking,
    terminal_descent, mci_state, selenographic, apollo_landing,
    AscentStage, ascent_mass, ascent_dv, Orbiter, orbiter_mass, orbiter_dv,
    # coming back
    LunarAscentResult, AscentMoonLog, ReturnResult, lunar_ascent,
    tune_lunar_ascent, lunar_rendezvous, plan_rendezvous,
    trans_earth_injection, moonreturn, print_return_summary,
    DescentConfig, nominal,
    # lunar gravity field
    Mascon, LunarGravity, default_mascons, lunar_gravity, gravity_anomaly,
    # descent navigation, radar and hazard avoidance
    LandingRadar, DescentNav, perfect_nav, NavState, init_nav, nav_altitude,
    nav_propagate!, radar_update!, nav_error, HazardScan, redesignate,
    # maneuvers
    lambert, hohmann, plane_change_dv, impulsive_prop, stumpff,
    cw_stm, cw_propagate, cw_two_impulse,
    # mesh & panel aero
    TriMesh, read_stl, write_stl, mesh_area, mesh_volume, mass_properties,
    lathe_mesh, box_mesh, merge_meshes, rocket_mesh, pod_mesh, probe_mesh, pod_radius, pod_crew,
    interstage_length,
    PanelAero, panel_aero, cp_max_newtonian, trim_alpha,
    # config
    MissionSpec, load_mission, run_mission,
    # output
    write_csv, write_trajectory_csv, write_events_csv, write_montecarlo_csv,
    write_ascent_csv, write_cislunar_csv,
    print_summary

end # module
