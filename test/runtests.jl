using Test
using SatelliteSim

@testset "SatelliteSim" begin

@testset "atmosphere (USSA76)" begin
    atm = USSA76()
    rho0, T0, p0, a0 = atmosphere_state(atm, 0.0)
    @test isapprox(rho0, 1.225; rtol = 1e-3)
    @test isapprox(T0, 288.15; atol = 0.01)
    @test isapprox(p0, 101325.0; rtol = 1e-6)
    @test isapprox(a0, 340.29; atol = 0.5)

    rho11, T11, _, _ = atmosphere_state(atm, 11019.0)  # tropopause (geometric)
    @test isapprox(T11, 216.65; atol = 0.2)
    @test isapprox(rho11, 0.3639; rtol = 0.01)

    rho50, _, _, _ = atmosphere_state(atm, 50.0e3)
    @test isapprox(rho50, 1.027e-3; rtol = 0.03)

    rho86, _, _, _ = atmosphere_state(atm, 86.0e3)
    @test isapprox(rho86, 6.958e-6; rtol = 0.05)

    rho120, _, _, _ = atmosphere_state(atm, 120.0e3)
    @test isapprox(rho120, 2.222e-8; rtol = 1e-6)

    # monotone decreasing density
    hs = 0:5e3:500e3
    rhos = [atmosphere_state(atm, Float64(h))[1] for h in hs]
    @test all(diff(rhos) .< 0)

    scaled = ScaledAtmosphere(atm, 1.2)
    @test atmosphere_state(scaled, 30e3)[1] ≈ 1.2 * atmosphere_state(atm, 30e3)[1]
end

@testset "gravity" begin
    r = (RE_EQ + 400e3, 0.0, 0.0)
    apm = gravity_accel(PointMassGravity(), r, 0.0)
    @test isapprox(apm[1], -MU_EARTH / (RE_EQ + 400e3)^2; rtol = 1e-12)
    aj2 = gravity_accel(J2Gravity(), r, 0.0)
    # J2 strengthens equatorial radial pull by ~1.5*J2*(Re/r)^2
    @test aj2[1] < apm[1]
    @test isapprox(aj2[1] / apm[1], 1 + 1.5 * 1.08262668e-3 * (RE_EQ / (RE_EQ + 400e3))^2; rtol = 1e-6)

    comp = CompositeGravity(PointMassGravity(), PointMassGravity())
    ac = gravity_accel(comp, r, 0.0)
    @test ac[1] ≈ 2 * apm[1]
end

@testset "frames" begin
    # geodetic round trip
    for (lat, lon, h) in ((0.7, -2.1, 400e3), (-1.2, 0.3, 120e3), (0.0, 0.0, 0.0))
        r = ecef_from_geodetic(lat, lon, h)
        lat2, lon2, h2 = geodetic_from_ecef(r)
        @test isapprox(lat2, lat; atol = 1e-9)
        @test isapprox(lon2, lon; atol = 1e-9)
        @test isapprox(h2, h; atol = 5e-3)  # Bowring single pass: mm-level
    end
    # kepler: circular orbit speed
    a = RE_EQ + 400e3
    r, v = state_from_elements(a, 0.0, 0.9, 0.5, 0.0, 0.0)
    @test isapprox(sqrt(sum(abs2, r)), a; rtol = 1e-12)
    @test isapprox(sqrt(sum(abs2, v)), sqrt(MU_EARTH / a); rtol = 1e-12)
    # vis-viva on an ellipse
    r2, v2 = state_from_elements(a, 0.3, 1.0, 0.1, 0.2, 2.0)
    eps1 = sum(abs2, v2) / 2 - MU_EARTH / sqrt(sum(abs2, r2))
    @test isapprox(eps1, -MU_EARTH / 2a; rtol = 1e-10)
end

@testset "aero tables" begin
    t = Table1D([0.0, 1.0, 2.0], [10.0, 20.0, 40.0])
    @test interp1(t, -1.0) == 10.0
    @test interp1(t, 3.0) == 40.0
    @test interp1(t, 0.5) == 15.0
    aero = default_capsule_aero()
    @test cd_coeff(aero, 25.0, 0.0) ≈ 1.52
    @test cd_coeff(aero, 25.0, 0.1) > cd_coeff(aero, 25.0, 0.0)  # off-trim drag rise
    @test SatelliteSim.cm_coeff(aero, 5.0, 0.1, 0.0) < 0          # restoring moment
    @test SatelliteSim.cm_coeff(aero, 5.0, -0.1, 0.0) > 0
end

@testset "heating" begin
    q = heating_convective(1e-4, 7500.0, 1.8)
    @test q > 0
    # hand check: 1.74153e-4*sqrt(1e-4/1.8)*7500^3
    @test isapprox(q, 1.74153e-4 * sqrt(1e-4 / 1.8) * 7500.0^3; rtol = 1e-12)
    @test heating_radiative(1e-4, 7500.0, 1.8) == 0.0   # negligible below 9 km/s
    @test heating_radiative(1e-4, 11000.0, 1.8) > 0.0
    @test wall_temperature(1e6, 0.85) > 1500
end

@testset "orbit propagation accuracy" begin
    # circular orbit, point-mass gravity: energy & radius conserved by RK4
    a = RE_EQ + 300e3
    r0, v0 = state_from_elements(a, 0.0, 0.9, 0.0, 0.0, 0.0)
    veh = default_reentry_pod()
    scn = Scenario(vehicle = veh, gravity = PointMassGravity(),
                   r0 = r0, v0 = v0, h_ei = 0.0,   # aero off everywhere
                   dt_orbit = 2.0, t_max = 5600.0) # ~ one period
    res = simulate(scn; log_dt_orbit = 60.0)
    rlast = res.log.vin[end] # inertial speed at end
    @test isapprox(rlast, sqrt(MU_EARTH / a); rtol = 1e-7)
    @test res.terminated == :timeout
end

@testset "propulsion" begin
    st = Stage(:test, 100.0, 900.0, 10.0e3, 300.0, 0.1)
    @test isapprox(stage_mdot(st), 10.0e3 / (9.80665 * 300.0); rtol = 1e-12)
    @test isapprox(stage_burn_time(st), 900.0 / stage_mdot(st); rtol = 1e-12)
    # pressure correction: sea level thrust < vacuum thrust
    @test stage_thrust(st, 101325.0) < st.thrust_vac
    @test stage_thrust(st, 0.0) == st.thrust_vac
    # Tsiolkovsky
    dv = stage_dv(st, 400.0)
    @test isapprox(dv, 9.80665 * 300.0 * log(1400.0 / 500.0); rtol = 1e-12)

    lv = default_moon_rocket()
    @test liftoff_mass(lv) > 50e3
    # the stack should carry ~LEO + TLI ideal delta-v with generous losses
    m_above(k) = SatelliteSim.stack_mass_above(lv, k + 1; fairing = k < 2)
    total_dv = sum(stage_dv(lv.stages[k], m_above(k)) for k in 1:3)
    @test 12.0e3 < total_dv < 15.0e3
end

@testset "moon ephemeris" begin
    r0, v0 = state_from_elements(RE_EQ + 200e3, 0.0, deg2rad_(28.5), 0.2, 0.0, 0.0)
    eph = coplanar_moon(r0, v0; phase0 = deg2rad_(90.0))
    # circular at the lunar distance for all t
    for t in (0.0, 1e5, 1e6)
        @test isapprox(sqrt(sum(abs2, moon_position(eph, t))), A_MOON; rtol = 1e-12)
    end
    # period: back to the same spot after a sidereal month
    s0 = moon_position(eph, 0.0)
    s1 = moon_position(eph, 2pi / N_MOON)
    @test all(isapprox.(s0, s1; atol = 1.0))
    # coplanar with the orbit: moon position ⟂ orbit normal
    h = (r0[2]*v0[3]-r0[3]*v0[2], r0[3]*v0[1]-r0[1]*v0[3], r0[1]*v0[2]-r0[2]*v0[1])
    hn = sqrt(sum(abs2, h))
    @test abs(sum(moon_position(eph, 12345.0) .* h) / (hn * A_MOON)) < 1e-12
    # velocity is the analytic derivative
    dt = 1.0
    sfd = (moon_position(eph, 1e5 + dt) .- moon_position(eph, 1e5 - dt)) ./ 2dt
    @test all(isapprox.(moon_velocity(eph, 1e5), sfd; rtol = 1e-6))
end

@testset "ascent to orbit" begin
    lv = default_moon_rocket()
    guid, asc = tune_ascent(lv, AscentGuidance())
    @test asc.reached_orbit
    el = asc.elements
    @test 180e3 < el.rp - RE_MEAN < 220e3     # near-circular parking orbit
    @test 180e3 < el.ra - RE_MEAN < 220e3
    @test isapprox(rad2deg_(el.i), 28.5; atol = 1.0)
    @test abs(asc.gamma_cut) < deg2rad_(0.1)
    @test asc.prop_left[end] == lv.stages[end].mprop  # kick stage untouched
    names = [e.name for e in asc.events]
    @test :liftoff in names && :seco in names
    @test :fairing_jettison in names
    # max-q in a sane band for a small launcher
    @test 20e3 < maximum(asc.log.qbar) < 90e3
end

@testset "circumlunar free return" begin
    ms = moonshot()
    cis = ms.cislunar
    @test cis.outcome == :entry_interface
    @test isapprox(cis.perilune_alt, 2000e3; atol = 30e3)
    @test isapprox(cis.vac_perigee_alt, 35e3; atol = 5e3)
    @test 2.5e3 < cis.dv_tli < 3.4e3
    # direct free return: entry on the FIRST post-flyby perigee pass, no
    # phasing loop back out past the Moon (regression: the design once
    # converged onto a 19.7-day two-revolution return)
    @test cis.miss_passes == 0
    @test cis.t - cis.t_perilune < 5.0 * 86400.0   # return leg ~ outbound leg
    rmax = maximum(hypot.(cis.log.rx, cis.log.ry, cis.log.rz))
    @test rmax < 1.15 * A_MOON                     # never far beyond the lunar distance
    # entry interface speed near lunar-return values
    ent = ms.entry
    ei = ent.events[findfirst(e -> e.name == :entry_interface, ent.events)]
    @test 10.5e3 < ei.vrel < 11.3e3
    @test ent.terminated == :splashdown
    @test ent.v_splash < 8.0
    @test ent.peak_gload < 30.0              # inside a survivable ballistic corridor
    # energy sanity on the coast: two-body + moon only, no drag above EI
    @test cis.m < SatelliteSim.liftoff_mass(ms.lv)

    # configurability: a custom launch vehicle flows through the whole chain
    lv = default_moon_rocket(payload = 300.0)
    ms2 = moonshot(lv = lv, hp_moon = 1500e3)
    @test ms2.lv === lv
    @test ms2.entry_scn.vehicle.mass == 300.0
    @test isapprox(ms2.cislunar.perilune_alt, 1500e3; atol = 75e3)
end

@testset "rigid body" begin
    # rotation basics
    q = quat_axis_angle((0.0, 0.0, 1.0), pi/2)
    v = qrotate(q, (1.0, 0.0, 0.0))
    @test all(isapprox.(v, (0.0, 1.0, 0.0); atol = 1e-12))
    @test all(isapprox.(qrotate_inv(q, v), (1.0, 0.0, 0.0); atol = 1e-12))
    q2 = quat_from_to((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    @test all(isapprox.(qrotate(q2, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0); atol = 1e-12))

    # torque-free tumble: energy and |angular momentum| conserved (RK4)
    I = (110.0, 137.0, 150.0)
    w = (0.3, -0.5, 0.8)
    q = (1.0, 0.0, 0.0, 0.0)
    E0 = rot_energy(I, w)
    L0 = sqrt(sum(abs2, ang_momentum(I, w)))
    dt = 0.01
    for _ in 1:20_000
        k1 = euler_wdot(I, w, (0.0, 0.0, 0.0))
        w2 = w .+ 0.5dt .* k1
        k2 = euler_wdot(I, w2, (0.0, 0.0, 0.0))
        w3 = w .+ 0.5dt .* k2
        k3 = euler_wdot(I, w3, (0.0, 0.0, 0.0))
        w4 = w .+ dt .* k3
        k4 = euler_wdot(I, w4, (0.0, 0.0, 0.0))
        w = w .+ (dt / 6) .* (k1 .+ 2 .* k2 .+ 2 .* k3 .+ k4)
    end
    @test isapprox(rot_energy(I, w), E0; rtol = 1e-8)
    @test isapprox(sqrt(sum(abs2, ang_momentum(I, w))), L0; rtol = 1e-8)
end

@testset "rcs" begin
    sys = default_pod_rcs()
    auth = torque_authority(sys)
    @test all(a -> a > 0, auth)                    # every axis controllable
    # couples are pure torques: net force cancels
    F = (0.0, 0.0, 0.0)
    for t in sys.thrusters
        F = F .+ t.thrust .* t.dir
    end
    @test all(abs.(F) .< 1e-9)
    # limit cycle: hand formula
    lc = limit_cycle_prop(1000.0, 28.0, 220.0, 0.01, deg2rad_(5.0), 86400.0;
                          nthr = 2, thrust = 10.0)
    w = 28.0 * 0.01 / 1000.0
    cycles = 86400.0 / (2 * deg2rad_(5.0) / w)
    @test isapprox(lc, cycles * 2 * 10.0 / (9.80665 * 220.0) * 0.01; rtol = 1e-12)
    # slew: bang-bang time
    p, ts = slew_prop(1000.0, 28.0, 220.0, 1.0 * pi)
    @test isapprox(ts, 2 * sqrt(pi * 1000.0 / 28.0); rtol = 1e-12)
    b = cruise_rcs_budget(default_kick_rcs(), 1000.0; duration = 20 * 86400.0)
    @test b.margin > 0                             # sized for the cruise
end

@testset "6-DOF entry vs 4-DOF" begin
    veh = default_reentry_pod()
    scn = scenario_from_elements(DeorbitElements(), veh)
    r4 = simulate(scn)
    r6 = simulate_entry6(scn; rcs = default_pod_rcs())   # wind-hold coast
    @test r6.terminated == :splashdown
    @test isapprox(r6.peak_gload, r4.peak_gload; rtol = 0.05)
    @test isapprox(r6.heat_load, r4.heat_load; rtol = 0.05)
    @test haversine(r4.lat_splash, r4.lon_splash,
                    r6.lat_splash, r6.lon_splash) < 20e3
    @test r6.rcs_used < 0.05                       # grams, not kilograms
    @test r6.max_alpha_after_peak < deg2rad_(10.0) # stable through supersonic
    # uncontrolled coast arrives far off-trim yet the shape self-rights
    r6f = simulate_entry6(scn)
    @test r6f.terminated == :splashdown
    ei = findfirst(e -> e.name == :entry_interface, r6f.events)
    i = findfirst(t -> t >= r6f.events[ei].t, r6f.log.t)
    @test r6f.log.alpha_t[i] > deg2rad_(45.0)      # broadside at EI...
    @test r6f.peak_gload < 1.10 * r4.peak_gload    # ...still survivable
    # tipoff-rate damping costs grams
    r6r = simulate_entry6(scn; rcs = default_pod_rcs(), rcs_mode = :rate_damp,
                          w0 = (deg2rad_(3.0), deg2rad_(-2.0), deg2rad_(1.5)))
    @test r6r.rcs_used < 0.05
end

@testset "TLI dispersion + TCM" begin
    ms = moonshot(tli_mag_err = 0.003, tli_point_err = deg2rad_(0.3))
    @test ms.cruise !== nothing
    c = ms.cruise
    @test 5.0 < c.tcm_dv < 150.0
    @test c.tcm_prop < 23.0                        # inside the kick margin
    @test c.rcs.margin > 0
    @test isapprox(ms.cislunar.perilune_alt, 2000e3; atol = 60e3)
    @test abs(ms.cislunar.vac_perigee_alt - 35e3) < 25e3
    @test ms.cislunar.miss_passes == 0             # corrected cruise comes straight home
    @test ms.entry.terminated == :splashdown
    @test ms.entry.peak_gload < 30.0
end

@testset "mission config (TOML)" begin
    spec = load_mission(joinpath(@__DIR__, "..", "missions", "moonshot.toml"))
    @test spec.pod_mass == 350.0
    @test spec.h_park == 200e3
    @test spec.hp_moon == 2000e3
    @test length(spec.lv.stages) == 3
    lv0 = default_moon_rocket()
    @test liftoff_mass(spec.lv) ≈ liftoff_mass(lv0)
    @test spec.lv.stages[3].isp_vac == lv0.stages[3].isp_vac
    ms = run_mission(spec)
    @test ms.cislunar.outcome == :entry_interface
    @test isapprox(ms.cislunar.perilune_alt, 2000e3; atol = 30e3)
end

@testset "mesh + mass properties" begin
    # unit box: exact polyhedral integrals
    b = box_mesh((0.0, 0.0, 0.0), (2.0, 1.0, 1.0))
    @test mesh_volume(b) ≈ 2.0
    mb = mass_properties(b, 12.0)
    @test all(isapprox.(mb.cg, (1.0, 0.5, 0.5); atol = 1e-12))
    @test isapprox(mb.inertia[1], 12.0 / 12 * (1 + 1); rtol = 1e-12)
    @test isapprox(mb.inertia[2], 12.0 / 12 * (4 + 1); rtol = 1e-12)
    # sphere: analytic within mesh resolution
    prof = [(cos(s), sin(s)) for s in range(0.0, 1.0 * pi; length = 41)]
    sph = lathe_mesh([(x, r) for (x, r) in prof]; nseg = 64)
    @test isapprox(mesh_volume(sph), 4pi / 3; rtol = 0.005)
    ms_ = mass_properties(sph, 100.0)
    @test isapprox(ms_.inertia[1], 40.0; rtol = 0.005)
    # STL round trip preserves the solid
    path = joinpath(mktempdir(), "sph.stl")
    write_stl(path, sph)
    sph2 = read_stl(path)
    @test length(sph2) == length(sph)
    @test isapprox(mesh_volume(sph2), mesh_volume(sph); rtol = 1e-6)
    # procedural launcher: closed, positive volume, one section per stage
    # plus the pod capsule and the fairing shell that encloses it
    rk, secs = rocket_mesh(diameter = 2.0, prop_masses = [10_000.0, 2_000.0])
    @test mesh_volume(rk) > 0
    @test length(secs) == 4
    @test secs[end-1].name == :pod
    @test secs[end].name == :fairing
    @test issorted([s.x0 for s in secs])
    # triangle ranges tile the merged soup exactly, in order
    @test secs[1].t0 == 1 && secs[end].t1 == length(rk)
    @test all(secs[i+1].t0 == secs[i].t1 + 1 for i in 1:length(secs)-1)
    # each section is itself a closed solid (viewers detach them individually)
    @test all(mesh_volume(TriMesh(rk.tris[s.t0:s.t1])) > 0 for s in secs)
    # barrel volume actually swallows the propellant it was sized for
    @test mesh_volume(rk) > (10_000.0 + 2_000.0) / 1020.0
end

@testset "Newtonian panel aero" begin
    @test isapprox(cp_max_newtonian(1e6), 1.8394; atol = 1e-3)  # M -> inf limit
    # sphere: CD = Cp_max/2 exactly in Newtonian theory
    prof = [(cos(s), sin(s)) for s in range(0.0, 1.0 * pi; length = 41)]
    sph = lathe_mesh([(x, r) for (x, r) in prof]; nseg = 64)
    pa = panel_aero(sph; sref = 1.0 * pi, lref = 2.0, ref = (0.0, 0.0, 0.0),
                    machs = [20.0])
    @test isapprox(cd_coeff(pa, 20.0, 0.0), cp_max_newtonian(20.0) / 2; rtol = 0.01)

    # capsule from committed geometry: stable, damped, trims at zero
    cap = read_stl(joinpath(@__DIR__, "..", "geometry", "capsule.stl"))
    mp = mass_properties(cap, 350.0)
    @test mp.offdiag_frac < 1e-3                    # axisymmetric
    pac = panel_aero(cap; sref = pi * 0.75^2, lref = 1.5, ref = mp.cg)
    @test cm_coeff(pac, 20.0, deg2rad_(10.0), 0.0) < 0     # restoring
    @test SatelliteSim.interp1(pac.cmq, 20.0) < 0          # damped
    @test abs(trim_alpha(pac)) < deg2rad_(1.0)
    @test 1.4 < cd_coeff(pac, 20.0, 0.0) < 2.0

    # geometry-to-trajectory: fly the pod on mesh-derived aero
    veh0 = default_reentry_pod()
    veh = Vehicle(name = "mesh-pod", mass = 350.0, sref = pi * 0.75^2,
                  lref = 1.5, rn = 1.8, iyy = mp.inertia[2], aero = pac,
                  chutes = veh0.chutes)
    r6 = simulate_entry6(scenario_from_elements(DeorbitElements(), veh);
                         inertia = mp.inertia, rcs = default_pod_rcs())
    r0 = simulate(scenario_from_elements(DeorbitElements(), veh0))
    @test r6.terminated == :splashdown
    @test isapprox(r6.peak_gload, r0.peak_gload; rtol = 0.15)
    @test isapprox(r6.heat_load, r0.heat_load; rtol = 0.15)

    # starship demo mesh: lifting body with a passive trim from its flaps
    ship = read_stl(joinpath(@__DIR__, "..", "geometry", "starship.stl"))
    mps = mass_properties(ship, 120_000.0)
    pas = panel_aero(ship; sref = 9.0 * 50.0, lref = 50.0, ref = mps.cg)
    @test cd_coeff(pas, 20.0, deg2rad_(90.0)) > 5 * cd_coeff(pas, 20.0, 0.0)
    @test cl_coeff(pas, 20.0, deg2rad_(20.0)) /
          cd_coeff(pas, 20.0, deg2rad_(20.0)) > 1.0       # slender-body L/D
    at = trim_alpha(pas)
    @test deg2rad_(15.0) < at < deg2rad_(65.0)            # stable belly-first trim
    @test SatelliteSim.interp1(pas.cmq, 20.0) < 0
end

@testset "maneuvers" begin
    a = RE_MEAN + 500e3
    vc = sqrt(MU_EARTH / a)
    T = 2pi * sqrt(a^3 / MU_EARTH)
    # lambert: quarter arc of a circular orbit recovers circular velocity
    v1, v2 = lambert((a, 0.0, 0.0), (0.0, a, 0.0), T / 4)
    @test all(isapprox.(v1, (0.0, vc, 0.0); atol = 1e-3))
    @test all(isapprox.(v2, (-vc, 0.0, 0.0); atol = 1e-3))
    # lambert vs a known ellipse: recover the velocity at nu1
    ae, ee = 10_000e3, 0.3
    E_of(nu) = 2 * atan(sqrt((1 - ee) / (1 + ee)) * tan(nu / 2))
    t_of(nu) = (E_of(nu) - ee * sin(E_of(nu))) / sqrt(MU_EARTH / ae^3)
    nu1, nu2 = deg2rad_(30.0), deg2rad_(150.0)
    r1v, v1a = state_from_elements(ae, ee, 0.0, 0.0, 0.0, nu1)
    r2v, _ = state_from_elements(ae, ee, 0.0, 0.0, 0.0, nu2)
    vl1, _ = lambert(r1v, r2v, t_of(nu2) - t_of(nu1))
    @test all(isapprox.(vl1, v1a; rtol = 1e-5))
    # hohmann sanity: LEO -> GEO
    dv1, dv2, tofh = hohmann(a, 42_164e3)
    @test isapprox(dv1 + dv2, 3900.0; atol = 150.0)   # the classic ~3.9 km/s
    @test isapprox(tofh / 3600, 5.2; atol = 0.3)
    @test isapprox(plane_change_dv(vc, deg2rad_(60.0)), vc; rtol = 1e-12)
    # CW two-impulse: transfer nulls the position, braking nulls the velocity
    n = sqrt(MU_EARTH / a^3)
    r0 = (-2000.0, -10_000.0, 500.0)
    v0 = (0.5, 1.0, -0.2)
    dvc1, dvc2, vreq = cw_two_impulse(r0, v0, n, 2000.0)
    rT, vT = cw_propagate(r0, vreq, n, 2000.0)
    @test sqrt(sum(abs2, rT)) < 1e-6
    @test all(isapprox.(vT .+ dvc2, (0.0, 0.0, 0.0); atol = 1e-9))
    @test isapprox(stumpff(0.0)[1], 0.5; atol = 1e-12)
    @test isapprox(stumpff(1e-8)[1], stumpff(-1e-8)[1]; atol = 1e-8)  # slope 1/24
end

@testset "propagation & burn correctness" begin
    # circular orbit through the cislunar coast: radius drift over one rev
    a = RE_MEAN + 200e3
    r0, v0 = state_from_elements(a, 0.0, deg2rad_(28.5), 0.0, 0.0, 0.0)
    eph = coplanar_moon(r0, v0; phase0 = deg2rad_(90.0))
    T = 2pi * sqrt(a^3 / MU_EARTH)
    L = SatelliteSim.CislunarLog()
    leg = SatelliteSim._coast_leg!(L, r0, v0, 0.0, eph;
                                   theta_g0 = 0.0, h_stop = 0.0, t_end = T,
                                   stop_after_flyby = false, log_every = 10^9)
    @test abs(sqrt(sum(abs2, leg.r)) - a) < 100.0     # meters after one rev
    eps0 = sum(abs2, v0) / 2 - MU_EARTH / a
    eps1 = sum(abs2, leg.v) / 2 - MU_EARTH / sqrt(sum(abs2, leg.r))
    @test isapprox(eps1, eps0; rtol = 1e-6)
    # TLI burn: duration matches the rocket equation, arc stays modest
    ms = moonshot()
    kick = ms.lv.stages[end]
    mdot = stage_mdot(kick)
    m0 = ms.cislunar.m * exp(ms.cislunar.dv_tli / (G0 * kick.isp_vac))
    @test isapprox(ms.cislunar.burn_duration, (m0 - ms.cislunar.m) / mdot; rtol = 1e-3)
    Tpark = 2pi * sqrt(ms.ascent.elements.a^3 / MU_EARTH)
    @test ms.cislunar.burn_duration / Tpark * 360 < 20.0   # burn arc < 20 deg
    # burn log rows carry instantaneous velocity (monotone speed increase)
    Lc = ms.cislunar.log
    ib = findall(p -> p == 1, Lc.phase)
    sp = [hypot(Lc.vx[i], Lc.vy[i], Lc.vz[i]) for i in ib]
    @test issorted(sp) && sp[end] - sp[1] > 2000.0
end

@testset "full reentry smoke test" begin
    veh = default_reentry_pod()
    scn = scenario_from_elements(DeorbitElements(), veh)
    res = simulate(scn)
    @test res.terminated == :splashdown
    @test res.v_splash < 8.0                 # under main chute
    @test 3.0 < res.peak_gload < 12.0        # ballistic LEO entry range
    @test 20.0 < res.peak_qdot / 1e4 < 200.0 # W/cm², small-capsule class
    ev = [e.name for e in res.events]
    @test :entry_interface in ev
    @test :deploy_drogue in ev
    @test :deploy_main in ev
    # peak heating should occur between ~40 and 75 km
    L = res.log
    imax = argmax(L.qdot_conv .+ L.qdot_rad)
    @test 35e3 < L.h[imax] < 80e3
end

end
