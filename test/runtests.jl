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
