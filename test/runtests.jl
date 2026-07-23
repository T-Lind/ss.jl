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
