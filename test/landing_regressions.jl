using Test, SatelliteSim

@testset "landing termination and achieved lunar orbit" begin
    S = SatelliteSim
    rpdi = R_MOON + 15e3
    vpdi = sqrt(MU_MOON * (2 / rpdi - 1 / (0.5 * (rpdi + R_MOON + 100e3))))
    l = Lander(mdry = 3500.0, mprop = 100.0, thrust = 45e3, isp = 311.0,
               throttle_min = 0.10)
    d = powered_descent(l, (rpdi, 0.0, 0.0), (0.0, vpdi, 0.0), 3600.0)
    @test d.outcome === :propellant
    @test isnan(d.t_gate)
    @test d.log.t[end] == d.t_touchdown
    @test d.log.m[end] == d.m == l.mdry
    @test d.log.h[end] > 0
    @test isapprox(d.log.v[end], S.vnorm(d.v); atol = 1e-9)

    impact = S._descent_leg(l, (R_MOON + 1.0, 0.0, 0.0), (-100.0, 1000.0, 0.0),
                            3600.0, 0.0, 0.0)
    @test impact.outcome === :surface
    @test 0.0 < impact.t < 0.5
    @test abs(impact.h) < 1e-6

    eph = coplanar_moon((7e6, 0.0, 0.0), (0.0, 7.5e3, 0.0))
    target = (cosd(-45.5) * cosd(177.6), cosd(-45.5) * sind(177.6), sind(-45.5))
    for h_actual in (100e3, 166e3)
        rp = R_MOON + h_actual
        r = (rp, 0.0, 0.0)
        v = (1.0, 1700.0, 0.0)  # small finite-perilune radial residual
        T = S.target_parking(eph, 0.0, r, v, 100e3, 15e3, target, 1)
        @test abs(S.vdot(r, T.v_park)) < 1e-6
        @test isapprox(S.vnorm(T.v_park), sqrt(MU_MOON / rp); atol = 1e-9)
        rr, vv, tt = r, T.v_park, 0.0
        for _ in 1:6000
            rr, vv = S._moon_step(rr, vv, T.wait / 6000; t = tt)
            tt += T.wait / 6000
        end
        Up = S.vunit(moonfixed_inv(target, T.t_pdi, eph))
        @test S.vdot(S.vunit(rr), Up) < -0.99998
        @test isapprox(S.vnorm(rr), rp; atol = 1.0)
    end
end
