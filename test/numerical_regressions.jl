using Test, SatelliteSim, LinearAlgebra

@testset "numerical reliability regressions" begin
    @testset "orbital element singular conventions preserve state" begin
        for e in (0.0, 0.3), i in (0.0, 1e-15, 0.5, Float64(pi)), nu in (0.3, 1.3, 4.7)
            r, v = state_from_elements(8e6, e, i, 0.7, 0.6, nu)
            el = elements_from_state(r, v)
            rr, vv = state_from_elements(el.a, el.e, el.i, el.raan, el.argp, el.nu)
            @test norm(collect(rr .- r)) < 1e-4
            @test norm(collect(vv .- v)) < 1e-7
        end
    end

    @testset "scalar targeting reports measured values and honours budgets" begin
        for budget in 1:12
            r = find_root(x -> NaN, 0.0, 1.0; max_iter = budget)
            @test r.iterations == length(r.history) <= budget
            @test !converged(r)
        end
        r = find_root(x -> exp(x) - 5, 0.0, 40.0; max_iter = 3)
        @test r.status === :max_iter
        @test r.value == exp(r.x) - 5
        @test (r.x, r.value) in r.history
        r = find_root(x -> 0.4 < x < 0.6 ? NaN : x - 0.5, 0.0, 1.0; xtol = 0.2)
        @test r.status === :infeasible && !converged(r)
        @test isfinite(r.value) && r.value == r.x - 0.5
        r = find_root(x -> 1e200 * (x - 0.5), 0.0, 1.0)
        @test converged(r) && r.x == 0.5
        @test converged(find_root(identity, 1.0, 1.0; target = 1.0))
        @test find_root(identity, 1.0, 1.0).status === :no_bracket
        @test_throws InterruptException find_root(x -> throw(InterruptException()), 0.0, 1.0)
        @test_throws ArgumentError find_root(identity, NaN, 1.0)
        @test_throws ArgumentError find_root(identity, 0.0, 1.0; max_iter = 0)
        @test_throws ArgumentError find_root(identity, 0.0, 1.0; ftol = -1.0)
    end

    @testset "Lambert includes hyperbolic transfers and respects controls" begin
        # Independent Cartesian RK4 propagation verifies the returned velocities.
        function propagate(r, v, tof)
            x = [r..., v...]
            rhs(x) = [x[4:6]; -MU_EARTH .* x[1:3] ./ norm(x[1:3])^3]
            n = 8000; dt = tof / n
            for _ in 1:n
                k1 = rhs(x); k2 = rhs(x + dt/2 * k1)
                k3 = rhs(x + dt/2 * k2); k4 = rhs(x + dt * k3)
                x += dt/6 * (k1 + 2k2 + 2k3 + k4)
            end
            x
        end
        a = RE_MEAN + 500e3
        r1, r2 = (a, 0.0, 0.0), (0.0, a, 0.0)
        for (tof, long_way) in ((100.0, false), (1000.0, false), (5000.0, true))
            v1, v2 = lambert(r1, r2, tof; long_way = long_way)
            x = propagate(r1, v1, tof)
            @test norm(x[1:3] - collect(r2)) < 1.0
            @test norm(x[4:6] - collect(v2)) < 0.01
        end
        @test_throws ErrorException lambert(r1, r2, 1000.0; max_iter = 1)
        @test_throws ArgumentError lambert(r1, r2, 0.0)
        @test_throws ArgumentError lambert(r1, r1, 1000.0)
    end

    @testset "Monte Carlo footprints cross the dateline" begin
        sample(i, lon; outcome = :splashdown, miss = NaN) =
            MCSample(i, 350.0, 1.0, 1.0, 0.0, 0.0, lon, miss,
                     1000.0, 4.5, 7.0, 4.5e5, 77e6, outcome)
        s = mc_statistics([sample(1, 179.9), sample(2, -179.9)])
        @test s.n_ok == 2 && s.n_fail == 0
        @test abs(abs(s.mean_lon) - 180.0) < 1e-9
        @test 11.0 < s.sigma_major_km < 11.2
        @test s.sigma_minor_km == 0.0
        @test isapprox(s.cep50_km, s.sigma_major_km; atol = 1e-9)
        @test isnan(s.mean_miss_km)
        asymmetric = mc_statistics([sample(1, 179.9), sample(2, -179.8), sample(3, 179.7)])
        @test isapprox(asymmetric.mean_lon, 179.9333333333333; atol = 1e-12)
        for samples in (MCSample[], [sample(1, NaN; outcome = :timeout)])
            s = mc_statistics(samples)
            @test s.n_ok == 0 && s.n_fail == length(samples)
            @test isnan(s.cep50_km) && isnan(s.mean_lon)
        end
        s = mc_statistics([sample(1, 12.0; miss = 3.0)])
        @test s.cep50_km < 1e-10 && s.mean_miss_km == 3.0
    end

    @testset "reentry timeout retains the exact final state" begin
        r, v = state_from_elements(RE_EQ + 400e3, 0.0, 0.5, 0.0, 0.0, 0.3)
        scn = Scenario(vehicle = default_reentry_pod(), r0 = r, v0 = v,
                       t0 = 10.0, t_max = 10.25, dt_orbit = 2.0)
        for fly in (simulate, simulate_entry6)
            res = fly(scn; log_dt_orbit = 1e9)
            @test res.terminated === :timeout
            @test res.log.t[end] == scn.t_max
            @test all(t -> t <= scn.t_max, res.log.t)
        end
        for dt in (0.0, -1.0, Inf, NaN), fly in (simulate, simulate_entry6)
            bad = Scenario(vehicle = scn.vehicle, r0 = r, v0 = v, dt_orbit = dt)
            @test_throws ArgumentError fly(bad)
        end
        @test_throws ArgumentError simulate(scn; log_dt_entry = 0.0)
        # A mission starting under the entry interface can peak at its initial state.
        hot = Scenario(vehicle = scn.vehicle, r0 = (RE_EQ + 50e3, 0.0, 0.0),
                       v0 = (0.0, 7500.0, 0.0), t_max = 0.0)
        res = simulate(hot)
        @test res.peak_gload == res.log.gload[1] > 0
        @test res.peak_qdot == res.log.qdot_conv[1] + res.log.qdot_rad[1] > 0
    end

    @testset "exhausted deorbit targeting returns the flown elements" begin
        veh = default_reentry_pod()
        el, res = target_deorbit(DeorbitElements(), veh; max_iter = 1)
        rerun = simulate(scenario_from_elements(el, veh))
        @test res.lat_splash == rerun.lat_splash
        @test res.lon_splash == rerun.lon_splash
        @test res.t_splash == rerun.t_splash
        @test_throws ArgumentError target_deorbit(el, veh; max_iter = 0)
    end
end
