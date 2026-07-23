# Monte Carlo dispersion analysis.
#
# Each sample perturbs vehicle, aerodynamic, atmospheric and initial-state
# parameters, reruns the full trajectory, and records the splashdown point
# and flight metrics. Runs are threaded (`julia -t auto`) and individually
# seeded, so results are reproducible regardless of thread count.

Base.@kwdef struct Dispersions
    mass_frac::Float64 = 0.03          # 1σ fractional mass error ("above or below target")
    cd_frac::Float64 = 0.05            # 1σ fractional drag-coefficient error
    cma_frac::Float64 = 0.05           # 1σ fractional pitch-stiffness error
    trim_alpha_sigma::Float64 = deg2rad_(0.5)  # 1σ trim-alpha (CG offset) error
    rho_frac::Float64 = 0.10           # 1σ lognormal density multiplier spread
    pos_sigma::Float64 = 1000.0        # 1σ per-axis initial position error [m]
    vel_sigma::Float64 = 2.0           # 1σ per-axis initial velocity error [m/s] (burn execution)
    chute_cda_frac::Float64 = 0.08     # 1σ fractional parachute drag-area error
    alpha0_sigma::Float64 = deg2rad_(2.0)  # 1σ initial attitude error
end

struct MCSample
    run::Int
    mass::Float64
    cd_mult::Float64
    rho_mult::Float64
    trim_alpha_deg::Float64
    lat_deg::Float64
    lon_deg::Float64
    miss_km::Float64
    t_flight::Float64
    v_splash::Float64
    peak_gload::Float64
    peak_qdot::Float64      # [W/m^2]
    heat_load::Float64      # [J/m^2]
    terminated::Symbol
end

function perturbed_scenario(scn::Scenario, disp::Dispersions, rng::AbstractRNG)
    veh = scn.vehicle
    mass = veh.mass * (1 + disp.mass_frac * randn(rng))
    aero = scaled_aero(veh.aero;
                       cd_mult = 1 + disp.cd_frac * randn(rng),
                       cma_mult = 1 + disp.cma_frac * randn(rng),
                       alpha_trim_delta = disp.trim_alpha_sigma * randn(rng))
    chutes = [Parachute(c.name, c.cda * (1 + disp.chute_cda_frac * randn(rng)),
                        c.mach_max, c.alt_max, c.fill_time) for c in veh.chutes]
    veh2 = Vehicle(; name = veh.name, mass = mass, sref = veh.sref, lref = veh.lref,
                   rn = veh.rn, iyy = veh.iyy, emissivity = veh.emissivity,
                   aero = aero, chutes = chutes)
    rho_mult = exp(disp.rho_frac * randn(rng))
    r0 = vadd(scn.r0, (disp.pos_sigma * randn(rng), disp.pos_sigma * randn(rng),
                       disp.pos_sigma * randn(rng)))
    v0 = vadd(scn.v0, (disp.vel_sigma * randn(rng), disp.vel_sigma * randn(rng),
                       disp.vel_sigma * randn(rng)))
    scn2 = Scenario(; vehicle = veh2,
                    atmosphere = ScaledAtmosphere(scn.atmosphere, rho_mult),
                    gravity = scn.gravity,
                    r0 = r0, v0 = v0,
                    alpha0 = scn.alpha0 + disp.alpha0_sigma * randn(rng),
                    t0 = scn.t0, theta_g0 = scn.theta_g0, h_ei = scn.h_ei,
                    bank = scn.bank, extra_accel = scn.extra_accel,
                    target_lat = scn.target_lat, target_lon = scn.target_lon,
                    dt_orbit = scn.dt_orbit, dt_entry = scn.dt_entry,
                    dt_descent = scn.dt_descent, t_max = scn.t_max)
    (scn2, mass, aero, rho_mult)
end

"""
    run_montecarlo(scn, disp; n=300, seed=2026) -> Vector{MCSample}

Threaded Monte Carlo around nominal scenario `scn`.
"""
function run_montecarlo(scn::Scenario, disp::Dispersions = Dispersions();
                        n::Int = 300, seed::Int = 2026)
    out = Vector{MCSample}(undef, n)
    Threads.@threads for i in 1:n
        rng = Xoshiro(seed + i)
        scn2, mass, aero, rho_mult = perturbed_scenario(scn, disp, rng)
        res = simulate(scn2; log_dt_orbit = 1e9, log_dt_entry = 1e9)  # metrics only
        out[i] = MCSample(i, mass,
                          scn2.vehicle.aero.cd0.y[end] / scn.vehicle.aero.cd0.y[end],
                          rho_mult,
                          rad2deg_(scn2.vehicle.aero.alpha_trim),
                          rad2deg_(res.lat_splash), rad2deg_(res.lon_splash),
                          res.miss_km, res.t_splash, res.v_splash,
                          res.peak_gload, res.peak_qdot, res.heat_load, res.terminated)
    end
    out
end

"""
    mc_statistics(samples) -> NamedTuple

Splashdown dispersion statistics: mean point, miss stats, CEP50, and the
1σ covariance ellipse of the lat/lon scatter (semi-axes in km + orientation).
"""
function mc_statistics(samples::Vector{MCSample})
    ok = [s for s in samples if s.terminated == :splashdown]
    lat = [s.lat_deg for s in ok]; lon = [s.lon_deg for s in ok]
    miss = [s.miss_km for s in ok]
    mlat, mlon = mean(lat), mean(lon)
    # local km per degree at the mean latitude
    kx = RE_MEAN * pi / 180 * cosd(mlat) / 1000   # per deg lon
    ky = RE_MEAN * pi / 180 / 1000                # per deg lat
    dx = (lon .- mlon) .* kx
    dy = (lat .- mlat) .* ky
    cxx, cyy, cxy = mean(dx .^ 2), mean(dy .^ 2), mean(dx .* dy)
    tr = cxx + cyy; det_ = cxx * cyy - cxy^2
    l1 = tr / 2 + sqrt(max(0.0, (tr / 2)^2 - det_))
    l2 = tr / 2 - sqrt(max(0.0, (tr / 2)^2 - det_))
    theta = 0.5 * atan(2cxy, cxx - cyy)
    # radial miss from the *mean* point (dispersion, excludes nominal bias)
    r = sqrt.(dx .^ 2 + dy .^ 2)
    (n_ok = length(ok), n_fail = length(samples) - length(ok),
     mean_lat = mlat, mean_lon = mlon,
     mean_miss_km = mean(miss), max_miss_km = maximum(miss),
     cep50_km = median(r), r95_km = quantile(sort(r), 0.95),
     sigma_major_km = sqrt(max(0.0, l1)), sigma_minor_km = sqrt(max(0.0, l2)),
     ellipse_angle_deg = rad2deg_(theta),
     mean_peak_g = mean(s.peak_gload for s in ok),
     mean_heat_MJm2 = mean(s.heat_load for s in ok) / 1e6)
end
