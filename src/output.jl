# CSV output and console summaries (stdlib-only; no DataFrames dependency).

function write_csv(path::AbstractString, cols::NamedTuple)
    names_ = collect(string.(keys(cols)))
    vecs = collect(values(cols))
    n = length(vecs[1])
    all(length(v) == n for v in vecs) || throw(ArgumentError("column length mismatch"))
    open(path, "w") do io
        println(io, join(names_, ","))
        for i in 1:n
            print(io, vecs[1][i])
            for j in 2:length(vecs)
                print(io, ',', vecs[j][i])
            end
            print(io, '\n')
        end
    end
    path
end

function write_trajectory_csv(path::AbstractString, res::SimResult)
    L = res.log
    write_csv(path, (
        t_s = L.t, alt_m = L.h,
        lat_deg = rad2deg_.(L.lat), lon_deg = rad2deg_.(L.lon),
        v_inertial_ms = L.vin, v_rel_ms = L.vrel,
        gamma_deg = rad2deg_.(L.gamma), heading_deg = rad2deg_.(L.psi),
        mach = L.mach, qbar_pa = L.qbar, gload = L.gload,
        alpha_deg = rad2deg_.(L.alpha), pitch_rate_dps = rad2deg_.(L.qrate),
        rho_kgm3 = L.rho,
        qdot_conv_wcm2 = L.qdot_conv ./ 1e4, qdot_rad_wcm2 = L.qdot_rad ./ 1e4,
        heat_load_jcm2 = L.qload ./ 1e4, t_wall_k = L.twall,
    ))
end

function write_events_csv(path::AbstractString, res::SimResult)
    ev = res.events
    write_csv(path, (
        event = [string(e.name) for e in ev],
        t_s = [e.t for e in ev],
        alt_m = [e.h for e in ev],
        mach = [e.mach for e in ev],
        v_rel_ms = [e.vrel for e in ev],
        lat_deg = [rad2deg_(e.lat) for e in ev],
        lon_deg = [rad2deg_(e.lon) for e in ev],
    ))
end

function write_montecarlo_csv(path::AbstractString, samples::Vector{MCSample})
    write_csv(path, (
        run = [s.run for s in samples],
        mass_kg = [s.mass for s in samples],
        cd_mult = [s.cd_mult for s in samples],
        rho_mult = [s.rho_mult for s in samples],
        trim_alpha_deg = [s.trim_alpha_deg for s in samples],
        lat_deg = [s.lat_deg for s in samples],
        lon_deg = [s.lon_deg for s in samples],
        miss_km = [s.miss_km for s in samples],
        t_flight_s = [s.t_flight for s in samples],
        v_splash_ms = [s.v_splash for s in samples],
        peak_gload = [s.peak_gload for s in samples],
        peak_qdot_wcm2 = [s.peak_qdot / 1e4 for s in samples],
        heat_load_jcm2 = [s.heat_load / 1e4 for s in samples],
        terminated = [string(s.terminated) for s in samples],
    ))
end

function print_summary(io::IO, res::SimResult, scn::Scenario)
    ei = findfirst(e -> e.name == :entry_interface, res.events)
    println(io, "== Flight summary ==")
    if ei !== nothing
        e = res.events[ei]
        @printf(io, "  Entry interface : t=%.1f s  V_rel=%.1f m/s  Mach %.1f  lat=%.2f°  lon=%.2f°\n",
                e.t, e.vrel, e.mach, rad2deg_(e.lat), rad2deg_(e.lon))
    end
    for e in res.events
        startswith(string(e.name), "deploy") || continue
        @printf(io, "  %-15s : t=%.1f s  h=%.2f km  Mach %.2f  V=%.1f m/s\n",
                e.name, e.t, e.h / 1000, e.mach, e.vrel)
    end
    if res.terminated == :splashdown
        @printf(io, "  Splashdown      : t=%.1f s  lat=%.3f°  lon=%.3f°  V=%.1f m/s\n",
                res.t_splash, rad2deg_(res.lat_splash), rad2deg_(res.lon_splash), res.v_splash)
        isnan(res.miss_km) ||
            @printf(io, "  Miss distance   : %.1f km  (target %.2f°, %.2f°)\n",
                    res.miss_km, rad2deg_(scn.target_lat), rad2deg_(scn.target_lon))
    else
        println(io, "  DID NOT SPLASH DOWN (", res.terminated, ")")
    end
    @printf(io, "  Peak g-load     : %.2f g\n", res.peak_gload)
    @printf(io, "  Peak q_dot      : %.1f W/cm²  (T_wall ≈ %.0f K)\n",
            res.peak_qdot / 1e4, wall_temperature(res.peak_qdot, scn.vehicle.emissivity))
    @printf(io, "  Peak dyn. press.: %.1f kPa\n", res.peak_qbar / 1000)
    @printf(io, "  Heat load       : %.1f MJ/m²\n", res.heat_load / 1e6)
end
print_summary(res::SimResult, scn::Scenario) = print_summary(stdout, res, scn)
