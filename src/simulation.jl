# Simulation driver: phase scheduling, event detection (entry interface,
# parachute deploys, splashdown) and trajectory logging.

struct FlightEvent
    name::Symbol
    t::Float64
    h::Float64
    mach::Float64
    vrel::Float64
    lat::Float64
    lon::Float64
end

struct TrajectoryLog
    t::Vector{Float64};      h::Vector{Float64}
    lat::Vector{Float64};    lon::Vector{Float64}
    vin::Vector{Float64};    vrel::Vector{Float64}
    gamma::Vector{Float64};  psi::Vector{Float64}
    mach::Vector{Float64};   qbar::Vector{Float64}
    gload::Vector{Float64};  alpha::Vector{Float64}
    qrate::Vector{Float64};  rho::Vector{Float64}
    qdot_conv::Vector{Float64}; qdot_rad::Vector{Float64}
    qload::Vector{Float64};  twall::Vector{Float64}
end
TrajectoryLog() = TrajectoryLog((Float64[] for _ in 1:18)...)

function push_log!(L::TrajectoryLog, d)
    push!(L.t, d.t); push!(L.h, d.h); push!(L.lat, d.lat); push!(L.lon, d.lon)
    push!(L.vin, d.vin); push!(L.vrel, d.vrel); push!(L.gamma, d.gamma); push!(L.psi, d.psi)
    push!(L.mach, d.mach); push!(L.qbar, d.qbar); push!(L.gload, d.gload); push!(L.alpha, d.alpha)
    push!(L.qrate, d.qrate); push!(L.rho, d.rho)
    push!(L.qdot_conv, d.qdot_conv); push!(L.qdot_rad, d.qdot_rad)
    push!(L.qload, d.qload); push!(L.twall, d.twall)
    nothing
end

struct SimResult
    log::TrajectoryLog
    events::Vector{FlightEvent}
    # splashdown summary
    t_splash::Float64
    lat_splash::Float64
    lon_splash::Float64
    v_splash::Float64
    miss_km::Float64          # NaN if scenario has no target
    peak_gload::Float64
    peak_qdot::Float64        # [W/m^2]
    peak_qbar::Float64
    heat_load::Float64        # [J/m^2]
    terminated::Symbol        # :splashdown | :timeout
end

"Geodetic altitude of state x at time t."
function _altitude(x::Vector{Float64}, scn::Scenario, t::Float64)
    theta = earth_rotation_angle(scn.theta_g0, t)
    _, _, h = geodetic_from_ecef(rot_z((x[1], x[2], x[3]), theta))
    h
end

"""
    simulate(scn; log_dt_orbit=5.0, log_dt_entry=0.5) -> SimResult

Run a scenario to splashdown (geodetic altitude 0) or `scn.t_max`.
Logging is decimated to roughly the requested cadences.
"""
function simulate(scn::Scenario; log_dt_orbit::Float64 = 5.0, log_dt_entry::Float64 = 0.5)
    _validate_simulation(scn, log_dt_orbit, log_dt_entry)
    x = initial_state(scn)
    xnew = similar(x)
    w = RK4Work(length(x))
    ctx = FlightContext(length(scn.vehicle.chutes))
    events = FlightEvent[]
    L = TrajectoryLog()

    t = scn.t0
    h = _altitude(x, scn, t)
    ctx.entered = h < scn.h_ei
    next_log = t

    peak_g = 0.0; peak_qdot = 0.0; peak_qbar = 0.0
    terminated = :timeout
    t_sp = NaN; lat_sp = NaN; lon_sp = NaN; v_sp = NaN

    record!(tnow) = begin
        d = flight_data(x, scn, ctx, tnow)
        push_log!(L, d)
        d
    end
    event!(name, tnow) = begin
        d = flight_data(x, scn, ctx, tnow)
        push!(events, FlightEvent(name, tnow, d.h, d.mach, d.vrel, d.lat, d.lon))
        d
    end

    initial = record!(t)
    peak_g = initial.gload
    peak_qdot = initial.qdot_conv + initial.qdot_rad
    peak_qbar = ctx.entered ? initial.qbar : 0.0

    while t < scn.t_max
        chutes_out = any_chute_deployed(ctx)
        dt = !ctx.entered ? scn.dt_orbit : (chutes_out ? scn.dt_descent : scn.dt_entry)
        dt = min(dt, scn.t_max - t)
        t + dt > t || throw(ArgumentError("integration step cannot advance time at this epoch"))

        rk4_step!(xnew, x, t, dt, w, scn, ctx)
        all(isfinite, xnew) || error("non-finite reentry state at t=$(t + dt)")
        hnew = _altitude(xnew, scn, t + dt)

        # --- entry interface crossing: bisect to the EI altitude ------------
        if !ctx.entered && hnew < scn.h_ei
            lo, hi = 0.0, dt
            for _ in 1:30
                mid = 0.5 * (lo + hi)
                rk4_step!(xnew, x, t, mid, w, scn, ctx)
                if _altitude(xnew, scn, t + mid) > scn.h_ei
                    lo = mid
                else
                    hi = mid
                end
                hi - lo < 1e-4 && break
            end
            rk4_step!(xnew, x, t, hi, w, scn, ctx)
            copyto!(x, xnew); t += hi
            ctx.entered = true
            event!(:entry_interface, t)
            record!(t); next_log = t + log_dt_entry
            continue
        end

        # --- splashdown: bisect to h = 0 ------------------------------------
        if hnew <= 0.0
            lo, hi = 0.0, dt
            for _ in 1:40
                mid = 0.5 * (lo + hi)
                rk4_step!(xnew, x, t, mid, w, scn, ctx)
                if _altitude(xnew, scn, t + mid) > 0.0
                    lo = mid
                else
                    hi = mid
                end
                hi - lo < 1e-5 && break
            end
            rk4_step!(xnew, x, t, hi, w, scn, ctx)
            copyto!(x, xnew); t += hi
            d = event!(:splashdown, t)
            record!(t)
            terminated = :splashdown
            t_sp = t; lat_sp = d.lat; lon_sp = d.lon; v_sp = d.vrel
            break
        end

        copyto!(x, xnew); t += dt

        # --- parachute deployment triggers ----------------------------------
        if ctx.entered
            d = flight_data(x, scn, ctx, t)
            peak_g = max(peak_g, d.gload)
            peak_qdot = max(peak_qdot, d.qdot_conv + d.qdot_rad)
            peak_qbar = max(peak_qbar, d.qbar)
            for (i, c) in enumerate(scn.vehicle.chutes)
                if isnan(ctx.chute_deploy_t[i]) && d.mach < c.mach_max && d.h < c.alt_max
                    ctx.chute_deploy_t[i] = t
                    push!(events, FlightEvent(Symbol(:deploy_, c.name), t, d.h, d.mach, d.vrel, d.lat, d.lon))
                end
            end
            if t >= next_log
                push_log!(L, d)
                next_log += log_dt_entry
            end
        elseif t >= next_log
            record!(t)
            next_log += log_dt_orbit
        end
    end

    # Even a metrics-only run retains its actual final state on timeout.
    if terminated == :timeout && L.t[end] != t
        record!(t)
    end

    miss = if !isnan(scn.target_lat) && terminated == :splashdown
        haversine(lat_sp, lon_sp, scn.target_lat, scn.target_lon) / 1000
    else
        NaN
    end

    SimResult(L, events, t_sp, lat_sp, lon_sp, v_sp, miss,
              peak_g, peak_qdot, peak_qbar, x[9], terminated)
end
