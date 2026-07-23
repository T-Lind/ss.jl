# Launch-to-orbit: 3-DOF + mass ascent dynamics over the rotating Earth.
#
# State vector (7): [r_eci(3); v_eci(3); m].
#
# Guidance is the classic sounding sequence:
#   1. vertical rise off the pad until `v_pitchover` (atmosphere-relative),
#   2. constant pitch-over kick of `kick_angle` toward the launch azimuth for
#      `kick_duration` seconds,
#   3. gravity turn — thrust along the atmosphere-relative velocity — for the
#      rest of stage 1 (keeps alpha ~ 0 through max-q, as real boosters do),
#   4. exoatmospheric closed-form steering for the upper stage(s): a
#      linear-tangent pitch law tan(theta) = tan(pitch0) - pitch_rate * t
#      in the instantaneous orbital plane, cut off when the orbit energy
#      reaches the target circular energy.
#
# `tune_ascent` closes the loop: a 2x2 damped-Newton on (pitch0, pitch_rate)
# drives (h, gamma) at cutoff to (h_target, 0) — a shooting method standing
# in for a real PEG-class guidance algorithm (see README extension points).
#
# Staging: stage k burns to depletion (or cutoff for the last exoatmospheric
# burn), its dry mass separates, the next stage ignites after `stage_gap`.
# The fairing jettisons at `fairing_alt`.

Base.@kwdef struct AscentGuidance
    site_lat::Float64 = deg2rad_(28.5)     # Cape-like launch site
    site_lon::Float64 = deg2rad_(-80.6)
    azimuth::Float64 = deg2rad_(90.0)      # launch azimuth [rad], from North toward East
    v_pitchover::Float64 = 40.0            # [m/s] relative speed to start the kick
    kick_angle::Float64 = deg2rad_(8.0)    # pitch-over tilt from vertical
    kick_duration::Float64 = 14.0          # [s]
    pitch0::Float64 = deg2rad_(26.0)       # linear-tangent initial pitch (above horizon)
    pitch_rate::Float64 = 1.45e-3          # d(tan theta)/dt [1/s]
    h_target::Float64 = 200.0e3            # parking-orbit altitude [m]
    fairing_alt::Float64 = 120.0e3         # fairing jettison altitude [m]
    stage_gap::Float64 = 4.0               # coast between stages [s]
end

"Launch azimuth [rad] that yields inclination `inc` from latitude `lat` (prograde)."
launch_azimuth(inc::Float64, lat::Float64) =
    asin(clamp(cos(inc) / cos(lat), -1.0, 1.0))

mutable struct AscentCtx
    stage::Int                # index of the currently-burning stage (0 = none)
    phase::Symbol             # :prelaunch | :vertical | :kick | :gravity_turn | :closed_loop | :coast
    t_stage_ign::Float64      # ignition time of the current stage
    t_kick0::Float64
    t_loop0::Float64          # closed-loop steering start time
    fairing_on::Bool
    burning::Bool
end

struct AscentEvent
    name::Symbol
    t::Float64
    h::Float64
    vrel::Float64
    m::Float64
end

struct AscentLog
    t::Vector{Float64}
    rx::Vector{Float64}; ry::Vector{Float64}; rz::Vector{Float64}
    h::Vector{Float64};  vrel::Vector{Float64}; vin::Vector{Float64}
    gamma::Vector{Float64}; mach::Vector{Float64}; qbar::Vector{Float64}
    m::Vector{Float64}; thrust::Vector{Float64}
    lat::Vector{Float64}; lon::Vector{Float64}; downrange::Vector{Float64}
end
AscentLog() = AscentLog((Float64[] for _ in 1:15)...)

struct AscentResult
    log::AscentLog
    events::Vector{AscentEvent}
    r::V3; v::V3; m::Float64; t::Float64
    elements::NamedTuple
    prop_left::Vector{Float64}     # per stage [kg]
    reached_orbit::Bool
    h_cut::Float64                 # altitude at cutoff [m]
    gamma_cut::Float64             # inertial flight-path angle at cutoff [rad]
end

"Commanded thrust direction (unit, ECI) for the current guidance phase."
function _steer(guid::AscentGuidance, ctx::AscentCtx, r::V3, v::V3, t::Float64,
                theta_g0::Float64)
    rhat = vunit(r)
    if ctx.phase === :vertical
        return rhat
    end
    omega = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega, r))
    if ctx.phase === :kick
        # tilt from vertical toward the azimuth direction (ENU at current spot)
        theta = earth_rotation_angle(theta_g0, t)
        lat, lon, _ = geodetic_from_ecef(rot_z(r, theta))
        eE, eN, _ = enu_basis(lat, lon)
        az_ecef = vadd(vscale(eN, cos(guid.azimuth)), vscale(eE, sin(guid.azimuth)))
        az_eci = rot_z(az_ecef, -theta)              # ECEF -> ECI
        az_h = vunit(vsub(az_eci, vscale(rhat, vdot(az_eci, rhat))))
        sk, ck = sincos(guid.kick_angle)
        return vadd(vscale(rhat, ck), vscale(az_h, sk))
    elseif ctx.phase === :gravity_turn
        return vunit(vrel)
    else # :closed_loop — linear tangent in the instantaneous orbital plane
        that = vunit(vsub(v, vscale(rhat, vdot(v, rhat))))   # horizontal along-track (inertial)
        tt = tan(guid.pitch0) - guid.pitch_rate * (t - ctx.t_loop0)
        th = atan(tt)
        return vadd(vscale(that, cos(th)), vscale(rhat, sin(th)))
    end
end

"Ascent state derivative. `st` is the burning stage (or `nothing` while coasting)."
function _ascent_deriv!(dx::Vector{Float64}, x::Vector{Float64},
                        lv::LaunchVehicle, guid::AscentGuidance, ctx::AscentCtx,
                        atm::AbstractAtmosphere, grav::AbstractGravity,
                        theta_g0::Float64, t::Float64)
    r = (x[1], x[2], x[3])
    v = (x[4], x[5], x[6])
    m = x[7]

    a = gravity_accel(grav, r, t)
    theta = earth_rotation_angle(theta_g0, t)
    _, _, h = geodetic_from_ecef(rot_z(r, theta))

    dm = 0.0
    thrust_mag = 0.0
    pamb = 0.0
    rho = 0.0; asnd = 300.0
    if h < 150.0e3
        rho, _, pamb, asnd = atmosphere_state(atm, max(h, 0.0))
    end

    if ctx.burning && ctx.stage >= 1
        st = lv.stages[ctx.stage]
        thrust_mag = stage_thrust(st, pamb)
        dm = -stage_mdot(st)
        dhat = _steer(guid, ctx, r, v, t, theta_g0)
        a = vadd(a, vscale(dhat, thrust_mag / m))
    end

    # aerodynamic drag on the stack (relative wind)
    if rho > 0
        omega = (0.0, 0.0, OMEGA_EARTH)
        vrel = vsub(v, vcross(omega, r))
        Vr = vnorm(vrel)
        if Vr > 1.0
            M = Vr / asnd
            D = 0.5 * rho * Vr * Vr * lv.sref * interp1(lv.cd, M)
            a = vadd(a, vscale(vrel, -D / (m * Vr)))
        end
    end

    dx[1] = v[1]; dx[2] = v[2]; dx[3] = v[3]
    dx[4] = a[1]; dx[5] = a[2]; dx[6] = a[3]
    dx[7] = dm
    nothing
end

function _rk4_ascent!(xo, x, t, dt, w, lv, guid, ctx, atm, grav, th0)
    n = length(x)
    _ascent_deriv!(w.k1, x, lv, guid, ctx, atm, grav, th0, t)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k1[i]; end
    _ascent_deriv!(w.k2, w.xt, lv, guid, ctx, atm, grav, th0, t + 0.5dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k2[i]; end
    _ascent_deriv!(w.k3, w.xt, lv, guid, ctx, atm, grav, th0, t + dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + dt * w.k3[i]; end
    _ascent_deriv!(w.k4, w.xt, lv, guid, ctx, atm, grav, th0, t + dt)
    @inbounds for i in 1:n
        xo[i] = x[i] + (dt / 6) * (w.k1[i] + 2w.k2[i] + 2w.k3[i] + w.k4[i])
    end
    nothing
end

"""
    simulate_ascent(lv, guid; atmosphere, gravity, theta_g0, dt, log_dt)
        -> AscentResult

Fly the launch vehicle from the pad to parking-orbit insertion (or propellant
depletion). The last stage that ignites exoatmospherically flies the
linear-tangent law and cuts off at the target circular-orbit energy; unspent
propellant in the final stage is reported for the trans-lunar injection.
"""
function simulate_ascent(lv::LaunchVehicle, guid::AscentGuidance;
                         atmosphere::AbstractAtmosphere = USSA76(),
                         gravity::AbstractGravity = J2Gravity(),
                         theta_g0::Float64 = 0.0,
                         dt::Float64 = 0.10, log_dt::Float64 = 1.0,
                         t_max::Float64 = 2.0e3)
    # initial state: on the pad, moving with the Earth
    r_ecef = ecef_from_geodetic(guid.site_lat, guid.site_lon, 0.0)
    r0 = rot_z(r_ecef, -theta_g0)                      # ECEF -> ECI at t=0
    omega = (0.0, 0.0, OMEGA_EARTH)
    v0 = vcross(omega, r0)
    x = [r0[1], r0[2], r0[3], v0[1], v0[2], v0[3], liftoff_mass(lv)]
    xnew = similar(x)
    w = RK4Work(7)

    nst = length(lv.stages)
    prop_left = [s.mprop for s in lv.stages]
    ctx = AscentCtx(1, :vertical, 0.0, NaN, NaN, true, true)
    events = AscentEvent[]
    L = AscentLog()
    r_site0 = r0

    a_t = RE_MEAN + guid.h_target
    e_target = -MU_EARTH / (2 * a_t)                  # circular energy at h_target

    t = 0.0
    next_log = 0.0
    reached = false
    h_cut = NaN; gam_cut = NaN
    ev!(name) = begin
        d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0)
        push!(events, AscentEvent(name, t, d.h, d.vrel, x[7]))
        d
    end
    logrec!(d) = begin
        push!(L.t, t); push!(L.rx, x[1]); push!(L.ry, x[2]); push!(L.rz, x[3])
        push!(L.h, d.h); push!(L.vrel, d.vrel); push!(L.vin, d.vin)
        push!(L.gamma, d.gamma); push!(L.mach, d.mach); push!(L.qbar, d.qbar)
        push!(L.m, x[7]); push!(L.thrust, d.thrust)
        push!(L.lat, d.lat); push!(L.lon, d.lon); push!(L.downrange, d.downrange)
    end
    ev!(:liftoff)

    burn_elapsed(st) = t - ctx.t_stage_ign

    while t < t_max
        # --- phase transitions ------------------------------------------------
        d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0)
        if ctx.phase === :vertical && d.vrel >= guid.v_pitchover
            ctx.phase = :kick; ctx.t_kick0 = t
            ev!(:pitchover)
        elseif ctx.phase === :kick && t - ctx.t_kick0 >= guid.kick_duration
            ctx.phase = :gravity_turn
            ev!(:gravity_turn)
        end
        if ctx.fairing_on && d.h >= guid.fairing_alt
            ctx.fairing_on = false
            x[7] -= lv.fairing_mass
            ev!(:fairing_jettison)
        end

        # --- burnout / staging / cutoff --------------------------------------
        if ctx.burning
            st = lv.stages[ctx.stage]
            tb = stage_burn_time(st)
            if burn_elapsed(st) >= tb - 1e-9
                prop_left[ctx.stage] = 0.0
                x[7] -= st.mdry                       # drop the spent stage
                ev!(Symbol(:sep_, st.name))
                if ctx.stage < nst
                    ctx.stage += 1
                    ctx.burning = false               # short inter-stage coast
                    ctx.t_stage_ign = t + guid.stage_gap
                else
                    ctx.stage = 0; ctx.burning = false
                    ctx.phase = :coast
                    h_cut = d.h; gam_cut = d.gamma
                    ev!(:propellant_depletion)
                    break
                end
            end
            # exoatmospheric closed-loop cutoff at target energy
            if ctx.burning && ctx.phase === :closed_loop
                eps_now = 0.5 * d.vin^2 - MU_EARTH / vnorm((x[1], x[2], x[3]))
                if eps_now >= e_target
                    st = lv.stages[ctx.stage]
                    prop_left[ctx.stage] -= stage_mdot(st) * burn_elapsed(st)
                    ctx.burning = false; ctx.phase = :coast
                    reached = true
                    h_cut = d.h; gam_cut = d.gamma
                    ev!(:seco)
                    break
                end
            end
        elseif ctx.stage >= 1 && ctx.stage <= nst && t >= ctx.t_stage_ign
            ctx.burning = true
            ctx.phase = :closed_loop                  # upper stages steer closed-loop
            isnan(ctx.t_loop0) && (ctx.t_loop0 = t)
            ev!(Symbol(:ignition_, lv.stages[ctx.stage].name))
        end

        if t >= next_log
            logrec!(d)
            next_log += log_dt
        end

        _rk4_ascent!(xnew, x, t, dt, w, lv, guid, ctx, atmosphere, gravity, theta_g0)
        copyto!(x, xnew); t += dt
    end
    # final log point
    d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0)
    logrec!(d)

    r = (x[1], x[2], x[3]); v = (x[4], x[5], x[6])
    el = elements_from_state(r, v)
    AscentResult(L, events, r, v, x[7], t, el, prop_left, reached, h_cut, gam_cut)
end

function _ascent_data(x, lv::LaunchVehicle, ctx::AscentCtx, theta_g0, t, r_site0)
    r = (x[1], x[2], x[3]); v = (x[4], x[5], x[6])
    theta = earth_rotation_angle(theta_g0, t)
    lat, lon, h = geodetic_from_ecef(rot_z(r, theta))
    omega = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega, r))
    Vr = vnorm(vrel)
    rho = 0.0; asnd = 300.0; pamb = 0.0
    if h < 150e3
        rho, _, pamb, asnd = atmosphere_state(USSA76(), max(h, 0.0))
    end
    qbar = 0.5 * rho * Vr * Vr
    rhat = vunit(r)
    vin = vnorm(v)
    gamma = vin > 1 ? asin(clamp(vdot(rhat, vscale(v, 1 / vin)), -1.0, 1.0)) : pi / 2
    thrust = ctx.burning && ctx.stage >= 1 ? stage_thrust(lv.stages[ctx.stage], pamb) : 0.0
    # downrange: great-circle from the launch site's inertial position
    dr = RE_MEAN * acos(clamp(vdot(vunit(r_site0), rhat), -1.0, 1.0))
    (h = h, vrel = Vr, vin = vin, gamma = gamma, mach = Vr / asnd, qbar = qbar,
     lat = lat, lon = lon, thrust = thrust, downrange = dr)
end

"""
    tune_ascent(lv, guid; tol_h=1.0e3, tol_gamma=deg2rad_(0.05), max_iter=12)
        -> (guid_tuned, result)

Damped-Newton shooting on (pitch0, pitch_rate) of the linear-tangent law,
driving cutoff altitude and flight-path angle to (h_target, 0).
"""
function tune_ascent(lv::LaunchVehicle, guid::AscentGuidance;
                     tol_h::Float64 = 1.0e3, tol_gamma::Float64 = deg2rad_(0.05),
                     max_iter::Int = 12, verbose::Bool = false, kwargs...)
    p1, p2 = guid.pitch0, guid.pitch_rate
    local res
    resid(g) = begin
        r = simulate_ascent(lv, g; kwargs...)
        (r.h_cut - g.h_target, r.gamma_cut, r)
    end
    rebuild(p1, p2) = AscentGuidance(guid.site_lat, guid.site_lon, guid.azimuth,
        guid.v_pitchover, guid.kick_angle, guid.kick_duration,
        p1, p2, guid.h_target, guid.fairing_alt, guid.stage_gap)
    for it in 1:max_iter
        g = rebuild(p1, p2)
        f1, f2, res = resid(g)
        verbose && @info "tune_ascent" it dh_km = f1/1e3 gamma_deg = rad2deg_(f2) p1_deg = rad2deg_(p1) p2 = p2
        (abs(f1) < tol_h && abs(f2) < tol_gamma && res.reached_orbit) &&
            return (g, res)
        d1 = deg2rad_(0.4); d2 = 5.0e-5
        f1a, f2a, _ = resid(rebuild(p1 + d1, p2))
        f1b, f2b, _ = resid(rebuild(p1, p2 + d2))
        j11 = (f1a - f1) / d1; j21 = (f2a - f2) / d1
        j12 = (f1b - f1) / d2; j22 = (f2b - f2) / d2
        det = j11 * j22 - j12 * j21
        abs(det) < 1e-12 && break
        dp1 = -( j22 * f1 - j12 * f2) / det
        dp2 = -(-j21 * f1 + j11 * f2) / det
        # damped, clamped update
        lim1 = deg2rad_(6.0); lim2 = 6.0e-4
        p1 += clamp(0.8 * dp1, -lim1, lim1)
        p2 += clamp(0.8 * dp2, -lim2, lim2)
    end
    g = rebuild(p1, p2)
    _, _, res = resid(g)
    (g, res)
end
