# Full 6-DOF entry: quaternion attitude, Euler rotational dynamics, and
# total-angle-of-attack aerodynamics for an axisymmetric capsule.
#
# State vector (15):
#   x[1:3]   r     ECI position [m]
#   x[4:6]   v     ECI velocity [m/s]
#   x[7:10]  q     body->ECI attitude quaternion
#   x[11:13] ω     body rates [rad/s]
#   x[14]    Q     stagnation heat load [J/m^2]
#   x[15]    m_rcs RCS propellant used [kg]
#
# Body frame: +x is the symmetry axis, pointing along the velocity at trim
# (heatshield leading). The existing Mach tables are reused through the
# total-AoA formulation: alpha_t is the angle between the relative wind and
# the symmetry axis; the restoring moment acts about the crossflow axis and
# the transverse rates see Cm_q damping. Roll carries a small friction-level
# damping (an axisymmetric blunt body generates no significant roll torque).
#
# The 4-DOF pitch-plane simulation remains the fast default; this model is
# its superset and the two are cross-validated in the test suite. RCS rate
# damping (an on/off law on the hardware in `rcs.jl`) is active while
# dynamic pressure is still negligible — exactly how a real pod nulls tipoff
# rates before entry interface — and draws real propellant.

"""
Rate-damping time constant [s]: the PWM law runs at full duty against any
body rate it could not null within this long, and scales the duty down below
it. Multiplied by an angular acceleration (`torque/inertia`) to get the
saturation RATE, so it is seconds and not a dimensionless gain.
"""
const RATE_NULL_S = 2.0

struct Entry6Log
    t::Vector{Float64};  h::Vector{Float64}
    lat::Vector{Float64}; lon::Vector{Float64}
    vrel::Vector{Float64}; mach::Vector{Float64}; qbar::Vector{Float64}
    gload::Vector{Float64}; alpha_t::Vector{Float64}
    wx::Vector{Float64}; wy::Vector{Float64}; wz::Vector{Float64}
    qdot::Vector{Float64}; qload::Vector{Float64}; twall::Vector{Float64}
    rcs_used::Vector{Float64}
end
Entry6Log() = Entry6Log((Float64[] for _ in 1:16)...)

struct Entry6Result
    log::Entry6Log
    events::Vector{FlightEvent}
    t_splash::Float64
    lat_splash::Float64
    lon_splash::Float64
    v_splash::Float64
    peak_gload::Float64
    peak_qdot::Float64
    peak_qbar::Float64
    heat_load::Float64
    rcs_used::Float64          # [kg]
    max_alpha_after_peak::Float64  # envelope check [rad]
    terminated::Symbol
end

"Extract Cm_q(M) from the aero database via the generic interface."
@inline _cmq_of(aero, M) =
    cm_coeff(aero, M, aero isa CapsuleAero ? aero.alpha_trim : 0.0, 1.0) -
    cm_coeff(aero, M, aero isa CapsuleAero ? aero.alpha_trim : 0.0, 0.0)

function _entry6_deriv!(dx, x, scn::Scenario, ctx::FlightContext,
                        inertia::V3, rcs::Union{Nothing,RCSystem},
                        rate_db::Float64, rcs_mode::Symbol, t::Float64)
    r = (x[1], x[2], x[3])
    v = (x[4], x[5], x[6])
    q = qnormalize((x[7], x[8], x[9], x[10]))
    w = (x[11], x[12], x[13])
    veh = scn.vehicle

    theta = earth_rotation_angle(scn.theta_g0, t)
    _, _, h = geodetic_from_ecef(rot_z(r, theta))

    a = gravity_accel(scn.gravity, r, t)
    a = vadd(a, scn.extra_accel(r, v, t)::V3)

    M_b = (0.0, 0.0, 0.0)
    qdot_heat = 0.0
    dmrcs = 0.0

    omega_e = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega_e, r))
    Vr = vnorm(vrel)
    qbar = 0.0

    if h < scn.h_ei && Vr > 1.0
        rho, _, _, asnd = atmosphere_state(scn.atmosphere, h)
        if rho > 0
            qbar = 0.5 * rho * Vr * Vr
            Mn = Vr / asnd
            vhat = vscale(vrel, 1 / Vr)
            xhat = qrotate(q, (1.0, 0.0, 0.0))       # symmetry axis, ECI
            ca = clamp(vdot(vhat, xhat), -1.0, 1.0)
            alpha_t = acos(ca)

            cda_ch = 0.0
            for (i, c) in enumerate(veh.chutes)
                td = ctx.chute_deploy_t[i]
                isnan(td) || (cda_ch += c.cda * chute_fill(c, t - td))
            end
            chutes_out = any_chute_deployed(ctx)

            D = qbar * (veh.sref * cd_coeff(veh.aero, Mn, alpha_t) + cda_ch)
            f = vscale(vhat, -D)
            if !chutes_out
                # lift in the total-AoA plane, along the off-wind body-axis component
                uax = vsub(xhat, vscale(vhat, ca))
                un = vnorm(uax)
                if un > 1e-9
                    L = qbar * veh.sref * cl_coeff(veh.aero, Mn, alpha_t)
                    f = vadd(f, vscale(uax, L / un))
                end
                # moments: restoring about the crossflow axis + transverse damping
                vb = qrotate_inv(q, vhat)
                e = vcross(vb, (1.0, 0.0, 0.0))
                en = vnorm(e)
                if en > 1e-9
                    Cm = cm_coeff(veh.aero, Mn, alpha_t, 0.0)
                    M_b = vadd(M_b, vscale(e, qbar * veh.sref * veh.lref * Cm / en))
                end
                # --- roll control: hold the commanded bank angle ----------
                # Lift points along the off-wind body-axis component, so its
                # angle about the wind IS the bank angle. That axis has no
                # aerodynamic restoring moment on an axisymmetric capsule —
                # pitch and yaw self-trim, roll does not — which makes it the
                # one attitude freedom worth spending propellant on, and the
                # reason a real capsule carries roll jets.
                if rcs !== nothing && rcs_mode === :bank_hold &&
                   un > 1e-9 && x[15] < rcs.prop
                    rhat_ = vunit(r)
                    sg = clamp(vdot(rhat_, vhat), -1.0, 1.0)
                    uh = vsub(rhat_, vscale(vhat, sg))
                    if vnorm(uh) > 1e-9
                        uh = vunit(uh)
                        sh = vcross(vhat, uh)              # right-handed with v, up
                        lh = vscale(uax, 1 / un)           # where lift actually points
                        phi = atan(vdot(lh, sh), vdot(lh, uh))
                        gl = sqrt(D * D + L * L) / (veh.mass * G0)
                        phi_c = bank_command(scn.bank, t, h, Vr, gl)
                        e_phi = rem(phi_c - phi, 2pi, RoundNearest)
                        # roll rate about the WIND, not the body axis: that is
                        # the rate that actually moves the lift vector
                        vb_ = qrotate_inv(q, vhat)
                        p_wind = vdot(w, vb_)
                        auth1 = torque_authority(rcs)[1]
                        wcmd = clamp(0.6 * e_phi, -deg2rad_(20.0), deg2rad_(20.0))
                        u = clamp(3.0 * (wcmd - p_wind) * inertia[1] / max(auth1, 1e-9),
                                  -1.0, 1.0)
                        M_b = vadd(M_b, vscale(vb_, auth1 * u))
                        dmrcs += rcs_mdot(rcs, 2) * abs(u)
                    end
                end
                cmq = _cmq_of(veh.aero, Mn)
                kd = qbar * veh.sref * veh.lref * veh.lref / (2 * Vr)
                M_b = vadd(M_b, (0.05 * cmq * kd * w[1],   # weak roll damping
                                 cmq * kd * w[2],
                                 cmq * kd * w[3]))
            else
                M_b = vadd(M_b, vscale(w, -0.5 * minimum(inertia)))  # relax under canopy
            end
            a = vadd(a, vscale(f, 1 / veh.mass))
            qdot_heat = heating_convective(rho, Vr, veh.rn) +
                        heating_radiative(rho, Vr, veh.rn)
        end
    end

    # RCS during the exoatmospheric coast only — the system passivates at
    # entry interface, exactly like a real pod (below EI the statically
    # stable aerodynamics own the attitude). PWM duty cycles; propellant
    # draw scales with commanded duty.
    #   :wind_hold  — PD law tracking the relative wind (what actually keeps
    #                 a pod trimmed through the coast; the velocity vector
    #                 rotates >100° between a deorbit burn and EI, so an
    #                 uncontrolled pod arrives at entry broadside)
    #   :rate_damp  — null body rates only (tipoff recovery)
    if rcs !== nothing && !ctx.entered && x[15] < rcs.prop
        auth = torque_authority(rcs)
        cmd = (0.0, 0.0, 0.0)
        if rcs_mode === :wind_hold && Vr > 1.0
            # gains from the plant: closed-loop wn = 0.1 rad/s, critically
            # damped — well inside the coast-phase integration step
            wn = 0.1
            kp = inertia[2] * wn * wn / auth[2]
            kd = 2.0 * inertia[2] * wn / auth[2]
            vb = qrotate_inv(q, vscale(vrel, 1 / Vr))
            e = vcross((1.0, 0.0, 0.0), vb)        # rotates +x toward the wind
            pd(i) = clamp(kp * e[i] - kd * w[i], -1.0, 1.0)
            cmd = (pd(1), pd(2), pd(3))
        else
            # Saturation rate: the rate the thrusters can null in
            # RATE_NULL_S of continuous firing. auth/I is an angular
            # ACCELERATION, so the constant carries units of seconds — it
            # read as a bare `2.0` and looked dimensionless, which is how a
            # rate and an acceleration came to be compared.
            wsat = RATE_NULL_S * auth[2] / inertia[2]
            cmd = rate_damp_command(w, rate_db, wsat)
        end
        if cmd != (0.0, 0.0, 0.0)
            M_b = vadd(M_b, (auth[1]*cmd[1], auth[2]*cmd[2], auth[3]*cmd[3]))
            # propellant follows the duty actually realizable per axis
            duty = (auth[1] > 0 ? abs(cmd[1]) : 0.0) +
                   (auth[2] > 0 ? abs(cmd[2]) : 0.0) +
                   (auth[3] > 0 ? abs(cmd[3]) : 0.0)
            # `+=`, not `=`. The two RCS blocks are meant to be exclusive —
            # this one passivates at entry interface, where bank-hold takes
            # over — but `ctx.entered` is flipped once per STEP while the
            # aerodynamic block above gates on instantaneous altitude, so on
            # the RK4 substeps that straddle the crossing both can run. An
            # assignment silently discarded the bank-hold flow for that
            # evaluation. The magnitude is one substep's worth of propellant;
            # the reason to fix it is that it was an accident either way.
            dmrcs += rcs_mdot(rcs, 2) * duty
        end
    end

    dw = euler_wdot(inertia, w, M_b)
    dq = qdot(q, w)

    dx[1] = v[1];  dx[2] = v[2];  dx[3] = v[3]
    dx[4] = a[1];  dx[5] = a[2];  dx[6] = a[3]
    dx[7] = dq[1]; dx[8] = dq[2]; dx[9] = dq[3]; dx[10] = dq[4]
    dx[11] = dw[1]; dx[12] = dw[2]; dx[13] = dw[3]
    dx[14] = qdot_heat
    dx[15] = dmrcs
    nothing
end

function _rk4_e6!(xo, x, t, dt, w, scn, ctx, inertia, rcs, rate_db, rcs_mode)
    n = length(x)
    _entry6_deriv!(w.k1, x, scn, ctx, inertia, rcs, rate_db, rcs_mode, t)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k1[i]; end
    _entry6_deriv!(w.k2, w.xt, scn, ctx, inertia, rcs, rate_db, rcs_mode, t + 0.5dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k2[i]; end
    _entry6_deriv!(w.k3, w.xt, scn, ctx, inertia, rcs, rate_db, rcs_mode, t + 0.5dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + dt * w.k3[i]; end
    _entry6_deriv!(w.k4, w.xt, scn, ctx, inertia, rcs, rate_db, rcs_mode, t + dt)
    @inbounds for i in 1:n
        xo[i] = x[i] + (dt / 6) * (w.k1[i] + 2w.k2[i] + 2w.k3[i] + w.k4[i])
    end
    # keep the quaternion on the unit sphere
    qn = sqrt(xo[7]^2 + xo[8]^2 + xo[9]^2 + xo[10]^2)
    @inbounds for i in 7:10; xo[i] /= qn; end
    nothing
end

function _e6_data(x, scn::Scenario, ctx::FlightContext, t)
    r = (x[1], x[2], x[3]); v = (x[4], x[5], x[6])
    q = (x[7], x[8], x[9], x[10])
    theta = earth_rotation_angle(scn.theta_g0, t)
    lat, lon, h = geodetic_from_ecef(rot_z(r, theta))
    omega_e = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega_e, r))
    Vr = vnorm(vrel)
    rho, _, _, asnd = h < 600e3 ? atmosphere_state(scn.atmosphere, h) : (0.0, 0.0, 0.0, 300.0)
    Mn = Vr / asnd
    qbar = 0.5 * rho * Vr * Vr
    xhat = qrotate(q, (1.0, 0.0, 0.0))
    alpha_t = Vr > 1 ? acos(clamp(vdot(vscale(vrel, 1/Vr), xhat), -1.0, 1.0)) : 0.0
    gload = 0.0; qd = 0.0
    if h < scn.h_ei && rho > 0 && Vr > 1
        veh = scn.vehicle
        cda_ch = 0.0
        for (i, c) in enumerate(veh.chutes)
            td = ctx.chute_deploy_t[i]
            isnan(td) || (cda_ch += c.cda * chute_fill(c, t - td))
        end
        D = qbar * (veh.sref * cd_coeff(veh.aero, Mn, alpha_t) + cda_ch)
        L = any_chute_deployed(ctx) ? 0.0 : qbar * veh.sref * cl_coeff(veh.aero, Mn, alpha_t)
        gload = hypot(D, L) / (veh.mass * G0)
        qd = heating_convective(rho, Vr, veh.rn) + heating_radiative(rho, Vr, veh.rn)
    end
    (t = t, h = h, lat = lat, lon = lon, vrel = Vr, mach = Mn, qbar = qbar,
     gload = gload, alpha_t = alpha_t, qdot = qd)
end

"""
    simulate_entry6(scn; inertia, rcs=nothing, w0=(0,0,0),
                    rate_db=deg2rad_(0.2), alpha0=scn.alpha0)
        -> Entry6Result

Full 6-DOF flight of `scn` from its initial state to splashdown. `inertia`
defaults to an axisymmetric capsule built from the vehicle's `iyy`
(roll inertia 80% of transverse). With an `rcs` system, tipoff rates `w0`
are actively nulled to `rate_db` before the atmosphere takes over.
"""
function simulate_entry6(scn::Scenario;
                         inertia::V3 = (0.8 * scn.vehicle.iyy,
                                        scn.vehicle.iyy, scn.vehicle.iyy),
                         rcs::Union{Nothing,RCSystem} = nothing,
                         rcs_mode::Symbol = :wind_hold,
                         w0::V3 = (0.0, 0.0, 0.0),
                         rate_db::Float64 = deg2rad_(0.2),
                         alpha0::Float64 = scn.alpha0,
                         log_dt_orbit::Float64 = 5.0,
                         log_dt_entry::Float64 = 0.5)
    _validate_simulation(scn, log_dt_orbit, log_dt_entry)
    # initial attitude: symmetry axis offset from the relative wind by alpha0
    r0 = scn.r0; v0 = scn.v0
    omega_e = (0.0, 0.0, OMEGA_EARTH)
    vrel0 = vsub(v0, vcross(omega_e, r0))
    vhat0 = vunit(vrel0)
    phat = vunit(vcross(vhat0, vunit(r0)))          # transverse axis
    xdes = vadd(vscale(vhat0, cos(alpha0)),
                vscale(vcross(phat, vhat0), sin(alpha0)))
    q0 = quat_from_to((1.0, 0.0, 0.0), vunit(xdes))

    x = [r0[1], r0[2], r0[3], v0[1], v0[2], v0[3],
         q0[1], q0[2], q0[3], q0[4], w0[1], w0[2], w0[3], 0.0, 0.0]
    xnew = similar(x)
    wk = RK4Work(15)
    ctx = FlightContext(length(scn.vehicle.chutes))
    events = FlightEvent[]
    L = Entry6Log()

    t = scn.t0
    theta = earth_rotation_angle(scn.theta_g0, t)
    _, _, h = geodetic_from_ecef(rot_z(r0, theta))
    ctx.entered = h < scn.h_ei
    next_log = t

    peak_g = 0.0; peak_qd = 0.0; peak_qb = 0.0
    terminated = :timeout
    t_sp = NaN; lat_sp = NaN; lon_sp = NaN; v_sp = NaN

    alt(xv, tv) = begin
        th = earth_rotation_angle(scn.theta_g0, tv)
        geodetic_from_ecef(rot_z((xv[1], xv[2], xv[3]), th))[3]
    end
    rec!(tv) = begin
        d = _e6_data(x, scn, ctx, tv)
        push!(L.t, d.t); push!(L.h, d.h); push!(L.lat, d.lat); push!(L.lon, d.lon)
        push!(L.vrel, d.vrel); push!(L.mach, d.mach); push!(L.qbar, d.qbar)
        push!(L.gload, d.gload); push!(L.alpha_t, d.alpha_t)
        push!(L.wx, x[11]); push!(L.wy, x[12]); push!(L.wz, x[13])
        push!(L.qdot, d.qdot); push!(L.qload, x[14])
        push!(L.twall, wall_temperature(d.qdot, scn.vehicle.emissivity))
        push!(L.rcs_used, x[15])
        d
    end
    ev!(name, tv) = begin
        d = _e6_data(x, scn, ctx, tv)
        push!(events, FlightEvent(name, tv, d.h, d.mach, d.vrel, d.lat, d.lon))
        d
    end
    initial = rec!(t)
    peak_g = initial.gload
    peak_qd = initial.qdot
    peak_qb = ctx.entered ? initial.qbar : 0.0

    while t < scn.t_max
        chutes_out = any_chute_deployed(ctx)
        dt = !ctx.entered ? scn.dt_orbit : (chutes_out ? scn.dt_descent : scn.dt_entry)
        dt = min(dt, scn.t_max - t)
        t + dt > t || throw(ArgumentError("integration step cannot advance time at this epoch"))

        _rk4_e6!(xnew, x, t, dt, wk, scn, ctx, inertia, rcs, rate_db, rcs_mode)
        all(isfinite, xnew) || error("non-finite 6-DOF state at t=$(t + dt)")
        hnew = alt(xnew, t + dt)

        if !ctx.entered && hnew < scn.h_ei
            lo, hi = 0.0, dt
            for _ in 1:30
                mid = 0.5 * (lo + hi)
                _rk4_e6!(xnew, x, t, mid, wk, scn, ctx, inertia, rcs, rate_db, rcs_mode)
                alt(xnew, t + mid) > scn.h_ei ? (lo = mid) : (hi = mid)
                hi - lo < 1e-4 && break
            end
            _rk4_e6!(xnew, x, t, hi, wk, scn, ctx, inertia, rcs, rate_db, rcs_mode)
            copyto!(x, xnew); t += hi
            ctx.entered = true
            ev!(:entry_interface, t)
            rec!(t); next_log = t + log_dt_entry
            continue
        end

        if hnew <= 0.0
            lo, hi = 0.0, dt
            for _ in 1:40
                mid = 0.5 * (lo + hi)
                _rk4_e6!(xnew, x, t, mid, wk, scn, ctx, inertia, rcs, rate_db, rcs_mode)
                alt(xnew, t + mid) > 0.0 ? (lo = mid) : (hi = mid)
                hi - lo < 1e-5 && break
            end
            _rk4_e6!(xnew, x, t, hi, wk, scn, ctx, inertia, rcs, rate_db, rcs_mode)
            copyto!(x, xnew); t += hi
            d = ev!(:splashdown, t)
            rec!(t)
            terminated = :splashdown
            t_sp = t; lat_sp = d.lat; lon_sp = d.lon; v_sp = d.vrel
            break
        end

        copyto!(x, xnew); t += dt

        if ctx.entered
            d = _e6_data(x, scn, ctx, t)
            peak_g = max(peak_g, d.gload)
            peak_qd = max(peak_qd, d.qdot)
            peak_qb = max(peak_qb, d.qbar)
            for (i, c) in enumerate(scn.vehicle.chutes)
                if isnan(ctx.chute_deploy_t[i]) && d.mach < c.mach_max && d.h < c.alt_max
                    ctx.chute_deploy_t[i] = t
                    push!(events, FlightEvent(Symbol(:deploy_, c.name), t, d.h,
                                              d.mach, d.vrel, d.lat, d.lon))
                end
            end
            if t >= next_log
                rec!(t); next_log += log_dt_entry
            end
        elseif t >= next_log
            rec!(t); next_log += log_dt_orbit
        end
    end

    if terminated == :timeout && L.t[end] != t
        rec!(t)
    end

    # stability envelope, post-hoc: max total AoA after peak deceleration
    # while still supersonic (subsonic capsule wobble is real physics and
    # the drogue's job, so the metric stops at Mach 2)
    max_alpha_post = 0.0
    if !isempty(L.gload)
        ipk = argmax(L.gload)
        for i in ipk+1:length(L.t)
            L.mach[i] > 2.0 && (max_alpha_post = max(max_alpha_post, L.alpha_t[i]))
        end
    end
    Entry6Result(L, events, t_sp, lat_sp, lon_sp, v_sp,
                 peak_g, peak_qd, peak_qb, x[14], x[15],
                 max_alpha_post, terminated)
end
