# 4-DOF equations of motion.
#
# State vector x (9 elements):
#   x[1:3]  r    ECI position [m]
#   x[4:6]  v    ECI inertial velocity [m/s]
#   x[7]    alpha  pitch-plane angle of attack [rad]      (4th DOF)
#   x[8]    q      body pitch rate [rad/s]
#   x[9]    Q      accumulated stagnation heat load [J/m^2]
#
# Translational dynamics are integrated in ECI Cartesian coordinates over a
# rotating Earth: aero forces use the atmosphere-relative velocity
# v_rel = v - omega_e x r. This makes launch / mid-course / third-body
# extensions straightforward (just add accelerations).
#
# Rotational DOF: classic pitch-plane formulation. The body pitch angle
# theta = gamma + alpha evolves as theta_dot = q, so
#   alpha_dot = q - gamma_dot,
#   Iyy q_dot = qbar Sref Lref Cm(M, alpha, qhat).
# gamma_dot is evaluated analytically from the current acceleration, keeping
# the rotational channel consistent with the 3D translational motion (valid
# while heading changes slowly, which holds for entry trajectories).
#
# Phase logic (per user spec):
#   h >= h_ei (120 km): pure orbital mechanics — no aero, attitude frozen.
#   h <  h_ei          : aerodynamics, pitch dynamics, heating, parachutes.

"""
    bank_command(bank, t, h, vrel, gload) -> Float64

Resolve a scenario's bank setting: a number is a fixed angle, anything else
is called as a roll-control law.
"""
@inline bank_command(b::Real, t, h, v, gl) = Float64(b)
@inline bank_command(b, t, h, v, gl) = Float64(b(t, h, v, gl))

"""
    gload_bank(g_target; bank_max = 150 deg, kp = 1.0) -> law

Roll-control law flying the entry at roughly constant deceleration — the job
Apollo's entry guidance did with its bank-angle command.

A capsule cannot change how much lift it makes; that is fixed by its trim
angle of attack. What it can do is point that lift anywhere on a cone about
the velocity vector by rolling. Lift **up** raises the trajectory into
thinner air and sheds load; lift **down** holds the vehicle in and prevents a
skip-out. So this commands lift-down while the load is below target
(capture), and rolls smoothly toward lift-up as it builds (pull-out).

`bank_max` stops short of pi so the law never commits to pure lift-down,
which would leave nothing in reserve to recover with.
"""
function gload_bank(g_target::Float64; bank_max::Float64 = deg2rad_(150.0),
                    kp::Float64 = 1.0)
    (t, h, v, gl) -> bank_max * clamp(kp * (g_target - gl) / g_target, 0.0, 1.0)
end

"Runtime (mutable) flight status carried alongside the state vector."
mutable struct FlightContext
    chute_deploy_t::Vector{Float64}  # NaN until deployed, else deployment time [s]
    entered::Bool                    # crossed entry interface
    FlightContext(nchutes::Int) = new(fill(NaN, nchutes), false)
end

any_chute_deployed(ctx::FlightContext) = any(!isnan, ctx.chute_deploy_t)

"""
    Scenario

Everything needed to run one flight. `extra_accel(r, v, t) -> V3` is a hook
for additional accelerations (thrust for launch/deorbit extensions, solar
radiation pressure, ...). `bank` orients the trim lift vector about the
relative velocity (0 = lift up, pi = lift down). A number is held fixed; a
callable `(t, h, vrel, gload) -> rad` is a roll-control law evaluated every
derivative, which is how a real capsule flies — see [`gload_bank`](@ref).
An axisymmetric capsule has no aerodynamic restoring moment about its own
axis, so bank is the one attitude freedom that needs RCS during entry, and
the only one worth spending propellant on.
"""
Base.@kwdef struct Scenario{V<:Vehicle,A<:AbstractAtmosphere,G<:AbstractGravity,F,B}
    vehicle::V
    atmosphere::A = USSA76()
    gravity::G = J2Gravity()
    r0::V3
    v0::V3
    alpha0::Float64 = deg2rad_(5.0)   # initial attitude offset (oscillates & damps below EI)
    t0::Float64 = 0.0
    theta_g0::Float64 = 0.0           # Earth rotation angle at t0 [rad]
    h_ei::Float64 = 120.0e3           # entry interface altitude [m]
    bank::B = 0.0
    extra_accel::F = (r, v, t) -> (0.0, 0.0, 0.0)
    target_lat::Float64 = NaN         # splashdown target [rad]
    target_lon::Float64 = NaN
    dt_orbit::Float64 = 1.0
    dt_entry::Float64 = 0.05
    dt_descent::Float64 = 0.2         # after first chute deploy
    t_max::Float64 = 3.0e4
end

initial_state(s::Scenario) =
    [s.r0[1], s.r0[2], s.r0[3], s.v0[1], s.v0[2], s.v0[3], s.alpha0, 0.0, 0.0]

function _validate_simulation(s::Scenario, log_dt_orbit, log_dt_entry)
    all(isfinite, (s.r0..., s.v0..., s.t0, s.t_max, s.h_ei, s.alpha0, s.theta_g0)) &&
        vnorm(s.r0) > 0 && s.t_max >= s.t0 ||
        throw(ArgumentError("simulation needs finite state and times, with t_max >= t0"))
    all(dt -> isfinite(dt) && dt > 0, (s.dt_orbit, s.dt_entry, s.dt_descent)) ||
        throw(ArgumentError("integration steps must be finite and positive"))
    all(dt -> dt > 0, (log_dt_orbit, log_dt_entry)) ||
        throw(ArgumentError("logging intervals must be positive"))
end

# Pitch dynamics are meaningful only away from vertical flight and before
# parachutes dominate the attitude.
const COSGAMMA_MIN = 0.05

"""
    dynamics!(dx, x, scn, ctx, t)

Evaluate the state derivative in place.
"""
function dynamics!(dx::Vector{Float64}, x::Vector{Float64},
                   scn::Scenario, ctx::FlightContext, t::Float64)
    r = (x[1], x[2], x[3])
    v = (x[4], x[5], x[6])
    alpha = x[7]
    qrate = x[8]

    veh = scn.vehicle
    theta = earth_rotation_angle(scn.theta_g0, t)
    lat, lon, h = geodetic_from_ecef(rot_z(r, theta))

    a = gravity_accel(scn.gravity, r, t)
    a = vadd(a, scn.extra_accel(r, v, t)::V3)

    dalpha = 0.0
    dq = 0.0
    qdot_heat = 0.0

    if h < scn.h_ei
        rho, T, _, asnd = atmosphere_state(scn.atmosphere, h)
        omega = (0.0, 0.0, OMEGA_EARTH)
        vrel = vsub(v, vcross(omega, r))
        Vr = vnorm(vrel)
        if rho > 0 && Vr > 1.0
            qbar = 0.5 * rho * Vr * Vr
            M = Vr / asnd
            vhat = vscale(vrel, 1 / Vr)
            rhat = vunit(r)
            singam = clamp(vdot(rhat, vhat), -1.0, 1.0)
            cosgam = sqrt(max(0.0, 1 - singam * singam))

            # --- drag area: body + parachutes -------------------------------
            cda_chutes = 0.0
            for (i, c) in enumerate(veh.chutes)
                td = ctx.chute_deploy_t[i]
                isnan(td) || (cda_chutes += c.cda * chute_fill(c, t - td))
            end
            chutes_out = any_chute_deployed(ctx)

            CD = cd_coeff(veh.aero, M, alpha)
            D = qbar * (veh.sref * CD + cda_chutes)

            # --- lift (suppressed once a canopy is out) ---------------------
            L = chutes_out ? 0.0 : qbar * veh.sref * cl_coeff(veh.aero, M, alpha)
            f_aero = vscale(vhat, -D)
            if L != 0.0 && cosgam > COSGAMMA_MIN
                uhat = vunit(vsub(rhat, vscale(vhat, singam)))   # in-plane "up" ⊥ v
                shat = vcross(vhat, uhat)                        # completes right-handed set
                # lift magnitude is bank-independent, so the load factor
                # driving the roll law can be formed here without circularity
                gl_now = sqrt(D * D + L * L) / (veh.mass * G0)
                bk = bank_command(scn.bank, t, h, Vr, gl_now)
                lhat = vadd(vscale(uhat, cos(bk)), vscale(shat, sin(bk)))
                f_aero = vadd(f_aero, vscale(lhat, L))
            end
            a = vadd(a, vscale(f_aero, 1 / veh.mass))

            # --- flight-path-angle rate (needed by the pitch DOF) -----------
            # sin(gamma) = rhat . vhat; differentiate with a_rel = dvrel/dt.
            arel = vsub(a, vcross(omega, v))
            rn_ = vnorm(r)
            drhat = vscale(vsub(v, vscale(rhat, vdot(v, rhat))), 1 / rn_)
            dvhat = vscale(vsub(arel, vscale(vhat, vdot(arel, vhat))), 1 / Vr)
            gamdot = (vdot(drhat, vhat) + vdot(rhat, dvhat)) / max(cosgam, COSGAMMA_MIN)

            # --- pitch DOF ---------------------------------------------------
            # Full kinematics whenever the pitch plane is well defined: the
            # aero torque scales with qbar and vanishes naturally in near-
            # vacuum, where alpha_dot = -gamma_dot is exactly the behavior of
            # an inertially-fixed body as the velocity vector rotates.
            if !chutes_out && cosgam > COSGAMMA_MIN
                qhat = qrate * veh.lref / (2 * Vr)
                Cm = cm_coeff(veh.aero, M, alpha, qhat)
                dq = qbar * veh.sref * veh.lref * Cm / veh.iyy
                dalpha = qrate - gamdot
            else
                # attitude irrelevant under canopy / near-vertical: relax gently
                dalpha = -0.2 * alpha
                dq = -0.5 * qrate
            end

            # --- stagnation heating -----------------------------------------
            qdot_heat = heating_convective(rho, Vr, veh.rn) +
                        heating_radiative(rho, Vr, veh.rn)
        end
    end

    dx[1] = v[1]; dx[2] = v[2]; dx[3] = v[3]
    dx[4] = a[1]; dx[5] = a[2]; dx[6] = a[3]
    dx[7] = dalpha
    dx[8] = dq
    dx[9] = qdot_heat
    return nothing
end

"""
    flight_data(x, scn, ctx, t) -> NamedTuple

Derived quantities for logging, event checks and analysis.
"""
function flight_data(x::Vector{Float64}, scn::Scenario, ctx::FlightContext, t::Float64)
    r = (x[1], x[2], x[3])
    v = (x[4], x[5], x[6])
    veh = scn.vehicle
    theta = earth_rotation_angle(scn.theta_g0, t)
    r_ecef = rot_z(r, theta)
    lat, lon, h = geodetic_from_ecef(r_ecef)

    omega = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega, r))
    Vr = vnorm(vrel)
    rho, T, _, asnd = h < 600e3 ? atmosphere_state(scn.atmosphere, h) : (0.0, 0.0, 0.0, 300.0)
    M = Vr / asnd
    qbar = 0.5 * rho * Vr * Vr

    rhat = vunit(r)
    vhat = Vr > 1 ? vscale(vrel, 1 / Vr) : (1.0, 0.0, 0.0)
    gamma = asin(clamp(vdot(rhat, vhat), -1.0, 1.0))

    # heading (deg from North, eastward positive) from ENU components of vrel
    vrel_ecef = rot_z(vrel, theta)
    eE, eN, _ = enu_basis(lat, lon)
    psi = atan(vdot(vrel_ecef, eE), vdot(vrel_ecef, eN))

    # aero acceleration magnitude for g-load
    gload = 0.0
    qdot_c = 0.0
    qdot_r = 0.0
    if h < scn.h_ei && rho > 0 && Vr > 1
        cda_chutes = 0.0
        for (i, c) in enumerate(veh.chutes)
            td = ctx.chute_deploy_t[i]
            isnan(td) || (cda_chutes += c.cda * chute_fill(c, t - td))
        end
        CD = cd_coeff(veh.aero, M, x[7])
        D = qbar * (veh.sref * CD + cda_chutes)
        L = any_chute_deployed(ctx) ? 0.0 : qbar * veh.sref * cl_coeff(veh.aero, M, x[7])
        gload = sqrt(D^2 + L^2) / (veh.mass * G0)
        qdot_c = heating_convective(rho, Vr, veh.rn)
        qdot_r = heating_radiative(rho, Vr, veh.rn)
    end

    (t = t, h = h, lat = lat, lon = lon, vin = vnorm(v), vrel = Vr,
     gamma = gamma, psi = psi, mach = M, qbar = qbar, gload = gload,
     alpha = x[7], qrate = x[8], rho = rho, T = T,
     qdot_conv = qdot_c, qdot_rad = qdot_r, qload = x[9],
     twall = wall_temperature(qdot_c + qdot_r, veh.emissivity))
end
