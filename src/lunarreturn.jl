# Coming back: lunar ascent, rendezvous, trans-Earth injection, entry.
#
# The landing chain ends on the surface, which is where a one-way mission ends
# and where a real one is only half over. This file flies the other half, and
# it is the half that dictates the architecture of the whole vehicle.
#
# The decisive number is that leaving the Moon from the surface costs about
# 1.85 km/s to orbit and another 0.9 km/s to leave lunar orbit for Earth. Carry
# all of that on the thing you land, and you land something enormous. Leave
# most of it in orbit and rendezvous with it afterwards, and you land something
# small — which is the entire argument for lunar-orbit rendezvous, made in 1962
# by John Houbolt against most of NASA, and won on exactly this arithmetic.
#
#   1. **Ascent** from the surface: ten seconds of vertical rise to get off the
#      ground, then a linear pitch program shot on (initial pitch, pitch rate)
#      to insert at a chosen periapsis with the flight path level. No
#      atmosphere, so it is a pure gravity-loss problem and the program is the
#      same two-parameter shoot the launch and the descent both use.
#   2. **Rendezvous** with whatever was left in lunar orbit: a Lambert transfer
#      searched over both lift-off time and transfer time, because ascent from
#      a rotating surface into a fixed orbit is a phasing problem before it is
#      a targeting one — you cannot leave whenever you like.
#   3. **Trans-Earth injection**: a prograde burn from lunar orbit, shot on
#      (ignition point, magnitude) so the Earth-relative trajectory arrives at
#      the entry interface with a survivable perigee.
#   4. **Entry**, handed to the same `Scenario`/`simulate` chain the flyby
#      mission uses, because by that point it is the same problem.
#
# One convenient artifact of the coplanar circular Moon: the Moon's spin axis
# is the mission-plane normal, so a landing site sits on the *equator of that
# plane* and stays in the orbit plane no matter how long the stay. On the real
# Moon a site drifts out of the orbiter's plane at up to half a degree an hour,
# and the plane change to catch up is what limits a surface stay to a few days.
# That limit does not exist here, and nothing below models it.

# --------------------------------------------------------------- logging ---

"Ascent-from-the-surface log, Moon-centred inertial."
struct AscentMoonLog
    t::Vector{Float64}          # seconds from lift-off
    h::Vector{Float64}          # altitude above the mean sphere [m]
    v::Vector{Float64}          # ground-relative speed [m/s]
    vh::Vector{Float64}; vv::Vector{Float64}
    m::Vector{Float64}
    pitch::Vector{Float64}      # thrust elevation above local horizontal [rad]
    x::Vector{Float64}; y::Vector{Float64}; z::Vector{Float64}
end
AscentMoonLog() = AscentMoonLog((Float64[] for _ in 1:10)...)

"""
Products of a lunar ascent: the insertion state and how much of the stage is
left once it is in orbit — which is the entire budget for catching the
orbiter.
"""
struct LunarAscentResult
    log::AscentMoonLog
    outcome::Symbol             # :insertion | :propellant | :impact | :timeout
    t_cut::Float64              # burn duration [s]
    r::V3; v::V3; m::Float64    # insertion state, Moon-centred inertial
    h_cut::Float64              # insertion altitude [m]
    gamma_cut::Float64          # insertion flight-path angle [rad]
    hp::Float64; ha::Float64    # resulting periapsis/apoapsis altitudes [m]
    pitch0::Float64             # pitch after the vertical rise [rad]
    pitch_rate::Float64         # [rad/s]
    dv_ideal::Float64           # ideal velocity spent [m/s]
    prop_left::Float64          # [kg]
end

"""
The whole trip home: ascent, rendezvous, trans-Earth injection, entry.
"""
struct ReturnResult
    ascent::LunarAscentResult
    stage::AscentStage
    orbiter::Orbiter
    t_liftoff::Float64          # mission time [s]
    stay::Float64               # surface stay [s]
    dv_rendezvous::Float64      # total, both burns [m/s]
    dv_transfer::Float64        # the Lambert departure impulse alone [m/s]
    dv_braking::Float64         # the arrival impulse alone [m/s]
    t_dock::Float64             # mission time of rendezvous [s]
    t_transfer::Float64         # coast time of the rendezvous transfer [s]
    prop_ascent_left::Float64   # ascent-stage propellant at docking [kg]
    dv_tei::Float64             # trans-Earth injection [m/s]
    t_tei::Float64              # mission time of TEI [s]
    m_tei::Float64              # orbiter mass after TEI [kg]
    prop_orbiter_left::Float64  # [kg]
    cis::CislunarResult         # the coast home
    entry::Union{Nothing,SimResult}
    scenario::Union{Nothing,Scenario}
end

# ---------------------------------------------------------------- ascent ---

"""
    lunar_ascent(stage, r0, t0, hhat; h_ins, ha_ins, t_rise, pitch0, pitch_rate,
                 cfg, log) -> LunarAscentResult

Fly an ascent from rest on the surface. `r0` is the lift-off position in the
Moon-centred inertial frame and `hhat` the normal of the plane to insert into
— the same one the descent came down in, so the ascent stage arrives in the
orbiter's plane by construction.

The vehicle starts *moving*: it is standing on a body that rotates, so its
initial velocity is the surface velocity, and those 4.6 m/s are free orbital
velocity in exactly the way an equatorial launch from Earth is.

The pitch program is the same shape as everything else here — a constant
followed by a ramp — with a vertical rise first, because a stage that pitches
over on the pad flies into the ground. Cutoff is on *energy*: the burn ends
when the orbit's semi-major axis reaches the target, which is a condition the
guidance can evaluate from its own state and which leaves the remaining
propellant for the rendezvous.
"""
function lunar_ascent(a::AscentStage, r0::V3, t0::Float64, hhat::V3;
                      h_ins::Float64 = 15.0e3, ha_ins::Float64 = 85.0e3,
                      t_rise::Float64 = 10.0,
                      pitch0::Float64 = deg2rad_(52.0),
                      pitch_rate::Float64 = -2.0e-3,
                      cfg::DescentConfig = DescentConfig(),
                      m0::Float64 = ascent_mass(a),
                      log::Union{Nothing,AscentMoonLog} = nothing,
                      dt::Float64 = 0.5, t_max::Float64 = 900.0,
                      log_every::Int = 4)
    mdot = _ascent_mdot(a)
    m_dry = m0 - a.mprop
    a_target = 0.5 * (2 * R_MOON + h_ins + ha_ins)
    e_target = -MU_MOON / (2 * a_target)

    r = r0
    v = _surface_vel(cfg, r0, t0)               # standing on a turning Moon
    m, t = m0, 0.0
    outcome = :timeout
    kount = 0

    pitch(tt) = tt < t_rise ? deg2rad_(90.0) :
                clamp(pitch0 + pitch_rate * (tt - t_rise),
                      deg2rad_(-15.0), deg2rad_(90.0))
    dir(rr, tt) = begin
        ur = vunit(rr); ut = vcross(hhat, ur)
        th = pitch(tt)
        vadd(vscale(ur, sin(th)), vscale(ut, cos(th)))   # prograde, unlike a descent
    end
    energy(rr, vv) = 0.5 * vdot(vv, vv) - MU_MOON / vnorm(rr)

    while t < t_max
        if log !== nothing && kount % log_every == 0
            ur = vunit(r); ut = vcross(hhat, ur)
            vr = vsub(v, _surface_vel(cfg, r, t0 + t))
            push!(log.t, t); push!(log.h, vnorm(r) - R_MOON)
            push!(log.v, vnorm(vr)); push!(log.vh, vdot(vr, ut))
            push!(log.vv, vdot(vr, ur)); push!(log.m, m)
            push!(log.pitch, pitch(t))
            push!(log.x, r[1]); push!(log.y, r[2]); push!(log.z, r[3])
        end
        kount += 1
        if energy(r, v) >= e_target
            outcome = :insertion
            break
        end
        if m <= m_dry + 1e-9
            outcome = :propellant
            break
        end
        if t > t_rise && vnorm(r) < R_MOON - 100.0
            outcome = :impact
            break
        end
        step = min(dt, (m - m_dry) / mdot)
        # the config carries no time offset here, so `tt` is mission time
        acc(rr, mm, tt) = vadd(_grav(cfg, rr, tt),
                               vscale(dir(rr, tt - t0), a.thrust / mm))
        ta = t0 + t
        k1r = v;                            k1v = acc(r, m, ta)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2))
        k2r = v2;                           k2v = acc(r2, m - mdot*step/2, ta + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                           k3v = acc(r3, m - mdot*step/2, ta + step/2)
        r4 = vadd(r, vscale(k3r, step));   v4 = vadd(v, vscale(k3v, step))
        k4r = v4;                           k4v = acc(r4, m - mdot*step, ta + step)
        rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        # cut exactly on the target energy rather than stepping past it
        if energy(rn, vn) > e_target
            lo, hi = 0.0, 1.0
            for _ in 1:40
                f = 0.5 * (lo + hi)
                rm = vadd(r, vscale(vsub(rn, r), f))
                vm = vadd(v, vscale(vsub(vn, v), f))
                if energy(rm, vm) < e_target; lo = f; else; hi = f; end
            end
            f = 0.5 * (lo + hi)
            r = vadd(r, vscale(vsub(rn, r), f))
            v = vadd(v, vscale(vsub(vn, v), f))
            m -= mdot * step * f
            t += step * f
            outcome = :insertion
            break
        end
        r, v, m, t = rn, vn, m - mdot*step, t + step
    end

    rn_ = vnorm(r); vn_ = vnorm(v)
    gamma = asin(clamp(vdot(vunit(r), vscale(v, 1 / vn_)), -1.0, 1.0))
    # osculating periapsis/apoapsis of the insertion orbit
    en = 0.5 * vn_^2 - MU_MOON / rn_
    sma = -MU_MOON / (2 * en)
    hvec = vcross(r, v)
    ecc = sqrt(max(0.0, 1 + 2 * en * vdot(hvec, hvec) / MU_MOON^2))
    LunarAscentResult(log === nothing ? AscentMoonLog() : log, outcome, t,
                      r, v, m, rn_ - R_MOON, gamma,
                      sma * (1 - ecc) - R_MOON, sma * (1 + ecc) - R_MOON,
                      pitch0, pitch_rate, G0 * a.isp * Base.log(m0 / m),
                      m - m_dry)
end

"""
    tune_lunar_ascent(stage, r0, t0, hhat; h_ins, ha_ins, cfg) -> (pitch0, rate, ok)

Shoot the ascent pitch program: two parameters against two targets — the
altitude and the flight-path angle at cutoff — by the same damped Newton the
launch ascent and the braking phase use. A grid seeds it, because a program
that pitches over too early flies into the ground and one that pitches over
too late runs out of propellant going straight up, and there is no gradient
connecting those two failures.
"""
function tune_lunar_ascent(a::AscentStage, r0::V3, t0::Float64, hhat::V3;
                           h_ins::Float64 = 15.0e3, ha_ins::Float64 = 85.0e3,
                           cfg::DescentConfig = DescentConfig(),
                           m0::Float64 = ascent_mass(a),
                           max_iter::Int = 25, verbose::Bool = false)
    function resid(p0, pr)
        res = lunar_ascent(a, r0, t0, hhat; h_ins = h_ins, ha_ins = ha_ins,
                           pitch0 = p0, pitch_rate = pr, cfg = cfg, m0 = m0)
        if res.outcome === :insertion
            ((res.h_cut - h_ins) / 1000.0, rad2deg_(res.gamma_cut) / 3.0, res)
        elseif res.outcome === :impact
            # came back down: the altitude residual is however far below the
            # target it ended, continuous through the failure
            (-h_ins / 1000.0 - 5.0, rad2deg_(res.gamma_cut) / 3.0, res)
        else
            # never made orbital energy — usually pitched over far too late and
            # spent the tank climbing. Penalise in the direction that pitches
            # over sooner and harder.
            ((res.h_cut - h_ins) / 1000.0 + 5.0,
             rad2deg_(res.gamma_cut) / 3.0 + 5.0, res)
        end
    end
    score(p0, pr) = (f = resid(p0, pr); hypot(f[1], f[2]))

    best = (Inf, deg2rad_(52.0), -2.0e-3)
    for p0 in deg2rad_.(20.0:5.0:80.0), pr in (-4.0e-3:5.0e-4:0.0)
        sc = score(p0, pr)
        sc < best[1] && (best = (sc, p0, pr))
    end
    verbose && @info "ascent seed" pitch0_deg = rad2deg_(best[2]) rate = best[3] score = best[1]

    p0, pr = best[2], best[3]
    bp = (best[1], p0, pr)
    for it in 1:max_iter
        f1, f2, res = resid(p0, pr)
        sc = hypot(f1, f2)
        sc < bp[1] && (bp = (sc, p0, pr))
        verbose && @info "ascent newton" it f1 f2 pitch0_deg = rad2deg_(p0) rate = pr outcome = res.outcome
        (abs(f1) < 0.05 && abs(f2) < 0.05 && res.outcome === :insertion) &&
            return (p0, pr, true)
        d1 = deg2rad_(0.5); d2 = 5.0e-5
        f1a, f2a, _ = resid(p0 + d1, pr)
        f1b, f2b, _ = resid(p0, pr + d2)
        j11 = (f1a - f1) / d1; j21 = (f2a - f2) / d1
        j12 = (f1b - f1) / d2; j22 = (f2b - f2) / d2
        det = j11 * j22 - j12 * j21
        abs(det) < 1e-16 && break
        dp0 = -( j22 * f1 - j12 * f2) / det
        dpr = -(-j21 * f1 + j11 * f2) / det
        p0 = clamp(p0 + clamp(0.7 * dp0, -deg2rad_(6.0), deg2rad_(6.0)),
                   deg2rad_(5.0), deg2rad_(89.0))
        pr = clamp(pr + clamp(0.7 * dpr, -5.0e-4, 5.0e-4), -8.0e-3, 2.0e-3)
    end
    f1, f2, res = resid(bp[2], bp[3])
    (bp[2], bp[3], abs(f1) < 0.5 && abs(f2) < 0.5 && res.outcome === :insertion)
end

# ------------------------------------------------------------ rendezvous ---

"""
    lunar_rendezvous(stage, r_ins, v_ins, t_ins, m; r_t, v_t, t_t, field, eph)
        -> NamedTuple

Catch the orbiter. A Lambert transfer from the insertion state to wherever the
target will be `t_transfer` seconds later, closed by a braking impulse that
matches its velocity; the transfer time is searched for the cheapest pair.

Chasing is not free and it is not symmetric: leave too late and the target is
ahead and pulling away, leave too early and you arrive under it and have to
wait a whole revolution. Which is why the *lift-off* time is a search variable
too — see [`plan_rendezvous`](@ref) — and why a lunar-orbit-rendezvous mission
launches from the surface on a window a few minutes wide.

Transfers that would clip the Moon are rejected outright rather than scored,
because a cheap trajectory through the surface is not cheap.
"""
function lunar_rendezvous(r_ins::V3, v_ins::V3, t_ins::Float64, m::Float64,
                          isp::Float64, prop_left::Float64;
                          r_t::V3, v_t::V3, field = nothing, eph = nothing,
                          t_min::Float64 = 1800.0, t_max::Float64 = 9000.0,
                          n::Int = 90, h_min::Float64 = 12.0e3)
    best = nothing
    for k in 0:n
        tof = t_min + (t_max - t_min) * k / n
        # where the target is when the chaser gets there
        rt, vt = r_t, v_t
        rt, vt = _propagate_moon(rt, vt, t_ins, tof; field = field, eph = eph)
        local v1, v2
        try
            v1, v2 = lambert(r_ins, rt, tof; mu = MU_MOON)
        catch
            continue
        end
        # a transfer that dips into the Moon is not a transfer
        rp = _peri_radius(r_ins, v1)
        rp < R_MOON + h_min && continue
        dv1 = vnorm(vsub(v1, v_ins))
        dv2 = vnorm(vsub(vt, v2))
        tot = dv1 + dv2
        if best === nothing || tot < best.dv_total
            best = (dv_total = tot, dv1 = dv1, dv2 = dv2, tof = tof,
                    v1 = v1, v2 = v2, r_dock = rt, v_dock = vt)
        end
    end
    best === nothing && return nothing
    # can the stage pay for it?
    m_after = m * exp(-best.dv_total / (G0 * isp))
    (best..., feasible = (m - m_after) <= prop_left,
     prop_used = m - m_after, m_after = m_after)
end

"Osculating periapsis radius of a Moon-centred state [m]."
function _peri_radius(r::V3, v::V3)
    en = 0.5 * vdot(v, v) - MU_MOON / vnorm(r)
    en >= 0 && return 0.0                       # hyperbolic: it leaves anyway
    sma = -MU_MOON / (2 * en)
    hv = vcross(r, v)
    ecc = sqrt(max(0.0, 1 + 2 * en * vdot(hv, hv) / MU_MOON^2))
    sma * (1 - ecc)
end

"Ballistic Moon-centred propagation of `dt` seconds from time `t`."
function _propagate_moon(r::V3, v::V3, t::Float64, dt::Float64;
                         field = nothing, eph = nothing, step::Float64 = 5.0)
    n = max(1, ceil(Int, abs(dt) / step))
    h = dt / n
    for k in 1:n
        r, v = _moon_step(r, v, h; t = t + (k - 1) * h, field = field, eph = eph)
    end
    (r, v)
end

"""
    plan_rendezvous(stage, r_site, hhat, t_earliest, r_t0, v_t0, t_t0; ...)
        -> NamedTuple

Find the lift-off time. The ascent trajectory is identical for every lift-off
time — the site is carried around by the Moon's own rotation, and so is the
gravity field, so the whole problem is invariant under that rotation — but the
*orbiter* is not, and the phase angle between the two at insertion is what
decides whether the transfer costs fifty metres per second or five hundred.

So: sweep lift-off across one orbital period of the target, fly the ascent from
each, price the cheapest Lambert transfer out of each, and take the best. What
comes out is a launch window, and its width is the reason ascent stages are
launched on the clock rather than when the crew is ready.
"""
function plan_rendezvous(a::AscentStage, r_site::V3, hhat::V3,
                         t_earliest::Float64, r_t0::V3, v_t0::V3, t_t0::Float64;
                         pitch0::Float64, pitch_rate::Float64,
                         h_ins::Float64 = 15.0e3, ha_ins::Float64 = 85.0e3,
                         cfg::DescentConfig = DescentConfig(),
                         m0::Float64 = ascent_mass(a),
                         n_window::Int = 36, verbose::Bool = false)
    field = cfg.field; eph = cfg.eph
    T_t = 2pi * sqrt(vnorm(r_t0)^3 / MU_MOON)
    best = nothing
    for k in 0:(n_window - 1)
        t_lift = t_earliest + T_t * k / n_window
        # the site has turned with the Moon by then
        rs = eph === nothing ? r_site :
             moonfixed_inv(moonfixed(r_site, t_earliest, eph), t_lift, eph)
        asc = lunar_ascent(a, rs, t_lift, hhat; h_ins = h_ins, ha_ins = ha_ins,
                           pitch0 = pitch0, pitch_rate = pitch_rate,
                           cfg = cfg, m0 = m0)
        asc.outcome === :insertion || continue
        t_ins = t_lift + asc.t_cut
        rt, vt = _propagate_moon(r_t0, v_t0, t_t0, t_ins - t_t0;
                                 field = field, eph = eph)
        rz = lunar_rendezvous(asc.r, asc.v, t_ins, asc.m, a.isp, asc.prop_left;
                              r_t = rt, v_t = vt, field = field, eph = eph)
        rz === nothing && continue
        verbose && @info "rendezvous window" t_lift dv = rz.dv_total tof = rz.tof feasible = rz.feasible
        if best === nothing || rz.dv_total < best.rz.dv_total
            best = (t_lift = t_lift, r_site = rs, ascent = asc, rz = rz, t_ins = t_ins)
        end
    end
    best
end

# -------------------------------------------------- trans-Earth injection ---

"""
    trans_earth_injection(r_m, v_m, t0, eph; h_ei, ...) -> NamedTuple

Leave lunar orbit for Earth. The burn is prograde and impulsive; the free
parameters are where in the orbit it happens and how big it is, and the target
is the perigee of the resulting Earth-relative trajectory.

The search is a sweep over ignition point with a bisection on magnitude inside
it, rather than a two-dimensional Newton, and deliberately: the residual is
discontinuous — below a threshold magnitude the vehicle does not leave the
Moon at all and the "perigee" it reports is meaningless — so a gradient method
started in the wrong place has nothing to descend. A sweep does not care.

Departing on the leading side of the Moon and retrograde to its orbital motion
is what makes the burn cheap; the sweep discovers that on its own, which is a
reasonable check that the sweep works.
"""
function trans_earth_injection(r_m::V3, v_m::V3, t0::Float64,
                               eph::CircularMoonEphemeris;
                               h_ei::Float64 = 50.0e3, n_point::Int = 24,
                               dv_lo::Float64 = 700.0, dv_hi::Float64 = 1600.0,
                               n_dv::Int = 16, t_coast::Float64 = 5.0 * 86400.0,
                               field = nothing, verbose::Bool = false)
    T = 2pi * sqrt(vnorm(r_m)^3 / MU_MOON)
    r_target = RE_MEAN + h_ei

    # Geocentric perigee radius of the trajectory the burn produces, read off
    # once the vehicle is 80,000 km clear of the Moon and the two-body elements
    # mean something again. NaN if it never gets there — which is what a burn
    # below lunar escape does, and there is no perigee to speak of in that case.
    function perigee(t_ign, dv)
        r, v = _propagate_moon(r_m, v_m, t0, t_ign - t0; field = field, eph = eph)
        vb = vadd(v, vscale(vunit(v), dv))
        rr = vadd(r, moon_position(eph, t_ign))
        vv = vadd(vb, moon_velocity(eph, t_ign))
        t = t_ign
        while t < t_ign + t_coast
            dtc = _cis_dt(rr, t, eph; eta = CIS_ETA)
            rn, vn = _cis_step(rr, vv, t, dtc, eph)
            t += dtc; rr, vv = rn, vn
            moon_distance(eph, rr, t) < R_MOON && return (NaN, rr, vv, t)
            if moon_distance(eph, rr, t) > 8.0e7
                el = elements_from_state(rr, vv)
                return (el.rp, rr, vv, t)
            end
            vnorm(rr) > 1.5 * A_MOON && return (NaN, rr, vv, t)
        end
        (NaN, rr, vv, t)
    end

    best = nothing
    for k in 0:(n_point - 1)
        t_ign = t0 + T * (k % max(n_point, 1)) / max(n_point, 1)
        # Sweep the magnitude and look for a sign change rather than assuming
        # the residual is monotone. It is not: below lunar escape there is no
        # perigee at all, just above it the trajectory loops out and comes back
        # on a path whose perigee swings through everything from grazing the
        # Earth to missing it entirely, and a bisection handed that bracket
        # blind converges to whichever side it started on.
        prev_dv = NaN; prev_p = NaN
        for i in 0:n_dv
            dv = dv_lo + (dv_hi - dv_lo) * i / n_dv
            p = perigee(t_ign, dv)[1]
            if !isnan(p) && !isnan(prev_p) &&
               (p - r_target) * (prev_p - r_target) <= 0.0
                lo, hi = prev_dv, dv
                plo = prev_p
                for _ in 1:44
                    mid = 0.5 * (lo + hi)
                    pm = perigee(t_ign, mid)[1]
                    isnan(pm) && break
                    if (pm - r_target) * (plo - r_target) > 0
                        lo = mid; plo = pm
                    else
                        hi = mid
                    end
                    hi - lo < 0.005 && break
                end
                dvb = 0.5 * (lo + hi)
                rp, rr, vv, t_far = perigee(t_ign, dvb)
                if !isnan(rp) && abs(rp - r_target) < 3.0e3
                    verbose && @info "TEI candidate" t_ign dv = dvb perigee_km = (rp - RE_MEAN) / 1e3
                    if best === nothing || dvb < best.dv
                        best = (t_ign = t_ign, dv = dvb, perigee = rp - RE_MEAN,
                                r = rr, v = vv, t = t_far)
                    end
                end
            end
            prev_dv = dv; prev_p = p
        end
    end
    best
end

"""
    tei_leg(r_m, v_m, t0, t_ign, dv, eph; field, t_max) -> (leg, log)

Apply a prograde impulse of `dv` at `t_ign` and fly the whole way home, in the
Earth-centred frame the return leg belongs in, using the same coast the flyby
mission uses. This is the *truth* against which the cheap sweep above is only
a first guess: the osculating perigee eighty thousand kilometres from the Moon
and the perigee that actually arrives differ by thousands of kilometres,
because the Moon is still there and still pulling.
"""
function tei_leg(r_m::V3, v_m::V3, t0::Float64, t_ign::Float64, dv::Float64,
                 eph::CircularMoonEphemeris; field = nothing,
                 t_max::Float64 = 12.0 * 86400.0, log_every::Int = 4)
    r, v = _propagate_moon(r_m, v_m, t0, t_ign - t0; field = field, eph = eph)
    vb = vadd(v, vscale(vunit(v), dv))
    rr = vadd(r, moon_position(eph, t_ign))
    vv = vadd(vb, moon_velocity(eph, t_ign))
    L = CislunarLog()
    leg = _coast_leg!(L, rr, vv, t_ign, eph; theta_g0 = 0.0, h_stop = 140.0e3,
                      t_end = t_ign + t_max, stop_after_flyby = false,
                      log_every = log_every, outbound = false)
    (leg, L)
end

"""
    refine_tei(r_m, v_m, t0, t_ign, dv0, eph; h_ei, field) -> (dv, leg, log)

Close the trans-Earth injection on the perigee that really arrives, by secant
on the burn magnitude with the full coast in the loop. Ten metres per second
at the Moon is a few hundred kilometres at the Earth, so this converges in a
handful of iterations from anywhere near the answer — and there is no cheaper
way to get the last few hundred kilometres, because the error being removed is
precisely the part the cheap model does not contain.
"""
function refine_tei(r_m::V3, v_m::V3, t0::Float64, t_ign::Float64, dv0::Float64,
                    eph::CircularMoonEphemeris; h_ei::Float64 = 50.0e3,
                    field = nothing, tol::Float64 = 2.0e3, max_iter::Int = 24,
                    verbose::Bool = false)
    # The residual has to be one quantity, and which one depends on whether the
    # trajectory got there. A pass that reaches entry interface reports its
    # perigee from two-body elements taken *at* the interface, where the Moon
    # is 380,000 km away and those elements mean something. A pass that misses
    # only has the osculating perigee read while it was still deep in the
    # Moon's field, and that is wrong by several hundred kilometres — so for a
    # miss, the honest measure is the lowest altitude the trajectory actually
    # flew. The two agree where it matters, at the boundary between them.
    f(dv) = begin
        leg, L = tei_leg(r_m, v_m, t0, t_ign, dv, eph; field = field,
                         log_every = 1)
        e = leg.outcome === :entry_interface ? leg.vac_perigee :
            (isempty(L.h) ? NaN : minimum(L.h))
        (e - h_ei, leg, L)
    end
    dv1 = dv0
    e1, leg1, L1 = f(dv1)
    isnan(e1) && return (dv1, leg1, L1)
    dv2 = dv1 + 2.0
    e2, leg2, L2 = f(dv2)
    for it in 1:max_iter
        verbose && @info "TEI secant" it dv = dv2 perigee_km = (e2 + h_ei) / 1e3 outcome = leg2.outcome
        (abs(e2) < tol && leg2.outcome === :entry_interface) &&
            return (dv2, leg2, L2)
        (isnan(e2) || abs(e2 - e1) < 1e-9) && break
        dvn = dv2 - e2 * (dv2 - dv1) / (e2 - e1)
        # a lunar departure is exquisitely sensitive: a hundred metres per
        # second is the difference between coming home and leaving for good,
        # so the step is leashed even when the secant is confident
        dvn = clamp(dvn, dv2 - 40.0, dv2 + 40.0)
        dvn = clamp(dvn, 600.0, 2000.0)
        dv1, e1 = dv2, e2
        dv2 = dvn
        e2, leg2, L2 = f(dv2)
    end
    (dv2, leg2, L2)
end

# ------------------------------------------------------- mission assembly --

"""
    moonreturn(ls; stage, stay, h_ins, ha_ins, h_ei, capsule, field, verbose)
        -> ReturnResult

Fly the trip home from a completed landing: ascent, rendezvous with the
orbiter `ls` left in lunar orbit, trans-Earth injection, and entry.

`ls` has to be a landing that *left something in orbit* — `moonlanding` with
an `orbiter`. Without one there is nothing to rendezvous with and nothing
carrying the propellant to leave, and the ascent stage does not have anywhere
near enough of its own: 2340 m/s buys a lunar orbit and a rendezvous, and
leaving lunar orbit for Earth costs another 900 on top.

`stay` is how long the crew is on the surface. It does not change the physics
here, for the reason given at the top of this file — a coplanar Moon rotates
about the mission-plane normal, so the site never leaves the orbiter's plane
— but it does change *where in its orbit* the orbiter is when the ascent stage
lifts off, which is the whole rendezvous problem, and `plan_rendezvous`
searches the next full revolution for the cheapest moment to go.

The capsule that enters is not the orbiter: the service half is jettisoned
after the last burn, exactly as Apollo did, so what meets the atmosphere is
`capsule` and it must fit inside the orbiter's dry mass.
"""
function moonreturn(ls::LandingResult;
                    stage::AscentStage = AscentStage(),
                    stay::Float64 = 21.6 * 3600.0,
                    h_ins::Float64 = 15.0e3,
                    ha_ins::Float64 = 85.0e3,
                    h_ei::Float64 = 50.0e3,
                    capsule::Union{Nothing,Vehicle} = nothing,
                    field::Union{Nothing,LunarGravity} = nothing,
                    fly_entry::Bool = true,
                    verbose::Bool = false)
    orb = ls.orbiter
    orb === nothing &&
        error("this landing left nothing in lunar orbit to come back to — " *
              "pass an `orbiter` to `moonlanding` first")
    ascent_mass(stage) <= ls.lander.mdry + 1.0 ||
        error("the ascent stage ($(round(ascent_mass(stage))) kg wet) does not " *
              "fit inside the lander's dry mass ($(round(ls.lander.mdry)) kg) — " *
              "it has to have been carried down")
    ls.descent.outcome === :touchdown ||
        @warn "flying a return from a descent that did not land cleanly" outcome = ls.descent.outcome

    eph = ls.eph
    # descent times were referenced to ignition; from here on everything is
    # mission time, so the config carries no offset
    cfg = DescentConfig(field = field, eph = eph, t0 = 0.0)
    hhat = _descent_normal(ls.r_orbiter, ls.v_orbiter)
    t_early = ls.t_touchdown + stay
    r_site = moonfixed_inv(moonfixed(ls.descent.r, ls.t_touchdown, eph), t_early, eph)

    # --- ascent program ----------------------------------------------------
    p0, pr, ok = tune_lunar_ascent(stage, r_site, t_early, hhat;
                                   h_ins = h_ins, ha_ins = ha_ins, cfg = cfg,
                                   verbose = verbose)
    ok || error("no ascent pitch program reaches a $(round(h_ins/1e3)) x " *
                "$(round(ha_ins/1e3)) km orbit with this stage " *
                "($(round(ascent_dv(stage))) m/s ideal)")

    # --- lift-off window and rendezvous ------------------------------------
    plan = plan_rendezvous(stage, r_site, hhat, t_early,
                           ls.r_orbiter, ls.v_orbiter, ls.t_loi;
                           pitch0 = p0, pitch_rate = pr, h_ins = h_ins,
                           ha_ins = ha_ins, cfg = cfg, verbose = verbose)
    plan === nothing && error("no rendezvous transfer found in the first " *
                              "revolution after lift-off")
    rz = plan.rz
    rz.feasible ||
        error("the rendezvous costs $(round(rz.dv_total)) m/s and the ascent " *
              "stage has $(round(plan.ascent.prop_left)) kg left, which is " *
              "$(round(rz.prop_used)) kg short")

    # re-fly the chosen ascent with a log this time
    L = AscentMoonLog()
    asc = lunar_ascent(stage, plan.r_site, plan.t_lift, hhat; h_ins = h_ins,
                       ha_ins = ha_ins, pitch0 = p0, pitch_rate = pr,
                       cfg = cfg, log = L)
    t_dock = plan.t_ins + rz.tof
    verbose && @info "rendezvous" liftoff = plan.t_lift dv = rz.dv_total tof = rz.tof

    # --- trans-Earth injection ---------------------------------------------
    # The sweep reads the perigee off the osculating elements once the vehicle
    # is 80,000 km clear of the Moon, and at that range the Moon is still
    # pulling hard enough to move the answer by thousands of kilometres by the
    # time the trajectory actually gets to Earth. So the sweep picks the
    # ignition point and a first magnitude, and then an outer loop flies the
    # whole coast, measures the perigee that really arrives, and moves the
    # inner target by the miss — the same two-level arrangement the free-return
    # design uses, for the same reason.
    tei = trans_earth_injection(rz.r_dock, rz.v_dock, t_dock, eph;
                                h_ei = h_ei, field = field, verbose = verbose)
    tei === nothing &&
        error("no trans-Earth injection in the revolution after docking hits a " *
              "$(round(h_ei/1e3)) km perigee")
    dv_tei, leg, CL = refine_tei(rz.r_dock, rz.v_dock, t_dock, tei.t_ign,
                                 tei.dv, eph; h_ei = h_ei, field = field,
                                 verbose = verbose)
    leg.outcome === :entry_interface ||
        error("the trans-Earth leg did not reach entry interface " *
              "(outcome: $(leg.outcome), perigee $(round(leg.vac_perigee/1e3, digits=1)) km)")
    m_tei = ls.m_orbiter * exp(-dv_tei / (G0 * orb.isp))
    m_tei <= orb.mdry &&
        error("trans-Earth injection ($(round(tei.dv)) m/s) empties the orbiter: " *
              "$(round(ls.m_orbiter - orb.mdry)) kg of propellant left after " *
              "insertion, $(round(ls.m_orbiter - m_tei)) kg needed")

    # --- coast home ---------------------------------------------------------
    cis = CislunarResult(CL, leg.outcome, leg.r, leg.v, leg.t, m_tei, dv_tei,
                         tei.t_ign, 0.0, NaN, NaN, leg.vac_perigee,
                         leg.gamma_end, leg.miss_passes, leg.first_perigee_alt)

    # --- entry --------------------------------------------------------------
    scn = nothing; ent = nothing
    if fly_entry && leg.outcome === :entry_interface
        cap = capsule === nothing ? apollo_capsule(mass = 0.72 * orb.mdry) : capsule
        cap.mass <= orb.mdry + 1.0 ||
            error("the entry capsule ($(round(cap.mass)) kg) does not fit " *
                  "inside the orbiter's dry mass ($(round(orb.mdry)) kg)")
        scn = Scenario(vehicle = cap, r0 = leg.r, v0 = leg.v, t0 = leg.t,
                       t_max = leg.t + 3.0e4, alpha0 = deg2rad_(5.0))
        ent = simulate(scn)
    end

    ReturnResult(asc, stage, orb, plan.t_lift, stay, rz.dv_total, rz.dv1, rz.dv2,
                 t_dock, rz.tof, rz.m_after - stage.mdry, dv_tei, tei.t_ign,
                 m_tei, m_tei - orb.mdry, cis, ent, scn)
end

function print_return_summary(io::IO, rr::ReturnResult, ls::LandingResult)
    a = rr.ascent
    println(io, "== Return from the surface ==")
    @printf(io, "  Surface stay    : %.1f h, lift-off at t=%.2f d\n",
            rr.stay / 3600, rr.t_liftoff / 86400)
    @printf(io, "  Ascent stage    : %.0f kg wet / %.0f kg dry, %.1f kN, %.0f m/s ideal\n",
            ascent_mass(rr.stage), rr.stage.mdry, rr.stage.thrust / 1e3,
            ascent_dv(rr.stage))
    @printf(io, "  Ascent          : %.0f s burn, pitch %.1f° %+.4f °/s, dv %.0f m/s\n",
            a.t_cut, rad2deg_(a.pitch0), rad2deg_(a.pitch_rate), a.dv_ideal)
    @printf(io, "  Insertion       : %.1f x %.1f km, gamma %+.2f°, %.0f kg propellant left\n",
            a.hp / 1e3, a.ha / 1e3, rad2deg_(a.gamma_cut), a.prop_left)
    @printf(io, "  Rendezvous      : dv %.1f m/s (%.1f transfer + %.1f braking), %.0f min coast\n",
            rr.dv_rendezvous, rr.dv_transfer, rr.dv_braking, rr.t_transfer / 60)
    @printf(io, "  Docking         : t=%.2f d, %.0f kg ascent propellant unused\n",
            rr.t_dock / 86400, rr.prop_ascent_left)
    @printf(io, "  TEI             : dv %.1f m/s at t=%.2f d (orbiter %.0f -> %.0f kg, %.0f kg left)\n",
            rr.dv_tei, rr.t_tei / 86400, ls.m_orbiter, rr.m_tei, rr.prop_orbiter_left)
    @printf(io, "  Return perigee  : %.1f km vacuum, gamma_EI %.2f°\n",
            rr.cis.vac_perigee_alt / 1e3, rad2deg_(rr.cis.gamma_end))
    if rr.entry === nothing
        println(io, "  Entry           : not flown (", rr.cis.outcome, ")")
    else
        e = rr.entry
        if e.terminated == :splashdown
            @printf(io, "  Splashdown      : lat=%.2f°, lon=%.2f°, %.1f m/s at t=%.2f d\n",
                    rad2deg_(e.lat_splash), rad2deg_(e.lon_splash), e.v_splash,
                    e.t_splash / 86400)
        else
            println(io, "  DID NOT SPLASH DOWN (", e.terminated, ")")
        end
        @printf(io, "  Entry loads     : %.1f g peak, %.0f W/cm² peak, %.0f MJ/m² heat load\n",
                e.peak_gload, e.peak_qdot / 1e4, e.heat_load / 1e6)
    end
    @printf(io, "  Mission total   : %.2f days pad to splashdown\n",
            (rr.entry === nothing ? rr.cis.t : rr.entry.t_splash) / 86400)
end
print_return_summary(rr::ReturnResult, ls::LandingResult) =
    print_return_summary(stdout, rr, ls)
