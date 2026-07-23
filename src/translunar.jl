# Cislunar flight: finite-burn trans-lunar injection, coast through the
# Earth-Moon system (restricted three-body dynamics: Earth point mass +
# differential lunar gravity from the ephemeris), and free-return targeting.
#
# Propagation is RK4 with a step size scheduled by the local dynamical
# timescale of whichever body dominates: dt ~ eta * sqrt(d^3/mu) for both the
# Earth and Moon distances, capped at `dt_max`. Around perilune this tightens
# to tens of seconds; on the long coast it relaxes to minutes.
#
# Free-return design: two parameters — TLI ignition time along the parking
# orbit (which sets the Earth-Moon phase at departure) and the burn delta-v —
# are shot at two targets: perilune altitude and the vacuum perigee altitude
# of the return leg (which sets the entry flight-path angle). A coarse scan
# seeds a damped 2x2 Newton iteration; every evaluation is a full finite-burn
# + coast simulation, so all three-body and finite-burn couplings are
# absorbed by the iteration, exactly like `target_deorbit` does for entry.

struct CislunarLog
    t::Vector{Float64}
    rx::Vector{Float64}; ry::Vector{Float64}; rz::Vector{Float64}
    vx::Vector{Float64}; vy::Vector{Float64}; vz::Vector{Float64}
    h::Vector{Float64}          # geodetic altitude [m]
    d_moon::Vector{Float64}     # distance to Moon center [m]
    mx::Vector{Float64}; my::Vector{Float64}; mz::Vector{Float64}  # Moon ECI position
    phase::Vector{Int}          # 0 coast(park) | 1 TLI burn | 2 outbound | 3 return
end
CislunarLog() = CislunarLog((Float64[] for _ in 1:12)..., Int[])

function _cis_push!(L::CislunarLog, t, r::V3, v::V3, eph::CircularMoonEphemeris,
                    theta_g0::Float64, phase::Int)
    theta = earth_rotation_angle(theta_g0, t)
    _, _, h = geodetic_from_ecef(rot_z(r, theta))
    m = moon_position(eph, t)
    push!(L.t, t)
    push!(L.rx, r[1]); push!(L.ry, r[2]); push!(L.rz, r[3])
    push!(L.vx, v[1]); push!(L.vy, v[2]); push!(L.vz, v[3])
    push!(L.h, h); push!(L.d_moon, vnorm(vsub(m, r)))
    push!(L.mx, m[1]); push!(L.my, m[2]); push!(L.mz, m[3])
    push!(L.phase, phase)
    nothing
end

struct CislunarResult
    log::CislunarLog
    outcome::Symbol            # :entry_interface | :lunar_impact | :escape | :timeout
    r::V3; v::V3; t::Float64   # state at termination
    m::Float64                 # stack mass after TLI [kg]
    dv_tli::Float64            # delivered TLI delta-v [m/s]
    t_tli::Float64             # TLI ignition time [s]
    burn_duration::Float64
    perilune_alt::Float64      # min altitude above lunar surface [m]
    t_perilune::Float64
    vac_perigee_alt::Float64   # osculating return perigee altitude [m] (NaN unless returning)
    gamma_end::Float64         # inertial FPA at termination [rad]
end

"Two-body + lunar third-body acceleration."
@inline function _cis_accel(r::V3, t::Float64, eph::CircularMoonEphemeris)
    rn = vnorm(r)
    a = vscale(r, -MU_EARTH / rn^3)
    s = moon_position(eph, t)
    d = vsub(s, r)
    dn = vnorm(d); sn = vnorm(s)
    vadd(a, vsub(vscale(d, MU_MOON / dn^3), vscale(s, MU_MOON / sn^3)))
end

"Local-timescale step size [s]."
@inline function _cis_dt(r::V3, t::Float64, eph::CircularMoonEphemeris;
                         eta::Float64 = 0.004, dt_max::Float64 = 240.0)
    re = vnorm(r)
    dm = moon_distance(eph, r, t)
    te = 2pi * sqrt(re^3 / MU_EARTH)
    tm = 2pi * sqrt(dm^3 / MU_MOON)
    min(dt_max, eta * min(te, tm))
end

# One RK4 step of the coast dynamics (allocation-free on tuples).
function _cis_step(r::V3, v::V3, t::Float64, dt::Float64, eph::CircularMoonEphemeris)
    k1v = _cis_accel(r, t, eph);                         k1r = v
    r2 = vadd(r, vscale(k1r, dt/2)); v2 = vadd(v, vscale(k1v, dt/2))
    k2v = _cis_accel(r2, t + dt/2, eph);                 k2r = v2
    r3 = vadd(r, vscale(k2r, dt/2)); v3 = vadd(v, vscale(k2v, dt/2))
    k3v = _cis_accel(r3, t + dt/2, eph);                 k3r = v3
    r4 = vadd(r, vscale(k3r, dt));   v4 = vadd(v, vscale(k3v, dt))
    k4v = _cis_accel(r4, t + dt, eph);                   k4r = v4
    rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), dt/6))
    vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), dt/6))
    (rn, vn)
end

"""
    tli_burn(r, v, t, m0, stage, dv_target, eph; dt=0.5)
        -> (r, v, t, m, dv_delivered, duration, log_ts)

Finite prograde burn: thrust along the inertial velocity until the ideal
delta-v spent (from the mass ratio) reaches `dv_target` or the stage runs
dry. Gravity (including lunar) acts throughout, so gravity losses are
physical rather than modeled.
"""
function tli_burn(r::V3, v::V3, t::Float64, m0::Float64, stage::Stage,
                  dv_target::Float64, eph::CircularMoonEphemeris,
                  prop_avail::Float64; dt::Float64 = 0.5)
    vex = G0 * stage.isp_vac
    m_cut = m0 * exp(-dv_target / vex)
    m_dry_limit = m0 - prop_avail
    m = m0
    md = stage_mdot(stage)
    ts = Float64[]; rs = NTuple{3,Float64}[]
    t0 = t
    while m > m_cut && m > m_dry_limit + 1e-9
        step = min(dt, (m - max(m_cut, m_dry_limit)) / md)
        # RK4 on (r, v, m) with prograde thrust
        acc(rr, vv, mm, tt) = vadd(_cis_accel(rr, tt, eph),
                                   vscale(vunit(vv), stage.thrust_vac / mm))
        k1r = v;                       k1v = acc(r, v, m, t)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2)); m2 = m - md*step/2
        k2r = v2;                      k2v = acc(r2, v2, m2, t + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                      k3v = acc(r3, v3, m2, t + step/2)
        r4 = vadd(r, vscale(k3r, step));   v4 = vadd(v, vscale(k3v, step));   m4 = m - md*step
        k4r = v4;                      k4v = acc(r4, v4, m4, t + step)
        r = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        v = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        m -= md * step
        t += step
        push!(ts, t); push!(rs, r)
    end
    dv_delivered = vex * log(m0 / m)
    (r, v, t, m, dv_delivered, t - t0, ts, rs)
end

"""
    fly_cislunar(r0, v0, t0, eph; t_ign, dv, stage, m_stack, prop_avail,
                 theta_g0=0, h_stop=140e3, t_max=15 days) -> CislunarResult

Coast on the parking orbit to `t_ign`, perform the finite TLI burn, then
coast through the Earth-Moon system until the trajectory descends through
`h_stop` (entry handoff), impacts the Moon, escapes, or times out.
"""
function fly_cislunar(r0::V3, v0::V3, t0::Float64, eph::CircularMoonEphemeris;
                      t_ign::Float64, dv::Float64, stage::Stage,
                      m_stack::Float64, prop_avail::Float64,
                      theta_g0::Float64 = 0.0,
                      h_stop::Float64 = 140.0e3,
                      t_max::Float64 = 30.0 * 86400.0,
                      stop_after_flyby::Bool = false,
                      log_every::Int = 4)
    L = CislunarLog()
    r, v, t = r0, v0, t0

    # --- parking coast ------------------------------------------------------
    kount = 0
    while t < t_ign
        dtp = min(_cis_dt(r, t, eph; dt_max = 30.0), t_ign - t)
        (kount % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, 0)
        r, v = _cis_step(r, v, t, dtp, eph)
        t += dtp
        kount += 1
    end

    # --- TLI burn -----------------------------------------------------------
    r, v, t, m, dv_del, tburn, bts, brs = tli_burn(r, v, t, m_stack, stage, dv, eph, prop_avail)
    for (tb, rb) in zip(bts, brs)
        _cis_push!(L, tb, rb, v, eph, theta_g0, 1)
    end

    # --- translunar / return coast -----------------------------------------
    outcome = :timeout
    peri_alt = Inf; t_peri = NaN
    vac_perigee = NaN
    d_prev = moon_distance(eph, r, t)
    outbound = true
    kount = 0
    gamma_end = NaN
    while t < t0 + t_max
        dtc = _cis_dt(r, t, eph)
        (kount % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, outbound ? 2 : 3)
        kount += 1
        rn_, vn_ = _cis_step(r, v, t, dtc, eph)
        tn = t + dtc

        # lunar events
        dm = moon_distance(eph, rn_, tn)
        alt_m = dm - R_MOON
        if alt_m < peri_alt
            peri_alt = alt_m; t_peri = tn
        end
        if alt_m <= 0.0
            r, v, t = rn_, vn_, tn
            outcome = :lunar_impact
            _cis_push!(L, t, r, v, eph, theta_g0, 2)
            break
        end
        if outbound && dm > d_prev && dm < 0.35 * A_MOON
            outbound = false          # passed perilune, now heading home
        end
        d_prev = dm

        # post-flyby osculating return perigee (usable as a targeting residual
        # long before the trajectory physically descends to the handoff)
        if !outbound && dm > 5.0e7
            el_ = elements_from_state(rn_, vn_)
            vac_perigee = el_.rp - RE_MEAN
        end
        # design-mode early exit: first flyby characterized, residuals ready
        if stop_after_flyby && !outbound && dm > 1.2e8
            r, v, t = rn_, vn_, tn
            outcome = :flyby_complete
            _cis_push!(L, t, r, v, eph, theta_g0, 3)
            break
        end

        # Earth events
        theta = earth_rotation_angle(theta_g0, tn)
        _, _, h = geodetic_from_ecef(rot_z(rn_, theta))
        if h <= h_stop && vdot(rn_, vn_) < 0
            # bisect the last step to the handoff altitude
            lo, hi = 0.0, dtc
            for _ in 1:40
                mid = 0.5 * (lo + hi)
                rm, vm = _cis_step(r, v, t, mid, eph)
                th = earth_rotation_angle(theta_g0, t + mid)
                _, _, hm = geodetic_from_ecef(rot_z(rm, th))
                if hm > h_stop; lo = mid; else; hi = mid; end
                hi - lo < 1e-4 && break
            end
            r, v = _cis_step(r, v, t, hi, eph)
            t += hi
            el = elements_from_state(r, v)
            vac_perigee = el.rp - RE_MEAN
            rhat = vunit(r); vin = vnorm(v)
            gamma_end = asin(clamp(vdot(rhat, vscale(v, 1 / vin)), -1.0, 1.0))
            outcome = :entry_interface
            _cis_push!(L, t, r, v, eph, theta_g0, 3)
            break
        end

        # escape check
        rn2 = vnorm(rn_)
        if rn2 > 2.0 * A_MOON
            r, v, t = rn_, vn_, tn
            outcome = :escape
            break
        end
        r, v, t = rn_, vn_, tn
    end

    CislunarResult(L, outcome, r, v, t, m, dv_del, t_ign, tburn,
                   peri_alt, t_peri, vac_perigee, gamma_end)
end

"""
    seed_free_return(r0, v0; ra_offset=60_000e3) -> (lead, t_flight, dv_seed)

Patched-conic seed for the free-return shooting. The transfer ellipse runs
from the parking radius to an apogee `ra_offset` beyond the lunar distance
(so the lunar-radius crossing is robustly transversal, not a grazing
apogee). Kepler's equation gives the time of flight to the crossing; the
required Moon lead angle at TLI is the crossing true anomaly minus the
Moon's travel during the flight.
"""
function seed_free_return(r0::V3, v0::V3; ra_offset::Float64 = 60_000.0e3)
    rp = vnorm(r0)
    ra = A_MOON + ra_offset
    at = 0.5 * (rp + ra)
    e = (ra - rp) / (ra + rp)
    p = at * (1 - e^2)
    # true anomaly where the transfer crosses the lunar distance
    nux = acos(clamp((p / A_MOON - 1) / e, -1.0, 1.0))
    # eccentric anomaly & Kepler time of flight from perigee
    Ex = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(nux / 2))
    Ex < 0 && (Ex += 2pi)
    tf = sqrt(at^3 / MU_EARTH) * (Ex - e * sin(Ex))
    lead = nux - N_MOON * tf                  # Moon lead angle at TLI [rad]
    vperi = sqrt(MU_EARTH * (2 / rp - 1 / at))
    dv = vperi - vnorm(v0)
    (lead, tf, dv)
end

"""
    tli_alignment_time(r0, v0, t0, eph; lead) -> t_align

First time after `t0 + 600 s` at which the Moon's in-plane angle ahead of
the spacecraft equals `lead` (the patched-conic departure phase).
"""
function tli_alignment_time(r0::V3, v0::V3, t0::Float64,
                            eph::CircularMoonEphemeris, lead::Float64)
    p = vunit(r0)
    h = vcross(r0, v0)
    q = vunit(vcross(h, r0))                 # along-track
    m = moon_position(eph, t0)
    phi0 = atan(vdot(m, q), vdot(m, p))      # Moon angle ahead of s/c at t0
    n_sc = sqrt(MU_EARTH / vnorm(r0)^3)      # ~circular parking rate
    dphi = mod(phi0 - lead, 2pi)             # phase to close (s/c catches up)
    dt = dphi / (n_sc - N_MOON)
    dt < 600.0 && (dt += 2pi / (n_sc - N_MOON))
    t0 + dt
end

"""
    design_free_return(r0, v0, t0, eph; stage, m_stack, prop_avail,
                       hp_moon_target=2000e3, hp_return_target=35e3,
                       max_iter=15, verbose=false) -> (t_ign, dv, result)

Shooting design of the circumlunar free return. Scans TLI ignition time over
one parking revolution around the patched-conic phase seed, then Newton-
iterates (t_ign, dv) on (perilune altitude, return vacuum perigee altitude).
"""
function design_free_return(r0::V3, v0::V3, t0::Float64, eph::CircularMoonEphemeris;
                            stage::Stage, m_stack::Float64, prop_avail::Float64,
                            theta_g0::Float64 = 0.0,
                            hp_moon_target::Float64 = 2000.0e3,
                            hp_return_target::Float64 = 35.0e3,
                            max_iter::Int = 15, verbose::Bool = false)
    # design evaluations stop right after the first flyby: fast, and immune
    # to later-revolution re-encounters contaminating the perilune metric
    fly(tig, dvv) = fly_cislunar(r0, v0, t0, eph; t_ign = tig, dv = dvv,
                                 stage = stage, m_stack = m_stack,
                                 prop_avail = prop_avail, theta_g0 = theta_g0,
                                 stop_after_flyby = true, t_max = 10.0 * 86400.0)

    # residuals scaled to km; a missing return perigee is a large penalty.
    # The perigee residual is measured against a PROXY target: the osculating
    # perigee read shortly after the flyby drifts by a few hundred km over
    # the multi-day return coast (continued lunar tides), so an outer
    # corrector shifts the proxy until the true descent perigee of the
    # verification flight lands on `hp_return_target`.
    # (NB: locals here are deliberately named r1/r2 — assigning f1/f2 inside
    # this closure would capture and clobber the Newton loop's variables.)
    proxy_target = Ref(hp_return_target)
    function resid(tig_, dvv_)
        res_ = fly(tig_, dvv_)
        r1 = isfinite(res_.perilune_alt) ? (res_.perilune_alt - hp_moon_target) / 1e3 : 1.0e5
        r2 = isnan(res_.vac_perigee_alt) ? 1.0e5 :
             (res_.vac_perigee_alt - proxy_target[]) / 1e3
        (r1, r2, res_)
    end

    # --- patched-conic alignment + encounter-window scan --------------------
    lead, tf_seed, dv_seed = seed_free_return(r0, v0)
    t_align = tli_alignment_time(r0, v0, t0, eph, lead)
    verbose && @info "free-return seed" t_align_hr = (t_align - t0)/3600 tf_days = tf_seed/86400 dv_seed

    # The encounter is narrow: one minute of ignition time sweeps the b-plane
    # by ~(n_sc - n_m) * 60 s * a_moon ~ 27,000 km, so a launch-window scan
    # sees an impact zone a few minutes wide, flanked by an escape family
    # (burn too early: the Moon slings the pod outward) and the free-return
    # family (burn later: the pod crosses ahead and is turned back home).
    # Lock onto the EARLIEST encounter (later grid points are slower
    # transfers with much steeper returns), then pick the best full-residual
    # point of a dense sub-scan as the Newton seed.
    peri_of(tig) = fly(tig, dv_seed).perilune_alt
    ts = t_align .+ collect(-1500.0:60.0:1500.0)
    ps = [peri_of(t) for t in ts]
    ifirst = findfirst(i -> i > 1 && i < length(ps) &&
                            ps[i] < 50_000.0e3 &&
                            ps[i] <= ps[i-1] && ps[i] <= ps[i+1], eachindex(ps))
    imin = ifirst === nothing ? argmin(ps) : ifirst
    tmin = ts[imin]
    verbose && @info "free-return scan" t_enc_hr = (tmin - t0)/3600 closest_km = ps[imin]/1e3

    best = (Inf, tmin, dv_seed)
    for dts in -240.0:15.0:300.0
        tig = tmin + dts
        f1, f2, _ = resid(tig, dv_seed)
        score = hypot(f1, min(abs(f2), 5.0e4))
        score < best[1] && (best = (score, tig, dv_seed))
    end
    tig, dvv = best[2], best[3]
    verbose && @info "free-return seedpoint" score = best[1] t_ign_hr = (tig - t0)/3600 dv = dvv

    # --- inner damped Newton on (t_ign, dv) ---------------------------------
    function newton!()
        local f1, f2, res
        for it in 1:max_iter
            f1, f2, res = resid(tig, dvv)
            verbose && @info "free-return newton" it f_perilune_km = f1 f_perigee_km = f2 t_ign_hr = (tig - t0)/3600 dv = dvv outcome = res.outcome
            (abs(f1) < 25.0 && abs(f2) < 2.0) && return true
            d1 = 5.0; d2 = 0.5                  # FD steps: 5 s, 0.5 m/s
            f1a, f2a, _ = resid(tig + d1, dvv)
            f1b, f2b, _ = resid(tig, dvv + d2)
            j11 = (f1a - f1) / d1; j21 = (f2a - f2) / d1
            j12 = (f1b - f1) / d2; j22 = (f2b - f2) / d2
            det = j11 * j22 - j12 * j21
            if abs(det) < 1e-14
                tig += 30.0
                continue
            end
            dt_ = -( j22 * f1 - j12 * f2) / det
            dd_ = -(-j21 * f1 + j11 * f2) / det
            tig += clamp(0.7 * dt_, -120.0, 120.0)
            dvv += clamp(0.7 * dd_, -10.0, 10.0)
        end
        false
    end

    # --- outer corrector: proxy perigee -> true descent perigee -------------
    # full-horizon verification flight of a converged design
    verify(tig_, dvv_) = fly_cislunar(r0, v0, t0, eph; t_ign = tig_, dv = dvv_,
                                      stage = stage, m_stack = m_stack,
                                      prop_avail = prop_avail, theta_g0 = theta_g0)
    local full
    for outer in 1:6
        converged = newton!()
        full = verify(tig, dvv)
        if converged && full.outcome == :entry_interface
            err = full.vac_perigee_alt - hp_return_target
            verbose && @info "free-return corrector" outer true_perigee_km = full.vac_perigee_alt/1e3 err_km = err/1e3
            abs(err) < 3.0e3 && return (tig, dvv, full)
            proxy_target[] -= err
        else
            @warn "free-return design stalled" converged outcome = full.outcome
            break
        end
    end
    f1, f2, res = resid(tig, dvv)
    @warn "free-return targeting did not fully converge" f_perilune_km = f1 f_perigee_km = f2 outcome = full.outcome
    (tig, dvv, full)
end
