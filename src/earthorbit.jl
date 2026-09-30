# Earth-orbit missions: launch -> parking orbit -> transfer burns on the kick
# stage -> the target orbit, with an optional deorbit and entry. No Moon in
# the objective, though the Moon rides along in the dynamics and the log —
# the coast uses the same Earth + lunar-tide propagator as the cislunar leg,
# and reusing `CislunarLog` verbatim is what lets every existing consumer
# (the panel's scene payload, the launch page's eci world) fly these
# missions without learning a new format.
#
# The burn model is velocity-to-gain: each planned burn computes the desired
# inertial velocity AT the burn point (from the target orbit's geometry) and
# thrusts along the instantaneous to-go vector until it is spent, the
# alignment overshoots, or the stage runs dry. Gravity acts throughout, so
# gravity losses are physical, and the achieved orbit is whatever the finite
# burn actually produced — reported, not asserted.
#
# GEO's plane change rides the standard GTO profile: the raise burn happens
# at an equator crossing of the parking orbit, which puts the transfer
# apogee at the opposite node, where the combined circularise-and-plane-
# change is cheapest and geometrically clean. Argument of perigee is a
# consequence of where the raise burn happens, not a commanded quantity —
# Molniya's achieved argp is reported alongside its reference, and
# commanding it would need a coast-phasing search this deliberately omits.

"""
    OrbitTarget

A named Earth orbit: perigee and apogee altitudes above the mean radius and
the inclination. Catalogue entries are defaults the caller may override.
"""
struct OrbitTarget
    name::Symbol
    perigee_alt::Float64      # [m] above RE_MEAN
    apogee_alt::Float64       # [m]
    inclination::Float64      # [rad]
    note::String
end

const ORBITS = Dict{Symbol,OrbitTarget}(
    :leo     => OrbitTarget(:leo, 200.0e3, 200.0e3, deg2rad_(28.5),
                            "direct ascent, no transfer"),
    :polar   => OrbitTarget(:polar, 500.0e3, 500.0e3, deg2rad_(90.0),
                            "polar; the ascent flies the azimuth, two burns raise the circle"),
    :geo     => OrbitTarget(:geo, 35786.0e3, 35786.0e3, 0.0,
                            "raise at the node, combined circularise + plane change at apogee"),
    :molniya => OrbitTarget(:molniya, 600.0e3, 39400.0e3, deg2rad_(63.4),
                            "critical inclination; argp is reported, not commanded"),
)

struct EarthOrbitResult
    lv::LaunchVehicle
    guid::AscentGuidance
    ascent::AscentResult
    eph::CircularMoonEphemeris
    log::CislunarLog
    burns::Vector{NamedTuple}   # (name, t_ign, duration, dv_plan, dv_delivered)
    elements::NamedTuple        # osculating elements at end of mission
    target::OrbitTarget
    on_target::Bool
    outcome::Symbol             # :on_orbit | :splashdown | :ascent_failed | :prop_depleted
    r::V3; v::V3; t::Float64
    m::Float64                  # stack mass at end (before entry, if any)
    entry_scn::Union{Nothing,Scenario}
    entry::Union{Nothing,SimResult}
end

# ------------------------------------------------------------------ coasts --

"Coast for a fixed duration, logging with the given phase tag."
function _eo_coast_time!(L::CislunarLog, r::V3, v::V3, t::Float64,
                         dt_total::Float64, eph::CircularMoonEphemeris;
                         phase::Int, theta_g0::Float64 = 0.0,
                         log_every::Int = 4)
    tend = t + dt_total
    kount = 0
    while t < tend
        dtp = min(_cis_dt(r, t, eph; dt_max = 120.0), tend - t)
        r, v = _cis_step(r, v, t, dtp, eph)
        t += dtp
        ((kount += 1) % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, phase)
    end
    (r, v, t)
end

"""
Coast until `f(r, v)` crosses zero on a falling edge (`dir = -1`, i.e. from
positive to negative — an apoapsis when `f` is the radial rate), a rising
edge (`dir = +1`), or either (`dir = 0`, a node when `f` is z). The edge
matters: right after an off-apsis burn the radial rate can start with either
sign, and an either-edge apsis search is how a circularisation ends up fired
at perigee. Arms only once `f` has been seen on the approach side, then
bisects the final step so the returned state sits on the crossing.
"""
function _eo_coast_until!(L::CislunarLog, r::V3, v::V3, t::Float64,
                          f::Function, eph::CircularMoonEphemeris;
                          dir::Int = 0, phase::Int, theta_g0::Float64 = 0.0,
                          t_max::Float64 = 3.0 * 86400.0, log_every::Int = 4)
    tend = t + t_max
    s_prev = f(r, v)
    armed = dir == 0 ? s_prev != 0.0 : s_prev * dir < 0.0
    kount = 0
    while t < tend
        dtp = _cis_dt(r, t, eph; dt_max = 120.0)
        rp_, vp_, tp_ = r, v, t
        r, v = _cis_step(r, v, t, dtp, eph)
        t += dtp
        s = f(r, v)
        crossed = armed && s_prev * s < 0.0 &&
                  (dir == 0 || s * dir > 0.0)
        if crossed
            # bisect [tp_, t] by re-stepping from the pre-crossing state
            lo_r, lo_v, lo_t = rp_, vp_, tp_
            width = dtp
            for _ in 1:12
                width /= 2
                rm, vm = _cis_step(lo_r, lo_v, lo_t, width, eph)
                if f(rm, vm) * s_prev > 0.0
                    lo_r, lo_v, lo_t = rm, vm, lo_t + width
                end
            end
            _cis_push!(L, lo_t, lo_r, lo_v, eph, theta_g0, phase)
            return (lo_r, lo_v, lo_t)
        end
        armed |= dir == 0 ? s != 0.0 : s * dir < 0.0
        s_prev = s
        ((kount += 1) % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, phase)
    end
    (r, v, t)   # never crossed inside t_max; the caller's tolerance decides
end

# ------------------------------------------------------------------- burns --

"""
Finite steered burn: thrust along `dirfn(r, v)` until `stopfn(r, v)` is
satisfied, the stage runs dry, or `t_burn_max` elapses. Steering by
direction with an orbit-shape cutoff — prograde until the apoapsis reaches
its target, say — keeps gravity losses honest-but-small; the first cut of
this chased a fixed target *velocity* instead, and a 15 kN kick stage spent
three times the plan fighting to hold a stale vector against gravity.
Returns the end state, the delivered delta-v, the duration, and whether
propellant ran out first.
"""
function _eo_burn!(L::CislunarLog, r::V3, v::V3, t::Float64, m::Float64,
                   stage::Stage, dirfn::Function, stopfn::Function,
                   eph::CircularMoonEphemeris, prop_avail::Float64;
                   theta_g0::Float64 = 0.0, dt::Float64 = 0.5,
                   log_every::Int = 2, t_burn_max::Float64 = 2400.0)
    vex = G0 * stage.isp_vac
    md = stage_mdot(stage)
    m_dry = m - prop_avail
    m0, t0 = m, t
    kount = 0
    dry = false
    onestep = function (r, v, t, m, step)
        uhat = dirfn(r, v)
        acc(rr, vv, mm, tt) = vadd(_cis_accel(rr, tt, eph),
                                   vscale(uhat, stage.thrust_vac / mm))
        k1r = v;                           k1v = acc(r, v, m, t)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2))
        m2 = m - md * step/2
        k2r = v2;                          k2v = acc(r2, v2, m2, t + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                          k3v = acc(r3, v3, m2, t + step/2)
        r4 = vadd(r, vscale(k3r, step));   v4 = vadd(v, vscale(k3v, step))
        k4r = v4;                          k4v = acc(r4, v4, m - md * step, t + step)
        (vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6)),
         vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6)),
         t + step, m - md * step)
    end
    while !stopfn(r, v)
        if m <= m_dry + 1e-9
            dry = true
            break
        end
        t - t0 > t_burn_max && (dry = true; break)
        step = min(dt, (m - m_dry) / md)
        rn, vn, tn, mn = onestep(r, v, t, m, step)
        if stopfn(rn, vn)
            # the cutoff fell inside this step: bisect the step width, or a
            # 0.5 s discretisation overshoots a GTO apoapsis by ~40 km
            for _ in 1:6
                step /= 2
                rh, vh, th, mh = onestep(r, v, t, m, step)
                stopfn(rh, vh) || ((r, v, t, m) = (rh, vh, th, mh))
            end
            rn, vn, tn, mn = onestep(r, v, t, m, step)
        end
        r, v, t, m = rn, vn, tn, mn
        ((kount += 1) % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, 1)
    end
    (r, v, t, m, vex * log(m0 / m), t - t0, dry)
end

"""
Finite velocity-to-gain burn for the shape manoeuvre: `vdesfn(r, v)` gives
the desired inertial velocity at the CURRENT position (recomputed every
step, so nothing chases a stale vector), thrust follows the instantaneous
to-go, and the burn ends when the to-go is spent or its projection on the
initial direction flips. At an apsis, matching the target orbit's velocity
pins periapsis, apoapsis AND plane at once — which is why the shape burn
cannot use a single-element cutoff: cutting on the periapsis alone quit
with a polar mission still 7° out of plane.
"""
function _eo_burn_vg!(L::CislunarLog, r::V3, v::V3, t::Float64, m::Float64,
                      stage::Stage, vdesfn::Function,
                      eph::CircularMoonEphemeris, prop_avail::Float64;
                      theta_g0::Float64 = 0.0, dt::Float64 = 0.5,
                      log_every::Int = 2, t_burn_max::Float64 = 2400.0)
    u0 = vunit(vsub(vdesfn(r, v), v))
    dirfn = (rr, vv) -> begin
        togo = vsub(vdesfn(rr, vv), vv)
        n = vnorm(togo)
        n > 1e-9 ? vscale(togo, 1.0 / n) : u0
    end
    stopfn = (rr, vv) -> begin
        togo = vsub(vdesfn(rr, vv), vv)
        vnorm(togo) < 0.5 || vdot(togo, u0) <= 0.0
    end
    _eo_burn!(L, r, v, t, m, stage, dirfn, stopfn, eph, prop_avail;
              theta_g0 = theta_g0, dt = dt, log_every = log_every,
              t_burn_max = t_burn_max)
end

"""
Unit thrust direction in the local horizontal, lying in the plane of
inclination `inc` through `r` (or the current plane when `inc` is NaN),
pointed along the motion. At an apsis this is prograde-in-the-target-plane:
it raises the opposite apsis and turns the plane in one burn — the combined
manoeuvre that makes GEO affordable.

The plane construction: a unit normal `n` perpendicular to `r` with
`n_z = cos(inc)` exists only when the latitude of `r` does not exceed the
inclination; below that the closest achievable plane (inclination = |lat|)
is used, which is the honest geometric limit of a plane change at this
point. The sign ambiguity resolves toward the current orbit normal so the
burn never asks for a direction reversal.
"""
function _eo_plane_dir(r::V3, v_now::V3, inc::Float64)
    rhat = vunit(r)
    n = if isnan(inc)
        vunit(vcross(r, v_now))
    else
        h_now = vunit(vcross(r, v_now))
        zhat = (0.0, 0.0, 1.0)
        zr = vdot(zhat, rhat)                       # sin(latitude)
        u = vunit(vsub(zhat, vscale(rhat, zr)))     # in-meridian, ⟂ r
        w = vunit(vcross(rhat, zhat))               # ⟂ r and ⟂ z
        alpha = clamp(cos(inc) / max(u[3], 1e-9), -1.0, 1.0)
        beta = sqrt(max(1.0 - alpha^2, 0.0))
        n1 = vadd(vscale(u, alpha), vscale(w, beta))
        n2 = vadd(vscale(u, alpha), vscale(w, -beta))
        vdot(n1, h_now) >= vdot(n2, h_now) ? n1 : n2
    end
    d = vunit(vcross(n, rhat))
    vdot(d, v_now) < 0.0 ? vscale(d, -1.0) : d
end

# ----------------------------------------------------------------- mission --

"""
    earthorbit(; target=:leo, lv, h_park, n_orbits, deorbit, hp_entry, ...)
        -> EarthOrbitResult

Design and fly an Earth-orbit mission: ascent to the parking orbit at the
target's inclination, transfer burns on the kick stage when the target orbit
differs from the parking orbit, `n_orbits` revolutions of the achieved
orbit, and optionally a retrograde deorbit burn sized to drop the vacuum
perigee to `hp_entry`, handing the pod to the existing entry simulator.

`perigee_alt`/`apogee_alt`/`inclination` override the catalogue entry.
Under `strict = false` a failed ascent returns with `outcome =
:ascent_failed` and everything that did fly, instead of throwing.
"""
function earthorbit(; target::Symbol = :leo,
                    lv::Union{Nothing,LaunchVehicle} = nothing,
                    pod_mass::Float64 = 350.0,
                    h_park::Float64 = 200.0e3,
                    perigee_alt::Float64 = NaN,
                    apogee_alt::Float64 = NaN,
                    inclination::Float64 = NaN,
                    n_orbits::Float64 = 2.0,
                    deorbit::Bool = false,
                    hp_entry::Float64 = 25.0e3,
                    kick_angle::Float64 = deg2rad_(8.0),
                    optimize_kick::Bool = false,
                    theta_g0::Float64 = 0.0,
                    site_lat::Float64 = deg2rad_(28.5),
                    site_lon::Float64 = deg2rad_(-80.6),
                    strict::Bool = true,
                    verbose::Bool = false)
    haskey(ORBITS, target) || target === :custom ||
        throw(ArgumentError("unknown orbit target $target; have custom, $(join(sort(collect(keys(ORBITS))), ", "))"))
    tgt0 = target === :custom ?
           OrbitTarget(:custom, 200.0e3, 200.0e3, deg2rad_(28.5),
                       "operator-defined perigee, apogee, and inclination") :
           ORBITS[target]
    hp = isnan(perigee_alt) ? tgt0.perigee_alt : perigee_alt
    ha = isnan(apogee_alt) ? tgt0.apogee_alt : apogee_alt
    inc = isnan(inclination) ? tgt0.inclination : inclination
    hp >= 100.0e3 || throw(ArgumentError("target perigee must be at least 100 km"))
    ha >= hp || throw(ArgumentError("target apogee must be at or above perigee"))
    0.0 <= inc <= pi || throw(ArgumentError("inclination must be between 0 and 180 degrees"))
    tgt = OrbitTarget(tgt0.name, hp, ha, inc, tgt0.note)
    lv === nothing && (lv = default_moon_rocket(payload = pod_mass))
    pod_mass = lv.payload_mass

    # ascent into the parking orbit at the target plane (launch_azimuth clamps
    # an inclination below the site latitude to the site latitude — for GEO
    # that is exactly right: the remainder is the apogee plane change's job)
    az = launch_azimuth(tgt.inclination, site_lat)
    guid0 = AscentGuidance(azimuth = az, h_target = h_park,
                           kick_angle = kick_angle, site_lat = site_lat,
                           site_lon = site_lon)
    guid, asc = tune_ascent(lv, guid0; optimize_kick = optimize_kick,
                            theta_g0 = theta_g0, verbose = verbose)
    # The spherical azimuth formula is several degrees off at steep
    # inclinations (the repo's own tests measure ~2° at 51.6°; a polar
    # command misses by ~6). One corrective re-fly — command the mirror of
    # the measured miss — brings the achieved plane within about a degree,
    # which is the difference between a cheap apogee touch-up and a
    # 900 m/s plane change the kick stage may not have.
    if asc.reached_orbit && tgt.inclination >= site_lat
        miss = tgt.inclination - asc.elements.i
        if abs(miss) > deg2rad_(1.0)
            az2 = launch_azimuth(tgt.inclination + miss, site_lat)
            guid2 = AscentGuidance(azimuth = az2, h_target = h_park,
                                   kick_angle = kick_angle, site_lat = site_lat,
                                   site_lon = site_lon)
            g2, a2 = tune_ascent(lv, guid2; optimize_kick = optimize_kick,
                                 theta_g0 = theta_g0, verbose = verbose)
            if a2.reached_orbit &&
               abs(a2.elements.i - tgt.inclination) < abs(miss)
                guid, asc = g2, a2
            end
        end
    end
    eph_seed = coplanar_moon((RE_MEAN + h_park, 0.0, 0.0), (0.0, 7.8e3, 0.0))
    if !(asc.reached_orbit &&
         asc.elements.rp > RE_MEAN + 0.5 * h_park &&
         abs(asc.gamma_cut) < deg2rad_(1.0))
        strict && error("ascent failed to reach orbit (h_cut=$(asc.h_cut/1e3) km, gamma=$(rad2deg_(asc.gamma_cut))°)")
        return EarthOrbitResult(lv, guid, asc, eph_seed, CislunarLog(),
                                NamedTuple[], asc.elements, tgt, false,
                                :ascent_failed, asc.r, asc.v, asc.t, asc.m,
                                nothing, nothing)
    end

    # jettison spent lower stages: kick stage + payload make the orbital stack
    m_stack = asc.m
    for k in 1:length(lv.stages)-1
        if asc.prop_left[k] > 0
            m_stack -= lv.stages[k].mdry + asc.prop_left[k]
        end
    end
    kick = lv.stages[end]
    prop = asc.prop_left[end]

    # the Moon is scenery here, but real scenery: same plane construction the
    # lunar missions use, so the sky and the log format match everywhere
    eph = coplanar_moon(asc.r, asc.v)
    L = CislunarLog()
    r, v, t, m = asc.r, asc.v, asc.t, m_stack
    _cis_push!(L, t, r, v, eph, theta_g0, 0)

    rp_t = RE_MEAN + tgt.perigee_alt
    ra_t = RE_MEAN + tgt.apogee_alt
    el0 = elements_from_state(r, v)
    Tpark = 2pi * sqrt(el0.a^3 / MU_EARTH)
    need_size = abs(el0.ra - ra_t) > 10.0e3 || abs(el0.rp - rp_t) > 10.0e3
    need_plane = abs(el0.i - tgt.inclination) > deg2rad_(0.5)

    burns = NamedTuple[]
    dry = false
    rdot(rr, vv) = vdot(rr, vv)
    prograde  = (rr, vv) -> vunit(vv)
    retro     = (rr, vv) -> vscale(vunit(vv), -1.0)
    # impulsive plans, for the ledger and the honesty check on gravity loss
    vcirc(rn) = sqrt(MU_EARTH / rn)
    vis(rn, a) = sqrt(MU_EARTH * max(2.0 / rn - 1.0 / a, 1.0e-12))

    if need_size || need_plane
        # burn 1 at an equator crossing when the plane must move (so the
        # transfer apogee lands on the opposite node); anywhere on the circle
        # otherwise, after a partial revolution for the view's sake
        if need_plane
            r, v, t = _eo_coast_until!(L, r, v, t, (rr, vv) -> rr[3], eph;
                                       dir = 0, phase = 0, theta_g0 = theta_g0,
                                       t_max = 2.0 * Tpark)
        else
            r, v, t = _eo_coast_time!(L, r, v, t, 0.3 * Tpark, eph;
                                      phase = 0, theta_g0 = theta_g0)
        end
        # raise the apoapsis: prograde until the osculating ra is there
        rn1 = vnorm(r)
        plan1 = vis(rn1, 0.5 * (rn1 + ra_t)) - vnorm(v)
        m_pre = m
        r, v, t, m, dv1, dur1, dry1 =
            _eo_burn!(L, r, v, t, m, kick, prograde,
                      (rr, vv) -> elements_from_state(rr, vv).ra >= ra_t,
                      eph, prop; theta_g0 = theta_g0)
        prop = max(prop - (m_pre - m), 0.0)
        push!(burns, (name = :raise, t_ign = t - dur1, duration = dur1,
                      dv_plan = plan1, dv = dv1))
        dry |= dry1
        if !dry
            # coast up to the transfer apoapsis (a falling edge of the radial
            # rate — an either-edge search is how the first cut of this fired
            # a circularisation at perigee), then shape the final orbit:
            # thrust in the target plane's horizontal until the periapsis is
            # up, which raises rp and turns the plane in the same burn
            r, v, t = _eo_coast_until!(L, r, v, t, rdot, eph;
                                       dir = -1, phase = 2, theta_g0 = theta_g0,
                                       t_max = 1.5 * 2pi * sqrt((0.5*(vnorm(r)+ra_t))^3 / MU_EARTH))
            rn2 = vnorm(r)
            di = need_plane ? abs(el0.i - tgt.inclination) : 0.0
            v1 = vnorm(v)
            v2 = vis(rn2, 0.5 * (rp_t + ra_t))
            plan2 = sqrt(max(v1^2 + v2^2 - 2*v1*v2*cos(di), 0.0))
            a_f = 0.5 * (rp_t + ra_t)
            e_f = (ra_t - rp_t) / (ra_t + rp_t)
            h_f = sqrt(MU_EARTH * a_f * (1.0 - e_f^2))
            vdesfn = (rr, vv) -> begin
                rn = vnorm(rr)
                vp = min(h_f / rn, vis(rn, a_f))
                vscale(_eo_plane_dir(rr, vv,
                                     need_plane ? tgt.inclination : NaN), vp)
            end
            m_pre = m
            r, v, t, m, dv2, dur2, dry2 =
                _eo_burn_vg!(L, r, v, t, m, kick, vdesfn, eph, prop;
                             theta_g0 = theta_g0)
            prop = max(prop - (m_pre - m), 0.0)
            push!(burns, (name = :shape, t_ign = t - dur2, duration = dur2,
                          dv_plan = plan2, dv = dv2))
            dry |= dry2
        end
    end

    # the mission orbit: n_orbits revolutions of whatever was achieved. The
    # on-target verdict is taken HERE — a later deorbit deliberately wrecks
    # the perigee, and judging the mission by its post-deorbit elements
    # would fail every successful re-entry flight.
    el1 = elements_from_state(r, v)
    tol_r = max(10.0e3, 0.005 * ra_t)
    on_target = !dry &&
                abs(el1.rp - rp_t) <= tol_r && abs(el1.ra - ra_t) <= tol_r &&
                abs(el1.i - tgt.inclination) <= deg2rad_(1.0)
    Tfin = el1.a > 0 ? 2pi * sqrt(el1.a^3 / MU_EARTH) : Tpark
    r, v, t = _eo_coast_time!(L, r, v, t, max(n_orbits, 0.25) * Tfin, eph;
                              phase = 3, theta_g0 = theta_g0)

    # optional deorbit: retrograde at apoapsis (any point of a circle), until
    # the vacuum perigee sits at hp_entry; the entry simulator coasts the
    # rest of the way down itself
    entry_scn = nothing
    entry = nothing
    if deorbit && !dry
        eln = elements_from_state(r, v)
        if eln.e > 0.01
            r, v, t = _eo_coast_until!(L, r, v, t, rdot, eph;
                                       dir = -1, phase = 3, theta_g0 = theta_g0,
                                       t_max = 1.5 * Tfin)
        end
        rnd = vnorm(r)
        pland = vnorm(v) - vis(rnd, 0.5 * (rnd + RE_MEAN + hp_entry))
        r, v, t, m, dvd, durd, dryd =
            _eo_burn!(L, r, v, t, m, kick, retro,
                      (rr, vv) -> elements_from_state(rr, vv).rp <= RE_MEAN + hp_entry,
                      eph, prop; theta_g0 = theta_g0)
        push!(burns, (name = :deorbit, t_ign = t - durd, duration = durd,
                      dv_plan = pland, dv = dvd))
        dry |= dryd
        if !dryd
            pod = default_reentry_pod(mass = pod_mass)
            entry_scn = Scenario(vehicle = pod, r0 = r, v0 = v, t0 = t,
                                 t_max = t + 3.0e4, theta_g0 = theta_g0,
                                 alpha0 = deg2rad_(5.0))
            entry = simulate(entry_scn)
        end
    end

    elf = elements_from_state(r, v)
    outcome = entry !== nothing ? :splashdown :
              dry ? :prop_depleted : :on_orbit
    EarthOrbitResult(lv, guid, asc, eph, L, burns, elf, tgt,
                     on_target, outcome, r, v, t, m,
                     entry_scn, entry)
end
