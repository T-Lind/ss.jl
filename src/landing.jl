# Lunar arrival and powered descent: lunar-orbit insertion at perilune,
# descent-orbit initiation, and a guided powered descent to touchdown.
#
# The flyby mission treats perilune as a place to pass through. Everything
# here happens because you decided to stop there instead, and the sequence is
# Apollo's, for Apollo's reasons:
#
#   1. arrive on a **free return** (`translunar_design`) so a failure to
#      insert is a trip home rather than a solar orbit,
#   2. **LOI** at perilune — the one place on the hyperbola where the burn is
#      purely retrograde and cheapest — into a circular parking orbit,
#   3. coast whole revolutions, which is what moves the landing site: the
#      ground track under a fixed orbit walks with the Moon's rotation and
#      with where in the orbit you choose to start down,
#   4. **DOI**: a small retrograde burn dropping periapsis to the descent
#      altitude, half a revolution before the landing site,
#   5. **powered descent** from that periapsis: a braking phase flown at full
#      thrust on a linear pitch program, closed by shooting on (pitch, pitch
#      rate) at a high-gate state, then a throttled terminal phase that flies
#      a commanded descent-rate profile to the surface.
#
# Frames. Everything from perilune on is integrated in a Moon-centred frame
# that falls freely with the Moon. That is not a convenience: the residual
# Earth term in that frame is the *tidal* difference, 2*mu_E*r/d^3, which at
# a 100 km lunar orbit is 2.5e-5 m/s^2 — 1.5e-5 of lunar gravity, four orders
# below the modelling error in a point-mass Moon. Lunar gravity is spherical
# here; the real Moon's mascons perturb a low orbit by kilometres per
# revolution, which is why real descents navigate rather than propagate.
#
# What is *not* modelled: terrain (the surface is a sphere of radius R_MOON),
# landing-radar updates, redesignation, hazard avoidance, and abort. The
# guidance flies open-loop in the braking phase and closed-loop on velocity
# in the terminal phase, which is enough to price a descent honestly.

"""
    Lander(name, mdry, mprop, thrust, isp, throttle_min, diameter)

A single-stage lander: everything that leaves the trans-lunar stack and puts
itself on the surface. `thrust` [N] is full-throttle vacuum thrust, and
`throttle_min` the deepest fraction the engine can hold — the number that
decides whether the vehicle can fly the last kilometre at all, because a
lander arrives at the surface light and a fixed-thrust engine would simply
push it back up.
"""
Base.@kwdef struct Lander
    name::Symbol = :lander
    mdry::Float64 = 3500.0
    mprop::Float64 = 9000.0
    thrust::Float64 = 45.0e3
    isp::Float64 = 311.0
    throttle_min::Float64 = 0.10
    diameter::Float64 = 4.2
end

"Wet mass of the lander [kg] — what the launcher has to throw."
lander_mass(l::Lander) = l.mdry + l.mprop

"Full-throttle mass flow [kg/s]."
@inline lander_mdot(l::Lander) = l.thrust / (G0 * l.isp)

"""
    default_lander(; payload = 0.0)

Apollo-LM-class single-stage lander: 12.5 t wet, 9 t of hypergolic
propellant, one 45 kN engine throttleable to 10%. `payload` adds surface
cargo to the dry mass. It carries enough for insertion, descent-orbit
initiation and the descent itself (about 2.8 km/s all told) with a real
hover margin left over.
"""
default_lander(; payload::Float64 = 0.0) =
    Lander(mdry = 3500.0 + payload, mprop = 9000.0, thrust = 45.0e3,
           isp = 311.0, throttle_min = 0.10, diameter = 4.2)

"Ideal vacuum delta-v the lander carries [m/s]."
lander_dv(l::Lander) = G0 * l.isp * log(lander_mass(l) / l.mdry)

# ---------------------------------------------------------------- logging --

"""
Powered-descent log. `downrange` is arc length over the sphere from the point
under the vehicle at ignition, so it is directly comparable with the maps
Apollo's crews used; `throttle` is the commanded fraction, which is the
number that says whether the engine could actually fly the trajectory.
"""
struct DescentLog
    t::Vector{Float64}          # seconds from powered-descent ignition
    h::Vector{Float64}          # altitude above the mean sphere [m]
    downrange::Vector{Float64}  # surface arc from ignition [m]
    v::Vector{Float64}          # speed in the Moon frame [m/s]
    vh::Vector{Float64}         # horizontal (along-track) component [m/s]
    vv::Vector{Float64}         # vertical (radial) component [m/s]
    m::Vector{Float64}          # mass [kg]
    throttle::Vector{Float64}   # commanded fraction of full thrust
    pitch::Vector{Float64}      # thrust elevation above local horizontal [rad]
    x::Vector{Float64}; y::Vector{Float64}; z::Vector{Float64}  # Moon-centred [m]
end
DescentLog() = DescentLog((Float64[] for _ in 1:12)...)

"""
Coast log in the lunar parking / descent orbit, Moon-centred inertial.
"""
struct LunarOrbitLog
    t::Vector{Float64}
    x::Vector{Float64}; y::Vector{Float64}; z::Vector{Float64}
    h::Vector{Float64}
    phase::Vector{Int}          # 0 parking orbit | 1 descent ellipse
end
LunarOrbitLog() = LunarOrbitLog(Float64[], Float64[], Float64[], Float64[],
                                Float64[], Int[])

"""
Products of a powered descent: the log, the touchdown state, and the numbers
that decide whether the vehicle survived it.
"""
struct DescentResult
    log::DescentLog
    outcome::Symbol             # :touchdown | :crash | :propellant | :diverged
    t_touchdown::Float64        # seconds from PDI
    t_gate::Float64             # seconds from PDI to high gate (end of braking)
    v_vertical::Float64         # touchdown sink rate [m/s] (positive = down)
    v_horizontal::Float64       # touchdown lateral speed [m/s]
    downrange::Float64          # surface arc flown from ignition [m]
    dv_braking::Float64         # ideal delta-v spent braking [m/s]
    dv_terminal::Float64        # ideal delta-v spent below high gate [m/s]
    prop_used::Float64          # [kg]
    prop_left::Float64          # [kg]
    hover_s::Float64            # seconds of hover the residual buys at touchdown mass
    min_throttle::Float64       # deepest commanded throttle
    pitch0::Float64             # braking-phase initial pitch [rad]
    pitch_rate::Float64         # braking-phase pitch rate [rad/s]
    r::V3; v::V3; m::Float64    # touchdown state, Moon-centred
end

"""
The whole landing mission: the launch and trans-lunar legs it shares with
`moonshot`, then insertion, the lunar-orbit coast, and the descent.
"""
struct LandingResult
    lv::LaunchVehicle
    lander::Lander
    guid::AscentGuidance
    ascent::AscentResult
    eph::CircularMoonEphemeris
    cislunar::CislunarResult    # truncated at perilune
    orbit::LunarOrbitLog
    descent::DescentResult
    dv_loi::Float64
    dv_doi::Float64
    t_loi::Float64              # mission time of insertion [s]
    t_doi::Float64              # mission time of descent-orbit initiation [s]
    t_pdi::Float64              # mission time of powered-descent ignition [s]
    t_touchdown::Float64        # mission time of touchdown [s]
    h_park_moon::Float64
    h_pdi::Float64
    n_rev::Int
    lat_land::Float64           # selenographic latitude [rad]
    lon_land::Float64           # longitude from the sub-Earth meridian [rad]
    prop_margin::Float64        # lander propellant left at touchdown [kg]
end

# ------------------------------------------------------- Moon-frame basics --

"Lunar point-mass acceleration in the Moon-centred frame."
@inline _moon_accel(r::V3) = vscale(r, -MU_MOON / vnorm(r)^3)

"RK4 step of a ballistic Moon-centred coast."
function _moon_step(r::V3, v::V3, dt::Float64)
    k1v = _moon_accel(r);                       k1r = v
    r2 = vadd(r, vscale(k1r, dt/2)); v2 = vadd(v, vscale(k1v, dt/2))
    k2v = _moon_accel(r2);                      k2r = v2
    r3 = vadd(r, vscale(k2r, dt/2)); v3 = vadd(v, vscale(k2v, dt/2))
    k3v = _moon_accel(r3);                      k3r = v3
    r4 = vadd(r, vscale(k3r, dt));   v4 = vadd(v, vscale(k3v, dt))
    k4v = _moon_accel(r4);                      k4r = v4
    (vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), dt/6)),
     vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), dt/6)))
end

"Moon-centred state [m, m/s] of an ECI state at time `t`."
mci_state(r::V3, v::V3, t::Float64, eph::CircularMoonEphemeris) =
    (vsub(r, moon_position(eph, t)), vsub(v, moon_velocity(eph, t)))

"""
    selenographic(r_m, t, eph) -> (lat, lon)

Latitude and longitude of a Moon-centred position, in the tidally-locked
Moon-fixed frame: longitude 0 is the sub-Earth meridian, +90° leads in the
direction of orbital motion, and ±180° is the middle of the far side.
Latitude is measured from the ephemeris plane. With a coplanar circular Moon
that plane is the mission plane rather than the true lunar equator, so the
latitude is a plane-relative figure — but the longitude, which is what says
near side or far side, is exactly right, because synchronous rotation ties
the Moon-fixed frame to the Earth-Moon line by construction.
"""
function selenographic(r_m::V3, t::Float64, eph::CircularMoonEphemeris)
    s = moon_position(eph, t)
    xhat = vunit(vscale(s, -1.0))               # Moon -> Earth
    zhat = vunit(vcross(s, moon_velocity(eph, t)))
    yhat = vcross(zhat, xhat)
    u = vunit(r_m)
    lat = asin(clamp(vdot(u, zhat), -1.0, 1.0))
    lon = atan(vdot(u, yhat), vdot(u, xhat))
    (lat, lon)
end

# ------------------------------------------------ coast to perilune (ECI) --

"""
    fly_to_perilune(r0, v0, t0, eph; t_ign, dv, stage, m_stack, prop_avail, eta)
        -> CislunarResult

The outbound half of the trans-lunar flight: parking coast, finite TLI burn,
then coast until closest approach to the Moon, located by bisection on the
range rate. Same dynamics and the same log as `fly_cislunar` — this simply
stops where the landing mission stops caring about the free return.
"""
function fly_to_perilune(r0::V3, v0::V3, t0::Float64, eph::CircularMoonEphemeris;
                         t_ign::Float64, dv::Float64, stage::Stage,
                         m_stack::Float64, prop_avail::Float64,
                         theta_g0::Float64 = 0.0, eta::Float64 = CIS_ETA,
                         t_max::Float64 = 12.0 * 86400.0, log_every::Int = 4)
    L = CislunarLog()
    r, v, t = r0, v0, t0
    kount = 0
    while t < t_ign
        dtp = min(_cis_dt(r, t, eph; eta = eta, dt_max = 30.0), t_ign - t)
        (kount % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, 0)
        r, v = _cis_step(r, v, t, dtp, eph)
        t += dtp
        kount += 1
    end

    r, v, t, m, dv_del, tburn, bts, brs, bvs =
        tli_burn(r, v, t, m_stack, stage, dv, eph, prop_avail)
    for (tb, rb, vb) in zip(bts, brs, bvs)
        _cis_push!(L, tb, rb, vb, eph, theta_g0, 1)
    end

    # range rate to the Moon; perilune is where it changes sign
    rate(rr, vv, tt) = vdot(vsub(rr, moon_position(eph, tt)),
                            vsub(vv, moon_velocity(eph, tt)))
    outcome = :timeout
    peri_alt = Inf; t_peri = NaN
    t_end = t0 + t_max
    kount = 0
    while t < t_end
        dtc = _cis_dt(r, t, eph; eta = eta)
        (kount % log_every == 0) && _cis_push!(L, t, r, v, eph, theta_g0, 2)
        kount += 1
        rn_, vn_ = _cis_step(r, v, t, dtc, eph)
        tn = t + dtc
        if moon_distance(eph, rn_, tn) - R_MOON <= 0.0
            r, v, t = rn_, vn_, tn
            outcome = :lunar_impact
            break
        end
        if rate(r, v, t) < 0.0 && rate(rn_, vn_, tn) >= 0.0
            # bisect the step onto closest approach
            lo, hi = 0.0, dtc
            for _ in 1:60
                mid = 0.5 * (lo + hi)
                rm, vm = _cis_step(r, v, t, mid, eph)
                if rate(rm, vm, t + mid) < 0.0; lo = mid; else; hi = mid; end
                hi - lo < 1e-6 && break
            end
            r, v = _cis_step(r, v, t, 0.5 * (lo + hi), eph)
            t += 0.5 * (lo + hi)
            peri_alt = moon_distance(eph, r, t) - R_MOON
            t_peri = t
            outcome = :perilune
            _cis_push!(L, t, r, v, eph, theta_g0, 2)
            break
        end
        r, v, t = rn_, vn_, tn
        if vnorm(r) > 2.0 * A_MOON
            outcome = :escape
            break
        end
    end
    CislunarResult(L, outcome, r, v, t, m, dv_del, t_ign, tburn,
                   peri_alt, t_peri, NaN, NaN, 0, NaN)
end

# ------------------------------------------------------- impulsive burns ---

"Mass after an impulsive burn of `dv` [m/s] on a lander."
_burn_mass(l::Lander, m::Float64, dv::Float64) = m * exp(-dv / (G0 * l.isp))

"""
    loi_burn(r_m, v_m) -> (dv, v_after)

Lunar-orbit insertion at perilune. At closest approach the relative velocity
is exactly perpendicular to the relative position (that is what closest
approach means), so the cheapest circularisation is purely retrograde and
its magnitude is just the speed excess over circular. Nothing is targeted
here — the altitude of the resulting circular orbit is the perilune the
trans-lunar design already flew to.
"""
function loi_burn(r_m::V3, v_m::V3)
    rn = vnorm(r_m)
    v_circ = sqrt(MU_MOON / rn)
    vhat = vunit(v_m)
    dv = vnorm(v_m) - v_circ
    (dv, vscale(vhat, v_circ))
end

"""
    doi_burn(r_m, v_m, h_pdi) -> (dv, v_after)

Descent-orbit initiation: a retrograde burn from the circular parking orbit
onto an ellipse whose periapsis is `h_pdi` above the surface, half a
revolution downrange. Twenty-odd metres per second buys a 100 km → 15 km
descent for free — the whole point of doing it as a coast rather than
burning down under power.
"""
function doi_burn(r_m::V3, v_m::V3, h_pdi::Float64)
    ra = vnorm(r_m)
    rp = R_MOON + h_pdi
    rp < ra || throw(ArgumentError("descent periapsis must be below the parking orbit"))
    a = 0.5 * (ra + rp)
    v_apo = sqrt(MU_MOON * (2 / ra - 1 / a))
    dv = vnorm(v_m) - v_apo
    (dv, vscale(vunit(v_m), v_apo))
end

"""
    coast_moon!(L, r, v, t, dt_total; phase, dt, log_every) -> (r, v)

Ballistic Moon-centred coast of `dt_total` seconds, logging as it goes.
"""
function coast_moon!(L::LunarOrbitLog, r::V3, v::V3, t::Float64,
                     dt_total::Float64; phase::Int = 0, dt::Float64 = 5.0,
                     log_every::Int = 4)
    n = max(1, ceil(Int, dt_total / dt))
    step = dt_total / n
    for k in 1:n
        if (k - 1) % log_every == 0
            push!(L.t, t); push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
            push!(L.h, vnorm(r) - R_MOON); push!(L.phase, phase)
        end
        r, v = _moon_step(r, v, step)
        t += step
    end
    push!(L.t, t); push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
    push!(L.h, vnorm(r) - R_MOON); push!(L.phase, phase)
    (r, v)
end

# ------------------------------------------------------- powered descent ---

"""
In-plane frame at a Moon-centred position: radial-out and along-track, the
latter fixed by the orbit normal `hhat` captured at ignition rather than by
the instantaneous velocity. That distinction is the whole difference between
a descent and a divergence: a frame built on the velocity flips end-for-end
the moment the vehicle stops flying forward, so "retrograde" reverses under
the guidance in the last thirty seconds of braking and the thrust that was
slowing the vehicle starts accelerating it back up. The normal is constant —
nothing here thrusts out of plane — so the along-track direction stays the
direction the vehicle was originally going, all the way to the surface.
"""
@inline function _descent_frame(r::V3, hhat::V3)
    ur = vunit(r)
    (ur, vcross(hhat, ur))
end

"Orbit normal of a Moon-centred state — the descent plane, fixed at ignition."
@inline _descent_normal(r::V3, v::V3) = vunit(vcross(r, v))

"""
    _descent_leg(lander, r0, v0, m0, pitch0, pitch_rate; vh_gate, dt, t_max)

Braking phase: full thrust, thrust elevation above the local horizontal
following `theta(t) = pitch0 + pitch_rate * t`, integrated until the
along-track speed falls through `vh_gate` — high gate — which is bisected
onto exactly. It also stops early if the tank runs dry, the vehicle reaches
the surface, it climbs away, or the clock runs out; the shooter needs those
apart to tell a miss from a divergence.

High gate is a *velocity* condition, not an altitude one, and that is what
makes the phase shootable. Braking is nearly all horizontal, so where the
vehicle ends up in altitude is the free variable the pitch program controls;
picking the altitude instead would fix the one thing the guidance has
authority over and leave the speed — the thing that has to be gone — as
whatever fell out.

Full thrust is not an approximation for effect: a braking phase wants every
newton it has, and Apollo held maximum thrust for all but the first and last
minutes of its descent. Throttling is what the terminal phase is for.
"""
function _descent_leg(l::Lander, r0::V3, v0::V3, m0::Float64,
                      pitch0::Float64, pitch_rate::Float64;
                      vh_gate::Float64 = 150.0, dt::Float64 = 0.5,
                      t_max::Float64 = 1200.0, log::Union{Nothing,DescentLog} = nothing,
                      r_ref::V3 = r0, log_every::Int = 4)
    r, v, m, t = r0, v0, m0, 0.0
    mdot = lander_mdot(l)
    m_dry = m0 - l.mprop
    h0 = vnorm(r0) - R_MOON
    hhat = _descent_normal(r0, v0)
    outcome = :gate
    kount = 0

    thrust_dir(rr, vv, tt) = begin
        ur, ut = _descent_frame(rr, hhat)
        th = clamp(pitch0 + pitch_rate * tt, -deg2rad_(60.0), deg2rad_(89.0))
        (vadd(vscale(ur, sin(th)), vscale(ut, -cos(th))), th)
    end

    vhof(rr, vv) = (ur = vunit(rr); vdot(vv, vcross(hhat, ur)))

    while t < t_max
        h = vnorm(r) - R_MOON
        if log !== nothing && kount % log_every == 0
            _, th = thrust_dir(r, v, t)
            _log_descent!(log, t, r, v, m, 1.0, th, r_ref, hhat)
        end
        kount += 1
        if vhof(r, v) <= vh_gate
            outcome = :gate
            break
        end
        if h <= 0.0
            outcome = :surface
            break
        end
        if m <= m_dry + 1e-9
            outcome = :propellant
            break
        end
        if h > h0 + 20.0e3
            outcome = :climbing
            break
        end
        step = min(dt, (m - m_dry) / mdot)
        # RK4 on (r, v) with mass drawn linearly across the step
        acc(rr, vv, mm, tt) = begin
            d, _ = thrust_dir(rr, vv, tt)
            vadd(_moon_accel(rr), vscale(d, l.thrust / mm))
        end
        k1r = v;                            k1v = acc(r, v, m, t)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2)); m2 = m - mdot*step/2
        k2r = v2;                           k2v = acc(r2, v2, m2, t + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                           k3v = acc(r3, v3, m2, t + step/2)
        r4 = vadd(r, vscale(k3r, step));   v4 = vadd(v, vscale(k3v, step));   m4 = m - mdot*step
        k4r = v4;                           k4v = acc(r4, v4, m4, t + step)
        rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        # land exactly on the gate rather than stepping past it: over half a
        # second the state is linear to well under a metre
        if vhof(rn, vn) < vh_gate
            lo, hi = 0.0, 1.0
            for _ in 1:40
                f = 0.5 * (lo + hi)
                rm = vadd(r, vscale(vsub(rn, r), f))
                vm = vadd(v, vscale(vsub(vn, v), f))
                if vhof(rm, vm) > vh_gate; lo = f; else; hi = f; end
            end
            f = 0.5 * (lo + hi)
            r = vadd(r, vscale(vsub(rn, r), f))
            v = vadd(v, vscale(vsub(vn, v), f))
            m -= mdot * step * f
            t += step * f
            outcome = :gate
            break
        end
        r, v, m, t = rn, vn, m - mdot*step, t + step
    end
    ur, ut = _descent_frame(r, hhat)
    (r = r, v = v, m = m, t = t, outcome = outcome,
     h = vnorm(r) - R_MOON, vv = vdot(v, ur), vh = vdot(v, ut))
end

"Push one sample onto a descent log."
function _log_descent!(L::DescentLog, t, r::V3, v::V3, m, throttle, pitch,
                       r_ref::V3, hhat::V3)
    ur, ut = _descent_frame(r, hhat)
    push!(L.t, t); push!(L.h, vnorm(r) - R_MOON)
    push!(L.downrange, R_MOON * acos(clamp(vdot(vunit(r), vunit(r_ref)), -1.0, 1.0)))
    push!(L.v, vnorm(v)); push!(L.vh, vdot(v, ut)); push!(L.vv, vdot(v, ur))
    push!(L.m, m); push!(L.throttle, throttle); push!(L.pitch, pitch)
    push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
    nothing
end

"""
    tune_braking(lander, r0, v0, m0; h_gate, vv_gate, vh_gate) -> (pitch0, rate, ok)

Shoot the braking phase. Two parameters (initial pitch, pitch rate) against
two targets — the altitude and sink rate at high gate — by damped Newton
with finite differences, exactly the structure `tune_ascent` uses for the
climb out of the atmosphere, because it is the same problem upside down.

The defaults are Apollo's high gate: 2.3 km up, 45 m/s down, with 150 m/s of
forward speed left for the approach phase to fly out. That last number is
not a detail — a braking phase taken all the way to zero horizontal velocity
has to end pointing straight up, which costs propellant and arrives with no
forward view of the site at all.

A coarse grid seeds the Newton: the residual surface has a cliff (programs
that point too high never come down, programs that point too low reach the
surface still moving at hundreds of metres per second), and a Newton started
on the wrong side of it walks off rather than converging. Failed legs return
a signed penalty that pushes the search back toward the feasible region
instead of stalling on a flat NaN.
"""
function tune_braking(l::Lander, r0::V3, v0::V3, m0::Float64;
                      h_gate::Float64 = 2300.0, vv_gate::Float64 = -45.0,
                      vh_gate::Float64 = 150.0, max_iter::Int = 25,
                      verbose::Bool = false)
    function resid(p0, pr)
        leg = _descent_leg(l, r0, v0, m0, p0, pr; vh_gate = vh_gate)
        if leg.outcome === :gate
            return ((leg.h - h_gate) / 1000.0, (leg.vv - vv_gate) / 100.0, leg)
        elseif leg.outcome === :climbing
            # too much lift: the whole program has to come down. The residual
            # keeps the altitude *ordering* so the search knows which way.
            return (leg.h / 1000.0, 10.0, leg)
        elseif leg.outcome === :surface
            # arrived at the ground still flying. "Altitude at high gate" is
            # below zero by however much speed was left over — a continuous
            # extension of the residual rather than a cliff, so the Jacobian
            # through this region still points somewhere useful.
            return (-h_gate / 1000.0 - leg.vh / 500.0,
                    (leg.vv - vv_gate) / 100.0, leg)
        else
            # dry, or out of clock, still above the gate and still fast
            return ((leg.h - h_gate) / 1000.0,
                    (leg.vv - vv_gate) / 100.0 - leg.vh / 500.0, leg)
        end
    end
    score(p0, pr) = (f = resid(p0, pr); hypot(f[1], f[2]))

    # coarse grid, then Newton, then — if it did not converge — a local grid
    # around the best point seen and another Newton. The residual surface has
    # a fold in it (programs that reach the surface early and programs that
    # never come down are on opposite sides), and a single Newton started on
    # the wrong side of the fold walks away from the solution rather than
    # into it.
    best = (Inf, 0.0, 0.0)
    for p0 in deg2rad_.(-6.0:3.0:24.0), pr in (0.0:1.5e-4:1.5e-3)
        sc = score(p0, pr)
        sc < best[1] && (best = (sc, p0, pr))
    end
    verbose && @info "braking seed" pitch0_deg = rad2deg_(best[2]) rate = best[3] score = best[1]

    function newton(p0, pr)
        local bp = (Inf, p0, pr)
        for it in 1:max_iter
            f1, f2, leg = resid(p0, pr)
            sc = hypot(f1, f2)
            sc < bp[1] && (bp = (sc, p0, pr))
            verbose && @info "braking newton" it f1 f2 pitch0_deg = rad2deg_(p0) rate = pr outcome = leg.outcome
            (abs(f1) < 0.1 && abs(f2) < 0.1 && leg.outcome === :gate) &&
                return (true, p0, pr)
            d1 = deg2rad_(0.4); d2 = 4.0e-5
            f1a, f2a, _ = resid(p0 + d1, pr)
            f1b, f2b, _ = resid(p0, pr + d2)
            j11 = (f1a - f1) / d1; j21 = (f2a - f2) / d1
            j12 = (f1b - f1) / d2; j22 = (f2b - f2) / d2
            det = j11 * j22 - j12 * j21
            abs(det) < 1e-14 && break
            dp0 = -( j22 * f1 - j12 * f2) / det
            dpr = -(-j21 * f1 + j11 * f2) / det
            p0 += clamp(0.7 * dp0, -deg2rad_(4.0), deg2rad_(4.0))
            pr += clamp(0.7 * dpr, -2.0e-4, 2.0e-4)
            p0 = clamp(p0, -deg2rad_(30.0), deg2rad_(60.0))
            pr = clamp(pr, -1.0e-3, 4.0e-3)
        end
        (false, bp[2], bp[3])
    end

    ok, p0, pr = newton(best[2], best[3])
    if !ok
        # refine locally around the best point and try once more
        bl = (Inf, p0, pr)
        for dp in deg2rad_.(-3.0:0.75:3.0), dr in (-3.0e-4:7.5e-5:3.0e-4)
            sc = score(p0 + dp, pr + dr)
            sc < bl[1] && (bl = (sc, p0 + dp, pr + dr))
        end
        verbose && @info "braking restart" pitch0_deg = rad2deg_(bl[2]) rate = bl[3] score = bl[1]
        ok, p0, pr = newton(bl[2], bl[3])
    end
    ok && return (p0, pr, true)

    f1, f2, leg = resid(p0, pr)
    # a loose finish is still a flyable descent — the terminal phase closes
    # on velocity, not on where exactly the braking phase handed over
    (p0, pr, abs(f1) < 1.0 && abs(f2) < 0.5 && leg.outcome === :gate)
end

"""
    terminal_descent(lander, r0, v0, m0; ...) -> NamedTuple

Closed-loop descent from high gate to the surface. The guidance holds a
commanded sink rate that tapers with altitude —
`v_cmd = -(v_touch + k*sqrt(h))`, the standard square-root profile, capped so
the vehicle does not dive — and nulls the horizontal component on a fixed
time constant. The commanded acceleration becomes a thrust vector, the
magnitude is clamped to the engine's throttle band, and what the engine
cannot deliver simply is not delivered: if the deepest throttle still
exceeds lunar gravity at the current mass, the vehicle climbs, and the log
shows it.
"""
function terminal_descent(l::Lander, r0::V3, v0::V3, m0::Float64;
                          m_dry::Float64, v_touch::Float64 = 0.8,
                          k_profile::Float64 = 0.85, v_cap::Float64 = 25.0,
                          tau_h::Float64 = 18.0, tau_v::Float64 = 5.0,
                          dt::Float64 = 0.1, t_max::Float64 = 900.0,
                          log::Union{Nothing,DescentLog} = nothing,
                          t0::Float64 = 0.0, r_ref::V3 = r0, log_every::Int = 10)
    r, v, m, t = r0, v0, m0, 0.0
    mdot_full = lander_mdot(l)
    hhat = _descent_normal(r0, v0)
    min_thr = 1.0
    outcome = :touchdown
    kount = 0

    command(rr, vv, mm) = begin
        ur, ut = _descent_frame(rr, hhat)
        h = vnorm(rr) - R_MOON
        vv_now = vdot(vv, ur); vh_now = vdot(vv, ut)
        v_cmd = -min(v_cap, v_touch + k_profile * sqrt(max(h, 0.0)))
        # Feed-forward on the profile itself. The commanded sink rate is a
        # function of altitude, so it moves as the vehicle descends, and a
        # pure proportional law lags it by tau_v * dv_cmd/dt — metres per
        # second of extra sink exactly where it hurts, because the profile
        # steepens as 1/sqrt(h) near the ground. Differentiating the profile
        # along the trajectory and commanding that outright leaves the
        # proportional term with nothing but the error to correct.
        dv_dh = v_cmd <= -v_cap ? 0.0 : -0.5 * k_profile / sqrt(max(h, 1.0))
        a_r = (v_cmd - vv_now) / tau_v + dv_dh * vv_now
        a_t = (0.0 - vh_now) / tau_h            # null the along-track drift
        # cancel gravity and the centrifugal relief of whatever speed remains
        g_eff = MU_MOON / vnorm(rr)^2 - vh_now^2 / vnorm(rr)
        ar_tot = a_r + g_eff
        # Thrust is finite, and the two channels are not equally important:
        # arriving with a few m/s of drift is a bad landing, arriving with an
        # unchecked sink rate is a crater. So the vertical demand is served
        # first and the along-track command gets whatever is left over.
        a_max = l.thrust / mm
        if abs(ar_tot) > a_max
            a_t = 0.0
        else
            lim = sqrt(max(a_max^2 - ar_tot^2, 0.0))
            a_t = clamp(a_t, -lim, lim)
        end
        a_des = vadd(vscale(ur, ar_tot), vscale(ut, a_t))
        an = vnorm(a_des)
        thr = clamp(mm * an / l.thrust, l.throttle_min, 1.0)
        dir = an > 1e-9 ? vscale(a_des, 1 / an) : ur
        (dir, thr)
    end

    while t < t_max
        h = vnorm(r) - R_MOON
        dir, thr = command(r, v, m)
        min_thr = min(min_thr, thr)
        if log !== nothing && kount % log_every == 0
            ur, _ = _descent_frame(r, hhat)
            _log_descent!(log, t0 + t, r, v, m, thr,
                          asin(clamp(vdot(dir, ur), -1.0, 1.0)), r_ref, hhat)
        end
        kount += 1
        if h <= 0.0
            outcome = :touchdown
            break
        end
        if m <= m_dry + 1e-9
            outcome = :propellant
            break
        end
        step = min(dt, (m - m_dry) / (thr * mdot_full))
        acc(rr, vv, mm) = begin
            d, th = command(rr, vv, mm)
            vadd(_moon_accel(rr), vscale(d, th * l.thrust / mm))
        end
        k1r = v;                         k1v = acc(r, v, m)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2))
        k2r = v2;                        k2v = acc(r2, v2, m - thr*mdot_full*step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                        k3v = acc(r3, v3, m - thr*mdot_full*step/2)
        r4 = vadd(r, vscale(k3r, step)); v4 = vadd(v, vscale(k3v, step))
        k4r = v4;                        k4v = acc(r4, v4, m - thr*mdot_full*step)
        rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        hn = vnorm(rn) - R_MOON
        if hn <= 0.0 && h > 0.0
            f = h / max(h - hn, 1e-9)           # linear touchdown interpolation
            rn = vadd(r, vscale(vsub(rn, r), f))
            vn = vadd(v, vscale(vsub(vn, v), f))
            m -= thr * mdot_full * step * f
            t += step * f
            r, v = rn, vn
            outcome = :touchdown
            break
        end
        r, v, m, t = rn, vn, m - thr*mdot_full*step, t + step
    end
    ur, ut = _descent_frame(r, hhat)
    (r = r, v = v, m = m, t = t, outcome = outcome, min_throttle = min_thr,
     v_vertical = -vdot(v, ur), v_horizontal = vdot(v, ut))
end

"""
    powered_descent(lander, r0, v0, m0; h_gate, vh_gate, vv_gate, verbose)
        -> DescentResult

Braking phase (shot open-loop) followed by the closed-loop terminal phase,
logged as one continuous descent.
"""
function powered_descent(l::Lander, r0::V3, v0::V3, m0::Float64;
                         h_gate::Float64 = 2300.0, vh_gate::Float64 = 150.0,
                         vv_gate::Float64 = -45.0, verbose::Bool = false)
    m_dry = m0 - l.mprop
    p0, pr, _ = tune_braking(l, r0, v0, m0; h_gate = h_gate, vh_gate = vh_gate,
                             vv_gate = vv_gate, verbose = verbose)
    L = DescentLog()
    leg = _descent_leg(l, r0, v0, m0, p0, pr; vh_gate = vh_gate, log = L, r_ref = r0)
    dv_brake = G0 * l.isp * log(m0 / leg.m)
    # A braking phase that reached high gate is worth flying out even if the
    # shooter finished loose: the terminal phase is closed-loop on velocity,
    # so it either saves the landing or it does not, and the touchdown state
    # says which. Only a leg that never reached the gate is unflyable.
    if leg.outcome !== :gate
        return DescentResult(L, leg.outcome === :gate ? :diverged : leg.outcome,
                             leg.t, leg.t, -leg.vv, leg.vh,
                             isempty(L.downrange) ? 0.0 : L.downrange[end],
                             dv_brake, 0.0, m0 - leg.m, leg.m - m_dry, 0.0, 1.0,
                             p0, pr, leg.r, leg.v, leg.m)
    end

    term = terminal_descent(l, leg.r, leg.v, leg.m; m_dry = m_dry, log = L,
                            t0 = leg.t, r_ref = r0)
    dv_term = G0 * l.isp * log(leg.m / term.m)
    _log_descent!(L, leg.t + term.t, term.r, term.v, term.m, term.min_throttle,
                  0.0, r0, _descent_normal(r0, v0))
    prop_left = term.m - m_dry
    # what the residual is actually worth: seconds of hover at touchdown mass
    hover = prop_left / (term.m * MU_MOON / vnorm(term.r)^2 / (G0 * l.isp))
    # Touchdown limits are the lander's, not the trajectory's: Apollo's LM was
    # designed for 3 m/s of sink and about 1.2 m/s of lateral drift before a
    # leg digs in and the vehicle tips. Anything outside that is a crash, and
    # calling it one is the whole point of flying the last kilometre.
    outcome = term.outcome === :touchdown ?
              (term.v_vertical > 3.0 || abs(term.v_horizontal) > 1.5 ?
               :crash : :touchdown) : term.outcome
    DescentResult(L, outcome, leg.t + term.t, leg.t, term.v_vertical, term.v_horizontal,
                  L.downrange[end], dv_brake, dv_term, m0 - term.m, prop_left,
                  hover, term.min_throttle, p0, pr, term.r, term.v, term.m)
end

# ------------------------------------------------------- mission assembly --

"""
    moonlanding(; lander, h_park, h_moon_park, h_pdi, n_rev, inclination,
                lv, hp_return, optimize_kick, verbose) -> LandingResult

Design and fly the whole landing mission: pad to lunar surface.

The launch and trans-lunar legs are the flyby mission's, unchanged — the
free return is designed to a perilune equal to `h_moon_park`, so the
insertion burn happens exactly where the coast already goes. From there the
lander flies itself: insertion, `n_rev` revolutions of parking orbit,
descent-orbit initiation, half a revolution of coast, and a powered descent
from `h_pdi`.

The payload of `lv` must be the lander's wet mass; `moonlanding` builds a
matching vehicle if none is given, and complains if the two disagree, since
a launcher that is not carrying this lander is not flying this mission.
"""
function moonlanding(; lander::Lander = default_lander(),
                     h_park::Float64 = 200.0e3,
                     h_moon_park::Float64 = 100.0e3,
                     h_pdi::Float64 = 15.0e3,
                     n_rev::Int = 1,
                     inclination::Float64 = deg2rad_(28.5),
                     lv::Union{Nothing,LaunchVehicle} = nothing,
                     hp_return::Float64 = 50.0e3,
                     h_gate::Float64 = 2000.0,
                     kick_angle::Float64 = deg2rad_(8.0),
                     optimize_kick::Bool = false,
                     cis_eta::Float64 = SatelliteSim.CIS_ETA,
                     # The free return is an *abort* path here, not an entry
                     # corridor: nobody flies it unless the insertion burn
                     # fails, so it is designed to kilometres rather than to
                     # the 250 m the flyby mission needs.
                     perigee_tol::Float64 = 5.0e3,
                     verbose::Bool = false)
    lv === nothing && (lv = default_moon_rocket(payload = lander_mass(lander)))
    abs(lv.payload_mass - lander_mass(lander)) < 1.0 ||
        error("launch vehicle payload ($(round(lv.payload_mass)) kg) is not the " *
              "lander's wet mass ($(round(lander_mass(lander))) kg)")

    des = translunar_design(lv; h_park = h_park, hp_moon = h_moon_park,
                            hp_return = hp_return, inclination = inclination,
                            kick_angle = kick_angle, optimize_kick = optimize_kick,
                            cis_eta = cis_eta, perigee_tol = perigee_tol,
                            tol_perigee_km = 25.0, verbose = verbose)
    asc, eph = des.ascent, des.eph

    cis = fly_to_perilune(asc.r, asc.v, asc.t, eph; t_ign = des.t_ign,
                          dv = des.dv, stage = des.kick, m_stack = des.m_stack,
                          prop_avail = asc.prop_left[end], eta = cis_eta)
    cis.outcome == :perilune ||
        error("trans-lunar leg did not reach perilune (outcome: $(cis.outcome))")

    # --- insertion ---------------------------------------------------------
    t_loi = cis.t
    r_m, v_m = mci_state(cis.r, cis.v, t_loi, eph)
    dv_loi, v_after = loi_burn(r_m, v_m)
    m = _burn_mass(lander, lander_mass(lander), dv_loi)
    m <= lander.mdry &&
        error("lunar-orbit insertion alone empties the lander " *
              "($(round(dv_loi)) m/s needed, $(round(lander_dv(lander))) m/s carried)")

    # --- parking orbit, DOI, coast to the descent periapsis ----------------
    OL = LunarOrbitLog()
    r_park, v_park = r_m, v_after
    T_park = 2pi * sqrt(vnorm(r_park)^3 / MU_MOON)
    r_park, v_park = coast_moon!(OL, r_park, v_park, t_loi, n_rev * T_park;
                                 phase = 0, dt = 5.0, log_every = 8)
    t_doi = t_loi + n_rev * T_park
    dv_doi, v_doi = doi_burn(r_park, v_park, h_pdi)
    m = _burn_mass(lander, m, dv_doi)
    a_desc = 0.5 * (vnorm(r_park) + R_MOON + h_pdi)
    t_transfer = pi * sqrt(a_desc^3 / MU_MOON)
    r_pdi, v_pdi = coast_moon!(OL, r_park, v_doi, t_doi, t_transfer;
                               phase = 1, dt = 2.0, log_every = 8)
    t_pdi = t_doi + t_transfer

    # --- powered descent ---------------------------------------------------
    # the lander's remaining propellant is what it flies the descent on
    flying = Lander(lander.name, lander.mdry, m - lander.mdry, lander.thrust,
                    lander.isp, lander.throttle_min, lander.diameter)
    desc = powered_descent(flying, r_pdi, v_pdi, m; h_gate = h_gate,
                           verbose = verbose)
    t_td = t_pdi + desc.t_touchdown
    lat, lon = selenographic(desc.r, t_td, eph)

    LandingResult(lv, lander, des.guid, asc, eph, cis, OL, desc,
                  dv_loi, dv_doi, t_loi, t_doi, t_pdi, t_td, h_moon_park, h_pdi,
                  n_rev, lat, lon, desc.prop_left)
end

function print_landing_summary(io::IO, ls::LandingResult)
    asc, cis, d = ls.ascent, ls.cislunar, ls.descent
    el = asc.elements
    println(io, "== Lunar landing summary ==")
    @printf(io, "  Liftoff mass    : %.1f t   (%s, %d stages)\n",
            liftoff_mass(ls.lv) / 1e3, ls.lv.name, length(ls.lv.stages))
    @printf(io, "  Lander          : %.1f t wet / %.1f t dry, %.0f kN, Isp %.0f s, %.0f m/s ideal\n",
            lander_mass(ls.lander) / 1e3, ls.lander.mdry / 1e3,
            ls.lander.thrust / 1e3, ls.lander.isp, lander_dv(ls.lander))
    @printf(io, "  Parking orbit   : %.1f x %.1f km  i=%.2f°\n",
            (el.rp - RE_MEAN) / 1e3, (el.ra - RE_MEAN) / 1e3, rad2deg_(el.i))
    @printf(io, "  TLI             : dv=%.1f m/s at t=%.2f h\n", cis.dv_tli, cis.t_tli / 3600)
    @printf(io, "  Perilune / LOI  : %.1f km at t=%.2f d, dv=%.1f m/s\n",
            cis.perilune_alt / 1e3, ls.t_loi / 86400, ls.dv_loi)
    @printf(io, "  Lunar orbit     : %.0f km circular, %d rev, DOI dv=%.1f m/s to %.1f km\n",
            ls.h_park_moon / 1e3, ls.n_rev, ls.dv_doi, ls.h_pdi / 1e3)
    @printf(io, "  Powered descent : %.1f s, dv %.0f m/s braking + %.0f m/s terminal, %.0f km downrange\n",
            d.t_touchdown, d.dv_braking, d.dv_terminal, d.downrange / 1e3)
    @printf(io, "  Braking program : pitch %.1f° + %.4f °/s, min throttle %.0f%%\n",
            rad2deg_(d.pitch0), rad2deg_(d.pitch_rate), 100 * d.min_throttle)
    if d.outcome === :touchdown
        @printf(io, "  TOUCHDOWN       : %.2f m/s down, %.2f m/s lateral at %.2f°%s, %.2f°%s (%s side)\n",
                d.v_vertical, d.v_horizontal, abs(rad2deg_(ls.lat_land)),
                ls.lat_land >= 0 ? "N" : "S", abs(rad2deg_(ls.lon_land)),
                ls.lon_land >= 0 ? "E" : "W",
                abs(ls.lon_land) < pi/2 ? "near" : "far")
    else
        println(io, "  DID NOT LAND SAFELY (", d.outcome, ")")
        @printf(io, "    arrived %.2f m/s down, %.2f m/s lateral\n",
                d.v_vertical, d.v_horizontal)
    end
    @printf(io, "  Propellant      : %.0f kg used, %.0f kg left (%.0f s of hover)\n",
            d.prop_used, d.prop_left, d.hover_s)
    @printf(io, "  Mission elapsed : %.2f days pad to surface\n", ls.t_touchdown / 86400)
end
print_landing_summary(ls::LandingResult) = print_landing_summary(stdout, ls)
