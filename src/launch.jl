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
    # --- suborbital ------------------------------------------------------
    # An orbit is cut off on ENERGY, because energy is what an orbit is. A
    # suborbital flight is not going to orbit, so neither of those is the
    # quantity to close on: a hop closes on the apogee it wants and a ballistic
    # shot closes on where it wants to come down. `cutoff` picks which, and the
    # two targets below feed it. `pitch_hold`, when set, replaces the
    # linear-tangent law with a constant commanded pitch — a sounding rocket
    # holds an attitude, it does not fly a law tuned to arrive horizontal.
    cutoff::Symbol = :energy               # :energy | :apogee | :range
    apogee_target::Float64 = NaN           # [m] above RE_MEAN, for :apogee
    range_target::Float64 = NaN            # [m] great-circle ground range, for :range
    pitch_hold::Float64 = NaN              # [rad] above the local horizon
end

"""
A copy of `g` with named fields replaced. Guidance is a plain positional struct
and it now carries fifteen fields; spelling them all out at each call site is
exactly how one of them ends up silently dropped, which is what the two
rebuilders in the tuner used to do.
"""
_reguid(g::AscentGuidance; kw...) =
    AscentGuidance(; (f => get(kw, f, getfield(g, f))
                      for f in fieldnames(AscentGuidance))...)

"Launch azimuth [rad] that yields inclination `inc` from latitude `lat` (prograde)."
launch_azimuth(inc::Float64, lat::Float64) =
    asin(clamp(cos(inc) / cos(lat), -1.0, 1.0))

# ------------------------------------------------------------ launch window --
# `tune_ascent` closes the pitch program on (insertion altitude, gamma = 0),
# which pins the orbit's SIZE and SHAPE but says nothing about where its plane
# sits in inertial space. The plane is set by when you launch: the site is
# carried around by the Earth, and the vehicle inherits wherever it happens to
# be. That is fine for a single flight designed in isolation — every mission in
# this repo so far has simply accepted whatever RAAN it got — but the moment a
# second vehicle has to reach the first one, the launch epoch stops being free.
#
# So the free variable for plane targeting is TIME, not pitch, and it is
# closed-form rather than another shooting problem.

"Geocentric latitude [rad] of a site, via the same WGS-84 point the ascent starts from."
function site_geocentric_lat(lat::Float64, lon::Float64)
    r = ecef_from_geodetic(lat, lon, 0.0)
    atan(r[3], hypot(r[1], r[2]))
end

"""
    launch_window(guid, inc, raan; theta_g0=0.0, after=0.0) -> Vector{NamedTuple}

Epochs at or after `after` [s] when the site of `guid` rotates into the orbit
plane `(inc, raan)`, so a direct ascent inserts into that plane.

A direct launch has the vehicle in the target plane from liftoff, so the site's
inertial position must lie in it — `u_site · ĥ = 0`. With this codebase's
element convention that reduces to

    sin(raan − α) = −tan(φ) · cot(inc),   α = lon + θ_g0 + ω⊕·t

for the site's geocentric latitude φ. Two roots per sidereal day, returned in
time order as `(t, node, azimuth)` with `node ∈ (:ascending, :descending)`;
the descending pass enters the same plane heading south, so it flies the
supplementary azimuth. **No** roots when `|tan φ · cot inc| > 1`, which is the
familiar "a site cannot reach an inclination below its own latitude" — here it
is the arcsine running out of domain rather than a rule bolted on.

The two latitudes in play are deliberately different. The epoch uses the
geocentric latitude, because the condition is about where the site's position
vector actually points and geodetic would misplace it by up to 0.19° (~21 km).
The azimuth uses the geodetic latitude, because that is what `launch_azimuth`
and the guidance already fly — one imperfect convention beats two disagreeing
ones, and neither corrects for the rotating launch site, which is the larger
error and shows up as the achieved-vs-target plane residual.
"""
function launch_window(guid::AscentGuidance, inc::Float64, raan::Float64;
                       theta_g0::Float64 = 0.0, after::Float64 = 0.0)
    out = NamedTuple[]
    abs(sin(inc)) < 1e-12 && return out          # equatorial: no node defined
    phi = site_geocentric_lat(guid.site_lat, guid.site_lon)
    k = -tan(phi) / tan(inc)
    abs(k) > 1.0 && return out                   # plane never passes overhead
    base = asin(k)
    sidereal = 2pi / OMEGA_EARTH
    az0 = launch_azimuth(inc, guid.site_lat)
    for (node, dO) in ((:ascending, base), (:descending, pi - base))
        alpha = raan - dO
        t = (alpha - guid.site_lon - theta_g0) / OMEGA_EARTH
        t = after + mod(t - after, sidereal)
        push!(out, (t = t, node = node,
                    azimuth = node === :ascending ? az0 : pi - az0))
    end
    sort!(out, by = w -> w.t)
    out
end

"""
    next_launch_window(guid, inc, raan; node=:ascending, theta_g0=0.0, after=0.0)

The next `launch_window` opportunity of the requested `node`, or `nothing` if
the plane is unreachable from the site.
"""
function next_launch_window(guid::AscentGuidance, inc::Float64, raan::Float64;
                            node::Symbol = :ascending, theta_g0::Float64 = 0.0,
                            after::Float64 = 0.0)
    ws = launch_window(guid, inc, raan; theta_g0 = theta_g0, after = after)
    i = findfirst(w -> w.node === node, ws)
    i === nothing ? nothing : ws[i]
end

mutable struct AscentCtx
    stage::Int                # index of the currently-burning stage (0 = none)
    phase::Symbol             # :prelaunch | :vertical | :kick | :gravity_turn | :closed_loop | :coast
    t_stage_ign::Float64      # ignition time of the current stage
    t_kick0::Float64
    t_loop0::Float64          # closed-loop steering start time
    fairing_on::Bool
    burning::Bool
    burned::Float64           # propellant drawn from the current stage [kg]
    bstate::Vector{Symbol}    # per booster set: :waiting | :burning | :spent | :gone
    b_tign::Vector{Float64}   # ignition time of each set [s]
    b_tspent::Vector{Float64} # burnout time of each set [s]
end

AscentCtx(stage, phase, t_ign, t_kick0, t_loop0, fairing_on, burning, nb::Int = 0) =
    AscentCtx(stage, phase, t_ign, t_kick0, t_loop0, fairing_on, burning, 0.0,
              fill(:waiting, nb), fill(NaN, nb), fill(NaN, nb))

"Booster sets still physically attached to the stack."
booster_attached(ctx::AscentCtx) = [s !== :gone for s in ctx.bstate]

# Automatic axial-acceleration limiting keeps a high-thrust configuration from
# turning the nominal pitch program into a short, violent hop. Two g is above
# the reference launcher's natural acceleration and below typical crew limits.
const MAX_ASCENT_ACCEL = 2.0G0
const MIN_CORE_THROTTLE = 0.20

"""
    core_throttle(lv, ctx, mass, pamb) -> Float64

Core thrust fraction after applying both a booster-set throttle command and an
automatic axial-acceleration limit. Excess thrust therefore shortens neither
the guidance window nor the useful burn: an overpowered vehicle throttles and
keeps the surplus propellant for later manoeuvres.
"""
function core_throttle(lv::LaunchVehicle, ctx::AscentCtx,
                       mass::Float64 = Inf, pamb::Float64 = 0.0)
    thr = 1.0
    if ctx.stage == 1
        for (i, b) in enumerate(lv.boosters)
            ctx.bstate[i] === :burning && (thr = min(thr, b.core_throttle))
        end
    end
    if isfinite(mass) && ctx.stage >= 1 && isempty(lv.boosters) &&
       pad_thrust(lv) / liftoff_mass(lv) > MAX_ASCENT_ACCEL
        fixed = sum(booster_thrust(b, pamb) for (i, b) in enumerate(lv.boosters)
                    if ctx.bstate[i] === :burning; init = 0.0)
        core = stage_thrust(lv.stages[ctx.stage], pamb)
        limit = (MAX_ASCENT_ACCEL * mass - fixed) / max(core, 1.0)
        thr = min(thr, clamp(limit, MIN_CORE_THROTTLE, 1.0))
    end
    thr
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
# how far below and above the local horizon the closed-loop law may point
const PITCH_CMD_MIN = -25.0 * pi / 180
const PITCH_CMD_MAX = 85.0 * pi / 180

"""
Perigee radius of the osculating orbit through this inertial state [m], or
`Inf` on an escape trajectory. Cutting an ascent off on energy alone says
nothing about where that energy points: the same specific energy describes a
circular orbit and a dive, and only the perigee tells them apart.
"""
function _perigee_radius(r::V3, v::V3)
    rr = vnorm(r)
    eps = 0.5 * vdot(v, v) - MU_EARTH / rr
    eps >= 0.0 && return Inf
    hv = vcross(r, v)
    a = -MU_EARTH / (2 * eps)
    e = sqrt(max(0.0, 1 + 2 * eps * vdot(hv, hv) / MU_EARTH^2))
    a * (1 - e)
end

# an insertion whose perigee is below this is a suborbital arc, not an orbit
const SECO_HP_MIN = 100.0e3

"""
Apogee radius of the osculating trajectory through this state [m], or `Inf` if
it is not coming back. This is what a hop closes on, and it is right for a
straight-up flight too: as the angular momentum goes to zero the eccentricity
goes to one and `a(1+e)` goes to `2a`, which is exactly the radius a purely
radial climb of that energy reaches.
"""
function _apogee_radius(r::V3, v::V3)
    rr = vnorm(r)
    eps = 0.5 * vdot(v, v) - MU_EARTH / rr
    eps >= 0.0 && return Inf
    hv = vcross(r, v)
    a = -MU_EARTH / (2 * eps)
    e = sqrt(max(0.0, 1 + 2 * eps * vdot(hv, hv) / MU_EARTH^2))
    a * (1 + e)
end

"""
Great-circle ground range [m] the free-flight arc through this state will cover
before its radius comes back to `RE_MEAN`, or `Inf` if it never does (an orbit,
an escape, or a trajectory whose perigee is above the surface).

This is the exact Keplerian answer rather than the textbook free-flight range
equation: take the true anomaly now, take the true anomaly where the radius is
back to the surface, and sweep forward through apogee between them. Written that
way there are no quadrant special cases and the lofted and depressed solutions
come out of the same expression.
"""
function _ballistic_range(r::V3, v::V3)
    el = elements_from_state(r, v)
    (el.e >= 1.0 || el.a <= 0.0) && return Inf
    el.rp >= RE_MEAN && return Inf                 # it stays up: not a shot
    p = el.a * (1 - el.e^2)
    cnu = clamp((p / RE_MEAN - 1) / el.e, -1.0, 1.0)
    nu_i = acos(cnu)                               # ascending-side solution
    # forward from where we are, out through apogee (pi), down to 2pi - nu_i
    psi = (2pi - nu_i) - el.nu
    psi <= 0.0 && return 0.0
    RE_MEAN * psi
end

"""
Has the suborbital target been met? `:apogee` closes on the apogee of the
current osculating arc, `:range` on where that arc comes down. Both are pure
functions of the state, so they hold under any steering and at any stage.
"""
function _suborbital_cut(guid::AscentGuidance, r::V3, v::V3, dr::Float64)
    if guid.cutoff === :apogee
        isnan(guid.apogee_target) && return false
        return _apogee_radius(r, v) >= RE_MEAN + guid.apogee_target
    elseif guid.cutoff === :range
        isnan(guid.range_target) && return false
        return dr + _ballistic_range(r, v) >= guid.range_target
    end
    false
end

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
        # A commanded attitude, when one is given: sounding rockets and
        # ballistic shots hold a pitch, they do not fly a law whose whole
        # purpose is to arrive horizontal at a target altitude. Held straight
        # up the horizontal component vanishes and `that` is whatever is left
        # of a numerically zero vector, so the pure-vertical case is taken
        # explicitly rather than left to a normalise of noise.
        if !isnan(guid.pitch_hold)
            th = clamp(guid.pitch_hold, -0.5pi, 0.5pi)
            sh, ch = sincos(th)
            ch < 1e-6 && return rhat
            vh = vsub(v, vscale(rhat, vdot(v, rhat)))
            vnorm(vh) < 1.0 && return rhat
            return vadd(vscale(vunit(vh), ch), vscale(rhat, sh))
        end
        tt = tan(guid.pitch0) - guid.pitch_rate * (t - ctx.t_loop0)
        # The linear-tangent law has no floor of its own: tan(theta) falls
        # without limit, so any burn that outlasts the window it was tuned for
        # walks the thrust vector round past the horizon and on into a dive.
        # A launcher above its target genuinely does push a little below
        # horizontal to arrest the climb — a few degrees of it — but it never
        # aims at the ground, and a vehicle that does is spending propellant to
        # make its own orbit worse.
        th = clamp(atan(tt), PITCH_CMD_MIN, PITCH_CMD_MAX)

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

    # Core stage, throttled while strap-ons carry the load, plus every
    # booster set that is currently lit. They all push along the same
    # commanded direction, so thrust simply sums.
    if ctx.burning && ctx.stage >= 1
        st = lv.stages[ctx.stage]
        thr = core_throttle(lv, ctx, m, pamb)
        thrust_mag = thr * stage_thrust(st, pamb)
        dm = -thr * stage_mdot(st)
    end
    for (i, b) in enumerate(lv.boosters)
        ctx.phase !== :coast && ctx.bstate[i] === :burning || continue
        thrust_mag += booster_thrust(b, pamb)
        dm -= booster_mdot(b)
    end
    if thrust_mag > 0.0
        dhat = _steer(guid, ctx, r, v, t, theta_g0)
        a = vadd(a, vscale(dhat, thrust_mag / m))
    end

    # aerodynamic drag on the stack (relative wind); attached boosters put
    # their own frontal area into the flow
    if rho > 0
        omega = (0.0, 0.0, OMEGA_EARTH)
        vrel = vsub(v, vcross(omega, r))
        Vr = vnorm(vrel)
        if Vr > 1.0
            M = Vr / asnd
            sref = isempty(lv.boosters) ? lv.sref :
                   frontal_area(lv, booster_attached(ctx))
            D = 0.5 * rho * Vr * Vr * sref * interp1(lv.cd, M)
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
    # A stack with no fairing has nothing to jettison, and arming the jettison
    # anyway meant a bare vehicle still announced a FAIRING JETTISON at 120 km
    # and dropped zero kilograms doing it.
    ctx = AscentCtx(1, :vertical, 0.0, NaN, NaN, lv.fairing_mass > 0, true,
                    length(lv.boosters))
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
        d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
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

    while t < t_max
        # --- phase transitions ------------------------------------------------
        d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
        if ctx.phase === :vertical && d.vrel >= guid.v_pitchover
            ctx.phase = :kick; ctx.t_kick0 = t
            ev!(:pitchover)
        elseif ctx.phase === :kick && t - ctx.t_kick0 >= guid.kick_duration
            ctx.phase = :gravity_turn
            ev!(:gravity_turn)
        end
        # A commanded attitude takes over from the gravity turn when the turn
        # has brought the vehicle down to it. That is the whole relationship
        # between the two: you fly the gravity turn because it costs no angle of
        # attack, and you stop flying it once it has delivered the attitude you
        # wanted. A hop commands the vertical, is already there, and so never
        # enters the turn at all — which matters, because a gravity turn is
        # unstable to lateral perturbation by construction (the thrust follows
        # the velocity, so any tip compounds), and left in one a vertical launch
        # walked 38 km downrange by 40 km of altitude on Coriolis alone.
        if ctx.phase === :gravity_turn && !isnan(guid.pitch_hold) &&
           d.gamma_rel <= guid.pitch_hold + 1e-9
            ctx.phase = :closed_loop
            isnan(ctx.t_loop0) && (ctx.t_loop0 = t)
            ev!(:pitch_hold)
        end
        if ctx.fairing_on && d.h >= guid.fairing_alt
            ctx.fairing_on = false
            x[7] -= lv.fairing_mass
            ev!(:fairing_jettison)
        end

        # A clearly failed orbital ascent must not keep firing while diving.
        # Ten degrees down is far outside any insertion corridor; shut the core
        # down and report the failed state instead of spending fuel on impact.
        if ctx.burning && isempty(lv.boosters) && guid.cutoff === :energy && t > 60.0 &&
           d.gamma < deg2rad_(-10.0)
            st = lv.stages[ctx.stage]
            prop_left[ctx.stage] = st.mprop - ctx.burned
            ctx.burning = false; ctx.phase = :coast
            reached = false; h_cut = d.h; gam_cut = d.gamma
            ev!(:powered_descent_abort)
            break
        end
        # The ground is the floor. Nothing here stopped a trajectory that came
        # back down from carrying on through the surface and out the far side,
        # so a flight that had already crashed went on being integrated as if
        # it were flying. It ends where it hits, and how hard it hit is the
        # event's own velocity — a few m/s is a landing, anything else is not.
        # Powered or not: the surface is an unconditional terminal boundary.
        # A bad trial must be allowed to fail, but never to tunnel through Earth.
        if t > 0.5 && d.h <= 0.0
            # Put the terminal state on the ellipsoid rather than leaving the
            # final fixed step a few metres underground.
            theta = earth_rotation_angle(theta_g0, t)
            rs = rot_z(ecef_from_geodetic(d.lat, d.lon, 0.0), -theta)
            x[1] = rs[1]; x[2] = rs[2]; x[3] = rs[3]
            # The inverse ellipsoid conversion is approximate; close the final
            # few metres explicitly so the logged state is on h = 0.
            for _ in 1:4
                d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
                abs(d.h) < 1e-6 && break
                rh = vunit((x[1], x[2], x[3]))
                x[1] -= d.h * rh[1]; x[2] -= d.h * rh[2]; x[3] -= d.h * rh[3]
            end
            d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
            h_cut = 0.0; gam_cut = d.gamma
            ctx.burning = false; ctx.phase = :coast
            reached = false
            ev!(:ground_impact)
            break
        end

        # --- strap-on boosters: light, burn out, then drop --------------------
        # Each transition is checked in turn rather than as a chain of
        # elseifs, so a set with no separation delay goes on the same step it
        # runs dry instead of hanging on for one more.
        for (i, b) in enumerate(lv.boosters)
            if ctx.bstate[i] === :waiting && t >= b.ignition_delay
                ctx.bstate[i] = :burning
                ctx.b_tign[i] = t
                ev!(Symbol(:ignition_, b.stage.name))
            end
            if ctx.bstate[i] === :burning &&
               t - ctx.b_tign[i] >= stage_burn_time(b.stage) - 1e-9
                ctx.bstate[i] = :spent
                ctx.b_tspent[i] = t
                ev!(Symbol(:burnout_, b.stage.name))
            end
            if ctx.bstate[i] === :spent && t - ctx.b_tspent[i] >= b.sep_delay
                ctx.bstate[i] = :gone
                x[7] -= b.count * b.stage.mdry     # the propellant is already gone
                ev!(Symbol(:sep_, b.stage.name))
            end
        end

        # --- burnout / staging / cutoff --------------------------------------
        if ctx.burning
            st = lv.stages[ctx.stage]
            # Depletion is tracked by propellant drawn, not elapsed time: a
            # throttled core burns for longer than its rated burn time.
            if ctx.burned >= st.mprop - 1e-9
                prop_left[ctx.stage] = 0.0
                x[7] -= st.mdry                       # drop the spent stage
                ev!(Symbol(:sep_, st.name))
                # All strap-ons belong to the first core. Drop their unused
                # fuel too; a waiting or still-burning set cannot propel an
                # upper stage after its attachment has separated.
                if ctx.stage == 1
                    for (i, b) in enumerate(lv.boosters)
                        ctx.bstate[i] === :gone && continue
                        prop = ctx.bstate[i] === :waiting ? b.stage.mprop :
                               ctx.bstate[i] === :burning ?
                               max(0.0, b.stage.mprop - stage_mdot(b.stage) * (t - ctx.b_tign[i])) : 0.0
                        ctx.bstate[i] = :gone
                        x[7] -= b.count * (b.stage.mdry + prop)
                        ev!(Symbol(:sep_, b.stage.name))
                    end
                end
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
            # suborbital cutoff: on the apogee wanted, or on where the arc comes
            # down. Checked at any phase and any stage, unlike the orbital
            # cutoff — a sounding rocket can meet its apogee inside the first
            # stage's burn and never reach a closed-loop phase at all.
            if ctx.burning && guid.cutoff !== :energy && d.h > 200.0 &&
               _suborbital_cut(guid, (x[1], x[2], x[3]), (x[4], x[5], x[6]),
                               d.downrange)
                st = lv.stages[ctx.stage]
                prop_left[ctx.stage] = st.mprop - ctx.burned
                ctx.burning = false; ctx.phase = :coast
                reached = false
                h_cut = d.h; gam_cut = d.gamma
                ev!(:seco)
                break
            end
            # exoatmospheric closed-loop cutoff at target energy
            if ctx.burning && ctx.phase === :closed_loop && guid.cutoff === :energy
                eps_now = 0.5 * d.vin^2 - MU_EARTH / vnorm((x[1], x[2], x[3]))
                if eps_now >= e_target
                    st = lv.stages[ctx.stage]
                    prop_left[ctx.stage] = st.mprop - ctx.burned
                    ctx.burning = false; ctx.phase = :coast
                    # Energy alone is not an orbit: the same specific energy
                    # describes a circular orbit and a dive, and only the
                    # perigee tells them apart. Reaching the target energy with
                    # the velocity pointed at the ground used to be reported as
                    # "in orbit", which is how a mission ended up flying a
                    # trajectory whose perigee was inside the planet. The
                    # cutoff INSTANT is deliberately unchanged — tune_ascent
                    # solves a 2x2 on (h_cut, gamma_cut) and a moving cutoff
                    # makes those discontinuous in its own parameters — so this
                    # decides what the result is called, not when it happens.
                    rp = _perigee_radius((x[1], x[2], x[3]), (x[4], x[5], x[6]))
                    reached = rp >= RE_MEAN + SECO_HP_MIN
                    h_cut = d.h; gam_cut = d.gamma
                    ev!(:seco)
                    !reached && ev!(:insertion_below_surface)
                    break
                end
            end
        elseif ctx.stage >= 1 && ctx.stage <= nst && t >= ctx.t_stage_ign
            ctx.burning = true
            ctx.burned = 0.0
            ctx.phase = :closed_loop                  # upper stages steer closed-loop
            isnan(ctx.t_loop0) && (ctx.t_loop0 = t)
            ev!(Symbol(:ignition_, lv.stages[ctx.stage].name))
        end

        if t >= next_log
            # Log the engines and mass after this step's discrete transitions.
            d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
            logrec!(d)
            next_log += log_dt
        end

        # Step to the next event rather than past it. Burnouts are the one
        # boundary a fixed step gets visibly wrong: overshooting it burns
        # propellant the stage does not have, so the last step of a burn is
        # trimmed to land exactly on depletion.
        thr_now = ctx.burning && ctx.stage >= 1 ?
                  core_throttle(lv, ctx, x[7], d.pamb) : 0.0
        dts = dt
        if thr_now > 0
            mdot_core = thr_now * stage_mdot(lv.stages[ctx.stage])
            dts = min(dts, (lv.stages[ctx.stage].mprop - ctx.burned) / mdot_core)
        end
        for (i, b) in enumerate(lv.boosters)
            ctx.bstate[i] === :burning || continue
            dts = min(dts, stage_burn_time(b.stage) - (t - ctx.b_tign[i]))
        end
        dts = clamp(dts, 1e-6, dt)
        m_before = x[7]
        _rk4_ascent!(xnew, x, t, dts, w, lv, guid, ctx, atmosphere, gravity, theta_g0)
        copyto!(x, xnew); t += dts
        if ctx.burning && ctx.stage >= 1
            # Bill the stage what actually left the tank. `thr_now` is the
            # throttle at the START of the step; with the acceleration limiter
            # engaged it falls as mass is consumed, so `thr_now * mdot * dt`
            # over-bills and depletion fires while propellant is still aboard —
            # a phantom mass the stage never actually carries away.
            #
            # With no boosters every kilogram the step removed is core
            # propellant, so the integrator's own mass change is the exact
            # figure. With boosters the limiter is disabled (`core_throttle`
            # only limits when there are none) and the throttle is constant, so
            # the rated rate is already exact.
            ctx.burned += isempty(lv.boosters) ?
                (m_before - x[7]) :
                thr_now * stage_mdot(lv.stages[ctx.stage]) * dts
        end
    end
    # final log point
    d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere)
    logrec!(d)

    r = (x[1], x[2], x[3]); v = (x[4], x[5], x[6])
    el = elements_from_state(r, v)
    AscentResult(L, events, r, v, x[7], t, el, prop_left, reached, h_cut, gam_cut)
end

function _ascent_data(x, lv::LaunchVehicle, ctx::AscentCtx, theta_g0, t, r_site0,
                      atm::AbstractAtmosphere = USSA76())
    r = (x[1], x[2], x[3]); v = (x[4], x[5], x[6])
    theta = earth_rotation_angle(theta_g0, t)
    lat, lon, h = geodetic_from_ecef(rot_z(r, theta))
    omega = (0.0, 0.0, OMEGA_EARTH)
    vrel = vsub(v, vcross(omega, r))
    Vr = vnorm(vrel)
    rho = 0.0; asnd = 300.0; pamb = 0.0
    if h < 150e3
        # The SAME model the dynamics integrate, not a hardcoded standard one:
        # otherwise a dispersed ascent flies scaled air while its guidance,
        # events, step-trim and logs are computed for standard air.
        rho, _, pamb, asnd = atmosphere_state(atm, max(h, 0.0))
    end
    qbar = 0.5 * rho * Vr * Vr
    rhat = vunit(r)
    vin = vnorm(v)
    gamma = vin > 1 ? asin(clamp(vdot(rhat, vscale(v, 1 / vin)), -1.0, 1.0)) : pi / 2
    # The flight path angle relative to the ATMOSPHERE, which is the one an
    # attitude is judged against: a rocket standing still on the pad is flying
    # straight up by this measure and at 62 degrees by the inertial one, because
    # inertially it is already going 400 m/s sideways with the planet.
    gamma_rel = Vr > 1 ? asin(clamp(vdot(rhat, vscale(vrel, 1 / Vr)), -1.0, 1.0)) : pi / 2
    # logged thrust is what the stack is actually producing: the throttled
    # core plus every strap-on still burning
    thrust = ctx.burning && ctx.stage >= 1 ?
             core_throttle(lv, ctx, x[7], pamb) *
             stage_thrust(lv.stages[ctx.stage], pamb) : 0.0
    for (i, b) in enumerate(lv.boosters)
        ctx.phase !== :coast && ctx.bstate[i] === :burning && (thrust += booster_thrust(b, pamb))
    end
    # downrange: great-circle from the launch site's inertial position
    dr = RE_MEAN * acos(clamp(vdot(vunit(r_site0), rhat), -1.0, 1.0))
    (h = h, vrel = Vr, vin = vin, gamma = gamma, gamma_rel = gamma_rel,
     mach = Vr / asnd, qbar = qbar, pamb = pamb,
     lat = lat, lon = lon, thrust = thrust, downrange = dr)
end

"""
    tune_ascent(lv, guid; tol_h, tol_gamma, max_iter, optimize_kick) -> (guid, result)

Damped-Newton shooting on (pitch0, pitch_rate) of the linear-tangent law,
driving cutoff altitude and flight-path angle to (h_target, 0).

The pitch-over kick is left alone by default. It is not a constraint — the
2x2 above already pins the insertion state — but it decides how much of the
climb is spent fighting gravity, and the best value moves with thrust-to-
weight: a stack with strap-on boosters lifts off so hard that the reference
8-degree kick lofts it, and it arrives at the target energy having wasted
much of the extra impulse. `optimize_kick = true` scans kick angles, solves
the 2x2 inside each, and keeps whichever puts the most mass in orbit. It
costs a few seconds, so it is opt-in.
"""
function tune_ascent(lv::LaunchVehicle, guid::AscentGuidance;
                     tol_h::Float64 = 1.0e3, tol_gamma::Float64 = deg2rad_(0.05),
                     max_iter::Int = 30, verbose::Bool = false,
                     optimize_kick::Bool = false, recover_kick::Bool = true, kwargs...)
    if optimize_kick
        return _tune_with_kick(lv, guid; tol_h = tol_h, tol_gamma = tol_gamma,
                               max_iter = max_iter, verbose = verbose, kwargs...)
    end
    p1, p2 = guid.pitch0, guid.pitch_rate
    local res
    resid(g) = begin
        r = simulate_ascent(lv, g; kwargs...)
        (r.h_cut - g.h_target, r.gamma_cut, r)
    end
    rebuild(p1, p2) = _reguid(guid; pitch0 = p1, pitch_rate = p2)
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
    if recover_kick && guid.cutoff === :energy && !res.reached_orbit &&
       pad_thrust(lv) > 1.05 * liftoff_mass(lv) * G0
        return _tune_with_kick(lv, guid; tol_h = tol_h, tol_gamma = tol_gamma,
                               max_iter = max_iter, verbose = verbose, kwargs...)
    end
    (g, res)
end

"Guidance with the pitch-over kick replaced."
_with_kick(g::AscentGuidance, ka::Float64) = _reguid(g; kick_angle = ka)

"""
    _tune_with_kick(lv, guid; ...) -> (guid, result)

Scan the pitch-over kick, solving the 2x2 pitch problem inside each
candidate, and keep the trajectory that reaches the target orbit with the
most mass. A coarse ladder locates the peak; one refinement pass at half the
spacing sharpens it. Candidates that miss the insertion tolerance or run out
of propellant score nothing, so the search never trades the orbit away for
mass.
"""
function _tune_with_kick(lv::LaunchVehicle, guid::AscentGuidance;
                         tol_h, tol_gamma, max_iter, verbose, kwargs...)
    # Each candidate is an independent shooting solve reading only immutable
    # inputs — `simulate_ascent` allocates its own state, work buffers and log
    # — so a ladder runs across threads with no sharing. Results are collected
    # positionally and reduced afterwards rather than racing on a running best.
    function scan(angles)
        out = Vector{Any}(undef, length(angles))
        Threads.@threads for i in eachindex(angles)
            ka = angles[i]
            if ka <= 0
                out[i] = nothing
                continue
            end
            g, r = tune_ascent(lv, _with_kick(guid, ka); tol_h = tol_h,
                               tol_gamma = tol_gamma, max_iter = max_iter,
                               optimize_kick = false, recover_kick = false, kwargs...)
            ok = r.reached_orbit && abs(r.h_cut - g.h_target) < tol_h &&
                 abs(r.gamma_cut) < tol_gamma
            out[i] = ok ? (g, r) : nothing
        end
        filter(!isnothing, out)
    end
    pick(cands) = isempty(cands) ? nothing : cands[argmax([c[2].m for c in cands])]

    # The ladder used to be 4 to 19 degrees in even 3-degree steps, and both
    # halves of that were wrong for a heavy stack.
    #
    # The FLOOR was the fatal one. A Saturn V lifts off at a thrust-to-weight of
    # 1.20 and needs about one degree; at two it no longer inserts at all.
    # Nothing in a 4-degree floor could find that, so the scan returned empty,
    # fell back to whatever angle was typed, and the vehicle flew into the sea —
    # which is what "the Saturn V doesn't work" looked like from the outside.
    #
    # The EVEN SPACING was wrong for the same reason. Sensitivity to this angle
    # is not uniform: between 1 and 2 degrees a Saturn V goes from orbit to no
    # orbit, while between 13 and 16 a light stack barely notices. A ladder
    # spaced roughly geometrically spends its samples where the answer changes,
    # and refinement follows the LOCAL spacing rather than a fixed half-step for
    # the same reason.
    ladder = deg2rad_.([0.6, 1.0, 1.5, 2.2, 3.0, 4.0, 5.5, 7.5, 10.0, 13.5, 18.0])
    best = pick(scan(ladder))
    if best !== nothing                                    # refine around the peak
        ka = best[1].kick_angle
        i = argmin(abs.(ladder .- ka))
        lo = i > 1 ? sqrt(ladder[i-1] * ka) : ka * 0.8
        hi = i < length(ladder) ? sqrt(ladder[i+1] * ka) : ka * 1.25
        best = pick(vcat([best], scan([lo, hi])))
        verbose && @info "kick scan" kick_deg = rad2deg_(best[1].kick_angle) m = best[2].m
    end
    best === nothing ? tune_ascent(lv, guid; tol_h = tol_h, tol_gamma = tol_gamma,
                                   max_iter = max_iter, optimize_kick = false,
                                    recover_kick = false, kwargs...) : best
end
