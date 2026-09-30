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
# below anything else here, and the reason the Earth can simply be dropped.
#
# How much world to fly through is the caller's choice, and it is made with a
# `DescentConfig`. Left empty — which is the default — the Moon is a point
# mass, the surface is a sphere of radius `R_MOON`, and the guidance reads the
# integrator's own state, which is enough to price a descent and not enough to
# test one. Filled in, the descent is flown over procedural terrain
# (`terrain.jl`), through an oblate and mascon-lumped gravity field
# (`moon.jl`), on a navigation state that starts wrong and is corrected by
# landing radar (`landingnav.jl`), to a landing point the vehicle picks for
# itself at high gate. Each of those turns on a way to fail that the sphere
# hid.
#
# Still not modelled: abort, staging during descent, plume-surface interaction,
# and any attitude dynamics at all — thrust points where guidance asks, with no
# rate limit and no RCS budget to pay for it.

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

"""
    AscentStage(; name, mdry, mprop, thrust, isp)

The part of the lander that comes back. Its wet mass is carried *inside* the
lander's dry mass — the descent stage is a launch pad that gets left behind —
so a `Lander` with `mdry = 3500` carrying a 2800 kg ascent stage really has
700 kg of descent structure under it.

The defaults are Apollo-LM-proportioned and about half the size: 1.3 t dry on
1.5 t of hypergolic propellant through a 15.6 kN fixed-thrust engine, which is
2340 m/s of ideal velocity against the roughly 1850 m/s it costs to reach
lunar orbit. The margin is the rendezvous. See `lunarreturn.jl` for what it
does with it.
"""
Base.@kwdef struct AscentStage
    name::Symbol = :ascent
    mdry::Float64 = 1300.0
    mprop::Float64 = 1500.0
    thrust::Float64 = 15.6e3
    isp::Float64 = 311.0
end

"Wet mass of an ascent stage [kg]."
ascent_mass(a::AscentStage) = a.mdry + a.mprop

"Ideal vacuum delta-v it carries [m/s]."
ascent_dv(a::AscentStage) = G0 * a.isp * log(ascent_mass(a) / a.mdry)

"Full-throttle mass flow [kg/s]."
@inline _ascent_mdot(a::AscentStage) = a.thrust / (G0 * a.isp)

"""
    Orbiter(; name, mdry, mprop, thrust, isp)

What waits in lunar orbit: the vehicle that never lands, does its own orbit
insertion, holds station while the lander is away, and burns for home once the
ascent stage has caught up with it. Apollo called it the command and service
module; the arithmetic calls it "the place to leave the trans-Earth
propellant", and leaving it there rather than landing it is the whole argument
for lunar-orbit rendezvous.

The default carries about 2000 m/s of ideal velocity, which has to cover
insertion (about 925 m/s) and trans-Earth injection (about 900 m/s) with the
rest for trim. It is deliberately tight — that is what the real one was.
"""
Base.@kwdef struct Orbiter
    name::Symbol = :orbiter
    mdry::Float64 = 5200.0
    mprop::Float64 = 4800.0
    thrust::Float64 = 45.0e3
    isp::Float64 = 314.0
end

"Wet mass of the orbiter [kg]."
orbiter_mass(o::Orbiter) = o.mdry + o.mprop

"Ideal vacuum delta-v it carries [m/s]."
orbiter_dv(o::Orbiter) = G0 * o.isp * log(orbiter_mass(o) / o.mdry)

# ---------------------------------------------------------------- logging --

"""
Powered-descent log. `downrange` is arc length over the surface from the point
under the vehicle at ignition, so it is directly comparable with the maps
Apollo's crews used; `throttle` is the commanded fraction, which is the
number that says whether the engine could actually fly the trajectory.

`h` is height above the *ground*, not above the mean sphere — over terrain the
two differ by the `elev` column, and the gap between them is what a lander
flying on a sphere would have got wrong. `nav_dh` and `nav_dr` are how wrong
the vehicle believed itself to be at each sample; both are zero when nothing
is modelling navigation, which is to say when the vehicle is assumed to know.
"""
struct DescentLog
    t::Vector{Float64}          # seconds from powered-descent ignition
    h::Vector{Float64}          # altitude above the ground below [m]
    downrange::Vector{Float64}  # surface arc from ignition [m]
    v::Vector{Float64}          # speed in the Moon frame [m/s]
    vh::Vector{Float64}         # horizontal (along-track) component [m/s]
    vv::Vector{Float64}         # vertical (radial) component [m/s]
    m::Vector{Float64}          # mass [kg]
    throttle::Vector{Float64}   # commanded fraction of full thrust
    pitch::Vector{Float64}      # thrust elevation above local horizontal [rad]
    x::Vector{Float64}; y::Vector{Float64}; z::Vector{Float64}  # Moon-centred [m]
    elev::Vector{Float64}       # ground elevation above the mean sphere [m]
    nav_dh::Vector{Float64}     # navigation altitude error [m] (est - truth)
    nav_dr::Vector{Float64}     # navigation position error magnitude [m]
    cross::Vector{Float64}      # crossrange offset to the designated site [m]
end
DescentLog() = DescentLog((Float64[] for _ in 1:16)...)

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
    outcome::Symbol             # :touchdown | :timeout | :crash | :tipped | :propellant | :diverged
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
    slope::Float64              # ground slope where it came to rest [rad]
    elev::Float64               # ground elevation there, above the mean sphere [m]
    site_score::Float64         # hazard score of the chosen site [rad], NaN if none
    site_score_nominal::Float64 # hazard score of the site it would have taken
    redesignated::Float64       # how far the aim point moved [m]
    nav_err::Float64            # final navigation position error [m]
    nav_dh::Float64             # final navigation altitude error [m]
    radar_locked::Bool          # did the landing radar ever acquire
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
    # what was left in lunar orbit, if anything: the vehicle the ascent stage
    # has to come back to. `nothing` is a one-way mission.
    orbiter::Union{Nothing,Orbiter}
    m_orbiter::Float64          # orbiter mass after its own insertion burn [kg]
    r_orbiter::V3               # orbiter state at insertion, Moon-centred
    v_orbiter::V3
end

# ------------------------------------------------------- Moon-frame basics --

"""
Lunar gravity in the Moon-centred frame. With no field it is the point mass
the whole trans-lunar chain uses; with one it is oblate and lumpy, and the
lumps are fixed to the Moon rather than to inertial space, which is why the
time and the ephemeris have to come along.
"""
@inline _moon_accel(r::V3) = vscale(r, -MU_MOON / vnorm(r)^3)
@inline _moon_accel(r::V3, t::Float64, field, eph) = lunar_gravity(r, field, t, eph)

"RK4 step of a ballistic Moon-centred coast."
function _moon_step(r::V3, v::V3, dt::Float64; t::Float64 = 0.0,
                    field = nothing, eph = nothing)
    acc(rr, tt) = _moon_accel(rr, tt, field, eph)
    k1v = acc(r, t);                            k1r = v
    r2 = vadd(r, vscale(k1r, dt/2)); v2 = vadd(v, vscale(k1v, dt/2))
    k2v = acc(r2, t + dt/2);                    k2r = v2
    r3 = vadd(r, vscale(k2r, dt/2)); v3 = vadd(v, vscale(k2v, dt/2))
    k3v = acc(r3, t + dt/2);                    k3r = v3
    r4 = vadd(r, vscale(k3r, dt));   v4 = vadd(v, vscale(k3v, dt))
    k4v = acc(r4, t + dt);                      k4r = v4
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
    xhat, yhat, zhat = moonfixed_basis(eph, t)
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

# -------------------------------------------------------- site targeting ---
# Where a chosen landing site puts the parking orbit.
#
# The descent periapsis is the ANTIPODE of the DOI burn on the parking circle —
# DOI drops the apoapsis where the vehicle is, so the ellipse reaches its low
# point half a revolution later, on the opposite side of the Moon. To touch down
# at a chosen selenographic point the parking orbit has to be tilted so that the
# site's antipode lies ON the orbit at the DOI instant, and the vehicle has to be
# there. The plane is the one through the arrival perilune (the LOI burn point,
# which the orbit must contain) and the site's Moon-fixed direction at PDI; the
# phasing is the wait that carries the vehicle from the perilune round to the
# site's antipode at the circular rate — never more than one extra revolution.
# Both depend on the PDI time, and the PDI time depends on the wait, so this is
# a fixed point; it contracts hard (the site turns at a lunar day against a
# two-hour orbit) and settles in a handful of passes.
function target_parking(eph::CircularMoonEphemeris, t_loi::Float64, r_m::V3,
                        v_m::V3, h_moon_park::Float64, h_pdi::Float64,
                        u_t::V3, n_rev::Int = 0)
    rp = R_MOON + h_moon_park
    v_circ = sqrt(MU_MOON / rp)
    rhat = vunit(r_m)
    a_desc = 0.5 * (rp + R_MOON + h_pdi)
    t_transfer = pi * sqrt(a_desc^3 / MU_MOON)
    omega = sqrt(MU_MOON / rp^3)              # circular rate [rad/s]
    T_park = 2pi / omega
    t_pdi = t_loi + n_rev * T_park + t_transfer
    v_park = v_m
    t_doi = t_loi + n_rev * T_park
    for _ in 1:60
        Up = vunit(moonfixed_inv(u_t, t_pdi, eph))
        cr = vcross(rhat, Up)
        # perilune and the site on top of each other: any plane through them is
        # fine, so keep the arrival plane and let the phase carry the vehicle
        h = vnorm(cr) > 1e-9 ? vunit(cr) : vunit(vcross(rhat, (0.0, 0.0, 1.0)))
        vp = vsub(v_m, vscale(h, vdot(v_m, h)))
        if vnorm(vp) < 1e-12
            vp = vcross(h, rhat)
        end
        what = vunit(vp)
        v_park = vscale(what, v_circ)
        # the vehicle at DOI must sit at the site's antipode
        a = atan(-vdot(Up, what), -vdot(Up, rhat))
        a < 0 && (a += 2pi)
        t_doi = t_loi + n_rev * T_park + a / omega
        t_pdi_new = t_doi + t_transfer
        if abs(t_pdi_new - t_pdi) < 1e-4
            t_pdi = t_pdi_new
            break
        end
        t_pdi = t_pdi_new
    end
    (v_park = v_park, v_circ = v_circ, t_doi = t_doi, t_pdi = t_pdi,
     wait = t_doi - t_loi, dv = vnorm(vsub(v_park, v_m)))
end

"Rotate a vector about a unit axis by `ang` (Rodrigues)."
function rot_about(u::V3, axis::V3, ang::Float64)
    a = vunit(axis); c = cos(ang); s = sin(ang)
    vadd(vadd(vscale(u, c), vscale(vcross(a, u), s)),
         vscale(a, vdot(a, u) * (1 - c)))
end

# The parking coast, DOI and the half-ellipse to the descent periapsis, with no
# logging — what the site-aiming loop needs to know where a nominal descent
# would come down, without paying for the viewer's orbit log.
function descent_orbit(eph::CircularMoonEphemeris, t_loi::Float64, r_m::V3,
                       v_park::V3, wait::Float64, h_pdi::Float64, field)
    L = LunarOrbitLog()
    r, v = r_m, v_park
    r, v = coast_moon!(L, r, v, t_loi, wait; phase = 0, dt = 5.0,
                       log_every = typemax(Int), field = field, eph = eph)
    t_doi = t_loi + wait
    dv_doi, v_doi = doi_burn(r, v, h_pdi)
    a = 0.5 * (vnorm(r) + R_MOON + h_pdi)
    tt = pi * sqrt(a^3 / MU_MOON)
    r_pdi, v_pdi = coast_moon!(L, r, v_doi, t_doi, tt; phase = 1, dt = 2.0,
                               log_every = typemax(Int), field = field, eph = eph)
    (r_pdi = r_pdi, v_pdi = v_pdi, t_pdi = t_doi + tt, dv_doi = dv_doi)
end

"""
    coast_moon!(L, r, v, t, dt_total; phase, dt, log_every) -> (r, v)

Ballistic Moon-centred coast of `dt_total` seconds, logging as it goes.
"""
function coast_moon!(L::LunarOrbitLog, r::V3, v::V3, t::Float64,
                     dt_total::Float64; phase::Int = 0, dt::Float64 = 5.0,
                     log_every::Int = 4, field = nothing, eph = nothing)
    n = max(1, ceil(Int, dt_total / dt))
    step = dt_total / n
    for k in 1:n
        if (k - 1) % log_every == 0
            push!(L.t, t); push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
            push!(L.h, vnorm(r) - R_MOON); push!(L.phase, phase)
        end
        r, v = _moon_step(r, v, step; t = t, field = field, eph = eph)
        t += step
    end
    push!(L.t, t); push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
    push!(L.h, vnorm(r) - R_MOON); push!(L.phase, phase)
    (r, v)
end

# ------------------------------------------------------- powered descent ---

"""
    DescentConfig(; surface, field, nav, hazard, eph, t0)

Everything the descent knows about the world it is descending into. Every
field is optional, and with all of them left out the descent is flown exactly
as it was before any of this existed: a point-mass Moon, a spherical surface,
and a guidance system that reads the integrator's own state vector.

  * `surface` — a [`SurfaceModel`](@ref), so altitude means height above the
    ground rather than above a sphere.
  * `field` — a [`LunarGravity`](@ref), so the Moon is oblate and lumpy.
  * `nav` — a [`DescentNav`](@ref), so the guidance flies on an estimate that
    starts wrong and has to be corrected by radar rather than on the truth.
  * `hazard` — a [`HazardScan`](@ref), so the vehicle picks its own landing
    point at high gate instead of arriving wherever the braking phase aimed.
  * `eph`, `t0` — the ephemeris and the mission time at ignition, which are
    what tie the Moon-fixed frame (where the terrain and the mascons live) to
    the inertial frame the descent is integrated in.

Turning any of these on turns on a failure mode that was previously invisible.
That is the point of them: a descent that always succeeds has not been tested.
"""
Base.@kwdef struct DescentConfig
    surface::Union{Nothing,SurfaceModel} = nothing
    field::Union{Nothing,LunarGravity} = nothing
    nav::Union{Nothing,DescentNav} = nothing
    hazard::Union{Nothing,HazardScan} = nothing
    eph::Union{Nothing,CircularMoonEphemeris} = nothing
    t0::Float64 = 0.0
end

"""
    nominal(cfg) -> DescentConfig

The world as the *designer* sees it: sphere, point mass, perfect knowledge.
The braking pitch program is shot against this and then flown against the
real one, which is how a pre-computed open-loop program is actually produced
— nobody shoots a trajectory through terrain they have not flown over yet.
Everything the two worlds disagree about is left for the closed-loop terminal
phase to absorb, and how much of it there is to absorb is the interesting
number.
"""
nominal(cfg::DescentConfig) = DescentConfig(eph = cfg.eph, t0 = cfg.t0)

"Ground radius under a Moon-centred position, at descent time `t`."
@inline _ground(cfg::DescentConfig, r::V3, t::Float64) =
    surface_radius(cfg.surface, r, cfg.t0 + t)

"Height above the ground directly below [m]."
@inline _alt(cfg::DescentConfig, r::V3, t::Float64) = vnorm(r) - _ground(cfg, r, t)

"""
Velocity of the ground itself at a Moon-centred position, in the inertial
frame. The Moon turns once a month, which is 4.6 m/s at the equator — small
against a 1.7 km/s orbit and enormous against a lander's 1.2 m/s lateral
touchdown limit. A guidance law that nulls *inertial* velocity therefore
arrives sliding sideways at four times the speed that tips the vehicle over,
and a vehicle that hovers over a chosen site for a minute and a half while
holding inertial velocity to zero drifts four hundred metres off it. Both of
those are landings ruined by a rotation rate you can barely see on a plot.
"""
@inline function _surface_vel(cfg::DescentConfig, r::V3, t::Float64)
    cfg.eph === nothing && return (0.0, 0.0, 0.0)
    _, _, zh = moonfixed_basis(cfg.eph, cfg.t0 + t)
    vcross(vscale(zh, N_MOON), r)
end

"Gravity at a Moon-centred position, at descent time `t`."
@inline _grav(cfg::DescentConfig, r::V3, t::Float64) =
    cfg.field === nothing || cfg.eph === nothing ? _moon_accel(r) :
    lunar_gravity(r, cfg.field, cfg.t0 + t, cfg.eph)

"""
In-plane frame at a Moon-centred position: radial-out and along-track, the
latter fixed by the orbit normal `hhat` captured at ignition rather than by
the instantaneous velocity. That distinction is the whole difference between
a descent and a divergence: a frame built on the velocity flips end-for-end
the moment the vehicle stops flying forward, so "retrograde" reverses under
the guidance in the last thirty seconds of braking and the thrust that was
slowing the vehicle starts accelerating it back up. The normal is constant —
the braking phase does not thrust out of plane — so the along-track direction
stays the direction the vehicle was originally going, all the way down.

`hhat` itself is the third axis: crossrange, which is unused until the
vehicle is allowed to redesignate its landing point and then becomes the axis
it translates along to miss a crater.
"""
@inline function _descent_frame(r::V3, hhat::V3)
    ur = vunit(r)
    (ur, vcross(hhat, ur))
end

"Orbit normal of a Moon-centred state — the descent plane, fixed at ignition."
@inline _descent_normal(r::V3, v::V3) = vunit(vcross(r, v))

"""
    _descent_leg(lander, r0, v0, m0, pitch0, pitch_rate; vh_gate, cfg, nav, ...)

Braking phase: full thrust, thrust elevation above the local horizontal
following `theta(t) = pitch0 + pitch_rate * t`, integrated until the
along-track speed falls through `vh_gate` — high gate. It also stops early if
the tank runs dry, the vehicle reaches the surface, it climbs away, or the
clock runs out; the shooter needs those apart to tell a miss from a
divergence.

High gate is a *velocity* condition, not an altitude one, and that is what
makes the phase shootable. Braking is nearly all horizontal, so where the
vehicle ends up in altitude is the free variable the pitch program controls;
picking the altitude instead would fix the one thing the guidance has
authority over and leave the speed — the thing that has to be gone — as
whatever fell out.

Full thrust is not an approximation for effect: a braking phase wants every
newton it has, and Apollo held maximum thrust for all but the first and last
minutes of its descent. Throttling is what the terminal phase is for.

When a `nav` state is supplied, the gate is called on what the vehicle
*believes* its along-track speed is, because that is the only number it has.
The nav state is propagated and radar-corrected alongside the truth, so the
braking phase is also where the radar acquires — which is exactly where it
acquired on Apollo, and for the same reason: it is the last chance to fix the
altitude before altitude starts mattering.
"""
function _descent_leg(l::Lander, r0::V3, v0::V3, m0::Float64,
                      pitch0::Float64, pitch_rate::Float64;
                      vh_gate::Float64 = 150.0, dt::Float64 = 0.5,
                      t_max::Float64 = 1200.0, log::Union{Nothing,DescentLog} = nothing,
                      r_ref::V3 = r0, log_every::Int = 4,
                      cfg::DescentConfig = DescentConfig(),
                      nav::Union{Nothing,NavState} = nothing)
    r, v, m, t = r0, v0, m0, 0.0
    mdot = lander_mdot(l)
    m_dry = m0 - l.mprop
    h0 = _alt(cfg, r0, 0.0)
    hhat = _descent_normal(r0, v0)
    # :gate is a *result*, set only by the two gate-crossing breaks below. The
    # initial value is the failure the loop can fall out of: running `t_max`
    # out still above the gate and still fast. It used to be :gate, which
    # reported a clock exhaustion as a hit and let an unflyable leg through.
    outcome = :timeout
    kount = 0
    radar = nav === nothing || cfg.nav === nothing ? nothing : cfg.nav.radar

    thrust_dir(rr, tt) = begin
        ur, ut = _descent_frame(rr, hhat)
        th = clamp(pitch0 + pitch_rate * tt, -deg2rad_(60.0), deg2rad_(89.0))
        (vadd(vscale(ur, sin(th)), vscale(ut, -cos(th))), th)
    end

    vhof(rr, vv, tt) = (ur = vunit(rr);
                        vdot(vsub(vv, _surface_vel(cfg, rr, tt)), vcross(hhat, ur)))
    # the number the *vehicle* has: its own estimate when it is navigating,
    # the truth when nothing is modelling how it found out
    guide_vh() = nav === nothing ? vhof(r, v, t) : vhof(nav.r, nav.v, t)

    while t < t_max
        h = _alt(cfg, r, t)
        if log !== nothing && kount % log_every == 0
            _, th = thrust_dir(r, t)
            _log_descent!(log, t, r, v, m, 1.0, th, r_ref, hhat, cfg, nav)
        end
        kount += 1
        if guide_vh() <= vh_gate
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
        acc(rr, mm, tt) = begin
            d, _ = thrust_dir(rr, tt)
            vadd(_grav(cfg, rr, tt), vscale(d, l.thrust / mm))
        end
        k1r = v;                            k1v = acc(r, m, t)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2)); m2 = m - mdot*step/2
        k2r = v2;                           k2v = acc(r2, m2, t + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                           k3v = acc(r3, m2, t + step/2)
        r4 = vadd(r, vscale(k3r, step));   v4 = vadd(v, vscale(k3v, step));   m4 = m - mdot*step
        k4r = v4;                           k4v = acc(r4, m4, t + step)
        rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        # land exactly on the gate rather than stepping past it: over half a
        # second the state is linear to well under a metre. Only worth doing
        # when the gate is called on truth — a vehicle calling it on its own
        # estimate at 4 Hz cannot split a control cycle either.
        if nav === nothing && vhof(rn, vn, t + step) < vh_gate
            lo, hi = 0.0, 1.0
            for _ in 1:40
                f = 0.5 * (lo + hi)
                rm = vadd(r, vscale(vsub(rn, r), f))
                vm = vadd(v, vscale(vsub(vn, v), f))
                if vhof(rm, vm, t + step * f) > vh_gate; lo = f; else; hi = f; end
            end
            f = 0.5 * (lo + hi)
            r = vadd(r, vscale(vsub(rn, r), f))
            v = vadd(v, vscale(vsub(vn, v), f))
            m -= mdot * step * f
            t += step * f
            outcome = :gate
            break
        end
        if nav !== nothing
            d, _ = thrust_dir(r, t)
            nav_propagate!(nav, vscale(d, l.thrust / m), step)
            radar === nothing ||
                radar_update!(nav, radar, rn, vn, t + step, cfg.surface,
                              cfg.t0 + t + step, hhat)
        end
        r, v, m, t = rn, vn, m - mdot*step, t + step
    end
    ur, ut = _descent_frame(r, hhat)
    vr = vsub(v, _surface_vel(cfg, r, t))
    (r = r, v = v, m = m, t = t, outcome = outcome,
     h = _alt(cfg, r, t), vv = vdot(vr, ur), vh = vdot(vr, ut))
end

"Push one sample onto a descent log."
function _log_descent!(L::DescentLog, t, r::V3, v::V3, m, throttle, pitch,
                       r_ref::V3, hhat::V3, cfg::DescentConfig = DescentConfig(),
                       nav::Union{Nothing,NavState} = nothing,
                       target::Union{Nothing,V3} = nothing)
    ur, ut = _descent_frame(r, hhat)
    # velocities are logged relative to the ground, which is what a landing is
    # measured against — the 4.6 m/s the surface itself carries is invisible on
    # a plot of a 1.7 km/s orbit and decisive on a plot of a touchdown
    vr = vsub(v, _surface_vel(cfg, r, t))
    push!(L.t, t); push!(L.h, _alt(cfg, r, t))
    push!(L.downrange, R_MOON * acos(clamp(vdot(vunit(r), vunit(r_ref)), -1.0, 1.0)))
    push!(L.v, vnorm(vr)); push!(L.vh, vdot(vr, ut)); push!(L.vv, vdot(vr, ur))
    push!(L.m, m); push!(L.throttle, throttle); push!(L.pitch, pitch)
    push!(L.x, r[1]); push!(L.y, r[2]); push!(L.z, r[3])
    push!(L.elev, ground_elevation(cfg.surface, r, cfg.t0 + t))
    if nav === nothing
        push!(L.nav_dh, 0.0); push!(L.nav_dr, 0.0)
    else
        dr, _, dh = nav_error(nav, r, v, cfg.surface, cfg.t0 + t)
        push!(L.nav_dh, dh); push!(L.nav_dr, dr)
    end
    push!(L.cross, target === nothing || cfg.eph === nothing ? 0.0 :
          vdot(vscale(vsub(moonfixed_inv(target, cfg.t0 + t, cfg.eph), vunit(r)),
                      R_MOON), hhat))
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

The shoot is always flown against the nominal world — sphere, point mass,
perfect state — whatever world the descent will actually be flown in. That is
not a shortcut: an open-loop pitch program *is* a pre-computed object, and
pre-computing it against terrain the vehicle has not reached yet would be
assuming away the very error the closed-loop phase exists to absorb.
"""
function tune_braking(l::Lander, r0::V3, v0::V3, m0::Float64;
                      h_gate::Float64 = 2300.0, vv_gate::Float64 = -45.0,
                      vh_gate::Float64 = 150.0, max_iter::Int = 25,
                      cfg::DescentConfig = DescentConfig(),
                      verbose::Bool = false)
    ncfg = nominal(cfg)
    function resid(p0, pr)
        leg = _descent_leg(l, r0, v0, m0, p0, pr; vh_gate = vh_gate, cfg = ncfg)
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
the vehicle does not dive — and flies the horizontal channels to null, or to
a designated landing point if it has one. The commanded acceleration becomes
a thrust vector, the magnitude is clamped to the engine's throttle band, and
what the engine cannot deliver simply is not delivered: if the deepest
throttle still exceeds lunar gravity at the current mass, the vehicle climbs,
and the log shows it.

The command is computed once per cycle and held across it, because that is
what a digital autopilot does. It is also computed from whatever the vehicle
knows — its own navigation estimate when it is navigating — while the vehicle
itself is integrated over the real ground. Everything interesting about a
landing lives in the gap between those two.
"""
function terminal_descent(l::Lander, r0::V3, v0::V3, m0::Float64;
                          m_dry::Float64, v_touch::Float64 = 0.8,
                          k_profile::Float64 = 0.85, v_cap::Float64 = 25.0,
                          tau_h::Float64 = 18.0, tau_v::Float64 = 5.0,
                          dt::Float64 = 0.1, t_max::Float64 = 900.0,
                          log::Union{Nothing,DescentLog} = nothing,
                          t0::Float64 = 0.0, r_ref::V3 = r0, log_every::Int = 10,
                          cfg::DescentConfig = DescentConfig(),
                          nav::Union{Nothing,NavState} = nothing,
                          target::Union{Nothing,V3} = nothing,
                          arrival::Float64 = 30.0, vh_cap::Float64 = 180.0)
    r, v, m, t = r0, v0, m0, 0.0
    mdot_full = lander_mdot(l)
    hhat = _descent_normal(r0, v0)
    min_thr = 1.0
    # As in `_descent_leg`: :touchdown is set only by the ground-contact breaks.
    # Falling out of the loop on `t_max` is a timeout, not a landing.
    outcome = :timeout
    kount = 0
    radar = nav === nothing || cfg.nav === nothing ? nothing : cfg.nav.radar

    # position error to the designated site, in the along-track / crossrange
    # pair, from whatever position the vehicle believes it holds. Times here
    # are seconds from powered-descent ignition — `t0` is the handover from
    # the braking phase — because the terrain and the mascons are fixed to a
    # Moon that is turning, and a phase that restarts its own clock looks
    # them up half a kilometre away from where it is.
    offsets(rr, ta) = begin
        target === nothing && return (0.0, 0.0)
        ui = moonfixed_inv(target, cfg.t0 + ta, cfg.eph)
        w = vscale(vsub(ui, vunit(rr)), R_MOON)
        ur, ut = _descent_frame(rr, hhat)
        (vdot(w, ut), vdot(w, hhat))
    end

    command(rr, vv, mm, hh, ta) = begin
        ur, ut = _descent_frame(rr, hhat)
        # horizontal channels fly relative to the ground, which is moving
        vr = vsub(vv, _surface_vel(cfg, rr, ta))
        vv_now = vdot(vv, ur); vh_now = vdot(vr, ut); vc_now = vdot(vr, hhat)
        d_rem, c_rem = offsets(rr, ta)
        # Do not descend faster than the approach can converge. The two
        # channels are otherwise independent, and independent is wrong: the
        # sink-rate profile runs out of altitude on its own schedule, and if
        # the vehicle is still eight hundred metres from its landing site when
        # that happens it lands eight hundred metres from its landing site —
        # which, on ground chosen precisely because everywhere else was worse,
        # is the same as not having chosen. So the offset sets a floor on how
        # long the descent has to take, and the profile is clipped to it. A
        # pilot flying the last minute by hand does exactly this, and calls it
        # hovering until the site is underneath.
        off = hypot(d_rem, c_rem)
        t_go = off > 25.0 ? arrival * Base.log(off / 25.0) : 0.0
        v_allow = t_go > 0.1 ? hh / t_go : Inf
        v_cmd = -min(v_cap, v_touch + k_profile * sqrt(max(hh, 0.0)), v_allow)
        # Feed-forward on the profile itself. The commanded sink rate is a
        # function of altitude, so it moves as the vehicle descends, and a
        # pure proportional law lags it by tau_v * dv_cmd/dt — metres per
        # second of extra sink exactly where it hurts, because the profile
        # steepens as 1/sqrt(h) near the ground. Differentiating the profile
        # along the trajectory and commanding that outright leaves the
        # proportional term with nothing but the error to correct.
        dv_dh = v_cmd <= -v_cap ? 0.0 : -0.5 * k_profile / sqrt(max(hh, 1.0))
        a_r = (v_cmd - vv_now) / tau_v + dv_dh * vv_now
        vh_des = clamp(d_rem / arrival, -vh_cap, vh_cap)
        vc_des = clamp(c_rem / arrival, -0.2 * vh_cap, 0.2 * vh_cap)
        a_t = (vh_des - vh_now) / tau_h
        a_c = (vc_des - vc_now) / tau_h
        # cancel gravity and the centrifugal relief of whatever speed remains
        g_eff = MU_MOON / vnorm(rr)^2 - vh_now^2 / vnorm(rr)
        ar_tot = a_r + g_eff
        # Thrust is finite, and the channels are not equally important:
        # arriving with a few m/s of drift is a bad landing, arriving with an
        # unchecked sink rate is a crater. So the vertical demand is served
        # first and the two horizontal commands share whatever is left over.
        a_max = l.thrust / mm
        if abs(ar_tot) > a_max
            a_t = 0.0; a_c = 0.0
        else
            lim = sqrt(max(a_max^2 - ar_tot^2, 0.0))
            ah = hypot(a_t, a_c)
            if ah > lim && ah > 0.0
                a_t *= lim / ah; a_c *= lim / ah
            end
        end
        a_des = vadd(vadd(vscale(ur, ar_tot), vscale(ut, a_t)), vscale(hhat, a_c))
        an = vnorm(a_des)
        thr = clamp(mm * an / l.thrust, l.throttle_min, 1.0)
        dir = an > 1e-9 ? vscale(a_des, 1 / an) : ur
        (dir, thr)
    end

    # what the vehicle flies on: its own estimate, or the truth if nothing is
    # modelling how it found out
    guide(ta) = nav === nothing ? (r, v, _alt(cfg, r, ta)) :
                                  (nav.r, nav.v, nav_altitude(nav))

    while t < t_max
        ta = t0 + t
        h = _alt(cfg, r, ta)
        rg, vg, hg = guide(ta)
        dir, thr = command(rg, vg, m, hg, ta)
        min_thr = min(min_thr, thr)
        if log !== nothing && kount % log_every == 0
            ur, _ = _descent_frame(r, hhat)
            _log_descent!(log, ta, r, v, m, thr,
                          asin(clamp(vdot(dir, ur), -1.0, 1.0)), r_ref, hhat,
                          cfg, nav, target)
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
        # zero-order hold on the command across the control cycle: the thrust
        # direction and throttle a real vehicle flies are constants between
        # guidance updates, not functions re-evaluated inside the integrator
        a_th = vscale(dir, thr * l.thrust)
        acc(rr, mm, tt) = vadd(_grav(cfg, rr, tt), vscale(a_th, 1 / mm))
        k1r = v;                         k1v = acc(r, m, ta)
        r2 = vadd(r, vscale(k1r, step/2)); v2 = vadd(v, vscale(k1v, step/2))
        k2r = v2;                        k2v = acc(r2, m - thr*mdot_full*step/2, ta + step/2)
        r3 = vadd(r, vscale(k2r, step/2)); v3 = vadd(v, vscale(k2v, step/2))
        k3r = v3;                        k3v = acc(r3, m - thr*mdot_full*step/2, ta + step/2)
        r4 = vadd(r, vscale(k3r, step)); v4 = vadd(v, vscale(k3v, step))
        k4r = v4;                        k4v = acc(r4, m - thr*mdot_full*step, ta + step)
        rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step/6))
        vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step/6))
        hn = _alt(cfg, rn, ta + step)
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
        if nav !== nothing
            nav_propagate!(nav, vscale(a_th, 1 / m), step)
            radar === nothing ||
                radar_update!(nav, radar, rn, vn, ta + step, cfg.surface,
                              cfg.t0 + ta + step, hhat)
        end
        r, v, m, t = rn, vn, m - thr*mdot_full*step, t + step
    end
    ur, ut = _descent_frame(r, hhat)
    vrel = vsub(v, _surface_vel(cfg, r, t0 + t))
    (r = r, v = v, m = m, t = t, outcome = outcome, min_throttle = min_thr,
     v_vertical = -vdot(v, ur),
     v_horizontal = hypot(vdot(vrel, ut), vdot(vrel, hhat)))
end

"""
    powered_descent(lander, r0, v0, m0; h_gate, vh_gate, vv_gate, cfg, verbose)
        -> DescentResult

Braking phase (shot open-loop against the nominal world, flown against the
real one) followed by the closed-loop terminal phase, logged as one continuous
descent. If `cfg` carries a hazard scan, the landing point is chosen at high
gate, between the two.
"""
function powered_descent(l::Lander, r0::V3, v0::V3, m0::Float64;
                         h_gate::Float64 = 2300.0, vh_gate::Float64 = 150.0,
                         vv_gate::Float64 = -45.0, h_ref::Float64 = 0.0,
                         cfg::DescentConfig = DescentConfig(),
                         verbose::Bool = false)
    m_dry = m0 - l.mprop
    # The pitch program is shot on the sphere, so a gate 2 km over ground that
    # stands 1.5 km high has to be aimed at 3.5 km over the sphere. Getting
    # this one number wrong is the whole difference between arriving at high
    # gate with two kilometres to fly the approach in and arriving with five
    # hundred metres — which is not enough to stop, whatever the guidance
    # does afterwards. Apollo carried a landing-site radius in its targeting
    # for exactly this reason, and this is that number.
    p0, pr, _ = tune_braking(l, r0, v0, m0; h_gate = h_gate + h_ref,
                             vh_gate = vh_gate,
                             vv_gate = vv_gate, cfg = cfg, verbose = verbose)
    hhat = _descent_normal(r0, v0)
    nav = cfg.nav === nothing ? nothing : init_nav(cfg.nav, r0, v0, hhat)
    L = DescentLog()
    leg = _descent_leg(l, r0, v0, m0, p0, pr; vh_gate = vh_gate, log = L,
                       r_ref = r0, cfg = cfg, nav = nav)
    dv_brake = G0 * l.isp * log(m0 / leg.m)
    nav_dr0, _, nav_dh0 =
        nav === nothing ? (0.0, 0.0, 0.0) :
        nav_error(nav, leg.r, leg.v, cfg.surface, cfg.t0 + leg.t)
    # A braking phase that reached high gate is worth flying out even if the
    # shooter finished loose: the terminal phase is closed-loop on velocity,
    # so it either saves the landing or it does not, and the touchdown state
    # says which. Only a leg that never reached the gate is unflyable.
    if leg.outcome !== :gate
        return DescentResult(L, leg.outcome,
                             leg.t, leg.t, -leg.vv, leg.vh,
                             isempty(L.downrange) ? 0.0 : L.downrange[end],
                             dv_brake, 0.0, m0 - leg.m, leg.m - m_dry, 0.0, 1.0,
                             p0, pr, leg.r, leg.v, leg.m,
                             0.0, ground_elevation(cfg.surface, leg.r, cfg.t0 + leg.t),
                             NaN, NaN, 0.0, nav_dr0, nav_dh0,
                             nav !== nothing && nav.locked_h)
    end

    # --- landing-point designation ----------------------------------------
    _, ut_gate = _descent_frame(leg.r, hhat)
    target = nothing
    score = NaN; score0 = NaN; moved = 0.0
    if cfg.hazard !== nothing && cfg.surface !== nothing && cfg.eph !== nothing
        # where the vehicle would arrive if it simply flew its forward speed
        # out on the approach time constant: the aim point it is redesignating
        # away from
        rg = nav === nothing ? leg.r : nav.r
        vg = nav === nothing ? leg.v : nav.v
        ur, ut = _descent_frame(rg, hhat)
        lead = vdot(vg, ut) * cfg.hazard.arrival
        # The scan looks at the ground from where the vehicle actually is —
        # a sensor sees real terrain, not the terrain under where the
        # navigation filter thinks it is. But the site it picks then has to be
        # flown to using that same filter, so the answer is expressed as an
        # offset from the *estimated* position rather than as a place on the
        # Moon. Register it any other way and the vehicle flies its own
        # navigation error straight into the crater it just avoided: the site
        # would be chosen on truth and approached on an estimate, and the two
        # differ by the several hundred metres nothing on board can measure.
        target, score, score0, moved =
            redesignate(cfg.hazard, cfg.surface, leg.r, leg.v, cfg.t0 + leg.t,
                        hhat, lead)
        if nav !== nothing
            eph = cfg.surface.eph
            u_true = vunit(moonfixed(leg.r, cfg.t0 + leg.t, eph))
            u_nav = vunit(moonfixed(nav.r, cfg.t0 + leg.t, eph))
            target = vunit(vadd(target, vsub(u_nav, u_true)))
        end
        verbose && @info "redesignation" lead score_deg = rad2deg_(score) nominal_deg = rad2deg_(score0) moved
    end

    n_before = length(L.t)
    term = terminal_descent(l, leg.r, leg.v, leg.m; m_dry = m_dry, log = L,
                            t0 = leg.t, r_ref = r0, cfg = cfg, nav = nav,
                            target = target,
                            arrival = cfg.hazard === nothing ? 30.0 : cfg.hazard.arrival,
                            # chasing a designated point is a position loop, and
                            # it needs a tighter inner constant than the pure
                            # drift-nulling one does or it never settles
                            tau_h = target === nothing || cfg.hazard === nothing ?
                                    18.0 : cfg.hazard.tau,
                            vh_cap = max(60.0, 1.2 * abs(vdot(leg.v, ut_gate))))
    dv_term = G0 * l.isp * log(leg.m / term.m)
    # the touchdown sample carries the attitude it landed in, not a zero: the
    # thrust elevation is what says which way up the vehicle is, and a vehicle
    # logged as thrusting horizontally at the moment of contact is a vehicle
    # lying on its side
    _log_descent!(L, leg.t + term.t, term.r, term.v, term.m, term.min_throttle,
                  length(L.pitch) > n_before ? L.pitch[end] : deg2rad_(90.0),
                  r0, hhat, cfg, nav, target)
    prop_left = term.m - m_dry
    # what the residual is actually worth: seconds of hover at touchdown mass
    hover = prop_left / (term.m * MU_MOON / vnorm(term.r)^2 / (G0 * l.isp))

    # the ground it actually arrived on
    t_td = cfg.t0 + leg.t + term.t
    slope = 0.0
    if cfg.surface !== nothing
        u_td = vunit(moonfixed(term.r, t_td, cfg.surface.eph))
        slope = terrain_slope(cfg.surface.terrain, u_td)
    end
    elev = ground_elevation(cfg.surface, term.r, t_td)
    nav_dr, _, nav_dh = nav === nothing ? (0.0, 0.0, 0.0) :
                        nav_error(nav, term.r, term.v, cfg.surface, t_td)

    # Touchdown limits are the lander's, not the trajectory's: Apollo's LM was
    # designed for 3 m/s of sink and about 1.2 m/s of lateral drift before a
    # leg digs in and the vehicle tips, and for standing on ground no steeper
    # than about 12°. Anything outside that is a crash, and calling it one is
    # the whole point of flying the last kilometre.
    outcome = if term.outcome !== :touchdown
        term.outcome
    elseif term.v_vertical > 3.0 || abs(term.v_horizontal) > 1.5
        :crash
    elseif slope > deg2rad_(12.0)
        :tipped
    else
        :touchdown
    end
    DescentResult(L, outcome, leg.t + term.t, leg.t, term.v_vertical, term.v_horizontal,
                  L.downrange[end], dv_brake, dv_term, m0 - term.m, prop_left,
                  hover, term.min_throttle, p0, pr, term.r, term.v, term.m,
                  slope, elev, score, score0, moved, nav_dr, nav_dh,
                  nav !== nothing && nav.locked_h)
end


# ------------------------------------------------------- mission assembly --

"""
    moonlanding(; lander, h_park, h_moon_park, h_pdi, n_rev, inclination,
                lv, hp_return, terrain, field, nav, hazard, survey_error,
                optimize_kick, verbose) -> LandingResult

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

`terrain`, `field`, `nav` and `hazard` decide how much of the real Moon the
descent has to cope with; all four default to off, and with them off this is
the descent onto a smooth sphere that the rest of the repo was built against.
Switch on `terrain` and the ground moves by kilometres; switch on `field` and
the parking orbit stops being the ellipse the burn put it in; switch on `nav`
and the vehicle stops knowing where it is. [`apollo_landing`](@ref) turns on
all four at once.

`survey_error` [m] is how well the landing site's elevation was measured from
orbit before the descent. The predicted site is found by flying the descent
once over a smooth sphere, the true ground elevation there is read off with
this much error added, and *that* is what the navigation filter is told. It is
the difference between a lander that knows roughly how high its site sits and
one that assumes the mean sphere, which is the difference between a landing
and a crater. Pass `NaN` to survey nothing.
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
                     terrain::Union{Nothing,LunarTerrain} = nothing,
                     field::Union{Nothing,LunarGravity} = nothing,
                     nav::Union{Nothing,DescentNav} = nothing,
                     hazard::Union{Nothing,HazardScan} = nothing,
                     survey_error::Float64 = 60.0,
                     orbiter::Union{Nothing,Orbiter} = nothing,
                     kick_angle::Float64 = deg2rad_(8.0),
                     optimize_kick::Bool = false,
                     cis_eta::Float64 = SatelliteSim.CIS_ETA,
                     # The free return is an *abort* path here, not an entry
                     # corridor: nobody flies it unless the insertion burn
                     # fails, so it is designed to kilometres rather than to
                     # the 250 m the flyby mission needs.
                     perigee_tol::Float64 = 5.0e3,
                     theta_g0::Float64 = 0.0,
                     site_lat::Float64 = deg2rad_(28.5),
                     site_lon::Float64 = deg2rad_(-80.6),
                     target_lat::Float64 = NaN,
                     target_lon::Float64 = NaN,
                     verbose::Bool = false)
    m_payload = lander_mass(lander) +
                (orbiter === nothing ? 0.0 : orbiter_mass(orbiter))
    lv === nothing && (lv = default_moon_rocket(payload = m_payload))
    abs(lv.payload_mass - m_payload) < 1.0 ||
        error("launch vehicle payload ($(round(lv.payload_mass)) kg) is not the " *
              "mass being sent to the Moon ($(round(m_payload)) kg: a " *
              "$(round(lander_mass(lander))) kg lander" *
              (orbiter === nothing ? "" :
               " and a $(round(orbiter_mass(orbiter))) kg orbiter") * ")")

    des = translunar_design(lv; h_park = h_park, hp_moon = h_moon_park,
                            hp_return = hp_return, inclination = inclination,
                            kick_angle = kick_angle, optimize_kick = optimize_kick,
                            cis_eta = cis_eta, perigee_tol = perigee_tol,
                            theta_g0 = theta_g0, site_lat = site_lat,
                            site_lon = site_lon,
                            tol_perigee_km = 30.0, verbose = verbose)
    asc, eph = des.ascent, des.eph

    cis = fly_to_perilune(asc.r, asc.v, asc.t, eph; t_ign = des.t_ign,
                          dv = des.dv, stage = des.kick, m_stack = des.m_stack,
                          prop_avail = asc.prop_left[end], eta = cis_eta,
                          theta_g0 = theta_g0)
    cis.outcome == :perilune ||
        error("trans-lunar leg did not reach perilune (outcome: $(cis.outcome))")

    # --- insertion ---------------------------------------------------------
    t_loi = cis.t
    r_m, v_m = mci_state(cis.r, cis.v, t_loi, eph)
    T_park = 2pi * sqrt(vnorm(r_m)^3 / MU_MOON)
    aiming = isfinite(target_lat) && isfinite(target_lon)
    dv_loi0, v_after0 = loi_burn(r_m, v_m)

    # A chosen site is reached by inserting into the parking orbit that passes
    # over it — one combined LOI + plane-change burn, then a wait of at most an
    # extra revolution before DOI. But the powered descent does not land where
    # it is ignited: it brakes for six minutes and comes down several hundred
    # kilometres downrange. So the PDI aim is walked UP-range of the site by the
    # descent's own measured downrange, re-aiming against a nominal descent until
    # the miss is gone. With no site the burn is the plain retrograde
    # circularisation and the wait is the requested whole revolutions.
    tgt = nothing
    if aiming
        u_site = (cos(target_lat) * cos(target_lon),
                  cos(target_lat) * sin(target_lon), sin(target_lat))
        u_aim = u_site
        for _ in 1:5
            T = target_parking(eph, t_loi, r_m, v_m, h_moon_park, h_pdi,
                               u_aim, n_rev)
            mi = _burn_mass(lander, lander_mass(lander), T.dv)
            flyi = Lander(lander.name, lander.mdry, mi - lander.mdry,
                          lander.thrust, lander.isp, lander.throttle_min,
                          lander.diameter)
            orb = descent_orbit(eph, t_loi, r_m, T.v_park, T.wait, h_pdi, field)
            dry = powered_descent(flyi, orb.r_pdi, orb.v_pdi, mi;
                                  h_gate = h_gate,
                                  cfg = DescentConfig(eph = eph, t0 = orb.t_pdi))
            tgt = T
            dry.outcome == :touchdown || break
            u_land = vunit(moonfixed(dry.r, orb.t_pdi + dry.t_touchdown, eph))
            hf = vunit(vcross(moonfixed(orb.r_pdi, orb.t_pdi, eph),
                              moonfixed(orb.v_pdi, orb.t_pdi, eph)))
            err = atan(vdot(vcross(u_land, u_site), hf), vdot(u_land, u_site))
            u_aim = rot_about(u_aim, hf, err)
            abs(err) < 2e-5 && break
        end
    end
    dv_loi = tgt === nothing ? dv_loi0 : tgt.dv
    v_after = tgt === nothing ? v_after0 : tgt.v_park
    m = _burn_mass(lander, lander_mass(lander), dv_loi)
    m <= lander.mdry &&
        error("lunar-orbit insertion alone empties the lander " *
              "($(round(dv_loi)) m/s needed, $(round(lander_dv(lander))) m/s carried)")
    # the orbiter arrives on the same trajectory and pays the same delta-v out
    # of its own tanks, then stays where the burn put it
    m_orb = 0.0
    if orbiter !== nothing
        m_orb = orbiter_mass(orbiter) * exp(-dv_loi / (G0 * orbiter.isp))
        m_orb <= orbiter.mdry &&
            error("lunar-orbit insertion alone empties the orbiter " *
                  "($(round(dv_loi)) m/s needed, $(round(orbiter_dv(orbiter))) m/s carried)")
    end

    # --- parking orbit, DOI, coast to the descent periapsis ----------------
    OL = LunarOrbitLog()
    r_park, v_park = r_m, v_after
    wait = tgt === nothing ? n_rev * T_park : tgt.wait
    r_park, v_park = coast_moon!(OL, r_park, v_park, t_loi, wait;
                                 phase = 0, dt = 5.0, log_every = 8,
                                 field = field, eph = eph)
    t_doi = t_loi + wait
    dv_doi, v_doi = doi_burn(r_park, v_park, h_pdi)
    m = _burn_mass(lander, m, dv_doi)
    a_desc = 0.5 * (vnorm(r_park) + R_MOON + h_pdi)
    t_transfer = pi * sqrt(a_desc^3 / MU_MOON)
    r_pdi, v_pdi = coast_moon!(OL, r_park, v_doi, t_doi, t_transfer;
                               phase = 1, dt = 2.0, log_every = 8,
                               field = field, eph = eph)
    t_pdi = t_doi + t_transfer

    # --- powered descent ---------------------------------------------------
    # the lander's remaining propellant is what it flies the descent on
    flying = Lander(lander.name, lander.mdry, m - lander.mdry, lander.thrust,
                    lander.isp, lander.throttle_min, lander.diameter)
    surf = terrain === nothing ? nothing : SurfaceModel(terrain, eph)

    # Survey the site the way a real mission does: fly the descent once over a
    # smooth sphere to find out where it is going to end up, then look up what
    # the ground there actually does. That one elevation is then the reference
    # for everything — the altitude the braking phase is aimed at, and the
    # radius the navigation filter measures its altitude against. A mission
    # that skips this step designs its descent to the mean sphere, and the
    # mean sphere is nowhere in particular.
    h_ref = 0.0
    if surf !== nothing && !isnan(survey_error)
        dry = powered_descent(flying, r_pdi, v_pdi, m; h_gate = h_gate,
                              cfg = DescentConfig(eph = eph, t0 = t_pdi))
        u_aim = vunit(moonfixed(dry.r, t_pdi + dry.t_touchdown, eph))
        seed = nav === nothing ? 0x00537EE1 : nav.seed ⊻ 0x00537EE1
        h_ref = terrain_height(terrain, u_aim) + survey_error * _ngauss(seed, 11)
        verbose && @info "site survey" elevation = h_ref
        if nav !== nothing && isnan(nav.site_elev)
            nav = DescentNav(nav.dr_down, nav.dr_radial, nav.dr_cross,
                             nav.dv_down, nav.dv_radial, nav.dv_cross,
                             h_ref, nav.radar, nav.seed)
        end
    end

    cfg = DescentConfig(surface = surf, field = field, nav = nav,
                        hazard = hazard, eph = eph, t0 = t_pdi)
    desc = powered_descent(flying, r_pdi, v_pdi, m; h_gate = h_gate,
                           h_ref = h_ref, cfg = cfg, verbose = verbose)
    t_td = t_pdi + desc.t_touchdown
    lat, lon = selenographic(desc.r, t_td, eph)

    LandingResult(lv, lander, des.guid, asc, eph, cis, OL, desc,
                  dv_loi, dv_doi, t_loi, t_doi, t_pdi, t_td, h_moon_park, h_pdi,
                  n_rev, lat, lon, desc.prop_left,
                  orbiter, m_orb, r_m, v_after)
end

"""
    apollo_landing(; terrain, kwargs...) -> LandingResult

The landing mission with everything switched on: procedural terrain under the
vehicle, an oblate and mascon-lumped Moon around it, orbit-determination error
and landing radar in place of perfect knowledge, and hazard avoidance choosing
the touchdown point at high gate. Any keyword `moonlanding` takes still
applies.

This is the configuration worth quoting. The one with everything off lands
every time, which tells you about the guidance law and nothing about the
Moon.
"""
apollo_landing(; terrain::LunarTerrain = LunarTerrain(), kwargs...) =
    moonlanding(; terrain = terrain, field = LunarGravity(), nav = DescentNav(),
                hazard = HazardScan(), kwargs...)

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
    if d.elev != 0.0 || d.slope != 0.0
        @printf(io, "  Ground          : %+.0f m off the mean sphere, %.1f° slope, %s\n",
                d.elev, rad2deg_(d.slope),
                isnan(d.site_score) ? "site not redesignated" :
                @sprintf("site %.1f° (was %.1f°), moved %.0f m",
                         rad2deg_(d.site_score), rad2deg_(d.site_score_nominal),
                         d.redesignated))
    end
    if d.nav_err != 0.0
        @printf(io, "  Navigation      : %.0f m position error at touchdown, %+.0f m in altitude, radar %s\n",
                d.nav_err, d.nav_dh, d.radar_locked ? "acquired" : "NEVER ACQUIRED")
    end
    if d.outcome === :touchdown
        @printf(io, "  TOUCHDOWN       : %.2f m/s down, %.2f m/s lateral at %.2f°%s, %.2f°%s (%s side)\n",
                d.v_vertical, d.v_horizontal, abs(rad2deg_(ls.lat_land)),
                ls.lat_land >= 0 ? "N" : "S", abs(rad2deg_(ls.lon_land)),
                ls.lon_land >= 0 ? "E" : "W",
                abs(ls.lon_land) < pi/2 ? "near" : "far")
    elseif d.outcome === :tipped
        @printf(io, "  TIPPED OVER     : came to rest on %.1f° ground (limit 12°)\n",
                rad2deg_(d.slope))
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
