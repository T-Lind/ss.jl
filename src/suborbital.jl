# Suborbital missions: a flight that never intends to reach orbit, flown as an
# ascent with a target it can actually close on, a ballistic arc, and the same
# entry the returning missions use.
#
# Two profiles, because there are two things people mean by "suborbital":
#
#   :hop        straight up and back down, closing on an APOGEE. This is a
#               sounding rocket or a New Shepard flight: no pitch-over at all,
#               the engines cut when the arc they have built reaches the
#               altitude asked for, and the capsule comes down near the pad.
#
#   :downrange  a ballistic shot, closing on where it comes DOWN. The vehicle
#               pitches over, holds a lofted attitude, and cuts off the moment
#               the free-flight arc through its own state reaches the ground
#               range asked for. This is a Mercury-Redstone or any range shot.
#
# Neither can use the orbital cutoff. That one closes on specific energy, which
# is the right quantity for an orbit and the wrong one here: the same energy
# describes an arc that lands 200 km downrange and one that lands 2000, and it
# says nothing at all about apogee. `AscentGuidance.cutoff` picks the quantity,
# and `_apogee_radius` / `_ballistic_range` in launch.jl compute it from the
# state, so the tests hold under any steering and at any stage.
#
# What flies the arc is the payload — the capsule — handed to the existing
# entry simulator at cutoff, exactly as the orbital missions hand it over after
# their deorbit burn. So the arc gets real drag, real heating and real
# parachutes, and the panel and the launch view fly it without learning
# anything new. The spent booster is not tracked past separation.
#
# The commanded target is corrected for what the atmosphere takes. A hop that
# cuts off at 40 km still has 40 km of air to climb through, and the drag in it
# costs several kilometres of apogee; a shot loses ground range the same way.
# One secant iteration on the command closes that, which is the difference
# between "asked for 100 km" and "reached 100 km".

"""
    SuborbitalResult

A flown suborbital mission: the ascent, the ballistic arc and entry as a single
`SimResult`, and what it actually achieved against what was asked.

`outcome` is `:splashdown` when the capsule came down, `:short` when the vehicle
ran out of propellant before meeting its target (the arc is still flown and
reported — a shot that falls short is a result, not an error), and
`:ascent_failed` if it never got off the ground.
"""
struct SuborbitalResult
    lv::LaunchVehicle
    guid::AscentGuidance
    ascent::AscentResult
    profile::Symbol
    target_apogee::Float64        # [m] what was asked for
    target_range::Float64         # [m]
    apogee::Float64               # [m] what was reached
    range::Float64                # [m] ground range at splashdown
    t_apogee::Float64             # [s] from liftoff
    outcome::Symbol
    entry_scn::Union{Nothing,Scenario}
    entry::Union{Nothing,SimResult}
end

"Apogee altitude [m] and the time it happens, from a flown entry log."
function _arc_apogee(res::SimResult)
    isempty(res.log.t) && return (NaN, NaN)
    i = argmax(res.log.h)
    (res.log.h[i], res.log.t[i])
end

"Great-circle ground range [m] from the launch site to the splashdown point."
function _ground_range(lat0::Float64, lon0::Float64, lat1::Float64, lon1::Float64)
    (isnan(lat1) || isnan(lon1)) && return NaN
    d = sin(lat0) * sin(lat1) + cos(lat0) * cos(lat1) * cos(lon1 - lon0)
    RE_MEAN * acos(clamp(d, -1.0, 1.0))
end

"""
Fly one suborbital attempt at a given commanded target, without correction.
Returns the ascent, the arc, and what it achieved.
"""
function _subfly(lv::LaunchVehicle, guid::AscentGuidance, pod_mass::Float64,
                 theta_g0::Float64)
    asc = simulate_ascent(lv, guid; t_max = 2.0e3)
    cut = findlast(e -> e.name === :seco, asc.events)
    pod = default_reentry_pod(mass = pod_mass)
    # Straight up, the capsule is flying backwards through its own wake at
    # apogee and there is no meaningful angle of attack to seed; the pitch
    # dynamics need something non-degenerate, so it is seeded small either way.
    scn = Scenario(vehicle = pod, r0 = asc.r, v0 = asc.v, t0 = asc.t,
                   t_max = asc.t + 6.0e3, theta_g0 = theta_g0,
                   alpha0 = deg2rad_(2.0))
    arc = simulate(scn)
    hap, tap = _arc_apogee(arc)
    rng = _ground_range(guid.site_lat, guid.site_lon, arc.lat_splash, arc.lon_splash)
    (asc = asc, scn = scn, arc = arc, apogee = hap, t_apogee = tap,
     range = rng, cut = cut !== nothing)
end

"""
    suborbital(; profile=:hop, apogee=100e3, downrange=250e3, lv, pod_mass, ...)
        -> SuborbitalResult

Design and fly a suborbital mission. `profile` is `:hop` (vertical, closing on
`apogee`) or `:downrange` (lofted ballistic, closing on `downrange`); the other
target is reported rather than commanded, because a vehicle has one degree of
freedom at cutoff and cannot be given two.

`loft` is the attitude a `:downrange` shot holds once it is out of the gravity
turn. 40° above the horizon is near the range-optimal value for a shot of a few
hundred kilometres; steeper trades range for apogee, shallower the other way.

Under `strict = false` a vehicle that cannot meet its target returns with
`outcome = :short` and everything it did fly, instead of throwing.
"""
function suborbital(; profile::Symbol = :hop,
                    lv::Union{Nothing,LaunchVehicle} = nothing,
                    pod_mass::Float64 = 350.0,
                    apogee::Float64 = 100.0e3,
                    downrange::Float64 = 250.0e3,
                    loft::Float64 = deg2rad_(40.0),
                    azimuth::Float64 = deg2rad_(90.0),
                    kick_angle::Float64 = deg2rad_(8.0),
                    theta_g0::Float64 = 0.0,
                    site_lat::Float64 = deg2rad_(28.5),
                    site_lon::Float64 = deg2rad_(-80.6),
                    iterations::Int = 3,
                    strict::Bool = true,
                    verbose::Bool = false)
    profile in (:hop, :downrange) ||
        throw(ArgumentError("unknown suborbital profile $profile; have hop, downrange"))
    apogee > 0 || throw(ArgumentError("apogee must be positive"))
    profile === :downrange && !(downrange > 0) &&
        throw(ArgumentError("downrange must be positive"))

    lv === nothing && (lv = default_moon_rocket(payload = pod_mass))
    pod_mass = lv.payload_mass

    # A hop does not pitch over at all, and holds the vertical the whole way up:
    # there is nowhere to be going but up. A shot pitches over as any launcher
    # does and then holds the lofted attitude through the upper stages.
    base = profile === :hop ?
        AscentGuidance(azimuth = azimuth, kick_angle = 0.0, kick_duration = 0.0,
                       pitch_hold = 0.5pi, cutoff = :apogee,
                       apogee_target = apogee, site_lat = site_lat,
                       site_lon = site_lon,
                       fairing_alt = min(60.0e3, 0.55 * apogee)) :
        AscentGuidance(azimuth = azimuth, kick_angle = kick_angle,
                       pitch_hold = loft, cutoff = :range,
                       range_target = downrange, site_lat = site_lat,
                       site_lon = site_lon,
                       fairing_alt = 60.0e3)

    # Secant correction on the COMMAND. The cutoff test is a vacuum arc through
    # the current state, and the flight after it is not in a vacuum: a hop that
    # cuts at 40 km still has 40 km of air to climb, and that costs kilometres
    # of apogee. Commanding the miss back in is one line and it is the whole
    # difference between asking for 100 km and reaching it.
    goal = profile === :hop ? apogee : downrange
    got(f) = profile === :hop ? f.apogee : f.range
    cmd = goal
    best = nothing
    prev_cmd = NaN; prev_err = NaN
    for it in 1:max(iterations, 1)
        g = profile === :hop ? _reguid(base; apogee_target = cmd) :
                               _reguid(base; range_target = cmd)
        f = _subfly(lv, g, pod_mass, theta_g0)
        err = isfinite(got(f)) ? got(f) - goal : NaN
        verbose && @info "suborbital" it cmd_km = cmd/1e3 got_km = got(f)/1e3 err_km = err/1e3
        if best === nothing || (isfinite(err) &&
           (!isfinite(best.err) || abs(err) < abs(best.err)))
            best = (g = g, f = f, err = err)
        end
        # a flight that never cut off cannot be corrected: it is short, and
        # commanding it higher only makes it shorter
        (!f.cut || !isfinite(err)) && break
        abs(err) < 0.004 * goal && break
        cmd_new = if isfinite(prev_err) && abs(err - prev_err) > 1e-6
            cmd - err * (cmd - prev_cmd) / (err - prev_err)
        else
            cmd - err                       # first step: command the miss back
        end
        prev_cmd = cmd; prev_err = err
        cmd = clamp(cmd_new, 0.25 * goal, 4.0 * goal)
    end

    g, f = best.g, best.f
    outcome = !f.cut ? :short :
              isnan(f.range) ? :timeout : :splashdown
    if outcome === :short && strict
        error("the vehicle could not reach its suborbital target " *
              "(profile $profile, apogee $(f.apogee/1e3) km, range $(f.range/1e3) km)")
    end
    SuborbitalResult(lv, g, f.asc, profile, apogee,
                     profile === :hop ? NaN : downrange,
                     f.apogee, f.range, f.t_apogee, outcome, f.scn, f.arc)
end
