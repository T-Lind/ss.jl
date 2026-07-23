# Mission scenarios.
#
# The reference mission: a small unpropelled reentry pod that has already
# performed its deorbit burn. The simulation starts at the apoapsis (~400 km,
# ex-LEO) of the resulting deorbit ellipse whose vacuum perigee (~25 km) is
# deep enough in the atmosphere to guarantee capture. Above the 120 km entry
# interface the motion is purely orbital; below it aerodynamics take over.
#
# Splashdown target: Pacific Ocean off the US west coast.

const WEST_COAST_TARGET_LAT = deg2rad_(32.5)     # ~ 300 km offshore of San Diego
const WEST_COAST_TARGET_LON = deg2rad_(-121.5)

Base.@kwdef struct DeorbitElements
    apoapsis_alt::Float64 = 400.0e3   # [m] above mean radius
    periapsis_alt::Float64 = 25.0e3   # vacuum perigee [m] — sets entry FPA
    inclination::Float64 = deg2rad_(51.6)
    raan::Float64 = 0.0               # tuned by targeting
    argp::Float64 = 0.0               # tuned by targeting
    nu0::Float64 = pi                 # start at apoapsis (half rev before entry)
end

function scenario_from_elements(el::DeorbitElements, veh::Vehicle;
                                target_lat = WEST_COAST_TARGET_LAT,
                                target_lon = WEST_COAST_TARGET_LON,
                                atmosphere = USSA76(),
                                kwargs...)
    ra = RE_MEAN + el.apoapsis_alt
    rp = RE_MEAN + el.periapsis_alt
    a = 0.5 * (ra + rp)
    e = (ra - rp) / (ra + rp)
    r0, v0 = state_from_elements(a, e, el.inclination, el.raan, el.argp, el.nu0)
    Scenario(; vehicle = veh, atmosphere = atmosphere, r0 = r0, v0 = v0,
             target_lat = target_lat, target_lon = target_lon, kwargs...)
end

"""
    target_deorbit(el, veh; max_iter=8, tol_deg=0.1, verbose=false)
        -> (el_tuned, result)

Solve for the RAAN and argument of perigee that put splashdown on the
scenario target, by fixed-point iteration on the spherical-geometry
relations (argument of latitude -> latitude, RAAN -> longitude). Converges
in a handful of full trajectory simulations; each correction uses the
actually-simulated splashdown point, so all aero/rotation coupling is
absorbed. This is also the natural seed for a future closed-loop guidance
extension.
"""
function target_deorbit(el::DeorbitElements, veh::Vehicle;
                        max_iter::Int = 8, tol_deg::Float64 = 0.1, verbose::Bool = false,
                        kwargs...)
    raan, argp = el.raan, el.argp
    local res
    for it in 1:max_iter
        eli = DeorbitElements(el.apoapsis_alt, el.periapsis_alt, el.inclination,
                              raan, argp, el.nu0)
        scn = scenario_from_elements(eli, veh; kwargs...)
        res = simulate(scn)
        res.terminated == :splashdown ||
            error("targeting run did not reach splashdown (terminated: $(res.terminated))")

        dlat = scn.target_lat - res.lat_splash
        dlon = rem(scn.target_lon - res.lon_splash, 2pi, RoundNearest)
        verbose && @info "targeting" it miss_km = res.miss_km dlat_deg = rad2deg_(dlat) dlon_deg = rad2deg_(dlon)
        if abs(dlat) < deg2rad_(tol_deg) && abs(dlon) < deg2rad_(tol_deg)
            return (DeorbitElements(el.apoapsis_alt, el.periapsis_alt, el.inclination,
                                    raan, argp, el.nu0), res)
        end

        # latitude via argument of latitude on the (ascending) final arc:
        # sin(lat) = sin(i) sin(u), so du = dlat / (sin i * cos u / cos lat) —
        # use the simple small-correction form du ≈ dlat / cos(asin(...)) ≈ dlat.
        sini = sin(el.inclination)
        u_now = asin(clamp(sin(res.lat_splash) / sini, -1.0, 1.0))
        u_tgt = asin(clamp(sin(scn.target_lat) / sini, -1.0, 1.0))
        argp += u_tgt - u_now
        raan += dlon
    end
    @warn "targeting did not converge to $(tol_deg) deg; using last iterate"
    (DeorbitElements(el.apoapsis_alt, el.periapsis_alt, el.inclination,
                     raan, argp, el.nu0), res)
end

"""
    west_coast_scenario(; mass=350.0, verbose=false) -> (scn, el, res)

Fully-targeted reference scenario: deorbit ellipse tuned so the nominal
vehicle splashes down at the west-coast target.
"""
function west_coast_scenario(; mass::Float64 = 350.0, verbose::Bool = false)
    veh = default_reentry_pod(mass = mass)
    el0 = DeorbitElements()
    el, res = target_deorbit(el0, veh; verbose = verbose)
    scn = scenario_from_elements(el, veh)
    (scn, el, res)
end
