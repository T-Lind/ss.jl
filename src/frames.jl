# Frame conversions and geodesy (ECI <-> ECEF <-> geodetic WGS-84).
#
# ECI here is an Earth-centered inertial frame whose x-axis coincides with the
# ECEF x-axis at t = t0 when `theta_g0 = 0` (precession/nutation neglected —
# appropriate for minutes-to-hours flight times at this fidelity).

"Earth rotation angle at time t [s] given angle at t=0."
@inline earth_rotation_angle(theta_g0::Float64, t::Float64) = theta_g0 + OMEGA_EARTH * t

"""
    geodetic_from_ecef(r_ecef) -> (lat, lon, h)

WGS-84 geodetic latitude [rad], longitude [rad], altitude [m] from an ECEF
position, via Bowring's closed-form approximation (sub-millimeter for LEO).
"""
function geodetic_from_ecef(r::V3)
    x, y, z = r
    lon = atan(y, x)
    p = hypot(x, y)
    if p < 1e-9
        lat = copysign(pi / 2, z)
        h = abs(z) - RE_POL
        return (lat, lon, h)
    end
    # Bowring
    ep2 = E2_WGS84 / (1 - E2_WGS84)
    b = RE_POL
    theta = atan(z * RE_EQ, p * b)
    st, ct = sin(theta), cos(theta)
    lat = atan(z + ep2 * b * st^3, p - E2_WGS84 * RE_EQ * ct^3)
    sl = sin(lat)
    N = RE_EQ / sqrt(1 - E2_WGS84 * sl * sl)
    h = p / cos(lat) - N
    (lat, lon, h)
end

"""
    ecef_from_geodetic(lat, lon, h) -> V3
"""
function ecef_from_geodetic(lat::Float64, lon::Float64, h::Float64)
    sl, cl = sin(lat), cos(lat)
    N = RE_EQ / sqrt(1 - E2_WGS84 * sl * sl)
    ((N + h) * cl * cos(lon), (N + h) * cl * sin(lon), (N * (1 - E2_WGS84) + h) * sl)
end

"""
    enu_basis(lat, lon) -> (e_east, e_north, e_up)

Unit vectors of the local East-North-Up frame expressed in ECEF axes.
"""
function enu_basis(lat::Float64, lon::Float64)
    sl, cl = sin(lat), cos(lat)
    so, co = sin(lon), cos(lon)
    e_east  = (-so, co, 0.0)
    e_north = (-sl * co, -sl * so, cl)
    e_up    = (cl * co, cl * so, sl)
    (e_east, e_north, e_up)
end

"""
    haversine(lat1, lon1, lat2, lon2) -> distance [m]

Great-circle distance on a sphere of mean Earth radius. Inputs in radians.
"""
function haversine(lat1, lon1, lat2, lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)^2 + cos(lat1) * cos(lat2) * sin(dlon / 2)^2
    2 * RE_MEAN * asin(min(1.0, sqrt(a)))
end

"""
    elements_from_state(r, v; mu=MU_EARTH) -> NamedTuple

Osculating classical elements from an ECI Cartesian state. Returns
`(a, e, i, raan, argp, nu, rp, ra, energy)` with `rp`/`ra` the periapsis and
apoapsis radii (`ra = Inf` for `e >= 1`). Angles in radians.
"""
function elements_from_state(r::V3, v::V3; mu::Float64 = MU_EARTH)
    rn = vnorm(r); vn = vnorm(v)
    h = vcross(r, v)
    hn = vnorm(h)
    energy = 0.5 * vn * vn - mu / rn
    a = -mu / (2 * energy)
    # eccentricity vector
    ev = vsub(vscale(vcross(v, h), 1 / mu), vscale(r, 1 / rn))
    e = vnorm(ev)
    i = acos(clamp(h[3] / hn, -1.0, 1.0))
    nvec = vcross((0.0, 0.0, 1.0), h)          # node vector
    nn = vnorm(nvec)
    raan = nn > 1e-12 ? atan(nvec[2], nvec[1]) : 0.0
    argp = if nn > 1e-12 && e > 1e-12
        w = acos(clamp(vdot(nvec, ev) / (nn * e), -1.0, 1.0))
        ev[3] < 0 ? 2pi - w : w
    else
        0.0
    end
    nu = if e > 1e-12
        f = acos(clamp(vdot(ev, r) / (e * rn), -1.0, 1.0))
        vdot(r, v) < 0 ? 2pi - f : f
    else
        0.0
    end
    p = hn * hn / mu
    rp = p / (1 + e)
    ra = e < 1 ? p / (1 - e) : Inf
    (a = a, e = e, i = i, raan = raan, argp = argp, nu = nu,
     rp = rp, ra = ra, energy = energy)
end

"""
    state_from_elements(a, e, i, raan, argp, nu; mu=MU_EARTH) -> (r, v)

Classical Keplerian elements (angles in radians, `a` in meters) to ECI
Cartesian position/velocity. Standard perifocal -> ECI rotation.
"""
function state_from_elements(a::Float64, e::Float64, i::Float64,
                             raan::Float64, argp::Float64, nu::Float64;
                             mu::Float64 = MU_EARTH)
    p = a * (1 - e^2)
    r = p / (1 + e * cos(nu))
    # perifocal
    rp = (r * cos(nu), r * sin(nu), 0.0)
    vf = sqrt(mu / p)
    vp = (-vf * sin(nu), vf * (e + cos(nu)), 0.0)
    co, so = cos(raan), sin(raan)
    cw, sw = cos(argp), sin(argp)
    ci, si = cos(i), sin(i)
    # R3(-raan) R1(-i) R3(-argp)
    R11 = co * cw - so * sw * ci; R12 = -co * sw - so * cw * ci; R13 = so * si
    R21 = so * cw + co * sw * ci; R22 = -so * sw + co * cw * ci; R23 = -co * si
    R31 = sw * si;                R32 = cw * si;                 R33 = ci
    reci = (R11 * rp[1] + R12 * rp[2] + R13 * rp[3],
            R21 * rp[1] + R22 * rp[2] + R23 * rp[3],
            R31 * rp[1] + R32 * rp[2] + R33 * rp[3])
    veci = (R11 * vp[1] + R12 * vp[2] + R13 * vp[3],
            R21 * vp[1] + R22 * vp[2] + R23 * vp[3],
            R31 * vp[1] + R32 * vp[2] + R33 * vp[3])
    (reci, veci)
end
