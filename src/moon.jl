# Lunar ephemeris.
#
# Mid-fidelity model: the Moon moves on a circle of radius `a` at the sidereal
# mean motion, in a fixed plane spanned by the orthonormal in-plane basis
# (p, q). This is the right level for mission design of a coplanar
# circumlunar free return: eccentricity (±21,000 km) and plane evolution are
# dispersions to layer on later (swap in a real ephemeris via the same
# interface — any callable `t -> V3` works with `ThirdBodyGravity`).

"""
    CircularMoonEphemeris(p, q, a, n, phase0)

Moon position: `s(t) = a (p cos(phase0 + n t) + q sin(phase0 + n t))`.
Construct from a parking-orbit state with [`coplanar_moon`](@ref) to model an
in-plane trans-lunar injection window.
"""
struct CircularMoonEphemeris
    p::V3
    q::V3
    a::Float64
    n::Float64
    phase0::Float64
end

"Moon ECI position [m] at time `t` [s]."
@inline function moon_position(e::CircularMoonEphemeris, t::Float64)
    th = e.phase0 + e.n * t
    c, s = cos(th), sin(th)
    (e.a * (c * e.p[1] + s * e.q[1]),
     e.a * (c * e.p[2] + s * e.q[2]),
     e.a * (c * e.p[3] + s * e.q[3]))
end

"Moon ECI velocity [m/s] at time `t` [s]."
@inline function moon_velocity(e::CircularMoonEphemeris, t::Float64)
    th = e.phase0 + e.n * t
    c, s = cos(th), sin(th)
    an = e.a * e.n
    (an * (-s * e.p[1] + c * e.q[1]),
     an * (-s * e.p[2] + c * e.q[2]),
     an * (-s * e.p[3] + c * e.q[3]))
end

# Make the ephemeris itself callable so it plugs into ThirdBodyGravity.
(e::CircularMoonEphemeris)(t::Float64) = moon_position(e, t)

"""
    coplanar_moon(r, v; phase0 = 0.0, a = A_MOON) -> CircularMoonEphemeris

Ephemeris for a Moon orbiting in the plane of the (r, v) orbit, prograde with
it: `p` along `r`, `q` completing the in-plane right-handed basis. `phase0`
is the Moon's angle ahead of `r` at t = 0. This encodes the coplanar-TLI
window assumption: real missions launch when the parking-orbit plane contains
the Moon at arrival; here the plane match is exact by construction.
"""
function coplanar_moon(r::V3, v::V3; phase0::Float64 = 0.0, a::Float64 = A_MOON)
    p = vunit(r)
    h = vcross(r, v)
    q = vunit(vcross(h, r))          # in-plane, along-track
    ph = phase0
    c, s = cos(ph), sin(ph)
    pp = vadd(vscale(p, c), vscale(q, s))
    qq = vadd(vscale(p, -s), vscale(q, c))
    CircularMoonEphemeris(pp, qq, a, N_MOON, 0.0)
end

"Distance from ECI position `r` to the Moon's center at time `t` [m]."
@inline moon_distance(e::CircularMoonEphemeris, r::V3, t::Float64) =
    vnorm(vsub(moon_position(e, t), r))

"Altitude above the mean lunar surface [m]."
@inline moon_altitude(e::CircularMoonEphemeris, r::V3, t::Float64) =
    moon_distance(e, r, t) - R_MOON
