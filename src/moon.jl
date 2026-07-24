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

# --------------------------------------------------------- gravity field ---
#
# A point-mass Moon is exact enough for a trans-lunar trajectory and visibly
# wrong for a low lunar orbit. The Moon is the lumpiest body we have close
# measurements of: the maria sit over concentrated mass excesses — mascons —
# that pull a 100 km orbit around by kilometres per revolution. That is not a
# refinement, it is the reason Lunar Orbiter's periapsis wandered by tens of
# kilometres, the reason Apollo's descent orbits had to be re-determined from
# tracking on the revolution before the burn, and the reason a descent that
# propagates its own state instead of navigating arrives somewhere else.
#
# Two terms are modelled: the oblateness (J2), and a handful of point mass
# anomalies standing in for the mascons. Each anomaly is a *pair* — the excess
# `dmu` placed under the surface and the same `dmu` removed from the centre —
# so the monopole cancels and the total GM stays `MU_MOON` no matter how many
# are added. What is left is a genuine anomaly field that decays with distance
# the way a real one does, rather than a Moon that quietly gains mass.

"""
    Mascon(name, lat, lon, depth, dmu)

A concentrated mass anomaly, positioned in the tidally-locked Moon-fixed frame
(`lon` from the sub-Earth meridian) at `depth` metres below the mean surface,
carrying `dmu` [m^3/s^2] of gravitational parameter in excess of the smooth
Moon underneath it.
"""
struct Mascon
    name::Symbol
    lat::Float64
    lon::Float64
    depth::Float64
    dmu::Float64
end

"""
    default_mascons() -> Vector{Mascon}

Five near-side mass concentrations under the great maria, at roughly their
real selenographic positions and roughly the right strength: about ten parts
per million of the Moon's mass each, which gives the few-hundred-milligal
free-air anomaly the maria actually show and the kilometre-per-revolution
periapsis walk a 100 km orbit actually suffers.

They are buried deep — 140 km — for a reason that is not geology. A real
mascon is a slab a couple of hundred kilometres across, and a point mass only
resembles a slab from further away than the slab is wide. Putting the point
at the depth that matches the *observed ratio* between the surface anomaly and
the anomaly at orbital altitude is the cheapest way to get a field that is
right where a spacecraft flies, which is the only place this model is used.

The positions are real; the magnitudes are round numbers chosen to give the
right effect, not values fitted to a gravity model.
"""
default_mascons() = [
    Mascon(:imbrium,    deg2rad_( 33.0), deg2rad_(-16.0), 140.0e3, 4.9e7),
    Mascon(:serenitatis,deg2rad_( 28.0), deg2rad_( 18.0), 140.0e3, 4.2e7),
    Mascon(:crisium,    deg2rad_( 17.0), deg2rad_( 59.0), 140.0e3, 3.4e7),
    Mascon(:nectaris,   deg2rad_(-15.0), deg2rad_( 34.0), 140.0e3, 2.3e7),
    Mascon(:humorum,    deg2rad_(-24.0), deg2rad_(-39.0), 140.0e3, 2.1e7),
]

"""
    LunarGravity(; j2, mascons)

Non-spherical lunar gravity. `j2 = 2.0323e-4` is the Moon's oblateness — a
fifth of the Earth's, because the Moon spins 27 times slower. `mascons` are
the mass anomalies; pass an empty vector for an oblate-but-smooth Moon.

Pass `nothing` instead of one of these, anywhere a field is accepted, and the
Moon is a point mass — which is what every result produced before this
existed was flown against.
"""
Base.@kwdef struct LunarGravity
    j2::Float64 = 2.0323e-4
    mascons::Vector{Mascon} = default_mascons()
end

"Moon-fixed unit direction of a mascon."
@inline _mascon_dir(m::Mascon) =
    (cos(m.lat) * cos(m.lon), cos(m.lat) * sin(m.lon), sin(m.lat))

"""
    lunar_gravity(r, t, field, eph) -> V3

Acceleration [m/s^2] at a Moon-centred inertial position. `field === nothing`
gives the point mass alone; otherwise the J2 and mascon anomalies are added on
top of it, evaluated in the Moon-fixed frame at time `t` — which is what makes
them rotate under the orbit at the Moon's own rate, once every 27 days, the
way the real lumps do.
"""
@inline function lunar_gravity(r::V3, ::Nothing, ::Any, ::Any)
    rn = vnorm(r)
    vscale(r, -MU_MOON / (rn * rn * rn))
end

function lunar_gravity(r::V3, field::LunarGravity, t::Float64,
                       eph::CircularMoonEphemeris)
    rn = vnorm(r)
    a = vscale(r, -MU_MOON / (rn * rn * rn))
    xh, yh, zh = moonfixed_basis(eph, t)

    if field.j2 != 0.0
        z = vdot(r, zh)
        k = -1.5 * field.j2 * MU_MOON * R_MOON^2 / rn^5
        zr2 = (z / rn)^2
        a = vadd(a, vscale(vadd(vscale(r, 1.0 - 5.0 * zr2), vscale(zh, 2.0 * z)), k))
    end

    for m in field.mascons
        d = _mascon_dir(m)
        # Moon-fixed direction into inertial components
        p = vscale(vadd(vadd(vscale(xh, d[1]), vscale(yh, d[2])), vscale(zh, d[3])),
                   R_MOON - m.depth)
        s = vsub(r, p)
        sn = vnorm(s)
        sn < 1.0 && continue
        # the anomaly proper: mass added at p, the same mass removed from the
        # centre, so nothing changes at long range
        a = vadd(a, vscale(s, -m.dmu / (sn * sn * sn)))
        a = vadd(a, vscale(r,  m.dmu / (rn * rn * rn)))
    end
    a
end

"""
    gravity_anomaly(field, lat, lon, h, eph, t) -> Float64

How much *stronger* gravity is at altitude `h` over a Moon-fixed point than a
smooth Moon of the same mass would make it — the free-air anomaly, positive
over a mass excess. Multiply by 1e5 for milligal, which is the unit gravity
maps are drawn in: a few hundred mGal at the surface and something under a
hundred at orbital altitude is what the great maria actually show.
"""
function gravity_anomaly(field::LunarGravity, lat::Float64, lon::Float64,
                         h::Float64, eph::CircularMoonEphemeris, t::Float64)
    d = (cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
    r = vscale(moonfixed_inv(d, t, eph), R_MOON + h)
    u = vunit(r)
    # gravity points inward, so a *stronger* field is a more negative radial
    # component; the anomaly is conventionally the increase in strength
    -vdot(vsub(lunar_gravity(r, field, t, eph), lunar_gravity(r, nothing, t, eph)), u)
end
