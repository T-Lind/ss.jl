# Lunar terrain: a deterministic, unbounded, analytic surface.
#
# Everything above this file treated the Moon as a sphere of radius `R_MOON`.
# That is a defensible approximation for trajectory design and an indefensible
# one for a landing, because the two numbers a descent actually lives or dies
# on — how far the ground is, and how flat it is where you are about to put
# the vehicle — are exactly the two the sphere throws away. Lunar relief is
# ±8 km against a 1737 km radius; a lander that trusts the sphere near a mare
# rim believes it is a kilometre higher than it is, and arrives at the surface
# at whatever speed the profile it was flying happened to command a kilometre
# up. That failure is modelled here, and so is the sensor that fixes it
# (`landingnav.jl`).
#
# The surface is *procedural*: there is no height map, no data file, and no
# resolution limit. `terrain_height` is a pure function of direction, built
# from an integer hash, so the same query returns the same metre anywhere on
# the Moon, at any zoom, in Julia and in the browser's renderer alike. The
# hash is 32-bit precisely so that `Math.imul` reproduces it exactly: every
# crater lands in the same place in both, and the two heights then agree to
# about a nanometre, the last bit of disagreement being what `sin` does
# differently in two libms. Sampling a 5 m footpad separation costs the same
# as sampling a 500 km region, which is what lets the guidance evaluate a
# landing site at the resolution that matters while the viewer draws exactly
# the ground that was flown over.
#
# Three layers, largest first:
#
#   1. **Relief** — a handful of smooth harmonics standing in for the basins
#      and highlands, ±3 km over thousands of kilometres. This is the layer
#      that fools a lander flying on a mean sphere.
#   2. **Craters** — a size-class cascade from 40 km down to ~50 m, each class
#      scattered on its own lattice, with bowl, rim and ejecta. Real crater
#      counts follow a power law in diameter, so each class is a factor of 2.6
#      smaller and roughly as numerous per unit area as the last is per its
#      own cell: what you get is the characteristic "craters all the way
#      down" surface rather than a few big holes on a smooth plain.
#   3. **Roughness** — two octaves of value noise at 40 m and 15 m, which is
#      the scale that decides whether a footpad sits flat or on a rock.
#
# What it is not: real. This is the *statistics* of a lunar surface, not the
# Moon's actual topography — Mare Tranquillitatis is not at longitude 23° E
# here, and no crater in it is Little West. Use it to price a descent, to
# exercise a landing radar, and to give hazard avoidance something to avoid.

# ------------------------------------------------------------------- hash --
#
# 32-bit throughout, and deliberately so: JavaScript has no 64-bit integers
# but `Math.imul` is an exact 32-bit multiply, so this mixer is reproducible
# in the browser to the bit. Change a constant here and `launch_page.html`
# draws a different Moon than the one the simulation flew over.

@inline function _hash32(a::UInt32, b::UInt32, c::UInt32, d::UInt32)
    h = a * 0x9E3779B1
    h = (h ⊻ b) * 0x85EBCA77
    h = (h ⊻ c) * 0xC2B2AE3D
    h = (h ⊻ d) * 0x27D4EB2F
    h ⊻= h >> 15
    h *= 0x2545F491
    h ⊻= h >> 13
    h *= 0x9E3779B1
    h ⊻= h >> 16
    h
end

"Hash of an integer lattice cell, as a `UInt32`."
@inline _cell_hash(i::Int, j::Int, k::Int, salt::UInt32) =
    _hash32(unsafe_trunc(UInt32, i), unsafe_trunc(UInt32, j),
            unsafe_trunc(UInt32, k), salt)

"Uniform [0, 1) draw number `n` from a cell hash (n = 0, 1, 2, ...)."
@inline _cell_rand(h::UInt32, n::Int) =
    Float64(_hash32(h, 0x000000FF, unsafe_trunc(UInt32, n), 0x165667B1)) / 4.294967296e9

# ------------------------------------------------------------ cube-sphere --
#
# Craters are scattered on a lattice, and a lattice needs a chart. Latitude
# and longitude is the obvious one and the wrong one: cells collapse at the
# poles, so crater density would climb to infinity there. The cube-sphere —
# six square faces projected onto the sphere — distorts by at most about 30%
# at the face corners and has no singularity anywhere.
#
# Faces are numbered 0..5 as +x, -x, +y, -y, +z, -z. `_face_axes` gives, for
# each face, the axis it is normal to and the two axes that span it.

"Dominant face (0-based) of a unit direction."
@inline function _dominant_face(u::V3)
    ax, ay, az = abs(u[1]), abs(u[2]), abs(u[3])
    if ax >= ay && ax >= az
        u[1] >= 0 ? 0 : 1
    elseif ay >= az
        u[2] >= 0 ? 2 : 3
    else
        u[3] >= 0 ? 4 : 5
    end
end

"""
    _face_project(u, f) -> (ok, s, t)

Project a direction onto face `f`'s square chart. The chart is meaningful
only where the direction is on the face's side of the cube and not edge-on to
it; `(s, t)` then run over [-1, 1] across the face itself. `ok` is false
otherwise.
"""
@inline function _face_project(u::V3, f::Int)
    sg = iseven(f) ? 1.0 : -1.0
    if f < 2
        w = sg * u[1]
        w <= 0.35 && return (false, 0.0, 0.0)    # ~70° off the face normal
        (true, sg * u[2] / w, u[3] / w)
    elseif f < 4
        w = sg * u[2]
        w <= 0.35 && return (false, 0.0, 0.0)
        (true, sg * u[3] / w, u[1] / w)
    else
        w = sg * u[3]
        w <= 0.35 && return (false, 0.0, 0.0)
        (true, sg * u[1] / w, u[2] / w)
    end
end

"Unit direction of a point `(s, t)` on face `f`'s chart."
@inline function _face_unit(f::Int, s::Float64, t::Float64)
    sg = iseven(f) ? 1.0 : -1.0
    if f < 2
        vunit((sg, sg * s, t))
    elseif f < 4
        vunit((t, sg, sg * s))
    else
        vunit((sg * s, t, sg))
    end
end

# --------------------------------------------------------------- terrain ---

"""
    LunarTerrain(; seed, relief, d_max, classes, ratio, density, rough)

A procedural lunar surface. Every field is a knob on the *statistics* of the
ground, not on any particular piece of it:

  * `seed` — changes the Moon. Same seed, same surface, forever.
  * `relief` [m] — amplitude of the long-wavelength basin-and-highland layer,
    the one that decides whether a lander flying on a mean sphere is high or
    low over its site. At the default the ground stands a typical 700 m and
    an occasional 3 km off the mean sphere, against the real Moon's ±8 km.
  * `d_max` [m], `classes`, `ratio` — the crater cascade: `classes` size
    classes from `d_max` downward, each `ratio` times smaller than the last.
    The default eight classes span 40 km down to about 50 m.
  * `density` — expected craters per lattice cell per class, so how saturated
    the surface is. Each class then covers roughly `0.63 * density` of the
    ground, so at the default 0.18 about a third of the surface is inside no
    crater at all and a third of randomly chosen sites are flat enough to
    land on. Push it past 0.35 and craters overlap heavily, which is what an
    old highland looks like and why nobody lands on one.
  * `rough` [m] — amplitude of the metre-scale layer that sets footpad-scale
    slope.

`LunarTerrain()` is a mare-like plain: modest relief, moderate crater density.
[`highland_terrain`](@ref) is the same surface saturated and rougher.
"""
Base.@kwdef struct LunarTerrain
    seed::UInt32 = 0x00C0FFEE
    relief::Float64 = 1800.0
    d_max::Float64 = 40.0e3
    classes::Int = 8
    ratio::Float64 = 2.6
    density::Float64 = 0.18
    rough::Float64 = 0.35
end

"""
    highland_terrain(; seed)

Saturated, rough, high-relief ground: crater on crater, twice the small-scale
roughness of the default plain. A lander that can put itself down here can put
itself down anywhere, which is exactly why nobody sent Apollo 11 to one.
"""
highland_terrain(; seed::UInt32 = 0x00BADBED) =
    LunarTerrain(seed = seed, relief = 3000.0, d_max = 70.0e3, classes = 9,
                 ratio = 2.4, density = 0.35, rough = 1.2)

"""
    mare_terrain(; seed)

A flooded basin floor: shallow relief, sparse and small craters, smooth at
footpad scale. This is a Sea of Tranquillity, and the reason the first
landing went to one.
"""
mare_terrain(; seed::UInt32 = 0x000A11CE) =
    LunarTerrain(seed = seed, relief = 700.0, d_max = 18.0e3, classes = 7,
                 ratio = 2.6, density = 0.10, rough = 0.18)

# ---- layer 1: long-wavelength relief ---------------------------------------

"""
Smooth basin-and-highland field: six harmonics of the direction cosines, phase
offsets drawn from the seed. Wavelengths run from a hemisphere down to about
600 km, so this layer is featureless at descent scale and dominant at
navigation scale — which is the whole reason it is here.
"""
@inline function _relief(tr::LunarTerrain, u::V3)
    p1 = _cell_rand(tr.seed, 1) * 6.2831853
    p2 = _cell_rand(tr.seed, 2) * 6.2831853
    p3 = _cell_rand(tr.seed, 3) * 6.2831853
    x, y, z = u
    s = 0.55 * sin(1.7x + 2.1y + p1) * cos(1.3z - 0.9y + p2)
    s += 0.28 * sin(3.9y - 2.7z + p2) * cos(3.1x + p3)
    s += 0.17 * sin(7.3z + 5.1x + p3) * sin(6.1y + p1)
    tr.relief * s
end

# ---- layer 2: craters ------------------------------------------------------

"""
Radial profile of a crater as a fraction of its depth, at normalised radius
`s = distance / crater radius`: a bowl rising from the floor to a rim 15%
of the depth above the datum, then an ejecta blanket falling back to zero by
2.2 radii. Both halves are smoothsteps, which puts the steepest ground
halfway up the inner wall and levels the crest — the shape a fresh crater
relaxes into, and the reason its walls come out near the angle of repose
rather than as a knife edge.

Central peaks, terraces and degradation are not modelled, so every crater
here is young. Craters also simply superpose, so a small one inside a large
one excavates from the large one's floor. That is roughly right and gets less
right the more they overlap.
"""
@inline function _crater_profile(s::Float64)
    if s <= 1.0
        -1.0 + 1.15 * s * s * (3.0 - 2.0 * s)
    elseif s < 2.2
        e = (2.2 - s) / 1.2
        0.15 * e * e * (3.0 - 2.0 * e)
    else
        0.0
    end
end

"""
Depth of a crater of diameter `d` [m]. Small craters are bowls, 20% as deep
as they are wide. Past about 15 km they collapse into complex craters and
depth grows only as the cube root of diameter, so a 40 km crater is 3 km deep
rather than 8 — which is why the biggest features on a real Moon are also the
gentlest to fly over, and why using the simple-crater ratio everywhere buries
the surface in canyons.
"""
@inline _crater_depth(d::Float64) = min(0.2 * d, 1044.0 * (d / 1000.0)^0.301)

"""
Crater contribution at direction `u` from one size class. The class scatters
craters on a cube-sphere lattice of `n` cells per face edge; a cell either
holds one crater or does not, with probability `density`.

Every crater belongs to exactly one face — the dominant face of its own
centre — so a cell whose candidate strays over a face edge simply produces
nothing, and the neighbouring face produces it instead. Sampling looks at
every face the query direction projects into, which near an edge is two and
near a corner three. Between them there is neither a gap nor a doubling at
the seams, which is the failure mode this arrangement exists to avoid.
"""
function _craters_class(tr::LunarTerrain, u::V3, cls::Int)
    d0 = tr.d_max / tr.ratio^(cls - 1)
    reach = 1.15 * d0 / R_MOON                  # ejecta radius of the largest
    n = max(2, ceil(Int, 1.0 / max(reach, 1e-9)))
    n > 1_000_000 && (n = 1_000_000)
    cell = 2.0 / n
    salt = tr.seed ⊻ (0x51ED270B * unsafe_trunc(UInt32, cls))
    h = 0.0
    for f in 0:5
        ok, s, t = _face_project(u, f)
        ok || continue
        # only chase a face the point is actually inside (plus a cell of slop)
        (abs(s) > 1.0 + 2 * cell || abs(t) > 1.0 + 2 * cell) && continue
        i0 = floor(Int, (s + 1.0) / cell)
        j0 = floor(Int, (t + 1.0) / cell)
        for di in -1:1, dj in -1:1
            i, j = i0 + di, j0 + dj
            ch = _cell_hash(i, j, f, salt)
            _cell_rand(ch, 0) < tr.density || continue
            cs = -1.0 + (i + _cell_rand(ch, 1)) * cell
            ct = -1.0 + (j + _cell_rand(ch, 2)) * cell
            (abs(cs) > 1.0 || abs(ct) > 1.0) && continue
            cu = _face_unit(f, cs, ct)
            _dominant_face(cu) == f || continue  # someone else's crater
            dia = d0 * (0.55 + 0.9 * _cell_rand(ch, 3))
            rad = 0.5 * dia
            ang = acos(clamp(vdot(u, cu), -1.0, 1.0))
            dist = R_MOON * ang
            dist > 2.2 * rad && continue
            h += _crater_depth(dia) * _crater_profile(dist / rad)
        end
    end
    h
end

# ---- layer 3: metre-scale roughness ----------------------------------------

"Smoothstep weight for value-noise interpolation."
@inline _smooth(x) = x * x * (3.0 - 2.0 * x)

"""
One octave of 3-D value noise on a lattice of spacing `wave` metres, sampled
along the direction `u` scaled up to lunar radius. Returns [-1, 1].
"""
function _value_noise(u::V3, wave::Float64, salt::UInt32)
    p = vscale(u, R_MOON / wave)
    i0 = floor(Int, p[1]); j0 = floor(Int, p[2]); k0 = floor(Int, p[3])
    fx = _smooth(p[1] - i0); fy = _smooth(p[2] - j0); fz = _smooth(p[3] - k0)
    acc = 0.0
    for dk in 0:1
        wz = dk == 0 ? 1.0 - fz : fz
        for dj in 0:1
            wy = dj == 0 ? 1.0 - fy : fy
            for di in 0:1
                wx = di == 0 ? 1.0 - fx : fx
                g = Float64(_cell_hash(i0 + di, j0 + dj, k0 + dk, salt)) / 4.294967296e9
                acc += wx * wy * wz * (2.0 * g - 1.0)
            end
        end
    end
    acc
end

# ------------------------------------------------------------- the surface --

"""
    terrain_height(terrain, u) -> Float64

Elevation of the surface above the mean sphere, in metres, in the direction
of the unit vector `u` (Moon-fixed frame). Sum of relief, every crater class,
and two octaves of roughness. Pure, deterministic, and defined everywhere.
"""
function terrain_height(tr::LunarTerrain, u::V3)
    h = _relief(tr, u)
    for c in 1:tr.classes
        h += _craters_class(tr, u, c)
    end
    h += tr.rough * _value_noise(u, 40.0, tr.seed ⊻ 0x7A5C1E39)
    h += 0.45 * tr.rough * _value_noise(u, 15.0, tr.seed ⊻ 0x1B873593)
    h
end

"Radius of the surface [m] in direction `u`: `R_MOON` plus the elevation."
@inline terrain_radius(tr::LunarTerrain, u::V3) = R_MOON + terrain_height(tr, u)

"""
    terrain_normal(terrain, u; baseline) -> V3

Outward surface normal, from central differences over `baseline` metres. The
baseline is not a numerical detail: slope is a scale-dependent quantity, and
what a lander cares about is the slope across its own footpad circle, not
across a kilometre. The default 8 m is roughly an Apollo LM's stance.
"""
function terrain_normal(tr::LunarTerrain, u::V3; baseline::Float64 = 8.0)
    e1, e2 = _tangents(u)
    d = baseline / R_MOON
    hp1 = terrain_radius(tr, vunit(vadd(u, vscale(e1, d))))
    hm1 = terrain_radius(tr, vunit(vsub(u, vscale(e1, d))))
    hp2 = terrain_radius(tr, vunit(vadd(u, vscale(e2, d))))
    hm2 = terrain_radius(tr, vunit(vsub(u, vscale(e2, d))))
    g1 = (hp1 - hm1) / (2 * baseline)
    g2 = (hp2 - hm2) / (2 * baseline)
    vunit(vsub(u, vadd(vscale(e1, g1), vscale(e2, g2))))
end

"Local surface slope [rad] over `baseline` metres."
terrain_slope(tr::LunarTerrain, u::V3; baseline::Float64 = 8.0) =
    acos(clamp(vdot(terrain_normal(tr, u; baseline = baseline), u), -1.0, 1.0))

"Orthonormal tangent pair at a unit direction."
@inline function _tangents(u::V3)
    a = abs(u[3]) < 0.9 ? (0.0, 0.0, 1.0) : (1.0, 0.0, 0.0)
    e = vunit(vcross(a, u))
    (e, vcross(u, e))
end

"""
    surface_offset(u, e1, e2, d1, d2) -> V3

The unit direction reached by walking `d1`, `d2` metres along the tangent
pair from `u`. Exact for the sphere and correct to the metre over the tens of
metres a landing site is scored across.
"""
@inline surface_offset(u::V3, e1::V3, e2::V3, d1::Float64, d2::Float64) =
    vunit(vadd(u, vadd(vscale(e1, d1 / R_MOON), vscale(e2, d2 / R_MOON))))

# ---------------------------------------------------------- site scoring ---

"""
    site_hazard(terrain, u; radius, baseline, samples) -> (score, slope, relief)

How dangerous the ground is under `u`, scored the way a landing site actually
fails. Two things tip a lander: the ground it stands on being tilted, and the
ground under one footpad being a different height from the ground under
another. So the disc of `radius` metres is sampled on a ring plus its centre;
`slope` is the worst local slope found, `relief` the peak-to-valley height
spread across the disc, and `score` combines them into an equivalent tilt in
radians — `atan(relief / radius)` is the tilt a rigid vehicle would take if
its pads straddled the extremes.

A lander with Apollo's geometry tips somewhere past 12°, so a score under
about 6° is a site, and anything past 10° is a bad day.
"""
function site_hazard(tr::LunarTerrain, u::V3; radius::Float64 = 15.0,
                     baseline::Float64 = 8.0, samples::Int = 8)
    e1, e2 = _tangents(u)
    h0 = terrain_radius(tr, u)
    hmin = h0; hmax = h0
    worst = terrain_slope(tr, u; baseline = baseline)
    for k in 0:(samples - 1)
        a = 2pi * k / samples
        for fr in (0.55, 1.0)
            p = surface_offset(u, e1, e2, fr * radius * cos(a), fr * radius * sin(a))
            hh = terrain_radius(tr, p)
            hh < hmin && (hmin = hh)
            hh > hmax && (hmax = hh)
            sl = terrain_slope(tr, p; baseline = baseline)
            sl > worst && (worst = sl)
        end
    end
    spread = hmax - hmin
    (max(worst, atan(spread / radius)), worst, spread)
end

"""
    safe_site(terrain, u0, e_down, e_cross; reach, step, radius) -> (u, offset_down, offset_cross, score)

Pick the least hazardous landing point within reach of `u0`, searching a grid
that runs `reach` metres downrange and to either side in `step` increments —
the same shape as the footprint a lander at high gate can actually redesignate
into, which is much longer downrange than across.

`e_down` and `e_cross` are the downrange and crossrange unit tangents at `u0`.
Returns the chosen direction, its offset from `u0` in metres, and its hazard
score. Ties break toward the smaller offset, because propellant spent flying
across the surface is propellant not available to hover over it.
"""
function safe_site(tr::LunarTerrain, u0::V3, e_down::V3, e_cross::V3;
                   reach::Float64 = 900.0, step::Float64 = 120.0,
                   cross_reach::Float64 = 360.0, radius::Float64 = 15.0)
    best_u = u0
    best = site_hazard(tr, u0; radius = radius)[1]
    best_d = 0.0; best_c = 0.0
    # a nudge per metre of travel, so a marginally better site far away loses
    penalty = 1.0e-6                             # rad per metre of offset
    best += 0.0
    nd = floor(Int, reach / step)
    nc = floor(Int, cross_reach / step)
    for i in -nd:nd, j in -nc:nc
        (i == 0 && j == 0) && continue
        d = i * step; c = j * step
        u = surface_offset(u0, e_down, e_cross, d, c)
        sc = site_hazard(tr, u; radius = radius)[1] + penalty * hypot(d, c)
        if sc < best
            best = sc; best_u = u; best_d = d; best_c = c
        end
    end
    (best_u, best_d, best_c, best)
end

# ------------------------------------------------- Moon-fixed bookkeeping ---

"""
    moonfixed_basis(eph, t) -> (xhat, yhat, zhat)

The tidally-locked Moon-fixed axes at time `t`, expressed in the inertial
frame: `xhat` points at the Earth (the sub-Earth meridian, longitude zero),
`zhat` is the ephemeris pole, `yhat` completes the triad and leads in the
direction of orbital motion. Synchronous rotation is what makes this
definition exact rather than conventional — the Moon really does keep one
face to us, so the Earth direction *is* a body-fixed axis.
"""
@inline function moonfixed_basis(eph::CircularMoonEphemeris, t::Float64)
    s = moon_position(eph, t)
    xhat = vunit(vscale(s, -1.0))
    zhat = vunit(vcross(s, moon_velocity(eph, t)))
    (xhat, vcross(zhat, xhat), zhat)
end

"Components of a Moon-centred inertial vector in the Moon-fixed frame."
@inline function moonfixed(r::V3, t::Float64, eph::CircularMoonEphemeris)
    x, y, z = moonfixed_basis(eph, t)
    (vdot(r, x), vdot(r, y), vdot(r, z))
end

"Inertial components of a Moon-fixed vector."
@inline function moonfixed_inv(rf::V3, t::Float64, eph::CircularMoonEphemeris)
    x, y, z = moonfixed_basis(eph, t)
    vadd(vadd(vscale(x, rf[1]), vscale(y, rf[2])), vscale(z, rf[3]))
end

"""
    SurfaceModel(terrain, eph)

Terrain plus the ephemeris needed to know which way the Moon is facing. This
is what turns "elevation in direction `u`" into "how far the ground is below
the vehicle at time `t`", and it is the only object the descent integrators
need in order to fly over real ground instead of a sphere.

Pass `nothing` instead, anywhere one of these is accepted, and the surface is
a sphere of radius `R_MOON` — which is what every result committed before
terrain existed was flown against.
"""
struct SurfaceModel
    terrain::LunarTerrain
    eph::CircularMoonEphemeris
end

"Radius of the ground [m] under a Moon-centred inertial position at time `t`."
@inline surface_radius(sm::SurfaceModel, r::V3, t::Float64) =
    terrain_radius(sm.terrain, vunit(moonfixed(r, t, sm.eph)))
@inline surface_radius(::Nothing, ::V3, ::Float64) = R_MOON

"Height of the vehicle above the ground directly below it [m]."
@inline surface_altitude(sm, r::V3, t::Float64) = vnorm(r) - surface_radius(sm, r, t)

"Elevation of the ground above the mean sphere under a position [m]."
@inline ground_elevation(sm::SurfaceModel, r::V3, t::Float64) =
    surface_radius(sm, r, t) - R_MOON
@inline ground_elevation(::Nothing, ::V3, ::Float64) = 0.0

"""
    terrain_profile(sm, r0, v0, t; span, n) -> (arc, elevation)

Ground elevation along the track a vehicle at `(r0, v0)` is flying over, from
`-span/2` to `+span/2` metres of surface arc about the point below it. This is
the cut through the terrain that a descent plot needs, and the same one the
viewer draws.
"""
function terrain_profile(sm::SurfaceModel, r0::V3, v0::V3, t::Float64;
                         span::Float64 = 20.0e3, n::Int = 200)
    u0 = vunit(moonfixed(r0, t, sm.eph))
    vf = moonfixed(v0, t, sm.eph)
    e = vunit(vsub(vf, vscale(u0, vdot(vf, u0))))
    arc = Vector{Float64}(undef, n)
    el = Vector{Float64}(undef, n)
    for k in 1:n
        d = -span / 2 + span * (k - 1) / (n - 1)
        arc[k] = d
        el[k] = terrain_height(sm.terrain, vunit(vadd(u0, vscale(e, d / R_MOON))))
    end
    (arc, el)
end
