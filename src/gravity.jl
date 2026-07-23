# Gravity models.
#
# Extension points:
#   * subtype `AbstractGravity` and implement `gravity_accel(model, r, t)`;
#   * `CompositeGravity` sums any number of models, so third-body point
#     masses (Moon, Sun) can be added by implementing an ephemeris-backed
#     `ThirdBodyGravity` and appending it to the composite.

abstract type AbstractGravity end

"""
    gravity_accel(model, r_eci, t) -> V3

Gravitational acceleration [m/s^2] at ECI position `r_eci` [m], time `t` [s].
"""
function gravity_accel end

"Point-mass (two-body) gravity."
struct PointMassGravity <: AbstractGravity
    mu::Float64
end
PointMassGravity() = PointMassGravity(MU_EARTH)

function gravity_accel(g::PointMassGravity, r::V3, t::Float64)
    rn = vnorm(r)
    vscale(r, -g.mu / rn^3)
end

"Point mass plus the J2 oblateness perturbation (dominant non-spherical term)."
struct J2Gravity <: AbstractGravity
    mu::Float64
    re::Float64
    j2::Float64
end
J2Gravity() = J2Gravity(MU_EARTH, RE_EQ, J2_EARTH)

function gravity_accel(g::J2Gravity, r::V3, t::Float64)
    x, y, z = r
    rn = vnorm(r)
    r2 = rn * rn
    zr2 = (z * z) / r2
    k = 1.5 * g.j2 * (g.re * g.re) / r2
    c = -g.mu / (rn * r2)
    ax = c * x * (1 + k * (1 - 5zr2))
    ay = c * y * (1 + k * (1 - 5zr2))
    az = c * z * (1 + k * (3 - 5zr2))
    (ax, ay, az)
end

"""
    ThirdBodyGravity(mu, ephemeris)

Differential (tidal) acceleration from a third body such as the Moon or Sun.
`ephemeris(t)` must return the body's ECI position [m] at time `t` [s].
The standard formulation  a = mu * ( (s-r)/|s-r|^3 - s/|s|^3 )  is used, where
`s` is the third-body position, so the indirect term is included.
Not enabled in the default scenarios (its effect on a <1 h reentry is
negligible), but ready for mid-course / cislunar extensions.
"""
struct ThirdBodyGravity{F} <: AbstractGravity
    mu::Float64
    ephemeris::F
end

function gravity_accel(g::ThirdBodyGravity, r::V3, t::Float64)
    s = g.ephemeris(t)::V3
    d = vsub(s, r)
    dn = vnorm(d)
    sn = vnorm(s)
    vsub(vscale(d, g.mu / dn^3), vscale(s, g.mu / sn^3))
end

"Sum of several gravity models."
struct CompositeGravity{T<:Tuple} <: AbstractGravity
    models::T
end
CompositeGravity(models::AbstractGravity...) = CompositeGravity(models)

function gravity_accel(g::CompositeGravity, r::V3, t::Float64)
    a = (0.0, 0.0, 0.0)
    for m in g.models
        a = vadd(a, gravity_accel(m, r, t))
    end
    a
end
