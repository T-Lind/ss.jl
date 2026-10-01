# Maneuver planning: two-body transfer and rendezvous mathematics.
#
#   * Lambert's problem (universal variables, Bate-Mueller-White): the
#     velocity pair that connects two position vectors in a given time of
#     flight — the workhorse behind transfer design, intercept, faster
#     lunar trajectories, and TCM re-targeting.
#   * Closed-form impulsive building blocks: Hohmann transfers, plane
#     changes, propellant costs.
#   * Clohessy-Wiltshire relative motion: the linearized dynamics of a
#     chaser about a circular-orbit target (RIC frame), with the two-impulse
#     intercept solution used for rendezvous.
#
# Everything here is two-body/linearized DESIGN math; the flight itself is
# always re-simulated with the full dynamics (see the rendezvous script,
# which verifies the CW solution against a nonlinear propagation).

"Stumpff functions C(z), S(z) with series fallbacks near z = 0."
function stumpff(z::Float64)
    if z > 1e-6
        sz = sqrt(z)
        C = (1 - cos(sz)) / z
        S = (sz - sin(sz)) / (z * sz)
    elseif z < -1e-6
        sz = sqrt(-z)
        C = (cosh(sz) - 1) / (-z)
        S = (sinh(sz) - sz) / (-z * sz)
    else
        C = 1/2 - z/24 + z^2/720
        S = 1/6 - z/120 + z^2/5040
    end
    (C, S)
end

"""
    lambert(r1, r2, tof; mu=MU_EARTH, long_way=false) -> (v1, v2)

Universal-variables Lambert solver: velocities at departure and arrival for
the transfer from `r1` to `r2` [m] in `tof` [s]. `long_way` selects the
transfer angle > 180°. Zero-revolution solutions only. Throws if the
iteration fails (e.g. tof too short for the geometry).
"""
function lambert(r1::V3, r2::V3, tof::Float64;
                 mu::Float64 = MU_EARTH, long_way::Bool = false,
                 tol::Float64 = 1e-9, max_iter::Int = 60)
    r1n = vnorm(r1); r2n = vnorm(r2)
    all(isfinite, (r1..., r2..., tof, mu, tol)) && r1n > 0 && r2n > 0 &&
        tof > 0 && mu > 0 && tol > 0 && max_iter > 0 ||
        throw(ArgumentError("lambert requires finite nonzero positions and positive tof, mu, tol and max_iter"))
    vnorm(vcross(vunit(r1), vunit(r2))) > 1e-14 ||
        throw(ArgumentError("lambert: collinear positions leave the transfer plane undefined"))
    cosd = clamp(vdot(r1, r2) / (r1n * r2n), -1.0, 1.0)
    A = sqrt(r1n * r2n * (1 + cosd)) * (long_way ? -1.0 : 1.0)
    abs(A) < 1e-9 && error("lambert: 180° transfer is singular (plane undefined)")

    yfun(z, C, S) = r1n + r2n + A * (z * S - 1) / sqrt(C)

    # bisection on z (robust; t is monotone increasing in z)
    zlo, zhi = -4pi^2, 4pi^2
    local z, y, C, S
    tfun(z) = begin
        C, S = stumpff(z)
        y = yfun(z, C, S)
        y < 0 && return NaN
        x = sqrt(y / C)
        (x^3 * S + A * sqrt(y)) / sqrt(mu)
    end
    # y < 0 is the zero-time edge, not a reason to discard the entire
    # hyperbolic branch. Keep it as the lower bound and bisect toward it.
    # Long-way transfers can need a more negative z for a short flight.
    for _ in 1:60
        tlo = tfun(zlo)
        (!isfinite(tlo) || tlo <= tof) && break
        zlo *= 2
    end
    tlo = tfun(zlo)
    (!isfinite(tlo) || tlo <= tof) || error("lambert: no feasible time bracket")
    done = false
    for _ in 1:max_iter
        z = 0.5 * (zlo + zhi)
        tz = tfun(z)
        (isnan(tz) || tz < tof) ? (zlo = z) : (zhi = z)
        if abs(zhi - zlo) < tol
            done = true
            break
        end
    end
    done || error("lambert: iteration budget exhausted")
    z = 0.5 * (zlo + zhi)
    C, S = stumpff(z)
    y = yfun(z, C, S)

    f = 1 - y / r1n
    g = A * sqrt(y / mu)
    gdot = 1 - y / r2n
    v1 = vscale(vsub(r2, vscale(r1, f)), 1 / g)
    v2 = vscale(vsub(vscale(r2, gdot), r1), 1 / g)
    (v1, v2)
end

"""
    hohmann(r1, r2; mu=MU_EARTH) -> (dv1, dv2, tof)

Coplanar circular-to-circular Hohmann transfer between radii `r1`, `r2` [m].
"""
function hohmann(r1::Float64, r2::Float64; mu::Float64 = MU_EARTH)
    at = 0.5 * (r1 + r2)
    v1c = sqrt(mu / r1); v2c = sqrt(mu / r2)
    vp = sqrt(mu * (2 / r1 - 1 / at))
    va = sqrt(mu * (2 / r2 - 1 / at))
    (abs(vp - v1c), abs(v2c - va), pi * sqrt(at^3 / mu))
end

"Delta-v of a pure plane change of `di` [rad] at speed `v` [m/s]."
plane_change_dv(v::Float64, di::Float64) = 2 * v * sin(di / 2)

"Propellant mass to realize `dv` [m/s] impulsively from mass `m0` at `isp` [s]."
impulsive_prop(m0::Float64, dv::Float64, isp::Float64) =
    m0 * (1 - exp(-dv / (G0 * isp)))

# ---------------------------------------------------- relative motion (CW) --

"""
    cw_stm(n, t) -> (Φrr, Φrv, Φvr, Φvv)

Clohessy-Wiltshire state transition submatrices (3x3 tuples-of-tuples) for
mean motion `n` [rad/s] and time `t` [s]. RIC frame: x radial (away from
Earth), y along-track, z cross-track; target on a circular orbit.
"""
function cw_stm(n::Float64, t::Float64)
    s, c = sincos(n * t)
    Φrr = ((4 - 3c, 0.0, 0.0),
           (6 * (s - n * t), 1.0, 0.0),
           (0.0, 0.0, c))
    Φrv = ((s / n, 2 * (1 - c) / n, 0.0),
           (2 * (c - 1) / n, (4s - 3n * t) / n, 0.0),
           (0.0, 0.0, s / n))
    Φvr = ((3n * s, 0.0, 0.0),
           (6n * (c - 1), 0.0, 0.0),
           (0.0, 0.0, -n * s))
    Φvv = ((c, 2s, 0.0),
           (-2s, 4c - 3, 0.0),
           (0.0, 0.0, c))
    (Φrr, Φrv, Φvr, Φvv)
end

@inline _mat3vec(M, v) = (M[1][1]*v[1] + M[1][2]*v[2] + M[1][3]*v[3],
                          M[2][1]*v[1] + M[2][2]*v[2] + M[2][3]*v[3],
                          M[3][1]*v[1] + M[3][2]*v[2] + M[3][3]*v[3])

"Propagate CW relative state (r, v) by time `t`."
function cw_propagate(r::V3, v::V3, n::Float64, t::Float64)
    Φrr, Φrv, Φvr, Φvv = cw_stm(n, t)
    (vadd(_mat3vec(Φrr, r), _mat3vec(Φrv, v)),
     vadd(_mat3vec(Φvr, r), _mat3vec(Φvv, v)))
end

"""
    cw_two_impulse(r0, v0, n, tof) -> (dv1, dv2, v_transfer)

Two-impulse rendezvous: the burn `dv1` that puts the chaser on a transfer
arriving at the target (origin) after `tof`, and the braking burn `dv2`
that nulls the arrival velocity. Requires Φrv invertible (avoid tof at
exact multiples of the half-period).
"""
function cw_two_impulse(r0::V3, v0::V3, n::Float64, tof::Float64)
    Φrr, Φrv, Φvr, Φvv = cw_stm(n, tof)
    # solve Φrv * v_req = -Φrr * r0  (3x3, z decoupled)
    b = vscale(_mat3vec(Φrr, r0), -1.0)
    a11, a12 = Φrv[1][1], Φrv[1][2]
    a21, a22 = Φrv[2][1], Φrv[2][2]
    det = a11 * a22 - a12 * a21
    abs(det) < 1e-12 && error("cw_two_impulse: singular tof (near n·t = 2πk)")
    vx = ( a22 * b[1] - a12 * b[2]) / det
    vy = (-a21 * b[1] + a11 * b[2]) / det
    # cross-track decouples; singular only if there is an offset to null there
    vz = if abs(Φrv[3][3]) < 1e-9
        abs(b[3]) < 1e-6 ? 0.0 :
            error("cw_two_impulse: cross-track singular tof (n·t = πk)")
    else
        b[3] / Φrv[3][3]
    end
    v_req = (vx, vy, vz)
    dv1 = vsub(v_req, v0)
    _, v_arr = cw_propagate(r0, v_req, n, tof)
    (dv1, vscale(v_arr, -1.0), v_req)
end
