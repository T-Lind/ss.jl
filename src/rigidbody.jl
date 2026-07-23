# Rigid-body attitude: quaternion kinematics and Euler's rotational
# dynamics with a diagonal (principal-axes) inertia tensor.
#
# Conventions:
#   * Quaternion q = (w, x, y, z), unit norm, maps BODY -> INERTIAL:
#     v_eci = q ⊗ (0, v_body) ⊗ q*.
#   * Body rates ω are expressed in the body frame; kinematics
#     q̇ = ½ q ⊗ (0, ω).
#   * Euler's equations with diagonal inertia I = diag(Ixx, Iyy, Izz):
#     I ω̇ = M_body − ω × (I ω).

const Quat = NTuple{4,Float64}

@inline qmul(a::Quat, b::Quat) = (
    a[1]*b[1] - a[2]*b[2] - a[3]*b[3] - a[4]*b[4],
    a[1]*b[2] + a[2]*b[1] + a[3]*b[4] - a[4]*b[3],
    a[1]*b[3] - a[2]*b[4] + a[3]*b[1] + a[4]*b[2],
    a[1]*b[4] + a[2]*b[3] - a[3]*b[2] + a[4]*b[1],
)

@inline qconj(q::Quat) = (q[1], -q[2], -q[3], -q[4])

@inline function qnormalize(q::Quat)
    n = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2)
    (q[1]/n, q[2]/n, q[3]/n, q[4]/n)
end

"Rotate body-frame vector `v` into the inertial frame."
@inline function qrotate(q::Quat, v::V3)
    # q ⊗ (0,v) ⊗ q*  (expanded, no allocations)
    t = qmul(q, (0.0, v[1], v[2], v[3]))
    r = qmul(t, qconj(q))
    (r[2], r[3], r[4])
end

"Rotate inertial vector into the body frame."
@inline qrotate_inv(q::Quat, v::V3) = qrotate(qconj(q), v)

"Quaternion for rotation of `angle` [rad] about unit `axis`."
@inline function quat_axis_angle(axis::V3, angle::Float64)
    s, c = sincos(angle / 2)
    (c, s * axis[1], s * axis[2], s * axis[3])
end

"""
    quat_from_to(a, b) -> Quat

Shortest rotation taking unit vector `a` to unit vector `b`.
"""
function quat_from_to(a::V3, b::V3)
    d = vdot(a, b)
    if d > 1 - 1e-12
        return (1.0, 0.0, 0.0, 0.0)
    elseif d < -1 + 1e-12
        # 180°: any axis ⊥ a
        ax = abs(a[1]) < 0.9 ? vunit(vcross(a, (1.0, 0.0, 0.0))) :
                               vunit(vcross(a, (0.0, 1.0, 0.0)))
        return (0.0, ax[1], ax[2], ax[3])
    end
    ax = vcross(a, b)
    q = (1.0 + d, ax[1], ax[2], ax[3])
    qnormalize(q)
end

"Quaternion derivative for body rates ω: q̇ = ½ q ⊗ (0, ω)."
@inline function qdot(q::Quat, w::V3)
    h = qmul(q, (0.0, w[1], w[2], w[3]))
    (0.5h[1], 0.5h[2], 0.5h[3], 0.5h[4])
end

"""
    euler_wdot(I, w, M) -> V3

Angular acceleration from Euler's equations, diagonal inertia
`I = (Ixx, Iyy, Izz)`, body torque `M`.
"""
@inline function euler_wdot(I::V3, w::V3, M::V3)
    ((M[1] - (I[3] - I[2]) * w[2] * w[3]) / I[1],
     (M[2] - (I[1] - I[3]) * w[3] * w[1]) / I[2],
     (M[3] - (I[2] - I[1]) * w[1] * w[2]) / I[3])
end

"Rotational kinetic energy ½ ωᵀIω [J]."
@inline rot_energy(I::V3, w::V3) =
    0.5 * (I[1]*w[1]^2 + I[2]*w[2]^2 + I[3]*w[3]^2)

"Body-frame angular momentum Iω."
@inline ang_momentum(I::V3, w::V3) = (I[1]*w[1], I[2]*w[2], I[3]*w[3])
