# Minimal allocation-free 3-vector helpers built on NTuple{3,Float64}.
# (Keeps the core dependency-free; StaticArrays would be the natural upgrade.)

const V3 = NTuple{3,Float64}

@inline v3(x, y, z) = (Float64(x), Float64(y), Float64(z))
@inline vadd(a::V3, b::V3) = (a[1]+b[1], a[2]+b[2], a[3]+b[3])
@inline vsub(a::V3, b::V3) = (a[1]-b[1], a[2]-b[2], a[3]-b[3])
@inline vscale(a::V3, s::Real) = (a[1]*s, a[2]*s, a[3]*s)
@inline vdot(a::V3, b::V3) = a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
@inline vcross(a::V3, b::V3) = (a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1])
@inline vnorm(a::V3) = sqrt(vdot(a, a))
@inline function vunit(a::V3)
    n = vnorm(a)
    n > 0 ? vscale(a, 1/n) : (0.0, 0.0, 0.0)
end

# Rotation of an ECI vector into ECEF axes by Earth rotation angle theta:
# ecef = R3(theta) * eci
@inline function rot_z(a::V3, theta::Float64)
    c, s = cos(theta), sin(theta)
    (c*a[1] + s*a[2], -s*a[1] + c*a[2], a[3])
end
