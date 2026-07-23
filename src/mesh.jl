# Triangle meshes: STL input/output and rigid-body mass properties.
#
# Meshes are stored as a triangle soup (three vertices per face). Mass
# properties use the standard signed-tetrahedron decomposition (divergence
# theorem; Eberly's polyhedron formulas), so they are exact for closed,
# consistently-wound meshes: uniform density is assumed and scaled to the
# requested total mass.

"Triangle soup: `tris[k] = (v1, v2, v3)` in meters, outward winding (CCW)."
struct TriMesh
    tris::Vector{NTuple{3,V3}}
end

Base.length(m::TriMesh) = length(m.tris)

"Face normal (unnormalized, |n| = 2·area)."
@inline face_normal2(t::NTuple{3,V3}) = vcross(vsub(t[2], t[1]), vsub(t[3], t[1]))

"Total surface area [m^2]."
mesh_area(m::TriMesh) = 0.5 * sum(vnorm(face_normal2(t)) for t in m.tris)

"Signed volume [m^3] (positive for outward winding of a closed mesh)."
mesh_volume(m::TriMesh) =
    sum(vdot(t[1], vcross(t[2], t[3])) for t in m.tris) / 6.0

"""
    read_stl(path) -> TriMesh

Binary or ASCII STL (autodetected).
"""
function read_stl(path::AbstractString)
    data = read(path)
    is_ascii = length(data) >= 6 && String(data[1:5]) == "solid" &&
               (length(data) < 84 ||
                84 + 50 * reinterpret(UInt32, data[81:84])[1] != length(data))
    tris = NTuple{3,V3}[]
    if is_ascii
        verts = V3[]
        for line in split(String(data), '\n')
            w = split(strip(line))
            if length(w) == 4 && w[1] == "vertex"
                push!(verts, (parse(Float64, w[2]), parse(Float64, w[3]),
                              parse(Float64, w[4])))
                if length(verts) == 3
                    push!(tris, (verts[1], verts[2], verts[3]))
                    empty!(verts)
                end
            end
        end
    else
        n = reinterpret(UInt32, data[81:84])[1]
        off = 85                          # 1-indexed: first record after header+count
        f32(o) = Float64(reinterpret(Float32, data[o:o+3])[1])
        for _ in 1:n
            v(k) = (f32(off + 12 + 12(k-1)), f32(off + 16 + 12(k-1)),
                    f32(off + 20 + 12(k-1)))
            push!(tris, (v(1), v(2), v(3)))
            off += 50
        end
    end
    TriMesh(tris)
end

"Write a binary STL."
function write_stl(path::AbstractString, m::TriMesh; name::String = "ss.jl")
    open(path, "w") do io
        hdr = zeros(UInt8, 80)
        hdr[1:min(end, length(name))] .= UInt8[c for c in name[1:min(80, length(name))]]
        write(io, hdr)
        write(io, UInt32(length(m.tris)))
        for t in m.tris
            n = vunit(face_normal2(t))
            for x in (n..., t[1]..., t[2]..., t[3]...)
                write(io, Float32(x))
            end
            write(io, UInt16(0))
        end
    end
    path
end

"""
    mass_properties(m, mass) -> (volume, cg, inertia, offdiag_frac)

Volume [m^3], center of mass [m], principal-ish inertia diagonal
(Ixx, Iyy, Izz) about the CG [kg·m^2] for uniform density scaled to `mass`,
and the largest product-of-inertia magnitude relative to the smallest
diagonal term (a consistency check — near zero for symmetric bodies, and
the diagonal-inertia 6-DOF assumes it is small).
"""
function mass_properties(m::TriMesh, mass::Float64)
    # Eberly, "Polyhedral Mass Properties": accumulate integrals per face
    intg = zeros(10)   # 1, x, y, z, x², y², z², xy, yz, zx
    for t in m.tris
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = t
        a1 = x1 - x0; b1 = y1 - y0; c1 = z1 - z0
        a2 = x2 - x0; b2 = y2 - y0; c2 = z2 - z0
        d0 = b1 * c2 - b2 * c1
        d1 = a2 * c1 - a1 * c2
        d2 = a1 * b2 - a2 * b1
        function subexpr(w0, w1, w2)
            t0 = w0 + w1
            f1 = t0 + w2
            t1 = w0 * w0
            t2 = t1 + w1 * t0
            f2 = t2 + w2 * f1
            f3 = w0 * t1 + w1 * t2 + w2 * f2
            g0 = f2 + w0 * (f1 + w0)
            g1 = f2 + w1 * (f1 + w1)
            g2 = f2 + w2 * (f1 + w2)
            (f1, f2, f3, g0, g1, g2)
        end
        f1x, f2x, f3x, g0x, g1x, g2x = subexpr(x0, x1, x2)
        f1y, f2y, f3y, g0y, g1y, g2y = subexpr(y0, y1, y2)
        f1z, f2z, f3z, g0z, g1z, g2z = subexpr(z0, z1, z2)
        intg[1] += d0 * f1x
        intg[2] += d0 * f2x; intg[3] += d1 * f2y; intg[4] += d2 * f2z
        intg[5] += d0 * f3x; intg[6] += d1 * f3y; intg[7] += d2 * f3z
        intg[8] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x)
        intg[9] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y)
        intg[10] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z)
    end
    intg[1] /= 6
    intg[2:4] ./= 24
    intg[5:7] ./= 60
    intg[8:10] ./= 120

    vol = intg[1]
    vol > 0 || error("mesh volume non-positive — check closedness/winding")
    cg = (intg[2] / vol, intg[3] / vol, intg[4] / vol)
    rho = mass / vol
    # inertia about origin, then shift to CG
    Ixx = rho * (intg[6] + intg[7]) - mass * (cg[2]^2 + cg[3]^2)
    Iyy = rho * (intg[5] + intg[7]) - mass * (cg[3]^2 + cg[1]^2)
    Izz = rho * (intg[5] + intg[6]) - mass * (cg[1]^2 + cg[2]^2)
    Ixy = rho * intg[8] - mass * cg[1] * cg[2]
    Iyz = rho * intg[9] - mass * cg[2] * cg[3]
    Izx = rho * intg[10] - mass * cg[3] * cg[1]
    offd = maximum(abs, (Ixy, Iyz, Izx)) / minimum((Ixx, Iyy, Izz))
    (volume = vol, cg = cg, inertia = (Ixx, Iyy, Izz), offdiag_frac = offd)
end

# --------------------------------------------------------- mesh builders --

"""
    lathe_mesh(profile; nseg=48) -> TriMesh

Surface of revolution about the x-axis. `profile` is a vector of
`(x, radius)` pairs from nose to tail; radii of 0 at the ends close the
body. Winding is outward for a profile marching in +x.
"""
function lathe_mesh(profile::Vector{Tuple{Float64,Float64}}; nseg::Int = 48)
    ring(x, r) = [ (x, r * cos(2pi * k / nseg), r * sin(2pi * k / nseg)) for k in 0:nseg-1 ]
    tris = NTuple{3,V3}[]
    prev = nothing
    for (x, r) in profile
        cur = r > 1e-12 ? ring(x, r) : fill((x, 0.0, 0.0), nseg)
        if prev !== nothing
            for k in 1:nseg
                k2 = mod1(k + 1, nseg)
                a, b = prev[k], prev[k2]
                c, d = cur[k], cur[k2]
                a != b && push!(tris, (a, c, b))
                c != d && push!(tris, (b, c, d))
            end
        end
        prev = cur
    end
    ensure_outward(TriMesh(tris))
end

"Flip winding if the signed volume is negative (normals then point outward)."
function ensure_outward(m::TriMesh)
    mesh_volume(m) >= 0 ? m :
        TriMesh([(t[1], t[3], t[2]) for t in m.tris])
end

"Axis-aligned closed box between corners `lo` and `hi` (12 triangles)."
function box_mesh(lo::V3, hi::V3)
    x0, y0, z0 = lo; x1, y1, z1 = hi
    p = [(x0,y0,z0), (x1,y0,z0), (x1,y1,z0), (x0,y1,z0),
         (x0,y0,z1), (x1,y0,z1), (x1,y1,z1), (x0,y1,z1)]
    f = [(1,3,2),(1,4,3), (5,6,7),(5,7,8), (1,2,6),(1,6,5),
         (2,3,7),(2,7,6), (3,4,8),(3,8,7), (4,1,5),(4,5,8)]
    ensure_outward(TriMesh([(p[a], p[b], p[c]) for (a,b,c) in f]))
end

"Concatenate meshes (triangle soup union; volumes add, overlaps double-count)."
merge_meshes(ms::TriMesh...) = TriMesh(vcat((m.tris for m in ms)...))
