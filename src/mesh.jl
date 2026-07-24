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

"Translate every vertex of a mesh by `d` (internal helper)."
_translate_mesh(m::TriMesh, d::V3) =
    TriMesh([(vadd(t[1], d), vadd(t[2], d), vadd(t[3], d)) for t in m.tris])

"Closed engine bell: exit plane at `xexit`, throat `len` above it, offset to (y, z)."
function _bell_mesh(xexit, len, rex, rt, y, z; nseg::Int = 24)
    prof = Tuple{Float64,Float64}[(xexit, 0.0), (xexit, rex)]
    for f in range(0.0, 1.0; length = 7)[2:end]
        push!(prof, (xexit + len * f, rt + (rex - rt) * (1 - f)^1.6))
    end
    push!(prof, (xexit + len, 0.0))
    _translate_mesh(lathe_mesh(prof; nseg = nseg), (0.0, y, z))
end

"""
    rocket_mesh(; diameter, prop_masses, densities, fairing_len, nseg)
        -> (mesh, sections)

Procedural launch-vehicle geometry with real detailing: a first stage with
an engine skirt and a five-bell cluster, one cylindrical barrel per stage
sized so its tank volume holds `prop_masses[k]` of propellant at
`densities[k]` (plus ullage and an engine/interstage bay), recessed
interstage collars with a nested vacuum bell on every upper stage, cable
raceways, RCS pods and a payload adapter cone on the kick stage, a blunt
entry-pod capsule on the adapter, and an ogive fairing enclosing both.
Body +x is the nose axis; the tail plate sits at x = 0 with the first-stage
bells extending to x ≈ -0.32·diameter.

Each piece is a closed solid, so `sections` carries both the axial extent
and the triangle range of every component in the merged soup:
`(name, x0, x1, t0, t1)` bottom-up with `:pod` then `:fairing` last —
viewers can hide/detach a section by dropping `tris[t0:t1]`.
"""
function rocket_mesh(; diameter::Float64 = 1.8,
                     prop_masses::Vector{Float64} = [42000.0, 9500.0, 950.0],
                     densities::Vector{Float64} = fill(1020.0, length(prop_masses)),
                     fairing_len::Float64 = 2.2 * diameter,
                     nseg::Int = 48)
    length(prop_masses) == length(densities) ||
        throw(ArgumentError("prop_masses and densities length mismatch"))
    D = diameter
    r = D / 2
    A = pi * r^2
    K = length(prop_masses)
    nb = max(16, nseg ÷ 2)
    parts = TriMesh[]
    sections = NamedTuple[]
    tcount = 0
    function finish!(name, x0, x1, ms::Vector{TriMesh})
        n = sum(length, ms)
        append!(parts, ms)
        push!(sections, (name = name, x0 = x0, x1 = x1,
                         t0 = tcount + 1, t1 = tcount + n))
        tcount += n
    end
    raceway(xa, xb) = box_mesh((xa, 0.955r, -0.030D), (xb, r + 0.048D, 0.030D))

    x = 0.28D                                    # engine-skirt cone length
    for (k, (mp, rho)) in enumerate(zip(prop_masses, densities))
        len = mp / (rho * A) * 1.15 + 0.9D       # tank + ullage + engine bay
        ms = TriMesh[]
        if k == 1
            # skirt + barrel, five-bell cluster half-recessed below the plate
            push!(ms, lathe_mesh(Tuple{Float64,Float64}[
                (0.0, 0.0), (0.0, 0.80r), (0.28D, r), (x + len, r), (x + len, 0.0)];
                nseg = nseg))
            push!(ms, _bell_mesh(-0.32D, 0.42D, 0.105D, 0.050D, 0.0, 0.0; nseg = nb))
            for a in (0.25pi):(0.5pi):(1.99pi)
                push!(ms, _bell_mesh(-0.32D, 0.42D, 0.105D, 0.050D,
                                     0.52r * cos(a), 0.52r * sin(a); nseg = nb))
            end
            push!(ms, raceway(0.30D, x + len - 0.02D))
            finish!(:stage1, -0.32D, x + len, ms)
        else
            # recessed interstage collar, then the barrel; the vacuum bell
            # nests down into the stage below (revealed at separation)
            push!(ms, lathe_mesh(Tuple{Float64,Float64}[
                (x, 0.0), (x, 0.945r), (x + 0.10D, 0.945r), (x + 0.10D, r),
                (x + len, r), (x + len, 0.0)]; nseg = nseg))
            bex, blen = k == K ? (0.085D, 0.22D) : (0.155D, 0.36D)
            push!(ms, _bell_mesh(x + 0.04D - blen, blen, bex, 0.045D, 0.0, 0.0;
                                 nseg = nb))
            k < K && push!(ms, raceway(x + 0.12D, x + len - 0.02D))
            if k == K
                # kick stage: four RCS pods + the payload adapter cone
                xm = x + 0.5 * len
                rc = 0.955r + 0.024D
                for (py, pz) in ((1, 0), (-1, 0), (0, 1), (0, -1))
                    hy = py == 0 ? 0.033D : 0.024D
                    hz = pz == 0 ? 0.033D : 0.024D
                    push!(ms, box_mesh((xm - 0.05D, py * rc - hy, pz * rc - hz),
                                       (xm + 0.05D, py * rc + hy, pz * rc + hz)))
                end
                push!(ms, lathe_mesh(Tuple{Float64,Float64}[
                    (x + len, 0.0), (x + len, 0.90r),
                    (x + len + 0.13D, 0.44r), (x + len + 0.13D, 0.0)]; nseg = nseg))
            end
            finish!(Symbol(:stage, k), x, x + len + (k == K ? 0.13D : 0.0), ms)
        end
        x += len
    end
    # pod: blunt capsule seated on the adapter, inside the fairing (base caps
    # offset a hair so coincident flat faces don't z-fight in viewers)
    rp = min(0.75, 0.85r)
    lp = 2.0rp
    xb = x + 0.13D + 0.02
    finish!(:pod, x, xb + lp, TriMesh[lathe_mesh(Tuple{Float64,Float64}[
        (xb, 0.0), (xb, 0.92rp), (xb + 0.06rp, rp), (xb + 0.14rp, rp),
        (xb + 0.75lp, 0.40rp), (xb + 0.82lp, 0.34rp), (xb + 0.86lp, 0.34rp),
        (xb + 0.97lp, 0.24rp), (xb + lp, 0.0)]; nseg = nseg)])
    # fairing: closed shell — cylindrical shoulder, power-law ogive, eased tip
    x0f = x
    xsh = x0f + 0.12 * fairing_len
    prof = Tuple{Float64,Float64}[(x0f, 0.0), (x0f, r), (xsh, r)]
    for f in range(0.0, 1.0; length = 11)[2:end-1]
        push!(prof, (xsh + f * 0.88 * fairing_len, r * (1 - f^2)^0.60))
    end
    push!(prof, (x0f + 0.985 * fairing_len, 0.055r))
    push!(prof, (x0f + fairing_len, 0.0))
    finish!(:fairing, x0f, x0f + fairing_len, TriMesh[lathe_mesh(prof; nseg = nseg)])
    (merge_meshes(parts...), sections)
end
