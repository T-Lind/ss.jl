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
    _cluster(n, rmax, rb_max) -> [(y, z, exit_radius)]

Lay out `n` engine bells inside radius `rmax`, packed so they do not
overlap: one on the axis, a plain ring, or a ring around a centre engine
once there are five or more and the count is odd (the octaweb arrangement).
Bells shrink as the count grows, which is what real clusters do.
"""
function _cluster(n::Int, rmax::Float64, rb_max::Float64)
    n <= 1 && return [(0.0, 0.0, rb_max)]
    centre = isodd(n) && n >= 5
    m = centre ? n - 1 : n
    s = sin(pi / m)
    rr = rmax / (1 + s)
    rb = min(rb_max, 0.92 * rr * s)
    centre && (rb = min(rb, 0.5rr))
    out = [(rr * cos(2pi * (i - 1) / m + pi / m),
            rr * sin(2pi * (i - 1) / m + pi / m), rb) for i in 1:m]
    centre && pushfirst!(out, (0.0, 0.0, rb))
    out
end

# ------------------------------------------------------------ capsule kit --
#
# The crew pod is the one part of the stack you can be inside, so it is built
# from primitives that carry a real wall: `_shell_mesh` revolves a contour and
# offsets it along its own surface normal, cutting watertight apertures for
# windows and rimming them. Everything below stays a closed solid, so mass
# properties and the section-volume tests keep working.

"Rotate about +z by `pitch`, then about +x by `roll`, then translate."
function _place_mesh(m::TriMesh; roll::Float64 = 0.0, pitch::Float64 = 0.0,
                     shift::V3 = (0.0, 0.0, 0.0))
    cp, sp = cos(pitch), sin(pitch)
    cr, sr = cos(roll), sin(roll)
    function f(v)
        x, y, z = v
        x1, y1 = cp * x - sp * y, sp * x + cp * y
        y2, z2 = cr * y1 - sr * z, sr * y1 + cr * z
        (x1 + shift[1], y2 + shift[2], z2 + shift[3])
    end
    TriMesh([(f(t[1]), f(t[2]), f(t[3])) for t in m.tris])
end

_negv(v::V3) = (-v[1], -v[2], -v[3])

"Emit a quad as two triangles wound so the face normal follows `outward`."
function _quad!(tris::Vector{NTuple{3,V3}}, p::V3, q::V3, r::V3, s::V3, outward::V3)
    if vdot(vcross(vsub(q, p), vsub(r, p)), outward) >= 0
        push!(tris, (p, q, r)); push!(tris, (p, r, s))
    else
        push!(tris, (p, r, q)); push!(tris, (p, s, r))
    end
end

"True if angle `a` lies in the arc from `a0` counter-clockwise to `a1`."
_ang_in(a, a0, a1) = mod(a - a0, 2pi) <= mod(a1 - a0, 2pi)

"Wall half-angle of an interstage transition cone [rad] — shallow, as built."
const TAPER_HALFANGLE = deg2rad(17.0)

"""
    _taper_len(r, rj, D) -> Float64

Axial length of the cone that takes a stage of radius `r` to the joint radius
`rj` of the stage above, holding the wall angle at [`TAPER_HALFANGLE`](@ref)
in either direction (necking down to a narrower upper stage, or flaring out
to a wider one). Returns 0 when the two radii are within 2%, so a uniform
stack keeps its plain barrel-to-barrel joint.
"""
function _taper_len(r::Float64, rj::Float64, D::Float64)
    dr = abs(rj - r)
    dr < 0.02r && return 0.0
    clamp(dr / tan(TAPER_HALFANGLE), 0.10D, 6.0D)
end

"""
    interstage_length(d_below, d_above) -> Float64

Length [m] of the transition cone that joins a stage of diameter `d_below` to
the stage of diameter `d_above` above it — zero when the two match, and the
same for a flare as for a neck of equal size. This is structure the stack
carries in addition to its tanks; see [`rocket_mesh`](@ref).
"""
interstage_length(d_below::Float64, d_above::Float64) =
    _taper_len(d_below / 2, d_above / 2, d_below)

"""
    _arc_slab(x0, x1, r0, r1, a0, a1; nseg=10) -> TriMesh

Closed solid spanning `x0..x1` axially, `r0..r1` radially and the angular
sector `a0..a1` about +x — curved cabin panels, equipment racks, consoles.
"""
function _arc_slab(x0, x1, r0, r1, a0, a1; nseg::Int = 10)
    na = max(2, nseg)
    th = [a0 + (a1 - a0) * k / na for k in 0:na]
    P(x, r, k) = (x, r * cos(th[k+1]), r * sin(th[k+1]))
    rad(k) = (0.0, cos(th[k+1]), sin(th[k+1]))
    tng(k) = (0.0, -sin(th[k+1]), cos(th[k+1]))
    tris = NTuple{3,V3}[]
    for k in 0:na-1
        _quad!(tris, P(x0,r1,k), P(x1,r1,k), P(x1,r1,k+1), P(x0,r1,k+1), rad(k))
        _quad!(tris, P(x0,r0,k), P(x1,r0,k), P(x1,r0,k+1), P(x0,r0,k+1), _negv(rad(k)))
        _quad!(tris, P(x0,r0,k), P(x0,r1,k), P(x0,r1,k+1), P(x0,r0,k+1), (-1.0,0.0,0.0))
        _quad!(tris, P(x1,r0,k), P(x1,r1,k), P(x1,r1,k+1), P(x1,r0,k+1), (1.0,0.0,0.0))
    end
    _quad!(tris, P(x0,r0,0),  P(x1,r0,0),  P(x1,r1,0),  P(x0,r1,0),  _negv(tng(0)))
    _quad!(tris, P(x0,r0,na), P(x1,r0,na), P(x1,r1,na), P(x0,r1,na), tng(na))
    ensure_outward(TriMesh(tris))
end

"""
    _shell_mesh(prof, thick; nseg=24, holes=()) -> TriMesh

Watertight shell of revolution. `prof` is the outer `(x, radius)` contour;
the inner surface is offset by `thick` along the local surface normal
(negative offsets outward, giving a raised collar or panel). `holes` is a
list of `(cell_lo, cell_hi, a0, a1)` apertures — a profile-cell range and an
angular sector — cut clean through the wall and rimmed all the way round.

Because the sector test wraps, passing `(1, ncell, a1, a0)` removes
*everything except* `a0..a1`, which is how a curved panel that follows a
cone is made: a shell that only exists where you want material.
"""
function _shell_mesh(prof::Vector{Tuple{Float64,Float64}}, thick::Float64;
                     nseg::Int = 24, holes = NTuple{4,Float64}[])
    n = length(prof)
    n >= 2 || throw(ArgumentError("shell profile needs at least two points"))
    m = n - 1                                     # axial cells
    inw = Vector{Tuple{Float64,Float64}}(undef, n)
    for i in 1:n
        i0, i1 = max(1, i - 1), min(n, i + 1)
        tx = prof[i1][1] - prof[i0][1]
        tr = prof[i1][2] - prof[i0][2]
        L = hypot(tx, tr)
        if L < 1e-12
            tx, tr, L = 1.0, 0.0, 1.0
        end
        inw[i] = (tr / L, -tx / L)                # inward = -(outward normal)
    end
    # angular indices wrap exactly: sin(2pi) is not 0 in binary, so the seam
    # would otherwise be a hairline crack rather than a shared edge
    th = [2pi * k / nseg for k in 0:nseg-1]
    ct(k) = 2pi * (k + 0.5) / nseg                # cell-centre angle
    O(i, k) = (prof[i][1], prof[i][2] * cos(th[mod(k,nseg)+1]),
                           prof[i][2] * sin(th[mod(k,nseg)+1]))
    function Q(i, k)
        x = prof[i][1] + thick * inw[i][1]
        r = max(prof[i][2] + thick * inw[i][2], 1e-4)
        (x, r * cos(th[mod(k,nseg)+1]), r * sin(th[mod(k,nseg)+1]))
    end
    cut(i, k) = any(h -> i >= h[1] && i <= h[2] && _ang_in(ct(k), h[3], h[4]), holes)
    solid(i, k) = 1 <= i <= m && !cut(i, mod(k, nseg))
    # a negative offset raises the wall outward, which swaps which of the two
    # revolved surfaces faces out of the solid
    sg = thick >= 0 ? 1.0 : -1.0

    tris = NTuple{3,V3}[]
    for i in 1:m, k in 0:nseg-1
        solid(i, k) || continue
        ca, sa = cos(ct(k)), sin(ct(k))
        ox = -(inw[i][1] + inw[i+1][1]); orr = -(inw[i][2] + inw[i+1][2])
        ref = (sg * ox, sg * orr * ca, sg * orr * sa)
        _quad!(tris, O(i,k), O(i,k+1), O(i+1,k+1), O(i+1,k), ref)
        _quad!(tris, Q(i,k), Q(i,k+1), Q(i+1,k+1), Q(i+1,k), _negv(ref))
    end
    for i in 1:m+1, k in 0:nseg-1                 # rims across profile stations
        a, b = solid(i - 1, k), solid(i, k)
        a == b && continue
        j = clamp(i, 1, n)
        ca, sa = cos(ct(k)), sin(ct(k))
        t3 = (-inw[j][2], inw[j][1] * ca, inw[j][1] * sa)
        _quad!(tris, O(j,k), O(j,k+1), Q(j,k+1), Q(j,k), a ? t3 : _negv(t3))
    end
    for i in 1:m, k in 0:nseg-1                   # rims along aperture sides
        a, b = solid(i, k - 1), solid(i, k)
        a == b && continue
        ca, sa = cos(th[mod(k,nseg)+1]), sin(th[mod(k,nseg)+1])
        tg = (0.0, -sa, ca)
        _quad!(tris, O(i,k), O(i+1,k), Q(i+1,k), Q(i,k), a ? tg : _negv(tg))
    end
    ensure_outward(TriMesh(tris))
end

"""
    pod_mesh(; radius=0.75, nseg=24, ncrew=0) -> (hull, glass, cabin, height)

Apollo-proportioned crew capsule: a spherical-section ablative heat shield,
a 32.5° conical afterbody built as a real pressure shell with three glazed
window apertures, a side hatch, RCS quads and a forward docking tunnel —
plus the cabin behind it all: deck, crew couches, main display console and
equipment racks. Each return is a vector of closed solids. The window panes
come back separately from the hull so a viewer can drop them and look out
through the apertures from inside the cabin.

Sits with the heat-shield apex at x = 0 and the tunnel at x = `height`;
`ncrew` defaults to what the diameter can actually seat.
"""
function pod_mesh(; radius::Float64 = 0.75, nseg::Int = 24, ncrew::Int = 0)
    rp = radius
    nc = max(16, nseg)
    Rs = 2.4rp                                    # heat-shield spherical radius
    xsh = Rs - sqrt(Rs^2 - rp^2)                  # shoulder station
    ts = 0.055rp                                  # ablator thickness
    tw = 0.050rp                                  # pressure-wall thickness
    ta = tan(deg2rad(32.5))                       # afterbody half-angle
    rf = 0.26rp                                   # forward radius
    Lc = (rp - rf) / ta
    xb0 = xsh + 0.05rp                            # hull base rim
    xtop = xb0 + Lc
    hgt = xtop + 0.34rp
    crew = ncrew > 0 ? ncrew : (rp >= 1.10 ? 3 : rp >= 0.85 ? 2 : 1)
    ext = TriMesh[]; glass = TriMesh[]; cab = TriMesh[]

    # --- heat shield: spherical cap, ablator thickness, closed at the axis --
    shield = Tuple{Float64,Float64}[]
    for f in range(0.0, 1.0; length = 9)
        u = rp * f
        push!(shield, (Rs - sqrt(max(Rs^2 - u^2, 0.0)), u))
    end
    push!(shield, (xsh + 0.06rp, rp))
    for f in range(1.0, 0.0; length = 9)
        u = 0.985rp * f
        push!(shield, (ts + Rs - sqrt(max(Rs^2 - u^2, 0.0)), u))
    end
    push!(ext, lathe_mesh(shield; nseg = nc))

    # --- conical pressure hull with glazed apertures ------------------------
    NB = 12
    cone = Tuple{Float64,Float64}[(xb0, rp)]
    for i in 1:NB
        f = i / NB
        push!(cone, (xb0 + f * Lc, rp + (rf - rp) * f))
    end
    rcone(x) = rp - ta * (x - xb0)
    wins = [(0.0, deg2rad(22.0)), (deg2rad(68.0), deg2rad(15.0)),
            (deg2rad(-68.0), deg2rad(15.0))]     # (centre, half-width)
    wi0, wi1 = 5, 7                              # window cell band
    hatch = (pi, deg2rad(42.0))
    holes = NTuple{4,Float64}[(Float64(wi0), Float64(wi1), w[1] - w[2], w[1] + w[2])
                              for w in wins]
    push!(ext, _shell_mesh(cone, tw; nseg = nc, holes = holes))

    # window frames (raised collar around each pane) and the panes themselves
    for w in wins
        fr = cone[wi0-1:wi1+2]
        ncell = length(fr) - 1
        push!(ext, _shell_mesh(fr, -0.030rp; nseg = nc, holes = NTuple{4,Float64}[
            (1.0, Float64(ncell), w[1] + w[2] + 0.10, w[1] - w[2] - 0.10),
            (2.0, Float64(ncell - 1), w[1] - w[2], w[1] + w[2])]))
        pn = cone[wi0:wi1+1]
        push!(glass, _shell_mesh(pn, 0.012rp; nseg = nc, holes = NTuple{4,Float64}[
            (1.0, Float64(length(pn) - 1), w[1] + w[2], w[1] - w[2])]))
    end
    # side hatch: raised panel over its own sector, with a small port
    hh = cone[4:9]
    nhc = length(hh) - 1
    push!(ext, _shell_mesh(hh, -0.026rp; nseg = nc, holes = NTuple{4,Float64}[
        (1.0, Float64(nhc), hatch[1] + hatch[2], hatch[1] - hatch[2])]))

    # RCS quads on the upper cone: a housing with fore- and aft-firing nozzles
    xq = xb0 + 0.80Lc
    rq = rcone(xq)
    for a in (0.25pi):(0.5pi):(1.99pi)
        push!(ext, _place_mesh(box_mesh((xq - 0.10rp, rq - 0.02rp, -0.07rp),
                                        (xq + 0.10rp, rq + 0.05rp, 0.07rp));
                               roll = a))
        for (sg, pit) in ((-1.0, 0.0), (1.0, Float64(pi)))
            noz = _place_mesh(_bell_mesh(0.0, 0.055rp, 0.030rp, 0.014rp, 0.0, 0.0;
                                         nseg = 10);
                              pitch = pit, shift = (xq + sg * 0.105rp, rq + 0.02rp, 0.0))
            push!(ext, _place_mesh(noz; roll = a))
        end
    end
    # forward compartment and docking tunnel
    push!(ext, lathe_mesh(Tuple{Float64,Float64}[
        (xtop - 0.02rp, 0.0), (xtop - 0.02rp, rf),
        (xtop + 0.10rp, 0.245rp), (xtop + 0.16rp, 0.225rp),
        (xtop + 0.16rp, 0.0)]; nseg = nc))
    push!(ext, lathe_mesh(Tuple{Float64,Float64}[
        (xtop + 0.14rp, 0.0), (xtop + 0.14rp, 0.215rp),
        (hgt - 0.05rp, 0.200rp), (hgt - 0.05rp, 0.240rp),
        (hgt, 0.240rp), (hgt, 0.0)]; nseg = nc))

    # --- cabin --------------------------------------------------------------
    # every fitting is sized against the pressure wall where it actually sits,
    # so nothing punches through the cone as the capsule is scaled
    rin(x) = rcone(x) - tw
    xfl = xb0 + 0.04rp                            # deck
    push!(cab, lathe_mesh(Tuple{Float64,Float64}[
        (xfl, 0.0), (xfl, 0.96rin(xfl + 0.045rp)),
        (xfl + 0.045rp, 0.96rin(xfl + 0.045rp)), (xfl + 0.045rp, 0.0)]; nseg = nc))
    xa = xfl + 0.045rp
    zs = crew == 1 ? [0.0] : crew == 2 ? [-0.30rp, 0.30rp] : [-0.44rp, 0.0, 0.44rp]
    hw = crew >= 3 ? 0.14rp : 0.17rp              # couch half-width
    ln = 0.60 * rin(xa + 0.07rp)                  # couch half-length
    for zc in zs
        push!(cab, box_mesh((xa + 0.02rp, -ln,      zc - hw),
                            (xa + 0.07rp,  0.36ln,  zc + hw)))         # back pan
        push!(cab, box_mesh((xa + 0.07rp, 0.19ln, zc - 0.78hw),
                            (xa + 0.15rp, 0.41ln, zc + 0.78hw)))       # headrest
        push!(cab, _place_mesh(box_mesh((-0.025rp, -0.20ln, -0.88hw),
                                        ( 0.025rp,  0.20ln,  0.88hw));
                               pitch = deg2rad(-50.0),
                               shift = (xa + 0.10rp, -1.06ln, zc)))    # leg rest
        for sg in (-1.0, 1.0)                                          # side rails
            push!(cab, box_mesh((xa + 0.05rp, -0.96ln, zc + sg * hw - 0.022rp),
                                (xa + 0.13rp,  0.31ln, zc + sg * hw + 0.022rp)))
        end
        for (sy, sg) in ((-0.84, -1.0), (-0.84, 1.0), (0.24, -1.0), (0.24, 1.0))
            push!(cab, box_mesh((xfl + 0.045rp, sy * ln - 0.022rp,
                                 zc + sg * hw * 0.8 - 0.022rp),
                                (xa + 0.02rp,   sy * ln + 0.022rp,
                                 zc + sg * hw * 0.8 + 0.022rp)))       # struts
        end
    end
    # main display console: an annular panel facing the crew, with instruments
    xcon = xb0 + 0.62Lc
    rcin = rin(xcon)
    push!(cab, lathe_mesh(Tuple{Float64,Float64}[
        (xcon, 0.20rcin), (xcon, 0.92rcin), (xcon + 0.05rp, 0.92rcin),
        (xcon + 0.05rp, 0.20rcin), (xcon, 0.20rcin)]; nseg = nc))
    for (k, a) in enumerate(range(-1.05, 1.05; length = 5))
        cy, cz = 0.58rcin * cos(a), 0.58rcin * sin(a)
        push!(cab, box_mesh((xcon - 0.035rp, cy - 0.10rcin, cz - 0.13rcin),
                            (xcon,           cy + 0.10rcin, cz + 0.13rcin)))
        isodd(k) && push!(cab, _arc_slab(xcon - 0.020rp, xcon, 0.26rcin, 0.35rcin,
                                         a - 0.16, a + 0.16; nseg = 6))
    end
    # equipment racks against the cabin wall, clear of the couches
    for (a0, a1) in ((deg2rad(100.0), deg2rad(136.0)), (deg2rad(224.0), deg2rad(260.0)))
        x0r, x1r = xa, xa + 0.26rp
        push!(cab, _arc_slab(x0r, x1r, 0.72rin(x1r), 0.97rin(x1r), a0, a1; nseg = 8))
        x2r, x3r = x1r + 0.04rp, x1r + 0.30rp
        push!(cab, _arc_slab(x2r, x3r, 0.68rin(x3r), 0.96rin(x3r),
                             a0 + 0.08, a1 - 0.08; nseg = 8))
    end

    # --- fit-out ------------------------------------------------------------
    # Wall lining, in two bands that deliberately stop short of the glazing:
    # the panes sit on cone cells 5-7, so a liner from the deck to 0.32 Lc and
    # another from 0.60 Lc upward leaves every viewport clear while giving the
    # pressure shell an inside face of its own. Without it the cabin is read
    # through the back of the outer cone, which is why it looked like a tent.
    xw0, xw1 = xb0 + 0.32Lc, xb0 + 0.60Lc
    for (xa_, xb_) in ((xfl + 0.05rp, xw0), (xw1, xtop - 0.10rp))
        # the profile has to CLOSE — lathe_mesh sweeps a loop, and an open one
        # is a surface with a seam, not a solid: its signed volume is garbage
        ro, ri = 0.985rin(xb_), 0.945rin(xb_)
        push!(cab, lathe_mesh(Tuple{Float64,Float64}[
            (xa_, ro), (xa_, ri), (xb_, ri), (xb_, ro), (xa_, ro)]; nseg = nc))
    end
    # Handrails. A crew member in zero g moves by pulling on these, so they run
    # the full height of the cabin and stand proud of the wall by a hand's
    # width. Placed on the rolls between the viewports, the side hatch and the
    # equipment racks, which is the only clear real estate there is.
    for a in (deg2rad(34.0), deg2rad(90.0), deg2rad(270.0), deg2rad(326.0))
        for (x0h, x1h) in ((xa + 0.05rp, xw0 - 0.03rp), (xw1 + 0.03rp, xtop - 0.14rp))
            # size every radius off the NARROW end: the wall tapers along the
            # rail, so a radius taken at the middle punches out through the top
            rr = 0.88rin(x1h)
            push!(cab, _place_mesh(box_mesh((x0h, rr - 0.018rp, -0.018rp),
                                            (x1h, rr + 0.018rp, 0.018rp));
                                   roll = a))
            for xs in (x0h, x1h)                       # stand-offs to the wall
                push!(cab, _place_mesh(box_mesh((xs - 0.014rp, rr, -0.012rp),
                                                (xs + 0.014rp, 0.96rin(xs + 0.02rp),
                                                 0.012rp));
                                       roll = a))
            end
        end
    end
    # Overhead stowage: lockers ringing the upper cone where it narrows toward
    # the tunnel, with a recessed door face so they read as lockers and not as
    # a band of wall.
    for a0 in (deg2rad(6.0), deg2rad(78.0), deg2rad(150.0), deg2rad(222.0),
               deg2rad(294.0))
        a1 = a0 + deg2rad(56.0)
        xl0, xl1 = xtop - 0.36rp, xtop - 0.13rp
        push!(cab, _arc_slab(xl0, xl1, 0.70rin(xl1), 0.96rin(xl1), a0, a1; nseg = 7))
        push!(cab, _arc_slab(xl0 - 0.018rp, xl0, 0.74rin(xl1), 0.92rin(xl1),
                             a0 + 0.05, a1 - 0.05; nseg = 6))
    end
    # Inner hatch surround on the wall the outside hatch is cut into, and a
    # grab loop over the tunnel where the crew pull themselves through.
    let ah = pi, dh = deg2rad(38.0), xh0 = xb0 + 0.26Lc, xh1 = xb0 + 0.72Lc
        push!(cab, _arc_slab(xh0, xh1, 0.93rin(xh1), 0.985rin(xh1),
                             ah - dh, ah - dh + 0.09; nseg = 3))
        push!(cab, _arc_slab(xh0, xh1, 0.93rin(xh1), 0.985rin(xh1),
                             ah + dh - 0.09, ah + dh; nseg = 3))
    end
    # the tunnel mouth necks the wall down to 0.21 rp, so the loops live well
    # inside that or they come out through the forward bulkhead
    for a in (0.0, 0.5pi, 1.0pi, 1.5pi)
        push!(cab, _place_mesh(box_mesh((xtop - 0.10rp, 0.105rp, -0.016rp),
                                        (xtop - 0.02rp, 0.180rp, 0.016rp));
                               roll = a))
    end
    (ext, glass, cab, hgt)
end

"""
    rocket_mesh(; diameter, diameters, prop_masses, densities, n_engines,
                  fairing_len, nseg) -> (mesh, sections)

Procedural launch-vehicle geometry with real detailing: a first stage with
an engine skirt and a five-bell cluster, one cylindrical barrel per stage
sized so its tank volume holds `prop_masses[k]` of propellant at
`densities[k]` (plus ullage and an engine/interstage bay), recessed
interstage collars with a nested vacuum bell on every upper stage, cable
raceways, RCS pods and a payload adapter cone on the kick stage, a crew
capsule (see [`pod_mesh`](@ref)) on the adapter, and an ogive fairing
enclosing both. Body +x is the nose axis; the tail plate sits at x = 0 with
the first-stage bells extending to x ≈ -0.32·diameter.

Each piece is a closed solid, so `sections` carries both the axial extent
and the triangle range of every component in the merged soup:
`(name, x0, x1, t0, t1)` bottom-up, ending with the capsule's three parts —
`:pod` (hull), `:glass` (window panes) and `:cabin` (interior) — then
`:fairing`. Viewers hide or detach a section by dropping `tris[t0:t1]`:
drop `:glass` and `:cabin` becomes visible through the apertures.

`boosters` adds strap-on sets clustered around the first stage, each entry a
named tuple `(count, diameter, prop_mass, density, n_engines)`. A set is one
section named `:booster<i>`, appended after the core stack — so the section
list is bottom-up for the stack itself, with strap-ons last.

Stages may have their own diameters via `diameters`. A barrel's length
follows from its own cross-section, so widening a stage makes it shorter for
the same propellant load, and where two neighbours differ the lower one is
capped with a transition cone ([`_taper_len`](@ref)) carrying its radius to
the joint — necking down to a narrower upper stage or flaring out to a wider
one, at a constant shallow wall angle either way. The cone sits above the
tank, so it lengthens the stack without eating tank volume, and it belongs to
the lower stage's section: it departs at separation, as a real interstage
does. Uniform stacks emit no cone and are unchanged. The fairing and capsule
are sized by the topmost stage.
"""
function rocket_mesh(; diameter::Float64 = 1.8,
                     prop_masses::Vector{Float64} = [42000.0, 9500.0, 950.0],
                     densities::Vector{Float64} = fill(bulk_density(PROPELLANTS[:kerolox]),
                                                       length(prop_masses)),
                     n_engines::Vector{Int} = ones(Int, length(prop_masses)),
                     diameters::Vector{Float64} = fill(diameter, length(prop_masses)),
                     boosters::Vector = NamedTuple[],
                     fairing_len::Float64 = 2.2 * last(diameters),
                     nseg::Int = 48)
    length(prop_masses) == length(densities) ||
        throw(ArgumentError("prop_masses and densities length mismatch"))
    length(n_engines) == length(prop_masses) ||
        throw(ArgumentError("n_engines and prop_masses length mismatch"))
    length(diameters) == length(prop_masses) ||
        throw(ArgumentError("diameters and prop_masses length mismatch"))
    all(>(0), diameters) || throw(ArgumentError("stage diameters must be positive"))
    K = length(prop_masses)
    D = diameters[1]
    r = D / 2
    A = pi * r^2
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
    raceway(xa, xb, r, D) = box_mesh((xa, 0.955r, -0.030D), (xb, r + 0.048D, 0.030D))

    x = 0.28D                                    # engine-skirt cone length
    rtop_prev = D / 2                            # radius at the top of stage k-1
    for (k, (mp, rho, ne)) in enumerate(zip(prop_masses, densities, n_engines))
        D = diameters[k]                         # this stage's own diameter
        r = D / 2
        A = pi * r^2
        len = mp / (rho * A) * 1.15 + 0.9D       # tank + ullage + engine bay
        # Transition to the stage above: the adapter belongs to the lower
        # stage (it is the top of its structure and departs with it), and is
        # a cone whenever the two diameters differ — necking down or flaring
        # out. It is appended above the barrel, so tank volume is untouched.
        rj = k < K ? diameters[k+1] / 2 : r      # joint radius with the stage above
        ltap = _taper_len(r, rj, D)
        xtop = x + len + ltap                    # top of this stage's structure
        ms = TriMesh[]
        if k == 1
            # skirt + barrel, engine cluster half-recessed below the plate
            prof1 = Tuple{Float64,Float64}[
                (0.0, 0.0), (0.0, 0.80r), (0.28D, r), (x + len, r)]
            ltap > 0 && push!(prof1, (xtop, rj))
            push!(prof1, (xtop, 0.0))
            push!(ms, lathe_mesh(prof1; nseg = nseg))
            for (by, bz, bs) in _cluster(ne, 0.80r, 0.105D)
                push!(ms, _bell_mesh(-0.32D, 4.0bs, bs, 0.48bs, by, bz; nseg = nb))
            end
            push!(ms, raceway(0.30D, x + len - 0.02D, r, D))
            finish!(:stage1, -0.32D, xtop, ms)
        else
            # Interstage collar: tucked inside whatever the stage below ends
            # at, then opened out to this stage's own radius.
            rbase = min(0.945r, 0.98 * rtop_prev)
            profk = Tuple{Float64,Float64}[
                (x, 0.0), (x, rbase), (x + 0.10D, rbase), (x + 0.13D, r),
                (x + len, r)]
            ltap > 0 && push!(profk, (xtop, rj))
            push!(profk, (xtop, 0.0))
            push!(ms, lathe_mesh(profk; nseg = nseg))
            bex, blen, thr = k == K ? (0.085D, 0.22D, 0.53) : (0.155D, 0.36D, 0.29)
            for (by, bz, bs) in _cluster(ne, 0.80r, bex)
                L = blen * bs / bex
                push!(ms, _bell_mesh(x + 0.04D - L, L, bs, thr * bs, by, bz; nseg = nb))
            end
            k < K && push!(ms, raceway(x + 0.12D, x + len - 0.02D, r, D))
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
            finish!(Symbol(:stage, k), x, xtop + (k == K ? 0.13D : 0.0), ms)
        end
        x += len + ltap
        rtop_prev = ltap > 0 ? rj : r
    end
    D = diameters[K]                             # fairing and capsule ride on top
    r = D / 2
    # pod: crew capsule seated on the adapter, inside the fairing. The cabin
    # is its own section so a viewer can cull the hull and look inside; it
    # shares the pod's axial extent so `sections` stays sorted by x0.
    rp = min(0.75, 0.85r)
    xb = x + 0.13D + 0.02                        # a hair off the adapter face
    phull, pglass, pcab, lp = pod_mesh(; radius = rp, nseg = max(20, nseg ÷ 2))
    shift = (xb, 0.0, 0.0)
    place!(nm, ms) = finish!(nm, x, xb + lp,
                             TriMesh[_place_mesh(m; shift = shift) for m in ms])
    place!(:pod, phull)
    place!(:glass, pglass)
    place!(:cabin, pcab)
    #= fairing follows =#
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

    # --- strap-on boosters ------------------------------------------------
    # Clustered around the first stage, standing on the same plane, each a
    # barrel sized by its own propellant load with an ogive nose and its own
    # bells. A whole set is one section, because a set separates together.
    r1 = diameters[1] / 2
    for (bi, b) in enumerate(boosters)
        db = b.diameter
        rb = db / 2
        lb = b.prop_mass / (b.density * pi * rb^2) * 1.15 + 0.9db
        xn = 0.28db + lb                             # nose starts above the barrel
        bprof = Tuple{Float64,Float64}[(0.0, 0.0), (0.0, 0.80rb), (0.28db, rb), (xn, rb)]
        for f in range(0.0, 1.0; length = 8)[2:end-1]
            push!(bprof, (xn + f * 1.75rb, rb * (1 - f^2)^0.55))
        end
        push!(bprof, (xn + 1.75rb, 0.0))
        one = TriMesh[lathe_mesh(bprof; nseg = max(16, nseg ÷ 2))]
        for (by, bz, bs) in _cluster(b.n_engines, 0.78rb, 0.105db)
            push!(one, _bell_mesh(-0.30db, 3.6bs, bs, 0.46bs, by, bz; nseg = nb))
        end
        R = r1 + rb                                  # flank of the core, touching
        set = TriMesh[]
        for j in 0:(b.count - 1)
            a = 2pi * j / b.count
            sh = (0.0, R * cos(a), R * sin(a))
            append!(set, (_place_mesh(m; roll = a, shift = sh) for m in one))
        end
        finish!(Symbol(:booster, bi), -0.30db, xn + 1.75rb, set)
    end
    (merge_meshes(parts...), sections)
end

"""
    rocket_mesh(lv::LaunchVehicle; diameter, kwargs...) -> (mesh, sections)

The geometry a launch vehicle actually implies: every barrel is sized by
its own propellant's bulk density, and every stage gets its own engine
count. Swap a stage from kerolox to hydrolox and the same propellant mass
needs a three-times longer tank — the vehicle visibly grows.
"""
rocket_mesh(lv::LaunchVehicle; diameter::Float64 = 2 * sqrt(lv.sref / pi),
            kwargs...) =
    rocket_mesh(; diameter = diameter,
                prop_masses = [s.mprop for s in lv.stages],
                densities = [bulk_density(s.prop) for s in lv.stages],
                n_engines = [s.n_engines for s in lv.stages],
                diameters = [stage_diameter(s, diameter) for s in lv.stages],
                boosters = [(count = b.count,
                             diameter = stage_diameter(b.stage, diameter),
                             prop_mass = b.stage.mprop,
                             density = bulk_density(b.stage.prop),
                             n_engines = b.stage.n_engines) for b in lv.boosters],
                kwargs...)
