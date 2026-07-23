# Aerodynamics from geometry: modified-Newtonian panel method.
#
# For hypersonic flow the local pressure coefficient on a windward panel is
#   Cp = Cp_max(M) · sin²θ,     θ = local impact angle,
# with Cp_max from the stagnation pressure behind a normal shock (the
# "modified" part — it recovers 1.839 at M→∞ for γ=1.4) and Cp = 0 in
# shadow. Integrating over the mesh gives CA/CN/Cm(M, α); a second sweep
# with a superimposed pitch rate (each panel sees the local velocity
# u∞ + ω×r) yields the damping derivative Cm_q the 6-DOF needs.
#
# Validity: Newtonian theory is a hypersonic approximation — good above
# Mach ~4-5 for blunt/slender bodies, meaningless subsonic. `PanelAero`
# therefore clamps Mach lookups at its lowest computed Mach and should be
# paired with drogue-class devices before the transonic regime, exactly as
# the capsule missions here do. Self-shadowing by OTHER parts of the body
# (concavities) is not occluded — panels are shadowed only by their own
# orientation — which is the standard first-order panel treatment.
#
# The α sweep is planar (wind tilted in the body x-y plane); using the
# resulting tables through the axisymmetric total-AoA formulation in
# `entry6.jl` is exact for bodies of revolution and a windward-meridian
# approximation for winged shapes like a Starship (full (α, β) maps are the
# documented next step).

"Modified-Newtonian stagnation pressure coefficient at Mach `M` (γ = 1.4)."
function cp_max_newtonian(M::Float64; gamma::Float64 = GAMMA_AIR)
    M = max(M, 1.05)
    g = gamma
    p02_pinf = ((g + 1)^2 * M^2 / (4g * M^2 - 2(g - 1)))^(g / (g - 1)) *
               ((1 - g + 2g * M^2) / (g + 1))
    (p02_pinf - 1) / (0.5g * M^2)
end

"""
    PanelAero <: AbstractAeroDatabase

Mesh-derived aero tables: CA/CN/Cm on an (α, Mach) grid plus Cm_q(M),
about the reference point `ref` (normally the CG) with `sref`/`lref`.
Implements the same `cd_coeff`/`cl_coeff`/`cm_coeff` interface as
`CapsuleAero`, so it plugs straight into the 4-DOF and 6-DOF entry sims.
"""
struct PanelAero <: AbstractAeroDatabase
    alphas::Vector{Float64}
    machs::Vector{Float64}
    ca::Matrix{Float64}      # [alpha, mach]
    cn::Matrix{Float64}
    cm::Matrix{Float64}
    cmq::Table1D             # vs mach
    sref::Float64
    lref::Float64
    ref::V3
    alpha_trim::Float64      # kept for interface parity (0 for symmetric bodies)
end

"Bilinear clamped interpolation on the (alpha, mach) grid (degenerate axes OK)."
function _interp2(as, ms, tab, a, m)
    a = clamp(a, as[1], as[end]); m = clamp(m, ms[1], ms[end])
    if length(as) == 1
        i, fa = 1, 0.0
        i1 = 1
    else
        i = clamp(searchsortedlast(as, a), 1, length(as) - 1)
        fa = (a - as[i]) / (as[i+1] - as[i])
        i1 = i + 1
    end
    if length(ms) == 1
        j, fm = 1, 0.0
        j1 = 1
    else
        j = clamp(searchsortedlast(ms, m), 1, length(ms) - 1)
        fm = (m - ms[j]) / (ms[j+1] - ms[j])
        j1 = j + 1
    end
    (tab[i, j] * (1 - fa) + tab[i1, j] * fa) * (1 - fm) +
    (tab[i, j1] * (1 - fa) + tab[i1, j1] * fa) * fm
end

function cd_coeff(a::PanelAero, M::Float64, alpha::Float64)
    al = abs(alpha)
    ca = _interp2(a.alphas, a.machs, a.ca, al, M)
    cn = _interp2(a.alphas, a.machs, a.cn, al, M)
    ca * cos(al) + cn * sin(al)
end

function cl_coeff(a::PanelAero, M::Float64, alpha::Float64)
    al = abs(alpha)
    ca = _interp2(a.alphas, a.machs, a.ca, al, M)
    cn = _interp2(a.alphas, a.machs, a.cn, al, M)
    (cn * cos(al) - ca * sin(al)) * sign(alpha)
end

function cm_coeff(a::PanelAero, M::Float64, alpha::Float64, qhat::Float64)
    cm = _interp2(a.alphas, a.machs, a.cm, abs(alpha), M) * sign(alpha)
    cm + interp1(a.cmq, M) * qhat
end

"""
    panel_aero(mesh; sref, lref, ref, machs=[4,6,10,20,30],
               alphas=deg2rad_.(0:5:180)) -> PanelAero

Build the aero database from a mesh. `ref` is the moment reference (use the
CG from `mass_properties`). Convention: body +x is the nose/velocity axis;
α tilts the wind in the x-y plane toward +y.
"""
function panel_aero(mesh::TriMesh; sref::Float64, lref::Float64, ref::V3,
                    machs::Vector{Float64} = [4.0, 6.0, 10.0, 20.0, 30.0],
                    alphas::Vector{Float64} = collect(deg2rad_.(0.0:5.0:180.0)))
    # precompute panel centroids, unit normals, areas
    n = length(mesh)
    cen = Vector{V3}(undef, n); nrm = Vector{V3}(undef, n); area = zeros(n)
    for (k, t) in enumerate(mesh.tris)
        cen[k] = vscale(vadd(vadd(t[1], t[2]), t[3]), 1/3)
        n2 = face_normal2(t)
        a2 = vnorm(n2)
        area[k] = 0.5 * a2
        nrm[k] = a2 > 0 ? vscale(n2, 1 / a2) : (0.0, 0.0, 1.0)
    end

    # force/moment coefficients for freestream direction uhat (unit, points
    # from the body INTO the oncoming flow reversed — i.e. flow velocity
    # direction), optionally with a pitch rate about `axis` through `ref`
    function coeffs(uhat::V3, cpmax::Float64, wvec::V3)
        F = (0.0, 0.0, 0.0); Mm = (0.0, 0.0, 0.0)
        for k in 1:n
            u = uhat
            s = 1.0
            if wvec != (0.0, 0.0, 0.0)
                # air relative to a panel on the rotating body: u∞ - (ω/V)×r
                ul = vsub(uhat, vcross(wvec, vsub(cen[k], ref)))
                un = vnorm(ul)
                u = vscale(ul, 1 / un)
                s = un * un              # local dynamic-pressure scale
            end
            ct = vdot(u, nrm[k])
            if ct < 0.0                  # windward
                cp = cpmax * ct * ct * s
                dF = vscale(nrm[k], -cp * area[k])
                F = vadd(F, dF)
                Mm = vadd(Mm, vcross(vsub(cen[k], ref), dF))
            end
        end
        (F, Mm)
    end

    na, nm = length(alphas), length(machs)
    CA = zeros(na, nm); CN = zeros(na, nm); CM = zeros(na, nm)
    CMQ = zeros(nm)
    for (j, M) in enumerate(machs)
        cpm = cp_max_newtonian(M)
        for (i, al) in enumerate(alphas)
            # wind blows along -x at α=0, tilted toward -y with α
            u = (-cos(al), -sin(al), 0.0)
            F, Mm = coeffs(u, cpm, (0.0, 0.0, 0.0))
            CA[i, j] = -F[1] / sref                       # axial (+ = drag at α=0)
            CN[i, j] = -F[2] / sref                       # normal (+ toward wind side)
            # pitch moment about the axis ê = v̂_b × x̂ (entry6's convention):
            # v̂_b = -u, so ê = (-u) × x̂
            e = vcross(vscale(u, -1.0), (1.0, 0.0, 0.0))
            en = vnorm(e)
            CM[i, j] = en > 1e-9 ? vdot(Mm, vscale(e, 1 / en)) / (sref * lref) : 0.0
        end
        # damping at a representative small α: superimpose pitch rate qhat
        al = deg2rad_(10.0)
        u = (-cos(al), -sin(al), 0.0)
        e = vunit(vcross(vscale(u, -1.0), (1.0, 0.0, 0.0)))
        qhat = 0.02                       # nondimensional q·Lref/(2V)
        w = vscale(e, qhat * 2 / lref)    # per unit V
        _, M0 = coeffs(u, cpm, (0.0, 0.0, 0.0))
        _, M1 = coeffs(u, cpm, w)
        CMQ[j] = (vdot(M1, e) - vdot(M0, e)) / (sref * lref) / qhat
    end

    PanelAero(alphas, machs, CA, CN, CM, Table1D(machs, CMQ), sref, lref, ref, 0.0)
end

"""
    trim_alpha(a::PanelAero; mach=20.0) -> Float64

First zero crossing of Cm(α) with restoring slope (Cm' < 0), or NaN if the
body has no stable trim in the sweep. α = 0 counts when Cm(0)≈0 and the
slope is restoring.
"""
function trim_alpha(a::PanelAero; mach::Float64 = 20.0)
    cm(al) = cm_coeff(a, mach, al, 0.0)
    as = a.alphas
    c0 = cm(as[1] + 1e-6)
    for i in 2:length(as)
        c1 = cm(as[i])
        if c0 >= 0 && c1 < 0            # downward zero crossing: restoring
            lo, hi = as[i-1], as[i]
            for _ in 1:40
                mid = 0.5 * (lo + hi)
                cm(mid) >= 0 ? (lo = mid) : (hi = mid)
            end
            return 0.5 * (lo + hi)
        end
        c0 = c1
    end
    abs(cm(1e-6)) < 1e-4 && cm(deg2rad_(2.0)) < 0 ? 0.0 : NaN
end