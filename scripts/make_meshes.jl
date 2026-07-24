# Generate the demo geometry: a blunt entry capsule (matching the reference
# pod's dimensions) and a simplified Starship-class vehicle, written as
# binary STL into geometry/.
#
# Usage: julia --project scripts/make_meshes.jl

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Printf

outdir = joinpath(@__DIR__, "..", "geometry")
mkpath(outdir)

# --- capsule: spherical-cap heatshield + conical afterbody -----------------
# body +x = velocity axis (heatshield faces +x); D = 1.5 m, Rn = 1.8 m
function capsule_mesh(; d = 1.5, rn = 1.8, nseg = 64)
    rmax = d / 2
    # heatshield: sphere of radius rn, cap subtending the shoulder
    smax = asin(rmax / rn)
    prof = Tuple{Float64,Float64}[]
    for s in range(0.0, smax; length = 13)
        push!(prof, (rn * (cos(s) - cos(smax)), rn * sin(s)))
    end
    reverse!(prof)                        # nose (max x) first -> march -x
    prof = [(x, r) for (x, r) in prof]
    x_shoulder = prof[end][1]             # = 0 at shoulder by construction
    # afterbody: 20 deg cone to a small flat back cover
    back_r = 0.25 * rmax
    x_back = x_shoulder - (rmax - back_r) / tan(deg2rad_(70.0))
    push!(prof, (x_back, back_r))
    push!(prof, (x_back, 0.0))
    prof2 = [(prof[1][1] + 1e-9, 0.0); prof]   # close the nose
    lathe_mesh(reverse(prof2); nseg = nseg)    # profile marching +x
end

cap = capsule_mesh()
write_stl(joinpath(outdir, "capsule.stl"), cap; name = "ss.jl capsule")
mp = mass_properties(cap, 350.0)
@printf("capsule : %4d tris  vol %.3f m³  cg x %.3f m  I (%.0f, %.0f, %.0f)  offdiag %.4f\n",
        length(cap), mp.volume, mp.cg[1], mp.inertia..., mp.offdiag_frac)

# --- simplified Starship: 9 m cylinder + ogive nose + 4 flaps --------------
# body +x = nose axis; flaps modeled as thin closed boxes on the -y side
# (the belly for the windward-meridian aero sweep)
function starship_mesh(; d = 9.0, len = 50.0, nose = 12.0, nseg = 64)
    r = d / 2
    prof = Tuple{Float64,Float64}[(len, 0.0)]
    for f in range(1.0, 0.0; length = 9)[2:end]      # ogive-ish nose
        x = len - nose * (1 - f)
        push!(prof, (x, r * sqrt(1 - f^2)))
    end
    push!(prof, (0.0, r))
    push!(prof, (0.0, 0.0))
    body = lathe_mesh(reverse(prof); nseg = nseg)
    # flaps: fore pair near the nose, aft pair at the base; span ±z,
    # protruding on the -y (belly) side, 0.3 m thick
    fl(x0, x1, span) = merge_meshes(
        box_mesh((x0, -r - 0.0, 1.0), (x1, -r + 0.3, 1.0 + span)),
        box_mesh((x0, -r - 0.0, -1.0 - span), (x1, -r + 0.3, -1.0)))
    fore = fl(len - nose - 6.0, len - nose - 1.0, 4.0)
    aft = fl(1.0, 9.0, 5.5)
    merge_meshes(body, fore, aft)
end

ship = starship_mesh()
write_stl(joinpath(outdir, "starship.stl"), ship; name = "ss.jl starship-demo")
mps = mass_properties(ship, 120_000.0)     # dry-ish + residuals, entry mass
@printf("starship: %4d tris  vol %.0f m³  cg x %.2f m  I (%.2e, %.2e, %.2e)  offdiag %.4f\n",
        length(ship), mps.volume, mps.cg[1], mps.inertia..., mps.offdiag_frac)

# --- Sable launcher: stacked stages + ogive fairing (panel visualizer) -----
# built from the reference vehicle itself, so tank lengths follow each
# stage's propellant density and the bells follow its engine count
rk, secs = rocket_mesh(default_moon_rocket())
write_stl(joinpath(outdir, "sable.stl"), rk; name = "ss.jl sable")
@printf("sable   : %4d tris  vol %.1f m³  length %.1f m  (%s)\n",
        length(rk), mesh_volume(rk), secs[end].x1,
        join(string.(s.name for s in secs), ", "))

println("wrote geometry/capsule.stl, geometry/starship.stl, geometry/sable.stl")
