# Design and fly the lunar landing mission — pad to the surface of the Moon and
# back to a splashdown — and write the trajectory products:
#   output/landing_descent.csv   powered descent, ignition to touchdown
#   output/landing_orbit.csv     lunar parking orbit + descent ellipse
#   output/landing_terrain.csv   ground profile along the final approach
#   output/landing_ascent.csv    lift-off to lunar orbit
#   output/landing_events.csv    combined event timeline
#   output/landing_summary.txt   console summary
#
# The landing is flown at full fidelity: procedural terrain, an oblate and
# mascon-lumped Moon, orbit-determination error corrected by landing radar, and
# hazard avoidance choosing the touchdown point at high gate. Set
# `SS_PLAIN=1` to fly the smooth-sphere version instead, which is what every
# result in this repo predating terrain was flown against.
#
# Usage: julia --project -t auto scripts/run_landing.jl

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Printf

outdir = joinpath(@__DIR__, "..", "output")
mkpath(outdir)

plain = get(ENV, "SS_PLAIN", "0") != "0"

lander = default_lander()
stage = AscentStage()
orbiter = Orbiter()
# The reference Sable cannot throw twelve tonnes anywhere; a Starship-class
# vehicle can throw the lander *and* the orbiter that brings the crew home.
# That is the whole story of why lunar landing missions look the way they do.
lv = starship_expendable(payload = lander_mass(lander) + orbiter_mass(orbiter))

println("Designing lunar landing mission", plain ? " (smooth sphere)" : "", "...")
t = @elapsed ls = plain ?
    moonlanding(lander = lander, lv = lv, orbiter = orbiter,
                kick_angle = deg2rad_(5.0)) :
    apollo_landing(lander = lander, lv = lv, orbiter = orbiter,
                   kick_angle = deg2rad_(5.0))
@printf("design + descent complete in %.1f s\n\n", t)
print_landing_summary(ls)

println("\nComing home...")
t = @elapsed rr = moonreturn(ls, stage = stage)
@printf("ascent + rendezvous + TEI + entry in %.1f s\n\n", t)
print_return_summary(rr, ls)

d = ls.descent.log
open(joinpath(outdir, "landing_descent.csv"), "w") do io
    println(io, "t_s,alt_m,downrange_m,v_ms,v_horiz_ms,v_vert_ms,mass_kg,throttle," *
                "pitch_deg,ground_elev_m,nav_alt_err_m,nav_pos_err_m,crossrange_m")
    for i in eachindex(d.t)
        @printf(io, "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.4f,%.3f,%.1f,%.2f,%.2f,%.2f\n",
                d.t[i], d.h[i], d.downrange[i], d.v[i], d.vh[i], d.vv[i],
                d.m[i], d.throttle[i], rad2deg_(d.pitch[i]), d.elev[i],
                d.nav_dh[i], d.nav_dr[i], d.cross[i])
    end
end

o = ls.orbit
open(joinpath(outdir, "landing_orbit.csv"), "w") do io
    println(io, "t_s,x_m,y_m,z_m,alt_m,phase")
    for i in eachindex(o.t)
        @printf(io, "%.2f,%.1f,%.1f,%.1f,%.1f,%d\n",
                o.t[i], o.x[i], o.y[i], o.z[i], o.h[i], o.phase[i])
    end
end

# the ground the last minutes were flown over, as a cut along the approach
if !plain
    sm = SurfaceModel(LunarTerrain(), ls.eph)
    arc, elev = terrain_profile(sm, ls.descent.r, ls.descent.v, ls.t_touchdown;
                                span = 12.0e3, n = 400)
    open(joinpath(outdir, "landing_terrain.csv"), "w") do io
        println(io, "arc_m,elevation_m")
        for i in eachindex(arc)
            @printf(io, "%.1f,%.2f\n", arc[i], elev[i])
        end
    end
end

a = rr.ascent.log
open(joinpath(outdir, "landing_ascent.csv"), "w") do io
    println(io, "t_s,alt_m,v_ms,v_horiz_ms,v_vert_ms,mass_kg,pitch_deg,x_m,y_m,z_m")
    for i in eachindex(a.t)
        @printf(io, "%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.3f,%.1f,%.1f,%.1f\n",
                a.t[i], a.h[i], a.v[i], a.vh[i], a.vv[i], a.m[i],
                rad2deg_(a.pitch[i]), a.x[i], a.y[i], a.z[i])
    end
end

open(joinpath(outdir, "landing_events.csv"), "w") do io
    println(io, "phase,event,t_s")
    for e in ls.ascent.events
        @printf(io, "ascent,%s,%.2f\n", e.name, e.t)
    end
    @printf(io, "cislunar,tli_ignition,%.2f\n", ls.cislunar.t_tli)
    @printf(io, "cislunar,tli_cutoff,%.2f\n",
            ls.cislunar.t_tli + ls.cislunar.burn_duration)
    @printf(io, "lunar,loi,%.2f\n", ls.t_loi)
    @printf(io, "lunar,doi,%.2f\n", ls.t_doi)
    @printf(io, "lunar,pdi,%.2f\n", ls.t_pdi)
    @printf(io, "lunar,high_gate,%.2f\n", ls.t_pdi + ls.descent.t_gate)
    @printf(io, "lunar,touchdown,%.2f\n", ls.t_touchdown)
    @printf(io, "return,liftoff,%.2f\n", rr.t_liftoff)
    @printf(io, "return,orbit_insertion,%.2f\n", rr.t_liftoff + rr.ascent.t_cut)
    @printf(io, "return,docking,%.2f\n", rr.t_dock)
    @printf(io, "return,tei,%.2f\n", rr.t_tei)
    @printf(io, "return,entry_interface,%.2f\n", rr.cis.t)
    rr.entry === nothing ||
        @printf(io, "return,splashdown,%.2f\n", rr.entry.t_splash)
end

open(joinpath(outdir, "landing_summary.txt"), "w") do io
    print_landing_summary(io, ls)
    println(io)
    print_return_summary(io, rr, ls)
end

println("\nwrote landing_{descent,orbit,terrain,ascent,events}.csv + summary to output/")
