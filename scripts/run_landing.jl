# Design and fly the lunar landing mission — pad to the surface of the Moon —
# and write the trajectory products:
#   output/landing_descent.csv   powered descent, ignition to touchdown
#   output/landing_orbit.csv     lunar parking orbit + descent ellipse
#   output/landing_events.csv    combined event timeline
#   output/landing_summary.txt   console summary
#
# Usage: julia --project -t auto scripts/run_landing.jl

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Printf

outdir = joinpath(@__DIR__, "..", "output")
mkpath(outdir)

lander = default_lander()
# The reference Sable cannot throw twelve tonnes anywhere; a Starship-class
# vehicle can throw it and much more. That is the whole story of why lunar
# landing missions look the way they do.
lv = starship_expendable(payload = lander_mass(lander))

println("Designing lunar landing mission...")
t = @elapsed ls = moonlanding(lander = lander, lv = lv,
                              kick_angle = deg2rad_(5.0))
@printf("design + flight complete in %.1f s\n\n", t)

print_landing_summary(ls)

d = ls.descent.log
open(joinpath(outdir, "landing_descent.csv"), "w") do io
    println(io, "t_s,alt_m,downrange_m,v_ms,v_horiz_ms,v_vert_ms,mass_kg,throttle,pitch_deg")
    for i in eachindex(d.t)
        @printf(io, "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.4f,%.3f\n",
                d.t[i], d.h[i], d.downrange[i], d.v[i], d.vh[i], d.vv[i],
                d.m[i], d.throttle[i], rad2deg_(d.pitch[i]))
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
end

open(joinpath(outdir, "landing_summary.txt"), "w") do io
    print_landing_summary(io, ls)
end

println("\nwrote landing_{descent,orbit,events}.csv + summary to output/")
