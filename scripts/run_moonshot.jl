# Design and fly the full circumlunar free-return mission, then write the
# trajectory products consumed by scripts/make_3d.py:
#   output/moonshot_ascent.csv     launch -> parking-orbit insertion
#   output/moonshot_cislunar.csv   parking coast, TLI burn, translunar +
#                                  return coast (with Moon positions)
#   output/moonshot_entry.csv      entry interface -> splashdown
#   output/moonshot_events.csv     combined event timeline
#   output/moonshot_summary.txt    console summary
#
# Usage: julia --project scripts/run_moonshot.jl

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Printf

outdir = joinpath(@__DIR__, "..", "output")
mkpath(outdir)

println("Designing circumlunar free-return mission...")
t = @elapsed ms = moonshot(verbose = false)
@printf("design + flight complete in %.1f s\n\n", t)

print_moonshot_summary(ms)

write_ascent_csv(joinpath(outdir, "moonshot_ascent.csv"), ms.ascent)
write_cislunar_csv(joinpath(outdir, "moonshot_cislunar.csv"), ms.cislunar)
write_trajectory_csv(joinpath(outdir, "moonshot_entry.csv"), ms.entry)

# combined event timeline
open(joinpath(outdir, "moonshot_events.csv"), "w") do io
    println(io, "phase,event,t_s,alt_m,v_ms")
    for e in ms.ascent.events
        @printf(io, "ascent,%s,%.2f,%.1f,%.1f\n", e.name, e.t, e.h, e.vrel)
    end
    cis = ms.cislunar
    @printf(io, "cislunar,tli_ignition,%.2f,,\n", cis.t_tli)
    @printf(io, "cislunar,tli_cutoff,%.2f,,\n", cis.t_tli + cis.burn_duration)
    @printf(io, "cislunar,perilune,%.2f,%.1f,\n", cis.t_perilune, cis.perilune_alt)
    @printf(io, "cislunar,entry_handoff,%.2f,140000,\n", cis.t)
    for e in ms.entry.events
        @printf(io, "entry,%s,%.2f,%.1f,%.1f\n", e.name, e.t, e.h, e.vrel)
    end
end

open(joinpath(outdir, "moonshot_summary.txt"), "w") do io
    print_moonshot_summary(io, ms)
end

println("\nwrote moonshot_{ascent,cislunar,entry,events}.csv + summary to output/")
