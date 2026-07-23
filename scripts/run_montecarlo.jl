# Monte Carlo dispersion analysis around the targeted nominal.
#
#   julia --project -t auto scripts/run_montecarlo.jl [N]
#
# Reuses output/tuned_elements.csv from run_nominal.jl when present,
# otherwise re-targets first.

using SatelliteSim
using Printf

n = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 300
outdir = joinpath(dirname(@__DIR__), "output")
mkpath(outdir)

tuned = joinpath(outdir, "tuned_elements.csv")
scn = if isfile(tuned)
    vals = parse.(Float64, split(readlines(tuned)[2], ","))
    el = DeorbitElements(vals...)
    scenario_from_elements(el, default_reentry_pod())
else
    println("No tuned elements found — targeting first...")
    first(west_coast_scenario())
end

disp = Dispersions()   # defaults: mass ±3%, CD ±5%, density ±10%, burn errors, ...
println("Running $n Monte Carlo samples on $(Threads.nthreads()) threads...")
t0 = time()
samples = run_montecarlo(scn, disp; n = n)
@printf("Done in %.1f s\n\n", time() - t0)

stats = mc_statistics(samples)
println("== Monte Carlo splashdown statistics ($(stats.n_ok) ok, $(stats.n_fail) failed) ==")
@printf("  Mean splash point : %.3f°N, %.3f°E\n", stats.mean_lat, stats.mean_lon)
@printf("  Target            : %.3f°N, %.3f°E\n",
        rad2deg_(scn.target_lat), rad2deg_(scn.target_lon))
@printf("  Mean miss (target): %.1f km   max %.1f km\n", stats.mean_miss_km, stats.max_miss_km)
@printf("  CEP50 (dispersion): %.1f km   R95 %.1f km\n", stats.cep50_km, stats.r95_km)
@printf("  1σ ellipse        : %.1f × %.1f km at %.1f°\n",
        stats.sigma_major_km, stats.sigma_minor_km, stats.ellipse_angle_deg)
@printf("  Mean peak g       : %.2f g\n", stats.mean_peak_g)
@printf("  Mean heat load    : %.2f MJ/m²\n", stats.mean_heat_MJm2)

write_montecarlo_csv(joinpath(outdir, "montecarlo.csv"), samples)
open(joinpath(outdir, "montecarlo_summary.txt"), "w") do io
    for (k, v) in pairs(stats)
        println(io, k, " = ", v)
    end
end
println("\nWrote output/montecarlo.csv, output/montecarlo_summary.txt")
