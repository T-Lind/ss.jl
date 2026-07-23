# Run the targeted nominal west-coast reentry and write CSV outputs.
#
#   julia --project scripts/run_nominal.jl

using SatelliteSim
using Printf

outdir = joinpath(dirname(@__DIR__), "output")
mkpath(outdir)

println("Targeting deorbit for west-coast splashdown...")
scn, el, _ = west_coast_scenario(verbose = true)

println("Running nominal trajectory...")
res = simulate(scn)
print_summary(res, scn)

@printf("\nDeorbit solution: apo %.0f km, peri %.0f km, i=%.1f°, RAAN=%.4f°, argp=%.4f°\n",
        el.apoapsis_alt / 1e3, el.periapsis_alt / 1e3,
        rad2deg_(el.inclination), rad2deg_(el.raan), rad2deg_(el.argp))
bc = ballistic_coefficient(scn.vehicle)
@printf("Vehicle: %.0f kg, β = %.1f kg/m²\n", scn.vehicle.mass, bc)

write_trajectory_csv(joinpath(outdir, "nominal_trajectory.csv"), res)
write_events_csv(joinpath(outdir, "nominal_events.csv"), res)

# persist the tuned elements so other scripts (Monte Carlo, plots) reuse them
open(joinpath(outdir, "tuned_elements.csv"), "w") do io
    println(io, "apoapsis_alt,periapsis_alt,inclination,raan,argp,nu0")
    println(io, join((el.apoapsis_alt, el.periapsis_alt, el.inclination,
                      el.raan, el.argp, el.nu0), ","))
end
println("\nWrote output/nominal_trajectory.csv, output/nominal_events.csv, output/tuned_elements.csv")
