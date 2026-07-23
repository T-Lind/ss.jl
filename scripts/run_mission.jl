# Fly a mission from a TOML spec.
#
# Usage: julia --project scripts/run_mission.jl missions/moonshot.toml

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim

path = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "missions", "moonshot.toml")
spec = load_mission(path)
println("mission: ", spec.name, "  (", spec.lv.name, ", ",
        round(liftoff_mass(spec.lv) / 1e3, digits = 1), " t)")
ms = run_mission(spec)
print_moonshot_summary(ms)
