# Julia-native plotting (requires the Plots.jl package):
#
#   julia --project -e 'using Pkg; Pkg.add("Plots")'   # once
#   julia --project scripts/make_plots.jl
#
# Renders the same figures as scripts/make_plots.py from the CSVs written by
# run_nominal.jl / run_montecarlo.jl. Kept separate from the core package so
# SatelliteSim itself stays dependency-free.

using Printf

const OUT = joinpath(dirname(@__DIR__), "output")
const PLOTS_DIR = joinpath(OUT, "plots")
mkpath(PLOTS_DIR)

try
    @eval using Plots
catch
    error("Plots.jl is not installed. Run: julia --project -e 'using Pkg; Pkg.add(\"Plots\")'")
end

function read_csv(path)
    lines = readlines(path)
    hdr = Symbol.(split(lines[1], ","))
    raw = [split(l, ",") for l in lines[2:end]]
    cols = Dict{Symbol,Any}()
    for (j, h) in enumerate(hdr)
        vals = [r[j] for r in raw]
        parsed = tryparse.(Float64, vals)
        cols[h] = any(isnothing, parsed) ? vals : Float64.(parsed)
    end
    cols
end

traj = read_csv(joinpath(OUT, "nominal_trajectory.csv"))
ev = read_csv(joinpath(OUT, "nominal_events.csv"))
mc = read_csv(joinpath(OUT, "montecarlo.csv"))

t_ei = ev[:t_s][findfirst(==("entry_interface"), ev[:event])]
entry = traj[:t_s] .>= t_ei - 20
te = (traj[:t_s][entry] .- t_ei) ./ 60

default(linewidth = 2, grid = true, gridalpha = 0.3, framestyle = :box,
        fontfamily = "sans-serif", label = "")

# flight profile
p1 = plot(traj[:t_s] ./ 60, traj[:alt_m] ./ 1e3,
          xlabel = "time [min]", ylabel = "altitude [km]", title = "Altitude vs time")
scatter!(p1, ev[:t_s] ./ 60, ev[:alt_m] ./ 1e3, ms = 4, color = :orangered)
p2 = plot(traj[:v_rel_ms][entry] ./ 1e3, traj[:alt_m][entry] ./ 1e3,
          xlabel = "relative velocity [km/s]", ylabel = "altitude [km]",
          title = "Velocity-altitude")
p3 = plot(te, traj[:gload][entry], xlabel = "time from EI [min]",
          ylabel = "g-load", title = "Aerodynamic g-load")
p4 = plot(te, traj[:qbar_pa][entry] ./ 1e3, xlabel = "time from EI [min]",
          ylabel = "dynamic pressure [kPa]", title = "Dynamic pressure")
savefig(plot(p1, p2, p3, p4, layout = (2, 2), size = (1100, 750)),
        joinpath(PLOTS_DIR, "nominal_profile_jl.png"))

# aerothermal
p1 = plot(te, traj[:qdot_conv_wcm2][entry], label = "convective",
          xlabel = "time from EI [min]", ylabel = "heat flux [W/cm²]",
          title = "Stagnation heating")
plot!(p1, te, traj[:qdot_rad_wcm2][entry], label = "radiative")
p2 = plot(traj[:qdot_conv_wcm2][entry] .+ traj[:qdot_rad_wcm2][entry],
          traj[:alt_m][entry] ./ 1e3, xlabel = "heat flux [W/cm²]",
          ylabel = "altitude [km]", title = "Heat flux vs altitude")
p3 = plot(te, traj[:heat_load_jcm2][entry] ./ 1e3, xlabel = "time from EI [min]",
          ylabel = "heat load [kJ/cm²]", title = "Integrated heat load")
savefig(plot(p1, p2, p3, layout = (1, 3), size = (1300, 420)),
        joinpath(PLOTS_DIR, "nominal_heating_jl.png"))

# attitude
p1 = plot(te, traj[:alpha_deg][entry], xlabel = "time from EI [min]",
          ylabel = "α [deg]", title = "AoA oscillation & damping")
p2 = plot(te, traj[:pitch_rate_dps][entry], xlabel = "time from EI [min]",
          ylabel = "q [deg/s]", title = "Pitch rate")
savefig(plot(p1, p2, layout = (1, 2), size = (1100, 420)),
        joinpath(PLOTS_DIR, "nominal_attitude_jl.png"))

# ground track + MC footprint (no basemap: plain lat/lon)
pre = traj[:t_s] .< t_ei
p1 = plot(traj[:lon_deg][pre], traj[:lat_deg][pre], label = "orbital phase",
          xlabel = "longitude [deg]", ylabel = "latitude [deg]", title = "Ground track")
plot!(p1, traj[:lon_deg][.!pre], traj[:lat_deg][.!pre], label = "entry phase")
scatter!(p1, [-121.5], [32.5], marker = :star5, ms = 8, label = "target")
ok = mc[:terminated] .== "splashdown"
p2 = scatter(mc[:lon_deg][ok], mc[:lat_deg][ok], ms = 2, alpha = 0.5,
             xlabel = "longitude [deg]", ylabel = "latitude [deg]",
             title = "MC splashdown footprint", label = "MC")
scatter!(p2, [-121.5], [32.5], marker = :star5, ms = 8, label = "target")
savefig(plot(p1, p2, layout = (1, 2), size = (1300, 500)),
        joinpath(PLOTS_DIR, "groundtrack_jl.png"))

println("Wrote *_jl.png plots to ", PLOTS_DIR)
