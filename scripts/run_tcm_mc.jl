# Monte Carlo over TLI execution errors: how much mid-course correction
# does the kick stage need to carry?
#
# The nominal free return is designed once; each threaded sample then
# executes the TLI with dispersed magnitude/pointing, flies to the TCM
# epoch, designs and applies the two-stage correction, and coasts home.
# Reported: the TCM delta-v distribution, its kick-propellant cost against
# the post-TLI margin, and the fraction of samples that still make the
# entry corridor.
#
# Usage: julia --project -t auto scripts/run_tcm_mc.jl [N] [sigma_mag_%] [sigma_point_deg]

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Random
using Statistics
using Printf

N = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 100
sigma_mag = (length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 0.2) / 100
sigma_point = deg2rad_(length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 0.25)

println("designing nominal mission...")
lv = default_moon_rocket()
guid, asc = tune_ascent(lv, AscentGuidance())
el = asc.elements
Tpark = 2pi * sqrt(el.a^3 / MU_EARTH)
n_sc = 2pi / Tpark
lead, _, _ = seed_free_return(asc.r, asc.v)
eph = coplanar_moon(asc.r, asc.v;
                    phase0 = lead + (n_sc - N_MOON) * 0.55 * Tpark - N_MOON * asc.t)
kick = lv.stages[end]
m_stack = asc.m - (lv.stages[2].mdry + asc.prop_left[2])
t_ign, dv, nom, dstatus = design_free_return(asc.r, asc.v, asc.t, eph;
                                             stage = kick, m_stack = m_stack,
                                             prop_avail = asc.prop_left[end])
dstatus in (:converged, :outside_tolerance) ||
    error("free-return targeting did not converge (status: $dstatus) — " *
          "dispersing about a design that missed its target says nothing")
nomfly = fly_cislunar(asc.r, asc.v, asc.t, eph; t_ign = t_ign, dv = dv,
                      stage = kick, m_stack = m_stack,
                      prop_avail = asc.prop_left[end],
                      stop_after_flyby = true, t_max = 10.0 * 86400.0)
refleg = fly_cislunar(asc.r, asc.v, asc.t, eph; t_ign = t_ign, dv = dv,
                      stage = kick, m_stack = m_stack,
                      prop_avail = asc.prop_left[end],
                      t_max = nomfly.t_perilune - asc.t)
margin0 = nom.m - (kick.mdry + lv.payload_mass)
@printf("nominal: TLI dv %.1f m/s, post-TLI kick margin %.1f kg\n", dv, margin0)
@printf("sampling %d missions: sigma_mag %.2f%%, sigma_point %.2f deg\n\n",
        N, 100sigma_mag, rad2deg_(sigma_point))

results = Vector{Union{Nothing,NamedTuple}}(nothing, N)
Threads.@threads for i in 1:N
    rng = MersenneTwister(1000 + i)
    me = sigma_mag * randn(rng)
    pe = sigma_point * randn(rng)
    results[i] = try
        cis, tcm_dv = fly_cislunar_tcm(asc.r, asc.v, asc.t, eph;
                                       t_ign = t_ign, dv = dv, stage = kick,
                                       m_stack = m_stack,
                                       prop_avail = asc.prop_left[end],
                                       r_ref = refleg.r, t_ref = refleg.t,
                                       dv_scale = 1.0 + me, point_err = pe,
                                       hp_perigee_proxy = nomfly.vac_perigee_alt)
        tcm_prop = cis.m * (exp(tcm_dv / (G0 * kick.isp_vac)) - 1)
        good = cis.outcome == :entry_interface &&
               abs(cis.vac_perigee_alt - 35e3) < 25e3
        (mag = me, point = pe, tcm_dv = tcm_dv, tcm_prop = tcm_prop,
         perilune = cis.perilune_alt, perigee = cis.vac_perigee_alt,
         good = good)
    catch
        nothing
    end
end

ok = [r for r in results if r !== nothing]
dvs = sort([r.tcm_dv for r in ok])
q(p) = dvs[clamp(round(Int, p * length(dvs)), 1, length(dvs))]
@printf("completed %d/%d samples, %d reach the corridor\n",
        length(ok), N, count(r -> r.good, ok))
@printf("TCM dv    : mean %.1f  median %.1f  p95 %.1f  p99 %.1f  max %.1f m/s\n",
        mean(dvs), q(0.5), q(0.95), q(0.99), dvs[end])
props = sort([r.tcm_prop for r in ok])
@printf("TCM prop  : mean %.1f  p95 %.1f  max %.1f kg  (post-TLI margin %.1f kg)\n",
        mean(props), props[clamp(round(Int, 0.95length(props)), 1, length(props))],
        props[end], margin0)
@printf("prop short: %d/%d samples exceed the kick margin\n",
        count(r -> r.tcm_prop > margin0, ok), length(ok))

open(joinpath(@__DIR__, "..", "output", "tcm_montecarlo.csv"), "w") do io
    println(io, "mag_err,point_err_deg,tcm_dv_ms,tcm_prop_kg,perilune_km,perigee_km,corridor_ok")
    for r in ok
        @printf(io, "%.5f,%.4f,%.2f,%.2f,%.1f,%.2f,%d\n",
                r.mag, rad2deg_(r.point), r.tcm_dv, r.tcm_prop,
                r.perilune / 1e3, r.perigee / 1e3, r.good ? 1 : 0)
    end
end
println("\nwrote output/tcm_montecarlo.csv")
