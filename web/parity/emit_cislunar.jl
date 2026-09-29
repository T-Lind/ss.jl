# Reference for the cislunar parity test: the patched-conic seed, a fixed
# propagation, and the converged free-return design on the reference kick stage.
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"

lv = S.default_moon_rocket()
kick = lv.stages[end]
r0 = (RE_MEAN + 200.0e3, 0.0, 0.0)
v0 = (0.0, sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0)
t0 = 0.0
eph = S.coplanar_moon(r0, v0)
m_stack = 1400.0
prop_avail = 900.0

lead, tf, dv_seed = S.seed_free_return(r0, v0)
t_align = S.tli_alignment_time(r0, v0, t0, eph, lead)

fixed = S.fly_cislunar(r0, v0, t0, eph; t_ign = t_align, dv = dv_seed,
                       stage = kick, m_stack = m_stack, prop_avail = prop_avail)

t_ign, dv, full, status = S.design_free_return(r0, v0, t0, eph;
                                               stage = kick, m_stack = m_stack,
                                               prop_avail = prop_avail)

dd = minimum(fixed.log.d_moon)

open(ARGS[1], "w") do io
    print(io, "{\"seed\":[", fm(lead), ",", fm(tf), ",", fm(dv_seed), "]")
    print(io, ",\"t_align\":", fm(t_align))
    print(io, ",\"fixed\":{\"outcome\":\"", fixed.outcome, "\",\"perilune_alt\":",
          fm(fixed.perilune_alt), ",\"vac_perigee_alt\":", fm(fixed.vac_perigee_alt),
          ",\"t\":", fm(fixed.t), ",\"d_moon_min\":", fm(dd),
          ",\"n\":", length(fixed.log.t), "}")
    print(io, ",\"design\":{\"status\":\"", status, "\",\"t_ign\":", fm(t_ign),
          ",\"dv\":", fm(dv), ",\"outcome\":\"", full.outcome,
          "\",\"perilune_alt\":", fm(full.perilune_alt),
          ",\"vac_perigee_alt\":", fm(full.vac_perigee_alt),
          ",\"t\":", fm(full.t), "}}")
end
