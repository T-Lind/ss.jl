# Reference for the full circumlunar mission.
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"

ms = S.moonshot()
asc = ms.ascent
cis = ms.cislunar
ent = ms.entry
el = asc.elements

open(ARGS[1], "w") do io
    print(io, "{\"design_status\":\"", ms.design_status, "\"")
    print(io, ",\"ascent\":{\"rp\":", fm(el.rp), ",\"ra\":", fm(el.ra),
          ",\"i\":", fm(el.i), ",\"m\":", fm(asc.m),
          ",\"prop_left_end\":", fm(asc.prop_left[end]), ",\"t\":", fm(asc.t), "}")
    print(io, ",\"cis\":{\"outcome\":\"", cis.outcome, "\",\"t_tli\":", fm(cis.t_tli),
          ",\"dv_tli\":", fm(cis.dv_tli), ",\"m\":", fm(cis.m),
          ",\"perilune_alt\":", fm(cis.perilune_alt),
          ",\"t_perilune\":", fm(cis.t_perilune),
          ",\"vac_perigee_alt\":", fm(cis.vac_perigee_alt), ",\"t\":", fm(cis.t), "}")
    print(io, ",\"entry\":{\"terminated\":\"", ent.terminated, "\",\"t_splash\":", fm(ent.t_splash),
          ",\"lat_splash\":", fm(ent.lat_splash), ",\"lon_splash\":", fm(ent.lon_splash),
          ",\"v_splash\":", fm(ent.v_splash), ",\"peak_gload\":", fm(ent.peak_gload),
          ",\"peak_qdot\":", fm(ent.peak_qdot), ",\"peak_qbar\":", fm(ent.peak_qbar),
          ",\"heat_load\":", fm(ent.heat_load), "}}")
end
