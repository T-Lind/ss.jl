# Reference for the powered descent and the full lunar landing mission.
# `moonlanding` is flown exactly as test/runtests.jl's "lunar landing" testset
# flies it: the stock Sable cannot lift a 12.5 t lander, so the launcher is a
# Starship-class stack carrying the lander's wet mass.
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"

lnd = S.default_lander()
lv = S.starship_expendable(payload = S.lander_mass(lnd))
ls = S.moonlanding(lander = lnd, lv = lv, kick_angle = deg2rad_(5.0))
d = ls.descent

# powered descent from a descent-orbit periapsis, as runtests.jl does it
lander = S.Lander(mdry = 3500.0, mprop = 5700.0, thrust = 45e3, isp = 311.0,
                  throttle_min = 0.10)
rpdi = R_MOON + 15e3
a_d = 0.5 * (rpdi + R_MOON + 100e3)
vpdi = sqrt(MU_MOON * (2 / rpdi - 1 / a_d))
pd = S.powered_descent(lander, (rpdi, 0.0, 0.0), (0.0, vpdi, 0.0), 9200.0)

open(ARGS[1], "w") do io
    print(io, "{\"moonlanding\":{")
    print(io, "\"dv_loi\":", fm(ls.dv_loi), ",\"dv_doi\":", fm(ls.dv_doi),
          ",\"t_loi\":", fm(ls.t_loi), ",\"t_doi\":", fm(ls.t_doi),
          ",\"t_pdi\":", fm(ls.t_pdi), ",\"t_touchdown\":", fm(ls.t_touchdown),
          ",\"lat_land\":", fm(ls.lat_land), ",\"lon_land\":", fm(ls.lon_land),
          ",\"prop_margin\":", fm(ls.prop_margin),
          ",\"perilune_alt\":", fm(ls.cislunar.perilune_alt),
          ",\"descent\":{")
    print(io, "\"outcome\":\"", d.outcome, "\",\"v_vertical\":", fm(d.v_vertical),
          ",\"v_horizontal\":", fm(d.v_horizontal),
          ",\"prop_used\":", fm(d.prop_used), ",\"prop_left\":", fm(d.prop_left),
          ",\"hover_s\":", fm(d.hover_s), ",\"min_throttle\":", fm(d.min_throttle),
          ",\"dv_braking\":", fm(d.dv_braking), ",\"dv_terminal\":", fm(d.dv_terminal),
          ",\"t_touchdown\":", fm(d.t_touchdown), ",\"t_gate\":", fm(d.t_gate),
          ",\"downrange\":", fm(d.downrange),
          ",\"pitch0\":", fm(d.pitch0), ",\"pitch_rate\":", fm(d.pitch_rate), "}}")
    print(io, ",\"powered\":{")
    print(io, "\"rpdi\":", fm(rpdi), ",\"vpdi\":", fm(vpdi), ",\"m0\":9200.0",
          ",\"outcome\":\"", pd.outcome, "\",\"v_vertical\":", fm(pd.v_vertical),
          ",\"v_horizontal\":", fm(pd.v_horizontal),
          ",\"prop_used\":", fm(pd.prop_used), ",\"prop_left\":", fm(pd.prop_left),
          ",\"hover_s\":", fm(pd.hover_s), ",\"min_throttle\":", fm(pd.min_throttle),
          ",\"dv_braking\":", fm(pd.dv_braking), ",\"dv_terminal\":", fm(pd.dv_terminal),
          ",\"t_touchdown\":", fm(pd.t_touchdown), ",\"t_gate\":", fm(pd.t_gate),
          ",\"downrange\":", fm(pd.downrange),
          ",\"pitch0\":", fm(pd.pitch0), ",\"pitch_rate\":", fm(pd.pitch_rate), "}}")
end
