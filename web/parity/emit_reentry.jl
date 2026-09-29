# Reference for the LEO reentry parity test: fly the targeted west-coast
# mission in Julia and emit the tuned deorbit elements, the full trajectory log,
# the events and the summary. The JS port flies the same thing and must match.
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"
function arr(io, v)
    print(io, "[")
    for (i, x) in enumerate(v)
        i > 1 && print(io, ",")
        print(io, fm(x))
    end
    print(io, "]")
end

veh = S.default_reentry_pod()
el, _ = S.target_deorbit(S.DeorbitElements(), veh)
scn = S.scenario_from_elements(el, veh)
res = S.simulate(scn)
L = res.log

open(ARGS[1], "w") do io
    print(io, "{\"el\":{\"apoapsis_alt\":", fm(el.apoapsis_alt),
          ",\"periapsis_alt\":", fm(el.periapsis_alt),
          ",\"inclination\":", fm(el.inclination),
          ",\"raan\":", fm(el.raan), ",\"argp\":", fm(el.argp),
          ",\"nu0\":", fm(el.nu0), "},\"log\":{")
    for (i, k) in enumerate([:t, :h, :lat, :lon, :vin, :vrel, :gamma, :psi,
                             :mach, :qbar, :gload, :alpha, :qrate, :rho,
                             :qdot_conv, :qdot_rad, :qload, :twall])
        i > 1 && print(io, ",")
        print(io, "\"", k, "\":")
        arr(io, getfield(L, k))
    end
    print(io, "},\"events\":[")
    for (i, e) in enumerate(res.events)
        i > 1 && print(io, ",")
        print(io, "{\"name\":\"", e.name, "\",\"t\":", fm(e.t), ",\"h\":", fm(e.h),
              ",\"mach\":", fm(e.mach), ",\"vrel\":", fm(e.vrel),
              ",\"lat\":", fm(e.lat), ",\"lon\":", fm(e.lon), "}")
    end
    print(io, "],\"summary\":{\"t_splash\":", fm(res.t_splash),
          ",\"lat_splash\":", fm(res.lat_splash),
          ",\"lon_splash\":", fm(res.lon_splash),
          ",\"v_splash\":", fm(res.v_splash), ",\"miss_km\":", fm(res.miss_km),
          ",\"peak_gload\":", fm(res.peak_gload),
          ",\"peak_qdot\":", fm(res.peak_qdot),
          ",\"peak_qbar\":", fm(res.peak_qbar),
          ",\"heat_load\":", fm(res.heat_load),
          ",\"terminated\":\"", res.terminated, "\"}}")
end
