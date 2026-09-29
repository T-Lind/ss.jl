# Reference for the ascent parity test: tune the reference Sable's pitch
# program and emit the tuned guidance plus the full ascent log, events and
# summary. The JS port runs the same tune_ascent and must match.
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

lv = S.default_moon_rocket()
guid, res = S.tune_ascent(lv, S.AscentGuidance())
L = res.log

open(ARGS[1], "w") do io
    print(io, "{\"guid\":{\"pitch0\":", fm(guid.pitch0),
          ",\"pitch_rate\":", fm(guid.pitch_rate),
          ",\"kick_angle\":", fm(guid.kick_angle),
          ",\"h_target\":", fm(guid.h_target), "},\"log\":{")
    for (i, k) in enumerate([:t, :rx, :ry, :rz, :h, :vrel, :vin, :gamma,
                             :mach, :qbar, :m, :thrust, :lat, :lon, :downrange])
        i > 1 && print(io, ",")
        print(io, "\"", k, "\":")
        arr(io, getfield(L, k))
    end
    print(io, "},\"events\":[")
    for (i, e) in enumerate(res.events)
        i > 1 && print(io, ",")
        print(io, "{\"name\":\"", e.name, "\",\"t\":", fm(e.t), ",\"h\":", fm(e.h),
              ",\"vrel\":", fm(e.vrel), ",\"m\":", fm(e.m), "}")
    end
    el = res.elements
    print(io, "],\"summary\":{\"m\":", fm(res.m), ",\"t\":", fm(res.t),
          ",\"h_cut\":", fm(res.h_cut), ",\"gamma_cut\":", fm(res.gamma_cut),
          ",\"reached_orbit\":", res.reached_orbit ? "true" : "false",
          ",\"prop_left\":")
    arr(io, res.prop_left)
    print(io, ",\"elements\":{\"a\":", fm(el.a), ",\"e\":", fm(el.e),
          ",\"i\":", fm(el.i), ",\"rp\":", fm(el.rp), ",\"ra\":", fm(el.ra), "}}}")
end
