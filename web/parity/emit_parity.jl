# Emit Julia results for the browser port to compare against. Writes a JSON
# file so the Node parity tests can diff the two implementations on identical
# inputs. Run from the repo root:
#
#   julia --project=. web/parity/emit_parity.jl web/parity/golden.json
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"

hs = sort(unique(vcat(
    collect(range(-50.0, 900_000.0; length = 401)),
    [0.0, 11000.0, 20000.0, 47000.0, 71000.0, 86000.0, 86000.0001,
     90_000.0, 500_000.0, 500_000.0001, 1.0e6])))

pos = [(7.0e6, 0.0, 0.0), (RE_MEAN + 2.0e5, 1.0e5, -3.0e5),
       (0.0, 0.0, RE_MEAN), (-6.0e6, 2.0e6, 1.0e6),
       (2.0e7, -1.5e7, 3.0e6)]

open(ARGS[1], "w") do io
    print(io, "{\"atmosphere\":[")
    for (i, h) in enumerate(hs)
        i > 1 && print(io, ",")
        rho, T, p, a = S.atmosphere_state(S.USSA76(), h)
        print(io, "{\"h\":", fm(h), ",\"rho\":", fm(rho), ",\"T\":", fm(T),
              ",\"p\":", fm(p), ",\"a\":", fm(a), "}")
    end
    print(io, "],\"gravity\":[")
    for (i, r) in enumerate(pos)
        i > 1 && print(io, ",")
        a = S.gravity_accel(S.J2Gravity(), r, 0.0)
        print(io, "{\"r\":[", fm(r[1]), ",", fm(r[2]), ",", fm(r[3]),
              "],\"a\":[", fm(a[1]), ",", fm(a[2]), ",", fm(a[3]), "]}")
    end
    print(io, "]}")
end
