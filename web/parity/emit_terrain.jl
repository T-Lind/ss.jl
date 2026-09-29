# Reference for the procedural terrain. The 32-bit hash is meant to be bit-
# reproducible in JavaScript via Math.imul, so heights should agree to libm
# precision.
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
vec(io, v) = print(io, "[", fm(v[1]), ",", fm(v[2]), ",", fm(v[3]), "]")

lats = deg2rad_.([-60, -20, 0, 20, 45, 80])
lons = deg2rad_.([-150, -60, 0, 30, 120, 179])
us = [S.vunit((cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat))) for lat in lats for lon in lons]

tr = S.LunarTerrain()
trh = S.highland_terrain()
trm = S.mare_terrain()

u0 = us[10]
e1, e2 = S._tangents(u0)
sh = S.site_hazard(tr, u0)
su, sd, sc, ssc = S.safe_site(tr, u0, e1, e2)

r0 = (RE_MEAN + 200.0e3, 0.0, 0.0)
v0 = (0.0, sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0)
eph = S.coplanar_moon(r0, v0)
sm = S.SurfaceModel(tr, eph)
rr = S.vscale(S.moonfixed_inv(S.vunit((0.3, 0.4, 0.5)), 0.0, eph), R_MOON + 2500.0)

open(ARGS[1], "w") do io
    print(io, "{\"us\":[")
    for (i, u) in enumerate(us)
        i > 1 && print(io, ",")
        vec(io, u)
    end
    print(io, "],\"hd\":")
    arr(io, [S.terrain_height(tr, u) for u in us])
    print(io, ",\"hh\":")
    arr(io, [S.terrain_height(trh, u) for u in us])
    print(io, ",\"hm\":")
    arr(io, [S.terrain_height(trm, u) for u in us])
    print(io, ",\"u0\":")
    vec(io, u0)
    print(io, ",\"sh\":{\"score\":", fm(sh[1]), ",\"slope\":", fm(sh[2]), ",\"relief\":", fm(sh[3]), "}")
    print(io, ",\"safe\":{\"u\":")
    vec(io, su)
    print(io, ",\"d\":", fm(sd), ",\"c\":", fm(sc), ",\"score\":", fm(ssc), "}")
    print(io, ",\"slope\":", fm(S.terrain_slope(tr, u0)))
    print(io, ",\"sr\":", fm(S.surface_radius(sm, rr, 0.0)))
    print(io, ",\"sa\":", fm(S.surface_altitude(sm, rr, 0.0)), "}")
end
