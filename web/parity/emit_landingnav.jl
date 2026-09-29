# Reference for the descent navigation filter, RNG and hazard redesignation.
using SatelliteSim
const S = SatelliteSim

fm(x) = isfinite(Float64(x)) ? string(Float64(x)) : "null"
vec(io, v) = print(io, "[", fm(v[1]), ",", fm(v[2]), ",", fm(v[3]), "]")

seed = UInt32(0x5EED1A11)
nrand = [S._nrand(seed, k) for k in 1:8]
ngauss = [S._ngauss(seed, k) for k in 1:8]

cfg = S.DescentNav()
r = S.vscale(S.vunit((0.3, 0.4, 0.5)), R_MOON + 3000.0)
v = (10.0, -5.0, 3.0)
hhat = S.vunit(S.vcross(r, v))
nav = S.init_nav(cfg, r, v, hhat)

rr0 = (RE_MEAN + 200.0e3, 0.0, 0.0)
vv0 = (0.0, sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0)
eph = S.coplanar_moon(rr0, vv0)
surf = S.SurfaceModel(S.LunarTerrain(), eph)
scan = S.HazardScan()
u, sc, sc0, off = S.redesignate(scan, surf, r, v, 0.0, hhat, 500.0)

open(ARGS[1], "w") do io
    print(io, "{\"nrand\":[")
    for (i, x) in enumerate(nrand); i > 1 && print(io, ","); print(io, fm(x)); end
    print(io, "],\"ngauss\":[")
    for (i, x) in enumerate(ngauss); i > 1 && print(io, ","); print(io, fm(x)); end
    print(io, "],\"nav\":{\"r\":")
    vec(io, nav.r)
    print(io, ",\"v\":")
    vec(io, nav.v)
    print(io, ",\"r_ref\":", fm(nav.r_ref), ",\"seed\":", nav.seed, "}")
    print(io, ",\"redesignate\":{\"u\":")
    vec(io, u)
    print(io, ",\"sc\":", fm(sc), ",\"sc0\":", fm(sc0), ",\"off\":", fm(off), "}}")
end
