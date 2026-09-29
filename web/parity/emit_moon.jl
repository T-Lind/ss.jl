# Reference for the Moon parity test: ephemeris, Moon-fixed frame, and the
# non-spherical gravity field (J2 + mascons).
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
varr(io, v) = arr(io, collect(v))

r0 = (RE_MEAN + 200.0e3, 0.0, 0.0)
v0 = (0.0, sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0)
eph = S.coplanar_moon(r0, v0)
fd = S.LunarGravity()

ts = collect(range(0.0, 12 * 86400.0; length = 25))
pos = [S.moon_position(eph, t) for t in ts]
vel = [S.moon_velocity(eph, t) for t in ts]

# lunar gravity at a few Moon-centred positions, with and without the field
rr = [(R_MOON + 100.0e3, 0.0, 0.0),
      (R_MOON + 50.0e3, 3.0e4, -2.0e4),
      (R_MOON + 300.0e3, -6.0e4, 1.0e5)]
gt = [0.0, 1.0e5]
grav = [(r = r, t = t, a = S.lunar_gravity(r, fd, t, eph))
        for r in rr for t in gt]
grav0 = [(r = r, t = t, a = S.lunar_gravity(r, nothing, t, eph))
         for r in rr for t in gt]

anom = [S.gravity_anomaly(fd, deg2rad_(33.0), deg2rad_(-16.0), 100.0e3, eph, 0.0),
        S.gravity_anomaly(fd, deg2rad_(0.0), deg2rad_(0.0), 100.0e3, eph, 0.0)]

open(ARGS[1], "w") do io
    print(io, "{\"ts\":")
    arr(io, ts)
    print(io, ",\"pos\":[")
    for (i, p) in enumerate(pos)
        i > 1 && print(io, ",")
        print(io, "["); for j in 1:3; j > 1 && print(io, ","); print(io, fm(p[j])); end; print(io, "]")
    end
    print(io, "],\"vel\":[")
    for (i, p) in enumerate(vel)
        i > 1 && print(io, ",")
        print(io, "["); for j in 1:3; j > 1 && print(io, ","); print(io, fm(p[j])); end; print(io, "]")
    end
    print(io, "],\"grav\":[")
    for (i, g) in enumerate(grav)
        i > 1 && print(io, ",")
        print(io, "{\"r\":["); for j in 1:3; j > 1 && print(io, ","); print(io, fm(g.r[j])); end
        print(io, "],\"t\":", fm(g.t), ",\"a\":[")
        for j in 1:3; j > 1 && print(io, ","); print(io, fm(g.a[j])); end
        print(io, "]}")
    end
    print(io, "],\"grav0\":[")
    for (i, g) in enumerate(grav0)
        i > 1 && print(io, ",")
        print(io, "{\"r\":["); for j in 1:3; j > 1 && print(io, ","); print(io, fm(g.r[j])); end
        print(io, "],\"t\":", fm(g.t), ",\"a\":[")
        for j in 1:3; j > 1 && print(io, ","); print(io, fm(g.a[j])); end
        print(io, "]}")
    end
    print(io, "],\"anom\":")
    arr(io, anom)
    print(io, "}")
end
