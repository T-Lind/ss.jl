# Two-impulse rendezvous demo: a chaser 10 km behind and 2 km below a
# target in a 400 km circular orbit closes in half an orbit.
#
# The burns are DESIGNED with linearized Clohessy-Wiltshire dynamics and
# VERIFIED by integrating both spacecraft with the full nonlinear two-body
# dynamics — the miss distance at arrival is the linearization error, which
# should be a few hundred meters at this range (and is exactly the error a
# terminal proximity-ops phase absorbs in a real rendezvous).
#
# Usage: julia --project scripts/run_rendezvous.jl

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Printf

const S = SatelliteSim

# target: 400 km circular equatorial
a = RE_MEAN + 400e3
n = sqrt(MU_EARTH / a^3)
T = 2pi / n
rt = (a, 0.0, 0.0)
vt = (0.0, sqrt(MU_EARTH / a), 0.0)

# chaser offset in RIC: 2 km below (radial -), 10 km behind (along-track -)
ric0 = (-2000.0, -10000.0, 0.0)
vric0 = (0.0, 3 * n * 2000.0 / 2, 0.0)   # ~natural drift for the radial offset

tof = T / 2
dv1, dv2, vreq = cw_two_impulse(ric0, vric0, n, tof)
@printf("CW design: tof %.1f min   dv1 %.2f m/s   dv2 %.2f m/s   total %.2f m/s\n",
        tof / 60, S.vnorm(dv1), S.vnorm(dv2), S.vnorm(dv1) + S.vnorm(dv2))

# --- nonlinear verification ------------------------------------------------
# RIC basis at the target
function ric_basis(r::S.V3, v::S.V3)
    xr = S.vunit(r)
    zc = S.vunit(S.vcross(r, v))
    ya = S.vcross(zc, xr)
    (xr, ya, zc)
end
xr, ya, zc = ric_basis(rt, vt)
to_eci(u) = S.vadd(S.vadd(S.vscale(xr, u[1]), S.vscale(ya, u[2])), S.vscale(zc, u[3]))

rc = S.vadd(rt, to_eci(ric0))
# chaser inertial velocity: target velocity + relative + frame rotation ω×ρ
wxr = S.vcross(S.vscale(zc, n), to_eci(ric0))
vc = S.vadd(vt, S.vadd(to_eci(vric0), wxr))
vc = S.vadd(vc, to_eci(dv1))               # burn 1

function two_body_step(r, v, dt)
    acc(rr) = S.vscale(rr, -MU_EARTH / S.vnorm(rr)^3)
    k1v = acc(r); k1r = v
    r2 = S.vadd(r, S.vscale(k1r, dt/2)); v2 = S.vadd(v, S.vscale(k1v, dt/2))
    k2v = acc(r2); k2r = v2
    r3 = S.vadd(r, S.vscale(k2r, dt/2)); v3 = S.vadd(v, S.vscale(k2v, dt/2))
    k3v = acc(r3); k3r = v3
    r4 = S.vadd(r, S.vscale(k3r, dt)); v4 = S.vadd(v, S.vscale(k3v, dt))
    k4v = acc(r4); k4r = v4
    (S.vadd(r, S.vscale(S.vadd(S.vadd(k1r, S.vscale(S.vadd(k2r, k3r), 2.0)), k4r), dt/6)),
     S.vadd(v, S.vscale(S.vadd(S.vadd(k1v, S.vscale(S.vadd(k2v, k3v), 2.0)), k4v), dt/6)))
end

rtn, vtn, rcn, vcn = rt, vt, rc, vc
dt = 0.5
steps = round(Int, tof / dt)
for _ in 1:steps
    global rtn, vtn = two_body_step(rtn, vtn, dt)
    global rcn, vcn = two_body_step(rcn, vcn, dt)
end
miss = S.vnorm(S.vsub(rcn, rtn))
relv = S.vnorm(S.vsub(S.vadd(vcn, to_eci(dv2)), vtn))  # after braking burn... (basis rotated; approx)
@printf("nonlinear check: miss at arrival %.0f m  (%.2f%% of initial range)\n",
        miss, 100 * miss / S.vnorm(to_eci(ric0)))
@printf("propellant (500 kg chaser, Isp 220): %.2f kg\n",
        impulsive_prop(500.0, S.vnorm(dv1) + S.vnorm(dv2), 220.0))
miss < 500 || error("rendezvous verification failed: miss $(miss) m")
println("OK — CW design verified against nonlinear propagation")
