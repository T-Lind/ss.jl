using Test
using SatelliteSim

@testset "SatelliteSim" begin

@testset "atmosphere (USSA76)" begin
    atm = USSA76()
    rho0, T0, p0, a0 = atmosphere_state(atm, 0.0)
    @test isapprox(rho0, 1.225; rtol = 1e-3)
    @test isapprox(T0, 288.15; atol = 0.01)
    @test isapprox(p0, 101325.0; rtol = 1e-6)
    @test isapprox(a0, 340.29; atol = 0.5)

    rho11, T11, _, _ = atmosphere_state(atm, 11019.0)  # tropopause (geometric)
    @test isapprox(T11, 216.65; atol = 0.2)
    @test isapprox(rho11, 0.3639; rtol = 0.01)

    rho50, _, _, _ = atmosphere_state(atm, 50.0e3)
    @test isapprox(rho50, 1.027e-3; rtol = 0.03)

    rho86, _, _, _ = atmosphere_state(atm, 86.0e3)
    @test isapprox(rho86, 6.958e-6; rtol = 0.05)

    rho120, _, _, _ = atmosphere_state(atm, 120.0e3)
    @test isapprox(rho120, 2.222e-8; rtol = 1e-6)

    # monotone decreasing density
    hs = 0:5e3:500e3
    rhos = [atmosphere_state(atm, Float64(h))[1] for h in hs]
    @test all(diff(rhos) .< 0)

    scaled = ScaledAtmosphere(atm, 1.2)
    @test atmosphere_state(scaled, 30e3)[1] ≈ 1.2 * atmosphere_state(atm, 30e3)[1]
end

@testset "gravity" begin
    r = (RE_EQ + 400e3, 0.0, 0.0)
    apm = gravity_accel(PointMassGravity(), r, 0.0)
    @test isapprox(apm[1], -MU_EARTH / (RE_EQ + 400e3)^2; rtol = 1e-12)
    aj2 = gravity_accel(J2Gravity(), r, 0.0)
    # J2 strengthens equatorial radial pull by ~1.5*J2*(Re/r)^2
    @test aj2[1] < apm[1]
    @test isapprox(aj2[1] / apm[1], 1 + 1.5 * 1.08262668e-3 * (RE_EQ / (RE_EQ + 400e3))^2; rtol = 1e-6)

    comp = CompositeGravity(PointMassGravity(), PointMassGravity())
    ac = gravity_accel(comp, r, 0.0)
    @test ac[1] ≈ 2 * apm[1]
end

@testset "frames" begin
    # geodetic round trip
    for (lat, lon, h) in ((0.7, -2.1, 400e3), (-1.2, 0.3, 120e3), (0.0, 0.0, 0.0))
        r = ecef_from_geodetic(lat, lon, h)
        lat2, lon2, h2 = geodetic_from_ecef(r)
        @test isapprox(lat2, lat; atol = 1e-9)
        @test isapprox(lon2, lon; atol = 1e-9)
        @test isapprox(h2, h; atol = 5e-3)  # Bowring single pass: mm-level
    end
    # kepler: circular orbit speed
    a = RE_EQ + 400e3
    r, v = state_from_elements(a, 0.0, 0.9, 0.5, 0.0, 0.0)
    @test isapprox(sqrt(sum(abs2, r)), a; rtol = 1e-12)
    @test isapprox(sqrt(sum(abs2, v)), sqrt(MU_EARTH / a); rtol = 1e-12)
    # vis-viva on an ellipse
    r2, v2 = state_from_elements(a, 0.3, 1.0, 0.1, 0.2, 2.0)
    eps1 = sum(abs2, v2) / 2 - MU_EARTH / sqrt(sum(abs2, r2))
    @test isapprox(eps1, -MU_EARTH / 2a; rtol = 1e-10)
end

@testset "aero tables" begin
    t = Table1D([0.0, 1.0, 2.0], [10.0, 20.0, 40.0])
    @test interp1(t, -1.0) == 10.0
    @test interp1(t, 3.0) == 40.0
    @test interp1(t, 0.5) == 15.0
    aero = default_capsule_aero()
    @test cd_coeff(aero, 25.0, 0.0) ≈ 1.52
    @test cd_coeff(aero, 25.0, 0.1) > cd_coeff(aero, 25.0, 0.0)  # off-trim drag rise
    @test SatelliteSim.cm_coeff(aero, 5.0, 0.1, 0.0) < 0          # restoring moment
    @test SatelliteSim.cm_coeff(aero, 5.0, -0.1, 0.0) > 0
end

@testset "heating" begin
    q = heating_convective(1e-4, 7500.0, 1.8)
    @test q > 0
    # hand check: 1.74153e-4*sqrt(1e-4/1.8)*7500^3
    @test isapprox(q, 1.74153e-4 * sqrt(1e-4 / 1.8) * 7500.0^3; rtol = 1e-12)
    @test heating_radiative(1e-4, 7500.0, 1.8) == 0.0   # negligible below 9 km/s
    @test heating_radiative(1e-4, 11000.0, 1.8) > 0.0
    @test wall_temperature(1e6, 0.85) > 1500
end

@testset "orbit propagation accuracy" begin
    # circular orbit, point-mass gravity: energy & radius conserved by RK4
    a = RE_EQ + 300e3
    r0, v0 = state_from_elements(a, 0.0, 0.9, 0.0, 0.0, 0.0)
    veh = default_reentry_pod()
    scn = Scenario(vehicle = veh, gravity = PointMassGravity(),
                   r0 = r0, v0 = v0, h_ei = 0.0,   # aero off everywhere
                   dt_orbit = 2.0, t_max = 5600.0) # ~ one period
    res = simulate(scn; log_dt_orbit = 60.0)
    rlast = res.log.vin[end] # inertial speed at end
    @test isapprox(rlast, sqrt(MU_EARTH / a); rtol = 1e-7)
    @test res.terminated == :timeout
end

@testset "propulsion" begin
    st = Stage(:test, 100.0, 900.0, 10.0e3, 300.0, 0.1)
    @test isapprox(stage_mdot(st), 10.0e3 / (9.80665 * 300.0); rtol = 1e-12)
    @test isapprox(stage_burn_time(st), 900.0 / stage_mdot(st); rtol = 1e-12)
    # pressure correction: sea level thrust < vacuum thrust
    @test stage_thrust(st, 101325.0) < st.thrust_vac
    @test stage_thrust(st, 0.0) == st.thrust_vac
    # Tsiolkovsky
    dv = stage_dv(st, 400.0)
    @test isapprox(dv, 9.80665 * 300.0 * log(1400.0 / 500.0); rtol = 1e-12)

    lv = default_moon_rocket()
    @test liftoff_mass(lv) > 50e3
    # the stack should carry ~LEO + TLI ideal delta-v with generous losses
    m_above(k) = SatelliteSim.stack_mass_above(lv, k + 1; fairing = k < 2)
    total_dv = sum(stage_dv(lv.stages[k], m_above(k)) for k in 1:3)
    @test 12.0e3 < total_dv < 15.0e3
end

@testset "moon ephemeris" begin
    r0, v0 = state_from_elements(RE_EQ + 200e3, 0.0, deg2rad_(28.5), 0.2, 0.0, 0.0)
    eph = coplanar_moon(r0, v0; phase0 = deg2rad_(90.0))
    # circular at the lunar distance for all t
    for t in (0.0, 1e5, 1e6)
        @test isapprox(sqrt(sum(abs2, moon_position(eph, t))), A_MOON; rtol = 1e-12)
    end
    # period: back to the same spot after a sidereal month
    s0 = moon_position(eph, 0.0)
    s1 = moon_position(eph, 2pi / N_MOON)
    @test all(isapprox.(s0, s1; atol = 1.0))
    # coplanar with the orbit: moon position ⟂ orbit normal
    h = (r0[2]*v0[3]-r0[3]*v0[2], r0[3]*v0[1]-r0[1]*v0[3], r0[1]*v0[2]-r0[2]*v0[1])
    hn = sqrt(sum(abs2, h))
    @test abs(sum(moon_position(eph, 12345.0) .* h) / (hn * A_MOON)) < 1e-12
    # velocity is the analytic derivative
    dt = 1.0
    sfd = (moon_position(eph, 1e5 + dt) .- moon_position(eph, 1e5 - dt)) ./ 2dt
    @test all(isapprox.(moon_velocity(eph, 1e5), sfd; rtol = 1e-6))
end

@testset "ascent to orbit" begin
    lv = default_moon_rocket()
    guid, asc = tune_ascent(lv, AscentGuidance())
    @test asc.reached_orbit
    el = asc.elements
    @test 180e3 < el.rp - RE_MEAN < 220e3     # near-circular parking orbit
    @test 180e3 < el.ra - RE_MEAN < 220e3
    @test isapprox(rad2deg_(el.i), 28.5; atol = 1.0)
    @test abs(asc.gamma_cut) < deg2rad_(0.1)
    @test asc.prop_left[end] == lv.stages[end].mprop  # kick stage untouched
    names = [e.name for e in asc.events]
    @test :liftoff in names && :seco in names
    @test :fairing_jettison in names
    # max-q in a sane band for a small launcher
    @test 20e3 < maximum(asc.log.qbar) < 90e3

    # A stage burns exactly its propellant load. The final step of a burn is
    # trimmed to land on depletion, so the mass at separation is the lift-off
    # mass less all of stage 1's propellant and its dry structure — no
    # phantom propellant burned past empty, none thrown away short of it.
    sep1 = asc.events[findfirst(e -> e.name === :sep_sable1, asc.events)]
    @test isapprox(sep1.m,
                   liftoff_mass(lv) - lv.stages[1].mprop - lv.stages[1].mdry;
                   atol = 0.5)
    @test isapprox(sep1.t, stage_burn_time(lv.stages[1]); atol = 0.05)
end

@testset "strap-on boosters" begin
    base = default_moon_rocket()
    strap = Stage(:strap, 900.0, 12000.0, 380.0e3, 285.0, 0.32,
                  PROPELLANTS[:kerolox], 2)
    mk(nb; thr = 1.0, delay = 0.0, sep = 0.0) = LaunchVehicle(
        name = "Sable-H", stages = base.stages, fairing_mass = base.fairing_mass,
        payload_mass = base.payload_mass, sref = base.sref, cd = base.cd,
        boosters = nb == 0 ? BoosterSet[] :
            [BoosterSet(stage = strap, count = nb, core_throttle = thr,
                        ignition_delay = delay, sep_delay = sep)])

    # a vehicle with no boosters is the vehicle it always was
    @test liftoff_mass(mk(0)) == liftoff_mass(base)
    @test frontal_area(mk(0), Bool[]) == base.sref

    lv2 = mk(2)
    @test liftoff_mass(lv2) ≈ liftoff_mass(base) + 2 * (900.0 + 12000.0)
    @test pad_thrust(lv2) ≈ pad_thrust(base) + 2 * stage_thrust(strap, 101325.0)
    # attached boosters put their own frontal area into the flow, then stop
    @test frontal_area(lv2, [true]) > base.sref
    @test frontal_area(lv2, [false]) == base.sref

    guid, asc = tune_ascent(lv2, AscentGuidance())
    @test asc.reached_orbit
    names = [e.name for e in asc.events]
    @test :ignition_strap in names && :burnout_strap in names && :sep_strap in names
    # the set lights on the pad, runs its own burn time, and goes at once
    tign = asc.events[findfirst(e -> e.name === :ignition_strap, asc.events)].t
    tout = asc.events[findfirst(e -> e.name === :burnout_strap, asc.events)].t
    tsep = asc.events[findfirst(e -> e.name === :sep_strap, asc.events)].t
    @test tign == 0.0
    @test isapprox(tout - tign, stage_burn_time(strap); atol = 0.2)
    @test tsep == tout                                # sep_delay = 0
    # and they are gone before the core stages
    @test tsep < asc.events[findfirst(e -> e.name === :sep_sable1, asc.events)].t
    # mass drops by the whole set's dry mass at separation, not one booster's
    im = findfirst(t -> t > tsep, asc.log.t)
    @test asc.log.m[im] < asc.log.m[findlast(t -> t <= tsep, asc.log.t)] - 1500.0

    # holding the core down leaves it burning after the sides are away
    hot  = tune_ascent(mk(2), AscentGuidance())[2]
    cool = tune_ascent(mk(2; thr = 0.6), AscentGuidance())[2]
    sep1(r) = r.events[findfirst(e -> e.name === :sep_sable1, r.events)].t
    @test sep1(cool) > sep1(hot) + 20.0

    # a delayed set waits on the pad, and a separation delay carries the
    # dead weight for exactly that long
    late = simulate_ascent(mk(2; delay = 20.0), guid)
    @test late.events[findfirst(e -> e.name === :ignition_strap, late.events)].t ≥ 20.0
    hang = simulate_ascent(mk(2; sep = 12.0), guid)
    hb = hang.events[findfirst(e -> e.name === :burnout_strap, hang.events)].t
    hs = hang.events[findfirst(e -> e.name === :sep_strap, hang.events)].t
    @test isapprox(hs - hb, 12.0; atol = 0.2)

    # the kick scan turns strap-ons from a liability into a gain: at the
    # reference kick the extra impulse is spent lofting the stack
    _, flat = tune_ascent(lv2, AscentGuidance())
    _, opt  = tune_ascent(lv2, AscentGuidance(); optimize_kick = true)
    @test opt.reached_orbit && opt.m > flat.m
    @test opt.m > tune_ascent(base, AscentGuidance())[2].m   # actually helps
    @test abs(opt.h_cut - 200e3) < 2e3

    # geometry: a set is one section of `count` bodies beside the core
    m1, s1 = rocket_mesh(base; nseg = 20)
    m2, s2 = rocket_mesh(lv2; nseg = 20)
    b = only(filter(s -> s.name === :booster1, s2))
    @test isempty(filter(s -> s.name === :booster1, s1))
    @test mesh_volume(TriMesh(m2.tris[b.t0:b.t1])) > 0
    @test length(m2.tris) > length(m1.tris)
    # four boosters are twice the triangles of two, and stand off the axis
    m4, s4 = rocket_mesh(mk(4); nseg = 20)
    b4 = only(filter(s -> s.name === :booster1, s4))
    @test (b4.t1 - b4.t0 + 1) == 2 * (b.t1 - b.t0 + 1)
    off = [hypot(p[2], p[3]) for t in m2.tris[b.t0:b.t1] for p in t]
    @test minimum(off) > 0.1                          # none of it on the core axis
end

@testset "circumlunar free return" begin
    ms = moonshot()
    cis = ms.cislunar
    @test cis.outcome == :entry_interface
    @test isapprox(cis.perilune_alt, 2000e3; atol = 30e3)
    @test isapprox(cis.vac_perigee_alt, 50e3; atol = 5e3)
    @test 2.5e3 < cis.dv_tli < 3.4e3
    # direct free return: entry on the FIRST post-flyby perigee pass, no
    # phasing loop back out past the Moon (regression: the design once
    # converged onto a 19.7-day two-revolution return)
    @test cis.miss_passes == 0
    @test cis.t - cis.t_perilune < 5.0 * 86400.0   # return leg ~ outbound leg
    rmax = maximum(hypot.(cis.log.rx, cis.log.ry, cis.log.rz))
    @test rmax < 1.15 * A_MOON                     # never far beyond the lunar distance
    # entry interface speed near lunar-return values
    ent = ms.entry
    ei = ent.events[findfirst(e -> e.name == :entry_interface, ent.events)]
    @test 10.5e3 < ei.vrel < 11.3e3
    @test ent.terminated == :splashdown
    @test ent.v_splash < 8.0
    @test ent.peak_gload < 30.0              # inside a survivable ballistic corridor
    # energy sanity on the coast: two-body + moon only, no drag above EI
    @test cis.m < SatelliteSim.liftoff_mass(ms.lv)

    # configurability: a custom launch vehicle flows through the whole chain
    lv = default_moon_rocket(payload = 300.0)
    ms2 = moonshot(lv = lv, hp_moon = 1500e3)
    @test ms2.lv === lv
    @test ms2.entry_scn.vehicle.mass == 300.0
    @test isapprox(ms2.cislunar.perilune_alt, 1500e3; atol = 75e3)
end

@testset "numerics: order, invariants, step independence" begin
    # --- RK4 is actually 4th order --------------------------------------
    # A closed two-body orbit must return to where it started; the closure
    # error is pure truncation, so halving the step must cut it ~16x.
    g = PointMassGravity()
    a = RE_MEAN + 400e3
    r0 = (a, 0.0, 0.0); v0 = (0.0, sqrt(MU_EARTH / a), 0.0)
    T = 2pi * sqrt(a^3 / MU_EARTH)
    V = SatelliteSim
    function closure(dt)
        r, v, t = r0, v0, 0.0
        n = round(Int, T / dt); h = T / n
        for _ in 1:n
            k1v = gravity_accel(g, r, t);                                   k1r = v
            k2v = gravity_accel(g, V.vadd(r, V.vscale(k1r, h/2)), t+h/2);   k2r = V.vadd(v, V.vscale(k1v, h/2))
            k3v = gravity_accel(g, V.vadd(r, V.vscale(k2r, h/2)), t+h/2);   k3r = V.vadd(v, V.vscale(k2v, h/2))
            k4v = gravity_accel(g, V.vadd(r, V.vscale(k3r, h)),   t+h);     k4r = V.vadd(v, V.vscale(k3v, h))
            r = V.vadd(r, V.vscale(V.vadd(V.vadd(k1r, V.vscale(V.vadd(k2r,k3r),2.0)), k4r), h/6))
            v = V.vadd(v, V.vscale(V.vadd(V.vadd(k1v, V.vscale(V.vadd(k2v,k3v),2.0)), k4v), h/6))
            t += h
        end
        V.vnorm(V.vsub(r, r0))
    end
    e1, e2 = closure(16.0), closure(8.0)
    @test 3.7 < log2(e1 / e2) < 4.4        # observed order ~4
    @test e2 < 0.05                        # and small in absolute terms

    # --- Jacobi constant on the real cislunar coast ----------------------
    # The Moon is a circular coplanar ephemeris, so the coast IS the circular
    # restricted three-body problem and C_J is a true invariant: any drift is
    # integration error on the production dynamics, not a modelling choice.
    ms = moonshot()
    L = ms.cislunar.log
    n = N_MOON
    f = MU_MOON / (MU_EARTH + MU_MOON)
    i2 = max(2, length(L.t) ÷ 4)
    zh = V.vunit(V.vcross((L.mx[1],L.my[1],L.mz[1]), (L.mx[i2],L.my[i2],L.mz[i2])))
    om = V.vscale(zh, n)
    function jacobi(i)
        r = (L.rx[i], L.ry[i], L.rz[i]); v = (L.vx[i], L.vy[i], L.vz[i])
        m = (L.mx[i], L.my[i], L.mz[i])
        rho  = V.vsub(r, V.vscale(m, f))
        rhod = V.vsub(v, V.vscale(V.vcross(om, m), f))
        vrot = V.vsub(rhod, V.vcross(om, rho))
        perp2 = V.vdot(rho, rho) - V.vdot(rho, zh)^2
        U = 0.5 * n^2 * perp2 + MU_EARTH / V.vnorm(r) +
            MU_MOON / max(V.vnorm(V.vsub(m, r)), 1.0)
        2U - V.vdot(vrot, vrot)
    end
    cj = [jacobi(i) for i in eachindex(L.t) if L.phase[i] >= 2]
    @test length(cj) > 100
    @test (maximum(cj) - minimum(cj)) / abs(sum(cj)/length(cj)) < 2.0e-4

    # --- the answer must not depend on the step size ---------------------
    # Regression: the free-return corrector used to accept anything within
    # 3 km of the perigee target, so two distinct designs both qualified and
    # the search landed on either depending on numerical noise — 2 km of
    # perigee and 0.6 g of peak load with no code change. Halving the coast
    # step must now move the flown result by less than the tolerance allows.
    fine = moonshot(cis_eta = SatelliteSim.CIS_ETA / 2)
    @test abs(fine.cislunar.perilune_alt - ms.cislunar.perilune_alt) < 200.0
    @test abs(fine.cislunar.vac_perigee_alt - ms.cislunar.vac_perigee_alt) < 600.0
    @test abs(fine.entry.peak_gload - ms.entry.peak_gload) < 0.15
    # and both must actually sit on the requested target
    for r in (ms, fine)
        @test abs(r.cislunar.vac_perigee_alt - 50e3) < 2 * SatelliteSim.PERIGEE_TOL
    end
end

@testset "rigid body" begin
    # rotation basics
    q = quat_axis_angle((0.0, 0.0, 1.0), pi/2)
    v = qrotate(q, (1.0, 0.0, 0.0))
    @test all(isapprox.(v, (0.0, 1.0, 0.0); atol = 1e-12))
    @test all(isapprox.(qrotate_inv(q, v), (1.0, 0.0, 0.0); atol = 1e-12))
    q2 = quat_from_to((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    @test all(isapprox.(qrotate(q2, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0); atol = 1e-12))

    # torque-free tumble: energy and |angular momentum| conserved (RK4)
    I = (110.0, 137.0, 150.0)
    w = (0.3, -0.5, 0.8)
    q = (1.0, 0.0, 0.0, 0.0)
    E0 = rot_energy(I, w)
    L0 = sqrt(sum(abs2, ang_momentum(I, w)))
    dt = 0.01
    for _ in 1:20_000
        k1 = euler_wdot(I, w, (0.0, 0.0, 0.0))
        w2 = w .+ 0.5dt .* k1
        k2 = euler_wdot(I, w2, (0.0, 0.0, 0.0))
        w3 = w .+ 0.5dt .* k2
        k3 = euler_wdot(I, w3, (0.0, 0.0, 0.0))
        w4 = w .+ dt .* k3
        k4 = euler_wdot(I, w4, (0.0, 0.0, 0.0))
        w = w .+ (dt / 6) .* (k1 .+ 2 .* k2 .+ 2 .* k3 .+ k4)
    end
    @test isapprox(rot_energy(I, w), E0; rtol = 1e-8)
    @test isapprox(sqrt(sum(abs2, ang_momentum(I, w))), L0; rtol = 1e-8)
end

@testset "rcs" begin
    sys = default_pod_rcs()
    auth = torque_authority(sys)
    @test all(a -> a > 0, auth)                    # every axis controllable
    # couples are pure torques: net force cancels
    F = (0.0, 0.0, 0.0)
    for t in sys.thrusters
        F = F .+ t.thrust .* t.dir
    end
    @test all(abs.(F) .< 1e-9)
    # limit cycle: hand formula
    lc = limit_cycle_prop(1000.0, 28.0, 220.0, 0.01, deg2rad_(5.0), 86400.0;
                          nthr = 2, thrust = 10.0)
    w = 28.0 * 0.01 / 1000.0
    cycles = 86400.0 / (2 * deg2rad_(5.0) / w)
    @test isapprox(lc, cycles * 2 * 10.0 / (9.80665 * 220.0) * 0.01; rtol = 1e-12)
    # slew: bang-bang time
    p, ts = slew_prop(1000.0, 28.0, 220.0, 1.0 * pi)
    @test isapprox(ts, 2 * sqrt(pi * 1000.0 / 28.0); rtol = 1e-12)
    b = cruise_rcs_budget(default_kick_rcs(), 1000.0; duration = 20 * 86400.0)
    @test b.margin > 0                             # sized for the cruise
end

@testset "6-DOF entry vs 4-DOF" begin
    # Compared ballistically on purpose. The 4-DOF model holds the trim lift
    # vector at the commanded bank angle by construction; the 6-DOF model
    # lets the capsule roll. With lift those are different physical problems
    # — see the lifting-entry testset below — so the question "do the two
    # integrators agree on the same dynamics" is only well posed at zero lift.
    veh = default_reentry_pod(cl_trim_hyp = 0.0)
    scn = scenario_from_elements(DeorbitElements(), veh)
    r4 = simulate(scn)
    r6 = simulate_entry6(scn; rcs = default_pod_rcs())   # wind-hold coast
    @test r6.terminated == :splashdown
    @test isapprox(r6.peak_gload, r4.peak_gload; rtol = 0.05)
    @test isapprox(r6.heat_load, r4.heat_load; rtol = 0.05)
    @test haversine(r4.lat_splash, r4.lon_splash,
                    r6.lat_splash, r6.lon_splash) < 20e3
    @test r6.rcs_used < 0.05                       # grams, not kilograms
    @test r6.max_alpha_after_peak < deg2rad_(10.0) # stable through supersonic
    # uncontrolled coast arrives far off-trim yet the shape self-rights
    r6f = simulate_entry6(scn)
    @test r6f.terminated == :splashdown
    ei = findfirst(e -> e.name == :entry_interface, r6f.events)
    i = findfirst(t -> t >= r6f.events[ei].t, r6f.log.t)
    @test r6f.log.alpha_t[i] > deg2rad_(45.0)      # broadside at EI...
    @test r6f.peak_gload < 1.10 * r4.peak_gload    # ...still survivable
    # tipoff-rate damping costs grams
    r6r = simulate_entry6(scn; rcs = default_pod_rcs(), rcs_mode = :rate_damp,
                          w0 = (deg2rad_(3.0), deg2rad_(-2.0), deg2rad_(1.5)))
    @test r6r.rcs_used < 0.05
end

@testset "TLI dispersion + TCM" begin
    ms = moonshot(tli_mag_err = 0.003, tli_point_err = deg2rad_(0.3))
    @test ms.cruise !== nothing
    c = ms.cruise
    @test 5.0 < c.tcm_dv < 150.0
    @test c.tcm_prop < 23.0                        # inside the kick margin
    @test c.rcs.margin > 0
    @test isapprox(ms.cislunar.perilune_alt, 2000e3; atol = 60e3)
    @test abs(ms.cislunar.vac_perigee_alt - 35e3) < 25e3
    @test ms.cislunar.miss_passes == 0             # corrected cruise comes straight home
    @test ms.entry.terminated == :splashdown
    @test ms.entry.peak_gload < 30.0
end

@testset "mission config (TOML)" begin
    spec = load_mission(joinpath(@__DIR__, "..", "missions", "moonshot.toml"))
    @test spec.pod_mass == 350.0
    @test spec.h_park == 200e3
    @test spec.hp_moon == 2000e3
    @test length(spec.lv.stages) == 3
    lv0 = default_moon_rocket()
    @test liftoff_mass(spec.lv) ≈ liftoff_mass(lv0)
    @test spec.lv.stages[3].isp_vac == lv0.stages[3].isp_vac
    @test isempty(spec.lv.boosters)
    ms = run_mission(spec)
    @test ms.cislunar.outcome == :entry_interface
    @test isapprox(ms.cislunar.perilune_alt, 2000e3; atol = 30e3)

    # a [[vehicle.booster]] set round-trips through the spec, and a spec that
    # has one flies with the pitch kick re-found for it
    heavy = mktemp() do path, io
        write(io, """
        [mission]
        name = "heavy"
        pod_mass_kg = 350.0
        [vehicle]
        name = "Sable-H"
        diameter_m = 1.8
        [[vehicle.stage]]
        name = "sable1"
        prop_kg = 42000.0
        dry_kg = 3800.0
        thrust_vac_kn = 950.0
        isp_vac_s = 305.0
        exit_area_m2 = 0.80
        engines = 5
        [[vehicle.stage]]
        name = "sable2"
        prop_kg = 9500.0
        dry_kg = 900.0
        thrust_vac_kn = 95.0
        isp_vac_s = 345.0
        [[vehicle.stage]]
        name = "sablek"
        prop_kg = 950.0
        dry_kg = 140.0
        thrust_vac_kn = 15.0
        isp_vac_s = 315.0
        propellant = "hypergolic"
        [[vehicle.booster]]
        name = "strap"
        count = 2
        diameter_m = 1.53
        prop_kg = 12000.0
        dry_kg = 900.0
        thrust_vac_kn = 380.0
        isp_vac_s = 285.0
        exit_area_m2 = 0.32
        core_throttle = 0.75
        sep_delay_s = 2.0
        """)
        close(io)
        load_mission(path)
    end
    b = only(heavy.lv.boosters)
    @test b.count == 2
    @test b.core_throttle == 0.75
    @test b.sep_delay == 2.0
    @test b.stage.diameter == 1.53
    @test liftoff_mass(heavy.lv) ≈ liftoff_mass(spec.lv) + 2 * (900.0 + 12000.0)
    msh = run_mission(heavy)
    @test msh.cislunar.outcome == :entry_interface
    # the strap-ons pay for themselves: more mass reaches the parking orbit
    @test msh.ascent.m > ms.ascent.m
end

@testset "mesh + mass properties" begin
    # unit box: exact polyhedral integrals
    b = box_mesh((0.0, 0.0, 0.0), (2.0, 1.0, 1.0))
    @test mesh_volume(b) ≈ 2.0
    mb = mass_properties(b, 12.0)
    @test all(isapprox.(mb.cg, (1.0, 0.5, 0.5); atol = 1e-12))
    @test isapprox(mb.inertia[1], 12.0 / 12 * (1 + 1); rtol = 1e-12)
    @test isapprox(mb.inertia[2], 12.0 / 12 * (4 + 1); rtol = 1e-12)
    # sphere: analytic within mesh resolution
    prof = [(cos(s), sin(s)) for s in range(0.0, 1.0 * pi; length = 41)]
    sph = lathe_mesh([(x, r) for (x, r) in prof]; nseg = 64)
    @test isapprox(mesh_volume(sph), 4pi / 3; rtol = 0.005)
    ms_ = mass_properties(sph, 100.0)
    @test isapprox(ms_.inertia[1], 40.0; rtol = 0.005)
    # STL round trip preserves the solid
    path = joinpath(mktempdir(), "sph.stl")
    write_stl(path, sph)
    sph2 = read_stl(path)
    @test length(sph2) == length(sph)
    @test isapprox(mesh_volume(sph2), mesh_volume(sph); rtol = 1e-6)
    # procedural launcher: closed, positive volume, one section per stage plus
    # the pod capsule, its cabin interior, and the fairing shell over both
    rk, secs = rocket_mesh(diameter = 2.0, prop_masses = [10_000.0, 2_000.0])
    @test mesh_volume(rk) > 0
    @test length(secs) == 6
    @test [s.name for s in secs[end-3:end]] == [:pod, :glass, :cabin, :fairing]
    @test issorted([s.x0 for s in secs])
    # triangle ranges tile the merged soup exactly, in order
    @test secs[1].t0 == 1 && secs[end].t1 == length(rk)
    @test all(secs[i+1].t0 == secs[i].t1 + 1 for i in 1:length(secs)-1)
    # each section is itself a closed solid (viewers detach them individually)
    @test all(mesh_volume(TriMesh(rk.tris[s.t0:s.t1])) > 0 for s in secs)
    # glass and cabin live inside the pod's shell, so all three share an extent
    pod, cab = secs[end-3], secs[end-1]
    @test all(s -> s.x0 == pod.x0 && s.x1 == pod.x1, secs[end-3:end-1])
    @test mesh_volume(TriMesh(rk.tris[cab.t0:cab.t1])) <
          mesh_volume(TriMesh(rk.tris[pod.t0:pod.t1]))
    # barrel volume actually swallows the propellant it was sized for
    @test mesh_volume(rk) > (10_000.0 + 2_000.0) / 1020.0
end

@testset "scalar targeting" begin
    # a plain root, and one where the target is not zero
    r = find_root(x -> x^2 - 2, 0.0, 3.0)
    @test converged(r) && isapprox(r.x, sqrt(2); atol = 1e-3)
    r2 = find_root(x -> x^3, -1.0, 4.0; target = 8.0)
    @test converged(r2) && isapprox(r2.x, 2.0; atol = 1e-3)
    # every evaluation is recorded — with second-long missions the path is
    # most of what you learn from a solve
    @test length(r.history) == r.iterations >= 3
    @test all(h -> 0.0 <= h[1] <= 3.0, r.history)

    # a hit exactly on a bound short-circuits
    @test converged(find_root(x -> x - 1.0, 1.0, 5.0; ftol = 1e-9))
    # no sign change: report the closest end rather than a wrong answer
    nb = find_root(x -> x^2 + 1, 0.0, 3.0)
    @test nb.status === :no_bracket && nb.x == 0.0

    # a failing design is "past the feasible edge", not a crash: the search
    # retreats from an infeasible bound and still finds the boundary
    hard(x) = x > 2.5 ? error("ascent failed to reach orbit") : 1.0 - x
    rf = find_root(hard, 0.0, 10.0)
    @test converged(rf) && isapprox(rf.x, 1.0; atol = 1e-2)
    # the clipped interval is reported, so a caller can say *why* it clipped
    @test rf.hi < 10.0 && rf.lo == 0.0
    # a clean solve reports the bounds it was given, not the final bracket
    @test r.lo == 0.0 && r.hi == 3.0
    @test find_root(x -> NaN, 0.0, 1.0).status === :infeasible
    # NaN behaves the same as a throw
    @test converged(find_root(x -> x > 2.5 ? NaN : 1.0 - x, 0.0, 10.0))

    # Illinois beats bisection's iteration count on a nasty one-sided root
    steep(x) = exp(x) - 5.0
    rs = find_root(steep, 0.0, 40.0; xtol = 1e-6)
    @test converged(rs) && isapprox(rs.x, log(5); atol = 1e-4)
    @test rs.iterations < 30
    # the budget is honoured even when it cannot converge in time
    @test find_root(steep, 0.0, 40.0; xtol = 1e-14, max_iter = 6).iterations <= 6

    # decreasing functions and reversed bounds work the same
    @test isapprox(find_root(x -> 5.0 - x, 10.0, 0.0).x, 5.0; atol = 1e-3)
end

@testset "propellants & engines" begin
    # bulk density of a mixture: mass of both fluids over the volume of both
    kero = PROPELLANTS[:kerolox]
    vf, vox = propellant_volumes(kero, 1000.0)
    @test isapprox(1000.0 / (vf + vox), bulk_density(kero); rtol = 1e-12)
    @test isapprox(vox / vf * kero.rho_ox / kero.rho_fuel, kero.mr; rtol = 1e-12)
    # the spread across propellants is the whole point: hydrolox tanks are
    # three times the volume of kerolox ones for the same propellant mass
    @test 1000 < bulk_density(kero) < 1050
    @test 330 < bulk_density(PROPELLANTS[:hydrolox]) < 360
    @test bulk_density(kero) / bulk_density(PROPELLANTS[:hydrolox]) > 2.8
    # monopropellant: no oxidiser volume at all
    @test propellant_volumes(PROPELLANTS[:hydrazine], 100.0)[2] == 0.0
    @test_throws ArgumentError propellant(:unobtainium)
    @test_throws ArgumentError lookup_engine(:nonesuch)

    # a catalogue engine's derived exit area must reproduce its quoted
    # sea-level Isp through the same pressure correction the sim flies
    for e in values(ENGINES)
        e.isp_sl > 0 || continue
        st = Stage(:t, 0.0, 0.0, e.thrust_vac, e.isp_vac, e.ae)
        @test isapprox(e.isp_vac * stage_thrust(st, SatelliteSim.P0_SEA) / e.thrust_vac,
                       e.isp_sl; rtol = 1e-9)
        @test stage_thrust(st, 0.0) == e.thrust_vac          # vacuum unchanged
    end

    # clustering multiplies thrust and exit area, never Isp
    m1 = ENGINES[:merlin_1d]
    s9 = sized_stage(:booster; engine = :merlin_1d, n_engines = 9,
                     prop_mass = 411_000.0, diameter = 3.66)
    @test s9.thrust_vac ≈ 9 * m1.thrust_vac
    @test s9.ae ≈ 9 * m1.ae
    @test s9.isp_vac == m1.isp_vac
    @test s9.n_engines == 9
    # ...and lands near a real first stage of that size (~25.6 t dry)
    @test 22_000 < s9.mdry < 29_000

    # dry-mass fraction has to improve with size, or the curve is wrong
    small = stage_mass(m1, 1, 10_000.0, 1.8)
    big   = stage_mass(m1, 9, 400_000.0, 3.7)
    @test big.dry_fraction < small.dry_fraction
    @test small.tanks > 0 && big.engines ≈ 9 * m1.mass
    # cryogenic stages carry insulation, storables do not
    @test stage_mass(m1, 1, 10_000.0, 1.8).insulation > 0
    @test stage_mass(ENGINES[:aj10], 1, 1_000.0, 1.8).insulation == 0
    # an explicit dry mass overrides the estimate entirely
    @test sized_stage(:s; engine = :aj10, prop_mass = 950.0,
                      dry_mass = 140.0).mdry == 140.0

    # a legacy 6-argument Stage still works and is assumed dense and single-engine
    old = Stage(:legacy, 100.0, 1000.0, 20e3, 300.0, 0.0)
    @test old.n_engines == 1 && old.prop.name === :kerolox
    # propellant choice drives physical size through the mesh
    lens = map((:kerolox, :hydrolox)) do pk
        st = Stage(:s, 100.0, 9500.0, 20e3, 300.0, 0.0, PROPELLANTS[pk], 1)
        lv = LaunchVehicle(name = "t", stages = [st], fairing_mass = 50.0,
                           payload_mass = 100.0, sref = pi * 0.9^2,
                           cd = SatelliteSim.LV_CD_TABLE)
        last(rocket_mesh(lv; nseg = 16)[2]).x1
    end
    @test lens[2] > lens[1] + 5.0            # hydrolox stack is metres longer
    # engine count shows up as bells: more engines, more triangles
    ntri(n) = length(rocket_mesh(; prop_masses = [42000.0], n_engines = [n],
                                 nseg = 16)[1].tris)
    @test ntri(9) > ntri(5) > ntri(1)
    @test_throws ArgumentError rocket_mesh(prop_masses = [1.0, 2.0],
                                           n_engines = [1])
    @test_throws ArgumentError rocket_mesh(prop_masses = [1.0], diameters = [0.0])

    # per-stage diameter: the same propellant in a wider barrel is shorter,
    # and the interstage becomes a transition cone rather than breaking
    barrel(d) = (m, s) = rocket_mesh(; prop_masses = [42000.0, 9500.0],
                                     diameters = [d, 1.8], nseg = 20)[2][1]
    @test (barrel(2.6).x1 - barrel(2.6).x0) < (barrel(1.8).x1 - barrel(1.8).x0)
    stepped, ssec = rocket_mesh(; prop_masses = [42000.0, 9500.0, 950.0],
                                diameters = [2.6, 1.8, 1.2], nseg = 20)
    @test all(mesh_volume(TriMesh(stepped.tris[s.t0:s.t1])) > 0 for s in ssec)
    @test issorted([s.x0 for s in ssec])
    # the capsule and fairing ride on the topmost stage, so they follow it
    narrow = rocket_mesh(; prop_masses = [42000.0, 950.0],
                         diameters = [2.6, 1.0], nseg = 20)[2]
    wide   = rocket_mesh(; prop_masses = [42000.0, 950.0],
                         diameters = [2.6, 2.6], nseg = 20)[2]
    fw(secs) = only(filter(s -> s.name === :fairing, secs))
    @test (fw(narrow).x1 - fw(narrow).x0) < (fw(wide).x1 - fw(wide).x0)
    # --- interstage transition cone ------------------------------------
    # A neighbour of a different diameter is joined by a cone at the lower
    # stage's top, at a fixed shallow wall angle in either direction. It is
    # the lower stage that grows: the cone is its structure, not the tank's.
    tl = SatelliteSim._taper_len
    @test tl(0.9, 0.9, 1.8) == 0.0                      # uniform: no cone
    @test tl(1.3, 0.9, 2.6) ≈ 0.4 / tan(SatelliteSim.TAPER_HALFANGLE)
    @test tl(0.6, 0.9, 1.2) ≈ 0.3 / tan(SatelliteSim.TAPER_HALFANGLE)
    @test tl(0.9, 1.3, 1.8) > 0                          # flares out too
    s1len(dias) = (s = rocket_mesh(; prop_masses = [42000.0, 9500.0],
                                   diameters = dias, nseg = 20)[2][1];
                   s.x1 - s.x0)
    @test s1len([1.8, 1.2]) > s1len([1.8, 1.8])          # necking down adds a cone
    @test s1len([1.8, 2.6]) > s1len([1.8, 1.8])          # flaring out adds one too
    # the cone lands exactly on the upper stage's radius: no ledge, no gap
    for dias in ([2.6, 1.8], [1.2, 1.8], [1.8, 1.8])
        m, s = rocket_mesh(; prop_masses = [42000.0, 9500.0],
                           diameters = dias, nseg = 20)
        xj = s[1].x1
        rim = [hypot(p[2], p[3]) for t in m.tris[s[1].t0:s[1].t1] for p in t
               if abs(p[1] - xj) < 1e-9]
        @test !isempty(rim)
        @test maximum(rim) ≈ dias[2] / 2 rtol = 0.02      # meets the stage above
    end
    # both directions stay watertight and positively oriented
    for dias in ([2.6, 1.8, 1.2], [1.2, 1.8, 2.6], [1.0, 2.6, 1.4])
        m, s = rocket_mesh(; prop_masses = [42000.0, 9500.0, 950.0],
                           diameters = dias, nseg = 20)
        @test all(mesh_volume(TriMesh(m.tris[c.t0:c.t1])) > 0 for c in s)
        @test issorted([c.x0 for c in s])
    end

    # a stage carries its own diameter, or inherits the vehicle's
    @test stage_diameter(Stage(:a, 1.0, 1.0, 1.0, 300.0, 0.0), 1.8) == 1.8
    @test stage_diameter(sized_stage(:b; engine = :aj10, prop_mass = 100.0,
                                     diameter = 3.0), 1.8) == 3.0
end

@testset "crew capsule" begin
    # Every directed edge appears once and its reverse once: the mesh is
    # closed AND consistently wound. Window apertures and raised collars are
    # easy to get subtly wrong (a hairline seam crack, or a panel whose
    # surfaces face into its own rims), and both leaks are invisible on screen.
    function manifold(m)
        d = Dict{Tuple{NTuple{3,Float64},NTuple{3,Float64}},Int}()
        for t in m.tris, e in ((t[1],t[2]), (t[2],t[3]), (t[3],t[1]))
            d[e] = get(d, e, 0) + 1
        end
        all(n -> n == 1, values(d)) &&
            all(e -> get(d, (e[2], e[1]), 0) == 1, keys(d))
    end

    for rp in (0.55, 0.75, 1.20)
        ext, glass, cab, hgt = pod_mesh(; radius = rp, nseg = 24)
        @test length(glass) == 3                     # three glazed apertures
        @test all(manifold, ext) && all(manifold, glass) && all(manifold, cab)
        @test all(m -> mesh_volume(m) > 0, ext)
        @test all(m -> mesh_volume(m) > 0, glass)
        @test all(m -> mesh_volume(m) > 0, cab)
        # Apollo-ish proportions: a touch under 0.9 diameters tall
        @test 0.85 < hgt / (2rp) < 0.92
        # nothing in the cabin punches through the tapering pressure wall
        ta = tan(deg2rad_(32.5))
        xb0 = 2.4rp - sqrt((2.4rp)^2 - rp^2) + 0.05rp
        rwall(x) = rp - ta * (x - xb0) - 0.050rp
        @test all(hypot(v[2], v[3]) <= rwall(v[1]) for m in cab for t in m.tris for v in t)
    end
    # the hull really is pierced: the pressure shell has no triangle sitting
    # inside the main window's aperture (centred on +y, the crew's viewport)
    rp = 0.75
    ext, glass, _, _ = pod_mesh(; radius = rp, nseg = 24)
    ta = tan(deg2rad_(32.5))
    xb0 = 2.4rp - sqrt((2.4rp)^2 - rp^2) + 0.05rp
    Lc = (rp - 0.26rp) / ta
    xlo, xhi = xb0 + 4.3Lc / 12, xb0 + 6.7Lc / 12
    cen(t) = ntuple(j -> (t[1][j] + t[2][j] + t[3][j]) / 3, 3)
    function inwin(t)
        c = cen(t)
        xlo < c[1] < xhi && c[2] > 0 && abs(atan(c[3], c[2])) < deg2rad_(10.0)
    end
    @test !any(inwin, ext[2].tris)                       # shell: hole is open
    @test any(inwin, glass[1].tris)                      # pane: glass fills it
    # crew count follows what the cabin can actually seat
    @test length(pod_mesh(; radius = 0.55)[3]) < length(pod_mesh(; radius = 1.20)[3])
    @test length(pod_mesh(; radius = 0.55, ncrew = 3)[3]) ==
          length(pod_mesh(; radius = 1.20, ncrew = 3)[3])
end

@testset "lifting entry (lunar return corridor)" begin
    # A ballistic capsule cannot fly a lunar return with people aboard: the
    # same trajectory that peaks near 6 g with L/D 0.3 peaks at 18 g without
    # it, and stays above 15 g for tens of seconds. This is the Zond-5 result,
    # and it is why the default pod is lifting.
    ms = moonshot()
    @test ms.entry.peak_gload < 8.0                 # crew-survivable
    @test 45e3 < ms.cislunar.vac_perigee_alt < 55e3

    mkpod(cl) = default_reentry_pod(cl_trim_hyp = cl)
    fly(cl; bank = 0.0) = simulate(Scenario(
        vehicle = mkpod(cl), r0 = ms.cislunar.r, v0 = ms.cislunar.v,
        t0 = ms.cislunar.t, t_max = ms.cislunar.t + 4.0e4,
        alpha0 = deg2rad_(5.0), bank = bank))

    ball, lift = fly(0.0), fly(0.45)
    @test ball.peak_gload > 2.0 * lift.peak_gload    # lift roughly thirds it
    @test ball.peak_gload > 12.0
    @test lift.peak_gload < 8.0
    @test lift.terminated == :splashdown
    # the trade is integrated heating: a lifting entry soaks longer at a lower
    # peak rate, and that is what sizes the ablator
    @test lift.heat_load > ball.heat_load
    @test lift.peak_qdot < ball.peak_qdot
    # lift-down is the wrong way to point it: steeper, harder
    @test fly(0.45; bank = Float64(pi)).peak_gload > lift.peak_gload

    # the corridor really is bounded: too shallow and it never comes home
    shallow = moonshot(hp_return = 80e3)
    @test simulate(Scenario(vehicle = mkpod(0.45), r0 = shallow.cislunar.r,
                            v0 = shallow.cislunar.v, t0 = shallow.cislunar.t,
                            t_max = shallow.cislunar.t + 4.0e4,
                            alpha0 = deg2rad_(5.0))).terminated == :timeout
end

@testset "Newtonian panel aero" begin
    @test isapprox(cp_max_newtonian(1e6), 1.8394; atol = 1e-3)  # M -> inf limit
    # sphere: CD = Cp_max/2 exactly in Newtonian theory
    prof = [(cos(s), sin(s)) for s in range(0.0, 1.0 * pi; length = 41)]
    sph = lathe_mesh([(x, r) for (x, r) in prof]; nseg = 64)
    pa = panel_aero(sph; sref = 1.0 * pi, lref = 2.0, ref = (0.0, 0.0, 0.0),
                    machs = [20.0])
    @test isapprox(cd_coeff(pa, 20.0, 0.0), cp_max_newtonian(20.0) / 2; rtol = 0.01)

    # capsule from committed geometry: stable, damped, trims at zero
    cap = read_stl(joinpath(@__DIR__, "..", "geometry", "capsule.stl"))
    mp = mass_properties(cap, 350.0)
    @test mp.offdiag_frac < 1e-3                    # axisymmetric
    pac = panel_aero(cap; sref = pi * 0.75^2, lref = 1.5, ref = mp.cg)
    @test cm_coeff(pac, 20.0, deg2rad_(10.0), 0.0) < 0     # restoring
    @test SatelliteSim.interp1(pac.cmq, 20.0) < 0          # damped
    @test abs(trim_alpha(pac)) < deg2rad_(1.0)
    @test 1.4 < cd_coeff(pac, 20.0, 0.0) < 2.0

    # geometry-to-trajectory: fly the pod on mesh-derived aero. The panel
    # method sees an axisymmetric capsule and so produces no trim lift, which
    # is correct — trim lift comes from an offset CG, not from the outer mould
    # line. The hand-tabulated reference is therefore taken ballistic too, or
    # the comparison would be measuring lift rather than the aero model.
    veh0 = default_reentry_pod(cl_trim_hyp = 0.0)
    veh = Vehicle(name = "mesh-pod", mass = 350.0, sref = pi * 0.75^2,
                  lref = 1.5, rn = 1.8, iyy = mp.inertia[2], aero = pac,
                  chutes = veh0.chutes)
    r6 = simulate_entry6(scenario_from_elements(DeorbitElements(), veh);
                         inertia = mp.inertia, rcs = default_pod_rcs())
    r0 = simulate(scenario_from_elements(DeorbitElements(), veh0))
    @test r6.terminated == :splashdown
    @test isapprox(r6.peak_gload, r0.peak_gload; rtol = 0.15)
    @test isapprox(r6.heat_load, r0.heat_load; rtol = 0.15)

    # starship demo mesh: lifting body with a passive trim from its flaps
    ship = read_stl(joinpath(@__DIR__, "..", "geometry", "starship.stl"))
    mps = mass_properties(ship, 120_000.0)
    pas = panel_aero(ship; sref = 9.0 * 50.0, lref = 50.0, ref = mps.cg)
    @test cd_coeff(pas, 20.0, deg2rad_(90.0)) > 5 * cd_coeff(pas, 20.0, 0.0)
    @test cl_coeff(pas, 20.0, deg2rad_(20.0)) /
          cd_coeff(pas, 20.0, deg2rad_(20.0)) > 1.0       # slender-body L/D
    at = trim_alpha(pas)
    @test deg2rad_(15.0) < at < deg2rad_(65.0)            # stable belly-first trim
    @test SatelliteSim.interp1(pas.cmq, 20.0) < 0
end

@testset "maneuvers" begin
    a = RE_MEAN + 500e3
    vc = sqrt(MU_EARTH / a)
    T = 2pi * sqrt(a^3 / MU_EARTH)
    # lambert: quarter arc of a circular orbit recovers circular velocity
    v1, v2 = lambert((a, 0.0, 0.0), (0.0, a, 0.0), T / 4)
    @test all(isapprox.(v1, (0.0, vc, 0.0); atol = 1e-3))
    @test all(isapprox.(v2, (-vc, 0.0, 0.0); atol = 1e-3))
    # lambert vs a known ellipse: recover the velocity at nu1
    ae, ee = 10_000e3, 0.3
    E_of(nu) = 2 * atan(sqrt((1 - ee) / (1 + ee)) * tan(nu / 2))
    t_of(nu) = (E_of(nu) - ee * sin(E_of(nu))) / sqrt(MU_EARTH / ae^3)
    nu1, nu2 = deg2rad_(30.0), deg2rad_(150.0)
    r1v, v1a = state_from_elements(ae, ee, 0.0, 0.0, 0.0, nu1)
    r2v, _ = state_from_elements(ae, ee, 0.0, 0.0, 0.0, nu2)
    vl1, _ = lambert(r1v, r2v, t_of(nu2) - t_of(nu1))
    @test all(isapprox.(vl1, v1a; rtol = 1e-5))
    # hohmann sanity: LEO -> GEO
    dv1, dv2, tofh = hohmann(a, 42_164e3)
    @test isapprox(dv1 + dv2, 3900.0; atol = 150.0)   # the classic ~3.9 km/s
    @test isapprox(tofh / 3600, 5.2; atol = 0.3)
    @test isapprox(plane_change_dv(vc, deg2rad_(60.0)), vc; rtol = 1e-12)
    # CW two-impulse: transfer nulls the position, braking nulls the velocity
    n = sqrt(MU_EARTH / a^3)
    r0 = (-2000.0, -10_000.0, 500.0)
    v0 = (0.5, 1.0, -0.2)
    dvc1, dvc2, vreq = cw_two_impulse(r0, v0, n, 2000.0)
    rT, vT = cw_propagate(r0, vreq, n, 2000.0)
    @test sqrt(sum(abs2, rT)) < 1e-6
    @test all(isapprox.(vT .+ dvc2, (0.0, 0.0, 0.0); atol = 1e-9))
    @test isapprox(stumpff(0.0)[1], 0.5; atol = 1e-12)
    @test isapprox(stumpff(1e-8)[1], stumpff(-1e-8)[1]; atol = 1e-8)  # slope 1/24
end

@testset "propagation & burn correctness" begin
    # circular orbit through the cislunar coast: radius drift over one rev
    a = RE_MEAN + 200e3
    r0, v0 = state_from_elements(a, 0.0, deg2rad_(28.5), 0.0, 0.0, 0.0)
    eph = coplanar_moon(r0, v0; phase0 = deg2rad_(90.0))
    T = 2pi * sqrt(a^3 / MU_EARTH)
    L = SatelliteSim.CislunarLog()
    leg = SatelliteSim._coast_leg!(L, r0, v0, 0.0, eph;
                                   theta_g0 = 0.0, h_stop = 0.0, t_end = T,
                                   stop_after_flyby = false, log_every = 10^9)
    @test abs(sqrt(sum(abs2, leg.r)) - a) < 100.0     # meters after one rev
    eps0 = sum(abs2, v0) / 2 - MU_EARTH / a
    eps1 = sum(abs2, leg.v) / 2 - MU_EARTH / sqrt(sum(abs2, leg.r))
    @test isapprox(eps1, eps0; rtol = 1e-6)
    # TLI burn: duration matches the rocket equation, arc stays modest
    ms = moonshot()
    kick = ms.lv.stages[end]
    mdot = stage_mdot(kick)
    m0 = ms.cislunar.m * exp(ms.cislunar.dv_tli / (G0 * kick.isp_vac))
    @test isapprox(ms.cislunar.burn_duration, (m0 - ms.cislunar.m) / mdot; rtol = 1e-3)
    Tpark = 2pi * sqrt(ms.ascent.elements.a^3 / MU_EARTH)
    @test ms.cislunar.burn_duration / Tpark * 360 < 20.0   # burn arc < 20 deg
    # burn log rows carry instantaneous velocity (monotone speed increase)
    Lc = ms.cislunar.log
    ib = findall(p -> p == 1, Lc.phase)
    sp = [hypot(Lc.vx[i], Lc.vy[i], Lc.vz[i]) for i in ib]
    @test issorted(sp) && sp[end] - sp[1] > 2000.0
end

@testset "full reentry smoke test" begin
    veh = default_reentry_pod()
    scn = scenario_from_elements(DeorbitElements(), veh)
    res = simulate(scn)
    @test res.terminated == :splashdown
    @test res.v_splash < 8.0                 # under main chute
    @test 3.0 < res.peak_gload < 12.0        # ballistic LEO entry range
    @test 20.0 < res.peak_qdot / 1e4 < 200.0 # W/cm², small-capsule class
    ev = [e.name for e in res.events]
    @test :entry_interface in ev
    @test :deploy_drogue in ev
    @test :deploy_main in ev
    # peak heating should occur between ~40 and 75 km
    L = res.log
    imax = argmax(L.qdot_conv .+ L.qdot_rad)
    @test 35e3 < L.h[imax] < 80e3
end

end
