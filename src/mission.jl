# Full circumlunar mission: launch -> parking orbit -> trans-lunar injection
# -> free-return lunar flyby -> entry interface -> splashdown.
#
# The chain hands states between three simulators:
#   1. `simulate_ascent` / `tune_ascent` — pad to parking-orbit insertion,
#   2. `fly_cislunar` / `design_free_return` — TLI burn and the three-body
#      coast around the Moon back to the entry handoff (140 km),
#   3. the existing 4-DOF `simulate` — entry interface to splashdown, now at
#      lunar-return speed (~11 km/s instead of ~7.6 from LEO).
#
# Coplanar-TLI-window assumption: the Moon's circular orbit is constructed in
# the achieved parking-orbit plane (see `coplanar_moon`), with its phase set
# so the patched-conic departure geometry occurs about half a revolution
# after insertion. Real missions achieve the same alignment by choosing
# launch time and azimuth; modeling that wait costs nothing physically here.

"""
    CruiseReport

Products of flying the cruise with TLI execution errors: the applied
dispersion, the mid-course correction that fixed it, its kick-stage
propellant cost, and the analytic RCS attitude budget for the coast.
"""
struct CruiseReport
    tli_mag_err::Float64       # fractional delta-v execution error
    tli_point_err::Float64     # in-plane pointing error [rad]
    tcm_dv::Float64            # correction magnitude [m/s]
    tcm_time::Float64          # correction epoch [s]
    tcm_prop::Float64          # kick-stage propellant for the TCM [kg]
    rcs::NamedTuple            # cruise_rcs_budget output
end

# The legs past the ascent are Union{Nothing,...} because a mission that
# fails is still a mission that FLEW: with `strict = false` the chain returns
# whatever legs completed instead of throwing the partial trajectory away.
# `cislunar === nothing` means the ascent never reached orbit;
# `entry === nothing` with a cislunar leg means the coast ended somewhere
# other than entry interface (its `outcome` says where). Under the default
# `strict = true` every field is populated exactly as before.
struct MoonshotResult
    lv::LaunchVehicle
    guid::AscentGuidance
    ascent::AscentResult
    eph::Union{Nothing,CircularMoonEphemeris}
    cislunar::Union{Nothing,CislunarResult}
    entry_scn::Union{Nothing,Scenario}
    entry::Union{Nothing,SimResult}
    cruise::Union{Nothing,CruiseReport}
    # What the free-return corrector made of the problem: :converged,
    # :outside_tolerance, :unreachable or :stalled (see design_free_return).
    # A stalled design flies and lands like any other, so this is the only
    # thing that distinguishes the mission from a flyby that never happened.
    design_status::Symbol
end
MoonshotResult(lv, guid, asc, eph, cis, scn, entry, cruise) =
    MoonshotResult(lv, guid, asc, eph, cis, scn, entry, cruise, :converged)

"""
    translunar_design(lv; h_park, hp_moon, hp_return, inclination,
                      optimize_kick, cis_eta, perigee_tol, theta_g0, verbose)

Everything both lunar missions share: fly the ascent, build the coplanar
lunar ephemeris in the achieved plane, and design the free return. Returns
the ascent products, the ephemeris, the TLI solution `(t_ign, dv)`, the
verification flight, and the trans-lunar stack mass — from which a flyby
mission carries on to entry and a landing mission stops at perilune.

The landing mission arrives on a free return for the same reason Apollo did:
if the insertion burn does not happen, the trajectory comes home on its own.
"""
function translunar_design(lv::LaunchVehicle;
                           h_park::Float64 = 200.0e3,
                           hp_moon::Float64 = 2000.0e3,
                           hp_return::Float64 = 50.0e3,
                           inclination::Float64 = deg2rad_(28.5),
                           kick_angle::Float64 = deg2rad_(8.0),
                           optimize_kick::Bool = false,
                           cis_eta::Float64 = SatelliteSim.CIS_ETA,
                           perigee_tol::Float64 = SatelliteSim.PERIGEE_TOL,
                           tol_perigee_km::Float64 = 2.0,
                           # Earth rotation angle at liftoff, which is what
                           # decides the inertial PLANE the ascent inserts
                           # into — see `launch_window`. Both lunar missions
                           # share this leg, so both inherit the epoch.
                           theta_g0::Float64 = 0.0,
                           # strict = true throws on a failed leg (the
                           # behaviour every script and test was built on);
                           # strict = false returns the partial design with
                           # `cis = nothing`, so a caller can serve the
                           # ascent that DID fly instead of an error string
                           strict::Bool = true,
                           verbose::Bool = false)
    az = launch_azimuth(inclination, deg2rad_(28.5))
    guid0 = AscentGuidance(azimuth = az, h_target = h_park,
                           kick_angle = kick_angle)
    guid, asc = tune_ascent(lv, guid0; optimize_kick = optimize_kick,
                            theta_g0 = theta_g0, verbose = verbose)
    partial = (guid = guid, ascent = asc, eph = nothing, t_ign = NaN,
               dv = NaN, cis = nothing, m_stack = NaN, kick = lv.stages[end],
               design_status = :no_design)
    if !asc.reached_orbit
        strict && error("ascent failed to reach orbit (h_cut=$(asc.h_cut/1e3) km, gamma=$(rad2deg_(asc.gamma_cut))°)")
        return partial
    end
    # Reaching the target *energy* is not the same as reaching the target
    # *orbit*: a stack whose pitch program the shooter could not close arrives
    # fast and steep, and the elements come back with the perigee underground.
    # Saying so here beats designing a trans-lunar injection off it.
    el0 = asc.elements
    if !(el0.rp > RE_MEAN + 0.5 * h_park && abs(asc.gamma_cut) < deg2rad_(1.0))
        strict && error("ascent reached orbital energy but not the orbit " *
              "(perigee $(round((el0.rp - RE_MEAN)/1e3, digits=0)) km, " *
              "gamma $(round(rad2deg_(asc.gamma_cut), digits=2))°) — the pitch " *
              "program did not close at a $(round(rad2deg_(kick_angle), digits=1))° " *
              "pitch-over kick; try another kick angle or turn on the kick search")
        return partial
    end

    # jettison the insertion stage (with any residuals) before the TLI coast:
    # the kick stage + payload alone make the trans-lunar stack
    m_stack = asc.m
    for k in 1:length(lv.stages)-1
        if asc.prop_left[k] > 0
            m_stack -= lv.stages[k].mdry + asc.prop_left[k]
        end
    end

    el = asc.elements
    Tpark = 2pi * sqrt(el.a^3 / MU_EARTH)
    n_sc = 2pi / Tpark
    lead, tf, dv_seed = seed_free_return(asc.r, asc.v)
    # put the patched-conic alignment ~0.55 revs after insertion so the
    # design scan (one revolution wide) brackets it
    t_des = 0.55 * Tpark
    phase_at_insertion = lead + (n_sc - N_MOON) * t_des
    eph = coplanar_moon(asc.r, asc.v;
                        phase0 = phase_at_insertion - N_MOON * asc.t)

    kick = lv.stages[end]
    t_ign, dv, cis, dstatus = design_free_return(asc.r, asc.v, asc.t, eph;
                                        eta = cis_eta, perigee_tol = perigee_tol,
                                        theta_g0 = theta_g0,
                                        stage = kick, m_stack = m_stack,
                                        prop_avail = asc.prop_left[end],
                                        hp_moon_target = hp_moon,
                                        hp_return_target = hp_return,
                                        tol_perigee_km = tol_perigee_km,
                                        verbose = verbose)
    (guid = guid, ascent = asc, eph = eph, t_ign = t_ign, dv = dv,
     cis = cis, m_stack = m_stack, kick = kick, design_status = dstatus)
end

"""
    moonshot(; pod_mass=350.0, h_park=200e3, hp_moon=2000e3, hp_return=50e3,
             inclination=deg2rad_(28.5), verbose=false) -> MoonshotResult

Design and fly the whole mission. `hp_moon` is the perilune altitude of the
flyby; `hp_return` the vacuum perigee of the return leg, which sets the entry
flight-path angle and so the whole character of the entry.

The corridor is narrow. Below ~35 km the descent is steep enough that a
ballistic capsule pulls 18 g; above ~65 km it skips back out and never comes
home. The default 50 km gives gamma_EI ~ -6.25 deg, which with the pod's
L/D ~ 0.3 flown lift-up peaks near 6 g — the Apollo entry point.
"""
function moonshot(; pod_mass::Float64 = 350.0,
                  h_park::Float64 = 200.0e3,
                  hp_moon::Float64 = 2000.0e3,
                  hp_return::Float64 = 50.0e3,
                  inclination::Float64 = deg2rad_(28.5),
                  lv::Union{Nothing,LaunchVehicle} = nothing,
                  tli_mag_err::Float64 = 0.0,
                  tli_point_err::Float64 = 0.0,
                  tcm_delay::Float64 = 86400.0,
                  kick_angle::Float64 = deg2rad_(8.0),
                  optimize_kick::Bool = false,
                  # Earth rotation angle at liftoff. Every simulator in the
                  # chain already took this; `moonshot` simply never passed it,
                  # so every mission implicitly lifted off at Greenwich hour
                  # angle 0 and no two flights could be placed on a common
                  # clock. Set it (or get it from `launch_window`) and the
                  # flight lands in a definite inertial plane.
                  theta_g0::Float64 = 0.0,
                  # numerical knobs, exposed so a convergence study needs no
                  # source edit: coast step as a fraction of the local orbital
                  # period, and the free return's perigee acceptance band [m]
                  cis_eta::Float64 = SatelliteSim.CIS_ETA,
                  perigee_tol::Float64 = SatelliteSim.PERIGEE_TOL,
                  # strict = false: a failed leg ends the mission where the
                  # simulation ended and returns everything that DID fly,
                  # instead of discarding a fully-materialized trajectory
                  # for the sake of an error string
                  strict::Bool = true,
                  # How wide the capsule that re-enters actually is [m]. NaN
                  # takes the same mass fit the mesh draws and the panel
                  # quotes, so the flown capsule is the drawn capsule.
                  #
                  # This used to be a fixed 1.5 m for every payload, because
                  # `default_reentry_pod` was called with the mass alone. A
                  # 45 t capsule therefore re-entered behind a 1.77 m^2 heat
                  # shield: nothing that heavy can decelerate through that
                  # little area, so it skipped back out of the atmosphere and
                  # the entry integrator ran to t_max having never landed.
                  # That is the whole of the "heavy vehicles never come home"
                  # failure, and it was invisible because the number was right
                  # in the geometry and wrong only in the physics.
                  pod_diameter::Float64 = NaN,
                  verbose::Bool = false)
    # --- 1-3. launch, ephemeris, free-return design ------------------------
    # a supplied launch vehicle wins; its payload IS the pod
    lv === nothing && (lv = default_moon_rocket(payload = pod_mass))
    pod_mass = lv.payload_mass
    des = translunar_design(lv; h_park = h_park, hp_moon = hp_moon,
                            hp_return = hp_return, inclination = inclination,
                            kick_angle = kick_angle, optimize_kick = optimize_kick,
                            cis_eta = cis_eta, perigee_tol = perigee_tol,
                            theta_g0 = theta_g0, strict = strict,
                            verbose = verbose)
    guid, asc, eph = des.guid, des.ascent, des.eph
    dstatus = des.design_status
    des.cis === nothing &&
        return MoonshotResult(lv, guid, asc, nothing, nothing, nothing,
                              nothing, nothing, dstatus)
    t_ign, dv, cis = des.t_ign, des.dv, des.cis
    m_stack, kick = des.m_stack, des.kick
    if cis.outcome != :entry_interface
        strict && error("free-return design did not come home (outcome: $(cis.outcome))")
        return MoonshotResult(lv, guid, asc, eph, cis, nothing, nothing, nothing, dstatus)
    end
    # A design the corrector never closed is not the mission that was asked
    # for. It propagates, reaches entry interface and lands — which is exactly
    # why it has to be refused here rather than flown and reported as a flyby.
    if dstatus === :stalled || dstatus === :unreachable
        strict && error("free-return targeting did not converge (status: $dstatus) — " *
                        "the trajectory returned misses the requested perilune")
        return MoonshotResult(lv, guid, asc, eph, cis, nothing, nothing, nothing, dstatus)
    end

    # --- 3b. optional dispersed execution + mid-course correction ----------
    cruise = nothing
    if tli_mag_err != 0.0 || tli_point_err != 0.0
        # measure the nominal design's proxy perigee (flyby-exit osculating
        # value) so the TCM reproduces the same physical return, and grab the
        # nominal perilune-epoch position as the return-to-reference target
        nomfly = fly_cislunar(asc.r, asc.v, asc.t, eph;
                              eta = cis_eta, theta_g0 = theta_g0,
                              t_ign = t_ign, dv = dv, stage = kick,
                              m_stack = m_stack, prop_avail = asc.prop_left[end],
                              stop_after_flyby = true, t_max = 10.0 * 86400.0)
        proxy = nomfly.vac_perigee_alt
        refleg = fly_cislunar(asc.r, asc.v, asc.t, eph;
                              eta = cis_eta, theta_g0 = theta_g0,
                              t_ign = t_ign, dv = dv, stage = kick,
                              m_stack = m_stack, prop_avail = asc.prop_left[end],
                              t_max = nomfly.t_perilune - asc.t)
        cis_d, tcm_dv = fly_cislunar_tcm(asc.r, asc.v, asc.t, eph;
                                         eta = cis_eta, theta_g0 = theta_g0,
                                         t_ign = t_ign, dv = dv, stage = kick,
                                         m_stack = m_stack,
                                         prop_avail = asc.prop_left[end],
                                         r_ref = refleg.r, t_ref = refleg.t,
                                         dv_scale = 1.0 + tli_mag_err,
                                         point_err = tli_point_err,
                                         tcm_delay = tcm_delay,
                                         hp_moon_target = hp_moon,
                                         hp_perigee_proxy = proxy,
                                         verbose = verbose)
        if cis_d.outcome != :entry_interface
            strict && error("dispersed cruise did not come home (outcome: $(cis_d.outcome))")
            return MoonshotResult(lv, guid, asc, eph, cis_d, nothing, nothing, nothing, dstatus)
        end
        tcm_prop = cis_d.m * (exp(tcm_dv / (G0 * kick.isp_vac)) - 1)
        rcs_budget = cruise_rcs_budget(default_kick_rcs(), 1000.0;
                                       duration = cis_d.t - cis_d.t_tli)
        cruise = CruiseReport(tli_mag_err, tli_point_err, tcm_dv,
                              cis_d.t_tli + tcm_delay, tcm_prop, rcs_budget)
        cis = cis_d
    end

    # --- 4. entry handoff: jettison the spent kick stage, fly the pod ------
    # resolved here rather than in the signature because `lv` may have replaced
    # pod_mass above, and the fit has to see the mass actually being flown
    pod_d = isfinite(pod_diameter) && pod_diameter > 0 ?
            pod_diameter : 2 * pod_radius(pod_mass)
    pod = default_reentry_pod(mass = pod_mass, diameter = pod_d)
    scn = Scenario(vehicle = pod, r0 = cis.r, v0 = cis.v,
                   t0 = cis.t, t_max = cis.t + 3.0e4, theta_g0 = theta_g0,
                   alpha0 = deg2rad_(5.0))
    entry = simulate(scn)

    MoonshotResult(lv, guid, asc, eph, cis, scn, entry, cruise, dstatus)
end

function print_moonshot_summary(io::IO, ms::MoonshotResult)
    asc, cis, ent = ms.ascent, ms.cislunar, ms.entry
    el = asc.elements
    println(io, "== Moonshot summary ==")
    straps = sum(b.count for b in ms.lv.boosters; init = 0)
    @printf(io, "  Liftoff mass    : %.1f t   (%s, %d stages%s + fairing)\n",
            liftoff_mass(ms.lv) / 1e3, ms.lv.name, length(ms.lv.stages),
            straps > 0 ? " + $straps strap-ons" : "")
    @printf(io, "  Parking orbit   : %.1f x %.1f km  i=%.2f°  (insertion t=%.1f s, m=%.0f kg)\n",
            (el.rp - RE_MEAN) / 1e3, (el.ra - RE_MEAN) / 1e3, rad2deg_(el.i), asc.t, asc.m)
    @printf(io, "  Kick-stage prop : %.1f kg at insertion\n", asc.prop_left[end])
    @printf(io, "  TLI             : t=%.2f h  dv=%.1f m/s  burn %.1f s  (m -> %.0f kg)\n",
            cis.t_tli / 3600, cis.dv_tli, cis.burn_duration, cis.m)
    @printf(io, "  Perilune        : %.0f km altitude at t=%.2f d\n",
            cis.perilune_alt / 1e3, cis.t_perilune / 86400)
    @printf(io, "  Return perigee  : %.1f km vacuum  gamma_EI-ish=%.2f°\n",
            cis.vac_perigee_alt / 1e3, rad2deg_(cis.gamma_end))
    if ms.cruise !== nothing
        c = ms.cruise
        @printf(io, "  TLI dispersion  : %+.2f%% magnitude, %+.2f° pointing\n",
                100 * c.tli_mag_err, rad2deg_(c.tli_point_err))
        @printf(io, "  TCM at T+%.1f h : dv=%.1f m/s  (%.1f kg kick propellant)\n",
                c.tcm_time / 3600, c.tcm_dv, c.tcm_prop)
        @printf(io, "  Cruise RCS      : %.2f kg (%.2f limit-cycle + %.2f slews + %.2f settling), margin %.2f kg\n",
                c.rcs.total, c.rcs.limit_cycle, c.rcs.slews, c.rcs.settling, c.rcs.margin)
    end
    ei = findfirst(e -> e.name == :entry_interface, ent.events)
    if ei !== nothing
        e = ent.events[ei]
        @printf(io, "  Entry interface : V_rel=%.0f m/s  Mach %.1f  (t=%.2f d)\n",
                e.vrel, e.mach, e.t / 86400)
    end
    if ent.terminated == :splashdown
        @printf(io, "  Splashdown      : lat=%.2f°  lon=%.2f°  V=%.1f m/s  (t=%.2f d)\n",
                rad2deg_(ent.lat_splash), rad2deg_(ent.lon_splash), ent.v_splash,
                ent.t_splash / 86400)
    else
        println(io, "  DID NOT SPLASH DOWN (", ent.terminated, ")")
    end
    @printf(io, "  Entry peak load : %.1f g   peak q_dot %.0f W/cm²   heat load %.0f MJ/m²\n",
            ent.peak_gload, ent.peak_qdot / 1e4, ent.heat_load / 1e6)
end
print_moonshot_summary(ms::MoonshotResult) = print_moonshot_summary(stdout, ms)
