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

struct MoonshotResult
    lv::LaunchVehicle
    guid::AscentGuidance
    ascent::AscentResult
    eph::CircularMoonEphemeris
    cislunar::CislunarResult
    entry_scn::Scenario
    entry::SimResult
    cruise::Union{Nothing,CruiseReport}
end

"""
    moonshot(; pod_mass=350.0, h_park=200e3, hp_moon=2000e3, hp_return=35e3,
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
                  optimize_kick::Bool = false,
                  # numerical knobs, exposed so a convergence study needs no
                  # source edit: coast step as a fraction of the local orbital
                  # period, and the free return's perigee acceptance band [m]
                  cis_eta::Float64 = SatelliteSim.CIS_ETA,
                  perigee_tol::Float64 = SatelliteSim.PERIGEE_TOL,
                  verbose::Bool = false)
    # --- 1. launch to parking orbit ----------------------------------------
    # a supplied launch vehicle wins; its payload IS the pod
    lv === nothing && (lv = default_moon_rocket(payload = pod_mass))
    pod_mass = lv.payload_mass
    az = launch_azimuth(inclination, deg2rad_(28.5))
    guid0 = AscentGuidance(azimuth = az, h_target = h_park)
    guid, asc = tune_ascent(lv, guid0; optimize_kick = optimize_kick,
                            verbose = verbose)
    asc.reached_orbit ||
        error("ascent failed to reach orbit (h_cut=$(asc.h_cut/1e3) km, gamma=$(rad2deg_(asc.gamma_cut))°)")

    # jettison the insertion stage (with any residuals) before the TLI coast:
    # the kick stage + pod alone make the trans-lunar stack
    m_stack = asc.m
    for k in 1:length(lv.stages)-1
        if asc.prop_left[k] > 0
            m_stack -= lv.stages[k].mdry + asc.prop_left[k]
        end
    end

    # --- 2. lunar ephemeris in the achieved orbit plane --------------------
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

    # --- 3. free-return design ---------------------------------------------
    kick = lv.stages[end]
    t_ign, dv, cis = design_free_return(asc.r, asc.v, asc.t, eph;
                                        eta = cis_eta, perigee_tol = perigee_tol,
                                        stage = kick, m_stack = m_stack,
                                        prop_avail = asc.prop_left[end],
                                        hp_moon_target = hp_moon,
                                        hp_return_target = hp_return,
                                        verbose = verbose)
    cis.outcome == :entry_interface ||
        error("free-return design did not come home (outcome: $(cis.outcome))")

    # --- 3b. optional dispersed execution + mid-course correction ----------
    cruise = nothing
    if tli_mag_err != 0.0 || tli_point_err != 0.0
        # measure the nominal design's proxy perigee (flyby-exit osculating
        # value) so the TCM reproduces the same physical return, and grab the
        # nominal perilune-epoch position as the return-to-reference target
        nomfly = fly_cislunar(asc.r, asc.v, asc.t, eph;
                              eta = cis_eta,
                              t_ign = t_ign, dv = dv, stage = kick,
                              m_stack = m_stack, prop_avail = asc.prop_left[end],
                              stop_after_flyby = true, t_max = 10.0 * 86400.0)
        proxy = nomfly.vac_perigee_alt
        refleg = fly_cislunar(asc.r, asc.v, asc.t, eph;
                              eta = cis_eta,
                              t_ign = t_ign, dv = dv, stage = kick,
                              m_stack = m_stack, prop_avail = asc.prop_left[end],
                              t_max = nomfly.t_perilune - asc.t)
        cis_d, tcm_dv = fly_cislunar_tcm(asc.r, asc.v, asc.t, eph;
                                         eta = cis_eta,
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
        cis_d.outcome == :entry_interface ||
            error("dispersed cruise did not come home (outcome: $(cis_d.outcome))")
        tcm_prop = cis_d.m * (exp(tcm_dv / (G0 * kick.isp_vac)) - 1)
        rcs_budget = cruise_rcs_budget(default_kick_rcs(), 1000.0;
                                       duration = cis_d.t - cis_d.t_tli)
        cruise = CruiseReport(tli_mag_err, tli_point_err, tcm_dv,
                              cis_d.t_tli + tcm_delay, tcm_prop, rcs_budget)
        cis = cis_d
    end

    # --- 4. entry handoff: jettison the spent kick stage, fly the pod ------
    pod = default_reentry_pod(mass = pod_mass)
    scn = Scenario(vehicle = pod, r0 = cis.r, v0 = cis.v,
                   t0 = cis.t, t_max = cis.t + 3.0e4,
                   alpha0 = deg2rad_(5.0))
    entry = simulate(scn)

    MoonshotResult(lv, guid, asc, eph, cis, scn, entry, cruise)
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
