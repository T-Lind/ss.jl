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

struct MoonshotResult
    lv::LaunchVehicle
    guid::AscentGuidance
    ascent::AscentResult
    eph::CircularMoonEphemeris
    cislunar::CislunarResult
    entry_scn::Scenario
    entry::SimResult
end

"""
    moonshot(; pod_mass=350.0, h_park=200e3, hp_moon=2000e3, hp_return=35e3,
             inclination=deg2rad_(28.5), verbose=false) -> MoonshotResult

Design and fly the whole mission. `hp_moon` is the perilune altitude of the
flyby; `hp_return` the vacuum perigee of the return leg (sets the entry
flight-path angle: ~35 km gives gamma_EI near -5.9 deg, mid-corridor for a
ballistic lunar return).
"""
function moonshot(; pod_mass::Float64 = 350.0,
                  h_park::Float64 = 200.0e3,
                  hp_moon::Float64 = 2000.0e3,
                  hp_return::Float64 = 35.0e3,
                  inclination::Float64 = deg2rad_(28.5),
                  verbose::Bool = false)
    # --- 1. launch to parking orbit ----------------------------------------
    lv = default_moon_rocket(payload = pod_mass)
    az = launch_azimuth(inclination, deg2rad_(28.5))
    guid0 = AscentGuidance(azimuth = az, h_target = h_park)
    guid, asc = tune_ascent(lv, guid0; verbose = verbose)
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
                                        stage = kick, m_stack = m_stack,
                                        prop_avail = asc.prop_left[end],
                                        hp_moon_target = hp_moon,
                                        hp_return_target = hp_return,
                                        verbose = verbose)
    cis.outcome == :entry_interface ||
        error("free-return design did not come home (outcome: $(cis.outcome))")

    # --- 4. entry handoff: jettison the spent kick stage, fly the pod ------
    pod = default_reentry_pod(mass = pod_mass)
    scn = Scenario(vehicle = pod, r0 = cis.r, v0 = cis.v,
                   t0 = cis.t, t_max = cis.t + 3.0e4,
                   alpha0 = deg2rad_(5.0))
    entry = simulate(scn)

    MoonshotResult(lv, guid, asc, eph, cis, scn, entry)
end

function print_moonshot_summary(io::IO, ms::MoonshotResult)
    asc, cis, ent = ms.ascent, ms.cislunar, ms.entry
    el = asc.elements
    println(io, "== Moonshot summary ==")
    @printf(io, "  Liftoff mass    : %.1f t   (%s, %d stages + fairing)\n",
            liftoff_mass(ms.lv) / 1e3, ms.lv.name, length(ms.lv.stages))
    @printf(io, "  Parking orbit   : %.1f x %.1f km  i=%.2f°  (insertion t=%.1f s, m=%.0f kg)\n",
            (el.rp - RE_MEAN) / 1e3, (el.ra - RE_MEAN) / 1e3, rad2deg_(el.i), asc.t, asc.m)
    @printf(io, "  Kick-stage prop : %.1f kg at insertion\n", asc.prop_left[end])
    @printf(io, "  TLI             : t=%.2f h  dv=%.1f m/s  burn %.1f s  (m -> %.0f kg)\n",
            cis.t_tli / 3600, cis.dv_tli, cis.burn_duration, cis.m)
    @printf(io, "  Perilune        : %.0f km altitude at t=%.2f d\n",
            cis.perilune_alt / 1e3, cis.t_perilune / 86400)
    @printf(io, "  Return perigee  : %.1f km vacuum  gamma_EI-ish=%.2f°\n",
            cis.vac_perigee_alt / 1e3, rad2deg_(cis.gamma_end))
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
