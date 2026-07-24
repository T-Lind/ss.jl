# Descent navigation: the state the guidance actually flies on, and the radar
# that keeps it honest.
#
# Until now the guidance read the integrator's state vector, which is to say
# it knew exactly where it was. That is the single most flattering assumption
# a landing simulation can make, and it hides the failure that terrain
# introduces: a lander does not fly to the ground, it flies to *where it
# believes the ground is*. Get that wrong by a kilometre — which the mean
# sphere does routinely, because lunar relief is kilometres — and the vehicle
# runs a beautifully executed profile into a hillside at the speed the profile
# called for a kilometre higher up.
#
# So there are two states here. The truth state is integrated over the real
# terrain and the real gravity field. The navigation state is what the vehicle
# thinks, and it starts wrong (orbit determination from tracking is good to
# hundreds of metres, not metres), propagates wrong (the onboard model is a
# point-mass Moon; the real one has mascons), and is corrected by the only
# instrument that looks at the ground: the landing radar.
#
# The radar is the reason Apollo landed rather than crashed, and it is modelled
# as the thing it is — an *altimeter that measures the ground ahead*, not a
# position fix. It has an acquisition altitude, a beam tilted off nadir so its
# footprint leads the vehicle, noise proportional to range, and a gain the
# filter weights it in with. Everything it cannot see, it does not fix.

"""
    LandingRadar(; h_lock, h_lock_vel, dt_update, sigma_h, scale_h, sigma_v,
                   beam_tilt, gain_h, gain_v)

A downward-looking landing radar: one range beam and a velocity triad.

  * `h_lock`, `h_lock_vel` [m] — altitudes at which the range beam and the
    velocity beams acquire. Apollo's altimeter came in around 12 km and the
    Doppler beams a good deal lower, which is why the altitude channel is the
    one that saves the descent and the velocity channel is the one that
    polishes it.
  * `beam_tilt` [rad] — how far off nadir the range beam points. Not a
    detail: a tilted beam measures the ground it is *about to fly over*, so
    over sloping terrain the altimeter reads the hill ahead rather than the
    valley underneath, and the guidance flies to the former.
  * `sigma_h` [m] and `scale_h` — fixed and range-proportional noise. Real
    radar altimeters are percentage instruments, so the measurement is worst
    exactly where the vehicle first needs it.
  * `gain_h`, `gain_v` — how much of each residual the filter takes per
    update. This is a first-order complementary filter, not a Kalman filter:
    the states it estimates are so nearly observable that the difference is
    two lines of code and no change in outcome.
"""
Base.@kwdef struct LandingRadar
    h_lock::Float64 = 12.0e3
    h_lock_vel::Float64 = 7.0e3
    dt_update::Float64 = 0.25
    sigma_h::Float64 = 2.5
    scale_h::Float64 = 0.012
    sigma_v::Float64 = 0.30
    beam_tilt::Float64 = deg2rad_(22.0)
    gain_h::Float64 = 0.22
    gain_v::Float64 = 0.16
end

"""
    DescentNav(; dr_*, dv_*, site_elev, radar, seed)

The navigation configuration of a powered descent.

`dr_down`, `dr_radial`, `dr_cross` [m] and the matching `dv_*` [m/s] are the
one-sigma orbit-determination errors at powered-descent ignition, in the
local downrange / radial / crossrange frame. The defaults are Apollo-era
figures: tracking from Earth pins a lunar orbit to a few hundred metres
downrange and rather better radially, and the downrange component is the one
that matters because it moves the landing site.

`site_elev` [m] is what the vehicle has been *told* the ground elevation at
the site is, above the mean sphere. `NaN` — the default — means it has been
told nothing and assumes the sphere, which is the case terrain punishes.
`moonlanding` fills this in from an orbital survey with a stated error when
asked to.

`seed` makes the errors and the radar noise reproducible: the same
configuration flies the same descent every time, which is what makes a
dispersion study mean anything.
"""
Base.@kwdef struct DescentNav
    dr_down::Float64 = 900.0
    dr_radial::Float64 = 250.0
    dr_cross::Float64 = 300.0
    dv_down::Float64 = 0.6
    dv_radial::Float64 = 0.3
    dv_cross::Float64 = 0.4
    site_elev::Float64 = NaN
    radar::Union{Nothing,LandingRadar} = LandingRadar()
    seed::UInt32 = 0x5EED1A11
end

"Perfect navigation: no initial error, no radar needed, the state is known."
perfect_nav() = DescentNav(dr_down = 0.0, dr_radial = 0.0, dr_cross = 0.0,
                           dv_down = 0.0, dv_radial = 0.0, dv_cross = 0.0,
                           radar = nothing)

# ------------------------------------------------------- deterministic RNG --

"Uniform (0, 1) draw `k` from a seed — reproducible, and independent of Random."
@inline _nrand(seed::UInt32, k::Int) =
    (Float64(_hash32(seed, unsafe_trunc(UInt32, k), 0x9E3779B9, 0x85EBCA6B)) + 0.5) /
    4.294967296e9

"Standard normal draw `k` from a seed, by Box-Muller."
@inline function _ngauss(seed::UInt32, k::Int)
    u1 = _nrand(seed, 2k)
    u2 = _nrand(seed, 2k + 1)
    sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2)
end

# ------------------------------------------------------------- nav state ---

"""
The vehicle's own estimate of where it is, propagated with the onboard model
and corrected by the radar. `bias_h` is the altitude reference the estimate
is measured against: the mean sphere unless the site was surveyed.
"""
mutable struct NavState
    r::V3
    v::V3
    r_ref::Float64          # assumed ground radius [m]
    locked_h::Bool
    locked_v::Bool
    t_next::Float64         # next radar update [s from PDI]
    n_update::Int
    seed::UInt32
end

"""
    init_nav(cfg, r, v, hhat) -> NavState

Seed the navigation state from truth plus the orbit-determination error, drawn
in the downrange/radial/crossrange frame at ignition.
"""
function init_nav(cfg::DescentNav, r::V3, v::V3, hhat::V3)
    ur = vunit(r)
    ut = vcross(hhat, ur)               # downrange
    dr = vadd(vadd(vscale(ut, cfg.dr_down * _ngauss(cfg.seed, 1)),
                   vscale(ur, cfg.dr_radial * _ngauss(cfg.seed, 2))),
              vscale(hhat, cfg.dr_cross * _ngauss(cfg.seed, 3)))
    dv = vadd(vadd(vscale(ut, cfg.dv_down * _ngauss(cfg.seed, 4)),
                   vscale(ur, cfg.dv_radial * _ngauss(cfg.seed, 5))),
              vscale(hhat, cfg.dv_cross * _ngauss(cfg.seed, 6)))
    r_ref = isnan(cfg.site_elev) ? R_MOON : R_MOON + cfg.site_elev
    NavState(vadd(r, dr), vadd(v, dv), r_ref, false, false, 0.0, 0,
             cfg.seed ⊻ 0x2C1B3C6D)
end

"Estimated altitude above the assumed ground [m]."
@inline nav_altitude(n::NavState) = vnorm(n.r) - n.r_ref

"""
    nav_propagate!(nav, a_thrust, dt)

Advance the estimate one control cycle. The onboard model is a point-mass
Moon; the thrust term comes from the inertial platform, which measures the
non-gravitational acceleration directly and to a precision that does not
matter here. So the estimate drifts for exactly one reason — the gravity the
model leaves out — which is the honest reason a real one drifts.
"""
function nav_propagate!(n::NavState, a_thrust::V3, dt::Float64)
    a = vadd(lunar_gravity(n.r, nothing, 0.0, 0.0), a_thrust)
    n.r = vadd(n.r, vadd(vscale(n.v, dt), vscale(a, 0.5 * dt * dt)))
    n.v = vadd(n.v, vscale(a, dt))
    nothing
end

"""
    radar_update!(nav, radar, r_true, v_true, t, surf, t_abs, hhat) -> Bool

One radar cycle, if one is due and the beams have acquired. Returns whether
anything was measured.

The range beam is tilted forward, so it illuminates the ground the vehicle is
about to reach rather than the ground under it — the measurement is taken
against the terrain at the beam's footprint, and over sloping ground the
difference between the two is a real bias that the filter faithfully believes.
The velocity beams see the surface directly and simply give the Moon-frame
velocity with noise.
"""
function radar_update!(n::NavState, radar::LandingRadar, r::V3, v::V3,
                       t::Float64, surf, t_abs::Float64, hhat::V3)
    t < n.t_next && return false
    n.t_next = t + radar.dt_update
    ur = vunit(r)
    ut = vcross(hhat, ur)
    h_true = surface_altitude(surf, r, t_abs)
    h_true <= 0.0 && return false
    updated = false

    if h_true <= radar.h_lock
        n.locked_h = true
        # beam footprint: tilted off nadir, so it leads the vehicle
        lead = h_true * tan(radar.beam_tilt)
        rb = vadd(r, vscale(ut, lead))
        h_beam = vnorm(r) - surface_radius(surf, rb, t_abs)
        k = n.n_update
        noise = (radar.sigma_h + radar.scale_h * h_true) * _ngauss(n.seed, 1000 + k)
        h_meas = h_beam + noise
        n.r = vadd(n.r, vscale(vunit(n.r), radar.gain_h * (h_meas - nav_altitude(n))))
        updated = true
    end
    if h_true <= radar.h_lock_vel
        n.locked_v = true
        k = n.n_update
        dv = (radar.sigma_v * _ngauss(n.seed, 2000 + 3k),
              radar.sigma_v * _ngauss(n.seed, 2000 + 3k + 1),
              radar.sigma_v * _ngauss(n.seed, 2000 + 3k + 2))
        v_meas = vadd(v, dv)
        n.v = vadd(n.v, vscale(vsub(v_meas, n.v), radar.gain_v))
        updated = true
    end
    n.n_update += 1
    updated
end

"""
    nav_error(nav, r_true, v_true) -> (position [m], velocity [m/s], altitude [m])

How wrong the vehicle currently is. The altitude component is reported
separately because it is the one that decides the landing: a lander that is a
kilometre off downrange lands a kilometre downrange, while a lander that is a
hundred metres off in altitude lands hard.
"""
nav_error(n::NavState, r::V3, v::V3, surf, t_abs::Float64) =
    (vnorm(vsub(n.r, r)), vnorm(vsub(n.v, v)),
     nav_altitude(n) - surface_altitude(surf, r, t_abs))

# ------------------------------------------------- hazard avoidance ---------

"""
    HazardScan(; reach, cross_reach, step, radius, arrival)

Site redesignation at high gate: how far the guidance is allowed to look for
somewhere better to land, and how finely.

The footprint is deliberately lopsided — `reach` metres downrange against
`cross_reach` to either side — because that is the shape of the set a lander
at high gate can actually reach. It is still flying forward at a hundred-odd
metres per second, so moving the aim point downrange is nearly free while
moving it sideways has to be paid for out of the same thrust that is holding
the vehicle up.

`radius` is the disc each candidate is scored over, which should be a few
times the vehicle's stance: what matters is not the height of one point but
whether all the footpads can reach the ground at once.

`arrival` and `tau` [s] are the outer and inner time constants of the
position loop the terminal guidance flies the aim point out on: commanded
speed is the remaining distance over `arrival`, and commanded acceleration is
the speed error over `tau`. Together they set a damping ratio of
`sqrt(arrival/tau)/2` — the defaults give 0.87, just short of critical, and
settle in about 80 seconds. That number has to be smaller than the hundred-odd
seconds the vertical channel takes to fly 2 km down, or the vehicle reaches
the ground while it is still travelling sideways to its own landing site,
which is a novel way to crash and one this had to be tuned out of.
"""
Base.@kwdef struct HazardScan
    reach::Float64 = 900.0
    cross_reach::Float64 = 300.0
    step::Float64 = 100.0
    radius::Float64 = 15.0
    arrival::Float64 = 30.0
    tau::Float64 = 10.0
end

"""
    redesignate(scan, surf, r, v, t_abs, hhat, lead) -> (u_target, score, score0, offset)

Choose a landing point at high gate. The nominal aim point is where the
vehicle would arrive if it simply flew its horizontal velocity out — `lead`
metres downrange — and the scan looks for the least hazardous ground within
reach of that. Returns the chosen site as a Moon-fixed unit direction, its
hazard score, the score of the nominal point it replaced, and how far it had
to move.

This is Apollo's landing-point designator with the crew taken out of the loop:
the same decision, made on the same information, at the same moment in the
descent — the difference being that a computer can score four hundred sites in
the time it takes a commander to look at one.
"""
function redesignate(scan::HazardScan, surf::SurfaceModel, r::V3, v::V3,
                     t_abs::Float64, hhat::V3, lead::Float64)
    eph = surf.eph
    # everything in the Moon-fixed frame, where the terrain lives
    rf = moonfixed(r, t_abs, eph)
    vf = moonfixed(v, t_abs, eph)
    hf = moonfixed(hhat, t_abs, eph)
    u0 = vunit(rf)
    e_down = vunit(vcross(hf, u0))
    e_cross = vcross(u0, e_down)
    u_nom = surface_offset(u0, e_down, e_cross, lead, 0.0)
    score0 = site_hazard(surf.terrain, u_nom; radius = scan.radius)[1]
    u, d, c, sc = safe_site(surf.terrain, u_nom, e_down, e_cross;
                            reach = scan.reach, cross_reach = scan.cross_reach,
                            step = scan.step, radius = scan.radius)
    (u, sc, score0, hypot(d, c))
end
