# Reaction-control system: thruster hardware, on/off attitude control, and
# the analytic propellant budgets used for long coast phases.
#
# Numerical integration of a deadband limit cycle over a 20-day cruise would
# spend the whole run stepping millisecond pulses, so the cruise budget uses
# the standard closed-form results (single-axis rigid body, pulse pairs):
#
#   * limit cycle: each MIB pulse pair reverses the drift rate
#     Δω = n_thr·F·L·t_mib / I; the vehicle coasts across the 2·θ_db deadband
#     between pulses, so the duty cycle — and the propellant rate — follows
#     directly from geometry.
#   * rest-to-rest slew through θ: bang-bang with torque T = n_thr·F·L,
#     t_burn = 2·√(θ·I/T) total thruster on-time (half accelerating, half
#     braking).
#
# The 6-DOF entry simulation uses the same hardware through an on/off
# rate-damping law integrated directly (pulses are resolvable there because
# the phase lasts minutes, not weeks).

"One RCS thruster: body-frame position [m], thrust direction (unit), thrust [N]."
struct RCSThruster
    pos::V3
    dir::V3
    thrust::Float64
end

"Body torque produced by a thruster [N·m]."
@inline thruster_torque(t::RCSThruster) = vcross(t.pos, vscale(t.dir, t.thrust))

"""
    RCSystem(thrusters; isp, prop, mib)

`prop` is usable propellant [kg]; `mib` the minimum impulse bit expressed as
minimum on-time [s].
"""
Base.@kwdef struct RCSystem
    thrusters::Vector{RCSThruster}
    isp::Float64 = 220.0
    prop::Float64 = 5.0
    mib::Float64 = 0.02
end

"""
    torque_authority(sys) -> V3

Max torque about each body axis (sum of positive contributions per axis;
the negative direction is symmetric for the paired layouts built here).
"""
function torque_authority(sys::RCSystem)
    ax = ay = az = 0.0
    for t in sys.thrusters
        m = thruster_torque(t)
        ax += max(m[1], 0.0); ay += max(m[2], 0.0); az += max(m[3], 0.0)
    end
    (ax, ay, az)
end

"""
Total mass flow of `n` thrusters firing [kg/s].

Bills every thruster at `thrusters[1]`'s force. That is exact for the
symmetric layouts `three_axis_set` builds — which is everything this repo
flies — and wrong for a hand-assembled set of mixed thrusters. The analytic
budgets in this file make the same assumption. Both would need the firing set
passed in to do better, and nothing here has a reason to build such a set yet.
"""
@inline rcs_mdot(sys::RCSystem, n::Int = 1) =
    n * sys.thrusters[1].thrust / (G0 * sys.isp)

"""
    rate_damp_command(w, rate_db, wsat) -> V3 duty cycles in [-1, 1]

Per-axis pulse-width-modulated rate damping: zero inside the deadband,
proportional (-w/wsat) above it, saturating at full duty. `wsat` is the rate
at which the thrusters run continuously; below it the duty cycle — and the
propellant draw — scales down, which is how a real PWM controller avoids
chattering across the deadband at the control period.
"""
@inline function rate_damp_command(w::V3, rate_db::Float64, wsat::Float64)
    d(x) = abs(x) > rate_db ? -clamp(x / wsat, -1.0, 1.0) : 0.0
    (d(w[1]), d(w[2]), d(w[3]))
end

"""
    limit_cycle_prop(I, torque, isp, t_mib, theta_db, duration; nthr=2)

Propellant [kg] to hold a ±`theta_db` deadband for `duration` seconds about
one axis with inertia `I`, pair torque `torque`, and MIB on-time `t_mib`.

Standard drift-free limit cycle. One MIB pulse changes the rate by

    Δω = torque·t_mib / I

and the pulse fires at a deadband edge to *reverse* the drift — from -ω₀ to
+ω₀ — so the coast rate is **half** the impulse bit:

    ω₀ = Δω / 2

The vehicle then crosses the full 2·theta_db band at ω₀, giving one pulse per
traverse and a traverse time of 4·theta_db·I / (torque·t_mib).

That factor of two is worth stating because it was wrong here: this used ω₀ =
Δω, which halves the coast time and bills exactly twice the propellant. It
errs conservative, so nothing ever looked broken — the discrepancy only shows
against a directly-integrated limit cycle, which is what `test/runtests.jl`
now compares against rather than re-evaluating this same expression.
"""
function limit_cycle_prop(I::Float64, torque::Float64, isp::Float64,
                          t_mib::Float64, theta_db::Float64, duration::Float64;
                          nthr::Int = 2, thrust::Float64 = torque)
    dw = torque * t_mib / I                # rate step from one MIB pulse
    w0 = dw / 2                            # ...which reverses ±w0, so coast at half
    t_coast = 2 * theta_db / w0            # deadband traverse time
    cycles = duration / t_coast
    mdot = nthr * thrust / (G0 * isp)      # while firing (thrust per pulse pair)
    cycles * mdot * t_mib
end

"""
    slew_prop(I, torque, isp, angle; nthr=2)

Propellant [kg] for one rest-to-rest bang-bang slew through `angle` [rad].
Returns (prop, slew_time).
"""
function slew_prop(I::Float64, torque::Float64, isp::Float64, angle::Float64;
                   nthr::Int = 2, thrust::Float64 = torque)
    t_half = sqrt(angle * I / torque)
    mdot = nthr * thrust / (G0 * isp)
    (mdot * 2 * t_half, 2 * t_half)
end

"""
    couple_pair(axis, arm, F) -> (RCSThruster, RCSThruster)

Two thrusters forming a pure torque couple of `2·arm·F` about `axis`
(1 = roll x, 2 = pitch y, 3 = yaw z).
"""
function couple_pair(axis::Int, arm::Float64, F::Float64)
    if axis == 1       # roll: tangential firings at ±y
        (RCSThruster((0.0,  arm, 0.0), (0.0, 0.0,  1.0), F),
         RCSThruster((0.0, -arm, 0.0), (0.0, 0.0, -1.0), F))
    elseif axis == 2   # pitch: ±z firings at ±x
        (RCSThruster(( arm, 0.0, 0.0), (0.0, 0.0,  1.0), F),
         RCSThruster((-arm, 0.0, 0.0), (0.0, 0.0, -1.0), F))
    else               # yaw: ±y firings at ±x
        (RCSThruster(( arm, 0.0, 0.0), (0.0,  1.0, 0.0), F),
         RCSThruster((-arm, 0.0, 0.0), (0.0, -1.0, 0.0), F))
    end
end

"Mirror a couple pair to torque the opposite direction."
flip_pair(p) = (RCSThruster(p[1].pos, vscale(p[1].dir, -1.0), p[1].thrust),
                RCSThruster(p[2].pos, vscale(p[2].dir, -1.0), p[2].thrust))

"Full three-axis set: a +/- couple per axis (12 thrusters)."
function three_axis_set(arm::Float64, F::Float64)
    ths = RCSThruster[]
    for ax in 1:3
        p = couple_pair(ax, arm, F)
        push!(ths, p[1], p[2])
        m = flip_pair(p)
        push!(ths, m[1], m[2])
    end
    ths
end

"""
    default_pod_rcs() -> RCSystem

12 × 20 N thrusters (a ± couple per axis, 0.65 m arms) on the 1.5 m pod,
hydrazine-class Isp. Wind-hold through the coast and tipoff-rate damping.
"""
default_pod_rcs() = RCSystem(thrusters = three_axis_set(0.65, 20.0),
                             isp = 220.0, prop = 6.0, mib = 0.02)

"""
    default_kick_rcs() -> RCSystem

12 × 10 N thrusters (± couples, 1.4 m arms) on the kick stage for cruise
attitude control and pre-burn settling. Sized by the limit-cycle budget:
propellant scales with the impulse bit `T·t_mib`, so multi-day cruises want
small thrusters and short minimum pulses — the first cut of this stage
(25 N, 20 ms) blew through its tank in 19 days of deadband cycling.
"""
default_kick_rcs() = RCSystem(thrusters = three_axis_set(1.4, 10.0),
                              isp = 220.0, prop = 12.0, mib = 0.01)

"A 180° reorientation should take about this long [s], on any stack."
const SLEW_180_S = 120.0

"""
    sized_kick_rcs(I_t, radius, m_stack) -> RCSystem

An attitude-control system sized for the stack it is bolted to, rather than
the one fixed 12 × 10 N set that `default_kick_rcs` describes.

Two sizing rules, both from what the hardware has to achieve:

  * **Authority** — every stack should take about `SLEW_180_S` to turn around,
    so control torque scales with transverse inertia: `T = π·I / t_half²`.
    Thrusters sit on the skin, so the moment arm is the body radius and the
    force follows. A 5 N floor keeps the result buildable — below that you are
    describing a cold-gas jet, not a thruster.
  * **Capacity** — 1% of the coasting stack's mass, floored at 4 kg. Apollo
    carried about 450 kg of RCS propellant on a 45 t CSM+LM stack, which is
    where that 1% comes from.

Without this the same 12 kg tank was assumed for every vehicle in the
catalogue. That is survivable while inertia is also pinned, and stops being
survivable the moment it is not: a Saturn V's cruise stack has 2.0×10⁶ kg·m²
of transverse inertia, five thousand times the reference vehicle's, and it
budgets 17.7 kg of attitude propellant against a 12 kg tank — a vehicle that
flew to the Moon nine times, reported as unable to hold attitude.
"""
function sized_kick_rcs(I_t::Real, radius::Real, m_stack::Real)
    t_half = SLEW_180_S / 2
    T = pi * max(Float64(I_t), 1.0) / t_half^2      # couple torque needed [N·m]
    arm = max(Float64(radius), 0.3)
    F = max(T / (2 * arm), 5.0)
    RCSystem(thrusters = three_axis_set(arm, F), isp = 220.0,
             prop = max(0.01 * Float64(m_stack), 4.0), mib = 0.01)
end

# ---------------------------------------------------------- disturbances --
#
# The limit cycle above is DRIFT-FREE: it assumes no external torque, so the
# only thing propellant buys is reversing a rate the thrusters themselves
# imparted. That is a fair description of cislunar cruise. It is a poor one of
# low Earth orbit, where gravity gradient and residual aerodynamics pump
# momentum into the vehicle continuously and dumping that momentum — not
# deadband chatter — is what the tank is actually spent on.

"""
    gravity_gradient_torque(mu, r, dI) -> Float64

Peak gravity-gradient torque [N·m] on a body with transverse-to-roll inertia
difference `dI` at radius `r`:  T = 3·mu·dI / (2·r³).

This is the worst-case attitude (principal axis 45° off local vertical); it
swings to zero when an axis is aligned with the radius vector. A propellant
budget is sized on the worst case, so the peak is what is used.
"""
gravity_gradient_torque(mu::Float64, r::Float64, dI::Float64) =
    1.5 * mu * abs(dI) / r^3

"""
    aero_torque(rho, v, cd, area, arm) -> Float64

Aerodynamic disturbance torque [N·m] from the offset between centre of
pressure and centre of mass: T = ½·rho·v²·cd·area·arm.

Below about 500 km this is the term that dominates a low orbit's attitude
budget, and above about 800 km it is nothing at all — which is most of why a
single fixed "orbit" RCS number cannot be right for both.
"""
aero_torque(rho::Float64, v::Float64, cd::Float64, area::Float64, arm::Float64) =
    0.5 * rho * v^2 * cd * area * arm

"""
    momentum_dump_prop(sys, torque, t_dist, duration) -> Float64

Propellant [kg] to reject a steady disturbance torque `t_dist` for `duration`
seconds with control torque `torque`.

Momentum accumulates at `t_dist`; the thrusters remove it at `torque`, so the
duty cycle is simply `t_dist/torque` and the propellant is the mass flow times
the time actually spent firing. Independent of deadband and of minimum impulse
bit — once the vehicle is dumping secular momentum, the deadband only decides
how the firing is chopped up, not how much of it there is.
"""
momentum_dump_prop(sys::RCSystem, torque::Float64, t_dist::Float64,
                   duration::Float64) =
    rcs_mdot(sys, 2) * clamp(t_dist / max(torque, 1e-9), 0.0, 1.0) * duration

# ------------------------------------------------------------- the budget --

"""
    rcs_budget(sys, I_t; duration, ...) -> NamedTuple

RCS propellant budget for a coast, **with the discrete events placed in
time**.

Three consumers, and they do not have the same shape:

  * **hold** — deadband limit cycling about two transverse axes, or, where a
    disturbance torque dominates, dumping the momentum that torque pumps in.
    The two are combined with `max` rather than added: they are competing
    descriptions of the same pulses, not separate ones, and once secular
    momentum sets the firing rate the drift-free cycle no longer happens.
    Continuous in time.
  * **slews** — rest-to-rest reorientations. Each takes `2·√(θI/T)` seconds,
    which for a real stack is under a minute. **Discrete.**
  * **settling** — ullage burns before each main-engine ignition. **Discrete**,
    and at epochs the mission already knows.

`t_burns` are the ignition epochs [s]; each gets a settling burn and a slew to
point at it. Two more slews bracket the coast — the turnaround after the
departure burn and the attitude taken up for the next major event — so the
count follows the flight plan instead of being a hardcoded four.

Returns the component totals plus `t` / `used`: a cumulative curve carrying a
vertical step at every discrete event. That distinction is the point. Three
quarters of a cislunar budget is slews, each lasting well under a minute, and
drawing it as a smooth ramp across six days states the opposite of what the
model says — that the propellant goes on continuous housekeeping.
"""
function rcs_budget(sys::RCSystem, I_t::Float64;
                    duration::Float64,
                    t0::Float64 = 0.0,
                    t_burns::AbstractVector{<:Real} = Float64[],
                    slew_angle::Float64 = 1.0 * pi,
                    theta_db::Float64 = deg2rad_(5.0),
                    settling_s::Float64 = 10.0,
                    disturbance_torque::Float64 = 0.0,
                    capacity::Float64 = sys.prop)
    dur = max(duration, 0.0)
    T = max(torque_authority(sys)[3], 1e-9)
    F = sys.thrusters[1].thrust
    mdot = rcs_mdot(sys, 2)

    # --- continuous: hold attitude for the whole coast ---------------------
    lc = 2 * limit_cycle_prop(I_t, T, sys.isp, sys.mib, theta_db, dur;
                              nthr = 2, thrust = F)
    dump = momentum_dump_prop(sys, T, disturbance_torque, dur)
    hold = max(lc, dump)
    hold_rate = dur > 0 ? hold / dur : 0.0

    # --- discrete: a slew to each burn attitude, and one at each end -------
    burns = sort(Float64[t for t in t_burns if t0 <= t <= t0 + dur])
    p_slew, t_slew = slew_prop(I_t, T, sys.isp, slew_angle; nthr = 2, thrust = F)
    p_settle = mdot * settling_s
    steps = Tuple{Float64,Float64,String}[]           # (epoch, kg, what)
    push!(steps, (t0, p_slew, "post-burn turnaround"))
    for tb in burns
        push!(steps, (max(tb - t_slew - settling_s, t0), p_slew, "slew to burn attitude"))
        push!(steps, (max(tb - settling_s, t0), p_settle, "ullage settling"))
    end
    push!(steps, (t0 + dur, p_slew, "attitude for the next event"))
    sort!(steps; by = first)

    sl = p_slew * (length(burns) + 2)
    settle = p_settle * length(burns)
    total = hold + sl + settle

    # --- the curve: a uniform grid, plus a doubled sample at every step ----
    grid = collect(range(t0, t0 + dur; length = 121))
    ts = Float64[]
    for (te, _, _) in steps
        push!(ts, prevfloat(te), te)
    end
    times = sort!(unique!(vcat(grid, ts)))
    used = similar(times)
    for (i, t) in enumerate(times)
        u = hold_rate * (t - t0)
        for (te, kg, _) in steps
            te <= t && (u += kg)
        end
        used[i] = u
    end

    (limit_cycle = lc, dump = dump, hold = hold, slews = sl, settling = settle,
     total = total, margin = capacity - total, capacity = capacity,
     n_slews = length(burns) + 2, slew_time = t_slew,
     disturbance_torque = disturbance_torque,
     t = times, used = used,
     events = [(t = te, kg = kg, what = what) for (te, kg, what) in steps])
end

"""
    cruise_rcs_budget(sys, I_transverse; duration, ...) -> NamedTuple

`rcs_budget` with cislunar-coast defaults: no meaningful disturbance torque
out there, and the mid-course correction as the one ignition to settle for.
"""
cruise_rcs_budget(sys::RCSystem, I_t::Float64; duration::Float64,
                  t_tli::Float64 = 0.0,
                  t_events::AbstractVector{<:Real} = Float64[],
                  kwargs...) =
    rcs_budget(sys, I_t; duration = duration, t0 = t_tli, t_burns = t_events,
               kwargs...)

"""
    orbit_rcs_budget(sys, I_t, I_roll; duration, alt, v, area, ...) -> NamedTuple

`rcs_budget` for an Earth orbit, where the disturbance environment is the
whole story.

Gravity gradient and residual aerodynamics are both evaluated at `alt` and
combined; the aerodynamic arm defaults to 8% of the body length, a typical
centre-of-pressure to centre-of-mass offset for a stack that was never
designed to be aerodynamically trimmed.

The slew angle is smaller than the cislunar default: an orbiting vehicle
re-points between burns and to track a ground station, it does not turn
around.
"""
function orbit_rcs_budget(sys::RCSystem, I_t::Float64, I_roll::Float64;
                          duration::Float64,
                          alt::Float64,
                          v::Float64,
                          area::Float64,
                          body_length::Float64,
                          cp_offset::Float64 = 0.08,
                          atmosphere = nothing,
                          slew_angle::Float64 = 0.5 * pi,
                          kwargs...)
    rho = atmosphere === nothing ? 0.0 : first(atmosphere_state(atmosphere, alt))
    r = RE_MEAN + max(alt, 0.0)
    t_gg = gravity_gradient_torque(MU_EARTH, r, I_t - I_roll)
    t_aero = aero_torque(rho, v, 2.2, area, cp_offset * body_length)
    rcs_budget(sys, I_t; duration = duration, slew_angle = slew_angle,
               disturbance_torque = t_gg + t_aero, kwargs...)
end
