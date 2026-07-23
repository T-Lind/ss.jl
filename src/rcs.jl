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

"Total mass flow of `n` thrusters firing [kg/s]."
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
Standard drift-free limit cycle: rate after a pulse ω = torque·t_mib/I, coast
across 2·theta_db, one pulse pair per traverse.
"""
function limit_cycle_prop(I::Float64, torque::Float64, isp::Float64,
                          t_mib::Float64, theta_db::Float64, duration::Float64;
                          nthr::Int = 2, thrust::Float64 = torque)
    w = torque * t_mib / I                 # rate imparted by one MIB pulse
    t_coast = 2 * theta_db / w             # deadband traverse time
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

"""
    cruise_rcs_budget(sys, I_transverse; duration, n_slews, slew_angle,
                      theta_db, settling_s) -> NamedTuple

Analytic RCS propellant budget for a coast phase: deadband limit cycling
about two transverse axes, `n_slews` reorientation slews, and propellant-
settling burns before main-engine ignitions. Returns the pieces and total.
"""
function cruise_rcs_budget(sys::RCSystem, I_t::Float64;
                           duration::Float64,
                           n_slews::Int = 4,
                           slew_angle::Float64 = 1.0 * pi,
                           theta_db::Float64 = deg2rad_(5.0),
                           settling_s::Float64 = 10.0)
    T = max(torque_authority(sys)[3], 1e-9)
    lc = 2 * limit_cycle_prop(I_t, T, sys.isp, sys.mib, theta_db, duration;
                              nthr = 2, thrust = sys.thrusters[1].thrust)
    sl = 0.0
    for _ in 1:n_slews
        p, _ = slew_prop(I_t, T, sys.isp, slew_angle;
                         nthr = 2, thrust = sys.thrusters[1].thrust)
        sl += p
    end
    settle = 2 * sys.thrusters[1].thrust / (G0 * sys.isp) * settling_s
    total = lc + sl + settle
    (limit_cycle = lc, slews = sl, settling = settle, total = total,
     margin = sys.prop - total)
end
