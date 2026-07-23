# Aerodynamic database: Mach-dependent coefficient tables with clamped
# linear interpolation, plus small-angle-of-attack increments about trim.
#
# Model (mid-fidelity, blunt entry capsule flying heatshield-forward):
#   CD(M, alpha) = CD0(M) * (1 + kd * (alpha - alpha_trim)^2)
#   CL(M, alpha) = CLa(M) * (alpha - alpha_trim) + CL_trim(M)
#   Cm(M, alpha, q) = Cma(M) * (alpha - alpha_trim) + Cmq(M) * q*Lref/(2 Vrel)
#
# alpha is the body pitch-plane angle of attack (the 4th degree of freedom).
# A symmetric ballistic pod has alpha_trim = 0 and CL_trim = 0; an offset-CG
# capsule (Soyuz/Apollo style) is modeled by nonzero alpha_trim + CL_trim,
# giving a constant trim L/D that the bank angle then orients.
#
# Extension point: replace `CapsuleAero` with any type implementing
# cd_coeff / cl_coeff / cm_coeff (e.g. full CN/CA/Cm tables in alpha and Mach).

struct Table1D
    x::Vector{Float64}
    y::Vector{Float64}
    function Table1D(x, y)
        length(x) == length(y) || throw(ArgumentError("table length mismatch"))
        issorted(x) || throw(ArgumentError("table abscissa must be sorted"))
        new(collect(Float64, x), collect(Float64, y))
    end
end

"Clamped linear interpolation."
function interp1(t::Table1D, x::Float64)
    xs, ys = t.x, t.y
    n = length(xs)
    x <= xs[1] && return ys[1]
    x >= xs[n] && return ys[n]
    i = searchsortedlast(xs, x)
    f = (x - xs[i]) / (xs[i+1] - xs[i])
    ys[i] + f * (ys[i+1] - ys[i])
end

abstract type AbstractAeroDatabase end

Base.@kwdef struct CapsuleAero <: AbstractAeroDatabase
    cd0::Table1D          # zero-offset drag coefficient vs Mach
    cla::Table1D          # lift-curve slope [1/rad] vs Mach
    cma::Table1D          # pitch stiffness [1/rad] vs Mach (negative = stable)
    cmq::Table1D          # pitch damping [1/rad] vs Mach (negative = damped)
    cl_trim::Table1D      # trim lift coefficient vs Mach
    alpha_trim::Float64 = 0.0   # trim angle of attack [rad]
    kd_alpha2::Float64 = 0.4    # quadratic drag rise with off-trim alpha [1/rad^2]
end

@inline function cd_coeff(a::CapsuleAero, M::Float64, alpha::Float64)
    da = alpha - a.alpha_trim
    interp1(a.cd0, M) * (1 + a.kd_alpha2 * da * da)
end

@inline function cl_coeff(a::CapsuleAero, M::Float64, alpha::Float64)
    interp1(a.cla, M) * (alpha - a.alpha_trim) + interp1(a.cl_trim, M)
end

"Pitching-moment coefficient. `qhat = q * Lref / (2 Vrel)` is the nondimensional pitch rate."
@inline function cm_coeff(a::CapsuleAero, M::Float64, alpha::Float64, qhat::Float64)
    interp1(a.cma, M) * (alpha - a.alpha_trim) + interp1(a.cmq, M) * qhat
end

"""
    default_capsule_aero(; alpha_trim = 0.0, cl_trim_hyp = 0.0)

Generic blunt-capsule aerodynamic tables (Apollo/Soyuz-class shape, scaled),
representative of published capsule data at trim: subsonic CD ~0.8, transonic
rise, hypersonic CD ~1.5; statically stable (Cm_alpha < 0) at all Mach with
weakest damping transonically.
"""
function default_capsule_aero(; alpha_trim::Float64 = 0.0, cl_trim_hyp::Float64 = 0.0)
    mach = [0.3, 0.7, 0.9, 1.1, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0]
    cd0  = [0.78, 0.85, 1.00, 1.25, 1.35, 1.40, 1.42, 1.45, 1.48, 1.50, 1.52, 1.52]
    cla  = [0.25, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.50, 0.50, 0.50, 0.50, 0.50]
    cma  = [-0.06, -0.05, -0.04, -0.07, -0.10, -0.12, -0.12, -0.11, -0.10, -0.10, -0.10, -0.10]
    cmq  = [-0.20, -0.12, -0.08, -0.15, -0.25, -0.30, -0.30, -0.30, -0.30, -0.30, -0.30, -0.30]
    # trim lift ramps in supersonically (capsule flies near-ballistic subsonic)
    cltr = cl_trim_hyp .* [0.0, 0.0, 0.0, 0.3, 0.6, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    CapsuleAero(
        cd0 = Table1D(mach, cd0),
        cla = Table1D(mach, cla),
        cma = Table1D(mach, cma),
        cmq = Table1D(mach, cmq),
        cl_trim = Table1D(mach, cltr),
        alpha_trim = alpha_trim,
    )
end

"Return a copy of the database with CD, Cm_alpha scaled (Monte Carlo knob)."
function scaled_aero(a::CapsuleAero; cd_mult = 1.0, cma_mult = 1.0, alpha_trim_delta = 0.0)
    CapsuleAero(
        cd0 = Table1D(a.cd0.x, a.cd0.y .* cd_mult),
        cla = a.cla,
        cma = Table1D(a.cma.x, a.cma.y .* cma_mult),
        cmq = a.cmq,
        cl_trim = a.cl_trim,
        alpha_trim = a.alpha_trim + alpha_trim_delta,
        kd_alpha2 = a.kd_alpha2,
    )
end
