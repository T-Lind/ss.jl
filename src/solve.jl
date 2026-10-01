# Scalar targeting for expensive mission evaluations. Illinois regula falsi
# retains a sign-changing bracket; its weights are separate from measured data.

"""
    SolveResult

Outcome of scalar targeting. `x` and `value` always identify the same actual
finite evaluation, when one exists. `iterations` counts all calls to `f`;
`history` records them, including failed evaluations as NaN.

Statuses: `:converged`, `:no_bracket` (usable endpoints have the same sign),
`:max_iter` (evaluation budget exhausted), or `:infeasible` (no usable interval,
including a failed interior evaluation that breaks the continuity assumption).
`lo` and `hi` report the usable search bounds, before bracket refinement.
"""
struct SolveResult
    x::Float64
    value::Float64
    target::Float64
    iterations::Int
    status::Symbol
    lo::Float64
    hi::Float64
    history::Vector{Tuple{Float64,Float64}}
end

converged(r::SolveResult) = r.status === :converged

"""
    find_root(f, lo, hi; target=0.0, xtol=0.0, ftol=0.0, max_iter=24)

Find a root of `f(x) - target` with Illinois regula falsi. Non-finite values
or exceptions mean an infeasible design. Failed endpoints retreat toward the
opposite bound, with at most eight retries and within the total `max_iter`
evaluation budget. A failed interior evaluation returns `:infeasible` rather
than inventing a sign for an unflown mission. Interrupts propagate.

`xtol=0` selects 1e-4 of the initial interval; `ftol` is a metric tolerance.
Convergence by interval width assumes a continuous metric on the bracket.
"""
function find_root(f, lo::Float64, hi::Float64; target::Float64 = 0.0,
                   xtol::Float64 = 0.0, ftol::Float64 = 0.0,
                   max_iter::Int = 24)
    all(isfinite, (lo, hi, target, xtol, ftol)) ||
        throw(ArgumentError("bounds, target and tolerances must be finite"))
    xtol >= 0 && ftol >= 0 || throw(ArgumentError("tolerances must be nonnegative"))
    max_iter > 0 || throw(ArgumentError("max_iter must be positive"))
    lo, hi = min(lo, hi), max(lo, hi)
    isfinite(hi - lo) || throw(ArgumentError("search interval is too large"))
    hist = Tuple{Float64,Float64}[]
    xt = xtol > 0 ? xtol : 1e-4 * max(hi - lo, eps())
    best_x, best_v, best_f = NaN, NaN, Inf
    ulo, uhi = lo, hi

    result(status) = SolveResult(best_x, best_v, target, length(hist),
                                status, ulo, uhi, hist)
    function eval!(x)
        length(hist) < max_iter || return NaN
        v = try
            Float64(f(x))
        catch err
            err isa InterruptException && rethrow()
            NaN
        end
        residual = v - target
        push!(hist, (x, v))
        if isfinite(residual) && abs(residual) < best_f
            best_x, best_v, best_f = x, v, abs(residual)
        end
        residual
    end
    function usable(x, toward)
        v = eval!(x)
        for _ in 1:8
            (isfinite(v) || length(hist) >= max_iter || x == toward) && break
            x += 0.5 * (toward - x)
            v = eval!(x)
        end
        x, v
    end

    lo, flo = usable(lo, hi)
    isfinite(flo) || return result(length(hist) >= max_iter ? :max_iter : :infeasible)
    abs(flo) <= ftol && return result(:converged)
    length(hist) >= max_iter && return result(:max_iter)
    hi, fhi = usable(hi, lo)
    ulo, uhi = lo, hi
    isfinite(fhi) || return result(length(hist) >= max_iter ? :max_iter : :infeasible)
    abs(fhi) <= ftol && return result(:converged)
    signbit(flo) == signbit(fhi) && return result(:no_bracket)

    side = 0
    vlo, vhi = hist[findlast(h -> h[1] == lo, hist)][2], hist[end][2]
    function bracket_result()
        best_x, best_v = abs(vlo - target) <= abs(vhi - target) ? (lo, vlo) : (hi, vhi)
        result(:converged)
    end
    while hi - lo > xt
        length(hist) >= max_iter && return result(:max_iter)
        # Normalise weights before interpolation to avoid residual overflow.
        scale = max(abs(flo), abs(fhi))
        a, b = abs(flo) / scale, abs(fhi) / scale
        x = lo + (hi - lo) * a / (a + b)
        x = clamp(x, lo + 0.01 * (hi - lo), hi - 0.01 * (hi - lo))
        lo < x < hi || return bracket_result() # adjacent floating-point bounds
        fx = eval!(x)
        isfinite(fx) || return result(:infeasible)
        abs(fx) <= ftol && return result(:converged)
        if signbit(fx) != signbit(flo)
            hi, fhi = x, fx
            vhi = hist[end][2]
            side == -1 && (flo *= 0.5)
            side = -1
        else
            lo, flo = x, fx
            vlo = hist[end][2]
            side == 1 && (fhi *= 0.5)
            side = 1
        end
    end
    bracket_result()
end
