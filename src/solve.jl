# Scalar targeting: hold everything fixed, vary one parameter, drive one
# mission metric to a value.
#
# Every evaluation here is a whole mission — design a free return, fly an
# ascent, fly an entry — so the cost model is "a second per call, budget
# about a dozen". That rules out anything needing derivatives or a dense
# scan, and rewards a bracketing method that never leaves the interval it
# started with. Illinois-modified regula falsi does that: it keeps a valid
# bracket at every step (so a failure still returns bounds you can trust)
# while converging superlinearly, unlike plain bisection.
#
# Missions also *fail* — too heavy a pod never reaches orbit and the chain
# throws. The solver treats a non-finite evaluation as "past the feasible
# edge" and walks back toward the endpoint that worked, so searching up to
# an infeasible bound finds the boundary instead of crashing on it.

"""
    SolveResult

Outcome of a scalar solve: the parameter value `x`, the metric `value`
there, and the `history` of every (x, value) pair evaluated — worth showing,
because with expensive evaluations the path is most of the information.
`status` is `:converged`, `:no_bracket` (the metric never crossed the
target between the bounds), or `:infeasible` (no usable evaluation at all).

`lo`/`hi` are the bounds actually searched. They differ from the ones asked
for when an end produced no valid mission and the search had to walk inward
— which is a different problem from a genuine no-crossing, and wants
different advice, so it is reported rather than hidden.
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
        -> SolveResult

Find `x` in `[lo, hi]` with `f(x) == target`, using Illinois-modified
regula falsi. `f` may return a non-finite value (or throw) to mean "not a
feasible design"; the search then retreats from that side toward a bound
that did evaluate.

`xtol` defaults to 1e-4 of the initial interval and `ftol` to zero, so the
solve normally stops on the parameter rather than the metric — the metric's
own numerical noise across a full mission is larger than any sensible ftol.
"""
function find_root(f, lo::Float64, hi::Float64; target::Float64 = 0.0,
                   xtol::Float64 = 0.0, ftol::Float64 = 0.0,
                   max_iter::Int = 24)
    lo, hi = min(lo, hi), max(lo, hi)
    hist = Tuple{Float64,Float64}[]
    xt = xtol > 0 ? xtol : 1e-4 * max(hi - lo, eps())
    n = 0

    function eval!(x)
        n += 1
        v = try
            Float64(f(x))
        catch
            NaN
        end
        push!(hist, (x, v))
        v - target
    end

    # A bound that does not evaluate is walked inward until it does; without
    # this, "how much payload fits" fails on the very bound you are probing.
    function usable(x, toward)
        v = eval!(x)
        k = 0
        while !isfinite(v) && k < 8
            x = x + 0.5 * (toward - x)
            v = eval!(x)
            k += 1
        end
        (x, v)
    end

    lo0, hi0 = lo, hi
    lo, flo = usable(lo, hi)
    isfinite(flo) ||
        return SolveResult(NaN, NaN, target, n, :infeasible, lo0, hi0, hist)
    hi, fhi = usable(hi, lo)
    isfinite(fhi) ||
        return SolveResult(lo, flo + target, target, n, :infeasible, lo, hi0, hist)
    # the usable interval, fixed here: the working bracket shrinks to the
    # root as the solve proceeds, which is not what a caller wants reported
    ulo, uhi = lo, hi

    if abs(flo) <= ftol
        return SolveResult(lo, flo + target, target, n, :converged, ulo, uhi, hist)
    elseif abs(fhi) <= ftol
        return SolveResult(hi, fhi + target, target, n, :converged, ulo, uhi, hist)
    end
    if flo * fhi > 0
        # no crossing: hand back whichever end came closest, and say so
        x, v = abs(flo) <= abs(fhi) ? (lo, flo) : (hi, fhi)
        return SolveResult(x, v + target, target, n, :no_bracket, ulo, uhi, hist)
    end

    side = 0
    x, fx = lo, flo
    while n < max_iter && hi - lo > xt
        x = lo - flo * (hi - lo) / (fhi - flo)          # regula falsi step
        # keep the iterate strictly inside, or a stalled end can pin it
        x = clamp(x, lo + 0.01 * (hi - lo), hi - 0.01 * (hi - lo))
        fx = eval!(x)
        if !isfinite(fx)                                 # infeasible interior
            hi = x; fhi = sign(flo) * abs(fhi) * 0.5
            continue
        end
        if abs(fx) <= ftol
            return SolveResult(x, fx + target, target, n, :converged, ulo, uhi, hist)
        end
        if fx * flo < 0
            hi, fhi = x, fx
            # Illinois: halve the retained end's value so it cannot stagnate
            side == -1 && (flo *= 0.5)
            side = -1
        else
            lo, flo = x, fx
            side == 1 && (fhi *= 0.5)
            side = 1
        end
    end
    SolveResult(x, fx + target, target, n,
                hi - lo <= xt ? :converged : :no_bracket, ulo, uhi, hist)
end
