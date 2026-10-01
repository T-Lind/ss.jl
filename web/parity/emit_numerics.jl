# Scalar solver and singular-element reference cases for the browser engine.
using SatelliteSim

# Full precision for solver histories, rather than the panel's display rounding.
function json(io, x)
    if x isa AbstractDict
        print(io, '{')
        for (i, (k, v)) in enumerate(x)
            i > 1 && print(io, ',')
            json(io, k); print(io, ':'); json(io, v)
        end
        print(io, '}')
    elseif x isa Union{AbstractVector,Tuple}
        print(io, '[')
        for (i, v) in enumerate(x)
            i > 1 && print(io, ',')
            json(io, v)
        end
        print(io, ']')
    elseif x isa Union{AbstractString,Symbol}
        print(io, repr(string(x)))
    else
        print(io, isfinite(x) ? string(x) : "null")
    end
end

cases = Dict{String,Any}()
for (name, f, lo, hi, opts) in (
    ("root", x -> x^2 - 2, 0.0, 3.0, (;)),
    ("target", x -> x^3, -1.0, 4.0, (; target = 8.0)),
    ("budget", x -> exp(x) - 5, 0.0, 40.0, (; max_iter = 3)),
    ("failed", x -> NaN, 0.0, 1.0, (; max_iter = 2)),
    ("gap", x -> 0.4 < x < 0.6 ? NaN : x - 0.5, 0.0, 1.0, (;)),
    ("clipped", x -> x > 2.5 ? NaN : 1 - x, 0.0, 10.0, (;)),
    ("overflow", x -> 1e200 * (x - 0.5), 0.0, 1.0, (;)),
)
    res = find_root(f, lo, hi; opts...)
    cases[name] = Dict(string(k) => getfield(res, k) for k in fieldnames(SolveResult))
end
open(ARGS[1], "w") do io
    json(io, cases)
end
