# Freeze ss.jl into a Windows application directory with a real .exe.
#
#   julia --project=build build/build_app.jl [outdir]
#
# Produces `dist/ssjl/ssjl.exe` (plus the Julia runtime it needs). Double-click
# it and the desktop window opens; close the window and everything exits.
#
# WHY A SEPARATE ENVIRONMENT
#
# PackageCompiler is a build-time tool, not something the simulator uses. The
# main Project.toml has ZERO external dependencies — every import is a stdlib —
# and that is worth keeping true, because it is why this compiles fast, why CI
# needs no registry, and why `create_app` below is as small and as reliable as
# it is. Adding a heavyweight dep to the root manifest to build the root
# package would trade all of that away for nothing.
#
# WHAT THIS BUYS
#
# Startup. Today the first mission compiles the stack — the panel prints
# "warming up" and takes a few seconds before it will answer. `create_app`
# runs that same work at BUILD time and freezes the result into the sysimage,
# so the shipped app opens its window immediately.

using PackageCompiler

const ROOT = dirname(@__DIR__)
const OUT = length(ARGS) >= 1 ? ARGS[1] : joinpath(ROOT, "dist", "ssjl")

isdir(dirname(OUT)) || mkpath(dirname(OUT))

@info "compiling ss.jl into an application" source=ROOT dest=OUT

create_app(
    ROOT, OUT;
    executables = ["ssjl" => "julia_main"],
    precompile_execution_file = joinpath(@__DIR__, "precompile_workload.jl"),
    # The mission stack is the expensive thing to compile and the whole point
    # of freezing it, so let the compiler work on it properly.
    incremental = false,
    filter_stdlibs = false,
    force = true,
    include_lazy_artifacts = false,
)

# The pages, the shared ES modules and the two scripts that serve them are
# DATA, not code: `create_app` freezes compiled methods and knows nothing about
# them. They are read from disk at run time (which is also what makes editing a
# page show up on refresh), so they have to travel next to the executable.
#
# `julia_main` looks for them at ../share/scripts relative to bin/ssjl.exe.
const SHARE = joinpath(OUT, "share", "scripts")
rm(SHARE; recursive = true, force = true)
mkpath(SHARE)
for f in readdir(joinpath(ROOT, "scripts"))
    src = joinpath(ROOT, "scripts", f)
    # only what the app serves or runs — the plotting and analysis scripts are
    # developer tools and several of them want Python
    if isdir(src)
        f == "static" && cp(src, joinpath(SHARE, f))
    elseif endswith(f, ".html") || f in ("panelapp.jl", "desktop.jl", "panel.jl")
        cp(src, joinpath(SHARE, f))
    end
end

@info "done" exe=joinpath(OUT, "bin", "ssjl.exe") assets=SHARE
