# Build the native window (`host/`), or explain why there isn't one.
#
# Separate from build_app.jl so the expensive half of the build (a
# nonincremental system image, ~20 minutes) is never reached with a host that
# was never going to compile.
#
# Its absence is a warning rather than an error: `bin/ssjl-server.exe` runs on
# its own and opens a browser, which is what a build on a machine without a
# Rust toolchain should degrade to. The release is expected to have the window.

"""
    build_host(root) -> String or nothing

Compile `host/` in release mode and return the path to the produced binary.
Returns `nothing` — with a warning — when there is no crate or no toolchain.

Cargo's output is captured rather than inherited. Inheriting it means this
process holds a pipe open for the child, and a build that is interrupted
midway leaves the parent stuck at exit printing "waiting for IO to finish"
with no indication of what it is waiting for.
"""
function build_host(root::AbstractString)
    crate = joinpath(root, "host")
    if !isfile(joinpath(crate, "Cargo.toml"))
        @warn "no host/ crate — the build will have no native window" crate
        return nothing
    end
    if Sys.which("cargo") === nothing
        @warn """
              No Rust toolchain (https://rustup.rs), so no native window will be
              built. bin/ssjl-server.exe still works and opens a browser."""
        return nothing
    end
    @info "building the native window" crate
    out = IOBuffer()
    ok = try
        run(pipeline(Cmd(`cargo build --release --locked`; dir = crate);
                     stdout = out, stderr = out))
        true
    catch err
        @warn "the native window failed to build" exception = err
        false
    end
    log = String(take!(out))
    ok || (println(log); return nothing)

    exe = joinpath(crate, "target", "release", Sys.iswindows() ? "ssjl.exe" : "ssjl")
    if !isfile(exe)
        @warn "cargo reported success but produced no binary" expected = exe
        println(log)
        return nothing
    end
    @info "native window built" exe size_kb = round(Int, filesize(exe) / 1024)
    exe
end
