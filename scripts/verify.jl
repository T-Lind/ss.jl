# Reproducible mission verification without rewriting the committed CSVs.
# julia --project=. -t auto scripts/verify.jl [--full] [--samples=N] [--output=PATH]
using SatelliteSim, Printf, TOML, SHA

function verify_main(args)
    full = false
    samples = 64
    output = joinpath(@__DIR__, "..", "output", "verification.toml")
    for arg in args
        if arg == "--full"
            full = true
        elseif startswith(arg, "--samples=")
            samples = parse(Int, split(arg, '='; limit = 2)[2])
            samples > 0 || throw(ArgumentError("--samples must be positive"))
        elseif startswith(arg, "--output=")
            output = split(arg, '='; limit = 2)[2]
            isempty(output) && throw(ArgumentError("--output needs a path"))
        elseif arg in ("--help", "-h")
            println("Usage: julia --project=. -t auto scripts/verify.jl [--full] [--samples=N] [--output=PATH]")
            println("Default: LEO entry, circumlunar TOML mission, Earth orbit, suborbital hop, seeded footprint.")
            println("--full adds terrain landing, lunar ascent, rendezvous and return. Writes a TOML report; exits nonzero on failure.")
            return 0
        else
            throw(ArgumentError("unknown option: $arg"))
        end
    end
    root = dirname(@__DIR__)
    spec = joinpath(root, "missions", "moonshot.toml")
    gitread(args...) = try readchomp(Cmd(["git", "-C", root, args...])) catch; "unavailable" end
    report = Dict{String,Any}(
        "schema_version" => 1, "generated_at_unix" => time(),
        "julia_version" => string(VERSION), "platform" => string(Sys.MACHINE),
        "threads" => Threads.nthreads(), "revision" => gitread("rev-parse", "HEAD"),
        "working_tree_dirty" => !isempty(gitread("status", "--porcelain")),
        "mission_spec_sha256" => bytes2hex(sha256(read(spec))),
        "montecarlo_seed" => 2026, "montecarlo_samples" => samples, "full" => full,
        "cases" => Dict{String,Any}[],
    )
    function mission!(f, name)
        checks = Dict{String,Any}[]
        check(label, pass; value = "", expected = "") = push!(checks,
            Dict("name" => label, "passed" => Bool(pass), "value" => value, "expected" => expected))
        row = Dict{String,Any}("name" => name, "checks" => checks)
        start = time()
        try
            f(check)
            row["passed"] = !isempty(checks) && all(c["passed"] for c in checks)
        catch err
            err isa InterruptException && rethrow()
            row["passed"] = false
            row["error"] = sprint(showerror, err)
        end
        row["elapsed_seconds"] = time() - start
        push!(report["cases"], row)
        @printf("%-30s %s (%.2f s)\n", name, row["passed"] ? "PASS" : "FAIL", row["elapsed_seconds"])
        for c in checks
            c["passed"] || println("  ", c["name"], ": ", c["value"], "; expected ", c["expected"])
        end
        haskey(row, "error") && println("  ", row["error"])
    end
    scn = Ref{Any}(nothing)
    mission!("targeted LEO entry") do check
        s, _, res = west_coast_scenario()
        scn[] = s
        check("termination", res.terminated == :splashdown; value = string(res.terminated), expected = "splashdown")
        check("target miss km", res.miss_km < 15; value = res.miss_km, expected = "< 15")
        check("peak g", 3 < res.peak_gload < 12; value = res.peak_gload, expected = "3..12")
        check("heating W/cm2", 20 < res.peak_qdot / 1e4 < 200; value = res.peak_qdot / 1e4, expected = "20..200")
        check("splash speed m/s", res.v_splash < 8; value = res.v_splash, expected = "< 8")
    end
    mission!("circumlunar TOML mission") do check
        ms = run_mission(spec)
        check("design", ms.design_status == :converged; value = string(ms.design_status), expected = "converged")
        check("perilune km", abs(ms.cislunar.perilune_alt - 2e6) < 2e3;
              value = ms.cislunar.perilune_alt / 1e3, expected = "2000 +/- 2")
        check("return perigee km", abs(ms.cislunar.vac_perigee_alt - 50e3) < 1e3;
              value = ms.cislunar.vac_perigee_alt / 1e3, expected = "50 +/- 1")
        check("first pass", ms.cislunar.miss_passes == 0;
              value = ms.cislunar.miss_passes, expected = "0")
        check("splashdown", ms.entry !== nothing && ms.entry.terminated == :splashdown;
              expected = "entry reaches splashdown")
        if ms.entry !== nothing
            check("duration days", 5 < ms.entry.t_splash / 86400 < 8;
                  value = ms.entry.t_splash / 86400, expected = "5..8")
            check("peak g", 3 < ms.entry.peak_gload < 10; value = ms.entry.peak_gload, expected = "3..10")
        end
    end
    mission!("Earth LEO mission") do check
        eo = earthorbit()
        check("orbit achieved", eo.on_target; value = eo.on_target, expected = "true")
        check("perigee km", abs((eo.elements.rp - RE_MEAN) - 200e3) < 10e3;
              value = (eo.elements.rp - RE_MEAN) / 1e3, expected = "200 +/- 10")
    end
    mission!("100 km suborbital hop") do check
        hop = suborbital(profile = :hop, apogee = 100e3, strict = false)
        check("termination", hop.outcome == :splashdown; value = string(hop.outcome), expected = "splashdown")
        check("apogee km", abs(hop.apogee - 100e3) < 4e3; value = hop.apogee / 1e3, expected = "100 +/- 4")
        check("range km", hop.range < 15e3; value = hop.range / 1e3, expected = "< 15")
    end
    mission!("seeded LEO footprint") do check
        scn[] === nothing && error("targeted LEO scenario was unavailable")
        runs = run_montecarlo(scn[]; n = samples, seed = 2026)
        stats = mc_statistics(runs)
        check("successful samples", stats.n_ok == samples; value = stats.n_ok, expected = string(samples))
        check("finite dispersion", isfinite(stats.cep50_km) && isfinite(stats.r95_km);
              value = stats.cep50_km, expected = "finite CEP50 and R95")
        check("radial quantiles", stats.r95_km >= stats.cep50_km;
              value = stats.r95_km, expected = "R95 >= CEP50")
    end
    if full
        mission!("lunar landing and return") do check
            lnd = default_lander(); orb = Orbiter()
            lv = starship_expendable(payload = lander_mass(lnd) + orbiter_mass(orb))
            ls = apollo_landing(lander = lnd, lv = lv, orbiter = orb, kick_angle = deg2rad_(5.0))
            check("terrain touchdown", ls.descent.outcome == :touchdown;
                  value = string(ls.descent.outcome), expected = "touchdown")
            check("lander propellant kg", ls.prop_margin > 0; value = ls.prop_margin, expected = "> 0")
            rr = moonreturn(ls)
            check("lunar insertion", rr.ascent.outcome == :insertion; value = string(rr.ascent.outcome), expected = "insertion")
            check("rendezvous delta-v m/s", rr.dv_rendezvous < 200; value = rr.dv_rendezvous, expected = "< 200")
            check("return splashdown", rr.entry !== nothing && rr.entry.terminated == :splashdown;
                  expected = "entry reaches splashdown")
            check("orbiter propellant kg", rr.prop_orbiter_left > 0; value = rr.prop_orbiter_left, expected = "> 0")
        end
    end
    report["passed"] = all(row["passed"] for row in report["cases"])
    mkpath(dirname(abspath(output)))
    open(output, "w") do io
        TOML.print(io, report; sorted = true)
    end
    println("Report: ", abspath(output))
    report["passed"] ? 0 : 1
end

if abspath(PROGRAM_FILE) == @__FILE__
    exit(verify_main(ARGS))
end
