# Local mission-control panel: configure, run, and explore missions in the
# browser. Pure stdlib (raw Sockets HTTP) — no package dependencies.
#
#   julia --project -t auto scripts/panel.jl [port]
#
# then open http://localhost:8137 (default port). The page posts
# form-encoded parameters to /api/run and /api/sweep; the server executes
# the full mission design + flight (~1 s per run after warmup) and returns
# JSON with metrics, decimated trajectories, and events.

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Sockets
using Printf

const PAGE_PATH = joinpath(@__DIR__, "panel_page.html")
const LAUNCH_PATH = joinpath(@__DIR__, "launch_page.html")
const PORT = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 8137

# ---------------------------------------------------------------- helpers --

"Minimal URL decoding (+, %xx)."
function urldecode(s::AbstractString)
    buf = IOBuffer()
    i = firstindex(s)
    while i <= lastindex(s)
        c = s[i]
        if c == '+'
            write(buf, ' ')
        elseif c == '%' && i + 2 <= lastindex(s)
            write(buf, parse(UInt8, s[i+1:i+2]; base = 16))
            i += 2
        else
            write(buf, c)
        end
        i = nextind(s, i)
    end
    String(take!(buf))
end

"Parse an application/x-www-form-urlencoded body into a Dict."
function parse_form(body::AbstractString)
    d = Dict{String,String}()
    for pair in split(body, '&'; keepempty = false)
        kv = split(pair, '='; limit = 2)
        d[urldecode(kv[1])] = length(kv) == 2 ? urldecode(kv[2]) : ""
    end
    d
end

getf(d, k, def) = haskey(d, k) && !isempty(d[k]) ? parse(Float64, d[k]) : def

"Tiny JSON writer: handles Dict/Vector/String/Number/Bool/Nothing/Symbol."
function json(io::IO, x)
    if x isa AbstractDict
        print(io, '{')
        first_ = true
        for (k, v) in x
            first_ || print(io, ',')
            json(io, string(k)); print(io, ':'); json(io, v)
            first_ = false
        end
        print(io, '}')
    elseif x isa Union{AbstractVector,Tuple}
        print(io, '[')
        for (i, v) in enumerate(x)
            i > 1 && print(io, ',')
            json(io, v)
        end
        print(io, ']')
    elseif x isa AbstractString || x isa Symbol
        print(io, '"')
        for c in string(x)
            c == '"' ? print(io, "\\\"") :
            c == '\\' ? print(io, "\\\\") :
            c == '\n' ? print(io, "\\n") : print(io, c)
        end
        print(io, '"')
    elseif x isa Bool || x === nothing
        print(io, x === nothing ? "null" : x)
    elseif x isa AbstractFloat
        isfinite(x) ? print(io, round(x; sigdigits = 8)) : print(io, "null")
    else
        print(io, x)
    end
end
json(x) = sprint(json, x)

# ------------------------------------------------------------ mission api --

"Build a LaunchVehicle from panel parameters."
function lv_from_params(p)
    payload = getf(p, "pod_mass", 350.0)
    LaunchVehicle(
        name = "Sable (panel)",
        stages = [
            Stage(:sable1, getf(p, "s1_dry", 3800.0), getf(p, "s1_prop", 42000.0),
                  getf(p, "s1_thrust_kn", 950.0) * 1e3, getf(p, "s1_isp", 305.0), 0.80),
            Stage(:sable2, getf(p, "s2_dry", 900.0), getf(p, "s2_prop", 9500.0),
                  getf(p, "s2_thrust_kn", 95.0) * 1e3, getf(p, "s2_isp", 345.0), 0.0),
            Stage(:sablek, getf(p, "s3_dry", 140.0), getf(p, "s3_prop", 950.0),
                  getf(p, "s3_thrust_kn", 15.0) * 1e3, getf(p, "s3_isp", 315.0), 0.0),
        ],
        fairing_mass = getf(p, "fairing", 150.0),
        payload_mass = payload,
        sref = pi * (getf(p, "diameter", 1.8) / 2)^2,
        cd = SatelliteSim.LV_CD_TABLE,
    )
end

"Decimate a vector to at most n points (keeping ends)."
function deci(v, n)
    length(v) <= n && return collect(Float64, v)
    idx = unique(round.(Int, range(1, length(v); length = n)))
    Float64[v[i] for i in idx]
end
deci_idx(len, n) = len <= n ? collect(1:len) :
                   unique(round.(Int, range(1, len; length = n)))

function panel_mission(p)::Dict{String,Any}
    ms = moonshot(
        pod_mass = getf(p, "pod_mass", 350.0),
        h_park = getf(p, "h_park_km", 200.0) * 1e3,
        hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3,
        hp_return = getf(p, "hp_return_km", 35.0) * 1e3,
        inclination = deg2rad_(getf(p, "incl_deg", 28.5)),
        lv = lv_from_params(p),
        tli_mag_err = getf(p, "tli_mag_err_pct", 0.0) / 100,
        tli_point_err = deg2rad_(getf(p, "tli_point_err_deg", 0.0)),
    )
    asc, cis, ent = ms.ascent, ms.cislunar, ms.entry
    el = asc.elements

    # 3D scene payload: true ECI geometry in units of 1000 km. The client
    # renders the inclined trajectory plane, the textured globe about the real
    # pole (scene z = ECI z), and launch/splashdown markers fixed to the
    # rotating surface — all in one consistent frame.
    L = cis.log
    k = max(2, length(L.t) ÷ 3)
    nrm = SatelliteSim.vunit(SatelliteSim.vcross((L.mx[1], L.my[1], L.mz[1]),
                                                 (L.mx[k], L.my[k], L.mz[k])))
    idx = deci_idx(length(L.t), 1600)
    px = Float64[]; py = Float64[]; pz = Float64[]
    mx = Float64[]; my = Float64[]; mz = Float64[]
    tt = Float64[]; pp = Int[]
    for i in idx
        push!(px, L.rx[i] / 1e6); push!(py, L.ry[i] / 1e6); push!(pz, L.rz[i] / 1e6)
        push!(mx, L.mx[i] / 1e6); push!(my, L.my[i] / 1e6); push!(mz, L.mz[i] / 1e6)
        push!(tt, L.t[i]); push!(pp, L.phase[i])
    end

    EL = ent.log
    eidx = deci_idx(length(EL.t), 500)
    AL = asc.log
    aidx = deci_idx(length(AL.t), 400)

    # ascent track is already ECI; the entry log is geodetic — rebuild ECI
    ax3 = [AL.rx[i] / 1e6 for i in aidx]
    ay3 = [AL.ry[i] / 1e6 for i in aidx]
    az3 = [AL.rz[i] / 1e6 for i in aidx]
    at3 = [AL.t[i] for i in aidx]
    ex3 = Float64[]; ey3 = Float64[]; ez3 = Float64[]; et3 = Float64[]
    for i in eidx
        re_ = ecef_from_geodetic(EL.lat[i], EL.lon[i], EL.h[i])
        th = SatelliteSim.earth_rotation_angle(0.0, EL.t[i])
        reci = SatelliteSim.rot_z(re_, -th)
        push!(ex3, reci[1] / 1e6); push!(ey3, reci[2] / 1e6); push!(ez3, reci[3] / 1e6)
        push!(et3, EL.t[i])
    end

    prop_margin = cis.m - (ms.lv.stages[end].mdry + ms.lv.payload_mass)
    ei = findfirst(e -> e.name == :entry_interface, ent.events)

    # did the free-return design actually hit its targets? (a prop-starved
    # TLI still "flies", but the result is not the requested mission)
    hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3
    hp_ret = getf(p, "hp_return_km", 35.0) * 1e3
    on_target = abs(cis.perilune_alt - hp_moon) <= max(0.05 * hp_moon, 50e3) &&
                abs(cis.vac_perigee_alt - hp_ret) <= 20e3

    events = Any[]
    for e in asc.events
        push!(events, Dict("phase" => "ascent", "name" => string(e.name), "t" => e.t))
    end
    push!(events, Dict("phase" => "cislunar", "name" => "tli_ignition", "t" => cis.t_tli))
    push!(events, Dict("phase" => "cislunar", "name" => "tli_cutoff",
                       "t" => cis.t_tli + cis.burn_duration))
    push!(events, Dict("phase" => "cislunar", "name" => "perilune", "t" => cis.t_perilune))
    push!(events, Dict("phase" => "cislunar", "name" => "entry_handoff", "t" => cis.t))
    for e in ent.events
        push!(events, Dict("phase" => "entry", "name" => string(e.name), "t" => e.t))
    end

    Dict{String,Any}(
        "ok" => true,
        "metrics" => Dict(
            "on_target" => on_target,
            "tcm_dv" => ms.cruise === nothing ? nothing : ms.cruise.tcm_dv,
            "tcm_prop_kg" => ms.cruise === nothing ? nothing : ms.cruise.tcm_prop,
            "rcs_margin_kg" => ms.cruise === nothing ? nothing : ms.cruise.rcs.margin,
            "liftoff_t" => liftoff_mass(ms.lv) / 1e3,
            "park_perigee_km" => (el.rp - RE_MEAN) / 1e3,
            "park_apogee_km" => (el.ra - RE_MEAN) / 1e3,
            "incl_deg" => rad2deg_(el.i),
            "tli_dv" => cis.dv_tli,
            "tli_burn_s" => cis.burn_duration,
            "prop_margin_kg" => prop_margin,
            "perilune_km" => cis.perilune_alt / 1e3,
            "t_perilune_d" => cis.t_perilune / 86400,
            "vac_perigee_km" => cis.vac_perigee_alt / 1e3,
            "ei_v_ms" => ei === nothing ? NaN : ent.events[ei].vrel,
            "peak_g" => ent.peak_gload,
            "peak_q_wcm2" => ent.peak_qdot / 1e4,
            "heat_mj" => ent.heat_load / 1e6,
            "splash_lat" => rad2deg_(ent.lat_splash),
            "splash_lon" => rad2deg_(ent.lon_splash),
            "v_splash" => ent.v_splash,
            "t_days" => ent.t_splash / 86400,
        ),
        "cis" => Dict("t" => tt, "x" => px, "y" => py, "z" => pz,
                      "mx" => mx, "my" => my, "mz" => mz, "ph" => pp,
                      "n" => [nrm[1], nrm[2], nrm[3]]),
        "asc3d" => Dict("t" => at3, "x" => ax3, "y" => ay3, "z" => az3),
        "ent3d" => Dict("t" => et3, "x" => ex3, "y" => ey3, "z" => ez3),
        "sites" => Dict(
            "launch_lat" => rad2deg_(ms.guid.site_lat),
            "launch_lon" => rad2deg_(ms.guid.site_lon),
            "splash_lat" => rad2deg_(ent.lat_splash),
            "splash_lon" => rad2deg_(ent.lon_splash),
        ),
        "ascent" => Dict("t" => deci(AL.t[aidx], 400), "h" => deci(AL.h[aidx] ./ 1e3, 400),
                         "v" => deci(AL.vrel[aidx], 400), "qbar" => deci(AL.qbar[aidx] ./ 1e3, 400),
                         "gamma" => deci(AL.gamma[aidx], 400), "mach" => deci(AL.mach[aidx], 400),
                         "thrust" => deci(AL.thrust[aidx], 400), "m" => deci(AL.m[aidx], 400),
                         "dr" => deci(AL.downrange[aidx], 400)),
        "entry" => Dict("t" => deci(EL.t[eidx] .- EL.t[1], 500), "h" => deci(EL.h[eidx] ./ 1e3, 500),
                        "v" => deci(EL.vrel[eidx], 500), "g" => deci(EL.gload[eidx], 500),
                        "q" => deci((EL.qdot_conv[eidx] .+ EL.qdot_rad[eidx]) ./ 1e4, 500)),
        "events" => events,
    )
end

"Procedural rocket geometry for the panel's vehicle viewer."
function rocket_geometry(p)::Dict{String,Any}
    d = getf(p, "diameter", 1.8)
    props = [getf(p, "s1_prop", 42000.0), getf(p, "s2_prop", 9500.0),
             getf(p, "s3_prop", 950.0)]
    mesh, secs = rocket_mesh(diameter = d, prop_masses = props, nseg = 36)
    Dict{String,Any}(
        "ok" => true,
        "length" => secs[end].x1,
        "diameter" => d,
        "sections" => [Dict("name" => string(s.name), "x0" => s.x0, "x1" => s.x1,
                            "t0" => s.t0, "t1" => s.t1)
                       for s in secs],
        "tris" => [Float64[t[1]..., t[2]..., t[3]...] for t in mesh.tris],
    )
end

const SWEEPABLE = ["pod_mass", "h_park_km", "hp_moon_km", "hp_return_km", "incl_deg",
                   "s3_prop", "s3_isp", "s2_prop", "diameter"]

function run_sweep(p)::Dict{String,Any}
    param = get(p, "sweep_param", "pod_mass")
    param in SWEEPABLE || return Dict{String,Any}("ok" => false,
        "error" => "unknown sweep parameter: $param")
    lo = getf(p, "sweep_min", 250.0)
    hi = getf(p, "sweep_max", 450.0)
    nv = clamp(Int(getf(p, "sweep_n", 9.0)), 2, 41)
    vals = collect(range(lo, hi; length = nv))
    runs = Vector{Any}(undef, nv)
    Threads.@threads for i in 1:nv
        q = copy(p)
        q[param] = string(vals[i])
        runs[i] = try
            r = panel_mission(q)
            Dict("ok" => true, "metrics" => r["metrics"])
        catch err
            Dict("ok" => false, "error" => sprint(showerror, err))
        end
    end
    Dict{String,Any}("ok" => true, "param" => param, "values" => vals, "runs" => runs)
end

# -------------------------------------------------------------- http loop --

function respond(sock, status, ctype, body)
    write(sock, "HTTP/1.1 $status\r\nContent-Type: $ctype\r\n" *
                "Content-Length: $(sizeof(body))\r\nConnection: close\r\n\r\n")
    write(sock, body)
end

function handle(sock)
    try
        reqline = readline(sock)
        isempty(reqline) && return
        parts = split(reqline, ' ')
        length(parts) < 2 && return
        method, path = parts[1], parts[2]
        clen = 0
        while true
            h = readline(sock)
            (isempty(h) || h == "\r") && break
            if startswith(lowercase(h), "content-length:")
                clen = parse(Int, strip(split(h, ':')[2]))
            end
        end
        body = clen > 0 ? String(read(sock, clen)) : ""

        if method == "GET" && (path == "/" || startswith(path, "/?"))
            # re-read per request so page edits show on refresh (dev-friendly)
            respond(sock, "200 OK", "text/html; charset=utf-8", read(PAGE_PATH, String))
        elseif method == "GET" && (path == "/launch" || startswith(path, "/launch?"))
            respond(sock, "200 OK", "text/html; charset=utf-8", read(LAUNCH_PATH, String))
        elseif method == "POST" && path == "/api/run"
            p = parse_form(body)
            out = try
                panel_mission(p)
            catch err
                Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))
            end
            respond(sock, "200 OK", "application/json", json(out))
        elseif method == "POST" && path == "/api/sweep"
            p = parse_form(body)
            out = try
                run_sweep(p)
            catch err
                Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))
            end
            respond(sock, "200 OK", "application/json", json(out))
        elseif method == "POST" && path == "/api/geometry"
            p = parse_form(body)
            out = try
                rocket_geometry(p)
            catch err
                Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))
            end
            respond(sock, "200 OK", "application/json", json(out))
        else
            respond(sock, "404 Not Found", "text/plain", "not found")
        end
    catch
        # client went away mid-request; nothing to do
    finally
        close(sock)
    end
end

println("warming up (first mission run compiles the stack)...")
t0 = time()
panel_mission(Dict{String,String}())
@printf("ready in %.1f s — panel at http://localhost:%d  (Ctrl-C to stop)\n",
        time() - t0, PORT)

server = listen(IPv4(127, 0, 0, 1), PORT)
while true
    sock = accept(server)
    @async handle(sock)
end
