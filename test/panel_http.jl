# HTTP-level tests for the mission-control panel.
#
# The invariant this file exists to protect: *no request ever gets an empty
# reply*. A socket closed without a response is indistinguishable, in a
# browser, from the server being down — it surfaces as "Failed to fetch" with
# nothing in the log to explain it.

using Test
using Sockets

include(joinpath(@__DIR__, "..", "scripts", "panelapp.jl"))

@testset "panel serialization and numeric inputs" begin
    @test PanelApp.json("\t\r\n\b\f\0") == "\"\\u0009\\u000d\\u000a\\u0008\\u000c\\u0000\""
    @test PanelApp.json("\"\\Moon 🌕") == "\"\\\"\\\\Moon 🌕\""
    for text in ("NaN", "Inf", "-Inf", "1e999")
        @test_throws ArgumentError PanelApp.getf(Dict("diameter" => text), "diameter", 1.8)
    end
    @test PanelApp.getf(Dict("diameter" => "  "), "diameter", 1.8) == 1.8
    @test PanelApp.getf(Dict("diameter" => " 2.4 "), "diameter", 1.8) == 2.4
end

"""
A TCP port free on the IPv4 loopback interface, verified free on the IPv6
loopback too when this host has one to offer.

`start_panel`'s own IPv6 bind is best-effort (a host without IPv6 still
works), so an IPv4-only probe here could hand back a port that is free on
IPv4 but already taken on IPv6 — the soft IPv6 failure downstream and this
port choice would then have unrelated causes, and a test asserting on the
IPv6 listener would fail for the wrong reason.
"""
function free_port()
    # cheap capability check, once: does this host have an IPv6 loopback at
    # all? If not, every port collides on IPv6 the same way, and retrying is
    # pointless — that is exactly the condition start_panel already tolerates.
    ipv6_capable = try
        close(listen(IPv6(0, 0, 0, 0, 0, 0, 0, 1), 0))
        true
    catch
        false
    end
    for _ in 1:20
        s4 = listen(IPv4(127, 0, 0, 1), 0)
        p = Int(getsockname(s4)[2])
        close(s4)
        ipv6_capable || return p
        s6 = try
            listen(IPv6(0, 0, 0, 0, 0, 0, 0, 1), p)
        catch
            nothing
        end
        s6 === nothing && continue   # this specific port collided on IPv6; retry
        close(s6)
        return p
    end
    error("could not find a port free on both IPv4 and IPv6 after 20 tries")
end

"Split a raw HTTP response into (header block, body)."
function split_response(raw::AbstractString)
    i = findfirst("\r\n\r\n", raw)
    i === nothing && return (raw, "")
    (raw[1:first(i)-1], raw[last(i)+1:end])
end

"""
    http(method, path; port, host, body, timeout) -> (status, headers, body)

Minimal HTTP/1.1 client — enough to talk to the panel and nothing more.
Errors if no response arrives within `timeout` seconds, which is exactly the
failure this suite is here to catch.
"""
function http(method::AbstractString, path::AbstractString;
              port::Int, host = Sockets.localhost,
              body::AbstractString = "", timeout::Float64 = 60.0)
    out = Ref{String}("")
    t = @async begin
        sock = connect(host, port)
        req = "$method $path HTTP/1.1\r\nHost: localhost\r\n"
        if !isempty(body)
            req *= "Content-Type: application/x-www-form-urlencoded\r\n" *
                   "Content-Length: $(sizeof(body))\r\n"
        end
        req *= "Connection: close\r\n\r\n" * body
        write(sock, req)
        out[] = read(sock, String)
        close(sock)
    end
    timedwait(() -> istaskdone(t), timeout) === :ok ||
        error("no response within $timeout s: $method $path")
    istaskfailed(t) && throw(TaskFailedException(t))
    head, bod = split_response(out[])
    lines = split(head, "\r\n")
    status = parse(Int, split(lines[1], ' ')[2])
    hdrs = Dict{String,String}()
    for l in lines[2:end]
        kv = split(l, ':'; limit = 2)
        length(kv) == 2 && (hdrs[lowercase(strip(kv[1]))] = String(strip(kv[2])))
    end
    (status, hdrs, bod)
end

@testset "earth orbit" begin
    # each requested target is reached within tolerance — or the failure is
    # the honest physical one
    eo = earthorbit(target = :leo, strict = false)
    @test eo.outcome === :on_orbit
    @test eo.on_target
    @test isempty(eo.burns)                       # direct ascent, no transfer

    custom = earthorbit(target = :custom, perigee_alt = 250e3,
                        apogee_alt = 250e3, inclination = deg2rad_(40.0),
                        strict = false)
    @test custom.target.name === :custom
    @test custom.on_target
    @test abs((custom.elements.rp - RE_MEAN) / 1e3 - 250) < 15
    @test abs(rad2deg_(custom.elements.i) - 40.0) < 1.0

    eo = earthorbit(target = :polar, strict = false)
    @test eo.on_target
    @test abs(rad2deg_(eo.elements.i) - 90.0) < 1.0

    eo = earthorbit(target = :molniya, strict = false)
    @test eo.on_target
    @test abs((eo.elements.ra - SatelliteSim.RE_MEAN)/1e3 - 39400) < 250
    @test abs(rad2deg_(eo.elements.i) - 63.4) < 1.0
    # the finite burn's gravity loss is small and positive against the plan
    for b in eo.burns
        @test b.dv >= b.dv_plan - 5.0
        @test b.dv <= b.dv_plan * 1.05 + 10.0
    end

    # the reference kick stage cannot reach GEO (~4.3 km/s against ~3.3 of
    # capacity); saying so is correct behaviour, not a bug
    eo = earthorbit(target = :geo, strict = false)
    @test eo.outcome === :prop_depleted

    # the round trip: up, around, retrograde burn, splashdown on parachutes
    eo = earthorbit(target = :leo, deorbit = true, n_orbits = 1.0, strict = false)
    @test eo.outcome === :splashdown
    @test eo.entry !== nothing
    @test eo.entry.v_splash < 15.0
    @test 2.0 < eo.entry.peak_gload < 12.0        # ballistic LEO entry range
end

@testset "stage naming" begin
    # the stem the ascent's sep_/ignition_ events are built from
    @test PanelApp.stage_slug("Sable") == "sable"
    @test PanelApp.stage_slug("Sable (panel)") == "sable"     # the default
    @test PanelApp.stage_slug("Saturn V") == "saturn_v"
    @test PanelApp.stage_slug("Falcon-class") == "falcon_class"
    @test PanelApp.stage_slug("Starship") == "starship"
    # a name with nothing usable in it still has to produce a legal symbol
    @test PanelApp.stage_slug("") == "sable"
    @test PanelApp.stage_slug("!!!") == "sable"
    @test PanelApp.stage_slug("  ") == "sable"
    # no leading/trailing underscore, which would render as a leading space
    @test !startswith(PanelApp.stage_slug("(x) Ares"), "_")
    @test !endswith(PanelApp.stage_slug("Ares I "), "_")
    # long names are cut so the HUD abbreviator still has room
    @test length(PanelApp.stage_slug("A Very Long Vehicle Name Indeed")) <= 14
end

@testset "panel http" begin
    # the module must reuse the already-loaded package rather than loading a
    # second copy off LOAD_PATH — a duplicate would give us two incompatible
    # sets of types with identical names
    @test PanelApp.SatelliteSim === SatelliteSim

    port = free_port()
    srv = PanelApp.start_panel(port)
    try
        st, hdrs, bod = http("GET", "/api/catalogue"; port = port)
        @test st == 200
        @test occursin("propellants", bod)
        @test occursin("engines", bod)
        @test occursin("kerolox", bod)
        @test occursin("l_diameter", bod)  # builder capability handshake

        # --- a request must always be answered ------------------------------
        # A malformed percent-escape makes urldecode throw. That used to
        # happen outside the handler's try block, so the socket closed with
        # no response at all.
        st, hdrs, bod = http("POST", "/api/run";
                             port = port, body = "pod_mass=35%ZZ")
        @test st == 200
        @test !isempty(bod)
        @test occursin("\"ok\":false", bod)

        # A field the user typed wrong is a result, not a crash
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port, body = "diameter=not-a-number")
        @test st == 200
        @test occursin("\"ok\":false", bod)

        # A request line the server cannot parse still gets an answer
        st, hdrs, bod = http("BOGUS", "/api/run"; port = port, body = "x=1")
        @test st == 404
        @test !isempty(bod)

        # A valid geometry request still works
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=3&nboost=0&diameter=1.8")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"name\":\"cabin\"", bod)

        # Landing geometry carries the lander itself, not the stale capsule
        # payload from the launcher preset.
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "mode=landing&nstages=3&diameter=10.1" *
                                    "&fairing_on=0&pod_mass=45000" *
                                    "&l_dry=3500&l_prop=9000&l_diameter=4.2")
        @test st == 200
        @test occursin("\"kind\":\"lander\"", bod)
        @test occursin("\"mass_kg\":12500", bod)
        @test occursin("\"diameter_m\":4.2", bod)
        @test occursin("\"name\":\"lander\"", bod)
        @test !occursin("\"name\":\"pod\"", bod)

        # --- the dry-mass estimate the builder shows against what it flies --
        # A stage that sizes itself must agree with its own estimate, or the
        # builder flags a correct stage as wrong: dry_estimate has to recognise
        # the catalogue engine from the stage it built, not fall back to a
        # generic thrust-to-weight. A hand-entered stage has no such engine and
        # is judged against the generic figure, which still has to be sane.
        # every value of one key, in the order the stages array emits them
        nums(b, k) = [parse(Float64, m[1]) for m in
                      eachmatch(Regex("\"" * k * "\":\\s*([-0-9.eE+]+)"), b)]
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=3&nboost=0&diameter=3.7" *
                                    "&s1_engine=merlin_1d&s1_engines=9" *
                                    "&s1_prop=411000&s1_dry_auto=1&s1_diameter=3.7")
        @test st == 200
        dry, est, frac = nums(bod, "dry_kg"), nums(bod, "dry_est_kg"),
                         nums(bod, "dry_frac")
        @test length(dry) == 3 && length(est) == 3
        @test est[1] > 0
        @test abs(dry[1] - est[1]) <= 1e-6 * est[1]
        # a Falcon-class first stage is ~25 t dry on 411 t of kerolox
        @test 20_000 <= dry[1] <= 32_000
        @test 0.04 <= frac[1] <= 0.09

        # 140 kg of structure under 9.5 t of propellant is not buildable, and
        # the estimate has to say so loudly enough for the page to flag it
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=3&nboost=0&s3_engine=manual" *
                                    "&s3_dry=140&s3_prop=9500&s3_isp=315" *
                                    "&s3_thrust_kn=15")
        @test st == 200
        dry, est, frac = nums(bod, "dry_kg"), nums(bod, "dry_est_kg"),
                         nums(bod, "dry_frac")
        @test dry[3] == 140
        @test est[3] > 3 * dry[3]
        @test frac[3] < 0.02

        # --- the builder page and the engine-mixture rule -------------------
        st, hdrs, bod = http("GET", "/build"; port = port)
        @test st == 200
        @test hdrs["content-type"] == "text/html; charset=utf-8"
        @test occursin("<html", lowercase(bod))
        st, hdrs, bod = http("GET", "/build?nstages=3&diameter=1.8"; port = port)
        @test st == 200

        # --- the analysis page ----------------------------------------------
        # The plots, the event log, the sweep and the solver moved off mission
        # control. Like /launch it takes the whole mission as a query string
        # and flies its own run, so the route has to accept one.
        st, hdrs, bod = http("GET", "/analysis"; port = port)
        @test st == 200
        @test hdrs["content-type"] == "text/html; charset=utf-8"
        @test occursin("<html", lowercase(bod))
        @test occursin("/static/charts.js", bod)
        @test occursin("/static/metrics.js", bod)
        for id in ("c_asc_mach", "c_asc_gamma", "c_asc_mass", "c_asc_path",
                   "c_coast_v", "c_energy", "c_lunar", "c_ent_qbar",
                   "c_ent_alpha", "c_ent_heat", "c_ent_wall", "c_ent_range",
                   "c_rcs_remaining", "c_rcs_rate", "c_desc_mass", "c_desc_nav")
            @test occursin("id=\"$id\"", bod)
        end
        @test occursin("drag to pan", bod)
        st, hdrs, bod = http("GET", "/analysis?mode=landing&nstages=3"; port = port)
        @test st == 200
        # a near-miss must not be served the page — /analysisx is not a route
        st, hdrs, bod = http("GET", "/analysisx"; port = port)
        @test st == 404

        # Mission control links out to it and no longer draws any chart of its
        # own. If a chart canvas reappears there, this split has been undone by
        # accident rather than on purpose.
        st, hdrs, bod = http("GET", "/"; port = port)
        @test st == 200
        @test occursin("/analysis", bod)
        @test !occursin("id=\"c_asc_h\"", bod)
        # Only the main Run button and the stale-result banner may invoke the
        # simulator. Presets, mode/orbit controls and keyboard shortcuts merely
        # invalidate the result currently on screen.
        @test length(collect(eachmatch(r"doRun\(\);", bod))) == 2
        @test !occursin("kbd\" style=\"margin-left:6px\">R", bod)
        @test !occursin("id=\"sweepchart\"", bod)

        # Configuration URLs are context, not permission to run. The launch
        # view used to POST /api/run merely by opening a builder link.
        st, hdrs, launch = http("GET", "/launch?mode=orbit&orbit=leo"; port = port)
        @test st == 200
        @test !occursin("api.post('/api/run'", launch)
        @test occursin("showEmpty(true)", launch)

        # --- navigation must not depend on opening a window -----------------
        # v0.3.1 shipped with `target="_blank"` on the analysis and launch-view
        # links. A WebView2 with no NewWindowRequested handler REFUSES a new
        # window and reports nothing at all, so in the desktop application both
        # links were dead: clicking them did nothing, no error, no navigation.
        # The vehicle-builder link, which had no target, worked — which is
        # exactly the shape of the bug report.
        #
        # There is no browser here to prove that with, so the assertion is on
        # the thing that caused it: no in-app link opens a new window.
        for page in ("/", "/build", "/analysis", "/launch", "/models", "/models?view=cabin")
            st, hdrs, bod = http("GET", page; port = port)
            @test st == 200
            @test !occursin("target=\"_blank\"", bod)
            @test !occursin("window.open(", bod)
        end

        # --- the run store --------------------------------------------------
        # /launch and /analysis read a flown trajectory by id rather than
        # re-flying one from a query string. Re-flying was both seconds of
        # duplicated simulation and a correctness bug: the trajectory shown was
        # a fresh one that only resembled the run whose numbers had been read.
        st, hdrs, bod = http("GET", "/api/runs"; port = port)
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"runs\":", bod)

        # An unknown id is a 404 that explains itself — history is capped and
        # in memory, so an id can legitimately stop resolving and "not found"
        # on its own would read as a bug.
        st, hdrs, bod = http("GET", "/api/runs/nosuchrun"; port = port)
        @test st == 404
        @test occursin("\"ok\":false", bod)
        @test occursin("nosuchrun", bod)

        # Flying through the route (not panel_mission directly) files the run
        # and gives it an id, which is what the pages then pass around.
        st, hdrs, bod = http("POST", "/api/run";
                             port = port,
                             body = "mode=suborbital&sub_profile=hop&sub_apogee_km=90")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"id\":\"r", bod)
        rid = match(r"\"id\":\"(r\d+)\"", bod)[1]

        # and it is retrievable, whole, by that id
        st, hdrs, bod = http("GET", "/api/runs/$rid"; port = port)
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"id\":\"$rid\"", bod)
        # the configuration that produced it travels with it — that is what
        # lets a history entry put the form back rather than only replay a plot
        @test occursin("\"params\":", bod)
        @test occursin("\"sub_apogee_km\":\"90\"", bod)
        @test occursin("\"apogee_km\":", bod)          # the trajectory itself

        # it now appears in the history list, with a digest and no trajectory
        st, hdrs, bod = http("GET", "/api/runs"; port = port)
        @test st == 200
        @test occursin("\"id\":\"$rid\"", bod)
        @test occursin("\"mode\":\"suborbital\"", bod)
        @test occursin("\"outcome\":", bod)
        # a digest is metadata, not a flight: no sample arrays
        @test !occursin("\"asc3d\"", bod)

        # A FAILED run is not history. Nothing was flown, so there is nothing
        # to go back to, and filing it would put an entry in the list that
        # restores to nothing.
        st, hdrs, bod = http("GET", "/api/runs"; port = port)
        before = length(collect(eachmatch(r"\"id\":\"r\d+\"", bod)))
        st, hdrs, bod = http("POST", "/api/run";
                             port = port,
                             body = "nstages=2&s1_engine=raptor_2&s1_propellant=kerolox")
        @test st == 200
        @test occursin("\"ok\":false", bod)
        @test !occursin("\"id\":\"r", bod)      # no id handed out for a non-flight
        st, hdrs, bod = http("GET", "/api/runs"; port = port)
        @test length(collect(eachmatch(r"\"id\":\"r\d+\"", bod))) == before

        # A named engine owns its mixture: naming both an engine and a
        # DIFFERENT propellant is an error the user must see, not a silent
        # override (the form used to display a mixture the server was not
        # flying). The error carries the engine and both propellant names.
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=2&s1_engine=raptor_2&s1_propellant=kerolox")
        @test st == 200
        @test occursin("\"ok\":false", bod)
        @test occursin("raptor_2", bod)
        @test occursin("methalox", bod)
        @test occursin("kerolox", bod)

        # The matched pair is fine, an omitted mixture is fine, and manual
        # mode keeps free choice.
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=2&s1_engine=raptor_2&s1_propellant=methalox")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port, body = "nstages=2&s1_engine=raptor_2")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        st, hdrs, bod = http("POST", "/api/geometry";
                             port = port,
                             body = "nstages=2&s1_engine=manual&s1_propellant=methalox")
        @test st == 200
        @test occursin("\"ok\":true", bod)

        # The rule guards the flight path too, not merely the preview
        st, hdrs, bod = http("POST", "/api/run";
                             port = port,
                             body = "mode=flyby&s2_engine=rl10b2&s2_propellant=kerolox")
        @test st == 200
        @test occursin("\"ok\":false", bod)
        @test occursin("hydrolox", bod)

        # --- a mission that fails still FLIES -------------------------------
        # A stack with almost no propellant depletes in seconds and never
        # reaches orbit. That used to be ok:false plus an error string, with
        # the fully-simulated ascent thrown away; now every leg that flew
        # comes back, ok:true, with the outcome named — and the launch page
        # flies it to wherever the simulation actually ended.
        st, hdrs, bod = http("POST", "/api/run";
                             port = port, timeout = 180.0,
                             body = "_job=failed-flight-test&mode=flyby&nstages=2&" *
                                    "s1_prop=3000&s1_dry=2500&" *
                                    "s2_prop=100&s2_dry=140")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"outcome\":\"ascent_failed\"", bod)
        @test occursin("\"ascent\":", bod)       # the leg that DID fly
        @test occursin("\"launch_lat\"", bod)    # the launch page's hard needs
        @test occursin("\"liftoff_t\"", bod)
        @test !occursin("\"cis\":", bod)         # and no leg it did not fly
        @test !occursin("\"_job\"", bod)          # transport state is not a run parameter
        st, hdrs, prog = http("GET", "/api/progress?job=failed-flight-test";
                              port = port)
        @test st == 200
        @test occursin("\"stage\":\"complete\"", prog)
        @test occursin("\"done\":true", prog)

        # --- the staleness guard has to be told about new controls ----------
        #
        # The builder compares the controls it offers (its `NEEDS` list)
        # against what the server says it understands (`features`) and warns
        # when the running process is older than the page. That guard only
        # works if the two lists are kept in step, and NOTHING checked that
        # they were — so adding a control and forgetting the manifest raised
        # a red banner on a server that was in fact perfectly current.
        #
        # Parsed out of the page rather than restated here: a copy of the list
        # in this file would drift from the page exactly as the manifest did.
        st, hdrs, cat = http("GET", "/api/catalogue"; port = port)
        st, hdrs, page = http("GET", "/build"; port = port)
        m = match(r"const NEEDS = \[(.*?)\];"s, page)
        @test m !== nothing
        needs = [String(x.captures[1]) for x in eachmatch(r"'([^']+)'", m.captures[1])]
        @test length(needs) >= 7
        for f in needs
            @test occursin("\"$f\"", cat)
        end

        # --- Earth-orbit missions -------------------------------------------
        bod = cat
        @test occursin("\"orbits\"", bod)
        @test occursin("molniya", bod)
        st, hdrs, bod = http("POST", "/api/run";
                             port = port, timeout = 180.0,
                             body = "mode=orbit&orbit=leo")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"mode\":\"orbit\"", bod)
        @test occursin("\"outcome\":\"nominal\"", bod)
        @test occursin("\"orbit_rp_km\"", bod)
        @test occursin("\"cis\":", bod)          # the coast, in the shared shape
        st, hdrs, bod = http("POST", "/api/run";
                             port = port, timeout = 180.0,
                             body = "mode=orbit&orbit=molniya")
        @test occursin("\"on_target\":true", bod)
        @test occursin("raise_ignition", bod)
        @test occursin("shape_cutoff", bod)
        st, hdrs, bod = http("POST", "/api/run";
                             port = port, timeout = 240.0,
                             body = "mode=orbit&orbit=leo&deorbit=1&n_orbits=2")
        @test occursin("\"outcome\":\"nominal\"", bod)
        @test occursin("deorbit_ignition", bod)
        @test occursin("entry_handoff", bod)
        @test occursin("splashdown", bod)

        # --- method handling ------------------------------------------------
        st, hdrs, bod = http("HEAD", "/api/catalogue"; port = port)
        @test st == 200
        @test isempty(bod)                       # HEAD carries no body ...
        @test haskey(hdrs, "content-length")     # ... but does say how long
        @test parse(Int, hdrs["content-length"]) > 0

        st, hdrs, bod = http("OPTIONS", "/api/run"; port = port)
        @test st == 204

        st, hdrs, bod = http("GET", "/no/such/route"; port = port)
        @test st == 404
        @test occursin("\"ok\":false", bod)
        @test hdrs["content-type"] == "application/json"

        # --- shared ES modules ----------------------------------------------
        # The three pages import their formatting, API and vehicle-storage
        # helpers from here instead of each keeping a copy. A module served
        # with the wrong media type is refused by the browser outright, and
        # the console blames CORS, so the content type is asserted.
        for name in ("fmt.js", "api.js", "vehicle.js", "selftest.js",
                     "charts.js", "groundtrack.js", "moongroundtrack.js", "metrics.js",
                     "lander_model.js", "engine_layout.js", "engine_model.js",
                     "celestial.js", "flight_state.js")
            st, hdrs, bod = http("GET", "/static/$name"; port = port)
            @test st == 200
            @test hdrs["content-type"] == "text/javascript; charset=utf-8"
            @test occursin("export", bod)
        end
        st, hdrs, bod = http("GET", "/static/ne_110m_admin_0_countries.geojson";
                             port = port)
        @test st == 200
        @test hdrs["content-type"] == "application/geo+json; charset=utf-8"
        @test occursin("FeatureCollection", bod)

        # The lunar map must survive the desktop server as bytes, including
        # bytes that are not UTF-8. HEAD advertises the same size without a body.
        image = read(joinpath(PanelApp.STATIC_DIR[], "moon_map.jpg"))
        st, hdrs, bod = http("GET", "/static/moon_map.jpg?v=2"; port = port)
        @test st == 200
        @test hdrs["content-type"] == "image/jpeg"
        @test parse(Int, hdrs["content-length"]) == length(image)
        @test collect(codeunits(bod)) == image
        st, hdrs, bod = http("HEAD", "/static/moon_map.jpg"; port = port)
        @test st == 200
        @test hdrs["content-type"] == "image/jpeg"
        @test parse(Int, hdrs["content-length"]) == length(image)
        @test isempty(bod)

        # `limit` is the only thing that colours a metric anywhere in the app,
        # and `drawLine` the only thing that draws a chart. Both were private
        # to mission control until the analysis page needed them; a second copy
        # of either is the `fmt` mistake again.
        st, hdrs, bod = http("GET", "/static/metrics.js"; port = port)
        @test occursin("export function limit", bod)
        @test occursin("prop_margin_kg", bod)
        st, hdrs, bod = http("GET", "/static/charts.js"; port = port)
        @test occursin("export function drawLine", bod)
        for capability in ("pointerdown", "zoomView", "Export plotted data",
                           "text/csv", "toBlob", "_view")
            @test occursin(capability, bod)
        end

        # The console language. A stylesheet served as the wrong media type is
        # dropped as silently as a module is, and the page still renders —
        # just unstyled — so the type is worth asserting.
        st, hdrs, bod = http("GET", "/static/tokens.css"; port = port)
        @test st == 200
        @test hdrs["content-type"] == "text/css; charset=utf-8"
        # the reserved-meaning colours and the verdict component are what the
        # other pages import this file for
        @test occursin("--amber", bod)
        @test occursin("--nominal", bod)
        @test occursin("--failed", bod)
        @test occursin(".verdict", bod)
        @test occursin(".mgroups", bod)
        @test occursin(".appnav", bod)

        # `fin` is the guard three pages got wrong independently; if this
        # module ever stops exporting it, every one of them breaks at once,
        # which is the trade this extraction makes.
        st, hdrs, bod = http("GET", "/static/fmt.js"; port = port)
        @test occursin("export const fin", bod)
        st, hdrs, bod = http("GET", "/static/vehicle.js"; port = port)
        @test occursin("ssjl.vehicle", bod)

        # A query string is how a cache-buster would arrive; it must not
        # defeat the name match.
        st, hdrs, bod = http("GET", "/static/fmt.js?v=2"; port = port)
        @test st == 200

        # The name is whitelisted rather than sanitised, so every one of these
        # fails on the same rule — no separator can be expressed at all. This
        # process reads the user's own filesystem; loopback-only is not a
        # reason to hand out arbitrary files.
        for bad in ("../panelapp.jl", "..%2Fpanelapp.jl", "..\\panelapp.jl",
                    "sub/dir.js", "fmt.js.bak", "nope.js", "C:/Windows/win.ini",
                    "../Project.toml", "tokens.css.bak", "tokens.scss",
                    "../static/moon_map.jpg", "moon_map.jpg.bak", "sub/moon_map.jpg")
            st, hdrs, bod = http("GET", "/static/$bad"; port = port)
            @test st == 404
            @test occursin("\"ok\":false", bod)
            @test !occursin("PanelApp", bod)     # never the file's contents
        end

        # --- the liveness signal ---------------------------------------------
        # v0.3.0 shipped a desktop launcher that treated "the browser process
        # we spawned exited" as "the user closed the window". Those are not the
        # same statement — Edge exits early when it hands the URL to a copy of
        # itself — and the server was torn down under a window that had just
        # opened, giving ERR_CONNECTION_REFUSED. Traffic is the honest signal,
        # so the launcher now waits on this instead.
        before = PanelApp.LAST_REQUEST[]
        @test before > 0                       # every request above set it
        sleep(0.05)
        st, hdrs, bod = http("GET", "/api/health"; port = port)
        @test st == 200
        @test PanelApp.LAST_REQUEST[] > before  # and it moves

        # --- health ---------------------------------------------------------
        st, hdrs, bod = http("GET", "/api/health"; port = port)
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("uptime_s", bod)
        @test occursin("threads", bod)

        # --- both IP stacks -------------------------------------------------
        # `localhost` resolves to ::1 first on Windows 11 and on most modern
        # Linux. Binding IPv4 only makes every new connection pay for a
        # failed attempt before falling back.
        st, hdrs, bod = http("GET", "/api/health"; port = port, host = ip"127.0.0.1")
        @test st == 200
        # start_panel's IPv6 bind is best-effort — a host without IPv6 still
        # works — so this only holds when a second listener actually came up.
        if length(srv.listeners) > 1
            st, hdrs, bod = http("GET", "/api/health"; port = port, host = ip"::1")
            @test st == 200
        end

        # Production/container binding is explicit and reachable through the
        # loopback address even though the socket itself listens on 0.0.0.0.
        public_port = free_port()
        public_srv = PanelApp.start_panel(public_port; public = true)
        try
            @test length(public_srv.listeners) == 1
            st, _, bod = http("GET", "/api/health"; port = public_port,
                              host = ip"127.0.0.1")
            @test st == 200
            @test occursin("\"ok\":true", bod)
        finally
            PanelApp.stop_panel(public_srv)
        end

        # --- a slow request must not block the whole server -----------------
        # Guarded: with one thread there is nothing to interleave with, and
        # CI runs the suite single-threaded.
        if Threads.nthreads() > 1
            slow = @async http("POST", "/api/run"; port = port,
                               body = "mode=flyby&pod_mass=350", timeout = 180.0)
            yield()
            t0 = time()
            st, hdrs, bod = http("GET", "/api/health"; port = port, timeout = 30.0)
            dt = time() - t0
            @test st == 200
            @test dt < 3.0        # answered while the mission is still flying
            st_slow, _, bod_slow = fetch(slow)
            @test st_slow == 200
            @test occursin("\"ok\":true", bod_slow)
        end

        # --- concurrent runs agree ------------------------------------------
        # panel_mission is called from several threads at once by run_sweep
        # and now by the server too; two identical requests must agree.
        a = @async http("POST", "/api/run"; port = port,
                        body = "mode=flyby&pod_mass=350", timeout = 180.0)
        b = @async http("POST", "/api/run"; port = port,
                        body = "mode=flyby&pod_mass=350", timeout = 180.0)
        (_, _, ba) = fetch(a)
        (_, _, bb) = fetch(b)
        @test occursin("\"ok\":true", ba)
        # Every run now carries its own id and the wall-clock time it was
        # filed, so two identical requests are no longer byte-identical and
        # must not be asserted to be. What has to agree is the FLIGHT — strip
        # the two fields that are deliberately unique and compare the rest,
        # which is the trajectory, the metrics and the events.
        drop_ids(s) = replace(s, r"\"(id|at)\":(\"[^\"]*\"|[-0-9.eE+]+),?" => "")
        @test drop_ids(ba) == drop_ids(bb)
        @test ba != bb                       # ...and the ids really are distinct
        @test occursin(r"\"id\":\"r\d+\"", ba) && occursin(r"\"id\":\"r\d+\"", bb)

        # --- suborbital over the wire ---------------------------------------
        # A suborbital run carries no cislunar leg at all, so the payload has
        # to be the ascent-plus-entry shape every consumer already handles —
        # and the two profiles have to close on the two different targets.
        num(b, k) = (m = match(Regex("\"" * k * "\":\\s*([-0-9.eE+]+)"), b);
                     m === nothing ? NaN : parse(Float64, m[1]))
        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = "mode=suborbital&sub_profile=hop&" *
                                    "sub_apogee_km=110&pod_mass=350")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"mode\":\"suborbital\"", bod)
        @test occursin("\"outcome\":\"nominal\"", bod)
        @test occursin("\"on_target\":true", bod)
        @test !occursin("\"cis\":", bod)                # no cislunar leg
        @test occursin("\"asc3d\"", bod) && occursin("\"ent3d\"", bod)
        @test abs(num(bod, "apogee_km") - 110.0) < 5.0  # closed on the ask
        @test num(bod, "range_km") < 15.0               # and came down at home
        @test num(bod, "cutoff_h_km") < num(bod, "apogee_km")
        @test num(bod, "v_splash") < 12.0
        # the viewers frame their ENTRY phase on this event, and the arc starts
        # below the interface going UP so the entry simulator never emits it
        @test occursin("\"entry_interface\"", bod)
        @test occursin("\"apogee\"", bod)

        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = "mode=suborbital&sub_profile=downrange&" *
                                    "sub_range_km=500&pod_mass=350")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"on_target\":true", bod)
        @test abs(num(bod, "range_km") - 500.0) < 25.0
        @test num(bod, "apogee_km") > 40.0              # a shot is still lofted

        # --- a heavy capsule has to come home too -----------------------------
        # The entry pod used to be built with the mass alone, so every capsule
        # re-entered behind a fixed 1.5 m heat shield no matter how heavy it
        # was. A 45 t Apollo stack therefore could not decelerate: it skipped
        # back out, the integrator ran its full t_max, and the splash fields
        # stayed NaN — served as null under an "outcome":"nominal" that the
        # launch page then died reading.
        #
        # Both halves are asserted here: the flight arrives, and the outcome
        # and the metrics agree that it arrived.
        satv = "vname=Saturn+V&nstages=3&diameter=10.1&pod_mass=45000&kick_deg=1" *
               "&opt_kick=1&fairing_on=0&s1_engine=f1&s1_engines=5&s1_prop=2077000" *
               "&s1_dry=130000&s1_diameter=10.1&s2_engine=j2&s2_engines=5" *
               "&s2_prop=451000&s2_dry=36000&s2_diameter=10.1&s3_engine=j2" *
               "&s3_engines=1&s3_prop=106600&s3_dry=13500&s3_diameter=6.6&mode=flyby"
        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = satv)
        @test st == 200
        @test occursin("\"ok\":true", bod)
        # stages are named after the vehicle, not after the reference one:
        # the launch view spells `sep_saturn_v1` "SATURN V1 SEPARATION", and it
        # used to announce a Saturn V staging as "SABLE1 SEPARATION"
        @test occursin("sep_saturn_v1", bod)
        @test !occursin("sable", bod)
        @test occursin("\"entry_interface\"", bod)
        @test occursin("\"splashdown\"", bod)
        @test occursin("\"outcome\":\"nominal\"", bod)
        # nominal and a null splashdown metric must never co-occur again
        @test !occursin("\"t_days\":null", bod)
        @test !occursin("\"splash_lat\":null", bod)
        @test !occursin("\"v_splash\":null", bod)
        # and it is a real lunar-return entry, not a graze: Apollo peaked near
        # 6 g, and the skipping-out failure showed up as a peak under 1
        @test 4.0 < num(bod, "peak_g") < 9.0
        @test num(bod, "perilune_km") > 1000.0

        # the same invariant from the other side, on the reference vehicle
        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = "mode=flyby&pod_mass=350")
        @test occursin("\"outcome\":\"nominal\"", bod)
        @test occursin("\"splashdown\"", bod)
        @test occursin("\"rcs\":", bod)
        @test num(bod, "rcs_used_kg") > 0
        @test num(bod, "rcs_margin_kg") > 0
        for field in ("alpha", "qrate", "qbar", "mach", "heat", "twall")
            @test occursin("\"$field\":", bod)
        end
        @test !occursin("\"t_days\":null", bod)
        @test isfinite(num(bod, "t_days")) && num(bod, "t_days") > 1.0
        @test 4.0 < num(bod, "peak_g") < 9.0

        # --- the payload is a spacecraft plus its cargo -----------------------
        #
        # It was one number doing three jobs: what the launcher lifts, what
        # re-enters, and what the capsule's size is fitted from. One number
        # cannot be a spacecraft AND its cargo, which is why the reference
        # vehicle sat at a "350 kg capsule" — lighter than anything anyone has
        # ever flown a person in.
        #
        # A bare `pod_mass` still has to fly exactly as it did, because saved
        # runs and hand-written URLs carry it.
        legacy = http("POST", "/api/run"; port = port, timeout = 300.0,
                      body = "mode=flyby&pod_mass=350")[3]
        split_ = http("POST", "/api/run"; port = port, timeout = 300.0,
                      body = "mode=flyby&payload_kind=bus&bus_mass=200&cargo_mass=150")[3]
        @test isapprox(num(legacy, "liftoff_t"), num(split_, "liftoff_t"); rtol = 1e-9)

        # cargo is really carried: drop it and the stack gets lighter
        nocargo = http("POST", "/api/run"; port = port, timeout = 300.0,
                       body = "mode=flyby&payload_kind=bus&bus_mass=200&cargo_mass=0")[3]
        @test num(nocargo, "liftoff_t") < num(split_, "liftoff_t")
        # ...and it re-enters INSIDE the spacecraft's envelope rather than
        # widening it, so it raises the ballistic coefficient and the heating
        @test num(nocargo, "peak_q_wcm2") < num(split_, "peak_q_wcm2")

        # the geometry endpoint reports the composition, not just the lump
        st, hdrs, bod = http("POST", "/api/geometry"; port = port,
            body = "payload_kind=bus&bus_mass=200&cargo_mass=150")
        @test st == 200
        @test isapprox(num(bod, "spacecraft_kg"), 200.0; rtol = 1e-9)
        @test isapprox(num(bod, "cargo_kg"), 150.0; rtol = 1e-9)
        @test isapprox(num(bod, "mass_kg"), 350.0; rtol = 1e-9)
        @test occursin("\"kind\":\"bus\"", bod)
        # a bus has nobody in it whatever the crewed box says
        st, hdrs, bod = http("POST", "/api/geometry"; port = port,
            body = "payload_kind=bus&bus_mass=200&crewed=1")
        @test occursin("\"crewed\":false", bod)

        # --- the attitude budget answers to the vehicle -----------------------
        # It used to be bit-identical for every stack: inertia was pinned at
        # 1000 kg m^2 and the hardware at one fixed 12 kg tank, so the only
        # input that reached it was how long the coast lasted.
        heavy = http("POST", "/api/run"; port = port, timeout = 300.0,
                     body = "mode=flyby&payload_kind=bus&bus_mass=200&cargo_mass=40")[3]
        @test num(heavy, "rcs_used_kg") != num(split_, "rcs_used_kg")

        # --- a design that never converged is not a flyby ---------------------
        # The corrector emits "free-return design stalled" and "did not fully
        # converge" with a perilune residual of ~249,000 km, then returns the
        # last iterate anyway. That trajectory propagates, reaches entry
        # interface and splashes down — so it used to be served as
        # "outcome":"nominal" with a perilune a quarter of a million km from
        # the Moon, and the launch view narrated it as a flyby.
        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = "mode=flyby&nstages=3&nboost=4&pod_mass=900" *
                                    "&s3_prop=1900&s3_dry=250")
        @test st == 200
        @test occursin("\"ok\":true", bod)             # still a result, not an error
        @test occursin("\"outcome\":\"design_failed\"", bod)
        @test !occursin("\"outcome\":\"nominal\"", bod)
        @test occursin("\"design_status\":\"stalled\"", bod)
        @test occursin("\"flyby\":false", bod)
        @test !occursin("\"name\":\"perilune\"", bod)
        @test !occursin("\"splashdown\"", bod)         # nothing past the parking orbit
        @test occursin("\"asc3d\"", bod)               # the ascent that DID fly is served

        # and the converged reference says so, on the same field
        st, hdrs, bod = http("POST", "/api/run"; port = port, timeout = 300.0,
                             body = "mode=flyby&pod_mass=350")
        @test occursin("\"outcome\":\"nominal\"", bod)
        @test !occursin("\"design_status\":\"stalled\"", bod)
        # :outside_tolerance is ROUTINE — the reference lands a couple of
        # hundred metres off a 250 m band — and must never be treated as failure
        @test occursin("\"design_status\":\"converged\"", bod) ||
              occursin("\"design_status\":\"outside_tolerance\"", bod)

        # --- the last-resort 500 --------------------------------------------
        # `route`'s own catch is the first of the two "answer it whatever
        # happens" layers, and nothing exercised it: every shipped page file
        # exists, so `read(PAGE_PATH[], String)` never threw. Pointing the
        # Ref at a file that is not there does, and the response still has to
        # be a real, parseable body — not a closed socket.
        orig_page = PanelApp.PAGE_PATH[]
        PanelApp.PAGE_PATH[] = joinpath(@__DIR__, "no-such-panel-page.html")
        try
            st, hdrs, bod = http("GET", "/"; port = port)
            @test st == 500
            @test !isempty(bod)
            @test occursin("\"ok\":false", bod)
        finally
            PanelApp.PAGE_PATH[] = orig_page
        end
    finally
        PanelApp.stop_panel(srv)
    end
end
