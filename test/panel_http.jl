# HTTP-level tests for the mission-control panel.
#
# The invariant this file exists to protect: *no request ever gets an empty
# reply*. A socket closed without a response is indistinguishable, in a
# browser, from the server being down — it surfaces as "Failed to fetch" with
# nothing in the log to explain it.

using Test
using Sockets

include(joinpath(@__DIR__, "..", "scripts", "panelapp.jl"))

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
                             body = "mode=flyby&nstages=2&s1_prop=3000&s1_dry=2500&" *
                                    "s2_prop=100&s2_dry=140")
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("\"outcome\":\"ascent_failed\"", bod)
        @test occursin("\"ascent\":", bod)       # the leg that DID fly
        @test occursin("\"launch_lat\"", bod)    # the launch page's hard needs
        @test occursin("\"liftoff_t\"", bod)
        @test !occursin("\"cis\":", bod)         # and no leg it did not fly

        # --- Earth-orbit missions -------------------------------------------
        st, hdrs, bod = http("GET", "/api/catalogue"; port = port)
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
        @test ba == bb

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
