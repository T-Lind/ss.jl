# HTTP-level tests for the mission-control panel.
#
# The invariant this file exists to protect: *no request ever gets an empty
# reply*. A socket closed without a response is indistinguishable, in a
# browser, from the server being down — it surfaces as "Failed to fetch" with
# nothing in the log to explain it.

using Test
using Sockets

include(joinpath(@__DIR__, "..", "scripts", "panelapp.jl"))

"A free TCP port on the loopback interface."
function free_port()
    s = listen(IPv4(127, 0, 0, 1), 0)
    p = Int(getsockname(s)[2])
    close(s)
    p
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
        st, hdrs, bod = http("GET", "/api/health"; port = port, host = ip"::1")
        @test st == 200

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
    finally
        PanelApp.stop_panel(srv)
    end
end
