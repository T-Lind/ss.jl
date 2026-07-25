# Phase 0 — Panel Server Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `scripts/panel.jl` answer every request it receives, on both IP stacks, without blocking its own accept loop — and prove it with an automated HTTP test suite.

**Architecture:** Split the panel script into a `PanelApp` module (all logic, no side effects) plus a six-line entry point, so a test can start a server on an ephemeral port. Restructure request handling into `read_request` → `route` → `write_response`, each layered so that no code path can close a socket without writing a response. Add dual-stack loopback listeners, per-request logging, and a health endpoint.

**Tech Stack:** Julia 1.9+, stdlib only (`Sockets`, `Printf`, `Test`). No new dependencies — the repo is deliberately dependency-free.

## Global Constraints

- **Julia compat floor is 1.9** (`Project.toml:[compat] julia = "1.9"`), and CI runs the matrix `['1.9', '1']`. Do not use language or stdlib features newer than 1.9.
- **Standard library only.** No new entries in `[deps]` beyond Julia stdlibs. The README's opening claim is "pure Julia (standard library only — no package dependencies)".
- **CI runs the test suite single-threaded.** `julia-actions/julia-runtest@v1` does not set `--threads`. Any test that depends on real parallelism must be guarded with `Threads.nthreads() > 1` and skipped otherwise.
- **Port 8137 is the user's live session.** Never bind it in tests or examples; tests must use a discovered free port.
- **Preserve existing response semantics:** a mission that fails to *design* still returns HTTP 200 with `{"ok": false, "error": ...}`, because `scripts/panel_page.html:754` and `scripts/launch_page.html:3441` both read `j.ok` from a parsed body. HTTP 500 is reserved for genuinely unexpected server faults.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `scripts/panelapp.jl` | Create | `module PanelApp` — every function currently in `panel.jl`, plus the HTTP layer and `start_panel`/`stop_panel`/`main`. No top-level side effects. |
| `scripts/panel.jl` | Rewrite | Six-line entry point: include the module, call `main(port)`. |
| `test/panel_http.jl` | Create | A minimal stdlib HTTP client and the `@testset "panel http"` suite. |
| `test/runtests.jl` | Modify (append) | `include("panel_http.jl")` after the existing outer testset closes. |
| `Project.toml` | Modify | Add `Sockets` to `[extras]` and the `test` target — the test suite needs it and stdlibs used by tests must be declared for `Pkg.test()`. |

The split is an enabling change: `scripts/panel.jl` currently ends with top-level statements that warm up and then `listen` forever (`scripts/panel.jl:833-843`), so it cannot be loaded by a test without hanging. At 843 lines and about to grow (Phase 1 adds an Earth-orbit mission chain, Phase 3 a vehicle page route), splitting it is warranted on its own.

---

### Task 1: Make the panel loadable and startable from a test

**Files:**
- Create: `scripts/panelapp.jl`
- Rewrite: `scripts/panel.jl`
- Create: `test/panel_http.jl`
- Modify: `test/runtests.jl` (append one line after the final `end`)
- Modify: `Project.toml:[extras]` and `Project.toml:[targets]`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - `PanelApp.start_panel(port::Int) -> PanelApp.PanelServer`
  - `PanelApp.stop_panel(s::PanelApp.PanelServer) -> Nothing`
  - `PanelApp.main(port::Int = 8137)` — warms up, starts, blocks forever
  - `PanelApp.PanelServer` with fields `listeners::Vector{Sockets.TCPServer}`, `acceptors::Vector{Task}`, `port::Int`
  - In `test/panel_http.jl`: `free_port() -> Int` and
    `http(method, path; port, host = Sockets.localhost, body = "", timeout = 60.0) -> (status::Int, headers::Dict{String,String}, body::String)`

- [ ] **Step 1: Write the failing test**

Create `test/panel_http.jl`:

```julia
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
    finally
        PanelApp.stop_panel(srv)
    end
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: FAIL. `scripts/panelapp.jl` does not exist, so the `include` raises
`SystemError: opening file ... panelapp.jl`.

- [ ] **Step 3: Create the module**

Create `scripts/panelapp.jl`. Start it with this header and module opening:

```julia
# Local mission-control panel: configure, run, and explore missions in the
# browser. Pure stdlib (raw Sockets HTTP) — no package dependencies.
#
# This file defines the panel and starts nothing. `scripts/panel.jl` is the
# entry point that runs it, and `test/panel_http.jl` starts one on an
# ephemeral port — which is only possible because loading this file has no
# side effects.

module PanelApp

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Sockets
using Printf

const PAGE_PATH = joinpath(@__DIR__, "panel_page.html")
const LAUNCH_PATH = joinpath(@__DIR__, "launch_page.html")
```

Then move **lines 20 through 742 of `scripts/panel.jl` verbatim** — from the
`# ---------------------------------------------------------------- helpers --`
banner through the closing `end` of `run_sweep`. Do not edit them; this step is
a move, and any behaviour change here would be invisible to the tests written
so far.

Note what is deliberately *not* carried over: `const PORT = length(ARGS) >= 1 ?
...` (line 18) reads `ARGS` at load time and moves into `main` instead.

Then append the HTTP layer. Keep `respond` and `handle` exactly as they are in
`scripts/panel.jl:746-831` for now — Task 2 rewrites them, and changing them
here would mean this task's test cannot tell a successful move from a lucky
rewrite. After them, add:

```julia
# ----------------------------------------------------------------- server --

"A running panel: its listeners, their accept loops, and the port they share."
struct PanelServer
    listeners::Vector{Sockets.TCPServer}
    acceptors::Vector{Task}
    port::Int
end

"""
    start_panel(port) -> PanelServer

Bind and start accepting. Returns immediately; the accept loops run as tasks.
"""
function start_panel(port::Int)
    listeners = Sockets.TCPServer[listen(IPv4(127, 0, 0, 1), port)]
    acceptors = Task[]
    for l in listeners
        push!(acceptors, @async begin
            while isopen(l)
                sock = try
                    accept(l)
                catch
                    break          # listener closed: the loop is done
                end
                @async handle(sock)
            end
        end)
    end
    PanelServer(listeners, acceptors, port)
end

"Close every listener; in-flight requests finish on their own."
function stop_panel(s::PanelServer)
    foreach(close, s.listeners)
    nothing
end

"""
    main(port)

What `scripts/panel.jl` calls: warm the stack up so the first browser request
is not also the first compile, then serve until interrupted.
"""
function main(port::Int = 8137)
    println("warming up (first mission run compiles the stack)...")
    t0 = time()
    panel_mission(Dict{String,String}())
    @printf("ready in %.1f s — panel at http://localhost:%d  (Ctrl-C to stop)\n",
            time() - t0, port)
    srv = start_panel(port)
    wait(srv.acceptors[1])
    srv
end

end # module
```

- [ ] **Step 4: Rewrite the entry point**

Replace the entire contents of `scripts/panel.jl` with:

```julia
# Local mission-control panel: configure, run, and explore missions in the
# browser. Pure stdlib (raw Sockets HTTP) — no package dependencies.
#
#   julia --project -t auto scripts/panel.jl [port]
#
# then open http://localhost:8137 (default port). The page posts form-encoded
# parameters to /api/run and /api/sweep; the server executes the full mission
# design + flight (~1 s per run after warmup) and returns JSON with metrics,
# decimated trajectories, and events.
#
# All of the logic lives in `panelapp.jl` as a module, so the test suite can
# start a server without running this script.

include(joinpath(@__DIR__, "panelapp.jl"))

PanelApp.main(length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 8137)
```

- [ ] **Step 5: Declare Sockets for the test environment**

In `Project.toml`, change the `[extras]` and `[targets]` sections to:

```toml
[extras]
Sockets = "6462fe0b-24de-5631-8697-dd941f90decc"
Test = "8dfed614-e22c-5e08-85e1-65c5234f0b40"

[targets]
test = ["Test", "Sockets"]
```

- [ ] **Step 6: Hook the suite up**

In `test/runtests.jl`, after the final `end` that closes
`@testset "SatelliteSim"`, append:

```julia

# The panel is a script rather than part of the package, but it is the primary
# way this simulator gets used, and its HTTP layer has its own failure modes.
include("panel_http.jl")
```

- [ ] **Step 7: Run the test to verify it passes**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS, including the new `panel http` testset. The whole suite should
still pass — this task changed no behaviour.

- [ ] **Step 8: Verify the entry point still works by hand**

Run: `julia --project -t auto scripts/panel.jl 8138`

Expected: prints `warming up ...` then `ready in N.N s — panel at
http://localhost:8138`. In another shell,
`curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8138/api/catalogue`
prints `200`. Stop it with Ctrl-C.

- [ ] **Step 9: Commit**

```bash
git add scripts/panelapp.jl scripts/panel.jl test/panel_http.jl test/runtests.jl Project.toml
git commit -m "Make the panel a module, so its HTTP layer can be tested

scripts/panel.jl ended with top-level statements that warmed up and then
listened forever, so nothing could load it without hanging. The logic moves
verbatim into a PanelApp module and the script becomes a six-line entry
point; start_panel/stop_panel let a test bind an ephemeral port.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Never close a socket without answering

**Files:**
- Modify: `scripts/panelapp.jl` (replace `respond` and `handle`; add `read_request`, `write_response`, `safe_call`, `route`)
- Modify: `test/panel_http.jl` (add cases to `@testset "panel http"`)

**Interfaces:**
- Consumes: `PanelApp.start_panel`, `PanelApp.stop_panel`, `http`, `free_port` from Task 1.
- Produces:
  - `PanelApp.read_request(sock) -> Union{Nothing,Tuple{String,String,String}}` — `(method, path, body)`, or `nothing` when the peer closed before sending anything
  - `PanelApp.write_response(sock, status::AbstractString, ctype::AbstractString, body::AbstractString; head::Bool = false) -> Nothing`
  - `PanelApp.safe_call(f, body::AbstractString) -> Dict{String,Any}`
  - `PanelApp.route(method, path, body) -> Tuple{String,String,String}` — `(status, content_type, payload)`; never throws

The defect: in `scripts/panel.jl:774-781` (now in `panelapp.jl`), `parse_form(body)`
and `json(out)` sit *outside* the inner `try`. Anything they throw reaches the
bare `catch` in `handle`, which closes the socket having written nothing.
Confirmed reachable — `POST /api/run` with body `pod_mass=35%ZZ` makes
`urldecode` call `parse(UInt8, "ZZ"; base = 16)`, and `curl` reports
`Empty reply from server`.

- [ ] **Step 1: Write the failing tests**

In `test/panel_http.jl`, inside the `try` block of `@testset "panel http"`, after
the catalogue assertions, add:

```julia
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: FAIL fast (not a timeout) on the `pod_mass=35%ZZ` case. The server
closes the socket having written nothing, so `read(sock, String)` returns `""`
immediately; `split_response("")` yields an empty header block, and
`split(lines[1], ' ')[2]` raises `BoundsError: attempt to access 1-element
Vector{SubString{String}} at index [2]`.

That BoundsError *is* the bug's signature — an empty reply — but it is a poor
error message to hit repeatedly. Leave the client as written: once Task 2's
fix lands it never triggers again, and making the client tolerate empty
replies would blunt the very test that catches them.

- [ ] **Step 3: Replace the HTTP layer**

In `scripts/panelapp.jl`, delete `respond` and `handle` entirely and put this in
their place (keeping the `# ------ http loop --` banner):

```julia
"""
Read one request. Returns `(method, path, body)`, or `nothing` if the peer
closed before sending anything — a browser opening and dropping a speculative
connection is not an error worth answering.

Every parse here is total: a `Content-Length` that is not a number is treated
as absent rather than thrown, because throwing at this point is what leaves a
socket unanswered.
"""
function read_request(sock)
    reqline = readline(sock)
    isempty(reqline) && return nothing
    parts = split(reqline, ' ')
    length(parts) < 2 && return ("BAD", "/", "")
    method, path = String(parts[1]), String(parts[2])
    clen = 0
    while true
        h = readline(sock)
        (isempty(h) || h == "\r") && break
        if startswith(lowercase(h), "content-length:")
            kv = split(h, ':'; limit = 2)
            clen = something(tryparse(Int, strip(kv[2])), 0)
        end
    end
    body = clen > 0 ? String(read(sock, clen)) : ""
    (method, path, body)
end

"Write a complete response. `head` suppresses the body but keeps the headers."
function write_response(sock, status::AbstractString, ctype::AbstractString,
                        body::AbstractString; head::Bool = false)
    write(sock, "HTTP/1.1 $status\r\nContent-Type: $ctype\r\n" *
                "Content-Length: $(sizeof(body))\r\nConnection: close\r\n\r\n")
    head || write(sock, body)
    nothing
end

"""
Parse the form and call `f` on it, turning any failure into a result.

This is where the panel's error contract lives: a mission that cannot be
designed, or a field the user mistyped, is a *200 with `ok: false`* — the
pages read `j.ok` from a parsed body, so an HTTP error code would give them
nothing to show. Genuine server faults are the 500 in `handle`.
"""
function safe_call(f, body::AbstractString)
    try
        f(parse_form(body))
    catch err
        Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))
    end
end

"""
    route(method, path, body) -> (status, content_type, payload)

Total function: every input produces a response, including an unknown route
and an unreadable page file.
"""
function route(method::AbstractString, path::AbstractString,
               body::AbstractString)
    try
        if method in ("GET", "HEAD") && (path == "/" || startswith(path, "/?"))
            # re-read per request so page edits show on refresh (dev-friendly)
            return ("200 OK", "text/html; charset=utf-8", read(PAGE_PATH, String))
        elseif method in ("GET", "HEAD") &&
               (path == "/launch" || startswith(path, "/launch?"))
            return ("200 OK", "text/html; charset=utf-8", read(LAUNCH_PATH, String))
        elseif method in ("GET", "HEAD") && path == "/api/catalogue"
            return ("200 OK", "application/json", json(catalogue_payload()))
        elseif method == "POST" && path == "/api/run"
            return ("200 OK", "application/json", json(safe_call(panel_mission, body)))
        elseif method == "POST" && path == "/api/sweep"
            return ("200 OK", "application/json", json(safe_call(run_sweep, body)))
        elseif method == "POST" && path == "/api/geometry"
            return ("200 OK", "application/json", json(safe_call(rocket_geometry, body)))
        elseif method == "POST" && path == "/api/solve"
            return ("200 OK", "application/json", json(safe_call(run_solve, body)))
        end
        return ("404 Not Found", "application/json",
                json(Dict{String,Any}("ok" => false,
                                      "error" => "no route for $method $path")))
    catch err
        return ("500 Internal Server Error", "application/json",
                json(Dict{String,Any}("ok" => false,
                                      "error" => sprint(showerror, err))))
    end
end

"""
Serve one connection, and answer it whatever happens.

Two layers, because one is not enough: `route` turns any error below it into a
response, and the `catch` here is the last resort for a failure in `route`'s
own serialization or in the socket write.
"""
function handle(sock)
    try
        req = read_request(sock)
        req === nothing && return
        method, path, body = req
        status, ctype, payload = route(method, path, body)
        write_response(sock, status, ctype, payload; head = method == "HEAD")
    catch err
        try
            write_response(sock, "500 Internal Server Error", "application/json",
                           json(Dict{String,Any}("ok" => false,
                                                 "error" => sprint(showerror, err))))
        catch
            # the socket itself is gone; nothing left to say
        end
    finally
        close(sock)
    end
end
```

- [ ] **Step 4: Extract the catalogue payload**

`route` above calls `catalogue_payload()`, which does not exist yet — the
catalogue Dict is currently built inline in the old `handle`. Add it just above
`read_request`:

```julia
"""
What the page builds its dropdowns from, so the UI can never offer a
propellant or an engine the simulator does not have.
"""
catalogue_payload() = Dict{String,Any}(
    "propellants" => [Dict("name" => string(k), "bulk" => bulk_density(v))
                      for (k, v) in sort(collect(PROPELLANTS), by = first)],
    "engines" => [Dict("name" => string(k),
                       "thrust_kn" => v.thrust_vac / 1e3,
                       "isp_vac" => v.isp_vac, "isp_sl" => v.isp_sl,
                       "mass_kg" => v.mass,
                       "propellant" => string(v.prop.name))
                  for (k, v) in sort(collect(ENGINES), by = first)],
    "solve_metrics" => SOLVE_METRICS,
    "landing_metrics" => LANDING_METRICS,
    "max_stages" => 5)
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS, all of `panel http` included.

- [ ] **Step 6: Commit**

```bash
git add scripts/panelapp.jl test/panel_http.jl
git commit -m "Answer every request, including the ones that fail

parse_form and json sat outside the handler's try block, so a malformed
percent-escape closed the socket having written nothing — which a browser
reports as 'Failed to fetch' with no server-side trace. Requests now go
through read_request -> route -> write_response, with route total and a
last-resort 500 behind it. Content-Length parsing is tryparse, for the same
reason.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: HEAD, OPTIONS, and an honest 404

**Files:**
- Modify: `scripts/panelapp.jl` (`route`)
- Modify: `test/panel_http.jl`

**Interfaces:**
- Consumes: `PanelApp.route` from Task 2.
- Produces: no new names. `route` gains an `OPTIONS` branch returning
  `("204 No Content", "text/plain", "")`.

`curl -I http://127.0.0.1:8138/api/catalogue` currently reports
`Weird server reply` — the old handler 404'd HEAD and then wrote a body anyway.
Task 2 already routes HEAD alongside GET and suppresses the body in
`write_response`; this task adds OPTIONS and locks both down with tests.

- [ ] **Step 1: Write the failing tests**

In `test/panel_http.jl`, after the Task 2 cases, add:

```julia
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: FAIL on the OPTIONS case — `route` has no OPTIONS branch, so it falls
through to the 404 and `st == 404`, not `204`. The HEAD and 404 cases should
already pass from Task 2; if they do not, fix Task 2's code rather than working
around it here.

- [ ] **Step 3: Add the OPTIONS branch**

In `route`, immediately before the `return ("404 Not Found", ...)` line, insert:

```julia
        elseif method == "OPTIONS"
            # no CORS here — the panel is same-origin — but a bare 404 for a
            # preflight is a confusing thing to hand a browser
            return ("204 No Content", "text/plain", "")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS.

- [ ] **Step 5: Verify by hand**

Run: `julia --project -t auto scripts/panel.jl 8138` in one shell, then in another:

```bash
curl -sI http://127.0.0.1:8138/api/catalogue | head -3
curl -s -o /dev/null -w "%{http_code}\n" -X OPTIONS http://127.0.0.1:8138/api/run
```

Expected: the first prints `HTTP/1.1 200 OK` and a `Content-Length` with no
`Weird server reply` warning; the second prints `204`.

- [ ] **Step 6: Commit**

```bash
git add scripts/panelapp.jl test/panel_http.jl
git commit -m "Answer HEAD and OPTIONS instead of 404-ing them with a body

curl -I reported 'Weird server reply' because the old handler sent a 404
header and then wrote a body anyway.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: A health endpoint and a request log

**Files:**
- Modify: `scripts/panelapp.jl` (`route`, `handle`, add `health_payload`, add `START_TIME`)
- Modify: `test/panel_http.jl`

**Interfaces:**
- Consumes: `PanelApp.route`, `PanelApp.handle` from Task 2.
- Produces:
  - `PanelApp.health_payload() -> Dict{String,Any}` with keys `ok`, `uptime_s`, `julia`, `threads`
  - `PanelApp.START_TIME::Base.RefValue{Float64}` — a `Ref` so `start_panel` can
    reset it without the binding being non-`const`
  - `handle` gains a log line per request on stdout.

The reported "Failed to fetch" was not reproducible from a browser. This task
is the instrument that turns the next occurrence into a record instead of a
guess.

- [ ] **Step 1: Write the failing test**

In `test/panel_http.jl`, add inside the `try` block:

```julia
        # --- health ---------------------------------------------------------
        st, hdrs, bod = http("GET", "/api/health"; port = port)
        @test st == 200
        @test occursin("\"ok\":true", bod)
        @test occursin("uptime_s", bod)
        @test occursin("threads", bod)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: FAIL with `st == 404` — there is no `/api/health` route.

- [ ] **Step 3: Add the endpoint**

In `scripts/panelapp.jl`, just above `catalogue_payload`, add:

```julia
"When this server started, so /api/health can report how long it has been up."
const START_TIME = Ref(time())

"""
Liveness and environment, for a browser or a script that wants to know the
panel is actually there before blaming the network.
"""
health_payload() = Dict{String,Any}(
    "ok" => true,
    "uptime_s" => time() - START_TIME[],
    "julia" => string(VERSION),
    "threads" => Threads.nthreads())
```

In `route`, add a branch next to the catalogue one:

```julia
        elseif method in ("GET", "HEAD") && path == "/api/health"
            return ("200 OK", "application/json", json(health_payload()))
```

In `start_panel`, set the clock as the first statement of the function body:

```julia
    START_TIME[] = time()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS.

- [ ] **Step 5: Add the request log**

In `handle`, track the outcome and log it. Replace the function with:

```julia
function handle(sock)
    t0 = time()
    method, path, status, nbytes = "-", "-", "500", 0
    try
        req = read_request(sock)
        req === nothing && return
        method, path, body = req
        st, ctype, payload = route(method, path, body)
        status = first(st, 3)
        nbytes = sizeof(payload)
        write_response(sock, st, ctype, payload; head = method == "HEAD")
    catch err
        try
            payload = json(Dict{String,Any}("ok" => false,
                                            "error" => sprint(showerror, err)))
            nbytes = sizeof(payload)
            write_response(sock, "500 Internal Server Error",
                           "application/json", payload)
        catch
            # the socket itself is gone; nothing left to say
        end
    finally
        # One line per request. This is how an intermittent "Failed to fetch"
        # gets diagnosed from a record rather than from a hypothesis.
        @printf("[panel] %-7s %-44s %s %7.0f ms %9d B\n",
                method, first(path, 44), status, 1e3 * (time() - t0), nbytes)
        flush(stdout)
        close(sock)
    end
end
```

- [ ] **Step 6: Verify the log by hand**

Run: `julia --project -t auto scripts/panel.jl 8138` in one shell, then in another:

```bash
curl -s -o /dev/null http://127.0.0.1:8138/api/health
curl -s -o /dev/null http://127.0.0.1:8138/no/such/route
```

Expected: the server's shell shows two lines, e.g.

```
[panel] GET     /api/health                                  200       1 ms        84 B
[panel] GET     /no/such/route                               404       0 ms        52 B
```

- [ ] **Step 7: Run the full suite and commit**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS.

```bash
git add scripts/panelapp.jl test/panel_http.jl
git commit -m "Add /api/health and log every request

The reported 'Failed to fetch' did not reproduce under 40 sequential, 30
parallel and mixed bursts, so the next occurrence needs to leave evidence:
method, path, status, duration and size, one line each.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Listen on both IP stacks

**Files:**
- Modify: `scripts/panelapp.jl` (`start_panel`)
- Modify: `test/panel_http.jl`

**Interfaces:**
- Consumes: `PanelApp.start_panel` from Task 1.
- Produces: no new names. `start_panel` binds IPv6 loopback in addition to IPv4,
  best-effort.

Measured on Windows 11: `http://[::1]:8138` fails outright, and a connection via
`localhost` takes **0.209 s** against **0.002 s** for `127.0.0.1` — the cost of
an IPv6 attempt failing before the fallback. Chrome resolves `localhost` the
same way.

- [ ] **Step 1: Write the failing test**

In `test/panel_http.jl`, add inside the `try` block:

```julia
        # --- both IP stacks -------------------------------------------------
        # `localhost` resolves to ::1 first on Windows 11 and on most modern
        # Linux. Binding IPv4 only makes every new connection pay for a
        # failed attempt before falling back.
        st, hdrs, bod = http("GET", "/api/health"; port = port, host = ip"127.0.0.1")
        @test st == 200
        st, hdrs, bod = http("GET", "/api/health"; port = port, host = ip"::1")
        @test st == 200
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: FAIL on the `ip"::1"` case — `connect` raises
`IOError: connect: connection refused (ECONNREFUSED)` inside the client task,
surfacing as `TaskFailedException`.

- [ ] **Step 3: Bind both**

In `start_panel`, replace the listener construction with:

```julia
    listeners = Sockets.TCPServer[listen(IPv4(127, 0, 0, 1), port)]
    # `localhost` resolves to ::1 before 127.0.0.1 on Windows and on most
    # modern Linux, so an IPv4-only bind makes every new connection pay for a
    # failed attempt first. Best-effort: a host without IPv6 still works.
    try
        push!(listeners, listen(IPv6(0, 0, 0, 0, 0, 0, 0, 1), port))
    catch err
        @warn "IPv6 loopback unavailable; localhost falls back to IPv4" err
    end
```

The `for l in listeners` accept loop below it already handles however many
listeners there are, and `stop_panel` already closes all of them.

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS.

- [ ] **Step 5: Verify the latency by hand**

Run: `julia --project -t auto scripts/panel.jl 8138`, then:

```bash
curl -s -o /dev/null -w "localhost  %{time_total}s\n" http://localhost:8138/api/health
curl -s -o /dev/null -w "127.0.0.1  %{time_total}s\n" http://127.0.0.1:8138/api/health
curl -s -o /dev/null -w "[::1]      %{time_total}s\n" "http://[::1]:8138/api/health"
```

Expected: all three succeed, and `localhost` is now comparable to `127.0.0.1`
rather than ~200 ms slower.

- [ ] **Step 6: Commit**

```bash
git add scripts/panelapp.jl test/panel_http.jl
git commit -m "Listen on IPv6 loopback as well as IPv4

localhost resolves ::1 first on Windows 11, so an IPv4-only bind cost a
failed connection attempt on every socket: 0.209 s via localhost against
0.002 s via 127.0.0.1, measured.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Stop blocking the accept loop

**Files:**
- Modify: `scripts/panelapp.jl` (`start_panel`)
- Modify: `test/panel_http.jl`

**Interfaces:**
- Consumes: `PanelApp.start_panel` from Task 1, `PanelApp.health_payload` from Task 4.
- Produces: no new names. Connections are dispatched with `Threads.@spawn`
  instead of `@async`.

`@async` schedules on the calling thread and `panel_mission` never yields, so
`accept` cannot run while a mission is being flown. Six concurrent runs took
11.9 s — 2 s each, fully serialized. `run_sweep` already calls `panel_mission`
from `Threads.@threads`, so concurrent execution is not a new assumption; this
task tests it explicitly rather than inheriting it.

- [ ] **Step 1: Write the failing test**

In `test/panel_http.jl`, add inside the `try` block:

```julia
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
```

- [ ] **Step 2: Run tests to verify the state before the change**

Run: `julia --project -t auto -e 'using Pkg; Pkg.test(julia_args=["-t","4"])'`

Expected: FAIL on `@test dt < 3.0` — the health request queues behind the
mission and takes roughly as long as the run. The concurrency-agreement test
should already pass.

If `Pkg.test(julia_args=...)` is unavailable on the installed Julia, run the
file directly instead:
`julia --project -t 4 -e 'using SatelliteSim; include("test/panel_http.jl")'`

- [ ] **Step 3: Dispatch each connection to a thread**

In `start_panel`, change the inner dispatch from `@async handle(sock)` to
`Threads.@spawn handle(sock)`, and add the reason above it:

```julia
                # @async would schedule on this thread, and panel_mission
                # never yields — so a mission in flight stopped `accept` from
                # running at all. Six concurrent runs took 11.9 s, serialized.
                Threads.@spawn handle(sock)
```

Leave the outer `@async` on the accept loop itself: it is IO-bound and yields
in `accept`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `julia --project -e 'using Pkg; Pkg.test(julia_args=["-t","4"])'`

Expected: PASS, including the `dt < 3.0` assertion.

- [ ] **Step 5: Verify single-threaded operation still works**

Run: `julia --project -e 'using Pkg; Pkg.test()'`

Expected: PASS. The threaded assertions skip, everything else runs. This
confirms the change is safe on CI's default single thread.

- [ ] **Step 6: Verify the original measurement by hand**

Run: `julia --project -t auto scripts/panel.jl 8138`, then:

```bash
B="mode=flyby&vname=Sable&pod_mass=350&nstages=3&nboost=0&diameter=1.8"
time (for i in 1 2 3 4 5 6; do
  curl -s -o /dev/null -X POST --data "$B" http://127.0.0.1:8138/api/run &
done; wait)
```

Expected: substantially less than the 11.9 s measured before this task, on a
machine with several threads. Record the number in the commit message.

- [ ] **Step 7: Commit**

```bash
git add scripts/panelapp.jl test/panel_http.jl
git commit -m "Serve each connection on a thread, not on the accept loop

@async schedules on the calling thread and panel_mission never yields, so
accept could not run while a mission was in flight: six concurrent runs took
11.9 s, fully serialized, and every field edit fires /api/geometry.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Close out the phase

**Files:**
- Modify: `README.md` (the Running section — document `/api/health` and the request log)

**Interfaces:**
- Consumes: everything above.
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Run the whole suite both ways**

```bash
julia --project -e 'using Pkg; Pkg.test()'
julia --project -e 'using Pkg; Pkg.test(julia_args=["-t","4"])'
```

Expected: PASS both times.

- [ ] **Step 2: Run the reference mission scripts CI runs**

```bash
julia --project=. scripts/run_mission.jl missions/moonshot.toml
julia --project=. scripts/run_rendezvous.jl
```

Expected: both complete as before. Neither touches the panel, but they confirm
the `Project.toml` edit did not disturb the environment.

- [ ] **Step 3: Confirm the browser program still works**

Start `julia --project -t auto scripts/panel.jl 8138` and open
`http://localhost:8138/` in a browser. Check: the page loads, a mission runs and
draws, the launch view opens from the rocket chip and reaches its overlay, and
the server's shell shows a log line per request with sane durations.

- [ ] **Step 4: Document the new endpoints**

In `README.md`, in the section describing `scripts/panel.jl`, add a short
paragraph:

```markdown
The panel logs one line per request — method, path, status, duration, bytes —
and answers `GET /api/health` with its uptime, Julia version and thread count.
Both exist because an intermittent browser-side "Failed to fetch" is otherwise
invisible from the server: it leaves no trace unless the server writes one.
It listens on IPv4 and IPv6 loopback, so `localhost` resolves either way
without a failed connection attempt first.
```

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "Document the panel's health endpoint and request log

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Phase gate

Phase 0 is done when:

- `julia --project -e 'using Pkg; Pkg.test()'` passes single-threaded,
- the same passes with `-t 4` including the non-blocking assertion,
- `curl -I`, `curl -X OPTIONS`, and a malformed body each get a well-formed
  response,
- `http://[::1]:PORT/api/health` answers,
- the panel and launch pages both work in a browser against the rebuilt server.

Phase 1 (Earth-orbit missions and the named phase vocabulary) gets its own plan,
written once this one lands.
