# Earth-orbit missions, and a panel that can show them

*Design — 2026-07-24*

## 1. Why

`ss.jl` currently flies three missions, and all three end at the Moon or in the
Pacific. The mission catalogue has no Earth-orbit entry at all, which is odd for
a program whose ascent guidance, J2 gravity, Lambert solver, Hohmann and
plane-change math are all already written and tested. Adding LEO, GEO, Molniya
and friends is mostly a matter of chaining pieces that exist.

Doing that, though, exposes four things about the browser program that are
already wrong and would get worse with a third mission family:

* the launch view's timeline renders as an invisible hairline,
* the orbit view collapses a there-and-back trajectory into a sliver and offers
  no way to zoom the Moon unless you are landing on it,
* the launcher is configured through a cramped sidebar card that is already the
  busiest thing on the page,
* the HTTP server has paths that close a socket without answering.

This document covers all four plus the new mission family, as one design in four
build phases.

## 2. Evidence

Everything in this section was measured on this machine against
`scripts/panel.jl` on port 8138, not inferred from reading.

### 2.1 The timeline is drawn, and drawn invisibly

The phase table is **already fully derived from the run**. A nominal
free-return produced seven phases and sixteen events with nothing hard-coded:

```
ASCENT     -15 →    475      liftoff, pitchover, gravity_turn, max_q,
ORBIT      475 →   3281      sep_sable1, ignition_sable2, fairing_jettison,
TLI       3281 →   3491      seco, tli_ignition, tli_cutoff, perilune,
TRANSLUNAR 3491 → 277154     entry_handoff, entry_interface, deploy_drogue,
FLYBY   277154 → 287954      deploy_main, splashdown
RETURN  287954 → 562101
ENTRY   562101 → 568104
```

`buildData()` in `scripts/launch_page.html:884` builds that from `run.events`
alone, and it is right. What is wrong is the rendering:

| Measured | Value | Consequence |
|---|---|---|
| `#seek` canvas backing store | 300 × 150 | never resized to the element |
| `#seek` CSS box | 1667 × 18 | backing stretched 5.5× — blurry |
| `devicePixelRatio` | 1.5 | `drawSeek` sets `canvas.width` in CSS px |
| track height | 2 px at `bottom: 2px` | a hairline at the screen edge |
| label font | 8 px, no backing plate | illegible over lit terrain |
| click target | ~9 px tall, under the camera chips | effectively unhittable |

`#stats` and `#clock` both carry an explicit background plate, with a comment
saying why: *"A shadow alone loses on light ground, so the readouts carry their
own backing."* `#seek` never got the same treatment.

**So "the timeline doesn't work" is a presentation bug, not a data bug.** The
derivation is sound and this design keeps it, extends its vocabulary, and makes
it visible.

### 2.2 The orbit view draws both legs

Outbound is phase 2 (green `#199e70`) and return is phase 3 (orange `#c98500`),
assigned in `src/translunar.jl:252,270` and coloured in
`scripts/panel_page.html:646`. Both are on screen. At fit zoom they close into a
lens a few pixels wide and read as one stroke; pressing `F` for the rotating
frame separates them cleanly.

The genuine gap: `$('moonview').classList.toggle('hidden', !landing())`
(`scripts/panel_page.html:1774`) gates Moon-centred view to landing missions, so
on a flyby there is no way to zoom the Moon.

### 2.3 Engine choice already drives the mixture — silently

`stage_from_params` (`scripts/panel.jl:113`) takes the propellant from the
selected engine, and `bulk_density` sets tank length from it. The physics is
right. `buildStages()` never writes the resolved values back into the
`mixture`, `thrust_kn` or `isp` fields, so choosing an RL10 leaves "kerolox"
sitting in the form and the change looks inert.

### 2.4 "Failed to fetch" — three real defects, symptom not reproduced

Forty sequential POSTs, thirty parallel POSTs, and mixed bursts across every
endpoint all returned 200. The server also survived aborted requests, malformed
bodies, garbage request lines and short bodies without dying. So this is not a
crash, and it is not ordinary load.

Three defects that produce exactly this symptom:

1. **Unanswered sockets.** In `handle` (`scripts/panel.jl:774`), `parse_form`,
   `json(out)` and `respond` are all *outside* the inner `try`. Anything thrown
   there hits the bare outer `catch`, which closes the socket with no response.
   Confirmed reachable: `POST /api/run` with body `pod_mass=35%ZZ` →
   `curl: Empty reply from server`.
2. **IPv4-only bind.** `listen(IPv4(127,0,0,1), PORT)`. On Windows 11 `localhost`
   resolves `::1` first: `http://[::1]:8138` fails outright, and connecting via
   `localhost` measured **0.209 s** against **0.002 s** for `127.0.0.1` — a
   failed connection attempt on every new socket.
3. **Full serialization.** `@async handle(sock)` never yields during compute, so
   `accept` cannot run while a mission is being flown. Six concurrent runs took
   11.9 s. `/api/geometry` fires on every field `change`, so edits queue behind
   an in-flight run.

Because the reported symptom was not reproduced, Phase 0 adds per-request
logging so the next occurrence is diagnosed from a record rather than a guess.

## 3. Scope

**In scope.** An Earth-orbit mission family with a named-and-editable target
catalogue; transfer burns flown finitely on the kick stage; optional deorbit and
entry; a shared phase vocabulary between Julia and the browser; a visible,
usable, mission-agnostic timeline; a focus-aware orbit camera; a dedicated
vehicle configuration page; engine-to-propellant coupling surfaced in the UI;
server reliability and instrumentation.

**Out of scope.** On-orbit life (nodal regression over weeks, eclipse seasons,
station-keeping budgets, ground tracks). Rendezvous or constellation missions.
Interplanetary targets beyond a single hyperbolic escape case. A saved on-disk
vehicle fleet — vehicle state stays client-side.

**Deliberately unchanged.** The lunar flyby and landing chains, the entry and
6-DOF simulators, and the physics of ascent. This work adds a sibling mission
and repairs the program around it.

## 4. Architecture

### 4.1 A named phase vocabulary (the keystone)

Today a trajectory sample carries `ph`, an integer 0–5, decoded by a hard-coded
map in the page and a hard-coded `renderLegend()` that branches on
`landing()`. Three mission families will not fit in that.

Replace it with names. The run payload gains:

```json
"legend": [ {"key":"ascent",  "label":"ascent",       "color":"#8fbef0"},
            {"key":"parking", "label":"parking orbit","color":"#3987e5"},
            {"key":"transfer","label":"transfer",     "color":"#199e70"} ]
```

`legend` lists **only the phases this run actually flew**, in flight order.
Track samples index into it. Events already carry a `phase` string.

Consequences, all of which are things the user asked for:

* the legend under the scene is generated, never branched on mission type;
* a LEO run shows three entries and a lunar landing shows seven, automatically;
* an unrecognised phase gets a default colour and a humanised label instead of
  disappearing;
* the launch-view timeline builds its segments from the same list.

Phase keys are defined once in Julia (`src/phases.jl`) and consumed by both
pages. Colours live with the key so the scene, the legend and the timeline can
never disagree.

### 4.2 Phase 0 — the server

`scripts/panel.jl`:

* **Always answer.** One `try` spanning parse → dispatch → serialize → write.
  Any throw produces `500` with `{"ok":false,"error":...}`. No path closes a
  socket silently.
* **Stop blocking the accept loop.** `Threads.@spawn` per connection instead of
  `@async`. `panel_mission` is already called concurrently by `run_sweep`'s
  `Threads.@threads`, so it is exercised under threads today; Phase 0 adds an
  explicit test rather than assuming.
* **Bind both stacks.** An IPv4 and an IPv6 loopback listener feeding one
  handler, so `localhost` resolves either way with no failed attempt.
* **Answer HEAD and OPTIONS** rather than 404-with-a-body (`curl -I` currently
  reports "Weird server reply").
* **Log every request**: method, path, status, milliseconds, bytes. This is how
  the reported "Failed to fetch" gets diagnosed.
* `GET /api/health` returning version and uptime.

### 4.3 Phase 1 — Earth-orbit missions

New `src/earthorbit.jl`, exported from `SatelliteSim`.

**Target.**

```julia
struct OrbitTarget
    name::Symbol
    perigee_alt::Float64    # [m] above mean radius
    apogee_alt::Float64     # [m]; NaN when the target is an escape
    inclination::Float64    # [rad]
    argp::Float64           # [rad] — reference value, reported not commanded
    c3::Float64             # [m^2/s^2] escape energy; NaN for closed orbits
    note::String
end
```

Exactly one of `apogee_alt` and `c3` is finite. `argp` is carried so the
achieved value can be compared against the reference and the miss reported —
see the limitation at the end of this section.

**Catalogue** (`ORBITS`, served over `/api/catalogue` alongside `ENGINES` and
`PROPELLANTS`, so the UI can never offer an orbit the simulator cannot fly):

| key | perigee | apogee | inc | argp | note |
|---|---|---|---|---|---|
| `leo` | 200 km | 200 km | 28.5° | — | direct ascent, no transfer |
| `iss` | 400 km | 400 km | 51.6° | — | station inclination |
| `sso` | 700 km | 700 km | 98.2° | — | retrograde; exercises the azimuth solver |
| `gto` | 200 km | 35 786 km | 28.5° | 180° | one burn, stop at the ellipse |
| `geo` | 35 786 km | 35 786 km | 0° | — | combined circularise + plane change |
| `molniya` | 600 km | 39 400 km | 63.4° | 270° | critical inclination |
| `tundra` | 24 000 km | 47 000 km | 63.4° | 270° | 24 h period |
| `meo` | 20 180 km | 20 180 km | 55° | — | GPS-like |
| `escape` | 200 km | C3 = 0 | 28.5° | — | hyperbolic, "shoots out into space" |

Catalogue entries are defaults, not constraints: the panel copies them into
editable perigee/apogee/inclination/argp fields.

**Chain — `earthorbit(...)`.**

1. `tune_ascent` to `h_park` at the target inclination, azimuth from
   `launch_azimuth`. `|cos i / cos φ| > 1` is reported as an honest
   infeasibility, not clamped: a 28.5° site cannot reach a 20° orbit directly.
2. **Plan.** Impulsive design from `maneuvers.jl`:
   * target within 10 km of the parking radii **and** 0.1° of its inclination →
     no transfer burns; the mission is ascent plus coast. This is the LEO, ISS
     and SSO path;
   * otherwise a two-burn Hohmann — apogee raise at parking perigee, then at
     apogee a **combined** circularise-and-plane-change using vis-viva with the
     law of cosines. Doing the plane change at apogee is the entire reason GEO
     is affordable, and the Δv ledger should show it;
   * `escape` → a single burn at parking perigee to the requested C3.
3. **Fly.** Each planned burn executes as a finite burn on the kick stage, on
   its real thrust and Isp, using the same machinery as `tli_burn`. Propellant
   margin is reported; a launcher that cannot reach GEO says so.
4. **Coast.** N revolutions of the achieved orbit under `J2Gravity` for the
   viewer, N configurable, default 2.
5. **Deorbit (optional).** A retrograde burn at the achieved apoapsis — or
   immediately, for a circular orbit where apoapsis is undefined — sized to
   lower the vacuum perigee to `hp_entry` (default 25 km), then a hand-off to
   `scenario_from_elements` and the existing `simulate`. The splashdown point is
   **reported, not targeted**: `target_deorbit` exists and could steer to a
   site, but aiming the entry is a separate concern from reaching the orbit and
   is left out of this design.

**Result** — `EarthOrbitResult` carrying the ascent, the burn plan with planned
and achieved Δv per burn, the coast log with phase keys, achieved osculating
elements, the optional entry, and an `on_target` flag comparing achieved
`(rp, ra, i)` against the request.

**Panel.** The mission switch becomes three-way (Earth orbit / free-return flyby
/ lunar landing). A new "Orbit target" card holds the catalogue picker and the
editable elements. New metrics — achieved perigee/apogee/inclination, transfer
Δv per burn, total Δv, propellant margin, period, C3 for escape — and their
entries in `SOLVE_METRICS` so sweep and solve work on them the same way.

**Known limitation, stated rather than hidden.** Argument of perigee is a
*consequence* of where the first burn happens, not a commanded quantity. v1
targets `(rp, ra, i)`, reports the achieved argp, and notes the miss for
Molniya and Tundra. Commanding argp needs a coast-phasing search and is
deferred.

### 4.4 Phase 2 — timeline and orbit camera

**Timeline** (`scripts/launch_page.html`). Same derivation, fixed rendering:

* 44 px tall, sitting above the camera chip row, with its own backing plate;
* backing store sized `clientWidth * devicePixelRatio`, context scaled to match;
* segments proportional to the phase list with the phase name and its duration
  legible at 11 px;
* event ticks with labels on hover, and a hover preview of the time under the
  cursor before you commit to a click;
* current phase emphasised; the whole bar is a click and drag target;
* `←`/`→` step between events.

Cinematic pacing (`warp`, `pace`) stays a client concern, keyed by phase name
with a **default for unknown phases** — so a mission family added later degrades
to sane pacing instead of vanishing from the bar. This is what makes the
timeline mission-agnostic: a LEO run yields ASCENT / PARKING / TRANSFER, a GEO
run adds CIRCULARISE and ON ORBIT, an escape run ends in ESCAPE, and a lunar
landing still yields its seven.

**Orbit camera** (`scripts/panel_page.html`):

* **auto** by default: frames the vehicle and whichever body dominates, rescaling
  as the trajectory progresses;
* focus chips — Earth, Moon, vehicle, fit — override auto until `auto` is
  pressed again, and are available in **every** mission mode;
* a Moon inset that appears whenever the trajectory comes within a threshold of
  the Moon, so the encounter is legible without leaving the wide view;
* direction arrowheads along the track, and the frame toggle relabelled to say
  what it does ("Earth–Moon rotating — shows the figure-8") rather than naming
  the state it will switch to;
* the legend is generated from `run.legend` (§4.1).

The inset is a second camera and a second render pass. If it measurably costs
frame rate it gets reported, not silently dropped.

### 4.5 Phase 3 — vehicle page and engine coupling

**`/vehicle`**, layout C: a left rail listing the stack (reorderable, dashed row
to add a stage, a strap-on section), the 3D scene in the centre, and an
inspector on the right showing every field of the selection. Clicking a stage in
the scene selects it in the rail and vice versa.

Vehicle configuration lives in `localStorage` plus URL parameters — the same
field names both pages already use — so there is no server state and no sync
protocol. The panel's Launcher card collapses to a read-only summary (stages,
liftoff mass, pad TWR, total Δv) with a link to `/vehicle`.

Live feedback on every edit, from `/api/geometry`, which already returns all of
it: liftoff mass, pad TWR, per-stage Δv, burn time, stage length, total Δv. A
stage that cannot lift what sits above it is marked.

**Engine coupling.** Selecting an engine writes the resolved values back into
the visible fields — mixture, vacuum thrust, Isp, exit area — and the scene
restretches the tank, because hydrolox at 343 kg/m³ against kerolox at
1023 kg/m³ needs roughly three times the barrel for the same propellant mass.
Fields the engine now owns are marked derived, the way `size_by` already greys
whichever of propellant-mass and tank-length is not being driven.

## 5. Testing

**Julia (`test/runtests.jl`).** A new `@testset "earth orbit"`:

* every catalogue entry reaches its target `(rp, ra, i)` within tolerance, or
  reports infeasible for a stated reason;
* transfer Δv agrees with the closed-form Hohmann-plus-plane-change to a few
  percent — the finite-burn loss is the difference and should be small and
  positive;
* propellant is conserved across the burns and the margin matches the stage
  load;
* SSO exercises the retrograde azimuth path;
* the escape case yields C3 ≥ 0 and a hyperbolic eccentricity;
* deorbit from LEO reaches splashdown through the existing entry sim.

**Server.** `scripts/test_panel_http.jl` starts the panel on an ephemeral port
and asserts that **every** endpoint returns a well-formed response — including
for malformed bodies, bad percent-escapes, HEAD, OPTIONS, unknown paths, and
concurrent bursts. The single invariant: *no request ever gets an empty reply.*

**Browser.** A short manual checklist per phase, verified against a running
panel: timeline visible and clickable at three window sizes; focus chips work in
all three mission modes; a hydrolox engine visibly restretches the tank; a LEO
run shows fewer timeline phases than a lunar one.

## 6. Risks

* **The reference vehicle cannot reach GEO.** Correct behaviour, bad first
  impression. Each catalogue target ships with a suggested vehicle preset so the
  first run of a target succeeds.
* **`Threads.@spawn` per connection** assumes `panel_mission` is thread-safe.
  `run_sweep` already relies on this; Phase 0 tests it explicitly rather than
  inheriting the assumption.
* **The Moon inset** costs a render pass. Measured, and reported if it is too
  expensive.
* **The unreproduced fetch failure** may survive Phase 0. Request logging is the
  mitigation: the next occurrence leaves a record.
* **Phase-vocabulary migration** touches both pages and every mission chain. It
  lands in Phase 1 with the lunar missions converted first, so the existing
  missions prove the vocabulary before the new one depends on it.

## 7. Sequence

| Phase | Content | Gate |
|---|---|---|
| 0 | server reliability + logging | HTTP test suite green |
| 1 | phase vocabulary, `earthorbit.jl`, catalogue, panel mode | Julia tests green; a GEO and a Molniya run complete |
| 2 | timeline, orbit camera, legend | manual checklist across three mission modes |
| 3 | `/vehicle` page, engine coupling | manual checklist; panel card reduced |

Phase 0 is first because every later phase is verified through the browser, and
a server that occasionally fails to answer makes every other result
untrustworthy.
