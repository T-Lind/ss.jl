# A launch page that holds 60 fps, and an entry that is drawn in the right frame

*Design — 2026-07-24*

## 1. Why

Two complaints, one file. `scripts/launch_page.html` is choppy from the moment
the vehicle leaves the pad, and every camera during the entry phase draws
something wrong — a hard seam across the frame, a planet that is flat and
speckled, a capsule that is hard to find.

They turn out to be closely related. Both were introduced by the same commit
(`567618e`, "Give the planet a real cloud deck, and the Moon a real surface"),
and the entry defect *causes* part of the performance defect: it draws a 52 ms
shader pass through the whole high-altitude entry, where that pass has no
business running at all.

This document covers both. It does not touch the Earth-orbit mission family or
the vehicle page — those are
`2026-07-24-earth-orbit-missions-and-panel-rework-design.md`, still accurate,
still unbuilt past Phase 0.

## 2. Evidence

Everything here was measured on this machine against `scripts/panel.jl` on port
8138, in Chrome, on a nominal `Sable` free-return. Numbers are median (p50) and
p90 of `requestAnimationFrame` deltas over ~85 frames.

### 2.1 The machine, and the bill it is being handed

| Measured | Value |
|---|---|
| GPU | `ANGLE (Intel, Intel(R) UHD Graphics (0x00004626) Direct3D11)` |
| `devicePixelRatio` | 1.5 |
| canvas CSS box | 1707 × 842 |
| canvas backing store | **2560 × 1263 = 3 233 280 px** |

An integrated Intel UHD 630, asked for 3.23 million pixels per frame.

### 2.2 Two shaders are the entire frame

Ascent (pad world, `running`, `simT` 45 → 69), disabling passes by
intercepting `gl.drawArrays` on the current program:

| Configuration | p50 | p90 | implied fps |
|---|---|---|---|
| everything | **136.8 ms** | 242.4 ms | 7 |
| − planet/sky pass (`SPACE`) | 52.5 ms | 82.8 ms | 19 |
| − cloud-deck plane (`CLOUD`) | 84.7 ms | 155.7 ms | 12 |
| − both | **16.9 ms** | 20.7 ms | 59 |
| **no draw calls at all** | **16.8 ms** | 17.9 ms | 60 |

Read the last two rows together. With those two passes gone the frame costs
**16.9 ms**, and drawing *nothing whatsoever* costs **16.8 ms**. The difference
is under a tenth of a millisecond, which is the honest measurement of what the
rest of the program costs: the lit meshes, the particle system, the decal
sheet, the HUD, the JS state integration, all of it, together, are free.

By subtraction:

* planet/sky pass ≈ **84 ms**
* cloud-deck plane ≈ **52 ms**
* everything else ≈ **0.1 ms**

A null-render floor of 16.8 ms also confirms `requestAnimationFrame` is
delivering at 60 Hz and is not being throttled by the automation harness, so
these are real GPU-bound frame times rather than an artefact of how they were
collected.

### 2.3 Why those two passes are expensive

Neither is doing anything wasteful in the small; both are doing a great deal
per pixel. The planet pass evaluates band-limited 3D gradient-noise fbm
repeatedly for every fragment:

| Field | Octaves | Line |
|---|---|---|
| cloud shape | 10 | `launch_page.html:493` |
| cloud weather | 5 | `:488` |
| cloud shadow lookup (a second cloud evaluation) | — | `:726` |
| land mask — coast / noise / base | 6 + 4 + 4 | `:657-687` |
| flow warp | 3 + 3 | `:657-658` |
| settlement lights | 7 | `:744` |
| Moon craters | 3×3 cell loop + analytic gradient | `:521-585` |

Each octave hashes eight lattice corners. That is 40-plus octaves per pixel,
times 3.23 million pixels, times 60 — a bill no integrated GPU pays. The cost
also *rises* with altitude as the band limit admits finer octaves, which is
why it degrades as the vehicle climbs: 58 ms at T+23 s, 137 ms at T+60 s.

Scaling confirms it is fragment-bound rather than CPU- or geometry-bound:
quartering the pixel count took a 58 ms frame to 21 ms.

**This is not a criticism of that shader.** Its commit message is a careful
account of real problems solved — the lattice moiré, the band limit, the
coverage quantile, the scale split. The values are right. What is wrong is
that all of it is evaluated at native resolution every frame on hardware that
cannot afford it.

### 2.4 The entry world is drawn in the wrong frame

The entry world anchors its local frame at the **splashdown site**:
`earthC: [0, -RE, 0]`, `eMat: anchorMat(run.sites.splash_lat, ...)`
(`launch_page.html:3492-3495`). Entry begins roughly 6 600 km uprange of that
point, so for most of the phase the vehicle is nowhere near the origin — yet
`cam.eye[1]` is used as though it were altitude:

```js
gl.uniform1f(SPACE.u.uDeck,
             sc.world === 'pad' || sc.world === 'entry' ? deckFade(cam.eye[1]) : 0);   // :3647
drawClouds(sc, view, proj, fogCol, fogDen, cam.eye[1]);                                // :3740
```

Measured across the phase:

| `simT` | true altitude | `cam.eye[1]` | `deckFade` | horizontal distance from frame origin |
|---|---|---|---|---|
| 562200 | 64 351 m | **−3 281 844** | **1.0** | 5 645 km |
| 562261 | 84 422 m | **−3 714 605** | **1.0** | 5 884 km |
| 562600 | 272 266 m | **−6 227 186** | **1.0** | 6 642 km |
| 567300 | 7 971 m | 7 981 | 1.0 | 0 km |
| 568080 | 52 m | 4 | 1.0 | 0 km |

`deckFade` returns 0 above 45 km by design (`:3752`). Because `camY` is a large
negative number it returns **1.0 for the entire high-altitude entry**, so
`drawClouds` draws the deck — a flat quad at local y = 7 000 — while the camera
sits millions of metres from it. That is the hard full-width seam in the
reported screenshot, and the flat wash below it.

Two more consequences of the same mistake:

* `PUP = [0, 1, 0]` (`:3489`) is the local up **at the splash site only**, so
  particle buoyancy pushes plasma along an axis that is not up.
* the planet pass is told to stand its own cloud shell down by `uDeck = 1`,
  though `:832-833` scales that by `exp(-|xz|/45000)` about the frame origin,
  which far downrange is zero — so the shell survives. This one is
  accidentally harmless, and worth not "fixing" into a regression.

Confirmed by inspection at the GPU: the uniforms are **correct** —
`uEc` length 6 455 422, `uER` 6 371 000, and a replicated float32 ray-sphere
test returns a valid hit at t = 84 422 m. So the geometry feeding the shader is
sound and the defect is in what the page tells the shader about the deck, not
in where it says the planet is.

**Not yet isolated.** Neutralising `deckFade` alone does *not* restore the
planet: it still renders flat blue with bright speckles, and with the cloud
pass skipped that region is still drawn by the planet pass. So at least one
further defect exists in the planet pass's compositing for this camera regime.
Candidates, in order of suspicion: `covE` falling slightly short of 1 across
the disc so the star field bleeds through at low weight; and the settlement
light field (`:744`) printing over the day side. This design does not guess —
Task 1 bisects it against a measured reference.

The capsule is not missing from the mesh pass: at `simT` 562261 in chase view
the pod is drawn with 10 392 vertices and `secGone` does not contain it. It is
lost *visually*, against a background that is wrong.

## 3. Scope

**In scope.** Rendering the two costly fullscreen passes at reduced resolution
behind an adaptive scaler; correcting the entry world's use of `cam.eye[1]`;
bisecting and fixing the remaining entry compositing defect; a frame-time
readout; a recorded frame budget so this cannot silently regress.

**Out of scope.** The Earth-orbit mission family, the `/vehicle` page, the
timeline rendering and the orbit camera — all in the companion spec. Any
reduction in the shader's octave counts or feature set: resolution is the lever
this design pulls, and octave trimming is held as a named fallback rather than
done pre-emptively.

**Deliberately unchanged.** The noise fields, the cloud climatology, the crater
field, the photometry. Their values are right.

## 4. Architecture

### 4.1 Scale-separated rendering

One offscreen framebuffer, `bgFB`, with a colour texture and a depth
attachment, sized `round(w*scale) × round(h*scale)`.

Frame order becomes:

1. bind `bgFB`, viewport to the scaled size, clear;
2. **depth-only prepass** of the vehicle meshes at the scaled size — depth
   writes on, colour masked off, no shading. A few thousand triangles, so it
   costs approximately nothing, and it is what lets step 4 resolve correctly;
3. planet/sky pass (`SPACE`) — `depthMask(false)`, as today;
4. cloud-deck plane (`CLOUD`) — depth-tested against the prepass, so the deck
   is correctly hidden where the vehicle is in front of it;
5. bind the default framebuffer, viewport to native, blit `bgFB`'s colour up
   with a single textured quad and linear filtering;
6. lit meshes, particles and HUD at **native** resolution, over the top.

The sky and the planet are low-frequency and survive the upsample; the vehicle
and its plume, which carry the edges the eye actually tracks, never leave
native resolution. That split is the whole point.

**Known limitation, stated rather than hidden.** Steps 5–6 draw the vehicle
over a composited background with a freshly cleared native depth buffer, so the
deck cannot occlude the vehicle in the one case where the camera is *above* the
7 km deck looking down at a vehicle *below* it. Today that case is drawn
correctly. The fallback, if it reads badly during ascent, is to blit `bgFB`'s
depth alongside its colour and keep the native pass depth-aware. This is
checked explicitly in §5, not assumed away.

### 4.2 The adaptive scaler

A rolling median of the last 30 frame times drives `scale`:

* median > 20 ms → `scale -= 0.05`
* median < 13 ms → `scale += 0.05`
* clamp to `[0.35, 1.0]`, and require 30 frames between adjustments so it
  settles instead of oscillating.

The dead band between 13 and 20 ms is deliberate: it brackets the 16.7 ms vsync
interval, so a frame that is comfortably making rate is left alone.

Starting `scale` is 1.0 for one second, so the scaler measures the machine
rather than trusting a guess. On the hardware in §2.1 the arithmetic predicts it
settles near **0.34** — √(16 ⁄ 136) — and this design does not pretend that is
free. It is roughly 870 × 430 for the sky. Whether that softening is a fair
price is a judgement the user makes by looking at it, which is why §4.3 puts the
number on screen.

`scale` is also clamped so `w*scale` never falls below 480 px, because below
that the limb antialiasing in the shader is working on a footprint it was not
written for.

### 4.3 A frame-time readout

The existing stats plate gains two optional lines, off by default and toggled
by `p`: median frame time in ms, and the current background scale. This is how
the tradeoff in §4.2 becomes visible rather than theoretical, and how the
measurements in §2.2 can be reproduced on any machine without a debugger.

### 4.4 The entry frame

* `deckFade` takes a **true geometric altitude**, not `cam.eye[1]`. `drawScene`
  already computes exactly this as `camAlt` (`:3612`); it is passed in rather
  than recomputed, so there is one definition of altitude in the file.
* `deckFade` additionally returns 0 when the camera is further than **150 km**
  horizontally from the local frame origin, because that is the region over
  which the drawn plane has any validity. The plane and the shell hand over
  through `exp(-|xz|/45000)` (`:833`); the gate is that profile made explicit
  on the JS side. 150 km rather than something tighter: the pad world runs all
  the way to orbit insertion, so the vehicle goes genuinely downrange while
  still below the 45 km altitude gate, and a tight gate would pop the deck off
  mid-ascent. At 150 km the plane already contributes `exp(-3.33)` = 3.6%, so
  nothing visible is given up, while the entry distances that caused the bug
  are 5 645-6 642 km — four decades clear of the threshold.
* the call site moves into a function, `deckWeight(world, camEye, earthC)`.
  This is not tidying: the original `deckFade` was *correct given an altitude*
  and returns 0 for 84 422 m exactly as it should. The defect was entirely in
  what the call site passed it. So assertions on `deckFade` alone would have
  been green on this bug, and the thing that needs to be testable is the
  computation that turns a camera into a weight.
* `PUP` becomes the true local up, `normalize(cam.eye - earthC)`, in the entry
  and pad worlds alike. At the pad the two agree to within the width of the
  complex, so this is a no-op there and a correction downrange.
* the remaining compositing defect is bisected (§5, Task 1) and fixed on the
  evidence, not on this document's guesses.

Note that this is also a performance fix: it stops a 52 ms pass from running
through the entire high-altitude entry.

## 5. Testing

**Frame budget, recorded.** A checklist run against a live panel, capturing
p50 and p90 frame times and the settled scale, in three worlds (pad during
ascent, eci during cruise, entry at 84 km) at three window sizes. The numbers
go in the task report. The gate is p50 ≤ 20 ms in all nine cells; the *record*
is the point, so the next change to this file can be compared rather than
guessed at.

**The occlusion case from §4.1.** Ascent, chase camera, climbing through 7 km,
watching whether the vehicle shows through the deck. Pass = it does not, or the
depth-blit fallback is adopted and the cell re-run.

**Entry correctness, per camera.** At 84 km and at 8 km, in all seven camera
modes, with `parts` cleared so no particle can be mistaken for an artefact:
no full-width seam; a lit planet with clouds where the planet is; the capsule
visible in the exterior views. Screenshots kept for the three that were wrong.

**A regression test for the frame confusion.** Both `deckFade(alt, rng)` and
`deckWeight(world, camEye, earthC)` are pure, so they are testable without a
GPU, behind `window.__selftest()` and auto-run on `?selftest=1`.

The assertions that matter are the `deckWeight` ones, built from camera
positions with a known true altitude and known range from the frame origin:
0 at 84 km and 5 884 km downrange, 0 at 272 km and 6 642 km downrange, but
still 1 at 8 km over the splash site and 1 on the pad 5 km downrange. Solving
for those positions is what makes `camEye[1]` come out at about −3.7e6, which
is the number the old code was reading as an altitude — so one assertion pins
its sign, putting the reason on the record rather than in a commit message.

`deckFade`'s own arithmetic is asserted separately (altitude gate, the
thin-shell exclusion at deck height, the range gate), but note that **every
one of those passes against the buggy code.** That is the point: a test suite
covering only `deckFade` would have been green throughout, which is why §4.4
extracts the call site.

**Stated limitation:** `__selftest` cannot run in CI — CI is Julia-only and has
no browser, and adding a JS runner would mean a build step for a page whose
whole value is being one dependency-free file. It is run by hand at the end of
the entry-fix task and again at close-out.

**No new Julia work**, so `test/runtests.jl` and `test/panel_http.jl` must
simply stay green — run once at the end rather than per task, since nothing
here touches the simulator.

## 6. Risks

* **0.34 is too soft.** The measured scaler target may look worse than 7 fps
  feels. Mitigated by §4.3 making it visible and by octave trimming being held
  ready as a named, scoped fallback rather than a redesign.
* **The occlusion limitation in §4.1 is visible.** Explicitly tested, with the
  depth-blit fallback already identified.
* **The unbisected entry defect is worse than expected** — e.g. it needs a
  change to the compositing order rather than a one-line fix. Task 1 is
  therefore diagnosis-only, gated on a written explanation before any edit, so
  a surprise reshapes the plan instead of being patched around.
* **The scaler oscillates** on a machine near the dead band. The 30-frame
  hold-off and the 7 ms band are the mitigation; if it still hunts, the band
  widens rather than the scaler being made cleverer.
* **A blit costs bandwidth.** One fullscreen textured quad, measured, not
  assumed — if it does not pay for itself at high `scale` the scaler skips the
  indirection and renders direct when `scale == 1.0`.

## 7. Sequence

| Task | Content | Gate |
|---|---|---|
| 1 | Bisect the entry compositing defect; write down the cause | a written explanation that predicts the observed pixels |
| 2 | Fix the entry frame: `deckFade`, `PUP`, plus Task 1's finding | entry checklist clean in all seven cameras; `deckFade` unit tests pass |
| 3 | `bgFB`, depth prepass, blit composite at fixed `scale = 0.5` | no seam, no vehicle-through-cloud; frame time roughly halves |
| 4 | The adaptive scaler and the `p` readout | nine-cell frame budget recorded, p50 ≤ 20 ms |
| 5 | Close out: full Julia suite, README note on the readout | CI green |

Task 1 comes first because Task 3 changes how everything is composited, and
bisecting a shader through a new framebuffer is two unknowns at once. Task 2
lands before the performance work for the same reason, and because it removes
a 52 ms pass that would otherwise pollute Task 4's measurements.
