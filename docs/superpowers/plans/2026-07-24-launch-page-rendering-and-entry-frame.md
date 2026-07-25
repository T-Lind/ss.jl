# Launch Page Rendering & Entry Frame Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `scripts/launch_page.html` hold ~60 fps during ascent, and draw the entry phase in a coordinate frame it is actually valid in.

**Architecture:** The two costly fullscreen noise passes (planet/sky and cloud deck) move into an offscreen framebuffer rendered at an adaptive fraction of native resolution; the vehicle meshes, particles and HUD stay at native resolution and composite over it. Separately, the entry world stops using `cam.eye[1]` as an altitude — it is the splash-site frame's y axis, and entry begins ~6,600 km from that origin.

**Tech Stack:** Vanilla JS + WebGL **1** (`getContext('webgl')`), no build step, no dependencies. Julia 1.12.6 for the (untouched) simulator. The page is one self-contained HTML file served fresh per request by `scripts/panelapp.jl`.

## Global Constraints

- **WebGL 1 only.** No `WEBGL2`, no `drawBuffers`, no depth *textures*. Framebuffer depth uses a `DEPTH_COMPONENT16` renderbuffer.
- **No new dependencies, no build step, no `package.json`.** The page must remain a single file that works when opened by the panel.
- **Do not change the noise fields.** `NOISE_GLSL`, `CLOUDS_GLSL`, `MOON_GLSL`, `STARS_GLSL` and the octave counts are out of scope. The lever is resolution, not detail. (Octave trimming is a named fallback only, and only if Task 4's gate cannot be met.)
- **Page edits need only a browser refresh**; `scripts/panelapp.jl` edits need a server restart. Nothing in this plan touches `panelapp.jl`.
- **Test panel on port 8138.** Port 8137 is often the user's live session — never kill it.
- Start the panel with: `julia --project -t auto scripts/panel.jl 8138`
- **Never trust a frame timing without checking rAF is not throttled.** Stub `gl.drawArrays` to a no-op and sample; a null-render p50 near 16.8 ms means rAF is delivering a true 60 Hz. If the null-render p50 is far above 16.8 ms the window is occluded and every timing in that session is worthless.
- Measured reference numbers this plan is judged against, all on ascent, 2560×1263 backing store, Intel UHD 630: full frame **136.8 ms**, minus planet **52.5 ms**, minus cloud **84.7 ms**, minus both **16.9 ms**, no draw calls **16.8 ms**.

---

## File Structure

**Modified:** `scripts/launch_page.html` — the only source file this plan touches.

Within it, the work lands in four places, and the plan deliberately splits `drawScene` into named passes so that each is small enough to reason about:

| Region | Responsibility | Change |
|---|---|---|
| `deckFade` (~`:3751`) | deck weight from altitude and range | signature becomes `(alt, rng)`; add the 150 km range gate |
| new: `deckWeight` (beside `deckFade`) | camera → deck weight; **the call site that carried the bug** | created, so it can be asserted on |
| `buildScene` pad/entry branches (`:3421`, `:3489`) | per-frame scene description | `PUP` becomes true local up |
| `drawScene` (`:3599`) | frame orchestration | split into `drawPlanet` / `drawWorldMeshes` / composite; add offscreen target |
| new: `bgTarget`, `BLIT`, `blitBackground` | scaled background plumbing | created |
| new: `perfPush`, `bgScale`, `perfMed`, `perfShow` | the adaptive scaler and its readout state | created |
| `drawHUD` (`:3798`), `initControls` (`:3903`), `#keys` (`:100`) | readout | add `p` toggle and two rows |
| new: `window.__selftest` | 17 assertions on the two pure functions | created |

**Also modified:** `README.md` — one paragraph documenting the `p` readout, in Task 5.

**Not modified:** `src/**` , `test/**`, `scripts/panelapp.jl`, `scripts/panel.jl`, `.github/workflows/ci.yml`. No Julia behaviour changes, so the Julia suite is a regression check run once at the end rather than per task.

---

## A note on testing this file

There is no JS test runner in this repo and this plan does not add one — CI is Julia-only and has no browser. Instead, the pure functions get assertions behind `window.__selftest()`, auto-run when the page is loaded with `?selftest=1`. That is a real automated test, just browser-hosted, and it matches a project whose UI is one dependency-free HTML file.

**Stated limitation:** `__selftest` does not run in CI. It must be run by hand (one `javascript_tool` call, or by eye) at the end of Tasks 2 and 5. This is why Task 2's gate names it explicitly.

Everything else in this plan is verified by measurement in a real browser, because a frame budget cannot be asserted anywhere else.

---

### Task 1: Diagnose the remaining entry compositing defect

**Diagnosis only. Do not edit the shader in this task.** The deliverable is a written explanation that predicts the observed pixels. If you find yourself wanting to change code to see what happens, that is fine — but revert it before the commit, and the commit contains only the written finding.

**Why first:** Task 3 changes how everything composites. Bisecting a shader through a brand-new framebuffer is two unknowns at once.

**What is already known** (do not re-derive; verify if cheap):
- The uniforms reaching the GPU are correct. At `simT` 562261: `uEc` = `[-5883535.5, -2656403.8, -34]` (length 6,455,422), `uER` = 6,371,000, `uNear` = 0, `uDeck` = 1, `uPxA` = 2.393e-4. A float32 replication of `sphereHit` returns a valid hit at t = 84,421.84 m.
- Neutralising `deckFade` to 0 does **not** restore the planet.
- Skipping the `CLOUD` program does **not** remove the wrong region: with it skipped, pixels at 30%/70%/95% down the frame read `[0,0,0]`, `[26,41,69]`, `[54,81,131]`. So the **planet pass** draws that blue-grey.
- The capsule is drawn: pod section present, 10,392 vertices, not in `secGone`.

**Files:**
- Create: `.superpowers/sdd/2026-07-24-launch-page-rendering-and-entry-frame/task-1-finding.md`
- Read only: `scripts/launch_page.html:597-908` (the `SPACE` program)

**Interfaces:**
- Consumes: nothing.
- Produces: a written cause that Task 2 Step 6 implements. Task 2 depends on this file existing and naming a specific expression and line number.

- [ ] **Step 1: Start a panel and reach the broken frame**

```bash
julia --project -t auto scripts/panel.jl 8138
```

Open `http://127.0.0.1:8138/`, click **launch view** (this guarantees a valid parameter set — hand-built query strings fail with `ascent failed to reach orbit`). Wait for `POST /api/run` to return ~178 kB in the panel log, then click the overlay to start.

In the page console, freeze the exact frame:

```js
running = false; simT = 562261; parts.length = 0; debris.length = 0;
document.dispatchEvent(new KeyboardEvent('keydown',{key:'3',bubbles:true}));  // chase
```

- [ ] **Step 2: Confirm rAF is not throttled**

```js
const s=(n)=>new Promise(r=>{const d=[];let l=performance.now();
  const st=()=>{const t=performance.now();d.push(t-l);l=t;
    if(d.length<n)requestAnimationFrame(st);else r(d);};requestAnimationFrame(st);});
const rd=gl.drawArrays.bind(gl); gl.drawArrays=function(){};
const d=(await s(60)).slice(10).sort((a,b)=>a-b); gl.drawArrays=rd;
d[Math.floor(d.length/2)];
```

Expected: a number near **16.8**. If it is far higher, the window is occluded — un-occlude it and repeat. Do not proceed on throttled timings.

- [ ] **Step 3: Bisect the compositing in `main()`**

The suspects, in order. Test each by reading back a pixel known to be on the planet's disc — pick one and reuse it:

```js
const P = () => { const b=new Uint8Array(4);
  gl.readPixels(Math.round(cv.width*0.3), Math.round(cv.height*0.30), 1,1,
                gl.RGBA, gl.UNSIGNED_BYTE, b); return Array.from(b); };
```

(`cv.height*0.30` in `readPixels` coordinates is 70% down the *screen*, because `readPixels` origin is bottom-left. That is the blue-grey region.)

To test a hypothesis, recompile the `SPACE` fragment shader with one expression forced, using the page's own `program()` helper, then re-render and read the pixel back. Forcing `covE` to 1.0 is the first and most informative:

1. **`covE` falls short of 1 across the disc** — `limbCov` at `:639`. Force `float covE = 1.0;` after `:892`. If the planet appears correctly lit, this is the cause.
2. **`skyCol` is bleeding through** — the `opaque > 0.997` guard at `:901`. Log/force `opaque` to 1.0 for the same pixel.
3. **`earthCol` itself is returning near-flat blue** — meaning the land mask, terminator or the `uNear`/`lodS` path is degenerate for this camera. Force `earthCol`'s return to `vec3(1,0,0)`; if the region turns red, `earthCol` *is* being reached and the defect is inside it, which redirects the bisect into `:647-760`.
4. **The settlement-light field is printing over the day side** — `:744`, 7 octaves of `pop`. Force its contribution to zero and look for the speckles disappearing.

- [ ] **Step 4: Write the finding**

Create `.superpowers/sdd/2026-07-24-launch-page-rendering-and-entry-frame/task-1-finding.md` containing:

- the exact expression and line number at fault;
- **why** it misbehaves in this camera regime specifically and not on the pad or in `eci` — a cause that only says "it is wrong here" has not been found yet;
- the pixel readback before and after forcing it;
- the minimal correction, as the literal replacement line;
- whether the correction is safe for the pad and `eci` worlds, and how that was checked.

If bisection shows the cause is **not** in the four suspects, say so plainly and record what was eliminated with its evidence. A correct "none of these, here is what I ruled out and the next three places to look" is a successful Task 1 — it reshapes Task 2 instead of being patched around. Do not invent a fix to have something to report.

- [ ] **Step 5: Verify no source file changed**

```bash
git status --porcelain scripts/launch_page.html
```

Expected: **empty output.** If the shader still has a forced expression in it, revert:

```bash
git checkout -- scripts/launch_page.html
```

- [ ] **Step 6: Commit**

```bash
git add .superpowers/sdd/2026-07-24-launch-page-rendering-and-entry-frame/task-1-finding.md
git commit -m "Find why the planet goes flat during entry"
```

---

### Task 2: Fix the entry frame

**Files:**
- Modify: `scripts/launch_page.html` — `deckFade` (~`:3751`), `drawScene` (`:3612-3648`, `:3740`), `drawClouds` (`:3757`), `buildScene` pad branch (`:3421`) and entry branch (`:3489`)
- Modify: `scripts/launch_page.html` — add `window.__selftest` near `boot()` (~`:4030`)
- Read: `.superpowers/sdd/2026-07-24-launch-page-rendering-and-entry-frame/task-1-finding.md`

**Interfaces:**
- Consumes: Task 1's finding — the faulty expression, line number and replacement line.
- Produces:
  - `deckFade(alt: number, rng: number) -> number` — weight in `[0,1]`. `alt` is true geometric altitude above mean radius in metres; `rng` is horizontal distance from the local frame origin in metres. **Both required**; the old one-argument form must not survive anywhere.
  - `deckWeight(world: string, camEye: number[], earthC: number[]) -> number` — the **call site**, extracted so it is testable. This is the function that carries the defect, so this is the one the assertions have to exercise.
  - `drawClouds(sc, view, proj, fogCol, fogDen, deck: number)` — last parameter is now the already-computed weight, not `camY`. `drawClouds` no longer calls `deckFade` itself.
  - `window.__selftest() -> {pass: number, fail: number, failures: string[]}`
  - `DECK_RNG = 150000` — the range gate constant.

**Why `deckWeight` has to exist.** The original `deckFade(camY)` is *correct given an altitude* — hand it 84,422 and it returns 0 exactly as it should. The defect is that the call site handed it `cam.eye[1]`, which is the splash-site frame's y axis. So assertions on `deckFade` alone would have **passed against the buggy code** and caught nothing. The call site is what must be under test, which means it has to be a function rather than an expression buried in `drawScene`.

- [ ] **Step 1: Write the failing assertions**

Add just above `boot()` (~`:4030`):

```js
// ------------------------------------------------------------- self-test --
// There is no JS test runner in this repo and adding one would mean a build
// step for a page whose whole point is being a single dependency-free file.
// Run by hand, or load the page with ?selftest=1.
//
// The assertions that matter here are the deckWeight ones. The original
// deckFade was fine given an altitude — the bug was that drawScene handed it
// cam.eye[1], which is an altitude only in the pad world and is about -3.7e6
// halfway through an entry. So the CALL SITE is what has to be under test;
// testing deckFade alone would have passed against the broken code.
window.__selftest = function () {
  const out = { pass: 0, fail: 0, failures: [] };
  // fn is a thunk so that a missing or throwing function is a failure with a
  // readable message, not a crash that hides every assertion after it.
  const eq = (name, fn, want, tol) => {
    let got;
    try { got = fn(); }
    catch (e) { out.fail++; out.failures.push(`${name}: threw ${e.message}`); return; }
    if (Math.abs(got - want) <= (tol === undefined ? 1e-9 : tol)) out.pass++;
    else { out.fail++; out.failures.push(`${name}: got ${got}, want ${want}`); }
  };
  // Build a camera position with a known true altitude and a known horizontal
  // range from the local frame origin, in a frame whose origin is on the
  // surface with earthC one radius below — which is what the pad and entry
  // worlds both use. Solving rng^2 + (y+RE)^2 = (RE+alt)^2 for y is what makes
  // camEye[1] come out hugely negative at range, and that negative number is
  // exactly what the old code was reading as an altitude.
  const EC = [0, -RE, 0];
  const eyeAt = (alt, rng) => [rng, Math.sqrt((RE + alt)**2 - rng**2) - RE, 0];

  // --- the defect. Measured (altitude, range) pairs from a real entry; every
  // one of these drew the deck plane from millions of metres away before the
  // fix, and every one must now be 0.
  eq('entry 64 km, 5645 km downrange', () => deckWeight('entry', eyeAt(64351, 5645000), EC), 0);
  eq('entry 84 km, 5884 km downrange', () => deckWeight('entry', eyeAt(84422, 5884000), EC), 0);
  eq('entry 272 km, 6642 km downrange', () => deckWeight('entry', eyeAt(272266, 6642000), EC), 0);
  // and the sign of the thing that fooled it, asserted so the reason is on the
  // record rather than in a commit message
  eq('camEye[1] is nothing like an altitude at range',
     () => Math.sign(eyeAt(84422, 5884000)[1]), -1);

  // --- the cases the range gate must NOT break
  eq('entry 8 km, over the splash site', () => deckWeight('entry', eyeAt(7981, 0), EC), 1);
  eq('entry at the water', () => deckWeight('entry', eyeAt(4, 0), EC), 1);
  eq('pad at 30 km, 5 km downrange', () => deckWeight('pad', eyeAt(30000, 5000), EC), 1);
  // --- worlds with no drawn deck at all
  eq('eci has no deck', () => deckWeight('eci', eyeAt(30000, 0), EC), 0);
  eq('moon has no deck', () => deckWeight('moon', eyeAt(30000, 0), EC), 0);

  // --- deckFade's own arithmetic, unchanged by this fix except for the gate
  eq('above the 45 km cutoff', () => deckFade(50000, 0), 0);
  eq('at the cutoff', () => deckFade(45000, 0), 0);
  eq('30 km, at the origin', () => deckFade(30000, 0), 1);
  eq('40 km fades on altitude', () => deckFade(40000, 0), 1/3, 1e-6);
  // the thin-shell exclusion: the camera must not sit inside the quad
  eq('exactly at deck height', () => deckFade(7000, 0), 0);
  eq('70 m under the deck', () => deckFade(6930, 0), 0.5, 1e-6);
  // the range gate. The shader hands over on exp(-|xz|/45000) about the
  // origin, so by 150 km the plane contributes 3.6% and nothing is given up,
  // while a late ascent tens of km downrange keeps the deck it should have.
  eq('60 km downrange still keeps the deck', () => deckFade(20000, 60000), 1);
  eq('past the range gate', () => deckFade(20000, 150001), 0);

  console.log(`__selftest: ${out.pass} passed, ${out.fail} failed`,
              out.failures.length ? out.failures : '');
  return out;
};
if (new URLSearchParams(location.search).get('selftest') === '1')
  setTimeout(window.__selftest, 0);
```

That is **17 assertions.**

- [ ] **Step 2: Run them and watch the right ones fail**

Reload with `?selftest=1` appended to the launch URL, then in the console:

```js
window.__selftest();
```

Expected: **`{pass: 8, fail: 9}`** — and it matters *which* nine:

- the **eight** `deckWeight` assertions all report `threw deckWeight is not defined`, because the function does not exist yet;
- `past the range gate` fails, returning 1 instead of 0, because today's `deckFade` ignores a second argument.

Record the actual list — it is the before-image for Step 8. If `fail` is not 9, the starting state is not what this plan assumed, which is worth stopping over rather than pressing on.

Note what does **not** fail: all eight assertions that call `deckFade` with a true altitude pass against the broken code, including every altitude-gate row. That is the whole reason `deckWeight` is being extracted — a test that only covered `deckFade` would have been green on the bug.

- [ ] **Step 3: Change `deckFade`**

Replace the whole function at ~`:3751`:

```js
// The drawn deck and the planet pass's own cloud shell are the same clouds at
// the same altitude. The plane has the resolution near the world origin and
// runs out of it at range; the shell is the other way round. deckFade is the
// weight of the handover — one number, used by both, so it always sums to one.
const DECK_H = 7000;
// The plane is flat, and centred on the LOCAL FRAME ORIGIN — the pad in the
// pad world, the splash site in the entry world. So its weight depends on the
// camera's true altitude and on how far it has travelled from that origin.
// It used to be handed cam.eye[1], which is an altitude only in the pad world:
// an entry starts ~6600 km uprange of splashdown, where cam.eye[1] is about
// -3.7e6 at a true altitude of 84 km. That returned 1.0 instead of 0.0 and
// drew the quad from millions of metres away — the seam across the frame.
const DECK_RNG = 150000;
function deckFade(alt, rng) {
  if (alt > 45000) return 0;
  // The shader hands over on exp(-|xz|/45000) about the origin (see uDeck's
  // use in cloudLayer), so by 150 km the plane contributes 3.6% and nothing is
  // given up by dropping it — while a late ascent tens of km downrange keeps
  // the deck it should have.
  if (rng > DECK_RNG) return 0;
  return Math.min(1, (45000 - alt)/15000)
       * Math.min(1, Math.abs(alt - DECK_H)/140);
}
// The call site, as a function rather than an expression inside drawScene, so
// that it can be asserted on. This is where the bug lived: it is easy to get
// deckFade right and still hand it the wrong number.
function deckWeight(world, camEye, earthC) {
  // only the two worlds that draw a deck plane at all
  if (world !== 'pad' && world !== 'entry') return 0;
  const alt = Math.max(V.len(V.sub(camEye, earthC)) - RE, 2);
  // Horizontal distance from the local frame origin. Both worlds put the
  // origin on the surface with +y up through it, so this is the offset along
  // the ground from the point the frame is anchored to — the pad, or the
  // splash site.
  const rng = Math.hypot(camEye[0], camEye[2]);
  return deckFade(alt, rng);
}
```

- [ ] **Step 4: Use `deckWeight` in `drawScene` and pass the result down**

Immediately after the `fogDen` line (`:3617`), insert:

```js
  const deck = deckWeight(sc.world, cam.eye, sc.earthC);
```

Replace the `uDeck` upload at `:3647-3648`:

```js
  gl.uniform1f(SPACE.u.uDeck, deck);
```

Replace **both** `drawClouds` calls — `:3695` (pad branch) and `:3740` (entry branch) — with the same line:

```js
    drawClouds(sc, view, proj, fogCol, fogDen, deck);
```

- [ ] **Step 5: Make `drawClouds` take the weight**

At `:3757`, change the signature and drop the internal recomputation:

```js
function drawClouds(sc, view, proj, fogCol, fogDen, deck) {
  if (deck <= 0) return;
```

and replace the `uFade` upload (`:3765`):

```js
  gl.uniform1f(CLOUD.u.uFade, deck);
```

Leave every other line of `drawClouds` alone. Verify by grep that no `deckFade(` call passes one argument:

```bash
grep -n 'deckFade(' scripts/launch_page.html
```

Expected: the definition, the single call inside `deckWeight`, and the calls inside `__selftest` — all two-argument. `drawScene` must no longer call `deckFade` directly; it calls `deckWeight`.

- [ ] **Step 6: Apply Task 1's finding**

Read `.superpowers/sdd/2026-07-24-launch-page-rendering-and-entry-frame/task-1-finding.md` and make the single replacement it names. Nothing more — if the finding says the fix is larger than one expression, stop and report rather than improvising a bigger change.

- [ ] **Step 7: Fix `PUP` in both surface worlds**

`PUP` drives particle buoyancy in `stepParticles` (`:2865-2872`), so it must be the real up, not the frame's y axis.

In the pad branch, replace `:3421` (`PUP = [0, 1, 0];`):

```js
    // True local up. At the pad this is [0,1,0] to within the width of the
    // complex, so this is a no-op here and a correction downrange.
    PUP = V.norm(V.sub(st.pos, [0, -RE, 0]));
```

In the entry branch, replace `:3489` (`PUP = [0, 1, 0];`):

```js
  // Entry runs ~6600 km from the splash-site origin, where the frame's y axis
  // is nothing like up. Buoyancy has to follow the real radial.
  PUP = V.norm(V.sub(es.pos, [0, -RE, 0]));
```

- [ ] **Step 8: Verify the assertions pass**

Reload with `?selftest=1`:

```js
window.__selftest();
```

Expected: **`{pass: 17, fail: 0, failures: []}`**

- [ ] **Step 9: Verify the entry visually, in all seven cameras**

Reach an entry frame with particles cleared, so nothing can be mistaken for a particle artefact:

```js
running=false; simT=562261; parts.length=0; debris.length=0;
buildScene(phaseAt(simT));            // camY here was -3714605 before the fix
JSON.stringify({deck: deckFade(84422, 5884000)});   // must be 0
```

Then step cameras `1`–`7` and screenshot each:

```js
for (const k of ['1','2','3','4','5','6','7'])
  document.dispatchEvent(new KeyboardEvent('keydown',{key:k,bubbles:true}));
```

Pass criteria, every camera:
- **no hard full-width seam** anywhere in the frame;
- a **lit planet with clouds** filling the region below the horizon — not a flat blue-grey wash;
- the capsule visible in the exterior views (`wide`, `chase`, `onboard`, `free`).

Repeat at `simT = 567300` (8 km, near the origin, `deck` should be 1) and confirm the deck is still drawn there — this is the case the range gate must **not** break.

Keep the screenshots for chase, onboard and cabin; they were the three that were wrong.

- [ ] **Step 10: Confirm the pad world did not regress**

```js
running=true; simT=45; manualWarp=1;
```

Watch ascent from T+45 s through T+120 s. The deck must still appear below 45 km and fade out above it, exactly as before. `deckFade` values on the pad are unchanged by construction (Step 3 keeps the same arithmetic, and the pad's horizontal range stays far under the 150 km gate), and Step 8's `pad at 30 km, 5 km downrange`, `30 km`, `40 km` and `60 km downrange` rows assert it.

- [ ] **Step 11: Commit**

```bash
git add scripts/launch_page.html
git commit -m "Draw the entry in a frame the deck plane is valid in"
```

---

### Task 3: Render the background at half resolution

Fixed `bgScale = 0.5` here. Task 4 makes it adaptive. Splitting them keeps "did the composite change the picture?" separate from "did the scaler pick a sane number?".

**Files:**
- Modify: `scripts/launch_page.html` — add `bgTarget`/`BLIT`/`blitBackground` after `quadBuf` (`:2715`); split `drawScene` (`:3599-3746`)

**Interfaces:**
- Consumes: `deckFade`, `drawClouds(sc, view, proj, fogCol, fogDen, deck)` from Task 2.
- Produces:
  - `bgScale: number` — module-level, `0.5` in this task, driven by Task 4.
  - `bgTarget(w, h)` — ensures the offscreen target is `w×h`; idempotent.
  - `blitBackground()` — upscales the target to the bound framebuffer.
  - `drawPlanet(sc, cam, sun, asp, pxH, deck)` — the `SPACE` pass. `pxH` is the **render target height in pixels**, which is what `uPxA` must be derived from.
  - `drawWorldMeshes(sc, proj, inCab)` — every mesh draw for the current world, no clouds and no particles. Callable twice per frame.

- [ ] **Step 1: Add the offscreen target and the blit**

Insert immediately after `:2716` (`let skyBuf, partDyn;`):

```js
// ------------------------------------------- scaled background target --
// Measured on an Intel UHD 630 at 2560x1263: the planet/sky pass costs ~84 ms
// and the cloud deck ~52 ms of a 136.8 ms frame, and everything else in this
// page — meshes, particles, decals, HUD, the JS integration — costs 0.1 ms
// together. So the only knob worth turning is how many pixels those two cover.
// They are also both low-frequency, which is what makes upscaling them cheap
// in quality terms; the vehicle and its plume, which carry the edges the eye
// tracks, never leave native resolution.
let bgScale = 0.5;
let bgFB = null, bgTex = null, bgDepth = null, bgW = 0, bgH = 0;
function bgTarget(w, h) {
  if (bgFB && bgW === w && bgH === h) return;
  if (!bgFB) {
    bgFB = gl.createFramebuffer();
    bgTex = gl.createTexture();
    bgDepth = gl.createRenderbuffer();
  }
  bgW = w; bgH = h;
  gl.bindTexture(gl.TEXTURE_2D, bgTex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
  // LINEAR, because this texture exists to be magnified
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  // WebGL 1 has no depth textures without an extension, and nothing here needs
  // to sample depth — a renderbuffer is enough to depth-test the deck against
  // the vehicle.
  gl.bindRenderbuffer(gl.RENDERBUFFER, bgDepth);
  gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT16, w, h);
  gl.bindFramebuffer(gl.FRAMEBUFFER, bgFB);
  gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, bgTex, 0);
  gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT,
                             gl.RENDERBUFFER, bgDepth);
  if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE)
    throw new Error('background framebuffer incomplete at ' + w + 'x' + h);
  gl.bindFramebuffer(gl.FRAMEBUFFER, null);
}
const BLIT = program(`
attribute vec2 aP; varying vec2 vT;
void main(){ vT = aP*0.5 + 0.5; gl_Position = vec4(aP, 0.0, 1.0); }`, `
precision mediump float;
varying vec2 vT; uniform sampler2D uSrc;
void main(){ gl_FragColor = vec4(texture2D(uSrc, vT).rgb, 1.0); }`);
function blitBackground() {
  gl.useProgram(BLIT.p);
  gl.disable(gl.DEPTH_TEST);
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, bgTex);
  gl.uniform1i(BLIT.u.uSrc, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, skyBuf);
  gl.enableVertexAttribArray(BLIT.a.aP);
  gl.vertexAttribPointer(BLIT.a.aP, 2, gl.FLOAT, false, 0, 0);
  gl.drawArrays(gl.TRIANGLES, 0, 6);
  gl.enable(gl.DEPTH_TEST);
}
```

- [ ] **Step 2: Extract the planet pass out of `drawScene`**

Cut `:3623-3658` (from `gl.useProgram(SPACE.p);` through its `gl.drawArrays`) into a new function placed just before `drawScene`. Note the **one substantive change**: `uPxA` is derived from `pxH`, the render target height, not the canvas height.

```js
// The planet, the Moon, the sky and the stars, camera-relative in f64.
// pxH is the height of the target being rendered INTO — uPxA is one pixel's
// angular size, and it is what every fractal in here band-limits against. Feed
// it the canvas height while rendering to a half-size target and the shader
// admits octaves finer than the pixels it is actually filling, which aliases;
// feeding it the target height also drops those octaves, so a smaller target
// is cheaper than its pixel count alone suggests.
function drawPlanet(sc, cam, sun, asp, pxH, deck) {
  gl.useProgram(SPACE.p);
  gl.depthMask(false);
  const camF = V.norm(V.sub(cam.look, cam.eye));
  const camR = V.norm(V.cross(camF, cam.up)), camU = V.cross(camR, camF);
  gl.uniform3fv(SPACE.u.uR, camR); gl.uniform3fv(SPACE.u.uU, camU);
  gl.uniform3fv(SPACE.u.uF, camF); gl.uniform3fv(SPACE.u.uSun, sun);
  gl.uniform1f(SPACE.u.uTan, Math.tan(cam.fov/2));
  gl.uniform1f(SPACE.u.uAsp, asp);
  gl.uniform1f(SPACE.u.uPxA, 2*Math.tan(cam.fov/2)/pxH);
  // scintillation runs on the wall clock, not the (warpable) mission clock
  gl.uniform1f(SPACE.u.uT, performance.now()/1000);
  gl.uniform3fv(SPACE.u.uEc, V.sub(sc.earthC, cam.eye));
  gl.uniform1f(SPACE.u.uER, RE);
  gl.uniform3fv(SPACE.u.uMc, sc.moonC ? V.sub(sc.moonC, cam.eye) : [0,0,0]);
  gl.uniform1f(SPACE.u.uMR, RM);
  gl.uniform1f(SPACE.u.uHasMoon, sc.moonC ? 1 : 0);
  gl.uniformMatrix3fv(SPACE.u.uEMat, false, sc.eMat);
  gl.uniform1f(SPACE.u.uSpin, sc.spin);
  gl.uniform1f(SPACE.u.uNear, sc.near);
  gl.uniform1f(SPACE.u.uScorch, scorch);
  gl.uniform1f(SPACE.u.uDeck, deck);
  gl.uniformMatrix3fv(SPACE.u.uCRot, false, CSEED);
  gl.uniform3fv(SPACE.u.uCamW, cam.eye);
  gl.activeTexture(gl.TEXTURE0);
  gl.bindTexture(gl.TEXTURE_2D, landTex);
  gl.uniform1i(SPACE.u.uLand, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, skyBuf);
  gl.enableVertexAttribArray(SPACE.a.aP);
  gl.vertexAttribPointer(SPACE.a.aP, 2, gl.FLOAT, false, 0, 0);
  gl.drawArrays(gl.TRIANGLES, 0, 6);
  gl.depthMask(true);
}
```

- [ ] **Step 3: Extract the mesh draws out of `drawScene`**

Cut the four world branches (`:3674-3741`) into a function, **leaving the two `drawClouds` calls behind** — they move into the new frame order in Step 4. Everything else is verbatim.

```js
// Every mesh for the current world. No clouds, no particles: this runs twice
// per frame — once depth-only into the scaled target so the cloud deck is
// occluded correctly, once for real at native resolution. Drawing it twice is
// affordable precisely because the mesh pass measured 0.1 ms.
function drawWorldMeshes(sc, proj, inCab) {
  // from inside, the cabin is drawn and the window panes are dropped, so the
  // apertures in the pressure shell become real openings onto the world
  const skip = s => secGone[s.name] ||
    (s.name === 'cabin' ? !inCab : s.name === 'glass' ? inCab : false);

  if (sc.world === 'pad') {
    // The pad world runs all the way to orbit insertion, but the complex is
    // 250 m across: from 20 km up it is a couple of pixels, so beyond that
    // it is not drawn at all. The ground cameras never move, so they keep it.
    if (V.len(sc.cam.eye) < 20000) {
      drawWhole(padBuf, padN, mIdent(), proj);
      drawWhole(towerBuf, towerN, mIdent(), proj);
      bindMeshBuf(armBuf);
      for (const a of armDraw) drawRange(armMat(a, simT), proj, a.first, a.count);
    }
    const rocketM = sc.craftM;
    bindMeshBuf(rocketBuf);
    for (const s of secDraw) {
      if (skip(s)) continue;
      drawRange(rocketM, proj, s.first, s.count);
    }
    for (const d of debris) {
      const dbz = [0,0,1], dby = V.cross(dbz, d.bx);
      const m = mMul(mBasis(d.bx, dby, dbz, d.p), mRotAxis(d.ax, d.a));
      drawRange(m, proj, d.draw.first, d.draw.count);
    }
  } else if (sc.world === 'eci') {
    const cs = sc.cs;
    const stackM = sc.craftM;
    bindMeshBuf(rocketBuf);
    // craft = whatever is still attached: kick stage + capsule, until the
    // stage is jettisoned ahead of entry interface
    for (const s of secDraw) {
      if (s.name === 'pod' || s.name === 'cabin' || s.name === 'glass') {
        if (!skip(s)) drawRange(stackM, proj, s.first, s.count);
        continue;
      }
      if (s.name !== LASTST) continue;
      if (!(simT >= tHand)) drawRange(stackM, proj, s.first, s.count);
      else {
        // jettisoned kick stage drifting away behind
        const off = V.scale(cs.bx, -(3 + 0.6*(simT - tHand)));
        const m2 = mMul(mMul(mTrans(off), stackM), mRotAxis([0,0,1], 0.05*(simT - tHand)));
        drawRange(m2, proj, s.first, s.count);
      }
    }
  } else if (sc.world === 'moon') {
    // The ground, coarsest ring first: they nest, and the depth buffer would
    // sort them anyway, but drawing outward-in means the near ring overwrites
    // the far one's edge rather than fighting it.
    for (let k = moonRings.length - 1; k >= 0; k--)
      drawWhole(moonRings[k].buf, moonRings[k].n, mIdent(), proj);
    if (landerBuf) drawWhole(landerBuf, landerN, sc.craftM, proj);
  } else {
    const es = sc.es;
    const podM = sc.craftM;
    bindMeshBuf(rocketBuf);
    for (const s of secDraw)
      if ((s.name === 'pod' || s.name === 'cabin' || s.name === 'glass') && !skip(s))
        drawRange(podM, proj, s.first, s.count);
    // parachutes: canopy axis opposite the velocity, scale-in on deploy
    const drawChute = (buf, n, t0, lineLen) => {
      if (!isFinite(t0) || simT < t0) return;
      const s = Math.min(1, (simT - t0)/1.2);
      const cx = V.scale(es.vdir, -1), cz = [0,0,1], cyv = V.cross(cz, cx);
      const cp = V.add(es.pos, V.scale(cx, 2.2 + lineLen*s));
      drawWhole(buf, n, mBasis(cx, cyv, cz, cp, 0.15 + 0.85*s), proj);
    };
    if (isFinite(tDrog) && !(simT >= tMain)) drawChute(drogueBuf, drogueN, tDrog, 3.2);
    else drawChute(mainBuf, mainN, tMain, 8.5);
  }
}
```

- [ ] **Step 4: Rewrite `drawScene` as the new frame order**

Replace everything from `function drawScene(sc) {` through the closing brace before the `deckFade` comment block:

```js
function drawScene(sc) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth*dpr | 0, h = cv.clientHeight*dpr | 0;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const cam = sc.cam, asp = w/h;
  const inCab = cam.mode === 'cabin' || cam.mode === 'window';
  const sun = sc.sun || SUN;
  const view = mLookAt(cam.eye, cam.look, cam.up);
  // the console sits ~0.3 m from the crew's eye, so the cabin needs a much
  // nearer clip plane than the outside views
  const proj = mPersp(cam.fov, asp,
                      inCab ? 0.04 : sc.nearZ || 0.35, 3.0e6);
  const camAlt = Math.max(V.len(V.sub(cam.eye, sc.earthC)) - RE, 2);
  // no air on the Moon, so no haze, no scattered fill, and a black sky at
  // noon — the single most recognisable thing about the place
  const dens = sc.world === 'moon' ? 0 : Math.exp(-camAlt/9000);
  const fogCol = [0.62*dens + 0.02, 0.75*dens + 0.04, 0.92*dens + 0.08];
  const fogDen = (sc.world === 'eci' || sc.world === 'moon') ? 0 : 5e-5*dens + 2e-7;
  // a cabin has its own lighting: raise the fill so interior surfaces facing
  // away from the sun still read (the plasma/engine point light still applies)
  const amb = inCab ? 0.52 : sc.world === 'eci' ? 0.24 :
              sc.world === 'moon' ? 0.22 : 0.30 + 0.16*dens;
  const deck = deckWeight(sc.world, cam.eye, sc.earthC);
  const rotSun = V.sub(mApply(view, V.add(cam.eye, sun)), mApply(view, cam.eye));
  const engV = mApply(view, sc.engW);
  const setupMeshes = () => beginMeshes(view, V.norm(rotSun), fogDen, fogCol,
                                        amb, sc.engI, engV, sc.engC);

  // ---- background, at bgScale: planet/sky then the cloud deck ----
  const sw = Math.max(480, Math.round(w*bgScale)),
        sh = Math.max(270, Math.round(h*bgScale));
  bgTarget(sw, sh);
  gl.bindFramebuffer(gl.FRAMEBUFFER, bgFB);
  gl.viewport(0, 0, sw, sh);
  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0, 0, 0, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  // Depth-only prepass, so the deck is hidden where the vehicle is in front of
  // it exactly as it was when both were drawn at native resolution. Only worth
  // doing when there is a deck to occlude.
  if (deck > 0) {
    gl.colorMask(false, false, false, false);
    setupMeshes();
    drawWorldMeshes(sc, proj, inCab);
    gl.colorMask(true, true, true, true);
  }
  // The planet pass fills EVERY pixel of the target — depth test off, not
  // merely depth-write off. If it were allowed to fail against the prepass it
  // would leave the vehicle's silhouette at cleared black, and magnifying that
  // hole draws a dark halo a couple of native pixels wide around the vehicle.
  gl.disable(gl.DEPTH_TEST);
  drawPlanet(sc, cam, sun, asp, sh, deck);
  gl.enable(gl.DEPTH_TEST);
  if (deck > 0) drawClouds(sc, view, proj, fogCol, fogDen, deck);

  // ---- native: composite the background, then everything with an edge ----
  gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  gl.viewport(0, 0, w, h);
  gl.clear(gl.DEPTH_BUFFER_BIT);
  blitBackground();
  setupMeshes();
  drawWorldMeshes(sc, proj, inCab);
  drawParticles(view, proj, cam.fov,
                sc.world === 'entry' ? [0.60, 0.50, 0.45] :
                sc.world === 'moon' ? [0.60, 0.58, 0.54] : undefined);
}
```

- [ ] **Step 5: Check it renders at all, and that nothing throws**

Reload. In the console:

```js
gl.getError();                      // expected: 0
```

Then read console messages for `framebuffer incomplete` or shader link errors. Expected: none.

- [ ] **Step 6: Verify the picture is unchanged apart from background sharpness**

At `bgScale = 0.5`, compare against the Task 2 screenshots at the same `simT` and camera. Specifically confirm:
- **no dark halo** around the vehicle silhouette (this is what Step 4's depth-test-off comment prevents — if you see one, the planet pass is being depth-tested; fix that rather than working around it);
- **no seam**, in pad or entry;
- the vehicle and plume are **as sharp as before** — they are still native;
- the deck is still drawn on the pad below 45 km.

- [ ] **Step 7: Verify the cloud-deck occlusion case**

This is the case the depth prepass exists for, and the one §4.1 of the spec flags as a known limitation.

```js
running=true; simT=45; manualWarp=1;
document.dispatchEvent(new KeyboardEvent('keydown',{key:'3',bubbles:true}));  // chase
```

Watch through 7 km. The vehicle must **not** show through the deck when the camera is above the deck looking down at it.

If it does show through: that is the known limitation, not a mistake in this task. Record it in the task report with a screenshot, and adopt the fallback — keep the scaled depth buffer and stop clearing native depth blindly, by rendering the native mesh pass against a depth buffer seeded from the background. Do **not** silently leave it broken, and do not expand scope without reporting first.

- [ ] **Step 8: Measure the win**

```js
const s=(n)=>new Promise(r=>{const d=[];let l=performance.now();
  const st=()=>{const t=performance.now();d.push(t-l);l=t;
    if(d.length<n)requestAnimationFrame(st);else r(d);};requestAnimationFrame(st);});
const med=async(n=100)=>{const d=(await s(n)).slice(15).sort((a,b)=>a-b);
  return +d[Math.floor(d.length/2)].toFixed(1);};
running=true; simT=45; manualWarp=1;
await new Promise(r=>setTimeout(r,400));
({ scale: bgScale, backPx: cv.width*cv.height, p50: await med() });
```

Expected: p50 **materially below** the 136.8 ms baseline — around 40–55 ms at `bgScale = 0.5`, since the two passes cost roughly a quarter as much plus a blit. Record the number. If it is not below 80 ms, the passes are not actually going to the smaller target — check `sw`/`sh` and that `bgTarget` is not being handed the canvas size.

- [ ] **Step 9: Commit**

```bash
git add scripts/launch_page.html
git commit -m "Render the sky and the deck into a half-size target"
```

---

### Task 4: Adaptive scaler and the frame readout

**Files:**
- Modify: `scripts/launch_page.html` — `bgScale` block (added in Task 3); `frame` (`:3328`); `drawHUD` (`:3838`); `initControls` keydown (`:3948`); `#keys` (`:100`)

**Interfaces:**
- Consumes: `bgScale` from Task 3.
- Produces:
  - `perfPush(ms: number)` — feeds one rAF delta to the scaler.
  - `perfMed: number` — last computed median frame time in ms, `0` until 30 frames have been seen.
  - `perfShow: boolean` — whether the readout rows are displayed.

- [ ] **Step 1: Add the scaler beside `bgScale`**

Replace the `let bgScale = 0.5;` line added in Task 3 with:

```js
// Start at 1.0 and let the first second measure the machine, rather than
// hard-coding a guess for one GPU. The arithmetic says an Intel UHD 630 at
// 2560x1263 settles near 0.34 — sqrt(16/136) — and a faster GPU will simply
// stay at 1.0.
let bgScale = 1.0;
let perfMed = 0, perfShow = false;
// 13 and 20 ms bracket the 16.7 ms vsync interval, so a frame comfortably
// making rate is left alone instead of being hunted around.
const FRAME_HI = 20, FRAME_LO = 13;
const fWin = [];
let fHold = 0;
function perfPush(ms) {
  // A tab switch, a breakpoint or a GC pause is not a slow frame, and letting
  // one drive the scale would drop resolution for the rest of the flight.
  if (!(ms > 0) || ms > 500) return;
  if (fHold > 0) { fHold--; return; }
  fWin.push(ms);
  if (fWin.length < 30) return;
  const s = fWin.slice().sort((a, b) => a - b);
  perfMed = s[Math.floor(s.length/2)];
  fWin.length = 0;
  const was = bgScale;
  if (perfMed > FRAME_HI) bgScale = Math.max(0.35, bgScale - 0.05);
  else if (perfMed < FRAME_LO) bgScale = Math.min(1.0, bgScale + 0.05);
  // After a change, ignore 30 frames: the new resolution needs to be in effect
  // before it is judged, or the scale chases its own last decision.
  if (bgScale !== was) fHold = 30;
}
```

- [ ] **Step 2: Feed it the raw rAF delta**

In `frame` (`:3328`), `dtReal` is clamped to 0.05 s, which would hide every frame worse than 50 ms — exactly the ones that matter. Capture the raw delta before the clamp. Replace `:3330-3332`:

```js
  const raw = now - lastNow;
  const dtReal = Math.min(0.05, raw/1000 || 0.016);
  lastNow = now;
  if (!started) return;
  perfPush(raw);
```

- [ ] **Step 3: Add the readout rows**

In `drawHUD`, replace the final line (`:3838`):

```js
  if (perfShow)
    rows = rows.concat([['FRAME', perfMed ? perfMed.toFixed(1) + ' ms' : '—'],
                        ['BG SCALE', bgScale.toFixed(2)]]);
  $('stats').innerHTML = rows.map(r => `<b>${r[0]}</b><span>${r[1]}</span>`).join('<br>');
```

- [ ] **Step 4: Bind `p`**

In the `keydown` handler inside `initControls` (`:3948-3954`), add beside the `m` line:

```js
    if (e.key === 'p') perfShow = !perfShow;
```

And update the hint at `:100`:

```html
  <div><b>1</b>–<b>7</b> cameras · <b>space</b> pause · <b>m</b> mute · <b>p</b> frame time</div>
```

- [ ] **Step 5: Verify the scaler converges and the readout works**

Reload, start the flight, press `p`. Then:

```js
running=true; simT=45; manualWarp=1;
await new Promise(r=>setTimeout(r,6000));
({ bgScale, perfMed });
```

Expected: `bgScale` has settled below 1.0 and `perfMed` is between 13 and 20 ms. Confirm the two rows appear in the stats plate and that pressing `p` again hides them.

Watch for 30 s and confirm `bgScale` **stops moving** rather than oscillating. If it hunts between two adjacent values, widen the band (raise `FRAME_HI`) rather than adding cleverness.

- [ ] **Step 6: Record the nine-cell frame budget**

This is the gate, and the record is the point — it is what lets the next change to this file be compared instead of guessed at.

For each of three worlds × three window sizes, capture p50 and p90 and the settled `bgScale`:

- worlds: **pad** (`simT=45, running=true, manualWarp=1`), **eci** (`simT=3400`), **entry** (`simT=562261`)
- window sizes: maximized; ~1280×800; ~800×600 (use `mcp__claude-in-chrome__resize_window`)

```js
const s=(n)=>new Promise(r=>{const d=[];let l=performance.now();
  const st=()=>{const t=performance.now();d.push(t-l);l=t;
    if(d.length<n)requestAnimationFrame(st);else r(d);};requestAnimationFrame(st);});
const cell=async()=>{const d=(await s(120)).slice(20).sort((a,b)=>a-b);
  return {p50:+d[Math.floor(0.5*(d.length-1))].toFixed(1),
          p90:+d[Math.floor(0.9*(d.length-1))].toFixed(1),
          scale:+bgScale.toFixed(2), backPx:cv.width*cv.height};};
```

Allow ~6 s at each cell for the scaler to settle before sampling.

**Gate: p50 ≤ 20 ms in all nine cells.** Put the table in the task report.

If a cell cannot meet it at `bgScale = 0.35`, do not lower the clamp — report it, and name octave trimming as the follow-up (spec §6, first risk). Also report the `bgScale` each cell settled at, since that is the quality cost the user asked to be able to see.

- [ ] **Step 7: Commit**

```bash
git add scripts/launch_page.html
git commit -m "Let the machine pick the background resolution"
```

---

### Task 5: Close out

**Files:**
- Modify: `README.md`
- Read only: everything else

**Interfaces:**
- Consumes: all prior tasks.
- Produces: nothing further depends on this.

- [ ] **Step 1: Confirm no Julia behaviour changed**

```bash
git diff --stat main...HEAD -- src/ test/ scripts/panel.jl scripts/panelapp.jl
```

Expected: **empty.** This plan touches `scripts/launch_page.html`, `README.md` and the plan/spec docs only. If anything else appears, stop and explain it.

- [ ] **Step 2: Run the Julia suite**

```bash
julia --project -e 'push!(LOAD_PATH, "src"); using SatelliteSim; include("test/runtests.jl")'
```

Expected: green, ~40 s. It is unchanged code, so a failure here means something unrelated to this plan and should be reported, not fixed silently.

- [ ] **Step 3: Run the panel HTTP suite**

```bash
julia --project -t auto test/panel_http.jl
```

Expected: green.

- [ ] **Step 4: Run the self-test one more time**

Load `http://127.0.0.1:8138/launch?...&selftest=1` (reuse the launch-view URL) and:

```js
window.__selftest();
```

Expected: `{pass: 17, fail: 0, failures: []}`

- [ ] **Step 5: Document the readout in the README**

Find the section describing the launch view (search for `/launch`) and add:

```markdown
Press `p` in the launch view to show the frame time and the background render
scale. The planet/sky and cloud-deck shaders are the entire frame budget —
measured at 84 ms and 52 ms of a 136.8 ms frame on an integrated Intel UHD 630,
against 0.1 ms for every mesh, particle and HUD draw combined — so those two
passes render into an offscreen target at a fraction of native resolution while
the vehicle stays sharp at full resolution. The scale adapts to hold ~60 fps, so
it will read 1.00 on a fast GPU and lower on a slow one. Lower means a softer
sky, never a softer vehicle.
```

- [ ] **Step 6: Commit**

```bash
git add README.md
git commit -m "Document the launch view's frame-time readout"
```

- [ ] **Step 7: Report**

State, with numbers rather than adjectives:
- the before and after p50 on ascent (baseline is 136.8 ms);
- the nine-cell table from Task 4 Step 6, including the `bgScale` each cell settled at;
- what Task 1 found;
- whether the Task 3 Step 7 occlusion case passed or needed the fallback;
- anything left undone.

---

## Phase gate

Phase A is done when: the nine-cell budget is recorded with p50 ≤ 20 ms everywhere; the entry phase is clean in all seven cameras at both 84 km and 8 km; `__selftest` is 13/13; the Julia and panel suites are green; and the `bgScale` the scaler settles at is written down so the quality tradeoff is a number the user can argue with.

Then, and only then, the next phase is Earth-orbit missions — `docs/superpowers/specs/2026-07-24-earth-orbit-missions-and-panel-rework-design.md`, §4.1 and §4.3, plus the `polar` catalogue entry at 90° agreed during design.
